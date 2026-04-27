// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A topology reconciliation.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-ffa: " fmt

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/virtio_config.h>
#include <linux/virtio_features.h>
#include <linux/virtio_msg_bus_provider.h>
#include <linux/virtio_msg_protocol.h>
#include <linux/virtio_msg_transport.h>
#include <linux/workqueue.h>
#include <linux/xarray.h>

#include "virtio_msg_bus_dma_driver_helper.h"
#include "virtio_msg_bus_ffa.h"
#include "virtio_msg_bus_ffa_driver_priv.h"
#include "virtio_msg_bus_ffa_protocol.h"
#include "virtio_msg_bus_queue_helper.h"

#define VIRTIO_MSG_FFA_BUS_NAME			"virtio-msg-ffa"
#define VIRTIO_MSG_FFA_EVENT_QUEUE_DEPTH	32
#define VIRTIO_MSG_FFA_EVENT_QUEUE_TIMEOUT_MS	5000
#define VIRTIO_MSG_FFA_TOPOLOGY_WINDOW_COUNT	256U
#define VIRTIO_MSG_FFA_TOPOLOGY_MAX_WINDOWS	8192U
#define VIRTIO_MSG_FFA_TOPOLOGY_MAX_DEVS	65536U
#define VIRTIO_MSG_FFA_INIT_BUSY_RETRY_MAX	60
#define VIRTIO_MSG_FFA_INIT_BUSY_DELAY_MS	50

struct virtio_msg_ffa_vmdev {
	struct virtio_msg_transport_device vmdev;
	struct virtio_msg_ffa_driver *drv;
	u16 dev_num;
	bool provider_attached;
	bool transport_prepared;
	bool device_registered;
	spinlock_t cb_lock; /* Protects provider callback inflight accounting. */
	bool cb_blocked; /* Prevent new callback entry during unregister. */
	unsigned int cb_inflight;
	struct completion cb_zero_inflight;
	struct list_head pending;
	struct virtio_msg_bus_queue_helper event_queue;
};

struct virtio_msg_ffa_topology {
	struct virtio_msg_ffa_driver *drv;
	struct delayed_work reconcile_work;
	struct mutex lock; /* Protects devices and reconcile flags. */
	struct xarray devices;
	refcount_t refcount;
	struct completion release;
	u32 init_busy_retries;
	bool reconcile_requested;
	bool initial_reconcile_done;
	bool stopping;
};

static struct virtio_msg_ffa_topology *
virtio_msg_ffa_topology_get_rcu(struct virtio_msg_ffa_driver *drv)
{
	struct virtio_msg_ffa_topology *topo;

	rcu_read_lock();
	topo = virtio_msg_ffa_driver_topology_get(drv);
	if (topo && !refcount_inc_not_zero(&topo->refcount))
		topo = NULL;
	rcu_read_unlock();

	return topo;
}

static void virtio_msg_ffa_topology_put(struct virtio_msg_ffa_topology *topo)
{
	if (!topo)
		return;

	if (refcount_dec_and_test(&topo->refcount))
		complete(&topo->release);
}

static u16 virtio_msg_ffa_topology_peer_vm(struct virtio_msg_ffa_topology *topo)
{
	if (!topo || !topo->drv || !topo->drv->fdev)
		return 0;

	return (u16)topo->drv->fdev->vm_id;
}

static bool
virtio_msg_ffa_topology_queue_delay(struct virtio_msg_ffa_topology *topo,
				    unsigned long delay)
{
	if (!topo)
		return false;
	if (!refcount_inc_not_zero(&topo->refcount))
		return false;

	if (!schedule_delayed_work(&topo->reconcile_work, delay)) {
		virtio_msg_ffa_topology_put(topo);
		return false;
	}

	return true;
}

static struct virtio_msg_ffa_vmdev *
virtio_msg_ffa_vmdev_from_transport(struct virtio_msg_transport_device *vmdev)
{
	if (!vmdev)
		return NULL;

	return container_of(vmdev, struct virtio_msg_ffa_vmdev, vmdev);
}

static bool virtio_msg_ffa_vmdev_event_avail_valid
					(const struct virtio_msg *request)
{
	u16 msg_size;

	if (!request)
		return false;

	msg_size = le16_to_cpu(request->msg_size);
	if (request->type != VIRTIO_MSG_TYPE_TRANSPORT)
		return false;
	if (request->msg_id != VIRTIO_MSG_EVENT_AVAIL)
		return false;
	if (msg_size != sizeof(*request) +
			sizeof(struct virtio_msg_event_avail))
		return false;

	return true;
}

static int
virtio_msg_ffa_vmdev_event_submit
		(void *context,
		 const struct virtio_msg_bus_queue_helper_item *item)
{
	struct virtio_msg_ffa_vmdev *shell = context;
	const struct virtio_msg *request;

	if (!shell || !shell->drv || !item)
		return -EINVAL;
	if (item->type != VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_MSG)
		return -EINVAL;

	request = item->vmsg;
	if (!virtio_msg_ffa_vmdev_event_avail_valid(request))
		return -EINVAL;
	if (item->msg_size != le16_to_cpu(request->msg_size))
		return -EMSGSIZE;

	return virtio_msg_ffa_driver_submit_event(shell->drv, request);
}

int virtio_msg_ffa_vmdev_enqueue_event(struct virtio_msg_transport_device *vmdev,
				       const struct virtio_msg *request,
				       gfp_t gfp)
{
	struct virtio_msg_ffa_vmdev *shell;

	if (!request)
		return -EINVAL;
	if (!virtio_msg_ffa_vmdev_event_avail_valid(request))
		return -EINVAL;

	shell = virtio_msg_ffa_vmdev_from_transport(vmdev);
	if (!shell)
		return -EINVAL;

	return virtio_msg_bus_queue_helper_enqueue(&shell->event_queue,
						   request, gfp);
}

struct virtio_msg_ffa_driver *
virtio_msg_ffa_vmdev_callback_begin(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_ffa_vmdev *shell;
	unsigned long flags;

	shell = virtio_msg_ffa_vmdev_from_transport(vmdev);
	if (!shell)
		return NULL;

	spin_lock_irqsave(&shell->cb_lock, flags);
	if (shell->cb_blocked) {
		spin_unlock_irqrestore(&shell->cb_lock, flags);
		return NULL;
	}
	if (!shell->cb_inflight)
		reinit_completion(&shell->cb_zero_inflight);
	shell->cb_inflight++;
	spin_unlock_irqrestore(&shell->cb_lock, flags);

	return shell->drv;
}

void virtio_msg_ffa_vmdev_callback_end(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_ffa_vmdev *shell;
	unsigned long flags;

	shell = virtio_msg_ffa_vmdev_from_transport(vmdev);
	if (!shell)
		return;

	spin_lock_irqsave(&shell->cb_lock, flags);
	if (WARN_ON(!shell->cb_inflight)) {
		spin_unlock_irqrestore(&shell->cb_lock, flags);
		return;
	}
	shell->cb_inflight--;
	if (!shell->cb_inflight)
		complete(&shell->cb_zero_inflight);
	spin_unlock_irqrestore(&shell->cb_lock, flags);
}

void
virtio_msg_ffa_vmdev_callback_wait_zero(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_ffa_vmdev *shell;
	unsigned long flags;

	shell = virtio_msg_ffa_vmdev_from_transport(vmdev);
	if (!shell)
		return;

	spin_lock_irqsave(&shell->cb_lock, flags);
	if (!shell->cb_inflight)
		complete(&shell->cb_zero_inflight);
	spin_unlock_irqrestore(&shell->cb_lock, flags);

	wait_for_completion(&shell->cb_zero_inflight);
}

static void
virtio_msg_ffa_vmdev_callback_block(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_ffa_vmdev *shell;
	unsigned long flags;

	shell = virtio_msg_ffa_vmdev_from_transport(vmdev);
	if (!shell)
		return;

	spin_lock_irqsave(&shell->cb_lock, flags);
	shell->cb_blocked = true;
	if (!shell->cb_inflight)
		complete(&shell->cb_zero_inflight);
	spin_unlock_irqrestore(&shell->cb_lock, flags);

	wait_for_completion(&shell->cb_zero_inflight);
}

static void virtio_msg_ffa_topology_set_required_features
		(struct virtio_msg_transport_device *vmdev)
{
	u64 required[VIRTIO_FEATURES_U64S];

	virtio_features_zero(required);
	virtio_features_set_bit(required, VIRTIO_F_VERSION_1);
	virtio_features_set_bit(required, VIRTIO_F_ACCESS_PLATFORM);
	virtio_msg_transport_set_required_features(vmdev, required);
}

static int
virtio_msg_ffa_topology_enumerate_devices(struct virtio_msg_ffa_topology *topo,
					  unsigned long *present)
{
	struct virtio_msg_bus_get_devices req;
	u8 resp_msg_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *resp_msg = (struct virtio_msg *)resp_msg_buf;
	const struct virtio_msg_bus_get_devices_resp *resp;
	size_t resp_len;
	size_t bitmap_len;
	u16 offset = 0;
	u16 count = VIRTIO_MSG_FFA_TOPOLOGY_WINDOW_COUNT;
	u16 next_offset;
	u16 resp_count;
	unsigned int window;
	unsigned int i;
	int ret;

	if (!topo || !present)
		return -EINVAL;

	for (window = 0; window < VIRTIO_MSG_FFA_TOPOLOGY_MAX_WINDOWS; window++) {
		req.offset = cpu_to_le16(offset);
		req.count = cpu_to_le16(count);

		ret = virtio_msg_ffa_topology_send_bus_request
				(topo->drv, VIRTIO_MSG_BUS_GET_DEVICES,
				 &req, sizeof(req), resp_msg,
				 sizeof(resp_msg_buf), &resp_len);
		if (ret)
			return ret;

		if (resp_len < sizeof(*resp_msg) + sizeof(*resp))
			return -EPROTO;

		resp = (const struct virtio_msg_bus_get_devices_resp *)
			resp_msg->payload;
		resp_count = le16_to_cpu(resp->count);
		if (resp_count > count)
			return -EPROTO;

		if (le16_to_cpu(resp->offset) != offset)
			return -EPROTO;

		bitmap_len = DIV_ROUND_UP(resp_count, 8);
		if (resp_len != sizeof(*resp_msg) + sizeof(*resp) + bitmap_len)
			return -EPROTO;

		for (i = 0; i < resp_count; i++) {
			unsigned int dev_num = offset + i;

			if (dev_num >= VIRTIO_MSG_FFA_TOPOLOGY_MAX_DEVS)
				break;
			if (resp->bitmap[i / 8] & BIT(i % 8))
				set_bit(dev_num, present);
		}

		next_offset = le16_to_cpu(resp->next_offset);
		if (!next_offset)
			return 0;
		if (next_offset <= offset)
			return -EPROTO;

		offset = next_offset;
	}

	return -EOVERFLOW;
}

static void
virtio_msg_ffa_topology_mark_initial_done(struct virtio_msg_ffa_topology *topo)
{
	if (!topo)
		return;

	mutex_lock(&topo->lock);
	topo->init_busy_retries = 0;
	topo->initial_reconcile_done = true;
	mutex_unlock(&topo->lock);
}

static bool
virtio_msg_ffa_topology_initial_done(struct virtio_msg_ffa_topology *topo)
{
	bool done;

	if (!topo)
		return false;

	mutex_lock(&topo->lock);
	done = topo->initial_reconcile_done;
	mutex_unlock(&topo->lock);

	return done;
}

static bool
virtio_msg_ffa_topology_retry_init_busy(struct virtio_msg_ffa_topology *topo,
					int ret)
{
	struct device *parent;
	unsigned long delay;
	u32 attempt;

	if (!topo || ret != -EBUSY)
		return false;

	mutex_lock(&topo->lock);
	if (topo->initial_reconcile_done ||
	    topo->init_busy_retries >= VIRTIO_MSG_FFA_INIT_BUSY_RETRY_MAX ||
	    topo->stopping || virtio_msg_ffa_endpoint_disabled(topo->drv)) {
		mutex_unlock(&topo->lock);
		return false;
	}

	topo->init_busy_retries++;
	attempt = topo->init_busy_retries;
	topo->reconcile_requested = true;
	mutex_unlock(&topo->lock);

	parent = virtio_msg_ffa_driver_parent(topo->drv);
	if (parent)
		dev_dbg(parent,
			"topology init busy retry: peer_vm=%u attempt=%u/%u delay_ms=%u\n",
			virtio_msg_ffa_topology_peer_vm(topo), attempt,
			VIRTIO_MSG_FFA_INIT_BUSY_RETRY_MAX,
			VIRTIO_MSG_FFA_INIT_BUSY_DELAY_MS);
	else
		pr_debug("topology init busy retry: peer_vm=%u attempt=%u/%u delay_ms=%u\n",
			 virtio_msg_ffa_topology_peer_vm(topo), attempt,
			 VIRTIO_MSG_FFA_INIT_BUSY_RETRY_MAX,
			 VIRTIO_MSG_FFA_INIT_BUSY_DELAY_MS);

	delay = msecs_to_jiffies(VIRTIO_MSG_FFA_INIT_BUSY_DELAY_MS);
	virtio_msg_ffa_topology_queue_delay(topo, delay);
	return true;
}

static void
virtio_msg_ffa_topology_warn_init_busy_exhausted
			(struct virtio_msg_ffa_topology *topo, int ret)
{
	struct device *parent;

	if (!topo)
		return;

	parent = virtio_msg_ffa_driver_parent(topo->drv);
	if (parent)
		dev_warn_ratelimited(parent,
				     "topology init busy retry exhausted: peer_vm=%u ret=%d\n",
				     virtio_msg_ffa_topology_peer_vm(topo), ret);
	else
		pr_warn_ratelimited("topology init busy retry exhausted: peer_vm=%u ret=%d\n",
				    virtio_msg_ffa_topology_peer_vm(topo), ret);
}

static struct virtio_msg_ffa_vmdev *
virtio_msg_ffa_topology_publish_device(struct virtio_msg_ffa_topology *topo,
				       u16 dev_num)
{
	const struct virtio_msg_bus_provider_ops *provider_ops;
	const struct virtio_msg_bus_dma_driver_provider_ops *area_ops;
	struct device *parent;
	struct virtio_msg_ffa_vmdev *shell;
	int ret;

	if (!topo)
		return ERR_PTR(-EINVAL);

	provider_ops = virtio_msg_ffa_bus_ops_get();
	area_ops = virtio_msg_ffa_area_provider_ops_get();
	parent = virtio_msg_ffa_driver_parent(topo->drv);
	if (!provider_ops || !area_ops || !parent)
		return ERR_PTR(-ENODEV);

	shell = kzalloc(sizeof(*shell), GFP_KERNEL);
	if (!shell)
		return ERR_PTR(-ENOMEM);

	shell->drv = topo->drv;
	shell->dev_num = dev_num;
	shell->vmdev.dev_num = dev_num;
	spin_lock_init(&shell->cb_lock);
	shell->cb_blocked = false;
	init_completion(&shell->cb_zero_inflight);
	complete(&shell->cb_zero_inflight);
	INIT_LIST_HEAD(&shell->pending);

	ret = virtio_msg_bus_queue_helper_init
			(&shell->event_queue, parent, FFA_BUS_MAX_MSG_SIZE,
			 VIRTIO_MSG_FFA_EVENT_QUEUE_DEPTH,
			 VIRTIO_MSG_FFA_EVENT_QUEUE_TIMEOUT_MS,
			 virtio_msg_ffa_vmdev_event_submit, NULL, shell);
	if (ret)
		goto err_free;

	ret = virtio_msg_transport_attach_provider(&shell->vmdev, provider_ops,
						   VIRTIO_MSG_FFA_BUS_NAME,
						   topo->drv);
	if (ret)
		goto err_free;
	shell->provider_attached = true;

	virtio_msg_ffa_topology_set_required_features(&shell->vmdev);

	ret = virtio_msg_transport_prepare_device(&shell->vmdev);
	if (ret)
		goto err_detach;
	shell->transport_prepared = true;

	shell->vmdev.vdev.vmap.dma_dev = parent;
	shell->vmdev.vdev.dev.parent = parent;
	ret = virtio_msg_bus_dma_driver_helper_install
			(&shell->vmdev, area_ops,
			 virtio_msg_ffa_driver_area_ctx(topo->drv));
	if (ret)
		goto err_unprepare;

	if (topo->drv->fdev)
		dev_info(&topo->drv->fdev->dev,
			 "device publish prepared: peer_vm=%u dev_num=%u\n",
			 (u16)topo->drv->fdev->vm_id, dev_num);

	return shell;

err_unprepare:
	shell->vmdev.vdev.vmap.dma_dev = NULL;
	shell->vmdev.vdev.dev.parent = NULL;
	if (shell->transport_prepared) {
		virtio_msg_transport_unprepare_device(&shell->vmdev);
		shell->transport_prepared = false;
	}
err_detach:
	if (shell->provider_attached) {
		virtio_msg_transport_detach_provider(&shell->vmdev);
		shell->provider_attached = false;
	}
err_free:
	virtio_msg_bus_queue_helper_stop(&shell->event_queue);
	kfree(shell);
	return ERR_PTR(ret);
}

static void
virtio_msg_ffa_topology_unpublish_device(struct virtio_msg_ffa_vmdev *shell)
{
	if (!shell)
		return;

	virtio_msg_ffa_vmdev_callback_block(&shell->vmdev);
	virtio_msg_bus_queue_helper_stop(&shell->event_queue);

	if (shell->device_registered) {
		if (shell->drv && shell->drv->fdev)
			dev_info(&shell->drv->fdev->dev,
				 "device unregister: peer_vm=%u dev_num=%u\n",
				 (u16)shell->drv->fdev->vm_id,
				 shell->dev_num);
		virtio_msg_transport_unregister_device(&shell->vmdev);
		shell->device_registered = false;
	}

	if (virtio_msg_bus_dma_driver_helper_is_installed(&shell->vmdev))
		virtio_msg_bus_dma_driver_helper_cleanup(&shell->vmdev);

	if (shell->transport_prepared) {
		virtio_msg_transport_unprepare_device(&shell->vmdev);
		shell->transport_prepared = false;
	}

	if (shell->provider_attached) {
		virtio_msg_transport_detach_provider(&shell->vmdev);
		shell->provider_attached = false;
	}

	shell->vmdev.vdev.vmap.dma_dev = NULL;
	shell->vmdev.vdev.dev.parent = NULL;
	if (shell->drv && shell->drv->fdev)
		dev_info(&shell->drv->fdev->dev,
			 "device unpublish: peer_vm=%u dev_num=%u\n",
			 (u16)shell->drv->fdev->vm_id, shell->dev_num);
	kfree(shell);
}

static void
virtio_msg_ffa_topology_remove_stale(struct virtio_msg_ffa_topology *topo,
				     unsigned long *present)
{
	struct virtio_msg_ffa_vmdev *shell;
	unsigned long index;

	for (;;) {
		shell = NULL;

		mutex_lock(&topo->lock);
		xa_for_each(&topo->devices, index, shell) {
			if (!test_bit(index, present)) {
				xa_erase(&topo->devices, index);
				break;
			}
			shell = NULL;
		}
		mutex_unlock(&topo->lock);

		if (!shell)
			break;

		virtio_msg_ffa_topology_unpublish_device(shell);
	}
}

static int
virtio_msg_ffa_topology_prepare_new(struct virtio_msg_ffa_topology *topo,
				    unsigned long *present,
				    struct list_head *prepared)
{
	struct virtio_msg_ffa_vmdev *shell;
	unsigned long dev_num;
	int ret;

	if (!topo || !present || !prepared)
		return -EINVAL;

	for_each_set_bit(dev_num, present, VIRTIO_MSG_FFA_TOPOLOGY_MAX_DEVS) {
		mutex_lock(&topo->lock);
		if (xa_load(&topo->devices, dev_num)) {
			mutex_unlock(&topo->lock);
			continue;
		}
		mutex_unlock(&topo->lock);

		shell = virtio_msg_ffa_topology_publish_device(topo, dev_num);
		if (IS_ERR(shell)) {
			ret = PTR_ERR(shell);
			goto err_unwind;
		}

		mutex_lock(&topo->lock);
		if (xa_load(&topo->devices, dev_num)) {
			mutex_unlock(&topo->lock);
			virtio_msg_ffa_topology_unpublish_device(shell);
			continue;
		}

		ret = xa_err(xa_store(&topo->devices, dev_num, shell, GFP_KERNEL));
		mutex_unlock(&topo->lock);
		if (ret) {
			virtio_msg_ffa_topology_unpublish_device(shell);
			goto err_unwind;
		}

		list_add_tail(&shell->pending, prepared);
	}

	return 0;

err_unwind:
	while (!list_empty(prepared)) {
		shell = list_first_entry(prepared,
					 struct virtio_msg_ffa_vmdev,
					 pending);
		list_del_init(&shell->pending);
		mutex_lock(&topo->lock);
		if (xa_load(&topo->devices, shell->dev_num) == shell)
			xa_erase(&topo->devices, shell->dev_num);
		mutex_unlock(&topo->lock);
		virtio_msg_ffa_topology_unpublish_device(shell);
	}
	return ret;
}

static void
virtio_msg_ffa_topology_unwind_prepared(struct virtio_msg_ffa_topology *topo,
					struct list_head *prepared)
{
	struct virtio_msg_ffa_vmdev *shell;
	struct virtio_msg_ffa_vmdev *tmp;

	if (!topo || !prepared)
		return;

	list_for_each_entry_safe(shell, tmp, prepared, pending) {
		list_del_init(&shell->pending);
		mutex_lock(&topo->lock);
		if (xa_load(&topo->devices, shell->dev_num) == shell)
			xa_erase(&topo->devices, shell->dev_num);
		mutex_unlock(&topo->lock);
		virtio_msg_ffa_topology_unpublish_device(shell);
	}
}

static int
virtio_msg_ffa_topology_register_prepared(struct virtio_msg_ffa_topology *topo,
					  struct list_head *prepared)
{
	struct virtio_msg_ffa_vmdev *shell;
	struct virtio_msg_ffa_vmdev *tmp;
	int ret;

	if (!topo || !prepared)
		return -EINVAL;

	list_for_each_entry_safe(shell, tmp, prepared, pending) {
		ret = virtio_msg_transport_register_prepared_device(&shell->vmdev);
		if (ret) {
			mutex_lock(&topo->lock);
			if (xa_load(&topo->devices, shell->dev_num) == shell)
				xa_erase(&topo->devices, shell->dev_num);
			mutex_unlock(&topo->lock);
			list_del_init(&shell->pending);
			virtio_msg_ffa_topology_unpublish_device(shell);
			virtio_msg_ffa_topology_unwind_prepared(topo, prepared);
			return ret;
		}
		shell->device_registered = true;
		if (topo->drv->fdev)
			dev_info(&topo->drv->fdev->dev,
				 "device register: peer_vm=%u dev_num=%u\n",
				 (u16)topo->drv->fdev->vm_id, shell->dev_num);
		list_del_init(&shell->pending);
	}

	return 0;
}

static int virtio_msg_ffa_topology_reconcile_once
			(struct virtio_msg_ffa_topology *topo)
{
	unsigned long *present;
	LIST_HEAD(prepared);
	int ret;

	if (!topo)
		return -EINVAL;
	if (virtio_msg_ffa_endpoint_disabled(topo->drv))
		return 0;
	if (topo->drv->fdev)
		dev_info(&topo->drv->fdev->dev,
			 "topology reconcile start: peer_vm=%u\n",
			 (u16)topo->drv->fdev->vm_id);

	ret = virtio_msg_ffa_driver_prepare_runtime(topo->drv);
	if (virtio_msg_ffa_topology_retry_init_busy(topo, ret))
		return -EAGAIN;
	if (ret) {
		if (ret == -EBUSY &&
		    !virtio_msg_ffa_topology_initial_done(topo))
			virtio_msg_ffa_topology_warn_init_busy_exhausted(topo,
									 ret);
		virtio_msg_ffa_endpoint_disable(topo->drv);
		return ret;
	}

	present = bitmap_zalloc(VIRTIO_MSG_FFA_TOPOLOGY_MAX_DEVS, GFP_KERNEL);
	if (!present)
		return -ENOMEM;

	ret = virtio_msg_ffa_topology_enumerate_devices(topo, present);
	if (virtio_msg_ffa_topology_retry_init_busy(topo, ret)) {
		ret = -EAGAIN;
		goto out_free;
	}
	if (ret) {
		if (ret == -EBUSY &&
		    !virtio_msg_ffa_topology_initial_done(topo)) {
			virtio_msg_ffa_topology_warn_init_busy_exhausted(topo,
									 ret);
			virtio_msg_ffa_endpoint_disable(topo->drv);
		}
		goto out_free;
	}

	virtio_msg_ffa_topology_mark_initial_done(topo);
	virtio_msg_ffa_topology_remove_stale(topo, present);
	ret = virtio_msg_ffa_topology_prepare_new(topo, present, &prepared);
	if (ret)
		goto out_free;

	if (list_empty(&prepared))
		goto out_free;

	ret = virtio_msg_ffa_topology_register_prepared(topo, &prepared);

out_free:
	bitmap_free(present);
	if (topo->drv->fdev)
		dev_info(&topo->drv->fdev->dev,
			 "topology reconcile result: peer_vm=%u ret=%d\n",
			 (u16)topo->drv->fdev->vm_id, ret);
	return ret;
}

static void virtio_msg_ffa_topology_reconcile_work(struct work_struct *work)
{
	struct virtio_msg_ffa_topology *topo =
		container_of(to_delayed_work(work),
			     struct virtio_msg_ffa_topology, reconcile_work);
	struct device *parent;
	int ret;

	parent = virtio_msg_ffa_driver_parent(topo->drv);

	for (;;) {
		mutex_lock(&topo->lock);
		if (topo->stopping ||
		    virtio_msg_ffa_endpoint_disabled(topo->drv)) {
			topo->reconcile_requested = false;
			mutex_unlock(&topo->lock);
			break;
		}

		if (!topo->reconcile_requested) {
			mutex_unlock(&topo->lock);
			break;
		}

		topo->reconcile_requested = false;
		mutex_unlock(&topo->lock);

		ret = virtio_msg_ffa_topology_reconcile_once(topo);
		if (ret == -EAGAIN)
			break;
		if (ret && parent)
			dev_warn_ratelimited(parent,
					     "topology reconcile failed: %d\n",
					     ret);
	}

	virtio_msg_ffa_topology_put(topo);
}

int virtio_msg_ffa_topology_init(struct virtio_msg_ffa_driver *drv)
{
	struct virtio_msg_ffa_topology *topo;

	if (!drv)
		return -EINVAL;

	topo = kzalloc(sizeof(*topo), GFP_KERNEL);
	if (!topo)
		return -ENOMEM;

	topo->drv = drv;
	mutex_init(&topo->lock);
	xa_init(&topo->devices);
	refcount_set(&topo->refcount, 1);
	init_completion(&topo->release);
	INIT_DELAYED_WORK(&topo->reconcile_work,
			  virtio_msg_ffa_topology_reconcile_work);
	virtio_msg_ffa_driver_topology_set(drv, topo);

	virtio_msg_ffa_topology_schedule(drv);

	return 0;
}

void virtio_msg_ffa_topology_destroy(struct virtio_msg_ffa_topology *topo)
{
	struct virtio_msg_ffa_vmdev *shell;
	unsigned long index;

	if (!topo)
		return;

	mutex_lock(&topo->lock);
	topo->stopping = true;
	mutex_unlock(&topo->lock);
	if (cancel_delayed_work_sync(&topo->reconcile_work))
		virtio_msg_ffa_topology_put(topo);

	for (;;) {
		shell = NULL;

		mutex_lock(&topo->lock);
		xa_for_each(&topo->devices, index, shell) {
			xa_erase(&topo->devices, index);
			break;
		}
		mutex_unlock(&topo->lock);

		if (!shell)
			break;

		virtio_msg_ffa_topology_unpublish_device(shell);
	}

	xa_destroy(&topo->devices);
	virtio_msg_ffa_topology_put(topo);
	wait_for_completion(&topo->release);
	kfree(topo);
}

void virtio_msg_ffa_topology_schedule(struct virtio_msg_ffa_driver *drv)
{
	struct virtio_msg_ffa_topology *topo;
	bool do_schedule = false;

	if (!drv)
		return;

	if (virtio_msg_ffa_endpoint_disabled(drv))
		return;

	topo = virtio_msg_ffa_topology_get_rcu(drv);
	if (!topo)
		return;

	mutex_lock(&topo->lock);
	if (!topo->stopping && !virtio_msg_ffa_endpoint_disabled(drv)) {
		topo->reconcile_requested = true;
		do_schedule = true;
	}
	mutex_unlock(&topo->lock);
	if (do_schedule) {
		if (mod_delayed_work(system_wq, &topo->reconcile_work, 0))
			virtio_msg_ffa_topology_put(topo);
	} else {
		virtio_msg_ffa_topology_put(topo);
	}
}

int virtio_msg_ffa_topology_handle_event(struct virtio_msg_ffa_driver *drv,
					 const struct virtio_msg_bus_event_device *event)
{
	u16 dev_num;
	u16 dev_state;

	if (!drv || !event)
		return -EINVAL;
	dev_num = le16_to_cpu(event->dev_num);
	dev_state = le16_to_cpu(event->dev_state);
	if (drv->fdev)
		dev_dbg(&drv->fdev->dev,
			"topology event queued: peer_vm=%u dev_num=%u state=%u\n",
			(u16)drv->fdev->vm_id, dev_num, dev_state);

	switch (dev_state) {
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_ADDED:
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED:
		virtio_msg_ffa_topology_schedule(drv);
		break;
	default:
		break;
	}

	return 0;
}

int virtio_msg_ffa_topology_handle_transport_event(struct virtio_msg_ffa_driver *drv,
						   const struct virtio_msg *msg)
{
	struct virtio_msg_ffa_topology *topo;
	struct virtio_msg_ffa_vmdev *shell;
	u16 dev_num;
	int ret = 0;

	if (!drv || !msg)
		return -EINVAL;

	dev_num = le16_to_cpu(msg->dev_num);
	topo = virtio_msg_ffa_topology_get_rcu(drv);
	if (!topo)
		return 0;

	mutex_lock(&topo->lock);
	shell = xa_load(&topo->devices, dev_num);
	if (shell)
		ret = virtio_msg_transport_handle_device_event(&shell->vmdev,
							       msg);
	else if (drv->fdev) {
		/*
		 * Before initial enumeration completes there is no transport
		 * device to receive an event. Discard it because the peer must
		 * not emit device events before driver initialisation completes.
		 */
		dev_dbg(&drv->fdev->dev,
			"transport event dropped: peer_vm=%u dev_num=%u id=0x%02x\n",
			(u16)drv->fdev->vm_id, dev_num, msg->msg_id);
	}
	mutex_unlock(&topo->lock);
	virtio_msg_ffa_topology_put(topo);

	return ret;
}
