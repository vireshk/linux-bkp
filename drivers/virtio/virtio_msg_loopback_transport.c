// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message loopback slot base scaffold.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-loopback: " fmt

#include <asm/barrier.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/virtio_config.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "virtio_msg_bus_dma.h"
#include "virtio_msg_loopback_internal.h"
#define VM_LOG_COMPONENT VM_LOG_COMPONENT_LOOPBACK
#include "virtio_msg_debug.h"

static void
virtio_msg_loopback_deferred_event_list_free(struct list_head *events)
{
	struct virtio_msg_loopback_deferred_event *entry, *tmp;

	if (!events)
		return;

	list_for_each_entry_safe(entry, tmp, events, node) {
		list_del(&entry->node);
		kfree(entry);
	}
}

static void
virtio_msg_loopback_slot_purge_deferred_events
	(struct virtio_msg_loopback_slot *slot)
{
	LIST_HEAD(events);
	unsigned long flags;

	if (!slot)
		return;

	spin_lock_irqsave(&slot->deferred_event_lock, flags);
	list_splice_init(&slot->deferred_events, &events);
	spin_unlock_irqrestore(&slot->deferred_event_lock, flags);

	virtio_msg_loopback_deferred_event_list_free(&events);
}

static struct virtio_msg_loopback_deferred_event *
virtio_msg_loopback_deferred_event_find_locked
	(struct virtio_msg_loopback_slot *slot, u32 vq_index)
{
	struct virtio_msg_loopback_deferred_event *entry;

	lockdep_assert_held(&slot->deferred_event_lock);

	list_for_each_entry(entry, &slot->deferred_events, node) {
		if (entry->vq_index == vq_index)
			return entry;
	}

	return NULL;
}

static void
virtio_msg_loopback_deferred_event_queue_locked
	(struct virtio_msg_loopback_slot *slot,
	 struct virtio_msg_loopback_deferred_event *entry)
{
	struct virtio_msg_loopback_deferred_event *existing;

	lockdep_assert_held(&slot->deferred_event_lock);

	if (!entry)
		return;

	existing = virtio_msg_loopback_deferred_event_find_locked(slot,
								  entry->vq_index);
	if (existing) {
		existing->next_offset = entry->next_offset;
		kfree(entry);
		return;
	}

	list_add_tail(&entry->node, &slot->deferred_events);
}

static int
virtio_msg_loopback_defer_event(struct virtio_msg_loopback_slot *slot,
				u32 vq_index, u32 next_offset)
{
	struct virtio_msg_loopback_deferred_event *entry;
	unsigned long flags;

	if (!slot)
		return -EINVAL;

	spin_lock_irqsave(&slot->deferred_event_lock, flags);
	entry = virtio_msg_loopback_deferred_event_find_locked(slot, vq_index);
	if (entry) {
		entry->next_offset = next_offset;
		spin_unlock_irqrestore(&slot->deferred_event_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&slot->deferred_event_lock, flags);

	entry = kzalloc(sizeof(*entry), GFP_ATOMIC);
	if (!entry)
		return -ENOSPC;

	entry->vq_index = vq_index;
	entry->next_offset = next_offset;

	spin_lock_irqsave(&slot->deferred_event_lock, flags);
	virtio_msg_loopback_deferred_event_queue_locked(slot, entry);
	spin_unlock_irqrestore(&slot->deferred_event_lock, flags);

	return 0;
}

static void virtio_msg_loopback_deferred_event_workfn(struct work_struct *work)
{
	struct virtio_msg_loopback_slot *slot;
	struct virtio_msg_loopback *loopback;
	struct virtio_msg_dispatch_ctx dctx = {
		.flags = VIRTIO_MSG_DISPATCH_F_NONBLOCK,
	};
	struct virtio_msg *request;
	struct virtio_msg_event_avail *event;
	struct virtio_msg_loopback_deferred_event *entry;
	unsigned long flags;
	u8 request_buf[sizeof(*request) + sizeof(*event)];
	u32 handle;
	int ret;

	slot = container_of(to_delayed_work(work),
			    struct virtio_msg_loopback_slot, deferred_event_work);
	loopback = slot->loopback;
	if (!loopback)
		return;

	if (atomic_read(&slot->pending_async_exports) > 0)
		return;

	for (;;) {
		if (atomic_read(&slot->pending_async_exports) > 0)
			return;

		spin_lock_irqsave(&slot->deferred_event_lock, flags);
		if (list_empty(&slot->deferred_events)) {
			spin_unlock_irqrestore(&slot->deferred_event_lock, flags);
			return;
		}
		entry = list_first_entry(&slot->deferred_events,
					 struct virtio_msg_loopback_deferred_event,
					 node);
		list_del_init(&entry->node);
		spin_unlock_irqrestore(&slot->deferred_event_lock, flags);

		/*
		 * Gate may have reopened after we popped one deferred event.
		 * Put it back without publishing until pending shares clear.
		 */
		if (atomic_read(&slot->pending_async_exports) > 0) {
			spin_lock_irqsave(&slot->deferred_event_lock, flags);
			virtio_msg_loopback_deferred_event_queue_locked(slot,
									entry);
			spin_unlock_irqrestore(&slot->deferred_event_lock,
					       flags);
			return;
		}

		memset(request_buf, 0, sizeof(request_buf));
		request = (struct virtio_msg *)request_buf;
		virtio_msg_prepare(request, VIRTIO_MSG_EVENT_AVAIL,
				   VIRTIO_MSG_TOKEN_EVENT, sizeof(*event));
		request->dev_num = cpu_to_le16(slot->dev_num);
		event = virtio_msg_payload(request);
		event->vq_index = cpu_to_le32(entry->vq_index);
		event->next_offset = cpu_to_le32(entry->next_offset);

		handle = READ_ONCE(loopback->bridge.endpoint.handle);
		if (!handle)
			ret = -ENODEV;
		else
			ret = virtio_msg_bus_bridge_device_rx(handle, request, &dctx);

		if (!ret) {
			kfree(entry);
			continue;
		}

		if (ret == -EAGAIN || ret == -ENOSPC) {
			spin_lock_irqsave(&slot->deferred_event_lock, flags);
			virtio_msg_loopback_deferred_event_queue_locked(slot,
									entry);
			spin_unlock_irqrestore(&slot->deferred_event_lock, flags);
			(void)mod_delayed_work(system_wq, &slot->deferred_event_work,
					       msecs_to_jiffies
						(VIRTIO_MSG_LOOPBACK_RELAY_RETRY_DELAY_MS));
			return;
		}

		kfree(entry);
		if (ret == -ENODEV || ret == -ENOTCONN) {
			LIST_HEAD(drop_events);

			spin_lock_irqsave(&slot->deferred_event_lock, flags);
			list_splice_init(&slot->deferred_events, &drop_events);
			spin_unlock_irqrestore(&slot->deferred_event_lock, flags);

			virtio_msg_loopback_deferred_event_list_free(&drop_events);
			return;
		}
	}
}

static int
virtio_msg_loopback_get_caps(struct virtio_msg_transport_device *vmdev,
			     struct virtio_msg_provider_caps *caps)
{
	if (!vmdev || !caps)
		return -EINVAL;

	memset(caps, 0, sizeof(*caps));
	caps->name = VIRTIO_MSG_LOOPBACK_BUS_NAME;
	caps->msg_size = VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE;
	caps->revision = VIRTIO_MSG_REVISION_1;

	return 0;
}

static unsigned long
virtio_msg_loopback_pending_timeout_jiffies
	(const struct virtio_msg_loopback_relay_pending *pending)
{
	if (!pending)
		return 0;

	if (time_after(pending->deadline, jiffies))
		return pending->deadline - jiffies;

	return 0;
}

static u16
virtio_msg_loopback_next_request_token_locked(struct virtio_msg_loopback *loopback)
{
	u16 token;

	lockdep_assert_held(&loopback->pending_lock);

	if (!loopback->pending_next_seq)
		loopback->pending_next_seq = 1;

	for (;;) {
		token = (u16)loopback->pending_next_seq++;
		if (!loopback->pending_next_seq)
			loopback->pending_next_seq = 1;
		if (token > VIRTIO_MSG_TOKEN_FIXED)
			return token;
	}
}

static int
virtio_msg_loopback_request(struct virtio_msg_transport_device *vmdev,
			    const struct virtio_msg *request,
			    struct virtio_msg *response)
{
	struct virtio_msg_dispatch_ctx dctx = { 0 };
	struct virtio_msg_loopback_relay_pending *pending;
	struct virtio_msg *request_local;
	struct virtio_msg_loopback *loopback;
	u8 request_buf[VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE];
	unsigned long timeout;
	u16 request_size;
	u16 expected_dev_num;
	u16 dev_num;
	int ret;
	u32 handle;

	if (!vmdev || !request || !response)
		return -EINVAL;

	loopback = vmdev->provider_data;
	if (!loopback)
		return -ENODEV;

	request_size = le16_to_cpu(request->msg_size);
	if (request_size < sizeof(*request) ||
	    request_size > VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE)
		return -EMSGSIZE;
	if (request->type & VIRTIO_MSG_TYPE_BUS)
		return -EOPNOTSUPP;
	if (request->type & VIRTIO_MSG_TYPE_RESPONSE)
		return -EINVAL;

	dev_num = le16_to_cpu(request->dev_num);
	expected_dev_num = vmdev->dev_num;
	if (dev_num != expected_dev_num)
		return -ENODEV;

	request_local = (struct virtio_msg *)request_buf;
	memcpy(request_local, request, request_size);

	pending = kzalloc(sizeof(*pending), GFP_KERNEL);
	if (!pending)
		return -ENOMEM;

	init_completion(&pending->done);
	pending->dev_num = dev_num;
	pending->status = -ETIMEDOUT;
	pending->deadline = jiffies +
		msecs_to_jiffies(VIRTIO_MSG_LOOPBACK_RELAY_TIMEOUT_MS);
	pending->response = response;
	pending->response_capacity = VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE;
	refcount_set(&pending->refs, 1);

	mutex_lock(&loopback->pending_lock);
	if (!loopback->pending_next_seq)
		loopback->pending_next_seq = 1;
	pending->relay_seq = loopback->pending_next_seq++;
	if (!loopback->pending_next_seq)
		loopback->pending_next_seq = 1;
	pending->token = virtio_msg_loopback_next_request_token_locked(loopback);
	request_local->token = cpu_to_le16(pending->token);
	vm_trace(&vmdev->vdev.dev, "relay req msg_id=0x%02x dev=%u tok=%u\n",
		 request->msg_id, dev_num, pending->token);

	ret = xa_err(xa_store(&loopback->pending_relay_xa, pending->relay_seq,
			      pending,
			      GFP_KERNEL));
	mutex_unlock(&loopback->pending_lock);
	if (ret)
		goto out_put_pending;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle) {
		ret = -ENODEV;
		goto out_remove_pending;
	}

	dctx.flags = VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE;
	dctx.relay_seq = pending->relay_seq;
	ret = virtio_msg_bus_bridge_device_rx(handle, request_local, &dctx);
	if (ret) {
		vm_warn_rl(&vmdev->vdev.dev,
			   "relay bridge_rx err msg_id=0x%02x ret=%d\n",
			   request->msg_id, ret);
		goto out_remove_pending;
	}

	timeout = virtio_msg_loopback_pending_timeout_jiffies(pending);
	if (timeout)
		timeout = wait_for_completion_timeout(&pending->done, timeout);
	else if (completion_done(&pending->done))
		timeout = 1;
	if (!timeout) {
		vm_warn_rl(&vmdev->vdev.dev,
			   "relay timeout msg_id=0x%02x dev=%u tok=%u\n",
			   request->msg_id, dev_num, pending->token);
		WRITE_ONCE(pending->timed_out, true);
		mutex_lock(&loopback->pending_lock);
		if (xa_erase(&loopback->pending_relay_xa, pending->relay_seq) ==
		    pending) {
			pending->completed = true;
			pending->status = -ETIMEDOUT;
		}
		mutex_unlock(&loopback->pending_lock);
		virtio_msg_bus_bridge_device_relay_drop(handle,
							pending->dev_num,
							pending->token);
		ret = -ETIMEDOUT;
		goto out_put_pending;
	}

	ret = pending->status;
	goto out_put_pending;

out_remove_pending:
	mutex_lock(&loopback->pending_lock);
	if (xa_erase(&loopback->pending_relay_xa, pending->relay_seq) ==
	    pending) {
		pending->completed = true;
		pending->status = ret;
	}
	mutex_unlock(&loopback->pending_lock);
out_put_pending:
	virtio_msg_loopback_pending_put(pending);
	return ret;
}

static int
virtio_msg_loopback_send_event(struct virtio_msg_transport_device *vmdev,
			       const struct virtio_msg *request)
{
	struct virtio_msg_dispatch_ctx dctx = {
		.flags = VIRTIO_MSG_DISPATCH_F_NONBLOCK,
	};
	const struct virtio_msg_event_avail *event;
	struct virtio_msg_loopback_slot *slot;
	struct virtio_msg_loopback *loopback;
	u16 request_size;
	u16 dev_num;
	u32 vq_index = 0;
	u32 next_offset = 0;
	int ret;
	u32 handle;
	bool avail_event = false;

	if (!vmdev || !request)
		return -EINVAL;

	loopback = vmdev->provider_data;
	if (!loopback)
		return -ENODEV;
	if (!(request->msg_id & VIRTIO_MSG_ID_EVENT_BIT))
		return -EINVAL;
	if (request->type & (VIRTIO_MSG_TYPE_RESPONSE | VIRTIO_MSG_TYPE_BUS))
		return -EINVAL;

	request_size = le16_to_cpu(request->msg_size);
	if (request_size < sizeof(*request) ||
	    request_size > VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE)
		return -EMSGSIZE;

	dev_num = le16_to_cpu(request->dev_num);
	if (dev_num != vmdev->dev_num)
		return -ENODEV;

	slot = virtio_msg_loopback_slot_get(loopback, dev_num);
	if (!slot)
		return -ENODEV;

	if (request->msg_id == VIRTIO_MSG_EVENT_AVAIL) {
		avail_event = true;
		if (request_size != sizeof(*request) + sizeof(*event))
			return -EMSGSIZE;

		event = (const struct virtio_msg_event_avail *)request->payload;
		vq_index = le32_to_cpu(event->vq_index);
		next_offset = le32_to_cpu(event->next_offset);

		if (atomic_read(&slot->pending_async_exports) > 0) {
			vm_trace(&vmdev->vdev.dev,
				 "defer EVENT_AVAIL: dev=%u vq=%u next=%u pending_exports=%d\n",
				 dev_num, vq_index, next_offset,
				 atomic_read(&slot->pending_async_exports));
			ret = virtio_msg_loopback_defer_event(slot, vq_index,
							      next_offset);
			if (!ret &&
			    atomic_read(&slot->pending_async_exports) == 0)
				(void)mod_delayed_work
					(system_wq, &slot->deferred_event_work, 0);
			return ret;
		}
	}

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle)
		return -ENODEV;

	if (avail_event)
		vm_trace(&vmdev->vdev.dev,
			 "send EVENT_AVAIL: dev=%u vq=%u next=%u handle=%u\n",
			 dev_num, vq_index, next_offset, handle);
	else
		vm_trace(&vmdev->vdev.dev,
			 "send event: msg_id=0x%02x dev=%u handle=%u\n",
			 request->msg_id, dev_num, handle);

	ret = virtio_msg_bus_bridge_device_rx(handle, request, &dctx);
	if (avail_event && (ret == -EAGAIN || ret == -ENOSPC)) {
		ret = virtio_msg_loopback_defer_event(slot, vq_index, next_offset);
		if (!ret)
			(void)mod_delayed_work(system_wq,
					       &slot->deferred_event_work, 0);
		return ret;
	}
	if (ret) {
		if (avail_event)
			vm_warn_rl(&vmdev->vdev.dev,
				   "send EVENT_AVAIL failed: dev=%u vq=%u next=%u ret=%d\n",
				   dev_num, vq_index, next_offset, ret);
		else
			vm_warn_rl(&vmdev->vdev.dev,
				   "send event failed: msg_id=0x%02x dev=%u ret=%d\n",
				   request->msg_id, dev_num, ret);
	}

	return ret;
}

static int virtio_msg_loopback_send(struct virtio_msg_transport_device *vmdev,
				    const struct virtio_msg *request,
				    struct virtio_msg *response, u32 flags)
{
	if (!vmdev || !request)
		return -EINVAL;
	if (flags & ~VIRTIO_MSG_BUS_SEND_F_NONBLOCK)
		return -EINVAL;
	if (WARN_ON_ONCE((flags & VIRTIO_MSG_BUS_SEND_F_NONBLOCK) && response))
		return -EINVAL;
	if (flags & VIRTIO_MSG_BUS_SEND_F_NONBLOCK)
		return virtio_msg_loopback_send_event(vmdev, request);

	return virtio_msg_loopback_request(vmdev, request, response);
}

static void
virtio_msg_loopback_synchronize_cbs(struct virtio_msg_transport_device *vmdev)
{
	(void)vmdev;

	/*
	 * EVENT_USED arrives only after the bridge has published the
	 * corresponding vring updates to the shared queue memory. Order the
	 * subsequent vring_interrupt() reads after observing that event.
	 */
	smp_rmb();
}

static const struct virtio_msg_bus_provider_ops virtio_msg_loopback_bus_ops = {
	.get_caps		= virtio_msg_loopback_get_caps,
	.send			= virtio_msg_loopback_send,
	.synchronize_cbs	= virtio_msg_loopback_synchronize_cbs,
};

static void virtio_msg_loopback_slot_start_workfn(struct work_struct *work)
{
	struct virtio_msg_loopback_slot *slot;
	int ret;

	slot = container_of(work, struct virtio_msg_loopback_slot, start_work);
	ret = virtio_msg_loopback_slot_start(slot->loopback, slot->dev_num);
	if (ret && ret != -EBUSY)
		vm_warn_rl(NULL, "slot start failed: dev=%u ret=%d\n",
			   slot->dev_num, ret);
}

static void
virtio_msg_loopback_slot_reset_vmdev
	(struct virtio_msg_loopback_slot *slot)
{
	if (!slot)
		return;

	memset(&slot->vmdev, 0, sizeof(slot->vmdev));
	slot->vmdev.dev_num = slot->dev_num;
}

static void virtio_msg_loopback_root_state_init(struct virtio_msg_loopback *loopback)
{
	unsigned int dev_num;

	mutex_init(&loopback->pending_lock);
	xa_init(&loopback->pending_relay_xa);
	loopback->pending_next_seq = 1;

	mutex_init(&loopback->area_lock);
	xa_init(&loopback->map_area_xa_by_id);
	xa_init(&loopback->map_area_xa_by_offset);
	xa_init(&loopback->map_area_xa_by_dma_addr);
	spin_lock_init(&loopback->exact_lock);
	INIT_LIST_HEAD(&loopback->exact_exports);
	loopback->map_next_offset = PAGE_SIZE;

	for (dev_num = 0; dev_num < VIRTIO_MSG_LOOPBACK_MAX_DEVS; dev_num++) {
		struct virtio_msg_loopback_slot *slot;

		slot = &loopback->slots[dev_num];
		memset(slot, 0, sizeof(*slot));
		slot->loopback = loopback;
		INIT_WORK(&slot->start_work,
			  virtio_msg_loopback_slot_start_workfn);
		atomic_set(&slot->pending_async_exports, 0);
		spin_lock_init(&slot->deferred_event_lock);
		INIT_LIST_HEAD(&slot->deferred_events);
		INIT_DELAYED_WORK(&slot->deferred_event_work,
				  virtio_msg_loopback_deferred_event_workfn);
		slot->dev_num = dev_num;
		virtio_msg_loopback_slot_reset_vmdev(slot);
	}
}

static void virtio_msg_loopback_root_state_destroy(struct virtio_msg_loopback *loopback)
{
	unsigned int dev_num;

	for (dev_num = 0; dev_num < VIRTIO_MSG_LOOPBACK_MAX_DEVS; dev_num++) {
		cancel_work_sync(&loopback->slots[dev_num].start_work);
		cancel_delayed_work_sync(&loopback->slots[dev_num].deferred_event_work);
		virtio_msg_loopback_slot_purge_deferred_events
			(&loopback->slots[dev_num]);
		virtio_msg_bus_dma_cleanup(&loopback->slots[dev_num].vmdev);
	}

	xa_destroy(&loopback->pending_relay_xa);
	xa_destroy(&loopback->map_area_xa_by_id);
	xa_destroy(&loopback->map_area_xa_by_offset);
	xa_destroy(&loopback->map_area_xa_by_dma_addr);
}

static struct device *
virtio_msg_loopback_slot_register_dma_dev(u16 dev_num)
{
	char dma_dev_name[sizeof(VIRTIO_MSG_LOOPBACK_DMA_DEV_NAME) + 4];

	snprintf(dma_dev_name, sizeof(dma_dev_name), "%s%u",
		 VIRTIO_MSG_LOOPBACK_DMA_DEV_NAME, dev_num);

	return root_device_register(dma_dev_name);
}

int virtio_msg_loopback_slot_start(struct virtio_msg_loopback *loopback,
				   u16 dev_num)
{
	struct virtio_msg_loopback_slot *slot;
	u64 required_features[VIRTIO_FEATURES_U64S];
	struct device *dma_dev;
	int ret;

	if (!loopback)
		return -EINVAL;

	slot = virtio_msg_loopback_slot_get(loopback, dev_num);
	if (!slot)
		return -EINVAL;
	if (slot->provider_attached && slot->transport_prepared &&
	    slot->device_registered)
		return 0;
	if (slot->provider_attached || slot->transport_prepared ||
	    slot->device_registered)
		return -EBUSY;
	vm_info(NULL, "loopback slot start: dev=%u\n", dev_num);

	dma_dev = virtio_msg_loopback_slot_register_dma_dev(slot->dev_num);
	if (IS_ERR(dma_dev))
		return PTR_ERR(dma_dev);

	ret = dma_coerce_mask_and_coherent(dma_dev, DMA_BIT_MASK(64));
	if (ret) {
		root_device_unregister(dma_dev);
		return ret;
	}

	slot->dma_dev = dma_dev;

	ret = virtio_msg_transport_attach_provider(&slot->vmdev,
						   &virtio_msg_loopback_bus_ops,
						   VIRTIO_MSG_LOOPBACK_BUS_NAME,
						   loopback);
	if (ret)
		goto out_unregister_dma_dev;
	slot->provider_attached = true;
	virtio_features_zero(required_features);
	virtio_features_set_bit(required_features, VIRTIO_F_VERSION_1);
	virtio_features_set_bit(required_features, VIRTIO_F_ACCESS_PLATFORM);
	virtio_msg_transport_set_required_features(&slot->vmdev,
						   required_features);

	ret = virtio_msg_transport_prepare_device(&slot->vmdev);
	if (ret) {
		virtio_msg_transport_detach_provider(&slot->vmdev);
		slot->provider_attached = false;
		goto out_unregister_dma_dev;
	}
	slot->transport_prepared = true;

	slot->vmdev.vdev.vmap.dma_dev = dma_dev;
	slot->vmdev.vdev.dev.parent = dma_dev;
	ret = virtio_msg_bus_dma_install(&slot->vmdev,
					 virtio_msg_loopback_bridge_dma_provider_ops_get(),
					 loopback);
	if (ret)
		goto out_unprepare;

	ret = virtio_msg_transport_register_prepared_device(&slot->vmdev);
	if (ret) {
		virtio_msg_bus_dma_cleanup(&slot->vmdev);
		goto out_unprepare;
	}
	slot->device_registered = true;

	return 0;

out_unprepare:
	if (slot->transport_prepared) {
		virtio_msg_transport_unprepare_device(&slot->vmdev);
		slot->transport_prepared = false;
	}
	if (slot->provider_attached) {
		virtio_msg_transport_detach_provider(&slot->vmdev);
		slot->provider_attached = false;
	}
out_unregister_dma_dev:
	slot->vmdev.vdev.vmap.dma_dev = NULL;
	slot->vmdev.vdev.dev.parent = NULL;
	root_device_unregister(slot->dma_dev);
	slot->dma_dev = NULL;
	virtio_msg_loopback_slot_reset_vmdev(slot);
	return ret;
}

void virtio_msg_loopback_slot_stop(struct virtio_msg_loopback *loopback,
				   u16 dev_num)
{
	struct virtio_msg_loopback_slot *slot;

	if (!loopback)
		return;

	slot = virtio_msg_loopback_slot_get(loopback, dev_num);
	if (!slot)
		return;
	vm_info(NULL, "loopback slot stop: dev=%u\n", dev_num);

	cancel_work_sync(&slot->start_work);
	cancel_delayed_work_sync(&slot->deferred_event_work);
	virtio_msg_loopback_slot_purge_deferred_events(slot);
	if (slot->device_registered) {
		virtio_msg_transport_unregister_device(&slot->vmdev);
		slot->device_registered = false;
		slot->transport_prepared = false;
	}
	if (virtio_msg_bus_dma_is_installed(&slot->vmdev))
		virtio_msg_bus_dma_cleanup(&slot->vmdev);

	if (slot->transport_prepared) {
		virtio_msg_transport_unprepare_device(&slot->vmdev);
		slot->transport_prepared = false;
	}

	if (slot->provider_attached) {
		virtio_msg_transport_detach_provider(&slot->vmdev);
		slot->provider_attached = false;
	}

	slot->vmdev.vdev.vmap.dma_dev = NULL;
	slot->vmdev.vdev.dev.parent = NULL;
	if (slot->dma_dev) {
		root_device_unregister(slot->dma_dev);
		slot->dma_dev = NULL;
	}

	virtio_msg_loopback_slot_reset_vmdev(slot);
}

void virtio_msg_loopback_slot_stop_all(struct virtio_msg_loopback *loopback)
{
	u16 dev_num;

	if (!loopback)
		return;

	for (dev_num = 0; dev_num < VIRTIO_MSG_LOOPBACK_MAX_DEVS; dev_num++)
		virtio_msg_loopback_slot_stop(loopback, dev_num);
}

static struct virtio_msg_loopback *virtio_msg_loopback;

static int __init virtio_msg_loopback_init(void)
{
	struct virtio_msg_loopback *loopback;
	int ret;

	loopback = kzalloc(sizeof(*loopback), GFP_KERNEL);
	if (!loopback)
		return -ENOMEM;
	vm_info(NULL, "loopback init\n");

	virtio_msg_loopback_root_state_init(loopback);

	ret = virtio_msg_loopback_bridge_start(loopback);
	if (ret) {
		virtio_msg_loopback_root_state_destroy(loopback);
		kfree(loopback);
		return ret;
	}

	virtio_msg_loopback = loopback;
	return 0;
}

static void __exit virtio_msg_loopback_exit(void)
{
	struct virtio_msg_loopback *loopback = virtio_msg_loopback;

	if (!loopback)
		return;
	vm_info(NULL, "loopback exit\n");

	virtio_msg_loopback = NULL;
	virtio_msg_loopback_slot_stop_all(loopback);
	virtio_msg_loopback_bridge_stop(loopback);
	virtio_msg_loopback_root_state_destroy(loopback);
	kfree(loopback);
}

module_init(virtio_msg_loopback_init);
module_exit(virtio_msg_loopback_exit);

MODULE_DESCRIPTION("Virtio message loopback base scaffold");
MODULE_LICENSE("GPL");
