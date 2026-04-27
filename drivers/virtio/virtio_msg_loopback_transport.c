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

#include "virtio_msg_bus_dma_driver_helper.h"
#include "virtio_msg_bus_queue_helper.h"
#include "virtio_msg_loopback_priv.h"

#define VIRTIO_MSG_LOOPBACK_ORDERED_QUEUE_DEPTH	32U

static int
virtio_msg_loopback_ordered_queue_submit
		(void *context,
		 const struct virtio_msg_bus_queue_helper_item *item)
{
	struct virtio_msg_loopback_slot *slot = context;

	if (!slot || !item)
		return -EINVAL;

	switch (item->type) {
	case VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_MSG:
		return virtio_msg_loopback_bridge_publish_event_sleepable
			(slot->loopback, item->vmsg);
	default:
		return -EINVAL;
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

static u16
virtio_msg_loopback_next_request_token_locked(struct virtio_msg_loopback *loopback)
{
	u16 token;

	lockdep_assert_held(&loopback->pending_lock);

	if (loopback->next_token <= VIRTIO_MSG_TOKEN_FIXED)
		loopback->next_token = VIRTIO_MSG_TOKEN_FIXED + 1;

	for (;;) {
		token = loopback->next_token++;
		if (loopback->next_token <= VIRTIO_MSG_TOKEN_FIXED)
			loopback->next_token = VIRTIO_MSG_TOKEN_FIXED + 1;
		if (token > VIRTIO_MSG_TOKEN_FIXED)
			return token;
	}
}

static u64
virtio_msg_loopback_next_relay_seq_locked(struct virtio_msg_loopback *loopback)
{
	u64 relay_seq;

	lockdep_assert_held(&loopback->pending_lock);

	if (!loopback->next_relay_seq)
		loopback->next_relay_seq = 1;

	relay_seq = loopback->next_relay_seq++;
	if (!loopback->next_relay_seq)
		loopback->next_relay_seq = 1;

	return relay_seq;
}

static int
virtio_msg_loopback_request(struct virtio_msg_transport_device *vmdev,
			    const struct virtio_msg *request,
			    struct virtio_msg *response)
{
	struct virtio_msg_dispatch_ctx dctx = { 0 };
	struct virtio_msg_loopback_relay_pending pending = { 0 };
	struct virtio_msg *request_local;
	struct virtio_msg_loopback *loopback;
	u8 request_buf[VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE];
	unsigned long deadline;
	unsigned long now;
	unsigned long timeout;
	u16 request_size;
	u16 expected_dev_num;
	u16 dev_num;
	bool pending_erased;
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

	init_completion(&pending.done);
	pending.dev_num = dev_num;
	pending.status = -ETIMEDOUT;
	pending.response = response;

	mutex_lock(&loopback->pending_lock);
	pending.relay_seq = virtio_msg_loopback_next_relay_seq_locked(loopback);
	pending.token = virtio_msg_loopback_next_request_token_locked(loopback);
	request_local->token = cpu_to_le16(pending.token);
	dev_dbg(&vmdev->vdev.dev, "relay req msg_id=0x%02x dev=%u tok=%u\n",
		request->msg_id, dev_num, pending.token);

	ret = xa_err(xa_store(&loopback->pending_relay_xa, pending.relay_seq,
			      &pending,
			      GFP_KERNEL));
	mutex_unlock(&loopback->pending_lock);
	if (ret)
		return ret;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle) {
		ret = -ENODEV;
		goto out_remove_pending;
	}

	dctx.flags = VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE;
	dctx.relay_seq = pending.relay_seq;
	deadline = jiffies +
		msecs_to_jiffies(VIRTIO_MSG_LOOPBACK_RELAY_TIMEOUT_MS);
	ret = virtio_msg_bus_bridge_device_rx(handle, request_local, &dctx);
	if (ret) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "relay bridge_rx err msg_id=0x%02x ret=%d\n",
			   request->msg_id, ret);
		goto out_remove_pending;
	}

	now = jiffies;
	if (time_after(deadline, now)) {
		timeout = wait_for_completion_timeout(&pending.done,
						      deadline - now);
	} else if (completion_done(&pending.done)) {
		timeout = 1;
	} else {
		timeout = 0;
	}
	if (!timeout) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "relay timeout msg_id=0x%02x dev=%u tok=%u\n",
			   request->msg_id, dev_num, pending.token);
		mutex_lock(&loopback->pending_lock);
		pending.timed_out = true;
		pending_erased =
			xa_erase(&loopback->pending_relay_xa,
				 pending.relay_seq) == &pending;
		if (pending_erased)
			pending.status = -ETIMEDOUT;
		mutex_unlock(&loopback->pending_lock);
		if (pending_erased) {
			virtio_msg_bus_bridge_device_relay_drop(handle,
								pending.dev_num,
								pending.token);
		} else {
			wait_for_completion(&pending.done);
		}
		ret = -ETIMEDOUT;
		return ret;
	}

	return pending.status;

out_remove_pending:
	mutex_lock(&loopback->pending_lock);
	pending_erased =
		xa_erase(&loopback->pending_relay_xa,
			 pending.relay_seq) == &pending;
	if (pending_erased)
		pending.status = ret;
	mutex_unlock(&loopback->pending_lock);
	if (!pending_erased)
		wait_for_completion(&pending.done);
	return ret;
}

static int
virtio_msg_loopback_send_event(struct virtio_msg_transport_device *vmdev,
			       const struct virtio_msg *request)
{
	const struct virtio_msg_event_avail *event;
	struct virtio_msg_loopback_slot *slot;
	struct virtio_msg_loopback *loopback;
	u16 request_size;
	u16 dev_num;
	u32 vq_index = 0;
	u32 next_offset = 0;
	int ret;
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
	}

	if (avail_event)
		dev_dbg(&vmdev->vdev.dev,
			"queue EVENT_AVAIL: dev=%u vq=%u next=%u\n",
			 dev_num, vq_index, next_offset);
	else
		dev_dbg(&vmdev->vdev.dev,
			"queue event: msg_id=0x%02x dev=%u\n",
			 request->msg_id, dev_num);

	ret = virtio_msg_bus_queue_helper_enqueue(&slot->ordered_queue, request,
						  GFP_ATOMIC);
	if (ret) {
		if (avail_event)
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "queue EVENT_AVAIL failed: dev=%u vq=%u next=%u ret=%d\n",
				   dev_num, vq_index, next_offset, ret);
		else
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "queue event failed: msg_id=0x%02x dev=%u ret=%d\n",
				   request->msg_id, dev_num, ret);
	}

	return ret;
}

static int virtio_msg_loopback_send(struct virtio_msg_transport_device *vmdev,
				    const struct virtio_msg *request,
				    struct virtio_msg *response)
{
	if (!vmdev || !request)
		return -EINVAL;
	if (!response)
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

static void
virtio_msg_loopback_slot_stop_sync(struct virtio_msg_loopback *loopback,
				   u16 dev_num, bool cancel_stop_work);

static void virtio_msg_loopback_slot_start_workfn(struct work_struct *work)
{
	struct virtio_msg_loopback_slot *slot;
	int ret;

	slot = container_of(work, struct virtio_msg_loopback_slot, start_work);
	ret = virtio_msg_loopback_slot_start(slot->loopback, slot->dev_num);
	if (ret && ret != -EBUSY && ret != -EAGAIN && ret != -ENODEV &&
	    ret != -ENOTCONN)
		pr_warn_ratelimited("slot start failed: dev=%u ret=%d\n",
				    slot->dev_num, ret);
}

static void virtio_msg_loopback_slot_stop_workfn(struct work_struct *work)
{
	struct virtio_msg_loopback_slot *slot;
	int ret;

	slot = container_of(work, struct virtio_msg_loopback_slot, stop_work);
	virtio_msg_loopback_slot_stop_sync(slot->loopback, slot->dev_num, false);

	ret = virtio_msg_loopback_bridge_slot_can_start(slot->loopback,
							slot->dev_num);
	if (!ret)
		schedule_work(&slot->start_work);
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
	loopback->next_relay_seq = 1;
	loopback->next_token = VIRTIO_MSG_TOKEN_FIXED + 1;

	mutex_init(&loopback->area_lock);
	xa_init(&loopback->map_area_xa_by_dma_addr);
	virtio_msg_bus_dma_device_helper_init(&loopback->map_helper);
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
		INIT_WORK(&slot->stop_work,
			  virtio_msg_loopback_slot_stop_workfn);
		slot->dev_num = dev_num;
		virtio_msg_loopback_slot_reset_vmdev(slot);
	}
}

static void virtio_msg_loopback_root_state_destroy(struct virtio_msg_loopback *loopback)
{
	unsigned int dev_num;

	for (dev_num = 0; dev_num < VIRTIO_MSG_LOOPBACK_MAX_DEVS; dev_num++) {
		cancel_work_sync(&loopback->slots[dev_num].stop_work);
		cancel_work_sync(&loopback->slots[dev_num].start_work);
		virtio_msg_bus_queue_helper_stop
			(&loopback->slots[dev_num].ordered_queue);
		virtio_msg_bus_dma_driver_helper_cleanup
			(&loopback->slots[dev_num].vmdev);
	}

	xa_destroy(&loopback->pending_relay_xa);
	virtio_msg_bus_dma_device_helper_destroy(&loopback->map_helper);
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
	if (atomic_read(&slot->stopping))
		return -EBUSY;
	if (slot->provider_attached && slot->transport_prepared &&
	    slot->device_registered)
		return 0;
	if (slot->provider_attached || slot->transport_prepared ||
	    slot->device_registered)
		return -EBUSY;

	ret = virtio_msg_loopback_bridge_slot_can_start(loopback, dev_num);
	if (ret)
		return ret;

	pr_debug("loopback slot start: dev=%u\n", dev_num);

	dma_dev = virtio_msg_loopback_slot_register_dma_dev(slot->dev_num);
	if (IS_ERR(dma_dev))
		return PTR_ERR(dma_dev);

	ret = dma_coerce_mask_and_coherent(dma_dev, DMA_BIT_MASK(64));
	if (ret) {
		root_device_unregister(dma_dev);
		return ret;
	}

	slot->dma_dev = dma_dev;

	ret = virtio_msg_bus_queue_helper_init
		(&slot->ordered_queue, dma_dev, VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE,
		 VIRTIO_MSG_LOOPBACK_ORDERED_QUEUE_DEPTH,
		 VIRTIO_MSG_LOOPBACK_RELAY_TIMEOUT_MS,
		 virtio_msg_loopback_ordered_queue_submit,
		 NULL, slot);
	if (ret)
		goto out_unregister_dma_dev;

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

	ret = virtio_msg_loopback_bridge_slot_can_start(loopback, dev_num);
	if (ret)
		goto out_unprepare;

	virtio_msg_loopback_bridge_dma_install_begin(slot);
	ret = virtio_msg_bus_dma_driver_helper_install
		(&slot->vmdev,
		 virtio_msg_loopback_bridge_dma_provider_ops_get(), loopback);
	if (!ret)
		ret = virtio_msg_loopback_bridge_dma_install_finish(slot);
	else
		(void)virtio_msg_loopback_bridge_dma_install_finish(slot);
	if (ret) {
		virtio_msg_bus_dma_driver_helper_cleanup(&slot->vmdev);
		goto out_unprepare;
	}

	ret = virtio_msg_loopback_bridge_slot_can_start(loopback, dev_num);
	if (ret) {
		virtio_msg_bus_dma_driver_helper_cleanup(&slot->vmdev);
		goto out_unprepare;
	}

	ret = virtio_msg_transport_register_prepared_device(&slot->vmdev);
	if (ret) {
		virtio_msg_bus_dma_driver_helper_cleanup(&slot->vmdev);
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
	virtio_msg_bus_queue_helper_stop(&slot->ordered_queue);
	slot->vmdev.vdev.vmap.dma_dev = NULL;
	slot->vmdev.vdev.dev.parent = NULL;
	root_device_unregister(slot->dma_dev);
	slot->dma_dev = NULL;
	virtio_msg_loopback_slot_reset_vmdev(slot);
	return ret;
}

static void
virtio_msg_loopback_slot_stop_sync(struct virtio_msg_loopback *loopback,
				   u16 dev_num, bool cancel_stop_work)
{
	struct virtio_msg_loopback_slot *slot;

	if (!loopback)
		return;

	slot = virtio_msg_loopback_slot_get(loopback, dev_num);
	if (!slot)
		return;
	pr_debug("loopback slot stop: dev=%u\n", dev_num);

	atomic_inc(&slot->stopping);
	if (cancel_stop_work)
		cancel_work_sync(&slot->stop_work);
	cancel_work_sync(&slot->start_work);
	virtio_msg_bus_queue_helper_stop(&slot->ordered_queue);
	if (slot->device_registered) {
		virtio_msg_transport_unregister_device(&slot->vmdev);
		slot->device_registered = false;
		slot->transport_prepared = false;
	}
	if (virtio_msg_bus_dma_driver_helper_is_installed(&slot->vmdev))
		virtio_msg_bus_dma_driver_helper_cleanup(&slot->vmdev);

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
	atomic_dec(&slot->stopping);
}

void virtio_msg_loopback_slot_stop(struct virtio_msg_loopback *loopback,
				   u16 dev_num)
{
	virtio_msg_loopback_slot_stop_sync(loopback, dev_num, true);
}

void virtio_msg_loopback_slot_stop_async(struct virtio_msg_loopback *loopback,
					 u16 dev_num)
{
	struct virtio_msg_loopback_slot *slot;

	if (!loopback)
		return;

	slot = virtio_msg_loopback_slot_get(loopback, dev_num);
	if (!slot)
		return;

	schedule_work(&slot->stop_work);
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
	pr_debug("loopback init\n");

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
	pr_debug("loopback exit\n");

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
