// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A device-role event runtime.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/byteorder/little_endian.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "virtio_msg_bus_ffa_device_priv.h"
#include "virtio_msg_bus_ffa_xfer.h"

#define VIRTIO_MSG_FFA_EVENT_QUEUE_DEPTH		128
#define VIRTIO_MSG_FFA_EVENT_MSG_SIZE			\
	(sizeof(struct virtio_msg) + sizeof(struct virtio_msg_bus_event_device))

struct virtio_msg_ffa_device_event_entry {
	u64 generation;
	u16 msg_size;
	u8 msg[];
};

int virtio_msg_ffa_device_send_selected_locked(struct virtio_msg_ffa_device *vdev,
					       const struct virtio_msg *msg,
					       size_t msg_len)
{
	const struct virtio_msg_ffa_device_method *method;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	method = vdev->runtime_method;
	if (!method || !method->ops || !method->ops->send_locked)
		return -EOPNOTSUPP;

	return method->ops->send_locked(vdev, msg, msg_len);
}

static int
virtio_msg_ffa_device_send_event_locked(struct virtio_msg_ffa_device *vdev,
					const struct virtio_msg *msg,
					size_t msg_len)
{
	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (!vdev->event_configured)
		return -EACCES;

	return virtio_msg_ffa_device_send_selected_locked(vdev, msg, msg_len);
}

static bool
virtio_msg_ffa_device_event_valid(const struct virtio_msg *msg, size_t msg_len)
{
	const struct virtio_msg_bus_event_device *event;
	u16 dev_state;

	if (!msg || msg_len != VIRTIO_MSG_FFA_EVENT_MSG_SIZE)
		return false;
	if (le16_to_cpu(msg->msg_size) != msg_len)
		return false;
	if (msg->type != (VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_REQUEST))
		return false;
	if (msg->msg_id != VIRTIO_MSG_BUS_EVENT_DEVICE)
		return false;
	if (le16_to_cpu(msg->dev_num) != 0)
		return false;

	event = (const struct virtio_msg_bus_event_device *)msg->payload;
	dev_state = le16_to_cpu(event->dev_state);
	switch (dev_state) {
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_ADDED:
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED:
		return true;
	default:
		return false;
	}
}

static int
virtio_msg_ffa_device_event_queue_submit
		(void *context,
		 const struct virtio_msg_bus_queue_helper_item *item)
{
	struct virtio_msg_ffa_device_event_entry *entry;
	struct virtio_msg_ffa_device *vdev = context;
	const struct virtio_msg *msg;
	u8 wire_buf[VIRTIO_MSG_FFA_EVENT_MSG_SIZE];
	struct virtio_msg *wire_msg = (struct virtio_msg *)wire_buf;
	u16 token;
	int ret;

	if (!vdev || !item ||
	    item->type != VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_OPAQUE)
		return -EINVAL;

	entry = item->opaque;
	if (!entry)
		return -EINVAL;

	msg = (const struct virtio_msg *)entry->msg;
	if (!virtio_msg_ffa_device_event_valid(msg, entry->msg_size))
		return -EINVAL;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down) ||
	    entry->generation != READ_ONCE(vdev->event_generation) ||
	    !vdev->event_configured) {
		ret = 0;
	} else {
		memcpy(wire_msg, msg, entry->msg_size);
		ret = virtio_msg_ffa_next_token_locked(&vdev->ep, &token);
		if (ret)
			goto out_unlock;
		wire_msg->token = cpu_to_le16(token);
		ret = virtio_msg_ffa_device_send_event_locked
			(vdev, wire_msg, entry->msg_size);
	}
out_unlock:
	mutex_unlock(&vdev->lock);

	return ret;
}

static void
virtio_msg_ffa_device_event_queue_release
		(void *context,
		 const struct virtio_msg_bus_queue_helper_item *item, int status)
{
	(void)context;
	(void)status;

	if (!item || item->type != VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_OPAQUE)
		return;

	kfree(item->opaque);
}

int virtio_msg_ffa_device_event_enqueue(struct virtio_msg_ffa_device *vdev,
					u16 dev_num, u16 dev_state)
{
	struct virtio_msg_ffa_device_event_entry *entry;
	struct virtio_msg_bus_event_device event;
	struct virtio_msg *req;
	size_t msg_size = VIRTIO_MSG_FFA_EVENT_MSG_SIZE;
	u64 generation;
	int ret;

	if (!vdev)
		return -EINVAL;
	generation = READ_ONCE(vdev->event_generation);
	if (!READ_ONCE(vdev->event_configured)) {
		if (vdev->fdev)
			dev_dbg(&vdev->fdev->dev,
				"device event dropped before event configuration dev=%u state=%u\n",
				dev_num, dev_state);
		else
			pr_debug("device event dropped before event configuration dev=%u state=%u\n",
				 dev_num, dev_state);
		return 0;
	}

	entry = kzalloc(struct_size(entry, msg, msg_size), GFP_ATOMIC);
	if (!entry)
		return -ENOSPC;

	entry->generation = generation;
	entry->msg_size = msg_size;

	req = (struct virtio_msg *)entry->msg;
	virtio_msg_prepare(req, VIRTIO_MSG_BUS_EVENT_DEVICE,
			   VIRTIO_MSG_TOKEN_FIXED, sizeof(event), 0);
	req->type = VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_REQUEST;

	event.dev_num = cpu_to_le16(dev_num);
	event.dev_state = cpu_to_le16(dev_state);
	memcpy(req->payload, &event, sizeof(event));

	ret = virtio_msg_bus_queue_helper_enqueue_opaque(&vdev->event_queue,
							 entry, GFP_ATOMIC);
	if (ret) {
		kfree(entry);
		if (ret == -ENOMEM)
			ret = -ENOSPC;
	}
	return ret;
}

void virtio_msg_ffa_device_event_runtime_reset_locked(struct virtio_msg_ffa_device *vdev)
{
	u64 event_generation;

	lockdep_assert_held(&vdev->lock);

	if (!vdev)
		return;

	vdev->event_configured = false;
	event_generation = READ_ONCE(vdev->event_generation) + 1;
	WRITE_ONCE(vdev->event_generation, event_generation);
	virtio_msg_ffa_device_runtime_clear_locked(vdev);
}

int virtio_msg_ffa_device_event_runtime_init(struct virtio_msg_ffa_device *vdev)
{
	struct device *dev;
	int ret;

	if (!vdev)
		return -EINVAL;

	dev = vdev->fdev ? &vdev->fdev->dev : NULL;
	ret = virtio_msg_bus_queue_helper_init
		(&vdev->event_queue, dev, VIRTIO_MSG_FFA_EVENT_MSG_SIZE,
		 VIRTIO_MSG_FFA_EVENT_QUEUE_DEPTH,
		 VIRTIO_MSG_FFA_RELAY_TIMEOUT_MS,
		 virtio_msg_ffa_device_event_queue_submit,
		 virtio_msg_ffa_device_event_queue_release, vdev);
	if (ret)
		return ret;

	vdev->event_configured = false;
	WRITE_ONCE(vdev->event_generation, 0);

	return 0;
}

void virtio_msg_ffa_device_event_runtime_quiesce(struct virtio_msg_ffa_device *vdev)
{
	if (!vdev)
		return;

	virtio_msg_bus_queue_helper_stop(&vdev->event_queue);
}
