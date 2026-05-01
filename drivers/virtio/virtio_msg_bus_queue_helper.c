// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus queue helper.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-bus-queue-helper: " fmt

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/virtio_msg_protocol.h>

#include "virtio_msg_bus_queue_helper.h"

struct virtio_msg_bus_queue_entry {
	struct list_head node;
	struct virtio_msg_bus_queue_helper_item item;
	unsigned long retention_deadline;
	u8 msg[];
};

static const char *
virtio_msg_bus_queue_helper_item_name
		(const struct virtio_msg_bus_queue_helper_item *item)
{
	if (!item)
		return "unknown";

	switch (item->type) {
	case VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_MSG:
		return "message";
	case VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_OPAQUE:
		return "opaque";
	default:
		return "unknown";
	}
}

static void
virtio_msg_bus_queue_helper_release_entry
		(struct virtio_msg_bus_queue_helper *queue,
		 struct virtio_msg_bus_queue_entry *entry, int status)
{
	if (!entry)
		return;

	if (queue->release)
		queue->release(queue->context, &entry->item, status);
	kfree(entry);
}

static void virtio_msg_bus_queue_helper_drop_entries
		(struct virtio_msg_bus_queue_helper *queue, struct list_head *entries,
		 int status)
{
	struct virtio_msg_bus_queue_entry *entry, *tmp;

	list_for_each_entry_safe(entry, tmp, entries, node) {
		list_del(&entry->node);
		if (queue->dev) {
			if (entry->item.type ==
			    VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_MSG) {
				dev_warn_ratelimited(queue->dev,
						     "drop queued message: msg_id=0x%02x status=%d\n",
						     entry->item.vmsg->msg_id,
						     status);
			} else {
				dev_warn_ratelimited(queue->dev,
						     "drop queued %s item: status=%d\n",
						     virtio_msg_bus_queue_helper_item_name
							(&entry->item),
						     status);
			}
		}
		virtio_msg_bus_queue_helper_release_entry(queue, entry, status);
	}
}

static struct virtio_msg_bus_queue_entry *
virtio_msg_bus_queue_helper_pop_locked(struct virtio_msg_bus_queue_helper *queue)
{
	struct virtio_msg_bus_queue_entry *entry;

	lockdep_assert_held(&queue->lock);

	if (list_empty(&queue->entries))
		return NULL;

	entry = list_first_entry(&queue->entries,
				 struct virtio_msg_bus_queue_entry, node);
	list_del(&entry->node);
	queue->depth--;
	return entry;
}

static void virtio_msg_bus_queue_helper_workfn(struct work_struct *work)
{
	struct virtio_msg_bus_queue_helper *queue;
	struct virtio_msg_bus_queue_entry *entry;
	struct delayed_work *dwork;
	LIST_HEAD(dropped);
	unsigned long flags;
	int ret;

	dwork = to_delayed_work(work);
	queue = container_of(dwork, struct virtio_msg_bus_queue_helper, work);

	for (;;) {
		spin_lock_irqsave(&queue->lock, flags);
		if (queue->stopped) {
			list_splice_init(&queue->entries, &dropped);
			queue->depth = 0;
			spin_unlock_irqrestore(&queue->lock, flags);
			virtio_msg_bus_queue_helper_drop_entries(queue, &dropped,
								 -ECANCELED);
			return;
		}

		entry = virtio_msg_bus_queue_helper_pop_locked(queue);
		spin_unlock_irqrestore(&queue->lock, flags);
		if (!entry)
			return;

		if (queue->retention_timeout_jiffies &&
		    time_after(jiffies, entry->retention_deadline)) {
			if (queue->dev) {
				if (entry->item.type ==
				    VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_MSG) {
					dev_warn_ratelimited(queue->dev,
							     "drop expired message: msg_id=0x%02x\n",
							     entry->item.vmsg->msg_id);
				} else {
					dev_warn_ratelimited(queue->dev,
							     "drop expired %s item\n",
							     virtio_msg_bus_queue_helper_item_name
								(&entry->item));
				}
			}
			virtio_msg_bus_queue_helper_release_entry(queue, entry,
								  -ETIMEDOUT);
			continue;
		}

		ret = queue->submit(queue->context, &entry->item);
		if (ret && queue->dev) {
			if (entry->item.type ==
			    VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_MSG) {
				dev_warn_ratelimited(queue->dev,
						     "queued message failed: msg_id=0x%02x ret=%d\n",
						     entry->item.vmsg->msg_id,
						     ret);
			} else {
				dev_warn_ratelimited(queue->dev,
						     "queued %s item failed: ret=%d\n",
						     virtio_msg_bus_queue_helper_item_name
							(&entry->item),
						     ret);
			}
		}
		virtio_msg_bus_queue_helper_release_entry(queue, entry, ret);
	}
}

int
virtio_msg_bus_queue_helper_init(struct virtio_msg_bus_queue_helper *queue,
				 struct device *dev, u32 max_msg_size,
				 u32 max_depth,
				 unsigned int retention_timeout_ms,
				 virtio_msg_bus_queue_helper_submit_fn submit,
				 virtio_msg_bus_queue_helper_release_fn release,
				 void *context)
{
	if (!queue || !submit || !max_msg_size || !max_depth)
		return -EINVAL;

	spin_lock_init(&queue->lock);
	INIT_DELAYED_WORK(&queue->work, virtio_msg_bus_queue_helper_workfn);
	INIT_LIST_HEAD(&queue->entries);
	queue->dev = dev;
	queue->submit = submit;
	queue->release = release;
	queue->context = context;
	queue->max_msg_size = max_msg_size;
	queue->max_depth = max_depth;
	queue->retention_timeout_jiffies =
		msecs_to_jiffies(retention_timeout_ms);
	queue->depth = 0;
	queue->stopped = false;
	queue->initialized = true;

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_queue_helper_init);

int
virtio_msg_bus_queue_helper_enqueue(struct virtio_msg_bus_queue_helper *queue,
				    const struct virtio_msg *vmsg, gfp_t gfp)
{
	struct virtio_msg_bus_queue_entry *entry;
	unsigned long flags;
	u16 msg_size;
	int ret = 0;

	if (!queue || !vmsg)
		return -EINVAL;
	if (!queue->initialized)
		return -ENODEV;

	msg_size = le16_to_cpu(vmsg->msg_size);
	if (msg_size < sizeof(*vmsg) || msg_size > queue->max_msg_size)
		return -EMSGSIZE;

	entry = kzalloc(sizeof(*entry) + msg_size, gfp);
	if (!entry)
		return -ENOMEM;

	INIT_LIST_HEAD(&entry->node);
	if (queue->retention_timeout_jiffies)
		entry->retention_deadline =
			jiffies + queue->retention_timeout_jiffies;
	entry->item.type = VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_MSG;
	entry->item.msg_size = msg_size;
	entry->item.vmsg = (const struct virtio_msg *)entry->msg;
	memcpy(entry->msg, vmsg, msg_size);

	spin_lock_irqsave(&queue->lock, flags);
	if (queue->stopped) {
		ret = -ESHUTDOWN;
	} else if (queue->depth >= queue->max_depth) {
		ret = -ENOSPC;
	} else {
		list_add_tail(&entry->node, &queue->entries);
		queue->depth++;
		entry = NULL;
		queue_delayed_work(system_wq, &queue->work, 0);
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	kfree(entry);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_queue_helper_enqueue);

int
virtio_msg_bus_queue_helper_enqueue_opaque(struct virtio_msg_bus_queue_helper
					   *queue, void *opaque, gfp_t gfp)
{
	struct virtio_msg_bus_queue_entry *entry;
	unsigned long flags;
	int ret = 0;

	if (!queue || !opaque)
		return -EINVAL;
	if (!queue->initialized)
		return -ENODEV;

	entry = kzalloc(sizeof(*entry), gfp);
	if (!entry)
		return -ENOMEM;

	INIT_LIST_HEAD(&entry->node);
	if (queue->retention_timeout_jiffies)
		entry->retention_deadline =
			jiffies + queue->retention_timeout_jiffies;
	entry->item.type = VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_OPAQUE;
	entry->item.opaque = opaque;

	spin_lock_irqsave(&queue->lock, flags);
	if (queue->stopped) {
		ret = -ESHUTDOWN;
	} else if (queue->depth >= queue->max_depth) {
		ret = -ENOSPC;
	} else {
		list_add_tail(&entry->node, &queue->entries);
		queue->depth++;
		entry = NULL;
		queue_delayed_work(system_wq, &queue->work, 0);
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	kfree(entry);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_queue_helper_enqueue_opaque);

void virtio_msg_bus_queue_helper_stop(struct virtio_msg_bus_queue_helper *queue)
{
	LIST_HEAD(dropped);
	unsigned long flags;

	if (!queue || !queue->initialized)
		return;

	spin_lock_irqsave(&queue->lock, flags);
	queue->stopped = true;
	list_splice_init(&queue->entries, &dropped);
	queue->depth = 0;
	spin_unlock_irqrestore(&queue->lock, flags);

	cancel_delayed_work_sync(&queue->work);
	virtio_msg_bus_queue_helper_drop_entries(queue, &dropped, -ECANCELED);
	queue->initialized = false;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_queue_helper_stop);

MODULE_DESCRIPTION("Virtio message bus queue helper");
MODULE_LICENSE("GPL");
