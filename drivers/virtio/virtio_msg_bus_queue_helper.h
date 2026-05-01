/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus queue helper.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_QUEUE_HELPER_H
#define _VIRTIO_MSG_BUS_QUEUE_HELPER_H

#include <linux/gfp.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct device;
struct virtio_msg;

enum virtio_msg_bus_queue_helper_item_type {
	VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_MSG = 1,
	VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_OPAQUE,
};

/**
 * struct virtio_msg_bus_queue_helper_item - queued bus work item
 * @type: Item payload type.
 * @msg_size: Size of @vmsg when @type is
 *	%VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_MSG.
 * @vmsg: Copied virtio message, valid for message items.
 * @opaque: Bus-owned pointer, valid for opaque items.
 *
 * Callback users must treat this object as borrowed storage.  The helper owns
 * the backing entry and releases it after the submit/release callbacks return.
 */
struct virtio_msg_bus_queue_helper_item {
	enum virtio_msg_bus_queue_helper_item_type type;
	u16 msg_size;
	union {
		const struct virtio_msg *vmsg;
		void *opaque;
	};
};

typedef int (*virtio_msg_bus_queue_helper_submit_fn)(void *context,
		const struct virtio_msg_bus_queue_helper_item *item);
typedef void (*virtio_msg_bus_queue_helper_release_fn)(void *context,
		const struct virtio_msg_bus_queue_helper_item *item, int status);

/**
 * struct virtio_msg_bus_queue_helper - ordered sleepable submit queue
 * @lock: Protects @entries, @depth, and @stopped.
 * @work: Worker that drains entries in FIFO order.
 * @entries: Pending copied messages and opaque entries.
 * @dev: Device used for diagnostics.
 * @submit: Sleepable submit callback.
 * @release: Optional callback invoked when helper ownership ends.
 * @context: Callback context.
 * @max_msg_size: Largest accepted message copy.
 * @max_depth: Maximum number of queued entries.
 * @retention_timeout_jiffies: Maximum age of a queued entry, or zero to
 *	disable age-based expiration.
 * @depth: Current number of queued entries.
 * @stopped: Reject new entries and make the worker drop queued entries.
 * @initialized: True once the queue has been initialized.
 */
struct virtio_msg_bus_queue_helper {
	spinlock_t lock; /* Protects queued entries and stop state. */
	struct delayed_work work;
	struct list_head entries;
	struct device *dev;
	virtio_msg_bus_queue_helper_submit_fn submit;
	virtio_msg_bus_queue_helper_release_fn release;
	void *context;
	u32 max_msg_size;
	u32 max_depth;
	unsigned long retention_timeout_jiffies;
	u32 depth;
	bool stopped;
	bool initialized;
};

/**
 * virtio_msg_bus_queue_helper_init() - initialize an ordered submit queue
 * @queue: Queue helper to initialize.
 * @dev: Device used for diagnostics.
 * @max_msg_size: Largest accepted message copy.
 * @max_depth: Maximum number of queued entries.
 * @retention_timeout_ms: Maximum age of a queued entry in milliseconds. Zero
 *	disables age-based expiration.
 * @submit: Sleepable submit callback.
 * @release: Optional callback invoked when helper ownership ends.
 * @context: Callback context.
 *
 * Nonzero retention timeouts are measured from enqueue time. The helper drops
 * an entry if it is still queued after its retention deadline. Submit failures
 * are reported once through @release and are not retried by the helper.
 */
int
virtio_msg_bus_queue_helper_init(struct virtio_msg_bus_queue_helper *queue,
				 struct device *dev, u32 max_msg_size,
				 u32 max_depth,
				 unsigned int retention_timeout_ms,
				 virtio_msg_bus_queue_helper_submit_fn submit,
				 virtio_msg_bus_queue_helper_release_fn release,
				 void *context);
int
virtio_msg_bus_queue_helper_enqueue(struct virtio_msg_bus_queue_helper *queue,
				    const struct virtio_msg *vmsg, gfp_t gfp);
int
virtio_msg_bus_queue_helper_enqueue_opaque(struct virtio_msg_bus_queue_helper
					   *queue, void *opaque, gfp_t gfp);
void virtio_msg_bus_queue_helper_stop(struct virtio_msg_bus_queue_helper *queue);

#endif /* _VIRTIO_MSG_BUS_QUEUE_HELPER_H */
