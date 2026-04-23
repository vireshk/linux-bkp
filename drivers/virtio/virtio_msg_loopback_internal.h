/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message loopback base scaffold internal interfaces.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_LOOPBACK_INTERNAL_H
#define _VIRTIO_MSG_LOOPBACK_INTERNAL_H

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/dma-map-ops.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/virtio_msg_bus_bridge_device.h>
#include <linux/virtio_msg_transport.h>
#include <linux/workqueue.h>
#include <linux/xarray.h>

#include "virtio_msg_bus_dma.h"

#define VIRTIO_MSG_LOOPBACK_BUS_NAME		"loopback"
#define VIRTIO_MSG_LOOPBACK_BUS_ID		"default"
#define VIRTIO_MSG_LOOPBACK_DMA_DEV_NAME	"virtio_msg_loopback_dma."
#define VIRTIO_MSG_LOOPBACK_MAX_DEVS		32
#define VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE	128U
#define VIRTIO_MSG_LOOPBACK_RELAY_TIMEOUT_MS	5000U
#define VIRTIO_MSG_LOOPBACK_RELAY_RETRY_DELAY_MS	10U

enum virtio_msg_loopback_map_state {
	VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY = 1,
	VIRTIO_MSG_LOOPBACK_MAP_STATE_ACTIVE = 2,
	VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING = 3,
};

struct device;

struct virtio_msg_loopback_deferred_event {
	struct list_head node;
	u32 vq_index;
	u32 next_offset;
};

struct virtio_msg_loopback_relay_pending {
	struct completion done;
	u64 relay_seq;
	u16 dev_num;
	u16 token;
	int status;
	u16 response_capacity;
	u16 response_size;
	bool completed;
	bool timed_out;
	unsigned long deadline;
	struct virtio_msg *response;
	refcount_t refs;
};

struct virtio_msg_loopback_page_window {
	u64 map_id;
	u64 mmap_offset;
	u64 bus_addr;
	u64 length;
	unsigned int owner_count;
	enum virtio_msg_loopback_map_state state;
	bool del_req_queued;
	struct completion revoke_done;
	int revoke_status;
	bool revoke_settled;
	struct completion map_add_done; /* signalled when ADD ACK_OK or REJECT */
	int map_add_status;             /* 0 on ACK_OK, -ENOMEM on REJECT */
	bool map_add_settled;           /* true once map_add_done is signalled */
	bool map_add_async;             /* true when async activation owns retries */
	refcount_t refs;
	void *kva;
	struct device *dma_dev;
	void *dma_cpu_addr;
	dma_addr_t dma_addr;
	unsigned long dma_attrs;
	bool dma_mmap;
	struct page *page;
	unsigned int npages; /* 1 for single-page windows, >1 for multi-page pool windows */
};

struct virtio_msg_loopback_slot;

struct virtio_msg_loopback_exact_export {
	struct list_head node;
	refcount_t refs;
	bool dead;
	bool activate_ref_held;
	bool activate_running;
	bool activate_requeue;

	struct virtio_msg_bus_dma_export provider_export;
	struct virtio_msg_loopback_slot *slot;
	struct delayed_work activate_work;
	bool async_activate;
	bool gate_held;
	bool windows_ready;

	unsigned int num_page_windows;
	struct virtio_msg_loopback_page_window **page_windows;
	unsigned int num_unique_windows;
	struct virtio_msg_loopback_page_window **unique_windows;
};

struct virtio_msg_loopback;

struct virtio_msg_loopback_bridge {
	struct virtio_msg_bus_bridge_device endpoint;
	struct vmsg_bus_resolver resolver;
	char bus_id[VMSG_BRIDGE_UAPI_BUS_ID_LEN];
	bool resolver_registered;
	bool endpoint_registered;
};

struct virtio_msg_loopback_slot {
	struct virtio_msg_loopback *loopback;
	struct virtio_msg_transport_device vmdev;
	struct device *dma_dev;
	u16 dev_num;
	struct work_struct start_work;
	atomic_t pending_async_exports;
	spinlock_t deferred_event_lock; /* Protects deferred_events updates. */
	struct list_head deferred_events;
	struct delayed_work deferred_event_work;
	bool provider_attached;
	bool transport_prepared;
	bool device_registered;
};

struct virtio_msg_loopback {
	struct virtio_msg_loopback_bridge bridge;
	struct virtio_msg_loopback_slot slots[VIRTIO_MSG_LOOPBACK_MAX_DEVS];
	/* Protects pending_relay_xa (keyed by relay_seq) and pending_next_seq updates. */
	struct mutex pending_lock;
	struct xarray pending_relay_xa;
	u64 pending_next_seq;
	/* Protects exact-export/page-window state and map_area_xa_* tables. */
	struct mutex area_lock;
	struct xarray map_area_xa_by_id;
	struct xarray map_area_xa_by_offset;
	struct xarray map_area_xa_by_dma_addr;
	spinlock_t exact_lock; /* Protects exact_exports list and atomic transitions. */
	struct list_head exact_exports;
	struct workqueue_struct *exact_wq;
	u64 map_next_offset;
};

int virtio_msg_loopback_bridge_start(struct virtio_msg_loopback *loopback);
void virtio_msg_loopback_bridge_stop(struct virtio_msg_loopback *loopback);
int virtio_msg_loopback_slot_start(struct virtio_msg_loopback *loopback,
				   u16 dev_num);
void virtio_msg_loopback_slot_stop(struct virtio_msg_loopback *loopback,
				   u16 dev_num);
void virtio_msg_loopback_slot_stop_all(struct virtio_msg_loopback *loopback);
int virtio_msg_loopback_pending_complete_response
	(struct virtio_msg_loopback *loopback, const struct virtio_msg *msg,
	 u16 msg_size, const struct virtio_msg_dispatch_ctx *dctx);
void virtio_msg_loopback_pending_abort_all(struct virtio_msg_loopback *loopback,
					   int status);
const struct virtio_msg_bus_dma_provider_ops *
virtio_msg_loopback_bridge_dma_provider_ops_get(void);

static inline bool virtio_msg_loopback_dev_num_valid(u16 dev_num)
{
	return dev_num < VIRTIO_MSG_LOOPBACK_MAX_DEVS;
}

static inline struct virtio_msg_loopback_slot *
virtio_msg_loopback_slot_get(struct virtio_msg_loopback *loopback, u16 dev_num)
{
	if (!loopback || !virtio_msg_loopback_dev_num_valid(dev_num))
		return NULL;

	return &loopback->slots[dev_num];
}

static inline void
virtio_msg_loopback_pending_get(struct virtio_msg_loopback_relay_pending *pending)
{
	refcount_inc(&pending->refs);
}

static inline void
virtio_msg_loopback_pending_put(struct virtio_msg_loopback_relay_pending *pending)
{
	if (refcount_dec_and_test(&pending->refs))
		kfree(pending);
}

static inline void
virtio_msg_loopback_page_window_get(struct virtio_msg_loopback_page_window *window)
{
	refcount_inc(&window->refs);
}

static inline void
virtio_msg_loopback_page_window_put(struct virtio_msg_loopback_page_window *window)
{
	if (!window)
		return;

	if (refcount_dec_and_test(&window->refs)) {
		if (window->kva)
			vunmap(window->kva);
		if (window->page)
			put_page(window->page);
		kfree(window);
	}
}

static inline void
virtio_msg_loopback_exact_export_free(struct virtio_msg_loopback_exact_export *exact)
{
	unsigned int i;

	if (!exact)
		return;

	for (i = 0; i < exact->num_page_windows; i++)
		virtio_msg_loopback_page_window_put(exact->page_windows[i]);
	kfree(exact->unique_windows);
	kfree(exact->page_windows);
	kfree(exact);
}

static inline void
virtio_msg_loopback_exact_get(struct virtio_msg_loopback_exact_export *exact)
{
	refcount_inc(&exact->refs);
}

static inline void
virtio_msg_loopback_exact_put(struct virtio_msg_loopback_exact_export *exact)
{
	if (!exact)
		return;

	if (refcount_dec_and_test(&exact->refs))
		virtio_msg_loopback_exact_export_free(exact);
}

#endif /* _VIRTIO_MSG_LOOPBACK_INTERNAL_H */
