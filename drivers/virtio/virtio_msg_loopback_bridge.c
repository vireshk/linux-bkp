// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message loopback bridge base scaffold.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-loopback: " fmt

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#include "virtio_msg_loopback_internal.h"
#define VM_LOG_COMPONENT VM_LOG_COMPONENT_LOOPBACK
#include "virtio_msg_debug.h"

enum virtio_msg_loopback_origin {
	VIRTIO_MSG_LOOPBACK_ORIGIN_BRIDGE = 0,
	VIRTIO_MSG_LOOPBACK_ORIGIN_TRANSPORT_REQUEST,
	VIRTIO_MSG_LOOPBACK_ORIGIN_TRANSPORT_EVENT,
};

static enum virtio_msg_loopback_origin
virtio_msg_loopback_origin_classify(const struct virtio_msg_dispatch_ctx *dctx)
{
	if (!dctx)
		return VIRTIO_MSG_LOOPBACK_ORIGIN_BRIDGE;
	if (dctx->flags & VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE)
		return VIRTIO_MSG_LOOPBACK_ORIGIN_TRANSPORT_REQUEST;
	if (dctx->flags & VIRTIO_MSG_DISPATCH_F_NONBLOCK)
		return VIRTIO_MSG_LOOPBACK_ORIGIN_TRANSPORT_EVENT;

	return VIRTIO_MSG_LOOPBACK_ORIGIN_BRIDGE;
}

static int
virtio_msg_loopback_validate_slot_dev_num(const struct virtio_msg *msg)
{
	if (!msg)
		return -EINVAL;
	if (!virtio_msg_loopback_dev_num_valid(le16_to_cpu(msg->dev_num)))
		return -ENODEV;

	return 0;
}

static struct virtio_msg_loopback_slot *
virtio_msg_loopback_slot_lookup_routable(struct virtio_msg_loopback *loopback,
					 u16 dev_num)
{
	struct virtio_msg_loopback_slot *slot;

	slot = virtio_msg_loopback_slot_get(loopback, dev_num);
	if (!slot || !slot->transport_prepared)
		return NULL;

	return slot;
}

static struct virtio_msg_loopback_slot *
virtio_msg_loopback_bridge_slot_from_dma_dev(struct virtio_msg_loopback *loopback,
					     struct device *dma_dev)
{
	unsigned int dev_num;

	if (!loopback || !dma_dev)
		return NULL;

	for (dev_num = 0; dev_num < VIRTIO_MSG_LOOPBACK_MAX_DEVS; dev_num++) {
		struct virtio_msg_loopback_slot *slot;

		slot = &loopback->slots[dev_num];
		if (READ_ONCE(slot->dma_dev) == dma_dev)
			return slot;
	}

	return NULL;
}

static int
virtio_msg_loopback_relay_pending_retry_status
	(struct virtio_msg_loopback *loopback,
	 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_loopback_relay_pending *pending;
	unsigned long key;
	int ret = 0;

	if (!loopback || !dctx || !dctx->relay_seq)
		return -EINVAL;

	key = (unsigned long)dctx->relay_seq;

	mutex_lock(&loopback->pending_lock);
	pending = xa_load(&loopback->pending_relay_xa, key);
	if (!pending)
		ret = -ESHUTDOWN;
	else if (time_after_eq(jiffies, pending->deadline))
		ret = -ETIMEDOUT;
	mutex_unlock(&loopback->pending_lock);

	return ret;
}

static int
virtio_msg_loopback_pending_complete_timeout
	(struct virtio_msg_loopback *loopback,
	 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_loopback_relay_pending *pending;
	unsigned long key;

	if (!loopback || !dctx || !dctx->relay_seq)
		return -EINVAL;

	key = (unsigned long)dctx->relay_seq;

	mutex_lock(&loopback->pending_lock);
	pending = xa_erase(&loopback->pending_relay_xa, key);
	if (!pending) {
		mutex_unlock(&loopback->pending_lock);
		return -ENOENT;
	}

	virtio_msg_loopback_pending_get(pending);
	pending->status = -ETIMEDOUT;
	pending->timed_out = true;
	pending->completed = true;
	mutex_unlock(&loopback->pending_lock);

	complete(&pending->done);
	virtio_msg_loopback_pending_put(pending);

	return 0;
}

static int
virtio_msg_loopback_relay_to_userspace_sleepable(struct virtio_msg_loopback *loopback,
						 const struct virtio_msg *msg,
						 u16 msg_size,
						 const struct virtio_msg_dispatch_ctx *dctx)
{
	u16 token;
	u16 dev_num;
	u32 handle;
	int ret;

	if (!loopback || !msg || msg_size < sizeof(*msg) || !dctx)
		return -EINVAL;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle)
		return -ENODEV;

	dev_num = le16_to_cpu(msg->dev_num);
	token = le16_to_cpu(msg->token);

	for (;;) {
		ret = virtio_msg_bus_bridge_device_publish_rx(handle, msg);
		if (ret != -ENOSPC)
			return ret;

		ret = virtio_msg_loopback_relay_pending_retry_status(loopback, dctx);
		if (ret == -ETIMEDOUT) {
			ret = virtio_msg_bus_bridge_device_relay_drop(handle, dev_num,
								      token);
			if (ret && ret != -ENODEV && ret != -ENOTCONN)
				return ret;

			ret = virtio_msg_loopback_pending_complete_timeout
				(loopback, dctx);
			if (ret == -ENOENT)
				return -ESHUTDOWN;

			return ret;
		}
		if (ret)
			return ret;

		msleep(VIRTIO_MSG_LOOPBACK_RELAY_RETRY_DELAY_MS);
		handle = READ_ONCE(loopback->bridge.endpoint.handle);
		if (!handle)
			return -ENODEV;
	}
}

static int
virtio_msg_loopback_relay_to_userspace_nonblock(struct virtio_msg_loopback
						*loopback,
						const struct virtio_msg *msg,
						u16 msg_size)
{
	if (!loopback || !msg || msg_size < sizeof(*msg))
		return -EINVAL;

	return virtio_msg_bus_bridge_device_publish_rx_nonblock
		(&loopback->bridge.endpoint, msg);
}

static int
virtio_msg_loopback_map_id_to_index(u64 map_id, unsigned long *index)
{
	if (!map_id || !index)
		return -EINVAL;
	if (map_id > (u64)ULONG_MAX)
		return -EOVERFLOW;

	*index = (unsigned long)map_id;
	return 0;
}

static int
virtio_msg_loopback_map_offset_to_index(u64 mmap_offset, unsigned long *index)
{
	u64 shifted;

	if (!index || !PAGE_ALIGNED(mmap_offset))
		return -EINVAL;

	shifted = mmap_offset >> PAGE_SHIFT;
	if (!shifted || shifted > (u64)ULONG_MAX)
		return -EOVERFLOW;

	*index = (unsigned long)shifted;
	return 0;
}

static int
virtio_msg_loopback_map_dma_addr_to_index(u64 dma_addr, unsigned long *index)
{
	if (!index)
		return -EINVAL;
	if (dma_addr > (u64)ULONG_MAX)
		return -EOVERFLOW;

	*index = (unsigned long)dma_addr;
	return 0;
}

static struct virtio_msg_loopback_page_window *
virtio_msg_loopback_page_window_lookup_by_id_locked(struct virtio_msg_loopback *loopback,
						    u64 map_id)
{
	unsigned long index;

	lockdep_assert_held(&loopback->area_lock);

	if (virtio_msg_loopback_map_id_to_index(map_id, &index))
		return NULL;

	return xa_load(&loopback->map_area_xa_by_id, index);
}

static struct virtio_msg_loopback_page_window *
virtio_msg_loopback_page_window_lookup_by_offset_locked(struct virtio_msg_loopback *loopback,
							u64 mmap_offset)
{
	unsigned long index;

	lockdep_assert_held(&loopback->area_lock);

	if (virtio_msg_loopback_map_offset_to_index(mmap_offset, &index))
		return NULL;

	return xa_load(&loopback->map_area_xa_by_offset, index);
}

static struct virtio_msg_loopback_page_window *
virtio_msg_loopback_page_window_lookup_by_bus_addr_locked(struct virtio_msg_loopback *loopback,
							  u64 bus_addr)
{
	unsigned long index;

	lockdep_assert_held(&loopback->area_lock);

	if (virtio_msg_loopback_map_dma_addr_to_index(bus_addr, &index))
		return NULL;

	return xa_load(&loopback->map_area_xa_by_dma_addr, index);
}

static int
virtio_msg_loopback_map_alloc_mmap_offset_locked(struct virtio_msg_loopback *loopback,
						 u64 mmap_length,
						 u64 *mmap_offset)
{
	u64 next_offset;

	lockdep_assert_held(&loopback->area_lock);

	if (!mmap_offset || !mmap_length || !PAGE_ALIGNED(mmap_length))
		return -EINVAL;
	if (check_add_overflow(loopback->map_next_offset, mmap_length,
			       &next_offset))
		return -EOVERFLOW;

	*mmap_offset = loopback->map_next_offset;
	loopback->map_next_offset = next_offset;
	return 0;
}

static int
virtio_msg_loopback_page_window_track_locked(struct virtio_msg_loopback *loopback,
					     struct virtio_msg_loopback_page_window *window)
{
	unsigned long bus_index;
	unsigned long offset_index;
	int ret;

	lockdep_assert_held(&loopback->area_lock);

	ret = virtio_msg_loopback_map_dma_addr_to_index(window->bus_addr,
							&bus_index);
	if (ret)
		return ret;

	ret = virtio_msg_loopback_map_offset_to_index(window->mmap_offset,
						      &offset_index);
	if (ret)
		return ret;

	ret = xa_err(xa_store(&loopback->map_area_xa_by_dma_addr, bus_index,
			      window, GFP_KERNEL));
	if (ret)
		return ret;

	ret = xa_err(xa_store(&loopback->map_area_xa_by_offset, offset_index,
			      window, GFP_KERNEL));
	if (ret)
		xa_erase(&loopback->map_area_xa_by_dma_addr, bus_index);

	return ret;
}

static void
virtio_msg_loopback_exact_export_track_locked(struct virtio_msg_loopback *loopback,
					      struct virtio_msg_loopback_exact_export *exact)
{
	lockdep_assert_held(&loopback->exact_lock);
	list_add_tail(&exact->node, &loopback->exact_exports);
}

static void
virtio_msg_loopback_page_window_index_id_locked(struct virtio_msg_loopback *loopback,
						struct virtio_msg_loopback_page_window *window)
{
	unsigned long id_index;

	lockdep_assert_held(&loopback->area_lock);

	if (virtio_msg_loopback_map_id_to_index(window->map_id, &id_index))
		return;

	(void)xa_store(&loopback->map_area_xa_by_id, id_index, window,
		       GFP_KERNEL);
}

/* Forward declaration — defined after page_window_alloc_multi. */
static void
virtio_msg_loopback_page_window_unindex_all_bus_addrs_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window);

static void
virtio_msg_loopback_page_window_unindex_bus_addr_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window)
{
	unsigned long bus_index;

	lockdep_assert_held(&loopback->area_lock);

	if (!window)
		return;

	/* Multi-page windows register one entry per page; remove all of them. */
	if (window->npages > 1) {
		virtio_msg_loopback_page_window_unindex_all_bus_addrs_locked
			(loopback, window);
		return;
	}

	if (!virtio_msg_loopback_map_dma_addr_to_index(window->bus_addr, &bus_index))
		xa_erase(&loopback->map_area_xa_by_dma_addr, bus_index);
}

static void
virtio_msg_loopback_page_window_remove_locked(struct virtio_msg_loopback *loopback,
					      struct virtio_msg_loopback_page_window *window)
{
	unsigned long id_index;
	unsigned long offset_index;

	lockdep_assert_held(&loopback->area_lock);

	if (!window)
		return;

	if (!virtio_msg_loopback_map_id_to_index(window->map_id, &id_index))
		xa_erase(&loopback->map_area_xa_by_id, id_index);
	if (!virtio_msg_loopback_map_offset_to_index(window->mmap_offset,
						     &offset_index))
		xa_erase(&loopback->map_area_xa_by_offset, offset_index);
	virtio_msg_loopback_page_window_unindex_bus_addr_locked(loopback, window);
}

static void
virtio_msg_loopback_page_window_settle_revoke_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window, int status)
{
	lockdep_assert_held(&loopback->area_lock);

	if (!window || window->revoke_settled)
		return;

	window->revoke_status = status;
	window->revoke_settled = true;
	complete_all(&window->revoke_done);
}

static void
virtio_msg_loopback_page_window_settle_add_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window, int status)
{
	lockdep_assert_held(&loopback->area_lock);

	if (!window || window->map_add_settled)
		return;

	window->map_add_status = status;
	window->map_add_settled = true;
	complete_all(&window->map_add_done);
}

static void
virtio_msg_loopback_page_window_clear_id_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window)
{
	unsigned long id_index;

	lockdep_assert_held(&loopback->area_lock);

	if (!window || !window->map_id)
		return;

	if (!virtio_msg_loopback_map_id_to_index(window->map_id, &id_index))
		xa_erase(&loopback->map_area_xa_by_id, id_index);

	window->map_id = 0;
}

static void
virtio_msg_loopback_page_window_prepare_add_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window, bool async_activate)
{
	lockdep_assert_held(&loopback->area_lock);

	if (!window)
		return;

	reinit_completion(&window->map_add_done);
	window->map_add_status = 0;
	window->map_add_settled = false;
	window->map_add_async = async_activate;
}

static int
virtio_msg_loopback_page_window_wait_add_result
	(struct virtio_msg_loopback_page_window *window, unsigned int timeout_ms)
{
	if (!window)
		return 0;

	if (!wait_for_completion_timeout(&window->map_add_done,
					 msecs_to_jiffies(timeout_ms)))
		return -ETIMEDOUT;

	return window->map_add_status;
}

static void virtio_msg_loopback_exact_activate_workfn(struct work_struct *work);

static void
virtio_msg_loopback_exact_queue_activate(struct virtio_msg_loopback *loopback,
					 struct virtio_msg_loopback_exact_export *exact)
{
	unsigned long flags;
	bool requeue = false;
	bool take_ref = false;

	if (!loopback || !loopback->exact_wq || !exact)
		return;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	if (!exact->activate_ref_held) {
		exact->activate_ref_held = true;
		take_ref = true;
	} else if (exact->activate_running) {
		exact->activate_requeue = true;
		requeue = true;
	}
	spin_unlock_irqrestore(&loopback->exact_lock, flags);

	if (requeue) {
		(void)mod_delayed_work(loopback->exact_wq, &exact->activate_work,
				       0);
		return;
	}

	if (!take_ref)
		return;

	virtio_msg_loopback_exact_get(exact);
	if (!queue_delayed_work(loopback->exact_wq, &exact->activate_work, 0)) {
		spin_lock_irqsave(&loopback->exact_lock, flags);
		exact->activate_ref_held = false;
		exact->activate_requeue = false;
		spin_unlock_irqrestore(&loopback->exact_lock, flags);
		virtio_msg_loopback_exact_put(exact);
	}
}

static void
virtio_msg_loopback_exact_cancel_activate(struct virtio_msg_loopback *loopback,
					  struct virtio_msg_loopback_exact_export *exact)
{
	unsigned long flags;
	bool drop_ref = false;

	if (!loopback || !loopback->exact_wq || !exact)
		return;

	cancel_delayed_work_sync(&exact->activate_work);

	spin_lock_irqsave(&loopback->exact_lock, flags);
	if (exact->activate_ref_held) {
		exact->activate_ref_held = false;
		drop_ref = true;
	}
	exact->activate_running = false;
	exact->activate_requeue = false;
	spin_unlock_irqrestore(&loopback->exact_lock, flags);

	if (drop_ref)
		virtio_msg_loopback_exact_put(exact);
}

static struct virtio_msg_loopback_exact_export *
virtio_msg_loopback_exact_export_alloc_validated
	(const struct virtio_msg_bus_dma_export *provider_export,
	 gfp_t gfp)
{
	struct virtio_msg_loopback_exact_export *exact;

	/*
	 * Callers must prevalidate provider_export via
	 * virtio_msg_loopback_provider_export_validate().
	 */
	if (!provider_export || !provider_export->npages ||
	    !provider_export->pages)
		return ERR_PTR(-EINVAL);

	exact = kzalloc(sizeof(*exact), gfp);
	if (!exact)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&exact->node);
	refcount_set(&exact->refs, 1);
	WRITE_ONCE(exact->dead, false);
	exact->activate_ref_held = false;
	exact->activate_running = false;
	exact->activate_requeue = false;
	INIT_DELAYED_WORK(&exact->activate_work,
			  virtio_msg_loopback_exact_activate_workfn);
	exact->num_page_windows = provider_export->npages;
	exact->page_windows = kcalloc(exact->num_page_windows,
				      sizeof(*exact->page_windows), gfp);
	if (!exact->page_windows) {
		kfree(exact);
		return ERR_PTR(-ENOMEM);
	}

	exact->unique_windows = kcalloc(exact->num_page_windows,
					sizeof(*exact->unique_windows),
					gfp);
	if (!exact->unique_windows) {
		kfree(exact->page_windows);
		kfree(exact);
		return ERR_PTR(-ENOMEM);
	}

	return exact;
}

static int
virtio_msg_loopback_exact_export_append_unique_window
	(struct virtio_msg_loopback_exact_export *exact,
	 struct virtio_msg_loopback_page_window *window)
{
	unsigned int i;

	if (!exact || !window)
		return -EINVAL;

	for (i = 0; i < exact->num_unique_windows; i++) {
		if (exact->unique_windows[i] == window)
			return 0;
	}

	if (exact->num_unique_windows >= exact->num_page_windows)
		return -EOVERFLOW;

	exact->unique_windows[exact->num_unique_windows++] = window;

	return 0;
}

static int
virtio_msg_loopback_page_window_init_dma_mmap
	(struct virtio_msg_loopback_page_window *window,
	 const struct virtio_msg_bus_dma_export *provider_export,
	 unsigned int page_index)
{
	u64 dma_base;
	u64 page_offset;

	if (!window || !provider_export || !provider_export->coherent)
		return 0;
	if (!provider_export->dma_dev || !provider_export->cpu_addr)
		return -EINVAL;
	if (!dma_can_mmap(provider_export->dma_dev))
		return -EOPNOTSUPP;
	if (check_sub_overflow((u64)provider_export->dma_addr,
			       (u64)provider_export->page_offset, &dma_base))
		return -EOVERFLOW;
	if (check_mul_overflow((u64)page_index, (u64)PAGE_SIZE, &page_offset))
		return -EOVERFLOW;

	window->dma_dev = provider_export->dma_dev;
	window->dma_cpu_addr = (char *)provider_export->cpu_addr -
			       provider_export->page_offset + page_offset;
	window->dma_addr = dma_base + page_offset;
	window->dma_attrs = provider_export->attrs;
	window->dma_mmap = true;

	return 0;
}

static struct virtio_msg_loopback_page_window *
virtio_msg_loopback_page_window_alloc
	(struct page *page, u64 bus_addr,
	 const struct virtio_msg_bus_dma_export *provider_export,
	 unsigned int page_index)
{
	struct virtio_msg_loopback_page_window *window;
	int ret;

	if (!page || !PAGE_ALIGNED(bus_addr))
		return ERR_PTR(-EINVAL);

	window = kzalloc(sizeof(*window), GFP_KERNEL);
	if (!window)
		return ERR_PTR(-ENOMEM);

	init_completion(&window->revoke_done);
	init_completion(&window->map_add_done);
	refcount_set(&window->refs, 1);
	window->bus_addr = bus_addr;
	window->length = PAGE_SIZE;
	window->npages = 1;
	window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY;
	ret = virtio_msg_loopback_page_window_init_dma_mmap(window,
							    provider_export,
							    page_index);
	if (ret) {
		kfree(window);
		return ERR_PTR(ret);
	}
	if (window->dma_mmap)
		return window;
	get_page(page);
	window->page = page;
	window->kva = vmap(&window->page, 1, VM_MAP | VM_USERMAP,
			   PAGE_KERNEL);
	if (!window->kva) {
		virtio_msg_loopback_page_window_put(window);
		return ERR_PTR(-ENOMEM);
	}

	return window;
}

/*
 * Allocate a multi-page window for a physically-contiguous region (e.g. a
 * DMA pool chunk).  pages[] are borrowed from the pool chunk's dma_alloc_attrs()
 * allocation and must NOT be individually get_page()/put_page()'d here; the
 * pool chunk's dma_free_attrs() call is the authoritative lifetime owner.
 */
static struct virtio_msg_loopback_page_window *
virtio_msg_loopback_page_window_alloc_multi(struct page **pages,
					    unsigned int npages, u64 bus_addr,
					    const struct virtio_msg_bus_dma_export
						    *provider_export)
{
	struct virtio_msg_loopback_page_window *window;
	int ret;

	if (!pages || !npages || !PAGE_ALIGNED(bus_addr))
		return ERR_PTR(-EINVAL);

	window = kzalloc(sizeof(*window), GFP_KERNEL);
	if (!window)
		return ERR_PTR(-ENOMEM);

	init_completion(&window->revoke_done);
	init_completion(&window->map_add_done);
	refcount_set(&window->refs, 1);
	window->bus_addr = bus_addr;
	window->length = (u64)npages * PAGE_SIZE;
	window->npages = npages;
	window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY;
	ret = virtio_msg_loopback_page_window_init_dma_mmap(window,
							    provider_export, 0);
	if (ret) {
		kfree(window);
		return ERR_PTR(ret);
	}
	if (window->dma_mmap)
		return window;
	/* window->page is NULL: no per-page lifecycle management needed. */
	window->kva = vmap(pages, npages, VM_MAP | VM_USERMAP, PAGE_KERNEL);
	if (!window->kva) {
		kfree(window);
		return ERR_PTR(-ENOMEM);
	}

	return window;
}

/*
 * Register a multi-page window in both XArrays: one mmap_offset entry and one
 * bus_addr entry per page in the window's range.  On failure, all entries
 * already stored are rolled back.
 */
static int
virtio_msg_loopback_page_window_track_multi_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window)
{
	unsigned long offset_index;
	unsigned int i;
	int ret;

	lockdep_assert_held(&loopback->area_lock);

	ret = virtio_msg_loopback_map_offset_to_index(window->mmap_offset,
						      &offset_index);
	if (ret)
		return ret;

	ret = xa_err(xa_store(&loopback->map_area_xa_by_offset, offset_index,
			      window, GFP_KERNEL));
	if (ret)
		return ret;

	for (i = 0; i < window->npages; i++) {
		unsigned long bus_index;
		u64 page_bus_addr;

		if (check_add_overflow(window->bus_addr, (u64)i * PAGE_SIZE,
				       &page_bus_addr)) {
			ret = -EOVERFLOW;
			goto err_unwind;
		}
		if (virtio_msg_loopback_map_dma_addr_to_index(page_bus_addr,
							      &bus_index)) {
			ret = -EOVERFLOW;
			goto err_unwind;
		}
		ret = xa_err(xa_store(&loopback->map_area_xa_by_dma_addr,
				      bus_index, window, GFP_KERNEL));
		if (ret)
			goto err_unwind;
	}

	return 0;

err_unwind:
	while (i--) {
		unsigned long bus_index;
		u64 page_bus_addr;

		if (!check_add_overflow(window->bus_addr, (u64)i * PAGE_SIZE,
					&page_bus_addr) &&
		    !virtio_msg_loopback_map_dma_addr_to_index(page_bus_addr,
							       &bus_index))
			xa_erase(&loopback->map_area_xa_by_dma_addr, bus_index);
	}
	xa_erase(&loopback->map_area_xa_by_offset, offset_index);
	return ret;
}

/* Remove all per-page bus_addr entries for a multi-page window. */
static void
virtio_msg_loopback_page_window_unindex_all_bus_addrs_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window)
{
	unsigned int i;

	lockdep_assert_held(&loopback->area_lock);

	if (!window)
		return;

	for (i = 0; i < window->npages; i++) {
		unsigned long bus_index;
		u64 page_bus_addr;

		if (check_add_overflow(window->bus_addr, (u64)i * PAGE_SIZE,
				       &page_bus_addr))
			continue;
		if (!virtio_msg_loopback_map_dma_addr_to_index(page_bus_addr,
							       &bus_index))
			xa_erase(&loopback->map_area_xa_by_dma_addr, bus_index);
	}
}

static struct virtio_msg_loopback_exact_export *
virtio_msg_loopback_exact_export_untrack_locked(struct virtio_msg_loopback *loopback,
						struct virtio_msg_loopback_exact_export *exact)
{
	lockdep_assert_held(&loopback->exact_lock);

	if (!exact || list_empty(&exact->node))
		return NULL;

	list_del_init(&exact->node);

	return exact;
}

static void
virtio_msg_loopback_bridge_exact_unpublish(struct virtio_msg_loopback *loopback,
					   struct virtio_msg_loopback_exact_export *exact)
{
	struct virtio_msg_loopback_slot *slot;
	unsigned long flags;
	bool release_gate = false;

	if (!loopback || !exact)
		return;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	(void)virtio_msg_loopback_exact_export_untrack_locked(loopback, exact);
	slot = exact->slot;
	if (exact->gate_held) {
		exact->gate_held = false;
		release_gate = true;
	}
	spin_unlock_irqrestore(&loopback->exact_lock, flags);
	vm_trace(NULL,
		 "unpublish export: dma=%#llx len=%#llx async=%u gate_release=%u\n",
			(unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length,
			exact->async_activate, release_gate);

	if (release_gate && slot &&
	    atomic_dec_and_test(&slot->pending_async_exports))
		(void)mod_delayed_work(system_wq, &slot->deferred_event_work, 0);
}

static bool
virtio_msg_loopback_bridge_exact_claim_windows(struct virtio_msg_loopback *loopback,
					       struct virtio_msg_loopback_exact_export *exact)
{
	unsigned long flags;
	bool do_cleanup = false;

	if (!loopback || !exact)
		return false;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	if (exact->windows_ready) {
		exact->windows_ready = false;
		do_cleanup = true;
	}
	spin_unlock_irqrestore(&loopback->exact_lock, flags);

	return do_cleanup;
}

static bool
virtio_msg_loopback_page_window_is_tracked_locked(struct virtio_msg_loopback *loopback,
						  struct virtio_msg_loopback_page_window *window)
{
	lockdep_assert_held(&loopback->area_lock);

	return virtio_msg_loopback_page_window_lookup_by_bus_addr_locked(loopback,
									 window->bus_addr) ==
	       window;
}

/*
 * Fast path for physically-contiguous all-new pages: create one multi-page
 * window covering the entire export and issue a single MAP_EVENT_ADD instead
 * of one per page.  All exact->page_windows[] slots point to the same window;
 * exact->unique_windows[] records that logical window once, while the refcount
 * and owner_count are still bumped once per page slot so release accounting
 * remains symmetric.
 */
static int
virtio_msg_loopback_attach_multi_page_window_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_exact_export *exact,
	 const struct virtio_msg_bus_dma_export *provider_export)
{
	struct virtio_msg_loopback_page_window *window;
	u64 bus_addr = provider_export->dma_addr - provider_export->page_offset;
	u64 mmap_length = (u64)provider_export->npages * PAGE_SIZE;
	u64 mmap_offset;
	unsigned int i;
	int ret;

	lockdep_assert_held(&loopback->area_lock);

	window = virtio_msg_loopback_page_window_alloc_multi
		(provider_export->pages, provider_export->npages, bus_addr,
		 provider_export);
	if (IS_ERR(window))
		return PTR_ERR(window);

	ret = virtio_msg_loopback_map_alloc_mmap_offset_locked(loopback,
							       mmap_length,
							       &mmap_offset);
	if (ret) {
		virtio_msg_loopback_page_window_put(window);
		return ret;
	}
	window->mmap_offset = mmap_offset;

	ret = virtio_msg_loopback_page_window_track_multi_locked(loopback, window);
	if (ret) {
		virtio_msg_loopback_page_window_put(window);
		return ret;
	}

	ret = virtio_msg_loopback_exact_export_append_unique_window(exact,
								    window);
	if (ret) {
		virtio_msg_loopback_page_window_remove_locked(loopback, window);
		virtio_msg_loopback_page_window_put(window);
		return ret;
	}

	for (i = 0; i < exact->num_page_windows; i++) {
		virtio_msg_loopback_page_window_get(window);
		exact->page_windows[i] = window;
		window->owner_count++;
	}

	return 0;
}

static int
virtio_msg_loopback_exact_export_attach_page_windows_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_exact_export *exact,
	 const struct virtio_msg_bus_dma_export *provider_export)
{
	u64 bus_addr = provider_export->dma_addr - provider_export->page_offset;
	unsigned int i;

	lockdep_assert_held(&loopback->area_lock);

	/*
	 * Fast path: all pages are physically contiguous and none are already
	 * tracked.  Create one multi-page window and issue a single
	 * MAP_EVENT_ADD for the whole range.  This is the common case for DMA
	 * pool chunk initial registration (dma_alloc_attrs gives contiguous
	 * pages by construction).
	 */
	if (provider_export->npages > 1) {
		bool contiguous = true, all_new = true;

		for (i = 0; i < provider_export->npages; i++) {
			u64 page_bus_addr;

			if (!provider_export->pages[i]) {
				contiguous = false;
				all_new = false;
				break;
			}
			if (i > 0 &&
			    page_to_pfn(provider_export->pages[i]) !=
			    page_to_pfn(provider_export->pages[i - 1]) + 1)
				contiguous = false;
			if (all_new &&
			    (!check_add_overflow(bus_addr, (u64)i * PAGE_SIZE,
						&page_bus_addr) &&
			     virtio_msg_loopback_page_window_lookup_by_bus_addr_locked
				     (loopback, page_bus_addr)))
				all_new = false;
			if (!contiguous && !all_new)
				break;
		}

		if (contiguous && all_new)
			return virtio_msg_loopback_attach_multi_page_window_locked
				(loopback, exact, provider_export);
	}

	for (i = 0; i < exact->num_page_windows; i++) {
		struct virtio_msg_loopback_page_window *window;
		u64 *mmap_offset;
		u64 window_bus_addr;
		int ret;

		if (!provider_export->pages[i])
			return -EINVAL;
		if (check_add_overflow(bus_addr, (u64)i * PAGE_SIZE,
				       &window_bus_addr))
			return -EOVERFLOW;

		window = virtio_msg_loopback_page_window_lookup_by_bus_addr_locked
			(loopback, window_bus_addr);
		if (window) {
			if (window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_ACTIVE)
				return -EBUSY;
			/*
			 * For multi-page windows the bus_addr lookup already
			 * proves correctness; only check identity for single-page
			 * windows where window->page carries the expected value.
			 */
			if (window->npages == 1 && window->page &&
			    window->page != provider_export->pages[i])
				return -EINVAL;
		} else {
			window =
				virtio_msg_loopback_page_window_alloc
					(provider_export->pages[i],
					 window_bus_addr,
					 provider_export, i);
			if (IS_ERR(window))
				return PTR_ERR(window);

			mmap_offset = &window->mmap_offset;
			ret = virtio_msg_loopback_map_alloc_mmap_offset_locked(loopback,
									       PAGE_SIZE,
									       mmap_offset);
			if (ret) {
				virtio_msg_loopback_page_window_put(window);
				return ret;
			}

			ret = virtio_msg_loopback_page_window_track_locked(loopback, window);
			if (ret) {
				virtio_msg_loopback_page_window_put(window);
				return ret;
			}
		}

		ret = virtio_msg_loopback_exact_export_append_unique_window
			(exact, window);
		if (ret)
			return ret;

		virtio_msg_loopback_page_window_get(window);
		exact->page_windows[i] = window;
		window->owner_count++;
	}

	return 0;
}

static void
virtio_msg_loopback_exact_export_release_page_windows_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_exact_export *exact)
{
	unsigned int i;

	lockdep_assert_held(&loopback->area_lock);

	for (i = 0; i < exact->num_page_windows; i++) {
		struct virtio_msg_loopback_page_window *window;

		window = exact->page_windows[i];
		if (!window || !window->owner_count)
			continue;

		window->owner_count--;
		if (!window->owner_count)
			vm_trace(NULL,
				 "owner drop: bus=%#llx len=%#llx map_id=%#llx state=%u\n",
					(unsigned long long)window->bus_addr,
					(unsigned long long)window->length,
					(unsigned long long)window->map_id,
					window->state);
	}
}

static bool
virtio_msg_loopback_page_window_wait_revoke_timeout
	(struct virtio_msg_loopback_page_window *window, unsigned int timeout_ms)
{
	if (!window)
		return true;

	return wait_for_completion_timeout(&window->revoke_done,
					   msecs_to_jiffies(timeout_ms));
}

static int
virtio_msg_loopback_page_window_wait_revoke_result
	(struct virtio_msg_loopback_page_window *window, unsigned int timeout_ms)
{
	if (!virtio_msg_loopback_page_window_wait_revoke_timeout(window,
								 timeout_ms))
		return -EINPROGRESS;

	return window->revoke_status;
}

static void
virtio_msg_loopback_bridge_exact_cleanup_wait_revoke
	(struct virtio_msg_loopback_page_window *window, bool wait, int *ret_sync,
	 u64 map_id, u64 bus_addr, u64 length, const char *label)
{
	int ret;

	if (!wait || !ret_sync || *ret_sync)
		return;

	ret = virtio_msg_loopback_page_window_wait_revoke_result
		(window, VIRTIO_MSG_LOOPBACK_RELAY_TIMEOUT_MS);
	*ret_sync = ret;
	if (!*ret_sync)
		vm_trace(NULL,
			 "%s: map_id=%#llx bus=%#llx len=%#llx status=%d\n",
				label,
				(unsigned long long)map_id,
				(unsigned long long)bus_addr,
				(unsigned long long)length, ret);
}

static void
virtio_msg_loopback_page_window_remove_terminal_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window, int status)
{
	lockdep_assert_held(&loopback->area_lock);

	virtio_msg_loopback_page_window_settle_revoke_locked(loopback, window,
							     status);
	virtio_msg_loopback_page_window_remove_locked(loopback, window);
}

static int
virtio_msg_loopback_bridge_exact_cleanup
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_exact_export *exact, bool sync)
{
	struct virtio_msg_loopback_page_window *window;
	unsigned int i;
	u32 handle;
	int ret_sync = 0;

	if (!loopback || !exact)
		return -EINVAL;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	vm_trace(NULL,
		 "cleanup export begin: dma=%#llx len=%#llx sync=%u windows=%u\n",
			(unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length,
			sync, exact->num_unique_windows);

	mutex_lock(&loopback->area_lock);
	virtio_msg_loopback_exact_export_release_page_windows_locked(loopback,
								     exact);
	mutex_unlock(&loopback->area_lock);

	for (i = 0; i < exact->num_unique_windows; i++) {
		u64 map_id = 0;
		u64 bus_addr = 0;
		u64 length = 0;
		enum virtio_msg_loopback_map_state old_state = 0;
		int ret;
		bool drop_window = false;
		bool wait_revoke = false;

		window = exact->unique_windows[i];
		if (!window)
			continue;

		mutex_lock(&loopback->area_lock);
		if (window->owner_count) {
			mutex_unlock(&loopback->area_lock);
			continue;
		}
		if (window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING ||
		    window->del_req_queued) {
			bus_addr = window->bus_addr;
			length = window->length;
			map_id = window->map_id;
			wait_revoke = sync;
			mutex_unlock(&loopback->area_lock);
			vm_trace(NULL,
				 "cleanup wait revoke: map_id=%#llx bus=%#llx len=%#llx sync=%u\n",
					(unsigned long long)map_id,
					(unsigned long long)bus_addr,
					(unsigned long long)length, sync);
			virtio_msg_loopback_bridge_exact_cleanup_wait_revoke
				(window, wait_revoke, &ret_sync, map_id,
				 bus_addr, length, "cleanup revoke done");
			continue;
		}
		if (!virtio_msg_loopback_page_window_is_tracked_locked(loopback,
								       window)) {
			mutex_unlock(&loopback->area_lock);
			continue;
		}

		if (!handle || !window->map_id) {
			bus_addr = window->bus_addr;
			length = window->length;
			map_id = window->map_id;
			virtio_msg_loopback_page_window_remove_terminal_locked
				(loopback, window, 0);
			drop_window = true;
			wait_revoke = sync;
			mutex_unlock(&loopback->area_lock);
			vm_trace(NULL,
				 "cleanup local remove: map_id=%#llx bus=%#llx len=%#llx handle=%u\n",
					(unsigned long long)map_id,
					(unsigned long long)bus_addr,
					(unsigned long long)length, handle);
			virtio_msg_loopback_bridge_exact_cleanup_wait_revoke
				(window, wait_revoke, &ret_sync, map_id,
				 bus_addr, length, "cleanup local revoke done");
			if (drop_window)
				virtio_msg_loopback_page_window_put(window);
			continue;
		}

		map_id = window->map_id;
		bus_addr = window->bus_addr;
		length = window->length;
		old_state = window->state;
		if (window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY) {
			window->del_req_queued = true;
			vm_trace(NULL,
				 "revoke trigger queued: map_id=%#llx bus=%#llx len=%#llx state=%u\n",
					(unsigned long long)map_id,
					(unsigned long long)bus_addr,
					(unsigned long long)length, old_state);
		} else {
			window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING;
			virtio_msg_loopback_page_window_unindex_bus_addr_locked
				(loopback, window);
			vm_trace(NULL,
				 "revoke trigger: map_id=%#llx bus=%#llx len=%#llx old=%u new=%u\n",
					(unsigned long long)map_id,
					(unsigned long long)bus_addr,
					(unsigned long long)length, old_state,
					window->state);
		}
		wait_revoke = sync;
		mutex_unlock(&loopback->area_lock);

		vm_trace(NULL,
			 "tx MAP_DEL_REQ: map_id=%#llx bus=%#llx len=%#llx\n",
			       (unsigned long long)map_id,
			       (unsigned long long)bus_addr,
			       (unsigned long long)length);
		ret = virtio_msg_bus_bridge_device_map_event_del_req(handle, map_id);
		if (ret)
			vm_trace(NULL,
				 "tx MAP_DEL_REQ failed: map_id=%#llx bus=%#llx len=%#llx ret=%d\n",
				       (unsigned long long)map_id,
				       (unsigned long long)bus_addr,
				       (unsigned long long)length, ret);

		mutex_lock(&loopback->area_lock);
		if (!window->owner_count &&
		    (window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING ||
		     window->del_req_queued)) {
			if (ret) {
				window->del_req_queued = false;
				virtio_msg_loopback_page_window_remove_terminal_locked
					(loopback, window, ret);
				vm_trace(NULL,
					 "cleanup remove after del_req failure: map_id=%#llx bus=%#llx len=%#llx ret=%d\n",
						(unsigned long long)map_id,
						(unsigned long long)bus_addr,
						(unsigned long long)length, ret);
				drop_window = true;
				if (!ret_sync)
					ret_sync = ret;
			}
		}
		mutex_unlock(&loopback->area_lock);

		if (!drop_window)
			virtio_msg_loopback_bridge_exact_cleanup_wait_revoke
				(window, wait_revoke, &ret_sync, map_id,
				 bus_addr, length, "cleanup revoke done");
		if (drop_window)
			virtio_msg_loopback_page_window_put(window);
	}

	vm_trace(NULL,
		 "cleanup export done: dma=%#llx len=%#llx ret=%d\n",
			(unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length, ret_sync);
	return ret_sync;
}

static void
virtio_msg_loopback_exact_release_gate(struct virtio_msg_loopback *loopback,
				       struct virtio_msg_loopback_exact_export *exact)
{
	struct virtio_msg_loopback_slot *slot = NULL;
	unsigned long flags;
	bool release = false;

	if (!loopback || !exact || !exact->async_activate)
		return;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	if (exact->gate_held) {
		exact->gate_held = false;
		slot = exact->slot;
		release = true;
	}
	spin_unlock_irqrestore(&loopback->exact_lock, flags);
	if (release)
		vm_trace(NULL,
			 "release gate: dma=%#llx len=%#llx async=%u\n",
				(unsigned long long)exact->provider_export.dma_addr,
				(unsigned long long)exact->provider_export.length,
				exact->async_activate);

	if (release && slot &&
	    atomic_dec_and_test(&slot->pending_async_exports))
		(void)mod_delayed_work(system_wq, &slot->deferred_event_work, 0);
}

static int
virtio_msg_loopback_exact_activate_sleepable
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_exact_export *exact, bool *retryable)
{
	const struct virtio_msg_bus_dma_export *provider_export;
	bool all_active = true;
	u32 handle;
	int ret = 0;
	unsigned int i;

	if (retryable)
		*retryable = false;
	if (!loopback || !exact)
		return -EINVAL;
	if (READ_ONCE(exact->dead))
		return 0;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle)
		return -ENODEV;

	provider_export = &exact->provider_export;

	mutex_lock(&loopback->area_lock);
	if (!exact->windows_ready) {
		ret = virtio_msg_loopback_exact_export_attach_page_windows_locked
			(loopback, exact, provider_export);
		if (!ret)
			exact->windows_ready = true;
	}
	mutex_unlock(&loopback->area_lock);
	if (ret)
		goto out;

	for (i = 0; i < exact->num_unique_windows; i++) {
		struct virtio_msg_loopback_page_window *window;
		u64 map_id = 0;
		bool keep_window = false;
		bool submit = false;

		window = exact->unique_windows[i];
		if (!window)
			continue;

		mutex_lock(&loopback->area_lock);
		if (window->owner_count &&
		    window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY &&
		    !window->map_id &&
		    virtio_msg_loopback_page_window_is_tracked_locked(loopback,
								      window)) {
			virtio_msg_loopback_page_window_prepare_add_locked
				(loopback, window, exact->async_activate);
			submit = true;
		}
		mutex_unlock(&loopback->area_lock);
		if (!submit)
			continue;

		vm_trace(NULL,
			 "tx MAP_ADD: idx=%u bus=%#llx len=%#llx mmap_off=%#llx\n",
			       i, (unsigned long long)window->bus_addr,
			       (unsigned long long)window->length,
			       (unsigned long long)window->mmap_offset);

		ret = virtio_msg_bus_bridge_device_map_event_add(handle,
								 window->bus_addr,
								 window->length,
								 window->mmap_offset,
								 window->length,
								 0, &map_id);
		if (ret) {
			vm_trace(NULL,
				 "tx MAP_ADD failed: idx=%u bus=%#llx len=%#llx ret=%d\n",
				       i, (unsigned long long)window->bus_addr,
				       (unsigned long long)window->length, ret);
			goto out;
		}

		mutex_lock(&loopback->area_lock);
		if (window->owner_count &&
		    window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY &&
		    !window->map_id &&
		    virtio_msg_loopback_page_window_is_tracked_locked(loopback,
								      window)) {
			window->map_id = map_id;
			virtio_msg_loopback_page_window_index_id_locked(loopback, window);
			keep_window = true;
		}
		mutex_unlock(&loopback->area_lock);
		if (keep_window) {
			vm_trace(NULL,
				 "tx MAP_ADD queued: map_id=%#llx bus=%#llx len=%#llx\n",
				       (unsigned long long)map_id,
				       (unsigned long long)window->bus_addr,
				       (unsigned long long)window->length);
		} else {
			vm_trace(NULL,
				 "tx MAP_DEL_REQ rollback: map_id=%#llx bus=%#llx len=%#llx\n",
				       (unsigned long long)map_id,
				       (unsigned long long)window->bus_addr,
				       (unsigned long long)window->length);
			(void)virtio_msg_bus_bridge_device_map_event_del_req
				(handle, map_id);
		}
	}

	for (i = 0; i < exact->num_unique_windows; i++) {
		struct virtio_msg_loopback_page_window *window;
		bool wait_add = false;
		int add_status = 0;

		window = exact->unique_windows[i];
		if (!window)
			continue;

		mutex_lock(&loopback->area_lock);
		if (!window->owner_count ||
		    !virtio_msg_loopback_page_window_is_tracked_locked(loopback,
								      window)) {
			mutex_unlock(&loopback->area_lock);
			continue;
		}
		if (window->map_add_settled)
			add_status = window->map_add_status;
		else if (window->map_id)
			wait_add = true;
		mutex_unlock(&loopback->area_lock);

		if (wait_add)
			add_status = virtio_msg_loopback_page_window_wait_add_result
				(window, VIRTIO_MSG_LOOPBACK_RELAY_TIMEOUT_MS);

		if (add_status) {
			ret = add_status;
			goto out;
		}

		mutex_lock(&loopback->area_lock);
		if (window->owner_count &&
		    virtio_msg_loopback_page_window_is_tracked_locked(loopback,
								      window) &&
		    window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_ACTIVE)
			all_active = false;
		mutex_unlock(&loopback->area_lock);
	}

	if (all_active)
		virtio_msg_loopback_exact_release_gate(loopback, exact);

out:
	if (ret && exact->async_activate && retryable)
		*retryable = true;

	return ret;
}

static void virtio_msg_loopback_exact_activate_workfn(struct work_struct *work)
{
	struct virtio_msg_loopback_exact_export *exact;
	struct virtio_msg_loopback_slot *slot;
	struct virtio_msg_loopback *loopback;
	unsigned long flags;
	bool do_cleanup = false;
	bool retryable = false;
	int ret = 0;

	exact = container_of(to_delayed_work(work),
			     struct virtio_msg_loopback_exact_export,
			     activate_work);

	slot = READ_ONCE(exact->slot);
	loopback = slot ? slot->loopback : NULL;
	if (loopback) {
		spin_lock_irqsave(&loopback->exact_lock, flags);
		exact->activate_running = true;
		spin_unlock_irqrestore(&loopback->exact_lock, flags);
	}
	if (loopback && !READ_ONCE(exact->dead))
		ret = virtio_msg_loopback_exact_activate_sleepable(loopback, exact,
								   &retryable);
	if (loopback && retryable && !READ_ONCE(exact->dead)) {
		slot = READ_ONCE(exact->slot);
		if (slot)
			vm_warn_rl(&slot->vmdev.vdev.dev,
				   "async map activation retry: dev=%u ret=%d\n",
					     slot->dev_num, ret);

		spin_lock_irqsave(&loopback->exact_lock, flags);
		exact->activate_requeue = true;
		spin_unlock_irqrestore(&loopback->exact_lock, flags);
		(void)mod_delayed_work(loopback->exact_wq, &exact->activate_work,
				       msecs_to_jiffies
					(VIRTIO_MSG_LOOPBACK_RELAY_RETRY_DELAY_MS));
	}
	if (loopback && READ_ONCE(exact->dead))
		do_cleanup = virtio_msg_loopback_bridge_exact_claim_windows
			(loopback, exact);

	if (do_cleanup)
		(void)virtio_msg_loopback_bridge_exact_cleanup(loopback, exact,
								false);

	if (loopback) {
		bool keep_ref;

		spin_lock_irqsave(&loopback->exact_lock, flags);
		exact->activate_running = false;
		keep_ref = exact->activate_requeue;
		exact->activate_requeue = false;
		if (!keep_ref)
			exact->activate_ref_held = false;
		spin_unlock_irqrestore(&loopback->exact_lock, flags);
		if (keep_ref)
			return;
	}
	virtio_msg_loopback_exact_put(exact);
}

static void
virtio_msg_loopback_map_cleanup_all(struct virtio_msg_loopback *loopback)
{
	struct virtio_msg_loopback_exact_export *exact;
	struct virtio_msg_loopback_page_window *window;
	unsigned long flags;
	unsigned long index;
	u32 handle;
	u64 map_id;

	if (!loopback)
		return;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);

	for (;;) {
		bool do_cleanup;

		spin_lock_irqsave(&loopback->exact_lock, flags);
		if (list_empty(&loopback->exact_exports)) {
			spin_unlock_irqrestore(&loopback->exact_lock, flags);
			break;
		}

		exact = list_first_entry(&loopback->exact_exports,
					 struct virtio_msg_loopback_exact_export,
					 node);
		WRITE_ONCE(exact->dead, true);
		spin_unlock_irqrestore(&loopback->exact_lock, flags);

		virtio_msg_loopback_bridge_exact_unpublish(loopback, exact);
		virtio_msg_loopback_exact_cancel_activate(loopback, exact);
		do_cleanup = virtio_msg_loopback_bridge_exact_claim_windows
			(loopback, exact);

		if (do_cleanup)
			(void)virtio_msg_loopback_bridge_exact_cleanup(loopback,
									exact,
									false);

		virtio_msg_loopback_exact_put(exact);
	}

	for (;;) {
		index = 0;
		mutex_lock(&loopback->area_lock);
		window = xa_find(&loopback->map_area_xa_by_offset, &index,
				 ULONG_MAX, XA_PRESENT);
		if (!window) {
			mutex_unlock(&loopback->area_lock);
			break;
		}

		virtio_msg_loopback_page_window_get(window);
		map_id = 0;
		if (handle && window->map_id &&
		    window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING)
			map_id = window->map_id;
		virtio_msg_loopback_page_window_remove_terminal_locked
			(loopback, window,
			 window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING ?
			 -ENODEV : 0);
		mutex_unlock(&loopback->area_lock);

		if (handle && map_id)
			(void)virtio_msg_bus_bridge_device_map_event_del_req
				(handle, map_id);
		virtio_msg_loopback_page_window_put(window);
	}
}

static int
virtio_msg_loopback_provider_export_validate
	(const struct virtio_msg_bus_dma_export *provider_export)
{
	size_t mapped_span;

	if (!provider_export)
		return -EINVAL;
	if (!provider_export->length || !provider_export->mmap_length ||
	    !PAGE_ALIGNED(provider_export->mmap_length) ||
	    !provider_export->npages || !provider_export->pages)
		return -EINVAL;
	if (check_add_overflow(provider_export->page_offset,
			       provider_export->length, &mapped_span))
		return -EOVERFLOW;
	if (PAGE_ALIGN(mapped_span) != provider_export->mmap_length)
		return -EINVAL;
	if (provider_export->npages != provider_export->mmap_length >> PAGE_SHIFT)
		return -EINVAL;

	return 0;
}

static int
virtio_msg_loopback_bridge_dma_provider_add(struct virtio_msg_loopback *loopback,
					    struct virtio_msg_bus_dma_export *provider_export,
					     void **provider_handle, gfp_t gfp)
{
	struct virtio_msg_loopback_exact_export *exact;
	struct virtio_msg_loopback_slot *slot;
	struct virtio_msg_bus_dma_export local_export;
	u32 handle;
	unsigned long flags;
	int ret;
	unsigned int i;
	bool tracked = false;

	if (!loopback || !provider_export || !provider_handle)
		return -EINVAL;

	ret = virtio_msg_loopback_provider_export_validate(provider_export);
	if (ret)
		return ret;
	local_export = *provider_export;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle)
		return -ENODEV;

	slot = virtio_msg_loopback_bridge_slot_from_dma_dev(loopback,
							    provider_export->dma_dev);
	if (!slot)
		return -ENODEV;

	exact = virtio_msg_loopback_exact_export_alloc_validated(&local_export,
								 gfp);
	if (IS_ERR(exact))
		return PTR_ERR(exact);

	exact->provider_export = local_export;
	exact->slot = slot;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	virtio_msg_loopback_exact_export_track_locked(loopback, exact);
	spin_unlock_irqrestore(&loopback->exact_lock, flags);
	tracked = true;
	vm_trace(NULL,
		 "export create: dma=%#llx len=%#llx npages=%u async=0\n",
			(unsigned long long)local_export.dma_addr,
			(unsigned long long)local_export.length,
			local_export.npages);

	ret = virtio_msg_loopback_exact_activate_sleepable(loopback, exact, NULL);
	if (ret) {
		vm_trace(NULL,
			 "export create failed: dma=%#llx len=%#llx ret=%d\n",
				(unsigned long long)local_export.dma_addr,
				(unsigned long long)local_export.length, ret);
		goto rollback;
	}

	*provider_export = local_export;
	*provider_handle = exact;

	return 0;

rollback:
	vm_trace(NULL,
		 "export rollback: dma=%#llx len=%#llx ret=%d\n",
			(unsigned long long)local_export.dma_addr,
			(unsigned long long)local_export.length, ret);
	if (tracked) {
		spin_lock_irqsave(&loopback->exact_lock, flags);
		(void)virtio_msg_loopback_exact_export_untrack_locked(loopback,
								      exact);
		spin_unlock_irqrestore(&loopback->exact_lock, flags);
	}

	mutex_lock(&loopback->area_lock);
	virtio_msg_loopback_exact_export_release_page_windows_locked(loopback,
								     exact);
	mutex_unlock(&loopback->area_lock);

	for (i = 0; i < exact->num_unique_windows; i++) {
		struct virtio_msg_loopback_page_window *window;
		u64 map_id = 0;
		u32 state = 0;
		bool remove = false;

		window = exact->unique_windows[i];
		if (!window)
			continue;

		mutex_lock(&loopback->area_lock);
		if (!window->owner_count &&
		    virtio_msg_loopback_page_window_is_tracked_locked(loopback, window)) {
			map_id = window->map_id;
			state = window->state;
			virtio_msg_loopback_page_window_remove_terminal_locked
				(loopback, window, ret);
			remove = true;
		}
		mutex_unlock(&loopback->area_lock);
		if (!remove)
			continue;

		if (handle && map_id &&
		    state != VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING)
			(void)virtio_msg_bus_bridge_device_map_event_del_req
				(handle, map_id);
		virtio_msg_loopback_page_window_put(window);
	}

	virtio_msg_loopback_exact_put(exact);

	return ret;
}

static void
virtio_msg_loopback_bridge_dma_provider_del(struct virtio_msg_loopback *loopback,
					    struct virtio_msg_loopback_exact_export *exact)
{
	if (!loopback || !exact)
		return;

	vm_trace(NULL,
		 "export revoke trigger: dma=%#llx len=%#llx sync=0\n",
			(unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length);
	WRITE_ONCE(exact->dead, true);
	virtio_msg_loopback_bridge_exact_unpublish(loopback, exact);
	virtio_msg_loopback_exact_queue_activate(loopback, exact);
}

static int
virtio_msg_loopback_bridge_dma_provider_del_sync(struct virtio_msg_loopback *loopback,
						 struct virtio_msg_loopback_exact_export *exact)
{
	int ret = 0;

	if (!loopback || !exact)
		return -EINVAL;

	vm_trace(NULL,
		 "export revoke trigger: dma=%#llx len=%#llx sync=1\n",
			(unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length);
	WRITE_ONCE(exact->dead, true);
	virtio_msg_loopback_bridge_exact_unpublish(loopback, exact);
	virtio_msg_loopback_exact_cancel_activate(loopback, exact);
	(void)virtio_msg_loopback_bridge_exact_claim_windows(loopback, exact);
	ret = virtio_msg_loopback_bridge_exact_cleanup(loopback, exact, true);

	return ret;
}

static int
virtio_msg_loopback_bridge_dma_provider_add_atomic
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_bus_dma_export *provider_export,
	 void **provider_handle, gfp_t gfp)
{
	struct virtio_msg_loopback_exact_export *exact;
	struct virtio_msg_loopback_slot *slot;
	struct virtio_msg_bus_dma_export local_export;
	unsigned long flags;
	u32 handle;
	int ret;

	if (!loopback || !provider_export || !provider_handle)
		return -EINVAL;

	ret = virtio_msg_loopback_provider_export_validate(provider_export);
	if (ret)
		return ret;
	local_export = *provider_export;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle)
		return -ENODEV;

	slot = virtio_msg_loopback_bridge_slot_from_dma_dev(loopback,
							    provider_export->dma_dev);
	if (!slot)
		return -ENODEV;

	exact = virtio_msg_loopback_exact_export_alloc_validated(&local_export,
								 gfp);
	if (IS_ERR(exact))
		return PTR_ERR(exact);

	exact->provider_export = local_export;
	exact->slot = slot;
	exact->async_activate = true;
	exact->gate_held = true;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	virtio_msg_loopback_exact_export_track_locked(loopback, exact);
	spin_unlock_irqrestore(&loopback->exact_lock, flags);

	atomic_inc(&slot->pending_async_exports);
	vm_trace(NULL,
		 "export create: dma=%#llx len=%#llx npages=%u async=1\n",
			(unsigned long long)local_export.dma_addr,
			(unsigned long long)local_export.length,
			local_export.npages);

	virtio_msg_loopback_exact_queue_activate(loopback, exact);

	*provider_export = local_export;
	*provider_handle = exact;

	return 0;
}

static int
virtio_msg_loopback_dma_provider_export_record
	(void *ctx, struct virtio_msg_bus_dma_export *export,
	 void **provider_handle, gfp_t gfp)
{
	struct virtio_msg_loopback *loopback = ctx;

	if (!loopback)
		return -ENODEV;

	if (!gfpflags_allow_blocking(gfp))
		return virtio_msg_loopback_bridge_dma_provider_add_atomic
			(loopback, export, provider_handle, gfp);

	return virtio_msg_loopback_bridge_dma_provider_add
		(loopback, export, provider_handle, gfp);
}

static int
virtio_msg_loopback_dma_provider_export_del_record
	(void *ctx, const struct virtio_msg_bus_dma_export *export,
	 void *provider_handle, bool sync)
{
	struct virtio_msg_loopback *loopback = ctx;
	struct virtio_msg_loopback_exact_export *exact = provider_handle;

	if (!loopback || !export)
		return -EOPNOTSUPP;

	if (!exact)
		return 0;

	if (sync) {
		int ret;

		ret = virtio_msg_loopback_bridge_dma_provider_del_sync
			(loopback, exact);
		if (ret != -EINPROGRESS)
			virtio_msg_loopback_exact_put(exact);
		return ret;
	}

	virtio_msg_loopback_bridge_dma_provider_del(loopback, exact);
	virtio_msg_loopback_exact_put(exact);

	return 0;
}

static int
virtio_msg_loopback_dma_provider_query_caps
	(void *ctx, struct virtio_msg_bus_dma_provider_caps *caps)
{
	if (!ctx || !caps)
		return -EINVAL;

	memset(caps, 0, sizeof(*caps));

	return 0;
}

static const struct virtio_msg_bus_dma_provider_ops
virtio_msg_loopback_dma_provider_ops = {
	.export_add = virtio_msg_loopback_dma_provider_export_record,
	.export_del = virtio_msg_loopback_dma_provider_export_del_record,
	.query_caps = virtio_msg_loopback_dma_provider_query_caps,
};

const struct virtio_msg_bus_dma_provider_ops *
virtio_msg_loopback_bridge_dma_provider_ops_get(void)
{
	return &virtio_msg_loopback_dma_provider_ops;
}

static int
virtio_msg_loopback_bridge_get_caps(struct virtio_msg_bus_bridge_device *endpoint,
				    struct virtio_msg_bus_bridge_device_caps *caps)
{
	if (!endpoint || !caps)
		return -EINVAL;

	memset(caps, 0, sizeof(*caps));
	caps->name = VIRTIO_MSG_LOOPBACK_BUS_NAME;
	caps->msg_size = VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE;
	caps->revision = VIRTIO_MSG_REVISION_1;

	return 0;
}

static int
virtio_msg_loopback_bridge_handle_transport_event(struct virtio_msg_loopback *loopback,
						  const struct virtio_msg *msg,
						  u16 msg_size)
{
	int ret;

	if (!loopback || !msg)
		return -EINVAL;
	if (msg->msg_id != VIRTIO_MSG_EVENT_AVAIL)
		return -EOPNOTSUPP;
	if (msg_size != sizeof(*msg) + sizeof(struct virtio_msg_event_avail))
		return -EMSGSIZE;

	ret = virtio_msg_loopback_validate_slot_dev_num(msg);
	if (ret)
		return ret;

	return virtio_msg_loopback_relay_to_userspace_nonblock(loopback, msg,
								msg_size);
}

static int
virtio_msg_loopback_bridge_handle_transport_request(struct virtio_msg_loopback *loopback,
						    const struct virtio_msg *msg,
						    u16 msg_size,
						    const struct virtio_msg_dispatch_ctx *dctx)
{
	int ret;

	if (!loopback || !msg)
		return -EINVAL;

	ret = virtio_msg_loopback_validate_slot_dev_num(msg);
	if (ret)
		return ret;
	if (msg->msg_id & VIRTIO_MSG_ID_EVENT_BIT)
		return -EOPNOTSUPP;

	return virtio_msg_loopback_relay_to_userspace_sleepable(loopback, msg,
								msg_size, dctx);
}

static int
virtio_msg_loopback_bridge_handle_bridge_request(struct virtio_msg_loopback *loopback,
						 const struct virtio_msg *msg,
						  u16 msg_size)
{
	struct virtio_msg_loopback_slot *slot;
	u16 dev_num;
	int ret;

	if (!loopback || !msg || msg_size < sizeof(*msg))
		return -EINVAL;

	dev_num = le16_to_cpu(msg->dev_num);
	switch (msg->msg_id) {
	case VIRTIO_MSG_EVENT_CONFIG:
	case VIRTIO_MSG_EVENT_USED:
		ret = virtio_msg_loopback_validate_slot_dev_num(msg);
		if (ret)
			return ret;
		slot =
			virtio_msg_loopback_slot_lookup_routable(loopback,
								 dev_num);
		if (!slot)
			return -ENODEV;

		return virtio_msg_transport_handle_device_event
			(&slot->vmdev, msg);
	default:
		return -EOPNOTSUPP;
	}
}

static int
virtio_msg_loopback_bridge_mmap(struct virtio_msg_bus_bridge_device *endpoint,
				u64 mmap_offset, struct vm_area_struct *vma)
{
	struct virtio_msg_loopback_page_window *window;
	struct virtio_msg_loopback *loopback;
	struct vm_struct *area;
	u64 mmap_len;
	int ret;

	if (!endpoint || !vma || !mmap_offset || !PAGE_ALIGNED(mmap_offset)) {
		vm_trace(NULL,
			 "bridge mmap reject: endpoint=%p vma=%p offset=%#llx\n",
		       endpoint, vma, (unsigned long long)mmap_offset);
		return -EINVAL;
	}

	loopback = endpoint->priv;
	if (!loopback) {
		vm_trace(NULL, "bridge mmap reject: no loopback offset=%#llx\n",
			 (unsigned long long)mmap_offset);
		return -ENODEV;
	}

	mmap_len = (u64)(vma->vm_end - vma->vm_start);
	if (!mmap_len) {
		vm_trace(NULL, "bridge mmap reject: zero length offset=%#llx\n",
			 (unsigned long long)mmap_offset);
		return -EINVAL;
	}

	mutex_lock(&loopback->area_lock);
	window = virtio_msg_loopback_page_window_lookup_by_offset_locked(loopback,
									 mmap_offset);
	if (!window ||
	    window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING ||
	    (!window->dma_mmap && !window->kva) || mmap_len > window->length) {
		vm_trace(NULL,
			 "bridge mmap reject: offset=%#llx len=%#llx window=%p map_id=%#llx state=%u kva=%p dma_mmap=%u window_len=%#llx\n",
		       (unsigned long long)mmap_offset,
		       (unsigned long long)mmap_len, window,
		       window ? (unsigned long long)window->map_id : 0,
		       window ? window->state : 0, window ? window->kva : NULL,
		       window ? window->dma_mmap : 0,
		       window ? (unsigned long long)window->length : 0);
		mutex_unlock(&loopback->area_lock);
		return -ENOENT;
	}
	virtio_msg_loopback_page_window_get(window);
	mutex_unlock(&loopback->area_lock);

	if (window->dma_mmap) {
		pgoff_t saved_pgoff = vma->vm_pgoff;

		vma->vm_pgoff = 0;
		ret = dma_mmap_attrs(window->dma_dev, vma, window->dma_cpu_addr,
				     window->dma_addr, window->length,
				     window->dma_attrs);
		vma->vm_pgoff = saved_pgoff;
		if (ret) {
			vm_trace(NULL,
				 "bridge mmap dma failed: offset=%#llx len=%#llx cpu=%p dma=%#llx attrs=%#lx ret=%d\n",
			       (unsigned long long)mmap_offset,
			       (unsigned long long)mmap_len,
			       window->dma_cpu_addr,
			       (unsigned long long)window->dma_addr,
			       window->dma_attrs, ret);
		}
		virtio_msg_loopback_page_window_put(window);
		return ret;
	}

	area = find_vm_area(window->kva);
	ret = remap_vmalloc_range(vma, window->kva, 0);
	if (ret) {
		vm_trace(NULL,
			 "bridge mmap remap failed: offset=%#llx len=%#llx kva=%p area=%p area_flags=%#lx ret=%d\n",
		       (unsigned long long)mmap_offset,
		       (unsigned long long)mmap_len, window->kva, area,
		       area ? area->flags : 0UL, ret);
	}
	virtio_msg_loopback_page_window_put(window);
	return ret;
}

static bool
virtio_msg_loopback_bridge_map_event_ack_add_async_retry
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window,
	 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status,
	 enum virtio_msg_loopback_map_state old_state)
{
	lockdep_assert_held(&loopback->area_lock);

	if (event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_ADD ||
	    !window->map_add_async ||
	    window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY ||
	    !window->owner_count || window->del_req_queued)
		return false;

	virtio_msg_loopback_page_window_clear_id_locked(loopback, window);
	virtio_msg_loopback_page_window_settle_add_locked(loopback, window,
							  -EAGAIN);
	vm_trace(NULL,
		 "rx MAP_ACK: type=%u map_id=%#llx bus=%#llx len=%#llx ack=%u old=%u new=%u del_req_queued=%u\n",
		       event->type, (unsigned long long)event->map_id,
		       (unsigned long long)window->bus_addr,
		       (unsigned long long)window->length, ack_status, old_state,
		       window->state, window->del_req_queued);
	return true;
}

static int
virtio_msg_loopback_bridge_map_event_ack
	(struct virtio_msg_bus_bridge_device *endpoint,
	 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status)
{
	struct virtio_msg_loopback_page_window *window;
	struct virtio_msg_loopback *loopback;
	enum virtio_msg_loopback_map_state old_state;
	bool lookup_by_offset = false;

	if (!endpoint || !event)
		return -EINVAL;

	loopback = endpoint->priv;
	if (!loopback)
		return -ENODEV;
	if (event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_ADD &&
	    event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_REQ)
		return 0;

	mutex_lock(&loopback->area_lock);
	window = virtio_msg_loopback_page_window_lookup_by_id_locked(loopback,
								     event->map_id);
	if (!window) {
		window = virtio_msg_loopback_page_window_lookup_by_offset_locked
			(loopback, event->mmap_offset);
		lookup_by_offset = true;
	}
	if (!window) {
		mutex_unlock(&loopback->area_lock);
		vm_trace(NULL,
			 "rx MAP_ACK ignored: type=%u map_id=%#llx mmap_off=%#llx ack=%u (no window)\n",
			       event->type, (unsigned long long)event->map_id,
			       (unsigned long long)event->mmap_offset, ack_status);
		return 0;
	}
	if (event->type == VMSG_BRIDGE_UAPI_MAP_EVENT_ADD &&
	    lookup_by_offset && window->map_add_async &&
	    (!window->map_id || window->map_id != event->map_id)) {
		mutex_unlock(&loopback->area_lock);
		vm_trace(NULL,
			 "rx MAP_ACK ignored: type=%u map_id=%#llx mmap_off=%#llx ack=%u (async mismatch)\n",
			       event->type, (unsigned long long)event->map_id,
			       (unsigned long long)event->mmap_offset, ack_status);
		return 0;
	}
	old_state = window->state;

	switch (ack_status) {
	case VMSG_BRIDGE_UAPI_MAP_ACK_OK:
		if (event->type == VMSG_BRIDGE_UAPI_MAP_EVENT_ADD) {
			if (window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY) {
				if (window->del_req_queued) {
					window->del_req_queued = false;
					window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING;
					virtio_msg_loopback_page_window_unindex_bus_addr_locked
						(loopback, window);
				} else {
					window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_ACTIVE;
				}
				virtio_msg_loopback_page_window_settle_add_locked
					(loopback, window, 0);
				window->map_add_async = false;
			}
			vm_trace(NULL,
				 "rx MAP_ACK: type=%u map_id=%#llx bus=%#llx len=%#llx ack=%u old=%u new=%u del_req_queued=%u\n",
				       event->type,
				       (unsigned long long)window->map_id,
				       (unsigned long long)window->bus_addr,
				       (unsigned long long)window->length,
				       ack_status, old_state, window->state,
				       window->del_req_queued);
		} else {
			vm_trace(NULL,
				 "rx MAP_ACK: type=%u map_id=%#llx bus=%#llx len=%#llx ack=%u old=%u new=%u\n",
				       event->type,
				       (unsigned long long)window->map_id,
				       (unsigned long long)window->bus_addr,
				       (unsigned long long)window->length,
				       ack_status, old_state, 0U);
			virtio_msg_loopback_page_window_remove_terminal_locked
				(loopback, window, 0);
			virtio_msg_loopback_page_window_put(window);
		}
		break;
	case VMSG_BRIDGE_UAPI_MAP_ACK_RETRY:
		if (!virtio_msg_loopback_bridge_map_event_ack_add_async_retry
			    (loopback, window, event, ack_status, old_state)) {
			vm_trace(NULL,
				 "rx MAP_ACK: type=%u map_id=%#llx bus=%#llx len=%#llx ack=%u old=%u new=%u del_req_queued=%u\n",
				       event->type,
				       (unsigned long long)window->map_id,
				       (unsigned long long)window->bus_addr,
				       (unsigned long long)window->length,
				       ack_status, old_state, window->state,
				       window->del_req_queued);
		}
		break;
	case VMSG_BRIDGE_UAPI_MAP_ACK_REJECT:
		if (virtio_msg_loopback_bridge_map_event_ack_add_async_retry
			    (loopback, window, event, ack_status, old_state))
			break;
		if (event->type == VMSG_BRIDGE_UAPI_MAP_EVENT_ADD)
			virtio_msg_loopback_page_window_settle_add_locked
				(loopback, window, -ENOMEM);
		vm_trace(NULL,
			 "rx MAP_ACK: type=%u map_id=%#llx bus=%#llx len=%#llx ack=%u old=%u new=%u\n",
			       event->type,
			       (unsigned long long)window->map_id,
			       (unsigned long long)window->bus_addr,
			       (unsigned long long)window->length,
			       ack_status, old_state, 0U);
		virtio_msg_loopback_page_window_remove_terminal_locked
			(loopback, window,
			 event->type == VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_REQ ?
			 -EREMOTEIO : -EINVAL);
		virtio_msg_loopback_page_window_put(window);
		break;
	default:
		vm_trace(NULL,
			 "rx MAP_ACK ignored: type=%u map_id=%#llx bus=%#llx len=%#llx ack=%u state=%u\n",
			       event->type,
			       (unsigned long long)window->map_id,
			       (unsigned long long)window->bus_addr,
			       (unsigned long long)window->length,
			       ack_status, old_state);
		break;
	}

	mutex_unlock(&loopback->area_lock);
	return 0;
}

static int
virtio_msg_loopback_bridge_report_error
		(struct virtio_msg_bus_bridge_device *endpoint, u16 dev_num,
		 u16 token, u8 msg_id, int error,
		 const struct virtio_msg_dispatch_ctx *dctx);

static int
virtio_msg_loopback_bridge_tx_msg(struct virtio_msg_bus_bridge_device *endpoint,
				  const struct virtio_msg *msg, u16 msg_size,
				   const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_loopback *loopback;
	enum virtio_msg_loopback_origin origin;
	u8 type_mask;

	if (!endpoint || !msg || msg_size < sizeof(*msg))
		return -EINVAL;
	if (msg_size != le16_to_cpu(msg->msg_size))
		return -EINVAL;

	loopback = endpoint->priv;
	if (!loopback)
		return -ENODEV;

	type_mask = msg->type & (VIRTIO_MSG_TYPE_RESPONSE | VIRTIO_MSG_TYPE_BUS);
	if (type_mask & VIRTIO_MSG_TYPE_BUS)
		return -EOPNOTSUPP;
	if (type_mask & VIRTIO_MSG_TYPE_RESPONSE)
		return virtio_msg_loopback_pending_complete_response(loopback, msg,
								     msg_size,
								     dctx);

	origin = virtio_msg_loopback_origin_classify(dctx);
	if (msg->msg_id & VIRTIO_MSG_ID_EVENT_BIT)
		vm_trace(NULL,
			 "bridge event dispatch: msg_id=0x%02x dev=%u origin=%d\n",
			     msg->msg_id, le16_to_cpu(msg->dev_num), origin);
	switch (origin) {
	case VIRTIO_MSG_LOOPBACK_ORIGIN_TRANSPORT_REQUEST:
		return virtio_msg_loopback_bridge_handle_transport_request
			(loopback, msg, msg_size, dctx);
	case VIRTIO_MSG_LOOPBACK_ORIGIN_TRANSPORT_EVENT:
		return virtio_msg_loopback_bridge_handle_transport_event
			(loopback, msg, msg_size);
	case VIRTIO_MSG_LOOPBACK_ORIGIN_BRIDGE:
		return virtio_msg_loopback_bridge_handle_bridge_request
			(loopback, msg, msg_size);
	default:
		return -EINVAL;
	}
}

static int
virtio_msg_loopback_bridge_notify_device_event
		(struct virtio_msg_bus_bridge_device *endpoint, u16 dev_num,
		 u16 dev_state)
{
	struct virtio_msg_loopback *loopback;
	int ret = 0;

	if (!endpoint)
		return -EINVAL;

	loopback = endpoint->priv;
	if (!loopback)
		return -ENODEV;
	if (!virtio_msg_loopback_dev_num_valid(dev_num))
		return 0;

	switch (dev_state) {
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_READY:
		schedule_work(&loopback->slots[dev_num].start_work);
		break;
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED:
		virtio_msg_loopback_slot_stop(loopback, dev_num);
		break;
	default:
		break;
	}

	return ret;
}

static const struct virtio_msg_bus_bridge_device_ops virtio_msg_loopback_ops = {
	.get_caps		= virtio_msg_loopback_bridge_get_caps,
	.tx_msg			= virtio_msg_loopback_bridge_tx_msg,
	.report_error		= virtio_msg_loopback_bridge_report_error,
	.mmap			= virtio_msg_loopback_bridge_mmap,
	.map_event_ack		= virtio_msg_loopback_bridge_map_event_ack,
	.notify_device_event	= virtio_msg_loopback_bridge_notify_device_event,
};

static int
virtio_msg_loopback_pending_take(struct virtio_msg_loopback *loopback,
				 u16 dev_num, u16 token,
				 const struct virtio_msg_dispatch_ctx *dctx,
				 struct virtio_msg_loopback_relay_pending **out_pending)
{
	struct virtio_msg_loopback_relay_pending *pending;
	unsigned long key;
	u64 relay_seq;

	if (!loopback || !dctx || !out_pending)
		return -EINVAL;

	if (token <= VIRTIO_MSG_TOKEN_FIXED || !dctx->relay_seq)
		return -EINVAL;

	key = (unsigned long)dctx->relay_seq;
	mutex_lock(&loopback->pending_lock);
	pending = xa_load(&loopback->pending_relay_xa, key);
	if (!pending) {
		mutex_unlock(&loopback->pending_lock);
		return -ENOENT;
	}
	if (pending->dev_num != dev_num || pending->token != token) {
		virtio_msg_loopback_pending_get(pending);
		pending->status = -EPROTO;
		pending->completed = true;
		xa_erase(&loopback->pending_relay_xa, key);
		mutex_unlock(&loopback->pending_lock);
		complete(&pending->done);
		virtio_msg_loopback_pending_put(pending);
		return -EPROTO;
	}
	relay_seq = pending->relay_seq;
	pending = xa_erase(&loopback->pending_relay_xa, relay_seq);
	if (!pending) {
		mutex_unlock(&loopback->pending_lock);
		return -ENOENT;
	}
	virtio_msg_loopback_pending_get(pending);
	mutex_unlock(&loopback->pending_lock);
	*out_pending = pending;

	return 0;
}

static void
virtio_msg_loopback_pending_complete(struct virtio_msg_loopback_relay_pending *pending)
{
	if (!pending)
		return;

	complete(&pending->done);
	virtio_msg_loopback_pending_put(pending);
}

int virtio_msg_loopback_pending_complete_response(struct virtio_msg_loopback
						  *loopback,
						  const struct virtio_msg *msg,
						  u16 msg_size,
						  const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_loopback_relay_pending *pending;
	u16 token;
	u16 dev_num;
	int ret = 0;

	if (!loopback || !msg || msg_size < sizeof(*msg) || !dctx)
		return -EINVAL;

	token = le16_to_cpu(msg->token);
	dev_num = le16_to_cpu(msg->dev_num);

	ret = virtio_msg_loopback_pending_take(loopback, dev_num, token, dctx,
					       &pending);
	if (ret)
		return ret;

	vm_trace(NULL, "response complete: dev=%u token=%u\n", dev_num, token);

	mutex_lock(&loopback->pending_lock);
	if (READ_ONCE(pending->timed_out)) {
		pending->status = -ETIMEDOUT;
	} else if (msg_size > pending->response_capacity) {
		pending->status = -EMSGSIZE;
		ret = -EMSGSIZE;
	} else {
		memcpy(pending->response, msg, msg_size);
		pending->response_size = msg_size;
		pending->status = 0;
		ret = 0;
	}
	pending->completed = true;
	mutex_unlock(&loopback->pending_lock);

	virtio_msg_loopback_pending_complete(pending);
	return ret;
}

static int
virtio_msg_loopback_bridge_report_error
		(struct virtio_msg_bus_bridge_device *endpoint, u16 dev_num,
		 u16 token, u8 msg_id, int error,
		 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_loopback_relay_pending *pending;
	struct virtio_msg_loopback *loopback;
	int ret;

	if (!endpoint || error >= 0)
		return -EINVAL;
	if (token <= VIRTIO_MSG_TOKEN_FIXED || (msg_id & VIRTIO_MSG_ID_EVENT_BIT))
		return -EINVAL;

	loopback = endpoint->priv;
	if (!loopback)
		return -ENODEV;

	vm_trace(NULL, "error report: dev=%u tok=%u id=0x%02x err=%d\n",
		 dev_num, token, msg_id, error);
	ret = virtio_msg_loopback_pending_take(loopback, dev_num, token, dctx,
					       &pending);
	if (ret)
		return ret;

	mutex_lock(&loopback->pending_lock);
	if (READ_ONCE(pending->timed_out))
		pending->status = -ETIMEDOUT;
	else
		pending->status = error;
	pending->completed = true;
	mutex_unlock(&loopback->pending_lock);

	virtio_msg_loopback_pending_complete(pending);
	return 0;
}

void virtio_msg_loopback_pending_abort_all(struct virtio_msg_loopback *loopback,
					   int status)
{
	struct virtio_msg_loopback_relay_pending *pending;
	unsigned long index = 0;

	if (!loopback)
		return;

	for (;;) {
		mutex_lock(&loopback->pending_lock);
		pending = xa_find(&loopback->pending_relay_xa, &index,
				  ULONG_MAX, XA_PRESENT);
		if (!pending) {
			mutex_unlock(&loopback->pending_lock);
			break;
		}

		xa_erase(&loopback->pending_relay_xa, index);
		virtio_msg_loopback_pending_get(pending);
		pending->status = status;
		pending->timed_out = status == -ETIMEDOUT;
		pending->completed = true;
		mutex_unlock(&loopback->pending_lock);

		complete(&pending->done);
		virtio_msg_loopback_pending_put(pending);
		index = 0;
	}
}

static int
virtio_msg_loopback_validate_addr(const struct vmsg_bus_resolver *resolver,
				  char *bus_id)
{
	if (!resolver || !bus_id)
		return -EINVAL;

	if (!bus_id[0]) {
		strscpy(bus_id, VIRTIO_MSG_LOOPBACK_BUS_ID,
			VMSG_BRIDGE_UAPI_BUS_ID_LEN);
	}

	if (strcmp(bus_id, VIRTIO_MSG_LOOPBACK_BUS_ID))
		return -ENODEV;

	return 0;
}

static struct virtio_msg_bus_bridge_device *
virtio_msg_loopback_match_addr(const struct vmsg_bus_resolver *resolver,
			       const char *bus_id)
{
	struct virtio_msg_loopback_bridge *bridge;

	if (!resolver || !bus_id)
		return NULL;

	bridge = container_of(resolver, struct virtio_msg_loopback_bridge,
			      resolver);
	if (!bridge->endpoint_registered)
		return NULL;
	if (strcmp(bus_id, bridge->bus_id))
		return NULL;

	return &bridge->endpoint;
}

int virtio_msg_loopback_bridge_start(struct virtio_msg_loopback *loopback)
{
	struct virtio_msg_loopback_bridge *bridge;
	int ret;

	if (!loopback)
		return -EINVAL;

	bridge = &loopback->bridge;
	if (bridge->resolver_registered || bridge->endpoint_registered)
		return -EBUSY;
	vm_info(NULL, "loopback bridge start: bus='%s'\n",
		VIRTIO_MSG_LOOPBACK_BUS_NAME);

	memset(bridge, 0, sizeof(*bridge));
	strscpy(bridge->bus_id, VIRTIO_MSG_LOOPBACK_BUS_ID,
		sizeof(bridge->bus_id));
	strscpy(bridge->resolver.name, VIRTIO_MSG_LOOPBACK_BUS_NAME,
		sizeof(bridge->resolver.name));
	bridge->resolver.validate = virtio_msg_loopback_validate_addr;
	bridge->resolver.match = virtio_msg_loopback_match_addr;

	loopback->exact_wq = alloc_ordered_workqueue("virtio_msg_loopback_exact",
						     WQ_MEM_RECLAIM);
	if (!loopback->exact_wq)
		return -ENOMEM;

	virtio_msg_bus_bridge_device_init(&bridge->endpoint,
					  &virtio_msg_loopback_ops, loopback);

	ret = vmsg_bus_resolver_register(&bridge->resolver);
	if (ret) {
		destroy_workqueue(loopback->exact_wq);
		loopback->exact_wq = NULL;
		return ret;
	}
	bridge->resolver_registered = true;

	ret = virtio_msg_bus_bridge_device_register(VIRTIO_MSG_LOOPBACK_BUS_NAME,
						    bridge->bus_id,
						    &bridge->endpoint);
	if (ret) {
		vmsg_bus_resolver_unregister(&bridge->resolver);
		bridge->resolver_registered = false;
		destroy_workqueue(loopback->exact_wq);
		loopback->exact_wq = NULL;
		return ret;
	}
	bridge->endpoint_registered = true;
	vm_info(NULL, "loopback bridge started: bus='%s' endpoint=%u\n",
		VIRTIO_MSG_LOOPBACK_BUS_NAME, bridge->endpoint.handle);

	return 0;
}

void virtio_msg_loopback_bridge_stop(struct virtio_msg_loopback *loopback)
{
	struct virtio_msg_loopback_bridge *bridge;

	if (!loopback)
		return;

	bridge = &loopback->bridge;
	vm_info(NULL, "loopback bridge stop: bus='%s' endpoint=%u\n",
		VIRTIO_MSG_LOOPBACK_BUS_NAME, bridge->endpoint.handle);
	virtio_msg_loopback_pending_abort_all(loopback, -ESHUTDOWN);
	if (loopback->exact_wq)
		drain_workqueue(loopback->exact_wq);
	virtio_msg_loopback_map_cleanup_all(loopback);
	if (loopback->exact_wq) {
		destroy_workqueue(loopback->exact_wq);
		loopback->exact_wq = NULL;
	}

	if (bridge->endpoint_registered) {
		virtio_msg_bus_bridge_device_unregister(VIRTIO_MSG_LOOPBACK_BUS_NAME,
							bridge->bus_id,
							&bridge->endpoint);
		bridge->endpoint_registered = false;
	}

	if (bridge->resolver_registered) {
		vmsg_bus_resolver_unregister(&bridge->resolver);
		bridge->resolver_registered = false;
	}
}
