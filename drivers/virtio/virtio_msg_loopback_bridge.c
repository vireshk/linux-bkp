// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message loopback bridge base scaffold.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-loopback: " fmt

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#include "virtio_msg_loopback_priv.h"

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
virtio_msg_loopback_relay_to_userspace_sleepable(struct virtio_msg_loopback *loopback,
						 const struct virtio_msg *msg,
						 u16 msg_size,
						 const struct virtio_msg_dispatch_ctx *dctx)
{
	u32 handle;

	if (!loopback || !msg || msg_size < sizeof(*msg) || !dctx)
		return -EINVAL;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle)
		return -ENODEV;

	return virtio_msg_bus_bridge_device_publish_rx(handle, msg);
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
virtio_msg_loopback_relay_event_to_userspace_sleepable
	(struct virtio_msg_loopback *loopback, const struct virtio_msg *msg,
	 u16 msg_size)
{
	u32 handle;

	if (!loopback || !msg || msg_size < sizeof(*msg))
		return -EINVAL;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle)
		return -ENODEV;

	return virtio_msg_bus_bridge_device_publish_rx(handle, msg);
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
	int ret;

	lockdep_assert_held(&loopback->area_lock);

	ret = virtio_msg_loopback_map_dma_addr_to_index(window->bus_addr,
							&bus_index);
	if (ret)
		return ret;

	return xa_err(xa_store(&loopback->map_area_xa_by_dma_addr, bus_index,
			       window, GFP_KERNEL));
}

static void
virtio_msg_loopback_exact_export_track_locked(struct virtio_msg_loopback *loopback,
					      struct virtio_msg_loopback_exact_export *exact)
{
	lockdep_assert_held(&loopback->exact_lock);
	list_add_tail(&exact->node, &loopback->exact_exports);
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
	lockdep_assert_held(&loopback->area_lock);

	if (!window)
		return;

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
virtio_msg_loopback_page_window_prepare_add_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window)
{
	lockdep_assert_held(&loopback->area_lock);

	if (!window)
		return;

	reinit_completion(&window->map_add_done);
	window->map_add_status = 0;
	window->map_add_settled = false;
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

static void virtio_msg_loopback_exact_cleanup_workfn(struct work_struct *work);

static void
virtio_msg_loopback_exact_queue_cleanup(struct virtio_msg_loopback *loopback,
					struct virtio_msg_loopback_exact_export *exact)
{
	unsigned long flags;
	bool take_ref = false;

	if (!loopback || !loopback->exact_cleanup_wq || !exact)
		return;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	if (!exact->cleanup_ref_held) {
		exact->cleanup_ref_held = true;
		take_ref = true;
	}
	spin_unlock_irqrestore(&loopback->exact_lock, flags);

	if (!take_ref)
		return;

	virtio_msg_loopback_exact_get(exact);
	if (!queue_delayed_work(loopback->exact_cleanup_wq,
				&exact->cleanup_work, 0)) {
		spin_lock_irqsave(&loopback->exact_lock, flags);
		exact->cleanup_ref_held = false;
		spin_unlock_irqrestore(&loopback->exact_lock, flags);
		virtio_msg_loopback_exact_put(exact);
	}
}

static void
virtio_msg_loopback_exact_cancel_cleanup(struct virtio_msg_loopback *loopback,
					 struct virtio_msg_loopback_exact_export *exact)
{
	unsigned long flags;
	bool drop_ref = false;

	if (!loopback || !loopback->exact_cleanup_wq || !exact)
		return;

	cancel_delayed_work_sync(&exact->cleanup_work);

	spin_lock_irqsave(&loopback->exact_lock, flags);
	if (exact->cleanup_ref_held) {
		exact->cleanup_ref_held = false;
		drop_ref = true;
	}
	spin_unlock_irqrestore(&loopback->exact_lock, flags);

	if (drop_ref)
		virtio_msg_loopback_exact_put(exact);
}

static struct virtio_msg_loopback_exact_export *
virtio_msg_loopback_exact_export_alloc_validated
	(const struct virtio_msg_bus_dma_driver_export *provider_export,
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
	exact->cleanup_ref_held = false;
	INIT_DELAYED_WORK(&exact->cleanup_work,
			  virtio_msg_loopback_exact_cleanup_workfn);
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
	 const struct virtio_msg_bus_dma_driver_export *provider_export,
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

static u32
virtio_msg_loopback_page_window_map_flags
	(const struct virtio_msg_bus_dma_driver_export *provider_export)
{
	u32 flags = 0;

	if (!provider_export)
		return 0;

	if (provider_export->bridge_flags &
	    VIRTIO_MSG_BUS_BRIDGE_MAP_F_RETENTION_REQUESTED)
		flags |= VIRTIO_MSG_BUS_BRIDGE_MAP_F_RETENTION_REQUESTED;
	if (provider_export->coherent)
		flags |= VIRTIO_MSG_BUS_BRIDGE_MAP_F_RETENTION_REQUESTED;

	return flags;
}

static struct virtio_msg_loopback_page_window *
virtio_msg_loopback_page_window_alloc
	(struct page *page, u64 bus_addr,
	 const struct virtio_msg_bus_dma_driver_export *provider_export,
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
	window->map_flags =
		virtio_msg_loopback_page_window_map_flags(provider_export);
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
					    const struct virtio_msg_bus_dma_driver_export
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
	window->map_flags =
		virtio_msg_loopback_page_window_map_flags(provider_export);
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
 * Register one bus_addr entry per page in the window's range.  On failure,
 * all entries already stored are rolled back.
 */
static int
virtio_msg_loopback_page_window_track_multi_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window)
{
	unsigned int i;
	int ret;

	lockdep_assert_held(&loopback->area_lock);

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
	unsigned long flags;

	if (!loopback || !exact)
		return;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	(void)virtio_msg_loopback_exact_export_untrack_locked(loopback, exact);
	spin_unlock_irqrestore(&loopback->exact_lock, flags);
	pr_debug("unpublish export: dma=%#llx len=%#llx\n",
		 (unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length);
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

static bool
virtio_msg_loopback_page_window_mmap_allowed_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window,
	 const struct virtio_msg_bus_dma_device_map_info *map)
{
	lockdep_assert_held(&loopback->area_lock);

	if (!window || !map)
		return false;
	if (map->state != VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_QUEUED &&
	    map->state != VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ACTIVE)
		return false;
	if (window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY &&
	    window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_ACTIVE)
		return false;
	if (!virtio_msg_loopback_page_window_is_tracked_locked(loopback,
							       window))
		return false;
	if (!window->dma_mmap && !window->kva)
		return false;

	return true;
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
	 const struct virtio_msg_bus_dma_driver_export *provider_export)
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
	 const struct virtio_msg_bus_dma_driver_export *provider_export)
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
		u32 map_flags;
		int ret;

		if (!provider_export->pages[i])
			return -EINVAL;
		if (check_add_overflow(bus_addr, (u64)i * PAGE_SIZE,
				       &window_bus_addr))
			return -EOVERFLOW;
		map_flags =
			virtio_msg_loopback_page_window_map_flags(provider_export);

		window = virtio_msg_loopback_page_window_lookup_by_bus_addr_locked
			(loopback, window_bus_addr);
		if (window) {
			if (window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_ACTIVE &&
			    window->state !=
				    VIRTIO_MSG_LOOPBACK_MAP_STATE_REMOTE_RELEASED)
				return -EBUSY;
			if (window->map_flags != map_flags)
				return -EINVAL;
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
			pr_debug("owner drop: bus=%#llx len=%#llx map_id=%#llx state=%u\n",
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
		pr_debug("%s: map_id=%#llx bus=%#llx len=%#llx status=%d\n",
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

struct virtio_msg_loopback_page_window_action {
	u64 map_id;
	u64 bus_addr;
	u64 length;
	enum virtio_msg_loopback_map_state old_state;
	bool send_del_req;
	bool drop_window;
	bool wait_revoke;
};

static void
virtio_msg_loopback_page_window_action_init
	(struct virtio_msg_loopback_page_window_action *action,
	 struct virtio_msg_loopback_page_window *window)
{
	memset(action, 0, sizeof(*action));
	if (!window)
		return;

	action->map_id = window->map_id;
	action->bus_addr = window->bus_addr;
	action->length = window->length;
	action->old_state = window->state;
}

static bool
virtio_msg_loopback_page_window_begin_add_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window)
{
	lockdep_assert_held(&loopback->area_lock);

	if (!window->owner_count ||
	    (window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY &&
	     window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_REMOTE_RELEASED) ||
	    window->map_id ||
	    !virtio_msg_loopback_page_window_is_tracked_locked(loopback, window))
		return false;

	window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY;
	virtio_msg_loopback_page_window_prepare_add_locked(loopback, window);
	return true;
}

static int
virtio_msg_loopback_page_window_record_map_id_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window, u64 map_id)
{
	lockdep_assert_held(&loopback->area_lock);

	if (!window || !map_id)
		return -EINVAL;
	if (window->map_id && window->map_id != map_id)
		return -ESTALE;
	if (!virtio_msg_loopback_page_window_is_tracked_locked(loopback, window))
		return -ESTALE;

	window->map_id = map_id;
	return 0;
}

static void
virtio_msg_loopback_page_window_add_ack_ok_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window)
{
	lockdep_assert_held(&loopback->area_lock);

	if (window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY)
		return;

	if (window->del_req_queued) {
		window->del_req_queued = false;
		window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING;
		virtio_msg_loopback_page_window_unindex_bus_addr_locked
			(loopback, window);
	} else {
		window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_ACTIVE;
	}
	virtio_msg_loopback_page_window_settle_add_locked(loopback, window, 0);
}

static bool
virtio_msg_loopback_page_window_begin_revoke_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window, bool wait,
	 u32 handle, struct virtio_msg_loopback_page_window_action *action)
{
	lockdep_assert_held(&loopback->area_lock);

	virtio_msg_loopback_page_window_action_init(action, window);
	action->wait_revoke = wait;

	if (window->owner_count)
		return false;

	if (window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING ||
	    window->del_req_queued)
		return true;

	if (!virtio_msg_loopback_page_window_is_tracked_locked(loopback, window))
		return false;

	if (!handle || !window->map_id) {
		virtio_msg_loopback_page_window_remove_terminal_locked(loopback,
								       window, 0);
		action->drop_window = true;
		return true;
	}

	action->send_del_req = true;
	if (window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY) {
		window->del_req_queued = true;
		return true;
	}

	window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING;
	virtio_msg_loopback_page_window_unindex_bus_addr_locked(loopback,
								window);
	return true;
}

static bool
virtio_msg_loopback_page_window_finish_revoke_req_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window, int status)
{
	lockdep_assert_held(&loopback->area_lock);

	if (!status || window->owner_count ||
	    (window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING &&
	     !window->del_req_queued))
		return false;

	window->del_req_queued = false;
	virtio_msg_loopback_page_window_remove_terminal_locked(loopback, window,
							       status);
	return true;
}

static bool
virtio_msg_loopback_page_window_rollback_locked
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window, int status,
	 struct virtio_msg_loopback_page_window_action *action)
{
	lockdep_assert_held(&loopback->area_lock);

	virtio_msg_loopback_page_window_action_init(action, window);

	if (window->owner_count ||
	    !virtio_msg_loopback_page_window_is_tracked_locked(loopback, window))
		return false;

	virtio_msg_loopback_page_window_remove_terminal_locked(loopback, window,
							       status);
	action->drop_window = true;
	action->send_del_req = action->map_id &&
			       action->old_state !=
				       VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING;

	return true;
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
	pr_debug("cleanup export begin: dma=%#llx len=%#llx sync=%u windows=%u\n",
		 (unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length,
			sync, exact->num_unique_windows);

	mutex_lock(&loopback->area_lock);
	virtio_msg_loopback_exact_export_release_page_windows_locked(loopback,
								     exact);
	mutex_unlock(&loopback->area_lock);

	for (i = 0; i < exact->num_unique_windows; i++) {
		struct virtio_msg_loopback_page_window_action action;
		int ret;
		bool handled;

		window = exact->unique_windows[i];
		if (!window)
			continue;

		mutex_lock(&loopback->area_lock);
		handled = virtio_msg_loopback_page_window_begin_revoke_locked
			(loopback, window, sync, handle, &action);
		mutex_unlock(&loopback->area_lock);
		if (!handled)
			continue;

		if (action.drop_window) {
			pr_debug("cleanup local remove: map_id=%#llx bus=%#llx len=%#llx handle=%u\n",
				 (unsigned long long)action.map_id,
					(unsigned long long)action.bus_addr,
					(unsigned long long)action.length, handle);
			virtio_msg_loopback_bridge_exact_cleanup_wait_revoke
				(window, action.wait_revoke, &ret_sync,
				 action.map_id, action.bus_addr, action.length,
				 "cleanup local revoke done");
			virtio_msg_loopback_page_window_put(window);
			continue;
		}

		if (!action.send_del_req) {
			pr_debug("cleanup wait revoke: map_id=%#llx bus=%#llx len=%#llx sync=%u\n",
				 (unsigned long long)action.map_id,
					(unsigned long long)action.bus_addr,
					(unsigned long long)action.length, sync);
			virtio_msg_loopback_bridge_exact_cleanup_wait_revoke
				(window, action.wait_revoke, &ret_sync,
				 action.map_id, action.bus_addr, action.length,
				 "cleanup revoke done");
			continue;
		}

		if (action.old_state == VIRTIO_MSG_LOOPBACK_MAP_STATE_MMAP_READY)
			pr_debug("revoke trigger queued: map_id=%#llx bus=%#llx len=%#llx state=%u\n",
				 (unsigned long long)action.map_id,
					(unsigned long long)action.bus_addr,
					(unsigned long long)action.length,
					action.old_state);
		else
			pr_debug("revoke trigger: map_id=%#llx bus=%#llx len=%#llx old=%u new=%u\n",
				 (unsigned long long)action.map_id,
					(unsigned long long)action.bus_addr,
					(unsigned long long)action.length,
					action.old_state,
					VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING);

		pr_debug("tx MAP_DEL_REQ: map_id=%#llx bus=%#llx len=%#llx\n",
			 (unsigned long long)action.map_id,
			       (unsigned long long)action.bus_addr,
			       (unsigned long long)action.length);
		ret = virtio_msg_bus_dma_device_helper_map_del_req
			(&loopback->map_helper, handle, action.map_id);
		if (ret == -EALREADY)
			ret = 0;
		if (ret)
			pr_debug("tx MAP_DEL_REQ failed: map_id=%#llx bus=%#llx len=%#llx ret=%d\n",
				 (unsigned long long)action.map_id,
				       (unsigned long long)action.bus_addr,
				       (unsigned long long)action.length, ret);

		mutex_lock(&loopback->area_lock);
		action.drop_window =
			virtio_msg_loopback_page_window_finish_revoke_req_locked
				(loopback, window, ret);
		mutex_unlock(&loopback->area_lock);

		if (action.drop_window) {
			pr_debug("cleanup remove after del_req failure: map_id=%#llx bus=%#llx len=%#llx ret=%d\n",
				 (unsigned long long)action.map_id,
					(unsigned long long)action.bus_addr,
					(unsigned long long)action.length, ret);
			if (!ret_sync)
				ret_sync = ret;
			virtio_msg_loopback_page_window_put(window);
			continue;
		}

		virtio_msg_loopback_bridge_exact_cleanup_wait_revoke
			(window, action.wait_revoke, &ret_sync, action.map_id,
			 action.bus_addr, action.length, "cleanup revoke done");
	}

	pr_debug("cleanup export done: dma=%#llx len=%#llx ret=%d\n",
		 (unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length, ret_sync);
	return ret_sync;
}

static int
virtio_msg_loopback_exact_activate_sleepable
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_exact_export *exact)
{
	const struct virtio_msg_bus_dma_driver_export *provider_export;
	struct virtio_msg_loopback_page_window **queued_windows = NULL;
	unsigned int queued_count = 0;
	u32 handle;
	int ret = 0;
	unsigned int i;

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

	if (exact->num_unique_windows) {
		queued_windows = kcalloc(exact->num_unique_windows,
					 sizeof(*queued_windows), GFP_KERNEL);
		if (!queued_windows) {
			ret = -ENOMEM;
			goto out;
		}
	}

	for (i = 0; i < exact->num_unique_windows; i++) {
		struct virtio_msg_loopback_page_window *window;
		struct virtio_msg_bus_dma_device_map_desc map_desc;
		u64 map_id = 0;
		bool submit = false;

		window = exact->unique_windows[i];
		if (!window)
			continue;

		mutex_lock(&loopback->area_lock);
		submit = virtio_msg_loopback_page_window_begin_add_locked
			(loopback, window);
		mutex_unlock(&loopback->area_lock);
		if (!submit)
			continue;

		pr_debug("tx MAP_ADD: idx=%u bus=%#llx len=%#llx mmap_off=%#llx\n",
			 i, (unsigned long long)window->bus_addr,
			       (unsigned long long)window->length,
			       (unsigned long long)window->mmap_offset);

		memset(&map_desc, 0, sizeof(map_desc));
		map_desc.bus_addr = window->bus_addr;
		map_desc.length = window->length;
		map_desc.mmap_offset = window->mmap_offset;
		map_desc.mmap_length = window->length;
		map_desc.flags = window->map_flags;
		map_desc.cookie = window;

		ret = virtio_msg_bus_dma_device_helper_map_add
			(&loopback->map_helper, handle, &map_desc, &map_id);
		if (ret) {
			pr_debug("tx MAP_ADD failed: idx=%u bus=%#llx len=%#llx ret=%d\n",
				 i, (unsigned long long)window->bus_addr,
				       (unsigned long long)window->length, ret);
			goto wait_queued;
		}
		queued_windows[queued_count++] = window;

		mutex_lock(&loopback->area_lock);
		ret = virtio_msg_loopback_page_window_record_map_id_locked
			(loopback, window, map_id);
		mutex_unlock(&loopback->area_lock);
		if (ret) {
			(void)virtio_msg_bus_dma_device_helper_map_del_req
				(&loopback->map_helper, handle, map_id);
			pr_debug("tx MAP_ADD stale: map_id=%#llx bus=%#llx len=%#llx ret=%d\n",
				 (unsigned long long)map_id,
					(unsigned long long)window->bus_addr,
					(unsigned long long)window->length, ret);
			goto wait_queued;
		}

		pr_debug("tx MAP_ADD queued: map_id=%#llx bus=%#llx len=%#llx\n",
			 (unsigned long long)map_id,
			       (unsigned long long)window->bus_addr,
			       (unsigned long long)window->length);
	}

wait_queued:
	for (i = 0; i < queued_count; i++) {
		struct virtio_msg_loopback_page_window *window;
		int add_status = 0;
		bool wait_add = false;

		window = queued_windows[i];

		mutex_lock(&loopback->area_lock);
		if (!window->owner_count ||
		    !virtio_msg_loopback_page_window_is_tracked_locked(loopback,
								      window)) {
			mutex_unlock(&loopback->area_lock);
			continue;
		}
		if (window->map_add_settled)
			add_status = window->map_add_status;
		else
			wait_add = true;
		mutex_unlock(&loopback->area_lock);

		if (wait_add)
			add_status = virtio_msg_loopback_page_window_wait_add_result
				(window, VIRTIO_MSG_LOOPBACK_RELAY_TIMEOUT_MS);

		if (add_status) {
			if (!ret)
				ret = add_status;
		}
	}

out:
	kfree(queued_windows);
	return ret;
}

static void virtio_msg_loopback_exact_cleanup_workfn(struct work_struct *work)
{
	struct virtio_msg_loopback_exact_export *exact;
	struct virtio_msg_loopback_slot *slot;
	struct virtio_msg_loopback *loopback;
	unsigned long flags;
	bool do_cleanup = false;

	exact = container_of(to_delayed_work(work),
			     struct virtio_msg_loopback_exact_export,
			     cleanup_work);

	slot = READ_ONCE(exact->slot);
	loopback = slot ? slot->loopback : NULL;
	if (loopback && READ_ONCE(exact->dead)) {
		do_cleanup = virtio_msg_loopback_bridge_exact_claim_windows
			(loopback, exact);
	}

	if (do_cleanup)
		(void)virtio_msg_loopback_bridge_exact_cleanup(loopback, exact,
								false);

	if (loopback) {
		spin_lock_irqsave(&loopback->exact_lock, flags);
		exact->cleanup_ref_held = false;
		spin_unlock_irqrestore(&loopback->exact_lock, flags);
	}
	virtio_msg_loopback_exact_put(exact);
}

struct virtio_msg_loopback_map_cleanup_ctx {
	struct virtio_msg_loopback *loopback;
	u32 handle;
};

static void
virtio_msg_loopback_map_cleanup_one
	(const struct virtio_msg_bus_dma_device_map_info *map, void *data)
{
	struct virtio_msg_loopback_map_cleanup_ctx *ctx = data;
	struct virtio_msg_loopback_page_window *window;
	struct virtio_msg_loopback *loopback;
	u64 map_id = 0;

	if (!ctx || !ctx->loopback || !map)
		return;

	loopback = ctx->loopback;
	window = map->cookie;
	if (!window)
		return;

	mutex_lock(&loopback->area_lock);
	if (window->map_id != map->map_id ||
	    window->bus_addr != map->bus_addr ||
	    window->length != map->length ||
	    window->mmap_offset != map->mmap_offset ||
	    window->length != map->mmap_length ||
	    window->map_flags != map->flags) {
		mutex_unlock(&loopback->area_lock);
		return;
	}

	virtio_msg_loopback_page_window_get(window);
	if (ctx->handle && window->map_id &&
	    window->state != VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING)
		map_id = window->map_id;
	virtio_msg_loopback_page_window_remove_terminal_locked
		(loopback, window,
		 window->state == VIRTIO_MSG_LOOPBACK_MAP_STATE_DEL_PENDING ?
		 -ENODEV : 0);
	mutex_unlock(&loopback->area_lock);

	if (ctx->handle && map_id)
		(void)virtio_msg_bus_bridge_device_map_del(ctx->handle, map_id);
	virtio_msg_loopback_page_window_put(window);
}

static void
virtio_msg_loopback_map_cleanup_all(struct virtio_msg_loopback *loopback)
{
	struct virtio_msg_loopback_map_cleanup_ctx ctx;
	struct virtio_msg_loopback_exact_export *exact;
	unsigned long flags;

	if (!loopback)
		return;

	ctx.loopback = loopback;
	ctx.handle = READ_ONCE(loopback->bridge.endpoint.handle);

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
		virtio_msg_loopback_exact_cancel_cleanup(loopback, exact);
		do_cleanup = virtio_msg_loopback_bridge_exact_claim_windows
			(loopback, exact);

		if (do_cleanup)
			(void)virtio_msg_loopback_bridge_exact_cleanup(loopback,
									exact,
									false);

		virtio_msg_loopback_exact_put(exact);
	}

	virtio_msg_bus_dma_device_helper_purge(&loopback->map_helper,
					       virtio_msg_loopback_map_cleanup_one,
					       &ctx);
}

static int
virtio_msg_loopback_provider_export_validate
	(const struct virtio_msg_bus_dma_driver_export *provider_export)
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
	if (provider_export->bridge_flags &
	    ~VIRTIO_MSG_BUS_BRIDGE_MAP_F_RETENTION_REQUESTED)
		return -EINVAL;

	return 0;
}

void
virtio_msg_loopback_bridge_dma_install_begin(struct virtio_msg_loopback_slot *slot)
{
	if (!slot)
		return;

	WRITE_ONCE(slot->dma_initial_pool_status, 0);
	WRITE_ONCE(slot->dma_initial_pool_seen, false);
	WRITE_ONCE(slot->dma_initial_pool_installing, true);
}

int
virtio_msg_loopback_bridge_dma_install_finish(struct virtio_msg_loopback_slot *slot)
{
	int ret;

	if (!slot)
		return -EINVAL;

	WRITE_ONCE(slot->dma_initial_pool_installing, false);
	if (!READ_ONCE(slot->dma_initial_pool_seen))
		return 0;

	ret = READ_ONCE(slot->dma_initial_pool_status);
	return ret ?: 0;
}

static void
virtio_msg_loopback_bridge_dma_initial_pool_result
	(struct virtio_msg_loopback_slot *slot,
	 const struct virtio_msg_bus_dma_driver_export *provider_export,
	 int status)
{
	if (!slot || !provider_export ||
	    provider_export->type != VIRTIO_MSG_BUS_DMA_DRIVER_EXPORT_POOL_CHUNK ||
	    !READ_ONCE(slot->dma_initial_pool_installing))
		return;

	WRITE_ONCE(slot->dma_initial_pool_seen, true);
	if (status)
		WRITE_ONCE(slot->dma_initial_pool_status, status);
}

static int
virtio_msg_loopback_bridge_dma_provider_add
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_bus_dma_driver_export *provider_export,
	 void **provider_handle, gfp_t gfp)
{
	struct virtio_msg_loopback_exact_export *exact;
	struct virtio_msg_loopback_slot *slot;
	struct virtio_msg_bus_dma_driver_export local_export;
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

	slot = virtio_msg_loopback_bridge_slot_from_dma_dev(loopback,
							    provider_export->dma_dev);
	if (!slot)
		return -ENODEV;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle) {
		ret = -ENODEV;
		virtio_msg_loopback_bridge_dma_initial_pool_result
			(slot, provider_export, ret);
		return ret;
	}

	exact = virtio_msg_loopback_exact_export_alloc_validated(&local_export,
								 gfp);
	if (IS_ERR(exact)) {
		ret = PTR_ERR(exact);
		virtio_msg_loopback_bridge_dma_initial_pool_result
			(slot, provider_export, ret);
		return ret;
	}

	exact->provider_export = local_export;
	exact->slot = slot;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	virtio_msg_loopback_exact_export_track_locked(loopback, exact);
	spin_unlock_irqrestore(&loopback->exact_lock, flags);
	tracked = true;
	pr_debug("export create: dma=%#llx len=%#llx npages=%u\n",
		 (unsigned long long)local_export.dma_addr,
			(unsigned long long)local_export.length,
			local_export.npages);

	ret = virtio_msg_loopback_exact_activate_sleepable(loopback, exact);
	virtio_msg_loopback_bridge_dma_initial_pool_result
		(slot, provider_export, ret);
	if (ret) {
		pr_debug("export create failed: dma=%#llx len=%#llx ret=%d\n",
			 (unsigned long long)local_export.dma_addr,
				(unsigned long long)local_export.length, ret);
		goto rollback;
	}

	*provider_export = local_export;
	*provider_handle = exact;

	return 0;

rollback:
	pr_debug("export rollback: dma=%#llx len=%#llx ret=%d\n",
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
		struct virtio_msg_loopback_page_window_action action;

		window = exact->unique_windows[i];
		if (!window)
			continue;

		mutex_lock(&loopback->area_lock);
		(void)virtio_msg_loopback_page_window_rollback_locked
			(loopback, window, ret, &action);
		mutex_unlock(&loopback->area_lock);
		if (!action.drop_window)
			continue;

		if (handle && action.send_del_req)
			(void)virtio_msg_bus_dma_device_helper_map_del_req
				(&loopback->map_helper, handle, action.map_id);
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

	pr_debug("export revoke trigger: dma=%#llx len=%#llx sync=0\n",
		 (unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length);
	WRITE_ONCE(exact->dead, true);
	virtio_msg_loopback_bridge_exact_unpublish(loopback, exact);
	virtio_msg_loopback_exact_queue_cleanup(loopback, exact);
}

static int
virtio_msg_loopback_bridge_dma_provider_del_sync(struct virtio_msg_loopback *loopback,
						 struct virtio_msg_loopback_exact_export *exact)
{
	int ret = 0;

	if (!loopback || !exact)
		return -EINVAL;

	pr_debug("export revoke trigger: dma=%#llx len=%#llx sync=1\n",
		 (unsigned long long)exact->provider_export.dma_addr,
			(unsigned long long)exact->provider_export.length);
	WRITE_ONCE(exact->dead, true);
	virtio_msg_loopback_bridge_exact_unpublish(loopback, exact);
	virtio_msg_loopback_exact_cancel_cleanup(loopback, exact);
	(void)virtio_msg_loopback_bridge_exact_claim_windows(loopback, exact);
	ret = virtio_msg_loopback_bridge_exact_cleanup(loopback, exact, true);

	return ret;
}

static int
virtio_msg_loopback_dma_provider_export_record
	(void *ctx, struct virtio_msg_bus_dma_driver_export *export,
	 void **provider_handle, gfp_t gfp)
{
	struct virtio_msg_loopback *loopback = ctx;

	if (!loopback)
		return -ENODEV;

	if (!gfpflags_allow_blocking(gfp))
		return -EAGAIN;

	return virtio_msg_loopback_bridge_dma_provider_add
		(loopback, export, provider_handle, gfp);
}

static int
virtio_msg_loopback_dma_provider_export_del_record
	(void *ctx, const struct virtio_msg_bus_dma_driver_export *export,
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
	(void *ctx, struct virtio_msg_bus_dma_driver_provider_caps *caps)
{
	if (!ctx || !caps)
		return -EINVAL;

	memset(caps, 0, sizeof(*caps));

	return 0;
}

static const struct virtio_msg_bus_dma_driver_provider_ops
virtio_msg_loopback_dma_provider_ops = {
	.export_add = virtio_msg_loopback_dma_provider_export_record,
	.export_del = virtio_msg_loopback_dma_provider_export_del_record,
	.query_caps = virtio_msg_loopback_dma_provider_query_caps,
};

const struct virtio_msg_bus_dma_driver_provider_ops *
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
	if (msg_size < sizeof(*msg) ||
	    msg_size > VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE)
		return -EMSGSIZE;
	if (msg_size != le16_to_cpu(msg->msg_size))
		return -EINVAL;
	if (!(msg->msg_id & VIRTIO_MSG_ID_EVENT_BIT))
		return -EOPNOTSUPP;
	if (msg->type & VIRTIO_MSG_TYPE_BUS)
		return -EOPNOTSUPP;
	if (msg->type & VIRTIO_MSG_TYPE_RESPONSE)
		return -EINVAL;
	if (msg->msg_id == VIRTIO_MSG_EVENT_AVAIL &&
	    msg_size != sizeof(*msg) + sizeof(struct virtio_msg_event_avail))
		return -EMSGSIZE;

	ret = virtio_msg_loopback_validate_slot_dev_num(msg);
	if (ret)
		return ret;

	return virtio_msg_loopback_relay_to_userspace_nonblock(loopback, msg,
								msg_size);
}

int
virtio_msg_loopback_bridge_publish_event_sleepable(struct virtio_msg_loopback
						  *loopback,
						  const struct virtio_msg *msg)
{
	u16 msg_size;
	int ret;

	if (!loopback || !msg)
		return -EINVAL;

	msg_size = le16_to_cpu(msg->msg_size);
	if (msg_size < sizeof(*msg) ||
	    msg_size > VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE)
		return -EMSGSIZE;
	if (!(msg->msg_id & VIRTIO_MSG_ID_EVENT_BIT))
		return -EOPNOTSUPP;
	if (msg->type & VIRTIO_MSG_TYPE_BUS)
		return -EOPNOTSUPP;
	if (msg->type & VIRTIO_MSG_TYPE_RESPONSE)
		return -EINVAL;
	if (msg->msg_id == VIRTIO_MSG_EVENT_AVAIL &&
	    msg_size != sizeof(*msg) + sizeof(struct virtio_msg_event_avail))
		return -EMSGSIZE;

	ret = virtio_msg_loopback_validate_slot_dev_num(msg);
	if (ret)
		return ret;

	return virtio_msg_loopback_relay_event_to_userspace_sleepable
		(loopback, msg, msg_size);
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

static struct virtio_msg_loopback_page_window *
virtio_msg_loopback_page_window_from_map_locked
	(struct virtio_msg_loopback *loopback,
	 const struct virtio_msg_bus_dma_device_map_info *map)
{
	struct virtio_msg_loopback_page_window *window;

	lockdep_assert_held(&loopback->area_lock);

	if (!map || !map->cookie)
		return NULL;

	window = map->cookie;
	if (window->map_id && window->map_id != map->map_id)
		return NULL;
	if (window->bus_addr != map->bus_addr ||
	    window->length != map->length ||
	    window->mmap_offset != map->mmap_offset ||
	    window->length != map->mmap_length ||
	    window->map_flags != map->flags)
		return NULL;

	return window;
}

static int
virtio_msg_loopback_bridge_mmap(struct virtio_msg_bus_bridge_device *endpoint,
				u64 mmap_offset, struct vm_area_struct *vma)
{
	struct virtio_msg_bus_dma_device_map_info map;
	struct virtio_msg_loopback_page_window *window;
	struct virtio_msg_loopback *loopback;
	struct vm_struct *area;
	u64 mmap_len;
	int ret;

	if (!endpoint || !vma || !mmap_offset || !PAGE_ALIGNED(mmap_offset)) {
		pr_debug("bridge mmap reject: endpoint=%p vma=%p offset=%#llx\n",
			 endpoint, vma, (unsigned long long)mmap_offset);
		return -EINVAL;
	}

	loopback = endpoint->priv;
	if (!loopback) {
		pr_debug("bridge mmap reject: no loopback offset=%#llx\n",
			 (unsigned long long)mmap_offset);
		return -ENODEV;
	}

	mmap_len = (u64)(vma->vm_end - vma->vm_start);
	if (!mmap_len) {
		pr_debug("bridge mmap reject: zero length offset=%#llx\n",
			 (unsigned long long)mmap_offset);
		return -EINVAL;
	}

	mutex_lock(&loopback->area_lock);
	ret = virtio_msg_bus_dma_device_helper_lookup_by_mmap_offset
		(&loopback->map_helper, mmap_offset, mmap_len, &map);
	if (ret) {
		pr_debug("bridge mmap reject: offset=%#llx len=%#llx ret=%d\n",
			 (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len, ret);
		mutex_unlock(&loopback->area_lock);
		if (ret == -ERANGE)
			return -ENOENT;
		return ret;
	}

	window = virtio_msg_loopback_page_window_from_map_locked(loopback,
								 &map);
	if (!virtio_msg_loopback_page_window_mmap_allowed_locked
			(loopback, window, &map) ||
	    mmap_len > window->length) {
		pr_debug("bridge mmap reject: offset=%#llx len=%#llx window=%p map_id=%#llx state=%u helper_state=%u kva=%p dma_mmap=%u window_len=%#llx\n",
			 (unsigned long long)mmap_offset,
		       (unsigned long long)mmap_len, window,
		       window ? (unsigned long long)window->map_id : 0,
		       window ? window->state : 0, map.state,
		       window ? window->kva : NULL,
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
			pr_debug("bridge mmap dma failed: offset=%#llx len=%#llx cpu=%p dma=%#llx attrs=%#lx ret=%d\n",
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
		pr_debug("bridge mmap remap failed: offset=%#llx len=%#llx kva=%p area=%p area_flags=%#lx ret=%d\n",
			 (unsigned long long)mmap_offset,
		       (unsigned long long)mmap_len, window->kva, area,
		       area ? area->flags : 0UL, ret);
	}
	virtio_msg_loopback_page_window_put(window);
	return ret;
}

static struct virtio_msg_loopback_exact_export *
virtio_msg_loopback_exact_export_find_window
	(struct virtio_msg_loopback *loopback,
	 struct virtio_msg_loopback_page_window *window)
{
	struct virtio_msg_loopback_exact_export *exact;
	unsigned long flags;
	unsigned int i;

	if (!loopback || !window)
		return NULL;

	spin_lock_irqsave(&loopback->exact_lock, flags);
	list_for_each_entry(exact, &loopback->exact_exports, node) {
		for (i = 0; i < exact->num_unique_windows; i++) {
			if (exact->unique_windows[i] != window)
				continue;

			virtio_msg_loopback_exact_get(exact);
			spin_unlock_irqrestore(&loopback->exact_lock, flags);
			return exact;
		}
	}
	spin_unlock_irqrestore(&loopback->exact_lock, flags);

	return NULL;
}

static int
virtio_msg_loopback_bridge_map_response_prepare
	(const struct vmsg_bridge_uapi_map_event *event, u32 ack_status,
	 u16 type, struct vmsg_bridge_uapi_map_event *response)
{
	if (!event || !response)
		return -EINVAL;
	if (ack_status != VMSG_BRIDGE_UAPI_MAP_ACK_OK &&
	    ack_status != VMSG_BRIDGE_UAPI_MAP_ACK_REJECT)
		return -EINVAL;
	if (event->type != type)
		return -EINVAL;

	*response = *event;
	if (ack_status == VMSG_BRIDGE_UAPI_MAP_ACK_REJECT && !response->status)
		response->status = -EIO;

	return 0;
}

static void
virtio_msg_loopback_bridge_map_result_locked
	(struct virtio_msg_loopback *loopback,
	 const struct virtio_msg_bus_dma_device_map_result *result, int status)
{
	struct virtio_msg_loopback_page_window *window;
	enum virtio_msg_loopback_map_state old_state;

	lockdep_assert_held(&loopback->area_lock);

	if (!result ||
	    result->action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_IGNORED)
		return;

	window = virtio_msg_loopback_page_window_from_map_locked(loopback,
								 &result->map);
	if (!window) {
		pr_debug("rx MAP response ignored: type_action=%u map_id=%#llx mmap_off=%#llx status=%d (no window)\n",
			 result->action,
			       (unsigned long long)result->map.map_id,
			       (unsigned long long)result->map.mmap_offset,
			       status);
		return;
	}

	if (!window->map_id)
		window->map_id = result->map.map_id;
	old_state = window->state;
	if (!status)
		status = -EIO;

	switch (result->action) {
	case VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_ACTIVE:
	case VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_DEL_QUEUED:
		virtio_msg_loopback_page_window_add_ack_ok_locked(loopback,
								  window);
		pr_debug("rx MAP_ADD: map_id=%#llx bus=%#llx len=%#llx old=%u new=%u del_req_queued=%u\n",
			 (unsigned long long)window->map_id,
			 (unsigned long long)window->bus_addr,
			 (unsigned long long)window->length, old_state,
			 window->state, window->del_req_queued);
		break;
	case VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_FAILED:
	case VIRTIO_MSG_BUS_DMA_DEVICE_MAP_REMOVED:
		virtio_msg_loopback_page_window_settle_add_locked
			(loopback, window, status);
		pr_debug("rx MAP removed: map_id=%#llx bus=%#llx len=%#llx status=%d old=%u helper_old=%u\n",
			 (unsigned long long)window->map_id,
			 (unsigned long long)window->bus_addr,
			 (unsigned long long)window->length, status, old_state,
			 result->old_state);
		virtio_msg_loopback_page_window_remove_terminal_locked
			(loopback, window, status);
		virtio_msg_loopback_page_window_put(window);
		break;
	case VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_DONE:
	case VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED_DEL_DONE:
		pr_debug("rx MAP_DEL: map_id=%#llx bus=%#llx len=%#llx old=%u new=%u\n",
			 (unsigned long long)window->map_id,
			 (unsigned long long)window->bus_addr,
			 (unsigned long long)window->length, old_state, 0U);
		virtio_msg_loopback_page_window_remove_terminal_locked
			(loopback, window, 0);
		virtio_msg_loopback_page_window_put(window);
		break;
	default:
		pr_debug("rx MAP response ignored: map_id=%#llx bus=%#llx len=%#llx action=%u state=%u\n",
			 (unsigned long long)window->map_id,
			 (unsigned long long)window->bus_addr,
			 (unsigned long long)window->length, result->action,
			 old_state);
		break;
	}
}

static int
virtio_msg_loopback_bridge_map_add_response
	(struct virtio_msg_loopback *loopback,
	 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status)
{
	struct virtio_msg_bus_dma_device_map_result result;
	struct vmsg_bridge_uapi_map_event response;
	int ret;

	ret = virtio_msg_loopback_bridge_map_response_prepare
		(event, ack_status, VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP,
		 &response);
	if (ret)
		return ret;

	mutex_lock(&loopback->area_lock);
	ret = virtio_msg_bus_dma_device_helper_map_add_response
		(&loopback->map_helper, &response, &result);
	if (!ret)
		virtio_msg_loopback_bridge_map_result_locked(loopback, &result,
							     response.status);
	mutex_unlock(&loopback->area_lock);

	return ret;
}

static int
virtio_msg_loopback_bridge_map_del_response
	(struct virtio_msg_loopback *loopback,
	 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status)
{
	struct virtio_msg_bus_dma_device_map_result result;
	struct vmsg_bridge_uapi_map_event response;
	int ret;

	ret = virtio_msg_loopback_bridge_map_response_prepare
		(event, ack_status, VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP,
		 &response);
	if (ret)
		return ret;

	mutex_lock(&loopback->area_lock);
	ret = virtio_msg_bus_dma_device_helper_map_del_response
		(&loopback->map_helper, &response, &result);
	if (!ret)
		virtio_msg_loopback_bridge_map_result_locked(loopback, &result,
							     response.status);
	mutex_unlock(&loopback->area_lock);

	return ret;
}

static void
virtio_msg_loopback_bridge_set_removed(struct virtio_msg_loopback *loopback)
{
	int ret;

	ret = virtio_msg_bus_bridge_endpoint_set_offline
		(&loopback->bridge.endpoint,
		 VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE_REASON_REMOVED);
	if (ret && ret != -ENODEV && ret != -ENOTCONN)
		pr_warn_ratelimited("endpoint removal escalation failed: ret=%d\n",
				    ret);
}

static void
virtio_msg_loopback_bridge_map_release_forget
	(struct virtio_msg_loopback *loopback,
	 const struct vmsg_bridge_uapi_map_event *event)
{
	u32 handle;
	int ret;

	if (!loopback || !event || !event->map_id)
		return;

	handle = READ_ONCE(loopback->bridge.endpoint.handle);
	if (!handle)
		return;

	ret = virtio_msg_bus_bridge_device_map_del(handle, event->map_id);
	if (ret && ret != -EALREADY && ret != -ENOENT && ret != -ENODEV &&
	    ret != -ENOTCONN)
		pr_warn_ratelimited("released map forget failed: map_id=%#llx ret=%d\n",
				    (unsigned long long)event->map_id, ret);
}

static int
virtio_msg_loopback_bridge_map_released
	(struct virtio_msg_loopback *loopback,
	 const struct vmsg_bridge_uapi_map_event *event)
{
	struct virtio_msg_bus_dma_device_map_result result;
	struct virtio_msg_bus_dma_driver_release release;
	struct virtio_msg_loopback_page_window *window;
	struct virtio_msg_loopback_exact_export *exact;
	struct virtio_msg_loopback_slot *slot = NULL;
	enum virtio_msg_loopback_map_state old_state;
	bool endpoint_remove = false;
	unsigned int owner_count = 0;
	int ret;

	if (!event || event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_RELEASED)
		return -EINVAL;

	mutex_lock(&loopback->area_lock);
	ret = virtio_msg_bus_dma_device_helper_map_released
		(&loopback->map_helper, event, &result);
	if (ret) {
		mutex_unlock(&loopback->area_lock);
		return ret;
	}
	if (result.action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_IGNORED) {
		mutex_unlock(&loopback->area_lock);
		return 0;
	}
	if (result.action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED_DEL_DONE) {
		virtio_msg_loopback_bridge_map_result_locked(loopback, &result,
							     0);
		mutex_unlock(&loopback->area_lock);
		return 0;
	}
	if (result.action != VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED) {
		mutex_unlock(&loopback->area_lock);
		return 0;
	}

	window = virtio_msg_loopback_page_window_from_map_locked(loopback,
								 &result.map);
	if (!window) {
		mutex_unlock(&loopback->area_lock);
		pr_debug("rx MAP_RELEASED ignored: map_id=%#llx mmap_off=%#llx (no window)\n",
			 (unsigned long long)event->map_id,
			 (unsigned long long)event->mmap_offset);
		return 0;
	}

	old_state = window->state;
	owner_count = window->owner_count;
	window->map_id = 0;
	window->del_req_queued = false;
	window->state = VIRTIO_MSG_LOOPBACK_MAP_STATE_REMOTE_RELEASED;
	virtio_msg_loopback_page_window_get(window);
	mutex_unlock(&loopback->area_lock);

	virtio_msg_loopback_bridge_map_release_forget(loopback, event);

	exact = virtio_msg_loopback_exact_export_find_window(loopback, window);
	if (!exact) {
		endpoint_remove = owner_count;
	} else {
		ret = virtio_msg_bus_dma_export_released
			(&exact->provider_export, &release);
		if (ret && ret != -EOPNOTSUPP)
			pr_warn_ratelimited("retained map release handling failed: map_id=%#llx ret=%d\n",
					    (unsigned long long)event->map_id, ret);
		if (ret == -EOPNOTSUPP ||
		    (!ret &&
		     release.action ==
			     VIRTIO_MSG_BUS_DMA_DRIVER_RELEASE_IGNORED)) {
			if (owner_count)
				release.action =
					VIRTIO_MSG_BUS_DMA_DRIVER_RELEASE_RETAINED_ACTIVE;
		}

		if (release.action ==
		    VIRTIO_MSG_BUS_DMA_DRIVER_RELEASE_RETAINED_ACTIVE)
			slot = READ_ONCE(exact->slot);
		virtio_msg_loopback_exact_put(exact);
		endpoint_remove = !slot &&
			release.action ==
				VIRTIO_MSG_BUS_DMA_DRIVER_RELEASE_RETAINED_ACTIVE;
	}

	pr_debug("rx MAP_RELEASED: map_id=%#llx bus=%#llx len=%#llx old=%u owners=%u slot=%u endpoint_remove=%u\n",
		 (unsigned long long)event->map_id,
		 (unsigned long long)window->bus_addr,
		 (unsigned long long)window->length, old_state,
		 owner_count, slot ? slot->dev_num : 0,
		 endpoint_remove);

	if (slot)
		virtio_msg_loopback_slot_stop(loopback, slot->dev_num);
	else if (endpoint_remove)
		virtio_msg_loopback_bridge_set_removed(loopback);

	virtio_msg_loopback_page_window_put(window);
	return 0;
}

static int
virtio_msg_loopback_bridge_map_event_ack
	(struct virtio_msg_bus_bridge_device *endpoint,
	 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status)
{
	struct virtio_msg_loopback *loopback;

	if (!endpoint || !event)
		return -EINVAL;

	loopback = endpoint->priv;
	if (!loopback)
		return -ENODEV;

	switch (event->type) {
	case VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP:
		return virtio_msg_loopback_bridge_map_add_response(loopback,
								   event,
								   ack_status);
	case VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP:
		return virtio_msg_loopback_bridge_map_del_response(loopback,
								   event,
								   ack_status);
	case VMSG_BRIDGE_UAPI_MAP_EVENT_RELEASED:
		if (ack_status != VMSG_BRIDGE_UAPI_MAP_ACK_OK)
			return -EINVAL;
		return virtio_msg_loopback_bridge_map_released(loopback, event);
	default:
		return 0;
	}
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
	u8 type_mask;

	(void)dctx;

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
		return -EINVAL;

	return virtio_msg_loopback_bridge_handle_bridge_request(loopback, msg,
							       msg_size);
}

static int
virtio_msg_loopback_bridge_relay_request
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg, u16 msg_size,
		 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_loopback *loopback;

	if (!endpoint)
		return -EINVAL;

	loopback = endpoint->priv;
	if (!loopback)
		return -ENODEV;

	return virtio_msg_loopback_bridge_handle_transport_request(loopback, msg,
								  msg_size,
								  dctx);
}

static int
virtio_msg_loopback_bridge_relay_event
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg, u16 msg_size,
		 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_loopback *loopback;

	(void)dctx;

	if (!endpoint)
		return -EINVAL;

	loopback = endpoint->priv;
	if (!loopback)
		return -ENODEV;

	if (msg && (msg->msg_id & VIRTIO_MSG_ID_EVENT_BIT))
		pr_debug("bridge event dispatch: msg_id=0x%02x dev=%u\n",
			 msg->msg_id, le16_to_cpu(msg->dev_num));

	return virtio_msg_loopback_bridge_handle_transport_event(loopback, msg,
								msg_size);
}

static int
virtio_msg_loopback_bridge_relay_response
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg, u16 msg_size,
		 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_loopback *loopback;

	if (!endpoint)
		return -EINVAL;

	loopback = endpoint->priv;
	if (!loopback)
		return -ENODEV;

	return virtio_msg_loopback_pending_complete_response(loopback, msg,
							     msg_size, dctx);
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
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_ADDED:
		break;
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED:
		virtio_msg_loopback_slot_stop_async(loopback, dev_num);
		break;
	default:
		break;
	}

	return ret;
}

int virtio_msg_loopback_bridge_slot_can_start(struct virtio_msg_loopback *loopback,
					      u16 dev_num)
{
	struct virtio_msg_bus_bridge_device *endpoint;
	u8 bitmap = 0;
	u16 next_offset;
	u16 out_num;
	int ret;

	if (!loopback || !virtio_msg_loopback_dev_num_valid(dev_num))
		return -EINVAL;

	if (!loopback->bridge.endpoint_registered)
		return -ENOTCONN;

	endpoint = &loopback->bridge.endpoint;
	if (!READ_ONCE(endpoint->handle) ||
	    READ_ONCE(endpoint->rx_unregistered))
		return -ENOTCONN;

	ret = virtio_msg_bus_bridge_topology_get_devices_window
		(endpoint, dev_num, 1, &bitmap, sizeof(bitmap), &out_num,
		 &next_offset);
	if (ret)
		return ret;
	if (!out_num || !(bitmap & BIT(0)))
		return -ENODEV;

	if (!READ_ONCE(endpoint->handle) ||
	    READ_ONCE(endpoint->rx_unregistered))
		return -ENOTCONN;

	return 0;
}

static void
virtio_msg_loopback_bridge_endpoint_online(struct virtio_msg_bus_bridge_device *endpoint)
{
	u8 bitmap[DIV_ROUND_UP(VIRTIO_MSG_LOOPBACK_MAX_DEVS, 8)] = { 0 };
	struct virtio_msg_loopback *loopback;
	u16 next_offset;
	u16 out_num;
	u16 dev_num;
	int ret;

	if (!endpoint)
		return;

	loopback = endpoint->priv;
	if (!loopback)
		return;

	ret = virtio_msg_bus_bridge_topology_get_devices_window
		(endpoint, 0, VIRTIO_MSG_LOOPBACK_MAX_DEVS, bitmap,
		 sizeof(bitmap), &out_num, &next_offset);
	if (ret) {
		pr_warn_ratelimited("loopback topology reconciliation failed: ret=%d\n",
				    ret);
		return;
	}

	for (dev_num = 0; dev_num < VIRTIO_MSG_LOOPBACK_MAX_DEVS; dev_num++) {
		bool present;

		present = dev_num < out_num &&
			  (bitmap[dev_num >> 3] & BIT(dev_num & 0x7));
		if (present)
			schedule_work(&loopback->slots[dev_num].start_work);
		else
			virtio_msg_loopback_slot_stop(loopback, dev_num);
	}
}

static const struct virtio_msg_bus_bridge_device_ops virtio_msg_loopback_ops = {
	.get_caps		= virtio_msg_loopback_bridge_get_caps,
	.tx_msg			= virtio_msg_loopback_bridge_tx_msg,
	.tx_userspace_msg	= virtio_msg_loopback_bridge_tx_msg,
	.relay_request		= virtio_msg_loopback_bridge_relay_request,
	.relay_event		= virtio_msg_loopback_bridge_relay_event,
	.relay_response		= virtio_msg_loopback_bridge_relay_response,
	.report_error		= virtio_msg_loopback_bridge_report_error,
	.mmap			= virtio_msg_loopback_bridge_mmap,
	.map_event_ack		= virtio_msg_loopback_bridge_map_event_ack,
	.notify_device_event	= virtio_msg_loopback_bridge_notify_device_event,
	.endpoint_online	= virtio_msg_loopback_bridge_endpoint_online,
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
		pending->status = -EPROTO;
		xa_erase(&loopback->pending_relay_xa, key);
		mutex_unlock(&loopback->pending_lock);
		complete(&pending->done);
		return -EPROTO;
	}
	relay_seq = pending->relay_seq;
	pending = xa_erase(&loopback->pending_relay_xa, relay_seq);
	if (!pending) {
		mutex_unlock(&loopback->pending_lock);
		return -ENOENT;
	}
	mutex_unlock(&loopback->pending_lock);
	*out_pending = pending;

	return 0;
}

static void
virtio_msg_loopback_pending_complete(struct virtio_msg_loopback_relay_pending *pending)
{
	complete(&pending->done);
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
	if (ret) {
		if (ret == -ENOENT)
			return 0;
		return ret;
	}

	pr_debug("response complete: dev=%u token=%u\n", dev_num, token);

	mutex_lock(&loopback->pending_lock);
	if (pending->timed_out) {
		pending->status = -ETIMEDOUT;
	} else if (msg_size > VIRTIO_MSG_LOOPBACK_MAX_MSG_SIZE) {
		pending->status = -EMSGSIZE;
		ret = -EMSGSIZE;
	} else {
		memcpy(pending->response, msg, msg_size);
		pending->status = 0;
		ret = 0;
	}
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

	pr_debug("error report: dev=%u tok=%u id=0x%02x err=%d\n",
		 dev_num, token, msg_id, error);
	ret = virtio_msg_loopback_pending_take(loopback, dev_num, token, dctx,
					       &pending);
	if (ret) {
		if (ret == -ENOENT)
			return 0;
		return ret;
	}

	mutex_lock(&loopback->pending_lock);
	if (pending->timed_out)
		pending->status = -ETIMEDOUT;
	else
		pending->status = error;
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
		pending->status = status;
		pending->timed_out = status == -ETIMEDOUT;
		mutex_unlock(&loopback->pending_lock);

		complete(&pending->done);
		index = 0;
	}
}

static int
virtio_msg_loopback_validate_addr(const struct virtio_msg_bus_bridge_resolver *resolver,
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
virtio_msg_loopback_match_addr(const struct virtio_msg_bus_bridge_resolver *resolver,
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
	pr_debug("loopback bridge start: bus='%s'\n",
		 VIRTIO_MSG_LOOPBACK_BUS_NAME);

	memset(bridge, 0, sizeof(*bridge));
	strscpy(bridge->bus_id, VIRTIO_MSG_LOOPBACK_BUS_ID,
		sizeof(bridge->bus_id));
	strscpy(bridge->resolver.name, VIRTIO_MSG_LOOPBACK_BUS_NAME,
		sizeof(bridge->resolver.name));
	bridge->resolver.validate = virtio_msg_loopback_validate_addr;
	bridge->resolver.match = virtio_msg_loopback_match_addr;

	loopback->exact_cleanup_wq =
		alloc_ordered_workqueue("virtio_msg_lb_exact_cleanup",
					WQ_MEM_RECLAIM);
	if (!loopback->exact_cleanup_wq)
		return -ENOMEM;

	virtio_msg_bus_bridge_device_init(&bridge->endpoint,
					  &virtio_msg_loopback_ops, loopback);

	ret = virtio_msg_bus_bridge_resolver_register(&bridge->resolver);
	if (ret) {
		destroy_workqueue(loopback->exact_cleanup_wq);
		loopback->exact_cleanup_wq = NULL;
		return ret;
	}
	bridge->resolver_registered = true;

	ret = virtio_msg_bus_bridge_device_register(VIRTIO_MSG_LOOPBACK_BUS_NAME,
						    bridge->bus_id,
						    &bridge->endpoint);
	if (ret) {
		virtio_msg_bus_bridge_resolver_unregister(&bridge->resolver);
		bridge->resolver_registered = false;
		destroy_workqueue(loopback->exact_cleanup_wq);
		loopback->exact_cleanup_wq = NULL;
		return ret;
	}
	bridge->endpoint_registered = true;
	pr_debug("loopback bridge started: bus='%s' endpoint=%u\n",
		 VIRTIO_MSG_LOOPBACK_BUS_NAME, bridge->endpoint.handle);

	return 0;
}

void virtio_msg_loopback_bridge_stop(struct virtio_msg_loopback *loopback)
{
	struct virtio_msg_loopback_bridge *bridge;

	if (!loopback)
		return;

	bridge = &loopback->bridge;
	pr_debug("loopback bridge stop: bus='%s' endpoint=%u\n",
		 VIRTIO_MSG_LOOPBACK_BUS_NAME, bridge->endpoint.handle);
	virtio_msg_loopback_pending_abort_all(loopback, -ESHUTDOWN);
	if (loopback->exact_cleanup_wq)
		drain_workqueue(loopback->exact_cleanup_wq);
	virtio_msg_loopback_map_cleanup_all(loopback);
	if (loopback->exact_cleanup_wq) {
		destroy_workqueue(loopback->exact_cleanup_wq);
		loopback->exact_cleanup_wq = NULL;
	}

	if (bridge->endpoint_registered) {
		virtio_msg_bus_bridge_device_unregister(VIRTIO_MSG_LOOPBACK_BUS_NAME,
							bridge->bus_id,
							&bridge->endpoint);
		bridge->endpoint_registered = false;
	}

	if (bridge->resolver_registered) {
		virtio_msg_bus_bridge_resolver_unregister(&bridge->resolver);
		bridge->resolver_registered = false;
	}
}
