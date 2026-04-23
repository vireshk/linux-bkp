// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus DMA helper (Step 1 shell).
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-direct.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/iommu-dma.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/preempt.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/virtio_config.h>
#include <linux/virtio_msg_transport.h>

#define VM_LOG_COMPONENT VM_LOG_COMPONENT_DMA
#include "virtio_msg_debug.h"
#include "virtio_msg_bus_dma.h"

/*
 * Pool chunk size in MiB.  Each pre-registered chunk is exported once to the
 * VMM during endpoint init, after which sub-allocations carved from the same
 * chunk reuse already-ACTIVE page windows without producing additional
 * MAP_EVENT_ADD round-trips.  Set to 0 to disable the pool.
 */
static unsigned long virtio_msg_dma_pool_chunk_size_mb = 2;
module_param(virtio_msg_dma_pool_chunk_size_mb, ulong, 0444);
MODULE_PARM_DESC(virtio_msg_dma_pool_chunk_size_mb,
		 "DMA pool chunk size in MiB (0 = disabled, default: 2)");

/*
 * Per-mapping size ceiling for pool shadow allocations in KiB.  Mappings
 * larger than this bypass the pool and use a dedicated per-mapping
 * dma_alloc_attrs, which avoids consuming large chunks of shared pool space for one-shot
 * transfers (e.g. large virtio-blk requests).  Set to 0 to always use the
 * pool up to the chunk size limit.
 */
static unsigned long virtio_msg_dma_pool_max_alloc_kb = 64;
module_param(virtio_msg_dma_pool_max_alloc_kb, ulong, 0644);
MODULE_PARM_DESC(virtio_msg_dma_pool_max_alloc_kb,
		 "Max per-mapping size (KiB) eligible for DMA pool (0 = no limit, default: 64)");

#define VIRTIO_MSG_BUS_DMA_DEFERRED_RETRY_DELAY_MS	10U
#define VIRTIO_MSG_BUS_DMA_CLEANUP_WAIT_TIMEOUT_MS	5000U

/* Forward declarations needed by virtio_msg_bus_dma_shadow. */
struct virtio_msg_bus_dma;
struct virtio_msg_bus_dma_pool_chunk;

struct virtio_msg_bus_dma_key {
	dma_addr_t dma_addr;
	size_t length;
};

enum virtio_msg_bus_dma_mapping_kind {
	VIRTIO_MSG_BUS_DMA_MAPPING_STREAMING = 0,
	VIRTIO_MSG_BUS_DMA_MAPPING_COHERENT,
};

enum virtio_msg_bus_dma_backing_mode {
	VIRTIO_MSG_BUS_DMA_BACKING_DIRECT = 0,
	VIRTIO_MSG_BUS_DMA_BACKING_SHADOW,
};

struct virtio_msg_bus_dma_shadow {
	struct page **src_pages;
	size_t src_page_offset;
	unsigned int src_npages;
	struct device *dev;
	void *shadow_cpu_addr;
	dma_addr_t shadow_dma_addr;
	size_t shadow_length;
	unsigned long shadow_attrs;
	bool copy_to_shadow;
	bool copy_from_shadow;
	/* Pool shadow fields: set when shadow memory comes from the DMA pool. */
	bool from_pool;
	struct virtio_msg_bus_dma *pool_dma;
	struct virtio_msg_bus_dma_pool_chunk *pool_chunk;
	unsigned int pool_page_offset;
	unsigned int pool_npages;
};

struct virtio_msg_bus_dma_mapping {
	struct list_head deferred_node;
	struct virtio_msg_bus_dma_key key;
	enum virtio_msg_bus_dma_mapping_kind kind;
	enum virtio_msg_bus_dma_backing_mode backing_mode;
	struct virtio_msg_bus_dma_export backing;
	struct virtio_msg_bus_dma_shadow shadow;
	struct sg_table backing_sgt;
	bool backing_sgt_valid;
	void *cpu_addr;
	unsigned long attrs;
	void *provider_handle;
};

struct virtio_msg_bus_dma {
	struct device *delegate_dev;
	const struct virtio_map_ops *saved_virtio_map_ops;
	union virtio_map saved_vmap;
	const struct dma_map_ops *saved_dma_ops;
	u64 *saved_dma_mask_ptr;
	u64 dma_mask_storage;
	u64 saved_coherent_dma_mask;
	u64 saved_bus_dma_limit;
	const struct bus_dma_region *saved_dma_range_map;
	struct device_dma_parameters dma_parms_storage;
	struct device_dma_parameters *saved_dma_parms_ptr;
#if defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_DEVICE) || \
	defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU) || \
	defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU_ALL)
	bool saved_dma_coherent;
#endif
#ifdef CONFIG_DMA_OPS_BYPASS
	bool saved_dma_ops_bypass;
#endif
#ifdef CONFIG_DMA_NEED_SYNC
	bool saved_dma_skip_sync;
#endif
#ifdef CONFIG_IOMMU_DMA
	bool saved_dma_iommu;
#endif
	struct xarray active_exports;
	spinlock_t active_exports_lock; /* Protects active exports and frees. */
	struct list_head deferred_local_frees;
	struct delayed_work deferred_local_free_work;
	const struct virtio_msg_bus_dma_provider_ops *provider_ops;
	void *provider_ctx;
	struct virtio_msg_bus_dma_provider_caps provider_caps;
	/* DMA pool: pre-registered shadow-buffer chunks. */
	struct list_head pool_chunks;   /* list of virtio_msg_bus_dma_pool_chunk */
	spinlock_t pool_lock;           /* protects pool_chunks and each bitmap */
	size_t pool_chunk_size;         /* length of each chunk (bytes) */
	struct work_struct pool_grow_work; /* deferred chunk alloc from atomic ctx */
	/*
	 * High-water mark of page counts that failed pool sub-alloc in atomic
	 * context because no existing chunk was large enough.  pool_grow_work
	 * reads this to size the new chunk appropriately and then resets it.
	 * Updated with cmpxchg so the maximum across concurrent callers wins.
	 */
	atomic_t pool_grow_min_pages;
	bool shim_installed;
};

/*
 * A single pre-registered DMA pool chunk.  The chunk is allocated via
 * dma_alloc_attrs() and exported to the VMM once during install.  Subsequent
 * shadow sub-allocations carved out of this chunk reuse the already-ACTIVE
 * page windows without generating further MAP_EVENT_ADD round-trips.
 */
struct virtio_msg_bus_dma_pool_chunk {
	struct list_head node;
	void *cpu_addr;                 /* chunk base CPU-virtual address */
	dma_addr_t dma_addr;            /* chunk base DMA address */
	size_t length;                  /* total chunk size in bytes */
	unsigned int npages;            /* total pages in chunk */
	unsigned long *bitmap;          /* set bit = page in use by sub-alloc */
	unsigned int free_pages;        /* pages available for sub-alloc */
	struct page **pages;            /* page pointer array [0..npages-1] */
	struct virtio_msg_bus_dma_export export;
	void *provider_handle;
};

static LIST_HEAD(virtio_msg_bus_dma_orphaned_local_frees);
static DEFINE_SPINLOCK(virtio_msg_bus_dma_orphaned_local_frees_lock);

static inline struct virtio_msg_transport_device *
virtio_msg_bus_dma_vmdev_from_vdev(struct virtio_device *vdev)
{
	return container_of(vdev, struct virtio_msg_transport_device, vdev);
}

static struct virtio_msg_bus_dma *
virtio_msg_bus_dma_from_dev(struct device *dev)
{
	struct virtio_msg_transport_device *vmdev;

	if (!dev || !is_virtio_device(dev))
		return NULL;

	vmdev = virtio_msg_bus_dma_vmdev_from_vdev(dev_to_virtio(dev));

	return vmdev ? vmdev->dma_shim : NULL;
}

bool virtio_msg_bus_dma_is_installed(const struct virtio_msg_transport_device *vmdev)
{
	return vmdev && vmdev->dma_shim && vmdev->dma_shim->shim_installed;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_is_installed);

struct device *virtio_msg_bus_dma_delegate_dev(const struct virtio_msg_transport_device *vmdev)
{
	if (!virtio_msg_bus_dma_is_installed(vmdev))
		return NULL;

	return vmdev->dma_shim->delegate_dev;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_delegate_dev);

/* ------------------------------------------------------------------ */
/* DMA pool: sub-allocator over pre-registered shadow-buffer chunks.  */
/* ------------------------------------------------------------------ */

struct virtio_msg_bus_dma_sg_extract {
	size_t page_offset;
	size_t mmap_length;
	unsigned int npages;
	struct page **pages;
	struct sg_table cloned_sgt;
	bool cloned_sgt_valid;
};

/* Forward declarations for helpers defined later in this file. */
static void virtio_msg_bus_dma_pool_grow_work(struct work_struct *work);
static void virtio_msg_bus_dma_sg_extract_release
		(struct virtio_msg_bus_dma_sg_extract *extract);
static int virtio_msg_bus_dma_extract_sg_data
		(struct sg_table *sgt, size_t length, gfp_t gfp, bool clone_sgt,
		 struct virtio_msg_bus_dma_sg_extract *extract);
static struct virtio_msg_bus_dma_mapping *
virtio_msg_bus_dma_alloc_sgt_record(struct sg_table *sgt, size_t length,
				    dma_addr_t dma_addr,
				    enum dma_data_direction dir, bool coherent,
				    struct device *dma_dev, void *cpu_addr,
				    unsigned long attrs,
				    gfp_t gfp,
				    enum virtio_msg_bus_dma_mapping_kind kind);
static void virtio_msg_bus_dma_free_record(struct virtio_msg_bus_dma_mapping *m);
static bool virtio_msg_bus_dma_mapping_is_shadow
		(const struct virtio_msg_bus_dma_mapping *mapping);
static bool virtio_msg_bus_dma_copy_to_shadow_needed(enum dma_data_direction dir);
static bool virtio_msg_bus_dma_copy_from_shadow_needed(enum dma_data_direction dir);
static void virtio_msg_bus_dma_mapping_release_sgt(struct virtio_msg_bus_dma_mapping *m);
static int virtio_msg_bus_dma_provider_export_del_record
		(struct virtio_msg_bus_dma *dma,
		 struct virtio_msg_bus_dma_mapping *mapping, bool sync);
static void virtio_msg_bus_dma_shadow_sync_for_device(struct virtio_msg_bus_dma *dma,
						      struct virtio_msg_bus_dma_mapping *mapping,
						      dma_addr_t dma_handle,
						      size_t size,
						      enum dma_data_direction dir);

static int
virtio_msg_bus_dma_pool_sub_alloc(struct virtio_msg_bus_dma *dma,
				  unsigned int npages,
				  struct virtio_msg_bus_dma_pool_chunk **out_chunk,
				  unsigned int *out_page_offset)
{
	struct virtio_msg_bus_dma_pool_chunk *chunk;
	unsigned long start;
	unsigned long flags;

	spin_lock_irqsave(&dma->pool_lock, flags);
	list_for_each_entry(chunk, &dma->pool_chunks, node) {
		if (chunk->free_pages < npages)
			continue;
		start = bitmap_find_next_zero_area(chunk->bitmap, chunk->npages,
						   0, npages, 0);
		if (start >= chunk->npages)
			continue;
		bitmap_set(chunk->bitmap, start, npages);
		chunk->free_pages -= npages;
		spin_unlock_irqrestore(&dma->pool_lock, flags);
		*out_chunk = chunk;
		*out_page_offset = (unsigned int)start;
		return 0;
	}
	spin_unlock_irqrestore(&dma->pool_lock, flags);
	return -ENOMEM;
}

static void
virtio_msg_bus_dma_pool_sub_free(struct virtio_msg_bus_dma *dma,
				 struct virtio_msg_bus_dma_pool_chunk *chunk,
				 unsigned int page_offset, unsigned int npages)
{
	unsigned long flags;

	spin_lock_irqsave(&dma->pool_lock, flags);
	bitmap_clear(chunk->bitmap, page_offset, npages);
	chunk->free_pages += npages;
	spin_unlock_irqrestore(&dma->pool_lock, flags);
}

static struct virtio_msg_bus_dma_pool_chunk *
virtio_msg_bus_dma_pool_chunk_alloc(struct virtio_msg_bus_dma *dma, gfp_t gfp)
{
	struct virtio_msg_bus_dma_pool_chunk *chunk;
	struct virtio_msg_bus_dma_sg_extract extract;
	struct sg_table sgt;
	size_t chunk_size = dma->pool_chunk_size;
	unsigned int npages;
	int ret;

	if (!chunk_size || !PAGE_ALIGNED(chunk_size))
		return ERR_PTR(-EINVAL);

	npages = chunk_size >> PAGE_SHIFT;

	chunk = kzalloc(sizeof(*chunk), gfp);
	if (!chunk)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&chunk->node);
	chunk->length = chunk_size;

	chunk->bitmap = bitmap_zalloc(npages, gfp);
	if (!chunk->bitmap) {
		ret = -ENOMEM;
		goto err_free_chunk;
	}

	chunk->cpu_addr = dma_alloc_attrs(dma->delegate_dev, chunk_size,
					  &chunk->dma_addr, gfp, 0);
	if (!chunk->cpu_addr) {
		ret = -ENOMEM;
		goto err_free_bitmap;
	}

	memset(&sgt, 0, sizeof(sgt));
	ret = dma_get_sgtable_attrs(dma->delegate_dev, &sgt, chunk->cpu_addr,
				    chunk->dma_addr, chunk_size, 0);
	if (ret)
		goto err_free_dma;

	ret = virtio_msg_bus_dma_extract_sg_data(&sgt, chunk_size, gfp, false,
						 &extract);
	sg_free_table(&sgt);
	if (ret)
		goto err_free_dma;

	chunk->pages = extract.pages;
	chunk->npages = extract.npages;
	chunk->free_pages = chunk->npages;
	extract.pages = NULL;
	virtio_msg_bus_dma_sg_extract_release(&extract);

	if (chunk->npages != npages) {
		ret = -EINVAL;
		goto err_free_pages;
	}

	/* Export the full chunk to the VMM (sleepable context). */
	memset(&chunk->export, 0, sizeof(chunk->export));
	chunk->export.dma_addr = chunk->dma_addr;
	chunk->export.local_dma_addr = chunk->dma_addr;
	chunk->export.length = chunk_size;
	chunk->export.mmap_length = chunk_size;
	chunk->export.page_offset = 0;
	chunk->export.npages = chunk->npages;
	chunk->export.pages = chunk->pages;
	chunk->export.dma_dev = dma->delegate_dev;
	chunk->export.cpu_addr = chunk->cpu_addr;
	chunk->export.attrs = 0;
	chunk->export.dir = DMA_BIDIRECTIONAL;
	chunk->export.coherent = true;

	ret = dma->provider_ops->export_add(dma->provider_ctx, &chunk->export,
					    &chunk->provider_handle, gfp);
	if (ret)
		goto err_free_pages;
	if (chunk->export.length != chunk_size ||
	    chunk->export.local_dma_addr != chunk->dma_addr) {
		ret = -EOPNOTSUPP;
		goto err_revoke_chunk_export;
	}

	return chunk;

err_revoke_chunk_export:
	(void)dma->provider_ops->export_del(dma->provider_ctx,
					    &chunk->export,
					    chunk->provider_handle, false);
	chunk->provider_handle = NULL;
err_free_pages:
	kfree(chunk->pages);
	chunk->pages = NULL;
err_free_dma:
	dma_free_attrs(dma->delegate_dev, chunk_size, chunk->cpu_addr,
		       chunk->dma_addr, 0);
err_free_bitmap:
	bitmap_free(chunk->bitmap);
err_free_chunk:
	kfree(chunk);
	return ERR_PTR(ret);
}

static void
virtio_msg_bus_dma_pool_chunk_free(struct virtio_msg_bus_dma *dma,
				   struct virtio_msg_bus_dma_pool_chunk *chunk)
{
	if (!chunk)
		return;

	if (chunk->provider_handle) {
		(void)dma->provider_ops->export_del(dma->provider_ctx,
						    &chunk->export,
						    chunk->provider_handle,
						    true);
		chunk->provider_handle = NULL;
	}

	dma_free_attrs(dma->delegate_dev, chunk->length, chunk->cpu_addr,
		       chunk->dma_addr, 0);
	kfree(chunk->pages);
	bitmap_free(chunk->bitmap);
	kfree(chunk);
}

/*
 * Work function: allocate and register a new pool chunk from process context.
 * Scheduled from atomic DMA map paths when the pool is exhausted, so that the
 * next retry finds a fresh, already-ACTIVE chunk without triggering a new
 * MAP_EVENT_ADD round-trip itself.
 */
static void virtio_msg_bus_dma_pool_grow_work(struct work_struct *work)
{
	struct virtio_msg_bus_dma *dma =
		container_of(work, struct virtio_msg_bus_dma, pool_grow_work);
	struct virtio_msg_bus_dma_pool_chunk *new_chunk;
	unsigned int min_pages;
	size_t chunk_size;

	if (!dma->pool_chunk_size)
		return;

	/*
	 * Honour the size hint set by prepare_shadow_from_pool when an atomic
	 * caller needed more pages than any existing chunk could serve.  Round
	 * the required bytes up to the nearest multiple of pool_chunk_size so
	 * all future standard-sized allocs share the new chunk too.
	 */
	min_pages = (unsigned int)atomic_xchg(&dma->pool_grow_min_pages, 0);
	if (min_pages) {
		size_t min_size = (size_t)min_pages << PAGE_SHIFT;

		chunk_size = roundup(min_size, dma->pool_chunk_size);
	} else {
		chunk_size = dma->pool_chunk_size;
	}

	/*
	 * Temporarily widen pool_chunk_size for the allocation if needed, and
	 * restore it afterwards so all later grow calls still use the default.
	 * A permanent bump would waste memory on devices that only occasionally
	 * see oversized requests and would affect all future chunks.
	 */
	if (chunk_size != dma->pool_chunk_size) {
		size_t saved = dma->pool_chunk_size;

		dma->pool_chunk_size = chunk_size;
		new_chunk = virtio_msg_bus_dma_pool_chunk_alloc(dma, GFP_KERNEL);
		dma->pool_chunk_size = saved;
	} else {
		new_chunk = virtio_msg_bus_dma_pool_chunk_alloc(dma, GFP_KERNEL);
	}

	if (IS_ERR(new_chunk)) {
		vm_warn_rl(dma->delegate_dev,
			   "dma-pool: deferred grow failed (err %ld)\n",
			   PTR_ERR(new_chunk));
		return;
	}

	vm_trace(dma->delegate_dev,
		 "dma-pool: grew chunk size=%zu (min_pages=%u)",
		 chunk_size, min_pages);

	spin_lock(&dma->pool_lock);
	list_add_tail(&new_chunk->node, &dma->pool_chunks);
	spin_unlock(&dma->pool_lock);
}

static int
virtio_msg_bus_dma_pool_init(struct virtio_msg_bus_dma *dma)
{
	struct virtio_msg_bus_dma_pool_chunk *chunk;
	size_t chunk_size;

	chunk_size = (size_t)virtio_msg_dma_pool_chunk_size_mb << 20;
	if (!chunk_size)
		return 0;
	if (!PAGE_ALIGNED(chunk_size))
		chunk_size = PAGE_ALIGN(chunk_size);

	dma->pool_chunk_size = chunk_size;

	chunk = virtio_msg_bus_dma_pool_chunk_alloc(dma, GFP_KERNEL);
	if (IS_ERR(chunk)) {
		vm_warn_rl(dma->delegate_dev,
			   "dma-pool: failed to allocate initial %zu-byte chunk (err %ld), pool disabled\n",
			   chunk_size, PTR_ERR(chunk));
		dma->pool_chunk_size = 0;
		return 0;   /* not fatal — fall back to per-mapping alloc */
	}

	spin_lock(&dma->pool_lock);
	list_add_tail(&chunk->node, &dma->pool_chunks);
	spin_unlock(&dma->pool_lock);

	return 0;
}

static void
virtio_msg_bus_dma_pool_cleanup(struct virtio_msg_bus_dma *dma)
{
	struct virtio_msg_bus_dma_pool_chunk *chunk, *tmp;
	unsigned long flags;
	LIST_HEAD(to_free);

	/* Ensure no in-flight grow work races with the chunk teardown. */
	cancel_work_sync(&dma->pool_grow_work);

	spin_lock_irqsave(&dma->pool_lock, flags);
	list_splice_init(&dma->pool_chunks, &to_free);
	spin_unlock_irqrestore(&dma->pool_lock, flags);

	list_for_each_entry_safe(chunk, tmp, &to_free, node) {
		list_del_init(&chunk->node);
		virtio_msg_bus_dma_pool_chunk_free(dma, chunk);
	}
}

static bool
virtio_msg_bus_dma_mapping_needs_deferred_release
	(const struct virtio_msg_bus_dma_mapping *mapping)
{
	if (!mapping)
		return false;

	if (mapping->kind == VIRTIO_MSG_BUS_DMA_MAPPING_COHERENT)
		return true;

	return virtio_msg_bus_dma_mapping_is_shadow(mapping) &&
	       !mapping->shadow.from_pool;
}

static void
virtio_msg_bus_dma_release_mapping_local
	(struct virtio_msg_bus_dma *dma, struct virtio_msg_bus_dma_mapping *mapping)
{
	if (!dma || !mapping)
		return;

	if (mapping->kind == VIRTIO_MSG_BUS_DMA_MAPPING_COHERENT)
		dma_free_attrs(dma->delegate_dev, mapping->backing.length,
			       mapping->cpu_addr,
			       mapping->backing.local_dma_addr,
			       mapping->attrs);

	virtio_msg_bus_dma_free_record(mapping);
}

static bool
virtio_msg_bus_dma_revoke_and_release_mapping
	(struct virtio_msg_bus_dma *dma, struct virtio_msg_bus_dma_mapping *mapping,
	 bool retry_inprogress)
{
	int ret;

	ret = virtio_msg_bus_dma_provider_export_del_record(dma, mapping, true);
	if (ret == -EINPROGRESS && retry_inprogress)
		return false;
	if (ret && ret != -EOPNOTSUPP)
		vm_warn_rl(dma->delegate_dev,
			   "provider sync revoke failed dma=%pad len=%zu ret=%d\n",
			   &mapping->backing.dma_addr,
			   mapping->backing.length, ret);

	virtio_msg_bus_dma_release_mapping_local(dma, mapping);
	return true;
}

static void
virtio_msg_bus_dma_drain_local_frees_once
	(struct virtio_msg_bus_dma *dma,
	 struct list_head *pending,
	 struct list_head *retry)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	struct virtio_msg_bus_dma_mapping *tmp;

	if (!dma || !pending || !retry)
		return;

	list_for_each_entry_safe(mapping, tmp, pending, deferred_node) {
		list_del_init(&mapping->deferred_node);
		if (!virtio_msg_bus_dma_revoke_and_release_mapping(dma, mapping,
								   true))
			list_add_tail(&mapping->deferred_node, retry);
	}
}

static void
virtio_msg_bus_dma_deferred_local_free_workfn(struct work_struct *work)
{
	struct virtio_msg_bus_dma *dma =
		container_of(to_delayed_work(work), struct virtio_msg_bus_dma,
			     deferred_local_free_work);
	unsigned long flags;
	LIST_HEAD(local);
	LIST_HEAD(retry);

	spin_lock_irqsave(&dma->active_exports_lock, flags);
	list_splice_init(&dma->deferred_local_frees, &local);
	spin_unlock_irqrestore(&dma->active_exports_lock, flags);

	virtio_msg_bus_dma_drain_local_frees_once(dma, &local, &retry);

	if (list_empty(&retry))
		return;

	spin_lock_irqsave(&dma->active_exports_lock, flags);
	list_splice_tail_init(&retry, &dma->deferred_local_frees);
	spin_unlock_irqrestore(&dma->active_exports_lock, flags);

	(void)mod_delayed_work(system_wq, &dma->deferred_local_free_work,
			       msecs_to_jiffies
				(VIRTIO_MSG_BUS_DMA_DEFERRED_RETRY_DELAY_MS));
}

static void
virtio_msg_bus_dma_cleanup_drain_local_frees
	(struct virtio_msg_bus_dma *dma,
	 struct list_head *pending)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	struct virtio_msg_bus_dma_mapping *tmp;
	unsigned long deadline;
	unsigned long flags;
	unsigned int retained = 0;
	LIST_HEAD(retry);

	if (!dma || !pending)
		return;

	deadline = jiffies +
		msecs_to_jiffies(VIRTIO_MSG_BUS_DMA_CLEANUP_WAIT_TIMEOUT_MS);

	while (!list_empty(pending)) {
		virtio_msg_bus_dma_drain_local_frees_once(dma, pending, &retry);

		if (list_empty(&retry))
			return;

		if (time_after_eq(jiffies, deadline))
			break;

		msleep(VIRTIO_MSG_BUS_DMA_DEFERRED_RETRY_DELAY_MS);
		list_splice_tail_init(&retry, pending);
	}

	spin_lock_irqsave(&virtio_msg_bus_dma_orphaned_local_frees_lock, flags);
	list_for_each_entry_safe(mapping, tmp, &retry, deferred_node) {
		list_del_init(&mapping->deferred_node);
		list_add_tail(&mapping->deferred_node,
			      &virtio_msg_bus_dma_orphaned_local_frees);
		retained++;
	}
	spin_unlock_irqrestore(&virtio_msg_bus_dma_orphaned_local_frees_lock,
			       flags);

	if (retained)
		vm_warn_rl(dma->delegate_dev,
			   "cleanup retaining %u DMA mappings with revoke in progress\n",
			   retained);
}

/*
 * Schedule a deferred pool grow from atomic context, optionally recording
 * a minimum page count that the new chunk must satisfy.  Always returns
 * -EAGAIN so callers can propagate DMA_MAPPING_ERROR without sleeping.
 */
static int
virtio_msg_bus_dma_pool_trigger_grow(struct virtio_msg_bus_dma *dma,
				     unsigned int min_pages)
{
	if (min_pages) {
		int old, cur = atomic_read(&dma->pool_grow_min_pages);

		do {
			old = cur;
			if ((unsigned int)old >= min_pages)
				break;
			cur = atomic_cmpxchg(&dma->pool_grow_min_pages,
					     old, (int)min_pages);
		} while (cur != old);
	}
	schedule_work(&dma->pool_grow_work);
	return -EAGAIN;
}

static int
virtio_msg_bus_dma_prepare_shadow_from_pool(struct virtio_msg_bus_dma *dma,
					    struct virtio_msg_bus_dma_mapping *mapping,
					    gfp_t gfp)
{
	struct virtio_msg_bus_dma_pool_chunk *chunk;
	unsigned int page_offset;
	unsigned int npages;
	struct page **new_pages;
	void *shadow_cpu_addr;
	dma_addr_t shadow_dma_addr;
	dma_addr_t export_dma_addr;
	size_t export_offset;
	unsigned int i;
	int ret;

	npages = DIV_ROUND_UP(mapping->backing.length, PAGE_SIZE);
	if (!npages)
		return -ENOMEM;

	/*
	 * In sleepable contexts, skip the pool for allocs that exceed the
	 * per-mapping ceiling to avoid fragmenting shared pool space for
	 * one-shot transfers.  In atomic contexts (e.g. virtio-blk queue_rq
	 * holding a spinlock) the ceiling is waived: dma_alloc_attrs and
	 * dma_get_sgtable_attrs cannot be called without sleeping, so the
	 * pool is the only viable shadow-allocation path.
	 */
	if (virtio_msg_dma_pool_max_alloc_kb &&
	    gfpflags_allow_blocking(gfp) &&
	    mapping->backing.length > (size_t)virtio_msg_dma_pool_max_alloc_kb << 10)
		return -ENOMEM;

	/*
	 * Reject anything that would not fit in any existing chunk.
	 * In sleepable context the caller will fall back to a per-mapping
	 * dma_alloc_attrs, which is fine.  In atomic context that fallback
	 * cannot sleep, so record the minimum chunk size needed and schedule
	 * a grow so the next blk-mq retry finds a large enough chunk ready.
	 * Return -EAGAIN (not -ENOMEM) so the caller propagates it upward
	 * and triggers a DMA_MAPPING_ERROR without touching the sleeping path.
	 */
	if (npages > (dma->pool_chunk_size >> PAGE_SHIFT)) {
		if (!gfpflags_allow_blocking(gfp))
			return virtio_msg_bus_dma_pool_trigger_grow(dma, npages);
		return -ENOMEM;
	}

	ret = virtio_msg_bus_dma_pool_sub_alloc(dma, npages, &chunk,
						&page_offset);
	if (ret && gfpflags_allow_blocking(gfp)) {
		/*
		 * All existing chunks are full.  Try to grow the pool by
		 * registering a new chunk with the VMM (sleepable context
		 * only — chunk_alloc waits for MAP_EVENT_ADD ACK_OK).
		 */
		struct virtio_msg_bus_dma_pool_chunk *new_chunk;

		new_chunk = virtio_msg_bus_dma_pool_chunk_alloc(dma, gfp);
		if (!IS_ERR(new_chunk)) {
			spin_lock(&dma->pool_lock);
			list_add_tail(&new_chunk->node, &dma->pool_chunks);
			spin_unlock(&dma->pool_lock);
			ret = virtio_msg_bus_dma_pool_sub_alloc(dma, npages,
								&chunk,
								&page_offset);
		}
	} else if (ret) {
		return virtio_msg_bus_dma_pool_trigger_grow(dma, 0);
	}
	if (ret)
		return ret;   /* pool exhausted and grow failed — fall back */

	shadow_cpu_addr = (u8 *)chunk->cpu_addr +
			  (size_t)page_offset * PAGE_SIZE;
	shadow_dma_addr = chunk->dma_addr +
			  (dma_addr_t)page_offset * PAGE_SIZE;
	export_offset = (size_t)page_offset << PAGE_SHIFT;
	if (check_add_overflow(chunk->export.dma_addr,
			       (dma_addr_t)export_offset, &export_dma_addr)) {
		virtio_msg_bus_dma_pool_sub_free(dma, chunk, page_offset, npages);
		return -EOVERFLOW;
	}

	new_pages = kcalloc(npages, sizeof(*new_pages), gfp);
	if (!new_pages) {
		virtio_msg_bus_dma_pool_sub_free(dma, chunk, page_offset, npages);
		return -ENOMEM;
	}
	for (i = 0; i < npages; i++)
		new_pages[i] = chunk->pages[page_offset + i];

	/* Save original page metadata as copy source. */
	mapping->shadow.src_pages = mapping->backing.pages;
	mapping->shadow.src_page_offset = mapping->backing.page_offset;
	mapping->shadow.src_npages = mapping->backing.npages;
	mapping->shadow.dev = dma->delegate_dev;
	mapping->shadow.shadow_cpu_addr = shadow_cpu_addr;
	mapping->shadow.shadow_dma_addr = shadow_dma_addr;
	mapping->shadow.shadow_length = (size_t)npages * PAGE_SIZE;
	mapping->shadow.shadow_attrs = 0;
	mapping->shadow.copy_to_shadow =
		virtio_msg_bus_dma_copy_to_shadow_needed(mapping->backing.dir);
	mapping->shadow.copy_from_shadow =
		virtio_msg_bus_dma_copy_from_shadow_needed(mapping->backing.dir);
	mapping->shadow.from_pool = true;
	mapping->shadow.pool_dma = dma;
	mapping->shadow.pool_chunk = chunk;
	mapping->shadow.pool_page_offset = page_offset;
	mapping->shadow.pool_npages = npages;

	/* Swap mapping->backing to the pool shadow pages. */
	virtio_msg_bus_dma_mapping_release_sgt(mapping);
	/* mapping->backing.pages (original) is now owned by shadow.src_pages */
	mapping->backing.pages = new_pages;
	mapping->backing.dma_addr = export_dma_addr;
	mapping->backing.local_dma_addr = shadow_dma_addr;
	mapping->backing.mmap_length = (size_t)npages * PAGE_SIZE;
	mapping->backing.page_offset = 0;
	mapping->backing.npages = npages;
	mapping->backing.dma_dev = dma->delegate_dev;
	mapping->backing.cpu_addr = shadow_cpu_addr;
	mapping->backing.attrs = 0;
	mapping->backing.coherent = false;
	mapping->key.dma_addr = export_dma_addr;
	mapping->backing_mode = VIRTIO_MSG_BUS_DMA_BACKING_SHADOW;

	virtio_msg_bus_dma_shadow_sync_for_device(dma, mapping,
						  mapping->key.dma_addr,
						  mapping->key.length,
						  mapping->backing.dir);
	return 0;
}

static void
virtio_msg_bus_dma_shadow_release
	(struct virtio_msg_bus_dma_shadow *shadow)
{
	if (!shadow)
		return;

	kfree(shadow->src_pages);
	shadow->src_pages = NULL;
	shadow->src_npages = 0;
	shadow->src_page_offset = 0;

	if (shadow->from_pool) {
		/*
		 * Pool shadow: return the sub-allocation to the pool bitmap.
		 * dma_free_attrs() must NOT be called — the backing memory
		 * belongs to the pool chunk and stays alive until pool teardown.
		 */
		if (shadow->pool_dma && shadow->pool_chunk)
			virtio_msg_bus_dma_pool_sub_free(shadow->pool_dma,
							 shadow->pool_chunk,
							 shadow->pool_page_offset,
							 shadow->pool_npages);
		shadow->pool_dma = NULL;
		shadow->pool_chunk = NULL;
		shadow->shadow_cpu_addr = NULL;
		shadow->shadow_dma_addr = 0;
		shadow->shadow_length = 0;
		return;
	}

	if (!shadow->dev || !shadow->shadow_cpu_addr || !shadow->shadow_length)
		return;

	dma_free_attrs(shadow->dev, shadow->shadow_length, shadow->shadow_cpu_addr,
		       shadow->shadow_dma_addr, shadow->shadow_attrs);
	shadow->shadow_cpu_addr = NULL;
	shadow->shadow_dma_addr = 0;
	shadow->shadow_length = 0;
}

static bool
virtio_msg_bus_dma_mapping_is_shadow
	(const struct virtio_msg_bus_dma_mapping *mapping)
{
	return mapping &&
	       mapping->backing_mode == VIRTIO_MSG_BUS_DMA_BACKING_SHADOW;
}

static void
virtio_msg_bus_dma_mapping_release_sgt
	(struct virtio_msg_bus_dma_mapping *mapping)
{
	if (!mapping || !mapping->backing_sgt_valid)
		return;

	sg_free_table(&mapping->backing_sgt);
	memset(&mapping->backing_sgt, 0, sizeof(mapping->backing_sgt));
	mapping->backing_sgt_valid = false;
	mapping->backing.sgt = NULL;
}

static void
virtio_msg_bus_dma_free_record
	(struct virtio_msg_bus_dma_mapping *mapping)
{
	if (!mapping)
		return;

	if (virtio_msg_bus_dma_mapping_is_shadow(mapping))
		virtio_msg_bus_dma_shadow_release(&mapping->shadow);
	virtio_msg_bus_dma_mapping_release_sgt(mapping);
	kfree(mapping->backing.pages);
	kfree(mapping);
}

static int
virtio_msg_bus_dma_provider_export_del_record
		(struct virtio_msg_bus_dma *dma,
		 struct virtio_msg_bus_dma_mapping *mapping, bool sync)
{
	if (mapping && mapping->shadow.from_pool)
		return 0;
	if (!dma || !mapping || !dma->provider_ops ||
	    !dma->provider_ops->export_del)
		return -EOPNOTSUPP;

	return dma->provider_ops->export_del(dma->provider_ctx,
					     &mapping->backing,
					     mapping->provider_handle, sync);
}

static int
virtio_msg_bus_dma_validate_provider_export
	(const struct virtio_msg_bus_dma_mapping *mapping,
	 const struct virtio_msg_bus_dma_export *expected_export)
{
	const struct virtio_msg_bus_dma_export *provider_export;
	size_t mapped_span;

	if (!mapping || !expected_export)
		return -EINVAL;
	provider_export = &mapping->backing;
	if (!provider_export->length || !provider_export->mmap_length ||
	    !PAGE_ALIGNED(provider_export->mmap_length) ||
	    !provider_export->npages || !provider_export->pages)
		return -EINVAL;
	if (provider_export->local_dma_addr != expected_export->local_dma_addr ||
	    provider_export->length != expected_export->length ||
	    provider_export->mmap_length != expected_export->mmap_length ||
	    provider_export->page_offset != expected_export->page_offset ||
	    provider_export->npages != expected_export->npages ||
	    provider_export->pages != expected_export->pages ||
	    provider_export->sgt != expected_export->sgt ||
	    provider_export->dma_dev != expected_export->dma_dev ||
	    provider_export->cpu_addr != expected_export->cpu_addr ||
	    provider_export->attrs != expected_export->attrs ||
	    provider_export->dir != expected_export->dir ||
	    provider_export->coherent != expected_export->coherent)
		return -EOPNOTSUPP;
	if (provider_export->length != mapping->key.length)
		return -EOPNOTSUPP;
	if (check_add_overflow(provider_export->page_offset,
			       provider_export->length, &mapped_span))
		return -EOVERFLOW;
	if (PAGE_ALIGN(mapped_span) != provider_export->mmap_length)
		return -EINVAL;
	if (provider_export->npages !=
	    provider_export->mmap_length >> PAGE_SHIFT)
		return -EINVAL;
	if (mapping->backing.coherent) {
		if (provider_export->dma_dev != mapping->backing.dma_dev ||
		    provider_export->cpu_addr != mapping->backing.cpu_addr ||
		    provider_export->attrs != mapping->backing.attrs)
			return -EOPNOTSUPP;
	}

	return 0;
}

static int
virtio_msg_bus_dma_provider_export_record
		(struct virtio_msg_bus_dma *dma,
		 struct virtio_msg_bus_dma_mapping *mapping, gfp_t gfp)
{
	struct virtio_msg_bus_dma_export expected_export;
	int ret;
	void *provider_handle = NULL;

	if (!dma || !mapping || !dma->provider_ops ||
	    !dma->provider_ops->export_add)
		return -EOPNOTSUPP;

	expected_export = mapping->backing;
	ret = dma->provider_ops->export_add(dma->provider_ctx, &mapping->backing,
					    &provider_handle, gfp);
	if (ret)
		return ret;

	ret = virtio_msg_bus_dma_validate_provider_export(mapping,
							  &expected_export);
	if (ret) {
		(void)dma->provider_ops->export_del(dma->provider_ctx,
						    &mapping->backing,
						    provider_handle, false);
		mapping->backing = expected_export;
		return ret;
	}

	mapping->provider_handle = provider_handle;
	mapping->key.dma_addr = mapping->backing.dma_addr;

	return 0;
}

static struct virtio_msg_bus_dma_mapping *
virtio_msg_bus_dma_alloc_page_record
	(struct page *page, size_t offset, size_t length, dma_addr_t dma_addr,
	 enum dma_data_direction dir, bool coherent, void *cpu_addr,
	 struct device *dma_dev,
	 unsigned long attrs, gfp_t gfp,
	 enum virtio_msg_bus_dma_mapping_kind kind)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	size_t page_offset;
	size_t mmap_length;
	unsigned int npages;
	unsigned int i;

	if (!page || !length)
		return ERR_PTR(-EINVAL);

	page += offset >> PAGE_SHIFT;
	page_offset = offset_in_page(offset);
	mmap_length = PAGE_ALIGN(page_offset + length);
	npages = mmap_length >> PAGE_SHIFT;

	mapping = kzalloc(sizeof(*mapping), gfp);
	if (!mapping)
		return ERR_PTR(-ENOMEM);

	mapping->backing.pages = kcalloc(npages, sizeof(*mapping->backing.pages),
					 gfp);
	if (!mapping->backing.pages) {
		kfree(mapping);
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(&mapping->deferred_node);
	for (i = 0; i < npages; i++)
		mapping->backing.pages[i] = page + i;

	mapping->key.dma_addr = dma_addr;
	mapping->key.length = length;
	mapping->kind = kind;
	mapping->backing_mode = VIRTIO_MSG_BUS_DMA_BACKING_DIRECT;
	mapping->backing.dma_addr = dma_addr;
	mapping->backing.local_dma_addr = dma_addr;
	mapping->backing.length = length;
	mapping->backing.mmap_length = mmap_length;
	mapping->backing.page_offset = page_offset;
	mapping->backing.npages = npages;
	mapping->backing.sgt = NULL;
	mapping->backing.dma_dev = dma_dev;
	mapping->backing.cpu_addr = cpu_addr;
	mapping->backing.attrs = attrs;
	mapping->backing.dir = dir;
	mapping->backing.coherent = coherent;
	mapping->cpu_addr = cpu_addr;
	mapping->attrs = attrs;

	return mapping;
}

static int
virtio_msg_bus_dma_clone_sgtable(struct sg_table *dst,
				 const struct sg_table *src, gfp_t gfp)
{
	struct scatterlist *dst_sg;
	struct scatterlist *src_sg;
	unsigned int copied = 0;
	int ret;

	if (!dst || !src || !src->orig_nents || !src->sgl)
		return -EINVAL;

	ret = sg_alloc_table(dst, src->orig_nents, gfp);
	if (ret)
		return ret;

	dst_sg = dst->sgl;
	for_each_sg(src->sgl, src_sg, src->orig_nents, copied) {
		sg_set_page(dst_sg, sg_page(src_sg), src_sg->length,
			    src_sg->offset);
		dst_sg = sg_next(dst_sg);
	}

	dst->nents = src->nents;

	return 0;
}

static void
virtio_msg_bus_dma_sg_extract_release
	(struct virtio_msg_bus_dma_sg_extract *extract)
{
	if (!extract)
		return;

	if (extract->cloned_sgt_valid) {
		sg_free_table(&extract->cloned_sgt);
		memset(&extract->cloned_sgt, 0, sizeof(extract->cloned_sgt));
		extract->cloned_sgt_valid = false;
	}

	kfree(extract->pages);
	extract->pages = NULL;
	extract->npages = 0;
	extract->page_offset = 0;
	extract->mmap_length = 0;
}

static int
virtio_msg_bus_dma_extract_sg_data
	(struct sg_table *sgt, size_t length, gfp_t gfp, bool clone_sgt,
	 struct virtio_msg_bus_dma_sg_extract *extract)
{
	struct scatterlist *sg;
	size_t page_offset = 0;
	size_t remaining = length;
	unsigned int expected_npages;
	unsigned int npages = 0;
	unsigned int page_index = 0;
	struct page **pages;
	bool first = true;
	int i;

	if (!extract)
		return -EINVAL;

	memset(extract, 0, sizeof(*extract));

	if (!sgt || !sgt->sgl || !length)
		return -EINVAL;

	for_each_sgtable_sg(sgt, sg, i) {
		size_t seg_len;
		unsigned int seg_pages;

		if (!remaining)
			break;

		seg_len = min_t(size_t, sg->length, remaining);
		if (first) {
			page_offset = sg->offset;
			first = false;
		} else if (sg->offset) {
			return -EINVAL;
		}

		if (remaining > seg_len &&
		    !PAGE_ALIGNED(sg->offset + seg_len))
			return -EINVAL;

		seg_pages = DIV_ROUND_UP(sg->offset + seg_len, PAGE_SIZE);
		if (!seg_pages)
			return -EINVAL;

		npages += seg_pages;
		remaining -= seg_len;
	}

	if (remaining || first)
		return -EINVAL;

	expected_npages = PAGE_ALIGN(page_offset + length) >> PAGE_SHIFT;
	if (npages != expected_npages)
		return -EINVAL;

	pages = kcalloc(npages, sizeof(*pages), gfp);
	if (!pages)
		return -ENOMEM;

	remaining = length;
	for_each_sgtable_sg(sgt, sg, i) {
		size_t seg_len;
		unsigned int seg_pages;
		unsigned int page_nr;

		if (!remaining)
			break;

		seg_len = min_t(size_t, sg->length, remaining);
		seg_pages = DIV_ROUND_UP(sg->offset + seg_len, PAGE_SIZE);
		for (page_nr = 0; page_nr < seg_pages; page_nr++)
			pages[page_index++] = sg_page(sg) + page_nr;
		remaining -= seg_len;
	}

	extract->pages = pages;
	extract->npages = npages;
	extract->page_offset = page_offset;
	extract->mmap_length = PAGE_ALIGN(page_offset + length);
	if (clone_sgt &&
	    !virtio_msg_bus_dma_clone_sgtable(&extract->cloned_sgt, sgt, gfp))
		extract->cloned_sgt_valid = true;

	return 0;
}

static struct virtio_msg_bus_dma_mapping *
virtio_msg_bus_dma_alloc_sgt_record
	(struct sg_table *sgt, size_t length, dma_addr_t dma_addr,
	 enum dma_data_direction dir, bool coherent, struct device *dma_dev,
	 void *cpu_addr,
	 unsigned long attrs, gfp_t gfp,
	 enum virtio_msg_bus_dma_mapping_kind kind)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	struct virtio_msg_bus_dma_sg_extract extract;
	int ret;

	ret = virtio_msg_bus_dma_extract_sg_data(sgt, length, gfp, true,
						 &extract);
	if (ret)
		return ERR_PTR(ret);

	mapping = kzalloc(sizeof(*mapping), gfp);
	if (!mapping) {
		virtio_msg_bus_dma_sg_extract_release(&extract);
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(&mapping->deferred_node);
	mapping->key.dma_addr = dma_addr;
	mapping->key.length = length;
	mapping->kind = kind;
	mapping->backing_mode = VIRTIO_MSG_BUS_DMA_BACKING_DIRECT;
	mapping->backing.dma_addr = dma_addr;
	mapping->backing.local_dma_addr = dma_addr;
	mapping->backing.length = length;
	mapping->backing.mmap_length = extract.mmap_length;
	mapping->backing.page_offset = extract.page_offset;
	mapping->backing.npages = extract.npages;
	mapping->backing.pages = extract.pages;
	mapping->backing.sgt = NULL;
	mapping->backing.dma_dev = dma_dev;
	mapping->backing.cpu_addr = cpu_addr;
	mapping->backing.attrs = attrs;
	mapping->backing.dir = dir;
	mapping->backing.coherent = coherent;
	mapping->cpu_addr = cpu_addr;
	mapping->attrs = attrs;
	extract.pages = NULL;

	if (extract.cloned_sgt_valid) {
		mapping->backing_sgt = extract.cloned_sgt;
		mapping->backing_sgt_valid = true;
		mapping->backing.sgt = &mapping->backing_sgt;
		memset(&extract.cloned_sgt, 0, sizeof(extract.cloned_sgt));
		extract.cloned_sgt_valid = false;
	}

	virtio_msg_bus_dma_sg_extract_release(&extract);

	return mapping;
}

static bool
virtio_msg_bus_dma_page_needs_shadow(struct page *page)
{
	struct folio *folio;

	if (!page)
		return true;

	folio = page_folio(page);
	return folio_test_slab(folio) || folio_test_large_kmalloc(folio);
}

static bool
virtio_msg_bus_dma_mapping_needs_shadow
	(const struct virtio_msg_bus_dma_mapping *mapping)
{
	unsigned int i;

	if (!mapping || !mapping->backing.pages || !mapping->backing.npages)
		return false;

	for (i = 0; i < mapping->backing.npages; i++) {
		if (virtio_msg_bus_dma_page_needs_shadow(mapping->backing.pages[i]))
			return true;
	}

	return false;
}

static bool
virtio_msg_bus_dma_copy_to_shadow_needed(enum dma_data_direction dir)
{
	return dir == DMA_TO_DEVICE || dir == DMA_BIDIRECTIONAL;
}

static bool
virtio_msg_bus_dma_copy_from_shadow_needed(enum dma_data_direction dir)
{
	return dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL;
}

static void
virtio_msg_bus_dma_copy_between_pages
	(struct page **dst_pages, unsigned int dst_npages, size_t dst_offset,
	 struct page **src_pages, unsigned int src_npages, size_t src_offset,
	 size_t length)
{
	size_t remaining = length;

	while (remaining) {
		unsigned int dst_page_index = dst_offset >> PAGE_SHIFT;
		unsigned int src_page_index = src_offset >> PAGE_SHIFT;
		size_t dst_page_offset = offset_in_page(dst_offset);
		size_t src_page_offset = offset_in_page(src_offset);
		size_t chunk;
		void *src_va;
		void *dst_va;

		if (WARN_ON_ONCE(dst_page_index >= dst_npages ||
				 src_page_index >= src_npages))
			return;

		chunk = min_t(size_t, remaining, PAGE_SIZE - dst_page_offset);
		chunk = min_t(size_t, chunk, PAGE_SIZE - src_page_offset);

		dst_va = kmap_local_page(dst_pages[dst_page_index]);
		src_va = kmap_local_page(src_pages[src_page_index]);
		memcpy((char *)dst_va + dst_page_offset,
		       (char *)src_va + src_page_offset, chunk);
		kunmap_local(src_va);
		kunmap_local(dst_va);

		dst_offset += chunk;
		src_offset += chunk;
		remaining -= chunk;
	}
}

static void
virtio_msg_bus_dma_shadow_copy_to_shadow
	(struct virtio_msg_bus_dma_mapping *mapping, size_t offset, size_t length)
{
	if (!virtio_msg_bus_dma_mapping_is_shadow(mapping) ||
	    !mapping->shadow.copy_to_shadow)
		return;

	virtio_msg_bus_dma_copy_between_pages
		(mapping->backing.pages, mapping->backing.npages,
		 mapping->backing.page_offset + offset,
		 mapping->shadow.src_pages, mapping->shadow.src_npages,
		 mapping->shadow.src_page_offset + offset, length);
}

static void
virtio_msg_bus_dma_shadow_copy_from_shadow
	(struct virtio_msg_bus_dma_mapping *mapping, size_t offset, size_t length)
{
	if (!virtio_msg_bus_dma_mapping_is_shadow(mapping) ||
	    !mapping->shadow.copy_from_shadow)
		return;

	virtio_msg_bus_dma_copy_between_pages
		(mapping->shadow.src_pages, mapping->shadow.src_npages,
		 mapping->shadow.src_page_offset + offset,
		 mapping->backing.pages, mapping->backing.npages,
		 mapping->backing.page_offset + offset, length);
}

static struct virtio_msg_bus_dma_mapping *
virtio_msg_bus_dma_find_record_locked
	(struct virtio_msg_bus_dma *dma, dma_addr_t dma_addr,
	 size_t length)
{
	struct virtio_msg_bus_dma_mapping *mapping;

	mapping = xa_load(&dma->active_exports, (unsigned long)dma_addr);
	if (!mapping || mapping->key.length != length)
		return NULL;

	return mapping;
}

static bool
virtio_msg_bus_dma_mapping_contains
	(const struct virtio_msg_bus_dma_mapping *mapping, dma_addr_t dma_addr,
	 size_t length)
{
	dma_addr_t mapping_end;
	dma_addr_t req_end;

	if (!mapping || !length || dma_addr < mapping->key.dma_addr)
		return false;
	if (check_add_overflow(mapping->key.dma_addr, mapping->key.length,
			       &mapping_end))
		return false;
	if (check_add_overflow(dma_addr, length, &req_end))
		return false;

	return req_end <= mapping_end;
}

static bool
virtio_msg_bus_dma_mapping_offset
	(const struct virtio_msg_bus_dma_mapping *mapping, dma_addr_t dma_addr,
	 size_t length, size_t *offset)
{
	if (!virtio_msg_bus_dma_mapping_contains(mapping, dma_addr, length))
		return false;

	*offset = dma_addr - mapping->key.dma_addr;

	return true;
}

static bool
virtio_msg_bus_dma_backing_dma_handle
	(const struct virtio_msg_bus_dma_mapping *mapping, dma_addr_t dma_addr,
	 size_t length, dma_addr_t *backing_dma_addr)
{
	size_t offset;

	if (!backing_dma_addr)
		return false;
	if (!virtio_msg_bus_dma_mapping_offset(mapping, dma_addr, length, &offset))
		return false;
	if (check_add_overflow(mapping->backing.local_dma_addr, (dma_addr_t)offset,
			       backing_dma_addr))
		return false;

	return true;
}

static struct virtio_msg_bus_dma_mapping *
virtio_msg_bus_dma_find_record_covering_locked
	(struct virtio_msg_bus_dma *dma, dma_addr_t dma_addr,
	 size_t length)
{
	unsigned long index;
	struct virtio_msg_bus_dma_mapping *best = NULL;
	struct virtio_msg_bus_dma_mapping *mapping;

	xa_for_each(&dma->active_exports, index, mapping) {
		if (!virtio_msg_bus_dma_mapping_contains(mapping, dma_addr,
							 length))
			continue;
		if (!best || mapping->key.dma_addr > best->key.dma_addr ||
		    (mapping->key.dma_addr == best->key.dma_addr &&
		     mapping->key.length < best->key.length))
			best = mapping;
	}

	return best;
}

static int
virtio_msg_bus_dma_track_export
	(struct virtio_msg_bus_dma *dma,
	 struct virtio_msg_bus_dma_mapping *mapping)
{
	unsigned long flags;
	unsigned long index = (unsigned long)mapping->key.dma_addr;
	int ret = 0;

	xa_lock_irqsave(&dma->active_exports, flags);
	spin_lock(&dma->active_exports_lock);
	if (xa_load(&dma->active_exports, index)) {
		ret = -EEXIST;
		goto out_unlock;
	}

	ret = __xa_insert(&dma->active_exports, index, mapping, GFP_ATOMIC);

out_unlock:
	spin_unlock(&dma->active_exports_lock);
	xa_unlock_irqrestore(&dma->active_exports, flags);

	return ret;
}

static struct virtio_msg_bus_dma_mapping *
virtio_msg_bus_dma_untrack_export
	(struct virtio_msg_bus_dma *dma, dma_addr_t dma_addr, size_t length)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	unsigned long flags;

	xa_lock_irqsave(&dma->active_exports, flags);
	spin_lock(&dma->active_exports_lock);
	mapping = virtio_msg_bus_dma_find_record_locked(dma, dma_addr, length);
	if (mapping)
		mapping = __xa_erase(&dma->active_exports, (unsigned long)dma_addr);
	spin_unlock(&dma->active_exports_lock);
	xa_unlock_irqrestore(&dma->active_exports, flags);

	return mapping;
}

static struct virtio_msg_bus_dma_mapping *
virtio_msg_bus_dma_lookup_export
	(struct virtio_msg_bus_dma *dma, dma_addr_t dma_addr, size_t length)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	unsigned long flags;

	xa_lock_irqsave(&dma->active_exports, flags);
	spin_lock(&dma->active_exports_lock);
	mapping = virtio_msg_bus_dma_find_record_locked(dma, dma_addr, length);
	if (!mapping)
		mapping = virtio_msg_bus_dma_find_record_covering_locked(dma,
									 dma_addr,
									 length);
	spin_unlock(&dma->active_exports_lock);
	xa_unlock_irqrestore(&dma->active_exports, flags);

	return mapping;
}

static void
virtio_msg_bus_dma_shadow_sync_for_device
	(struct virtio_msg_bus_dma *dma,
	 struct virtio_msg_bus_dma_mapping *mapping, dma_addr_t dma_handle,
	 size_t size,
	 enum dma_data_direction dir)
{
	dma_addr_t backing_dma_handle;
	bool in_range;
	size_t offset = 0;

	in_range = virtio_msg_bus_dma_mapping_offset(mapping, dma_handle, size, &offset);
	if (WARN_ON_ONCE(!in_range) ||
	    !virtio_msg_bus_dma_backing_dma_handle(mapping, dma_handle, size,
						   &backing_dma_handle)) {
		dma_sync_single_for_device(dma->delegate_dev,
					   mapping ? mapping->backing.local_dma_addr :
					   dma_handle,
					   size, dir);
		return;
	}

	if (virtio_msg_bus_dma_mapping_is_shadow(mapping) &&
	    mapping->shadow.copy_to_shadow)
		virtio_msg_bus_dma_shadow_copy_to_shadow(mapping, offset, size);

	dma_sync_single_for_device(dma->delegate_dev, backing_dma_handle, size,
				   dir);
}

static void
virtio_msg_bus_dma_shadow_sync_for_cpu
	(struct virtio_msg_bus_dma *dma,
	 struct virtio_msg_bus_dma_mapping *mapping, dma_addr_t dma_handle,
	 size_t size,
	 enum dma_data_direction dir)
{
	dma_addr_t backing_dma_handle;
	bool in_range;
	size_t offset = 0;

	in_range = virtio_msg_bus_dma_mapping_offset(mapping, dma_handle, size, &offset);
	if (WARN_ON_ONCE(!in_range) ||
	    !virtio_msg_bus_dma_backing_dma_handle(mapping, dma_handle, size,
						   &backing_dma_handle)) {
		dma_sync_single_for_cpu(dma->delegate_dev,
					mapping ? mapping->backing.local_dma_addr :
					dma_handle,
					size, dir);
		return;
	}

	dma_sync_single_for_cpu(dma->delegate_dev, backing_dma_handle, size,
				dir);
	if (virtio_msg_bus_dma_mapping_is_shadow(mapping) &&
	    mapping->shadow.copy_from_shadow)
		virtio_msg_bus_dma_shadow_copy_from_shadow(mapping, offset, size);
}

static int
virtio_msg_bus_dma_prepare_shadow_record
	(struct virtio_msg_bus_dma *dma,
	 struct virtio_msg_bus_dma_mapping *mapping, gfp_t gfp)
{
	struct virtio_msg_bus_dma_sg_extract extract;
	struct sg_table sgt;
	dma_addr_t shadow_dma_addr;
	void *shadow_cpu_addr;
	int ret;

	/*
	 * Pool-first path: if a pre-registered pool chunk has room, sub-allocate
	 * shadow memory from it.  This avoids dma_alloc_attrs per mapping and,
	 * more importantly, avoids a MAP_EVENT_ADD round-trip to the VMM because
	 * the pool pages are already ACTIVE.
	 */
	if (dma->pool_chunk_size && !list_empty(&dma->pool_chunks)) {
		ret = virtio_msg_bus_dma_prepare_shadow_from_pool(dma, mapping, gfp);
		if (!ret)
			return 0;
		if (ret != -ENOMEM)
			return ret;
		/* Pool exhausted; continue to per-mapping fallback below. */
	}

	/*
	 * Per-mapping DMA allocation is only safe in sleepable context.
	 * dma_get_sgtable_attrs internally calls sg_alloc_table with a
	 * hardcoded GFP_KERNEL regardless of the gfp argument, so reaching
	 * this path from an atomic/spinlock context (e.g. virtio-blk
	 * queue_rq) would trigger a sleeping-from-atomic BUG.  Fail fast
	 * here; the pool should have served this allocation — if it could
	 * not (allocation exceeds pool chunk capacity), the caller will
	 * return DMA_MAPPING_ERROR and the block layer will retry.
	 */
	if (!gfpflags_allow_blocking(gfp))
		return -ENOMEM;

	memset(&sgt, 0, sizeof(sgt));
	shadow_cpu_addr = dma_alloc_attrs(dma->delegate_dev,
					  mapping->backing.length,
					  &shadow_dma_addr, gfp, 0);
	if (!shadow_cpu_addr)
		return -ENOMEM;

	ret = dma_get_sgtable_attrs(dma->delegate_dev, &sgt, shadow_cpu_addr,
				    shadow_dma_addr, mapping->backing.length, 0);
	if (ret)
		goto err_free_shadow;

	ret = virtio_msg_bus_dma_extract_sg_data(&sgt, mapping->backing.length,
						 gfp, true, &extract);
	sg_free_table(&sgt);
	if (ret)
		goto err_free_shadow;

	mapping->shadow.src_pages = mapping->backing.pages;
	mapping->shadow.src_page_offset = mapping->backing.page_offset;
	mapping->shadow.src_npages = mapping->backing.npages;
	mapping->shadow.dev = dma->delegate_dev;
	mapping->shadow.shadow_cpu_addr = shadow_cpu_addr;
	mapping->shadow.shadow_dma_addr = shadow_dma_addr;
	mapping->shadow.shadow_length = mapping->backing.length;
	mapping->shadow.shadow_attrs = 0;
	mapping->shadow.copy_to_shadow =
		virtio_msg_bus_dma_copy_to_shadow_needed(mapping->backing.dir);
	mapping->shadow.copy_from_shadow =
		virtio_msg_bus_dma_copy_from_shadow_needed(mapping->backing.dir);

	virtio_msg_bus_dma_mapping_release_sgt(mapping);
	mapping->backing.pages = extract.pages;
	mapping->backing.dma_addr = shadow_dma_addr;
	mapping->backing.local_dma_addr = shadow_dma_addr;
	mapping->backing.mmap_length = extract.mmap_length;
	mapping->backing.page_offset = extract.page_offset;
	mapping->backing.npages = extract.npages;
	mapping->backing.coherent = false;
	mapping->key.dma_addr = shadow_dma_addr;
	mapping->backing_mode = VIRTIO_MSG_BUS_DMA_BACKING_SHADOW;
	extract.pages = NULL;
	if (extract.cloned_sgt_valid) {
		mapping->backing_sgt = extract.cloned_sgt;
		mapping->backing_sgt_valid = true;
		mapping->backing.sgt = &mapping->backing_sgt;
		memset(&extract.cloned_sgt, 0, sizeof(extract.cloned_sgt));
		extract.cloned_sgt_valid = false;
	} else {
		mapping->backing_sgt_valid = false;
		mapping->backing.sgt = NULL;
	}

	virtio_msg_bus_dma_sg_extract_release(&extract);

	virtio_msg_bus_dma_shadow_sync_for_device(dma, mapping,
						  mapping->key.dma_addr,
						  mapping->key.length,
						  mapping->backing.dir);

	return 0;

err_free_shadow:
	dma_free_attrs(dma->delegate_dev, mapping->backing.length, shadow_cpu_addr,
		       shadow_dma_addr, 0);
	return ret;
}

static void
virtio_msg_bus_dma_sync_record
	(struct virtio_msg_bus_dma *dma, dma_addr_t dma_handle, size_t size,
	 enum dma_data_direction dir, bool for_cpu)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	dma_addr_t backing_dma_handle;

	mapping = virtio_msg_bus_dma_lookup_export(dma, dma_handle, size);
	if (virtio_msg_bus_dma_mapping_is_shadow(mapping)) {
		if (for_cpu)
			virtio_msg_bus_dma_shadow_sync_for_cpu(dma, mapping,
							       dma_handle, size,
							       dir);
		else
			virtio_msg_bus_dma_shadow_sync_for_device(dma, mapping,
								  dma_handle, size,
								  dir);
		return;
	}

	if (!mapping ||
	    !virtio_msg_bus_dma_backing_dma_handle(mapping, dma_handle, size,
					      &backing_dma_handle))
		backing_dma_handle = mapping ? mapping->backing.local_dma_addr
					     : dma_handle;

	if (for_cpu)
		dma_sync_single_for_cpu(dma->delegate_dev, backing_dma_handle,
					size, dir);
	else
		dma_sync_single_for_device(dma->delegate_dev, backing_dma_handle,
					   size, dir);
}

static void
virtio_msg_bus_dma_defer_local_free_record
	(struct virtio_msg_bus_dma *dma,
	 struct virtio_msg_bus_dma_mapping *mapping)
{
	unsigned long flags;

	if (!dma || !mapping)
		return;

	spin_lock_irqsave(&dma->active_exports_lock, flags);
	list_add_tail(&mapping->deferred_node, &dma->deferred_local_frees);
	spin_unlock_irqrestore(&dma->active_exports_lock, flags);

	(void)mod_delayed_work(system_wq, &dma->deferred_local_free_work, 0);
}

static dma_addr_t
virtio_msg_bus_dma_map_phys_record(struct device *dev, phys_addr_t phys,
				   size_t size, enum dma_data_direction dir,
				   unsigned long attrs, gfp_t gfp)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	struct virtio_msg_bus_dma *dma;
	dma_addr_t dma_addr;
	struct page *page;
	int ret;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed) {
		vm_trace(dev,
			 "dma-map reject: shim unavailable phys=%pa len=%zu dir=%d attrs=%#lx\n",
			 &phys, size, dir, attrs);
		return DMA_MAPPING_ERROR;
	}

	if ((attrs & DMA_ATTR_MMIO) || !pfn_valid(PHYS_PFN(phys))) {
		vm_trace(dev,
			 "dma-map reject: unsupported phys=%pa len=%zu dir=%d attrs=%#lx\n",
			 &phys, size, dir, attrs);
		return DMA_MAPPING_ERROR;
	}

	page = pfn_to_page(PHYS_PFN(phys));
	mapping = virtio_msg_bus_dma_alloc_page_record
		(page, offset_in_page(phys), size, 0, dir, false, NULL,
		 dma->delegate_dev, attrs,
		 gfp, VIRTIO_MSG_BUS_DMA_MAPPING_STREAMING);
	if (IS_ERR(mapping)) {
		ret = PTR_ERR(mapping);
			vm_warn_rl(dma->delegate_dev,
				   "dma-map alloc record failed phys=%pa len=%zu dir=%d ret=%d\n",
				   &phys, size, dir, ret);
		return DMA_MAPPING_ERROR;
	}

	/*
	 * For non-slab pages in a non-sleeping context (e.g. virtio-blk's
	 * queue_rq holding a spinlock): try pool-shadow first.  Pool pages are
	 * already ACTIVE in the VMM so the per-mapping MAP_EVENT_ADD export
	 * step can be skipped entirely, making the map atomic-safe.
	 */
	if (!virtio_msg_bus_dma_mapping_needs_shadow(mapping) &&
	    !gfpflags_allow_blocking(gfp) &&
	    dma->pool_chunk_size &&
	    !list_empty(&dma->pool_chunks)) {
		ret = virtio_msg_bus_dma_prepare_shadow_from_pool(dma, mapping,
								  gfp);
		if (ret == 0) {
			dma_addr = mapping->key.dma_addr;
			goto track;
		}
		if (ret == -EAGAIN) {
			/*
			 * Pool full; grow work scheduled — fail fast and let
			 * blk-mq retry after the delayed requeue.
			 */
			vm_trace(dma->delegate_dev,
				 "dma-map pool busy phys=%pa size=%zu dir=%d\n",
				 &phys, size, dir);
			virtio_msg_bus_dma_free_record(mapping);
			return DMA_MAPPING_ERROR;
		}
		/* -ENOMEM: mapping exceeds pool slot size; try normal path */
	}

	ret = virtio_msg_bus_dma_prepare_shadow_record(dma, mapping, gfp);
	if (ret) {
		vm_warn_rl(dma->delegate_dev,
			   "dma-map shadow prepare failed phys=%pa len=%zu dir=%d ret=%d\n",
			   &phys, size, dir, ret);
		virtio_msg_bus_dma_free_record(mapping);
		return DMA_MAPPING_ERROR;
	}
	dma_addr = mapping->key.dma_addr;

	/*
	 * Pool-backed shadows (both slab and the forced non-slab path above):
	 * the pool chunk export is already ACTIVE in the VMM.  No per-mapping
	 * MAP_EVENT_ADD is needed — skip provider_export_record entirely.
	 */
	if (mapping->shadow.from_pool)
		goto track;

	ret = virtio_msg_bus_dma_provider_export_record(dma, mapping, gfp);
	if (ret) {
		vm_warn_rl(dma->delegate_dev,
			   "dma-map provider export failed phys=%pa len=%zu dir=%d ret=%d\n",
			   &phys, size, dir, ret);
		virtio_msg_bus_dma_free_record(mapping);
		return DMA_MAPPING_ERROR;
	}
	dma_addr = mapping->key.dma_addr;

track:
	ret = virtio_msg_bus_dma_track_export(dma, mapping);
	if (ret) {
		vm_warn_rl(dma->delegate_dev,
			   "dma-map track export failed phys=%pa len=%zu dir=%d ret=%d\n",
			   &phys, size, dir, ret);
		if (virtio_msg_bus_dma_mapping_needs_deferred_release(mapping)) {
			virtio_msg_bus_dma_defer_local_free_record(dma, mapping);
		} else {
			virtio_msg_bus_dma_provider_export_del_record(dma, mapping,
								      false);
			virtio_msg_bus_dma_free_record(mapping);
		}
		return DMA_MAPPING_ERROR;
	}
	vm_trace(dma->delegate_dev,
		 "dma-map done: phys=%pa size=%zu dir=%d dma=%pad path=shadow\n",
		 &phys, size, dir, &dma_addr);

	return dma_addr;
}

static void
virtio_msg_bus_dma_unmap_phys_record(struct device *dev, dma_addr_t dma_addr,
				     size_t size, enum dma_data_direction dir,
				     unsigned long attrs)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	struct virtio_msg_bus_dma *dma;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return;

	mapping = virtio_msg_bus_dma_untrack_export(dma, dma_addr, size);
	if (!mapping)
		goto out_unmap_direct;

	if (virtio_msg_bus_dma_mapping_is_shadow(mapping)) {
		virtio_msg_bus_dma_shadow_sync_for_cpu(dma, mapping,
						       mapping->key.dma_addr,
						       mapping->key.length, dir);
		if (!virtio_msg_bus_dma_mapping_needs_deferred_release(mapping))
			virtio_msg_bus_dma_free_record(mapping);
		else
			virtio_msg_bus_dma_defer_local_free_record(dma, mapping);
			vm_trace(dma->delegate_dev,
				 "dma-unmap done: dma=%pad size=%zu dir=%d path=shadow\n",
				 &dma_addr, size, dir);
		return;
	}

	if (mapping) {
		int ret;

		ret = virtio_msg_bus_dma_provider_export_del_record(dma, mapping,
								    false);
			if (ret && ret != -EOPNOTSUPP)
				vm_warn_rl(dma->delegate_dev,
					   "provider revoke failed dma=%pad len=%zu ret=%d\n",
					   &mapping->backing.dma_addr,
					   mapping->backing.length, ret);
	}

out_unmap_direct:
	dma_unmap_phys(dma->delegate_dev,
		       mapping ? mapping->backing.local_dma_addr : dma_addr,
		       size, dir, attrs);
	vm_trace(dma->delegate_dev,
		 "dma-unmap done: dma=%pad size=%zu dir=%d path=%s\n",
		 &dma_addr, size, dir, mapping ? "direct" : "passthrough");
	if (!mapping)
		return;

	virtio_msg_bus_dma_free_record(mapping);
}

static void *
virtio_msg_bus_dma_alloc(struct device *dev, size_t size, dma_addr_t *dma_handle,
			 gfp_t gfp, unsigned long attrs)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	struct virtio_msg_bus_dma *dma;
	struct sg_table sgt;
	void *cpu_addr;
	int ret;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return NULL;

	cpu_addr = dma_alloc_attrs(dma->delegate_dev, size, dma_handle, gfp,
				   attrs);
	if (!cpu_addr)
		return NULL;

	memset(&sgt, 0, sizeof(sgt));
	ret = dma_get_sgtable_attrs(dma->delegate_dev, &sgt, cpu_addr, *dma_handle,
				    size, attrs);
	if (ret) {
		dma_free_attrs(dma->delegate_dev, size, cpu_addr, *dma_handle,
			       attrs);
		return NULL;
	}

	mapping = virtio_msg_bus_dma_alloc_sgt_record
		(&sgt, size, *dma_handle, DMA_BIDIRECTIONAL, true,
		 dma->delegate_dev, cpu_addr, attrs, gfp,
		 VIRTIO_MSG_BUS_DMA_MAPPING_COHERENT);
	sg_free_table(&sgt);
	if (IS_ERR(mapping)) {
		dma_free_attrs(dma->delegate_dev, size, cpu_addr, *dma_handle,
			       attrs);
		return NULL;
	}

	ret = virtio_msg_bus_dma_provider_export_record(dma, mapping, gfp);
	if (ret) {
		virtio_msg_bus_dma_free_record(mapping);
		dma_free_attrs(dma->delegate_dev, size, cpu_addr, *dma_handle,
			       attrs);
		return NULL;
	}

	ret = virtio_msg_bus_dma_track_export(dma, mapping);
	if (ret) {
		if (virtio_msg_bus_dma_mapping_needs_deferred_release(mapping)) {
			virtio_msg_bus_dma_defer_local_free_record(dma, mapping);
		} else {
			virtio_msg_bus_dma_provider_export_del_record(dma, mapping,
								      false);
			virtio_msg_bus_dma_free_record(mapping);
			dma_free_attrs(dma->delegate_dev, size, cpu_addr,
				       *dma_handle, attrs);
		}
		return NULL;
	}
	*dma_handle = mapping->key.dma_addr;

	return cpu_addr;
}

static void
virtio_msg_bus_dma_free(struct device *dev, size_t size, void *vaddr,
			dma_addr_t dma_addr, unsigned long attrs)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	struct virtio_msg_bus_dma *dma;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return;

	mapping = virtio_msg_bus_dma_untrack_export(dma, dma_addr, size);
	if (!mapping) {
		dma_free_attrs(dma->delegate_dev, size, vaddr, dma_addr, attrs);
		return;
	}

	mapping->cpu_addr = vaddr;
	mapping->attrs = attrs;
	virtio_msg_bus_dma_defer_local_free_record(dma, mapping);
}

static int
virtio_msg_bus_dma_mmap(struct device *dev, struct vm_area_struct *vma,
			void *cpu_addr, dma_addr_t dma_addr, size_t size,
			unsigned long attrs)
{
	struct virtio_msg_bus_dma *dma;
	struct virtio_msg_bus_dma_mapping *mapping;
	dma_addr_t backing_dma_addr;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return -ENODEV;

	mapping = virtio_msg_bus_dma_lookup_export(dma, dma_addr, size);
	if (mapping &&
	    virtio_msg_bus_dma_backing_dma_handle(mapping, dma_addr, size,
						  &backing_dma_addr))
		dma_addr = backing_dma_addr;

	return dma_mmap_attrs(dma->delegate_dev, vma, cpu_addr, dma_addr, size,
			      attrs);
}

static int
virtio_msg_bus_dma_get_sgtable(struct device *dev, struct sg_table *sgt,
			       void *cpu_addr, dma_addr_t dma_addr, size_t size,
			       unsigned long attrs)
{
	struct virtio_msg_bus_dma *dma;
	struct virtio_msg_bus_dma_mapping *mapping;
	dma_addr_t backing_dma_addr;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return -ENODEV;

	mapping = virtio_msg_bus_dma_lookup_export(dma, dma_addr, size);
	if (mapping &&
	    virtio_msg_bus_dma_backing_dma_handle(mapping, dma_addr, size,
						  &backing_dma_addr))
		dma_addr = backing_dma_addr;

	return dma_get_sgtable_attrs(dma->delegate_dev, sgt, cpu_addr, dma_addr,
				     size, attrs);
}

static dma_addr_t
virtio_msg_bus_dma_map_phys(struct device *dev, phys_addr_t phys, size_t size,
			    enum dma_data_direction dir, unsigned long attrs)
{
	/*
	 * Propagate the correct GFP context to the DMA provider.  In softirq
	 * or IRQ context the provider cannot sleep; the atomic path returns
	 * DMA_MAPPING_ERROR so the caller can retry from process context.
	 */
	gfp_t gfp = preemptible() ? GFP_KERNEL : GFP_ATOMIC;

	return virtio_msg_bus_dma_map_phys_record(dev, phys, size, dir, attrs,
						  gfp);
}

static void
virtio_msg_bus_dma_unmap_phys(struct device *dev, dma_addr_t dma_handle,
			      size_t size, enum dma_data_direction dir,
			      unsigned long attrs)
{
	virtio_msg_bus_dma_unmap_phys_record(dev, dma_handle, size, dir, attrs);
}

static void
virtio_msg_bus_dma_sync_single_for_cpu(struct device *dev, dma_addr_t dma_handle,
				       size_t size, enum dma_data_direction dir)
{
	struct virtio_msg_bus_dma *dma;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return;

	virtio_msg_bus_dma_sync_record(dma, dma_handle, size, dir, true);
}

static void
virtio_msg_bus_dma_sync_single_for_device
	(struct device *dev, dma_addr_t dma_handle, size_t size,
	 enum dma_data_direction dir)
{
	struct virtio_msg_bus_dma *dma;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return;

	virtio_msg_bus_dma_sync_record(dma, dma_handle, size, dir, false);
}

static void
virtio_msg_bus_dma_sync_sg_for_cpu(struct device *dev, struct scatterlist *sg,
				   int nents, enum dma_data_direction dir)
{
	struct virtio_msg_bus_dma *dma;
	struct scatterlist *entry;
	int i;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return;

	for_each_sg(sg, entry, nents, i)
		virtio_msg_bus_dma_sync_record(dma, sg_dma_address(entry),
					       sg_dma_len(entry), dir, true);
}

static void
virtio_msg_bus_dma_sync_sg_for_device(struct device *dev, struct scatterlist *sg,
				      int nents, enum dma_data_direction dir)
{
	struct virtio_msg_bus_dma *dma;
	struct scatterlist *entry;
	int i;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return;

	for_each_sg(sg, entry, nents, i)
		virtio_msg_bus_dma_sync_record(dma, sg_dma_address(entry),
					       sg_dma_len(entry), dir, false);
}

static void virtio_msg_bus_dma_sg_set_addr(struct scatterlist *sg,
					   dma_addr_t addr)
{
	sg_dma_address(sg) = addr;
#ifdef CONFIG_NEED_SG_DMA_LENGTH
	sg_dma_len(sg) = sg->length;
#endif
}

static void virtio_msg_bus_dma_sg_clear_addr(struct scatterlist *sg)
{
	sg_dma_address(sg) = 0;
#ifdef CONFIG_NEED_SG_DMA_LENGTH
	sg_dma_len(sg) = 0;
#endif
}

static int
virtio_msg_bus_dma_map_sg(struct device *dev, struct scatterlist *sg, int nents,
			  enum dma_data_direction dir, unsigned long attrs)
{
	struct scatterlist *entry;
	int i;
	int j;

	for_each_sg(sg, entry, nents, i) {
		phys_addr_t phys = page_to_phys(sg_page(entry)) + entry->offset;
		dma_addr_t dma_addr;

		dma_addr = virtio_msg_bus_dma_map_phys(dev, phys, entry->length,
						       dir, attrs);
		if (dma_addr == DMA_MAPPING_ERROR)
			goto err_unmap;

		virtio_msg_bus_dma_sg_set_addr(entry, dma_addr);
	}

	return nents;

err_unmap:
	for_each_sg(sg, entry, i, j) {
		virtio_msg_bus_dma_unmap_phys(dev, sg_dma_address(entry),
					      entry->length, dir, attrs);
		virtio_msg_bus_dma_sg_clear_addr(entry);
	}

	return -ENOMEM;
}

static void
virtio_msg_bus_dma_unmap_sg(struct device *dev, struct scatterlist *sg, int nents,
			    enum dma_data_direction dir, unsigned long attrs)
{
	struct scatterlist *entry;
	int i;

	for_each_sg(sg, entry, nents, i) {
		virtio_msg_bus_dma_unmap_phys(dev, sg_dma_address(entry),
					      entry->length, dir, attrs);
		virtio_msg_bus_dma_sg_clear_addr(entry);
	}
}

static int virtio_msg_bus_dma_supported(struct device *dev, u64 mask)
{
	struct virtio_msg_bus_dma *dma;
	const struct dma_map_ops *ops;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return 0;

	ops = get_dma_ops(dma->delegate_dev);
	if (use_dma_iommu(dma->delegate_dev)) {
		if (WARN_ON(ops))
			return 0;
		return 1;
	}

	if (!ops)
		return dma_direct_supported(dma->delegate_dev, mask);
	if (!ops->dma_supported)
		return 1;

	return ops->dma_supported(dma->delegate_dev, mask);
}

static u64 virtio_msg_bus_dma_get_required_mask(struct device *dev)
{
	struct virtio_msg_bus_dma *dma;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return DMA_BIT_MASK(32);

	return dma_get_required_mask(dma->delegate_dev);
}

static size_t virtio_msg_bus_dma_max_mapping_size(struct device *dev)
{
	struct virtio_msg_bus_dma *dma;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return 0;

	return dma_max_mapping_size(dma->delegate_dev);
}

static unsigned long
virtio_msg_bus_dma_get_merge_boundary(struct device *dev)
{
	struct virtio_msg_bus_dma *dma;

	dma = virtio_msg_bus_dma_from_dev(dev);
	if (!dma || !dma->shim_installed)
		return 0;

	return dma_get_merge_boundary(dma->delegate_dev);
}

static const struct dma_map_ops virtio_msg_bus_dma_map_ops = {
	.alloc = virtio_msg_bus_dma_alloc,
	.free = virtio_msg_bus_dma_free,
	.mmap = virtio_msg_bus_dma_mmap,
	.get_sgtable = virtio_msg_bus_dma_get_sgtable,
	.map_phys = virtio_msg_bus_dma_map_phys,
	.unmap_phys = virtio_msg_bus_dma_unmap_phys,
	.map_sg = virtio_msg_bus_dma_map_sg,
	.unmap_sg = virtio_msg_bus_dma_unmap_sg,
	.sync_single_for_cpu = virtio_msg_bus_dma_sync_single_for_cpu,
	.sync_single_for_device = virtio_msg_bus_dma_sync_single_for_device,
	.sync_sg_for_cpu = virtio_msg_bus_dma_sync_sg_for_cpu,
	.sync_sg_for_device = virtio_msg_bus_dma_sync_sg_for_device,
	.dma_supported = virtio_msg_bus_dma_supported,
	.get_required_mask = virtio_msg_bus_dma_get_required_mask,
	.max_mapping_size = virtio_msg_bus_dma_max_mapping_size,
	.get_merge_boundary = virtio_msg_bus_dma_get_merge_boundary,
};

static struct device *
virtio_msg_bus_dma_delegate_from_map(union virtio_map map)
{
	struct virtio_msg_transport_device *vmdev;

	if (!map.dma_dev || !is_virtio_device(map.dma_dev))
		return map.dma_dev;

	vmdev = virtio_msg_bus_dma_vmdev_from_vdev(dev_to_virtio(map.dma_dev));
	if (!vmdev)
		return map.dma_dev;
	if (!virtio_msg_bus_dma_is_installed(vmdev))
		return map.dma_dev;

	return virtio_msg_bus_dma_delegate_dev(vmdev);
}

static dma_addr_t virtio_msg_bus_dma_vmap_page(union virtio_map map,
					       struct page *page,
					       unsigned long offset,
					       size_t size,
					       enum dma_data_direction dir,
					       unsigned long attrs)
{
	return dma_map_page_attrs(map.dma_dev, page, offset, size, dir, attrs);
}

static void virtio_msg_bus_dma_vunmap_page(union virtio_map map,
					   dma_addr_t map_handle, size_t size,
					   enum dma_data_direction dir,
					   unsigned long attrs)
{
	dma_unmap_page_attrs(map.dma_dev, map_handle, size, dir, attrs);
}

static void
virtio_msg_bus_dma_vsync_single_for_cpu(union virtio_map map,
					dma_addr_t map_handle, size_t size,
					enum dma_data_direction dir)
{
	dma_sync_single_for_cpu(map.dma_dev, map_handle, size, dir);
}

static void
virtio_msg_bus_dma_vsync_single_for_device(union virtio_map map,
					   dma_addr_t map_handle, size_t size,
					   enum dma_data_direction dir)
{
	dma_sync_single_for_device(map.dma_dev, map_handle, size, dir);
}

static void *virtio_msg_bus_dma_valloc(union virtio_map map, size_t size,
				       dma_addr_t *map_handle, gfp_t gfp)
{
	return dma_alloc_attrs(map.dma_dev, size, map_handle, gfp, 0);
}

static void virtio_msg_bus_dma_vfree(union virtio_map map, size_t size,
				     void *vaddr, dma_addr_t map_handle,
				     unsigned long attrs)
{
	dma_free_attrs(map.dma_dev, size, vaddr, map_handle, attrs);
}

static bool virtio_msg_bus_dma_vneed_sync(union virtio_map map,
					  dma_addr_t map_handle)
{
	struct virtio_msg_bus_dma *dma;
	struct virtio_msg_bus_dma_mapping *mapping;
	dma_addr_t backing_dma_addr;

	dma = virtio_msg_bus_dma_from_dev(map.dma_dev);
	if (dma && dma->shim_installed) {
		mapping = virtio_msg_bus_dma_lookup_export(dma, map_handle, 1);
		if (mapping &&
		    virtio_msg_bus_dma_backing_dma_handle(mapping, map_handle,
							  1,
							  &backing_dma_addr))
			map_handle = backing_dma_addr;
	}

	return dma_need_sync(virtio_msg_bus_dma_delegate_from_map(map),
			     map_handle);
}

static int virtio_msg_bus_dma_vmapping_error(union virtio_map map,
					     dma_addr_t map_handle)
{
	return dma_mapping_error(virtio_msg_bus_dma_delegate_from_map(map),
				 map_handle);
}

static size_t virtio_msg_bus_dma_vmax_mapping_size(union virtio_map map)
{
	return dma_max_mapping_size(virtio_msg_bus_dma_delegate_from_map(map));
}

static const struct virtio_map_ops virtio_msg_bus_dma_vmap_ops = {
	.map_page = virtio_msg_bus_dma_vmap_page,
	.unmap_page = virtio_msg_bus_dma_vunmap_page,
	.sync_single_for_cpu = virtio_msg_bus_dma_vsync_single_for_cpu,
	.sync_single_for_device = virtio_msg_bus_dma_vsync_single_for_device,
	.alloc = virtio_msg_bus_dma_valloc,
	.free = virtio_msg_bus_dma_vfree,
	.need_sync = virtio_msg_bus_dma_vneed_sync,
	.mapping_error = virtio_msg_bus_dma_vmapping_error,
	.max_mapping_size = virtio_msg_bus_dma_vmax_mapping_size,
};

static void
virtio_msg_bus_dma_uninstall(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_bus_dma *dma;
	struct device *dev;

	if (!vmdev)
		return;

	dma = vmdev->dma_shim;
	if (!dma || !dma->shim_installed)
		return;

	dev = &vmdev->vdev.dev;
	vmdev->vdev.map = dma->saved_virtio_map_ops;
	vmdev->vdev.vmap = dma->saved_vmap;
	set_dma_ops(dev, dma->saved_dma_ops);
	dev->dma_mask = dma->saved_dma_mask_ptr;
	dev->coherent_dma_mask = dma->saved_coherent_dma_mask;
	dev->bus_dma_limit = dma->saved_bus_dma_limit;
	dev->dma_range_map = dma->saved_dma_range_map;
	dev->dma_parms = dma->saved_dma_parms_ptr;
#if defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_DEVICE) || \
	defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU) || \
	defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU_ALL)
	dev->dma_coherent = dma->saved_dma_coherent;
#endif
#ifdef CONFIG_DMA_OPS_BYPASS
	dev->dma_ops_bypass = dma->saved_dma_ops_bypass;
#endif
#ifdef CONFIG_DMA_NEED_SYNC
	dev->dma_skip_sync = dma->saved_dma_skip_sync;
#endif
#ifdef CONFIG_IOMMU_DMA
	dev->dma_iommu = dma->saved_dma_iommu;
#endif
	dma->delegate_dev = NULL;
	dma->shim_installed = false;
}

static void
virtio_msg_bus_dma_provider_caps_init(struct virtio_msg_bus_dma *dma)
{
	if (!dma || !dma->provider_ops)
		return;

	memset(&dma->provider_caps, 0, sizeof(dma->provider_caps));
	if (dma->provider_ops->query_caps)
		(void)dma->provider_ops->query_caps(dma->provider_ctx,
						    &dma->provider_caps);
}

int virtio_msg_bus_dma_install(struct virtio_msg_transport_device *vmdev,
			       const struct virtio_msg_bus_dma_provider_ops *provider_ops,
			       void *provider_ctx)
{
	struct virtio_msg_bus_dma *dma;
	struct device *delegate_dev;
	struct device *dev;

	if (!vmdev || !provider_ops || !provider_ops->export_add ||
	    !provider_ops->export_del)
		return -EINVAL;
	if (virtio_msg_bus_dma_is_installed(vmdev))
		return 0;

	delegate_dev = vmdev->vdev.dev.parent;
	if (!delegate_dev)
		return -ENODEV;

	dma = kzalloc(sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;

	spin_lock_init(&dma->active_exports_lock);
	spin_lock_init(&dma->pool_lock);
	xa_init(&dma->active_exports);
	INIT_LIST_HEAD(&dma->deferred_local_frees);
	INIT_DELAYED_WORK(&dma->deferred_local_free_work,
			  virtio_msg_bus_dma_deferred_local_free_workfn);
	INIT_LIST_HEAD(&dma->pool_chunks);
	INIT_WORK(&dma->pool_grow_work, virtio_msg_bus_dma_pool_grow_work);
	atomic_set(&dma->pool_grow_min_pages, 0);
	dma->provider_ops = provider_ops;
	dma->provider_ctx = provider_ctx;
	virtio_msg_bus_dma_provider_caps_init(dma);

	dev = &vmdev->vdev.dev;
	dma->delegate_dev = delegate_dev;
	dma->saved_virtio_map_ops = vmdev->vdev.map;
	dma->saved_vmap = vmdev->vdev.vmap;
#ifdef CONFIG_ARCH_HAS_DMA_OPS
	dma->saved_dma_ops = dev->dma_ops;
#else
	dma->saved_dma_ops = NULL;
#endif
	dma->saved_dma_mask_ptr = dev->dma_mask;
	dma->saved_coherent_dma_mask = dev->coherent_dma_mask;
	dma->saved_bus_dma_limit = dev->bus_dma_limit;
	dma->saved_dma_range_map = dev->dma_range_map;
	dma->saved_dma_parms_ptr = dev->dma_parms;
#if defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_DEVICE) || \
	defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU) || \
	defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU_ALL)
	dma->saved_dma_coherent = dev->dma_coherent;
	dev->dma_coherent = dev_is_dma_coherent(delegate_dev);
#endif
#ifdef CONFIG_DMA_OPS_BYPASS
	dma->saved_dma_ops_bypass = dev->dma_ops_bypass;
	dev->dma_ops_bypass = false;
#endif
#ifdef CONFIG_DMA_NEED_SYNC
	dma->saved_dma_skip_sync = dev->dma_skip_sync;
	dev->dma_skip_sync = false;
#endif
#ifdef CONFIG_IOMMU_DMA
	dma->saved_dma_iommu = dev->dma_iommu;
	dev->dma_iommu = false;
#endif
	dma->dma_mask_storage = delegate_dev->dma_mask ?
				 *delegate_dev->dma_mask :
				 delegate_dev->coherent_dma_mask;
	dev->dma_mask = &dma->dma_mask_storage;
	dev->coherent_dma_mask = delegate_dev->coherent_dma_mask;
	dev->bus_dma_limit = delegate_dev->bus_dma_limit;
	dev->dma_range_map = delegate_dev->dma_range_map;
	if (delegate_dev->dma_parms) {
		dma->dma_parms_storage = *delegate_dev->dma_parms;
		dev->dma_parms = &dma->dma_parms_storage;
	} else {
		dev->dma_parms = NULL;
	}
	set_dma_ops(dev, &virtio_msg_bus_dma_map_ops);
	vmdev->vdev.map = &virtio_msg_bus_dma_vmap_ops;
	vmdev->vdev.vmap.dma_dev = dev;
	dma->shim_installed = true;
	vmdev->dma_shim = dma;

	virtio_msg_bus_dma_pool_init(dma);

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_install);

void virtio_msg_bus_dma_cleanup(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_bus_dma_mapping *mapping;
	struct virtio_msg_bus_dma *dma;
	int ret;
	unsigned long index;
	unsigned long flags;
	LIST_HEAD(deferred_local);

	if (!vmdev)
		return;

	dma = vmdev->dma_shim;
	if (!dma || !dma->shim_installed)
		return;

	cancel_delayed_work_sync(&dma->deferred_local_free_work);

	xa_for_each(&dma->active_exports, index, mapping) {
		xa_erase(&dma->active_exports, index);
		if (virtio_msg_bus_dma_mapping_needs_deferred_release(mapping)) {
			if (virtio_msg_bus_dma_mapping_is_shadow(mapping))
				virtio_msg_bus_dma_shadow_sync_for_cpu
					(dma, mapping,
					 mapping->key.dma_addr,
					 mapping->key.length,
					 mapping->backing.dir);
			list_add_tail(&mapping->deferred_node, &deferred_local);
			continue;
		}

		ret = virtio_msg_bus_dma_provider_export_del_record(dma, mapping,
								    false);
			if (ret && ret != -EOPNOTSUPP)
				vm_warn_rl(dma->delegate_dev,
					   "provider revoke failed dma=%pad len=%zu ret=%d\n",
					   &mapping->backing.dma_addr,
					   mapping->backing.length, ret);
		switch (mapping->kind) {
		case VIRTIO_MSG_BUS_DMA_MAPPING_COHERENT:
			dma_free_attrs(dma->delegate_dev,
				       mapping->backing.length,
				       mapping->cpu_addr,
				       mapping->backing.local_dma_addr,
				       mapping->attrs);
			break;
		case VIRTIO_MSG_BUS_DMA_MAPPING_STREAMING:
			if (virtio_msg_bus_dma_mapping_is_shadow(mapping))
				virtio_msg_bus_dma_shadow_sync_for_cpu
					(dma, mapping,
					 mapping->key.dma_addr,
					 mapping->key.length,
					 mapping->backing.dir);
			else
				dma_unmap_phys(dma->delegate_dev,
					       mapping->backing.local_dma_addr,
					       mapping->key.length,
					       mapping->backing.dir,
					       mapping->attrs);
			break;
		}
		virtio_msg_bus_dma_free_record(mapping);
	}

	spin_lock_irqsave(&dma->active_exports_lock, flags);
	list_splice_init(&dma->deferred_local_frees, &deferred_local);
	spin_unlock_irqrestore(&dma->active_exports_lock, flags);

	virtio_msg_bus_dma_cleanup_drain_local_frees(dma, &deferred_local);

	INIT_LIST_HEAD(&dma->deferred_local_frees);
	xa_destroy(&dma->active_exports);

	/* Free pre-registered pool chunks now that all active exports are gone. */
	virtio_msg_bus_dma_pool_cleanup(dma);

	virtio_msg_bus_dma_uninstall(vmdev);
	kfree(dma);
	vmdev->dma_shim = NULL;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_cleanup);
