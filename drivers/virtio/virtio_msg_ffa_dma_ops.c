// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtio message transport DMA OPS.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * The Virtio message transport allows virtio devices to be used over a virtual
 * virtio-msg channel. The channel interface is meant to be implemented using
 * the architecture specific hardware-assisted fast path, like ARM Firmware
 * Framework (FFA).
 */

#define pr_fmt(fmt) "virtio-msg-dma-ops: " fmt

#include <linux/arm_ffa.h>
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/of_reserved_mem.h>
#include <linux/virtio.h>
#include <linux/virtio_anchor.h>
#include <uapi/linux/virtio_msg.h>

#include "virtio_msg_internal.h"

static int virtio_msg_dma_supported(struct device *dev, u64 mask)
{
	return mask == DMA_BIT_MASK(64);
}

/* Reserved memory */
static void *virtio_msg_dma_alloc_rmem(struct device *dev, size_t size,
				       dma_addr_t *dma_handle, gfp_t gfp,
				       unsigned long attrs)
{
	size_t n_pages = PFN_UP(size);
	void *vaddr;
	int ret;

	vaddr = dma_direct_alloc(dev, size, dma_handle, gfp, attrs);
	if (!vaddr)
		return NULL;

	ret = vmsg_ffa_bus_area_share(to_ffa_dev(dev), dma_handle, n_pages);
	if (ret) {
		dma_direct_free(dev, size, vaddr, *dma_handle, attrs);
		return NULL;
	}

	return vaddr;
}

static void virtio_msg_dma_free_rmem(struct device *dev, size_t size,
				     void *vaddr, dma_addr_t dma_handle,
				     unsigned long attrs)
{
	int ret;

	ret = vmsg_ffa_bus_area_unshare(to_ffa_dev(dev), &dma_handle);
	if (ret)
		dev_err(dev, "%s: Failed to unshare area: %d", __func__, ret);

	dma_direct_free(dev, PFN_UP(size) << PAGE_SHIFT, vaddr, dma_handle, attrs);
}

static dma_addr_t virtio_msg_dma_map_phys_rmem(struct device *dev,
					       phys_addr_t phys, size_t size,
					       enum dma_data_direction dir,
					       unsigned long attrs)
{
	size_t n_pages = PFN_UP(offset_in_page(phys) + size);
	dma_addr_t dma_handle, swiotlb_dma;
	unsigned long dma_offset;

	if (WARN_ON(dir == DMA_NONE))
		return DMA_MAPPING_ERROR;

	if (!is_swiotlb_force_bounce(dev))
		return DMA_MAPPING_ERROR;

	swiotlb_dma = swiotlb_map(dev, phys, size, dir, attrs);
	if (swiotlb_dma == DMA_MAPPING_ERROR)
		return DMA_MAPPING_ERROR;

	dma_offset = offset_in_page(swiotlb_dma);
	dma_handle = swiotlb_dma - dma_offset;
	if (vmsg_ffa_bus_area_share(to_ffa_dev(dev), &dma_handle, n_pages)) {
		swiotlb_tbl_unmap_single(dev, dma_to_phys(dev, swiotlb_dma),
					 size, dir, attrs);
		return DMA_MAPPING_ERROR;
	}

	return dma_handle + dma_offset;
}

static void virtio_msg_dma_unmap_phys_rmem(struct device *dev,
					   dma_addr_t dma_handle, size_t size,
					   enum dma_data_direction dir,
					   unsigned long attrs)
{
	unsigned long dma_offset = offset_in_page(dma_handle);
	dma_addr_t swiotlb_dma;
	int ret;

	if (WARN_ON(dir == DMA_NONE))
		return;

	dma_handle -= dma_offset;

	ret = vmsg_ffa_bus_area_unshare(to_ffa_dev(dev), &dma_handle);
	if (ret)
		dev_err(dev, "%s: Failed to unshare area: %d", __func__, ret);

	swiotlb_dma = dma_handle + dma_offset;
	swiotlb_tbl_unmap_single(dev, dma_to_phys(dev, swiotlb_dma), size, dir,
				 attrs);
}

static void virtio_msg_dma_unmap_sg_rmem(struct device *dev,
					 struct scatterlist *sgl,
					 int nents, enum dma_data_direction dir,
					 unsigned long attrs)
{
	struct scatterlist *sg;
	unsigned int i;

	if (WARN_ON(dir == DMA_NONE))
		return;

	for_each_sg(sgl, sg, nents, i) {
		virtio_msg_dma_unmap_phys_rmem(dev, sg->dma_address,
					       sg_dma_len(sg), dir, attrs);
	}
}

static int virtio_msg_dma_map_sg_rmem(struct device *dev,
				      struct scatterlist *sgl,
				      int nents, enum dma_data_direction dir,
				      unsigned long attrs)
{
	struct scatterlist *sg;
	unsigned int i;

	if (WARN_ON(dir == DMA_NONE))
		return -EINVAL;

	for_each_sg(sgl, sg, nents, i) {
		sg->dma_address = virtio_msg_dma_map_phys_rmem(dev, sg_phys(sg),
							       sg->length, dir,
							       attrs);
		if (sg->dma_address == DMA_MAPPING_ERROR)
			goto out;

		sg_dma_len(sg) = sg->length;
	}

	return nents;

out:
	virtio_msg_dma_unmap_sg_rmem(dev, sgl, i, dir, attrs | DMA_ATTR_SKIP_CPU_SYNC);
	sg_dma_len(sgl) = 0;

	return -EIO;
}

const struct dma_map_ops virtio_msg_ffa_rmem_dma_ops = {
	.alloc = virtio_msg_dma_alloc_rmem,
	.free = virtio_msg_dma_free_rmem,
	.alloc_pages_op = dma_common_alloc_pages,
	.free_pages = dma_common_free_pages,
	.mmap = dma_common_mmap,
	.get_sgtable = dma_common_get_sgtable,
	.map_phys = virtio_msg_dma_map_phys_rmem,
	.unmap_phys = virtio_msg_dma_unmap_phys_rmem,
	.map_sg = virtio_msg_dma_map_sg_rmem,
	.unmap_sg = virtio_msg_dma_unmap_sg_rmem,
	.dma_supported = virtio_msg_dma_supported,
};

static bool virtio_msg_dma_ops_init(struct virtio_device *dev)
{
	/* DMA OPS should already be set by the underlying bus driver */
	if (dev->dev.parent->dma_ops)
		return true;

	return false;
}

int virtio_msg_ffa_dma_init(void)
{
	virtio_set_mem_acc_cb(virtio_msg_dma_ops_init);
	return 0;
}
