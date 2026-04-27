/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus DMA helper private xarray utilities.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_DMA_RECORD_H
#define _VIRTIO_MSG_BUS_DMA_RECORD_H

#include <linux/errno.h>
#include <linux/gfp.h>
#include <linux/types.h>
#include <linux/xarray.h>

static inline int
virtio_msg_bus_dma_xa_index_from_u64(u64 value, unsigned long *index)
{
	unsigned long candidate;

	if (!index)
		return -EINVAL;

	candidate = (unsigned long)value;
	if ((u64)candidate != value)
		return -EOVERFLOW;

	*index = candidate;
	return 0;
}

static inline int
virtio_msg_bus_dma_xa_insert_unique(struct xarray *xa, unsigned long index,
				    void *entry, gfp_t gfp)
{
	int ret;

	ret = xa_insert(xa, index, entry, gfp);
	if (ret == -EBUSY)
		return -EEXIST;

	return ret;
}

static inline int
virtio_msg_bus_dma_xa_insert_unique_locked(struct xarray *xa,
					   unsigned long index, void *entry,
					   gfp_t gfp)
{
	int ret;

	ret = __xa_insert(xa, index, entry, gfp);
	if (ret == -EBUSY)
		return -EEXIST;

	return ret;
}

static inline bool
virtio_msg_bus_dma_xa_erase_if_match(struct xarray *xa, unsigned long index,
				     const void *entry)
{
	if (xa_load(xa, index) != entry)
		return false;

	return xa_erase(xa, index) == entry;
}

static inline bool
virtio_msg_bus_dma_xa_erase_if_match_locked(struct xarray *xa,
					    unsigned long index,
					    const void *entry)
{
	if (xa_load(xa, index) != entry)
		return false;

	return __xa_erase(xa, index) == entry;
}

#endif /* _VIRTIO_MSG_BUS_DMA_RECORD_H */
