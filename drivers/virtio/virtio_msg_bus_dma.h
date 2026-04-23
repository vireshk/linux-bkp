/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus DMA helper (Step 1 shell).
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_DMA_H
#define _VIRTIO_MSG_BUS_DMA_H

#include <linux/dma-mapping.h>

struct device;
struct page;
struct sg_table;
struct virtio_msg_transport_device;

struct virtio_msg_bus_dma_export {
	/*
	 * @dma_addr is the bus-visible handle presented to virtio consumers.
	 * @local_dma_addr is the delegate-facing backing DMA address used for
	 * sync, unmap, mmap, and sgtable operations.
	 */
	dma_addr_t dma_addr;
	dma_addr_t local_dma_addr;
	size_t length;
	size_t mmap_length;
	size_t page_offset;
	unsigned int npages;
	struct page **pages;
	const struct sg_table *sgt;
	struct device *dma_dev;
	void *cpu_addr;
	unsigned long attrs;
	enum dma_data_direction dir;
	bool coherent;
};

struct virtio_msg_bus_dma_provider_caps {
	/* reserved for future capability bits */
};

struct virtio_msg_bus_dma_provider_ops {
	/*
	 * export_add() may rewrite only @export->dma_addr to the bus-visible
	 * value. @export->local_dma_addr and the metadata fields must remain
	 * valid backing descriptors.
	 */
	int (*export_add)(void *ctx,
			  struct virtio_msg_bus_dma_export *export,
			  void **provider_handle, gfp_t gfp);
	int (*export_del)(void *ctx,
			  const struct virtio_msg_bus_dma_export *export,
			  void *provider_handle, bool sync);
	int (*query_caps)(void *ctx,
			  struct virtio_msg_bus_dma_provider_caps *caps);
};

int virtio_msg_bus_dma_install
		(struct virtio_msg_transport_device *vmdev,
		 const struct virtio_msg_bus_dma_provider_ops *provider_ops,
		 void *provider_ctx);
void virtio_msg_bus_dma_cleanup(struct virtio_msg_transport_device *vmdev);
bool virtio_msg_bus_dma_is_installed(const struct virtio_msg_transport_device *vmdev);
struct device *virtio_msg_bus_dma_delegate_dev(const struct virtio_msg_transport_device *vmdev);

#endif /* _VIRTIO_MSG_BUS_DMA_H */
