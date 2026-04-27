/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus DMA helper (Step 1 shell).
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_DMA_DRIVER_HELPER_H
#define _VIRTIO_MSG_BUS_DMA_DRIVER_HELPER_H

#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <uapi/linux/virtio_msg_bus_bridge.h>

struct device;
struct page;
struct sg_table;
struct virtio_msg_transport_device;

/*
 * Provider revoke is unresolved and the backing must remain pinned. Providers
 * should use this single errno when remote access may still be possible.
 */
#define VIRTIO_MSG_BUS_DMA_PROVIDER_REVOKE_IN_PROGRESS	(-EINPROGRESS)

enum virtio_msg_bus_dma_driver_export_type {
	VIRTIO_MSG_BUS_DMA_DRIVER_EXPORT_ONESHOT = 0,
	VIRTIO_MSG_BUS_DMA_DRIVER_EXPORT_POOL_CHUNK,
};

enum virtio_msg_bus_dma_driver_release_action {
	VIRTIO_MSG_BUS_DMA_DRIVER_RELEASE_IGNORED = 0,
	VIRTIO_MSG_BUS_DMA_DRIVER_RELEASE_RETAINED_IDLE,
	VIRTIO_MSG_BUS_DMA_DRIVER_RELEASE_RETAINED_ACTIVE,
};

struct virtio_msg_bus_dma_driver_release {
	enum virtio_msg_bus_dma_driver_release_action action;
	void *release_ctx;
	unsigned int active_suballocs;
};

struct virtio_msg_bus_dma_driver_export {
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
	enum virtio_msg_bus_dma_driver_export_type type;
	u32 bridge_flags;
	void *release_ctx;
};

struct virtio_msg_bus_dma_driver_provider_caps {
	/* reserved for future capability bits */
};

struct virtio_msg_bus_dma_driver_provider_ops {
	/*
	 * export_add() may rewrite only @export->dma_addr to the bus-visible
	 * value. @export->local_dma_addr and the metadata fields must remain
	 * valid backing descriptors.
	 */
	int (*export_add)(void *ctx,
			  struct virtio_msg_bus_dma_driver_export *export,
			  void **provider_handle, gfp_t gfp);
	int (*export_del)(void *ctx,
			  const struct virtio_msg_bus_dma_driver_export *export,
			  void *provider_handle, bool sync);
	int (*query_caps)(void *ctx,
			  struct virtio_msg_bus_dma_driver_provider_caps *caps);
};

int virtio_msg_bus_dma_driver_helper_install
		(struct virtio_msg_transport_device *vmdev,
		 const struct virtio_msg_bus_dma_driver_provider_ops *provider_ops,
		 void *provider_ctx);
void virtio_msg_bus_dma_driver_helper_cleanup
		(struct virtio_msg_transport_device *vmdev);
bool virtio_msg_bus_dma_driver_helper_is_installed
		(const struct virtio_msg_transport_device *vmdev);
struct device *virtio_msg_bus_dma_driver_helper_delegate_dev
		(const struct virtio_msg_transport_device *vmdev);
int virtio_msg_bus_dma_export_released
		(const struct virtio_msg_bus_dma_driver_export *export,
		 struct virtio_msg_bus_dma_driver_release *release);

#endif /* _VIRTIO_MSG_BUS_DMA_DRIVER_HELPER_H */
