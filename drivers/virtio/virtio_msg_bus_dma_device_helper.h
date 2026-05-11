/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus bridge/device-side DMA helper.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_DMA_DEVICE_HELPER_H
#define _VIRTIO_MSG_BUS_DMA_DEVICE_HELPER_H

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/virtio_msg_bus_bridge_device.h>
#include <linux/xarray.h>

enum virtio_msg_bus_dma_device_map_state {
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_NONE = 0,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_QUEUED,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ACTIVE,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_QUEUED,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_REMOTE_RELEASED,
};

enum virtio_msg_bus_dma_device_map_action {
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_IGNORED = 0,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_ACTIVE,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_DEL_QUEUED,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_FAILED,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_DONE,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED_DEL_DONE,
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_REMOVED,
};

#define VIRTIO_MSG_BUS_DMA_DEVICE_ACK_IGNORED \
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_IGNORED
#define VIRTIO_MSG_BUS_DMA_DEVICE_ACK_ADD_ACTIVE \
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_ACTIVE
#define VIRTIO_MSG_BUS_DMA_DEVICE_ACK_ADD_DEL_QUEUED \
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_DEL_QUEUED
#define VIRTIO_MSG_BUS_DMA_DEVICE_ACK_DEL_DONE \
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_DONE
#define VIRTIO_MSG_BUS_DMA_DEVICE_ACK_REMOVED \
	VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_FAILED

struct virtio_msg_bus_dma_device_helper {
	struct mutex lock; /* Protects both map indexes. */
	struct xarray maps_by_id;
	struct xarray maps_by_mmap_offset;
};

struct virtio_msg_bus_dma_device_map_desc {
	u64 bus_addr;
	u64 length;
	u64 mmap_offset;
	u64 mmap_length;
	u32 flags;
	void *cookie;
};

struct virtio_msg_bus_dma_device_map_info {
	u64 map_id;
	u64 map_seq;
	u64 bus_addr;
	u64 length;
	u64 mmap_offset;
	u64 mmap_length;
	u32 flags;
	enum virtio_msg_bus_dma_device_map_state state;
	void *cookie;
};

struct virtio_msg_bus_dma_device_map_result {
	enum virtio_msg_bus_dma_device_map_action action;
	enum virtio_msg_bus_dma_device_map_state old_state;
	enum virtio_msg_bus_dma_device_map_state new_state;
	struct virtio_msg_bus_dma_device_map_info map;
};

#define virtio_msg_bus_dma_device_ack virtio_msg_bus_dma_device_map_result

void virtio_msg_bus_dma_device_helper_init(struct virtio_msg_bus_dma_device_helper *helper);
void virtio_msg_bus_dma_device_helper_destroy(struct virtio_msg_bus_dma_device_helper *helper);

int virtio_msg_bus_dma_device_helper_map_add(struct virtio_msg_bus_dma_device_helper *helper,
					     u32 handle,
					     const struct virtio_msg_bus_dma_device_map_desc *desc,
					     u64 *map_id);
int virtio_msg_bus_dma_device_helper_map_del_req(struct virtio_msg_bus_dma_device_helper *helper,
						 u32 handle, u64 map_id);
int
virtio_msg_bus_dma_device_helper_map_add_response(struct virtio_msg_bus_dma_device_helper *helper,
						  const struct vmsg_bridge_uapi_map_event *event,
						  struct virtio_msg_bus_dma_device_map_result *res);
int
virtio_msg_bus_dma_device_helper_map_del_response(struct virtio_msg_bus_dma_device_helper *helper,
						  const struct vmsg_bridge_uapi_map_event *event,
						  struct virtio_msg_bus_dma_device_map_result *res);
int
virtio_msg_bus_dma_device_helper_map_released(struct virtio_msg_bus_dma_device_helper *helper,
					      const struct vmsg_bridge_uapi_map_event *event,
					      struct virtio_msg_bus_dma_device_map_result *result);
int virtio_msg_bus_dma_device_helper_map_event_ack(struct virtio_msg_bus_dma_device_helper *helper,
						   const struct vmsg_bridge_uapi_map_event *event,
						   u32 ack_status,
						   struct virtio_msg_bus_dma_device_ack *ack);

int virtio_msg_bus_dma_device_helper_lookup_by_id(struct virtio_msg_bus_dma_device_helper *helper,
						  u64 map_id,
						  struct virtio_msg_bus_dma_device_map_info *map);
int
virtio_msg_bus_dma_device_helper_lookup_by_mmap_offset(struct virtio_msg_bus_dma_device_helper *h,
						       u64 mmap_offset,
						       u64 mmap_length,
						       struct virtio_msg_bus_dma_device_map_info
						       *map);
void
virtio_msg_bus_dma_device_helper_purge(struct virtio_msg_bus_dma_device_helper *helper,
				       void (*purge)(const struct virtio_msg_bus_dma_device_map_info
						     *map, void *data),
				       void *data);

#endif /* _VIRTIO_MSG_BUS_DMA_DEVICE_HELPER_H */
