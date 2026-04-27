/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message FF-A bus protocol definitions (Step 2.1).
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 *
 * This header contains only FF-A-private bus protocol wire definitions.
 */

#ifndef _VIRTIO_MSG_BUS_FFA_PROTOCOL_H
#define _VIRTIO_MSG_BUS_FFA_PROTOCOL_H

#include <linux/bits.h>
#include <linux/types.h>
#include <linux/uuid.h>

/*
 * FF-A direct message payload capacity available to the virtio-msg FF-A
 * binding.
 */
#define FFA_BUS_MAX_MSG_SIZE				104

/* Protocol VERSION starts the FF-A-private bus message id range. */
#define FFA_BUS_MSG_VERSION				0x80
#define FFA_BUS_MSG_AREA_SHARE				0x81
#define FFA_BUS_MSG_AREA_UNSHARE			0x82
#define FFA_BUS_MSG_RESET				0x83
#define FFA_BUS_MSG_EVENT_POLL				0x84
#define FFA_BUS_MSG_EVENT_CONFIGURE			0x85
#define FFA_BUS_MSG_FIFO_CONFIGURE			0x86
#define FFA_BUS_MSG_ERROR				0x87
#define FFA_BUS_EVENT_AREA_RELEASE			0xc0

/* virtio-msg FF-A device-role UUID: endpoint discovery and driver TX target. */
#define VIRTIO_MSG_FFA_DEVICE_UUID \
	UUID_INIT(0xc66028b5, 0x2498, 0x4aa1, \
		  0x9d, 0xe7, 0x77, 0xda, 0x61, 0x22, 0xab, 0xf0)

/* virtio-msg FF-A driver-role UUID: endpoint discovery and device TX target. */
#define VIRTIO_MSG_FFA_DRIVER_UUID \
	UUID_INIT(0xbd7fd089, 0x6795, 0x472b, \
		  0xb4, 0x7f, 0xdb, 0x0c, 0x5d, 0x9a, 0x71, 0x9d)

enum virtio_msg_ffa_bus_result {
	VIRTIO_MSG_FFA_BUS_SUCCESS = 0,
	VIRTIO_MSG_FFA_BUS_ERROR = 1,
	VIRTIO_MSG_FFA_BUS_BUSY = 2,
};

enum virtio_msg_ffa_bus_event_delivery {
	VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL = 0,
	VIRTIO_MSG_FFA_BUS_EVENT_DELIV_NOTIF = 1,
	VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT = 2,
	VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO = 3,
};

#define VIRTIO_MSG_FFA_BUS_FEATURE_DIRECT_RX		BIT(0)
#define VIRTIO_MSG_FFA_BUS_FEATURE_DIRECT_TX		BIT(1)
#define VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_RX		BIT(2)
#define VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_TX		BIT(3)
#define VIRTIO_MSG_FFA_BUS_FEATURE_NOTIF_RX		BIT(4)
#define VIRTIO_MSG_FFA_BUS_FEATURE_NOTIF_TX		BIT(5)
#define VIRTIO_MSG_FFA_BUS_FEATURE_FIFO			BIT(6)

#define VIRTIO_MSG_FFA_AREA_ATTR_TYPE_SHIFT		0
#define VIRTIO_MSG_FFA_AREA_ATTR_WRITEABLE		BIT(2)
#define VIRTIO_MSG_FFA_AREA_ATTR_EXECUTABLE		BIT(3)
#define VIRTIO_MSG_FFA_AREA_ATTR_SHAREABILITY_SHIFT	4
#define VIRTIO_MSG_FFA_AREA_ATTR_CACHEABILITY_SHIFT	6
#define VIRTIO_MSG_FFA_AREA_ATTR_MEM_TYPE_SHIFT		8
#define VIRTIO_MSG_FFA_AREA_ATTR_NON_SECURE		BIT(10)
#define VIRTIO_MSG_FFA_AREA_ATTR_DRIVER_RETENTION_REQUESTED	BIT(11)

enum virtio_msg_ffa_area_attr_type {
	VIRTIO_MSG_FFA_AREA_ATTR_TYPE_SHARE = 0,
	VIRTIO_MSG_FFA_AREA_ATTR_TYPE_LEND = 1,
	VIRTIO_MSG_FFA_AREA_ATTR_TYPE_DONATE = 2,
};

enum virtio_msg_ffa_area_attr_shareability {
	VIRTIO_MSG_FFA_AREA_ATTR_SHAREABILITY_NON = 0,
	VIRTIO_MSG_FFA_AREA_ATTR_SHAREABILITY_OUTER = 2,
	VIRTIO_MSG_FFA_AREA_ATTR_SHAREABILITY_INNER = 3,
};

enum virtio_msg_ffa_area_attr_cacheability {
	VIRTIO_MSG_FFA_AREA_ATTR_CACHEABILITY_NON = 1,
	VIRTIO_MSG_FFA_AREA_ATTR_CACHEABILITY_WB = 3,
};

enum virtio_msg_ffa_area_attr_mem_type {
	VIRTIO_MSG_FFA_AREA_ATTR_MEM_TYPE_NOT_SPECIFIED = 0,
	VIRTIO_MSG_FFA_AREA_ATTR_MEM_TYPE_DEVICE = 1,
	VIRTIO_MSG_FFA_AREA_ATTR_MEM_TYPE_NORMAL = 2,
};

#define VIRTIO_MSG_FFA_AREA_SHARING_ATTR_SHARE_RW_XN_INNER_WB_NORMAL_NS \
	((VIRTIO_MSG_FFA_AREA_ATTR_TYPE_SHARE <<		\
	  VIRTIO_MSG_FFA_AREA_ATTR_TYPE_SHIFT) |			\
	 VIRTIO_MSG_FFA_AREA_ATTR_WRITEABLE |			\
	 (VIRTIO_MSG_FFA_AREA_ATTR_SHAREABILITY_INNER <<	\
	  VIRTIO_MSG_FFA_AREA_ATTR_SHAREABILITY_SHIFT) |		\
	 (VIRTIO_MSG_FFA_AREA_ATTR_CACHEABILITY_WB <<		\
	  VIRTIO_MSG_FFA_AREA_ATTR_CACHEABILITY_SHIFT) |		\
	 (VIRTIO_MSG_FFA_AREA_ATTR_MEM_TYPE_NORMAL <<		\
	  VIRTIO_MSG_FFA_AREA_ATTR_MEM_TYPE_SHIFT) |		\
	 VIRTIO_MSG_FFA_AREA_ATTR_NON_SECURE)

#define VIRTIO_MSG_FFA_BUS_ADDR_AREA_SHIFT		48
#define VIRTIO_MSG_FFA_BUS_ADDR_OFFSET_MASK		GENMASK_ULL(47, 0)

/*
 * FF-A bus addresses are valid only for shared-area memory.
 * FIFO memory is handle-based and does not use this address encoding.
 */
static inline u64 virtio_msg_ffa_bus_addr_pack(u16 area_id, u64 offset)
{
	return ((u64)area_id << VIRTIO_MSG_FFA_BUS_ADDR_AREA_SHIFT) |
	       (offset & VIRTIO_MSG_FFA_BUS_ADDR_OFFSET_MASK);
}

static inline u16 virtio_msg_ffa_bus_addr_area_id(u64 bus_addr)
{
	return bus_addr >> VIRTIO_MSG_FFA_BUS_ADDR_AREA_SHIFT;
}

static inline u64 virtio_msg_ffa_bus_addr_offset(u64 bus_addr)
{
	return bus_addr & VIRTIO_MSG_FFA_BUS_ADDR_OFFSET_MASK;
}

struct virtio_msg_ffa_version_req {
	__le32 bus_version;
	__le32 transport_revision;
	__le32 ffa_version;
} __packed;

struct virtio_msg_ffa_version_resp {
	__le32 bus_version;
	__le32 transport_revision;
	__le32 ffa_version;
	__le32 feature_bits;
	__le32 bus_features;
	__le16 max_areas;
} __packed;

struct virtio_msg_ffa_area_share_req {
	__le16 area_id;
	__le64 mem_handle;
	__le64 mem_tag;
	__le32 page_count;
	__le32 sharing_attrs;
} __packed;

struct virtio_msg_ffa_area_share_resp {
	__le16 area_id;
	__le16 result;
} __packed;

struct virtio_msg_ffa_area_unshare_req {
	__le16 area_id;
} __packed;

struct virtio_msg_ffa_area_unshare_resp {
	__le16 area_id;
	__le16 result;
} __packed;

struct virtio_msg_ffa_reset_resp {
	__le16 result;
} __packed;

struct virtio_msg_ffa_event_configure_req {
	__u8 event_method;
	__u8 reserved;
	__le16 notif_id;
} __packed;

struct virtio_msg_ffa_event_configure_resp {
	__le16 result;
} __packed;

struct virtio_msg_ffa_fifo_configure_req {
	__le64 mem_handle;
	__le16 page_count;
	__le16 driver_notif_id;
} __packed;

struct virtio_msg_ffa_fifo_configure_resp {
	__le16 result;
	__le16 device_notif_id;
} __packed;

struct virtio_msg_ffa_error_resp {
	__le16 original_msg_id;
} __packed;

struct virtio_msg_ffa_area_release_event {
	__le16 area_id;
} __packed;

#define VIRTIO_MSG_FFA_FIFO_MAGIC			"VFFAFIFO"
#define VIRTIO_MSG_FFA_FIFO_VERSION			0
#define VIRTIO_MSG_FFA_FIFO_HEADER_SIZE			0x00c0
#define VIRTIO_MSG_FFA_FIFO_MSG_AREA_OFFSET		0x00c0

struct virtio_msg_ffa_fifo_hdr {
	__u8 magic[8];
	__le16 version;
	__u8 reserved_after_version[6];
	__le16 message_size;
	__le16 depth;
	__u8 reserved_after_depth[4];
	__le32 next_offset;
	__u8 reserved_before_read_index[36];
	__le16 read_index;
	__u8 reserved_after_read_index[2];
	__u8 reserved_before_write_index[60];
	__le16 write_index;
	__u8 reserved_after_write_index[2];
	__u8 reserved_tail[60];
} __packed;

#endif /* _VIRTIO_MSG_BUS_FFA_PROTOCOL_H */
