/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message transport protocol definitions.
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 * Written by Edgar E. Iglesias <edgar.iglesias@amd.com>
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 *
 * Step 4 keeps this header intentionally minimal and contract-focused.
 * Runtime transport internals are introduced in later steps.
 */

#ifndef _LINUX_VIRTIO_MSG_PROTOCOL_H
#define _LINUX_VIRTIO_MSG_PROTOCOL_H

#include <linux/bits.h>
#include <linux/types.h>

#define VIRTIO_MSG_GET_DEVICE_INFO		0x02
#define VIRTIO_MSG_GET_DEVICE_FEATURES		0x03
#define VIRTIO_MSG_SET_DRIVER_FEATURES		0x04
#define VIRTIO_MSG_GET_CONFIG			0x05
#define VIRTIO_MSG_SET_CONFIG			0x06
#define VIRTIO_MSG_GET_DEVICE_STATUS		0x07
#define VIRTIO_MSG_SET_DEVICE_STATUS		0x08
#define VIRTIO_MSG_GET_VQUEUE			0x09
#define VIRTIO_MSG_SET_VQUEUE			0x0a
#define VIRTIO_MSG_RESET_VQUEUE			0x0b
#define VIRTIO_MSG_GET_SHM			0x0c
#define VIRTIO_MSG_EVENT_CONFIG			0x40
#define VIRTIO_MSG_EVENT_AVAIL			0x41
#define VIRTIO_MSG_EVENT_USED			0x42

#define VIRTIO_MSG_MAX				VIRTIO_MSG_EVENT_USED
#define VIRTIO_MSG_MIN_SIZE			52
#define VIRTIO_MSG_MAX_SIZE			65535
/*
 * Revision 1 is the baseline virtio-message transport revision defined by
 * specifications/virtio-msg-transport.tex.
 */
#define VIRTIO_MSG_REVISION_1			0x1

#define VIRTIO_MSG_TYPE_REQUEST			0
#define VIRTIO_MSG_TYPE_RESPONSE		BIT(0)
#define VIRTIO_MSG_TYPE_TRANSPORT		0
#define VIRTIO_MSG_TYPE_BUS			BIT(1)
#define VIRTIO_MSG_ID_EVENT_BIT			BIT(6)

/**
 * struct virtio_msg - Generic virtio-msg frame header.
 * @type: Frame type and class flags.
 * @msg_id: Message opcode.
 * @dev_num: Device identifier for transport-class messages.
 * @token: Request/response correlation token.
 * @msg_size: Total frame size in bytes, header included.
 * @payload: Variable payload bytes.
 */
struct virtio_msg {
	__u8 type;
	__u8 msg_id;
	__le16 dev_num;
	__le16 token;
	__le16 msg_size;
	__u8 payload[];
} __packed;

static inline void *virtio_msg_payload(struct virtio_msg *vmsg)
{
	return &vmsg->payload;
}

struct virtio_msg_get_device_info_resp {
	__le32 device_id;
	__le32 vendor_id;
	__u8 device_uuid[16];
	__le32 num_feature_blocks;
	__le32 config_size;
	__le32 max_virtqueues;
	__le32 admin_vq_start;
	__le32 admin_vq_count;
} __packed;

struct virtio_msg_get_device_features_req {
	__le32 block_index;
	__le32 num_blocks;
} __packed;

struct virtio_msg_get_device_features_resp {
	__le32 block_index;
	__le32 num_blocks;
	__le32 features[];
} __packed;

struct virtio_msg_set_driver_features {
	__le32 block_index;
	__le32 num_blocks;
	__le32 features[];
} __packed;

struct virtio_msg_get_config {
	__le32 offset;
	__le32 length;
} __packed;

struct virtio_msg_get_config_resp {
	__le32 generation;
	__le32 offset;
	__le32 length;
	__u8 config[];
} __packed;

struct virtio_msg_set_config {
	__le32 generation;
	__le32 offset;
	__le32 length;
	__u8 config[];
} __packed;

struct virtio_msg_set_config_resp {
	__le32 generation;
	__le32 offset;
	__le32 length;
	__u8 config[];
} __packed;

struct virtio_msg_get_device_status_resp {
	__le32 status;
} __packed;

struct virtio_msg_set_device_status {
	__le32 status;
} __packed;

struct virtio_msg_set_device_status_resp {
	__le32 status;
} __packed;

struct virtio_msg_get_vqueue {
	__le32 index;
} __packed;

struct virtio_msg_get_vqueue_resp {
	__le32 index;
	__le32 max_size;
	__le32 cur_size;
	__le32 flags;
	__le64 descriptor_addr;
	__le64 driver_addr;
	__le64 device_addr;
} __packed;

#define VIRTIO_MSG_VQUEUE_F_ENABLED		BIT(0)
#define VIRTIO_MSG_SET_VQUEUE_STATE_OP_ENABLE	1

struct virtio_msg_set_vqueue {
	__le32 index;
	__le32 flags;
	__le32 size;
	__le32 reserved;
	__le64 descriptor_addr;
	__le64 driver_addr;
	__le64 device_addr;
} __packed;

struct virtio_msg_reset_vqueue {
	__le32 index;
} __packed;

struct virtio_msg_get_shm_req {
	__le32 shmid;
} __packed;

struct virtio_msg_get_shm_resp {
	__le32 shmid;
	__le32 reserved;
	__le64 length;
	__le64 address;
} __packed;

struct virtio_msg_event_config {
	__le32 device_status;
	__le32 generation;
	__le32 offset;
	__le32 length;
	__u8 data[];
} __packed;

struct virtio_msg_event_avail {
	__le32 vq_index;
	__le32 next_offset;
} __packed;

struct virtio_msg_event_used {
	__le32 vq_index;
} __packed;

#define VIRTIO_MSG_BUS_GET_DEVICES		0x02
#define VIRTIO_MSG_BUS_PING			0x03
#define VIRTIO_MSG_BUS_EVENT_DEVICE		0x40

#define VIRTIO_MSG_BUS_EVENT_DEV_STATE_ADDED	0x1
#define VIRTIO_MSG_BUS_EVENT_DEV_STATE_READY	\
	VIRTIO_MSG_BUS_EVENT_DEV_STATE_ADDED
#define VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED	0x2

struct virtio_msg_bus_get_devices {
	__le16 offset;
	__le16 count;
} __packed;

struct virtio_msg_bus_get_devices_resp {
	__le16 offset;
	__le16 next_offset;
	__le16 count;
	__u8 bitmap[];
} __packed;

struct virtio_msg_bus_event_device {
	__le16 dev_num;
	__le16 dev_state;
} __packed;

struct virtio_msg_bus_ping {
	__le32 data;
} __packed;

struct virtio_msg_bus_ping_resp {
	__le32 data;
} __packed;

#endif /* _LINUX_VIRTIO_MSG_PROTOCOL_H */
