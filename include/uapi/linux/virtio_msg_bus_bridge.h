/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Userspace ABI for virtio-msg bus bridge.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _UAPI_LINUX_VIRTIO_MSG_BUS_BRIDGE_H
#define _UAPI_LINUX_VIRTIO_MSG_BUS_BRIDGE_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define VMSG_BRIDGE_UAPI_VERSION			1

#define VMSG_BRIDGE_UAPI_RING_ENTRY_ALIGN		128U
#define VMSG_BRIDGE_UAPI_RING_ENTRIES_DEFAULT	256U

#define VMSG_BRIDGE_UAPI_CAP_F_ENDPOINT_ONLINE		(1U << 0)
#define VMSG_BRIDGE_UAPI_BUS_NAME_LEN		16U
#define VMSG_BRIDGE_UAPI_BUS_ID_LEN		48U
#define VMSG_BRIDGE_UAPI_RESOLVE_F_CREATE		(1U << 0)

#define VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_REMOVED	1U
#define VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_RESET	2U

/*
 * Bridge-local virtio-msg bus message IDs.
 *
 * These values are carried in the virtio-msg bus-frame msg_id field. The
 * generic virtio-msg frame header, type bits, token, dev_num and msg_size
 * fields are defined by the virtio-msg transport protocol.
 */
#define VIRTIO_MSG_BUS_BRIDGE_ERROR		0x88
#define VIRTIO_MSG_BUS_BRIDGE_MAP_ADD		0x89
#define VIRTIO_MSG_BUS_BRIDGE_MAP_DEL		0x8a
#define VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_ONLINE	0xc0
#define VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE	0xc1
#define VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_RESET	0xc2
#define VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_DEV_ADD	0xc3
#define VIRTIO_MSG_BUS_BRIDGE_TOPOLOGY_COMMIT	0xc4
#define VIRTIO_MSG_BUS_BRIDGE_MAP_RELEASED	0xc5

#define VIRTIO_MSG_BUS_BRIDGE_MAP_F_RETENTION_REQUESTED	(1U << 0)

struct virtio_msg_bus_bridge_endpoint_online {
	__le64 online_epoch;
	char bus_name[16];
	__le32 max_msg_size;
	__le32 reserved;
};

struct virtio_msg_bus_bridge_endpoint_offline {
	__le64 online_epoch;
	__le32 reason;
	__le32 reserved;
};

struct virtio_msg_bus_bridge_topology_reset {
	__le64 online_epoch;
	__le64 reserved;
};

struct virtio_msg_bus_bridge_topology_dev_add {
	__le64 online_epoch;
	__le16 dev_num;
	__u8 reserved[6];
};

struct virtio_msg_bus_bridge_topology_commit {
	__le64 online_epoch;
	__le16 num_devs;
	__u8 reserved[6];
};

struct virtio_msg_bus_bridge_map_add_req {
	__le64 online_epoch;
	__le64 map_seq;
	__le64 map_id;
	__le64 bus_addr;
	__le64 length;
	__le64 mmap_offset;
	__le64 mmap_length;
	__le32 flags;
	/* Reserved for future use, must be zero. */
	__le32 reserved;
};

struct virtio_msg_bus_bridge_map_del_req {
	__le64 online_epoch;
	__le64 map_seq;
	__le64 map_id;
	__le64 bus_addr;
	__le64 length;
	/* Reserved for future use, must be zero. */
	__le64 reserved;
};

struct virtio_msg_bus_bridge_map_add_resp {
	__le64 online_epoch;
	__le64 map_seq;
	__le64 map_id;
	/* 0 or signed negative Linux errno, little-endian two's-complement. */
	__le32 status;
	/* Reserved for future use, must be zero. */
	__le32 reserved;
};

struct virtio_msg_bus_bridge_map_del_resp {
	__le64 online_epoch;
	__le64 map_seq;
	__le64 map_id;
	/* 0 or signed negative Linux errno, little-endian two's-complement. */
	__le32 status;
	/* Reserved for future use, must be zero. */
	__le32 reserved;
};

struct virtio_msg_bus_bridge_map_released {
	__le64 online_epoch;
	__le64 map_seq;
	__le64 map_id;
	__le64 bus_addr;
	__le64 length;
};

struct virtio_msg_bus_bridge_error {
	__le16 original_dev_num;
	__u8 original_msg_id;
	__u8 reserved0;
	/* Signed negative Linux errno, little-endian two's-complement. */
	__le32 error;
};

/*
 * ENDPOINT_OFFLINE is a lifecycle transition for the current attach epoch, not
 * an implicit detach. Userspace keeps the file descriptor attached, stops
 * endpoint-scoped traffic, and waits for the next ENDPOINT_ONLINE or detaches
 * voluntarily.
 */

/**
 * struct vmsg_bridge_uapi_endpoint_addr - bus-scoped endpoint address
 * @bus_name: NUL-terminated bus implementation name (for example "ffa" or
 *	      "loopback"). Unused trailing bytes must be zero.
 * @bus_id: Bus-specific endpoint identifier string. Must be NUL-terminated
 *	    within this field and zero-padded after NUL.
 */
struct vmsg_bridge_uapi_endpoint_addr {
	char bus_name[VMSG_BRIDGE_UAPI_BUS_NAME_LEN];
	char bus_id[VMSG_BRIDGE_UAPI_BUS_ID_LEN];
};

/**
 * struct vmsg_bridge_uapi_resolve_endpoint - resolve bus-scoped address to handle
 * @addr: Bus-scoped endpoint address (input).
 * @handle: Kernel-assigned endpoint handle (output on success).
 * @flags: Resolve flags (%VMSG_BRIDGE_UAPI_RESOLVE_F_*).
 * @reserved: Reserved for future use, must be zero.
 *
 * Successful resolves return a non-zero @handle. The returned handle is used
 * in all endpoint-scoped ioctls through existing @endpoint_id fields.
 */
struct vmsg_bridge_uapi_resolve_endpoint {
	struct vmsg_bridge_uapi_endpoint_addr addr;
	__u32 handle;
	__u32 flags;
	__u64 reserved[2];
};

/**
 * struct vmsg_bridge_uapi_release_endpoint - release one resolved endpoint handle
 * @handle: Handle returned by %VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT.
 * @flags: Release flags, must be zero.
 * @reserved: Reserved for future use, must be zero.
 */
struct vmsg_bridge_uapi_release_endpoint {
	__u32 handle;
	__u32 flags;
	__u64 reserved;
};

/**
 * struct vmsg_bridge_uapi_ring_hdr - shared ring producer/consumer header
 * @prod: Producer cursor (monotonic u32, incremented by 1 per produced frame).
 * @cons: Consumer cursor (monotonic u32, incremented by 1 per consumed frame).
 * @entries: Ring slot count.
 * @entry_size: Size of each ring slot in bytes.
 * @flags: Ring flags, must be zero.
 * @reserved: Reserved for future use, must be zero.
 *
 * Ring cursor contract:
 * - @prod/@cons are absolute monotonic cursors, not modulo indices.
 * - Slot index is derived as (cursor %% @entries).
 * - Ring occupancy is (@prod - @cons) in unsigned 32-bit arithmetic.
 * - Valid occupancy range is 0..@entries.
 * - Empty ring: @prod == @cons.
 * - Full ring: (@prod - @cons) == @entries.
 */
struct vmsg_bridge_uapi_ring_hdr {
	__u32 prod;
	__u32 cons;
	__u32 entries;
	__u32 entry_size;
	__u32 flags;
	__u32 reserved;
};

/**
 * struct vmsg_bridge_uapi_ring_info - location and geometry of one shared ring
 * @offset: Byte offset within ring backing file descriptor.
 * @bytes: Ring mapping size in bytes.
 * @entries: Ring slot count. Must be %VMSG_BRIDGE_UAPI_RING_ENTRIES_DEFAULT.
 * @entry_size: Size of each slot in bytes. Must be the attach-time
 *		vmm_max_msg_size input rounded up to
 *		%VMSG_BRIDGE_UAPI_RING_ENTRY_ALIGN bytes.
 * @reserved: Reserved for future use, must be zero.
 */
struct vmsg_bridge_uapi_ring_info {
	__u64 offset;
	__u64 bytes;
	__u32 entries;
	__u32 entry_size;
	__u64 reserved;
};

/*
 * TX/RX ring slot payload contract:
 * - Each slot carries exactly one raw virtio-msg frame. The frame may be a
 *   transport-class frame or a bridge-local implementation-defined bus frame.
 * - Frame bytes start at slot offset 0.
 * - msg_size is read from struct virtio_msg header inside the frame.
 * - Remaining slot bytes are ignored.
 * - TX ring producer is userspace; TX ring consumer is kernel.
 * - RX ring producer is kernel; RX ring consumer is userspace.
 * - Bridge-local MAP_ADD and MAP_DEL frames use normal request/response
 *   completion. MAP_RELEASED is the only bridge-local map event.
 * - Standardized bus frames and reserved bus-message IDs on bridge rings are
 *   discarded by the bridge layer.
 * - Malformed virtio-msg headers and malformed bridge-local payload sizes
 *   fault the attached session.
 * - Producer publishes payload before updating @prod (release ordering).
 * - Consumer reads @prod with acquire ordering before reading payload.
 * - Consumer updates @cons only after finishing slot processing.
 */

/**
 * struct vmsg_bridge_uapi_caps - negotiated bridge capabilities
 * @uapi_version: Negotiated UAPI version (%VMSG_BRIDGE_UAPI_VERSION).
 * @revision: Transport revision.
 * @max_msg_size: Negotiated maximum raw virtio-msg frame size for both
 *		   transport-class frames and bridge-local bus frames.
 * @flags: Output flags (%VMSG_BRIDGE_UAPI_CAP_F_*).
 * @transport_features: Supported transport feature bits.
 * @bridge_features: Supported bridge UAPI feature bits.
 * @online_epoch: Current online epoch, or zero when the attach snapshot is
 *		  offline.
 * @reserved1: Reserved for future use, must be zero.
 */
struct vmsg_bridge_uapi_caps {
	__u32 uapi_version;
	__u32 revision;
	__u32 max_msg_size;
	__u32 flags;
	__u64 transport_features;
	__u64 bridge_features;
	__u64 online_epoch;
	__u64 reserved1;
};

/**
 * struct vmsg_bridge_uapi_attach - attach one userspace peer to one endpoint
 * @endpoint_id: Handle returned by %VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT.
 * @flags: Attach flags, must be zero.
 * @ring_fd: Shared memory file descriptor backing TX/RX rings. Must refer
 *           to a shmem-backed file (for example memfd_create() or tmpfs).
 * @kick_fd: Eventfd signaled by userspace when ring progress is made.
 *	     Userspace may signal this eventfd after advancing @tx.prod to
 *	     post TX work or after advancing @rx.cons to free RX slots.
 * @call_fd: Eventfd signaled by kernel when userspace should inspect RX ring
 *	     state or session status.
 * @vmm_max_msg_size: VMM maximum raw virtio-msg frame size for this attach
 *		      epoch. Must be in the
 *		      %VIRTIO_MSG_MIN_SIZE..%VIRTIO_MSG_MAX_SIZE range. Ring
 *		      slot sizing is derived from this value. It must also be
 *		      large enough to carry a raw
 *		      %VIRTIO_MSG_BUS_BRIDGE_MAP_ADD request or response frame,
 *		      including the virtio-msg header.
 * @tx: TX ring geometry.
 * @rx: RX ring geometry.
 * @caps: Negotiated capabilities filled by kernel. Userspace passes this
 *	  structure zero-initialized.
 * @reserved1: Reserved for future use, must be zero.
 */
struct vmsg_bridge_uapi_attach {
	__u32 endpoint_id;
	__u32 flags;
	__s32 ring_fd;
	__s32 kick_fd;
	__s32 call_fd;
	__u32 vmm_max_msg_size;
	struct vmsg_bridge_uapi_ring_info tx;
	struct vmsg_bridge_uapi_ring_info rx;
	struct vmsg_bridge_uapi_caps caps;
	__u64 reserved1[4];
};

#ifdef __KERNEL__
enum vmsg_bridge_uapi_map_event_type {
	VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP = 1,
	VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP = 2,
	VMSG_BRIDGE_UAPI_MAP_EVENT_RELEASED = 3,
};

/**
 * struct vmsg_bridge_uapi_map_event - ioctl-visible map completion event
 * @map_seq: Map publication sequence identity.
 * @map_id: Logical Linux mapping record.
 * @epoch: Endpoint online epoch.
 * @bus_addr: DMA-visible bridge bus range base.
 * @length: DMA-visible bridge bus range length.
 * @mmap_offset: Mapping token offset for MAP_ADD response completion.
 * @mmap_length: Mapping token length for MAP_ADD response completion.
 * @type: Event type (%VMSG_BRIDGE_UAPI_MAP_EVENT_*).
 * @reserved0: Reserved for future use, must be zero.
 * @flags: Map flags (%VIRTIO_MSG_BUS_BRIDGE_MAP_F_*).
 * @status: Request completion status. Zero or signed negative Linux errno.
 * @reserved1: Reserved for future use, must be zero.
 * @reserved2: Reserved for future use, must be zero.
 */
struct vmsg_bridge_uapi_map_event {
	__u64 map_seq;
	__u64 map_id;
	__u64 epoch;
	__u64 bus_addr;
	__u64 length;
	__u64 mmap_offset;
	__u64 mmap_length;
	__u16 type;
	__u16 reserved0;
	__u32 flags;
	__s32 status;
	__u32 reserved1;
	__u64 reserved2;
};
#endif

#define VMSG_BRIDGE_IOCTL_BASE	0xB9

#define VMSG_BRIDGE_IOCTL_ATTACH \
	_IOWR(VMSG_BRIDGE_IOCTL_BASE, 0x00, struct vmsg_bridge_uapi_attach)
#define VMSG_BRIDGE_IOCTL_DETACH	_IO(VMSG_BRIDGE_IOCTL_BASE, 0x01)
#define VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT \
	_IOWR(VMSG_BRIDGE_IOCTL_BASE, 0x0A, struct vmsg_bridge_uapi_resolve_endpoint)
#define VMSG_BRIDGE_IOCTL_RELEASE_ENDPOINT \
	_IOW(VMSG_BRIDGE_IOCTL_BASE, 0x0B, struct vmsg_bridge_uapi_release_endpoint)

#endif /* _UAPI_LINUX_VIRTIO_MSG_BUS_BRIDGE_H */
