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
#define VMSG_BRIDGE_UAPI_SET_DEVICES_F_CLEAR	(1U << 0)
#define VMSG_BRIDGE_UAPI_SET_DEVICES_BITMAP_MAX	8192U
#define VMSG_BRIDGE_UAPI_CTRL_PAYLOAD_MAX		64U
#define VMSG_BRIDGE_UAPI_BUS_NAME_LEN		16U
#define VMSG_BRIDGE_UAPI_BUS_ID_LEN		48U
#define VMSG_BRIDGE_UAPI_RESOLVE_F_CREATE		(1U << 0)

#define VMSG_BRIDGE_UAPI_CTRL_TYPE_ENDPOINT_ONLINE	4U
#define VMSG_BRIDGE_UAPI_CTRL_TYPE_ENDPOINT_OFFLINE	5U

#define VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_REMOVED	1U
#define VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_RESET	2U

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
 * @flags: Release flags, must be zero in v1.
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
 * @flags: Ring flags, must be zero in v1.
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
 * @entries: Ring slot count. v1 requires
 *	     %VMSG_BRIDGE_UAPI_RING_ENTRIES_DEFAULT.
 * @entry_size: Size of each slot in bytes. v1 requires the attach-time
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
 * - Each slot carries exactly one transport-class virtio-msg frame.
 * - Frame bytes start at slot offset 0.
 * - msg_size is read from struct virtio_msg header inside the frame.
 * - Remaining slot bytes are ignored.
 * - TX ring producer is userspace; TX ring consumer is kernel.
 * - RX ring producer is kernel; RX ring consumer is userspace.
 * - Bus-class frames do not traverse TX/RX rings.
 * - Producer publishes payload before updating @prod (release ordering).
 * - Consumer reads @prod with acquire ordering before reading payload.
 * - Consumer updates @cons only after finishing slot processing.
 */

/**
 * struct vmsg_bridge_uapi_caps - negotiated bridge capabilities
 * @uapi_version: Negotiated UAPI version (%VMSG_BRIDGE_UAPI_VERSION).
 * @revision: Transport revision.
 * @max_msg_size: Negotiated maximum raw virtio-msg frame size.
 * @flags: Output flags (%VMSG_BRIDGE_UAPI_CAP_F_*).
 * @transport_features: Supported transport feature bits.
 * @bridge_features: Supported bridge UAPI feature bits.
 * @reserved1: Reserved for future use, must be zero.
 */
struct vmsg_bridge_uapi_caps {
	__u32 uapi_version;
	__u32 revision;
	__u32 max_msg_size;
	__u32 flags;
	__u64 transport_features;
	__u64 bridge_features;
	__u64 reserved1[2];
};

/**
 * struct vmsg_bridge_uapi_attach - attach one userspace peer to one endpoint
 * @endpoint_id: Handle returned by %VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT.
 * @flags: Attach flags, must be zero in v1.
 * @ring_fd: Shared memory file descriptor backing TX/RX rings. Must refer
 *           to a shmem-backed file (for example memfd_create() or tmpfs).
 * @kick_fd: Eventfd signaled by userspace when TX work is posted.
 * @call_fd: Eventfd signaled by kernel when bridge-fd readiness may have
 *	     changed (for example control/map wakeups or RX publication).
 * @vmm_max_msg_size: VMM maximum raw virtio-msg frame size for this attach
 *		      epoch. Must be in the
 *		      %VIRTIO_MSG_MIN_SIZE..%VIRTIO_MSG_MAX_SIZE range. Ring
 *		      slot sizing is derived from this value.
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

enum vmsg_bridge_uapi_map_event_type {
	VMSG_BRIDGE_UAPI_MAP_EVENT_ADD = 1,
	VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_REQ = 2,
};

enum vmsg_bridge_uapi_map_ack_status {
	VMSG_BRIDGE_UAPI_MAP_ACK_OK = 0,
	VMSG_BRIDGE_UAPI_MAP_ACK_RETRY = 1,
	VMSG_BRIDGE_UAPI_MAP_ACK_REJECT = 2,
};

/**
 * struct vmsg_bridge_uapi_map_event - one map lifecycle event
 * @seq: Monotonic map event sequence id.
 * @map_id: Monotonic map identifier within one attach epoch.
 * @epoch: Attach epoch generation.
 * @bus_addr: Opaque bridge-driver-defined bus address for the range start.
 * @length: Range length in bytes.
 * @mmap_offset: Device mmap offset for mapped ranges. For ADD events, this
 *	mmap offset is valid for userspace mmap() after MAP_EVENT_RECV
 *	succeeds and remains valid after ACK_OK retires the ADD queue
 *	entry. The mapping token stays valid until a later DEL_REQ
 *	retirement, ENDPOINT_OFFLINE, detach, or other invalidation.
 * @mmap_length: Mappable length in bytes.
 * @type: Map event type (%VMSG_BRIDGE_UAPI_MAP_EVENT_*).
 * @reserved0: Reserved for future use, must be zero.
 * @flags: Event flags, must be zero in v1.
 * @reserved1: Reserved for future use, must be zero.
 *
 * Lifetime rules:
 * - ADD is valid only for the current online epoch of the current attach.
 * - A later ENDPOINT_OFFLINE can purge undelivered or stale map-event state.
 * - Userspace treats ENDPOINT_OFFLINE as the authoritative runtime signal that
 *   no previously published mapping token remains valid for future use.
 */
struct vmsg_bridge_uapi_map_event {
	__u64 seq;
	__u64 map_id;
	__u64 epoch;
	__u64 bus_addr;
	__u64 length;
	__u64 mmap_offset;
	__u64 mmap_length;
	__u16 type;
	__u16 reserved0;
	__u32 flags;
	__u64 reserved1[2];
};

/**
 * struct vmsg_bridge_uapi_map_event_recv - dequeue one map event
 * @event: Dequeued map event payload (output).
 * @flags: Receive flags, must be zero in v1.
 * @reserved0: Reserved for future use, must be zero.
 * @reserved1: Reserved for future use, must be zero.
 *
 * Userspace passes this structure zero-initialized. The ioctl never sleeps:
 * it returns -EAGAIN when no deliverable map-event queue head is available.
 */
struct vmsg_bridge_uapi_map_event_recv {
	struct vmsg_bridge_uapi_map_event event;
	__u32 flags;
	__u32 reserved0;
	__u64 reserved1[2];
};

/**
 * struct vmsg_bridge_uapi_map_event_ack - acknowledge one map event
 * @seq: Sequence id returned by MAP_EVENT_RECV.
 * @type: Event type returned by MAP_EVENT_RECV.
 * @status: Ack status (%VMSG_BRIDGE_UAPI_MAP_ACK_*).
 * @flags: Ack flags, must be zero in v1.
 * @reserved1: Reserved for future use, must be zero.
 *
 * Ack status rules:
 * - ACK_OK accepts and retires the delivered event.
 * - ACK_RETRY requests later redelivery; userspace must receive the event again
 *   before acknowledging it again.
 * - ACK_REJECT rejects the event and invalidates the current attached session.
 *   After a successful reject, poll() reports detached-session readiness and
 *   endpoint-scoped operations fail with ENOTCONN semantics.
 */
struct vmsg_bridge_uapi_map_event_ack {
	__u64 seq;
	__u16 type;
	__u16 status;
	__u32 flags;
	__u64 reserved1[2];
};

/**
 * struct vmsg_bridge_uapi_set_devices - apply a bitmap snapshot over one dev-num range
 * @endpoint_id: Handle returned by %VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT.
 * @first_dev_num: First transport dev_num in the bitmap range (inclusive).
 * @last_dev_num: Last transport dev_num in the bitmap range (inclusive).
 * @flags: Update flags (VMSG_BRIDGE_UAPI_SET_DEVICES_F_*).
 * @bitmap_bytes: Bitmap size in bytes. Must equal
 *		  DIV_ROUND_UP(last_dev_num - first_dev_num + 1, 8).
 * @reserved0: Reserved for future use, must be zero.
 * @bitmap_ptr: Userspace pointer to bitmap bytes, where bit index n maps
 *		(first_dev_num + n).
 *
 * Semantics:
 * - dev_num values occupy the u16 transport namespace; dev_num 0 is valid.
 * - Bits set in @bitmap_ptr add/preserve dev_num values in
 *   [first_dev_num, last_dev_num].
 * - Bits clear in @bitmap_ptr remove dev_num values in
 *   [first_dev_num, last_dev_num].
 * - If @flags has %VMSG_BRIDGE_UAPI_SET_DEVICES_F_CLEAR, the endpoint topology is
 *   cleared before applying this range. This allows chunked full snapshots.
 * - One 8192-byte bitmap can cover the full 0..65535 dev_num space.
 */
struct vmsg_bridge_uapi_set_devices {
	__u32 endpoint_id;
	__u16 first_dev_num;
	__u16 last_dev_num;
	__u32 flags;
	__u32 bitmap_bytes;
	__u32 reserved0;
	__aligned_u64 bitmap_ptr;
};

/**
 * struct vmsg_bridge_uapi_ctrl_endpoint_online - endpoint online transition
 * @endpoint_id: Handle returned by %VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT.
 * @bus_name: NUL-terminated provider name for the currently matched endpoint.
 *	      Unused trailing bytes must be zero.
 * @max_msg_size: Current bus-reported maximum raw virtio-msg frame size for
 *		  this endpoint.
 * @reserved0: Reserved for future use, must be zero.
 */
struct vmsg_bridge_uapi_ctrl_endpoint_online {
	__u32 endpoint_id;
	char bus_name[VMSG_BRIDGE_UAPI_BUS_NAME_LEN];
	__u32 max_msg_size;
	__u64 reserved0[2];
};

/**
 * struct vmsg_bridge_uapi_ctrl_endpoint_offline - endpoint offline transition
 * @endpoint_id: Handle returned by %VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT.
 * @reason: Reason code (%VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_*).
 * @reserved0: Reserved for future use, must be zero.
 */
struct vmsg_bridge_uapi_ctrl_endpoint_offline {
	__u32 endpoint_id;
	__u32 reason;
	__u64 reserved0;
};

/**
 * struct vmsg_bridge_uapi_control_msg - one dequeued control notification
 * @seq: Monotonic notification sequence id.
 * @type: Notification type (%VMSG_BRIDGE_UAPI_CTRL_TYPE_ENDPOINT_*).
 * @flags: Reserved for future use, must be zero.
 * @payload_len: Payload bytes valid in @payload.
 * @payload: Type-specific payload bytes.
 *
 * Failure-handling rules:
 * - ENDPOINT_OFFLINE reports endpoint availability loss for the current attach
 *   epoch; it does not detach the session.
 * - CONTROL_RECV never sleeps; empty queue returns -EAGAIN regardless of
 *   file status flags.
 * - poll() returning POLLERR|POLLHUP means the current attached session was
 *   detached or invalidated.
 * - poll() returning POLLERR without POLLHUP means bridge core latched an
 *   unrecoverable shared-ring fault for the current attach, including malformed
 *   ring state or invalid bridge-ring traffic.
 * - Both poll error cases require abandoning the current attach.
 */
struct vmsg_bridge_uapi_control_msg {
	__u64 seq;
	__u16 type;
	__u16 flags;
	__u32 payload_len;
	__u8 payload[VMSG_BRIDGE_UAPI_CTRL_PAYLOAD_MAX];
};

/**
 * struct vmsg_bridge_uapi_msg_error - userspace request-failure report
 * @dev_num: Transport dev_num of the failed request. dev_num 0 is valid
 *	     if userspace published it in the active topology snapshot.
 * @token: Request token of the failed request.
 * @msg_id: Original request opcode.
 * @reserved0: Reserved for future use, must be zero.
 * @error: Negative Linux errno for the failed request.
 * @flags: Report flags, must be zero in v1.
 * @reserved1: Reserved for future use, must be zero.
 *
 * This ioctl is valid only for transport requests that still have an
 * outstanding relay entry on the attached bridge session. Bus-class frames do
 * not traverse the bridge rings. ENOTCONN means the session no longer has a
 * live endpoint binding for request completion (for example detached or
 * endpoint-offline state).
 */
struct vmsg_bridge_uapi_msg_error {
	__u16 dev_num;
	__u16 token;
	__u8 msg_id;
	__u8 reserved0[3];
	__s32 error;
	__u32 flags;
	__u64 reserved1[2];
};

#define VMSG_BRIDGE_IOCTL_BASE	0xB9

#define VMSG_BRIDGE_IOCTL_ATTACH \
	_IOWR(VMSG_BRIDGE_IOCTL_BASE, 0x00, struct vmsg_bridge_uapi_attach)
#define VMSG_BRIDGE_IOCTL_DETACH	_IO(VMSG_BRIDGE_IOCTL_BASE, 0x01)
#define VMSG_BRIDGE_IOCTL_MAP_EVENT_RECV \
	_IOWR(VMSG_BRIDGE_IOCTL_BASE, 0x02, struct vmsg_bridge_uapi_map_event_recv)
#define VMSG_BRIDGE_IOCTL_MAP_EVENT_ACK \
	_IOW(VMSG_BRIDGE_IOCTL_BASE, 0x03, struct vmsg_bridge_uapi_map_event_ack)
#define VMSG_BRIDGE_IOCTL_SET_DEVICES \
	_IOW(VMSG_BRIDGE_IOCTL_BASE, 0x04, struct vmsg_bridge_uapi_set_devices)
#define VMSG_BRIDGE_IOCTL_CONTROL_RECV \
	_IOWR(VMSG_BRIDGE_IOCTL_BASE, 0x08, struct vmsg_bridge_uapi_control_msg)
#define VMSG_BRIDGE_IOCTL_MSG_ERROR \
	_IOW(VMSG_BRIDGE_IOCTL_BASE, 0x09, struct vmsg_bridge_uapi_msg_error)
#define VMSG_BRIDGE_IOCTL_RESOLVE_ENDPOINT \
	_IOWR(VMSG_BRIDGE_IOCTL_BASE, 0x0A, struct vmsg_bridge_uapi_resolve_endpoint)
#define VMSG_BRIDGE_IOCTL_RELEASE_ENDPOINT \
	_IOW(VMSG_BRIDGE_IOCTL_BASE, 0x0B, struct vmsg_bridge_uapi_release_endpoint)

#endif /* _UAPI_LINUX_VIRTIO_MSG_BUS_BRIDGE_H */
