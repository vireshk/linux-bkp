/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bridge-device provisional API.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 *
 * Step 4 keeps this header minimal and intentionally avoids exposing mutable
 * bridge-core session or transport runtime internals.
 */

#ifndef _LINUX_VIRTIO_MSG_BUS_BRIDGE_DEVICE_H
#define _LINUX_VIRTIO_MSG_BUS_BRIDGE_DEVICE_H

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/virtio_msg_bus_provider.h>
#include <linux/virtio_msg_protocol.h>
#include <linux/wait.h>
#include <uapi/linux/virtio_msg_bus_bridge.h>

struct vm_area_struct;
struct vmsg_bus_session;
struct virtio_msg_bus_bridge_device;
struct vmsg_bus_endpoint_topology;

#define VIRTIO_MSG_DISPATCH_F_NONBLOCK		BIT(0)
#define VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE	BIT(1)

/**
 * struct virtio_msg_dispatch_ctx - optional per-dispatch relay metadata
 * @flags: Dispatch behavior hints (%VIRTIO_MSG_DISPATCH_F_*).
 * @reserved0: Reserved for future use, must be zero.
 * @relay_seq: Caller-owned relay metadata for request dispatch validation.
 * @reserved1: Reserved for future use, must be zero.
 *
 * Ownership and lifetime:
 * - Callers own storage and may keep this object on stack.
 * - Callees must copy needed fields and must not retain @dctx pointers.
 *
 * Flag contract:
 * - %VIRTIO_MSG_DISPATCH_F_NONBLOCK: bridge core must not block on its mutexes
 *   and returns %-EAGAIN if lock contention prevents immediate dispatch.
 * - %VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE: caller is in sleepable endpoint
 *   request context. When set, @relay_seq must be non-zero.
 * - @relay_seq must be zero unless REQUEST_SLEEPABLE is set.
 */
struct virtio_msg_dispatch_ctx {
	u32 flags;
	u32 reserved0;
	u64 relay_seq;
	u64 reserved1;
};

/**
 * struct vmsg_bus_resolver - bus-name resolver registration for bridge core
 * @name: Bus name visible through userspace endpoint resolution.
 * @validate: Optional bus_id validator/canonicalizer.
 * @match: Endpoint lookup callback for one bus_id.
 */
struct vmsg_bus_resolver {
	char name[VMSG_BRIDGE_UAPI_BUS_NAME_LEN];
	int (*validate)(const struct vmsg_bus_resolver *resolver, char *bus_id);
	struct virtio_msg_bus_bridge_device *(*match)
		(const struct vmsg_bus_resolver *resolver, const char *bus_id);
};

int vmsg_bus_resolver_register(struct vmsg_bus_resolver *resolver);
void vmsg_bus_resolver_unregister(struct vmsg_bus_resolver *resolver);

/**
 * struct virtio_msg_bus_bridge_device_map_req - provider map request tuple
 * @bus_addr: Provider-defined bus address for range start.
 * @length: Requested length in bytes.
 * @flags: Provider-defined request flags.
 * @reserved0: Reserved for future use, must be zero.
 */
struct virtio_msg_bus_bridge_device_map_req {
	u64 bus_addr;
	u64 length;
	u32 flags;
	u32 reserved0;
};

/**
 * struct virtio_msg_bus_bridge_device_map_resp - provider map response tuple
 * @mmap_offset: Bridge mapping token used by userspace mmap flow.
 * @length: Mappable length in bytes.
 * @flags: Provider-defined response flags.
 * @reserved0: Reserved for future use, must be zero.
 */
struct virtio_msg_bus_bridge_device_map_resp {
	u64 mmap_offset;
	u64 length;
	u32 flags;
	u32 reserved0;
};

/**
 * struct virtio_msg_bus_bridge_device_unmap_req - provider unmap request tuple
 * @bus_addr: Provider-defined bus address for range start.
 * @length: Requested length in bytes.
 * @mmap_offset: Bridge mapping token associated with this mapping.
 * @flags: Provider-defined request flags.
 * @reserved0: Reserved for future use, must be zero.
 */
struct virtio_msg_bus_bridge_device_unmap_req {
	u64 bus_addr;
	u64 length;
	u64 mmap_offset;
	u32 flags;
	u32 reserved0;
};

/**
 * struct virtio_msg_bus_bridge_device_caps - bridge-facing endpoint capabilities
 * @name: Bus/provider identifier string.
 * @msg_size: Maximum message size supported by this endpoint.
 * @revision: Transport revision negotiated by this endpoint.
 * @transport_features: Transport feature bits supported by this endpoint.
 * @bridge_features: Bridge UAPI feature bits supported by this endpoint.
 */
struct virtio_msg_bus_bridge_device_caps {
	const char *name;
	u32 msg_size;
	u32 revision;
	u64 transport_features;
	u64 bridge_features;
};

/**
 * struct virtio_msg_bus_bridge_device_ops - endpoint callbacks used by bridge core
 * @get_caps: Return current provider-reported endpoint capabilities.
 *	@caps->name must fit %VMSG_BRIDGE_UAPI_BUS_NAME_LEN and
 *	@caps->msg_size is the provider-reported endpoint limit before bridge
 *	core applies the attach-time userspace clamp.
 * @tx_msg: Submit one transport-class virtio-msg frame.
 *	Return 0 on success, %-EAGAIN to retry later, %-ENODEV/%-ENOTCONN/
 *	%-ESHUTDOWN to disconnect the current bridge session, or any other
 *	negative errno to fault the bridge ring.
 * @report_error: Complete one pending transport request with a userspace-reported errno.
 * @mmap: Optional userspace mmap callback for active mapping token.
 * @map_event_ack: Optional map-event ACK callback.
 * @notify_device_event: Optional endpoint device-event publication callback.
 * @synchronize_cbs: Optional synchronization callback for async callback paths
 *	used before unregister completes and before offline transition
 *	completion.
 * @release: Optional endpoint release callback.
 */
struct virtio_msg_bus_bridge_device_ops {
	int (*get_caps)(struct virtio_msg_bus_bridge_device *endpoint,
			struct virtio_msg_bus_bridge_device_caps *caps);
	int (*tx_msg)(struct virtio_msg_bus_bridge_device *endpoint,
		      const struct virtio_msg *msg, u16 msg_size,
		      const struct virtio_msg_dispatch_ctx *dctx);
	int (*report_error)(struct virtio_msg_bus_bridge_device *endpoint,
			    u16 dev_num, u16 token, u8 msg_id, int error,
			    const struct virtio_msg_dispatch_ctx *dctx);
	int (*mmap)(struct virtio_msg_bus_bridge_device *endpoint, u64 mmap_offset,
		    struct vm_area_struct *vma);
	int (*map_event_ack)(struct virtio_msg_bus_bridge_device *endpoint,
			     const struct vmsg_bridge_uapi_map_event *event,
			     u32 ack_status);
	int (*notify_device_event)(struct virtio_msg_bus_bridge_device *endpoint,
				   u16 dev_num, u16 dev_state);
	void (*synchronize_cbs)(struct virtio_msg_bus_bridge_device *endpoint);
	void (*release)(struct virtio_msg_bus_bridge_device *endpoint);
};

/**
 * struct virtio_msg_bus_bridge_device - exported endpoint object
 * @ops: Bridge endpoint callbacks.
 * @priv: Endpoint-private pointer owned by provider implementation.
 * @handle: Kernel-assigned endpoint handle while registered.
 * @rx_inflight: Number of in-flight rx callbacks.
 * @rx_waitq: Waitqueue used to quiesce in-flight callbacks at unregister or
 *	live offline transition.
 * @rx_unregistered: True after unregister begins or the endpoint is forced
 *	offline; blocks new rx dispatch.
 * @topology: Bridge-core private cached endpoint topology for lockless RX
 *	target validation.
 * @publish_lock: Bridge-core private lock guarding cached publish-session state.
 * @publish_session: Bridge-core private cached session pointer used by
 *	nonblocking userspace publish path.
 */
struct virtio_msg_bus_bridge_device {
	const struct virtio_msg_bus_bridge_device_ops *ops;
	void *priv;
	u32 handle;
	atomic_t rx_inflight;
	wait_queue_head_t rx_waitq;
	bool rx_unregistered;
	struct vmsg_bus_endpoint_topology *topology;
	/* Protects cached publish-session state. */
	spinlock_t publish_lock;
	struct vmsg_bus_session *publish_session;
};

static inline void
virtio_msg_bus_bridge_device_init(struct virtio_msg_bus_bridge_device *endpoint,
				  const struct virtio_msg_bus_bridge_device_ops *ops,
				  void *priv)
{
	endpoint->ops = ops;
	endpoint->priv = priv;
	endpoint->handle = 0;
	atomic_set(&endpoint->rx_inflight, 0);
	init_waitqueue_head(&endpoint->rx_waitq);
	endpoint->rx_unregistered = false;
	endpoint->topology = NULL;
	spin_lock_init(&endpoint->publish_lock);
	endpoint->publish_session = NULL;
}

enum virtio_msg_bus_bridge_endpoint_offline_reason {
	VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE_REASON_REMOVED =
		VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_REMOVED,
	VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE_REASON_RESET =
		VMSG_BRIDGE_UAPI_ENDPOINT_OFFLINE_REASON_RESET,
};

int virtio_msg_bus_bridge_device_register(const char *bus_name,
					  const char *bus_id,
					  struct virtio_msg_bus_bridge_device *endpoint);
void virtio_msg_bus_bridge_device_unregister(const char *bus_name,
					     const char *bus_id,
					     struct virtio_msg_bus_bridge_device *endpoint);
int virtio_msg_bus_bridge_endpoint_set_offline
	(struct virtio_msg_bus_bridge_device *endpoint,
	 enum virtio_msg_bus_bridge_endpoint_offline_reason reason);
int virtio_msg_bus_bridge_endpoint_set_online(struct virtio_msg_bus_bridge_device *endpoint);
int virtio_msg_bus_bridge_endpoint_cleanup_begin(u32 endpoint_id);
void virtio_msg_bus_bridge_endpoint_cleanup_end(u32 endpoint_id);
/**
 * virtio_msg_bus_bridge_device_rx - inject one transport-class frame into bridge dispatch
 * @handle: Resolved endpoint handle.
 * @msg: Frame buffer to dispatch.
 * @dctx: Optional dispatch metadata, may be %NULL.
 *
 * Callback contract:
 * - The callee does not retain @msg/@dctx pointers after return.
 * - Bridge core rejects bus-class frames before provider dispatch.
 * - Callers provide sleepable context unless @dctx->flags requests non-blocking
 *   dispatch semantics.
 * - Response completion correlation remains endpoint-defined; @relay_seq is
 *   validated metadata for request dispatch context.
 */
int virtio_msg_bus_bridge_device_rx(u32 handle,
				    const struct virtio_msg *msg,
				    const struct virtio_msg_dispatch_ctx *dctx);
/**
 * virtio_msg_bus_bridge_device_report_error - fail one pending transport request relay
 * @handle: Resolved endpoint handle.
 * @dev_num: Transport device number from the original request.
 * @token: Request token from the original request.
 * @msg_id: Request opcode from the original request.
 * @error: Negative Linux errno to return to the waiting transport caller.
 */
int virtio_msg_bus_bridge_device_report_error(u32 handle, u16 dev_num,
					      u16 token, u8 msg_id,
					      int error);
int virtio_msg_bus_bridge_device_relay_drop(u32 handle, u16 dev_num,
					    u16 token);
/**
 * virtio_msg_bus_bridge_device_publish_rx - publish one transport-class frame to userspace
 * @handle: Resolved endpoint handle.
 * @msg: Frame buffer to enqueue on the bridge RX ring.
 *
 * Bridge core rejects bus-class frames and never publishes them to userspace.
 */
int virtio_msg_bus_bridge_device_publish_rx(u32 handle,
					    const struct virtio_msg *msg);
int virtio_msg_bus_bridge_device_publish_rx_nonblock
	(struct virtio_msg_bus_bridge_device *endpoint,
	 const struct virtio_msg *msg);

int virtio_msg_bus_bridge_device_map_event_add(u32 handle, u64 bus_addr,
					       u64 length, u64 mmap_offset,
					       u64 mmap_length, u32 flags,
					       u64 *map_id);
int virtio_msg_bus_bridge_device_map_event_del_req(u32 handle, u64 map_id);

int virtio_msg_bus_bridge_topology_add(struct virtio_msg_bus_bridge_device *endpoint,
				       u16 dev_num);
int virtio_msg_bus_bridge_topology_remove(struct virtio_msg_bus_bridge_device *endpoint,
					  u16 dev_num);
void virtio_msg_bus_bridge_topology_clear(struct virtio_msg_bus_bridge_device *endpoint);
int virtio_msg_bus_bridge_topology_get_devices_window(struct virtio_msg_bus_bridge_device
						      *endpoint, u16 offset, u16 count,
						      u8 *bitmap, size_t bitmap_len,
						      u16 *out_num, u16 *out_next_offset);

#endif /* _LINUX_VIRTIO_MSG_BUS_BRIDGE_DEVICE_H */
