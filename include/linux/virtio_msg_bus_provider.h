/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus provider API.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 *
 * Step 4 exposes only provisional callback contracts required by later units.
 */

#ifndef _LINUX_VIRTIO_MSG_BUS_PROVIDER_H
#define _LINUX_VIRTIO_MSG_BUS_PROVIDER_H

#include <linux/bits.h>
#include <linux/types.h>

struct virtio_msg;
struct virtio_msg_transport_device;

#define VIRTIO_MSG_TOKEN_EVENT			0
#define VIRTIO_MSG_TOKEN_FIXED			1

/* Revision 1 defines no transport feature bits. */
#define VIRTIO_MSG_TRANSPORT_F_SUPPORTED	0ULL
#define VIRTIO_MSG_BUS_SEND_F_NONBLOCK		BIT(0)

/**
 * struct virtio_msg_provider_caps - bus capabilities for one transport device
 * @name: Bus/provider identifier string.
 * @msg_size: Maximum message size supported for this device.
 * @revision: Transport revision negotiated by the bus/provider.
 * @transport_features: Transport feature bits supported by this device.
 */
struct virtio_msg_provider_caps {
	const char *name;
	u32 msg_size;
	u32 revision;
	u64 transport_features;
};

/**
 * struct virtio_msg_bus_provider_ops - provider callbacks used by transport core
 * @get_caps: Return bus capabilities and transport limits for @vmdev.
 * @send: Send one transport message.
 *	  Request-response traffic passes @response != NULL and @flags = 0.
 *	  Event-type one-way traffic passes @response = NULL and
 *	  @flags includes VIRTIO_MSG_BUS_SEND_F_NONBLOCK.
 *	  Implementations must reject @response != NULL with
 *	  VIRTIO_MSG_BUS_SEND_F_NONBLOCK using WARN_ON_ONCE() and -EINVAL.
 *	  Called from transport request and notify paths. Nonblocking sends
 *	  must avoid unbounded waits. Request-response sends are serialized
 *	  by the transport request mutex.
 *	  The callback must not retain @request or @response pointers.
 * @synchronize_cbs: Synchronize with async callback/event paths (optional).
 * @release: Release provider-private resources for @vmdev (optional).
 */
struct virtio_msg_bus_provider_ops {
	int (*get_caps)(struct virtio_msg_transport_device *vmdev,
			struct virtio_msg_provider_caps *caps);
	int (*send)(struct virtio_msg_transport_device *vmdev,
		    const struct virtio_msg *request,
		    struct virtio_msg *response, u32 flags);
	void (*synchronize_cbs)(struct virtio_msg_transport_device *vmdev);
	void (*release)(struct virtio_msg_transport_device *vmdev);
};

void virtio_msg_prepare(struct virtio_msg *vmsg, u8 msg_id,
			u16 token, u16 payload_size);
int virtio_msg_transport_handle_device_event(struct virtio_msg_transport_device *vmdev,
					     const struct virtio_msg *vmsg);

#endif /* _LINUX_VIRTIO_MSG_BUS_PROVIDER_H */
