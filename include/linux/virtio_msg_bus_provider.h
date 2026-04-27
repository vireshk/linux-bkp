/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus provider API.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 *
 * Bus providers expose sleepable request/response sends and quick event
 * enqueue sends to the transport core.
 */

#ifndef _LINUX_VIRTIO_MSG_BUS_PROVIDER_H
#define _LINUX_VIRTIO_MSG_BUS_PROVIDER_H

#include <asm/byteorder.h>

#include <linux/bits.h>
#include <linux/types.h>
#include <linux/virtio_msg_protocol.h>

struct virtio_msg_transport_device;

#define VIRTIO_MSG_TOKEN_EVENT			0
#define VIRTIO_MSG_TOKEN_FIXED			1

/*
 * Strict configuration generation is defined by the transport protocol but is
 * not supported by the baseline Linux transport profile.
 */
#define VIRTIO_MSG_F_STRICT_CONFIG_GENERATION	BIT_ULL(0)
#define VIRTIO_MSG_TRANSPORT_F_SUPPORTED	0ULL

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
 *	  Request-response traffic passes @response != NULL from sleepable
 *	  context under transport request serialization. Notify/event traffic
 *	  passes @response = NULL and may arrive from the transport notify path,
 *	  so providers must enqueue quickly and return without sleeping.
 *	  Providers own bus-side queue, retry, drop, token, and correlation
 *	  policy. Providers may use the optional virtio message bus queue helper
 *	  internally when they need FIFO handoff from non-sleepable paths.
 *	  The callback must not retain @request or @response pointers.
 * @synchronize_cbs: Synchronize with async callback/event paths (optional).
 *	  Providers that deliver inbound config/used events asynchronously must
 *	  not call virtio_msg_transport_handle_device_event() before the virtio
 *	  device is registered and the target virtqueue can receive callbacks.
 *	  They must also synchronize those paths during teardown so no inbound
 *	  event can race with freed transport state.
 * @release: Release provider-private resources for @vmdev (optional).
 */
struct virtio_msg_bus_provider_ops {
	int (*get_caps)(struct virtio_msg_transport_device *vmdev,
			struct virtio_msg_provider_caps *caps);
	int (*send)(struct virtio_msg_transport_device *vmdev,
		    const struct virtio_msg *request,
		    struct virtio_msg *response);
	void (*synchronize_cbs)(struct virtio_msg_transport_device *vmdev);
	void (*release)(struct virtio_msg_transport_device *vmdev);
};

static inline void virtio_msg_prepare(struct virtio_msg *vmsg, u8 msg_id,
				      u16 token, u16 payload_size, u16 dev_num)
{
	if (!vmsg)
		return;

	vmsg->type = VIRTIO_MSG_TYPE_TRANSPORT | VIRTIO_MSG_TYPE_REQUEST;
	vmsg->msg_id = msg_id;
	vmsg->dev_num = cpu_to_le16(dev_num);
	vmsg->token = cpu_to_le16(token);
	vmsg->msg_size = cpu_to_le16(sizeof(*vmsg) + payload_size);
}

/**
 * virtio_msg_transport_handle_device_event() - deliver one inbound event
 * @vmdev: Transport device receiving the event.
 * @vmsg: Event frame owned by the provider for this call.
 *
 * The transport validates every inbound event and dispatches accepted events
 * directly. Invalid, stale, or early events are dropped with a ratelimited
 * warning and do not report provider-visible delivery errors.
 *
 * Return: 0 for handled or dropped events, -EINVAL for invalid arguments.
 */
int virtio_msg_transport_handle_device_event(struct virtio_msg_transport_device *vmdev,
					     const struct virtio_msg *vmsg);

#endif /* _LINUX_VIRTIO_MSG_BUS_PROVIDER_H */
