/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message transport private runtime definitions.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 *
 * Public transport state stays in include/linux/virtio_msg_transport.h.
 */

#ifndef _VIRTIO_MSG_TRANSPORT_INTERNAL_H
#define _VIRTIO_MSG_TRANSPORT_INTERNAL_H

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/virtio_features.h>
#include <linux/virtio_msg_bus_provider.h>
#include <linux/virtio_msg_protocol.h>
#include <linux/virtio_msg_transport.h>
#include <linux/workqueue.h>

/**
 * struct virtio_msg_transport - private runtime transport container
 * @vmdev: Back-pointer to the owning transport shell.
 * @request_lock: Serializes blocking transport requests per device.
 * @release_done: Signals completion of release callback teardown.
 * @event_lock: Serializes coalesced inbound event state.
 * @event_work: Dispatches coalesced inbound events in worker context.
 * @request: Reusable request frame buffer.
 * @response: Reusable response frame buffer.
 * @pending_used_bitmap: Coalesced pending EVENT_USED queue indices.
 * @pending_used_snapshot: Worker-owned EVENT_USED bitmap pending dispatch.
 * @device_features: Cached feature blocks read from transport.
 * @msg_size: Negotiated transport frame size.
 * @transport_revision: Negotiated transport revision.
 * @config_size: Reported config space size from DEVICE_INFO.
 * @num_feature_bits: Reported feature bit count from DEVICE_INFO.
 * @max_vq_count: Reported maximum virtqueue count from DEVICE_INFO.
 * @admin_vq_start: Reported admin virtqueue start index from DEVICE_INFO.
 * @admin_vq_count: Reported admin virtqueue count from DEVICE_INFO.
 * @dev_num: Transport device identifier used in frame headers.
 * @next_token: Monotonic request token allocator.
 * @config_generation: Last observed config generation.
 * @config_warn_generation: Generation associated with pending warning risk.
 * @config_warn_reasons: Bitmask of pending warning risk reasons.
 * @config_warn_pending: True while warning risk is pending.
 * @config_warn_emitted: True once warning was emitted for current risk window.
 * @cached_status: Last observed status byte.
 * @config_shadow: Atomic-read config cache mirror.
 * @config_shadow_valid: True when @config_shadow contains coherent bytes.
 * @config_lock: Protects config shadow and warning state fields.
 * @pending_config_offset: Staged EVENT_CONFIG byte offset.
 * @pending_config_len: Staged EVENT_CONFIG byte length.
 * @pending_config_capacity: Byte capacity of @pending_config_data.
 * @pending_config_data: Staged EVENT_CONFIG payload bytes.
 * @pending_config_generation: Coalesced inbound config generation.
 * @pending_device_status: Coalesced inbound device status.
 * @prepared: True while transport-owned runtime is prepared but may not yet be
 *	      registered on the virtio bus.
 * @registered: True while this vmdev is registered on virtio bus.
 * @accept_events: True while inbound events may be queued.
 * @events_dispatch_ready: True once queued events may be dispatched.
 * @event_work_queued: True while @event_work is queued or running.
 * @config_pending: True when a coalesced EVENT_CONFIG is pending.
 * @unplugged: True once unregister starts and teardown must stay local.
 * @fatal: True after transport marks the device failed.
 */
struct virtio_msg_transport {
	struct virtio_msg_transport_device *vmdev;
	/* Serializes blocking request/response exchanges for one vmdev. */
	struct mutex request_lock;
	struct completion release_done;
	/* Protects coalesced inbound event state. */
	spinlock_t event_lock;
	struct work_struct event_work;
	struct virtio_msg *request;
	struct virtio_msg *response;
	unsigned long *pending_used_bitmap;
	unsigned long *pending_used_snapshot;
	u64 device_features[VIRTIO_FEATURES_U64S];
	u32 msg_size;
	u32 transport_revision;
	u32 config_size;
	u32 num_feature_bits;
	u32 max_vq_count;
	u16 admin_vq_start;
	u16 admin_vq_count;
	u16 dev_num;
	u16 next_token;
	u32 config_generation;
	u32 config_warn_generation;
	u32 pending_config_offset;
	u32 pending_config_len;
	u32 pending_config_capacity;
	u32 pending_config_generation;
	u32 pending_device_status;
	u8 config_warn_reasons;
	u8 cached_status;
	u8 *config_shadow;
	u8 *pending_config_data;
	bool config_shadow_valid;
	spinlock_t config_lock; /* Protects config shadow and warning state. */
	bool prepared;
	bool registered;
	bool accept_events;
	bool events_dispatch_ready;
	bool event_work_queued;
	bool config_pending;
	bool unplugged;
	bool fatal;
};

bool virtio_msg_transport_is_fatal(const struct virtio_msg_transport *transport);
void virtio_msg_transport_set_fatal(struct virtio_msg_transport_device *vmdev,
				    struct virtio_msg_transport *transport);
int virtio_msg_transport_validate_config_range(const struct virtio_msg_transport *transport,
					       u32 offset, u32 len);

#endif /* _VIRTIO_MSG_TRANSPORT_INTERNAL_H */
