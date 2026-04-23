/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message transport provisional API.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 *
 * Step 4 keeps this header contract-focused and avoids exposing transport
 * runtime queue/event/session internals.
 */

#ifndef _LINUX_VIRTIO_MSG_TRANSPORT_H
#define _LINUX_VIRTIO_MSG_TRANSPORT_H

#include <linux/types.h>
#include <linux/virtio.h>
#include <linux/virtio_features.h>
#include <linux/virtio_msg_protocol.h>

struct virtio_msg_bus_provider_ops;
struct virtio_msg_bus_dma;

/**
 * VIRTIO_MSG_TRANSPORT_MIN_BUF_SIZE - Minimum transport frame buffer size.
 */
#define VIRTIO_MSG_TRANSPORT_MIN_BUF_SIZE	48

/**
 * struct virtio_msg_transport_device - exported transport device shell
 * @vdev: Embedded virtio device object.
 * @dev_num: Binding-owned transport device number for this instance.
 * @provider_ops: Attached provider callbacks consumed by transport core.
 * @provider_name: Attached provider name.
 * @provider_data: Attached provider-private opaque pointer.
 * @private: Transport-core private runtime pointer.
 *
 * The Linux transport contract keeps @dev_num binding-owned.
 * The binding that creates a vmdev must assign the device number
 * discovered from its topology source before calling
 * virtio_msg_transport_prepare_device() or
 * virtio_msg_transport_register_device() and must not change it while the
 * vmdev remains prepared or registered. The transport validates DEVICE_INFO
 * against this binding-owned value and must never rewrite it.
 *
 * The binding-owned pre-register window starts after
 * virtio_msg_transport_prepare_device() succeeds and ends when
 * virtio_msg_transport_register_prepared_device() succeeds or
 * virtio_msg_transport_unprepare_device() begins. In that window a binding may
 * finish vmdev-local parent, DMA, map-ops, and event wiring before final
 * register_virtio_device() exposure.
 *
 * Providers must bind through virtio_msg_transport_attach_provider() and
 * must not mutate transport-core runtime state directly. The @send provider
 * callback runs in sleepable context under transport request serialization.
 * Event/notify callbacks are provider-facing and must avoid unbounded waits.
 * The generic transport does not install or own a DMA shim; any binding-local
 * DMA or virtio-map setup in the pre-register window remains owned by the
 * binding that prepares @vmdev.
 */
struct virtio_msg_transport_device {
	struct virtio_device vdev;
	u16 dev_num;
	u64 required_features[VIRTIO_FEATURES_U64S];
	struct virtio_msg_bus_dma *dma_shim;
	const struct virtio_msg_bus_provider_ops *provider_ops;
	const char *provider_name;
	void *provider_data;
	void *private;
};

int virtio_msg_transport_attach_provider(struct virtio_msg_transport_device *vmdev,
					 const struct virtio_msg_bus_provider_ops *provider_ops,
					 const char *provider_name,
					 void *provider_data);
void virtio_msg_transport_detach_provider(struct virtio_msg_transport_device *vmdev);
void virtio_msg_transport_set_required_features
		(struct virtio_msg_transport_device *vmdev,
		 const u64 required_features[VIRTIO_FEATURES_U64S]);

int virtio_msg_transport_prepare_device(struct virtio_msg_transport_device *vmdev);
int virtio_msg_transport_register_prepared_device
		(struct virtio_msg_transport_device *vmdev);
void virtio_msg_transport_unprepare_device(struct virtio_msg_transport_device *vmdev);
int virtio_msg_transport_register_device(struct virtio_msg_transport_device *vmdev);
void virtio_msg_transport_unregister_device(struct virtio_msg_transport_device *vmdev);

#endif /* _LINUX_VIRTIO_MSG_TRANSPORT_H */
