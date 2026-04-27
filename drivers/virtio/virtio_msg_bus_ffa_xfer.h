/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus over FF-A transfer-method helper contracts.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_FFA_XFER_H
#define _VIRTIO_MSG_BUS_FFA_XFER_H

#include <linux/arm_ffa.h>
#include <linux/bits.h>
#include <linux/kconfig.h>
#include <linux/types.h>

#include "virtio_msg_bus_ffa.h"

enum virtio_msg_ffa_xfer_role_mask {
	VIRTIO_MSG_FFA_XFER_ROLE_DRIVER = BIT(0),
	VIRTIO_MSG_FFA_XFER_ROLE_DEVICE = BIT(1),
};

enum virtio_msg_ffa_xfer_phase_flags {
	VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP = BIT(0),
	VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME = BIT(1),
};

#define VIRTIO_MSG_FFA_XFER_METHOD_MASK(method)	BIT(method)
#define VIRTIO_MSG_FFA_XFER_BOOTSTRAP_MASK \
	(VIRTIO_MSG_FFA_XFER_METHOD_MASK(VIRTIO_MSG_FFA_XFER_DIRECT) | \
	 VIRTIO_MSG_FFA_XFER_METHOD_MASK(VIRTIO_MSG_FFA_XFER_INDIRECT))

struct virtio_msg_ffa_device;
struct virtio_msg_ffa_driver;
struct virtio_msg_ffa_xfer_method_desc;

struct virtio_msg_ffa_xfer_driver_method_ops {
	/*
	 * init_runtime() must leave both message and event delivery ready for
	 * this method, or release any candidate state before returning an error.
	 */
	int (*init_runtime)(struct virtio_msg_ffa_driver *drv,
			    const struct virtio_msg_ffa_xfer_method_desc *desc);
};

struct virtio_msg_ffa_xfer_device_method_ops {
	/*
	 * init_runtime() must leave both message and event delivery ready for
	 * this method, or release any candidate state before returning an error.
	 */
	int (*init_runtime)(struct virtio_msg_ffa_device *vdev,
			    const struct virtio_msg_ffa_xfer_method_desc *desc);
};

struct virtio_msg_ffa_xfer_method_desc {
	enum virtio_msg_ffa_transfer_method method;
	const char *name;
	u8 role_mask;
	u8 phase_flags;
	u32 bus_feature_mask;
	enum virtio_msg_ffa_bus_event_delivery event_delivery;
	const struct virtio_msg_ffa_xfer_driver_method_ops *driver_ops;
	const struct virtio_msg_ffa_xfer_device_method_ops *device_ops;
};

struct virtio_msg_ffa_xfer_ctx {
	struct virtio_msg_ffa_endpoint *ep;
	struct ffa_device *fdev;
	void *priv;
};

struct virtio_msg_ffa_xfer_method_ops {
	int (*prepare_request)(struct virtio_msg_ffa_xfer_ctx *ctx,
			       const struct virtio_msg *req, size_t req_len,
			       void *wire_req, size_t wire_req_len);
	int (*parse_response)(struct virtio_msg_ffa_xfer_ctx *ctx,
			      const void *wire_resp, size_t wire_resp_len,
			      struct virtio_msg *resp, size_t resp_buf_len,
			      size_t *resp_len);
};

#if IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_INDIRECT)
extern const struct virtio_msg_ffa_xfer_method_ops
	virtio_msg_ffa_indirect_method_ops;
#endif

static inline unsigned long
virtio_msg_ffa_xfer_methods(u8 role_mask, u32 bus_features, bool negotiated)
{
	unsigned long methods = 0;

	if ((role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DRIVER) &&
	    IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_DIRECT) &&
	    (!negotiated ||
	     ((bus_features & (VIRTIO_MSG_FFA_BUS_FEATURE_DIRECT_RX |
			       VIRTIO_MSG_FFA_BUS_FEATURE_DIRECT_TX)) ==
	      (VIRTIO_MSG_FFA_BUS_FEATURE_DIRECT_RX |
	       VIRTIO_MSG_FFA_BUS_FEATURE_DIRECT_TX))))
		methods |= VIRTIO_MSG_FFA_XFER_METHOD_MASK
				(VIRTIO_MSG_FFA_XFER_DIRECT);

	if ((role_mask & (VIRTIO_MSG_FFA_XFER_ROLE_DRIVER |
			  VIRTIO_MSG_FFA_XFER_ROLE_DEVICE)) &&
	    IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_INDIRECT) &&
	    (!negotiated ||
	     ((bus_features & (VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_RX |
			       VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_TX)) ==
	      (VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_RX |
	       VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_TX))))
		methods |= VIRTIO_MSG_FFA_XFER_METHOD_MASK
				(VIRTIO_MSG_FFA_XFER_INDIRECT);

	if ((role_mask & (VIRTIO_MSG_FFA_XFER_ROLE_DRIVER |
			  VIRTIO_MSG_FFA_XFER_ROLE_DEVICE)) &&
	    IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_FIFO) &&
	    (!negotiated ||
	     (bus_features & VIRTIO_MSG_FFA_BUS_FEATURE_FIFO)) &&
	    (methods & VIRTIO_MSG_FFA_XFER_BOOTSTRAP_MASK))
		methods |= VIRTIO_MSG_FFA_XFER_METHOD_MASK
				(VIRTIO_MSG_FFA_XFER_FIFO);

	return methods;
}

static inline unsigned long
virtio_msg_ffa_xfer_bootstrap_methods(u8 role_mask, u32 bus_features,
				      bool negotiated)
{
	return virtio_msg_ffa_xfer_methods(role_mask, bus_features, negotiated) &
	       VIRTIO_MSG_FFA_XFER_BOOTSTRAP_MASK;
}

static inline enum virtio_msg_ffa_transfer_method
virtio_msg_ffa_xfer_target_method(const struct virtio_msg_ffa_endpoint *ep,
				  u8 role_mask,
				  enum virtio_msg_ffa_runtime_target target)
{
	enum virtio_msg_ffa_transfer_method method;
	unsigned long methods;

	if (!ep)
		return VIRTIO_MSG_FFA_XFER_NONE;

	if (target == VIRTIO_MSG_FFA_RUNTIME_TARGET_REQUEST) {
		method = ep->transfer_method;
	} else if (target == VIRTIO_MSG_FFA_RUNTIME_TARGET_EVENT) {
		switch (ep->event_delivery) {
		case VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT:
			method = VIRTIO_MSG_FFA_XFER_INDIRECT;
			break;
		case VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO:
			method = VIRTIO_MSG_FFA_XFER_FIFO;
			break;
		case VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL:
		case VIRTIO_MSG_FFA_BUS_EVENT_DELIV_NOTIF:
			method = VIRTIO_MSG_FFA_XFER_DIRECT;
			break;
		default:
			return VIRTIO_MSG_FFA_XFER_NONE;
		}
	} else {
		return VIRTIO_MSG_FFA_XFER_NONE;
	}

	methods = virtio_msg_ffa_xfer_methods(role_mask, ep->bus_features, true);
	if (!(methods & VIRTIO_MSG_FFA_XFER_METHOD_MASK(method)))
		return VIRTIO_MSG_FFA_XFER_NONE;

	return method;
}

#endif /* _VIRTIO_MSG_BUS_FFA_XFER_H */
