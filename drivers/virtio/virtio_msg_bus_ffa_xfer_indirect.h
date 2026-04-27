/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus over FF-A indirect transfer helper contracts.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_FFA_XFER_INDIRECT_H
#define _VIRTIO_MSG_BUS_FFA_XFER_INDIRECT_H

#include "virtio_msg_bus_ffa_xfer.h"

#ifdef CONFIG_VIRTIO_MSG_FFA_XFER_INDIRECT
extern const struct virtio_msg_ffa_xfer_method_ops
	virtio_msg_ffa_indirect_method_ops;

static inline int
virtio_msg_ffa_indirect_prepare_wire_request(struct virtio_msg_ffa_xfer_ctx *ctx,
					     const struct virtio_msg *req,
					     size_t req_len, void *wire_req,
					     size_t wire_req_len)
{
	if (!virtio_msg_ffa_indirect_method_ops.prepare_request)
		return -EOPNOTSUPP;

	return virtio_msg_ffa_indirect_method_ops.prepare_request
			(ctx, req, req_len, wire_req, wire_req_len);
}

static inline int
virtio_msg_ffa_indirect_parse_wire_response(struct virtio_msg_ffa_xfer_ctx *ctx,
					    const void *wire_resp,
					    size_t wire_resp_len,
					    struct virtio_msg *resp,
					    size_t resp_buf_len,
					    size_t *resp_len)
{
	if (!virtio_msg_ffa_indirect_method_ops.parse_response)
		return -EOPNOTSUPP;

	return virtio_msg_ffa_indirect_method_ops.parse_response
			(ctx, wire_resp, wire_resp_len, resp, resp_buf_len,
			 resp_len);
}
#endif

#endif /* _VIRTIO_MSG_BUS_FFA_XFER_INDIRECT_H */
