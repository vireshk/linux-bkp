// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A indirect-method framing helpers.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/byteorder/little_endian.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/string.h>

#include "virtio_msg_bus_ffa_xfer_indirect.h"

static int virtio_msg_ffa_indirect_prepare_request
			(struct virtio_msg_ffa_xfer_ctx *ctx,
			 const struct virtio_msg *req, size_t req_len,
			 void *wire_req, size_t wire_req_len,
			 size_t *wire_req_used)
{
	int ret;

	if (!ctx || !ctx->ep || !req || !wire_req)
		return -EINVAL;

	ret = virtio_msg_ffa_validate_inbound_msg(req, req_len);
	if (ret)
		return ret;
	if (wire_req_len < req_len)
		return -EMSGSIZE;

	memcpy(wire_req, req, req_len);

	if (wire_req_used)
		*wire_req_used = req_len;

	return 0;
}

static int virtio_msg_ffa_indirect_parse_frame
			(struct virtio_msg_ffa_xfer_ctx *ctx,
			 const void *wire_resp, size_t wire_resp_len,
			 struct virtio_msg *resp, size_t resp_buf_len,
			 size_t *resp_len)
{
	struct virtio_msg hdr;
	u16 msg_size;
	int ret;

	if (!ctx || !ctx->ep || !wire_resp || !resp || !resp_len)
		return -EINVAL;
	if (wire_resp_len < sizeof(hdr))
		return -EMSGSIZE;

	memcpy(&hdr, wire_resp, sizeof(hdr));
	msg_size = le16_to_cpu(hdr.msg_size);
	if (msg_size < sizeof(hdr) || msg_size > FFA_BUS_MAX_MSG_SIZE)
		return -EMSGSIZE;
	if (wire_resp_len < msg_size || msg_size > resp_buf_len)
		return -EMSGSIZE;

	memcpy(resp, wire_resp, msg_size);
	ret = virtio_msg_ffa_validate_inbound_msg(resp, msg_size);
	if (ret)
		return ret;

	*resp_len = msg_size;

	return 0;
}

static int virtio_msg_ffa_indirect_prepare_wire_request_cb
			(struct virtio_msg_ffa_xfer_ctx *ctx,
			 const struct virtio_msg *req, size_t req_len,
			 void *wire_req, size_t wire_req_len)
{
	return virtio_msg_ffa_indirect_prepare_request(ctx, req, req_len, wire_req,
						       wire_req_len, NULL);
}

static int virtio_msg_ffa_indirect_parse_wire_response_cb
			(struct virtio_msg_ffa_xfer_ctx *ctx,
			 const void *wire_resp, size_t wire_resp_len,
			 struct virtio_msg *resp, size_t resp_buf_len,
			 size_t *resp_len)
{
	return virtio_msg_ffa_indirect_parse_frame(ctx, wire_resp, wire_resp_len,
						   resp, resp_buf_len, resp_len);
}

const struct virtio_msg_ffa_xfer_method_ops
virtio_msg_ffa_indirect_method_ops = {
	.prepare_request = virtio_msg_ffa_indirect_prepare_wire_request_cb,
	.parse_response = virtio_msg_ffa_indirect_parse_wire_response_cb,
};
EXPORT_SYMBOL_GPL(virtio_msg_ffa_indirect_method_ops);
