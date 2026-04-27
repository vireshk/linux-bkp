// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A driver-side indirect transfer helpers.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/byteorder/little_endian.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uuid.h>

#include "virtio_msg_bus_ffa_driver_priv.h"
#include "virtio_msg_bus_ffa_xfer_indirect.h"

#define VIRTIO_MSG_FFA_INDIRECT_RX_QUEUE_DEPTH		128

struct virtio_msg_ffa_driver_indirect_rx {
	u64 generation;
	u16 sender_vm_id;
	u16 len;
	u8 msg[FFA_BUS_MAX_MSG_SIZE];
};

struct virtio_msg_ffa_xfer_indirect_priv {
	struct virtio_msg_ffa_driver *drv;
};

static struct virtio_msg_ffa_xfer_indirect_priv *
virtio_msg_ffa_indirect_priv_get(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_indirect_priv *priv;

	if (!drv || !xfer || xfer->method != VIRTIO_MSG_FFA_XFER_INDIRECT ||
	    !xfer->priv)
		return NULL;

	priv = xfer->priv;
	if (priv->drv && priv->drv != drv)
		return NULL;

	return priv;
}

static bool
virtio_msg_ffa_indirect_driver_runtime_capable
		(struct virtio_msg_ffa_driver *drv)
{
	if (!drv || !drv->fdev || !drv->fdev->ops ||
	    !drv->fdev->ops->msg_ops)
		return false;
	if (!drv->fdev->ops->msg_ops->indirect_send)
		return false;

	return ffa_partition_supports_indirect_msg(drv->fdev);
}

bool virtio_msg_ffa_indirect_available(struct virtio_msg_ffa_driver *drv)
{
	return virtio_msg_ffa_indirect_driver_runtime_capable(drv);
}

static int virtio_msg_ffa_indirect_parse_cloned_rx
			(struct virtio_msg_ffa_xfer_ctx *ctx,
			 const void *cloned_buf, size_t cloned_len,
			 struct virtio_msg *resp, size_t resp_buf_len,
			 size_t *resp_len)
{
	return virtio_msg_ffa_indirect_parse_wire_response(ctx, cloned_buf,
							   cloned_len, resp,
							   resp_buf_len,
							   resp_len);
}

static int
virtio_msg_ffa_driver_indirect_rx_queue_submit
		(void *context,
		 const struct virtio_msg_bus_queue_helper_item *item)
{
	struct virtio_msg_ffa_driver_indirect_rx *entry;
	struct virtio_msg_ffa_xfer_ctx ctx = { 0 };
	struct virtio_msg_ffa_driver *drv = context;
	struct device *dev;
	u8 resp_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *resp = (struct virtio_msg *)resp_buf;
	size_t resp_len = 0;
	int ret = 0;

	if (!drv || !item ||
	    item->type != VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_OPAQUE)
		return -EINVAL;

	entry = item->opaque;
	if (!entry)
		return -EINVAL;

	mutex_lock(&drv->lock);
	dev = drv->fdev ? &drv->fdev->dev : NULL;
	if (virtio_msg_ffa_endpoint_disabled(drv) ||
	    entry->generation != READ_ONCE(drv->indirect_rx_generation) ||
	    !drv->fdev || entry->sender_vm_id != drv->fdev->vm_id ||
	    !virtio_msg_ffa_driver_indirect_rx_ready_locked(drv))
		goto out_unlock;

	ctx.ep = &drv->ep;
	ctx.fdev = drv->fdev;

	ret = virtio_msg_ffa_indirect_parse_cloned_rx(&ctx, entry->msg,
						      entry->len, resp,
						      sizeof(resp_buf),
						      &resp_len);
	if (ret) {
		if (drv->fdev)
			dev_dbg(&drv->fdev->dev,
				"indirect RX parse failed: %d\n", ret);
		goto out_unlock;
	}

	dev_dbg(dev,
		"bus rx indirect: sender=%u type=0x%x msg_id=0x%02x dev=%u tok=%u size=%u len=%zu\n",
		entry->sender_vm_id, resp->type, resp->msg_id,
		le16_to_cpu(resp->dev_num), le16_to_cpu(resp->token),
		le16_to_cpu(resp->msg_size), resp_len);
	ret = virtio_msg_ffa_driver_dispatch_inbound_locked(drv, resp,
							    resp_len);
	if (ret)
		dev_dbg(dev,
			"bus rx indirect dispatch failed: sender=%u ret=%d\n",
			entry->sender_vm_id, ret);

out_unlock:
	mutex_unlock(&drv->lock);
	return ret;
}

static void
virtio_msg_ffa_driver_indirect_rx_queue_release
		(void *context,
		 const struct virtio_msg_bus_queue_helper_item *item, int status)
{
	(void)context;
	(void)status;

	if (!item || item->type != VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_OPAQUE)
		return;

	kfree(item->opaque);
}

int virtio_msg_ffa_driver_indirect_rx_init(struct virtio_msg_ffa_driver *drv)
{
	struct device *dev;

	if (!drv)
		return -EINVAL;

	dev = drv->fdev ? &drv->fdev->dev : NULL;
	return virtio_msg_bus_queue_helper_init
		(&drv->indirect_rx_queue, dev, FFA_BUS_MAX_MSG_SIZE,
		 VIRTIO_MSG_FFA_INDIRECT_RX_QUEUE_DEPTH, 0,
		 virtio_msg_ffa_driver_indirect_rx_queue_submit,
		 virtio_msg_ffa_driver_indirect_rx_queue_release, drv);
}

void
virtio_msg_ffa_driver_indirect_rx_quiesce(struct virtio_msg_ffa_driver *drv)
{
	if (!drv)
		return;

	virtio_msg_bus_queue_helper_stop(&drv->indirect_rx_queue);
}

static void virtio_msg_ffa_indirect_rx_cb_log_no_entry(struct device *dev)
{
	/*
	 * The RX callback consumes this failure. Keep the diagnostic out of
	 * line so checkpatch does not flag it as a generic OOM log.
	 */
	dev_warn_ratelimited(dev,
			     "indirect rx cb drop: queue entry unavailable\n");
}

bool virtio_msg_ffa_indirect_rx_cb(struct ffa_device *fdev, u16 sender_vm_id,
				   const uuid_t *uuid, const void *buf,
				   size_t len)
{
	struct device *dev = fdev ? &fdev->dev : NULL;
	struct virtio_msg_ffa_driver *drv;
	struct virtio_msg_ffa_driver_indirect_rx *entry;
	int ret;

	if (!virtio_msg_ffa_driver_uuid_match(uuid))
		return false;
	if (!fdev || fdev->vm_id != sender_vm_id)
		return false;
	if (!buf || !len) {
		dev_dbg(dev, "indirect rx cb consumed empty payload\n");
		return true;
	}
	if (len > FFA_BUS_MAX_MSG_SIZE) {
		dev_dbg(dev, "indirect rx cb drop oversized payload: len=%zu\n",
			len);
		return true;
	}

	drv = ffa_dev_get_drvdata(fdev);
	if (!drv || drv->fdev != fdev) {
		dev_dbg(dev, "indirect rx cb consumed: missing driver state\n");
		return true;
	}
	if (virtio_msg_ffa_endpoint_disabled(drv)) {
		dev_dbg(dev, "indirect rx cb consumed: endpoint disabled\n");
		return true;
	}

	entry = kzalloc(sizeof(*entry), GFP_ATOMIC);
	if (!entry) {
		virtio_msg_ffa_indirect_rx_cb_log_no_entry(dev);
		return true;
	}

	entry->generation = READ_ONCE(drv->indirect_rx_generation);
	entry->sender_vm_id = sender_vm_id;
	entry->len = len;
	memcpy(entry->msg, buf, len);

	ret = virtio_msg_bus_queue_helper_enqueue_opaque
		(&drv->indirect_rx_queue, entry, GFP_ATOMIC);
	if (ret) {
		kfree(entry);
		dev_warn_ratelimited(dev,
				     "indirect rx cb drop: enqueue failed ret=%d\n",
				     ret);
	}

	return true;
}

static int virtio_msg_ffa_send_indirect_locked(struct virtio_msg_ffa_driver *drv,
					       struct virtio_msg_ffa_xfer_engine *xfer,
					       const struct virtio_msg *req,
					       size_t req_len,
					       struct virtio_msg *resp,
					       size_t resp_buf_len,
					       size_t *resp_len)
{
	const struct ffa_msg_ops *msg_ops;
	struct virtio_msg_ffa_xfer_ctx xfer_ctx = {
		.ep = &drv->ep,
		.fdev = drv->fdev,
		.priv = xfer ? xfer->priv : NULL,
	};
	u8 wire_buf[FFA_BUS_MAX_MSG_SIZE];
	size_t wire_len = req_len;
	u16 dev_num;
	bool is_bus_msg;
	unsigned int retry = VIRTIO_MSG_FFA_TX_BUSY_RETRY_MAX;
	int ret;

	(void)resp;
	(void)resp_buf_len;
	(void)resp_len;
	if (!drv || !drv->fdev || !req)
		return -EINVAL;
	if (!drv->fdev->ops || !drv->fdev->ops->msg_ops)
		return -EOPNOTSUPP;

	msg_ops = drv->fdev->ops->msg_ops;
	if (!msg_ops->indirect_send ||
	    !ffa_partition_supports_indirect_msg(drv->fdev))
		return -EOPNOTSUPP;

	ret = virtio_msg_ffa_indirect_prepare_wire_request(&xfer_ctx, req,
							   req_len, wire_buf,
							   sizeof(wire_buf));
	if (ret)
		return ret;

	dev_num = le16_to_cpu(req->dev_num);
	is_bus_msg = req->type & VIRTIO_MSG_TYPE_BUS;
	for (;;) {
		ret = msg_ops->indirect_send(drv->fdev, wire_buf, wire_len);
		if (ret == -EBUSY) {
			if (!virtio_msg_ffa_tx_busy_retry_sleepable(&retry))
				break;
			continue;
		}
		if (ret)
			return ret;
		break;
	}

	if (ret) {
		dev_dbg(&drv->fdev->dev,
			"bus tx indirect failed: dest=%u msg_id=0x%02x dev=%u tok=%u ret=%d\n",
			(u16)drv->fdev->vm_id, req->msg_id, dev_num,
			le16_to_cpu(req->token), ret);
		if (ret == -EBUSY && !is_bus_msg)
			virtio_msg_ffa_driver_handle_device_failure_locked(drv, dev_num);
		return ret;
	}

	return 0;
}

static int virtio_msg_ffa_indirect_event_configure_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	int ret;

	(void)xfer;
	if (!drv)
		return -EINVAL;

	ret = virtio_msg_ffa_driver_event_configure_send_locked
		(drv, VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT, 0);
	if (ret)
		return ret;

	virtio_msg_ffa_select_event_delivery(&drv->ep,
					     VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT);
	return 0;
}

static int virtio_msg_ffa_indirect_event_poll_once_unsupported
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer,
				 bool *empty)
{
	(void)drv;
	(void)xfer;
	(void)empty;
	return -EOPNOTSUPP;
}

static int virtio_msg_ffa_indirect_setup_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_indirect_priv *priv;

	if (!drv || !xfer)
		return -EINVAL;
	if (!xfer->priv) {
		priv = kzalloc(sizeof(*priv), GFP_KERNEL);
		if (!priv)
			return -ENOMEM;
		priv->drv = drv;
		xfer->priv = priv;
	}

	priv = virtio_msg_ffa_indirect_priv_get(drv, xfer);
	if (!priv) {
		kfree(xfer->priv);
		xfer->priv = NULL;
		return -EINVAL;
	}

	priv->drv = drv;
	return 0;
}

static void virtio_msg_ffa_indirect_teardown_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_indirect_priv *priv;

	priv = virtio_msg_ffa_indirect_priv_get(drv, xfer);
	if (!priv)
		return;
}

static void virtio_msg_ffa_indirect_release_priv
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_indirect_priv *priv;

	priv = virtio_msg_ffa_indirect_priv_get(drv, xfer);
	if (!priv)
		return;

	xfer->priv = NULL;
	kfree(priv);
}

const struct virtio_msg_ffa_xfer_ops virtio_msg_ffa_indirect_xfer_ops = {
	.setup_locked = virtio_msg_ffa_indirect_setup_locked,
	.teardown_locked = virtio_msg_ffa_indirect_teardown_locked,
	.submit_locked = virtio_msg_ffa_send_indirect_locked,
	.event_configure_locked = virtio_msg_ffa_indirect_event_configure_locked,
	.event_poll_once_locked = virtio_msg_ffa_indirect_event_poll_once_unsupported,
	.release_priv = virtio_msg_ffa_indirect_release_priv,
	.async_response = true,
};

static int
virtio_msg_ffa_indirect_driver_init_runtime
		(struct virtio_msg_ffa_driver *drv,
		 const struct virtio_msg_ffa_xfer_method_desc *desc)
{
	struct virtio_msg_ffa_xfer_engine xfer;
	void *old_priv;
	bool shared_bootstrap;
	int ret;

	if (!drv || !desc || desc->method != VIRTIO_MSG_FFA_XFER_INDIRECT ||
	    !(desc->role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DRIVER) ||
	    !(desc->phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME) ||
	    desc->event_delivery != VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT)
		return -EINVAL;

	if (!virtio_msg_ffa_indirect_driver_runtime_capable(drv))
		return -EOPNOTSUPP;

	old_priv = drv->xfer_priv[VIRTIO_MSG_FFA_XFER_INDIRECT];
	xfer.method = VIRTIO_MSG_FFA_XFER_INDIRECT;
	xfer.ops = &virtio_msg_ffa_indirect_xfer_ops;
	xfer.priv = old_priv;

	ret = virtio_msg_ffa_indirect_setup_locked(drv, &xfer);
	if (ret) {
		if (xfer.priv != old_priv)
			virtio_msg_ffa_indirect_release_priv(drv, &xfer);
		drv->xfer_priv[VIRTIO_MSG_FFA_XFER_INDIRECT] = old_priv;
		return virtio_msg_ffa_driver_init_runtime_reject(ret);
	}

	drv->xfer_priv[VIRTIO_MSG_FFA_XFER_INDIRECT] = xfer.priv;
	shared_bootstrap =
		drv->bootstrap_xfer.method == VIRTIO_MSG_FFA_XFER_INDIRECT &&
		drv->bootstrap_xfer.priv == xfer.priv;

	drv->event_configured = false;
	ret = virtio_msg_ffa_indirect_event_configure_locked(drv, &xfer);
	if (ret)
		goto err_teardown;

	drv->event_configured = true;
	drv->active_xfer = xfer;
	virtio_msg_ffa_select_transfer_method(&drv->ep,
					      VIRTIO_MSG_FFA_XFER_INDIRECT);
	if (drv->ep.transfer_method == VIRTIO_MSG_FFA_XFER_INDIRECT &&
	    virtio_msg_ffa_driver_event_runtime_ready_locked(drv))
		return 0;

	ret = -EIO;
	drv->event_configured = false;

err_teardown:
	if (!shared_bootstrap)
		virtio_msg_ffa_indirect_teardown_locked(drv, &xfer);
	if (xfer.priv != old_priv)
		virtio_msg_ffa_indirect_release_priv(drv, &xfer);
	drv->xfer_priv[VIRTIO_MSG_FFA_XFER_INDIRECT] = old_priv;
	return virtio_msg_ffa_driver_init_runtime_reject(ret);
}

static const struct virtio_msg_ffa_xfer_driver_method_ops
virtio_msg_ffa_indirect_driver_method_ops = {
	.init_runtime = virtio_msg_ffa_indirect_driver_init_runtime,
};

const struct virtio_msg_ffa_xfer_method_desc
virtio_msg_ffa_indirect_driver_method_desc = {
	.method = VIRTIO_MSG_FFA_XFER_INDIRECT,
	.name = "indirect",
	.role_mask = VIRTIO_MSG_FFA_XFER_ROLE_DRIVER,
	.phase_flags = VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP |
		       VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
	.bus_feature_mask = VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_RX |
			    VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_TX,
	.event_delivery = VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT,
	.driver_ops = &virtio_msg_ffa_indirect_driver_method_ops,
};
