// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A direct-method framing helpers.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/byteorder/little_endian.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/virtio_msg_bus_provider.h>
#include <linux/virtio_msg_protocol.h>
#include <linux/workqueue.h>

#include "virtio_msg_bus_ffa_driver_priv.h"
#include "virtio_msg_bus_ffa_xfer.h"

#define VIRTIO_MSG_FFA_DIRECT_RETRY_BUDGET	3
#define VIRTIO_MSG_FFA_EVENT_POLL_DRAIN_BUDGET	64
#define VIRTIO_MSG_FFA_EVENT_POLL_INTERVAL_MS	100

struct virtio_msg_ffa_xfer_direct_priv {
	struct virtio_msg_ffa_driver *drv;
	u16 event_notif_id;
	bool event_notif_bound;
	bool event_poll_quiesced;
	bool event_poll_work_inited;
	struct delayed_work event_poll_work;
};

static bool virtio_msg_ffa_direct_is_busy_sentinel
			(const struct ffa_send_direct_data2 *resp)
{
	int i;

	if (!resp)
		return false;

	for (i = 0; i < ARRAY_SIZE(resp->data); i++) {
		if (resp->data[i])
			return false;
	}

	return true;
}

static int virtio_msg_ffa_direct_prepare_request
			(struct virtio_msg_ffa_xfer_ctx *ctx,
			 const struct virtio_msg *req, size_t req_len,
			 struct ffa_send_direct_data2 *wire_req)
{
	int ret;

	if (!ctx || !ctx->ep || !req || !wire_req)
		return -EINVAL;

	ret = virtio_msg_ffa_validate_inbound_msg(req, req_len);
	if (ret)
		return ret;
	if (req_len > sizeof(*wire_req))
		return -EMSGSIZE;

	memset(wire_req, 0, sizeof(*wire_req));
	memcpy(wire_req, req, req_len);

	return 0;
}

static int virtio_msg_ffa_direct_parse_response
			(struct virtio_msg_ffa_xfer_ctx *ctx,
			 const struct ffa_send_direct_data2 *wire_resp,
			 struct virtio_msg *resp, size_t resp_buf_len,
			 size_t *resp_len)
{
	struct virtio_msg hdr;
	u8 raw[sizeof(*wire_resp)];
	u16 msg_size;
	int ret;

	if (!ctx || !ctx->ep || !wire_resp || !resp || !resp_len)
		return -EINVAL;

	if (virtio_msg_ffa_direct_is_busy_sentinel(wire_resp))
		return -EAGAIN;

	memcpy(raw, wire_resp, sizeof(raw));
	memcpy(&hdr, raw, sizeof(hdr));
	msg_size = le16_to_cpu(hdr.msg_size);
	if (msg_size < sizeof(hdr) || msg_size > sizeof(raw))
		return -EMSGSIZE;
	if (msg_size > resp_buf_len)
		return -EMSGSIZE;

	memcpy(resp, raw, msg_size);
	ret = virtio_msg_ffa_validate_inbound_msg(resp, msg_size);
	if (ret)
		return ret;

	*resp_len = msg_size;
	return 0;
}

static bool virtio_msg_ffa_event_notif_capable(struct virtio_msg_ffa_driver *drv)
{
	return drv && drv->fdev &&
	       ffa_partition_supports_notify_recv(drv->fdev) &&
	       drv->fdev->ops && drv->fdev->ops->notifier_ops;
}

static void virtio_msg_ffa_direct_event_poll_work(struct work_struct *work);

static const struct ffa_notifier_ops *
virtio_msg_ffa_notifier_ops_get(struct ffa_device *fdev)
{
	if (!fdev || !fdev->ops)
		return NULL;

	return fdev->ops->notifier_ops;
}

static struct virtio_msg_ffa_xfer_direct_priv *
virtio_msg_ffa_direct_priv_get(struct virtio_msg_ffa_driver *drv,
			       struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_direct_priv *priv;

	if (!drv || !xfer || xfer->method != VIRTIO_MSG_FFA_XFER_DIRECT ||
	    !xfer->priv)
		return NULL;

	priv = xfer->priv;
	if (priv->drv && priv->drv != drv)
		return NULL;

	return priv;
}

static bool virtio_msg_ffa_direct_notif_id_is_valid(u16 notif_id)
{
	return notif_id >= VIRTIO_MSG_FFA_NOTIF_ID_MIN &&
	       notif_id <= VIRTIO_MSG_FFA_NOTIF_ID_MAX;
}

static void virtio_msg_ffa_direct_state_reset_locked
		(struct virtio_msg_ffa_xfer_direct_priv *priv)
{
	if (!priv)
		return;

	priv->event_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	priv->event_notif_bound = false;
	priv->event_poll_quiesced = true;
}

static int virtio_msg_ffa_direct_state_prepare_locked
		(struct virtio_msg_ffa_driver *drv,
		 struct virtio_msg_ffa_xfer_engine *xfer,
		 struct virtio_msg_ffa_xfer_direct_priv **out_priv)
{
	struct virtio_msg_ffa_xfer_direct_priv *priv;

	if (!xfer->priv) {
		xfer->priv = kzalloc(sizeof(*priv), GFP_KERNEL);
		if (!xfer->priv)
			return -ENOMEM;
	}

	priv = virtio_msg_ffa_direct_priv_get(drv, xfer);
	if (!priv) {
		kfree(xfer->priv);
		xfer->priv = NULL;
		return -EINVAL;
	}

	if (!priv->event_poll_work_inited) {
		INIT_DELAYED_WORK(&priv->event_poll_work,
				  virtio_msg_ffa_direct_event_poll_work);
		priv->event_poll_work_inited = true;
		virtio_msg_ffa_direct_state_reset_locked(priv);
	}

	priv->drv = drv;
	if (out_priv)
		*out_priv = priv;

	return 0;
}

static void virtio_msg_ffa_direct_quiesce_locked
		(struct virtio_msg_ffa_driver *drv,
		 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_direct_priv *priv;

	priv = virtio_msg_ffa_direct_priv_get(drv, xfer);
	if (!priv)
		return;

	priv->event_poll_quiesced = true;
}

static void virtio_msg_ffa_event_poll_schedule_locked
				(struct virtio_msg_ffa_xfer_direct_priv *priv,
				 unsigned long delay_ms)
{
	if (!priv || priv->event_poll_quiesced || !priv->event_poll_work_inited)
		return;

	schedule_delayed_work(&priv->event_poll_work,
			      msecs_to_jiffies(delay_ms));
}

static void virtio_msg_ffa_event_notif_cb(int notify_id, void *cb_data)
{
	struct virtio_msg_ffa_xfer_direct_priv *priv = cb_data;
	struct virtio_msg_ffa_driver *drv;

	(void)notify_id;
	if (!priv)
		return;

	drv = priv->drv;
	if (!drv)
		return;

	mutex_lock(&drv->lock);
	if (!priv->event_notif_bound || !drv->event_configured ||
	    virtio_msg_ffa_endpoint_disabled(drv) ||
	    drv->active_xfer.method != VIRTIO_MSG_FFA_XFER_DIRECT ||
	    drv->active_xfer.priv != priv ||
	    drv->ep.event_delivery != VIRTIO_MSG_FFA_BUS_EVENT_DELIV_NOTIF) {
		mutex_unlock(&drv->lock);
		return;
	}

	priv->event_poll_quiesced = false;
	virtio_msg_ffa_event_poll_schedule_locked(priv, 0);
	mutex_unlock(&drv->lock);
}

static int virtio_msg_ffa_event_notif_bind_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_direct_priv *priv,
				 u16 *notif_id)
{
	const struct ffa_notifier_ops *notifier_ops;
	int id;
	int ret;

	if (!drv || !priv || !notif_id)
		return -EINVAL;

	if (priv->event_notif_bound) {
		*notif_id = priv->event_notif_id;
		return 0;
	}

	notifier_ops = virtio_msg_ffa_notifier_ops_get(drv->fdev);
	if (!notifier_ops || !notifier_ops->notify_alloc)
		return -EOPNOTSUPP;

	ret = notifier_ops->notify_alloc(drv->fdev, false,
					 virtio_msg_ffa_event_notif_cb,
					 priv, &id);
	if (ret)
		return ret;
	if (id > U16_MAX || !virtio_msg_ffa_direct_notif_id_is_valid((u16)id)) {
		if (notifier_ops->notify_relinquish)
			notifier_ops->notify_relinquish(drv->fdev, id);
		return -ERANGE;
	}

	priv->event_notif_id = (u16)id;
	priv->event_notif_bound = true;
	*notif_id = (u16)id;
	return 0;
}

static int virtio_msg_ffa_event_notif_unbind_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_direct_priv *priv)
{
	const struct ffa_notifier_ops *notifier_ops;
	int ret;

	if (!drv || !priv || !priv->event_notif_bound)
		return 0;

	notifier_ops = virtio_msg_ffa_notifier_ops_get(drv->fdev);
	if (!notifier_ops || !notifier_ops->notify_relinquish)
		return -EOPNOTSUPP;

	ret = notifier_ops->notify_relinquish(drv->fdev, priv->event_notif_id);
	if (ret)
		return ret;

	priv->event_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	priv->event_notif_bound = false;
	return 0;
}

static int virtio_msg_ffa_direct_send_locked(struct virtio_msg_ffa_driver *drv,
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
	struct ffa_send_direct_data2 wire_msg;
	int retry = VIRTIO_MSG_FFA_DIRECT_RETRY_BUDGET;
	int ret;

	if (!drv || !drv->fdev || !req || !resp || !resp_len)
		return -EINVAL;
	if (!drv->fdev->ops || !drv->fdev->ops->msg_ops)
		return -EOPNOTSUPP;

	msg_ops = drv->fdev->ops->msg_ops;
	if (!msg_ops->sync_send_receive2 ||
	    !ffa_partition_supports_direct_req2_recv(drv->fdev))
		return -EOPNOTSUPP;

	while (retry--) {
		ret = virtio_msg_ffa_direct_prepare_request(&xfer_ctx, req, req_len,
							    &wire_msg);
		if (ret)
			return ret;

		ret = msg_ops->sync_send_receive2(drv->fdev, &wire_msg);
		if (ret == -EBUSY)
			continue;
		if (ret)
			return ret;

		if (virtio_msg_ffa_direct_is_busy_sentinel(&wire_msg))
			continue;

		return virtio_msg_ffa_direct_parse_response(&xfer_ctx, &wire_msg,
							    resp, resp_buf_len,
							    resp_len);
	}

	return -EBUSY;
}

bool virtio_msg_ffa_direct_available(struct virtio_msg_ffa_driver *drv)
{
	if (!drv || !drv->fdev || !drv->fdev->ops || !drv->fdev->ops->msg_ops)
		return false;
	if (!drv->fdev->ops->msg_ops->sync_send_receive2)
		return false;

	return ffa_partition_supports_direct_req2_recv(drv->fdev);
}

static int virtio_msg_ffa_event_poll_once_locked(struct virtio_msg_ffa_driver *drv,
						 struct virtio_msg_ffa_xfer_engine *xfer,
						 bool *empty)
{
	u8 req_buf[FFA_BUS_MAX_MSG_SIZE];
	u8 resp_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *req = (struct virtio_msg *)req_buf;
	struct virtio_msg *resp = (struct virtio_msg *)resp_buf;
	size_t req_len;
	size_t resp_len = 0;
	u8 expected_type;
	u16 token;
	int ret;

	if (!drv || !empty)
		return -EINVAL;
	if (drv->ep.event_delivery != VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL &&
	    drv->ep.event_delivery != VIRTIO_MSG_FFA_BUS_EVENT_DELIV_NOTIF)
		return -EOPNOTSUPP;

	memset(req_buf, 0, sizeof(req_buf));
	memset(resp_buf, 0, sizeof(resp_buf));
	ret = virtio_msg_ffa_next_token_locked(&drv->ep, &token);
	if (ret)
		return ret;
	virtio_msg_prepare(req, FFA_BUS_MSG_EVENT_POLL, token, 0, 0);
	req->type = VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_REQUEST;
	req_len = sizeof(*req);

	virtio_msg_ffa_driver_trace_msg(drv, "tx", "direct", req, req_len);
	ret = virtio_msg_ffa_direct_send_locked(drv, xfer, req, req_len, resp,
						sizeof(resp_buf), &resp_len);
	if (ret) {
		if (drv->fdev)
			dev_dbg(&drv->fdev->dev,
				"bus tx direct failed: dev=%u tok=%u ret=%d\n",
				le16_to_cpu(req->dev_num),
				le16_to_cpu(req->token), ret);
		else
			pr_debug("bus tx direct failed: dev=%u tok=%u ret=%d\n",
				 le16_to_cpu(req->dev_num),
				 le16_to_cpu(req->token), ret);
		return ret;
	}

	ret = virtio_msg_ffa_validate_inbound_msg(resp, resp_len);
	if (ret)
		return ret;

	virtio_msg_ffa_driver_trace_msg(drv, "rx", "direct", resp, resp_len);
	if (resp->msg_id == FFA_BUS_MSG_EVENT_POLL) {
		expected_type = VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_RESPONSE;
		if (resp->type != expected_type)
			return -EPROTO;
		if (le16_to_cpu(resp->dev_num) != 0)
			return -EPROTO;
		if (le16_to_cpu(resp->token) != le16_to_cpu(req->token))
			return -EPROTO;
		if (le16_to_cpu(resp->msg_size) != sizeof(*resp) ||
		    resp_len != sizeof(*resp))
			return -EPROTO;
		*empty = true;
		return 0;
	}

	if (!virtio_msg_ffa_msg_is_event(resp->msg_id))
		return -EPROTO;

	*empty = false;
	return virtio_msg_ffa_driver_dispatch_inbound_locked(drv, resp, resp_len);
}

static int virtio_msg_ffa_direct_event_configure_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_direct_priv *priv;
	u16 notif_id = 0;
	int unbind_ret;
	int ret;

	if (!drv)
		return -EINVAL;

	ret = virtio_msg_ffa_direct_state_prepare_locked(drv, xfer, &priv);
	if (ret)
		return ret;

	virtio_msg_ffa_direct_quiesce_locked(drv, xfer);

	if (virtio_msg_ffa_event_notif_capable(drv) &&
	    (drv->ep.bus_features & VIRTIO_MSG_FFA_BUS_FEATURE_NOTIF_RX) &&
	    (drv->ep.bus_features & VIRTIO_MSG_FFA_BUS_FEATURE_NOTIF_TX)) {
		ret = virtio_msg_ffa_event_notif_bind_locked(drv, priv, &notif_id);
		if (!ret) {
			ret = virtio_msg_ffa_driver_event_configure_send_locked
				(drv, VIRTIO_MSG_FFA_BUS_EVENT_DELIV_NOTIF,
				 notif_id);
			if (!ret) {
				virtio_msg_ffa_select_event_delivery
					(&drv->ep, VIRTIO_MSG_FFA_BUS_EVENT_DELIV_NOTIF);
				return 0;
			}
			unbind_ret = virtio_msg_ffa_event_notif_unbind_locked(drv, priv);
			if (unbind_ret && drv->fdev)
				dev_warn_ratelimited
					(&drv->fdev->dev,
					 "EVENT notification unbind failed: %d\n",
					 unbind_ret);
		}
	}

	ret = virtio_msg_ffa_driver_event_configure_send_locked
		(drv, VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL, 0);
	if (ret)
		return ret;

	priv->event_poll_quiesced = false;
	virtio_msg_ffa_select_event_delivery(&drv->ep,
					     VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL);
	virtio_msg_ffa_event_poll_schedule_locked(priv,
						  VIRTIO_MSG_FFA_EVENT_POLL_INTERVAL_MS);
	return 0;
}

static void virtio_msg_ffa_direct_teardown_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_direct_priv *priv;
	int ret;

	priv = virtio_msg_ffa_direct_priv_get(drv, xfer);
	if (!priv)
		return;

	virtio_msg_ffa_direct_quiesce_locked(drv, xfer);
	if (priv->event_poll_work_inited)
		cancel_delayed_work(&priv->event_poll_work);
	ret = virtio_msg_ffa_event_notif_unbind_locked(drv, priv);
	if (ret) {
		if (drv->fdev)
			dev_warn_ratelimited
				(&drv->fdev->dev,
				 "EVENT notification unbind failed: %d\n", ret);
		priv->event_poll_quiesced = true;
		return;
	}
	virtio_msg_ffa_direct_state_reset_locked(priv);
}

static int virtio_msg_ffa_direct_setup_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	return virtio_msg_ffa_direct_state_prepare_locked(drv, xfer, NULL);
}

static void virtio_msg_ffa_direct_release_priv
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_direct_priv *priv;

	priv = virtio_msg_ffa_direct_priv_get(drv, xfer);
	if (!priv)
		return;

	if (priv->event_poll_work_inited)
		cancel_delayed_work_sync(&priv->event_poll_work);

	xfer->priv = NULL;
	kfree(priv);
}

static void virtio_msg_ffa_direct_event_poll_work(struct work_struct *work)
{
	struct virtio_msg_ffa_xfer_direct_priv *priv =
		container_of(to_delayed_work(work),
			     struct virtio_msg_ffa_xfer_direct_priv,
			     event_poll_work);
	struct virtio_msg_ffa_driver *drv;
	int budget = VIRTIO_MSG_FFA_EVENT_POLL_DRAIN_BUDGET;
	int ret = 0;

	drv = priv->drv;
	if (!drv)
		return;

	mutex_lock(&drv->lock);
	if (priv->event_poll_quiesced || drv->endpoint_disabled ||
	    !drv->event_configured ||
	    drv->active_xfer.method != VIRTIO_MSG_FFA_XFER_DIRECT ||
	    drv->active_xfer.priv != priv ||
	    (drv->ep.event_delivery != VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL &&
	     drv->ep.event_delivery != VIRTIO_MSG_FFA_BUS_EVENT_DELIV_NOTIF)) {
		mutex_unlock(&drv->lock);
		return;
	}

	while (budget--) {
		bool empty = false;

		ret = virtio_msg_ffa_event_poll_once_locked
				(drv, &drv->active_xfer, &empty);
		if (ret)
			break;

		if (empty) {
			if (drv->ep.event_delivery ==
			    VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL) {
				priv->event_poll_quiesced = false;
				virtio_msg_ffa_event_poll_schedule_locked
					(priv, VIRTIO_MSG_FFA_EVENT_POLL_INTERVAL_MS);
			} else {
				priv->event_poll_quiesced = true;
			}
			mutex_unlock(&drv->lock);
			return;
		}
	}

	if (ret && drv->fdev)
		dev_warn_ratelimited(&drv->fdev->dev,
				     "EVENT_POLL failed: %d\n", ret);

	if (ret) {
		if (drv->ep.event_delivery ==
		    VIRTIO_MSG_FFA_BUS_EVENT_DELIV_NOTIF) {
			priv->event_poll_quiesced = true;
		} else {
			priv->event_poll_quiesced = false;
			virtio_msg_ffa_event_poll_schedule_locked
				(priv, VIRTIO_MSG_FFA_EVENT_POLL_INTERVAL_MS);
		}
	} else {
		priv->event_poll_quiesced = false;
		virtio_msg_ffa_event_poll_schedule_locked(priv, 0);
	}

	mutex_unlock(&drv->lock);
}

const struct virtio_msg_ffa_xfer_ops virtio_msg_ffa_direct_xfer_ops = {
	.setup_locked = virtio_msg_ffa_direct_setup_locked,
	.teardown_locked = virtio_msg_ffa_direct_teardown_locked,
	.quiesce_locked = virtio_msg_ffa_direct_quiesce_locked,
	.submit_locked = virtio_msg_ffa_direct_send_locked,
	.event_configure_locked = virtio_msg_ffa_direct_event_configure_locked,
	.event_poll_once_locked = virtio_msg_ffa_event_poll_once_locked,
	.release_priv = virtio_msg_ffa_direct_release_priv,
};

static int
virtio_msg_ffa_direct_driver_init_runtime
		(struct virtio_msg_ffa_driver *drv,
		 const struct virtio_msg_ffa_xfer_method_desc *desc)
{
	struct virtio_msg_ffa_xfer_engine xfer;
	void *old_priv;
	bool shared_bootstrap;
	int ret;

	if (!drv || !desc || desc->method != VIRTIO_MSG_FFA_XFER_DIRECT ||
	    !(desc->role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DRIVER) ||
	    !(desc->phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME))
		return -EINVAL;

	old_priv = drv->xfer_priv[VIRTIO_MSG_FFA_XFER_DIRECT];
	xfer.method = VIRTIO_MSG_FFA_XFER_DIRECT;
	xfer.ops = &virtio_msg_ffa_direct_xfer_ops;
	xfer.priv = old_priv;

	ret = virtio_msg_ffa_direct_setup_locked(drv, &xfer);
	if (ret) {
		if (xfer.priv != old_priv)
			virtio_msg_ffa_direct_release_priv(drv, &xfer);
		drv->xfer_priv[VIRTIO_MSG_FFA_XFER_DIRECT] = old_priv;
		return virtio_msg_ffa_driver_init_runtime_reject(ret);
	}

	drv->xfer_priv[VIRTIO_MSG_FFA_XFER_DIRECT] = xfer.priv;
	shared_bootstrap =
		drv->bootstrap_xfer.method == VIRTIO_MSG_FFA_XFER_DIRECT &&
		drv->bootstrap_xfer.priv == xfer.priv;

	drv->event_configured = false;
	ret = virtio_msg_ffa_direct_event_configure_locked(drv, &xfer);
	if (ret)
		goto err_teardown;

	drv->event_configured = true;
	drv->active_xfer = xfer;
	virtio_msg_ffa_select_transfer_method(&drv->ep,
					      VIRTIO_MSG_FFA_XFER_DIRECT);
	if (virtio_msg_ffa_driver_event_runtime_ready_locked(drv))
		return 0;

	ret = -EIO;
	drv->event_configured = false;

err_teardown:
	if (!shared_bootstrap)
		virtio_msg_ffa_direct_teardown_locked(drv, &xfer);
	if (xfer.priv != old_priv)
		virtio_msg_ffa_direct_release_priv(drv, &xfer);
	drv->xfer_priv[VIRTIO_MSG_FFA_XFER_DIRECT] = old_priv;
	return virtio_msg_ffa_driver_init_runtime_reject(ret);
}

static const struct virtio_msg_ffa_xfer_driver_method_ops
virtio_msg_ffa_direct_driver_method_ops = {
	.init_runtime = virtio_msg_ffa_direct_driver_init_runtime,
};

const struct virtio_msg_ffa_xfer_method_desc
virtio_msg_ffa_direct_driver_method_desc = {
	.method = VIRTIO_MSG_FFA_XFER_DIRECT,
	.name = "direct",
	.role_mask = VIRTIO_MSG_FFA_XFER_ROLE_DRIVER,
	.phase_flags = VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP |
		       VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
	.bus_feature_mask = VIRTIO_MSG_FFA_BUS_FEATURE_DIRECT_RX |
			    VIRTIO_MSG_FFA_BUS_FEATURE_DIRECT_TX,
	.event_delivery = VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL,
	.driver_ops = &virtio_msg_ffa_direct_driver_method_ops,
};
