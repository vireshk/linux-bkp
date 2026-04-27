// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A driver binding shell.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-ffa: " fmt

#include <asm/barrier.h>
#include <linux/arm_ffa.h>
#include <linux/bitops.h>
#include <linux/byteorder/generic.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/rcupdate.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uuid.h>
#include <linux/vmalloc.h>
#include <linux/virtio_msg_bus_provider.h>
#include <linux/virtio_msg_transport.h>
#include <linux/workqueue.h>

#include "virtio_msg_bus_ffa.h"
#include "virtio_msg_bus_ffa_driver_priv.h"
#include "virtio_msg_bus_ffa_xfer.h"

#define VIRTIO_MSG_FFA_RESET_RETRY_BUDGET	3
#define VIRTIO_MSG_FFA_ASYNC_TIMEOUT_MS			5000

static bool virtio_msg_ffa_type_match_masked(u8 type, u8 expected_type)
{
	const u8 type_mask = VIRTIO_MSG_TYPE_RESPONSE | VIRTIO_MSG_TYPE_BUS;

	return (type & type_mask) == (expected_type & type_mask);
}

static const char *
virtio_msg_ffa_xfer_method_name(enum virtio_msg_ffa_transfer_method method)
{
	switch (method) {
	case VIRTIO_MSG_FFA_XFER_DIRECT:
		return "direct";
	case VIRTIO_MSG_FFA_XFER_INDIRECT:
		return "indirect";
	case VIRTIO_MSG_FFA_XFER_FIFO:
		return "fifo";
	default:
		return "none";
	}
}

static const char *
virtio_msg_ffa_event_delivery_name(enum virtio_msg_ffa_bus_event_delivery method)
{
	switch (method) {
	case VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL:
		return "poll";
	case VIRTIO_MSG_FFA_BUS_EVENT_DELIV_NOTIF:
		return "notify";
	case VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT:
		return "indirect";
	case VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO:
		return "fifo";
	default:
		return "unknown";
	}
}

void virtio_msg_ffa_driver_trace_msg(struct virtio_msg_ffa_driver *drv,
				     const char *dir, const char *path,
				     const struct virtio_msg *msg,
				     size_t len)
{
	struct device *dev;
	u16 peer_vm;

	if (!drv || !msg)
		return;

	dev = drv->fdev ? &drv->fdev->dev : NULL;
	peer_vm = drv->fdev ? (u16)drv->fdev->vm_id : 0;
	virtio_msg_ffa_trace_msg(dev, peer_vm, dir, path, msg, len);
}

static const uuid_t virtio_msg_ffa_device_uuid = VIRTIO_MSG_FFA_DEVICE_UUID;
static const uuid_t virtio_msg_ffa_driver_uuid = VIRTIO_MSG_FFA_DRIVER_UUID;

static const struct ffa_device_id virtio_msg_ffa_local_uuid_ids[] = {
	{ .uuid = virtio_msg_ffa_driver_uuid },
	{}
};

bool virtio_msg_ffa_driver_uuid_match(const uuid_t *uuid)
{
	return uuid && uuid_equal(uuid, &virtio_msg_ffa_driver_uuid);
}

struct virtio_msg_ffa_service_user {
	struct list_head node;
	struct virtio_msg_ffa_driver *drv;
};

static DEFINE_MUTEX(virtio_msg_ffa_service_lock);
static int virtio_msg_ffa_service_users;
static LIST_HEAD(virtio_msg_ffa_service_list);
static struct ffa_driver virtio_msg_ffa_driver;

static int virtio_msg_ffa_prepare_runtime_locked
					(struct virtio_msg_ffa_driver *drv);
static int virtio_msg_ffa_send_by_selected_method_locked
					(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer,
				 const struct virtio_msg *req, size_t req_len,
				 struct virtio_msg *resp, size_t resp_buf_len,
				 size_t *resp_len);
static int virtio_msg_ffa_send_request_by_method_locked
			(struct virtio_msg_ffa_driver *drv,
			 struct virtio_msg_ffa_xfer_engine *xfer,
			 const struct virtio_msg *req, size_t req_len,
			 u8 expected_type, u8 expected_msg_id,
			 struct virtio_msg *resp, size_t resp_buf_len,
			 size_t *resp_len);
static u32 virtio_msg_ffa_driver_local_bus_features_locked
			(struct virtio_msg_ffa_driver *drv);
static void
virtio_msg_ffa_driver_store_effective_features_locked
			(struct virtio_msg_ffa_driver *drv);
static int virtio_msg_ffa_send_bus_request_locked(struct virtio_msg_ffa_driver *drv,
						  u8 msg_id,
						  const void *payload,
						  size_t payload_len,
						  struct virtio_msg *resp,
						  size_t resp_buf_len,
						  size_t *resp_len);
struct virtio_msg_ffa_driver_xfer_method {
	const struct virtio_msg_ffa_xfer_method_desc *desc;
	const struct virtio_msg_ffa_xfer_ops *ops;
	bool (*capable)(struct virtio_msg_ffa_driver *drv);
};

static const struct virtio_msg_ffa_driver_xfer_method *
virtio_msg_ffa_driver_xfer_method_find
				(enum virtio_msg_ffa_transfer_method method);
static int virtio_msg_ffa_xfer_engine_init
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer,
				 const struct virtio_msg_ffa_driver_xfer_method *method);
static int virtio_msg_ffa_xfer_setup_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer);
static void virtio_msg_ffa_xfer_teardown_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer);
static void
virtio_msg_ffa_xfer_priv_release_all(struct virtio_msg_ffa_driver *drv);
static int
virtio_msg_ffa_local_services_publish_locked(struct virtio_msg_ffa_driver *drv);
static void
virtio_msg_ffa_local_services_unpublish_locked(struct virtio_msg_ffa_driver *drv);

static const struct virtio_msg_ffa_driver_xfer_method
virtio_msg_ffa_driver_xfer_methods[] = {
#if IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_FIFO)
	{
		.desc = &virtio_msg_ffa_fifo_driver_method_desc,
		.ops = &virtio_msg_ffa_fifo_xfer_ops,
		.capable = virtio_msg_ffa_fifo_available,
	},
#endif
#if IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_INDIRECT)
	{
		.desc = &virtio_msg_ffa_indirect_driver_method_desc,
		.ops = &virtio_msg_ffa_indirect_xfer_ops,
		.capable = virtio_msg_ffa_indirect_available,
	},
#endif
};

static bool virtio_msg_ffa_xfer_priv_method_valid
			(enum virtio_msg_ffa_transfer_method method)
{
	return method > VIRTIO_MSG_FFA_XFER_NONE && method < 4;
}

static void *virtio_msg_ffa_xfer_priv_get
			(struct virtio_msg_ffa_driver *drv,
			 enum virtio_msg_ffa_transfer_method method)
{
	if (!drv || !virtio_msg_ffa_xfer_priv_method_valid(method))
		return NULL;

	return drv->xfer_priv[method];
}

static int virtio_msg_ffa_xfer_priv_set
			(struct virtio_msg_ffa_driver *drv,
			 enum virtio_msg_ffa_transfer_method method, void *priv)
{
	if (!drv || !virtio_msg_ffa_xfer_priv_method_valid(method))
		return -EINVAL;

	drv->xfer_priv[method] = priv;
	return 0;
}

static void *virtio_msg_ffa_xfer_priv_take
			(struct virtio_msg_ffa_driver *drv,
			 enum virtio_msg_ffa_transfer_method method)
{
	void *priv;

	if (!drv || !virtio_msg_ffa_xfer_priv_method_valid(method))
		return NULL;

	priv = drv->xfer_priv[method];
	drv->xfer_priv[method] = NULL;
	return priv;
}

static void virtio_msg_ffa_bus_request_begin_locked
					(struct virtio_msg_ffa_driver *drv,
					 u8 msg_id)
{
	lockdep_assert_held(&drv->lock);

	for (;;) {
		spin_lock(&drv->bus_req_lock);
		if (!test_and_set_bit(msg_id, drv->bus_req_inflight)) {
			spin_unlock(&drv->bus_req_lock);
			return;
		}
		spin_unlock(&drv->bus_req_lock);

		mutex_unlock(&drv->lock);
		wait_event(drv->bus_req_wait,
			   !test_bit(msg_id, drv->bus_req_inflight));
		mutex_lock(&drv->lock);
	}
}

static void virtio_msg_ffa_bus_request_end_locked
					(struct virtio_msg_ffa_driver *drv,
					 u8 msg_id)
{
	lockdep_assert_held(&drv->lock);

	spin_lock(&drv->bus_req_lock);
	clear_bit(msg_id, drv->bus_req_inflight);
	spin_unlock(&drv->bus_req_lock);
	wake_up_all(&drv->bus_req_wait);
}

static bool virtio_msg_ffa_xfer_engine_valid
			(const struct virtio_msg_ffa_xfer_engine *xfer)
{
	return xfer && xfer->method != VIRTIO_MSG_FFA_XFER_NONE && xfer->ops;
}

static void virtio_msg_ffa_xfer_engine_reset(struct virtio_msg_ffa_xfer_engine *xfer)
{
	if (!xfer)
		return;

	xfer->method = VIRTIO_MSG_FFA_XFER_NONE;
	xfer->ops = NULL;
	xfer->priv = NULL;
}

bool virtio_msg_ffa_driver_event_runtime_ready_locked(struct virtio_msg_ffa_driver *drv)
{
	enum virtio_msg_ffa_transfer_method event_method;

	if (!drv || !drv->event_configured || virtio_msg_ffa_endpoint_disabled(drv))
		return false;
	if (!virtio_msg_ffa_xfer_engine_valid(&drv->active_xfer))
		return false;

	event_method = virtio_msg_ffa_xfer_target_method
			(&drv->ep, VIRTIO_MSG_FFA_XFER_ROLE_DRIVER,
			 VIRTIO_MSG_FFA_RUNTIME_TARGET_EVENT);
	if (event_method == VIRTIO_MSG_FFA_XFER_NONE)
		return false;

	return drv->active_xfer.method == event_method;
}

bool
virtio_msg_ffa_driver_indirect_rx_ready_locked(struct virtio_msg_ffa_driver *drv)
{
	if (!drv || virtio_msg_ffa_endpoint_disabled(drv))
		return false;

	if (virtio_msg_ffa_driver_event_runtime_ready_locked(drv))
		return true;

	if (virtio_msg_ffa_xfer_engine_valid(&drv->bootstrap_xfer) &&
	    drv->bootstrap_xfer.method == VIRTIO_MSG_FFA_XFER_INDIRECT)
		return true;

	if (virtio_msg_ffa_xfer_engine_valid(&drv->active_xfer) &&
	    drv->active_xfer.method == VIRTIO_MSG_FFA_XFER_INDIRECT)
		return true;

	return false;
}

static int
virtio_msg_ffa_local_services_publish_locked(struct virtio_msg_ffa_driver *drv)
{
	struct virtio_msg_ffa_service_user *user;
	int ret = 0;

	if (!drv)
		return 0;

	mutex_lock(&virtio_msg_ffa_service_lock);
	list_for_each_entry(user, &virtio_msg_ffa_service_list, node) {
		if (user->drv == drv)
			goto out_unlock;
	}

	user = kzalloc(sizeof(*user), GFP_KERNEL);
	if (!user) {
		ret = -ENOMEM;
		goto out_unlock;
	}
	user->drv = drv;

	if (!virtio_msg_ffa_service_users) {
		ret = ffa_driver_services_publish(&virtio_msg_ffa_driver);
		if (ret) {
			kfree(user);
			goto out_unlock;
		}
	}

	list_add(&user->node, &virtio_msg_ffa_service_list);
	virtio_msg_ffa_service_users++;

out_unlock:
	mutex_unlock(&virtio_msg_ffa_service_lock);
	if (ret && drv->fdev)
		dev_warn_ratelimited(&drv->fdev->dev,
				     "local service publish failed: %d\n", ret);

	return ret;
}

static int virtio_msg_ffa_local_services_module_publish(void)
{
	int ret = 0;

	mutex_lock(&virtio_msg_ffa_service_lock);
	if (!virtio_msg_ffa_service_users)
		ret = ffa_driver_services_publish(&virtio_msg_ffa_driver);
	if (!ret)
		virtio_msg_ffa_service_users++;
	mutex_unlock(&virtio_msg_ffa_service_lock);

	if (ret)
		pr_debug("driver local service publish failed: %d\n", ret);
	else
		pr_debug("driver local service published\n");

	return ret;
}

static void virtio_msg_ffa_local_services_module_unpublish(void)
{
	int ret = 0;

	mutex_lock(&virtio_msg_ffa_service_lock);
	if (virtio_msg_ffa_service_users) {
		virtio_msg_ffa_service_users--;
		if (!virtio_msg_ffa_service_users)
			ret = ffa_driver_services_unpublish(&virtio_msg_ffa_driver);
	}
	mutex_unlock(&virtio_msg_ffa_service_lock);

	if (ret)
		pr_debug("driver local service unpublish failed: %d\n", ret);
}

static void
virtio_msg_ffa_local_services_unpublish_locked(struct virtio_msg_ffa_driver *drv)
{
	struct virtio_msg_ffa_service_user *user, *tmp;
	int ret = 0;
	bool found = false;

	if (!drv)
		return;

	mutex_lock(&virtio_msg_ffa_service_lock);
	list_for_each_entry_safe(user, tmp, &virtio_msg_ffa_service_list, node) {
		if (user->drv != drv)
			continue;

		list_del(&user->node);
		kfree(user);
		found = true;
		break;
	}
	if (!found)
		goto out_unlock;

	virtio_msg_ffa_service_users--;
	if (!virtio_msg_ffa_service_users)
		ret = ffa_driver_services_unpublish(&virtio_msg_ffa_driver);

out_unlock:
	mutex_unlock(&virtio_msg_ffa_service_lock);
	if (ret && drv->fdev)
		dev_warn_ratelimited(&drv->fdev->dev,
				     "local service unpublish failed: %d\n",
				     ret);
}

static void virtio_msg_ffa_method_teardown_locked(struct virtio_msg_ffa_driver *drv)
{
	const struct virtio_msg_ffa_driver_xfer_method *method;
	struct virtio_msg_ffa_xfer_engine xfer;
	struct virtio_msg_ffa_xfer_engine active_xfer;
	struct virtio_msg_ffa_xfer_engine bootstrap_xfer;
	unsigned int i;

	if (!drv)
		return;

	/*
	 * Teardown active/bootstrap engines first so method-private state owned
	 * by engine handles is cleaned up before shell fallback cleanup.
	 */
	active_xfer = drv->active_xfer;
	bootstrap_xfer = drv->bootstrap_xfer;
	virtio_msg_ffa_xfer_teardown_locked(drv, &active_xfer);
	if (bootstrap_xfer.method != active_xfer.method ||
	    bootstrap_xfer.priv != active_xfer.priv)
		virtio_msg_ffa_xfer_teardown_locked(drv, &bootstrap_xfer);

	/*
	 * Teardown non-selected methods that may still hold setup state.
	 */
	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_driver_xfer_methods); i++) {
		method = &virtio_msg_ffa_driver_xfer_methods[i];
		if (!method->desc)
			continue;
		if (method->desc->method == active_xfer.method ||
		    method->desc->method == bootstrap_xfer.method)
			continue;
		if (!virtio_msg_ffa_xfer_engine_init(drv, &xfer, method))
			virtio_msg_ffa_xfer_teardown_locked(drv, &xfer);
	}

	virtio_msg_ffa_xfer_engine_reset(&drv->active_xfer);
	virtio_msg_ffa_xfer_engine_reset(&drv->bootstrap_xfer);
}

static int virtio_msg_ffa_endpoint_teardown_locked(struct virtio_msg_ffa_driver *drv,
						   bool reset_success)
{
	struct virtio_msg_ffa_topology *topo;
	int ret = 0;

	if (!drv)
		return -EINVAL;

	WRITE_ONCE(drv->indirect_rx_generation,
		   READ_ONCE(drv->indirect_rx_generation) + 1);

	/* Teardown order is fixed, see plan-handle-review.md Step 4. */

	/* Unpublish local services before endpoint shutdown. */
	virtio_msg_ffa_local_services_unpublish_locked(drv);

	/* 1-2) Quiesce and teardown transfer-method-specific runtime state. */
	virtio_msg_ffa_method_teardown_locked(drv);

	/* 3) Stop topology publication / unpublish devices. */
	virtio_msg_ffa_endpoint_disable(drv);
	rcu_read_lock();
	topo = virtio_msg_ffa_driver_topology_get(drv);
	rcu_read_unlock();
	virtio_msg_ffa_driver_topology_set(drv, NULL);

	mutex_unlock(&drv->lock);
	synchronize_rcu();
	virtio_msg_ffa_topology_destroy(topo);
	mutex_lock(&drv->lock);

	/* 4) Abort exchanges with -ESHUTDOWN. */
	virtio_msg_ffa_exchange_abort_all(&drv->ep, -ESHUTDOWN);

	/* 5) Reclaim and untrack FF-A area exports for the endpoint. */
	virtio_msg_ffa_area_endpoint_cleanup(&drv->area_ctx);

	/* 6) Runtime method state already torn down above. */

	/* 7) Reset-success only: clear negotiation/selected-method state. */
	if (reset_success) {
		mutex_lock(&drv->ep.exchange_lock);
		drv->ep.exchange_shutdown = false;
		mutex_unlock(&drv->ep.exchange_lock);

		drv->ep.negotiation_done = false;
		drv->event_configured = false;
		virtio_msg_ffa_select_transfer_method(&drv->ep,
						      VIRTIO_MSG_FFA_XFER_NONE);
		virtio_msg_ffa_xfer_engine_reset(&drv->bootstrap_xfer);
		virtio_msg_ffa_xfer_engine_reset(&drv->active_xfer);
		drv->endpoint_disabled = false;

		ret = virtio_msg_ffa_topology_init(drv);
		if (ret)
			virtio_msg_ffa_endpoint_disable(drv);
	}

	return ret;
}

static void virtio_msg_ffa_handle_endpoint_failure_locked
				(struct virtio_msg_ffa_driver *drv, int err)
{
	if (!drv)
		return;

	if (drv->fdev)
		dev_warn_ratelimited(&drv->fdev->dev,
				     "endpoint failure after FIFO notify retries: %d\n",
				     err);

	(void)virtio_msg_ffa_endpoint_teardown_locked(drv, false);
}

void virtio_msg_ffa_driver_handle_endpoint_failure_locked(struct virtio_msg_ffa_driver *drv,
							  int err)
{
	virtio_msg_ffa_handle_endpoint_failure_locked(drv, err);
}

static int
virtio_msg_ffa_driver_route_event_locked(struct virtio_msg_ffa_driver *drv,
					 const struct virtio_msg *msg,
					 size_t msg_len, u8 type_bits)
{
	if (!drv || !msg)
		return -EINVAL;
	if (!drv->event_configured)
		return 0;

	if (type_bits == VIRTIO_MSG_TYPE_TRANSPORT)
		return virtio_msg_ffa_topology_handle_transport_event(drv, msg);

	if (type_bits == VIRTIO_MSG_TYPE_BUS &&
	    msg->msg_id == VIRTIO_MSG_BUS_EVENT_DEVICE) {
		const struct virtio_msg_bus_event_device *event;

		if (msg_len < sizeof(*msg) + sizeof(*event))
			return -EMSGSIZE;

		event = (const struct virtio_msg_bus_event_device *)msg->payload;
		return virtio_msg_ffa_topology_handle_event(drv, event);
	}

	if (type_bits == VIRTIO_MSG_TYPE_BUS &&
	    msg->msg_id == FFA_BUS_EVENT_AREA_RELEASE) {
		const struct virtio_msg_ffa_area_release_event *event;
		u16 area_id;

		if (msg_len < sizeof(*msg) + sizeof(*event))
			return -EMSGSIZE;

		event = (const struct virtio_msg_ffa_area_release_event *)msg->payload;
		area_id = le16_to_cpu(event->area_id);
		return virtio_msg_ffa_area_handle_release(&drv->area_ctx, area_id);
	}

	return 0;
}

static int
virtio_msg_ffa_driver_route_response_locked
		(struct virtio_msg_ffa_driver *drv,
		 const struct virtio_msg *msg, size_t msg_len,
		 const struct virtio_msg_ffa_exchange *exchange,
		 int match_status,
		 const struct virtio_msg_ffa_error_result *error)
{
	u16 dev_num;
	u16 token;

	if (!drv || !msg || !exchange)
		return -EINVAL;

	(void)error;
	dev_num = le16_to_cpu(msg->dev_num);
	token = le16_to_cpu(msg->token);
	if (match_status == -EPROTO) {
		if (drv->fdev)
			dev_dbg(&drv->fdev->dev,
				"bus rx response match failed: dev=%u tok=%u status=%d\n",
				dev_num, token, match_status);
		else
			pr_debug("bus rx response match failed: dev=%u tok=%u status=%d\n",
				 dev_num, token, match_status);
		return virtio_msg_ffa_exchange_complete(&drv->ep, dev_num, token,
							NULL, 0, match_status);
	}

	return virtio_msg_ffa_exchange_complete(&drv->ep, dev_num, token, msg,
						msg_len, 0);
}

int virtio_msg_ffa_driver_dispatch_inbound_locked(struct virtio_msg_ffa_driver *drv,
						  const struct virtio_msg *msg,
						  size_t msg_len)
{
	struct virtio_msg_ffa_inbound_result result;
	int ret;

	if (!drv || !msg)
		return -EINVAL;

	ret = virtio_msg_ffa_classify_inbound(&drv->ep, msg, msg_len, &result);
	if (ret) {
		if (drv->fdev)
			dev_dbg(&drv->fdev->dev,
				"bus rx dispatch classify failed: ret=%d\n", ret);
		else
			pr_debug("bus rx dispatch classify failed: ret=%d\n",
				 ret);
		return ret;
	}

	switch (result.class) {
	case VIRTIO_MSG_FFA_INBOUND_EVENT:
		return virtio_msg_ffa_driver_route_event_locked(drv, msg, msg_len,
								result.type_bits);
	case VIRTIO_MSG_FFA_INBOUND_RESPONSE:
		return virtio_msg_ffa_driver_route_response_locked
				(drv, msg, msg_len, &result.exchange,
				 result.match_status, &result.error);
	case VIRTIO_MSG_FFA_INBOUND_DROP:
	case VIRTIO_MSG_FFA_INBOUND_BUS_REQUEST:
	case VIRTIO_MSG_FFA_INBOUND_TRANSPORT_REQUEST:
	case VIRTIO_MSG_FFA_INBOUND_PRENEG_ERROR_RESPONSE:
	default:
		return 0;
	}
}

static void virtio_msg_ffa_handle_device_failure_locked
				(struct virtio_msg_ffa_driver *drv, u16 dev_num)
{
	struct virtio_msg_bus_event_device event;

	if (!drv)
		return;

	if (drv->fdev)
		dev_warn_ratelimited(&drv->fdev->dev,
				     "device %u failure; scheduling removal\n",
				     dev_num);

	event.dev_num = cpu_to_le16(dev_num);
	event.dev_state = cpu_to_le16(VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED);
	virtio_msg_ffa_topology_handle_event(drv, &event);
}

void virtio_msg_ffa_driver_handle_device_failure_locked(struct virtio_msg_ffa_driver *drv,
							u16 dev_num)
{
	virtio_msg_ffa_handle_device_failure_locked(drv, dev_num);
}

static const struct virtio_msg_ffa_driver_xfer_method *
virtio_msg_ffa_driver_xfer_method_find(enum virtio_msg_ffa_transfer_method method)
{
	const struct virtio_msg_ffa_driver_xfer_method *entry;
	unsigned int i;

	if (method == VIRTIO_MSG_FFA_XFER_NONE)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_driver_xfer_methods); i++) {
		entry = &virtio_msg_ffa_driver_xfer_methods[i];
		if (entry->desc && entry->desc->method == method)
			return entry;
	}

	return NULL;
}

static int virtio_msg_ffa_xfer_engine_init
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer,
				 const struct virtio_msg_ffa_driver_xfer_method *method)
{
	const struct virtio_msg_ffa_xfer_method_desc *desc;

	if (!drv || !xfer || !method || !method->desc || !method->ops)
		return -EINVAL;
	desc = method->desc;
	if (!(desc->role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DRIVER))
		return -EOPNOTSUPP;

	xfer->method = desc->method;
	xfer->ops = method->ops;
	xfer->priv = virtio_msg_ffa_xfer_priv_get(drv, desc->method);

	return 0;
}

static void virtio_msg_ffa_xfer_release_priv(struct virtio_msg_ffa_driver *drv,
					     enum virtio_msg_ffa_transfer_method method,
					     void *priv)
{
	struct virtio_msg_ffa_xfer_engine xfer;
	const struct virtio_msg_ffa_driver_xfer_method *entry;
	const struct virtio_msg_ffa_xfer_ops *ops;

	if (!drv || !priv || method == VIRTIO_MSG_FFA_XFER_NONE)
		return;

	entry = virtio_msg_ffa_driver_xfer_method_find(method);
	ops = entry ? entry->ops : NULL;
	if (!ops || !ops->release_priv)
		return;

	xfer.method = method;
	xfer.ops = ops;
	xfer.priv = priv;
	ops->release_priv(drv, &xfer);
}

static void
virtio_msg_ffa_xfer_priv_release_all(struct virtio_msg_ffa_driver *drv)
{
	const struct virtio_msg_ffa_driver_xfer_method *method;
	void *priv;
	unsigned int i;

	if (!drv)
		return;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_driver_xfer_methods); i++) {
		method = &virtio_msg_ffa_driver_xfer_methods[i];
		if (!method->desc)
			continue;

		priv = virtio_msg_ffa_xfer_priv_take(drv, method->desc->method);
		virtio_msg_ffa_xfer_release_priv(drv, method->desc->method, priv);
	}
}

static int virtio_msg_ffa_xfer_setup_locked(struct virtio_msg_ffa_driver *drv,
					    struct virtio_msg_ffa_xfer_engine *xfer)
{
	void *old_priv;
	int ret, state_ret;

	if (!drv || !virtio_msg_ffa_xfer_engine_valid(xfer))
		return -EINVAL;
	if (!xfer->ops->setup_locked)
		return -EOPNOTSUPP;

	old_priv = xfer->priv;
	ret = xfer->ops->setup_locked(drv, xfer);
	if (ret)
		goto out_err;
	if (!xfer->priv)
		return 0;

	state_ret = virtio_msg_ffa_xfer_priv_set(drv, xfer->method,
						 xfer->priv);
	if (state_ret) {
		/*
		 * Roll back method setup if state registration fails so no
		 * live method state remains untracked.
		 */
		if (xfer->ops->teardown_locked)
			xfer->ops->teardown_locked(drv, xfer);
		if (xfer->priv != old_priv)
			virtio_msg_ffa_xfer_release_priv(drv, xfer->method,
							 xfer->priv);
		xfer->priv = old_priv;
		return state_ret;
	}

	return 0;

out_err:
	if (xfer->priv != old_priv) {
		if (xfer->ops->teardown_locked)
			xfer->ops->teardown_locked(drv, xfer);
		virtio_msg_ffa_xfer_release_priv(drv, xfer->method, xfer->priv);
		xfer->priv = old_priv;
	}

	return ret;
}

static void virtio_msg_ffa_xfer_teardown_locked
			(struct virtio_msg_ffa_driver *drv,
			 struct virtio_msg_ffa_xfer_engine *xfer)
{
	if (!drv || !virtio_msg_ffa_xfer_engine_valid(xfer))
		return;
	if (!xfer->ops->teardown_locked)
		return;

	xfer->ops->teardown_locked(drv, xfer);
}

static void virtio_msg_ffa_xfer_quiesce_locked
			(struct virtio_msg_ffa_driver *drv,
			 struct virtio_msg_ffa_xfer_engine *xfer)
{
	if (!drv || !virtio_msg_ffa_xfer_engine_valid(xfer))
		return;
	if (!xfer->ops->quiesce_locked)
		return;

	xfer->ops->quiesce_locked(drv, xfer);
}

/*
 * Runtime event setup first prepares transfer-method state. For FIFO this
 * includes sharing the FIFO ring with FFA_MEM_SHARE and sending
 * FIFO_CONFIGURE. The method event_configure_locked() hook then sends
 * EVENT_CONFIGURE. Only after EVENT_CONFIGURE succeeds does the driver mark
 * events configured, copy active_xfer, select the endpoint runtime transfer
 * method, and publish local services.
 */
static int virtio_msg_ffa_event_configure_active_locked
			(struct virtio_msg_ffa_driver *drv,
			 struct virtio_msg_ffa_xfer_engine *xfer)
{
	int ret;

	if (!virtio_msg_ffa_xfer_engine_valid(xfer))
		return -EINVAL;

	if (!xfer->ops->event_configure_locked)
		return -EOPNOTSUPP;

	ret = virtio_msg_ffa_xfer_setup_locked(drv, xfer);
	if (ret)
		return ret;

	drv->event_configured = false;
	ret = xfer->ops->event_configure_locked(drv, xfer);
	if (ret)
		return ret;

	drv->event_configured = true;
	drv->active_xfer = *xfer;
	virtio_msg_ffa_select_transfer_method(&drv->ep, xfer->method);
	if (drv->fdev) {
		dev_info(&drv->fdev->dev,
			 "runtime method selected: peer_vm=%u xfer=%s event=%s\n",
			 (u16)drv->fdev->vm_id,
			 virtio_msg_ffa_xfer_method_name(xfer->method),
			 virtio_msg_ffa_event_delivery_name(drv->ep.event_delivery));
	}

	return virtio_msg_ffa_local_services_publish_locked(drv);
}

static int virtio_msg_ffa_send_by_selected_method_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer,
				 const struct virtio_msg *req, size_t req_len,
				 struct virtio_msg *resp, size_t resp_buf_len,
				 size_t *resp_len)
{
	u8 expected_type;

	if (!req)
		return -EINVAL;

	expected_type = req->type | VIRTIO_MSG_TYPE_RESPONSE;
	return virtio_msg_ffa_send_request_by_method_locked
				(drv, xfer, req, req_len, expected_type,
				 req->msg_id, resp, resp_buf_len, resp_len);
}

static int virtio_msg_ffa_exchange_complete_locked
			(struct virtio_msg_ffa_driver *drv,
			 const struct virtio_msg *req,
			 const struct virtio_msg *resp, size_t resp_len,
			 u8 expected_type, u8 expected_msg_id,
			 bool release_exchange)
{
	struct virtio_msg_ffa_error_result error_result;
	u16 dev_num, token;
	int ret;

	dev_num = le16_to_cpu(req->dev_num);
	token = le16_to_cpu(req->token);

	if (virtio_msg_ffa_msg_is_event(resp->msg_id)) {
		const u8 type_mask = VIRTIO_MSG_TYPE_RESPONSE | VIRTIO_MSG_TYPE_BUS;

		/* Route the event so it is not lost, but fail the request. */
		(void)virtio_msg_ffa_driver_route_event_locked
			(drv, resp, resp_len, resp->type & type_mask);
		ret = -EPROTO;
		goto out_release;
	}

	if (resp->msg_id == FFA_BUS_MSG_ERROR) {
		ret = virtio_msg_ffa_handle_error_resp(&drv->ep, resp, resp_len,
						       &error_result);
		if (ret == 1)
			ret = -EIO;
		else if (!ret)
			ret = -EPROTO;
		goto out_release;
	}

	if (!virtio_msg_ffa_type_match_masked(resp->type, expected_type) ||
	    resp->msg_id != expected_msg_id) {
		ret = -EPROTO;
		goto out_release;
	}

	if (le16_to_cpu(resp->dev_num) != le16_to_cpu(req->dev_num) ||
	    le16_to_cpu(resp->token) != le16_to_cpu(req->token)) {
		ret = -EPROTO;
		goto out_release;
	}

	ret = 0;

out_release:
	if (ret) {
		if (drv->fdev)
			dev_dbg(&drv->fdev->dev,
				"bus rx complete failed: ret=%d expected_type=0x%x expected_msg_id=0x%02x dev=%u tok=%u\n",
				ret, expected_type, expected_msg_id, dev_num, token);
		else
			pr_debug("bus rx complete failed: ret=%d expected_type=0x%x expected_msg_id=0x%02x dev=%u tok=%u\n",
				 ret, expected_type, expected_msg_id, dev_num, token);
	}
	if (release_exchange)
		virtio_msg_ffa_exchange_release(&drv->ep, dev_num, token, NULL);
	return ret;
}

static int virtio_msg_ffa_direct_event_ack_validate
				(const struct virtio_msg *req,
				 const struct virtio_msg *resp,
				 size_t resp_len)
{
	if (!req || !resp)
		return -EINVAL;

	if (resp->type != (VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_RESPONSE))
		return -EPROTO;
	if (resp->msg_id)
		return -EPROTO;
	if (le16_to_cpu(resp->dev_num))
		return -EPROTO;
	if (le16_to_cpu(resp->token) != le16_to_cpu(req->token))
		return -EPROTO;
	if (le16_to_cpu(resp->msg_size) != sizeof(*resp) ||
	    resp_len != sizeof(*resp))
		return -EPROTO;

	return 0;
}

static int virtio_msg_ffa_driver_wait_exchange_locked
				(struct virtio_msg_ffa_driver *drv,
				 u16 dev_num, u16 token,
				 struct virtio_msg *resp, size_t resp_buf_len,
				 size_t *resp_len)
{
	int ret;

	if (!drv || !resp || !resp_len)
		return -EINVAL;

	mutex_unlock(&drv->lock);
	ret = virtio_msg_ffa_exchange_wait(&drv->ep, dev_num, token, resp,
					   resp_buf_len, resp_len,
					   VIRTIO_MSG_FFA_ASYNC_TIMEOUT_MS);
	mutex_lock(&drv->lock);

	return ret;
}

static int virtio_msg_ffa_send_request_by_method_locked
			(struct virtio_msg_ffa_driver *drv,
			 struct virtio_msg_ffa_xfer_engine *xfer,
			 const struct virtio_msg *req, size_t req_len,
			 u8 expected_type, u8 expected_msg_id,
			 struct virtio_msg *resp, size_t resp_buf_len,
			 size_t *resp_len)
{
	enum virtio_msg_ffa_preneg_action preneg_action;
	u16 dev_num, token;
	bool need_response;
	bool ack_only = false;
	bool exchange_stored = false;
	struct virtio_msg *submit_resp = resp;
	const char *method_name;
	size_t submit_resp_buf_len = resp_buf_len;
	size_t *submit_resp_len = resp_len;
	u8 local_resp_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *local_resp = (struct virtio_msg *)local_resp_buf;
	size_t local_resp_len = 0;
	int ret;

	if (!drv || !req)
		return -EINVAL;
	if (resp && !resp_len)
		return -EINVAL;
	if (!resp && resp_len)
		return -EINVAL;

	ret = virtio_msg_ffa_validate_inbound_msg(req, req_len);
	if (ret)
		return ret;

	preneg_action = virtio_msg_ffa_classify_preneg_msg(&drv->ep, req);
	if (preneg_action == VIRTIO_MSG_FFA_PRENEG_REJECT_LOCAL_DROP)
		return -EACCES;
	if (preneg_action == VIRTIO_MSG_FFA_PRENEG_REJECT_DEVICE_ERROR)
		return -EPROTO;

	if (!virtio_msg_ffa_xfer_engine_valid(xfer) || !xfer->ops->submit_locked)
		return -EOPNOTSUPP;

	dev_num = le16_to_cpu(req->dev_num);
	token = le16_to_cpu(req->token);
	method_name = virtio_msg_ffa_xfer_method_name(xfer->method);
	virtio_msg_ffa_driver_trace_msg(drv, "tx", method_name, req, req_len);

	need_response = !!resp;
	if (need_response) {
		ret = virtio_msg_ffa_exchange_store(&drv->ep, dev_num, token,
						    expected_msg_id,
						    expected_type);
		if (ret)
			return ret;
		exchange_stored = true;
	} else if (!xfer->ops->async_response) {
		memset(local_resp_buf, 0, sizeof(local_resp_buf));
		submit_resp = local_resp;
		submit_resp_buf_len = sizeof(local_resp_buf);
		submit_resp_len = &local_resp_len;
		need_response = true;
		ack_only = true;
	}

	ret = xfer->ops->submit_locked(drv, xfer, req, req_len, submit_resp,
				       submit_resp_buf_len, submit_resp_len);
	if (ret) {
		if (drv->fdev)
			dev_dbg(&drv->fdev->dev,
				"bus tx submit failed: method=%s dev=%u tok=%u ret=%d\n",
				method_name, dev_num, token, ret);
		else
			pr_debug("bus tx submit failed: method=%s dev=%u tok=%u ret=%d\n",
				 method_name, dev_num, token, ret);
		goto out_release;
	}

	if (exchange_stored && xfer->ops->async_response) {
		ret = virtio_msg_ffa_driver_wait_exchange_locked
				(drv, dev_num, token, resp, resp_buf_len,
				 resp_len);
		if (ret) {
			if (drv->fdev)
				dev_dbg(&drv->fdev->dev,
					"bus rx wait failed: method=%s dev=%u tok=%u ret=%d\n",
					method_name, dev_num, token, ret);
			else
				pr_debug("bus rx wait failed: method=%s dev=%u tok=%u ret=%d\n",
					 method_name, dev_num, token, ret);
			goto out_release;
		}
	}

	if (!need_response)
		return 0;

	if (!xfer->ops->async_response)
		virtio_msg_ffa_driver_trace_msg(drv, "rx", method_name, submit_resp,
						*submit_resp_len);
	if (ack_only && virtio_msg_ffa_msg_is_event(req->msg_id))
		return virtio_msg_ffa_direct_event_ack_validate
				(req, submit_resp, *submit_resp_len);

	ret = virtio_msg_ffa_exchange_complete_locked
			(drv, req, submit_resp, *submit_resp_len,
			 expected_type, expected_msg_id, exchange_stored);
	if (!ret && ack_only &&
	    (le16_to_cpu(submit_resp->msg_size) != sizeof(*submit_resp) ||
	     *submit_resp_len != sizeof(*submit_resp)))
		ret = -EPROTO;
	return ret;

out_release:
	if (exchange_stored)
		virtio_msg_ffa_exchange_release(&drv->ep, dev_num, token, NULL);
	return ret;
}

static int virtio_msg_ffa_send_request_locked(struct virtio_msg_ffa_driver *drv,
					      const struct virtio_msg *req,
					      size_t req_len, u8 expected_type,
					      u8 expected_msg_id,
					      struct virtio_msg *resp,
					      size_t resp_buf_len,
					      size_t *resp_len)
{
	return virtio_msg_ffa_send_request_by_method_locked
				(drv, &drv->active_xfer, req, req_len,
				 expected_type, expected_msg_id, resp,
				 resp_buf_len, resp_len);
}

static bool virtio_msg_ffa_bus_msg_needs_bootstrap(u8 msg_id)
{
	switch (msg_id) {
	case FFA_BUS_MSG_VERSION:
	case FFA_BUS_MSG_RESET:
	case FFA_BUS_MSG_EVENT_CONFIGURE:
	case FFA_BUS_MSG_FIFO_CONFIGURE:
		return true;
	default:
		return false;
	}
}

static int virtio_msg_ffa_send_bus_request_locked
				(struct virtio_msg_ffa_driver *drv,
				 u8 msg_id, const void *payload, size_t payload_len,
				 struct virtio_msg *resp, size_t resp_buf_len,
				 size_t *resp_len)
{
	u8 req_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *req = (struct virtio_msg *)req_buf;
	struct virtio_msg_ffa_xfer_engine *xfer;
	size_t req_len;
	u16 token;
	int ret;

	if (!drv || !resp || !resp_len)
		return -EINVAL;

	virtio_msg_ffa_bus_request_begin_locked(drv, msg_id);
	ret = virtio_msg_ffa_next_token_locked(&drv->ep, &token);
	if (ret)
		goto out_request_end;
	ret = virtio_msg_ffa_prepare_frame(req, sizeof(req_buf), msg_id,
					   VIRTIO_MSG_TYPE_BUS |
					   VIRTIO_MSG_TYPE_REQUEST,
					   0, token, payload,
					   payload_len, &req_len);
	if (ret)
		goto out_request_end;

	xfer = &drv->active_xfer;
	if (virtio_msg_ffa_bus_msg_needs_bootstrap(msg_id))
		xfer = &drv->bootstrap_xfer;

	ret = virtio_msg_ffa_send_request_by_method_locked
				(drv, xfer, req, req_len,
				 VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_RESPONSE,
				 msg_id, resp, resp_buf_len, resp_len);

out_request_end:
	virtio_msg_ffa_bus_request_end_locked(drv, msg_id);
	return ret;
}

int virtio_msg_ffa_driver_send_bus_request_locked(struct virtio_msg_ffa_driver *drv,
						  u8 msg_id,
						  const void *payload,
						  size_t payload_len,
						  struct virtio_msg *resp,
						  size_t resp_buf_len,
						  size_t *resp_len)
{
	return virtio_msg_ffa_send_bus_request_locked(drv, msg_id, payload,
						      payload_len, resp,
						      resp_buf_len, resp_len);
}

int virtio_msg_ffa_area_ctx_send_bus_req(struct virtio_msg_ffa_area_ctx *ctx,
					 u8 msg_id, const void *payload,
					 size_t payload_len,
					 struct virtio_msg *resp,
					 size_t resp_buf_len,
					 size_t *resp_len)
{
	struct virtio_msg_ffa_driver *drv;
	int ret;

	if (!ctx || !ctx->fdev || !ctx->ep)
		return -EINVAL;

	drv = container_of(ctx, struct virtio_msg_ffa_driver, area_ctx);
	mutex_lock(&drv->lock);
	ret = virtio_msg_ffa_prepare_runtime_locked(drv);
	if (ret)
		goto out_unlock;

	ret = virtio_msg_ffa_send_bus_request_locked(drv, msg_id, payload,
						     payload_len, resp,
						     resp_buf_len, resp_len);

out_unlock:
	mutex_unlock(&drv->lock);
	return ret;
}

struct virtio_msg_ffa_area_ctx *
virtio_msg_ffa_area_ctx_alloc(struct virtio_msg_ffa_driver *drv)
{
	if (!drv)
		return NULL;

	drv->area_ctx.fdev = drv->fdev;
	drv->area_ctx.ep = &drv->ep;
	drv->area_ctx.send_bus_req = virtio_msg_ffa_area_ctx_send_bus_req;

	return &drv->area_ctx;
}

void virtio_msg_ffa_area_ctx_free(struct virtio_msg_ffa_area_ctx *ctx)
{
	if (!ctx)
		return;

	ctx->fdev = NULL;
	ctx->ep = NULL;
	ctx->send_bus_req = NULL;
}

int virtio_msg_ffa_topology_send_bus_request(struct virtio_msg_ffa_driver *drv,
					     u8 msg_id, const void *payload,
					     size_t payload_len,
					     struct virtio_msg *resp,
					     size_t resp_buf_len,
					     size_t *resp_len)
{
	int ret;

	if (!drv)
		return -EINVAL;

	mutex_lock(&drv->lock);
	ret = virtio_msg_ffa_send_bus_request_locked(drv, msg_id, payload,
						     payload_len, resp,
						     resp_buf_len, resp_len);
	mutex_unlock(&drv->lock);
	return ret;
}

int virtio_msg_ffa_driver_prepare_runtime(struct virtio_msg_ffa_driver *drv)
{
	int ret;

	if (!drv)
		return -EINVAL;

	mutex_lock(&drv->lock);
	ret = virtio_msg_ffa_prepare_runtime_locked(drv);
	mutex_unlock(&drv->lock);

	return ret;
}

int virtio_msg_ffa_driver_event_configure(struct virtio_msg_ffa_driver *drv)
{
	int ret;

	if (!drv)
		return -EINVAL;

	mutex_lock(&drv->lock);
	if (drv->event_configured &&
	    virtio_msg_ffa_xfer_engine_valid(&drv->active_xfer)) {
		ret = 0;
		goto out_unlock;
	}

	if (!virtio_msg_ffa_xfer_engine_valid(&drv->active_xfer)) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}

	ret = virtio_msg_ffa_event_configure_active_locked(drv,
							   &drv->active_xfer);
out_unlock:
	mutex_unlock(&drv->lock);

	return ret;
}

struct device *virtio_msg_ffa_driver_parent(struct virtio_msg_ffa_driver *drv)
{
	if (!drv || !drv->fdev)
		return NULL;

	return &drv->fdev->dev;
}

struct virtio_msg_ffa_area_ctx *
virtio_msg_ffa_driver_area_ctx(struct virtio_msg_ffa_driver *drv)
{
	if (!drv)
		return NULL;

	return &drv->area_ctx;
}

int virtio_msg_ffa_driver_area_add(struct virtio_msg_ffa_driver *drv,
				   const struct virtio_msg_ffa_area *area)
{
	if (!drv)
		return -EINVAL;

	return virtio_msg_ffa_area_add(&drv->ep, area);
}

int virtio_msg_ffa_driver_area_lookup(struct virtio_msg_ffa_driver *drv,
				      u16 area_id,
				      struct virtio_msg_ffa_area *out)
{
	if (!drv)
		return -EINVAL;

	return virtio_msg_ffa_area_lookup(&drv->ep, area_id, out);
}

int virtio_msg_ffa_driver_area_mark_released(struct virtio_msg_ffa_driver *drv,
					     u16 area_id)
{
	if (!drv)
		return -EINVAL;

	return virtio_msg_ffa_area_mark_released(&drv->ep, area_id);
}

int virtio_msg_ffa_driver_area_remove(struct virtio_msg_ffa_driver *drv,
				      u16 area_id,
				      struct virtio_msg_ffa_area *removed)
{
	if (!drv)
		return -EINVAL;

	return virtio_msg_ffa_area_remove(&drv->ep, area_id, removed);
}

struct virtio_msg_ffa_topology *
virtio_msg_ffa_driver_topology_get(struct virtio_msg_ffa_driver *drv)
{
	if (!drv)
		return NULL;

	return rcu_dereference(drv->topology);
}

void virtio_msg_ffa_driver_topology_set(struct virtio_msg_ffa_driver *drv,
					struct virtio_msg_ffa_topology *topo)
{
	if (!drv)
		return;

	rcu_assign_pointer(drv->topology, topo);
}

void virtio_msg_ffa_endpoint_disable(struct virtio_msg_ffa_driver *drv)
{
	if (!drv)
		return;

	WRITE_ONCE(drv->endpoint_disabled, true);
}

bool virtio_msg_ffa_endpoint_disabled(struct virtio_msg_ffa_driver *drv)
{
	if (!drv)
		return true;

	return READ_ONCE(drv->endpoint_disabled);
}

static int virtio_msg_ffa_version_negotiate_locked
			(struct virtio_msg_ffa_driver *drv)
{
	struct virtio_msg_ffa_version_req version_req;
	struct virtio_msg_ffa_version_resp *version_resp;
	u8 version_msg_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *version_msg =
		(struct virtio_msg *)version_msg_buf;
	size_t version_len;
	u32 peer_ffa_version;
	int ret;

	virtio_msg_ffa_version_begin(&drv->ep);
	virtio_msg_ffa_version_prepare_probe(&version_req, drv->local_ffa_version);
	memset(version_msg_buf, 0, sizeof(version_msg_buf));
	ret = virtio_msg_ffa_send_bus_request_locked(drv, FFA_BUS_MSG_VERSION,
						     &version_req,
						     sizeof(version_req),
						     version_msg,
						     sizeof(version_msg_buf),
						     &version_len);
	if (ret)
		return ret;

	if (version_len != sizeof(*version_msg) + sizeof(*version_resp))
		return -EPROTO;
	version_resp = (struct virtio_msg_ffa_version_resp *)version_msg->payload;
	ret = virtio_msg_ffa_version_accept_resp(&drv->ep, &version_req,
						 version_resp);
	if (ret < 0)
		return ret;
	virtio_msg_ffa_driver_store_effective_features_locked(drv);
	if (!drv->ep.bus_version)
		return -EOPNOTSUPP;
	peer_ffa_version = le32_to_cpu(version_resp->ffa_version);
	if (peer_ffa_version < FFA_VERSION_1_2)
		return -EOPNOTSUPP;
	drv->ep.ffa_version = min(drv->local_ffa_version, peer_ffa_version);

	virtio_msg_ffa_version_prepare_echo(&version_req, version_resp);
	memset(version_msg_buf, 0, sizeof(version_msg_buf));
	ret = virtio_msg_ffa_send_bus_request_locked(drv, FFA_BUS_MSG_VERSION,
						     &version_req,
						     sizeof(version_req),
						     version_msg,
						     sizeof(version_msg_buf),
						     &version_len);
	if (ret)
		return ret;

	if (version_len != sizeof(*version_msg) + sizeof(*version_resp))
		return -EPROTO;
	version_resp = (struct virtio_msg_ffa_version_resp *)version_msg->payload;
	ret = virtio_msg_ffa_version_accept_resp(&drv->ep, &version_req,
						 version_resp);
	if (ret != 1)
		return -EPROTO;
	virtio_msg_ffa_driver_store_effective_features_locked(drv);
	if (!drv->ep.bus_version)
		return -EOPNOTSUPP;
	peer_ffa_version = le32_to_cpu(version_resp->ffa_version);
	if (peer_ffa_version < FFA_VERSION_1_2)
		return -EOPNOTSUPP;
	drv->ep.ffa_version = min(drv->local_ffa_version, peer_ffa_version);

	return 0;
}

static int virtio_msg_ffa_reset_locked(struct virtio_msg_ffa_driver *drv)
{
	u8 reset_msg_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *reset_msg = (struct virtio_msg *)reset_msg_buf;
	struct virtio_msg_ffa_reset_resp *reset_resp;
	size_t reset_len;
	u16 result;
	int ret;

	virtio_msg_ffa_xfer_quiesce_locked(drv, &drv->active_xfer);
	virtio_msg_ffa_xfer_quiesce_locked(drv, &drv->bootstrap_xfer);
	memset(reset_msg_buf, 0, sizeof(reset_msg_buf));
	ret = virtio_msg_ffa_send_bus_request_locked(drv, FFA_BUS_MSG_RESET, NULL,
						     0, reset_msg,
						     sizeof(reset_msg_buf),
						     &reset_len);
	if (ret)
		return ret;

	if (reset_len != sizeof(*reset_msg) + sizeof(*reset_resp))
		return -EPROTO;
	reset_resp = (struct virtio_msg_ffa_reset_resp *)reset_msg->payload;
	result = le16_to_cpu(reset_resp->result);
	if (result == VIRTIO_MSG_FFA_BUS_BUSY) {
		drv->ep.reset_in_progress = true;
		return -EBUSY;
	}
	if (result != VIRTIO_MSG_FFA_BUS_SUCCESS)
		return -EIO;

	drv->ep.reset_in_progress = false;

	return virtio_msg_ffa_endpoint_teardown_locked(drv, true);
}

static int virtio_msg_ffa_reset_poll_locked(struct virtio_msg_ffa_driver *drv)
{
	int retries = VIRTIO_MSG_FFA_RESET_RETRY_BUDGET;
	int ret;

	while (retries--) {
		ret = virtio_msg_ffa_reset_locked(drv);
		if (ret != -EBUSY)
			return ret;

		if (!retries)
			break;
		usleep_range(1000, 2000);
	}

	if (drv->fdev)
		dev_warn_ratelimited(&drv->fdev->dev,
				     "reset remained busy after %d attempts\n",
				     VIRTIO_MSG_FFA_RESET_RETRY_BUDGET);

	return -ETIMEDOUT;
}

static void
virtio_msg_ffa_driver_store_effective_features_locked
					(struct virtio_msg_ffa_driver *drv)
{
	lockdep_assert_held(&drv->lock);

	drv->ep.bus_features &= drv->ep.local_bus_features;
	drv->ep.feature_bits &= VIRTIO_MSG_TRANSPORT_F_SUPPORTED;
}

static bool virtio_msg_ffa_driver_method_supports
			(const struct virtio_msg_ffa_driver_xfer_method *method,
			 u8 phase_flags, u32 bus_features, bool negotiated)
{
	const struct virtio_msg_ffa_xfer_method_desc *desc;

	if (!method || !method->desc)
		return false;
	desc = method->desc;
	if (desc->method == VIRTIO_MSG_FFA_XFER_NONE)
		return false;
	if (!(desc->role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DRIVER))
		return false;
	if (phase_flags && !(desc->phase_flags & phase_flags))
		return false;
	if (negotiated &&
	    ((bus_features & desc->bus_feature_mask) != desc->bus_feature_mask))
		return false;

	return true;
}

static bool virtio_msg_ffa_driver_method_capable
			(struct virtio_msg_ffa_driver *drv,
			 const struct virtio_msg_ffa_driver_xfer_method *method)
{
	return drv && method && method->capable && method->capable(drv);
}

static bool
virtio_msg_ffa_driver_xfer_capable(void *data,
				   enum virtio_msg_ffa_transfer_method method)
{
	const struct virtio_msg_ffa_driver_xfer_method *entry;

	entry = virtio_msg_ffa_driver_xfer_method_find(method);
	return virtio_msg_ffa_driver_method_capable(data, entry);
}

static u32 virtio_msg_ffa_driver_local_bus_features_locked
			(struct virtio_msg_ffa_driver *drv)
{
	const struct virtio_msg_ffa_driver_xfer_method *method;
	u32 deferred_features = 0;
	u32 bus_features = 0;
	bool bootstrap_capable = false;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_driver_xfer_methods); i++) {
		method = &virtio_msg_ffa_driver_xfer_methods[i];
		if (!virtio_msg_ffa_driver_method_supports
				(method, VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
				 0, false))
			continue;
		if (!virtio_msg_ffa_driver_xfer_capable(drv,
							method->desc->method))
			continue;

		if (method->desc->phase_flags &
		    VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP) {
			bus_features |= method->desc->bus_feature_mask;
			bootstrap_capable = true;
		} else {
			deferred_features |= method->desc->bus_feature_mask;
		}
	}

	if (bootstrap_capable)
		bus_features |= deferred_features;

	return bus_features;
}

static int virtio_msg_ffa_pick_bootstrap_xfer_locked
			(struct virtio_msg_ffa_driver *drv)
{
	const struct virtio_msg_ffa_driver_xfer_method *method;
	unsigned int i;

	if (!drv->fdev)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_driver_xfer_methods); i++) {
		method = &virtio_msg_ffa_driver_xfer_methods[i];
		if (!virtio_msg_ffa_driver_method_supports
				(method, VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP,
				 0, false))
			continue;
		if (!virtio_msg_ffa_driver_method_capable(drv, method))
			continue;

		return virtio_msg_ffa_xfer_engine_init(drv, &drv->bootstrap_xfer,
						       method);
	}

	return -EOPNOTSUPP;
}

static void virtio_msg_ffa_runtime_candidate_cleanup_locked
			(struct virtio_msg_ffa_driver *drv,
			 const struct virtio_msg_ffa_driver_xfer_method *method)
{
	struct virtio_msg_ffa_xfer_engine xfer;
	enum virtio_msg_ffa_transfer_method xfer_method;
	void *priv;
	bool shared_bootstrap;

	if (!drv || !method || !method->desc)
		return;

	xfer_method = method->desc->method;
	drv->event_configured = false;
	if (drv->ep.transfer_method == xfer_method)
		virtio_msg_ffa_select_transfer_method(&drv->ep,
						      VIRTIO_MSG_FFA_XFER_NONE);

	if (!virtio_msg_ffa_xfer_engine_valid(&drv->active_xfer) ||
	    drv->active_xfer.method != xfer_method)
		return;

	xfer = drv->active_xfer;
	virtio_msg_ffa_xfer_engine_reset(&drv->active_xfer);

	priv = virtio_msg_ffa_xfer_priv_get(drv, xfer_method);
	if (!priv || xfer.priv != priv)
		return;

	shared_bootstrap =
		drv->bootstrap_xfer.method == xfer.method &&
		drv->bootstrap_xfer.priv == xfer.priv;
	if (shared_bootstrap)
		return;

	virtio_msg_ffa_xfer_teardown_locked(drv, &xfer);
}

static int virtio_msg_ffa_event_configure_preferred_locked
					(struct virtio_msg_ffa_driver *drv)
{
	const struct virtio_msg_ffa_driver_xfer_method *method;
	int first_ret = 0;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_driver_xfer_methods); i++) {
		method = &virtio_msg_ffa_driver_xfer_methods[i];
		if (!virtio_msg_ffa_driver_method_supports
				(method, VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
				 drv->ep.bus_features, true))
			continue;
		if (!virtio_msg_ffa_driver_method_capable(drv, method))
			continue;
		if (!method->desc->driver_ops ||
		    !method->desc->driver_ops->init_runtime)
			continue;

		ret = method->desc->driver_ops->init_runtime(drv, method->desc);
		if (!ret)
			return virtio_msg_ffa_local_services_publish_locked(drv);

		if (!first_ret)
			first_ret = ret;
		if (drv->fdev)
			dev_warn_ratelimited(&drv->fdev->dev,
					     "runtime %s init failed ret=%d, trying next method\n",
					     method->desc->name, ret);
		virtio_msg_ffa_runtime_candidate_cleanup_locked(drv, method);
	}

	return first_ret ?: -EOPNOTSUPP;
}

int virtio_msg_ffa_driver_event_configure_send_locked(struct virtio_msg_ffa_driver *drv,
						      enum virtio_msg_ffa_bus_event_delivery method,
						      u16 notif_id)
{
	struct virtio_msg_ffa_event_configure_req req = { 0 };
	u8 resp_msg_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *resp_msg = (struct virtio_msg *)resp_msg_buf;
	size_t resp_len;
	__le16 result_le;
	u16 result;
	int ret;

	req.event_method = method;
	req.notif_id = cpu_to_le16(notif_id);

	memset(resp_msg_buf, 0, sizeof(resp_msg_buf));
	ret = virtio_msg_ffa_send_bus_request_locked(drv, FFA_BUS_MSG_EVENT_CONFIGURE,
						     &req, sizeof(req),
						     resp_msg,
						     sizeof(resp_msg_buf),
						     &resp_len);
	if (ret)
		return ret;
	if (resp_len != sizeof(*resp_msg) + sizeof(result_le))
		return -EPROTO;

	memcpy(&result_le, resp_msg->payload, sizeof(result_le));
	result = le16_to_cpu(result_le);
	if (result == VIRTIO_MSG_FFA_BUS_BUSY)
		return -EBUSY;
	if (result != VIRTIO_MSG_FFA_BUS_SUCCESS)
		return -EIO;

	return 0;
}

/*
 * Runtime setup starts by preparing a bootstrap transfer method so VERSION,
 * RESET, EVENT_CONFIGURE, and FIFO_CONFIGURE can be exchanged. If indirect is
 * selected for bootstrap, local services are published so the peer can deliver
 * indirect RX payloads while negotiation is still in progress. A pending RESET
 * is recovered before VERSION negotiation, then VERSION runs its probe/echo
 * sequence and stores the negotiated feature set. Event delivery is configured
 * last; the runtime transfer method is selected only after EVENT_CONFIGURE
 * succeeds. AREA_SHARE is not part of runtime negotiation and is sent later by
 * DMA area mapping.
 */
static int virtio_msg_ffa_prepare_runtime_locked
			(struct virtio_msg_ffa_driver *drv)
{
	int ret;

	if (drv->endpoint_disabled)
		return -EACCES;
	drv->ep.local_bus_features =
		virtio_msg_ffa_driver_local_bus_features_locked(drv);
	/*
	 * Runtime setup is ordered as bootstrap transport setup, reset recovery,
	 * VERSION negotiation, then event delivery configuration. The selected
	 * request path is not advertised as usable until event setup succeeds.
	 */
	if (drv->ep.negotiation_done && drv->event_configured &&
	    virtio_msg_ffa_xfer_engine_valid(&drv->active_xfer))
		return virtio_msg_ffa_local_services_publish_locked(drv);
	if (!virtio_msg_ffa_xfer_engine_valid(&drv->bootstrap_xfer)) {
		ret = virtio_msg_ffa_pick_bootstrap_xfer_locked(drv);
		if (ret)
			return ret;
	}
	ret = virtio_msg_ffa_xfer_setup_locked(drv, &drv->bootstrap_xfer);
	if (ret)
		return ret;
	if (drv->bootstrap_xfer.method == VIRTIO_MSG_FFA_XFER_INDIRECT) {
		ret = virtio_msg_ffa_local_services_publish_locked(drv);
		if (ret)
			return ret;
	}
	if (drv->ep.reset_in_progress) {
		ret = virtio_msg_ffa_reset_poll_locked(drv);
		if (ret) {
			if (ret == -ETIMEDOUT)
				virtio_msg_ffa_handle_endpoint_failure_locked
								(drv, ret);
			return ret;
		}
		if (!virtio_msg_ffa_xfer_engine_valid(&drv->bootstrap_xfer)) {
			ret = virtio_msg_ffa_pick_bootstrap_xfer_locked(drv);
			if (ret)
				return ret;
		}
		ret = virtio_msg_ffa_xfer_setup_locked(drv, &drv->bootstrap_xfer);
		if (ret)
			return ret;
		if (drv->bootstrap_xfer.method == VIRTIO_MSG_FFA_XFER_INDIRECT) {
			ret = virtio_msg_ffa_local_services_publish_locked(drv);
			if (ret)
				return ret;
		}
	}

	if (!drv->ep.negotiation_done) {
		ret = virtio_msg_ffa_version_negotiate_locked(drv);
		if (ret)
			return ret;
		if (drv->fdev) {
			dev_info(&drv->fdev->dev,
				 "runtime negotiated: peer_vm=%u bus=%u rev=%u feat=%#x bus_feat=%#x\n",
				 (u16)drv->fdev->vm_id,
				 drv->ep.bus_version,
				 drv->ep.transport_revision,
				 drv->ep.feature_bits,
				 drv->ep.bus_features);
		}
	}

	/*
	 * The current driver binding depends on event delivery to make forward
	 * progress, so missing EVENT_CONFIG support is treated as endpoint-
	 * fatal instead of degrading into a non-event-capable mode.
	 */
	return virtio_msg_ffa_event_configure_preferred_locked(drv);
}

static struct virtio_msg_ffa_driver *
virtio_msg_ffa_provider_callback_begin(struct virtio_msg_transport_device *vmdev)
{
	return virtio_msg_ffa_vmdev_callback_begin(vmdev);
}

static void
virtio_msg_ffa_provider_callback_end(struct virtio_msg_transport_device *vmdev)
{
	virtio_msg_ffa_vmdev_callback_end(vmdev);
}

static int virtio_msg_ffa_get_caps(struct virtio_msg_transport_device *vmdev,
				   struct virtio_msg_provider_caps *caps)
{
	struct virtio_msg_ffa_driver *drv;
	int ret = 0;

	if (!vmdev)
		return -EINVAL;

	drv = virtio_msg_ffa_provider_callback_begin(vmdev);
	if (!drv)
		return -ENODEV;
	if (!caps) {
		ret = -EINVAL;
		goto out_cb_end;
	}

	mutex_lock(&drv->lock);
	ret = virtio_msg_ffa_prepare_runtime_locked(drv);
	if (ret)
		goto out_unlock;

	memset(caps, 0, sizeof(*caps));
	caps->name = VIRTIO_MSG_FFA_BUS_NAME;
	caps->msg_size = FFA_BUS_MAX_MSG_SIZE;
	caps->revision = drv->ep.transport_revision;
	caps->transport_features =
		drv->ep.feature_bits & VIRTIO_MSG_TRANSPORT_F_SUPPORTED;

out_unlock:
	mutex_unlock(&drv->lock);
out_cb_end:
	virtio_msg_ffa_provider_callback_end(vmdev);
	return ret;
}

static int virtio_msg_ffa_send_request(struct virtio_msg_transport_device *vmdev,
				       const struct virtio_msg *request,
				       struct virtio_msg *response)
{
	struct virtio_msg_ffa_driver *drv;
	struct virtio_msg *request_local;
	u8 request_buf[FFA_BUS_MAX_MSG_SIZE];
	u16 token;
	size_t req_len, resp_len = 0;
	int ret = 0;

	if (!vmdev)
		return -EINVAL;

	drv = virtio_msg_ffa_provider_callback_begin(vmdev);
	if (!drv)
		return -ENODEV;
	if (!request || !response) {
		ret = -EINVAL;
		goto out_cb_end;
	}

	req_len = le16_to_cpu(request->msg_size);
	if (req_len > FFA_BUS_MAX_MSG_SIZE) {
		ret = -EMSGSIZE;
		goto out_cb_end;
	}

	mutex_lock(&drv->lock);
	ret = virtio_msg_ffa_prepare_runtime_locked(drv);
	if (ret)
		goto out_unlock;

	request_local = (struct virtio_msg *)request_buf;
	memcpy(request_local, request, req_len);
	ret = virtio_msg_ffa_next_token_locked(&drv->ep, &token);
	if (ret)
		goto out_unlock;
	request_local->token = cpu_to_le16(token);

	ret = virtio_msg_ffa_send_request_locked(drv, request_local, req_len,
						 VIRTIO_MSG_TYPE_RESPONSE,
						 request_local->msg_id,
						 response,
						 FFA_BUS_MAX_MSG_SIZE,
						 &resp_len);

out_unlock:
	mutex_unlock(&drv->lock);
out_cb_end:
	virtio_msg_ffa_provider_callback_end(vmdev);
	return ret;
}

int virtio_msg_ffa_driver_submit_event(struct virtio_msg_ffa_driver *drv,
				       const struct virtio_msg *request)
{
	u8 request_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *request_local = (struct virtio_msg *)request_buf;
	size_t req_len;
	u16 token;
	int ret;

	if (!drv || !request)
		return -EINVAL;

	req_len = le16_to_cpu(request->msg_size);
	if (req_len < sizeof(*request) || req_len > FFA_BUS_MAX_MSG_SIZE)
		return -EMSGSIZE;

	mutex_lock(&drv->lock);
	ret = virtio_msg_ffa_prepare_runtime_locked(drv);
	if (ret)
		goto out_unlock;
	if (!virtio_msg_ffa_driver_event_runtime_ready_locked(drv)) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	memcpy(request_local, request, req_len);
	ret = virtio_msg_ffa_next_token_locked(&drv->ep, &token);
	if (ret)
		goto out_unlock;
	request_local->token = cpu_to_le16(token);

	ret = virtio_msg_ffa_send_by_selected_method_locked
			(drv, &drv->active_xfer, request_local, req_len, NULL, 0,
			 NULL);

out_unlock:
	mutex_unlock(&drv->lock);
	return ret;
}

static int virtio_msg_ffa_send_event(struct virtio_msg_transport_device *vmdev,
				     const struct virtio_msg *request)
{
	struct virtio_msg_ffa_driver *drv;
	size_t req_len;
	int ret;

	if (!vmdev)
		return -EINVAL;
	if (!request)
		return -EINVAL;
	if (!virtio_msg_ffa_msg_is_event(request->msg_id))
		return -EINVAL;
	req_len = le16_to_cpu(request->msg_size);
	if (req_len < sizeof(*request) || req_len > FFA_BUS_MAX_MSG_SIZE)
		return -EMSGSIZE;

	drv = virtio_msg_ffa_provider_callback_begin(vmdev);
	if (!drv)
		return -ENODEV;

	ret = virtio_msg_ffa_vmdev_enqueue_event(vmdev, request, GFP_ATOMIC);
	if (ret == -ENOMEM)
		ret = -ENOSPC;

	virtio_msg_ffa_provider_callback_end(vmdev);
	return ret;
}

static int virtio_msg_ffa_send(struct virtio_msg_transport_device *vmdev,
			       const struct virtio_msg *request,
			       struct virtio_msg *response)
{
	if (!vmdev || !request)
		return -EINVAL;
	if (!response)
		return virtio_msg_ffa_send_event(vmdev, request);

	return virtio_msg_ffa_send_request(vmdev, request, response);
}

static void
virtio_msg_ffa_synchronize_cbs(struct virtio_msg_transport_device *vmdev)
{
	virtio_msg_ffa_vmdev_callback_wait_zero(vmdev);
}

static void virtio_msg_ffa_release(struct virtio_msg_transport_device *vmdev)
{
	(void)vmdev;
}

static const struct virtio_msg_bus_provider_ops virtio_msg_ffa_bus_ops = {
	.get_caps		= virtio_msg_ffa_get_caps,
	.send			= virtio_msg_ffa_send,
	.synchronize_cbs	= virtio_msg_ffa_synchronize_cbs,
	.release		= virtio_msg_ffa_release,
};

const struct virtio_msg_bus_provider_ops *virtio_msg_ffa_bus_ops_get(void)
{
	return &virtio_msg_ffa_bus_ops;
}

static int virtio_msg_ffa_probe(struct ffa_device *fdev)
{
	struct virtio_msg_ffa_driver *drv;
	int ret;

	if (!fdev)
		return -EINVAL;
	dev_info(&fdev->dev, "driver binding probe: peer_vm=%u\n",
		 (u16)fdev->vm_id);

	ret = dma_set_mask_and_coherent(&fdev->dev, DMA_BIT_MASK(64));
	if (ret)
		ret = dma_set_mask_and_coherent(&fdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(&fdev->dev, ret,
				     "failed to configure DMA mask\n");

	drv = kzalloc(sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->fdev = fdev;
	mutex_init(&drv->lock);
	spin_lock_init(&drv->bus_req_lock);
	init_waitqueue_head(&drv->bus_req_wait);
	virtio_msg_ffa_endpoint_init(&drv->ep);
	virtio_msg_ffa_area_ctx_alloc(drv);
	virtio_msg_ffa_version_begin(&drv->ep);
	virtio_msg_ffa_xfer_engine_reset(&drv->bootstrap_xfer);
	virtio_msg_ffa_xfer_engine_reset(&drv->active_xfer);
	WRITE_ONCE(drv->indirect_rx_generation, 0);
	drv->local_ffa_version = FFA_VERSION_1_2;
	if (fdev->ops && fdev->ops->info_ops &&
	    fdev->ops->info_ops->api_version_get)
		drv->local_ffa_version = fdev->ops->info_ops->api_version_get();
	drv->ep.local_bus_features =
		virtio_msg_ffa_driver_local_bus_features_locked(drv);
	ffa_dev_set_drvdata(fdev, drv);

	ret = virtio_msg_ffa_driver_indirect_rx_init(drv);
	if (ret)
		goto err_cleanup_endpoint;

	ret = virtio_msg_ffa_topology_init(drv);
	if (ret)
		goto err_stop_indirect_rx;

	dev_info(&fdev->dev, "driver binding probe complete: peer_vm=%u\n",
		 (u16)fdev->vm_id);
	return 0;

err_stop_indirect_rx:
	virtio_msg_ffa_driver_indirect_rx_quiesce(drv);
err_cleanup_endpoint:
	ffa_dev_set_drvdata(fdev, NULL);
	virtio_msg_ffa_exchange_abort_all(&drv->ep, -ESHUTDOWN);
	virtio_msg_ffa_endpoint_cleanup(&drv->ep);
	virtio_msg_ffa_area_ctx_free(&drv->area_ctx);
	kfree(drv);
	return ret;
}

static void virtio_msg_ffa_remove(struct ffa_device *fdev)
{
	struct virtio_msg_ffa_driver *drv;
	struct virtio_msg_ffa_topology *topo;

	if (!fdev)
		return;

	drv = ffa_dev_get_drvdata(fdev);
	if (!drv)
		return;
	dev_info(&fdev->dev, "driver binding remove: peer_vm=%u\n",
		 (u16)fdev->vm_id);

	ffa_dev_set_drvdata(fdev, NULL);
	mutex_lock(&drv->lock);
	virtio_msg_ffa_endpoint_disable(drv);
	WRITE_ONCE(drv->indirect_rx_generation,
		   READ_ONCE(drv->indirect_rx_generation) + 1);
	virtio_msg_ffa_exchange_abort_all(&drv->ep, -ESHUTDOWN);
	virtio_msg_ffa_local_services_unpublish_locked(drv);
	rcu_read_lock();
	topo = virtio_msg_ffa_driver_topology_get(drv);
	rcu_read_unlock();
	virtio_msg_ffa_driver_topology_set(drv, NULL);
	virtio_msg_ffa_method_teardown_locked(drv);
	mutex_unlock(&drv->lock);
	virtio_msg_ffa_driver_indirect_rx_quiesce(drv);
	synchronize_rcu();
	virtio_msg_ffa_topology_destroy(topo);
	virtio_msg_ffa_xfer_priv_release_all(drv);

	virtio_msg_ffa_endpoint_cleanup(&drv->ep);
	virtio_msg_ffa_area_ctx_free(&drv->area_ctx);
	kfree(drv);
}

static const struct ffa_device_id virtio_msg_ffa_device_ids[] = {
	{ .uuid = virtio_msg_ffa_device_uuid },
	{}
};

static struct ffa_driver virtio_msg_ffa_driver = {
	.name = "virtio-msg-ffa",
	.probe = virtio_msg_ffa_probe,
	.remove = virtio_msg_ffa_remove,
	.id_table = virtio_msg_ffa_device_ids,
	.local_uuid_table = virtio_msg_ffa_local_uuid_ids,
#if IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_INDIRECT)
	.rx_msg = virtio_msg_ffa_indirect_rx_cb,
#endif
};

static int __init virtio_msg_ffa_init(void)
{
	int ret;

	ret = ffa_register(&virtio_msg_ffa_driver);
	if (ret) {
		pr_debug("driver binding register failed: ffa ret=%d\n", ret);
		return ret;
	}

	ret = virtio_msg_ffa_local_services_module_publish();
	if (ret == -EOPNOTSUPP) {
		pr_debug("driver local service publish unsupported; continuing without dynamic publication\n");
		return 0;
	}
	if (ret) {
		ffa_unregister(&virtio_msg_ffa_driver);
		return ret;
	}

	return 0;
}

static void __exit virtio_msg_ffa_exit(void)
{
	virtio_msg_ffa_local_services_module_unpublish();
	ffa_unregister(&virtio_msg_ffa_driver);
}

module_init(virtio_msg_ffa_init);
module_exit(virtio_msg_ffa_exit);

MODULE_DESCRIPTION("Virtio message bus over FF-A driver binding");
MODULE_LICENSE("GPL");
