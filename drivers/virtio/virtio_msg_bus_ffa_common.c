// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A shared, method-agnostic runtime helpers.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/byteorder/little_endian.h>
#include <linux/arm_ffa.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/virtio_msg_bus_provider.h>

#include "virtio_msg_bus_ffa.h"
#include "virtio_msg_bus_ffa_xfer.h"

#define VIRTIO_MSG_FFA_MEM_OP_RETRY_MAX		20
#define VIRTIO_MSG_FFA_MEM_OP_RETRY_BASE_MS	20
#define VIRTIO_MSG_FFA_MEM_OP_RETRY_MAX_MS	100
#define VIRTIO_MSG_FFA_TX_BUSY_RETRY_MIN_US	1000
#define VIRTIO_MSG_FFA_TX_BUSY_RETRY_MAX_US	2000

static int virtio_msg_ffa_retry_busy_sleepable(int (*op)(void *data),
					       void *data)
{
	u32 delay = VIRTIO_MSG_FFA_MEM_OP_RETRY_BASE_MS;
	int attempt;
	int ret;

	for (attempt = 0; attempt < VIRTIO_MSG_FFA_MEM_OP_RETRY_MAX; attempt++) {
		ret = op(data);
		if (ret != -EBUSY)
			break;
		if (attempt + 1 >= VIRTIO_MSG_FFA_MEM_OP_RETRY_MAX)
			break;
		might_sleep();
		msleep(delay);
		delay = min_t(u32, delay * 2, VIRTIO_MSG_FFA_MEM_OP_RETRY_MAX_MS);
	}

	return ret;
}

bool virtio_msg_ffa_tx_busy_retry_sleepable(unsigned int *remaining)
{
	if (!remaining || !*remaining)
		return false;

	(*remaining)--;
	if (!*remaining)
		return false;

	might_sleep();
	usleep_range(VIRTIO_MSG_FFA_TX_BUSY_RETRY_MIN_US,
		     VIRTIO_MSG_FFA_TX_BUSY_RETRY_MAX_US);
	return true;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_tx_busy_retry_sleepable);

void virtio_msg_ffa_fifo_notify_backoff_reset(u32 *delay_ms)
{
	if (delay_ms)
		*delay_ms = VIRTIO_MSG_FFA_FIFO_NOTIFY_RETRY_BASE_MS;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_notify_backoff_reset);

u32 virtio_msg_ffa_fifo_notify_backoff_delay(u32 *delay_ms)
{
	if (!delay_ms)
		return VIRTIO_MSG_FFA_FIFO_NOTIFY_RETRY_BASE_MS;
	if (!*delay_ms)
		virtio_msg_ffa_fifo_notify_backoff_reset(delay_ms);

	return *delay_ms;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_notify_backoff_delay);

bool virtio_msg_ffa_fifo_notify_backoff_next(u32 *delay_ms,
					     u32 *next_delay_ms)
{
	u32 next_delay;

	if (!delay_ms || !next_delay_ms)
		return false;
	if (virtio_msg_ffa_fifo_notify_backoff_delay(delay_ms) >=
	    VIRTIO_MSG_FFA_FIFO_NOTIFY_RETRY_MAX_MS)
		return false;

	next_delay = min_t(u32, *delay_ms << 1,
			   VIRTIO_MSG_FFA_FIFO_NOTIFY_RETRY_MAX_MS);
	*delay_ms = next_delay;
	*next_delay_ms = next_delay;

	return true;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_notify_backoff_next);

struct virtio_msg_ffa_memory_share_ctx {
	struct ffa_device *fdev;
	struct ffa_mem_ops_args *args;
};

static int virtio_msg_ffa_memory_share_once(void *data)
{
	struct virtio_msg_ffa_memory_share_ctx *ctx = data;

	return ctx->fdev->ops->mem_ops->memory_share(ctx->args);
}

int virtio_msg_ffa_memory_share(struct ffa_device *fdev,
				struct ffa_mem_ops_args *args)
{
	struct virtio_msg_ffa_memory_share_ctx ctx = {
		.fdev = fdev,
		.args = args,
	};

	if (!fdev || !args)
		return -EINVAL;
	if (!fdev->ops || !fdev->ops->mem_ops ||
	    !fdev->ops->mem_ops->memory_share)
		return -EOPNOTSUPP;

	return virtio_msg_ffa_retry_busy_sleepable
			(virtio_msg_ffa_memory_share_once, &ctx);
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_memory_share);

struct virtio_msg_ffa_memory_retrieve_ctx {
	struct ffa_device *fdev;
	struct ffa_mem_retrieve_args *args;
	void *resp;
	size_t *resp_len;
};

static int virtio_msg_ffa_memory_retrieve_once(void *data)
{
	struct virtio_msg_ffa_memory_retrieve_ctx *ctx = data;

	return ctx->fdev->ops->mem_ops->memory_retrieve(ctx->args, ctx->resp,
							ctx->resp_len);
}

int virtio_msg_ffa_memory_retrieve(struct ffa_device *fdev,
				   struct ffa_mem_retrieve_args *args,
				   void *resp, size_t *resp_len)
{
	struct virtio_msg_ffa_memory_retrieve_ctx ctx = {
		.fdev = fdev,
		.args = args,
		.resp = resp,
		.resp_len = resp_len,
	};

	if (!fdev || !args)
		return -EINVAL;
	if (!fdev->ops || !fdev->ops->mem_ops ||
	    !fdev->ops->mem_ops->memory_retrieve)
		return -EOPNOTSUPP;

	return virtio_msg_ffa_retry_busy_sleepable
			(virtio_msg_ffa_memory_retrieve_once, &ctx);
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_memory_retrieve);

void virtio_msg_ffa_trace_msg(struct device *dev, u16 peer_vm,
			      const char *dir, const char *path,
			      const struct virtio_msg *msg, size_t len)
{
	if (!msg)
		return;

	dev_dbg(dev,
		"bus %s %s: peer_vm=%u type=0x%x msg_id=0x%02x dev=%u tok=%u size=%u len=%zu\n",
		dir, path ? path : "unknown", peer_vm, msg->type, msg->msg_id,
		le16_to_cpu(msg->dev_num), le16_to_cpu(msg->token),
		le16_to_cpu(msg->msg_size), len);
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_trace_msg);

int virtio_msg_ffa_prepare_frame(struct virtio_msg *msg, size_t buf_len,
				 u8 msg_id, u8 type, u16 dev_num, u16 token,
				 const void *payload, size_t payload_len,
				 size_t *msg_len)
{
	if (!msg || !msg_len)
		return -EINVAL;
	if (payload_len && !payload)
		return -EINVAL;
	if (buf_len < sizeof(*msg))
		return -EMSGSIZE;
	if (payload_len > buf_len - sizeof(*msg))
		return -EMSGSIZE;

	memset(msg, 0, buf_len);
	virtio_msg_prepare(msg, msg_id, token, payload_len, dev_num);
	msg->type = type;
	if (payload_len)
		memcpy(msg->payload, payload, payload_len);

	*msg_len = sizeof(*msg) + payload_len;
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_prepare_frame);

static void virtio_msg_ffa_free_xarray_entries(struct xarray *xa)
{
	unsigned long idx;
	void *entry;

	xa_for_each(xa, idx, entry) {
		xa_erase(xa, idx);
		kfree(entry);
	}
}

static void virtio_msg_ffa_exchange_put(struct virtio_msg_ffa_exchange *entry)
{
	if (!entry)
		return;

	if (refcount_dec_and_test(&entry->refcount))
		kfree(entry);
}

static void virtio_msg_ffa_exchange_snapshot(struct virtio_msg_ffa_exchange *dst,
					     const struct virtio_msg_ffa_exchange *src)
{
	dst->dev_num = src->dev_num;
	dst->token = src->token;
	dst->expected_msg_id = src->expected_msg_id;
	dst->expected_type = src->expected_type;
	dst->reserved0 = src->reserved0;
	dst->key = src->key;
}

void virtio_msg_ffa_endpoint_init(struct virtio_msg_ffa_endpoint *ep)
{
	if (!ep)
		return;

	memset(ep, 0, sizeof(*ep));
	ep->next_token = VIRTIO_MSG_TOKEN_FIXED + 1;
	xa_init(&ep->exchanges);
	xa_init(&ep->areas);
	xa_init(&ep->devices);
	mutex_init(&ep->exchange_lock);
	ep->exchange_shutdown = false;
	mutex_init(&ep->area_lock);
	mutex_init(&ep->device_lock);
	ep->transfer_method = VIRTIO_MSG_FFA_XFER_NONE;
	ep->event_delivery = VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_endpoint_init);

int virtio_msg_ffa_next_token_locked(struct virtio_msg_ffa_endpoint *ep,
				     u16 *token)
{
	u16 next;

	if (!ep || !token)
		return -EINVAL;

	next = ep->next_token;
	if (next <= VIRTIO_MSG_TOKEN_FIXED)
		next = VIRTIO_MSG_TOKEN_FIXED + 1;

	*token = next;
	ep->next_token = next + 1;
	if (ep->next_token <= VIRTIO_MSG_TOKEN_FIXED)
		ep->next_token = VIRTIO_MSG_TOKEN_FIXED + 1;

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_next_token_locked);

void virtio_msg_ffa_exchange_abort_all(struct virtio_msg_ffa_endpoint *ep,
				       int status)
{
	struct virtio_msg_ffa_exchange *entry;
	unsigned long idx = 0;

	if (!ep)
		return;
	if (WARN_ON(!status))
		return;

	mutex_lock(&ep->exchange_lock);
	ep->exchange_shutdown = true;
	for (;;) {
		entry = xa_find(&ep->exchanges, &idx, ULONG_MAX, XA_PRESENT);
		if (!entry)
			break;

		refcount_inc(&entry->refcount);
		entry->resp_len = 0;
		entry->status = status;
		xa_erase(&ep->exchanges, idx);
		mutex_unlock(&ep->exchange_lock);

		complete(&entry->done);
		virtio_msg_ffa_exchange_put(entry);
		virtio_msg_ffa_exchange_put(entry);

		idx = 0;
		mutex_lock(&ep->exchange_lock);
	}
	mutex_unlock(&ep->exchange_lock);
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_exchange_abort_all);

void virtio_msg_ffa_endpoint_cleanup(struct virtio_msg_ffa_endpoint *ep)
{
	if (!ep)
		return;

	mutex_lock(&ep->exchange_lock);
	WARN_ON(!xa_empty(&ep->exchanges));
	mutex_unlock(&ep->exchange_lock);
	xa_destroy(&ep->exchanges);

	mutex_lock(&ep->area_lock);
	virtio_msg_ffa_free_xarray_entries(&ep->areas);
	mutex_unlock(&ep->area_lock);
	xa_destroy(&ep->areas);

	mutex_lock(&ep->device_lock);
	virtio_msg_ffa_free_xarray_entries(&ep->devices);
	mutex_unlock(&ep->device_lock);
	xa_destroy(&ep->devices);
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_endpoint_cleanup);

int virtio_msg_ffa_area_add(struct virtio_msg_ffa_endpoint *ep,
			    const struct virtio_msg_ffa_area *area)
{
	struct virtio_msg_ffa_area *entry, *old;

	if (!ep || !area)
		return -EINVAL;

	entry = kmemdup(area, sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	mutex_lock(&ep->area_lock);
	old = xa_store(&ep->areas, area->area_id, entry, GFP_KERNEL);
	mutex_unlock(&ep->area_lock);
	if (IS_ERR(old)) {
		kfree(entry);
		return PTR_ERR(old);
	}

	kfree(old);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_area_add);

int virtio_msg_ffa_area_lookup(struct virtio_msg_ffa_endpoint *ep,
			       u16 area_id, struct virtio_msg_ffa_area *out)
{
	struct virtio_msg_ffa_area *entry;

	if (!ep || !out)
		return -EINVAL;

	mutex_lock(&ep->area_lock);
	entry = xa_load(&ep->areas, area_id);
	if (entry)
		*out = *entry;
	mutex_unlock(&ep->area_lock);
	if (!entry)
		return -ENOENT;

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_area_lookup);

int virtio_msg_ffa_area_mark_released(struct virtio_msg_ffa_endpoint *ep,
				      u16 area_id)
{
	struct virtio_msg_ffa_area *entry;

	if (!ep)
		return -EINVAL;

	mutex_lock(&ep->area_lock);
	entry = xa_load(&ep->areas, area_id);
	if (entry)
		entry->state = VIRTIO_MSG_FFA_AREA_RELEASED;
	mutex_unlock(&ep->area_lock);
	if (!entry)
		return -ENOENT;

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_area_mark_released);

int virtio_msg_ffa_area_remove(struct virtio_msg_ffa_endpoint *ep,
			       u16 area_id, struct virtio_msg_ffa_area *removed)
{
	struct virtio_msg_ffa_area *entry;

	if (!ep)
		return -EINVAL;

	mutex_lock(&ep->area_lock);
	entry = xa_erase(&ep->areas, area_id);
	mutex_unlock(&ep->area_lock);
	if (!entry)
		return -ENOENT;

	if (removed)
		*removed = *entry;
	kfree(entry);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_area_remove);

int virtio_msg_ffa_exchange_store(struct virtio_msg_ffa_endpoint *ep,
				  u16 dev_num, u16 token,
				  u8 expected_msg_id, u8 expected_type)
{
	struct virtio_msg_ffa_exchange *exchange;
	u32 key;
	int ret;

	if (!ep)
		return -EINVAL;

	key = virtio_msg_ffa_exchange_key(dev_num, token);
	exchange = kzalloc(sizeof(*exchange), GFP_KERNEL);
	if (!exchange)
		return -ENOMEM;

	exchange->dev_num = dev_num;
	exchange->token = token;
	exchange->expected_msg_id = expected_msg_id;
	exchange->expected_type = expected_type;
	exchange->key = key;
	refcount_set(&exchange->refcount, 1);
	init_completion(&exchange->done);
	exchange->resp_len = 0;
	exchange->status = -EINPROGRESS;

	mutex_lock(&ep->exchange_lock);
	if (ep->exchange_shutdown)
		ret = -ESHUTDOWN;
	else
		ret = xa_insert(&ep->exchanges, key, exchange, GFP_KERNEL);
	mutex_unlock(&ep->exchange_lock);
	if (ret)
		kfree(exchange);

	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_exchange_store);

int virtio_msg_ffa_exchange_lookup(struct virtio_msg_ffa_endpoint *ep,
				   u16 dev_num, u16 token,
				   struct virtio_msg_ffa_exchange *out)
{
	struct virtio_msg_ffa_exchange *entry;
	u32 key;

	if (!ep || !out)
		return -EINVAL;

	key = virtio_msg_ffa_exchange_key(dev_num, token);
	mutex_lock(&ep->exchange_lock);
	entry = xa_load(&ep->exchanges, key);
	if (!entry) {
		mutex_unlock(&ep->exchange_lock);
		return -ENOENT;
	}

	virtio_msg_ffa_exchange_snapshot(out, entry);
	mutex_unlock(&ep->exchange_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_exchange_lookup);

int virtio_msg_ffa_exchange_release(struct virtio_msg_ffa_endpoint *ep,
				    u16 dev_num, u16 token,
				    struct virtio_msg_ffa_exchange *released)
{
	struct virtio_msg_ffa_exchange *entry;
	u32 key;

	if (!ep)
		return -EINVAL;

	key = virtio_msg_ffa_exchange_key(dev_num, token);
	mutex_lock(&ep->exchange_lock);
	entry = xa_erase(&ep->exchanges, key);
	mutex_unlock(&ep->exchange_lock);
	if (!entry)
		return -ENOENT;

	if (released)
		virtio_msg_ffa_exchange_snapshot(released, entry);
	virtio_msg_ffa_exchange_put(entry);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_exchange_release);

int virtio_msg_ffa_exchange_complete(struct virtio_msg_ffa_endpoint *ep,
				     u16 dev_num, u16 token,
				     const struct virtio_msg *resp,
				     size_t resp_len, int status)
{
	struct virtio_msg_ffa_exchange *entry;
	u32 key;

	if (!ep)
		return -EINVAL;

	key = virtio_msg_ffa_exchange_key(dev_num, token);
	mutex_lock(&ep->exchange_lock);
	entry = xa_load(&ep->exchanges, key);
	if (!entry) {
		mutex_unlock(&ep->exchange_lock);
		return -ENOENT;
	}

	refcount_inc(&entry->refcount);
	if (!status) {
		if (!resp || !resp_len) {
			mutex_unlock(&ep->exchange_lock);
			virtio_msg_ffa_exchange_put(entry);
			return -EINVAL;
		}
		if (resp_len > sizeof(entry->resp_buf)) {
			mutex_unlock(&ep->exchange_lock);
			virtio_msg_ffa_exchange_put(entry);
			return -EMSGSIZE;
		}
		memcpy(entry->resp_buf, resp, resp_len);
		entry->resp_len = resp_len;
	} else {
		entry->resp_len = 0;
	}

	entry->status = status;
	mutex_unlock(&ep->exchange_lock);
	complete(&entry->done);
	virtio_msg_ffa_exchange_put(entry);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_exchange_complete);

int virtio_msg_ffa_exchange_wait(struct virtio_msg_ffa_endpoint *ep,
				 u16 dev_num, u16 token,
				 struct virtio_msg *resp,
				 size_t resp_buf_len,
				 size_t *resp_len,
				 unsigned long timeout_ms)
{
	struct virtio_msg_ffa_exchange *entry;
	unsigned long timeout;
	u32 key;
	int status;

	if (!ep || !resp || !resp_len)
		return -EINVAL;

	key = virtio_msg_ffa_exchange_key(dev_num, token);
	mutex_lock(&ep->exchange_lock);
	entry = xa_load(&ep->exchanges, key);
	if (!entry) {
		int err = ep->exchange_shutdown ? -ESHUTDOWN : -ENOENT;

		mutex_unlock(&ep->exchange_lock);
		return err;
	}
	refcount_inc(&entry->refcount);
	mutex_unlock(&ep->exchange_lock);

	timeout = msecs_to_jiffies(timeout_ms);
	if (!wait_for_completion_timeout(&entry->done, timeout)) {
		bool removed = false;

		mutex_lock(&ep->exchange_lock);
		if (xa_load(&ep->exchanges, key) == entry) {
			xa_erase(&ep->exchanges, key);
			removed = true;
		}
		mutex_unlock(&ep->exchange_lock);
		if (removed)
			virtio_msg_ffa_exchange_put(entry);
		virtio_msg_ffa_exchange_put(entry);
		return -ETIMEDOUT;
	}

	mutex_lock(&ep->exchange_lock);
	status = entry->status;
	if (status) {
		mutex_unlock(&ep->exchange_lock);
		virtio_msg_ffa_exchange_put(entry);
		return status;
	}
	if (entry->resp_len > resp_buf_len) {
		mutex_unlock(&ep->exchange_lock);
		virtio_msg_ffa_exchange_put(entry);
		return -EMSGSIZE;
	}
	memcpy(resp, entry->resp_buf, entry->resp_len);
	*resp_len = entry->resp_len;
	mutex_unlock(&ep->exchange_lock);
	virtio_msg_ffa_exchange_put(entry);

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_exchange_wait);

int virtio_msg_ffa_validate_inbound_msg(const struct virtio_msg *msg, size_t len)
{
	u16 dev_num;
	u16 msg_size;

	if (!msg)
		return -EINVAL;
	if (len > FFA_BUS_MAX_MSG_SIZE)
		return -EMSGSIZE;
	if (len < sizeof(*msg))
		return -EMSGSIZE;

	msg_size = le16_to_cpu(msg->msg_size);
	if (msg_size < sizeof(*msg) || msg_size > FFA_BUS_MAX_MSG_SIZE)
		return -EMSGSIZE;
	if (len < msg_size)
		return -EMSGSIZE;

	dev_num = le16_to_cpu(msg->dev_num);
	if ((msg->type & VIRTIO_MSG_TYPE_BUS) &&
	    msg->msg_id != FFA_BUS_MSG_ERROR && dev_num)
		return -EPROTO;

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_validate_inbound_msg);

static u16 virtio_msg_ffa_response_dev_num(const struct virtio_msg *msg)
{
	if ((msg->type & VIRTIO_MSG_TYPE_BUS) &&
	    msg->msg_id != FFA_BUS_MSG_ERROR)
		return 0;

	return le16_to_cpu(msg->dev_num);
}

int virtio_msg_ffa_match_inbound(struct virtio_msg_ffa_endpoint *ep,
				 const struct virtio_msg *msg,
				 struct virtio_msg_ffa_exchange *exchange,
				 bool *is_event)
{
	const u8 type_mask = VIRTIO_MSG_TYPE_RESPONSE | VIRTIO_MSG_TYPE_BUS;
	u16 dev_num, token;
	int ret;

	if (!ep || !msg)
		return -EINVAL;

	if (virtio_msg_ffa_msg_is_event(msg->msg_id)) {
		if (is_event)
			*is_event = true;
		return 0;
	}

	if (is_event)
		*is_event = false;

	dev_num = virtio_msg_ffa_response_dev_num(msg);
	token = le16_to_cpu(msg->token);
	ret = virtio_msg_ffa_exchange_lookup(ep, dev_num, token, exchange);
	if (ret)
		return ret;

	if (exchange->expected_type &&
	    ((msg->type & type_mask) != (exchange->expected_type & type_mask)))
		return -EPROTO;
	/*
	 * Keep inbound correlation keyed on (dev_num, token).
	 *
	 * msg_id mismatch is advisory only: deliver to the waiter and let the
	 * send-side validator decide whether to accept or reject it.
	 */

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_match_inbound);

enum virtio_msg_ffa_preneg_action
virtio_msg_ffa_classify_preneg_msg(const struct virtio_msg_ffa_endpoint *ep,
				   const struct virtio_msg *msg)
{
	if (!ep || !msg)
		return VIRTIO_MSG_FFA_PRENEG_REJECT_LOCAL_DROP;

	if (ep->negotiation_done)
		return VIRTIO_MSG_FFA_PRENEG_ALLOW;

	if (msg->msg_id == FFA_BUS_MSG_VERSION || msg->msg_id == FFA_BUS_MSG_RESET)
		return VIRTIO_MSG_FFA_PRENEG_ALLOW;

	if (virtio_msg_ffa_msg_is_event(msg->msg_id))
		return VIRTIO_MSG_FFA_PRENEG_REJECT_LOCAL_DROP;

	return VIRTIO_MSG_FFA_PRENEG_REJECT_DEVICE_ERROR;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_classify_preneg_msg);

static int
virtio_msg_ffa_error_resp_correlate(struct virtio_msg_ffa_endpoint *ep,
				    const struct virtio_msg *msg, size_t len,
				    bool consume,
				    struct virtio_msg_ffa_error_result *result)
{
	struct virtio_msg_ffa_exchange exchange;
	const struct virtio_msg_ffa_error_resp *error;
	u16 dev_num, token, original_msg_id;
	size_t expected_msg_size;
	int ret;

	if (result)
		memset(result, 0, sizeof(*result));

	ret = virtio_msg_ffa_validate_inbound_msg(msg, len);
	if (ret)
		return ret;
	if (msg->msg_id != FFA_BUS_MSG_ERROR)
		return -EINVAL;
	expected_msg_size = sizeof(*msg) + sizeof(*error);
	if (le16_to_cpu(msg->msg_size) != expected_msg_size)
		return -EMSGSIZE;

	dev_num = le16_to_cpu(msg->dev_num);
	token = le16_to_cpu(msg->token);
	if (consume)
		ret = virtio_msg_ffa_exchange_release(ep, dev_num, token, &exchange);
	else
		ret = virtio_msg_ffa_exchange_lookup(ep, dev_num, token, &exchange);
	if (ret == -ENOENT)
		return 0;
	if (ret)
		return ret;

	error = (const struct virtio_msg_ffa_error_resp *)msg->payload;
	original_msg_id = le16_to_cpu(error->original_msg_id);
	if (exchange.expected_msg_id && original_msg_id != exchange.expected_msg_id)
		return -EPROTO;

	if (result) {
		result->matched = true;
		result->dev_num = dev_num;
		result->token = token;
		result->original_msg_id = original_msg_id;
		result->expected_msg_id = exchange.expected_msg_id;
		result->expected_type = exchange.expected_type;
		result->owner =
			virtio_msg_ffa_error_owner_from_msg_id(original_msg_id);
	}

	return 1;
}

int virtio_msg_ffa_classify_inbound(struct virtio_msg_ffa_endpoint *ep,
				    const struct virtio_msg *msg, size_t msg_len,
				    struct virtio_msg_ffa_inbound_result *result)
{
	const u8 type_mask = VIRTIO_MSG_TYPE_RESPONSE | VIRTIO_MSG_TYPE_BUS;
	enum virtio_msg_ffa_preneg_action preneg_action;
	int ret;

	if (!ep || !msg || !result)
		return -EINVAL;

	memset(result, 0, sizeof(*result));

	ret = virtio_msg_ffa_validate_inbound_msg(msg, msg_len);
	if (ret)
		return ret;

	result->type_bits = msg->type & type_mask;
	if (virtio_msg_ffa_msg_is_event(msg->msg_id)) {
		preneg_action = virtio_msg_ffa_classify_preneg_msg(ep, msg);
		if (preneg_action == VIRTIO_MSG_FFA_PRENEG_REJECT_LOCAL_DROP)
			return 0;
		result->class = VIRTIO_MSG_FFA_INBOUND_EVENT;
		return 0;
	}

	if (msg->type & VIRTIO_MSG_TYPE_RESPONSE) {
		ret = virtio_msg_ffa_match_inbound(ep, msg, &result->exchange,
						   NULL);
		if (ret == -ENOENT) {
			if (msg->msg_id != FFA_BUS_MSG_ERROR)
				return 0;

			ret = virtio_msg_ffa_error_resp_correlate(ep, msg, msg_len,
								  false, NULL);
			if (ret < 0)
				return ret;
			return 0;
		}
		if (ret && ret != -EPROTO)
			return ret;

		result->class = VIRTIO_MSG_FFA_INBOUND_RESPONSE;
		result->match_status = ret;
		if (msg->msg_id == FFA_BUS_MSG_ERROR) {
			int corr_ret;

			corr_ret = virtio_msg_ffa_error_resp_correlate(ep, msg,
								       msg_len,
								       false,
								       &result->error);
			if (corr_ret < 0)
				result->match_status = -EPROTO;
			else
				result->match_status = corr_ret;
		}

		return 0;
	}

	preneg_action = virtio_msg_ffa_classify_preneg_msg(ep, msg);
	if (preneg_action == VIRTIO_MSG_FFA_PRENEG_REJECT_LOCAL_DROP)
		return 0;
	if (preneg_action == VIRTIO_MSG_FFA_PRENEG_REJECT_DEVICE_ERROR) {
		result->class = VIRTIO_MSG_FFA_INBOUND_PRENEG_ERROR_RESPONSE;
		return 0;
	}

	if (result->type_bits == VIRTIO_MSG_TYPE_BUS) {
		result->class = VIRTIO_MSG_FFA_INBOUND_BUS_REQUEST;
		return 0;
	}
	if (!result->type_bits) {
		result->class = VIRTIO_MSG_FFA_INBOUND_TRANSPORT_REQUEST;
		return 0;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_classify_inbound);

/*
 * VERSION negotiation starts with local runtime capability discovery, then
 * the probe/echo exchange narrows that local set with peer-advertised
 * features before any non-bootstrap runtime method is selected.
 */
void virtio_msg_ffa_version_begin(struct virtio_msg_ffa_endpoint *ep)
{
	if (!ep)
		return;

	ep->bus_version = 0;
	ep->transport_revision = 0;
	ep->feature_bits = 0;
	ep->bus_features = 0;
	ep->peer_feature_bits = 0;
	ep->peer_bus_features = 0;
	ep->max_areas = 0;
	ep->negotiation_done = false;
	ep->reset_in_progress = false;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_version_begin);

void virtio_msg_ffa_version_prepare_probe(struct virtio_msg_ffa_version_req *req,
					  u32 local_ffa_version)
{
	if (!req)
		return;

	req->bus_version = cpu_to_le32(0);
	req->transport_revision = cpu_to_le32(0);
	req->ffa_version = cpu_to_le32(local_ffa_version);
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_version_prepare_probe);

void virtio_msg_ffa_version_prepare_echo(struct virtio_msg_ffa_version_req *req,
					 const struct virtio_msg_ffa_version_resp *resp)
{
	if (!req || !resp)
		return;

	req->bus_version = resp->bus_version;
	req->transport_revision = resp->transport_revision;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_version_prepare_echo);

bool virtio_msg_ffa_version_is_echoed(const struct virtio_msg_ffa_version_req *req,
				      const struct virtio_msg_ffa_version_resp *resp)
{
	if (!req || !resp)
		return false;

	return req->bus_version == resp->bus_version &&
	       req->transport_revision == resp->transport_revision;
}

int virtio_msg_ffa_version_accept_resp(struct virtio_msg_ffa_endpoint *ep,
				       const struct virtio_msg_ffa_version_req *req,
				       const struct virtio_msg_ffa_version_resp *resp)
{
	u32 peer_bus_features;
	u32 peer_feature_bits;

	if (!ep || !req || !resp)
		return -EINVAL;

	peer_feature_bits = le32_to_cpu(resp->feature_bits);
	peer_bus_features = le32_to_cpu(resp->bus_features);

	ep->bus_version = le32_to_cpu(resp->bus_version);
	ep->transport_revision = le32_to_cpu(resp->transport_revision);
	ep->ffa_version = le32_to_cpu(resp->ffa_version);
	ep->peer_feature_bits = peer_feature_bits;
	ep->peer_bus_features = peer_bus_features;
	ep->feature_bits = peer_feature_bits & VIRTIO_MSG_TRANSPORT_F_SUPPORTED;
	ep->bus_features = peer_bus_features & ep->local_bus_features;
	ep->max_areas = le16_to_cpu(resp->max_areas);
	ep->negotiation_done = virtio_msg_ffa_version_is_echoed(req, resp);

	return ep->negotiation_done ? 1 : 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_version_accept_resp);

void virtio_msg_ffa_select_transfer_method(struct virtio_msg_ffa_endpoint *ep,
					   enum virtio_msg_ffa_transfer_method method)
{
	if (!ep)
		return;

	ep->transfer_method = method;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_select_transfer_method);

void virtio_msg_ffa_select_event_delivery(struct virtio_msg_ffa_endpoint *ep,
					  enum virtio_msg_ffa_bus_event_delivery event_delivery)
{
	if (!ep)
		return;

	ep->event_delivery = event_delivery;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_select_event_delivery);

enum virtio_msg_ffa_error_owner
virtio_msg_ffa_error_owner_from_msg_id(u16 original_msg_id)
{
	switch (original_msg_id) {
	case FFA_BUS_MSG_VERSION:
	case FFA_BUS_MSG_AREA_SHARE:
	case FFA_BUS_MSG_AREA_UNSHARE:
	case FFA_BUS_MSG_RESET:
	case FFA_BUS_MSG_EVENT_POLL:
	case FFA_BUS_MSG_EVENT_CONFIGURE:
	case FFA_BUS_MSG_FIFO_CONFIGURE:
	case FFA_BUS_MSG_ERROR:
	case FFA_BUS_EVENT_AREA_RELEASE:
		return VIRTIO_MSG_FFA_ERROR_OWNER_BUS;
	default:
		return VIRTIO_MSG_FFA_ERROR_OWNER_TRANSPORT;
	}
}

int virtio_msg_ffa_handle_error_resp(struct virtio_msg_ffa_endpoint *ep,
				     const struct virtio_msg *msg, size_t len,
				     struct virtio_msg_ffa_error_result *result)
{
	return virtio_msg_ffa_error_resp_correlate(ep, msg, len, true, result);
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_handle_error_resp);

MODULE_DESCRIPTION("Virtio message bus over FF-A common helpers");
MODULE_LICENSE("GPL");
