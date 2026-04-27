/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus over FF-A shared runtime declarations.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_FFA_H
#define _VIRTIO_MSG_BUS_FFA_H

#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/types.h>
#include <linux/virtio_msg_protocol.h>
#include <linux/xarray.h>

#include "virtio_msg_bus_ffa_protocol.h"

struct ffa_device;
struct ffa_mem_ops_args;
struct ffa_mem_retrieve_args;
struct device;

#define VIRTIO_MSG_FFA_BUS_NAME			"virtio-msg-ffa"
#define VIRTIO_MSG_FFA_TX_BUSY_RETRY_MAX		10

enum virtio_msg_ffa_transfer_method {
	VIRTIO_MSG_FFA_XFER_NONE = 0,
	VIRTIO_MSG_FFA_XFER_DIRECT,
	VIRTIO_MSG_FFA_XFER_INDIRECT,
	VIRTIO_MSG_FFA_XFER_FIFO,
};

enum virtio_msg_ffa_area_state {
	VIRTIO_MSG_FFA_AREA_UNUSED = 0,
	VIRTIO_MSG_FFA_AREA_SHARED,
	VIRTIO_MSG_FFA_AREA_UNSHARE_PENDING,
	VIRTIO_MSG_FFA_AREA_RELEASED,
};

enum virtio_msg_ffa_device_publication_state {
	VIRTIO_MSG_FFA_DEV_DISCOVERED = 0,
	VIRTIO_MSG_FFA_DEV_CONFIGURED,
	VIRTIO_MSG_FFA_DEV_PUBLISHED,
	VIRTIO_MSG_FFA_DEV_REMOVED,
};

enum virtio_msg_ffa_preneg_action {
	VIRTIO_MSG_FFA_PRENEG_ALLOW = 0,
	VIRTIO_MSG_FFA_PRENEG_REJECT_DEVICE_ERROR,
	VIRTIO_MSG_FFA_PRENEG_REJECT_LOCAL_DROP,
};

enum virtio_msg_ffa_error_owner {
	VIRTIO_MSG_FFA_ERROR_OWNER_UNKNOWN = 0,
	VIRTIO_MSG_FFA_ERROR_OWNER_TRANSPORT,
	VIRTIO_MSG_FFA_ERROR_OWNER_BUS,
};

enum virtio_msg_ffa_runtime_target {
	VIRTIO_MSG_FFA_RUNTIME_TARGET_REQUEST = 0,
	VIRTIO_MSG_FFA_RUNTIME_TARGET_EVENT,
};

struct virtio_msg_ffa_exchange {
	u16 dev_num;
	u16 token;
	u8 expected_msg_id;
	u8 expected_type;
	u16 reserved0;
	u32 key;
	refcount_t refcount;
	struct completion done;
	u8 resp_buf[FFA_BUS_MAX_MSG_SIZE];
	size_t resp_len;
	int status;
};

struct virtio_msg_ffa_area {
	u16 area_id;
	enum virtio_msg_ffa_area_state state;
	u64 mem_tag;
	u32 page_count;
	u32 sharing_attrs;
};

struct virtio_msg_ffa_device_state {
	u16 dev_num;
	enum virtio_msg_ffa_device_publication_state state;
};

struct virtio_msg_ffa_endpoint {
	/* VERSION negotiation state */
	u32 bus_version;
	u32 transport_revision;
	u32 ffa_version;
	u32 peer_feature_bits;
	u32 peer_bus_features;
	u32 feature_bits;
	u32 bus_features;
	u32 local_bus_features;
	u16 max_areas;
	bool negotiation_done;
	bool reset_in_progress;

	/* Selected runtime behavior from negotiated capabilities */
	enum virtio_msg_ffa_transfer_method transfer_method;
	enum virtio_msg_ffa_bus_event_delivery event_delivery;
	u16 next_token;

	/* Shared runtime registries */
	struct xarray exchanges;
	struct xarray areas;
	struct xarray devices;
	struct mutex exchange_lock; /* Protects @exchanges updates. */
	bool exchange_shutdown; /* Guarded by @exchange_lock. */
	struct mutex area_lock; /* Protects @areas updates. */
	struct mutex device_lock; /* Protects @devices updates. */
};

struct virtio_msg_ffa_error_result {
	bool matched;
	u16 dev_num;
	u16 token;
	u16 original_msg_id;
	u8 expected_msg_id;
	u8 expected_type;
	enum virtio_msg_ffa_error_owner owner;
};

enum virtio_msg_ffa_inbound_class {
	VIRTIO_MSG_FFA_INBOUND_DROP = 0,
	VIRTIO_MSG_FFA_INBOUND_EVENT,
	VIRTIO_MSG_FFA_INBOUND_RESPONSE,
	VIRTIO_MSG_FFA_INBOUND_BUS_REQUEST,
	VIRTIO_MSG_FFA_INBOUND_TRANSPORT_REQUEST,
	VIRTIO_MSG_FFA_INBOUND_PRENEG_ERROR_RESPONSE,
};

struct virtio_msg_ffa_inbound_result {
	enum virtio_msg_ffa_inbound_class class;
	u8 type_bits;
	struct virtio_msg_ffa_exchange exchange;
	int match_status;
	struct virtio_msg_ffa_error_result error;
};

struct virtio_msg_ffa_area_ctx;
typedef int (*virtio_msg_ffa_bus_req_fn)(struct virtio_msg_ffa_area_ctx *ctx,
					u8 msg_id, const void *payload,
					size_t payload_len,
					struct virtio_msg *resp,
					size_t resp_buf_len,
					size_t *resp_len);

struct virtio_msg_ffa_area_ctx {
	struct ffa_device *fdev;
	struct virtio_msg_ffa_endpoint *ep;
	virtio_msg_ffa_bus_req_fn send_bus_req;
};

static inline u32 virtio_msg_ffa_exchange_key(u16 dev_num, u16 token)
{
	return ((u32)dev_num << 16) | token;
}

static inline bool virtio_msg_ffa_msg_is_event(u8 msg_id)
{
	return !!(msg_id & VIRTIO_MSG_ID_EVENT_BIT);
}

static inline void
virtio_msg_ffa_queue_splice_locked(struct list_head *queue, u32 *depth,
				   struct list_head *free_list)
{
	if (!queue || !depth || !free_list)
		return;

	list_splice_init(queue, free_list);
	*depth = 0;
}

static inline struct list_head *
virtio_msg_ffa_queue_pop_locked(struct list_head *queue, u32 *depth)
{
	struct list_head *node;

	if (!queue || !depth || list_empty(queue))
		return NULL;
	if (!*depth)
		return NULL;

	node = queue->next;
	list_del(node);
	(*depth)--;

	return node;
}

static inline int
virtio_msg_ffa_queue_push_tail_locked(struct list_head *queue,
				      struct list_head *node, u32 *depth,
				      u32 max_depth)
{
	if (!queue || !node || !depth)
		return -EINVAL;
	if (*depth >= max_depth)
		return -ENOSPC;

	list_add_tail(node, queue);
	(*depth)++;

	return 0;
}

static inline void
virtio_msg_ffa_queue_push_head_locked(struct list_head *queue,
				      struct list_head *node, u32 *depth)
{
	if (!queue || !node || !depth)
		return;

	list_add(node, queue);
	(*depth)++;
}

void virtio_msg_ffa_endpoint_init(struct virtio_msg_ffa_endpoint *ep);
void virtio_msg_ffa_endpoint_cleanup(struct virtio_msg_ffa_endpoint *ep);
int virtio_msg_ffa_next_token_locked(struct virtio_msg_ffa_endpoint *ep,
				     u16 *token);
void virtio_msg_ffa_exchange_abort_all(struct virtio_msg_ffa_endpoint *ep,
				       int status);
int virtio_msg_ffa_area_add(struct virtio_msg_ffa_endpoint *ep,
			    const struct virtio_msg_ffa_area *area);
int virtio_msg_ffa_area_lookup(struct virtio_msg_ffa_endpoint *ep,
			       u16 area_id, struct virtio_msg_ffa_area *out);
int virtio_msg_ffa_area_mark_released(struct virtio_msg_ffa_endpoint *ep,
				      u16 area_id);
int virtio_msg_ffa_area_remove(struct virtio_msg_ffa_endpoint *ep,
			       u16 area_id, struct virtio_msg_ffa_area *removed);

int virtio_msg_ffa_exchange_store(struct virtio_msg_ffa_endpoint *ep,
				  u16 dev_num, u16 token,
				  u8 expected_msg_id, u8 expected_type);
int virtio_msg_ffa_exchange_lookup(struct virtio_msg_ffa_endpoint *ep,
				   u16 dev_num, u16 token,
				   struct virtio_msg_ffa_exchange *out);
int virtio_msg_ffa_exchange_release(struct virtio_msg_ffa_endpoint *ep,
				    u16 dev_num, u16 token,
				    struct virtio_msg_ffa_exchange *released);
int virtio_msg_ffa_exchange_complete(struct virtio_msg_ffa_endpoint *ep,
				     u16 dev_num, u16 token,
				     const struct virtio_msg *resp,
				     size_t resp_len, int status);
int virtio_msg_ffa_exchange_wait(struct virtio_msg_ffa_endpoint *ep,
				 u16 dev_num, u16 token,
				 struct virtio_msg *resp,
				 size_t resp_buf_len,
				 size_t *resp_len,
				 unsigned long timeout_ms);

int virtio_msg_ffa_validate_inbound_msg(const struct virtio_msg *msg, size_t len);
int virtio_msg_ffa_match_inbound(struct virtio_msg_ffa_endpoint *ep,
				 const struct virtio_msg *msg,
				 struct virtio_msg_ffa_exchange *exchange,
				 bool *is_event);
enum virtio_msg_ffa_preneg_action
virtio_msg_ffa_classify_preneg_msg(const struct virtio_msg_ffa_endpoint *ep,
				   const struct virtio_msg *msg);
int virtio_msg_ffa_classify_inbound(struct virtio_msg_ffa_endpoint *ep,
				    const struct virtio_msg *msg, size_t msg_len,
				    struct virtio_msg_ffa_inbound_result *result);

void virtio_msg_ffa_version_begin(struct virtio_msg_ffa_endpoint *ep);
void virtio_msg_ffa_version_prepare_probe(struct virtio_msg_ffa_version_req *req,
					  u32 local_ffa_version);
void virtio_msg_ffa_version_prepare_echo(struct virtio_msg_ffa_version_req *req,
					 const struct virtio_msg_ffa_version_resp *resp);
bool virtio_msg_ffa_version_is_echoed(const struct virtio_msg_ffa_version_req *req,
				      const struct virtio_msg_ffa_version_resp *resp);
int virtio_msg_ffa_version_accept_resp(struct virtio_msg_ffa_endpoint *ep,
				       const struct virtio_msg_ffa_version_req *req,
				       const struct virtio_msg_ffa_version_resp *resp);

void virtio_msg_ffa_select_transfer_method(struct virtio_msg_ffa_endpoint *ep,
					   enum virtio_msg_ffa_transfer_method method);
void virtio_msg_ffa_select_event_delivery(struct virtio_msg_ffa_endpoint *ep,
					  enum virtio_msg_ffa_bus_event_delivery event_delivery);
int virtio_msg_ffa_memory_share(struct ffa_device *fdev,
				struct ffa_mem_ops_args *args);
int virtio_msg_ffa_memory_retrieve(struct ffa_device *fdev,
				   struct ffa_mem_retrieve_args *args,
				   void *resp, size_t *resp_len);
bool virtio_msg_ffa_tx_busy_retry_sleepable(unsigned int *remaining);
void virtio_msg_ffa_trace_msg(struct device *dev, u16 peer_vm,
			      const char *dir, const char *path,
			      const struct virtio_msg *msg, size_t len);
int virtio_msg_ffa_prepare_frame(struct virtio_msg *msg, size_t buf_len,
				 u8 msg_id, u8 type, u16 dev_num, u16 token,
				 const void *payload, size_t payload_len,
				 size_t *msg_len);

enum virtio_msg_ffa_error_owner
virtio_msg_ffa_error_owner_from_msg_id(u16 original_msg_id);
int virtio_msg_ffa_handle_error_resp(struct virtio_msg_ffa_endpoint *ep,
				     const struct virtio_msg *msg, size_t len,
				     struct virtio_msg_ffa_error_result *result);

#endif /* _VIRTIO_MSG_BUS_FFA_H */
