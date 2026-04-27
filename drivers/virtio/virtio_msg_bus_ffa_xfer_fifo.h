/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus over FF-A FIFO transfer helper contracts.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_FFA_XFER_FIFO_H
#define _VIRTIO_MSG_BUS_FFA_XFER_FIFO_H

#include <linux/compiler_attributes.h>
#include <linux/minmax.h>
#include <linux/wait.h>

#include "virtio_msg_bus_ffa_xfer.h"

#define VIRTIO_MSG_FFA_FIFO_NOTIFY_RETRY_BASE_MS	1
#define VIRTIO_MSG_FFA_FIFO_NOTIFY_RETRY_MAX_MS		32
#define VIRTIO_MSG_FFA_FIFO_TX_TIMEOUT_MS		5000U

struct virtio_msg_ffa_fifo_ring {
	struct virtio_msg_ffa_fifo_hdr *hdr;
	u8 *entries;
	size_t total_bytes;
	u16 message_size;
	u16 depth;
};

struct virtio_msg_ffa_fifo_state {
	struct virtio_msg_ffa_fifo_ring tx;
	struct virtio_msg_ffa_fifo_ring rx;
	wait_queue_head_t space_waitq;
	void *region;
	size_t region_len;
	u64 mem_handle;
	u64 space_seq;
	u64 generation;
	u16 page_count;
	u16 local_notif_id;
	u16 peer_notif_id;
	bool rx_notif_bound;
	bool configured;
};

typedef int (*virtio_msg_ffa_fifo_rx_cb_t)(struct virtio_msg_ffa_endpoint *ep,
					   const struct virtio_msg *msg,
					   size_t msg_len, void *cb_data);

static inline void
virtio_msg_ffa_fifo_notify_backoff_reset(u32 *delay_ms)
{
	if (delay_ms)
		*delay_ms = VIRTIO_MSG_FFA_FIFO_NOTIFY_RETRY_BASE_MS;
}

static inline u32 virtio_msg_ffa_fifo_notify_backoff_delay(u32 *delay_ms)
{
	if (!delay_ms)
		return VIRTIO_MSG_FFA_FIFO_NOTIFY_RETRY_BASE_MS;
	if (!*delay_ms)
		virtio_msg_ffa_fifo_notify_backoff_reset(delay_ms);

	return *delay_ms;
}

static inline bool
virtio_msg_ffa_fifo_notify_backoff_next(u32 *delay_ms, u32 *next_delay_ms)
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

static inline void
virtio_msg_ffa_fifo_init(struct virtio_msg_ffa_fifo_state *state)
{
	if (state)
		init_waitqueue_head(&state->space_waitq);
}

static inline void
virtio_msg_ffa_fifo_space_wake(struct virtio_msg_ffa_fifo_state *state)
{
	if (!state)
		return;

	WRITE_ONCE(state->space_seq, READ_ONCE(state->space_seq) + 1);
	wake_up_all(&state->space_waitq);
}

static inline void
virtio_msg_ffa_fifo_state_reset(struct virtio_msg_ffa_fifo_state *state)
{
	if (!state)
		return;

	state->tx = (struct virtio_msg_ffa_fifo_ring){ };
	state->rx = (struct virtio_msg_ffa_fifo_ring){ };
	state->region = NULL;
	state->region_len = 0;
	state->mem_handle = 0;
	state->page_count = 0;
	state->local_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	state->peer_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	state->rx_notif_bound = false;
	state->configured = false;
	WRITE_ONCE(state->generation, READ_ONCE(state->generation) + 1);
	virtio_msg_ffa_fifo_space_wake(state);
}

static inline int
virtio_msg_ffa_fifo_wait_space(struct virtio_msg_ffa_fifo_state *state,
			       u64 space_seq, u64 generation,
			       long *remaining)
{
	long timeout;

	if (!state || !remaining)
		return -EINVAL;
	if (*remaining <= 0)
		return -ETIMEDOUT;

	timeout = wait_event_timeout
			(state->space_waitq,
			 READ_ONCE(state->space_seq) != space_seq ||
			 READ_ONCE(state->generation) != generation ||
			 !READ_ONCE(state->configured),
			 *remaining);
	if (!timeout) {
		*remaining = 0;
		return -ETIMEDOUT;
	}

	*remaining = timeout;
	return 0;
}

#if IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_FIFO)
int virtio_msg_ffa_fifo_configure_layout(struct virtio_msg_ffa_fifo_state *state,
					 void *region, size_t region_len,
					 u64 mem_handle, u16 page_count);
void virtio_msg_ffa_fifo_reset(struct virtio_msg_ffa_fifo_state *state);
int virtio_msg_ffa_fifo_bind_rx_notification(struct ffa_device *fdev,
					     struct virtio_msg_ffa_fifo_state *state,
					     ffa_notifier_cb cb,
					     void *cb_data);
int virtio_msg_ffa_fifo_unbind_rx_notification(struct ffa_device *fdev,
					       struct virtio_msg_ffa_fifo_state *state);
int virtio_msg_ffa_fifo_set_peer_notif_id(struct virtio_msg_ffa_fifo_state *state,
					  u16 peer_notif_id);
int virtio_msg_ffa_fifo_enqueue(struct virtio_msg_ffa_fifo_state *state,
				const struct virtio_msg *req,
				size_t req_len);
int virtio_msg_ffa_fifo_dequeue(struct virtio_msg_ffa_fifo_state *state,
				struct virtio_msg *resp,
				size_t resp_buf_len, size_t *resp_len,
				bool *was_full_before_read);
int virtio_msg_ffa_fifo_drain_rx(struct virtio_msg_ffa_xfer_ctx *ctx,
				 struct virtio_msg_ffa_fifo_state *state,
				 virtio_msg_ffa_fifo_rx_cb_t cb,
				 void *cb_data, u32 *drained);
int virtio_msg_ffa_fifo_notify_peer(struct ffa_device *fdev,
				    const struct virtio_msg_ffa_fifo_state *state);
#else
#define VIRTIO_MSG_FFA_FIFO_EOPNOTSUPP_STUB()	(-EOPNOTSUPP)

static inline int
virtio_msg_ffa_fifo_configure_layout
		(struct virtio_msg_ffa_fifo_state *state __maybe_unused,
		 void *region __maybe_unused, size_t region_len __maybe_unused,
		 u64 mem_handle __maybe_unused, u16 page_count __maybe_unused)
{
	return VIRTIO_MSG_FFA_FIFO_EOPNOTSUPP_STUB();
}

static inline void virtio_msg_ffa_fifo_reset(struct virtio_msg_ffa_fifo_state *state)
{
	virtio_msg_ffa_fifo_state_reset(state);
}

static inline int
virtio_msg_ffa_fifo_bind_rx_notification
		(struct ffa_device *fdev __maybe_unused,
		 struct virtio_msg_ffa_fifo_state *state __maybe_unused,
		 ffa_notifier_cb cb __maybe_unused,
		 void *cb_data __maybe_unused)
{
	return VIRTIO_MSG_FFA_FIFO_EOPNOTSUPP_STUB();
}

static inline int
virtio_msg_ffa_fifo_unbind_rx_notification
		(struct ffa_device *fdev __maybe_unused,
		 struct virtio_msg_ffa_fifo_state *state __maybe_unused)
{
	return VIRTIO_MSG_FFA_FIFO_EOPNOTSUPP_STUB();
}

static inline int
virtio_msg_ffa_fifo_set_peer_notif_id
		(struct virtio_msg_ffa_fifo_state *state __maybe_unused,
		 u16 peer_notif_id __maybe_unused)
{
	return VIRTIO_MSG_FFA_FIFO_EOPNOTSUPP_STUB();
}

static inline int
virtio_msg_ffa_fifo_enqueue
		(struct virtio_msg_ffa_fifo_state *state __maybe_unused,
		 const struct virtio_msg *req __maybe_unused,
		 size_t req_len __maybe_unused)
{
	return VIRTIO_MSG_FFA_FIFO_EOPNOTSUPP_STUB();
}

static inline int
virtio_msg_ffa_fifo_dequeue(struct virtio_msg_ffa_fifo_state *state,
			    struct virtio_msg *resp, size_t resp_buf_len,
			    size_t *resp_len, bool *was_full_before_read)
{
	(void)state;
	(void)resp;
	(void)resp_buf_len;
	if (resp_len)
		*resp_len = 0;
	if (was_full_before_read)
		*was_full_before_read = false;

	return -EOPNOTSUPP;
}

static inline int
virtio_msg_ffa_fifo_drain_rx(struct virtio_msg_ffa_xfer_ctx *ctx,
			     struct virtio_msg_ffa_fifo_state *state,
			     virtio_msg_ffa_fifo_rx_cb_t cb,
			     void *cb_data, u32 *drained)
{
	(void)ctx;
	(void)state;
	(void)cb;
	(void)cb_data;
	if (drained)
		*drained = 0;

	return -EOPNOTSUPP;
}

static inline int
virtio_msg_ffa_fifo_notify_peer
		(struct ffa_device *fdev __maybe_unused,
		 const struct virtio_msg_ffa_fifo_state *state __maybe_unused)
{
	return VIRTIO_MSG_FFA_FIFO_EOPNOTSUPP_STUB();
}

#undef VIRTIO_MSG_FFA_FIFO_EOPNOTSUPP_STUB
#endif

#endif /* _VIRTIO_MSG_BUS_FFA_XFER_FIFO_H */
