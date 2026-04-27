// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A FIFO-method helpers.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/byteorder/little_endian.h>
#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/overflow.h>
#include <linux/string.h>

#include "virtio_msg_bus_ffa_xfer_fifo.h"

static bool virtio_msg_ffa_notif_id_is_valid(u16 notif_id)
{
	return notif_id >= VIRTIO_MSG_FFA_NOTIF_ID_MIN &&
	       notif_id <= VIRTIO_MSG_FFA_NOTIF_ID_MAX;
}

static bool virtio_msg_ffa_notif_id_from_api_is_valid(int notif_id)
{
	if (notif_id < VIRTIO_MSG_FFA_NOTIF_ID_MIN || notif_id > U16_MAX)
		return false;

	return virtio_msg_ffa_notif_id_is_valid((u16)notif_id);
}

static u16 virtio_msg_ffa_fifo_read_index(const struct virtio_msg_ffa_fifo_ring *ring)
{
	/* Pair with writer store-release after entry consumption. */
	return le16_to_cpu(smp_load_acquire(&ring->hdr->read_index));
}

static u16 virtio_msg_ffa_fifo_write_index(const struct virtio_msg_ffa_fifo_ring *ring)
{
	/* Pair with writer store-release after entry publish. */
	return le16_to_cpu(smp_load_acquire(&ring->hdr->write_index));
}

static void virtio_msg_ffa_fifo_set_read_index(struct virtio_msg_ffa_fifo_ring *ring,
					       u16 read)
{
	/* Ensure entry reads complete before advancing read index. */
	smp_store_release(&ring->hdr->read_index, cpu_to_le16(read));
}

static void virtio_msg_ffa_fifo_set_write_index(struct virtio_msg_ffa_fifo_ring *ring,
						u16 write)
{
	/* Ensure entry writes are visible before publishing write index. */
	smp_store_release(&ring->hdr->write_index, cpu_to_le16(write));
}

static int virtio_msg_ffa_fifo_indices_get(const struct virtio_msg_ffa_fifo_ring *ring,
					   u16 *read, u16 *write)
{
	u16 rd, wr;

	if (!ring->depth)
		return -EPROTO;

	rd = virtio_msg_ffa_fifo_read_index(ring);
	wr = virtio_msg_ffa_fifo_write_index(ring);
	if (rd >= ring->depth || wr >= ring->depth)
		return -EPROTO;

	*read = rd;
	*write = wr;

	return 0;
}

static int virtio_msg_ffa_fifo_validate_ring(struct virtio_msg_ffa_fifo_ring *ring,
					     void *base, size_t available)
{
	struct virtio_msg_ffa_fifo_hdr *hdr = base;
	size_t total_bytes;
	u16 version, message_size, depth;

	if (available < VIRTIO_MSG_FFA_FIFO_HEADER_SIZE)
		return -EMSGSIZE;
	if (memcmp(hdr->magic, VIRTIO_MSG_FFA_FIFO_MAGIC, sizeof(hdr->magic)))
		return -EPROTO;

	version = le16_to_cpu(hdr->version);
	if (version != VIRTIO_MSG_FFA_FIFO_VERSION)
		return -EPROTONOSUPPORT;

	message_size = le16_to_cpu(hdr->message_size);
	depth = le16_to_cpu(hdr->depth);
	if (!depth || message_size < FFA_BUS_MAX_MSG_SIZE)
		return -EPROTO;

	if (check_mul_overflow((size_t)message_size, (size_t)depth, &total_bytes))
		return -EOVERFLOW;
	if (check_add_overflow(total_bytes,
			       (size_t)VIRTIO_MSG_FFA_FIFO_MSG_AREA_OFFSET,
			       &total_bytes))
		return -EOVERFLOW;
	if (total_bytes > available)
		return -EMSGSIZE;

	ring->hdr = hdr;
	ring->entries = (u8 *)base + VIRTIO_MSG_FFA_FIFO_MSG_AREA_OFFSET;
	ring->total_bytes = total_bytes;
	ring->message_size = message_size;
	ring->depth = depth;

	return 0;
}

static const struct ffa_notifier_ops *
virtio_msg_ffa_notifier_ops_get(struct ffa_device *fdev)
{
	if (!fdev || !fdev->ops)
		return NULL;

	return fdev->ops->notifier_ops;
}

static void
virtio_msg_ffa_fifo_clear_layout(struct virtio_msg_ffa_fifo_state *state)
{
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
}

static void
virtio_msg_ffa_fifo_advance_generation(struct virtio_msg_ffa_fifo_state *state)
{
	WRITE_ONCE(state->generation, READ_ONCE(state->generation) + 1);
	virtio_msg_ffa_fifo_space_wake(state);
}

int virtio_msg_ffa_fifo_configure_layout(struct virtio_msg_ffa_fifo_state *state,
					 void *region, size_t region_len,
					 u64 mem_handle, u16 page_count)
{
	struct virtio_msg_ffa_fifo_ring tx = { 0 };
	struct virtio_msg_ffa_fifo_ring rx = { 0 };
	u32 next_offset;
	int ret;

	if (!state || !region || !region_len)
		return -EINVAL;

	ret = virtio_msg_ffa_fifo_validate_ring(&tx, region, region_len);
	if (ret)
		return ret;

	next_offset = le32_to_cpu(tx.hdr->next_offset);
	if (next_offset < tx.total_bytes || next_offset >= region_len)
		return -EPROTO;

	ret = virtio_msg_ffa_fifo_validate_ring(&rx, (u8 *)region + next_offset,
						region_len - next_offset);
	if (ret)
		return ret;
	if (le32_to_cpu(rx.hdr->next_offset))
		return -EPROTO;

	virtio_msg_ffa_fifo_clear_layout(state);
	state->tx = tx;
	state->rx = rx;
	state->region = region;
	state->region_len = region_len;
	state->mem_handle = mem_handle;
	state->page_count = page_count;
	state->local_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	state->peer_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	state->configured = true;
	virtio_msg_ffa_fifo_advance_generation(state);

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_configure_layout);

void virtio_msg_ffa_fifo_reset(struct virtio_msg_ffa_fifo_state *state)
{
	virtio_msg_ffa_fifo_state_reset(state);
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_reset);

int virtio_msg_ffa_fifo_bind_rx_notification(struct ffa_device *fdev,
					     struct virtio_msg_ffa_fifo_state *state,
					     ffa_notifier_cb cb,
					     void *cb_data)
{
	const struct ffa_notifier_ops *notifier_ops;
	int notif_id;
	int ret;

	if (!state || !state->configured || !cb)
		return -EINVAL;

	notifier_ops = virtio_msg_ffa_notifier_ops_get(fdev);
	if (!notifier_ops || !notifier_ops->notify_alloc)
		return -EOPNOTSUPP;

	ret = notifier_ops->notify_alloc(fdev, false, cb, cb_data, &notif_id);
	if (ret)
		return ret;
	if (!virtio_msg_ffa_notif_id_from_api_is_valid(notif_id)) {
		if (notifier_ops->notify_relinquish)
			notifier_ops->notify_relinquish(fdev, notif_id);
		return -ERANGE;
	}

	state->local_notif_id = (u16)notif_id;
	state->rx_notif_bound = true;
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_bind_rx_notification);

int virtio_msg_ffa_fifo_unbind_rx_notification(struct ffa_device *fdev,
					       struct virtio_msg_ffa_fifo_state *state)
{
	const struct ffa_notifier_ops *notifier_ops;
	int ret;

	if (!state)
		return -EINVAL;
	if (!state->rx_notif_bound)
		return 0;

	if (!virtio_msg_ffa_notif_id_is_valid(state->local_notif_id))
		return -EINVAL;

	notifier_ops = virtio_msg_ffa_notifier_ops_get(fdev);
	if (!notifier_ops || !notifier_ops->notify_relinquish)
		return -EOPNOTSUPP;

	ret = notifier_ops->notify_relinquish(fdev, state->local_notif_id);
	if (ret)
		return ret;

	state->rx_notif_bound = false;
	state->local_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_unbind_rx_notification);

int virtio_msg_ffa_fifo_set_peer_notif_id(struct virtio_msg_ffa_fifo_state *state,
					  u16 peer_notif_id)
{
	if (!state || !state->configured)
		return -EINVAL;
	if (!virtio_msg_ffa_notif_id_is_valid(peer_notif_id))
		return -EINVAL;

	state->peer_notif_id = peer_notif_id;
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_set_peer_notif_id);

int virtio_msg_ffa_fifo_notify_peer(struct ffa_device *fdev,
				    const struct virtio_msg_ffa_fifo_state *state)
{
	const struct ffa_notifier_ops *notifier_ops;

	if (!state || !state->configured)
		return -EINVAL;
	if (!virtio_msg_ffa_notif_id_is_valid(state->peer_notif_id))
		return -EINVAL;

	notifier_ops = virtio_msg_ffa_notifier_ops_get(fdev);
	if (!notifier_ops || !notifier_ops->notify_send)
		return -EOPNOTSUPP;

	return notifier_ops->notify_send(fdev, state->peer_notif_id, false, 0);
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_notify_peer);

int virtio_msg_ffa_fifo_enqueue(struct virtio_msg_ffa_fifo_state *state,
				const struct virtio_msg *req,
				size_t req_len)
{
	struct virtio_msg_ffa_fifo_ring *ring;
	u16 read, write;
	u8 *entry;
	int ret;

	if (!state || !state->configured || !req)
		return -EINVAL;

	ret = virtio_msg_ffa_validate_inbound_msg(req, req_len);
	if (ret)
		return ret;

	ring = &state->tx;
	if (req_len > ring->message_size)
		return -EMSGSIZE;
	ret = virtio_msg_ffa_fifo_indices_get(ring, &read, &write);
	if (ret)
		return ret;
	if (((write + 1) % ring->depth) == read)
		return -ENOSPC;

	entry = ring->entries + ((size_t)write * ring->message_size);
	memset(entry, 0, ring->message_size);
	memcpy(entry, req, req_len);

	write = (write + 1) % ring->depth;
	virtio_msg_ffa_fifo_set_write_index(ring, write);

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_enqueue);

int virtio_msg_ffa_fifo_dequeue(struct virtio_msg_ffa_fifo_state *state,
				struct virtio_msg *resp,
				size_t resp_buf_len, size_t *resp_len,
				bool *was_full_before_read)
{
	struct virtio_msg_ffa_fifo_ring *ring;
	struct virtio_msg hdr;
	size_t entry_off;
	u16 read, write, msg_size;
	int ret;

	if (!state || !state->configured || !resp || !resp_len)
		return -EINVAL;

	ring = &state->rx;
	ret = virtio_msg_ffa_fifo_indices_get(ring, &read, &write);
	if (ret)
		return ret;
	if (read == write)
		return -ENOENT;
	if (was_full_before_read)
		*was_full_before_read = ((write + 1) % ring->depth) == read;

	entry_off = (size_t)read * ring->message_size;
	memcpy(&hdr, ring->entries + entry_off, sizeof(hdr));
	msg_size = le16_to_cpu(hdr.msg_size);

	if (msg_size < sizeof(hdr) || msg_size > ring->message_size ||
	    msg_size > FFA_BUS_MAX_MSG_SIZE) {
		read = (read + 1) % ring->depth;
		virtio_msg_ffa_fifo_set_read_index(ring, read);
		return -EPROTO;
	}
	if (msg_size > resp_buf_len)
		return -EMSGSIZE;

	memcpy(resp, ring->entries + entry_off, msg_size);
	ret = virtio_msg_ffa_validate_inbound_msg(resp, msg_size);
	read = (read + 1) % ring->depth;
	virtio_msg_ffa_fifo_set_read_index(ring, read);
	if (ret)
		return ret;

	*resp_len = msg_size;
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_dequeue);

int virtio_msg_ffa_fifo_drain_rx(struct virtio_msg_ffa_xfer_ctx *ctx,
				 struct virtio_msg_ffa_fifo_state *state,
				 virtio_msg_ffa_fifo_rx_cb_t cb,
				 void *cb_data, u32 *drained)
{
	u8 msg_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *msg = (struct virtio_msg *)msg_buf;
	u32 consumed = 0;
	bool was_full;
	size_t msg_len;
	int notify_ret;
	int ret;

	if (!ctx || !ctx->ep || !state || !cb)
		return -EINVAL;

	for (;;) {
		was_full = false;
		ret = virtio_msg_ffa_fifo_dequeue(state, msg, sizeof(msg_buf),
						  &msg_len, &was_full);
		if (ret == -ENOENT)
			break;
		if (ret) {
			if (was_full) {
				notify_ret = virtio_msg_ffa_fifo_notify_peer(ctx->fdev,
									     state);
				if (notify_ret)
					return notify_ret;
			}
			if (ret == -EPROTO)
				continue;
			return ret;
		}

		ret = cb(ctx->ep, msg, msg_len, cb_data);
		if (ret) {
			if (was_full) {
				notify_ret = virtio_msg_ffa_fifo_notify_peer(ctx->fdev,
									     state);
				if (notify_ret)
					return notify_ret;
			}
			return ret;
		}

		consumed++;
		if (was_full) {
			ret = virtio_msg_ffa_fifo_notify_peer(ctx->fdev, state);
			if (ret)
				return ret;
		}
	}

	if (drained)
		*drained = consumed;

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_ffa_fifo_drain_rx);
