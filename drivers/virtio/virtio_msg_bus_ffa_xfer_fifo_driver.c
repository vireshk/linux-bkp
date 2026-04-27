// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A driver-side FIFO transfer helpers.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/arm_ffa.h>
#include <linux/byteorder/little_endian.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "virtio_msg_bus_ffa_driver_priv.h"
#include "virtio_msg_bus_ffa_xfer_fifo.h"

#define VIRTIO_MSG_FFA_FIFO_MESSAGE_SIZE		128
#define VIRTIO_MSG_FFA_FIFO_DEPTH			30

struct virtio_msg_ffa_xfer_fifo_priv {
	struct virtio_msg_ffa_driver *drv;
	struct virtio_msg_ffa_fifo_state fifo;
	struct work_struct fifo_rx_work;
	struct delayed_work fifo_notify_work;
	bool fifo_work_inited;
	bool fifo_notify_pending;
	u32 fifo_notify_delay_ms;
	bool fifo_mem_shared;
	void *fifo_region;
	size_t fifo_region_len;
	struct page **fifo_pages;
	u16 fifo_page_count;
	u64 fifo_mem_handle;
};

static bool virtio_msg_ffa_fifo_mem_ops_ready(const struct virtio_msg_ffa_driver *drv)
{
	return drv && drv->fdev && drv->fdev->ops && drv->fdev->ops->mem_ops &&
	       drv->fdev->ops->mem_ops->memory_share &&
	       drv->fdev->ops->mem_ops->memory_reclaim;
}

static bool
virtio_msg_ffa_fifo_notifier_ops_ready(const struct virtio_msg_ffa_driver *drv)
{
	const struct ffa_notifier_ops *notifier_ops;

	if (!drv || !drv->fdev || !drv->fdev->ops)
		return false;
	if (!ffa_partition_supports_notify_recv(drv->fdev))
		return false;

	notifier_ops = drv->fdev->ops->notifier_ops;
	return notifier_ops && notifier_ops->notify_alloc &&
	       notifier_ops->notify_relinquish && notifier_ops->notify_send;
}

static bool
virtio_msg_ffa_fifo_driver_runtime_capable(struct virtio_msg_ffa_driver *drv)
{
	return virtio_msg_ffa_fifo_mem_ops_ready(drv) &&
	       virtio_msg_ffa_fifo_notifier_ops_ready(drv);
}

bool virtio_msg_ffa_fifo_available(struct virtio_msg_ffa_driver *drv)
{
	return virtio_msg_ffa_fifo_driver_runtime_capable(drv);
}

static struct virtio_msg_ffa_xfer_fifo_priv *
virtio_msg_ffa_fifo_priv_get(struct virtio_msg_ffa_driver *drv,
			     struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_fifo_priv *priv;

	if (!drv || !xfer || xfer->method != VIRTIO_MSG_FFA_XFER_FIFO ||
	    !xfer->priv)
		return NULL;

	priv = xfer->priv;
	if (priv->drv && priv->drv != drv)
		return NULL;

	return priv;
}

static void virtio_msg_ffa_fifo_region_release_locked
			(struct virtio_msg_ffa_driver *drv,
			 struct virtio_msg_ffa_xfer_fifo_priv *priv)
{
	unsigned int i;

	if (!drv || !priv)
		return;

	if (priv->fifo_mem_shared && virtio_msg_ffa_fifo_mem_ops_ready(drv))
		drv->fdev->ops->mem_ops->memory_reclaim(priv->fifo_mem_handle, 0);

	priv->fifo_mem_shared = false;
	priv->fifo_mem_handle = 0;

	if (priv->fifo_region) {
		vunmap(priv->fifo_region);
		priv->fifo_region = NULL;
	}

	if (priv->fifo_pages) {
		for (i = 0; i < priv->fifo_page_count; i++) {
			if (priv->fifo_pages[i])
				__free_page(priv->fifo_pages[i]);
		}
		kfree(priv->fifo_pages);
		priv->fifo_pages = NULL;
	}

	priv->fifo_region_len = 0;
	priv->fifo_page_count = 0;
}

static void virtio_msg_ffa_fifo_cleanup_locked
			(struct virtio_msg_ffa_driver *drv,
			 struct virtio_msg_ffa_xfer_fifo_priv *priv)
{
	struct virtio_msg_ffa_fifo_state unbind_fifo = { 0 };
	int ret;

	if (!drv || !priv)
		return;

	lockdep_assert_held(&drv->lock);

	unbind_fifo.rx_notif_bound = priv->fifo.rx_notif_bound;
	unbind_fifo.local_notif_id = priv->fifo.local_notif_id;
	priv->fifo.configured = false;
	priv->fifo.rx_notif_bound = false;
	priv->fifo.local_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	priv->fifo_notify_pending = false;
	virtio_msg_ffa_fifo_notify_backoff_reset(&priv->fifo_notify_delay_ms);
	virtio_msg_ffa_fifo_space_wake(&priv->fifo);

	if (unbind_fifo.rx_notif_bound) {
		mutex_unlock(&drv->lock);
		ret = virtio_msg_ffa_fifo_unbind_rx_notification(drv->fdev,
								 &unbind_fifo);
		mutex_lock(&drv->lock);
		if (ret && drv->fdev)
			dev_warn_ratelimited(&drv->fdev->dev,
					     "FIFO RX notification unbind failed: %d\n",
					     ret);
	}

	virtio_msg_ffa_fifo_reset(&priv->fifo);
	virtio_msg_ffa_fifo_region_release_locked(drv, priv);
}

static void virtio_msg_ffa_fifo_notify_schedule_locked
			(struct virtio_msg_ffa_driver *drv,
			 struct virtio_msg_ffa_xfer_fifo_priv *priv)
{
	u32 delay_ms;

	if (!drv || !priv)
		return;
	if (!priv->fifo.configured)
		return;

	delay_ms = virtio_msg_ffa_fifo_notify_backoff_delay
			(&priv->fifo_notify_delay_ms);
	priv->fifo_notify_pending = true;
	schedule_delayed_work(&priv->fifo_notify_work,
			      msecs_to_jiffies(delay_ms));
}

static int virtio_msg_ffa_fifo_region_init(struct virtio_msg_ffa_xfer_fifo_priv *priv)
{
	struct virtio_msg_ffa_fifo_hdr *tx_hdr;
	struct virtio_msg_ffa_fifo_hdr *rx_hdr;
	size_t ring_bytes;
	size_t total_bytes;
	size_t alloc_len;
	struct page **pages;
	void *region;
	u16 page_count;
	unsigned int i;
	int ret;

	if (!priv)
		return -EINVAL;

	if (check_mul_overflow((size_t)VIRTIO_MSG_FFA_FIFO_MESSAGE_SIZE,
			       (size_t)VIRTIO_MSG_FFA_FIFO_DEPTH, &ring_bytes))
		return -EOVERFLOW;
	if (check_add_overflow(ring_bytes,
			       (size_t)VIRTIO_MSG_FFA_FIFO_MSG_AREA_OFFSET,
			       &ring_bytes))
		return -EOVERFLOW;
	if (check_mul_overflow(ring_bytes, (size_t)2, &total_bytes))
		return -EOVERFLOW;

	page_count = DIV_ROUND_UP(total_bytes, PAGE_SIZE);
	if (!page_count)
		return -EINVAL;
	if (page_count > U16_MAX)
		return -EOVERFLOW;

	pages = kcalloc(page_count, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	for (i = 0; i < page_count; i++) {
		pages[i] = alloc_page(GFP_KERNEL);
		if (!pages[i]) {
			ret = -ENOMEM;
			goto err_free_pages;
		}
	}

	region = vmap(pages, page_count, VM_MAP, PAGE_KERNEL);
	if (!region) {
		ret = -ENOMEM;
		goto err_free_pages;
	}

	alloc_len = (size_t)page_count * PAGE_SIZE;
	memset(region, 0, alloc_len);

	tx_hdr = region;
	rx_hdr = (struct virtio_msg_ffa_fifo_hdr *)((u8 *)region + ring_bytes);

	memcpy(tx_hdr->magic, VIRTIO_MSG_FFA_FIFO_MAGIC, sizeof(tx_hdr->magic));
	tx_hdr->version = cpu_to_le16(VIRTIO_MSG_FFA_FIFO_VERSION);
	tx_hdr->message_size = cpu_to_le16(VIRTIO_MSG_FFA_FIFO_MESSAGE_SIZE);
	tx_hdr->depth = cpu_to_le16(VIRTIO_MSG_FFA_FIFO_DEPTH);
	tx_hdr->next_offset = cpu_to_le32(ring_bytes);

	memcpy(rx_hdr->magic, VIRTIO_MSG_FFA_FIFO_MAGIC, sizeof(rx_hdr->magic));
	rx_hdr->version = cpu_to_le16(VIRTIO_MSG_FFA_FIFO_VERSION);
	rx_hdr->message_size = cpu_to_le16(VIRTIO_MSG_FFA_FIFO_MESSAGE_SIZE);
	rx_hdr->depth = cpu_to_le16(VIRTIO_MSG_FFA_FIFO_DEPTH);
	rx_hdr->next_offset = cpu_to_le32(0);

	priv->fifo_region = region;
	priv->fifo_region_len = total_bytes;
	priv->fifo_pages = pages;
	priv->fifo_page_count = page_count;
	return 0;

err_free_pages:
	for (i = 0; i < page_count; i++) {
		if (pages[i])
			__free_page(pages[i]);
	}
	kfree(pages);
	return ret;
}

static int virtio_msg_ffa_fifo_share_region(struct virtio_msg_ffa_driver *drv,
					    struct virtio_msg_ffa_xfer_fifo_priv *priv)
{
	struct ffa_mem_region_attributes receiver = { 0 };
	struct ffa_mem_ops_args args = { 0 };
	struct sg_table sgt = { 0 };
	size_t share_len;
	int ret;

	if (!drv || !priv)
		return -EINVAL;
	if (!virtio_msg_ffa_fifo_mem_ops_ready(drv))
		return -EOPNOTSUPP;
	if (!priv->fifo_pages || !priv->fifo_page_count || !priv->fifo_region_len)
		return -EINVAL;

	share_len = (size_t)priv->fifo_page_count * PAGE_SIZE;
	ret = sg_alloc_table_from_pages(&sgt, priv->fifo_pages,
					priv->fifo_page_count, 0, share_len,
					GFP_KERNEL);
	if (ret)
		return ret;

	receiver.receiver = drv->fdev->vm_id;
	receiver.attrs = FFA_MEM_RW;

	args.use_txbuf = true;
	args.nattrs = 1;
	args.tag = 0;
	args.sg = sgt.sgl;
	args.attrs = &receiver;

	ret = virtio_msg_ffa_memory_share(drv->fdev, &args);
	sg_free_table(&sgt);
	if (ret) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-share: FFA_MEM_SHARE failed page_count=%u ret=%d\n",
				     priv->fifo_page_count, ret);
		return ret;
	}

	priv->fifo_mem_handle = args.g_handle;
	priv->fifo_mem_shared = true;
	return 0;
}

static int virtio_msg_ffa_fifo_configure_send_locked(struct virtio_msg_ffa_driver *drv,
						     u64 mem_handle,
						     u16 page_count,
						     u16 notif_id,
						     u16 *peer_notif_id)
{
	struct virtio_msg_ffa_fifo_configure_req req = { 0 };
	struct virtio_msg_ffa_fifo_configure_resp *resp;
	u8 resp_msg_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *resp_msg = (struct virtio_msg *)resp_msg_buf;
	size_t resp_len;
	u16 result;
	int ret;

	req.mem_handle = cpu_to_le64(mem_handle);
	req.page_count = cpu_to_le16(page_count);
	req.driver_notif_id = cpu_to_le16(notif_id);

	memset(resp_msg_buf, 0, sizeof(resp_msg_buf));
	ret = virtio_msg_ffa_driver_send_bus_request_locked(drv,
							    FFA_BUS_MSG_FIFO_CONFIGURE,
							    &req, sizeof(req),
							    resp_msg,
							    sizeof(resp_msg_buf),
							    &resp_len);
	if (ret) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-configure: request failed handle=%#llx pages=%u local_notif=%u ret=%d\n",
				     mem_handle, page_count, notif_id, ret);
		return ret;
	}
	if (resp_len != sizeof(*resp_msg) + sizeof(*resp)) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-configure: bad response length=%zu expected=%zu\n",
				     resp_len, sizeof(*resp_msg) + sizeof(*resp));
		return -EPROTO;
	}

	resp = (struct virtio_msg_ffa_fifo_configure_resp *)resp_msg->payload;
	result = le16_to_cpu(resp->result);
	if (result == VIRTIO_MSG_FFA_BUS_ERROR) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-configure: peer error result=%u handle=%#llx pages=%u local_notif=%u device_notif=%u\n",
				     result, mem_handle, page_count, notif_id,
				     le16_to_cpu(resp->device_notif_id));
		return -EIO;
	}
	if (result != VIRTIO_MSG_FFA_BUS_SUCCESS) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-configure: peer reserved result=%u handle=%#llx pages=%u local_notif=%u device_notif=%u\n",
				     result, mem_handle, page_count, notif_id,
				     le16_to_cpu(resp->device_notif_id));
		return -EPROTO;
	}

	if (peer_notif_id)
		*peer_notif_id = le16_to_cpu(resp->device_notif_id);
	return 0;
}

static int virtio_msg_ffa_fifo_rx_cb(struct virtio_msg_ffa_endpoint *ep,
				     const struct virtio_msg *msg,
				     size_t msg_len, void *cb_data)
{
	struct virtio_msg_ffa_driver *drv = cb_data;

	(void)ep;
	if (!drv)
		return -EINVAL;

	virtio_msg_ffa_driver_trace_msg(drv, "rx", "fifo", msg, msg_len);
	return virtio_msg_ffa_driver_dispatch_inbound_locked(drv, msg, msg_len);
}

static void virtio_msg_ffa_fifo_rx_notif_cb(int notify_id, void *cb_data)
{
	struct virtio_msg_ffa_xfer_fifo_priv *priv = cb_data;

	(void)notify_id;
	if (!priv || !priv->fifo_work_inited)
		return;

	virtio_msg_ffa_fifo_space_wake(&priv->fifo);
	schedule_work(&priv->fifo_rx_work);
}

static void virtio_msg_ffa_fifo_rx_work(struct work_struct *work)
{
	struct virtio_msg_ffa_xfer_fifo_priv *priv =
		container_of(work, struct virtio_msg_ffa_xfer_fifo_priv,
			     fifo_rx_work);
	struct virtio_msg_ffa_driver *drv;
	struct virtio_msg_ffa_xfer_ctx ctx = { 0 };
	int ret;

	if (!priv)
		return;

	drv = priv->drv;
	if (!drv)
		return;

	mutex_lock(&drv->lock);
	if (priv->drv != drv || !priv->fifo.configured) {
		mutex_unlock(&drv->lock);
		return;
	}

	ctx.ep = &drv->ep;
	ctx.fdev = drv->fdev;
	ctx.priv = &priv->fifo;

	ret = virtio_msg_ffa_fifo_drain_rx(&ctx, &priv->fifo,
					   virtio_msg_ffa_fifo_rx_cb, drv,
					   NULL);
	if (ret && drv->fdev)
		dev_warn_ratelimited(&drv->fdev->dev,
				     "FIFO RX drain failed: %d\n", ret);

	mutex_unlock(&drv->lock);
}

static void virtio_msg_ffa_fifo_notify_work(struct work_struct *work)
{
	struct virtio_msg_ffa_xfer_fifo_priv *priv =
		container_of(to_delayed_work(work),
			     struct virtio_msg_ffa_xfer_fifo_priv,
			     fifo_notify_work);
	struct virtio_msg_ffa_driver *drv;
	u32 next_delay;
	int ret;

	if (!priv)
		return;

	drv = priv->drv;
	if (!drv)
		return;

	mutex_lock(&drv->lock);
	if (priv->drv != drv || !priv->fifo_notify_pending ||
	    !priv->fifo.configured) {
		priv->fifo_notify_pending = false;
		virtio_msg_ffa_fifo_notify_backoff_reset
			(&priv->fifo_notify_delay_ms);
		mutex_unlock(&drv->lock);
		return;
	}

	ret = virtio_msg_ffa_fifo_notify_peer(drv->fdev, &priv->fifo);
	if (!ret) {
		priv->fifo_notify_pending = false;
		virtio_msg_ffa_fifo_notify_backoff_reset
			(&priv->fifo_notify_delay_ms);
		mutex_unlock(&drv->lock);
		return;
	}

	if (!virtio_msg_ffa_fifo_notify_backoff_next
			(&priv->fifo_notify_delay_ms, &next_delay)) {
		virtio_msg_ffa_driver_handle_endpoint_failure_locked(drv, ret);
		mutex_unlock(&drv->lock);
		return;
	}
	mutex_unlock(&drv->lock);

	schedule_delayed_work(&priv->fifo_notify_work,
			      msecs_to_jiffies(next_delay));
}

static int virtio_msg_ffa_send_fifo_locked(struct virtio_msg_ffa_driver *drv,
					   struct virtio_msg_ffa_xfer_engine *xfer,
					   const struct virtio_msg *req,
					   size_t req_len,
					   struct virtio_msg *resp,
					   size_t resp_buf_len,
					   size_t *resp_len)
{
	struct virtio_msg_ffa_xfer_fifo_priv *priv;
	struct virtio_msg_ffa_fifo_state *state;
	long remaining;
	u64 generation;
	u64 space_seq;
	u16 dev_num;
	bool is_bus_msg;
	int ret;

	(void)resp;
	(void)resp_buf_len;
	(void)resp_len;
	if (!drv || !req)
		return -EINVAL;

	priv = virtio_msg_ffa_fifo_priv_get(drv, xfer);
	if (!priv)
		return -EINVAL;

	state = &priv->fifo;
	if (!state->configured)
		return -EOPNOTSUPP;

	dev_num = le16_to_cpu(req->dev_num);
	is_bus_msg = req->type & VIRTIO_MSG_TYPE_BUS;
	remaining = msecs_to_jiffies(VIRTIO_MSG_FFA_FIFO_TX_TIMEOUT_MS);

	for (;;) {
		space_seq = READ_ONCE(state->space_seq);
		generation = READ_ONCE(state->generation);
		ret = virtio_msg_ffa_fifo_enqueue(state, req, req_len);
		if (ret != -ENOSPC)
			break;

		mutex_unlock(&drv->lock);
		ret = virtio_msg_ffa_fifo_wait_space(state, space_seq,
						     generation, &remaining);
		mutex_lock(&drv->lock);
		if (ret)
			break;

		priv = virtio_msg_ffa_fifo_priv_get(drv, xfer);
		if (!priv) {
			ret = -ESHUTDOWN;
			break;
		}

		state = &priv->fifo;
		if (!state->configured ||
		    READ_ONCE(state->generation) != generation) {
			ret = -ESHUTDOWN;
			break;
		}
	}

	if (ret) {
		dev_dbg(&drv->fdev->dev,
			"bus tx fifo failed: msg_id=0x%02x dev=%u tok=%u ret=%d\n",
			req->msg_id, dev_num, le16_to_cpu(req->token), ret);
		if ((ret == -ENOSPC || ret == -ETIMEDOUT) && !is_bus_msg)
			virtio_msg_ffa_driver_handle_device_failure_locked(drv, dev_num);
		return ret;
	}

	ret = virtio_msg_ffa_fifo_notify_peer(drv->fdev, state);
	if (ret)
		virtio_msg_ffa_fifo_notify_schedule_locked(drv, priv);

	return 0;
}

static int virtio_msg_ffa_fifo_setup_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_fifo_priv *priv;
	u16 peer_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	size_t share_len;
	u16 ffa_page_count;
	u32 ffa_pages;
	int ret;

	if (!drv)
		return -EINVAL;
	if (!xfer->priv) {
		priv = kzalloc(sizeof(*priv), GFP_KERNEL);
		if (!priv)
			return -ENOMEM;
		virtio_msg_ffa_fifo_init(&priv->fifo);
		virtio_msg_ffa_fifo_reset(&priv->fifo);
		xfer->priv = priv;
	}
	priv = virtio_msg_ffa_fifo_priv_get(drv, xfer);
	if (!priv) {
		kfree(xfer->priv);
		xfer->priv = NULL;
		return -EINVAL;
	}
	priv->drv = drv;
	if (!virtio_msg_ffa_fifo_mem_ops_ready(drv))
		return -EOPNOTSUPP;
	if (!priv->fifo_work_inited) {
		INIT_WORK(&priv->fifo_rx_work, virtio_msg_ffa_fifo_rx_work);
		INIT_DELAYED_WORK(&priv->fifo_notify_work,
				  virtio_msg_ffa_fifo_notify_work);
		priv->fifo_work_inited = true;
	}
	if (priv->fifo.configured)
		return 0;

	virtio_msg_ffa_fifo_cleanup_locked(drv, priv);

	ret = virtio_msg_ffa_fifo_region_init(priv);
	if (ret) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-setup: region init failed ret=%d\n",
				     ret);
		goto err_cleanup;
	}

	ret = virtio_msg_ffa_fifo_share_region(drv, priv);
	if (ret) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-setup: share region failed pages=%u ret=%d\n",
				     priv->fifo_page_count, ret);
		goto err_cleanup;
	}

	share_len = (size_t)priv->fifo_page_count * PAGE_SIZE;
	ffa_pages = DIV_ROUND_UP(share_len, FFA_PAGE_SIZE);
	if (!ffa_pages || ffa_pages > U16_MAX) {
		ret = -EOVERFLOW;
		goto err_cleanup;
	}
	ffa_page_count = ffa_pages;

	ret = virtio_msg_ffa_fifo_configure_layout(&priv->fifo, priv->fifo_region,
						   priv->fifo_region_len,
						   priv->fifo_mem_handle,
						   ffa_page_count);
	if (ret) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-setup: layout configure failed handle=%#llx pages=%u len=%zu ret=%d\n",
				     priv->fifo_mem_handle, ffa_page_count,
				     priv->fifo_region_len, ret);
		goto err_cleanup;
	}

	ret = virtio_msg_ffa_fifo_bind_rx_notification
		(drv->fdev, &priv->fifo, virtio_msg_ffa_fifo_rx_notif_cb, priv);
	if (ret) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-setup: local RX notification bind failed ret=%d\n",
				     ret);
		goto err_cleanup;
	}

	ret = virtio_msg_ffa_fifo_configure_send_locked(drv, priv->fifo_mem_handle,
							ffa_page_count,
							priv->fifo.local_notif_id,
							&peer_notif_id);
	if (ret) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-setup: FIFO_CONFIGURE failed handle=%#llx pages=%u local_notif=%u ret=%d\n",
				     priv->fifo_mem_handle, priv->fifo.page_count,
				     priv->fifo.local_notif_id, ret);
		goto err_cleanup;
	}

	ret = virtio_msg_ffa_fifo_set_peer_notif_id(&priv->fifo, peer_notif_id);
	if (ret) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-setup: invalid peer notification id=%u valid_range=%u..%u ret=%d\n",
				     peer_notif_id, VIRTIO_MSG_FFA_NOTIF_ID_MIN,
				     VIRTIO_MSG_FFA_NOTIF_ID_MAX, ret);
		goto err_cleanup;
	}

	priv->fifo_notify_pending = false;
	virtio_msg_ffa_fifo_notify_backoff_reset(&priv->fifo_notify_delay_ms);
	return 0;

err_cleanup:
	virtio_msg_ffa_fifo_cleanup_locked(drv, priv);
	return ret;
}

static int virtio_msg_ffa_fifo_event_configure_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_fifo_priv *priv;
	int ret;

	priv = virtio_msg_ffa_fifo_priv_get(drv, xfer);
	if (!drv || !priv || !priv->fifo.configured)
		return -EINVAL;

	ret = virtio_msg_ffa_driver_event_configure_send_locked
		(drv, VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO,
		 0);
	if (ret) {
		dev_warn_ratelimited(&drv->fdev->dev,
				     "fifo-event-configure: local_notif=%u ret=%d\n",
				     priv->fifo.local_notif_id, ret);
		return ret;
	}

	virtio_msg_ffa_select_event_delivery(&drv->ep,
					     VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO);
	return 0;
}

static int virtio_msg_ffa_fifo_event_poll_once_unsupported
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer,
				 bool *empty)
{
	(void)drv;
	(void)xfer;
	(void)empty;
	return -EOPNOTSUPP;
}

static void virtio_msg_ffa_fifo_teardown_locked
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_fifo_priv *priv;

	priv = virtio_msg_ffa_fifo_priv_get(drv, xfer);
	if (!priv)
		return;

	virtio_msg_ffa_fifo_cleanup_locked(drv, priv);
}

static void virtio_msg_ffa_fifo_release_priv
				(struct virtio_msg_ffa_driver *drv,
				 struct virtio_msg_ffa_xfer_engine *xfer)
{
	struct virtio_msg_ffa_xfer_fifo_priv *priv;

	priv = virtio_msg_ffa_fifo_priv_get(drv, xfer);
	if (!priv)
		return;

	if (priv->fifo_work_inited) {
		cancel_work_sync(&priv->fifo_rx_work);
		cancel_delayed_work_sync(&priv->fifo_notify_work);
	}

	xfer->priv = NULL;
	kfree(priv);
}

const struct virtio_msg_ffa_xfer_ops virtio_msg_ffa_fifo_xfer_ops = {
	.setup_locked = virtio_msg_ffa_fifo_setup_locked,
	.teardown_locked = virtio_msg_ffa_fifo_teardown_locked,
	.submit_locked = virtio_msg_ffa_send_fifo_locked,
	.event_configure_locked = virtio_msg_ffa_fifo_event_configure_locked,
	.event_poll_once_locked = virtio_msg_ffa_fifo_event_poll_once_unsupported,
	.release_priv = virtio_msg_ffa_fifo_release_priv,
	.async_response = true,
};

static int
virtio_msg_ffa_fifo_driver_init_runtime
		(struct virtio_msg_ffa_driver *drv,
		 const struct virtio_msg_ffa_xfer_method_desc *desc)
{
	struct virtio_msg_ffa_xfer_engine xfer;
	void *old_priv;
	int ret;

	if (!drv || !desc || desc->method != VIRTIO_MSG_FFA_XFER_FIFO ||
	    !(desc->role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DRIVER) ||
	    !(desc->phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME) ||
	    (desc->phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP) ||
	    desc->event_delivery != VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO)
		return -EINVAL;

	if (!virtio_msg_ffa_fifo_driver_runtime_capable(drv))
		return -EOPNOTSUPP;
	if (drv->bootstrap_xfer.method == VIRTIO_MSG_FFA_XFER_NONE ||
	    !drv->bootstrap_xfer.ops)
		return -EOPNOTSUPP;

	old_priv = drv->xfer_priv[VIRTIO_MSG_FFA_XFER_FIFO];
	xfer.method = VIRTIO_MSG_FFA_XFER_FIFO;
	xfer.ops = &virtio_msg_ffa_fifo_xfer_ops;
	xfer.priv = old_priv;

	ret = virtio_msg_ffa_fifo_setup_locked(drv, &xfer);
	if (ret) {
		if (xfer.priv != old_priv)
			virtio_msg_ffa_fifo_release_priv(drv, &xfer);
		drv->xfer_priv[VIRTIO_MSG_FFA_XFER_FIFO] = old_priv;
		return virtio_msg_ffa_driver_init_runtime_reject(ret);
	}

	drv->xfer_priv[VIRTIO_MSG_FFA_XFER_FIFO] = xfer.priv;
	drv->event_configured = false;
	ret = virtio_msg_ffa_fifo_event_configure_locked(drv, &xfer);
	if (ret)
		goto err_teardown;

	drv->event_configured = true;
	drv->active_xfer = xfer;
	virtio_msg_ffa_select_transfer_method(&drv->ep,
					      VIRTIO_MSG_FFA_XFER_FIFO);
	if (drv->ep.transfer_method == VIRTIO_MSG_FFA_XFER_FIFO &&
	    virtio_msg_ffa_driver_event_runtime_ready_locked(drv))
		return 0;

	ret = -EIO;
	drv->event_configured = false;

err_teardown:
	virtio_msg_ffa_fifo_teardown_locked(drv, &xfer);
	if (xfer.priv != old_priv)
		virtio_msg_ffa_fifo_release_priv(drv, &xfer);
	drv->xfer_priv[VIRTIO_MSG_FFA_XFER_FIFO] = old_priv;
	return virtio_msg_ffa_driver_init_runtime_reject(ret);
}

static const struct virtio_msg_ffa_xfer_driver_method_ops
virtio_msg_ffa_fifo_driver_method_ops = {
	.init_runtime = virtio_msg_ffa_fifo_driver_init_runtime,
};

const struct virtio_msg_ffa_xfer_method_desc
virtio_msg_ffa_fifo_driver_method_desc = {
	.method = VIRTIO_MSG_FFA_XFER_FIFO,
	.name = "fifo",
	.role_mask = VIRTIO_MSG_FFA_XFER_ROLE_DRIVER,
	.phase_flags = VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
	.bus_feature_mask = VIRTIO_MSG_FFA_BUS_FEATURE_FIFO,
	.event_delivery = VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO,
	.driver_ops = &virtio_msg_ffa_fifo_driver_method_ops,
};
