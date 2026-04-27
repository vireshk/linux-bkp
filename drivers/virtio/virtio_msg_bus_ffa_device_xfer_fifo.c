// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A device-role FIFO transfer runtime.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/byteorder/little_endian.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "virtio_msg_bus_ffa_device_priv.h"
#include "virtio_msg_bus_ffa_xfer_fifo.h"

struct virtio_msg_ffa_device_fifo {
	struct virtio_msg_ffa_device *vdev;
	struct virtio_msg_ffa_fifo_state fifo;
	struct work_struct rx_work;
	struct delayed_work notify_work;
	struct completion tx_idle;
	void *region;
	size_t region_len;
	struct page **pages;
	u64 mem_handle;
	unsigned int tx_inflight;
	u16 page_count;
	u32 notify_delay_ms;
	bool notify_pending;
	bool mem_retrieved;
	bool quiescing;
};

static const struct virtio_msg_ffa_xfer_method_desc
	virtio_msg_ffa_fifo_device_method_desc;

static bool
virtio_msg_ffa_device_mem_ops_ready(const struct virtio_msg_ffa_device *vdev)
{
	return vdev && vdev->fdev && vdev->fdev->ops && vdev->fdev->ops->mem_ops &&
	       vdev->fdev->ops->mem_ops->memory_retrieve &&
	       vdev->fdev->ops->mem_ops->memory_relinquish;
}

static bool
virtio_msg_ffa_device_notifier_ops_ready(const struct virtio_msg_ffa_device *vdev)
{
	const struct ffa_notifier_ops *notifier_ops;

	if (!vdev || !vdev->fdev || !vdev->fdev->ops)
		return false;
	if (!ffa_partition_supports_notify_recv(vdev->fdev))
		return false;

	notifier_ops = vdev->fdev->ops->notifier_ops;
	return notifier_ops && notifier_ops->notify_alloc &&
	       notifier_ops->notify_relinquish && notifier_ops->notify_send;
}

bool virtio_msg_ffa_device_fifo_capable(const struct virtio_msg_ffa_device *vdev)
{
	return virtio_msg_ffa_device_mem_ops_ready(vdev) &&
	       virtio_msg_ffa_device_notifier_ops_ready(vdev);
}

static struct virtio_msg_ffa_device_fifo *
virtio_msg_ffa_device_fifo_get_locked(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_fifo *priv;

	if (!vdev)
		return NULL;

	lockdep_assert_held(&vdev->lock);

	priv = vdev->fifo;
	if (!priv || priv->vdev != vdev)
		return NULL;

	return priv;
}

static bool
virtio_msg_ffa_device_fifo_bootstrap_ready_locked
		(const struct virtio_msg_ffa_device *vdev)
{
	if (!vdev || !vdev->ep.negotiation_done)
		return false;

	return virtio_msg_ffa_device_bootstrap_ready_locked(vdev);
}

static void
virtio_msg_ffa_device_fifo_notify_schedule_locked
		(struct virtio_msg_ffa_device_fifo *priv)
{
	u32 delay_ms;

	if (!priv || !priv->fifo.configured)
		return;

	delay_ms = virtio_msg_ffa_fifo_notify_backoff_delay
			(&priv->notify_delay_ms);
	priv->notify_pending = true;
	schedule_delayed_work(&priv->notify_work, msecs_to_jiffies(delay_ms));
}

static int
virtio_msg_ffa_device_fifo_tx_begin_locked
		(struct virtio_msg_ffa_device_fifo *priv)
{
	if (!priv)
		return -EOPNOTSUPP;

	lockdep_assert_held(&priv->vdev->lock);

	if (!priv->fifo.configured)
		return -EOPNOTSUPP;
	if (priv->quiescing)
		return -ESHUTDOWN;

	if (!priv->tx_inflight)
		reinit_completion(&priv->tx_idle);
	priv->tx_inflight++;

	return 0;
}

static void
virtio_msg_ffa_device_fifo_tx_end_locked
		(struct virtio_msg_ffa_device_fifo *priv)
{
	if (!priv)
		return;

	lockdep_assert_held(&priv->vdev->lock);

	if (WARN_ON_ONCE(!priv->tx_inflight))
		return;

	priv->tx_inflight--;
	if (!priv->tx_inflight)
		complete_all(&priv->tx_idle);
}

int virtio_msg_ffa_device_fifo_send_locked(struct virtio_msg_ffa_device *vdev,
					   const struct virtio_msg *msg,
					   size_t msg_len)
{
	struct virtio_msg_ffa_device_fifo *priv;
	long remaining;
	u64 generation;
	u64 space_seq;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (!virtio_msg_ffa_device_fifo_bootstrap_ready_locked(vdev))
		return -EOPNOTSUPP;

	priv = virtio_msg_ffa_device_fifo_get_locked(vdev);
	if (!priv || !priv->fifo.configured)
		return -EOPNOTSUPP;
	ret = virtio_msg_ffa_device_fifo_tx_begin_locked(priv);
	if (ret)
		return ret;

	virtio_msg_ffa_device_trace_msg(vdev, "tx", "fifo", msg, msg_len);
	remaining = msecs_to_jiffies(VIRTIO_MSG_FFA_FIFO_TX_TIMEOUT_MS);

	for (;;) {
		space_seq = READ_ONCE(priv->fifo.space_seq);
		generation = READ_ONCE(priv->fifo.generation);
		ret = virtio_msg_ffa_fifo_enqueue(&priv->fifo, msg, msg_len);
		if (ret != -ENOSPC)
			break;

		mutex_unlock(&vdev->lock);
		ret = virtio_msg_ffa_fifo_wait_space(&priv->fifo, space_seq,
						     generation, &remaining);
		mutex_lock(&vdev->lock);
		if (ret)
			break;

		if (READ_ONCE(vdev->shutting_down)) {
			ret = -ESHUTDOWN;
			break;
		}
		if (!READ_ONCE(vdev->endpoint_registered)) {
			ret = -ENODEV;
			break;
		}
		if (virtio_msg_ffa_device_fifo_get_locked(vdev) != priv ||
		    !priv->fifo.configured ||
		    READ_ONCE(priv->fifo.generation) != generation) {
			ret = -ESHUTDOWN;
			break;
		}
	}
	if (ret) {
		if (vdev->fdev)
			dev_dbg(&vdev->fdev->dev,
				"bus tx fifo failed: dev=%u tok=%u ret=%d\n",
				le16_to_cpu(msg->dev_num),
				le16_to_cpu(msg->token), ret);
		else
			pr_debug("bus tx fifo failed: dev=%u tok=%u ret=%d\n",
				 le16_to_cpu(msg->dev_num),
				 le16_to_cpu(msg->token), ret);
		goto out_tx;
	}

	ret = virtio_msg_ffa_fifo_notify_peer(vdev->fdev, &priv->fifo);
	if (ret)
		virtio_msg_ffa_device_fifo_notify_schedule_locked(priv);
	ret = 0;

out_tx:
	virtio_msg_ffa_device_fifo_tx_end_locked(priv);
	return ret;
}

static int
virtio_msg_ffa_device_fifo_pages_from_sg(struct scatterlist *sgl, u32 nents,
					 u16 ffa_page_count,
					 struct page ***pages_out,
					 u16 *kernel_page_count_out)
{
	struct page **pages;
	struct sg_page_iter piter;
	struct scatterlist *sg;
	u64 expected_len;
	u64 total_len = 0;
	u64 kernel_pages = 0;
	u32 idx = 0;
	u16 kernel_page_count;
	int i;

	if (!sgl || !nents || !ffa_page_count || !pages_out ||
	    !kernel_page_count_out)
		return -EINVAL;

	expected_len = (u64)ffa_page_count * FFA_PAGE_SIZE;
	for_each_sg(sgl, sg, nents, i) {
		u64 sg_pages;

		if (!sg->length || sg->offset ||
		    !IS_ALIGNED(sg->length, FFA_PAGE_SIZE))
			return -EPROTO;
		if (check_add_overflow(total_len, (u64)sg->length,
				       &total_len))
			return -EOVERFLOW;
		sg_pages = DIV_ROUND_UP(sg->length, PAGE_SIZE);
		if (check_add_overflow(kernel_pages, sg_pages, &kernel_pages))
			return -EOVERFLOW;
	}

	if (total_len != expected_len)
		return -EPROTO;
	if (!kernel_pages || kernel_pages > U16_MAX)
		return -EOVERFLOW;

	kernel_page_count = kernel_pages;
	pages = kcalloc(kernel_page_count, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	for_each_sg_page(sgl, &piter, nents, 0) {
		struct page *page = sg_page_iter_page(&piter);

		if (!page) {
			kfree(pages);
			return -EPROTO;
		}
		pages[idx++] = page;
	}
	if (idx != kernel_page_count) {
		kfree(pages);
		return -EPROTO;
	}

	*pages_out = pages;
	*kernel_page_count_out = kernel_page_count;
	return 0;
}

static int
virtio_msg_ffa_device_fifo_region_retrieve_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_fifo *priv,
		 u64 mem_handle, u16 page_count)
{
	struct ffa_mem_retrieve_args args = {
		.handle = mem_handle,
		.sender_id = vdev->peer_vm_id,
		.receiver_id = 0,
		.flags = FFA_MEM_RETRIEVE_TYPE_SHARE,
		.expected_page_count = page_count,
	};
	u16 kernel_page_count;
	struct page **pages;
	size_t region_len;
	void *region;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!virtio_msg_ffa_device_mem_ops_ready(vdev))
		return -EOPNOTSUPP;

	ret = virtio_msg_ffa_memory_retrieve(vdev->fdev, &args, NULL, NULL);
	if (ret)
		return ret;

	ret = virtio_msg_ffa_device_fifo_pages_from_sg(args.sg, args.nents,
						       page_count, &pages,
						       &kernel_page_count);
	if (ret)
		goto err_relinquish;

	region = vmap(pages, kernel_page_count, VM_MAP, PAGE_KERNEL);
	if (!region) {
		ret = -ENOMEM;
		kfree(pages);
		goto err_relinquish;
	}

	region_len = (size_t)page_count * FFA_PAGE_SIZE;
	priv->pages = pages;
	priv->page_count = kernel_page_count;
	priv->region = region;
	priv->region_len = region_len;
	priv->mem_handle = mem_handle;
	priv->mem_retrieved = true;

	return 0;

err_relinquish:
	(void)vdev->fdev->ops->mem_ops->memory_relinquish(mem_handle, 0);
	return ret;
}

static int
virtio_msg_ffa_device_fifo_release_region_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_fifo *priv)
{
	int ret = 0;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !priv)
		return -EINVAL;

	if (priv->region) {
		vunmap(priv->region);
		priv->region = NULL;
	}

	kfree(priv->pages);
	priv->pages = NULL;
	priv->region_len = 0;
	priv->page_count = 0;

	if (priv->mem_retrieved && virtio_msg_ffa_device_mem_ops_ready(vdev)) {
		ret = vdev->fdev->ops->mem_ops->memory_relinquish
				(priv->mem_handle, 0);
		if (!ret) {
			priv->mem_retrieved = false;
			priv->mem_handle = 0;
		}
	}

	return ret;
}

static int
virtio_msg_ffa_device_fifo_cleanup_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_fifo *priv)
{
	struct virtio_msg_ffa_fifo_state unbind_fifo = { 0 };
	int release_ret;
	int ret = 0;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !priv)
		return -EINVAL;

	unbind_fifo.rx_notif_bound = priv->fifo.rx_notif_bound;
	unbind_fifo.local_notif_id = priv->fifo.local_notif_id;
	priv->fifo.configured = false;
	priv->fifo.rx_notif_bound = false;
	priv->fifo.local_notif_id = VIRTIO_MSG_FFA_NOTIF_ID_INVALID;
	priv->notify_pending = false;
	virtio_msg_ffa_fifo_notify_backoff_reset(&priv->notify_delay_ms);
	virtio_msg_ffa_fifo_space_wake(&priv->fifo);

	if (unbind_fifo.rx_notif_bound) {
		mutex_unlock(&vdev->lock);
		ret = virtio_msg_ffa_fifo_unbind_rx_notification(vdev->fdev,
								 &unbind_fifo);
		mutex_lock(&vdev->lock);
		if (ret && vdev->fdev) {
			dev_warn_ratelimited(&vdev->fdev->dev,
					     "FIFO RX notification unbind failed: %d\n",
					     ret);
		}
	}

	virtio_msg_ffa_fifo_reset(&priv->fifo);

	release_ret = virtio_msg_ffa_device_fifo_release_region_locked(vdev, priv);
	return ret ?: release_ret;
}

static int
virtio_msg_ffa_device_fifo_rx_cb(struct virtio_msg_ffa_endpoint *ep,
				 const struct virtio_msg *msg,
				 size_t msg_len, void *cb_data)
{
	struct virtio_msg_ffa_device *vdev = cb_data;

	(void)ep;
	if (!vdev)
		return -EINVAL;

	virtio_msg_ffa_device_trace_msg(vdev, "rx", "fifo", msg, msg_len);
	return virtio_msg_ffa_device_dispatch_inbound_locked(vdev, msg, msg_len);
}

static void virtio_msg_ffa_device_fifo_rx_notif_cb(int notify_id, void *cb_data)
{
	struct virtio_msg_ffa_device_fifo *priv = cb_data;

	(void)notify_id;
	if (!priv || !priv->vdev)
		return;

	virtio_msg_ffa_fifo_space_wake(&priv->fifo);
	schedule_work(&priv->rx_work);
}

static void virtio_msg_ffa_device_fifo_rx_work(struct work_struct *work)
{
	struct virtio_msg_ffa_device_fifo *priv;
	struct virtio_msg_ffa_device *vdev;
	struct virtio_msg_ffa_xfer_ctx xfer_ctx = { 0 };
	int ret;

	priv = container_of(work, struct virtio_msg_ffa_device_fifo, rx_work);
	vdev = priv->vdev;
	if (!vdev)
		return;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down) ||
	    !READ_ONCE(vdev->endpoint_registered) ||
	    virtio_msg_ffa_device_fifo_get_locked(vdev) != priv ||
	    !priv->fifo.configured) {
		mutex_unlock(&vdev->lock);
		return;
	}

	xfer_ctx.ep = &vdev->ep;
	xfer_ctx.fdev = vdev->fdev;
	xfer_ctx.priv = &priv->fifo;
	ret = virtio_msg_ffa_fifo_drain_rx(&xfer_ctx, &priv->fifo,
					   virtio_msg_ffa_device_fifo_rx_cb,
					   vdev, NULL);
	if (ret && vdev->fdev)
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "FIFO RX drain failed: %d\n", ret);
	mutex_unlock(&vdev->lock);
}

static void virtio_msg_ffa_device_fifo_notify_work(struct work_struct *work)
{
	struct virtio_msg_ffa_device_fifo *priv;
	struct virtio_msg_ffa_device *vdev;
	u32 next_delay;
	int ret;

	priv = container_of(to_delayed_work(work),
			    struct virtio_msg_ffa_device_fifo, notify_work);
	vdev = priv->vdev;
	if (!vdev)
		return;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down) || !priv->notify_pending ||
	    !READ_ONCE(vdev->endpoint_registered) ||
	    virtio_msg_ffa_device_fifo_get_locked(vdev) != priv ||
	    !priv->fifo.configured) {
		priv->notify_pending = false;
		virtio_msg_ffa_fifo_notify_backoff_reset
			(&priv->notify_delay_ms);
		mutex_unlock(&vdev->lock);
		return;
	}

	ret = virtio_msg_ffa_fifo_notify_peer(vdev->fdev, &priv->fifo);
	if (!ret) {
		priv->notify_pending = false;
		virtio_msg_ffa_fifo_notify_backoff_reset
			(&priv->notify_delay_ms);
		mutex_unlock(&vdev->lock);
		return;
	}

	if (!virtio_msg_ffa_fifo_notify_backoff_next
			(&priv->notify_delay_ms, &next_delay)) {
		priv->notify_pending = false;
		mutex_unlock(&vdev->lock);
		if (vdev->fdev) {
			dev_warn_ratelimited(&vdev->fdev->dev,
					     "FIFO peer notify retries exhausted: %d\n",
					     ret);
		}
		return;
	}
	mutex_unlock(&vdev->lock);

	schedule_delayed_work(&priv->notify_work, msecs_to_jiffies(next_delay));
}

static int
virtio_msg_ffa_device_fifo_alloc_locked(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_fifo *priv;

	lockdep_assert_held(&vdev->lock);

	if (!vdev)
		return -EINVAL;
	if (vdev->fifo)
		return 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->vdev = vdev;
	virtio_msg_ffa_fifo_init(&priv->fifo);
	virtio_msg_ffa_fifo_reset(&priv->fifo);
	INIT_WORK(&priv->rx_work, virtio_msg_ffa_device_fifo_rx_work);
	INIT_DELAYED_WORK(&priv->notify_work,
			  virtio_msg_ffa_device_fifo_notify_work);
	init_completion(&priv->tx_idle);
	complete_all(&priv->tx_idle);
	virtio_msg_ffa_fifo_notify_backoff_reset(&priv->notify_delay_ms);
	vdev->fifo = priv;

	return 0;
}

static int
virtio_msg_ffa_device_fifo_configure_locked(struct virtio_msg_ffa_device *vdev,
					    u64 mem_handle, u16 page_count,
					    u16 driver_notif_id,
					    u16 *device_notif_id)
{
	struct virtio_msg_ffa_device_fifo *priv;
	struct device *dev;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !page_count)
		return -EINVAL;

	dev = vdev->fdev ? &vdev->fdev->dev : NULL;
	if (!(virtio_msg_ffa_xfer_methods(VIRTIO_MSG_FFA_XFER_ROLE_DEVICE,
					  vdev->ep.bus_features, true) &
	      VIRTIO_MSG_FFA_XFER_METHOD_MASK(VIRTIO_MSG_FFA_XFER_FIFO))) {
		if (dev)
			dev_warn_ratelimited(dev,
					     "fifo-configure: FIFO not negotiated handle=%#llx pages=%u driver_notif=%u features=%#x\n",
					     (unsigned long long)mem_handle,
					     page_count, driver_notif_id,
					     vdev->ep.bus_features);
		else
			pr_warn_ratelimited("fifo-configure: FIFO not negotiated handle=%#llx pages=%u driver_notif=%u features=%#x\n",
					    (unsigned long long)mem_handle,
					    page_count, driver_notif_id,
					    vdev->ep.bus_features);
		return -EOPNOTSUPP;
	}
	if (!virtio_msg_ffa_device_fifo_bootstrap_ready_locked(vdev)) {
		if (dev)
			dev_warn_ratelimited(dev,
					     "fifo-configure: bootstrap method not ready handle=%#llx pages=%u driver_notif=%u method=%u\n",
					     (unsigned long long)mem_handle,
					     page_count, driver_notif_id,
					     vdev->ep.transfer_method);
		else
			pr_warn_ratelimited("fifo-configure: bootstrap method not ready handle=%#llx pages=%u driver_notif=%u method=%u\n",
					    (unsigned long long)mem_handle,
					    page_count, driver_notif_id,
					    vdev->ep.transfer_method);
		return -EOPNOTSUPP;
	}
	if (!virtio_msg_ffa_device_fifo_capable(vdev)) {
		if (dev)
			dev_warn_ratelimited(dev,
					     "fifo-configure: FIFO not capable handle=%#llx pages=%u driver_notif=%u\n",
					     (unsigned long long)mem_handle,
					     page_count, driver_notif_id);
		else
			pr_warn_ratelimited("fifo-configure: FIFO not capable handle=%#llx pages=%u driver_notif=%u\n",
					    (unsigned long long)mem_handle,
					    page_count, driver_notif_id);
		return -EOPNOTSUPP;
	}
	dev = &vdev->fdev->dev;

	ret = virtio_msg_ffa_device_fifo_alloc_locked(vdev);
	if (ret)
		return ret;

	priv = virtio_msg_ffa_device_fifo_get_locked(vdev);
	if (!priv)
		return -EINVAL;
	if (priv->fifo.configured) {
		dev_warn_ratelimited(dev,
				     "fifo-configure: already configured handle=%#llx pages=%u driver_notif=%u local_notif=%u peer_notif=%u\n",
				     (unsigned long long)mem_handle, page_count,
				     driver_notif_id, priv->fifo.local_notif_id,
				     priv->fifo.peer_notif_id);
		return -EEXIST;
	}

	ret = virtio_msg_ffa_device_fifo_cleanup_locked(vdev, priv);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "fifo-configure: cleanup failed handle=%#llx pages=%u driver_notif=%u ret=%d\n",
				     (unsigned long long)mem_handle, page_count,
				     driver_notif_id, ret);
		return ret;
	}

	ret = virtio_msg_ffa_device_fifo_region_retrieve_locked
		(vdev, priv, mem_handle, page_count);
	if (ret) {
		if (ret == -EBUSY)
			dev_err(dev,
				"fifo-configure: MEM_RETRIEVE busy handle=%#llx pages=%u peer_vm=%u driver_notif=%u ret=%d\n",
				(unsigned long long)mem_handle, page_count,
				vdev->peer_vm_id, driver_notif_id, ret);
		else
			dev_err(dev,
				"fifo-configure: retrieve failed handle=%#llx pages=%u driver_notif=%u ret=%d\n",
				(unsigned long long)mem_handle, page_count,
				driver_notif_id, ret);
		goto err_cleanup;
	}

	ret = virtio_msg_ffa_fifo_configure_layout(&priv->fifo, priv->region,
						   priv->region_len,
						   mem_handle, page_count);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "fifo-configure: layout failed handle=%#llx pages=%u len=%zu driver_notif=%u ret=%d\n",
				     (unsigned long long)mem_handle, page_count,
				     priv->region_len, driver_notif_id, ret);
		goto err_cleanup;
	}

	/*
	 * Peer owns TX->RX orientation from its perspective; flip local ring
	 * roles so local enqueue targets the peer RX ring.
	 */
	swap(priv->fifo.tx, priv->fifo.rx);

	ret = virtio_msg_ffa_fifo_bind_rx_notification
		(vdev->fdev, &priv->fifo, virtio_msg_ffa_device_fifo_rx_notif_cb,
		 priv);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "fifo-configure: notification bind failed handle=%#llx pages=%u driver_notif=%u ret=%d\n",
				     (unsigned long long)mem_handle, page_count,
				     driver_notif_id, ret);
		goto err_cleanup;
	}

	ret = virtio_msg_ffa_fifo_set_peer_notif_id(&priv->fifo,
						    driver_notif_id);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "fifo-configure: peer notification invalid handle=%#llx pages=%u driver_notif=%u local_notif=%u ret=%d\n",
				     (unsigned long long)mem_handle, page_count,
				     driver_notif_id, priv->fifo.local_notif_id,
				     ret);
		goto err_cleanup;
	}

	priv->notify_pending = false;
	virtio_msg_ffa_fifo_notify_backoff_reset(&priv->notify_delay_ms);
	if (device_notif_id)
		*device_notif_id = priv->fifo.local_notif_id;
	dev_dbg(dev,
		"fifo-configure: complete handle=%#llx pages=%u driver_notif=%u device_notif=%u\n",
		(unsigned long long)mem_handle, page_count, driver_notif_id,
		priv->fifo.local_notif_id);

	return 0;

err_cleanup:
	(void)virtio_msg_ffa_device_fifo_cleanup_locked(vdev, priv);
	return ret;
}

int virtio_msg_ffa_device_fifo_cfg_locked(struct virtio_msg_ffa_device *vdev,
					  const struct virtio_msg *msg,
					  size_t msg_len, u16 *result,
					  u16 *device_notif_id)
{
	const struct virtio_msg_ffa_fifo_configure_req *req;
	struct virtio_msg_ffa_device_fifo *priv;
	u16 local_notif_id = 0;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg || !result || !device_notif_id)
		return -EINVAL;
	if (msg_len != sizeof(*msg) + sizeof(*req))
		return -EINVAL;

	priv = virtio_msg_ffa_device_fifo_get_locked(vdev);
	if (priv && priv->fifo.configured) {
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "fifo-configure: rejected while active local_notif=%u peer_notif=%u\n",
				     priv->fifo.local_notif_id,
				     priv->fifo.peer_notif_id);
		return -EEXIST;
	}

	req = (const struct virtio_msg_ffa_fifo_configure_req *)msg->payload;
	ret = virtio_msg_ffa_device_fifo_configure_locked
		(vdev, le64_to_cpu(req->mem_handle),
		 le16_to_cpu(req->page_count),
		 le16_to_cpu(req->driver_notif_id),
		 &local_notif_id);
	if (ret)
		*result = VIRTIO_MSG_FFA_BUS_ERROR;
	else
		*result = VIRTIO_MSG_FFA_BUS_SUCCESS;

	*device_notif_id = local_notif_id;
	return 0;
}

static int
virtio_msg_ffa_device_fifo_init_runtime
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg_ffa_xfer_method_desc *desc)
{
	struct virtio_msg_ffa_device_fifo *priv;
	int ret;

	if (!vdev || !desc || desc->method != VIRTIO_MSG_FFA_XFER_FIFO ||
	    !(desc->role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DEVICE) ||
	    !(desc->phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME) ||
	    (desc->phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP) ||
	    desc->event_delivery != VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO)
		return -EINVAL;
	if (!(virtio_msg_ffa_xfer_methods(VIRTIO_MSG_FFA_XFER_ROLE_DEVICE,
					  vdev->ep.bus_features, true) &
	      VIRTIO_MSG_FFA_XFER_METHOD_MASK(VIRTIO_MSG_FFA_XFER_FIFO)))
		return -EOPNOTSUPP;
	if (!virtio_msg_ffa_device_fifo_capable(vdev) ||
	    !virtio_msg_ffa_device_fifo_bootstrap_ready_locked(vdev))
		return -EOPNOTSUPP;

	priv = virtio_msg_ffa_device_fifo_get_locked(vdev);
	if (!priv || !priv->fifo.configured)
		return -EOPNOTSUPP;

	vdev->event_configured = false;
	ret = virtio_msg_ffa_device_runtime_select_locked(vdev, desc);
	if (ret)
		return ret;
	vdev->event_configured = true;
	if (vdev->ep.transfer_method == VIRTIO_MSG_FFA_XFER_FIFO &&
	    vdev->ep.event_delivery == VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO &&
	    vdev->event_configured && vdev->runtime_method &&
	    vdev->runtime_method->desc == desc)
		return 0;

	ret = -EIO;
	virtio_msg_ffa_device_runtime_clear_locked(vdev);
	(void)virtio_msg_ffa_device_fifo_cleanup_locked(vdev, priv);
	return ret;
}

int virtio_msg_ffa_device_fifo_event_locked(struct virtio_msg_ffa_device *vdev)
{
	lockdep_assert_held(&vdev->lock);

	return virtio_msg_ffa_device_fifo_init_runtime
			(vdev, &virtio_msg_ffa_fifo_device_method_desc);
}

void virtio_msg_ffa_device_fifo_reset_locked(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_fifo *priv;
	int ret;

	lockdep_assert_held(&vdev->lock);

	priv = virtio_msg_ffa_device_fifo_get_locked(vdev);
	if (!priv)
		return;

	ret = virtio_msg_ffa_device_fifo_cleanup_locked(vdev, priv);
	if (ret && vdev->fdev) {
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "device FIFO cleanup during reset failed: %d\n",
				     ret);
	}
}

void virtio_msg_ffa_device_fifo_runtime_quiesce(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_fifo *priv;

	if (!vdev)
		return;

	mutex_lock(&vdev->lock);
	priv = virtio_msg_ffa_device_fifo_get_locked(vdev);
	if (priv) {
		priv->quiescing = true;
		(void)virtio_msg_ffa_device_fifo_cleanup_locked(vdev, priv);
		vdev->fifo = NULL;
		/*
		 * Senders hold a FIFO lifetime reference while they may drop
		 * vdev->lock and sleep on fifo.space_waitq.
		 */
		while (priv->tx_inflight) {
			mutex_unlock(&vdev->lock);
			wait_for_completion(&priv->tx_idle);
			mutex_lock(&vdev->lock);
		}
	}
	mutex_unlock(&vdev->lock);
	if (!priv)
		return;

	cancel_work_sync(&priv->rx_work);
	cancel_delayed_work_sync(&priv->notify_work);
	kfree(priv);
}

static const struct virtio_msg_ffa_xfer_device_method_ops
virtio_msg_ffa_fifo_xfer_device_method_ops = {
	.init_runtime = virtio_msg_ffa_device_fifo_init_runtime,
};

static const struct virtio_msg_ffa_xfer_method_desc
virtio_msg_ffa_fifo_device_method_desc = {
	.method = VIRTIO_MSG_FFA_XFER_FIFO,
	.name = "fifo",
	.role_mask = VIRTIO_MSG_FFA_XFER_ROLE_DEVICE,
	.phase_flags = VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
	.bus_feature_mask = VIRTIO_MSG_FFA_BUS_FEATURE_FIFO,
	.event_delivery = VIRTIO_MSG_FFA_BUS_EVENT_DELIV_FIFO,
	.device_ops = &virtio_msg_ffa_fifo_xfer_device_method_ops,
};

static const struct virtio_msg_ffa_device_method_ops
virtio_msg_ffa_fifo_device_method_ops = {
	.capable = virtio_msg_ffa_device_fifo_capable,
	.init_runtime = virtio_msg_ffa_device_fifo_init_runtime,
	.reset_locked = virtio_msg_ffa_device_fifo_reset_locked,
	.quiesce = virtio_msg_ffa_device_fifo_runtime_quiesce,
	.send_locked = virtio_msg_ffa_device_fifo_send_locked,
	.event_configure_locked = virtio_msg_ffa_device_fifo_event_locked,
	.bus_configure_locked = virtio_msg_ffa_device_fifo_cfg_locked,
};

const struct virtio_msg_ffa_device_method virtio_msg_ffa_fifo_device_method = {
	.desc = &virtio_msg_ffa_fifo_device_method_desc,
	.ops = &virtio_msg_ffa_fifo_device_method_ops,
};
