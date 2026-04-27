// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A area publication helpers.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/arm_ffa.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/xarray.h>

#include "virtio_msg_bus_dma_driver_helper.h"
#include "virtio_msg_bus_ffa.h"
#include "virtio_msg_bus_ffa_driver_priv.h"
#include "virtio_msg_bus_ffa_protocol.h"

#define VIRTIO_MSG_FFA_AREA_ID_MIN	1U
#define VIRTIO_MSG_FFA_AREA_RETRY_DELAY_MS	1000

struct virtio_msg_ffa_area_export {
	struct ffa_device *fdev;
	struct virtio_msg_ffa_area_ctx *ctx;
	struct virtio_msg_bus_dma_driver_export export;
	struct delayed_work retry_work;
	struct mutex op_lock; /* Serializes per-area teardown transitions. */
	refcount_t refs;
	bool handle_dropped;
	bool retry_queued;
	bool completed;
	bool teardown_requested;
	bool area_released;
	bool reclaim_deferred;
	bool unshare_inflight;
	u64 g_handle;
	dma_addr_t bus_addr;
	u64 mem_tag;
	u32 page_count;
	u32 sharing_attrs;
	u16 area_id;
};

static DEFINE_IDA(virtio_msg_ffa_area_ida);
static DEFINE_XARRAY(virtio_msg_ffa_area_exports);
static DEFINE_MUTEX(virtio_msg_ffa_area_exports_lock);

static bool
virtio_msg_ffa_area_mem_ops_ready(const struct ffa_device *fdev)
{
	return fdev && fdev->ops && fdev->ops->mem_ops &&
	       fdev->ops->mem_ops->memory_share &&
	       fdev->ops->mem_ops->memory_reclaim;
}

static void
virtio_msg_ffa_area_share_dbg
	(struct virtio_msg_ffa_area_export *area_export, const char *stage,
	 int ret, u16 result, size_t resp_len)
{
	if (!area_export || !area_export->fdev)
		return;

	dev_dbg(&area_export->fdev->dev,
		"area-share: %s failed area=%u pages=%u attrs=%#x handle=%#llx tag=%#llx ret=%d result=%u resp_len=%zu\n",
		stage, area_export->area_id, area_export->page_count,
		area_export->sharing_attrs,
		(unsigned long long)area_export->g_handle,
		(unsigned long long)area_export->mem_tag, ret, result,
		resp_len);
}

static int
virtio_msg_ffa_area_build_sg_table(const struct virtio_msg_bus_dma_driver_export *export,
				   struct sg_table *sgt, gfp_t gfp)
{
	if (!export || !sgt)
		return -EINVAL;
	if (!export->pages || !export->npages || !export->mmap_length)
		return -EINVAL;

	return sg_alloc_table_from_pages(sgt, export->pages, export->npages,
					 0, export->mmap_length,
					 gfp);
}

static int
virtio_msg_ffa_area_protocol_page_count
			(const struct virtio_msg_bus_dma_driver_export *export,
			 u32 *page_count)
{
	u64 count;

	if (!export || !page_count || !export->mmap_length)
		return -EINVAL;

	count = DIV_ROUND_UP_ULL(export->mmap_length, FFA_PAGE_SIZE);
	if (!count || count > U32_MAX)
		return -EOVERFLOW;

	*page_count = count;
	return 0;
}

static void
virtio_msg_ffa_area_release_sg_table(const struct virtio_msg_bus_dma_driver_export *export,
				     struct sg_table *sgt)
{
	if (!export || !sgt)
		return;

	sg_free_table(sgt);
}

static void
virtio_msg_ffa_area_export_put(struct virtio_msg_ffa_area_export *area_export)
{
	if (!area_export)
		return;
	if (!refcount_dec_and_test(&area_export->refs))
		return;

	put_device(&area_export->fdev->dev);
	kfree(area_export);
}

static void
virtio_msg_ffa_area_export_get(struct virtio_msg_ffa_area_export *area_export)
{
	if (!area_export)
		return;

	refcount_inc(&area_export->refs);
}

static int
virtio_msg_ffa_area_export_track(struct virtio_msg_ffa_area_export *area_export)
{
	int ret;

	if (!area_export)
		return -EINVAL;

	mutex_lock(&virtio_msg_ffa_area_exports_lock);
	ret = xa_insert(&virtio_msg_ffa_area_exports, area_export->area_id,
			area_export, GFP_KERNEL);
	if (!ret)
		virtio_msg_ffa_area_export_get(area_export);
	mutex_unlock(&virtio_msg_ffa_area_exports_lock);
	return ret;
}

static void
virtio_msg_ffa_area_export_untrack(struct virtio_msg_ffa_area_export *area_export)
{
	bool untracked = false;

	if (!area_export)
		return;

	mutex_lock(&virtio_msg_ffa_area_exports_lock);
	if (xa_load(&virtio_msg_ffa_area_exports, area_export->area_id) ==
	    area_export) {
		xa_erase(&virtio_msg_ffa_area_exports, area_export->area_id);
		untracked = true;
	}
	mutex_unlock(&virtio_msg_ffa_area_exports_lock);

	if (untracked)
		virtio_msg_ffa_area_export_put(area_export);
}

static struct virtio_msg_ffa_area_export *
virtio_msg_ffa_area_export_lookup(u16 area_id)
{
	struct virtio_msg_ffa_area_export *area_export;

	mutex_lock(&virtio_msg_ffa_area_exports_lock);
	area_export = xa_load(&virtio_msg_ffa_area_exports, area_id);
	if (area_export)
		virtio_msg_ffa_area_export_get(area_export);
	mutex_unlock(&virtio_msg_ffa_area_exports_lock);
	return area_export;
}

static void
virtio_msg_ffa_area_queue_retry_locked(struct virtio_msg_ffa_area_export *area_export,
				       unsigned long delay_ms)
{
	if (area_export->retry_queued) {
		mod_delayed_work(system_wq, &area_export->retry_work,
				 msecs_to_jiffies(delay_ms));
		return;
	}

	area_export->retry_queued = true;
	virtio_msg_ffa_area_export_get(area_export);
	if (!schedule_delayed_work(&area_export->retry_work,
				   msecs_to_jiffies(delay_ms))) {
		area_export->retry_queued = false;
		virtio_msg_ffa_area_export_put(area_export);
	}
}

static int
virtio_msg_ffa_area_set_state(struct virtio_msg_ffa_area_export *area_export,
			      enum virtio_msg_ffa_area_state state)
{
	struct virtio_msg_ffa_area area;
	struct virtio_msg_ffa_area_ctx *ctx;
	int ret;

	if (!area_export)
		return -EINVAL;

	ctx = area_export->ctx;
	if (!ctx || !ctx->ep)
		return -ENODEV;

	ret = virtio_msg_ffa_area_lookup(ctx->ep, area_export->area_id, &area);
	if (ret) {
		memset(&area, 0, sizeof(area));
		area.area_id = area_export->area_id;
		area.mem_tag = area_export->mem_tag;
		area.page_count = area_export->page_count;
		area.sharing_attrs = area_export->sharing_attrs;
	}
	area.state = state;

	return virtio_msg_ffa_area_add(ctx->ep, &area);
}

static int
virtio_msg_ffa_area_registry_remove(struct virtio_msg_ffa_area_export *area_export)
{
	struct virtio_msg_ffa_area_ctx *ctx;
	int ret;

	if (!area_export)
		return -EINVAL;

	ctx = area_export->ctx;
	if (!ctx || !ctx->ep)
		return -ENODEV;

	ret = virtio_msg_ffa_area_remove(ctx->ep, area_export->area_id, NULL);
	if (ret == -ENOENT)
		return 0;
	return ret;
}

static int
virtio_msg_ffa_send_area_share(struct virtio_msg_ffa_area_export *area_export)
{
	struct virtio_msg_ffa_area_ctx *ctx;
	struct virtio_msg_ffa_area_share_req req = { 0 };
	struct virtio_msg_ffa_area_share_resp *resp;
	u8 resp_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *resp_msg = (struct virtio_msg *)resp_buf;
	size_t resp_len = 0;
	u16 result;
	int ret;

	if (!area_export)
		return -EINVAL;

	ctx = area_export->ctx;
	if (!ctx || !ctx->send_bus_req)
		return -ENODEV;

	req.area_id = cpu_to_le16(area_export->area_id);
	req.mem_handle = cpu_to_le64(area_export->g_handle);
	req.mem_tag = cpu_to_le64(area_export->mem_tag);
	req.page_count = cpu_to_le32(area_export->page_count);
	req.sharing_attrs = cpu_to_le32(area_export->sharing_attrs);

	memset(resp_buf, 0, sizeof(resp_buf));
	ret = ctx->send_bus_req(ctx, FFA_BUS_MSG_AREA_SHARE,
				&req, sizeof(req),
				resp_msg, sizeof(resp_buf),
				&resp_len);
	if (ret) {
		virtio_msg_ffa_area_share_dbg
			(area_export, "request", ret, 0, resp_len);
		return ret;
	}

	if (resp_len != sizeof(*resp_msg) + sizeof(*resp)) {
		virtio_msg_ffa_area_share_dbg
			(area_export, "response-length", -EPROTO, 0,
			 resp_len);
		return -EPROTO;
	}
	resp = (struct virtio_msg_ffa_area_share_resp *)resp_msg->payload;
	if (le16_to_cpu(resp->area_id) != area_export->area_id) {
		virtio_msg_ffa_area_share_dbg
			(area_export, "response-area-id", -EPROTO,
			 le16_to_cpu(resp->result), resp_len);
		return -EPROTO;
	}

	result = le16_to_cpu(resp->result);
	if (result == VIRTIO_MSG_FFA_BUS_SUCCESS)
		return 0;
	if (result == VIRTIO_MSG_FFA_BUS_BUSY) {
		virtio_msg_ffa_area_share_dbg
			(area_export, "peer-busy", -EBUSY, result,
			 resp_len);
		return -EBUSY;
	}
	virtio_msg_ffa_area_share_dbg
		(area_export, "peer-error", -EIO, result, resp_len);
	return -EIO;
}

static int
virtio_msg_ffa_send_area_unshare(struct virtio_msg_ffa_area_export *area_export)
{
	struct virtio_msg_ffa_area_ctx *ctx;
	struct virtio_msg_ffa_area_unshare_req req = { 0 };
	struct virtio_msg_ffa_area_unshare_resp *resp;
	u8 resp_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *resp_msg = (struct virtio_msg *)resp_buf;
	size_t resp_len;
	u16 result;
	int ret;

	if (!area_export)
		return -EINVAL;

	ctx = area_export->ctx;
	if (!ctx || !ctx->send_bus_req)
		return -ENODEV;

	req.area_id = cpu_to_le16(area_export->area_id);

	memset(resp_buf, 0, sizeof(resp_buf));
	ret = ctx->send_bus_req(ctx, FFA_BUS_MSG_AREA_UNSHARE,
				&req, sizeof(req),
				resp_msg, sizeof(resp_buf),
				&resp_len);
	if (ret)
		return ret;

	if (resp_len != sizeof(*resp_msg) + sizeof(*resp))
		return -EPROTO;
	resp = (struct virtio_msg_ffa_area_unshare_resp *)resp_msg->payload;
	if (le16_to_cpu(resp->area_id) != area_export->area_id)
		return -EPROTO;

	result = le16_to_cpu(resp->result);
	if (result == VIRTIO_MSG_FFA_BUS_SUCCESS)
		return 0;
	if (result == VIRTIO_MSG_FFA_BUS_BUSY)
		return -EBUSY;
	return -EIO;
}

static int
virtio_msg_ffa_area_memory_reclaim(struct virtio_msg_ffa_area_export *area_export)
{
	if (!area_export || !virtio_msg_ffa_area_mem_ops_ready(area_export->fdev))
		return -EOPNOTSUPP;

	return area_export->fdev->ops->mem_ops->memory_reclaim
			(area_export->g_handle, 0);
}

static bool
virtio_msg_ffa_area_remote_release_active
		(struct virtio_msg_ffa_area_export *area_export)
{
	struct virtio_msg_bus_dma_driver_release release;
	int ret;

	ret = virtio_msg_bus_dma_export_released(&area_export->export, &release);
	if (ret)
		return false;

	return release.action ==
	       VIRTIO_MSG_BUS_DMA_DRIVER_RELEASE_RETAINED_ACTIVE;
}

static int
virtio_msg_ffa_driver_area_local_teardown
		(struct virtio_msg_ffa_area_export *area_export)
{
	int ret;

	if (!area_export)
		return -EINVAL;

	lockdep_assert_held(&area_export->op_lock);

	ret = virtio_msg_ffa_area_memory_reclaim(area_export);
	if (ret)
		return ret;

	(void)virtio_msg_ffa_area_registry_remove(area_export);
	virtio_msg_ffa_area_export_untrack(area_export);
	ida_free(&virtio_msg_ffa_area_ida, area_export->area_id);

	area_export->teardown_requested = false;
	area_export->completed = true;
	area_export->reclaim_deferred = false;
	return 0;
}

static void
virtio_msg_ffa_area_share_failure_cleanup
		(struct virtio_msg_ffa_area_export *area_export)
{
	int ret;

	if (!area_export)
		return;

	mutex_lock(&area_export->op_lock);
	area_export->handle_dropped = true;
	area_export->area_released = true;
	area_export->teardown_requested = true;
	ret = virtio_msg_ffa_driver_area_local_teardown(area_export);
	if (ret) {
		(void)virtio_msg_ffa_area_set_state
				(area_export,
				 VIRTIO_MSG_FFA_AREA_UNSHARE_PENDING);
		virtio_msg_ffa_area_queue_retry_locked
				(area_export, VIRTIO_MSG_FFA_AREA_RETRY_DELAY_MS);
	}
	mutex_unlock(&area_export->op_lock);
}

static int
virtio_msg_ffa_area_try_complete(struct virtio_msg_ffa_area_export *area_export)
{
	bool send_unshare = false;
	int ret;

	if (!area_export)
		return -EINVAL;

	mutex_lock(&area_export->op_lock);
	if (area_export->completed || !area_export->teardown_requested) {
		mutex_unlock(&area_export->op_lock);
		return 0;
	}

	if (area_export->reclaim_deferred && !area_export->handle_dropped) {
		if (virtio_msg_ffa_area_remote_release_active(area_export)) {
			mutex_unlock(&area_export->op_lock);
			return -EAGAIN;
		}
		area_export->reclaim_deferred = false;
	}

	if (!area_export->area_released) {
		/*
		 * Avoid calling into topology/bus send paths while holding
		 * @op_lock (those take drv->lock). Mark the operation as
		 * inflight, drop @op_lock, and send AREA_UNSHARE.
		 */
		if (area_export->unshare_inflight) {
			mutex_unlock(&area_export->op_lock);
			return -EAGAIN;
		}

		area_export->unshare_inflight = true;
		send_unshare = true;
		mutex_unlock(&area_export->op_lock);
	}

	if (send_unshare) {
		ret = virtio_msg_ffa_send_area_unshare(area_export);

		mutex_lock(&area_export->op_lock);
		area_export->unshare_inflight = false;
		if (area_export->area_released || !ret) {
			area_export->area_released = true;
		} else if (ret == -EBUSY) {
			/*
			 * Keep retrying AREA_UNSHARE as a liveness/recovery
			 * measure in case the original teardown indication was
			 * lost or the peer needs another prompt.
			 */
			(void)virtio_msg_ffa_area_set_state
					(area_export,
					 VIRTIO_MSG_FFA_AREA_UNSHARE_PENDING);
			mutex_unlock(&area_export->op_lock);
			return -EAGAIN;
		} else if (ret) {
			mutex_unlock(&area_export->op_lock);
			return ret;
		}
	}

	ret = virtio_msg_ffa_driver_area_local_teardown(area_export);
	if (ret) {
		(void)virtio_msg_ffa_area_set_state
				(area_export,
				 VIRTIO_MSG_FFA_AREA_UNSHARE_PENDING);
		mutex_unlock(&area_export->op_lock);
		return -EAGAIN;
	}

	mutex_unlock(&area_export->op_lock);
	return 0;
}

static void
virtio_msg_ffa_area_retry_work(struct work_struct *work)
{
	struct virtio_msg_ffa_area_export *area_export =
		container_of(to_delayed_work(work),
			     struct virtio_msg_ffa_area_export, retry_work);
	int ret;

	mutex_lock(&area_export->op_lock);
	area_export->retry_queued = false;
	mutex_unlock(&area_export->op_lock);

	ret = virtio_msg_ffa_area_try_complete(area_export);
	if (ret == -EAGAIN) {
		mutex_lock(&area_export->op_lock);
		virtio_msg_ffa_area_queue_retry_locked
				(area_export, VIRTIO_MSG_FFA_AREA_RETRY_DELAY_MS);
		mutex_unlock(&area_export->op_lock);
	}
	virtio_msg_ffa_area_export_put(area_export);
}

static int
virtio_msg_ffa_area_export_add(void *ctx,
			       struct virtio_msg_bus_dma_driver_export *export,
			       void **provider_handle, gfp_t gfp)
{
	struct virtio_msg_ffa_area_export *area_export;
	struct virtio_msg_ffa_area_ctx *area_ctx = ctx;
	struct ffa_mem_region_attributes receiver = { 0 };
	struct ffa_mem_ops_args args = { 0 };
	struct sg_table local_sgt = { 0 };
	struct ffa_device *fdev = area_ctx ? area_ctx->fdev : NULL;
	u32 protocol_page_count;
	int area_id = -1;
	int ret;

	if (!area_ctx || !virtio_msg_ffa_area_mem_ops_ready(fdev) || !export ||
	    !provider_handle)
		return -EOPNOTSUPP;
	if (!gfpflags_allow_blocking(gfp))
		return -ENOMEM;

	area_export = kzalloc(sizeof(*area_export), gfp);
	if (!area_export)
		return -ENOMEM;

	get_device(&fdev->dev);
	area_export->fdev = fdev;
	area_export->ctx = area_ctx;
	refcount_set(&area_export->refs, 1);
	mutex_init(&area_export->op_lock);
	INIT_DELAYED_WORK(&area_export->retry_work, virtio_msg_ffa_area_retry_work);

	area_id = ida_alloc_range(&virtio_msg_ffa_area_ida,
				  VIRTIO_MSG_FFA_AREA_ID_MIN, U16_MAX, gfp);
	if (area_id < 0) {
		ret = area_id;
		goto err_put_export;
	}

	ret = virtio_msg_ffa_area_protocol_page_count(export,
						      &protocol_page_count);
	if (ret)
		goto err_free_area_id;

	ret = virtio_msg_ffa_area_build_sg_table(export, &local_sgt, gfp);
	if (ret)
		goto err_free_area_id;

	area_export->area_id = area_id;
	area_export->bus_addr =
		virtio_msg_ffa_bus_addr_pack(area_id, export->page_offset);
	area_export->export = *export;
	area_export->export.dma_addr = area_export->bus_addr;
	area_export->mem_tag = 0;
	area_export->page_count = protocol_page_count;
	area_export->sharing_attrs =
		VIRTIO_MSG_FFA_AREA_SHARING_ATTR_SHARE_RW_XN_INNER_WB_NORMAL_NS |
		VIRTIO_MSG_FFA_AREA_ATTR_DRIVER_RETENTION_REQUESTED;

	receiver.receiver = fdev->vm_id;
	receiver.attrs = FFA_MEM_RW;

	args.use_txbuf = true;
	args.nattrs = 1;
	args.tag = area_export->mem_tag;
	args.sg = local_sgt.sgl;
	args.attrs = &receiver;

	ret = virtio_msg_ffa_memory_share(fdev, &args);
	virtio_msg_ffa_area_release_sg_table(export, &local_sgt);
	if (ret) {
		dev_warn_ratelimited(&fdev->dev,
				     "area-export: FFA_MEM_SHARE failed area=%u npages=%u ret=%d\n",
				     area_id, export->npages, ret);
		goto err_free_area_id;
	}

	area_export->g_handle = args.g_handle;

	ret = virtio_msg_ffa_area_set_state(area_export,
					    VIRTIO_MSG_FFA_AREA_SHARED);
	if (ret)
		goto err_reclaim;

	ret = virtio_msg_ffa_area_export_track(area_export);
	if (ret)
		goto err_remove_area;

	ret = virtio_msg_ffa_send_area_share(area_export);
	if (ret)
		goto err_share_cleanup;

	export->dma_addr = area_export->bus_addr;

	*provider_handle = area_export;
	return 0;

err_share_cleanup:
	virtio_msg_ffa_area_share_failure_cleanup(area_export);
	virtio_msg_ffa_area_export_put(area_export);
	return ret;
err_remove_area:
	(void)virtio_msg_ffa_area_registry_remove(area_export);
err_reclaim:
	(void)virtio_msg_ffa_area_memory_reclaim(area_export);
err_free_area_id:
	if (area_id >= 0)
		ida_free(&virtio_msg_ffa_area_ida, area_id);
err_put_export:
	virtio_msg_ffa_area_export_put(area_export);
	return ret;
}

static int
virtio_msg_ffa_area_export_del(void *ctx,
			       const struct virtio_msg_bus_dma_driver_export *export,
			       void *provider_handle, bool sync)
{
	struct virtio_msg_ffa_area_export *area_export = provider_handle;
	bool drop_handle = false;
	int ret;

	(void)ctx;
	(void)export;

	if (!provider_handle)
		return 0;

	mutex_lock(&area_export->op_lock);
	if (!area_export->handle_dropped) {
		area_export->handle_dropped = true;
		drop_handle = true;
	}

	if (area_export->completed) {
		mutex_unlock(&area_export->op_lock);
		ret = 0;
		goto out_unlock;
	}

	area_export->teardown_requested = true;
	mutex_unlock(&area_export->op_lock);

	ret = virtio_msg_ffa_area_try_complete(area_export);
	if (ret == -EAGAIN) {
		mutex_lock(&area_export->op_lock);
		virtio_msg_ffa_area_queue_retry_locked
				(area_export, VIRTIO_MSG_FFA_AREA_RETRY_DELAY_MS);
		mutex_unlock(&area_export->op_lock);
		if (!sync)
			ret = 0;
	} else if (ret && !sync) {
		mutex_lock(&area_export->op_lock);
		virtio_msg_ffa_area_queue_retry_locked
				(area_export, VIRTIO_MSG_FFA_AREA_RETRY_DELAY_MS);
		mutex_unlock(&area_export->op_lock);
		ret = 0;
	}

out_unlock:
	if (drop_handle)
		virtio_msg_ffa_area_export_put(area_export);
	return ret;
}

void virtio_msg_ffa_area_endpoint_cleanup(struct virtio_msg_ffa_area_ctx *ctx)
{
	struct virtio_msg_ffa_area_export *area_export;
	struct ffa_device *fdev;
	unsigned long index;
	u16 *area_ids;
	size_t count = 0;
	size_t i = 0;

	if (!ctx || !ctx->fdev)
		return;

	fdev = ctx->fdev;
	if (!fdev)
		return;

	/*
	 * Snapshot matching AREA IDs first: completion paths take @op_lock and
	 * may untrack (which takes @virtio_msg_ffa_area_exports_lock), so avoid
	 * lock inversion by not holding the exports lock while queueing work.
	 */
	mutex_lock(&virtio_msg_ffa_area_exports_lock);
	xa_for_each(&virtio_msg_ffa_area_exports, index, area_export) {
		if (area_export && area_export->fdev == fdev)
			count++;
	}
	mutex_unlock(&virtio_msg_ffa_area_exports_lock);

	if (!count)
		return;

	area_ids = kcalloc(count, sizeof(*area_ids), GFP_KERNEL);
	if (!area_ids)
		return;

	mutex_lock(&virtio_msg_ffa_area_exports_lock);
	xa_for_each(&virtio_msg_ffa_area_exports, index, area_export) {
		if (!area_export || area_export->fdev != fdev)
			continue;
		if (i >= count)
			break;
		area_ids[i++] = area_export->area_id;
	}
	mutex_unlock(&virtio_msg_ffa_area_exports_lock);

	for (count = 0; count < i; count++) {
		area_export = virtio_msg_ffa_area_export_lookup(area_ids[count]);
		if (!area_export)
			continue;

		mutex_lock(&area_export->op_lock);
		if (!area_export->completed) {
			area_export->teardown_requested = true;
			virtio_msg_ffa_area_queue_retry_locked(area_export, 0);
		}
		mutex_unlock(&area_export->op_lock);

		virtio_msg_ffa_area_export_put(area_export);
	}

	kfree(area_ids);
}

int virtio_msg_ffa_area_handle_release(struct virtio_msg_ffa_area_ctx *ctx,
				       u16 area_id)
{
	struct virtio_msg_ffa_area_export *area_export;
	unsigned long retry_delay = 0;
	int ret;

	if (!ctx || !ctx->ep)
		return -EINVAL;

	ret = virtio_msg_ffa_area_mark_released(ctx->ep, area_id);
	if (ret && ret != -ENOENT)
		return ret;

	area_export = virtio_msg_ffa_area_export_lookup(area_id);
	if (!area_export)
		return 0;

	mutex_lock(&area_export->op_lock);
	if (!area_export->completed) {
		area_export->area_released = true;
		area_export->teardown_requested = true;
		if (virtio_msg_ffa_area_remote_release_active(area_export)) {
			area_export->reclaim_deferred = true;
			retry_delay = VIRTIO_MSG_FFA_AREA_RETRY_DELAY_MS;
		}
		virtio_msg_ffa_area_queue_retry_locked(area_export, retry_delay);
	}
	mutex_unlock(&area_export->op_lock);

	virtio_msg_ffa_area_export_put(area_export);
	return 0;
}

static int
virtio_msg_ffa_area_query_caps(void *ctx,
			       struct virtio_msg_bus_dma_driver_provider_caps *caps)
{
	if (!ctx || !caps)
		return -EINVAL;

	memset(caps, 0, sizeof(*caps));

	return 0;
}

static const struct virtio_msg_bus_dma_driver_provider_ops
virtio_msg_ffa_area_ops = {
	.export_add = virtio_msg_ffa_area_export_add,
	.export_del = virtio_msg_ffa_area_export_del,
	.query_caps = virtio_msg_ffa_area_query_caps,
};

const struct virtio_msg_bus_dma_driver_provider_ops *
virtio_msg_ffa_area_provider_ops_get(void)
{
	return &virtio_msg_ffa_area_ops;
}
