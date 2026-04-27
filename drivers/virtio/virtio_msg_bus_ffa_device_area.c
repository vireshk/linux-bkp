// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A device-role area lifecycle.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-ffa: " fmt

#include <linux/arm_ffa.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#include "virtio_msg_bus_ffa_device_priv.h"

#if IS_REACHABLE(CONFIG_VIRTIO_MSG_BRIDGE)

#define VIRTIO_MSG_FFA_DEVICE_TOPOLOGY_SCAN_DEVS	256U
#define VIRTIO_MSG_FFA_DEVICE_AREA_SHARE_TIMEOUT_MS	\
	VIRTIO_MSG_FFA_RELAY_TIMEOUT_MS

enum virtio_msg_ffa_device_area_bridge_state {
	VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_NONE = 0,
	VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ADD_DEFERRED,
	VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ADD_PENDING,
	VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ACTIVE,
	VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_REMOTE_RELEASED,
	VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_DEL_PENDING,
	VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_STALE_CLEANUP,
};

struct virtio_msg_ffa_device_area_share_waiter {
	struct completion done;
	u64 map_id;
	u16 area_id;
	int status;
};

struct virtio_msg_ffa_device_area {
	u16 area_id;
	u64 mem_handle;
	u64 map_id;
	u64 map_seq;
	u64 bus_addr;
	u64 mmap_offset;
	u64 length;
	void *region;
	struct page **pages;
	enum virtio_msg_ffa_device_area_bridge_state bridge_state;
	struct virtio_msg_ffa_device_area_share_waiter *share_waiter;
	u32 map_flags;
	u16 owner_dev_num;
	bool mem_retrieved;
	bool driver_retention_requested;
	bool owner_dev_num_valid;
	bool shared_by_multiple_devices;
	bool release_owed;
	bool unshare_pending_after_add;
	bool share_abandoned;
};

static bool
virtio_msg_ffa_device_mem_ops_ready(const struct virtio_msg_ffa_device *vdev)
{
	return vdev && vdev->fdev && vdev->fdev->ops && vdev->fdev->ops->mem_ops &&
	       vdev->fdev->ops->mem_ops->memory_retrieve &&
	       vdev->fdev->ops->mem_ops->memory_relinquish;
}

static u64 virtio_msg_ffa_device_area_mmap_offset(u16 area_id)
{
	return (u64)area_id << PAGE_SHIFT;
}

static bool virtio_msg_ffa_device_area_attrs_supported(u32 attrs)
{
	u32 type;
	u32 shareability;
	u32 cacheability;
	u32 mem_type;

	if (!attrs)
		return false;

	type = (attrs >> VIRTIO_MSG_FFA_AREA_ATTR_TYPE_SHIFT) & 0x3;
	if (type != VIRTIO_MSG_FFA_AREA_ATTR_TYPE_SHARE &&
	    type != VIRTIO_MSG_FFA_AREA_ATTR_TYPE_LEND)
		return false;
	if (!(attrs & VIRTIO_MSG_FFA_AREA_ATTR_WRITEABLE))
		return false;
	if (attrs & VIRTIO_MSG_FFA_AREA_ATTR_EXECUTABLE)
		return false;

	shareability = (attrs >> VIRTIO_MSG_FFA_AREA_ATTR_SHAREABILITY_SHIFT) & 0x3;
	if (shareability != VIRTIO_MSG_FFA_AREA_ATTR_SHAREABILITY_INNER &&
	    shareability != VIRTIO_MSG_FFA_AREA_ATTR_SHAREABILITY_OUTER)
		return false;

	cacheability = (attrs >> VIRTIO_MSG_FFA_AREA_ATTR_CACHEABILITY_SHIFT) & 0x3;
	if (cacheability != VIRTIO_MSG_FFA_AREA_ATTR_CACHEABILITY_WB)
		return false;

	mem_type = (attrs >> VIRTIO_MSG_FFA_AREA_ATTR_MEM_TYPE_SHIFT) & 0x3;
	if (mem_type != VIRTIO_MSG_FFA_AREA_ATTR_MEM_TYPE_NORMAL)
		return false;

	if (!(attrs & VIRTIO_MSG_FFA_AREA_ATTR_NON_SECURE))
		return false;
	if (attrs & GENMASK(31, 12))
		return false;

	return true;
}

static bool virtio_msg_ffa_device_area_bitmap_test(const u8 *bitmap, u32 bit)
{
	return bitmap[bit >> 3] & BIT(bit & 0x7);
}

static int
virtio_msg_ffa_device_area_identify_owner_locked
		(struct virtio_msg_ffa_device *vdev, u16 *dev_num)
{
	u8 bitmap[DIV_ROUND_UP(VIRTIO_MSG_FFA_DEVICE_TOPOLOGY_SCAN_DEVS, 8)];
	u16 next_offset;
	u16 out_num;
	u16 count;
	u32 offset = 0;
	u32 owners = 0;
	u16 owner = 0;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !dev_num)
		return -EINVAL;

	while (offset <= U16_MAX) {
		count = min_t(u32, VIRTIO_MSG_FFA_DEVICE_TOPOLOGY_SCAN_DEVS,
			      (u32)U16_MAX + 1U - offset);
		ret = virtio_msg_bus_bridge_topology_get_devices_window
			(&vdev->bridge, offset, count, bitmap, sizeof(bitmap),
			 &out_num, &next_offset);
		if (ret)
			return ret;

		while (out_num) {
			u32 bit = out_num - 1;

			if (virtio_msg_ffa_device_area_bitmap_test(bitmap, bit)) {
				owners++;
				owner = offset + bit;
				if (owners > 1)
					return -E2BIG;
			}
			out_num--;
		}

		if (!next_offset)
			break;
		offset = next_offset;
	}

	if (!owners)
		return -ENOENT;

	*dev_num = owner;
	return 0;
}

static void
virtio_msg_ffa_device_area_track_owner_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_area *area)
{
	u16 dev_num;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !area)
		return;

	ret = virtio_msg_ffa_device_area_identify_owner_locked(vdev, &dev_num);
	if (!ret) {
		area->owner_dev_num = dev_num;
		area->owner_dev_num_valid = true;
		area->shared_by_multiple_devices = false;
	} else if (ret == -E2BIG) {
		area->owner_dev_num_valid = false;
		area->shared_by_multiple_devices = true;
	}
}

static void
virtio_msg_ffa_device_area_share_dbg
	(struct virtio_msg_ffa_device *vdev, const char *stage, u16 area_id,
	 u32 page_count, u32 sharing_attrs, u64 mem_handle, u64 mem_tag,
	 int ret, u16 result)
{
	if (!vdev || !vdev->fdev)
		return;

	dev_dbg(&vdev->fdev->dev,
		"area-share: %s failed area=%u pages=%u attrs=%#x handle=%#llx tag=%#llx ret=%d result=%u\n",
		stage, area_id, page_count, sharing_attrs,
		(unsigned long long)mem_handle, (unsigned long long)mem_tag,
		ret, result);
}

static int
virtio_msg_ffa_device_area_notify_not_present_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_area *area,
		 bool *endpoint_remove)
{
	u16 dev_num;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !area || !endpoint_remove)
		return -EINVAL;

	if (area->owner_dev_num_valid) {
		dev_num = area->owner_dev_num;
		goto notify_device;
	}

	if (area->shared_by_multiple_devices) {
		*endpoint_remove = true;
		return 0;
	}

	ret = virtio_msg_ffa_device_area_identify_owner_locked(vdev, &dev_num);
	if (ret) {
		if (ret == -ENOENT || ret == -E2BIG) {
			*endpoint_remove = true;
			return 0;
		}
		return ret;
	}

	area->owner_dev_num = dev_num;
	area->owner_dev_num_valid = true;

notify_device:
	ret = virtio_msg_ffa_device_event_enqueue
		(vdev, dev_num, VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED);
	if (ret && vdev->fdev) {
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "device removal event failed dev=%u ret=%d\n",
				     dev_num, ret);
	}

	return 0;
}

static struct virtio_msg_ffa_device_area *
virtio_msg_ffa_device_area_lookup_id_locked(struct virtio_msg_ffa_device *vdev,
					    u16 area_id)
{
	lockdep_assert_held(&vdev->lock);

	return xa_load(&vdev->areas_by_id, area_id);
}

static int
virtio_msg_ffa_device_shared_area_lookup_locked(struct virtio_msg_ffa_device *vdev,
						u16 area_id,
						struct virtio_msg_ffa_area *out)
{
	lockdep_assert_held(&vdev->lock);

	if (!vdev || !out)
		return -EINVAL;

	return virtio_msg_ffa_area_lookup(&vdev->ep, area_id, out);
}

static int
virtio_msg_ffa_device_shared_area_set_state_locked(struct virtio_msg_ffa_device *vdev,
						   u16 area_id,
						   enum virtio_msg_ffa_area_state state)
{
	struct virtio_msg_ffa_area area;
	int ret;

	lockdep_assert_held(&vdev->lock);

	ret = virtio_msg_ffa_device_shared_area_lookup_locked(vdev, area_id, &area);
	if (ret)
		return ret;

	area.state = state;
	return virtio_msg_ffa_area_add(&vdev->ep, &area);
}

static void
virtio_msg_ffa_device_shared_area_remove_locked(struct virtio_msg_ffa_device *vdev,
						u16 area_id)
{
	lockdep_assert_held(&vdev->lock);

	if (!vdev)
		return;

	(void)virtio_msg_ffa_area_remove(&vdev->ep, area_id, NULL);
}

static void
virtio_msg_ffa_device_area_unmap_local(struct virtio_msg_ffa_device_area *area)
{
	if (!area)
		return;

	if (area->region) {
		vunmap(area->region);
		area->region = NULL;
	}
	kfree(area->pages);
	area->pages = NULL;
}

static void
virtio_msg_ffa_device_area_share_complete_locked
		(struct virtio_msg_ffa_device_area *area, int status)
{
	struct virtio_msg_ffa_device_area_share_waiter *waiter;

	if (!area || !area->share_waiter)
		return;

	waiter = area->share_waiter;
	area->share_waiter = NULL;
	waiter->status = status;
	complete(&waiter->done);
}

static bool
virtio_msg_ffa_device_area_share_waiter_matches_locked
		(struct virtio_msg_ffa_device_area *area,
		 const struct virtio_msg_ffa_device_area_share_waiter *waiter)
{
	if (!area || !waiter)
		return false;

	return area->share_waiter == waiter && area->area_id == waiter->area_id &&
	       area->map_id == waiter->map_id;
}

static void
virtio_msg_ffa_device_area_remove_locked(struct virtio_msg_ffa_device *vdev,
					 struct virtio_msg_ffa_device_area *area)
{
	lockdep_assert_held(&vdev->lock);

	if (!area)
		return;

	area->share_abandoned = true;
	virtio_msg_ffa_device_area_share_complete_locked(area, -ESHUTDOWN);
	if (xa_load(&vdev->areas_by_id, area->area_id) == area)
		xa_erase(&vdev->areas_by_id, area->area_id);
	virtio_msg_ffa_device_shared_area_remove_locked(vdev, area->area_id);
	area->map_id = 0;
	area->map_seq = 0;
	area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_NONE;
	virtio_msg_ffa_device_area_unmap_local(area);
	kfree(area);
}

static struct virtio_msg_ffa_device_area *
virtio_msg_ffa_device_area_from_map_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg_bus_dma_device_map_info *map)
{
	struct virtio_msg_ffa_device_area *area;

	lockdep_assert_held(&vdev->lock);

	if (!map || !map->cookie)
		return NULL;

	area = map->cookie;
	if (virtio_msg_ffa_device_area_lookup_id_locked(vdev, area->area_id) != area)
		return NULL;
	if (area->map_id != map->map_id ||
	    (area->map_seq && area->map_seq != map->map_seq) ||
	    area->bus_addr != map->bus_addr || area->mmap_offset != map->mmap_offset ||
	    area->length != map->length || area->length != map->mmap_length ||
	    area->map_flags != map->flags)
		return NULL;

	return area;
}

static struct virtio_msg_ffa_device_area *
virtio_msg_ffa_device_area_from_released_map_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg_bus_dma_device_map_info *map)
{
	struct virtio_msg_ffa_device_area *area;

	lockdep_assert_held(&vdev->lock);

	if (!map || !map->cookie)
		return NULL;

	area = map->cookie;
	if (virtio_msg_ffa_device_area_lookup_id_locked(vdev, area->area_id) != area)
		return NULL;
	if (area->map_id != map->map_id ||
	    (area->map_seq && area->map_seq != map->map_seq) ||
	    area->bus_addr != map->bus_addr || area->length != map->length)
		return NULL;

	return area;
}

static void
virtio_msg_ffa_device_area_purge_map
		(const struct virtio_msg_bus_dma_device_map_info *map, void *data)
{
	struct virtio_msg_ffa_device *vdev = data;
	struct virtio_msg_ffa_device_area *area;

	lockdep_assert_held(&vdev->lock);

	area = virtio_msg_ffa_device_area_from_map_locked(vdev, map);
	if (area) {
		area->share_abandoned = true;
		virtio_msg_ffa_device_area_share_complete_locked(area,
								 -ESHUTDOWN);
		area->map_id = 0;
		area->map_seq = 0;
		area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_NONE;
	}
}

static void
virtio_msg_ffa_device_area_purge_maps_locked(struct virtio_msg_ffa_device *vdev)
{
	lockdep_assert_held(&vdev->lock);

	virtio_msg_bus_dma_device_helper_purge(&vdev->area_maps,
					       virtio_msg_ffa_device_area_purge_map,
					       vdev);
}

static int
virtio_msg_ffa_device_area_pages_from_sg(struct scatterlist *sgl, u32 nents,
					 u64 expected_len,
					 struct page ***pages_out,
					 u32 *kernel_page_count_out)
{
	struct sg_page_iter piter;
	struct scatterlist *sg;
	struct page **pages;
	u64 total_len = 0;
	u64 kernel_pages = 0;
	u32 idx = 0;
	u32 kernel_page_count;
	int i;

	if (!sgl || !nents || !expected_len || !pages_out ||
	    !kernel_page_count_out)
		return -EINVAL;

	for_each_sg(sgl, sg, nents, i) {
		u64 sg_pages;

		if (!sg->length || !IS_ALIGNED(sg->length, FFA_PAGE_SIZE))
			return -EPROTO;
		if (sg->offset)
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
	if (!kernel_pages || kernel_pages > U32_MAX)
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
virtio_msg_ffa_device_area_retrieve_locked(struct virtio_msg_ffa_device *vdev,
					   struct virtio_msg_ffa_device_area *area,
					   const struct virtio_msg_ffa_area *shared_area)
{
	struct ffa_mem_retrieve_args args = {
		.handle = area->mem_handle,
		.tag = shared_area->mem_tag,
		.sender_id = vdev->peer_vm_id,
		.receiver_id = 0,
		.expected_page_count = shared_area->page_count,
	};
	u32 kernel_page_count;
	u32 type;
	void *region;
	int ret;

	lockdep_assert_held(&vdev->lock);
	if (!area || !shared_area)
		return -EINVAL;

	type = (shared_area->sharing_attrs >> VIRTIO_MSG_FFA_AREA_ATTR_TYPE_SHIFT) & 0x3;
	if (type == VIRTIO_MSG_FFA_AREA_ATTR_TYPE_LEND)
		args.flags = FFA_MEM_RETRIEVE_TYPE_LEND;
	else
		args.flags = FFA_MEM_RETRIEVE_TYPE_SHARE;

	ret = virtio_msg_ffa_memory_retrieve(vdev->fdev, &args, NULL, NULL);
	if (ret)
		return ret;

	ret = virtio_msg_ffa_device_area_pages_from_sg(args.sg, args.nents,
						       area->length,
						       &area->pages,
						       &kernel_page_count);
	if (ret)
		goto err_relinquish;

	region = vmap(area->pages, kernel_page_count, VM_MAP | VM_USERMAP,
		      PAGE_KERNEL);
	if (!region) {
		ret = -ENOMEM;
		goto err_relinquish;
	}

	area->region = region;
	area->mem_retrieved = true;
	return 0;

err_relinquish:
	virtio_msg_ffa_device_area_unmap_local(area);
	(void)vdev->fdev->ops->mem_ops->memory_relinquish(area->mem_handle, 0);
	return ret;
}

static int
virtio_msg_ffa_device_area_relinquish_locked(struct virtio_msg_ffa_device *vdev,
					     struct virtio_msg_ffa_device_area *area,
					     bool force_drop_local)
{
	int ret = 0;

	lockdep_assert_held(&vdev->lock);

	if (!area || !area->mem_retrieved) {
		virtio_msg_ffa_device_area_unmap_local(area);
		return 0;
	}

	ret = vdev->fdev->ops->mem_ops->memory_relinquish(area->mem_handle, 0);
	if (!ret || force_drop_local) {
		virtio_msg_ffa_device_area_unmap_local(area);
		area->mem_retrieved = false;
	}

	return ret;
}

static int virtio_msg_ffa_device_area_release_event_send_locked
		(struct virtio_msg_ffa_device *vdev, u16 area_id)
{
	struct virtio_msg_ffa_area_release_event event = {
		.area_id = cpu_to_le16(area_id),
	};
	u8 req_buf[sizeof(struct virtio_msg) + sizeof(event)];
	struct virtio_msg *req = (struct virtio_msg *)req_buf;
	u16 token;
	int ret;

	lockdep_assert_held(&vdev->lock);

	ret = virtio_msg_ffa_next_token_locked(&vdev->ep, &token);
	if (ret)
		return ret;

	memset(req_buf, 0, sizeof(req_buf));
	virtio_msg_prepare(req, FFA_BUS_EVENT_AREA_RELEASE,
			   token, sizeof(event), 0);
	req->type = VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_REQUEST;
	memcpy(req->payload, &event, sizeof(event));

	if (!vdev->event_configured) {
		/*
		 * Pre-event-configure teardown can only use the bootstrap
		 * method because the negotiated event path is not known.
		 */
		return virtio_msg_ffa_device_send_bootstrap_locked(vdev, req,
								   sizeof(req_buf));
	}

	return virtio_msg_ffa_device_send_selected_locked(vdev, req,
							 sizeof(req_buf));
}

static int
virtio_msg_ffa_device_area_local_teardown_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_area *area,
		 bool force_drop_local)
{
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !area)
		return -EINVAL;

	ret = virtio_msg_ffa_device_area_relinquish_locked(vdev, area,
							   force_drop_local);
	if (ret)
		return ret;

	ret = virtio_msg_ffa_device_shared_area_set_state_locked
			(vdev, area->area_id, VIRTIO_MSG_FFA_AREA_RELEASED);
	if (ret)
		return ret;

	ret = virtio_msg_ffa_device_area_release_event_send_locked
			(vdev, area->area_id);
	if (ret)
		return ret;

	area->release_owed = false;
	area->unshare_pending_after_add = false;
	area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_NONE;
	virtio_msg_ffa_device_area_remove_locked(vdev, area);
	return 0;
}

static int virtio_msg_ffa_device_area_finalize_deferred_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_area *area)
{
	struct virtio_msg_ffa_area shared_area;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!area || !area->release_owed)
		return 0;

	ret = virtio_msg_ffa_device_shared_area_lookup_locked
			(vdev, area->area_id, &shared_area);
	if (ret)
		return ret;

	if (shared_area.state != VIRTIO_MSG_FFA_AREA_UNSHARE_PENDING)
		return 0;

	return virtio_msg_ffa_device_area_local_teardown_locked(vdev, area,
							       false);
}

static void
virtio_msg_ffa_device_area_drop_failed_share_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_area *area, int status)
{
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !area)
		return;

	area->share_abandoned = true;
	area->release_owed = false;
	area->unshare_pending_after_add = false;
	area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_NONE;
	virtio_msg_ffa_device_area_share_complete_locked(area, status);

	ret = virtio_msg_ffa_device_area_relinquish_locked(vdev, area, true);
	if (ret && vdev->fdev) {
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "failed share relinquish failed area=%u ret=%d\n",
				     area->area_id, ret);
	}

	virtio_msg_ffa_device_area_remove_locked(vdev, area);
}

static int
virtio_msg_ffa_device_area_cleanup_abandoned_share_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_area *area)
{
	u32 handle;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !area)
		return -EINVAL;

	area->share_abandoned = true;
	area->release_owed = false;
	area->unshare_pending_after_add = false;

	if (!area->map_id)
		goto drop_now;

	handle = READ_ONCE(vdev->bridge.handle);
	if (!handle)
		goto purge_and_drop;

	ret = virtio_msg_bus_dma_device_helper_map_del_req
		(&vdev->area_maps, handle, area->map_id);
	if (ret == -EALREADY)
		ret = 0;
	if (!ret) {
		area->bridge_state =
			VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_DEL_PENDING;
		return 0;
	}
	if (ret == -ENOENT || ret == -ENODEV || ret == -ENOTCONN)
		goto purge_and_drop;

	area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_STALE_CLEANUP;
	return ret;

purge_and_drop:
	virtio_msg_ffa_device_area_purge_maps_locked(vdev);
	area->map_id = 0;
	area->map_seq = 0;
drop_now:
	virtio_msg_ffa_device_area_drop_failed_share_locked(vdev, area,
							    -ECANCELED);
	return 0;
}

static int
virtio_msg_ffa_device_area_publish_map_add_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_area *area,
		 struct virtio_msg_ffa_device_area_share_waiter *waiter)
{
	struct virtio_msg_bus_dma_device_map_desc map_desc;
	u64 map_id = 0;
	u32 handle;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !area)
		return -EINVAL;
	if (area->map_id ||
	    area->bridge_state == VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ACTIVE ||
	    area->bridge_state == VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_DEL_PENDING)
		return -EBUSY;

	handle = READ_ONCE(vdev->bridge.handle);
	if (!handle)
		return -ENOTCONN;

	memset(&map_desc, 0, sizeof(map_desc));
	map_desc.bus_addr = area->bus_addr;
	map_desc.length = area->length;
	map_desc.mmap_offset = area->mmap_offset;
	map_desc.mmap_length = area->length;
	map_desc.flags = area->map_flags;
	map_desc.cookie = area;

	area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ADD_PENDING;
	area->share_waiter = waiter;
	area->share_abandoned = false;
	ret = virtio_msg_bus_dma_device_helper_map_add(&vdev->area_maps,
						       handle, &map_desc,
						       &map_id);
	if (ret) {
		area->share_waiter = NULL;
		area->bridge_state =
			VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ADD_DEFERRED;
		return ret;
	}

	area->map_id = map_id;
	if (waiter)
		waiter->map_id = map_id;
	if (vdev->fdev)
		dev_dbg(&vdev->fdev->dev,
			"area-share map-add published: area=%u map_id=%#llx\n",
			area->area_id, (unsigned long long)map_id);
	else
		pr_debug("area-share map-add published: area=%u map_id=%#llx\n",
			 area->area_id, (unsigned long long)map_id);
	return 0;
}

static bool virtio_msg_ffa_device_area_publish_deferred_ret(int ret)
{
	return ret == -ENODEV || ret == -ENOTCONN || ret == -EAGAIN;
}

int virtio_msg_ffa_device_area_runtime_init(struct virtio_msg_ffa_device *vdev)
{
	if (!vdev)
		return -EINVAL;

	xa_init(&vdev->areas_by_id);
	virtio_msg_bus_dma_device_helper_init(&vdev->area_maps);
	return 0;
}

void virtio_msg_ffa_device_area_runtime_cleanup(struct virtio_msg_ffa_device *vdev)
{
	if (!vdev)
		return;

	virtio_msg_bus_dma_device_helper_destroy(&vdev->area_maps);
	xa_destroy(&vdev->areas_by_id);
}

void virtio_msg_ffa_device_area_runtime_reset_locked(struct virtio_msg_ffa_device *vdev,
						     bool emit_deferred_release)
{
	struct virtio_msg_ffa_device_area *area;
	struct virtio_msg_ffa_area shared_area;
	unsigned long area_id = 0;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev)
		return;

	virtio_msg_ffa_device_area_purge_maps_locked(vdev);

	for (;;) {
		area = xa_find(&vdev->areas_by_id, &area_id, ULONG_MAX, XA_PRESENT);
		if (!area)
			break;

		ret = virtio_msg_ffa_device_shared_area_lookup_locked(vdev, area->area_id,
								      &shared_area);
		if (ret || shared_area.state != VIRTIO_MSG_FFA_AREA_RELEASED) {
			ret = virtio_msg_ffa_device_area_relinquish_locked
				(vdev, area, true);
			if (ret && vdev->fdev) {
				dev_warn_ratelimited(&vdev->fdev->dev,
						     "area teardown relinquish failed area=%u ret=%d\n",
						     area->area_id, ret);
			}
		} else {
			virtio_msg_ffa_device_area_unmap_local(area);
		}

		if (emit_deferred_release && area->release_owed) {
			ret = virtio_msg_ffa_device_area_release_event_send_locked
				(vdev, area->area_id);
			if (ret && vdev->fdev) {
				dev_warn_ratelimited(&vdev->fdev->dev,
						     "area teardown release event failed area=%u ret=%d\n",
						     area->area_id, ret);
			}
		}

		virtio_msg_ffa_device_area_remove_locked(vdev, area);
	}
}

static int
virtio_msg_ffa_device_area_share_queue_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg_ffa_area_share_req *req,
		 struct virtio_msg_ffa_device_area_share_waiter *waiter,
		 u16 *result, bool *wait_for_map)
{
	struct virtio_msg_ffa_device_area *area;
	struct virtio_msg_ffa_area shared_area;
	u16 area_id;
	u32 page_count;
	u32 sharing_attrs;
	u64 area_len;
	u64 mem_handle;
	u64 mem_tag;
	const char *stage = "validate";
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !req || !waiter || !result || !wait_for_map)
		return -EINVAL;

	*result = VIRTIO_MSG_FFA_BUS_ERROR;
	*wait_for_map = false;
	area_id = le16_to_cpu(req->area_id);
	page_count = le32_to_cpu(req->page_count);
	sharing_attrs = le32_to_cpu(req->sharing_attrs);
	mem_handle = le64_to_cpu(req->mem_handle);
	mem_tag = le64_to_cpu(req->mem_tag);
	if (vdev->fdev)
		dev_dbg(&vdev->fdev->dev,
			"area-share queue: area=%u pages=%u attrs=%#x handle=%#llx tag=%#llx\n",
			area_id, page_count, sharing_attrs,
			(unsigned long long)mem_handle,
			(unsigned long long)mem_tag);
	else
		pr_debug("area-share queue: area=%u pages=%u attrs=%#x handle=%#llx tag=%#llx\n",
			 area_id, page_count, sharing_attrs,
			 (unsigned long long)mem_handle,
			 (unsigned long long)mem_tag);

	if (!virtio_msg_ffa_device_mem_ops_ready(vdev)) {
		ret = -EOPNOTSUPP;
		stage = "mem-ops";
		goto err_report;
	}

	if (!area_id || !page_count) {
		ret = -EINVAL;
		stage = "request";
		goto err_report;
	}
	if (!virtio_msg_ffa_device_area_attrs_supported(sharing_attrs)) {
		ret = -EINVAL;
		stage = "attrs";
		goto err_report;
	}
	if (virtio_msg_ffa_device_area_lookup_id_locked(vdev, area_id)) {
		ret = -EEXIST;
		stage = "area-id";
		goto err_report;
	}
	ret = virtio_msg_ffa_device_shared_area_lookup_locked
			(vdev, area_id, &shared_area);
	if (!ret) {
		ret = -EEXIST;
		stage = "shared-area-id";
		goto err_report;
	}
	if (ret != -ENOENT) {
		stage = "shared-area-lookup";
		goto err_report;
	}
	if (check_mul_overflow((u64)page_count, (u64)FFA_PAGE_SIZE,
			       &area_len)) {
		ret = -EOVERFLOW;
		stage = "length";
		goto err_report;
	}
	if (area_len - 1 > VIRTIO_MSG_FFA_BUS_ADDR_OFFSET_MASK) {
		ret = -EOVERFLOW;
		stage = "bus-address";
		goto err_report;
	}

	area = kzalloc(sizeof(*area), GFP_KERNEL);
	if (!area) {
		ret = -ENOMEM;
		stage = "alloc";
		goto err_report;
	}

	memset(&shared_area, 0, sizeof(shared_area));
	shared_area.area_id = area_id;
	shared_area.mem_tag = mem_tag;
	shared_area.page_count = page_count;
	shared_area.sharing_attrs = sharing_attrs;
	shared_area.state = VIRTIO_MSG_FFA_AREA_SHARED;

	area->area_id = area_id;
	area->mem_handle = mem_handle;
	area->bus_addr = virtio_msg_ffa_bus_addr_pack(area_id, 0);
	area->mmap_offset = virtio_msg_ffa_device_area_mmap_offset(area_id);
	area->length = area_len;
	area->driver_retention_requested =
		!!(sharing_attrs &
		   VIRTIO_MSG_FFA_AREA_ATTR_DRIVER_RETENTION_REQUESTED);
	if (area->driver_retention_requested)
		area->map_flags =
			VIRTIO_MSG_BUS_BRIDGE_MAP_F_RETENTION_REQUESTED;
	virtio_msg_ffa_device_area_track_owner_locked(vdev, area);

	stage = "retrieve";
	ret = virtio_msg_ffa_device_area_retrieve_locked(vdev, area, &shared_area);
	if (ret)
		goto err_free_area;

	stage = "area-add";
	ret = virtio_msg_ffa_area_add(&vdev->ep, &shared_area);
	if (ret)
		goto err_release_area;

	stage = "area-store";
	ret = xa_err(xa_store(&vdev->areas_by_id, area_id, area, GFP_KERNEL));
	if (ret)
		goto err_remove_shared;

	stage = "bridge-map-add";
	ret = virtio_msg_ffa_device_area_publish_map_add_locked(vdev, area,
								waiter);
	if (ret) {
		if (virtio_msg_ffa_device_area_publish_deferred_ret(ret)) {
			area->bridge_state =
				VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ADD_DEFERRED;
			if (vdev->fdev)
				dev_dbg(&vdev->fdev->dev,
					"area-share map-add deferred: area=%u ret=%d\n",
					area_id, ret);
			else
				pr_debug("area-share map-add deferred: area=%u ret=%d\n",
					 area_id, ret);
			*result = VIRTIO_MSG_FFA_BUS_SUCCESS;
			return 0;
		}
		goto err_remove_area;
	}

	*wait_for_map = true;
	return 0;

err_remove_area:
	virtio_msg_ffa_device_area_share_dbg
		(vdev, stage, area_id, page_count, sharing_attrs, mem_handle,
		 mem_tag, ret, *result);
	(void)virtio_msg_ffa_device_area_relinquish_locked(vdev, area, true);
	virtio_msg_ffa_device_area_remove_locked(vdev, area);
	return ret;
err_remove_shared:
	virtio_msg_ffa_device_area_share_dbg
		(vdev, stage, area_id, page_count, sharing_attrs, mem_handle,
		 mem_tag, ret, *result);
	virtio_msg_ffa_device_shared_area_remove_locked(vdev, area_id);
	(void)virtio_msg_ffa_device_area_relinquish_locked(vdev, area, true);
	kfree(area);
	return ret;
err_release_area:
	virtio_msg_ffa_device_area_share_dbg
		(vdev, stage, area_id, page_count, sharing_attrs, mem_handle,
		 mem_tag, ret, *result);
	(void)virtio_msg_ffa_device_area_relinquish_locked(vdev, area, true);
	kfree(area);
	return ret;
err_free_area:
	virtio_msg_ffa_device_area_share_dbg
		(vdev, stage, area_id, page_count, sharing_attrs, mem_handle,
		 mem_tag, ret, *result);
	kfree(area);
	return ret;
err_report:
	virtio_msg_ffa_device_area_share_dbg
		(vdev, stage, area_id, page_count, sharing_attrs, mem_handle,
		 mem_tag, ret, *result);
	return ret;
}

int virtio_msg_ffa_device_area_share(struct virtio_msg_ffa_device *vdev,
				     const struct virtio_msg_ffa_area_share_req *req,
				     u16 *result)
{
	struct virtio_msg_ffa_device_area_share_waiter waiter;
	struct virtio_msg_ffa_device_area *area;
	bool wait_for_map;
	long wait_ret;
	u16 area_id;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !req || !result)
		return -EINVAL;

	area_id = le16_to_cpu(req->area_id);
	memset(&waiter, 0, sizeof(waiter));
	init_completion(&waiter.done);
	waiter.area_id = area_id;
	waiter.status = -EINPROGRESS;

	ret = virtio_msg_ffa_device_area_share_queue_locked(vdev, req, &waiter,
							    result,
							    &wait_for_map);
	if (ret)
		return ret;
	if (!wait_for_map) {
		if (vdev->fdev)
			dev_dbg(&vdev->fdev->dev,
				"area-share response deferred: area=%u result=%u\n",
				area_id, *result);
		else
			pr_debug("area-share response deferred: area=%u result=%u\n",
				 area_id, *result);
		return 0;
	}

	mutex_unlock(&vdev->lock);
	wait_ret = wait_for_completion_interruptible_timeout
		(&waiter.done,
		 msecs_to_jiffies(VIRTIO_MSG_FFA_DEVICE_AREA_SHARE_TIMEOUT_MS));
	mutex_lock(&vdev->lock);

	if (wait_ret <= 0 && waiter.status == -EINPROGRESS) {
		ret = wait_ret ? (int)wait_ret : -ETIMEDOUT;
		area = virtio_msg_ffa_device_area_lookup_id_locked(vdev,
								   area_id);
		if (virtio_msg_ffa_device_area_share_waiter_matches_locked
				(area, &waiter)) {
			area->share_waiter = NULL;
			area->share_abandoned = true;
			(void)virtio_msg_ffa_device_area_cleanup_abandoned_share_locked
				(vdev, area);
		}
		if (vdev->fdev)
			dev_dbg(&vdev->fdev->dev,
				"area-share wait failed: area=%u map_id=%#llx ret=%d\n",
				area_id, (unsigned long long)waiter.map_id,
				ret);
		else
			pr_debug("area-share wait failed: area=%u map_id=%#llx ret=%d\n",
				 area_id, (unsigned long long)waiter.map_id,
				 ret);
		*result = VIRTIO_MSG_FFA_BUS_ERROR;
		return ret;
	}

	ret = waiter.status;
	area = virtio_msg_ffa_device_area_lookup_id_locked(vdev, area_id);
	if (!ret && area && !area->share_abandoned &&
	    area->map_id == waiter.map_id &&
	    area->bridge_state == VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ACTIVE) {
		*result = VIRTIO_MSG_FFA_BUS_SUCCESS;
	} else {
		if (!ret)
			ret = -ESTALE;
		*result = VIRTIO_MSG_FFA_BUS_ERROR;
	}

	if (vdev->fdev)
		dev_dbg(&vdev->fdev->dev,
			"area-share response: area=%u map_id=%#llx ret=%d result=%u\n",
			area_id, (unsigned long long)waiter.map_id, ret,
			*result);
	else
		pr_debug("area-share response: area=%u map_id=%#llx ret=%d result=%u\n",
			 area_id, (unsigned long long)waiter.map_id, ret,
			 *result);
	return ret;
}

int virtio_msg_ffa_device_area_unshare_locked(struct virtio_msg_ffa_device *vdev,
					      u16 area_id, u16 *result)
{
	struct virtio_msg_ffa_device_area *area;
	struct virtio_msg_ffa_area shared_area;
	u32 handle;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !result)
		return -EINVAL;

	*result = VIRTIO_MSG_FFA_BUS_ERROR;
	area = virtio_msg_ffa_device_area_lookup_id_locked(vdev, area_id);
	if (!area)
		return -ENOENT;

	ret = virtio_msg_ffa_device_shared_area_lookup_locked
			(vdev, area_id, &shared_area);
	if (ret)
		return ret;

	if (shared_area.state == VIRTIO_MSG_FFA_AREA_UNSHARE_PENDING &&
	    area->release_owed) {
		*result = VIRTIO_MSG_FFA_BUS_BUSY;
		return 0;
	}

	if (area->map_id) {
		if (area->bridge_state ==
		    VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ADD_PENDING) {
			ret = virtio_msg_ffa_device_shared_area_set_state_locked
				(vdev, area_id,
				 VIRTIO_MSG_FFA_AREA_UNSHARE_PENDING);
			if (ret)
				return ret;
			area->release_owed = true;
			area->unshare_pending_after_add = true;
			*result = VIRTIO_MSG_FFA_BUS_BUSY;
			return 0;
		}

		handle = READ_ONCE(vdev->bridge.handle);
		if (!handle) {
			virtio_msg_ffa_device_area_purge_maps_locked(vdev);
			goto relinquish_now;
		}

		ret = virtio_msg_bus_dma_device_helper_map_del_req
				(&vdev->area_maps, handle, area->map_id);
		if (ret == -EALREADY)
			ret = 0;
		if (ret == -ENODEV || ret == -ENOTCONN) {
			virtio_msg_ffa_device_area_purge_maps_locked(vdev);
			goto relinquish_now;
		}
		if (ret == -ENOENT &&
		    (area->bridge_state ==
			     VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_REMOTE_RELEASED ||
		     area->bridge_state ==
			     VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_STALE_CLEANUP)) {
			area->map_id = 0;
			area->map_seq = 0;
			goto relinquish_now;
		}
		if (ret)
			return ret;

		ret = virtio_msg_ffa_device_shared_area_set_state_locked
				(vdev, area_id,
				 VIRTIO_MSG_FFA_AREA_UNSHARE_PENDING);
		if (ret)
			return ret;
		area->release_owed = true;
		area->unshare_pending_after_add = false;
		area->bridge_state =
			VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_DEL_PENDING;
		*result = VIRTIO_MSG_FFA_BUS_BUSY;
		return 0;
	}

relinquish_now:
	ret = virtio_msg_ffa_device_area_relinquish_locked(vdev, area, false);
	if (ret)
		return ret;

	ret = virtio_msg_ffa_device_shared_area_set_state_locked
			(vdev, area_id, VIRTIO_MSG_FFA_AREA_RELEASED);
	if (ret)
		return ret;
	area->release_owed = false;
	area->unshare_pending_after_add = false;
	area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_NONE;
	virtio_msg_ffa_device_area_remove_locked(vdev, area);
	*result = VIRTIO_MSG_FFA_BUS_SUCCESS;
	return 0;
}

int virtio_msg_ffa_device_area_mmap_locked(struct virtio_msg_ffa_device *vdev,
					   u64 mmap_offset,
					   struct vm_area_struct *vma)
{
	struct virtio_msg_ffa_device_area *area;
	struct virtio_msg_bus_dma_device_map_info map;
	u64 mmap_len;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !vma || !mmap_offset || !PAGE_ALIGNED(mmap_offset)) {
		pr_debug("bridge mmap reject: vdev=%p vma=%p offset=%#llx\n",
			 vdev, vma, (unsigned long long)mmap_offset);
		return -EINVAL;
	}

	mmap_len = (u64)(vma->vm_end - vma->vm_start);
	if (!mmap_len) {
		pr_debug("bridge mmap reject: zero length offset=%#llx\n",
			 (unsigned long long)mmap_offset);
		return -EINVAL;
	}

	ret = virtio_msg_bus_dma_device_helper_lookup_by_mmap_offset
			(&vdev->area_maps, mmap_offset, mmap_len, &map);
	if (ret) {
		pr_debug("bridge mmap reject: offset=%#llx len=%#llx ret=%d\n",
			 (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len, ret);
		if (ret == -ERANGE)
			return -ENOENT;
		return ret;
	}

	area = virtio_msg_ffa_device_area_from_map_locked(vdev, &map);
	if (!area) {
		pr_debug("bridge mmap reject: stale map offset=%#llx len=%#llx\n",
			 (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len);
		return -ENOENT;
	}
	if (!area->region || mmap_len > area->length) {
		pr_debug("bridge mmap reject: offset=%#llx len=%#llx area_id=%u region=%p area_len=%#llx\n",
			 (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len, area->area_id,
			 area->region,
			 (unsigned long long)area->length);
		return -ENOENT;
	}

	ret = remap_vmalloc_range(vma, area->region, 0);
	if (ret) {
		pr_debug("bridge mmap remap failed: offset=%#llx len=%#llx area_id=%u region=%p ret=%d\n",
			 (unsigned long long)mmap_offset,
			 (unsigned long long)mmap_len, area->area_id,
			 area->region, ret);
	}

	return ret;
}

static int
virtio_msg_ffa_device_area_map_response_prepare
		(const struct vmsg_bridge_uapi_map_event *event, u32 ack_status,
		 u16 type, struct vmsg_bridge_uapi_map_event *response)
{
	if (!event || !response)
		return -EINVAL;
	if (ack_status != VMSG_BRIDGE_UAPI_MAP_ACK_OK &&
	    ack_status != VMSG_BRIDGE_UAPI_MAP_ACK_REJECT)
		return -EINVAL;
	if (event->type != type)
		return -EINVAL;

	*response = *event;
	if (ack_status == VMSG_BRIDGE_UAPI_MAP_ACK_REJECT &&
	    !response->status)
		response->status = -EIO;

	return 0;
}

static void
virtio_msg_ffa_device_area_map_release_forget
		(struct virtio_msg_ffa_device *vdev,
		 const struct vmsg_bridge_uapi_map_event *event)
{
	u32 handle;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !event || !event->map_id)
		return;

	handle = READ_ONCE(vdev->bridge.handle);
	if (!handle)
		return;

	ret = virtio_msg_bus_bridge_device_map_del(handle, event->map_id);
	if (ret && vdev->fdev) {
		if (ret == -EALREADY || ret == -ENOENT || ret == -ENODEV ||
		    ret == -ENOTCONN)
			return;
		dev_warn_ratelimited
			(&vdev->fdev->dev,
			 "released map cleanup failed map_id=%#llx ret=%d\n",
			 (unsigned long long)event->map_id, ret);
	}
}

static void
virtio_msg_ffa_device_area_release_after_failure_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_area *area)
{
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !area)
		return;

	ret = virtio_msg_ffa_device_area_local_teardown_locked(vdev, area,
							       false);
	if (ret && vdev->fdev) {
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "late map failure local teardown failed area=%u ret=%d\n",
				     area->area_id, ret);
	}
}

static int
virtio_msg_ffa_device_area_queue_del_after_add_locked
		(struct virtio_msg_ffa_device *vdev,
		 struct virtio_msg_ffa_device_area *area)
{
	u32 handle;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !area)
		return -EINVAL;
	if (!area->unshare_pending_after_add)
		return 0;

	handle = READ_ONCE(vdev->bridge.handle);
	if (!handle)
		goto finalize_now;

	ret = virtio_msg_bus_dma_device_helper_map_del_req
		(&vdev->area_maps, handle, area->map_id);
	if (ret == -EALREADY)
		ret = 0;
	if (ret == -ENODEV || ret == -ENOTCONN)
		goto finalize_now;
	if (ret)
		return ret;

	area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_DEL_PENDING;
	area->unshare_pending_after_add = false;
	return 0;

finalize_now:
	virtio_msg_ffa_device_area_purge_maps_locked(vdev);
	area->map_id = 0;
	area->map_seq = 0;
	area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_NONE;
	area->unshare_pending_after_add = false;
	return virtio_msg_ffa_device_area_finalize_deferred_locked(vdev, area);
}

int
virtio_msg_ffa_device_area_map_add_resp_locked(struct virtio_msg_ffa_device *vdev,
					       const struct vmsg_bridge_uapi_map_event *event,
					       u32 ack_status,
					       bool *endpoint_remove)
{
	struct virtio_msg_bus_dma_device_map_result result;
	struct vmsg_bridge_uapi_map_event response;
	struct virtio_msg_ffa_device_area *area;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!endpoint_remove)
		return -EINVAL;

	ret = virtio_msg_ffa_device_area_map_response_prepare
		(event, ack_status, VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP,
		 &response);
	if (ret)
		return ret;

	ret = virtio_msg_bus_dma_device_helper_map_add_response
		(&vdev->area_maps, &response, &result);
	if (ret)
		return ret;
	if (result.action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_IGNORED)
		return 0;

	area = virtio_msg_ffa_device_area_from_map_locked(vdev, &result.map);
	if (!area)
		return 0;

	area->map_id = result.map.map_id;
	area->map_seq = result.map.map_seq;
	if (vdev->fdev)
		dev_dbg(&vdev->fdev->dev,
			"area-share map-add response: area=%u map_id=%#llx action=%u status=%d\n",
			area->area_id, (unsigned long long)area->map_id,
			result.action, response.status);
	else
		pr_debug("area-share map-add response: area=%u map_id=%#llx action=%u status=%d\n",
			 area->area_id, (unsigned long long)area->map_id,
			 result.action, response.status);
	switch (result.action) {
	case VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_ACTIVE:
		if (area->share_abandoned) {
			ret = virtio_msg_ffa_device_area_cleanup_abandoned_share_locked
				(vdev, area);
			if (ret && vdev->fdev) {
				dev_warn_ratelimited(&vdev->fdev->dev,
						     "abandoned share cleanup failed area=%u ret=%d\n",
						     area->area_id, ret);
			}
			return 0;
		}
		area->bridge_state =
			VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ACTIVE;
		if (area->unshare_pending_after_add)
			virtio_msg_ffa_device_area_share_complete_locked(area,
									 -ECANCELED);
		else
			virtio_msg_ffa_device_area_share_complete_locked(area, 0);
		ret = virtio_msg_ffa_device_area_queue_del_after_add_locked
				(vdev, area);
		if (ret && vdev->fdev) {
			dev_warn_ratelimited(&vdev->fdev->dev,
					     "pending area unshare delete failed area=%u ret=%d\n",
					     area->area_id, ret);
		}
		if (ret)
			return ret;
		break;
	case VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_DEL_QUEUED:
		area->bridge_state =
			VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_DEL_PENDING;
		area->unshare_pending_after_add = true;
		virtio_msg_ffa_device_area_share_complete_locked(area,
								 -ECANCELED);
		break;
	case VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_FAILED:
		if (area->share_waiter || area->share_abandoned) {
			area->map_id = 0;
			area->map_seq = result.map.map_seq;
			virtio_msg_ffa_device_area_drop_failed_share_locked
				(vdev, area, -EIO);
			break;
		}
		area->bridge_state =
			VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_STALE_CLEANUP;
		ret = virtio_msg_ffa_device_area_notify_not_present_locked
			(vdev, area, endpoint_remove);
		if (ret && vdev->fdev) {
			dev_warn_ratelimited(&vdev->fdev->dev,
					     "late map failure device removal failed area=%u ret=%d\n",
					     area->area_id, ret);
		}
		virtio_msg_ffa_device_area_release_after_failure_locked
			(vdev, area);
		break;
	default:
		break;
	}

	return 0;
}

int
virtio_msg_ffa_device_area_map_del_resp_locked(struct virtio_msg_ffa_device *vdev,
					       const struct vmsg_bridge_uapi_map_event *event,
					       u32 ack_status)
{
	struct virtio_msg_bus_dma_device_map_result result;
	struct vmsg_bridge_uapi_map_event response;
	struct virtio_msg_ffa_device_area *area;
	bool finalize_deferred = false;
	int ret;

	lockdep_assert_held(&vdev->lock);

	ret = virtio_msg_ffa_device_area_map_response_prepare
		(event, ack_status, VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP,
		 &response);
	if (ret)
		return ret;

	ret = virtio_msg_bus_dma_device_helper_map_del_response
		(&vdev->area_maps, &response, &result);
	if (ret)
		return ret;
	if (result.action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_IGNORED)
		return 0;

	area = virtio_msg_ffa_device_area_from_map_locked(vdev, &result.map);
	if (!area)
		return 0;

	area->map_id = 0;
	area->map_seq = result.map.map_seq;
	if (result.action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_DONE ||
	    result.action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED_DEL_DONE)
		finalize_deferred = true;
	else if (result.action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_REMOVED &&
		 result.old_state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_QUEUED)
		finalize_deferred = true;

	if (!finalize_deferred) {
		area->bridge_state =
			VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_STALE_CLEANUP;
		return 0;
	}

	if (area->share_abandoned) {
		virtio_msg_ffa_device_area_drop_failed_share_locked
			(vdev, area, -ECANCELED);
		return 0;
	}

	area->bridge_state = VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_NONE;
	area->unshare_pending_after_add = false;
	ret = virtio_msg_ffa_device_area_finalize_deferred_locked(vdev, area);
	if (ret && vdev->fdev) {
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "deferred area release completion failed area=%u ret=%d\n",
				     area->area_id, ret);
	}

	return 0;
}

int
virtio_msg_ffa_device_area_map_released_locked(struct virtio_msg_ffa_device *vdev,
					       const struct vmsg_bridge_uapi_map_event *event,
					       bool *endpoint_remove)
{
	struct virtio_msg_bus_dma_device_map_result result;
	struct virtio_msg_ffa_device_area *area;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !event || !endpoint_remove)
		return -EINVAL;
	if (event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_RELEASED ||
	    event->status)
		return -EINVAL;

	ret = virtio_msg_bus_dma_device_helper_map_released
		(&vdev->area_maps, event, &result);
	if (ret)
		return ret;
	if (result.action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_IGNORED)
		return 0;
	if (result.action == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED_DEL_DONE) {
		area = virtio_msg_ffa_device_area_from_released_map_locked
			(vdev, &result.map);
		if (area) {
			area->map_id = 0;
			area->map_seq = result.map.map_seq;
			if (area->share_abandoned) {
				virtio_msg_ffa_device_area_drop_failed_share_locked
					(vdev, area, -ECANCELED);
				return 0;
			}
			area->bridge_state =
				VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_NONE;
			area->unshare_pending_after_add = false;
			ret = virtio_msg_ffa_device_area_finalize_deferred_locked
				(vdev, area);
			if (ret && vdev->fdev) {
				dev_warn_ratelimited
					(&vdev->fdev->dev,
					 "released area completion failed area=%u ret=%d\n",
					 area->area_id, ret);
			}
		}
		return 0;
	}
	if (result.action != VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED)
		return 0;

	area = virtio_msg_ffa_device_area_from_released_map_locked
		(vdev, &result.map);
	if (!area)
		return 0;

	area->map_id = 0;
	area->map_seq = result.map.map_seq;
	if (area->share_abandoned) {
		virtio_msg_ffa_device_area_map_release_forget(vdev, event);
		virtio_msg_ffa_device_area_drop_failed_share_locked
			(vdev, area, -ECANCELED);
		return 0;
	}
	area->bridge_state =
		VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_REMOTE_RELEASED;
	area->unshare_pending_after_add = false;
	virtio_msg_ffa_device_area_map_release_forget(vdev, event);

	if (area->driver_retention_requested)
		return virtio_msg_ffa_device_area_notify_not_present_locked
			(vdev, area, endpoint_remove);

	return 0;
}

int
virtio_msg_ffa_device_area_publish_deferred_locked(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_area *area;
	unsigned long area_id = 0;
	int first_ret = 0;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev)
		return -EINVAL;

	for (;;) {
		area = xa_find(&vdev->areas_by_id, &area_id, ULONG_MAX,
			       XA_PRESENT);
		if (!area)
			break;
		area_id = area->area_id + 1;

		if (area->bridge_state !=
		    VIRTIO_MSG_FFA_DEVICE_AREA_BRIDGE_ADD_DEFERRED)
			continue;
		if (area->map_id || area->share_abandoned)
			continue;

		ret = virtio_msg_ffa_device_area_publish_map_add_locked
			(vdev, area, NULL);
		if (!ret) {
			if (vdev->fdev)
				dev_dbg(&vdev->fdev->dev,
					"deferred area map-add queued: area=%u map_id=%#llx\n",
					area->area_id,
					(unsigned long long)area->map_id);
			else
				pr_debug("deferred area map-add queued: area=%u map_id=%#llx\n",
					 area->area_id,
					 (unsigned long long)area->map_id);
			continue;
		}
		if (virtio_msg_ffa_device_area_publish_deferred_ret(ret))
			return first_ret;
		if (!first_ret)
			first_ret = ret;
		if (vdev->fdev) {
			dev_warn_ratelimited(&vdev->fdev->dev,
					     "deferred area map-add failed area=%u ret=%d\n",
					     area->area_id, ret);
		}
	}

	return first_ret;
}

#else /* IS_REACHABLE(CONFIG_VIRTIO_MSG_BRIDGE) */

int virtio_msg_ffa_device_area_runtime_init(struct virtio_msg_ffa_device *vdev)
{
	(void)vdev;
	return 0;
}

void virtio_msg_ffa_device_area_runtime_cleanup(struct virtio_msg_ffa_device *vdev)
{
	(void)vdev;
}

void virtio_msg_ffa_device_area_runtime_reset_locked(struct virtio_msg_ffa_device *vdev,
						     bool emit_deferred_release)
{
	(void)vdev;
	(void)emit_deferred_release;
}

int virtio_msg_ffa_device_area_share(struct virtio_msg_ffa_device *vdev,
				     const struct virtio_msg_ffa_area_share_req *req,
				     u16 *result)
{
	(void)vdev;
	(void)req;
	(void)result;
	return -EOPNOTSUPP;
}

int virtio_msg_ffa_device_area_unshare_locked(struct virtio_msg_ffa_device *vdev,
					      u16 area_id, u16 *result)
{
	(void)vdev;
	(void)area_id;
	(void)result;
	return -EOPNOTSUPP;
}

int virtio_msg_ffa_device_area_mmap_locked(struct virtio_msg_ffa_device *vdev,
					   u64 mmap_offset,
					   struct vm_area_struct *vma)
{
	(void)vdev;
	(void)mmap_offset;
	(void)vma;
	return -EOPNOTSUPP;
}

int
virtio_msg_ffa_device_area_map_add_resp_locked(struct virtio_msg_ffa_device *vdev,
					       const struct vmsg_bridge_uapi_map_event *event,
					       u32 ack_status,
					       bool *endpoint_remove)
{
	(void)vdev;
	(void)event;
	(void)ack_status;
	(void)endpoint_remove;
	return -EOPNOTSUPP;
}

int
virtio_msg_ffa_device_area_map_del_resp_locked(struct virtio_msg_ffa_device *vdev,
					       const struct vmsg_bridge_uapi_map_event *event,
					       u32 ack_status)
{
	(void)vdev;
	(void)event;
	(void)ack_status;
	return -EOPNOTSUPP;
}

int
virtio_msg_ffa_device_area_map_released_locked(struct virtio_msg_ffa_device *vdev,
					       const struct vmsg_bridge_uapi_map_event *event,
					       bool *endpoint_remove)
{
	(void)vdev;
	(void)event;
	(void)endpoint_remove;
	return -EOPNOTSUPP;
}

int
virtio_msg_ffa_device_area_publish_deferred_locked(struct virtio_msg_ffa_device *vdev)
{
	(void)vdev;
	return -EOPNOTSUPP;
}

#endif /* IS_REACHABLE(CONFIG_VIRTIO_MSG_BRIDGE) */
