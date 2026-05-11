// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus bridge/device-side DMA helper.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "virtio_msg_bus_dma_device_helper.h"
#include "virtio_msg_bus_dma_record.h"

struct virtio_msg_bus_dma_device_map_record {
	struct virtio_msg_bus_dma_device_map_info map;
};

struct virtio_msg_bus_dma_device_add_ctx {
	struct virtio_msg_bus_dma_device_helper *helper;
	struct virtio_msg_bus_dma_device_map_record *record;
};

static int
virtio_msg_bus_dma_device_map_id_to_index(u64 map_id, unsigned long *index)
{
	if (!map_id)
		return -EINVAL;

	return virtio_msg_bus_dma_xa_index_from_u64(map_id, index);
}

static int
virtio_msg_bus_dma_device_mmap_offset_to_index(u64 mmap_offset,
					       unsigned long *index)
{
	u64 value64;

	if (!mmap_offset || !PAGE_ALIGNED(mmap_offset))
		return -EINVAL;

	value64 = mmap_offset >> PAGE_SHIFT;
	return virtio_msg_bus_dma_xa_index_from_u64(value64, index);
}

static void
virtio_msg_bus_dma_device_record_snapshot
		(const struct virtio_msg_bus_dma_device_map_record *record,
		 struct virtio_msg_bus_dma_device_map_info *map)
{
	if (map)
		*map = record->map;
}

static int
virtio_msg_bus_dma_device_record_insert_locked
		(struct virtio_msg_bus_dma_device_helper *helper,
		 struct virtio_msg_bus_dma_device_map_record *record)
{
	unsigned long id_index;
	unsigned long offset_index;
	int ret;

	lockdep_assert_held(&helper->lock);

	ret = virtio_msg_bus_dma_device_map_id_to_index(record->map.map_id,
							&id_index);
	if (ret)
		return ret;

	ret = virtio_msg_bus_dma_device_mmap_offset_to_index
		(record->map.mmap_offset, &offset_index);
	if (ret)
		return ret;

	ret = virtio_msg_bus_dma_xa_insert_unique(&helper->maps_by_id,
						  id_index, record,
						  GFP_KERNEL);
	if (ret)
		return ret;

	ret = virtio_msg_bus_dma_xa_insert_unique
		(&helper->maps_by_mmap_offset, offset_index, record,
		 GFP_KERNEL);
	if (ret) {
		virtio_msg_bus_dma_xa_erase_if_match(&helper->maps_by_id,
						     id_index, record);
		return ret;
	}

	return 0;
}

static void
virtio_msg_bus_dma_device_record_remove_locked
		(struct virtio_msg_bus_dma_device_helper *helper,
		 struct virtio_msg_bus_dma_device_map_record *record)
{
	unsigned long id_index;
	unsigned long offset_index;

	lockdep_assert_held(&helper->lock);

	if (!record)
		return;

	if (!virtio_msg_bus_dma_device_map_id_to_index
			(record->map.map_id, &id_index))
		virtio_msg_bus_dma_xa_erase_if_match(&helper->maps_by_id,
						     id_index, record);

	if (!virtio_msg_bus_dma_device_mmap_offset_to_index
			(record->map.mmap_offset, &offset_index))
		virtio_msg_bus_dma_xa_erase_if_match
			(&helper->maps_by_mmap_offset, offset_index, record);

	kfree(record);
}

static int
virtio_msg_bus_dma_device_map_add_install
		(struct virtio_msg_bus_bridge_device *endpoint, u64 map_id,
		 void *data)
{
	struct virtio_msg_bus_dma_device_add_ctx *ctx = data;
	struct virtio_msg_bus_dma_device_map_record *record;
	struct virtio_msg_bus_dma_device_helper *helper;
	int ret;

	(void)endpoint;

	if (!ctx || !ctx->helper || !ctx->record || !map_id)
		return -EINVAL;

	helper = ctx->helper;
	record = ctx->record;

	mutex_lock(&helper->lock);
	if (record->map.map_id) {
		ret = -EALREADY;
		goto out_unlock;
	}

	record->map.map_id = map_id;
	ret = virtio_msg_bus_dma_device_record_insert_locked(helper, record);
	if (ret)
		record->map.map_id = 0;

out_unlock:
	mutex_unlock(&helper->lock);
	return ret;
}

static void
virtio_msg_bus_dma_device_result_snapshot
		(struct virtio_msg_bus_dma_device_map_result *result,
		 enum virtio_msg_bus_dma_device_map_action action,
		 enum virtio_msg_bus_dma_device_map_state old_state,
		 enum virtio_msg_bus_dma_device_map_state new_state,
		 const struct virtio_msg_bus_dma_device_map_info *map)
{
	if (!result)
		return;

	result->action = action;
	result->old_state = old_state;
	result->new_state = new_state;
	if (map)
		result->map = *map;
}

static bool
virtio_msg_bus_dma_device_map_add_matches
		(const struct virtio_msg_bus_dma_device_map_record *record,
		 const struct vmsg_bridge_uapi_map_event *event)
{
	return record->map.map_id == event->map_id &&
	       (!record->map.map_seq || record->map.map_seq == event->map_seq) &&
	       record->map.bus_addr == event->bus_addr &&
	       record->map.length == event->length &&
	       record->map.mmap_offset == event->mmap_offset &&
	       record->map.mmap_length == event->mmap_length &&
	       record->map.flags == event->flags;
}

static bool
virtio_msg_bus_dma_device_map_del_matches
		(const struct virtio_msg_bus_dma_device_map_record *record,
		 const struct vmsg_bridge_uapi_map_event *event)
{
	return record->map.map_id == event->map_id &&
	       (!record->map.map_seq || record->map.map_seq == event->map_seq) &&
	       record->map.bus_addr == event->bus_addr &&
	       record->map.length == event->length &&
	       record->map.flags == event->flags;
}

static bool
virtio_msg_bus_dma_device_map_released_matches
		(const struct virtio_msg_bus_dma_device_map_record *record,
		 const struct vmsg_bridge_uapi_map_event *event)
{
	return record->map.map_id == event->map_id &&
	       (!record->map.map_seq || record->map.map_seq == event->map_seq) &&
	       record->map.bus_addr == event->bus_addr &&
	       record->map.length == event->length;
}

void virtio_msg_bus_dma_device_helper_init(struct virtio_msg_bus_dma_device_helper *helper)
{
	if (!helper)
		return;

	mutex_init(&helper->lock);
	xa_init(&helper->maps_by_id);
	xa_init(&helper->maps_by_mmap_offset);
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_init);

void virtio_msg_bus_dma_device_helper_destroy(struct virtio_msg_bus_dma_device_helper *helper)
{
	if (!helper)
		return;

	virtio_msg_bus_dma_device_helper_purge(helper, NULL, NULL);
	xa_destroy(&helper->maps_by_id);
	xa_destroy(&helper->maps_by_mmap_offset);
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_destroy);

int virtio_msg_bus_dma_device_helper_map_add(struct virtio_msg_bus_dma_device_helper *helper,
					     u32 handle,
					     const struct virtio_msg_bus_dma_device_map_desc *desc,
					     u64 *map_id)
{
	struct virtio_msg_bus_dma_device_map_record *record;
	struct virtio_msg_bus_dma_device_add_ctx ctx;
	struct virtio_msg_bus_bridge_map_add_install install;
	u64 end;
	u64 published_map_id = 0;
	bool installed = false;
	int ret;

	if (map_id)
		*map_id = 0;

	if (!helper || !handle || !desc)
		return -EINVAL;
	if (!desc->bus_addr || !desc->length || !desc->mmap_offset ||
	    !desc->mmap_length)
		return -EINVAL;
	if (!PAGE_ALIGNED(desc->bus_addr) || !PAGE_ALIGNED(desc->length) ||
	    !PAGE_ALIGNED(desc->mmap_offset) ||
	    !PAGE_ALIGNED(desc->mmap_length))
		return -EINVAL;
	if (desc->mmap_length < desc->length)
		return -EINVAL;
	if (check_add_overflow(desc->bus_addr, desc->length, &end))
		return -EOVERFLOW;
	if (check_add_overflow(desc->mmap_offset, desc->mmap_length, &end))
		return -EOVERFLOW;

	record = kzalloc(sizeof(*record), GFP_KERNEL);
	if (!record)
		return -ENOMEM;

	record->map.bus_addr = desc->bus_addr;
	record->map.length = desc->length;
	record->map.mmap_offset = desc->mmap_offset;
	record->map.mmap_length = desc->mmap_length;
	record->map.flags = desc->flags;
	record->map.cookie = desc->cookie;
	record->map.state = VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_QUEUED;

	ctx.helper = helper;
	ctx.record = record;
	install.install = virtio_msg_bus_dma_device_map_add_install;
	install.data = &ctx;

	ret = virtio_msg_bus_bridge_device_map_add_install
		(handle, desc->bus_addr, desc->length, desc->mmap_offset,
		 desc->mmap_length, desc->flags, &install, &published_map_id);
	if (ret) {
		mutex_lock(&helper->lock);
		if (record->map.map_id) {
			virtio_msg_bus_dma_device_record_remove_locked(helper,
								       record);
			installed = true;
		}
		mutex_unlock(&helper->lock);
		if (!installed)
			kfree(record);
		return ret;
	}

	if (map_id)
		*map_id = published_map_id;

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_map_add);

int virtio_msg_bus_dma_device_helper_map_del_req(struct virtio_msg_bus_dma_device_helper *helper,
						 u32 handle, u64 map_id)
{
	struct virtio_msg_bus_dma_device_map_record *record;
	enum virtio_msg_bus_dma_device_map_state state;
	unsigned long index;
	int ret;

	if (!helper || !handle || !map_id)
		return -EINVAL;

	ret = virtio_msg_bus_dma_device_map_id_to_index(map_id, &index);
	if (ret)
		return ret;

	mutex_lock(&helper->lock);
	record = xa_load(&helper->maps_by_id, index);
	if (!record) {
		mutex_unlock(&helper->lock);
		return -ENOENT;
	}

	state = record->map.state;
	if (state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_QUEUED) {
		mutex_unlock(&helper->lock);
		return -EALREADY;
	}
	if (state != VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_QUEUED &&
	    state != VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ACTIVE) {
		mutex_unlock(&helper->lock);
		return -EBUSY;
	}
	mutex_unlock(&helper->lock);

	ret = virtio_msg_bus_bridge_device_map_del(handle, map_id);
	if (ret && ret != -EALREADY)
		return ret;

	mutex_lock(&helper->lock);
	record = xa_load(&helper->maps_by_id, index);
	if (record &&
	    (record->map.state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_QUEUED ||
	     record->map.state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ACTIVE))
		record->map.state = VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_QUEUED;
	mutex_unlock(&helper->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_map_del_req);

int
virtio_msg_bus_dma_device_helper_map_add_response(struct virtio_msg_bus_dma_device_helper *helper,
						  const struct vmsg_bridge_uapi_map_event *event,
						  struct virtio_msg_bus_dma_device_map_result *res)
{
	struct virtio_msg_bus_dma_device_map_info snapshot;
	struct virtio_msg_bus_dma_device_map_record *record;
	enum virtio_msg_bus_dma_device_map_state old_state;
	unsigned long index;
	int ret;

	if (res)
		memset(res, 0, sizeof(*res));

	if (!helper || !event)
		return -EINVAL;
	if (event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP ||
	    event->status > 0)
		return -EINVAL;

	ret = virtio_msg_bus_dma_device_map_id_to_index(event->map_id, &index);
	if (ret)
		return 0;

	mutex_lock(&helper->lock);
	record = xa_load(&helper->maps_by_id, index);
	if (!record)
		goto out_unlock;

	if (!virtio_msg_bus_dma_device_map_add_matches(record, event))
		goto out_unlock;

	old_state = record->map.state;
	if (event->status) {
		record->map.map_seq = event->map_seq;
		virtio_msg_bus_dma_device_record_snapshot(record, &snapshot);
		virtio_msg_bus_dma_device_record_remove_locked(helper, record);
		virtio_msg_bus_dma_device_result_snapshot
			(res, VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_FAILED,
			 old_state,
			 VIRTIO_MSG_BUS_DMA_DEVICE_MAP_NONE, &snapshot);
		goto out_unlock;
	}

	record->map.map_seq = event->map_seq;
	if (old_state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_QUEUED) {
		record->map.state = VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ACTIVE;
		virtio_msg_bus_dma_device_record_snapshot(record, &snapshot);
		virtio_msg_bus_dma_device_result_snapshot
			(res, VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_ACTIVE,
			 old_state, record->map.state, &snapshot);
	} else if (old_state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_QUEUED) {
		virtio_msg_bus_dma_device_record_snapshot(record, &snapshot);
		virtio_msg_bus_dma_device_result_snapshot
			(res, VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_DEL_QUEUED,
			 old_state, old_state, &snapshot);
	}

out_unlock:
	mutex_unlock(&helper->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_map_add_response);

int
virtio_msg_bus_dma_device_helper_map_del_response(struct virtio_msg_bus_dma_device_helper *helper,
						  const struct vmsg_bridge_uapi_map_event *event,
						  struct virtio_msg_bus_dma_device_map_result *res)
{
	struct virtio_msg_bus_dma_device_map_info snapshot;
	struct virtio_msg_bus_dma_device_map_record *record;
	enum virtio_msg_bus_dma_device_map_state old_state;
	unsigned long index;
	int ret;

	if (res)
		memset(res, 0, sizeof(*res));

	if (!helper || !event)
		return -EINVAL;
	if (event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP ||
	    event->status > 0)
		return -EINVAL;

	ret = virtio_msg_bus_dma_device_map_id_to_index(event->map_id, &index);
	if (ret)
		return 0;

	mutex_lock(&helper->lock);
	record = xa_load(&helper->maps_by_id, index);
	if (!record)
		goto out_unlock;
	if (!virtio_msg_bus_dma_device_map_del_matches(record, event))
		goto out_unlock;

	old_state = record->map.state;
	if (event->status) {
		record->map.map_seq = event->map_seq;
		virtio_msg_bus_dma_device_record_snapshot(record, &snapshot);
		virtio_msg_bus_dma_device_record_remove_locked(helper, record);
		virtio_msg_bus_dma_device_result_snapshot
			(res, VIRTIO_MSG_BUS_DMA_DEVICE_MAP_REMOVED,
			 old_state, VIRTIO_MSG_BUS_DMA_DEVICE_MAP_NONE,
			 &snapshot);
		goto out_unlock;
	}

	if (old_state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_QUEUED ||
	    old_state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_REMOTE_RELEASED) {
		record->map.map_seq = event->map_seq;
		virtio_msg_bus_dma_device_record_snapshot(record, &snapshot);
		virtio_msg_bus_dma_device_record_remove_locked(helper, record);
		virtio_msg_bus_dma_device_result_snapshot
			(res, VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_DONE,
			 old_state, VIRTIO_MSG_BUS_DMA_DEVICE_MAP_NONE,
			 &snapshot);
	}

out_unlock:
	mutex_unlock(&helper->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_map_del_response);

int
virtio_msg_bus_dma_device_helper_map_released(struct virtio_msg_bus_dma_device_helper *helper,
					      const struct vmsg_bridge_uapi_map_event *event,
					      struct virtio_msg_bus_dma_device_map_result *result)
{
	struct virtio_msg_bus_dma_device_map_info snapshot;
	struct virtio_msg_bus_dma_device_map_record *record;
	enum virtio_msg_bus_dma_device_map_state old_state;
	enum virtio_msg_bus_dma_device_map_action action;
	unsigned long index;
	int ret;

	if (result)
		memset(result, 0, sizeof(*result));

	if (!helper || !event)
		return -EINVAL;
	if (event->type != VMSG_BRIDGE_UAPI_MAP_EVENT_RELEASED ||
	    event->status)
		return -EINVAL;

	ret = virtio_msg_bus_dma_device_map_id_to_index(event->map_id, &index);
	if (ret)
		return 0;

	mutex_lock(&helper->lock);
	record = xa_load(&helper->maps_by_id, index);
	if (!record)
		goto out_unlock;
	if (!virtio_msg_bus_dma_device_map_released_matches(record, event))
		goto out_unlock;

	record->map.map_seq = event->map_seq;
	old_state = record->map.state;
	if (old_state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_DEL_QUEUED)
		action = VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED_DEL_DONE;
	else if (old_state == VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ACTIVE)
		action = VIRTIO_MSG_BUS_DMA_DEVICE_MAP_RELEASED;
	else
		goto out_unlock;

	record->map.state = VIRTIO_MSG_BUS_DMA_DEVICE_MAP_REMOTE_RELEASED;
	virtio_msg_bus_dma_device_record_snapshot(record, &snapshot);
	virtio_msg_bus_dma_device_record_remove_locked(helper, record);
	virtio_msg_bus_dma_device_result_snapshot
		(result, action, old_state,
		 VIRTIO_MSG_BUS_DMA_DEVICE_MAP_REMOTE_RELEASED, &snapshot);

out_unlock:
	mutex_unlock(&helper->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_map_released);

int virtio_msg_bus_dma_device_helper_map_event_ack(struct virtio_msg_bus_dma_device_helper *helper,
						   const struct vmsg_bridge_uapi_map_event *event,
						   u32 ack_status,
						   struct virtio_msg_bus_dma_device_ack *ack)
{
	struct vmsg_bridge_uapi_map_event response;

	if (!event)
		return -EINVAL;
	if (ack_status != VMSG_BRIDGE_UAPI_MAP_ACK_OK &&
	    ack_status != VMSG_BRIDGE_UAPI_MAP_ACK_REJECT)
		return -EINVAL;

	response = *event;
	if (ack_status == VMSG_BRIDGE_UAPI_MAP_ACK_REJECT && !response.status)
		response.status = -EIO;

	switch (response.type) {
	case VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP:
		return virtio_msg_bus_dma_device_helper_map_add_response
			(helper, &response, ack);
	case VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP:
		return virtio_msg_bus_dma_device_helper_map_del_response
			(helper, &response, ack);
	case VMSG_BRIDGE_UAPI_MAP_EVENT_RELEASED:
		return virtio_msg_bus_dma_device_helper_map_released
			(helper, &response, ack);
	default:
		if (ack)
			memset(ack, 0, sizeof(*ack));
		return 0;
	}
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_map_event_ack);

int virtio_msg_bus_dma_device_helper_lookup_by_id(struct virtio_msg_bus_dma_device_helper *helper,
						  u64 map_id,
						  struct virtio_msg_bus_dma_device_map_info *map)
{
	struct virtio_msg_bus_dma_device_map_record *record;
	unsigned long index;
	int ret;

	if (!helper || !map)
		return -EINVAL;

	ret = virtio_msg_bus_dma_device_map_id_to_index(map_id, &index);
	if (ret)
		return ret;

	mutex_lock(&helper->lock);
	record = xa_load(&helper->maps_by_id, index);
	if (record)
		virtio_msg_bus_dma_device_record_snapshot(record, map);
	mutex_unlock(&helper->lock);

	return record ? 0 : -ENOENT;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_lookup_by_id);

int
virtio_msg_bus_dma_device_helper_lookup_by_mmap_offset(struct virtio_msg_bus_dma_device_helper *h,
						       u64 mmap_offset,
						       u64 mmap_length,
		struct virtio_msg_bus_dma_device_map_info *map)
{
	struct virtio_msg_bus_dma_device_helper *helper = h;
	struct virtio_msg_bus_dma_device_map_record *record;
	unsigned long index;
	int ret;

	if (!helper || !map)
		return -EINVAL;

	ret = virtio_msg_bus_dma_device_mmap_offset_to_index(mmap_offset,
							     &index);
	if (ret)
		return ret;

	mutex_lock(&helper->lock);
	record = xa_load(&helper->maps_by_mmap_offset, index);
	if (!record ||
	    (record->map.state != VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ADD_QUEUED &&
	     record->map.state != VIRTIO_MSG_BUS_DMA_DEVICE_MAP_ACTIVE)) {
		ret = -ENOENT;
		goto out_unlock;
	}
	if (mmap_length > record->map.mmap_length) {
		ret = -ERANGE;
		goto out_unlock;
	}

	virtio_msg_bus_dma_device_record_snapshot(record, map);
	ret = 0;

out_unlock:
	mutex_unlock(&helper->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_lookup_by_mmap_offset);

void
virtio_msg_bus_dma_device_helper_purge(struct virtio_msg_bus_dma_device_helper *helper,
				       void (*purge)(const struct virtio_msg_bus_dma_device_map_info
						     *map, void *data),
				       void *data)
{
	struct virtio_msg_bus_dma_device_map_info snapshot;
	struct virtio_msg_bus_dma_device_map_record *record;
	unsigned long index;

	if (!helper)
		return;

	for (;;) {
		index = 0;

		mutex_lock(&helper->lock);
		record = xa_find(&helper->maps_by_id, &index, ULONG_MAX,
				 XA_PRESENT);
		if (!record) {
			mutex_unlock(&helper->lock);
			break;
		}

		virtio_msg_bus_dma_device_record_snapshot(record, &snapshot);
		virtio_msg_bus_dma_device_record_remove_locked(helper, record);
		mutex_unlock(&helper->lock);

		if (purge)
			purge(&snapshot, data);
	}
}
EXPORT_SYMBOL_GPL(virtio_msg_bus_dma_device_helper_purge);

MODULE_LICENSE("GPL");
