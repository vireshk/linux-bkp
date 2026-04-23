// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message transport bootstrap core (Step 8).
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 *
 * This unit intentionally limits scope to provider attach/register flow plus
 * blocking bootstrap/config/status operations. Full queue/event transport
 * behavior is deferred to later steps.
 */

#define pr_fmt(fmt) "virtio-msg-transport: " fmt

#include <asm/barrier.h>
#include <linux/bitmap.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_msg_bus_provider.h>
#include <linux/virtio_msg_protocol.h>
#include <linux/virtio_msg_transport.h>

#include "virtio_msg_transport_internal.h"
#define VM_LOG_COMPONENT VM_LOG_COMPONENT_TRANSPORT
#include "virtio_msg_debug.h"

static inline struct virtio_msg_transport_device *
to_virtio_msg_transport_device(struct virtio_device *vdev)
{
	return container_of(vdev, struct virtio_msg_transport_device, vdev);
}

static inline struct virtio_msg_transport *
vdev_transport(struct virtio_device *vdev,
	       struct virtio_msg_transport_device **vmdevp)
{
	struct virtio_msg_transport_device *vmdev =
		to_virtio_msg_transport_device(vdev);

	if (vmdevp)
		*vmdevp = vmdev;
	return vmdev->private;
}

static inline struct virtio_msg_transport *
virtio_msg_transport_priv(struct virtio_msg_transport_device *vmdev)
{
	return vmdev ? vmdev->private : NULL;
}

bool virtio_msg_transport_is_fatal(const struct virtio_msg_transport *transport)
{
	return transport && transport->fatal;
}

static bool
virtio_msg_transport_is_unplugged(const struct virtio_msg_transport *transport)
{
	return transport && transport->unplugged;
}

static void
virtio_msg_transport_begin_unplug(struct virtio_msg_transport_device *vmdev,
				  struct virtio_msg_transport *transport)
{
	if (!vmdev || !transport)
		return;

	mutex_lock(&transport->request_lock);
	transport->unplugged = true;
	transport->fatal = true;
	WRITE_ONCE(transport->cached_status, 0);
	mutex_unlock(&transport->request_lock);

	virtio_break_device(&vmdev->vdev);
}

void virtio_msg_transport_set_fatal(struct virtio_msg_transport_device *vmdev,
				    struct virtio_msg_transport *transport)
{
	if (!vmdev || !transport || transport->fatal)
		return;

	transport->fatal = true;
	vm_dbg(&vmdev->vdev.dev, "fatal error set\n");
	if (transport->registered)
		virtio_break_device(&vmdev->vdev);
}

int virtio_msg_transport_validate_config_range(const struct virtio_msg_transport *transport,
					       u32 offset, u32 len)
{
	u64 end;

	if (!transport)
		return -EINVAL;

	end = (u64)offset + (u64)len;
	if (end > transport->config_size)
		return -EINVAL;

	return 0;
}

static int virtio_msg_payload_capacity(const struct virtio_msg_transport *transport)
{
	if (transport->msg_size < sizeof(struct virtio_msg))
		return -EMSGSIZE;

	return transport->msg_size - sizeof(struct virtio_msg);
}

enum {
	VIRTIO_MSG_CONFIG_WARN_EVENT_DROPPED = BIT(0),
	VIRTIO_MSG_CONFIG_WARN_GENERATION_COALESCED = BIT(1),
};

#define VIRTIO_MSG_GET_CONFIG_RETRY_BUDGET	3U
#define VIRTIO_MSG_REGISTER_RELEASE_TIMEOUT_MS	5000U

static int
virtio_msg_transport_config_refresh_full(struct virtio_msg_transport_device *vmdev,
					 struct virtio_msg_transport *transport);

static int
virtio_msg_transport_config_alloc(struct virtio_msg_transport *transport)
{
	if (!transport)
		return -EINVAL;

	transport->config_shadow_valid = false;
	kvfree(transport->config_shadow);
	transport->config_shadow = NULL;

	if (!transport->config_size) {
		transport->config_shadow_valid = true;
		return 0;
	}

	transport->config_shadow = kvzalloc(transport->config_size, GFP_KERNEL);
	if (!transport->config_shadow)
		return -ENOMEM;

	return 0;
}

static int
virtio_msg_transport_config_update_range(struct virtio_msg_transport *transport,
					 u32 offset, const void *src, u32 len)
{
	unsigned long flags;
	u8 *dst;
	int ret;

	if (!transport)
		return -EINVAL;
	if (!src && len)
		return -EINVAL;

	ret = virtio_msg_transport_validate_config_range(transport, offset, len);
	if (ret)
		return ret;

	if (!len) {
		spin_lock_irqsave(&transport->config_lock, flags);
		transport->config_shadow_valid = true;
		spin_unlock_irqrestore(&transport->config_lock, flags);
		return 0;
	}

	if (!transport->config_shadow)
		return -ENODATA;

	spin_lock_irqsave(&transport->config_lock, flags);
	dst = transport->config_shadow + offset;
	if (dst != src)
		memcpy(dst, src, len);
	transport->config_shadow_valid = true;
	spin_unlock_irqrestore(&transport->config_lock, flags);

	return 0;
}

static int
virtio_msg_transport_config_read_atomic(struct virtio_msg_transport *transport,
					u32 offset, void *dst, u32 len)
{
	unsigned long flags;
	int ret;

	if (!transport || !dst)
		return -EINVAL;

	ret = virtio_msg_transport_validate_config_range(transport, offset, len);
	if (ret)
		return ret;
	if (!len)
		return 0;

	spin_lock_irqsave(&transport->config_lock, flags);
	if (!transport->config_shadow || !transport->config_shadow_valid) {
		spin_unlock_irqrestore(&transport->config_lock, flags);
		return -ENODATA;
	}

	memcpy(dst, transport->config_shadow + offset, len);
	spin_unlock_irqrestore(&transport->config_lock, flags);

	return 0;
}

static void
virtio_msg_transport_config_warn_mark(struct virtio_msg_transport *transport,
				      u8 reasons, u32 generation)
{
	unsigned long flags;

	if (!transport || !reasons)
		return;

	spin_lock_irqsave(&transport->config_lock, flags);
	transport->config_warn_reasons |= reasons;
	transport->config_warn_generation =
		max(transport->config_warn_generation, generation);
	spin_unlock_irqrestore(&transport->config_lock, flags);
}

static void
virtio_msg_transport_config_warn_clear(struct virtio_msg_transport *transport)
{
	unsigned long flags;

	if (!transport)
		return;

	spin_lock_irqsave(&transport->config_lock, flags);
	transport->config_warn_reasons = 0;
	transport->config_warn_generation = 0;
	spin_unlock_irqrestore(&transport->config_lock, flags);
}

static void
virtio_msg_transport_config_warn_maybe_emit(struct virtio_msg_transport_device *vmdev,
					    struct virtio_msg_transport *transport)
{
	unsigned long flags;
	u32 generation;
	u8 reasons;

	if (!vmdev || !transport)
		return;

	spin_lock_irqsave(&transport->config_lock, flags);
	reasons = transport->config_warn_reasons;
	generation = transport->config_warn_generation;
	transport->config_warn_reasons = 0;
	spin_unlock_irqrestore(&transport->config_lock, flags);

	if (!reasons)
		return;

	vm_warn_rl(&vmdev->vdev.dev,
		   "atomic config cache risk: gen=%u dropped=%u coalesced=%u\n",
		   generation,
		   !!(reasons & VIRTIO_MSG_CONFIG_WARN_EVENT_DROPPED),
		   !!(reasons & VIRTIO_MSG_CONFIG_WARN_GENERATION_COALESCED));
}

static void virtio_msg_transport_cleanup_runtime(struct virtio_msg_transport *transport)
{
	if (transport && transport->max_vq_count) {
		if (transport->pending_used_bitmap)
			bitmap_zero(transport->pending_used_bitmap,
				    transport->max_vq_count);
		if (transport->pending_used_snapshot)
			bitmap_zero(transport->pending_used_snapshot,
				    transport->max_vq_count);
	}

	kfree(transport->request);
	kfree(transport->response);
	kvfree(transport->config_shadow);
	kvfree(transport->pending_config_data);
	bitmap_free(transport->pending_used_bitmap);
	bitmap_free(transport->pending_used_snapshot);
	memset(&transport->request, 0,
	       sizeof(*transport) - offsetof(struct virtio_msg_transport, request));
}

static int virtio_msg_validate_provider_ops
		(const struct virtio_msg_bus_provider_ops *provider_ops)
{
	if (!provider_ops || !provider_ops->get_caps || !provider_ops->send)
		return -EINVAL;

	return 0;
}

static int
virtio_msg_validate_provider_caps(const struct virtio_msg_provider_caps *caps)
{
	u32 min_size = max_t(u32, VIRTIO_MSG_MIN_SIZE,
			     VIRTIO_MSG_TRANSPORT_MIN_BUF_SIZE);

	if (!caps)
		return -EINVAL;
	if (caps->revision != VIRTIO_MSG_REVISION_1)
		return -EOPNOTSUPP;
	if (caps->msg_size < min_size || caps->msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EMSGSIZE;
	if (caps->transport_features & ~VIRTIO_MSG_TRANSPORT_F_SUPPORTED)
		return -EOPNOTSUPP;

	return 0;
}

static int
virtio_msg_transport_alloc_runtime(struct virtio_msg_transport *transport,
				   const struct virtio_msg_provider_caps *caps)
{
	transport->msg_size = caps->msg_size;
	transport->transport_revision = caps->revision;
	transport->request = kzalloc(transport->msg_size, GFP_KERNEL);
	if (!transport->request)
		goto err;

	transport->response = kzalloc(transport->msg_size, GFP_KERNEL);
	if (!transport->response)
		goto err;

	transport->unplugged = false;
	transport->fatal = false;
	WRITE_ONCE(transport->config_generation, 0);
	WRITE_ONCE(transport->cached_status, 0);
	virtio_features_zero(transport->device_features);

	return 0;

err:
	virtio_msg_transport_cleanup_runtime(transport);
	return -ENOMEM;
}

static int virtio_msg_transport_alloc_event_state(struct virtio_msg_transport *transport)
{
	int payload_capacity;
	int config_capacity;

	if (!transport)
		return -EINVAL;

	payload_capacity = virtio_msg_payload_capacity(transport);
	if (payload_capacity < 0)
		return payload_capacity;

	config_capacity = payload_capacity - sizeof(struct virtio_msg_event_config);
	if (config_capacity < 0)
		return -EMSGSIZE;

	transport->pending_config_capacity = config_capacity;
	if (transport->pending_config_capacity) {
		transport->pending_config_data =
			kvzalloc(transport->pending_config_capacity, GFP_KERNEL);
		if (!transport->pending_config_data)
			return -ENOMEM;
	}

	if (!transport->max_vq_count)
		return 0;

	transport->pending_used_bitmap =
		bitmap_zalloc(transport->max_vq_count, GFP_KERNEL);
	if (!transport->pending_used_bitmap)
		goto err_pending_config;

	transport->pending_used_snapshot =
		bitmap_zalloc(transport->max_vq_count, GFP_KERNEL);
	if (!transport->pending_used_snapshot) {
		bitmap_free(transport->pending_used_bitmap);
		transport->pending_used_bitmap = NULL;
		goto err_pending_config;
	}

	bitmap_zero(transport->pending_used_bitmap, transport->max_vq_count);
	bitmap_zero(transport->pending_used_snapshot, transport->max_vq_count);

	return 0;

err_pending_config:
	kvfree(transport->pending_config_data);
	transport->pending_config_data = NULL;
	transport->pending_config_capacity = 0;
	return -ENOMEM;
}

static u32
virtio_msg_transport_pending_depth_locked(struct virtio_msg_transport *transport)
{
	u32 depth = transport->config_pending ? 1 : 0;

	lockdep_assert_held(&transport->event_lock);

	if (transport->pending_used_bitmap && transport->max_vq_count)
		depth += bitmap_weight(transport->pending_used_bitmap,
				       transport->max_vq_count);

	return depth;
}

static void
virtio_msg_transport_reset_pending_config(struct virtio_msg_transport *transport)
{
	if (!transport)
		return;

	transport->config_pending = false;
	transport->pending_config_offset = 0;
	transport->pending_config_len = 0;
	transport->pending_config_generation = 0;
	transport->pending_device_status = 0;
}

static void
virtio_msg_transport_clear_pending_used(unsigned long *pending_used,
					u32 max_vq_count)
{
	if (pending_used && max_vq_count)
		bitmap_zero(pending_used, max_vq_count);
}

static int
virtio_msg_dispatch_config_event(struct virtio_msg_transport_device *vmdev,
				 struct virtio_msg_transport *transport,
				 u32 generation, u32 device_status,
				 bool refresh_full)
{
	int ret;

	if (refresh_full) {
		ret = virtio_msg_transport_config_refresh_full(vmdev, transport);
		if (ret)
			return ret;
	}

	if (generation > READ_ONCE(transport->config_generation))
		WRITE_ONCE(transport->config_generation, generation);
	WRITE_ONCE(transport->cached_status, (u8)device_status);
	virtio_config_changed(&vmdev->vdev);

	return 0;
}

static void
virtio_msg_synchronize_used_events(struct virtio_msg_transport_device *vmdev)
{
	if (vmdev->provider_ops && vmdev->provider_ops->synchronize_cbs)
		vmdev->provider_ops->synchronize_cbs(vmdev);
	else
		/* Best-effort fallback for providers without a sync hook. */
		smp_rmb();
}

static int
virtio_msg_dispatch_used_event(struct virtio_msg_transport_device *vmdev,
			       unsigned int index)
{
	struct virtqueue *vq;

	virtio_device_for_each_vq(&vmdev->vdev, vq) {
		if (vq->index != index)
			continue;

		(void)vring_interrupt(0, vq);
		return 0;
	}

	return -EINVAL;
}

static void virtio_msg_event_worker(struct work_struct *work)
{
	struct virtio_msg_transport *transport =
		container_of(work, struct virtio_msg_transport, event_work);
	struct virtio_msg_transport_device *vmdev = transport->vmdev;
	unsigned long flags;

	for (;;) {
		unsigned long *pending_used = NULL;
		unsigned long vq_index;
		u32 config_offset = 0;
		u32 config_len = 0;
		u32 config_generation = 0;
		u32 device_status = 0;
		bool config_pending = false;
		int ret = 0;

		spin_lock_irqsave(&transport->event_lock, flags);
		if (!transport->accept_events || !transport->events_dispatch_ready) {
			transport->event_work_queued = false;
			spin_unlock_irqrestore(&transport->event_lock, flags);
			return;
		}

		if (!transport->config_pending &&
		    (!transport->pending_used_bitmap ||
		     bitmap_empty(transport->pending_used_bitmap,
				  transport->max_vq_count))) {
			transport->event_work_queued = false;
			spin_unlock_irqrestore(&transport->event_lock, flags);
			return;
		}

		config_pending = transport->config_pending;
		config_offset = transport->pending_config_offset;
		config_len = transport->pending_config_len;
		config_generation = transport->pending_config_generation;
		device_status = transport->pending_device_status;
		if (config_pending && config_len) {
			ret = virtio_msg_transport_config_update_range
				(transport, config_offset,
				 transport->pending_config_data, config_len);
		}
		virtio_msg_transport_reset_pending_config(transport);
		if (transport->pending_used_bitmap &&
		    transport->pending_used_snapshot &&
		    transport->max_vq_count &&
		    !bitmap_empty(transport->pending_used_bitmap,
				  transport->max_vq_count)) {
			swap(transport->pending_used_bitmap,
			     transport->pending_used_snapshot);
			pending_used = transport->pending_used_snapshot;
		}
		transport->event_work_queued = false;
		spin_unlock_irqrestore(&transport->event_lock, flags);

		if (ret) {
			virtio_msg_transport_clear_pending_used
				(pending_used, transport->max_vq_count);
			virtio_msg_transport_set_fatal(vmdev, transport);
			return;
		}

		if (config_pending) {
			ret = virtio_msg_dispatch_config_event(vmdev, transport,
							       config_generation,
							       device_status,
							       !config_len);
			if (ret) {
				virtio_msg_transport_clear_pending_used
					(pending_used, transport->max_vq_count);
				virtio_msg_transport_set_fatal(vmdev, transport);
				return;
			}
		}

		if (!pending_used || !transport->max_vq_count)
			continue;

		if (!bitmap_empty(pending_used, transport->max_vq_count))
			virtio_msg_synchronize_used_events(vmdev);

		for_each_set_bit(vq_index, pending_used,
				 transport->max_vq_count) {
			virtio_msg_dispatch_used_event(vmdev, vq_index);
			__clear_bit(vq_index, pending_used);
		}
	}
}

static void
virtio_msg_transport_quiesce_events(struct virtio_msg_transport *transport)
{
	unsigned long flags;

	if (!transport)
		return;

	spin_lock_irqsave(&transport->event_lock, flags);
	transport->events_dispatch_ready = false;
	transport->event_work_queued = false;
	spin_unlock_irqrestore(&transport->event_lock, flags);

	flush_work(&transport->event_work);
}

static void
virtio_msg_transport_resume_events(struct virtio_msg_transport *transport,
				   bool dispatch_ready)
{
	unsigned long flags;
	bool queue_work = false;

	if (!transport)
		return;

	spin_lock_irqsave(&transport->event_lock, flags);
	transport->accept_events = true;
	transport->events_dispatch_ready = dispatch_ready;
	if (dispatch_ready &&
	    virtio_msg_transport_pending_depth_locked(transport) &&
	    !transport->event_work_queued) {
		transport->event_work_queued = true;
		queue_work = true;
	}
	spin_unlock_irqrestore(&transport->event_lock, flags);

	if (queue_work)
		schedule_work(&transport->event_work);
}

static void virtio_msg_transport_stop_events(struct virtio_msg_transport *transport)
{
	unsigned long flags;

	if (!transport)
		return;

	spin_lock_irqsave(&transport->event_lock, flags);
	transport->accept_events = false;
	transport->events_dispatch_ready = false;
	transport->event_work_queued = false;
	virtio_msg_transport_reset_pending_config(transport);
	spin_unlock_irqrestore(&transport->event_lock, flags);

	cancel_work_sync(&transport->event_work);

	if (transport->max_vq_count) {
		if (transport->pending_used_bitmap)
			bitmap_zero(transport->pending_used_bitmap,
				    transport->max_vq_count);
		if (transport->pending_used_snapshot)
			bitmap_zero(transport->pending_used_snapshot,
				    transport->max_vq_count);
	}
}

static int
virtio_msg_validate_response(struct virtio_msg_transport *transport,
			     const struct virtio_msg *request,
			     const struct virtio_msg *response,
			     size_t min_payload_size)
{
	u16 response_size;
	u8 expected_type;

	if (!transport || !request || !response)
		return -EINVAL;

	expected_type = request->type | VIRTIO_MSG_TYPE_RESPONSE;
	if (response->type != expected_type)
		return -EPROTO;
	if (response->msg_id != request->msg_id)
		return -EPROTO;
	if (response->dev_num != request->dev_num)
		return -EPROTO;

	response_size = le16_to_cpu(response->msg_size);
	if (response_size < sizeof(*response) + min_payload_size)
		return -EMSGSIZE;
	if (response_size > transport->msg_size)
		return -EMSGSIZE;

	return 0;
}

static int
virtio_msg_validate_event_config_range
		(struct virtio_msg_transport *transport,
		 const struct virtio_msg_event_config *cfg)
{
	return virtio_msg_transport_validate_config_range
		(transport, le32_to_cpu(cfg->offset),
		 le32_to_cpu(cfg->length));
}

static int virtio_msg_bus_send(struct virtio_msg_transport_device *vmdev,
			       const struct virtio_msg *request,
			       struct virtio_msg *response, u32 flags)
{
	if (!vmdev || !vmdev->provider_ops || !vmdev->provider_ops->send ||
	    !request)
		return -EINVAL;
	if (flags & ~VIRTIO_MSG_BUS_SEND_F_NONBLOCK)
		return -EINVAL;
	if (WARN_ON_ONCE((flags & VIRTIO_MSG_BUS_SEND_F_NONBLOCK) && response))
		return -EINVAL;

	return vmdev->provider_ops->send(vmdev, request, response, flags);
}

static int
virtio_msg_send(struct virtio_msg_transport_device *vmdev, u8 msg_id,
		const void *request_payload, size_t request_payload_size,
		size_t min_response_payload_size,
		struct virtio_msg **response_out)
{
	struct virtio_msg_transport *transport = virtio_msg_transport_priv(vmdev);
	struct virtio_msg *request;
	int payload_capacity;
	int ret;

	if (!vmdev || !transport)
		return -EINVAL;
	if (!request_payload && request_payload_size)
		return -EINVAL;
	if (request_payload_size > U16_MAX - sizeof(*request))
		return -EMSGSIZE;
	if (min_response_payload_size > U16_MAX - sizeof(*request))
		return -EMSGSIZE;

	payload_capacity = virtio_msg_payload_capacity(transport);
	if (payload_capacity < 0)
		return payload_capacity;
	if (request_payload_size > payload_capacity)
		return -EMSGSIZE;

	mutex_lock(&transport->request_lock);
	if (virtio_msg_transport_is_fatal(transport)) {
		vm_warn_rl(&vmdev->vdev.dev,
			   "tx rejected (fatal): msg_id=0x%02x\n", msg_id);
		ret = -EIO;
		goto out_unlock;
	}

	request = transport->request;
	virtio_msg_prepare(request, msg_id, 0,
			   request_payload_size);
	request->dev_num = cpu_to_le16(transport->dev_num);
	if (request_payload_size)
		memcpy(virtio_msg_payload(request), request_payload,
		       request_payload_size);

	memset(transport->response, 0, transport->msg_size);
	vm_trace(&vmdev->vdev.dev, "tx msg_id=0x%02x dev=%u\n",
		 msg_id, transport->dev_num);
	ret = virtio_msg_bus_send(vmdev, request, transport->response, 0);
	if (ret) {
		vm_warn_rl(&vmdev->vdev.dev, "tx error: msg_id=0x%02x ret=%d\n",
			   msg_id, ret);
		goto out_maybe_fatal;
	}

	ret = virtio_msg_validate_response(transport, request, transport->response,
					   min_response_payload_size);
	if (ret) {
		vm_warn_rl(&vmdev->vdev.dev,
			   "response invalid: msg_id=0x%02x ret=%d\n",
			   msg_id, ret);
		goto out_maybe_fatal;
	}

	if (response_out)
		*response_out = transport->response;

out_unlock:
	mutex_unlock(&transport->request_lock);
	return ret;

out_maybe_fatal:
	if (transport->registered)
		virtio_msg_transport_set_fatal(vmdev, transport);
	goto out_unlock;
}

static int
virtio_msg_get_device_info(struct virtio_msg_transport_device *vmdev,
			   struct virtio_msg_transport *transport)
{
	struct virtio_msg_get_device_info_resp *response_payload;
	struct virtio_msg *response;
	u32 num_feature_bits;
	u16 response_dev_num;
	int ret;

	ret = virtio_msg_send(vmdev, VIRTIO_MSG_GET_DEVICE_INFO, NULL, 0,
			      sizeof(*response_payload), &response);
	if (ret)
		return ret;

	response_payload = virtio_msg_payload(response);
	num_feature_bits = le32_to_cpu(response_payload->num_feature_bits);
	response_dev_num = le16_to_cpu(response->dev_num);
	if (num_feature_bits > VIRTIO_FEATURES_BITS)
		return -E2BIG;
	if (response_dev_num != transport->dev_num)
		return -EPROTO;

	transport->config_size = le32_to_cpu(response_payload->config_size);
	transport->num_feature_bits = num_feature_bits;
	transport->max_vq_count = le32_to_cpu(response_payload->max_vq_count);
	transport->admin_vq_start =
		le16_to_cpu(response_payload->admin_vq_start);
	transport->admin_vq_count =
		le16_to_cpu(response_payload->admin_vq_count);
	if ((u64)transport->admin_vq_start + transport->admin_vq_count >
	    transport->max_vq_count)
		return -EINVAL;

	vmdev->vdev.id.device = le32_to_cpu(response_payload->device_id);
	vmdev->vdev.id.vendor = le32_to_cpu(response_payload->vendor_id);
	if (!vmdev->vdev.id.device)
		return -ENODEV;

	vm_trace(&vmdev->vdev.dev,
		 "device info: dev=%u type=%u vendor=%u csz=%u vqs=%u\n",
		 transport->dev_num, vmdev->vdev.id.device,
		 vmdev->vdev.id.vendor, transport->config_size,
		 transport->max_vq_count);
	return 0;
}

static void
virtio_msg_transport_apply_required_features
		(struct virtio_msg_transport_device *vmdev, u64 *features)
{
	if (!vmdev || !features)
		return;

	for (int i = 0; i < VIRTIO_FEATURES_U64S; i++)
		features[i] |= vmdev->required_features[i];
}

static int
virtio_msg_transport_verify_required_features
		(struct virtio_msg_transport_device *vmdev)
{
	if (!vmdev)
		return -EINVAL;

	for (int word = 0; word < VIRTIO_FEATURES_U64S; word++) {
		u64 required = vmdev->required_features[word];

		while (required) {
			u32 bit = __ffs64(required);

			if (!virtio_has_feature(&vmdev->vdev, bit + word * 64))
				return -EINVAL;
			required &= ~BIT_ULL(bit);
		}
	}

	return 0;
}

static int
virtio_msg_get_features_locked(struct virtio_msg_transport_device *vmdev,
			       struct virtio_msg_transport *transport)
{
	struct virtio_msg_get_device_features_req request_payload;
	struct virtio_msg_get_device_features_resp *response_payload;
	struct virtio_msg *response;
	u64 features[VIRTIO_FEATURES_U64S];
	u32 num_blocks;
	int ret;

	virtio_features_zero(features);
	num_blocks = DIV_ROUND_UP(transport->num_feature_bits, 32);
	if (!num_blocks)
		goto out_copy_features;

	request_payload.block_index = 0;
	request_payload.num_blocks = cpu_to_le32(num_blocks);

	ret = virtio_msg_send(vmdev, VIRTIO_MSG_GET_DEVICE_FEATURES,
			      &request_payload, sizeof(request_payload),
			      sizeof(*response_payload) +
			      sizeof(response_payload->features[0]) * num_blocks,
			      &response);
	if (ret)
		return ret;

	response_payload = virtio_msg_payload(response);
	if (le32_to_cpu(response_payload->block_index) != 0 ||
	    le32_to_cpu(response_payload->num_blocks) != num_blocks)
		return -EPROTO;

	for (u32 block = 0; block < num_blocks; block++) {
		u32 word = block / 2;
		u32 shift = (block % 2) * 32;

		features[word] |= (u64)le32_to_cpu(response_payload->features[block]) << shift;
	}

out_copy_features:
	virtio_msg_transport_apply_required_features(vmdev, features);
	virtio_features_copy(transport->device_features, features);
	return 0;
}

static int
virtio_msg_set_features_locked(struct virtio_msg_transport_device *vmdev,
			       struct virtio_msg_transport *transport)
{
	struct {
		__le32 block_index;
		__le32 num_blocks;
		__le32 features[VIRTIO_FEATURES_U64S * 2];
	} buf = {};
	struct virtio_msg_set_driver_features *request_payload = (void *)&buf;
	size_t payload_len;
	u32 num_blocks;

	num_blocks = DIV_ROUND_UP(transport->num_feature_bits, 32);
	if (!num_blocks)
		return 0;

	payload_len = struct_size(request_payload, features, num_blocks);
	request_payload->block_index = 0;
	request_payload->num_blocks = cpu_to_le32(num_blocks);
	for (u32 block = 0; block < num_blocks; block++) {
		u32 word = block / 2;
		u32 shift = (block % 2) * 32;

		request_payload->features[block] =
			cpu_to_le32((u32)(vmdev->vdev.features_array[word] >> shift));
	}

	return virtio_msg_send(vmdev, VIRTIO_MSG_SET_DRIVER_FEATURES,
			       request_payload, payload_len, 0, NULL);
}

static bool virtio_msg_can_sleep(void)
{
#ifdef CONFIG_PREEMPT_COUNT
	return preemptible();
#else
	return in_task() && !irqs_disabled();
#endif
}

static int
virtio_msg_get_config_locked(struct virtio_msg_transport_device *vmdev,
			     struct virtio_msg_transport *transport,
			     u32 offset, void *buf, u32 len)
{
	struct virtio_msg_get_config request_payload;
	struct virtio_msg_get_config_resp *response_payload;
	struct virtio_msg *response;
	u8 *dst = buf;
	u32 expected_generation = 0;
	u32 max_chunk;
	u32 retries = 0;
	int payload_capacity;
	int ret;

	ret = virtio_msg_transport_validate_config_range(transport, offset, len);
	if (ret)
		return ret;
	if (!buf && len)
		return -EINVAL;

	if (!len)
		return 0;

	payload_capacity = virtio_msg_payload_capacity(transport);
	if (payload_capacity < 0)
		return payload_capacity;
	if (payload_capacity <= sizeof(*response_payload))
		return -EMSGSIZE;

	max_chunk = payload_capacity - sizeof(*response_payload);
	for (;;) {
		u32 done = 0;
		bool have_generation = false;
		bool retry = false;

		while (done < len) {
			u32 chunk = min(len - done, max_chunk);
			u32 generation;

			request_payload.offset = cpu_to_le32(offset + done);
			request_payload.length = cpu_to_le32(chunk);

			ret = virtio_msg_send(vmdev, VIRTIO_MSG_GET_CONFIG,
					      &request_payload, sizeof(request_payload),
					      sizeof(*response_payload) + chunk,
					      &response);
			if (ret)
				return ret;

			response_payload = virtio_msg_payload(response);
			if (response_payload->offset != request_payload.offset ||
			    response_payload->length != request_payload.length)
				return -EPROTO;

			generation = le32_to_cpu(response_payload->generation);
			if (!have_generation) {
				expected_generation = generation;
				have_generation = true;
			} else if (generation != expected_generation) {
				if (retries++ >= VIRTIO_MSG_GET_CONFIG_RETRY_BUDGET)
					return -EAGAIN;
				retry = true;
				break;
			}

			memcpy(dst + done, response_payload->config, chunk);
			done += chunk;
		}

		if (retry)
			continue;

		WRITE_ONCE(transport->config_generation, expected_generation);
		break;
	}

	ret = virtio_msg_transport_config_update_range(transport, offset, dst,
						       len);
	if (ret)
		return ret;

	virtio_msg_transport_config_warn_clear(transport);

	return 0;
}

static int
virtio_msg_set_config_raw_locked(struct virtio_msg_transport_device *vmdev,
				 struct virtio_msg_transport *transport,
				 u32 offset, const void *buf, u32 len)
{
	struct virtio_msg_set_config_resp *response_payload;
	struct virtio_msg_set_config *request_payload;
	struct virtio_msg *response;
	const u8 *src = buf;
	u32 response_length;
	u32 max_chunk;
	u32 done = 0;
	size_t payload_len;
	int payload_capacity;
	int ret;

	ret = virtio_msg_transport_validate_config_range(transport, offset, len);
	if (ret)
		return ret;
	if (!buf && len)
		return -EINVAL;

	if (!len)
		return 0;

	payload_capacity = virtio_msg_payload_capacity(transport);
	if (payload_capacity < 0)
		return payload_capacity;
	if (payload_capacity <= sizeof(*request_payload))
		return -EMSGSIZE;

	max_chunk = payload_capacity - sizeof(*request_payload);
	request_payload = kzalloc(struct_size(request_payload, config, max_chunk),
				  GFP_KERNEL);
	if (!request_payload)
		return -ENOMEM;

	while (done < len) {
		u32 chunk = min(len - done, max_chunk);

		payload_len = struct_size(request_payload, config, chunk);
		request_payload->generation =
			cpu_to_le32(READ_ONCE(transport->config_generation));
		request_payload->offset = cpu_to_le32(offset + done);
		request_payload->length = cpu_to_le32(chunk);
		memcpy(request_payload->config, src + done, chunk);

		ret = virtio_msg_send(vmdev, VIRTIO_MSG_SET_CONFIG,
				      request_payload, payload_len,
				      sizeof(*response_payload), &response);
		if (ret)
			goto out_free;

		response_payload = virtio_msg_payload(response);
		if (response_payload->offset != cpu_to_le32(offset + done)) {
			ret = -EPROTO;
			goto out_free;
		}
		response_length = le32_to_cpu(response_payload->length);
		if (!response_length) {
			ret = virtio_msg_transport_config_refresh_full(vmdev,
								       transport);
			goto out_free;
		}
		if (response_length > chunk) {
			ret = -EPROTO;
			goto out_free;
		}

		WRITE_ONCE(transport->config_generation,
			   le32_to_cpu(response_payload->generation));
		done += response_length;
	}

	ret = virtio_msg_transport_config_update_range(transport, offset, src,
						       len);
	if (!ret)
		virtio_msg_transport_config_warn_clear(transport);

out_free:
	kfree(request_payload);
	return ret;
}

static int
virtio_msg_transport_config_refresh_full(struct virtio_msg_transport_device *vmdev,
					 struct virtio_msg_transport *transport)
{
	int ret;

	if (!vmdev || !transport)
		return -EINVAL;

	if (!transport->config_size) {
		ret = virtio_msg_transport_config_update_range(transport, 0,
							       NULL, 0);
		if (ret)
			return ret;

		virtio_msg_transport_config_warn_clear(transport);
		return 0;
	}

	if (!transport->config_shadow)
		return -ENODATA;

	ret = virtio_msg_get_config_locked(vmdev, transport, 0,
					   transport->config_shadow,
					   transport->config_size);
	if (ret)
		return ret;

	virtio_msg_transport_config_warn_clear(transport);

	return 0;
}

static int
virtio_msg_get_status_locked(struct virtio_msg_transport_device *vmdev,
			     struct virtio_msg_transport *transport)
{
	struct virtio_msg_get_device_status_resp *response_payload;
	struct virtio_msg *response;
	int ret;

	ret = virtio_msg_send(vmdev, VIRTIO_MSG_GET_DEVICE_STATUS, NULL, 0,
			      sizeof(*response_payload), &response);
	if (ret)
		return ret;

	response_payload = virtio_msg_payload(response);
	WRITE_ONCE(transport->cached_status,
		   le32_to_cpu(response_payload->status));
	return 0;
}

static int
virtio_msg_set_status_locked(struct virtio_msg_transport_device *vmdev,
			     struct virtio_msg_transport *transport,
			     u8 status)
{
	struct virtio_msg_set_device_status_resp *response_payload;
	struct virtio_msg_set_device_status request_payload;
	struct virtio_msg *response;
	int ret;

	request_payload.status = cpu_to_le32(status);
	vm_trace(&vmdev->vdev.dev, "set status 0x%02x\n", status);
	ret = virtio_msg_send(vmdev, VIRTIO_MSG_SET_DEVICE_STATUS,
			      &request_payload, sizeof(request_payload),
			      sizeof(*response_payload), &response);
	if (ret)
		return ret;

	response_payload = virtio_msg_payload(response);
	WRITE_ONCE(transport->cached_status,
		   le32_to_cpu(response_payload->status));
	return 0;
}

static void virtio_msg_get_simple(struct virtio_device *vdev, unsigned int offset,
				  void *buf, unsigned int len)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);
	int ret;

	if (!transport)
		return;

	if (!len)
		return;

	if (!virtio_msg_can_sleep()) {
		ret = virtio_msg_transport_config_read_atomic(transport, offset,
							      buf, len);
		if (!ret)
			virtio_msg_transport_config_warn_maybe_emit(vmdev,
								    transport);
	} else {
		ret = virtio_msg_get_config_locked(vmdev, transport, offset, buf,
						   len);
	}

	if (ret) {
		memset(buf, 0, len);
		virtio_msg_transport_set_fatal(vmdev, transport);
	}
}

static void virtio_msg_set_simple(struct virtio_device *vdev, unsigned int offset,
				  const void *buf, unsigned int len)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);
	int ret;

	if (!transport)
		return;

	if (!len)
		return;

	if (!virtio_msg_can_sleep()) {
		virtio_msg_transport_set_fatal(vmdev, transport);
		return;
	}

	ret = virtio_msg_set_config_raw_locked(vmdev, transport, offset, buf, len);
	if (ret)
		virtio_msg_transport_set_fatal(vmdev, transport);
}

static u32 virtio_msg_generation_simple(struct virtio_device *vdev)
{
	struct virtio_msg_transport *transport = vdev_transport(vdev, NULL);

	if (!transport)
		return 0;

	return READ_ONCE(transport->config_generation);
}

static bool virtio_msg_get_shm_region(struct virtio_device *vdev,
				      struct virtio_shm_region *region, u8 id)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);
	struct virtio_msg_get_shm_resp *response_payload;
	struct virtio_msg_get_shm_req request_payload;
	struct virtio_msg *response;
	int ret;

	if (!transport || !region)
		return false;

	memset(region, 0, sizeof(*region));
	request_payload.index = cpu_to_le32(id);
	ret = virtio_msg_send(vmdev, VIRTIO_MSG_GET_SHM, &request_payload,
			      sizeof(request_payload),
			      sizeof(*response_payload), &response);
	if (ret)
		goto fatal;

	response_payload = virtio_msg_payload(response);
	if (response_payload->index != request_payload.index) {
		ret = -EPROTO;
		goto fatal;
	}

	region->len = le32_to_cpu(response_payload->length);
	if (!region->len)
		return false;

	region->addr = le32_to_cpu(response_payload->address);

	return true;

fatal:
	virtio_msg_transport_set_fatal(vmdev, transport);
	return false;
}

static u8 virtio_msg_get_status(struct virtio_device *vdev)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);
	int ret;

	if (!transport)
		return 0;

	if (virtio_msg_transport_is_unplugged(transport))
		return READ_ONCE(transport->cached_status);

	if (virtio_msg_can_sleep()) {
		ret = virtio_msg_get_status_locked(vmdev, transport);
		if (ret)
			virtio_msg_transport_set_fatal(vmdev, transport);
	}

	return READ_ONCE(transport->cached_status);
}

static void virtio_msg_set_status(struct virtio_device *vdev, u8 status)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);

	if (!transport)
		return;

	if (virtio_msg_transport_is_unplugged(transport)) {
		WRITE_ONCE(transport->cached_status, status);
		return;
	}

	if (virtio_msg_set_status_locked(vmdev, transport, status))
		virtio_msg_transport_set_fatal(vmdev, transport);
}

static void virtio_msg_reset(struct virtio_device *vdev)
{
	virtio_msg_set_status(vdev, 0);
}

struct virtio_msg_vq_info {
	u32 max_size;
	u32 size;
	u64 desc_addr;
	u64 driver_addr;
	u64 device_addr;
};

static bool virtio_msg_queue_event_avail(struct virtqueue *vq)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport;
	struct virtio_msg_event_avail *event;
	struct virtio_msg *request;
	u8 request_buf[sizeof(*request) + sizeof(*event)];
	u32 avail_idx = 0;
	u32 notify_data;
	int ret;

	if (!vq || !vq->vdev)
		return false;

	vmdev = to_virtio_msg_transport_device(vq->vdev);
	if (!vmdev || !vmdev->provider_ops || !vmdev->provider_ops->send)
		return false;

	transport = virtio_msg_transport_priv(vmdev);
	if (!transport || virtio_msg_transport_is_fatal(transport))
		return false;

	memset(request_buf, 0, sizeof(request_buf));
	request = (struct virtio_msg *)request_buf;
	virtio_msg_prepare(request, VIRTIO_MSG_EVENT_AVAIL,
			   VIRTIO_MSG_TOKEN_EVENT, sizeof(*event));
	request->dev_num = cpu_to_le16(transport->dev_num);
	event = virtio_msg_payload(request);
	event->vq_index = cpu_to_le32(vq->index);

	if (__virtio_test_bit(&vmdev->vdev, VIRTIO_F_NOTIFICATION_DATA)) {
		notify_data = vring_notification_data(vq);
		avail_idx = notify_data >> 16;

		/*
		 * For packed rings this carries next offset [30:0] and wrap in
		 * bit 31.
		 */
		if (virtio_has_feature(&vmdev->vdev, VIRTIO_F_RING_PACKED)) {
			u32 wrap = avail_idx & BIT(VRING_PACKED_EVENT_F_WRAP_CTR);

			avail_idx &= GENMASK(VRING_PACKED_EVENT_F_WRAP_CTR - 1, 0);
			if (wrap)
				avail_idx |= BIT(31);
		}
	}
	event->next_offset = cpu_to_le32(avail_idx);
	vm_trace(&vmdev->vdev.dev,
		 "tx EVENT_AVAIL: dev=%u vq=%u next=%u\n",
		 transport->dev_num, vq->index, avail_idx);

	ret = virtio_msg_bus_send(vmdev, request, NULL,
				  VIRTIO_MSG_BUS_SEND_F_NONBLOCK);
		if (ret == -ENOSPC || ret == -EAGAIN) {
			vm_trace(&vmdev->vdev.dev,
				 "tx EVENT_AVAIL deferred: dev=%u vq=%u next=%u ret=%d\n",
				 transport->dev_num, vq->index, avail_idx, ret);
		vm_warn_rl(&vmdev->vdev.dev,
			   "dropping EVENT_AVAIL: transient provider backpressure (%d)\n",
			   ret);
		return false;
	}
	if (ret)
		vm_trace(&vmdev->vdev.dev,
			 "tx EVENT_AVAIL failed: dev=%u vq=%u next=%u ret=%d\n",
			 transport->dev_num, vq->index, avail_idx, ret);

	return ret == 0;
}

static bool virtio_msg_notify(struct virtqueue *vq)
{
	return virtio_msg_queue_event_avail(vq);
}

static bool virtio_msg_notify_with_data(struct virtqueue *vq)
{
	return virtio_msg_queue_event_avail(vq);
}

static int virtio_msg_vq_get(struct virtio_msg_transport_device *vmdev,
			     u32 index, struct virtio_msg_vq_info *info)
{
	struct virtio_msg_get_vqueue request_payload;
	struct virtio_msg_get_vqueue_resp *response_payload;
	struct virtio_msg *response;
	int ret;

	if (!vmdev || !info)
		return -EINVAL;

	request_payload.index = cpu_to_le32(index);
	ret = virtio_msg_send(vmdev, VIRTIO_MSG_GET_VQUEUE,
			      &request_payload, sizeof(request_payload),
			      sizeof(*response_payload), &response);
	if (ret)
		return ret;

	response_payload = virtio_msg_payload(response);
	if (le32_to_cpu(response_payload->index) != index ||
	    le32_to_cpu(response_payload->reserved) != 0)
		return -EPROTO;

	info->max_size = le32_to_cpu(response_payload->max_size);
	info->size = le32_to_cpu(response_payload->size);
	info->desc_addr = le64_to_cpu(response_payload->descriptor_addr);
	info->driver_addr = le64_to_cpu(response_payload->driver_addr);
	info->device_addr = le64_to_cpu(response_payload->device_addr);

	if (!info->max_size)
		return -ENOENT;
	if (info->size > info->max_size)
		return -EPROTO;
	if (!info->size &&
	    (info->desc_addr || info->driver_addr || info->device_addr))
		return -EPROTO;
	if (info->size &&
	    (!info->desc_addr || !info->driver_addr || !info->device_addr))
		return -EPROTO;

	return 0;
}

static int virtio_msg_vq_set(struct virtio_msg_transport_device *vmdev,
			     struct virtqueue *vq, u32 index, u32 max_size)
{
	struct virtio_msg_set_vqueue request_payload;
	u32 size;

	if (!vmdev || !vq)
		return -EINVAL;

	size = virtqueue_get_vring_size(vq);
	if (!size || size > max_size)
		return -EINVAL;

	request_payload.index = cpu_to_le32(index);
	request_payload.unused = 0;
	request_payload.size = cpu_to_le32(size);
	request_payload.reserved = 0;
	request_payload.descriptor_addr =
		cpu_to_le64(virtqueue_get_desc_addr(vq));
	request_payload.driver_addr = cpu_to_le64(virtqueue_get_avail_addr(vq));
	request_payload.device_addr = cpu_to_le64(virtqueue_get_used_addr(vq));

	return virtio_msg_send(vmdev, VIRTIO_MSG_SET_VQUEUE, &request_payload,
			       sizeof(request_payload), 0, NULL);
}

static int virtio_msg_vq_reset(struct virtio_msg_transport_device *vmdev,
			       u32 index)
{
	struct virtio_msg_reset_vqueue request_payload;

	if (!vmdev)
		return -EINVAL;

	request_payload.index = cpu_to_le32(index);
	return virtio_msg_send(vmdev, VIRTIO_MSG_RESET_VQUEUE,
			       &request_payload, sizeof(request_payload),
			       0, NULL);
}

static struct virtqueue *
virtio_msg_setup_vq(struct virtio_msg_transport_device *vmdev, u32 index,
		    void (*callback)(struct virtqueue *vq), const char *name,
		    bool ctx)
{
	struct virtio_msg_vq_info info;
	struct virtio_msg_vq_info verify;
	union virtio_map map;
	struct virtqueue *vq;
	bool (*notify)(struct virtqueue *vq);
	u64 desc_addr;
	u64 driver_addr;
	u64 device_addr;
	u32 size;
	int ret;

	if (!vmdev || !name)
		return ERR_PTR(-EINVAL);

	if (__virtio_test_bit(&vmdev->vdev, VIRTIO_F_NOTIFICATION_DATA))
		notify = virtio_msg_notify_with_data;
	else
		notify = virtio_msg_notify;

	ret = virtio_msg_vq_get(vmdev, index, &info);
	if (ret)
		return ERR_PTR(ret);

	vm_trace(&vmdev->vdev.dev, "setup vq index=%u max_size=%u\n",
		 index, info.max_size);
	/* Endpoint queue must be inactive before transport setup. */
	if (info.size || info.desc_addr || info.driver_addr || info.device_addr)
		return ERR_PTR(-EBUSY);

	map.dma_dev = vmdev->vdev.vmap.dma_dev ?
		      vmdev->vdev.vmap.dma_dev :
		      vmdev->vdev.dev.parent;
	vq = vring_create_virtqueue_map(index, info.max_size, PAGE_SIZE,
					&vmdev->vdev, true, true, ctx,
					notify, callback, name, map);
	if (!vq)
		return ERR_PTR(-ENOMEM);

	vq->num_max = info.max_size;
	size = virtqueue_get_vring_size(vq);
	desc_addr = virtqueue_get_desc_addr(vq);
	driver_addr = virtqueue_get_avail_addr(vq);
	device_addr = virtqueue_get_used_addr(vq);

	ret = virtio_msg_vq_set(vmdev, vq, index, info.max_size);
	if (ret)
		goto out_reset_and_del_vq;

	ret = virtio_msg_vq_get(vmdev, index, &verify);
	if (ret)
		goto out_reset_and_del_vq;

	if (verify.size != size ||
	    verify.desc_addr != desc_addr ||
	    verify.driver_addr != driver_addr ||
	    verify.device_addr != device_addr) {
		vm_trace(&vmdev->vdev.dev, "setup vq %u verify mismatch\n", index);
		ret = -EIO;
		goto out_reset_and_del_vq;
	}

	return vq;

out_reset_and_del_vq:
	vm_trace(&vmdev->vdev.dev, "setup vq %u error ret=%d\n", index, ret);
	/*
	 * Best effort rollback: SET_VQUEUE may have partially programmed the
	 * endpoint even when setup is reported as failed.
	 */
	if (virtio_has_feature(&vmdev->vdev, VIRTIO_F_RING_RESET) &&
	    virtio_msg_vq_reset(vmdev, index))
		vm_warn_rl(&vmdev->vdev.dev,
			   "RESET_VQUEUE failed for setup rollback on vq %u\n",
			   index);

	vring_del_virtqueue(vq);
	return ERR_PTR(ret);
}

static void virtio_msg_del_vqs(struct virtio_device *vdev);

static int virtio_msg_find_vqs(struct virtio_device *vdev, unsigned int nvqs,
			       struct virtqueue *vqs[],
			       struct virtqueue_info vqs_info[],
			       struct irq_affinity *desc)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);
	int queue_idx = 0;
	int i;
	int ret;

	if (!vmdev || !transport)
		return -ENODEV;

	virtio_msg_transport_quiesce_events(transport);
	(void)desc;

	if (!nvqs)
		goto out_resume_events;

	for (i = 0; i < nvqs; i++) {
		struct virtqueue_info *vqi = &vqs_info[i];

		if (!vqi->name) {
			vqs[i] = NULL;
			continue;
		}

		if (queue_idx >= transport->max_vq_count) {
			ret = -EINVAL;
			goto out_del_vqs;
		}

		vqs[i] = virtio_msg_setup_vq(vmdev, queue_idx, vqi->callback,
					     vqi->name, vqi->ctx);
		if (IS_ERR(vqs[i])) {
			ret = PTR_ERR(vqs[i]);
			goto out_del_vqs;
		}

		queue_idx++;
	}

out_resume_events:
	virtio_msg_transport_resume_events(transport, true);
	return 0;

out_del_vqs:
	virtio_msg_del_vqs(vdev);
	return ret;
}

static void virtio_msg_del_vqs(struct virtio_device *vdev)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);
	struct virtqueue *vq, *n;

	virtio_msg_transport_stop_events(transport);
	vm_trace(&vdev->dev, "del vqs\n");

	list_for_each_entry_safe(vq, n, &vdev->vqs, list) {
		int ret;

		if (!virtio_msg_transport_is_unplugged(transport) &&
		    virtio_has_feature(vdev, VIRTIO_F_RING_RESET)) {
			ret = virtio_msg_vq_reset(vmdev, vq->index);
			if (ret)
				vm_warn_rl(&vdev->dev,
					   "RESET_VQUEUE failed for vq %u: %d\n",
					   vq->index, ret);
		}

		vring_del_virtqueue(vq);
	}
}

static void virtio_msg_synchronize_cbs(struct virtio_device *vdev)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);

	if (vmdev->provider_ops && vmdev->provider_ops->synchronize_cbs)
		vmdev->provider_ops->synchronize_cbs(vmdev);
	else
		synchronize_rcu();

	if (transport)
		flush_work(&transport->event_work);
}

static u64 virtio_msg_get_features(struct virtio_device *vdev)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);
	int ret;

	if (!transport)
		return 0;

	ret = virtio_msg_get_features_locked(vmdev, transport);
	if (ret) {
		virtio_msg_transport_set_fatal(vmdev, transport);
		return 0;
	}

	return transport->device_features[0];
}

static void virtio_msg_get_extended_features(struct virtio_device *vdev,
					     u64 *features)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);
	int ret;

	if (!features)
		return;

	if (!transport) {
		virtio_features_zero(features);
		return;
	}

	ret = virtio_msg_get_features_locked(vmdev, transport);
	if (ret) {
		virtio_msg_transport_set_fatal(vmdev, transport);
		virtio_features_zero(features);
		return;
	}

	virtio_features_copy(features, transport->device_features);
}

static int virtio_msg_finalize_features(struct virtio_device *vdev)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);
	int ret;

	if (!transport)
		return -ENODEV;

	ret = virtio_msg_set_features_locked(vmdev, transport);
	if (ret) {
		virtio_msg_transport_set_fatal(vmdev, transport);
		return ret;
	}

	ret = virtio_msg_transport_verify_required_features(vmdev);
	if (ret) {
		virtio_msg_transport_set_fatal(vmdev, transport);
		return ret;
	}

	return 0;
}

static const char *virtio_msg_bus_name(struct virtio_device *vdev)
{
	struct virtio_msg_transport_device *vmdev =
		to_virtio_msg_transport_device(vdev);

	if (vmdev->provider_name)
		return vmdev->provider_name;

	return "virtio_msg";
}

static const struct virtio_config_ops virtio_msg_config_ops = {
	.get			= virtio_msg_get_simple,
	.set			= virtio_msg_set_simple,
	.generation		= virtio_msg_generation_simple,
	.get_shm_region		= virtio_msg_get_shm_region,
	.get_status		= virtio_msg_get_status,
	.set_status		= virtio_msg_set_status,
	.reset			= virtio_msg_reset,
	.find_vqs		= virtio_msg_find_vqs,
	.del_vqs		= virtio_msg_del_vqs,
	.synchronize_cbs	= virtio_msg_synchronize_cbs,
	.get_features		= virtio_msg_get_features,
	.get_extended_features	= virtio_msg_get_extended_features,
	.finalize_features	= virtio_msg_finalize_features,
	.bus_name		= virtio_msg_bus_name,
};

static void virtio_msg_release_dev(struct device *dev)
{
	struct virtio_device *vdev = dev_to_virtio(dev);
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);

	if (!transport)
		return;

	mutex_lock(&transport->request_lock);
	virtio_msg_transport_cleanup_runtime(transport);
	transport->prepared = false;
	transport->registered = false;
	transport->fatal = false;
	mutex_unlock(&transport->request_lock);

	complete_all(&transport->release_done);
}

void virtio_msg_prepare(struct virtio_msg *vmsg, u8 msg_id,
			u16 token, u16 payload_size)
{
	if (!vmsg)
		return;

	vmsg->type = VIRTIO_MSG_TYPE_TRANSPORT | VIRTIO_MSG_TYPE_REQUEST;
	vmsg->msg_id = msg_id;
	vmsg->dev_num = cpu_to_le16(0);
	vmsg->token = cpu_to_le16(token);
	vmsg->msg_size = cpu_to_le16(sizeof(*vmsg) + payload_size);
}
EXPORT_SYMBOL_GPL(virtio_msg_prepare);

void virtio_msg_transport_set_required_features(struct virtio_msg_transport_device *vmdev,
						const u64 required_features[VIRTIO_FEATURES_U64S])
{
	if (!vmdev)
		return;

	if (!required_features) {
		virtio_features_zero(vmdev->required_features);
		return;
	}

	virtio_features_copy(vmdev->required_features, required_features);
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_set_required_features);

int virtio_msg_transport_handle_device_event(struct virtio_msg_transport_device *vmdev,
					     const struct virtio_msg *vmsg)
{
	const struct virtio_msg_event_config *cfg;
	const struct virtio_msg_event_used *used;
	struct virtio_msg_transport *transport;
	unsigned long flags;
	u16 msg_size;
	u32 config_offset = 0;
	u32 config_len;
	u32 depth;
	u32 generation = 0;
	u32 device_status = 0;
	u32 vq_index;
	u8 warn_reasons = 0;
	u32 warn_generation = 0;
	bool queue_work = false;
	int ret;

	if (!vmdev || !vmsg)
		return -EINVAL;

	transport = virtio_msg_transport_priv(vmdev);
	if (!transport || !transport->registered)
		return -ENODEV;
	if (virtio_msg_transport_is_fatal(transport))
		return -EIO;

	msg_size = le16_to_cpu(vmsg->msg_size);
	if (msg_size < sizeof(*vmsg) || msg_size > transport->msg_size)
		return -EMSGSIZE;
	if (vmsg->type != (VIRTIO_MSG_TYPE_TRANSPORT |
			   VIRTIO_MSG_TYPE_REQUEST))
		return -EINVAL;
	if (le16_to_cpu(vmsg->dev_num) != transport->dev_num)
		return -EINVAL;

	switch (vmsg->msg_id) {
	case VIRTIO_MSG_EVENT_CONFIG:
		cfg = (const struct virtio_msg_event_config *)vmsg->payload;
		if (msg_size < sizeof(*vmsg) + sizeof(*cfg))
			return -EMSGSIZE;
		config_offset = le32_to_cpu(cfg->offset);
		config_len = le32_to_cpu(cfg->length);
		generation = le32_to_cpu(cfg->generation);
		device_status = le32_to_cpu(cfg->device_status);
		if (msg_size != sizeof(*vmsg) + sizeof(*cfg) + config_len)
			return -EMSGSIZE;
		ret = virtio_msg_validate_event_config_range(transport, cfg);
		if (ret)
			return -EINVAL;
		if (config_len > transport->pending_config_capacity)
			return -EMSGSIZE;
		break;
	case VIRTIO_MSG_EVENT_USED:
		used = (const struct virtio_msg_event_used *)vmsg->payload;
		if (msg_size != sizeof(*vmsg) + sizeof(*used))
			return -EMSGSIZE;
		vq_index = le32_to_cpu(used->vq_index);
		if (vq_index >= transport->max_vq_count ||
		    !transport->pending_used_bitmap)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (vmsg->msg_id == VIRTIO_MSG_EVENT_USED)
		vm_trace(&vmdev->vdev.dev,
			 "rx EVENT_USED: dev=%u vq=%u\n",
			 le16_to_cpu(vmsg->dev_num), vq_index);
	else
		vm_trace(&vmdev->vdev.dev,
			 "rx EVENT_CONFIG: dev=%u generation=%u offset=%u len=%u\n",
			 le16_to_cpu(vmsg->dev_num), generation,
			 config_offset, config_len);
	spin_lock_irqsave(&transport->event_lock, flags);
	if (!transport->accept_events) {
		if (vmsg->msg_id == VIRTIO_MSG_EVENT_CONFIG) {
			warn_reasons |= VIRTIO_MSG_CONFIG_WARN_EVENT_DROPPED;
			warn_generation = generation;
		}
		spin_unlock_irqrestore(&transport->event_lock, flags);
		vm_trace(&vmdev->vdev.dev,
			 "drop event: msg_id=0x%02x dev=%u (events not accepted)\n",
			 vmsg->msg_id, le16_to_cpu(vmsg->dev_num));
		virtio_msg_transport_config_warn_mark(transport, warn_reasons,
						      warn_generation);
		return -ESHUTDOWN;
	}

	if (vmsg->msg_id == VIRTIO_MSG_EVENT_CONFIG) {
		bool had_pending = transport->config_pending;

		if (generation < READ_ONCE(transport->config_generation) ||
		    (transport->config_pending &&
		     generation < transport->pending_config_generation)) {
			warn_reasons |=
				VIRTIO_MSG_CONFIG_WARN_GENERATION_COALESCED;
			warn_generation = max(warn_generation, generation);
		} else {
			if (transport->config_pending &&
			    generation != transport->pending_config_generation) {
				warn_reasons |=
					VIRTIO_MSG_CONFIG_WARN_GENERATION_COALESCED;
				warn_generation = max(warn_generation,
						      max(generation,
							  transport->pending_config_generation));
			}
			transport->config_pending = true;
			if (!had_pending) {
				transport->pending_config_offset = config_offset;
				transport->pending_config_len = config_len;
			} else if (!transport->pending_config_len) {
				/* Keep pending full refresh sticky until worker runs. */
			} else if (!config_len) {
				transport->pending_config_offset = config_offset;
				transport->pending_config_len = config_len;
			} else if (transport->pending_config_offset != config_offset ||
				   transport->pending_config_len != config_len) {
				/* Coalesced partial ranges cannot be represented safely. */
				warn_reasons |=
					VIRTIO_MSG_CONFIG_WARN_GENERATION_COALESCED;
				warn_generation = max(warn_generation, generation);
				transport->pending_config_offset = 0;
				transport->pending_config_len = 0;
			}
			transport->pending_config_generation = generation;
			transport->pending_device_status = device_status;
			if (config_len &&
			    transport->pending_config_offset == config_offset &&
			    transport->pending_config_len == config_len)
				memcpy(transport->pending_config_data, cfg->data,
				       config_len);
		}
	} else {
		used = (const struct virtio_msg_event_used *)vmsg->payload;
		set_bit(le32_to_cpu(used->vq_index), transport->pending_used_bitmap);
	}

	depth = virtio_msg_transport_pending_depth_locked(transport);
	if (transport->events_dispatch_ready && depth &&
	    !transport->event_work_queued) {
		transport->event_work_queued = true;
		queue_work = true;
	}
	spin_unlock_irqrestore(&transport->event_lock, flags);

	virtio_msg_transport_config_warn_mark(transport, warn_reasons,
					      warn_generation);

	if (queue_work) {
		vm_trace(&vmdev->vdev.dev,
			 "queue event worker: msg_id=0x%02x dev=%u depth=%u\n",
			 vmsg->msg_id, le16_to_cpu(vmsg->dev_num), depth);
		schedule_work(&transport->event_work);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_handle_device_event);

int virtio_msg_transport_attach_provider(struct virtio_msg_transport_device *vmdev,
					 const struct virtio_msg_bus_provider_ops *provider_ops,
					 const char *provider_name,
					 void *provider_data)
{
	struct virtio_msg_transport *transport;
	int ret;

	if (!vmdev || !provider_name || !provider_name[0])
		return -EINVAL;

	ret = virtio_msg_validate_provider_ops(provider_ops);
	if (ret)
		return ret;

	if (vmdev->private)
		return -EBUSY;

	transport = kzalloc(sizeof(*transport), GFP_KERNEL);
	if (!transport)
		return -ENOMEM;

	mutex_init(&transport->request_lock);
	init_completion(&transport->release_done);
	spin_lock_init(&transport->event_lock);
	spin_lock_init(&transport->config_lock);
	INIT_WORK(&transport->event_work, virtio_msg_event_worker);
	transport->vmdev = vmdev;
	virtio_features_zero(transport->device_features);

	vmdev->provider_ops = provider_ops;
	vmdev->provider_name = provider_name;
	vmdev->provider_data = provider_data;
	vmdev->private = transport;
	vm_info(&vmdev->vdev.dev, "provider attached: bus='%s' dev=%u\n",
		provider_name, vmdev->dev_num);
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_attach_provider);

void virtio_msg_transport_detach_provider(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_transport *transport;

	if (!vmdev)
		return;

	transport = virtio_msg_transport_priv(vmdev);
	if (!transport)
		return;

	if (WARN_ON(transport->registered))
		return;

	vm_info(&vmdev->vdev.dev, "provider detached: bus='%s' dev=%u\n",
		vmdev->provider_name ? vmdev->provider_name : "?",
		vmdev->dev_num);
	virtio_msg_transport_cleanup_runtime(transport);
	transport->prepared = false;
	transport->vmdev = NULL;
	kfree(transport);
	vmdev->private = NULL;
	vmdev->provider_ops = NULL;
	vmdev->provider_name = NULL;
	vmdev->provider_data = NULL;
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_detach_provider);

int virtio_msg_transport_prepare_device(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_transport *transport = virtio_msg_transport_priv(vmdev);
	struct virtio_msg_provider_caps provider_caps;
	int ret;

	if (!vmdev || !transport || !vmdev->provider_ops ||
	    !vmdev->provider_name)
		return -EINVAL;
	if (transport->prepared || transport->registered)
		return -EBUSY;

	memset(&provider_caps, 0, sizeof(provider_caps));
	ret = vmdev->provider_ops->get_caps(vmdev, &provider_caps);
	if (ret)
		return ret;

	ret = virtio_msg_validate_provider_caps(&provider_caps);
	if (ret)
		return ret;

	ret = virtio_msg_transport_alloc_runtime(transport, &provider_caps);
	if (ret)
		return ret;
	transport->dev_num = vmdev->dev_num;

	ret = virtio_msg_get_device_info(vmdev, transport);
	if (ret) {
		vm_warn_rl(&vmdev->vdev.dev,
			   "device info failed: dev=%u ret=%d\n",
			   vmdev->dev_num, ret);
		virtio_msg_transport_cleanup_runtime(transport);
		return ret;
	}

	ret = virtio_msg_transport_config_alloc(transport);
	if (ret) {
		virtio_msg_transport_cleanup_runtime(transport);
		return ret;
	}

	ret = virtio_msg_transport_config_refresh_full(vmdev, transport);
	if (ret) {
		virtio_msg_transport_cleanup_runtime(transport);
		return ret;
	}

	ret = virtio_msg_transport_alloc_event_state(transport);
	if (ret) {
		virtio_msg_transport_cleanup_runtime(transport);
		return ret;
	}

	vmdev->vdev.config = &virtio_msg_config_ops;
	vmdev->vdev.dev.release = virtio_msg_release_dev;
	transport->prepared = true;
	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_prepare_device);

int virtio_msg_transport_register_prepared_device(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_transport *transport = virtio_msg_transport_priv(vmdev);
	int ret;

	if (!vmdev || !transport || !vmdev->provider_ops ||
	    !vmdev->provider_name)
		return -EINVAL;
	if (!transport->prepared || transport->registered)
		return -EINVAL;

	vm_info(&vmdev->vdev.dev, "device register: bus='%s' dev=%u\n",
		vmdev->provider_name, vmdev->dev_num);
	reinit_completion(&transport->release_done);
	transport->registered = true;
	/*
	 * Accept bridge events before register_virtio_device() runs probe so
	 * early EVENT_CONFIG/EVENT_USED messages are queued instead of dropped.
	 * Dispatch stays disabled until the core finishes setting up vqs.
	 */
	virtio_msg_transport_resume_events(transport, false);
	ret = register_virtio_device(&vmdev->vdev);
	if (ret) {
		vm_warn_rl(&vmdev->vdev.dev,
			   "device register failed: dev=%u ret=%d\n",
			   vmdev->dev_num, ret);
		virtio_msg_transport_stop_events(transport);
		transport->registered = false;
		/*
		 * register_virtio_device() initialized the device; caller must
		 * drop the reference on error so our release callback can run.
		 */
		put_device(&vmdev->vdev.dev);
		if (!wait_for_completion_timeout
				(&transport->release_done,
				 msecs_to_jiffies
					(VIRTIO_MSG_REGISTER_RELEASE_TIMEOUT_MS))) {
			vm_warn_rl(&vmdev->vdev.dev,
				   "timed out waiting for failed registration release\n");
			return -ETIMEDOUT;
		}
		return ret;
	}

	/*
	 * Keep async event dispatch enabled once registration succeeds.
	 * Probe/find_vqs may already have queued pending completions.
	 */
	virtio_msg_transport_resume_events(transport, true);

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_register_prepared_device);

void virtio_msg_transport_unprepare_device(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_transport *transport = virtio_msg_transport_priv(vmdev);

	if (!vmdev || !transport || transport->registered || !transport->prepared)
		return;

	mutex_lock(&transport->request_lock);
	virtio_msg_transport_cleanup_runtime(transport);
	transport->prepared = false;
	transport->fatal = false;
	mutex_unlock(&transport->request_lock);
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_unprepare_device);

int virtio_msg_transport_register_device(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_transport *transport = virtio_msg_transport_priv(vmdev);
	int ret;

	if (!vmdev || !transport)
		return -EINVAL;
	if (transport->registered)
		return -EBUSY;

	ret = virtio_msg_transport_prepare_device(vmdev);
	if (ret)
		return ret;

	ret = virtio_msg_transport_register_prepared_device(vmdev);
	if (ret && transport->prepared)
		virtio_msg_transport_unprepare_device(vmdev);

	return ret;
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_register_device);

void virtio_msg_transport_unregister_device(struct virtio_msg_transport_device *vmdev)
{
	struct virtio_msg_transport *transport = virtio_msg_transport_priv(vmdev);

	if (!vmdev || !transport || !transport->registered)
		return;

	vm_info(&vmdev->vdev.dev, "device unregister: bus='%s' dev=%u\n",
		vmdev->provider_name ? vmdev->provider_name : "?",
		vmdev->dev_num);
	virtio_msg_transport_stop_events(transport);
	if (vmdev->provider_ops && vmdev->provider_ops->synchronize_cbs)
		vmdev->provider_ops->synchronize_cbs(vmdev);
	virtio_msg_transport_begin_unplug(vmdev, transport);

	reinit_completion(&transport->release_done);
	unregister_virtio_device(&vmdev->vdev);
	wait_for_completion(&transport->release_done);

	if (vmdev->provider_ops && vmdev->provider_ops->release)
		vmdev->provider_ops->release(vmdev);
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_unregister_device);

MODULE_DESCRIPTION("Virtio message transport bootstrap core");
MODULE_LICENSE("GPL");
