// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message transport core.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 *
 * This unit owns provider attach/register flow, sleepable request/response
 * config and status operations, virtqueue setup, and device event dispatch.
 */

#define pr_fmt(fmt) "virtio-msg-transport: " fmt

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/preempt.h>
#include <linux/ratelimit.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_msg_bus_provider.h>
#include <linux/virtio_msg_protocol.h>
#include <linux/virtio_msg_transport.h>

#define VIRTIO_MSG_RX_TYPE_MASK		(VIRTIO_MSG_TYPE_RESPONSE | \
					 VIRTIO_MSG_TYPE_BUS)

#define VIRTIO_MSG_GET_CONFIG_RETRY_BUDGET	5U
#define VIRTIO_MSG_RESET_POLL_INTERVAL_MS	10U
#define VIRTIO_MSG_RESET_TIMEOUT_MS		10000U
#define VIRTIO_MSG_REGISTER_RELEASE_TIMEOUT_MS	5000U

/**
 * struct virtio_msg_transport - private runtime transport container
 * @vmdev: Back-pointer to the owning transport shell.
 * @request_lock: Serializes blocking transport requests per device.
 * @release_done: Signals completion of release callback teardown.
 * @request: Reusable request frame buffer.
 * @response: Reusable response frame buffer.
 * @device_features: Cached feature blocks read from transport.
 * @msg_size: Negotiated transport frame size.
 * @transport_revision: Negotiated transport revision.
 * @config_size: Reported config space size from DEVICE_INFO.
 * @num_feature_blocks: Reported feature block count from DEVICE_INFO.
 * @max_vq_count: Reported maximum virtqueue count from DEVICE_INFO.
 * @admin_vq_start: Reported admin virtqueue start index from DEVICE_INFO.
 * @admin_vq_count: Reported admin virtqueue count from DEVICE_INFO.
 * @dev_num: Transport device identifier used in frame headers.
 * @config_generation: Last observed config generation.
 * @cached_status: Last observed status byte.
 * @prepared: True while transport-owned runtime is prepared but may not yet be
 *	      registered on the virtio bus.
 * @registered: True while this vmdev is registered on virtio bus.
 * @unplugged: True once unregister starts and teardown must stay local.
 * @fatal: True after transport marks the device failed.
 */
struct virtio_msg_transport {
	struct virtio_msg_transport_device *vmdev;
	/* Serializes blocking request/response exchanges for one vmdev. */
	struct mutex request_lock;
	struct completion release_done;
	struct virtio_msg *request;
	struct virtio_msg *response;
	u64 device_features[VIRTIO_FEATURES_U64S];
	u32 msg_size;
	u32 transport_revision;
	u32 config_size;
	u32 num_feature_blocks;
	u32 max_vq_count;
	u32 admin_vq_start;
	u32 admin_vq_count;
	u16 dev_num;
	u32 config_generation;
	u8 cached_status;
	bool prepared;
	bool registered;
	bool unplugged;
	bool fatal;
};

struct virtio_msg_vq_info {
	u32 max_size;
	u32 size;
	u32 flags;
	u64 desc_addr;
	u64 driver_addr;
	u64 device_addr;
};

/* Forward declarations required by the section ordering below. */
static void virtio_msg_transport_set_fatal(struct virtio_msg_transport_device *vmdev,
					   struct virtio_msg_transport *transport);
static bool virtio_msg_queue_event_avail(struct virtqueue *vq);
static void virtio_msg_del_vqs(struct virtio_device *vdev);

/* === Access helpers === */
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

static bool virtio_msg_transport_is_fatal(const struct virtio_msg_transport *transport)
{
	return transport->fatal;
}

static bool
virtio_msg_transport_is_unplugged(const struct virtio_msg_transport *transport)
{
	return transport->unplugged;
}

/* === Provider lifecycle (Linux-specific) === */
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
	if (!caps)
		return -EINVAL;
	if (caps->revision != VIRTIO_MSG_REVISION_1)
		return -EOPNOTSUPP;
	if (caps->msg_size < VIRTIO_MSG_MIN_SIZE ||
	    caps->msg_size > VIRTIO_MSG_MAX_SIZE)
		return -EMSGSIZE;
	if (caps->transport_features & VIRTIO_MSG_F_STRICT_CONFIG_GENERATION)
		return -EOPNOTSUPP;
	if (caps->transport_features & ~VIRTIO_MSG_TRANSPORT_F_SUPPORTED)
		return -EOPNOTSUPP;

	return 0;
}

static void virtio_msg_transport_cleanup_runtime(struct virtio_msg_transport *transport)
{
	BUILD_BUG_ON(offsetof(struct virtio_msg_transport, request) !=
		     offsetofend(struct virtio_msg_transport, release_done));

	kfree(transport->request);
	kfree(transport->response);
	transport->request = NULL;
	transport->response = NULL;
	virtio_features_zero(transport->device_features);
	transport->msg_size = 0;
	transport->transport_revision = 0;
	transport->config_size = 0;
	transport->num_feature_blocks = 0;
	transport->max_vq_count = 0;
	transport->admin_vq_start = 0;
	transport->admin_vq_count = 0;
	transport->dev_num = 0;
	WRITE_ONCE(transport->config_generation, 0);
	WRITE_ONCE(transport->cached_status, 0);
	transport->prepared = false;
	transport->registered = false;
	transport->unplugged = false;
	transport->fatal = false;
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

/* === Common message format and exchange ===
 * Spec: §Basic Concepts / Common Message Format
 *       §Basic Concepts / Message Ordering
 */
static int virtio_msg_payload_capacity(const struct virtio_msg_transport *transport)
{
	if (transport->msg_size < sizeof(struct virtio_msg))
		return -EMSGSIZE;

	return transport->msg_size - sizeof(struct virtio_msg);
}

static int
virtio_msg_validate_response(struct virtio_msg_transport *transport,
			     const struct virtio_msg *request,
			     const struct virtio_msg *response,
			     size_t min_payload_size)
{
	u16 response_size;
	u8 expected_type;

	expected_type = (request->type & VIRTIO_MSG_RX_TYPE_MASK) |
			VIRTIO_MSG_TYPE_RESPONSE;
	if ((response->type & VIRTIO_MSG_RX_TYPE_MASK) != expected_type)
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

	payload_capacity = virtio_msg_payload_capacity(transport);
	if (payload_capacity < 0)
		return payload_capacity;
	if (request_payload_size > payload_capacity)
		return -EMSGSIZE;
	if (min_response_payload_size > payload_capacity)
		return -EMSGSIZE;

	mutex_lock(&transport->request_lock);
	if (virtio_msg_transport_is_fatal(transport)) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "tx rejected (fatal): msg_id=0x%02x\n", msg_id);
		ret = -EIO;
		goto out_unlock;
	}

	request = transport->request;
	virtio_msg_prepare(request, msg_id, 0, request_payload_size,
			   transport->dev_num);
	if (request_payload_size)
		memcpy(virtio_msg_payload(request), request_payload,
		       request_payload_size);

	memset(transport->response, 0, transport->msg_size);
	dev_dbg(&vmdev->vdev.dev, "tx msg_id=0x%02x dev=%u\n",
		msg_id, transport->dev_num);
	ret = vmdev->provider_ops->send(vmdev, request, transport->response);
	if (ret) {
		dev_warn_ratelimited(&vmdev->vdev.dev, "tx error: msg_id=0x%02x ret=%d\n",
				     msg_id, ret);
		goto out_maybe_fatal;
	}

	ret = virtio_msg_validate_response(transport, request, transport->response,
					   min_response_payload_size);
	if (ret) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
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

/* === Device discovery ===
 * Spec: §Device Discovery
 *       §Device Initialization / Device Information (VIRTIO_MSG_GET_DEVICE_INFO)
 */
static int
virtio_msg_get_device_info(struct virtio_msg_transport_device *vmdev,
			   struct virtio_msg_transport *transport)
{
	struct virtio_msg_get_device_info_resp *response_payload;
	struct virtio_msg *response;
	u32 num_feature_blocks;
	int ret;

	ret = virtio_msg_send(vmdev, VIRTIO_MSG_GET_DEVICE_INFO, NULL, 0,
			      sizeof(*response_payload), &response);
	if (ret)
		return ret;

	response_payload = virtio_msg_payload(response);
	num_feature_blocks = le32_to_cpu(response_payload->num_feature_blocks);
	if (num_feature_blocks > VIRTIO_FEATURES_BITS / 32)
		return -E2BIG;

	transport->config_size = le32_to_cpu(response_payload->config_size);
	transport->num_feature_blocks = num_feature_blocks;
	transport->max_vq_count = le32_to_cpu(response_payload->max_virtqueues);
	transport->admin_vq_start =
		le32_to_cpu(response_payload->admin_vq_start);
	transport->admin_vq_count =
		le32_to_cpu(response_payload->admin_vq_count);
	if ((u64)transport->admin_vq_start + transport->admin_vq_count >
	    transport->max_vq_count)
		return -EINVAL;

	vmdev->vdev.id.device = le32_to_cpu(response_payload->device_id);
	vmdev->vdev.id.vendor = le32_to_cpu(response_payload->vendor_id);
	if (!vmdev->vdev.id.device)
		return -ENODEV;

	dev_dbg(&vmdev->vdev.dev,
		"device info: dev=%u type=%u vendor=%u blocks=%u csz=%u vqs=%u\n",
		 transport->dev_num, vmdev->vdev.id.device,
		 vmdev->vdev.id.vendor, transport->num_feature_blocks,
		 transport->config_size, transport->max_vq_count);
	return 0;
}

/* === Device initialization: feature negotiation ===
 * Spec: §Device Initialization / Feature Negotiation
 *       (VIRTIO_MSG_GET_DEVICE_FEATURES, VIRTIO_MSG_SET_DRIVER_FEATURES)
 */
static int
virtio_msg_transport_verify_required_features
		(struct virtio_msg_transport_device *vmdev)
{
	for (int word = 0; word < VIRTIO_FEATURES_U64S; word++) {
		u64 required = vmdev->required_features[word];

		if ((vmdev->vdev.features_array[word] & required) != required)
			return -ENODEV;
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
	num_blocks = transport->num_feature_blocks;
	if (!num_blocks)
		goto out_copy_features;

	request_payload.block_index = cpu_to_le32(0);
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

		features[word] |=
			(u64)le32_to_cpu(response_payload->features[block]) << shift;
	}

out_copy_features:
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
	} request = {};
	size_t payload_len;
	u32 num_blocks;

	num_blocks = transport->num_feature_blocks;
	if (!num_blocks)
		return 0;

	payload_len = offsetof(typeof(request), features) +
		      num_blocks * sizeof(request.features[0]);
	request.block_index = cpu_to_le32(0);
	request.num_blocks = cpu_to_le32(num_blocks);
	for (u32 block = 0; block < num_blocks; block++) {
		u32 word = block / 2;
		u32 shift = (block % 2) * 32;

		request.features[block] =
			cpu_to_le32((u32)(vmdev->vdev.features_array[word] >>
					  shift));
	}

	return virtio_msg_send(vmdev, VIRTIO_MSG_SET_DRIVER_FEATURES,
			       &request, payload_len, 0, NULL);
}

/* === Device initialization: configuration space ===
 * Spec: §Basic Concepts / Configuration Semantics Profiles
 *       §Device Initialization / Device Configuration
 *       (VIRTIO_MSG_GET_CONFIG, VIRTIO_MSG_SET_CONFIG)
 */
static int
virtio_msg_transport_validate_config_range(const struct virtio_msg_transport *transport,
					   u32 offset, u32 len)
{
	u64 end = (u64)offset + (u64)len;

	if (end > transport->config_size)
		return -EINVAL;

	return 0;
}

static bool virtio_msg_can_sleep(void)
{
	return in_task() && !preempt_count() && !irqs_disabled();
}

static void
virtio_msg_report_nonsleepable_config(struct virtio_device *vdev, const char *op,
				      unsigned int offset, unsigned int len,
				      unsigned long caller)
{
	static DEFINE_RATELIMIT_STATE(rs, DEFAULT_RATELIMIT_INTERVAL,
				      DEFAULT_RATELIMIT_BURST);

	if (!__ratelimit(&rs))
		return;

	dev_warn_ratelimited(&vdev->dev,
			     "config %s from non-sleepable context: offset=%u len=%u caller=%pS task=%s pid=%d preempt_count=0x%x irqs_disabled=%d in_interrupt=%lu\n",
			     op, offset, len, (void *)caller, current->comm,
			     current->pid, preempt_count(), irqs_disabled(),
			     in_interrupt());
	dump_stack();
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
	/*
	 * Retry the whole read up to a fixed budget when chunks observe
	 * different config generations. A continuously mutating device returns
	 * -EAGAIN so callers can back off at a higher level.
	 */
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

	return 0;
}

static int
virtio_msg_set_config_locked(struct virtio_msg_transport_device *vmdev,
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
		/* Strict config generation is not negotiated in revision 1. */
		request_payload->generation = 0;
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
			ret = -EIO;
			goto out_free;
		}
		if (response_length != chunk) {
			ret = -EPROTO;
			goto out_free;
		}

		/*
		 * Ignore optional echoed config bytes. SET_CONFIG only confirms
		 * the accepted range and resulting generation; authoritative
		 * post-write contents must be read through GET_CONFIG.
		 */
		WRITE_ONCE(transport->config_generation,
			   le32_to_cpu(response_payload->generation));
		done += response_length;
	}

	ret = 0;

out_free:
	kfree(request_payload);
	return ret;
}

/* === Device initialization: device status ===
 * Spec: §Device Initialization / Status Information
 *       §Device Operation / Device Reset and Shutdown
 *       (VIRTIO_MSG_GET_DEVICE_STATUS, VIRTIO_MSG_SET_DEVICE_STATUS)
 */
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
	dev_dbg(&vmdev->vdev.dev, "set status 0x%02x\n", status);
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

static int
virtio_msg_reset_locked(struct virtio_msg_transport_device *vmdev,
			struct virtio_msg_transport *transport)
{
	unsigned long deadline;
	int ret;

	ret = virtio_msg_set_status_locked(vmdev, transport, 0);
	if (ret)
		return ret;

	deadline = jiffies + msecs_to_jiffies(VIRTIO_MSG_RESET_TIMEOUT_MS);
	while (READ_ONCE(transport->cached_status)) {
		ret = virtio_msg_get_status_locked(vmdev, transport);
		if (ret)
			return ret;
		if (!READ_ONCE(transport->cached_status))
			return 0;
		if (time_after(jiffies, deadline))
			return -ETIMEDOUT;
		msleep(VIRTIO_MSG_RESET_POLL_INTERVAL_MS);
	}

	return 0;
}

/* === Device initialization: virtqueue configuration ===
 * Spec: §Device Initialization / Virtqueue Configuration
 *       §Device Operation / Virtqueue Changes During Operation
 *       (VIRTIO_MSG_GET_VQUEUE, VIRTIO_MSG_SET_VQUEUE, VIRTIO_MSG_RESET_VQUEUE)
 */
static bool virtio_msg_vq_is_enabled(const struct virtio_msg_vq_info *info)
{
	return info->flags & VIRTIO_MSG_VQUEUE_F_ENABLED;
}

static int virtio_msg_vq_get(struct virtio_msg_transport_device *vmdev,
			     u32 index, struct virtio_msg_vq_info *info)
{
	struct virtio_msg_get_vqueue request_payload;
	struct virtio_msg_get_vqueue_resp *response_payload;
	struct virtio_msg *response;
	int ret;

	request_payload.index = cpu_to_le32(index);
	ret = virtio_msg_send(vmdev, VIRTIO_MSG_GET_VQUEUE,
			      &request_payload, sizeof(request_payload),
			      sizeof(*response_payload), &response);
	if (ret)
		return ret;

	response_payload = virtio_msg_payload(response);
	if (le32_to_cpu(response_payload->index) != index)
		return -EPROTO;

	info->max_size = le32_to_cpu(response_payload->max_size);
	info->size = le32_to_cpu(response_payload->cur_size);
	info->flags = le32_to_cpu(response_payload->flags);
	info->desc_addr = le64_to_cpu(response_payload->descriptor_addr);
	info->driver_addr = le64_to_cpu(response_payload->driver_addr);
	info->device_addr = le64_to_cpu(response_payload->device_addr);

	if (info->flags & ~VIRTIO_MSG_VQUEUE_F_ENABLED)
		return -EPROTO;

	if (!info->max_size) {
		if (info->size || info->flags || info->desc_addr ||
		    info->driver_addr || info->device_addr)
			return -EPROTO;
		return -ENOENT;
	}
	if (info->size > info->max_size)
		return -EPROTO;
	if (!info->size &&
	    (virtio_msg_vq_is_enabled(info) || info->desc_addr ||
	     info->driver_addr || info->device_addr))
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

	size = virtqueue_get_vring_size(vq);
	if (!size || size > max_size)
		return -EINVAL;

	request_payload.index = cpu_to_le32(index);
	request_payload.flags =
		cpu_to_le32(VIRTIO_MSG_SET_VQUEUE_STATE_OP_ENABLE);
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

	request_payload.index = cpu_to_le32(index);
	return virtio_msg_send(vmdev, VIRTIO_MSG_RESET_VQUEUE,
			       &request_payload, sizeof(request_payload),
			       0, NULL);
}

static int virtio_msg_vq_confirm(struct virtio_msg_transport_device *vmdev,
				 struct virtqueue *vq, u32 index, u32 max_size)
{
	struct virtio_msg_vq_info info;
	u32 size;
	int ret;

	ret = virtio_msg_vq_get(vmdev, index, &info);
	if (ret)
		return ret;

	size = virtqueue_get_vring_size(vq);
	if (info.max_size != max_size || info.size != size ||
	    !virtio_msg_vq_is_enabled(&info) ||
	    info.desc_addr != virtqueue_get_desc_addr(vq) ||
	    info.driver_addr != virtqueue_get_avail_addr(vq) ||
	    info.device_addr != virtqueue_get_used_addr(vq))
		return -EPROTO;

	return 0;
}

static int virtio_msg_vq_confirm_cleared(struct virtio_msg_transport_device *vmdev,
					 u32 index)
{
	struct virtio_msg_vq_info info;
	int ret;

	ret = virtio_msg_vq_get(vmdev, index, &info);
	if (ret == -ENOENT)
		return 0;
	if (ret)
		return ret;

	if (info.size || virtio_msg_vq_is_enabled(&info) ||
	    info.desc_addr || info.driver_addr || info.device_addr)
		return -EBUSY;

	return 0;
}

static void
virtio_msg_unwind_vq(struct virtio_msg_transport_device *vmdev,
		     struct virtio_msg_transport *transport,
		     struct virtqueue *vq)
{
	int ret;

	if (virtio_msg_transport_is_unplugged(transport) ||
	    virtio_msg_transport_is_fatal(transport))
		return;

	if (virtio_has_feature(&vmdev->vdev, VIRTIO_F_RING_RESET)) {
		ret = virtio_msg_vq_reset(vmdev, vq->index);
		if (ret) {
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "RESET_VQUEUE failed for vq %u: %d\n",
				   vq->index, ret);
		} else {
			ret = virtio_msg_vq_confirm_cleared(vmdev, vq->index);
			if (!ret)
				return;

			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "RESET_VQUEUE did not clear vq %u: %d\n",
				   vq->index, ret);
		}
	}

	ret = virtio_msg_reset_locked(vmdev, transport);
	if (ret)
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "device reset failed while unwinding vq %u: %d\n",
			   vq->index, ret);
	else
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "device reset while unwinding vq %u\n", vq->index);

	virtio_msg_transport_set_fatal(vmdev, transport);
}

static struct virtqueue *
virtio_msg_setup_vq(struct virtio_msg_transport_device *vmdev, u32 index,
		    void (*callback)(struct virtqueue *vq), const char *name,
		    bool ctx)
{
	struct virtio_msg_transport *transport;
	struct virtio_msg_vq_info info;
	union virtio_map map;
	struct virtqueue *vq;
	int ret;

	transport = virtio_msg_transport_priv(vmdev);

	ret = virtio_msg_vq_get(vmdev, index, &info);
	if (ret)
		return ERR_PTR(ret);

	dev_dbg(&vmdev->vdev.dev, "setup vq index=%u max_size=%u\n",
		index, info.max_size);
	/* Endpoint queue must be inactive before transport setup. */
	if (info.size || virtio_msg_vq_is_enabled(&info) || info.desc_addr ||
	    info.driver_addr || info.device_addr)
		return ERR_PTR(-EBUSY);

	map.dma_dev = vmdev->vdev.vmap.dma_dev ?
		      vmdev->vdev.vmap.dma_dev :
		      vmdev->vdev.dev.parent;
	vq = vring_create_virtqueue_map(index, info.max_size, PAGE_SIZE,
					&vmdev->vdev, true, true, ctx,
					virtio_msg_queue_event_avail, callback, name, map);
	if (!vq)
		return ERR_PTR(-ENOMEM);

	vq->num_max = info.max_size;

	ret = virtio_msg_vq_set(vmdev, vq, index, info.max_size);
	if (ret)
		goto out_del_vq;

	ret = virtio_msg_vq_confirm(vmdev, vq, index, info.max_size);
	if (ret) {
		virtio_msg_unwind_vq(vmdev, transport, vq);
		goto out_del_vq;
	}

	return vq;

out_del_vq:
	dev_dbg(&vmdev->vdev.dev, "setup vq %u error ret=%d\n", index, ret);
	vring_del_virtqueue(vq);
	return ERR_PTR(ret);
}

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

	(void)desc;

	if (!nvqs)
		return 0;

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

	dev_dbg(&vdev->dev, "del vqs\n");

	list_for_each_entry_safe(vq, n, &vdev->vqs, list) {
		if (vmdev && transport)
			virtio_msg_unwind_vq(vmdev, transport, vq);
		vring_del_virtqueue(vq);
	}
}

/* === Device operation: driver notifications ===
 * Spec: §Device Operation / Driver Notifications
 *       (VIRTIO_MSG_EVENT_AVAIL)
 */
static bool virtio_msg_queue_event_avail(struct virtqueue *vq)
{
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport;
	struct virtio_msg_event_avail *event;
	struct virtio_msg *request;
	u8 request_buf[sizeof(*request) + sizeof(*event)];
	u32 avail_idx = 0;
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
	virtio_msg_prepare(request, VIRTIO_MSG_EVENT_AVAIL, 0, sizeof(*event),
			   transport->dev_num);
	event = virtio_msg_payload(request);
	event->vq_index = cpu_to_le32(vq->index);

	if (__virtio_test_bit(&vmdev->vdev, VIRTIO_F_NOTIFICATION_DATA)) {
		avail_idx = vring_notification_data(vq) >> 16;

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
	dev_dbg(&vmdev->vdev.dev,
		"tx EVENT_AVAIL: dev=%u vq=%u next=%u\n",
		 transport->dev_num, vq->index, avail_idx);

	ret = vmdev->provider_ops->send(vmdev, request, NULL);
	if (ret) {
		dev_dbg(&vmdev->vdev.dev,
			"tx EVENT_AVAIL dropped: dev=%u vq=%u next=%u ret=%d\n",
			 transport->dev_num, vq->index, avail_idx, ret);
		dev_dbg(&vmdev->vdev.dev,
			"dropping EVENT_AVAIL: dev=%u vq=%u next=%u ret=%d\n",
		       transport->dev_num, vq->index, avail_idx, ret);
		return false;
	}

	return true;
}

/* === Device operation: device notifications ===
 * Spec: §Device Operation / Device Notifications
 *       (VIRTIO_MSG_EVENT_CONFIG, VIRTIO_MSG_EVENT_USED)
 */
static void
virtio_msg_dispatch_config_event(struct virtio_msg_transport_device *vmdev,
				 struct virtio_msg_transport *transport,
				 u32 generation, u32 device_status)
{
	WRITE_ONCE(transport->config_generation, generation);
	WRITE_ONCE(transport->cached_status, (u8)device_status);
	virtio_config_changed(&vmdev->vdev);
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

/* === Error signaling and fatal teardown ===
 * Spec: §Basic Concepts / Error Signaling
 *       §Device Operation / Device Reset and Shutdown
 */
static void virtio_msg_transport_set_fatal(struct virtio_msg_transport_device *vmdev,
					   struct virtio_msg_transport *transport)
{
	if (!vmdev || !transport || transport->fatal)
		return;

	transport->fatal = true;
	dev_dbg(&vmdev->vdev.dev, "fatal error set\n");
	if (transport->registered)
		virtio_break_device(&vmdev->vdev);
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

/* === virtio_config_ops callbacks ===
 * These thin wrappers adapt the Linux virtio core callback signatures to the
 * internal _locked / _simple helpers above.
 */
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
		virtio_msg_report_nonsleepable_config(vdev, "get", offset, len,
						      _RET_IP_);
		ret = -EWOULDBLOCK;
	} else {
		ret = virtio_msg_get_config_locked(vmdev, transport,
						   offset, buf, len);
	}

	if (ret) {
		if (buf)
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
		virtio_msg_report_nonsleepable_config(vdev, "set", offset, len,
						      _RET_IP_);
		virtio_msg_transport_set_fatal(vmdev, transport);
		return;
	}

	ret = virtio_msg_set_config_locked(vmdev, transport, offset, buf, len);
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
	struct virtio_msg_transport_device *vmdev;
	struct virtio_msg_transport *transport = vdev_transport(vdev, &vmdev);

	if (!transport)
		return;

	if (virtio_msg_transport_is_unplugged(transport)) {
		WRITE_ONCE(transport->cached_status, 0);
		return;
	}

	if (virtio_msg_reset_locked(vmdev, transport))
		virtio_msg_transport_set_fatal(vmdev, transport);
}

static void virtio_msg_synchronize_cbs(struct virtio_device *vdev)
{
	struct virtio_msg_transport_device *vmdev;

	vdev_transport(vdev, &vmdev);

	if (vmdev->provider_ops && vmdev->provider_ops->synchronize_cbs)
		vmdev->provider_ops->synchronize_cbs(vmdev);
	else
		synchronize_rcu();
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

	vring_transport_features(vdev);

	ret = virtio_msg_transport_verify_required_features(vmdev);
	if (ret) {
		virtio_msg_transport_set_fatal(vmdev, transport);
		return ret;
	}

	ret = virtio_msg_set_features_locked(vmdev, transport);
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

static const struct virtio_config_ops virtio_msg_config_ops = {
	.get			= virtio_msg_get_simple,
	.set			= virtio_msg_set_simple,
	.generation		= virtio_msg_generation_simple,
	.get_status		= virtio_msg_get_status,
	.set_status		= virtio_msg_set_status,
	.reset			= virtio_msg_reset,
	.find_vqs		= virtio_msg_find_vqs,
	.del_vqs		= virtio_msg_del_vqs,
	.synchronize_cbs	= virtio_msg_synchronize_cbs,
	.get_extended_features	= virtio_msg_get_extended_features,
	.finalize_features	= virtio_msg_finalize_features,
	.bus_name		= virtio_msg_bus_name,
	/*
	 * .get_shm_region is intentionally omitted until GET_SHM support is
	 * implemented.
	 */
};

int virtio_msg_transport_handle_device_event(struct virtio_msg_transport_device *vmdev,
					     const struct virtio_msg *vmsg)
{
	struct virtio_msg_transport *transport;
	u16 msg_size;
	int ret;

	if (!vmdev || !vmsg)
		return -EINVAL;

	transport = virtio_msg_transport_priv(vmdev);
	if (!transport || !transport->registered) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "drop event: msg_id=0x%02x (device not registered)\n",
			   vmsg->msg_id);
		return 0;
	}
	if (virtio_msg_transport_is_fatal(transport) ||
	    virtio_msg_transport_is_unplugged(transport)) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "drop event: msg_id=0x%02x dev=%u (device stopped)\n",
			   vmsg->msg_id, le16_to_cpu(vmsg->dev_num));
		return 0;
	}

	msg_size = le16_to_cpu(vmsg->msg_size);
	if (msg_size < sizeof(*vmsg) || msg_size > transport->msg_size) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "drop event: msg_id=0x%02x invalid size=%u\n",
			   vmsg->msg_id, msg_size);
		return 0;
	}
	if ((vmsg->type & VIRTIO_MSG_RX_TYPE_MASK) !=
	    VIRTIO_MSG_TYPE_TRANSPORT) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "drop event: msg_id=0x%02x invalid type=0x%02x\n",
			   vmsg->msg_id, vmsg->type);
		return 0;
	}
	if (le16_to_cpu(vmsg->dev_num) != transport->dev_num) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "drop event: msg_id=0x%02x invalid dev=%u expected=%u\n",
			   vmsg->msg_id, le16_to_cpu(vmsg->dev_num),
			   transport->dev_num);
		return 0;
	}

	switch (vmsg->msg_id) {
	case VIRTIO_MSG_EVENT_CONFIG: {
		const struct virtio_msg_event_config *cfg;
		u32 config_offset, config_len, generation, device_status;

		cfg = (const struct virtio_msg_event_config *)vmsg->payload;
		if (msg_size < sizeof(*vmsg) + sizeof(*cfg)) {
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "drop config event: short size=%u\n",
				   msg_size);
			return 0;
		}
		config_offset = le32_to_cpu(cfg->offset);
		config_len = le32_to_cpu(cfg->length);
		generation = le32_to_cpu(cfg->generation);
		device_status = le32_to_cpu(cfg->device_status);
		if (msg_size != sizeof(*vmsg) + sizeof(*cfg) + config_len) {
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "drop config event: size=%u len=%u\n",
				   msg_size, config_len);
			return 0;
		}
		ret = virtio_msg_transport_validate_config_range(transport,
								 config_offset,
								 config_len);
		if (ret) {
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "drop config event: offset=%u len=%u\n",
				   config_offset, config_len);
			return 0;
		}

		/*
		 * Optional config payload bytes are intentionally ignored:
		 * config contents are fetched with a sleepable GET_CONFIG
		 * exchange from the virtio config callback.
		 */
		dev_dbg(&vmdev->vdev.dev,
			"rx EVENT_CONFIG: dev=%u generation=%u offset=%u len=%u\n",
			 le16_to_cpu(vmsg->dev_num), generation,
			 config_offset, config_len);
		virtio_msg_dispatch_config_event(vmdev, transport, generation,
						 device_status);
		break;
	}
	case VIRTIO_MSG_EVENT_USED: {
		const struct virtio_msg_event_used *used;
		u32 vq_index;

		used = (const struct virtio_msg_event_used *)vmsg->payload;
		if (msg_size != sizeof(*vmsg) + sizeof(*used)) {
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "drop used event: size=%u\n", msg_size);
			return 0;
		}
		vq_index = le32_to_cpu(used->vq_index);
		if (vq_index >= transport->max_vq_count) {
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "drop used event: invalid vq=%u max=%u\n",
				   vq_index, transport->max_vq_count);
			return 0;
		}
		dev_dbg(&vmdev->vdev.dev,
			"rx EVENT_USED: dev=%u vq=%u\n",
			 le16_to_cpu(vmsg->dev_num), vq_index);
		ret = virtio_msg_dispatch_used_event(vmdev, vq_index);
		if (ret)
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "drop used event: no vq=%u\n", vq_index);
		break;
	}
	default:
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "drop event: invalid msg_id=0x%02x\n",
			   vmsg->msg_id);
		return 0;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(virtio_msg_transport_handle_device_event);

/* === Exported provider-facing lifecycle API === */
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
	transport->vmdev = vmdev;
	virtio_features_zero(transport->device_features);

	vmdev->provider_ops = provider_ops;
	vmdev->provider_name = provider_name;
	vmdev->provider_data = provider_data;
	vmdev->private = transport;
	dev_info(&vmdev->vdev.dev, "provider attached: bus='%s' dev=%u\n",
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

	dev_info(&vmdev->vdev.dev, "provider detached: bus='%s' dev=%u\n",
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
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "device info failed: dev=%u ret=%d\n",
			   vmdev->dev_num, ret);
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

	dev_info(&vmdev->vdev.dev, "device register: bus='%s' dev=%u\n",
		 vmdev->provider_name, vmdev->dev_num);
	reinit_completion(&transport->release_done);
	transport->registered = true;
	ret = register_virtio_device(&vmdev->vdev);
	if (ret) {
		dev_warn_ratelimited(&vmdev->vdev.dev,
				     "device register failed: dev=%u ret=%d\n",
			   vmdev->dev_num, ret);
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
			dev_warn_ratelimited(&vmdev->vdev.dev,
					     "timed out waiting for failed registration release\n");
			return -ETIMEDOUT;
		}
		return ret;
	}

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

	dev_info(&vmdev->vdev.dev, "device unregister: bus='%s' dev=%u\n",
		 vmdev->provider_name ? vmdev->provider_name : "?",
		vmdev->dev_num);
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

MODULE_DESCRIPTION("Virtio message transport core");
MODULE_LICENSE("GPL");
