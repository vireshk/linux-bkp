// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A device-role binding shell.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#define pr_fmt(fmt) "virtio-msg-ffa: " fmt

#include <linux/bitops.h>
#include <linux/byteorder/generic.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/virtio_msg_bus_provider.h>

#include "virtio_msg_bus_ffa_device_priv.h"
#include "virtio_msg_bus_ffa_xfer.h"

#define VIRTIO_MSG_FFA_BUS_VERSION_1_0		0x00010000U
#define VIRTIO_MSG_FFA_BRIDGE_BUS_NAME		"ffa"
#define VIRTIO_MSG_FFA_TRANSPORT_REVISION_1	VIRTIO_MSG_REVISION_1

static const uuid_t virtio_msg_ffa_device_uuid = VIRTIO_MSG_FFA_DEVICE_UUID;
static const uuid_t virtio_msg_ffa_driver_uuid = VIRTIO_MSG_FFA_DRIVER_UUID;

static const struct ffa_device_id virtio_msg_ffa_driver_ids[] = {
	{ .uuid = virtio_msg_ffa_driver_uuid },
	{}
};

static const struct ffa_device_id virtio_msg_ffa_local_uuid_ids[] = {
	{ .uuid = virtio_msg_ffa_device_uuid },
	{}
};

static LIST_HEAD(virtio_msg_ffa_device_list);
static DEFINE_MUTEX(virtio_msg_ffa_device_list_lock);
static struct ffa_driver virtio_msg_ffa_device_driver;
static bool virtio_msg_ffa_device_service_published;

static const struct virtio_msg_ffa_device_method * const
virtio_msg_ffa_device_methods[] = {
#if IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_FIFO)
	&virtio_msg_ffa_fifo_device_method,
#endif
	&virtio_msg_ffa_indirect_device_method,
};

void virtio_msg_ffa_device_trace_msg(struct virtio_msg_ffa_device *vdev,
				     const char *dir, const char *path,
				     const struct virtio_msg *msg,
				     size_t msg_len)
{
	struct device *dev;
	u16 peer_vm;

	if (!vdev || !msg)
		return;

	dev = vdev->fdev ? &vdev->fdev->dev : NULL;
	peer_vm = vdev->fdev ? (u16)vdev->fdev->vm_id : vdev->peer_vm_id;
	virtio_msg_ffa_trace_msg(dev, peer_vm, dir, path, msg, msg_len);
}

static int virtio_msg_ffa_bus_id_format(u16 peer_vm_id, char *buf, size_t len)
{
	int ret;

	if (!buf || !len)
		return -EINVAL;

	ret = snprintf(buf, len, "%u", peer_vm_id);
	if (ret < 0 || ret >= len)
		return -ENOSPC;

	return 0;
}

static int virtio_msg_ffa_bus_id_parse(const char *bus_id, u16 *peer_vm_id)
{
	const char *num;
	unsigned int vm_id;
	int ret;

	if (!bus_id || !peer_vm_id)
		return -EINVAL;

	num = bus_id;
	if (!*num)
		return -EINVAL;

	ret = kstrtouint(num, 0, &vm_id);
	if (ret)
		return ret;
	if (vm_id > U16_MAX)
		return -ERANGE;

	*peer_vm_id = (u16)vm_id;
	return 0;
}

static int
virtio_msg_ffa_resolver_validate(const struct virtio_msg_bus_bridge_resolver *resolver,
				 char *bus_id)
{
	u16 peer_vm_id;
	int ret;

	if (!resolver || !bus_id)
		return -EINVAL;
	if (!bus_id[0])
		return -EINVAL;

	ret = virtio_msg_ffa_bus_id_parse(bus_id, &peer_vm_id);
	if (ret)
		return ret;

	return virtio_msg_ffa_bus_id_format(peer_vm_id, bus_id,
					    VMSG_BRIDGE_UAPI_BUS_ID_LEN);
}

static struct virtio_msg_bus_bridge_device *
virtio_msg_ffa_resolver_match(const struct virtio_msg_bus_bridge_resolver *resolver,
			      const char *bus_id)
{
	struct virtio_msg_ffa_device *vdev;
	struct virtio_msg_bus_bridge_device *endpoint = NULL;
	u16 peer_vm_id;

	if (!resolver || !bus_id)
		return NULL;
	if (virtio_msg_ffa_bus_id_parse(bus_id, &peer_vm_id))
		return NULL;

	mutex_lock(&virtio_msg_ffa_device_list_lock);
	list_for_each_entry(vdev, &virtio_msg_ffa_device_list, node) {
		if (!READ_ONCE(vdev->endpoint_registered) ||
		    READ_ONCE(vdev->shutting_down))
			continue;
		if (vdev->peer_vm_id != peer_vm_id)
			continue;

		endpoint = &vdev->bridge;
		break;
	}
	mutex_unlock(&virtio_msg_ffa_device_list_lock);

	return endpoint;
}

static struct virtio_msg_bus_bridge_resolver virtio_msg_ffa_resolver = {
	.name = VIRTIO_MSG_FFA_BRIDGE_BUS_NAME,
	.validate = virtio_msg_ffa_resolver_validate,
	.match = virtio_msg_ffa_resolver_match,
};

static void virtio_msg_ffa_device_list_add(struct virtio_msg_ffa_device *vdev)
{
	if (!vdev)
		return;

	mutex_lock(&virtio_msg_ffa_device_list_lock);
	if (!vdev->listed) {
		list_add_tail(&vdev->node, &virtio_msg_ffa_device_list);
		vdev->listed = true;
	}
	mutex_unlock(&virtio_msg_ffa_device_list_lock);
}

static void virtio_msg_ffa_device_list_remove(struct virtio_msg_ffa_device *vdev)
{
	if (!vdev)
		return;

	mutex_lock(&virtio_msg_ffa_device_list_lock);
	if (vdev->listed) {
		list_del_init(&vdev->node);
		vdev->listed = false;
	}
	mutex_unlock(&virtio_msg_ffa_device_list_lock);
}

static void
virtio_msg_ffa_device_bridge_set_offline(struct virtio_msg_ffa_device *vdev,
					 enum virtio_msg_bus_bridge_endpoint_offline_reason reason)
{
	int ret;

	if (!vdev)
		return;

	ret = virtio_msg_bus_bridge_endpoint_set_offline(&vdev->bridge, reason);
	if (ret && ret != -ENODEV && vdev->fdev) {
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "bridge endpoint offline failed reason=%u ret=%d\n",
				     reason, ret);
	}
}

static void virtio_msg_ffa_device_bridge_set_online(struct virtio_msg_ffa_device *vdev)
{
	int ret;

	if (!vdev)
		return;

	ret = virtio_msg_bus_bridge_endpoint_set_online(&vdev->bridge);
	if (ret && ret != -ENODEV && vdev->fdev)
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "bridge endpoint online failed ret=%d\n",
				     ret);
}

static bool
virtio_msg_ffa_device_method_base_supports
		(const struct virtio_msg_ffa_device_method *method,
		 u8 phase_flags, u32 bus_features, bool negotiated)
{
	const struct virtio_msg_ffa_xfer_method_desc *desc;

	if (!method || !method->desc)
		return false;
	desc = method->desc;
	if (desc->method == VIRTIO_MSG_FFA_XFER_NONE)
		return false;
	if (!(desc->role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DEVICE))
		return false;
	if (phase_flags && !(desc->phase_flags & phase_flags))
		return false;
	if (negotiated &&
	    ((bus_features & desc->bus_feature_mask) !=
	     desc->bus_feature_mask))
		return false;

	return true;
}

static bool
virtio_msg_ffa_device_method_table_has_phase(u8 phase_flags, u32 bus_features,
					     bool negotiated)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_device_methods); i++) {
		if (virtio_msg_ffa_device_method_base_supports
				(virtio_msg_ffa_device_methods[i],
				 phase_flags, bus_features, negotiated))
			return true;
	}

	return false;
}

static bool
virtio_msg_ffa_device_method_supports
		(const struct virtio_msg_ffa_device_method *method,
		 u8 phase_flags, u32 bus_features, bool negotiated)
{
	if (!virtio_msg_ffa_device_method_base_supports
			(method, phase_flags, bus_features, negotiated))
		return false;

	if ((phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME) &&
	    !(method->desc->phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP) &&
	    !virtio_msg_ffa_device_method_table_has_phase
			(VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP,
			 bus_features, negotiated))
		return false;

	return true;
}

static bool
virtio_msg_ffa_device_method_capable
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg_ffa_device_method *method)
{
	return vdev && method && method->ops && method->ops->capable &&
	       method->ops->capable(vdev);
}

static const struct virtio_msg_ffa_device_method *
virtio_msg_ffa_device_method_find
		(enum virtio_msg_ffa_transfer_method xfer_method)
{
	const struct virtio_msg_ffa_device_method *method;
	unsigned int i;

	if (xfer_method == VIRTIO_MSG_FFA_XFER_NONE)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_device_methods); i++) {
		method = virtio_msg_ffa_device_methods[i];
		if (method && method->desc && method->desc->method == xfer_method)
			return method;
	}

	return NULL;
}

static const struct virtio_msg_ffa_device_method *
virtio_msg_ffa_device_method_find_desc
		(const struct virtio_msg_ffa_xfer_method_desc *desc)
{
	if (!desc)
		return NULL;

	return virtio_msg_ffa_device_method_find(desc->method);
}

static u32
virtio_msg_ffa_device_local_bus_features_locked
		(struct virtio_msg_ffa_device *vdev)
{
	const struct virtio_msg_ffa_device_method *method;
	u32 deferred_features = 0;
	u32 bus_features = 0;
	bool bootstrap_capable = false;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_device_methods); i++) {
		method = virtio_msg_ffa_device_methods[i];
		if (!virtio_msg_ffa_device_method_supports
				(method, VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
				 0, false))
			continue;
		if (!virtio_msg_ffa_device_method_capable(vdev, method))
			continue;

		if (method->desc->phase_flags &
		    VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP) {
			bus_features |= method->desc->bus_feature_mask;
			bootstrap_capable = true;
		} else {
			deferred_features |= method->desc->bus_feature_mask;
		}
	}

	if (bootstrap_capable)
		bus_features |= deferred_features;

	return bus_features;
}

static u32
virtio_msg_ffa_device_refresh_local_bus_features_locked(struct virtio_msg_ffa_device *vdev)
{
	if (!vdev || !vdev->fdev)
		return 0;

	lockdep_assert_held(&vdev->lock);

	vdev->ep.local_bus_features =
		virtio_msg_ffa_device_local_bus_features_locked(vdev);

	return vdev->ep.local_bus_features;
}

bool virtio_msg_ffa_device_bootstrap_ready_locked(const struct virtio_msg_ffa_device *vdev)
{
	lockdep_assert_held(&vdev->lock);

	return vdev && vdev->bootstrap_method && vdev->bootstrap_method->desc &&
	       vdev->bootstrap_method->ops &&
	       vdev->bootstrap_method->ops->send_locked;
}

int
virtio_msg_ffa_device_send_bootstrap_locked(struct virtio_msg_ffa_device *vdev,
					    const struct virtio_msg *msg,
					    size_t msg_len)
{
	const struct virtio_msg_ffa_device_method *method;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (!virtio_msg_ffa_device_bootstrap_ready_locked(vdev))
		return -EOPNOTSUPP;

	method = vdev->bootstrap_method;
	return method->ops->send_locked(vdev, msg, msg_len);
}

int virtio_msg_ffa_device_runtime_select_locked(struct virtio_msg_ffa_device *vdev,
						const struct virtio_msg_ffa_xfer_method_desc *desc)
{
	const struct virtio_msg_ffa_device_method *method;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !desc)
		return -EINVAL;

	method = virtio_msg_ffa_device_method_find_desc(desc);
	if (!method || !method->ops || !method->ops->send_locked)
		return -EINVAL;
	if (!virtio_msg_ffa_device_method_supports
			(method, VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
			 vdev->ep.bus_features, true))
		return -EOPNOTSUPP;

	vdev->runtime_method = method;
	virtio_msg_ffa_select_transfer_method(&vdev->ep, desc->method);
	virtio_msg_ffa_select_event_delivery(&vdev->ep, desc->event_delivery);

	return 0;
}

void virtio_msg_ffa_device_runtime_clear_locked(struct virtio_msg_ffa_device *vdev)
{
	lockdep_assert_held(&vdev->lock);

	if (!vdev)
		return;

	vdev->runtime_method = NULL;
	vdev->event_configured = false;
	virtio_msg_ffa_select_transfer_method(&vdev->ep,
					      VIRTIO_MSG_FFA_XFER_NONE);
	virtio_msg_ffa_select_event_delivery(&vdev->ep,
					     VIRTIO_MSG_FFA_BUS_EVENT_DELIV_POLL);
}

static int
virtio_msg_ffa_device_select_bootstrap_locked(struct virtio_msg_ffa_device *vdev)
{
	const struct virtio_msg_ffa_device_method *method;
	int first_ret = 0;
	unsigned int i;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !vdev->fdev)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_device_methods); i++) {
		method = virtio_msg_ffa_device_methods[i];
		if (!virtio_msg_ffa_device_method_supports
				(method, VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP,
				 0, false))
			continue;
		if (!virtio_msg_ffa_device_method_capable(vdev, method))
			continue;
		if (!method->ops->init_bootstrap)
			continue;

		ret = method->ops->init_bootstrap(vdev, method->desc);
		if (!ret) {
			vdev->bootstrap_method = method;
			return 0;
		}

		if (!first_ret)
			first_ret = ret;
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "bootstrap %s init failed ret=%d, trying next method\n",
				     method->desc->name, ret);
	}

	return first_ret ?: -EOPNOTSUPP;
}

static void virtio_msg_ffa_device_methods_reset_locked
		(struct virtio_msg_ffa_device *vdev)
{
	const struct virtio_msg_ffa_device_method *method;
	unsigned int i;

	lockdep_assert_held(&vdev->lock);

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_device_methods); i++) {
		method = virtio_msg_ffa_device_methods[i];
		if (method && method->ops && method->ops->reset_locked)
			method->ops->reset_locked(vdev);
	}
}

static void virtio_msg_ffa_device_methods_quiesce
		(struct virtio_msg_ffa_device *vdev)
{
	const struct virtio_msg_ffa_device_method *method;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_device_methods); i++) {
		method = virtio_msg_ffa_device_methods[i];
		if (method && method->ops && method->ops->quiesce)
			method->ops->quiesce(vdev);
	}
}

static bool
virtio_msg_ffa_device_response_needs_bootstrap_locked
		(struct virtio_msg_ffa_device *vdev, u8 msg_id)
{
	lockdep_assert_held(&vdev->lock);

	switch (msg_id) {
	case FFA_BUS_MSG_VERSION:
	case FFA_BUS_MSG_RESET:
	case FFA_BUS_MSG_EVENT_CONFIGURE:
	case FFA_BUS_MSG_FIFO_CONFIGURE:
		return true;
	default:
		return !vdev->event_configured;
	}
}

static int
virtio_msg_ffa_device_send_response_locked(struct virtio_msg_ffa_device *vdev,
					   const struct virtio_msg *req,
					   u8 msg_id, u8 type,
					   const void *payload,
					   size_t payload_len)
{
	u8 resp_buf[FFA_BUS_MAX_MSG_SIZE];
	struct virtio_msg *resp = (struct virtio_msg *)resp_buf;
	size_t resp_len;
	u16 dev_num = 0;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !req)
		return -EINVAL;

	if (msg_id == FFA_BUS_MSG_ERROR)
		dev_num = le16_to_cpu(req->dev_num);

	ret = virtio_msg_ffa_prepare_frame(resp, sizeof(resp_buf), msg_id,
					   type | VIRTIO_MSG_TYPE_RESPONSE,
					   dev_num, le16_to_cpu(req->token),
					   payload, payload_len, &resp_len);
	if (ret)
		return ret;

	if (virtio_msg_ffa_device_response_needs_bootstrap_locked(vdev, msg_id))
		return virtio_msg_ffa_device_send_bootstrap_locked
			(vdev, resp, resp_len);

	return virtio_msg_ffa_device_send_selected_locked
		(vdev, resp, resp_len);
}

static int
virtio_msg_ffa_device_send_error_locked(struct virtio_msg_ffa_device *vdev,
					const struct virtio_msg *req,
					u8 msg_id)
{
	struct virtio_msg_ffa_error_resp resp = {
		.original_msg_id = cpu_to_le16(msg_id),
	};

	lockdep_assert_held(&vdev->lock);

	return virtio_msg_ffa_device_send_response_locked
			(vdev, req, FFA_BUS_MSG_ERROR, VIRTIO_MSG_TYPE_BUS,
			 &resp, sizeof(resp));
}

static u64
virtio_msg_ffa_device_next_relay_seq_locked(struct virtio_msg_ffa_device *vdev)
{
	u64 relay_seq;

	lockdep_assert_held(&vdev->lock);

	relay_seq = vdev->relay_next_seq;
	if (!relay_seq)
		relay_seq = 1;

	vdev->relay_next_seq = relay_seq + 1;
	if (!vdev->relay_next_seq)
		vdev->relay_next_seq = 1;

	return relay_seq;
}

static int virtio_msg_ffa_version_pair_cmp(u32 bus_a, u32 rev_a,
					   u32 bus_b, u32 rev_b)
{
	if (bus_a != bus_b)
		return bus_a > bus_b ? 1 : -1;
	if (rev_a != rev_b)
		return rev_a > rev_b ? 1 : -1;

	return 0;
}

static int
virtio_msg_ffa_device_handle_version_locked(struct virtio_msg_ffa_device *vdev,
					    const struct virtio_msg *msg,
					    size_t msg_len)
{
	const struct virtio_msg_ffa_version_req *req;
	struct virtio_msg_ffa_version_resp resp = { 0 };
	const u32 supported_bus = VIRTIO_MSG_FFA_BUS_VERSION_1_0;
	const u32 supported_rev = VIRTIO_MSG_FFA_TRANSPORT_REVISION_1;
	u32 req_bus;
	u32 req_rev;
	u32 req_ffa_version;
	u32 resp_bus = 0;
	u32 resp_rev = 0;
	u32 bus_features = 0;
	int cmp;
	bool negotiate = false;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (msg_len != sizeof(*msg) + sizeof(*req))
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);

	req = (const struct virtio_msg_ffa_version_req *)msg->payload;
	req_bus = le32_to_cpu(req->bus_version);
	req_rev = le32_to_cpu(req->transport_revision);
	req_ffa_version = le32_to_cpu(req->ffa_version);
	vdev->ep.ffa_version = min(vdev->local_ffa_version, req_ffa_version);

	if (vdev->ep.reset_in_progress)
		goto out_respond;

	if (!req_bus && !req_rev) {
		if (vdev->ep.negotiation_done) {
			resp_bus = vdev->ep.bus_version;
			resp_rev = vdev->ep.transport_revision;
			bus_features = vdev->ep.bus_features;
		} else {
			resp_bus = supported_bus;
			resp_rev = supported_rev;
			bus_features =
				virtio_msg_ffa_device_refresh_local_bus_features_locked(vdev);
		}
		goto out_respond;
	}

	if (vdev->ep.negotiation_done) {
		if (req_bus == vdev->ep.bus_version &&
		    req_rev == vdev->ep.transport_revision) {
			resp_bus = vdev->ep.bus_version;
			resp_rev = vdev->ep.transport_revision;
			bus_features = vdev->ep.bus_features;
		}
		goto out_respond;
	}

	cmp = virtio_msg_ffa_version_pair_cmp(req_bus, req_rev,
					      supported_bus, supported_rev);
	if (!cmp) {
		resp_bus = supported_bus;
		resp_rev = supported_rev;
		bus_features =
			virtio_msg_ffa_device_refresh_local_bus_features_locked(vdev);
		negotiate = true;
	} else if (cmp > 0) {
		resp_bus = supported_bus;
		resp_rev = supported_rev;
		bus_features =
			virtio_msg_ffa_device_refresh_local_bus_features_locked(vdev);
	}

out_respond:
	if (negotiate) {
		vdev->ep.bus_version = resp_bus;
		vdev->ep.transport_revision = resp_rev;
		vdev->ep.feature_bits = 0;
		vdev->ep.bus_features = bus_features;
		vdev->ep.local_bus_features = bus_features;
		vdev->ep.max_areas = U16_MAX;
		vdev->ep.negotiation_done = true;
		vdev->ep.reset_in_progress = false;
		virtio_msg_ffa_device_runtime_clear_locked(vdev);
	}

	resp.bus_version = cpu_to_le32(resp_bus);
	resp.transport_revision = cpu_to_le32(resp_rev);
	resp.ffa_version = cpu_to_le32(vdev->local_ffa_version);
	resp.feature_bits = cpu_to_le32(0);
	resp.bus_features = cpu_to_le32(resp_bus ? bus_features : 0);
	resp.max_areas = cpu_to_le16(resp_bus ? U16_MAX : 0);

	return virtio_msg_ffa_device_send_response_locked
		(vdev, msg, FFA_BUS_MSG_VERSION, VIRTIO_MSG_TYPE_BUS,
		 &resp, sizeof(resp));
}

static int
virtio_msg_ffa_device_handle_reset_locked(struct virtio_msg_ffa_device *vdev,
					  const struct virtio_msg *msg,
					  size_t msg_len)
{
	struct virtio_msg_ffa_reset_resp resp = {
		.result = cpu_to_le16(VIRTIO_MSG_FFA_BUS_SUCCESS),
	};
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (msg_len != sizeof(*msg))
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);

	if (vdev->ep.reset_in_progress) {
		resp.result = cpu_to_le16(VIRTIO_MSG_FFA_BUS_BUSY);
		return virtio_msg_ffa_device_send_response_locked
			(vdev, msg, FFA_BUS_MSG_RESET, VIRTIO_MSG_TYPE_BUS,
			 &resp, sizeof(resp));
	}

	vdev->ep.reset_in_progress = true;
	mutex_unlock(&vdev->lock);
	virtio_msg_ffa_device_bridge_set_offline
		(vdev, VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE_REASON_RESET);
	mutex_lock(&vdev->lock);

	virtio_msg_ffa_device_area_runtime_reset_locked(vdev, true);
	virtio_msg_ffa_exchange_abort_all(&vdev->ep, -ESHUTDOWN);
	mutex_lock(&vdev->ep.exchange_lock);
	vdev->ep.exchange_shutdown = false;
	mutex_unlock(&vdev->ep.exchange_lock);
	virtio_msg_ffa_device_methods_reset_locked(vdev);
	virtio_msg_ffa_device_event_runtime_reset_locked(vdev);
	virtio_msg_ffa_version_begin(&vdev->ep);
	vdev->ep.reset_in_progress = true;
	virtio_msg_bus_bridge_topology_clear(&vdev->bridge);

	ret = virtio_msg_ffa_device_send_response_locked
		(vdev, msg, FFA_BUS_MSG_RESET, VIRTIO_MSG_TYPE_BUS,
		 &resp, sizeof(resp));
	mutex_unlock(&vdev->lock);
	virtio_msg_ffa_device_bridge_set_online(vdev);
	mutex_lock(&vdev->lock);
	vdev->ep.reset_in_progress = false;
	return ret;
}

static int
virtio_msg_ffa_device_handle_get_devices_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg *msg, size_t msg_len)
{
	const struct virtio_msg_bus_get_devices *req;
	struct virtio_msg_bus_get_devices_resp *resp;
	u8 payload[FFA_BUS_MAX_MSG_SIZE - sizeof(struct virtio_msg)] = { 0 };
	u16 offset;
	u16 count;
	u16 out_num = 0;
	u16 next_offset = 0;
	size_t max_count;
	size_t bitmap_len;
	size_t resp_payload_len;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (msg_len != sizeof(*msg) + sizeof(*req))
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);
	if (!READ_ONCE(vdev->endpoint_registered))
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);

	req = (const struct virtio_msg_bus_get_devices *)msg->payload;
	offset = le16_to_cpu(req->offset);
	count = le16_to_cpu(req->count);

	max_count = (sizeof(payload) - sizeof(*resp)) * 8;
	if (count > max_count)
		count = max_count;

	resp = (struct virtio_msg_bus_get_devices_resp *)payload;
	bitmap_len = DIV_ROUND_UP(count, 8);

	ret = virtio_msg_bus_bridge_topology_get_devices_window
			(&vdev->bridge, offset, count, resp->bitmap, bitmap_len,
			 &out_num, &next_offset);
	if (ret)
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);

	resp->offset = cpu_to_le16(offset);
	resp->next_offset = cpu_to_le16(next_offset);
	resp->count = cpu_to_le16(out_num);
	resp_payload_len = sizeof(*resp) + DIV_ROUND_UP(out_num, 8);

	return virtio_msg_ffa_device_send_response_locked
		(vdev, msg, VIRTIO_MSG_BUS_GET_DEVICES, VIRTIO_MSG_TYPE_BUS,
		 payload, resp_payload_len);
}

static int
virtio_msg_ffa_device_handle_ping_locked(struct virtio_msg_ffa_device *vdev,
					 const struct virtio_msg *msg,
					 size_t msg_len)
{
	const struct virtio_msg_bus_ping *req;
	struct virtio_msg_bus_ping_resp resp;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (msg_len != sizeof(*msg) + sizeof(*req))
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);

	req = (const struct virtio_msg_bus_ping *)msg->payload;
	resp.data = req->data;
	return virtio_msg_ffa_device_send_response_locked
		(vdev, msg, VIRTIO_MSG_BUS_PING, VIRTIO_MSG_TYPE_BUS,
		 &resp, sizeof(resp));
}

static int
virtio_msg_ffa_device_handle_event_configure_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg *msg, size_t msg_len)
{
	const struct virtio_msg_ffa_device_method *method;
	const struct virtio_msg_ffa_event_configure_req *req;
	__le16 resp;
	u16 result = VIRTIO_MSG_FFA_BUS_ERROR;
	unsigned int i;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (msg_len != sizeof(*msg) + sizeof(*req))
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);
	if (vdev->event_configured) {
		if (vdev->fdev)
			dev_dbg(&vdev->fdev->dev,
				"event configure rejected: already configured\n");
		else
			pr_debug("event configure rejected: already configured\n");
		return virtio_msg_ffa_device_send_error_locked
			(vdev, msg, FFA_BUS_MSG_EVENT_CONFIGURE);
	}

	req = (const struct virtio_msg_ffa_event_configure_req *)msg->payload;

	for (i = 0; i < ARRAY_SIZE(virtio_msg_ffa_device_methods); i++) {
		method = virtio_msg_ffa_device_methods[i];
		if (!virtio_msg_ffa_device_method_supports
				(method, VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
				 vdev->ep.bus_features, true))
			continue;
		if (method->desc->event_delivery != req->event_method)
			continue;
		if (!virtio_msg_ffa_device_method_capable(vdev, method))
			continue;
		if (!method->ops->event_configure_locked)
			continue;

		if (!method->ops->event_configure_locked(vdev))
			result = VIRTIO_MSG_FFA_BUS_SUCCESS;
		break;
	}

	resp = cpu_to_le16(result);
	return virtio_msg_ffa_device_send_response_locked
		(vdev, msg, FFA_BUS_MSG_EVENT_CONFIGURE, VIRTIO_MSG_TYPE_BUS,
		 &resp, sizeof(resp));
}

static int
virtio_msg_ffa_device_handle_fifo_configure_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg *msg, size_t msg_len)
{
	const struct virtio_msg_ffa_device_method *method;
	struct virtio_msg_ffa_fifo_configure_resp resp = { 0 };
	u16 device_notif_id = 0;
	u16 result = VIRTIO_MSG_FFA_BUS_ERROR;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (msg_len != sizeof(*msg) +
		       sizeof(struct virtio_msg_ffa_fifo_configure_req))
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);

	method = virtio_msg_ffa_device_method_find(VIRTIO_MSG_FFA_XFER_FIFO);
	if (method &&
	    virtio_msg_ffa_device_method_supports
			(method, VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
			 vdev->ep.bus_features, true) &&
	    virtio_msg_ffa_device_method_capable(vdev, method) &&
	    method->ops->bus_configure_locked) {
		ret = method->ops->bus_configure_locked
			(vdev, msg, msg_len, &result, &device_notif_id);
		if (ret)
			return virtio_msg_ffa_device_send_error_locked
					(vdev, msg, msg->msg_id);
	}

	resp.result = cpu_to_le16(result);
	resp.device_notif_id = cpu_to_le16(device_notif_id);
	return virtio_msg_ffa_device_send_response_locked
		(vdev, msg, FFA_BUS_MSG_FIFO_CONFIGURE, VIRTIO_MSG_TYPE_BUS,
		 &resp, sizeof(resp));
}

static int
virtio_msg_ffa_device_handle_area_share_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg *msg, size_t msg_len)
{
	const struct virtio_msg_ffa_area_share_req *req;
	struct virtio_msg_ffa_area_share_resp resp = { 0 };
	u16 result = VIRTIO_MSG_FFA_BUS_ERROR;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (msg_len != sizeof(*msg) + sizeof(*req))
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);

	req = (const struct virtio_msg_ffa_area_share_req *)msg->payload;
	ret = virtio_msg_ffa_device_area_share(vdev, req, &result);
	if (ret && vdev->fdev) {
		dev_dbg(&vdev->fdev->dev,
			"area share failed area=%u ret=%d\n",
			le16_to_cpu(req->area_id), ret);
	}

	resp.area_id = req->area_id;
	resp.result = cpu_to_le16(result);
	return virtio_msg_ffa_device_send_response_locked
		(vdev, msg, FFA_BUS_MSG_AREA_SHARE, VIRTIO_MSG_TYPE_BUS,
		 &resp, sizeof(resp));
}

static int
virtio_msg_ffa_device_handle_area_unshare_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg *msg, size_t msg_len)
{
	const struct virtio_msg_ffa_area_unshare_req *req;
	struct virtio_msg_ffa_area_unshare_resp resp = { 0 };
	u16 area_id;
	u16 result = VIRTIO_MSG_FFA_BUS_ERROR;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (msg_len != sizeof(*msg) + sizeof(*req))
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);

	req = (const struct virtio_msg_ffa_area_unshare_req *)msg->payload;
	area_id = le16_to_cpu(req->area_id);
	ret = virtio_msg_ffa_device_area_unshare_locked(vdev, area_id, &result);
	if (ret && vdev->fdev)
		dev_dbg(&vdev->fdev->dev,
			"area unshare failed area=%u ret=%d\n", area_id, ret);

	resp.area_id = req->area_id;
	resp.result = cpu_to_le16(result);
	return virtio_msg_ffa_device_send_response_locked
		(vdev, msg, FFA_BUS_MSG_AREA_UNSHARE, VIRTIO_MSG_TYPE_BUS,
		 &resp, sizeof(resp));
}

static int
virtio_msg_ffa_device_route_event_locked(struct virtio_msg_ffa_device *vdev,
					 const struct virtio_msg *msg,
					 size_t msg_len, u8 type_bits)
{
	struct virtio_msg_dispatch_ctx dctx = { 0 };
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	(void)msg_len;

	if (type_bits == VIRTIO_MSG_TYPE_BUS)
		return 0;
	if (type_bits != VIRTIO_MSG_TYPE_TRANSPORT)
		return 0;
	if (!READ_ONCE(vdev->endpoint_registered))
		return -ENODEV;

	dctx.flags = VIRTIO_MSG_DISPATCH_F_NONBLOCK;
	ret = virtio_msg_bus_bridge_device_rx(vdev->bridge.handle, msg, &dctx);
	if (ret == -EAGAIN || ret == -ENOSPC)
		return 0;

	return ret;
}

static int
virtio_msg_ffa_device_route_bus_request_locked(struct virtio_msg_ffa_device *vdev,
					       const struct virtio_msg *msg,
					       size_t msg_len)
{
	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	if (msg->type & VIRTIO_MSG_TYPE_RESPONSE)
		return 0;
	if (!(msg->type & VIRTIO_MSG_TYPE_BUS))
		return -EINVAL;

	switch (msg->msg_id) {
	case FFA_BUS_MSG_VERSION:
		return virtio_msg_ffa_device_handle_version_locked(vdev, msg,
								   msg_len);
	case FFA_BUS_MSG_RESET:
		return virtio_msg_ffa_device_handle_reset_locked(vdev, msg,
								 msg_len);
	case FFA_BUS_MSG_EVENT_CONFIGURE:
		return virtio_msg_ffa_device_handle_event_configure_locked(vdev,
									   msg,
									   msg_len);
	case FFA_BUS_MSG_FIFO_CONFIGURE:
		return virtio_msg_ffa_device_handle_fifo_configure_locked(vdev,
									  msg,
									  msg_len);
	case FFA_BUS_MSG_AREA_SHARE:
		return virtio_msg_ffa_device_handle_area_share_locked(vdev, msg,
								      msg_len);
	case FFA_BUS_MSG_AREA_UNSHARE:
		return virtio_msg_ffa_device_handle_area_unshare_locked(vdev, msg,
									msg_len);
	case VIRTIO_MSG_BUS_GET_DEVICES:
		return virtio_msg_ffa_device_handle_get_devices_locked(vdev, msg,
								       msg_len);
	case VIRTIO_MSG_BUS_PING:
		return virtio_msg_ffa_device_handle_ping_locked(vdev, msg,
								msg_len);
	default:
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);
	}
}

static int
virtio_msg_ffa_device_route_transport_request_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg *msg, size_t msg_len)
{
	struct virtio_msg_dispatch_ctx dctx = { 0 };
	u16 dev_num;
	u16 token;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;
	(void)msg_len;

	if (msg->type & (VIRTIO_MSG_TYPE_BUS | VIRTIO_MSG_TYPE_RESPONSE))
		return 0;
	if (!READ_ONCE(vdev->endpoint_registered))
		return -ENODEV;

	token = le16_to_cpu(msg->token);
	dev_num = le16_to_cpu(msg->dev_num);
	ret = virtio_msg_ffa_exchange_store(&vdev->ep, dev_num, token, msg->msg_id,
					    VIRTIO_MSG_TYPE_RESPONSE);
	if (ret)
		return virtio_msg_ffa_device_send_error_locked(vdev, msg,
							       msg->msg_id);

	dctx.flags = VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE;
	dctx.relay_seq = virtio_msg_ffa_device_next_relay_seq_locked(vdev);
	ret = virtio_msg_bus_bridge_device_rx(vdev->bridge.handle, msg, &dctx);
	if (ret) {
		virtio_msg_ffa_exchange_release(&vdev->ep, dev_num, token, NULL);
		(void)virtio_msg_ffa_device_send_error_locked(vdev, msg,
							      msg->msg_id);
	}

	return ret;
}

static int
virtio_msg_ffa_device_handle_preneg_error_locked(struct virtio_msg_ffa_device *vdev,
						 const struct virtio_msg *msg,
						 size_t msg_len)
{
	lockdep_assert_held(&vdev->lock);

	(void)msg_len;
	return virtio_msg_ffa_device_send_error_locked(vdev, msg, msg->msg_id);
}

int
virtio_msg_ffa_device_dispatch_inbound_locked(struct virtio_msg_ffa_device *vdev,
					      const struct virtio_msg *msg,
					      size_t msg_len)
{
	struct virtio_msg_ffa_inbound_result result;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg)
		return -EINVAL;

	ret = virtio_msg_ffa_classify_inbound(&vdev->ep, msg, msg_len,
					      &result);
	if (ret) {
		if (vdev->fdev)
			dev_dbg(&vdev->fdev->dev,
				"bus rx dispatch classify failed: ret=%d\n",
				ret);
		else
			pr_debug("bus rx dispatch classify failed: ret=%d\n",
				 ret);
		return ret;
	}
	if (vdev->ep.reset_in_progress &&
	    (result.class != VIRTIO_MSG_FFA_INBOUND_BUS_REQUEST ||
	     (msg->msg_id != FFA_BUS_MSG_VERSION &&
	      msg->msg_id != FFA_BUS_MSG_RESET)))
		return 0;

	switch (result.class) {
	case VIRTIO_MSG_FFA_INBOUND_EVENT:
		return virtio_msg_ffa_device_route_event_locked(vdev, msg,
								msg_len,
								result.type_bits);
	case VIRTIO_MSG_FFA_INBOUND_BUS_REQUEST:
		return virtio_msg_ffa_device_route_bus_request_locked(vdev, msg,
								      msg_len);
	case VIRTIO_MSG_FFA_INBOUND_TRANSPORT_REQUEST:
		return virtio_msg_ffa_device_route_transport_request_locked(vdev,
									    msg,
									    msg_len);
	case VIRTIO_MSG_FFA_INBOUND_PRENEG_ERROR_RESPONSE:
		return virtio_msg_ffa_device_handle_preneg_error_locked(vdev, msg,
									msg_len);
	case VIRTIO_MSG_FFA_INBOUND_RESPONSE:
	case VIRTIO_MSG_FFA_INBOUND_DROP:
	default:
		return 0;
	}
}

static int
virtio_msg_ffa_device_relay_to_userspace_sleepable
		(struct virtio_msg_ffa_device *vdev, const struct virtio_msg *msg,
		 u16 msg_size, const struct virtio_msg_dispatch_ctx *dctx)
{
	u32 handle;

	if (!vdev || !msg || !dctx)
		return -EINVAL;
	if (!(dctx->flags & VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE) ||
	    !dctx->relay_seq)
		return -EINVAL;
	if (msg_size != le16_to_cpu(msg->msg_size))
		return -EINVAL;

	handle = READ_ONCE(vdev->bridge.handle);
	if (!handle)
		return -ENODEV;

	return virtio_msg_bus_bridge_device_publish_rx(handle, msg);
}

static int
virtio_msg_ffa_device_relay_to_userspace_nonblock
		(struct virtio_msg_ffa_device *vdev, const struct virtio_msg *msg,
		 u16 msg_size)
{
	if (!vdev || !msg)
		return -EINVAL;
	if (msg_size != le16_to_cpu(msg->msg_size))
		return -EINVAL;

	return virtio_msg_bus_bridge_device_publish_rx_nonblock(&vdev->bridge,
								 msg);
}

static int
virtio_msg_ffa_device_relay_response_to_peer_locked
		(struct virtio_msg_ffa_device *vdev, const struct virtio_msg *msg,
		 u16 msg_size, const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_ffa_exchange exchange;
	u16 dev_num;
	u16 token;
	int ret;

	lockdep_assert_held(&vdev->lock);

	if (!vdev || !msg || !dctx)
		return -EINVAL;
	if (!(dctx->flags & VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE) ||
	    !dctx->relay_seq)
		return -EINVAL;
	if (msg->type & VIRTIO_MSG_TYPE_BUS)
		return -EOPNOTSUPP;
	if (!(msg->type & VIRTIO_MSG_TYPE_RESPONSE))
		return -EINVAL;

	dev_num = le16_to_cpu(msg->dev_num);
	token = le16_to_cpu(msg->token);
	ret = virtio_msg_ffa_exchange_lookup(&vdev->ep, dev_num, token,
					     &exchange);
	if (ret)
		return ret;
	if ((exchange.expected_type &
	     (VIRTIO_MSG_TYPE_RESPONSE | VIRTIO_MSG_TYPE_BUS)) !=
	    (msg->type & (VIRTIO_MSG_TYPE_RESPONSE | VIRTIO_MSG_TYPE_BUS)))
		return -EPROTO;
	if (exchange.expected_msg_id != msg->msg_id)
		return -EPROTO;

	ret = virtio_msg_ffa_device_send_selected_locked(vdev, msg, msg_size);
	if (ret)
		return ret;

	virtio_msg_ffa_exchange_release(&vdev->ep, dev_num, token, NULL);
	return 0;
}

static int
virtio_msg_ffa_device_bridge_get_caps(struct virtio_msg_bus_bridge_device *endpoint,
				      struct virtio_msg_bus_bridge_device_caps *caps)
{
	struct virtio_msg_ffa_device *vdev;

	if (!endpoint || !caps)
		return -EINVAL;

	vdev = endpoint->priv;
	if (!vdev)
		return -ENODEV;

	mutex_lock(&vdev->lock);
	memset(caps, 0, sizeof(*caps));
	caps->name = VIRTIO_MSG_FFA_BRIDGE_BUS_NAME;
	caps->msg_size = FFA_BUS_MAX_MSG_SIZE;
	caps->revision = vdev->ep.transport_revision ?
			 vdev->ep.transport_revision :
			 VIRTIO_MSG_FFA_TRANSPORT_REVISION_1;
	caps->transport_features = vdev->ep.feature_bits;
	mutex_unlock(&vdev->lock);

	return 0;
}

static int
virtio_msg_ffa_device_bridge_tx_msg(struct virtio_msg_bus_bridge_device *endpoint,
				    const struct virtio_msg *msg, u16 msg_size,
				    const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_ffa_device *vdev;
	struct virtio_msg *wire_msg;
	u8 wire_buf[FFA_BUS_MAX_MSG_SIZE];
	u16 token;
	int ret;

	(void)dctx;

	if (!endpoint || !msg || msg_size < sizeof(*msg))
		return -EINVAL;
	if (msg_size > FFA_BUS_MAX_MSG_SIZE)
		return -EMSGSIZE;
	if (msg_size != le16_to_cpu(msg->msg_size))
		return -EINVAL;

	vdev = endpoint->priv;
	if (!vdev)
		return -ENODEV;
	if (msg->type & VIRTIO_MSG_TYPE_BUS)
		return -EOPNOTSUPP;
	if (msg->type & VIRTIO_MSG_TYPE_RESPONSE)
		return -EINVAL;
	if (!virtio_msg_ffa_msg_is_event(msg->msg_id))
		return -EOPNOTSUPP;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down)) {
		ret = -ENODEV;
	} else if (!vdev->event_configured) {
		ret = -EACCES;
	} else {
		wire_msg = (struct virtio_msg *)wire_buf;
		memcpy(wire_msg, msg, msg_size);
		ret = virtio_msg_ffa_next_token_locked(&vdev->ep, &token);
		if (!ret) {
			wire_msg->token = cpu_to_le16(token);
			ret = virtio_msg_ffa_device_send_selected_locked
				(vdev, wire_msg, msg_size);
		}
	}
	mutex_unlock(&vdev->lock);
	return ret;
}

static int
virtio_msg_ffa_device_bridge_relay_request
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg, u16 msg_size,
		 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_ffa_device *vdev;

	if (!endpoint)
		return -EINVAL;

	vdev = endpoint->priv;
	if (!vdev)
		return -ENODEV;

	return virtio_msg_ffa_device_relay_to_userspace_sleepable(vdev, msg,
								  msg_size,
								  dctx);
}

static int
virtio_msg_ffa_device_bridge_relay_event
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg, u16 msg_size,
		 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_ffa_device *vdev;

	(void)dctx;

	if (!endpoint)
		return -EINVAL;

	vdev = endpoint->priv;
	if (!vdev)
		return -ENODEV;

	return virtio_msg_ffa_device_relay_to_userspace_nonblock(vdev, msg,
								 msg_size);
}

static int
virtio_msg_ffa_device_bridge_relay_response
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct virtio_msg *msg, u16 msg_size,
		 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_ffa_device *vdev;
	int ret;

	if (!endpoint)
		return -EINVAL;

	vdev = endpoint->priv;
	if (!vdev)
		return -ENODEV;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down))
		ret = -ENODEV;
	else
		ret = virtio_msg_ffa_device_relay_response_to_peer_locked
				(vdev, msg, msg_size, dctx);
	mutex_unlock(&vdev->lock);

	return ret;
}

static int
virtio_msg_ffa_device_bridge_report_error
		(struct virtio_msg_bus_bridge_device *endpoint, u16 dev_num,
		 u16 token, u8 msg_id, int error,
		 const struct virtio_msg_dispatch_ctx *dctx)
{
	struct virtio_msg_ffa_device *vdev;
	struct virtio_msg_ffa_exchange exchange;
	struct virtio_msg req = {
		.dev_num = cpu_to_le16(dev_num),
		.token = cpu_to_le16(token),
	};
	int ret;

	if (!endpoint || error >= 0 || !dctx)
		return -EINVAL;
	if (!(dctx->flags & VIRTIO_MSG_DISPATCH_F_REQUEST_SLEEPABLE) ||
	    !dctx->relay_seq)
		return -EINVAL;
	if (virtio_msg_ffa_msg_is_event(msg_id))
		return -EINVAL;

	vdev = endpoint->priv;
	if (!vdev)
		return -ENODEV;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down)) {
		ret = -ENODEV;
		goto out_unlock;
	}

	ret = virtio_msg_ffa_exchange_lookup(&vdev->ep, dev_num, token,
					     &exchange);
	if (ret)
		goto out_unlock;
	if (exchange.expected_msg_id != msg_id) {
		ret = -EPROTO;
		goto out_unlock;
	}

	ret = virtio_msg_ffa_device_send_error_locked(vdev, &req, msg_id);
	if (!ret)
		virtio_msg_ffa_exchange_release(&vdev->ep, dev_num, token, NULL);

out_unlock:
	mutex_unlock(&vdev->lock);
	return ret;
}

static int
virtio_msg_ffa_device_bridge_mmap(struct virtio_msg_bus_bridge_device *endpoint,
				  u64 mmap_offset, struct vm_area_struct *vma)
{
	struct virtio_msg_ffa_device *vdev;
	int ret;

	if (!endpoint || !vma)
		return -EINVAL;

	vdev = endpoint->priv;
	if (!vdev)
		return -ENODEV;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down))
		ret = -ENODEV;
	else
		ret = virtio_msg_ffa_device_area_mmap_locked(vdev, mmap_offset,
							     vma);
	mutex_unlock(&vdev->lock);

	return ret;
}

static int
virtio_msg_ffa_device_bridge_map_add_response
		(struct virtio_msg_ffa_device *vdev,
		 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status,
		 bool *endpoint_remove)
{
	int ret;

	if (!vdev || !event)
		return -EINVAL;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down))
		ret = 0;
	else
		ret = virtio_msg_ffa_device_area_map_add_resp_locked
				(vdev, event, ack_status, endpoint_remove);
	mutex_unlock(&vdev->lock);

	return ret;
}

static int
virtio_msg_ffa_device_bridge_map_del_response
		(struct virtio_msg_ffa_device *vdev,
		 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status,
		 bool *endpoint_remove)
{
	int ret;

	(void)endpoint_remove;

	if (!vdev || !event)
		return -EINVAL;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down))
		ret = 0;
	else
		ret = virtio_msg_ffa_device_area_map_del_resp_locked
				(vdev, event, ack_status);
	mutex_unlock(&vdev->lock);

	return ret;
}

static int
virtio_msg_ffa_device_bridge_map_released
		(struct virtio_msg_ffa_device *vdev,
		 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status,
		 bool *endpoint_remove)
{
	int ret;

	if (!vdev || !event || !endpoint_remove)
		return -EINVAL;
	if (ack_status != VMSG_BRIDGE_UAPI_MAP_ACK_OK)
		return -EINVAL;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down))
		ret = 0;
	else
		ret = virtio_msg_ffa_device_area_map_released_locked
				(vdev, event, endpoint_remove);
	mutex_unlock(&vdev->lock);

	return ret;
}

static int
virtio_msg_ffa_device_bridge_map_event_ack
		(struct virtio_msg_bus_bridge_device *endpoint,
		 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status)
{
	struct virtio_msg_ffa_device *vdev;
	bool endpoint_remove = false;
	int ret;

	if (!endpoint || !event)
		return -EINVAL;

	vdev = endpoint->priv;
	if (!vdev)
		return -ENODEV;

	switch (event->type) {
	case VMSG_BRIDGE_UAPI_MAP_EVENT_ADD_RESP:
		ret = virtio_msg_ffa_device_bridge_map_add_response
			(vdev, event, ack_status, &endpoint_remove);
		break;
	case VMSG_BRIDGE_UAPI_MAP_EVENT_DEL_RESP:
		ret = virtio_msg_ffa_device_bridge_map_del_response
			(vdev, event, ack_status, &endpoint_remove);
		break;
	case VMSG_BRIDGE_UAPI_MAP_EVENT_RELEASED:
		ret = virtio_msg_ffa_device_bridge_map_released
			(vdev, event, ack_status, &endpoint_remove);
		break;
	default:
		return 0;
	}

	if (endpoint_remove)
		virtio_msg_ffa_device_bridge_set_offline
			(vdev,
			 VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE_REASON_REMOVED);

	return ret;
}

static int
virtio_msg_ffa_device_bridge_notify_device_event
		(struct virtio_msg_bus_bridge_device *endpoint, u16 dev_num,
		 u16 dev_state)
{
	struct virtio_msg_ffa_device *vdev;

	if (!endpoint)
		return -EINVAL;

	vdev = endpoint->priv;
	if (!vdev)
		return -ENODEV;

	switch (dev_state) {
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_ADDED:
	case VIRTIO_MSG_BUS_EVENT_DEV_STATE_REMOVED:
		break;
	default:
		return -EINVAL;
	}

	return virtio_msg_ffa_device_event_enqueue(vdev, dev_num, dev_state);
}

static void
virtio_msg_ffa_device_bridge_endpoint_online
		(struct virtio_msg_bus_bridge_device *endpoint)
{
	struct virtio_msg_ffa_device *vdev;
	int ret;

	if (!endpoint)
		return;

	vdev = endpoint->priv;
	if (!vdev)
		return;

	mutex_lock(&vdev->lock);
	if (READ_ONCE(vdev->shutting_down))
		ret = 0;
	else
		ret = virtio_msg_ffa_device_area_publish_deferred_locked(vdev);
	mutex_unlock(&vdev->lock);

	if (ret && vdev->fdev) {
		dev_warn_ratelimited(&vdev->fdev->dev,
				     "deferred area publication failed: %d\n",
				     ret);
	}
}

static const struct virtio_msg_bus_bridge_device_ops virtio_msg_ffa_device_ops = {
	.get_caps = virtio_msg_ffa_device_bridge_get_caps,
	.tx_msg = virtio_msg_ffa_device_bridge_tx_msg,
	.tx_userspace_msg = virtio_msg_ffa_device_bridge_tx_msg,
	.relay_request = virtio_msg_ffa_device_bridge_relay_request,
	.relay_event = virtio_msg_ffa_device_bridge_relay_event,
	.relay_response = virtio_msg_ffa_device_bridge_relay_response,
	.report_error = virtio_msg_ffa_device_bridge_report_error,
	.mmap = virtio_msg_ffa_device_bridge_mmap,
	.map_event_ack = virtio_msg_ffa_device_bridge_map_event_ack,
	.notify_device_event = virtio_msg_ffa_device_bridge_notify_device_event,
	.endpoint_online = virtio_msg_ffa_device_bridge_endpoint_online,
};

static int virtio_msg_ffa_device_probe(struct ffa_device *fdev)
{
	struct virtio_msg_ffa_device *vdev;
	int ret;

	if (!fdev)
		return -EINVAL;
	dev_dbg(&fdev->dev, "device-role probe: peer_vm=%u uuid=%pUb\n",
		(u16)fdev->vm_id, &fdev->uuid);

	vdev = kzalloc(sizeof(*vdev), GFP_KERNEL);
	if (!vdev)
		return -ENOMEM;

	vdev->fdev = fdev;
	vdev->peer_vm_id = fdev->vm_id;
	vdev->relay_next_seq = 1;
	mutex_init(&vdev->lock);
	INIT_LIST_HEAD(&vdev->node);
	virtio_msg_ffa_endpoint_init(&vdev->ep);
	ret = virtio_msg_ffa_device_event_runtime_init(vdev);
	if (ret) {
		dev_dbg(&fdev->dev,
			"device-role probe failed: event runtime init ret=%d\n",
			ret);
		goto err_cleanup_endpoint;
	}
	mutex_lock(&vdev->lock);
	ret = virtio_msg_ffa_device_select_bootstrap_locked(vdev);
	mutex_unlock(&vdev->lock);
	if (ret) {
		dev_dbg(&fdev->dev,
			"device-role probe failed: bootstrap init ret=%d\n",
			ret);
		goto err_cleanup_event_runtime;
	}
	ret = virtio_msg_ffa_device_area_runtime_init(vdev);
	if (ret) {
		dev_dbg(&fdev->dev,
			"device-role probe failed: area runtime init ret=%d\n",
			ret);
		goto err_cleanup_methods;
	}
	virtio_msg_ffa_version_begin(&vdev->ep);
	mutex_lock(&vdev->lock);
	virtio_msg_ffa_device_runtime_clear_locked(vdev);
	mutex_unlock(&vdev->lock);
	mutex_lock(&vdev->lock);
	virtio_msg_ffa_device_refresh_local_bus_features_locked(vdev);
	mutex_unlock(&vdev->lock);

	vdev->local_ffa_version = FFA_VERSION_1_2;
	if (fdev->ops && fdev->ops->info_ops &&
	    fdev->ops->info_ops->api_version_get)
		vdev->local_ffa_version = fdev->ops->info_ops->api_version_get();

	ret = virtio_msg_ffa_bus_id_format(vdev->peer_vm_id, vdev->bus_id,
					   sizeof(vdev->bus_id));
	if (ret) {
		dev_dbg(&fdev->dev,
			"device-role probe failed: bus id format ret=%d\n",
			ret);
		goto err_cleanup_runtime;
	}

	virtio_msg_bus_bridge_device_init(&vdev->bridge, &virtio_msg_ffa_device_ops,
					  vdev);
	ret = virtio_msg_bus_bridge_device_register(VIRTIO_MSG_FFA_BRIDGE_BUS_NAME,
						    vdev->bus_id, &vdev->bridge);
	if (ret) {
		dev_dbg(&fdev->dev,
			"device-role probe failed: bridge register bus_id=%s ret=%d\n",
			vdev->bus_id, ret);
		goto err_cleanup_runtime;
	}

	virtio_msg_ffa_device_list_add(vdev);
	WRITE_ONCE(vdev->endpoint_registered, true);
	ffa_dev_set_drvdata(fdev, vdev);
	dev_info(&fdev->dev, "new endpoint %s:%s available\n",
		 VIRTIO_MSG_FFA_BRIDGE_BUS_NAME, vdev->bus_id);
	dev_info(&fdev->dev, "device-role probe complete: peer_vm=%u bus_id=%s\n",
		 (u16)fdev->vm_id, vdev->bus_id);

	return 0;

err_cleanup_runtime:
	mutex_lock(&vdev->lock);
	virtio_msg_ffa_device_area_runtime_reset_locked(vdev, false);
	virtio_msg_ffa_device_methods_reset_locked(vdev);
	mutex_unlock(&vdev->lock);
	virtio_msg_ffa_device_area_runtime_cleanup(vdev);
err_cleanup_methods:
	virtio_msg_ffa_device_methods_quiesce(vdev);
err_cleanup_event_runtime:
	virtio_msg_ffa_device_event_runtime_quiesce(vdev);
	virtio_msg_ffa_endpoint_cleanup(&vdev->ep);
	goto err_free_vdev;
err_cleanup_endpoint:
	virtio_msg_ffa_endpoint_cleanup(&vdev->ep);
err_free_vdev:
	kfree(vdev);
	return ret;
}

static void virtio_msg_ffa_device_remove(struct ffa_device *fdev)
{
	struct virtio_msg_ffa_device *vdev;

	if (!fdev)
		return;

	vdev = ffa_dev_get_drvdata(fdev);
	if (!vdev)
		return;

	ffa_dev_set_drvdata(fdev, NULL);
	virtio_msg_ffa_device_bridge_set_offline
		(vdev, VIRTIO_MSG_BUS_BRIDGE_ENDPOINT_OFFLINE_REASON_REMOVED);
	mutex_lock(&vdev->lock);
	virtio_msg_ffa_device_area_runtime_reset_locked(vdev, true);
	WRITE_ONCE(vdev->shutting_down, true);
	virtio_msg_ffa_exchange_abort_all(&vdev->ep, -ESHUTDOWN);
	virtio_msg_ffa_device_methods_reset_locked(vdev);
	virtio_msg_ffa_device_event_runtime_reset_locked(vdev);
	WRITE_ONCE(vdev->endpoint_registered, false);
	mutex_unlock(&vdev->lock);
	virtio_msg_ffa_device_methods_quiesce(vdev);
	virtio_msg_ffa_device_event_runtime_quiesce(vdev);

	virtio_msg_ffa_device_list_remove(vdev);
	dev_info(&fdev->dev, "endpoint %s:%s removed\n",
		 VIRTIO_MSG_FFA_BRIDGE_BUS_NAME, vdev->bus_id);
	virtio_msg_bus_bridge_device_unregister(VIRTIO_MSG_FFA_BRIDGE_BUS_NAME,
						vdev->bus_id, &vdev->bridge);
	virtio_msg_ffa_device_area_runtime_cleanup(vdev);
	virtio_msg_ffa_endpoint_cleanup(&vdev->ep);
	kfree(vdev);
}

static struct ffa_driver virtio_msg_ffa_device_driver = {
	.name = "virtio-msg-ffa-device",
	.probe = virtio_msg_ffa_device_probe,
	.remove = virtio_msg_ffa_device_remove,
	.id_table = virtio_msg_ffa_driver_ids,
	.local_uuid_table = virtio_msg_ffa_local_uuid_ids,
	.rx_msg = virtio_msg_ffa_device_indirect_rx_cb,
};

int virtio_msg_ffa_device_driver_register(void)
{
	int ret;

	ret = virtio_msg_bus_bridge_resolver_register(&virtio_msg_ffa_resolver);
	if (ret) {
		pr_debug("device-role register failed: resolver ret=%d\n", ret);
		return ret;
	}

	ret = ffa_register(&virtio_msg_ffa_device_driver);
	if (ret) {
		pr_debug("device-role register failed: ffa ret=%d\n", ret);
		virtio_msg_bus_bridge_resolver_unregister(&virtio_msg_ffa_resolver);
		return ret;
	}

	ret = ffa_driver_service_publish(&virtio_msg_ffa_device_driver,
					 &virtio_msg_ffa_device_uuid);
	if (ret == -EOPNOTSUPP) {
		pr_debug("device-role local service publish unsupported; continuing without dynamic publication\n");
		return 0;
	}
	if (ret) {
		pr_debug("device-role register failed: local service publish ret=%d\n",
			 ret);
		ffa_unregister(&virtio_msg_ffa_device_driver);
		virtio_msg_bus_bridge_resolver_unregister(&virtio_msg_ffa_resolver);
		return ret;
	}

	virtio_msg_ffa_device_service_published = true;
	pr_debug("device-role local service published\n");

	return 0;
}

void virtio_msg_ffa_device_driver_unregister(void)
{
	int ret;

	if (virtio_msg_ffa_device_service_published) {
		ret = ffa_driver_service_unpublish
				(&virtio_msg_ffa_device_driver,
				 &virtio_msg_ffa_device_uuid);
		if (ret)
			pr_debug("device-role local service unpublish failed: %d\n",
				 ret);
		virtio_msg_ffa_device_service_published = false;
	}

	ffa_unregister(&virtio_msg_ffa_device_driver);
	virtio_msg_bus_bridge_resolver_unregister(&virtio_msg_ffa_resolver);
}

static int __init virtio_msg_ffa_device_init(void)
{
	return virtio_msg_ffa_device_driver_register();
}

static void __exit virtio_msg_ffa_device_exit(void)
{
	virtio_msg_ffa_device_driver_unregister();
}

module_init(virtio_msg_ffa_device_init);
module_exit(virtio_msg_ffa_device_exit);

MODULE_DESCRIPTION("Virtio message bus over FF-A device binding");
MODULE_LICENSE("GPL");
