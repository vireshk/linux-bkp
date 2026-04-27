/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus over FF-A device-binding private declarations.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_FFA_DEVICE_PRIV_H
#define _VIRTIO_MSG_BUS_FFA_DEVICE_PRIV_H

#include <linux/arm_ffa.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/uuid.h>
#include <linux/virtio_msg_bus_bridge_device.h>
#include <linux/workqueue.h>
#include <linux/xarray.h>

#include "virtio_msg_bus_dma_device_helper.h"
#include "virtio_msg_bus_ffa.h"
#include "virtio_msg_bus_queue_helper.h"
#include "virtio_msg_bus_ffa_xfer.h"

#define VIRTIO_MSG_FFA_RELAY_TIMEOUT_MS		5000

struct page;
struct virtio_msg_ffa_device_fifo;
struct virtio_msg_ffa_device_indirect;
struct vm_area_struct;

struct virtio_msg_ffa_device_method_ops {
	bool (*capable)(const struct virtio_msg_ffa_device *vdev);
	int (*init_bootstrap)(struct virtio_msg_ffa_device *vdev,
			      const struct virtio_msg_ffa_xfer_method_desc *desc);
	int (*init_runtime)(struct virtio_msg_ffa_device *vdev,
			    const struct virtio_msg_ffa_xfer_method_desc *desc);
	void (*reset_locked)(struct virtio_msg_ffa_device *vdev);
	void (*quiesce)(struct virtio_msg_ffa_device *vdev);
	int (*send_locked)(struct virtio_msg_ffa_device *vdev,
			   const struct virtio_msg *msg, size_t msg_len);
	int (*event_configure_locked)(struct virtio_msg_ffa_device *vdev);
	int (*bus_configure_locked)(struct virtio_msg_ffa_device *vdev,
				    const struct virtio_msg *msg,
				    size_t msg_len, u16 *result,
				    u16 *device_notif_id);
};

struct virtio_msg_ffa_device_method {
	const struct virtio_msg_ffa_xfer_method_desc *desc;
	const struct virtio_msg_ffa_device_method_ops *ops;
};

struct virtio_msg_ffa_device {
	struct ffa_device *fdev;
	struct virtio_msg_ffa_endpoint ep;
	struct virtio_msg_bus_bridge_device bridge;
	struct virtio_msg_ffa_device_indirect __rcu *indirect;
#if IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_FIFO)
	struct virtio_msg_ffa_device_fifo *fifo;
#endif
	struct virtio_msg_bus_queue_helper event_queue;
	struct mutex lock; /* Serializes device-role endpoint state. */
	const struct virtio_msg_ffa_device_method *bootstrap_method;
	const struct virtio_msg_ffa_device_method *runtime_method;
	struct list_head node; /* Resolver match list node. */
	struct xarray areas_by_id; /* Authoritative area state keyed by area_id. */
	struct virtio_msg_bus_dma_device_helper area_maps;
	char bus_id[VMSG_BRIDGE_UAPI_BUS_ID_LEN];
	u16 peer_vm_id;
	u64 relay_next_seq;
	u64 event_generation;
	u32 local_ffa_version;
	bool event_configured;
	bool listed;
	bool endpoint_registered;
	bool shutting_down;
};

int virtio_msg_ffa_device_driver_register(void);
void virtio_msg_ffa_device_driver_unregister(void);
int virtio_msg_ffa_device_event_runtime_init(struct virtio_msg_ffa_device *vdev);
void virtio_msg_ffa_device_event_runtime_quiesce(struct virtio_msg_ffa_device *vdev);
void virtio_msg_ffa_device_event_runtime_reset_locked(struct virtio_msg_ffa_device *vdev);
int virtio_msg_ffa_device_event_enqueue(struct virtio_msg_ffa_device *vdev,
					u16 dev_num, u16 dev_state);
int virtio_msg_ffa_device_send_selected_locked(struct virtio_msg_ffa_device *vdev,
					       const struct virtio_msg *msg,
					       size_t msg_len);
int virtio_msg_ffa_device_send_bootstrap_locked
	(struct virtio_msg_ffa_device *vdev, const struct virtio_msg *msg,
	 size_t msg_len);
int virtio_msg_ffa_device_runtime_select_locked
	(struct virtio_msg_ffa_device *vdev,
	 const struct virtio_msg_ffa_xfer_method_desc *desc);
void virtio_msg_ffa_device_runtime_clear_locked(struct virtio_msg_ffa_device *vdev);
bool virtio_msg_ffa_device_bootstrap_ready_locked
	(const struct virtio_msg_ffa_device *vdev);
void virtio_msg_ffa_device_trace_msg(struct virtio_msg_ffa_device *vdev,
				     const char *dir, const char *path,
				     const struct virtio_msg *msg,
				     size_t msg_len);
int virtio_msg_ffa_device_dispatch_inbound_locked(struct virtio_msg_ffa_device *vdev,
						  const struct virtio_msg *msg,
						  size_t msg_len);
#if IS_ENABLED(CONFIG_VIRTIO_MSG_FFA_XFER_FIFO)
extern const struct virtio_msg_ffa_device_method
	virtio_msg_ffa_fifo_device_method;
bool virtio_msg_ffa_device_fifo_capable(const struct virtio_msg_ffa_device *vdev);
int virtio_msg_ffa_device_fifo_send_locked(struct virtio_msg_ffa_device *vdev,
					   const struct virtio_msg *msg,
					   size_t msg_len);
int virtio_msg_ffa_device_fifo_event_locked(struct virtio_msg_ffa_device *vdev);
int virtio_msg_ffa_device_fifo_cfg_locked(struct virtio_msg_ffa_device *vdev,
					  const struct virtio_msg *msg,
					  size_t msg_len, u16 *result,
					  u16 *device_notif_id);
void virtio_msg_ffa_device_fifo_reset_locked(struct virtio_msg_ffa_device *vdev);
void virtio_msg_ffa_device_fifo_runtime_quiesce(struct virtio_msg_ffa_device *vdev);
#else
static inline bool
virtio_msg_ffa_device_fifo_capable(const struct virtio_msg_ffa_device *vdev)
{
	(void)vdev;

	return false;
}

static inline int
virtio_msg_ffa_device_fifo_send_locked(struct virtio_msg_ffa_device *vdev,
				       const struct virtio_msg *msg,
				       size_t msg_len)
{
	(void)vdev;
	(void)msg;
	(void)msg_len;

	return -EOPNOTSUPP;
}

static inline int
virtio_msg_ffa_device_fifo_event_locked(struct virtio_msg_ffa_device *vdev)
{
	(void)vdev;

	return -EOPNOTSUPP;
}

static inline int
virtio_msg_ffa_device_fifo_cfg_locked(struct virtio_msg_ffa_device *vdev,
				      const struct virtio_msg *msg,
				      size_t msg_len, u16 *result,
				      u16 *device_notif_id)
{
	(void)vdev;
	(void)msg;
	(void)msg_len;
	(void)result;
	(void)device_notif_id;

	return -EOPNOTSUPP;
}

static inline void
virtio_msg_ffa_device_fifo_reset_locked(struct virtio_msg_ffa_device *vdev)
{
	(void)vdev;
}

static inline void
virtio_msg_ffa_device_fifo_runtime_quiesce(struct virtio_msg_ffa_device *vdev)
{
	(void)vdev;
}
#endif
extern const struct virtio_msg_ffa_device_method
	virtio_msg_ffa_indirect_device_method;
bool virtio_msg_ffa_device_indirect_capable
	(const struct virtio_msg_ffa_device *vdev);
int virtio_msg_ffa_device_indirect_runtime_init(struct virtio_msg_ffa_device *vdev);
void
virtio_msg_ffa_device_indirect_runtime_quiesce(struct virtio_msg_ffa_device *vdev);
void
virtio_msg_ffa_device_indirect_runtime_reset_locked
		(struct virtio_msg_ffa_device *vdev);
int
virtio_msg_ffa_device_indirect_event_configure_locked
		(struct virtio_msg_ffa_device *vdev);
int virtio_msg_ffa_device_send_indirect_locked(struct virtio_msg_ffa_device *vdev,
					       const struct virtio_msg *msg,
					       size_t msg_len);
bool virtio_msg_ffa_device_indirect_rx_cb(struct ffa_device *fdev,
					  u16 sender_vm_id,
					  const uuid_t *uuid,
					  const void *buf, size_t len);
int virtio_msg_ffa_device_area_runtime_init(struct virtio_msg_ffa_device *vdev);
void virtio_msg_ffa_device_area_runtime_cleanup(struct virtio_msg_ffa_device *vdev);
void virtio_msg_ffa_device_area_runtime_reset_locked
	(struct virtio_msg_ffa_device *vdev, bool emit_deferred_release);
int virtio_msg_ffa_device_area_share
	(struct virtio_msg_ffa_device *vdev,
	 const struct virtio_msg_ffa_area_share_req *req, u16 *result);
int virtio_msg_ffa_device_area_unshare_locked(struct virtio_msg_ffa_device *vdev,
					      u16 area_id, u16 *result);
int virtio_msg_ffa_device_area_mmap_locked(struct virtio_msg_ffa_device *vdev,
					   u64 mmap_offset,
					   struct vm_area_struct *vma);
int virtio_msg_ffa_device_area_map_add_resp_locked
	(struct virtio_msg_ffa_device *vdev,
	 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status,
	 bool *endpoint_remove);
int virtio_msg_ffa_device_area_map_del_resp_locked
	(struct virtio_msg_ffa_device *vdev,
	 const struct vmsg_bridge_uapi_map_event *event, u32 ack_status);
int virtio_msg_ffa_device_area_map_released_locked
	(struct virtio_msg_ffa_device *vdev,
	 const struct vmsg_bridge_uapi_map_event *event, bool *endpoint_remove);
int virtio_msg_ffa_device_area_publish_deferred_locked
	(struct virtio_msg_ffa_device *vdev);

#endif /* _VIRTIO_MSG_BUS_FFA_DEVICE_PRIV_H */
