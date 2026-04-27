/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message bus over FF-A driver-binding private declarations.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_BUS_FFA_DRIVER_PRIV_H
#define _VIRTIO_MSG_BUS_FFA_DRIVER_PRIV_H

#include <linux/bitops.h>
#include <linux/gfp.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "virtio_msg_bus_ffa.h"
#include "virtio_msg_bus_queue_helper.h"
#include "virtio_msg_bus_ffa_xfer.h"

struct device;
struct virtio_msg_bus_provider_ops;
struct virtio_msg_bus_dma_driver_provider_ops;
struct virtio_msg_ffa_driver;
struct virtio_msg_ffa_area_ctx;
struct virtio_msg_ffa_topology;
struct virtio_msg_transport_device;

struct virtio_msg_ffa_xfer_engine {
	enum virtio_msg_ffa_transfer_method method;
	const struct virtio_msg_ffa_xfer_ops *ops;
	void *priv;
};

struct virtio_msg_ffa_xfer_ops {
	int (*setup_locked)(struct virtio_msg_ffa_driver *drv,
			    struct virtio_msg_ffa_xfer_engine *xfer);
	void (*teardown_locked)(struct virtio_msg_ffa_driver *drv,
				struct virtio_msg_ffa_xfer_engine *xfer);
	void (*quiesce_locked)(struct virtio_msg_ffa_driver *drv,
			       struct virtio_msg_ffa_xfer_engine *xfer);
	int (*submit_locked)(struct virtio_msg_ffa_driver *drv,
			     struct virtio_msg_ffa_xfer_engine *xfer,
			     const struct virtio_msg *req, size_t req_len,
			     struct virtio_msg *resp, size_t resp_buf_len,
			     size_t *resp_len);
	int (*event_configure_locked)(struct virtio_msg_ffa_driver *drv,
				      struct virtio_msg_ffa_xfer_engine *xfer);
	int (*event_poll_once_locked)(struct virtio_msg_ffa_driver *drv,
				      struct virtio_msg_ffa_xfer_engine *xfer,
				      bool *empty);
	void (*release_priv)(struct virtio_msg_ffa_driver *drv,
			     struct virtio_msg_ffa_xfer_engine *xfer);
	bool async_response;
};

#ifdef CONFIG_VIRTIO_MSG_FFA_XFER_INDIRECT
bool virtio_msg_ffa_indirect_available(struct virtio_msg_ffa_driver *drv);
extern const struct virtio_msg_ffa_xfer_ops virtio_msg_ffa_indirect_xfer_ops;
extern const struct virtio_msg_ffa_xfer_method_desc
	virtio_msg_ffa_indirect_driver_method_desc;
#endif
#ifdef CONFIG_VIRTIO_MSG_FFA_XFER_FIFO
bool virtio_msg_ffa_fifo_available(struct virtio_msg_ffa_driver *drv);
extern const struct virtio_msg_ffa_xfer_ops virtio_msg_ffa_fifo_xfer_ops;
extern const struct virtio_msg_ffa_xfer_method_desc
	virtio_msg_ffa_fifo_driver_method_desc;
#endif

struct virtio_msg_ffa_driver {
	struct ffa_device *fdev;
	struct virtio_msg_ffa_endpoint ep;
	struct virtio_msg_ffa_xfer_engine bootstrap_xfer;
	struct virtio_msg_ffa_xfer_engine active_xfer;
	struct virtio_msg_bus_queue_helper indirect_rx_queue;
	void *xfer_priv[4];
	struct mutex lock; /* Serializes endpoint driver-side state. */
	spinlock_t bus_req_lock; /* Protects same-type bus request state. */
	DECLARE_BITMAP(bus_req_inflight, 256);
	wait_queue_head_t bus_req_wait;
	u32 local_ffa_version;
	bool event_configured;
	bool endpoint_disabled;
	struct virtio_msg_ffa_area_ctx area_ctx;
	struct virtio_msg_ffa_topology *topology;
	u64 indirect_rx_generation;
};

const struct virtio_msg_bus_provider_ops *virtio_msg_ffa_bus_ops_get(void);
const struct virtio_msg_bus_dma_driver_provider_ops *
virtio_msg_ffa_area_provider_ops_get(void);

struct virtio_msg_ffa_area_ctx *
virtio_msg_ffa_area_ctx_alloc(struct virtio_msg_ffa_driver *drv);
void virtio_msg_ffa_area_ctx_free(struct virtio_msg_ffa_area_ctx *ctx);
int virtio_msg_ffa_area_ctx_send_bus_req
		(struct virtio_msg_ffa_area_ctx *ctx, u8 msg_id,
		 const void *payload, size_t payload_len,
		 struct virtio_msg *resp, size_t resp_buf_len,
		 size_t *resp_len);

int virtio_msg_ffa_topology_init(struct virtio_msg_ffa_driver *drv);
void virtio_msg_ffa_topology_destroy(struct virtio_msg_ffa_topology *topo);
void virtio_msg_ffa_topology_schedule(struct virtio_msg_ffa_driver *drv);
int virtio_msg_ffa_topology_handle_event(struct virtio_msg_ffa_driver *drv,
					 const struct virtio_msg_bus_event_device *event);
int virtio_msg_ffa_topology_handle_transport_event(struct virtio_msg_ffa_driver *drv,
						   const struct virtio_msg *msg);
int virtio_msg_ffa_topology_send_bus_request(struct virtio_msg_ffa_driver *drv,
					     u8 msg_id, const void *payload,
					     size_t payload_len,
					     struct virtio_msg *resp,
					     size_t resp_buf_len,
					     size_t *resp_len);

struct virtio_msg_ffa_driver *
virtio_msg_ffa_vmdev_callback_begin(struct virtio_msg_transport_device *vmdev);
void virtio_msg_ffa_vmdev_callback_end(struct virtio_msg_transport_device *vmdev);
void
virtio_msg_ffa_vmdev_callback_wait_zero(struct virtio_msg_transport_device *vmdev);
int virtio_msg_ffa_vmdev_enqueue_event(struct virtio_msg_transport_device *vmdev,
				       const struct virtio_msg *request,
				       gfp_t gfp);

struct device *virtio_msg_ffa_driver_parent(struct virtio_msg_ffa_driver *drv);
struct virtio_msg_ffa_area_ctx *
virtio_msg_ffa_driver_area_ctx(struct virtio_msg_ffa_driver *drv);
int virtio_msg_ffa_driver_area_add(struct virtio_msg_ffa_driver *drv,
				   const struct virtio_msg_ffa_area *area);
int virtio_msg_ffa_driver_area_lookup(struct virtio_msg_ffa_driver *drv,
				      u16 area_id,
				      struct virtio_msg_ffa_area *out);
int virtio_msg_ffa_driver_area_mark_released(struct virtio_msg_ffa_driver *drv,
					     u16 area_id);
int virtio_msg_ffa_driver_area_remove(struct virtio_msg_ffa_driver *drv,
				      u16 area_id,
				      struct virtio_msg_ffa_area *removed);

void virtio_msg_ffa_area_endpoint_cleanup(struct virtio_msg_ffa_area_ctx *ctx);
int virtio_msg_ffa_area_handle_release(struct virtio_msg_ffa_area_ctx *ctx,
				       u16 area_id);

struct virtio_msg_ffa_topology *
virtio_msg_ffa_driver_topology_get(struct virtio_msg_ffa_driver *drv);
void virtio_msg_ffa_driver_topology_set(struct virtio_msg_ffa_driver *drv,
					struct virtio_msg_ffa_topology *topo);
int virtio_msg_ffa_driver_send_bus_request_locked(struct virtio_msg_ffa_driver *drv,
						  u8 msg_id,
						  const void *payload,
						  size_t payload_len,
						  struct virtio_msg *resp,
						  size_t resp_buf_len,
						  size_t *resp_len);
int virtio_msg_ffa_driver_event_configure_send_locked(struct virtio_msg_ffa_driver *drv,
						      enum virtio_msg_ffa_bus_event_delivery method,
						      u16 notif_id);
int virtio_msg_ffa_driver_dispatch_inbound_locked(struct virtio_msg_ffa_driver *drv,
						  const struct virtio_msg *msg,
						  size_t msg_len);
void virtio_msg_ffa_driver_trace_msg(struct virtio_msg_ffa_driver *drv,
				     const char *dir, const char *path,
				     const struct virtio_msg *msg,
				     size_t msg_len);
bool virtio_msg_ffa_driver_event_runtime_ready_locked(struct virtio_msg_ffa_driver *drv);
int virtio_msg_ffa_driver_submit_event(struct virtio_msg_ffa_driver *drv,
				       const struct virtio_msg *request);
bool
virtio_msg_ffa_driver_indirect_rx_ready_locked(struct virtio_msg_ffa_driver *drv);

/*
 * init_runtime() owns transient busy handling. Once a method is rejected, the
 * generic selector must not see -EBUSY as a retry signal.
 */
static inline int virtio_msg_ffa_driver_init_runtime_reject(int ret)
{
	return ret == -EBUSY ? -EIO : ret;
}

#ifdef CONFIG_VIRTIO_MSG_FFA_XFER_INDIRECT
int virtio_msg_ffa_driver_indirect_rx_init(struct virtio_msg_ffa_driver *drv);
void virtio_msg_ffa_driver_indirect_rx_quiesce(struct virtio_msg_ffa_driver *drv);
bool virtio_msg_ffa_indirect_rx_cb(struct ffa_device *fdev, u16 sender_vm_id,
				   const uuid_t *uuid, const void *buf,
				   size_t len);
#else
static inline int
virtio_msg_ffa_driver_indirect_rx_init(struct virtio_msg_ffa_driver *drv)
{
	(void)drv;

	return 0;
}

static inline void
virtio_msg_ffa_driver_indirect_rx_quiesce(struct virtio_msg_ffa_driver *drv)
{
	(void)drv;
}
#endif
void virtio_msg_ffa_driver_handle_device_failure_locked(struct virtio_msg_ffa_driver *drv,
							u16 dev_num);
void virtio_msg_ffa_driver_handle_endpoint_failure_locked(struct virtio_msg_ffa_driver *drv,
							  int err);
int virtio_msg_ffa_driver_prepare_runtime(struct virtio_msg_ffa_driver *drv);
int virtio_msg_ffa_driver_event_configure(struct virtio_msg_ffa_driver *drv);
void virtio_msg_ffa_endpoint_disable(struct virtio_msg_ffa_driver *drv);
bool virtio_msg_ffa_endpoint_disabled(struct virtio_msg_ffa_driver *drv);
bool virtio_msg_ffa_driver_uuid_match(const uuid_t *uuid);

#endif /* _VIRTIO_MSG_BUS_FFA_DRIVER_PRIV_H */
