/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Virtio message transport header.
 *
 * Copyright (C) 2026 Google LLC and Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 */

#ifndef _DRIVERS_VIRTIO_MSG_INTERNAL_H
#define _DRIVERS_VIRTIO_MSG_INTERNAL_H

#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/virtio.h>
#include <linux/wait.h>
#include <uapi/linux/virtio_msg.h>

/* Fixed tokens used by kernel */
#define TOKEN_EVENT	0
#define TOKEN_FIXED	1

struct reserved_mem;
struct virtio_msg_device;

/*
 * struct virtio_msg_ops - Virtio message bus operations.
 * @bus_info: Return bus information.
 * @transfer: Transfer a message.
 * @synchronize_cbs: Synchronize with the virtqueue callbacks (optional).
 * @release: Release the resources corresponding to the device (optional).
 */
struct virtio_msg_ops {
	const char *(*bus_info)(struct virtio_msg_device *vmdev, u16 *msg_size, u32 *rev);
	int (*transfer)(struct virtio_msg_device *vmdev, struct virtio_msg *request,
			struct virtio_msg *response);
	void (*synchronize_cbs)(struct virtio_msg_device *vmdev);
	void (*release)(struct virtio_msg_device *vmdev);
};

/*
 * Representation of a device using virtio message
 * transport.
 */
struct virtio_msg_device {
	struct virtio_device vdev;
	struct virtio_msg_ops *ops;
	const char *bus_name;
	void *bus_data;
	u32 generation_count;
	u32 config_size;
	u16 msg_size;
	u16 dev_id;

	struct virtio_msg *request;
	struct virtio_msg *response;
};

int virtio_msg_register(struct virtio_msg_device *vmdev);
void virtio_msg_unregister(struct virtio_msg_device *vmdev);

void virtio_msg_prepare(struct virtio_msg *vmsg, u8 msg_id,
			u16 token, u16 payload_size);
int virtio_msg_event(struct virtio_msg_device *vmdev, struct virtio_msg *vmsg);

/* Virtio msg userspace interface */
struct virtio_msg_user_device;

struct virtio_msg_user_ops {
	int (*handle)(struct virtio_msg_user_device *vmudev, struct virtio_msg *vmsg);
};

/* Host side device using virtio message */
struct virtio_msg_user_device {
	struct virtio_msg_user_ops *ops;
	struct miscdevice misc;
	wait_queue_head_t wait;
	struct virtio_msg *vmsg;
	struct device *parent;
	char *name;
	u8 id;
};

#if IS_REACHABLE(CONFIG_VIRTIO_MSG_USER)
int virtio_msg_user_register(struct virtio_msg_user_device *vmudev);
void virtio_msg_user_unregister(struct virtio_msg_user_device *vmudev);
#else
static inline int virtio_msg_user_register(struct virtio_msg_user_device *vmudev)
{
	return -EOPNOTSUPP;
}

static inline void virtio_msg_user_unregister(struct virtio_msg_user_device *vmudev) {}
#endif /* CONFIG_VIRTIO_MSG_USER */

#if IS_REACHABLE(CONFIG_VIRTIO_MSG_FFA_DMA_OPS)
struct ffa_device;
struct scatterlist;

extern const struct dma_map_ops virtio_msg_ffa_rmem_dma_ops;
extern const struct dma_map_ops virtio_msg_ffa_dev_dma_ops;

int vmsg_ffa_bus_area_share_sgl(struct ffa_device *ffa_dev,
				struct scatterlist *sgl, int nents,
				dma_addr_t *dma_handle);
int vmsg_ffa_bus_area_share(struct ffa_device *ffa_dev, dma_addr_t *dma_handle,
			    size_t n_pages);
int vmsg_ffa_bus_area_unshare(struct ffa_device *ffa_dev, dma_addr_t *dma_handle);

int virtio_msg_ffa_dma_init(void);
#else
static inline int virtio_msg_ffa_dma_init(void)
{
	return 0;
}
#endif

#endif /* _DRIVERS_VIRTIO_MSG_INTERNAL_H */
