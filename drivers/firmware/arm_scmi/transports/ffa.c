// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2021 Linaro Ltd.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/uaccess.h>
#include <linux/uuid.h>
#include <uapi/linux/tee.h>
#include <linux/arm_ffa.h>
#include <linux/scatterlist.h>

#include "../common.h"

#define SCMI_FFA_MAX_MSG_SIZE		128

enum scmi_ffa_pta_cmd {
	/*
	 * FFA_SCMI_CMD_CAPABILITIES - Get channel capabilities
	 *
	 * [out]    data0: Cmd FFA_SCMI_CMD_CAPABILITIES
	 * [in]    data1: Capability bit mask
	 */
	FFA_SCMI_CMD_CAPABILITIES = 0,

	/*
	 * FFA_SCMI_CMD_GET_CHANNEL - Get channel handle
	 *
	 * [out]    data0: Cmd FFA_SCMI_CMD_GET_CHANNEL
	 * [out]    data1: Channel identifier
	 * [in]     data1: Returned channel handle
	 * [out]    data2: Shared memory handle (optional)
	 */
	FFA_SCMI_CMD_GET_CHANNEL = 1,

	/*
	 * FFA_SCMI_CMD_MSG_SEND_DIRECT_REQ - Process direct SCMI message
	 * with shared memory
	 *
	 * [out]    data0: Cmd FFA_SCMI_CMD_MSG_SEND_DIRECT_REQ
	 * [out]    data1: Channel handle
	 * [in/out] data2: Response size
	 *
	 */
	FFA_SCMI_CMD_MSG_SEND_DIRECT_REQ = 2,

	/*
	 * FFA_SCMI_CMD_SEND_MSG2 - Process SCMI message in RXTX buffer
	 *
	 * Use FFA RX/TX message to exchange request with the SCMI server.
	 */
	FFA_SCMI_CMD_MSG_SEND2 = 3,
};

/*
 * FFA SCMI service capabilities bit flags (32bit)
 *
 * FFA_SCMI_CAPS_RXTX_BUFFER
 * When set, FFA transport layer uses the FFA RX/TX buffer with indirect message
 * command FFA_MSG_SEND2.
 *
 * FFA_SCMI_CAPS_SHARED_BUFFER
 * When set, FFA transport layer shares a page with the server. Direct message
 * is used to send command and receive response. Notification is not supported.
 */
#define FFA_SCMI_CAPS_NONE		0
#define FFA_SCMI_CAPS_RXTX_BUFFER	BIT(0)
#define FFA_SCMI_CAPS_SHARED_BUFFER	BIT(1)
#define FFA_SCMI_CAPS_MASK		(FFA_SCMI_CAPS_RXTX_BUFFER | \
					 FFA_SCMI_CAPS_SHARED_BUFFER)

/**
 * struct scmi_ffa_channel - Description of an FFA SCMI channel
 *
 * @channel_id: FFA channel ID used for this transport
 * @tee_shm: Shared memory object or NULL if using RX/TX buffer
 * @rx_len: Response size
 * @mu: Mutex protection on channel access
 * @cinfo: SCMI channel information
 * @req: Shared memory buffer
 * @link: Reference in agent's channel list
 */
struct scmi_ffa_channel {
	u32 channel_id;
	struct tee_shm *tee_shm;
	u32 rx_len;
	struct mutex mu;
	struct scmi_chan_info *cinfo;
	union {
		struct scmi_shared_mem __iomem *shmem;
		struct scmi_msg_payld *msg;
	} req;
	struct list_head link;
};

/**
 * struct scmi_ffa_service - FFA transport private data
 *
 * @dev: Device used for communication with Secure Partition
 * @caps: Supported capabilities
 * @mu: Mutex for protection of @channel_list
 * @channel_list: List of all created channels for the agent
 */
struct scmi_ffa_service {
	struct ffa_device *ffa_dev;
	u32 caps;
	struct mutex mu;
	struct list_head channel_list;
};

/* There can be only 1 SCMI service in FFA we connect to */
static struct scmi_ffa_service *ffa_service;

static struct scmi_transport_core_operations *core;

static int get_capabilities(struct scmi_ffa_service *service)
{
	struct ffa_device *ffa_dev = service->ffa_dev;
	const struct ffa_msg_ops *msg_ops = ffa_dev->ops->msg_ops;
	struct ffa_send_direct_data data = {
		.data0 = FFA_SCMI_CMD_CAPABILITIES,
		};
	unsigned int caps = 0;
	int ret;

	ret = msg_ops->sync_send_receive(ffa_dev, &data);
	if (ret) {
		dev_err(&ffa_dev->dev, "Can't get FFA SCMI caps %d / %#x\n",
			ret, (u32)data.data0);
		return -EOPNOTSUPP;
	}

	caps = data.data1;
	if (!(caps & FFA_SCMI_CAPS_MASK)) {
		dev_err(&ffa_dev->dev, "FFA SCMI doesn't support neither direct nor indirect message\n");
		return -EOPNOTSUPP;
	}

	if (!(caps & FFA_SCMI_CAPS_SHARED_BUFFER)) {
		dev_err(&ffa_dev->dev, "Current FFA transport version supports only direct message\n");
		return -EOPNOTSUPP;
	}

	service->caps = caps;

	return 0;
}

static int get_channel_id(struct scmi_ffa_channel *channel, uint32_t channel_id)
{
	struct ffa_device *ffa_dev = ffa_service->ffa_dev;
	const struct ffa_msg_ops *msg_ops = ffa_dev->ops->msg_ops;
	struct ffa_send_direct_data data = {
		.data0 = FFA_SCMI_CMD_GET_CHANNEL,
		.data1 = (u32)channel_id,
		.data2 = channel->tee_shm->sec_world_id,
		};
	int ret;

	ret = msg_ops->sync_send_receive(ffa_dev, &data);
	if (ret || data.data0) {
		dev_err(&ffa_dev->dev, "Can't get channel id 0x%x : %d / %#x\n",
			channel_id, ret, (u32)data.data0);
		return -EOPNOTSUPP;
	}

	/* From now on use channel identifer provided by FFA SCMI service */
	channel->channel_id = data.data1;

	return 0;
}

static int invoke_ffa_indirect_msg_channel(struct scmi_ffa_channel *channel, void  *msg,  size_t msg_size)
{
	struct ffa_device *ffa_dev = ffa_service->ffa_dev;

	dev_dbg(&ffa_dev->dev, "%s channel_id 0x%x size %lu\n", __func__, channel->channel_id, msg_size);

	return -EIO;
}

static int invoke_ffa_direct_msg_channel(struct scmi_ffa_channel *channel, size_t msg_size)
{
	struct ffa_device *ffa_dev = ffa_service->ffa_dev;
	const struct ffa_msg_ops *msg_ops = ffa_dev->ops->msg_ops;
	struct ffa_send_direct_data data = {
		.data0 = FFA_SCMI_CMD_MSG_SEND_DIRECT_REQ,
		.data1 = (u32)channel->channel_id,
		.data2 = msg_size,
	};
	int ret;

	dev_dbg(&ffa_dev->dev, "%s channel_id 0x%x size %lu\n", __func__, channel->channel_id, msg_size);

	ret = msg_ops->sync_send_receive(ffa_dev, &data);
	if (ret || data.data0) {
		dev_err(&ffa_dev->dev, "Can't invoke channel %x: %d / %#x\n",
			channel->channel_id, ret, (u32)data.data0);
		return -EIO;
	}

	/* Save response size */
	channel->rx_len = data.data2;

	return 0;
}

static bool scmi_ffa_chan_available(struct device_node *of_node, int idx)
{
	u32 channel_id;

	return !of_property_read_u32_index(of_node, "linaro,ffa-channel-id",
					   idx, &channel_id);
}

static int ffa_memory_share(struct ffa_device *ffa_dev, struct tee_shm *shm, unsigned long start)
{
	const struct ffa_mem_ops *mem_ops = ffa_dev->ops->mem_ops;
	struct ffa_mem_region_attributes mem_attr = {
		.receiver = ffa_dev->vm_id,
		.attrs = FFA_MEM_RW,
	};
	struct ffa_mem_ops_args args = {
		.use_txbuf = true,
		.attrs = &mem_attr,
		.nattrs = 1,
	};
	struct sg_table sgt;
	int rc = 0;

	shm->sec_world_id = 0xDEADBEEF;

	/* Share the pages with SCMI server SP */
	rc = sg_alloc_table_from_pages(&sgt, shm->pages, shm->num_pages, 0,
				       shm->num_pages * PAGE_SIZE, GFP_KERNEL);
	if (rc)
		return rc;

	args.sg = sgt.sgl;
	rc = mem_ops->memory_share(&args);
	sg_free_table(&sgt);
	if (rc) {
		return rc;
	}

	/* Save memory handle */
	shm->sec_world_id = args.g_handle;
	return 0;
}

static int ffa_shm_alloc_kernel_buf(struct scmi_ffa_channel *channel)
{
	size_t msg_size = SCMI_FFA_MAX_MSG_SIZE;
	struct ffa_device *ffa_dev = ffa_service->ffa_dev;
	struct page *page;
	struct tee_shm *shm;
	void *shbuf;
	int ret;

	/*
	 * If only MSG_SEND_DIRECT_REQ is supported, allocate and share a page
	 * Otherwise use RX/TX buffer
	 */
	if (ffa_service->caps & ~(FFA_SCMI_CAPS_SHARED_BUFFER))
		return 0;

	/*
	 * Ignore alignment since this is already going to be page aligned
	 * and there's no need for any larger alignment.
	 */
	page = alloc_pages(GFP_KERNEL | __GFP_ZERO, 1);
	if (!page)
		return -ENOMEM;

	/* Allocate shared memory object */
	shm = kzalloc(sizeof(*shm), GFP_KERNEL);
	if (!shm) {
		ret = -ENOMEM;
		goto err_free_page;
	}

	/* Init shared memory object */
	shm->flags = 0;
	shm->kaddr = page_address(page);
	shm->paddr = page_to_phys(page);
	shm->size = PAGE_SIZE;
	shm->offset = 0;

	shm->pages = kcalloc(1, sizeof(*shm->pages), GFP_KERNEL);
	if (!shm->pages) {
		ret = -ENOMEM;
		goto err_free_shm;
	}

	shm->pages[0] = page;
	shm->num_pages = 1;

	ret = ffa_memory_share(ffa_dev, shm, (unsigned long)shm->kaddr);
	if (ret)
		goto err_put_shm_pages;

	/* Init Channel message */
	shbuf = tee_shm_get_va(shm, 0);
	memset(shbuf, 0, msg_size);
	channel->tee_shm = shm;

	channel->req.msg = shbuf;
	channel->rx_len = msg_size;

	return 0;

err_put_shm_pages:
	kfree(shm->pages);
err_free_shm:
	kfree(shm);
err_free_page:
	free_pages((unsigned long)page_address(page), 1);
	return ret;
}

static void ffa_shm_free_kernel_buf(struct tee_shm *shm)
{
	struct ffa_device *ffa_dev = ffa_service->ffa_dev;
	const struct ffa_mem_ops *mem_ops = ffa_dev->ops->mem_ops;
	int ret;

	/* Nothing to free */
	if (shm == NULL)
		return;

	/* Reclaim shared memory */
	ret = mem_ops->memory_reclaim(shm->sec_world_id, 0);
	if (ret)
		pr_err("mem_reclaim: 0x%llx %d", shm->sec_world_id, ret);

	free_pages((unsigned long)shm->kaddr, get_order(shm->size));
	kfree(shm);
}

static int scmi_ffa_chan_setup(struct scmi_chan_info *cinfo, struct device *dev, bool tx)
{
	struct scmi_ffa_channel *channel;
	uint32_t channel_id;
	int ret;

	/* Notification not supported so far */
	if (!tx)
		return -ENODEV;

	channel = devm_kzalloc(dev, sizeof(*channel), GFP_KERNEL);
	if (!channel)
		return -ENOMEM;

	ret = of_property_read_u32_index(cinfo->dev->of_node, "linaro,ffa-channel-id",
					 0, &channel_id);
	if (ret)
		return ret;

	cinfo->transport_info = channel;
	channel->cinfo = cinfo;
	mutex_init(&channel->mu);

	/*
	 * Alloc and Share mem with SCMI SP if necessary
	 */
	ret = ffa_shm_alloc_kernel_buf(channel);
	if (ret)
		return ret;

	/* Get SCMI channel Id */
	ret = get_channel_id(channel, channel_id);
	if (ret) {
		ffa_shm_free_kernel_buf(channel->tee_shm);
		return ret;
	}

	/* Enable polling */
	cinfo->no_completion_irq = true;

	mutex_lock(&ffa_service->mu);
	list_add(&channel->link, &ffa_service->channel_list);
	mutex_unlock(&ffa_service->mu);

	return 0;
}

static int scmi_ffa_chan_free(int id, void *p, void *data)
{
	struct scmi_chan_info *cinfo = p;
	struct scmi_ffa_channel *channel = cinfo->transport_info;

	mutex_lock(&ffa_service->mu);
	list_del(&channel->link);
	mutex_unlock(&ffa_service->mu);

	ffa_shm_free_kernel_buf(channel->tee_shm);

	cinfo->transport_info = NULL;
	channel->cinfo = NULL;

	return 0;
}

static int scmi_ffa_send_message(struct scmi_chan_info *cinfo,
				   struct scmi_xfer *xfer)
{
	struct scmi_ffa_channel *channel = cinfo->transport_info;
	int ret;

	mutex_lock(&channel->mu);

	/* Fill the message buffer */
	core->msg->tx_prepare(channel->req.msg, xfer);

	if (channel->tee_shm) {
		/* With direct message we share the message buffer with server */
		ret = invoke_ffa_direct_msg_channel(channel,
						    core->msg->command_size(xfer));
	} else {
		/* With indirect message, the message buffer is cpoied in ffa buffer */
		ret = invoke_ffa_indirect_msg_channel(channel,
						      channel->req.msg,
						      core->msg->command_size(xfer));
	}

	if (ret)
		mutex_unlock(&channel->mu);

	return ret;
}

static void scmi_ffa_fetch_response(struct scmi_chan_info *cinfo,
				      struct scmi_xfer *xfer)
{
	struct scmi_ffa_channel *channel = cinfo->transport_info;

	core->msg->fetch_response(channel->req.msg, channel->rx_len, xfer);
}

static void scmi_ffa_mark_txdone(struct scmi_chan_info *cinfo, int ret,
				   struct scmi_xfer *__unused)
{
	struct scmi_ffa_channel *channel = cinfo->transport_info;

	mutex_unlock(&channel->mu);
}

static struct scmi_transport_ops scmi_ffa_ops = {
	.chan_available = scmi_ffa_chan_available,
	.chan_setup = scmi_ffa_chan_setup,
	.chan_free = scmi_ffa_chan_free,
	.send_message = scmi_ffa_send_message,
	.mark_txdone = scmi_ffa_mark_txdone,
	.fetch_response = scmi_ffa_fetch_response,
};

static struct scmi_desc scmi_ffa_desc = {
	.ops = &scmi_ffa_ops,
	.max_rx_timeout_ms = 300,
	.max_msg = 20,
	.max_msg_size = SCMI_FFA_MAX_MSG_SIZE,
	.sync_cmds_completed_on_ret = true,
};

static const struct of_device_id scmi_of_match[] = {
	{ .compatible = "linaro,scmi-ffa" },
	{ /* Sentinel */ },
};

DEFINE_SCMI_TRANSPORT_DRIVER(scmi_ffa, scmi_ffa_driver, scmi_ffa_desc,
			     scmi_of_match, core);

/* FF-A service registration */

#define SCMI_FFA_VERSION_MAJOR	1
#define SCMI_FFA_VERSION_MINOR	0

static bool scmi_ffa_api_is_compatible(struct ffa_device *ffa_dev,
					const struct ffa_ops *ops)
{
	const struct ffa_msg_ops *msg_ops = ops->msg_ops;
	const struct ffa_info_ops *info_ops = ops->info_ops;
	u32 version;

	// TODO understand why 32bits mode works whereas native got some problems
	msg_ops->mode_32bit_set(ffa_dev);

	version = info_ops->api_version_get();

	if (version < (SCMI_FFA_VERSION_MAJOR << 16 | SCMI_FFA_VERSION_MINOR)) {
		dev_err(&ffa_dev->dev, "scmi_ffa_api_is_NOT_compatible %x", version);
		return false;
	}

	return true;
}

static int scmi_ffa_service_probe(struct ffa_device *ffa_dev)
{
	struct device *dev = &ffa_dev->dev;
	struct scmi_ffa_service *service;
	const struct ffa_ops *ffa_ops;
	int ret;

	/* Only one SCMI FFA device allowed */
	if (ffa_service) {
		dev_err(dev, "A SCMI FFA device was already initialized: only one allowed\n");
		return -EBUSY;
	}

	ffa_ops = ffa_dev->ops;

	if (!scmi_ffa_api_is_compatible(ffa_dev, ffa_ops))
		return -EINVAL;

	service = devm_kzalloc(dev, sizeof(*service), GFP_KERNEL);
	if (!service)
		return -ENOMEM;

	service->ffa_dev = ffa_dev;
	INIT_LIST_HEAD(&service->channel_list);
	mutex_init(&service->mu);

	ret = get_capabilities(service);
	if (ret)
		goto err;

	ret = platform_driver_register(&scmi_ffa_driver);
	if (ret) {
		service = NULL;
		goto err;
	}

	ffa_dev_set_drvdata(ffa_dev, service);

	/* Ensure ffa service is visible */
	smp_store_mb(ffa_service, service);

	dev_dbg(&ffa_dev->dev, "Probed with vm_id %x uuid %pUl\n", ffa_dev->vm_id, &ffa_dev->uuid );

err:
	return ret;
}

static void scmi_ffa_service_remove(struct ffa_device *ffa_dev)
{
	struct scmi_ffa_service *service = ffa_dev_get_drvdata(ffa_dev);

	if (!service)
		return;

	platform_driver_unregister(&scmi_ffa_driver);

	if (!list_empty(&ffa_service->channel_list))
		return;

	/* Ensure cleared reference is visible before resources are released */
	smp_store_mb(ffa_service, NULL);

	mutex_destroy(&service->mu);

	dev_dbg(&ffa_dev->dev, "%s id %u vm_id %x uuid %pUl\n", __func__, ffa_dev->id, ffa_dev->vm_id, &ffa_dev->uuid );
}

static const struct ffa_device_id scmi_ffa_service_id[] = {
	{ UUID_INIT(0x79b55c73, 0x1d8c, 0x44b9,
		    0x85,0x93,0x61,0xe1, 0x77,0x0a,0xd8,0xd2) },
//	{ UUID_INIT(0xb4b5671e, 0x4a90, 0x4fe1,
//		    0xb8,0x1f,0xfb,0x13, 0xda,0xe1,0xda,0xcb) },
	{}
};
//MODULE_DEVICE_TABLE(ffa, scmi_ffa_service_id);

static struct ffa_driver scmi_ffa_service_driver = {
	.name = "ffa-scmi",
	.probe = scmi_ffa_service_probe,
	.remove = scmi_ffa_service_remove,
	.id_table = scmi_ffa_service_id,
};

static int scmi_transport_ffa_init(void)
{
	if (IS_REACHABLE(CONFIG_ARM_FFA_TRANSPORT))
		return ffa_register(&scmi_ffa_service_driver);
	else
		return -EOPNOTSUPP;
}
module_init(scmi_transport_ffa_init);

static void scmi_transport_ffa_exit(void)
{
	if (IS_REACHABLE(CONFIG_ARM_FFA_TRANSPORT))
		ffa_unregister(&scmi_ffa_service_driver);
}
module_exit(scmi_transport_ffa_exit);

MODULE_AUTHOR("Vincent Guittot <vincent.guittot@linaro.org>");
MODULE_DESCRIPTION("SCMI FFA Transport driver");
MODULE_LICENSE("GPL");
