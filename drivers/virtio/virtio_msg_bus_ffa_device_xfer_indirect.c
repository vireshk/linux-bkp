// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio message bus over FF-A device-role indirect transfer runtime.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#include <linux/byteorder/little_endian.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uuid.h>
#include <linux/workqueue.h>

#include "virtio_msg_bus_ffa_device_priv.h"
#include "virtio_msg_bus_ffa_xfer_indirect.h"

#define VIRTIO_MSG_FFA_INDIRECT_RX_QUEUE_DEPTH		128
#define VIRTIO_MSG_FFA_INDIRECT_TX_QUEUE_DEPTH		128
#define VIRTIO_MSG_FFA_INDIRECT_TX_RETRY_DELAY_MS	10

struct virtio_msg_ffa_device_indirect_rx {
	u64 generation;
	u16 sender_vm_id;
	u16 len;
	u8 msg[FFA_BUS_MAX_MSG_SIZE];
};

struct virtio_msg_ffa_device_indirect_tx {
	struct list_head node;
	u64 generation;
	size_t len;
	u8 msg[FFA_BUS_MAX_MSG_SIZE];
};

struct virtio_msg_ffa_device_indirect {
	struct virtio_msg_ffa_device *vdev;
	struct virtio_msg_bus_queue_helper rx_queue;
	spinlock_t tx_lock; /* Protects queued indirect TX frames. */
	struct list_head tx_queue;
	struct delayed_work tx_work;
	u64 rx_generation;
	u64 tx_generation;
	u32 tx_depth;
};

static const uuid_t virtio_msg_ffa_device_uuid = VIRTIO_MSG_FFA_DEVICE_UUID;
static const struct virtio_msg_ffa_xfer_method_desc
	virtio_msg_ffa_indirect_device_method_desc;

static struct virtio_msg_ffa_device_indirect *
virtio_msg_ffa_device_indirect_get_locked(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_indirect *priv;

	if (!vdev)
		return NULL;

	lockdep_assert_held(&vdev->lock);

	priv = rcu_dereference_protected(vdev->indirect,
					 lockdep_is_held(&vdev->lock));
	if (!priv || priv->vdev != vdev)
		return NULL;

	return priv;
}

static struct virtio_msg_ffa_device_indirect *
virtio_msg_ffa_device_indirect_get_rcu(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_indirect *priv;

	if (!vdev)
		return NULL;

	priv = rcu_dereference(vdev->indirect);
	if (!priv || priv->vdev != vdev)
		return NULL;

	return priv;
}

bool virtio_msg_ffa_device_indirect_capable(const struct virtio_msg_ffa_device *vdev)
{
	if (!vdev || !vdev->fdev || !vdev->fdev->ops ||
	    !vdev->fdev->ops->msg_ops)
		return false;
	if (!vdev->fdev->ops->msg_ops->indirect_send)
		return false;

	return ffa_partition_supports_indirect_msg(vdev->fdev);
}

static void
virtio_msg_ffa_device_indirect_tx_purge
		(struct virtio_msg_ffa_device_indirect *priv)
{
	struct virtio_msg_ffa_device_indirect_tx *tx;
	struct virtio_msg_ffa_device_indirect_tx *tmp;
	LIST_HEAD(free_list);
	unsigned long flags;

	if (!priv)
		return;

	spin_lock_irqsave(&priv->tx_lock, flags);
	virtio_msg_ffa_queue_splice_locked(&priv->tx_queue,
					   &priv->tx_depth, &free_list);
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	list_for_each_entry_safe(tx, tmp, &free_list, node) {
		list_del(&tx->node);
		kfree(tx);
	}
}

static int
virtio_msg_ffa_device_indirect_tx_defer_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg *msg, size_t msg_len)
{
	struct virtio_msg_ffa_device_indirect_tx *tx;
	struct virtio_msg_ffa_device_indirect *priv;
	unsigned long flags;
	u32 depth;
	int ret;

	lockdep_assert_held(&vdev->lock);

	priv = virtio_msg_ffa_device_indirect_get_locked(vdev);
	if (!priv || !msg)
		return -EINVAL;
	if (msg_len > FFA_BUS_MAX_MSG_SIZE)
		return -EMSGSIZE;

	tx = kmalloc(sizeof(*tx), GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	tx->generation = priv->tx_generation;
	tx->len = msg_len;
	memcpy(tx->msg, msg, msg_len);

	spin_lock_irqsave(&priv->tx_lock, flags);
	if (READ_ONCE(vdev->shutting_down)) {
		ret = -ESHUTDOWN;
		goto out_unlock_free;
	}
	if (!READ_ONCE(vdev->endpoint_registered)) {
		ret = -ENODEV;
		goto out_unlock_free;
	}

	ret = virtio_msg_ffa_queue_push_tail_locked
			(&priv->tx_queue, &tx->node, &priv->tx_depth,
			 VIRTIO_MSG_FFA_INDIRECT_TX_QUEUE_DEPTH);
	if (ret)
		goto out_unlock_free;

	depth = priv->tx_depth;
	schedule_delayed_work(&priv->tx_work,
			      msecs_to_jiffies
				(VIRTIO_MSG_FFA_INDIRECT_TX_RETRY_DELAY_MS));
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	if (vdev->fdev)
		dev_dbg(&vdev->fdev->dev,
			"bus tx indirect deferred: dev=%u tok=%u depth=%u\n",
			le16_to_cpu(msg->dev_num), le16_to_cpu(msg->token),
			depth);
	else
		pr_debug("bus tx indirect deferred: dev=%u tok=%u depth=%u\n",
			 le16_to_cpu(msg->dev_num), le16_to_cpu(msg->token),
			 depth);
	return 0;

out_unlock_free:
	spin_unlock_irqrestore(&priv->tx_lock, flags);
	kfree(tx);
	return ret;
}

static int
virtio_msg_ffa_device_send_indirect_now_locked
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg *msg, size_t msg_len, bool defer_busy)
{
	struct virtio_msg_ffa_xfer_ctx xfer_ctx = { 0 };
	struct virtio_msg_ffa_device_indirect *priv;
	const struct ffa_msg_ops *msg_ops;
	struct virtio_msg *wire_msg;
	u8 wire_buf[FFA_BUS_MAX_MSG_SIZE];
	size_t wire_len;
	unsigned int retry = VIRTIO_MSG_FFA_TX_BUSY_RETRY_MAX;
	int ret = -EIO;

	lockdep_assert_held(&vdev->lock);

	priv = virtio_msg_ffa_device_indirect_get_locked(vdev);
	if (!priv || !msg)
		return -EINVAL;
	if (!virtio_msg_ffa_device_indirect_capable(vdev))
		return -EOPNOTSUPP;
	if (READ_ONCE(vdev->shutting_down))
		return -ESHUTDOWN;

	msg_ops = vdev->fdev->ops->msg_ops;
	xfer_ctx.ep = &vdev->ep;
	xfer_ctx.fdev = vdev->fdev;
	xfer_ctx.priv = priv;
	ret = virtio_msg_ffa_indirect_prepare_wire_request
			(&xfer_ctx, msg, msg_len, wire_buf, sizeof(wire_buf));
	if (ret)
		return ret;

	wire_msg = (struct virtio_msg *)wire_buf;
	wire_len = le16_to_cpu(wire_msg->msg_size);
	if (wire_len < sizeof(*wire_msg) || wire_len > sizeof(wire_buf))
		return -EMSGSIZE;

	virtio_msg_ffa_device_trace_msg(vdev, "tx", "indirect", wire_msg,
					wire_len);
	for (;;) {
		ret = msg_ops->indirect_send(vdev->fdev, wire_msg, wire_len);
		if (ret == -EBUSY) {
			if (!virtio_msg_ffa_tx_busy_retry_sleepable(&retry))
				break;
			continue;
		}
		if (ret)
			goto out_trace;
		return 0;
	}

out_trace:
	dev_dbg(&vdev->fdev->dev,
		"bus tx indirect failed: peer_vm=%u dev=%u tok=%u ret=%d\n",
		(u16)vdev->fdev->vm_id, le16_to_cpu(wire_msg->dev_num),
		le16_to_cpu(wire_msg->token), ret);
	if (ret == -EBUSY && defer_busy)
		return virtio_msg_ffa_device_indirect_tx_defer_locked
				(vdev, msg, msg_len);
	return ret;
}

int virtio_msg_ffa_device_send_indirect_locked(struct virtio_msg_ffa_device *vdev,
					       const struct virtio_msg *msg,
					       size_t msg_len)
{
	return virtio_msg_ffa_device_send_indirect_now_locked(vdev, msg, msg_len,
							     true);
}

static void virtio_msg_ffa_device_indirect_tx_work(struct work_struct *work)
{
	struct virtio_msg_ffa_device_indirect_tx *tx;
	struct virtio_msg_ffa_device_indirect *priv;
	struct virtio_msg_ffa_device *vdev;
	struct list_head *node;
	unsigned long flags;
	int ret;

	priv = container_of(to_delayed_work(work),
			    struct virtio_msg_ffa_device_indirect, tx_work);
	vdev = priv->vdev;
	if (!vdev)
		return;

	for (;;) {
		spin_lock_irqsave(&priv->tx_lock, flags);
		node = virtio_msg_ffa_queue_pop_locked(&priv->tx_queue,
						       &priv->tx_depth);
		if (!node) {
			spin_unlock_irqrestore(&priv->tx_lock, flags);
			return;
		}
		spin_unlock_irqrestore(&priv->tx_lock, flags);

		tx = list_entry(node, struct virtio_msg_ffa_device_indirect_tx,
				node);

		mutex_lock(&vdev->lock);
		if (READ_ONCE(vdev->shutting_down) ||
		    !READ_ONCE(vdev->endpoint_registered) ||
		    tx->generation != priv->tx_generation) {
			ret = -ESHUTDOWN;
		} else {
			ret = virtio_msg_ffa_device_send_indirect_now_locked
				(vdev, (const struct virtio_msg *)tx->msg,
				 tx->len, false);
		}
		mutex_unlock(&vdev->lock);

		if (ret == -EAGAIN || ret == -EBUSY || ret == -ENOSPC) {
			spin_lock_irqsave(&priv->tx_lock, flags);
			if (READ_ONCE(vdev->shutting_down) ||
			    tx->generation != READ_ONCE(priv->tx_generation)) {
				spin_unlock_irqrestore(&priv->tx_lock, flags);
				kfree(tx);
				return;
			}
			virtio_msg_ffa_queue_push_head_locked
				(&priv->tx_queue, &tx->node, &priv->tx_depth);
			spin_unlock_irqrestore(&priv->tx_lock, flags);

			schedule_delayed_work
				(&priv->tx_work,
				 msecs_to_jiffies
					(VIRTIO_MSG_FFA_INDIRECT_TX_RETRY_DELAY_MS));
			return;
		}

		if (ret && vdev->fdev) {
			struct virtio_msg *msg = (struct virtio_msg *)tx->msg;

			dev_warn_ratelimited(&vdev->fdev->dev,
					     "deferred indirect TX dropped dev=%u tok=%u ret=%d\n",
					     le16_to_cpu(msg->dev_num),
					     le16_to_cpu(msg->token), ret);
		}

		kfree(tx);
	}
}

static int
virtio_msg_ffa_device_indirect_rx_queue_submit
		(void *context,
		 const struct virtio_msg_bus_queue_helper_item *item)
{
	struct virtio_msg_ffa_device_indirect_rx *entry;
	struct virtio_msg_ffa_device *vdev = context;
	struct virtio_msg_ffa_device_indirect *priv;
	const struct virtio_msg *msg;
	size_t msg_len;
	int ret = 0;

	if (!vdev || !item ||
	    item->type != VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_OPAQUE)
		return -EINVAL;

	entry = item->opaque;
	if (!entry)
		return -EINVAL;

	msg = (const struct virtio_msg *)entry->msg;
	if (virtio_msg_ffa_validate_inbound_msg(msg, entry->len))
		return -EINVAL;

	msg_len = le16_to_cpu(msg->msg_size);

	mutex_lock(&vdev->lock);
	priv = virtio_msg_ffa_device_indirect_get_locked(vdev);
	if (!priv || READ_ONCE(vdev->shutting_down) ||
	    !READ_ONCE(vdev->endpoint_registered) ||
	    entry->generation != READ_ONCE(priv->rx_generation) ||
	    entry->sender_vm_id != vdev->peer_vm_id) {
		ret = 0;
	} else {
		virtio_msg_ffa_device_trace_msg(vdev, "rx", "indirect", msg,
						msg_len);
		ret = virtio_msg_ffa_device_dispatch_inbound_locked
			(vdev, msg, msg_len);
	}
	mutex_unlock(&vdev->lock);

	return ret;
}

static void
virtio_msg_ffa_device_indirect_rx_queue_release
		(void *context,
		 const struct virtio_msg_bus_queue_helper_item *item, int status)
{
	(void)context;
	(void)status;

	if (!item || item->type != VIRTIO_MSG_BUS_QUEUE_HELPER_ITEM_OPAQUE)
		return;

	kfree(item->opaque);
}

static int
virtio_msg_ffa_device_indirect_rx_enqueue(struct virtio_msg_ffa_device *vdev,
					  u16 sender_vm_id,
					  const void *buf, size_t len)
{
	struct virtio_msg_ffa_device_indirect_rx *entry;
	struct virtio_msg_ffa_device_indirect *priv;
	const struct virtio_msg *msg = buf;
	int ret;

	if (!vdev || !buf || !len)
		return -EINVAL;
	if (len > FFA_BUS_MAX_MSG_SIZE)
		return -EMSGSIZE;
	if (virtio_msg_ffa_validate_inbound_msg(msg, len))
		return -EINVAL;

	entry = kzalloc(sizeof(*entry), GFP_ATOMIC);
	if (!entry)
		return -ENOSPC;

	entry->sender_vm_id = sender_vm_id;
	entry->len = len;
	memcpy(entry->msg, buf, len);

	rcu_read_lock();
	priv = virtio_msg_ffa_device_indirect_get_rcu(vdev);
	if (!priv) {
		rcu_read_unlock();
		kfree(entry);
		return -ENODEV;
	}

	entry->generation = READ_ONCE(priv->rx_generation);
	ret = virtio_msg_bus_queue_helper_enqueue_opaque(&priv->rx_queue, entry,
							 GFP_ATOMIC);
	rcu_read_unlock();
	if (ret) {
		kfree(entry);
		if (ret == -ENOMEM)
			ret = -ENOSPC;
	}

	return ret;
}

void virtio_msg_ffa_device_indirect_runtime_reset_locked(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_indirect *priv;

	lockdep_assert_held(&vdev->lock);

	priv = virtio_msg_ffa_device_indirect_get_locked(vdev);
	if (!priv)
		return;

	WRITE_ONCE(priv->rx_generation, READ_ONCE(priv->rx_generation) + 1);
	priv->tx_generation++;
	virtio_msg_ffa_device_indirect_tx_purge(priv);
}

int virtio_msg_ffa_device_indirect_runtime_init(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_indirect *priv;
	struct device *dev;
	int ret;

	if (!vdev)
		return -EINVAL;
	if (rcu_access_pointer(vdev->indirect))
		return 0;
	if (!virtio_msg_ffa_device_indirect_capable(vdev))
		return -EOPNOTSUPP;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->vdev = vdev;
	dev = vdev->fdev ? &vdev->fdev->dev : NULL;
	ret = virtio_msg_bus_queue_helper_init
		(&priv->rx_queue, dev, FFA_BUS_MAX_MSG_SIZE,
		 VIRTIO_MSG_FFA_INDIRECT_RX_QUEUE_DEPTH, 0,
		 virtio_msg_ffa_device_indirect_rx_queue_submit,
		 virtio_msg_ffa_device_indirect_rx_queue_release, vdev);
	if (ret)
		goto err_free_priv;

	spin_lock_init(&priv->tx_lock);
	INIT_LIST_HEAD(&priv->tx_queue);
	INIT_DELAYED_WORK(&priv->tx_work,
			  virtio_msg_ffa_device_indirect_tx_work);
	WRITE_ONCE(priv->rx_generation, 0);
	WRITE_ONCE(priv->tx_generation, 0);
	rcu_assign_pointer(vdev->indirect, priv);

	return 0;

err_free_priv:
	kfree(priv);
	return ret;
}

void
virtio_msg_ffa_device_indirect_runtime_quiesce(struct virtio_msg_ffa_device *vdev)
{
	struct virtio_msg_ffa_device_indirect *priv;

	if (!vdev)
		return;

	mutex_lock(&vdev->lock);
	priv = virtio_msg_ffa_device_indirect_get_locked(vdev);
	if (priv)
		RCU_INIT_POINTER(vdev->indirect, NULL);
	mutex_unlock(&vdev->lock);
	if (!priv)
		return;

	synchronize_rcu();
	virtio_msg_bus_queue_helper_stop(&priv->rx_queue);
	cancel_delayed_work_sync(&priv->tx_work);
	virtio_msg_ffa_device_indirect_tx_purge(priv);
	kfree(priv);
}

static int
virtio_msg_ffa_device_indirect_init_bootstrap
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg_ffa_xfer_method_desc *desc)
{
	if (!vdev || !desc || desc->method != VIRTIO_MSG_FFA_XFER_INDIRECT ||
	    !(desc->role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DEVICE) ||
	    !(desc->phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP))
		return -EINVAL;
	if (!virtio_msg_ffa_device_indirect_capable(vdev))
		return -EOPNOTSUPP;

	return virtio_msg_ffa_device_indirect_runtime_init(vdev);
}

static int
virtio_msg_ffa_device_indirect_init_runtime
		(struct virtio_msg_ffa_device *vdev,
		 const struct virtio_msg_ffa_xfer_method_desc *desc)
{
	if (!vdev || !desc || desc->method != VIRTIO_MSG_FFA_XFER_INDIRECT ||
	    !(desc->role_mask & VIRTIO_MSG_FFA_XFER_ROLE_DEVICE) ||
	    !(desc->phase_flags & VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME) ||
	    desc->event_delivery != VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT)
		return -EINVAL;
	if (!virtio_msg_ffa_device_indirect_capable(vdev) ||
	    !virtio_msg_ffa_device_indirect_get_locked(vdev))
		return -EOPNOTSUPP;

	if (virtio_msg_ffa_device_runtime_select_locked(vdev, desc))
		return -EIO;
	vdev->event_configured = true;
	if (vdev->ep.transfer_method == VIRTIO_MSG_FFA_XFER_INDIRECT &&
	    vdev->ep.event_delivery == VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT &&
	    vdev->runtime_method &&
	    vdev->runtime_method->desc == desc)
		return 0;

	virtio_msg_ffa_device_runtime_clear_locked(vdev);
	return -EIO;
}

int virtio_msg_ffa_device_indirect_event_configure_locked(struct virtio_msg_ffa_device *vdev)
{
	lockdep_assert_held(&vdev->lock);

	return virtio_msg_ffa_device_indirect_init_runtime
			(vdev, &virtio_msg_ffa_indirect_device_method_desc);
}

static const struct virtio_msg_ffa_xfer_device_method_ops
virtio_msg_ffa_indirect_device_method_ops = {
	.init_runtime = virtio_msg_ffa_device_indirect_init_runtime,
};

static const struct virtio_msg_ffa_xfer_method_desc
virtio_msg_ffa_indirect_device_method_desc = {
	.method = VIRTIO_MSG_FFA_XFER_INDIRECT,
	.name = "indirect",
	.role_mask = VIRTIO_MSG_FFA_XFER_ROLE_DEVICE,
	.phase_flags = VIRTIO_MSG_FFA_XFER_PHASE_BOOTSTRAP |
		       VIRTIO_MSG_FFA_XFER_PHASE_RUNTIME,
	.bus_feature_mask = VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_RX |
			    VIRTIO_MSG_FFA_BUS_FEATURE_INDIRECT_TX,
	.event_delivery = VIRTIO_MSG_FFA_BUS_EVENT_DELIV_INDIRECT,
	.device_ops = &virtio_msg_ffa_indirect_device_method_ops,
};

static const struct virtio_msg_ffa_device_method_ops
virtio_msg_ffa_indirect_device_ops = {
	.capable = virtio_msg_ffa_device_indirect_capable,
	.init_bootstrap = virtio_msg_ffa_device_indirect_init_bootstrap,
	.init_runtime = virtio_msg_ffa_device_indirect_init_runtime,
	.reset_locked = virtio_msg_ffa_device_indirect_runtime_reset_locked,
	.quiesce = virtio_msg_ffa_device_indirect_runtime_quiesce,
	.send_locked = virtio_msg_ffa_device_send_indirect_locked,
	.event_configure_locked =
		virtio_msg_ffa_device_indirect_event_configure_locked,
};

const struct virtio_msg_ffa_device_method
virtio_msg_ffa_indirect_device_method = {
	.desc = &virtio_msg_ffa_indirect_device_method_desc,
	.ops = &virtio_msg_ffa_indirect_device_ops,
};

bool virtio_msg_ffa_device_indirect_rx_cb(struct ffa_device *fdev,
					  u16 sender_vm_id,
					  const uuid_t *uuid,
					  const void *buf, size_t len)
{
	struct device *dev = fdev ? &fdev->dev : NULL;
	struct virtio_msg_ffa_device *vdev;
	int ret;

	if (!fdev || !buf || !len)
		return false;
	if (!uuid || !uuid_equal(uuid, &virtio_msg_ffa_device_uuid))
		return false;
	if (fdev->vm_id != sender_vm_id)
		return false;
	if (len > FFA_BUS_MAX_MSG_SIZE) {
		dev_dbg(dev, "indirect rx cb consumed oversized payload: len=%zu\n",
			len);
		return true;
	}

	rcu_read_lock();
	vdev = ffa_dev_get_drvdata(fdev);
	if (!vdev) {
		rcu_read_unlock();
		dev_dbg(dev, "indirect rx cb consumed: missing device state\n");
		return true;
	}

	ret = virtio_msg_ffa_device_indirect_rx_enqueue(vdev, sender_vm_id, buf,
							len);
	rcu_read_unlock();
	if (ret)
		dev_dbg(dev, "device-role indirect RX enqueue dropped: %d\n",
			ret);

	return true;
}
