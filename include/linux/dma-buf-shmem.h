/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * dma-buf helpers for transporting shared-memory descriptors across
 * transports such as vsock.
 *
 * Copyright (C) 2025
 */

#ifndef _LINUX_DMA_BUF_SHMEM_H
#define _LINUX_DMA_BUF_SHMEM_H

#include <linux/list.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/vm_sockets.h>

/**
 * struct dma_shmem_provider - importer for transport-level descriptors
 * @proto_id:	Protocol identifier encoded by the exporter (must be > 0).
 * @name:	Human readable name for debug logging.
 * @owner:	Module owner for reference counting.
 * @import:	Callback invoked to turn the descriptor into a struct file *
 *		referencing a dma-buf. Returns 0 on success and stores the
 *		newly created file in *@filep.
 *
 * Providers register themselves so that transports (e.g. vsock) can hand over
 * opaque metadata received from a remote peer and obtain a local dma-buf.
 */
#if IS_ENABLED(CONFIG_DMA_SHARED_BUFFER)
struct dma_shmem_provider {
	const char *name;
	struct module *owner;
	bool (*matches)(const struct vsock_shmem_desc *desc);
	int (*import)(const struct vsock_shmem_desc *desc, struct file **filep);
	struct list_head node;
};

int dma_shmem_register_provider(struct dma_shmem_provider *provider);
void dma_shmem_unregister_provider(struct dma_shmem_provider *provider);

int dma_shmem_import(const struct vsock_shmem_desc *desc, struct file **filep);
#else
struct dma_shmem_provider;

static inline int dma_shmem_register_provider(struct dma_shmem_provider *provider)
{
	return -EOPNOTSUPP;
}

static inline void dma_shmem_unregister_provider(struct dma_shmem_provider *provider)
{
}

static inline int dma_shmem_import(const struct vsock_shmem_desc *desc,
				   struct file **filep)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* _LINUX_DMA_BUF_SHMEM_H */

