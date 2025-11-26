// SPDX-License-Identifier: GPL-2.0-only
/*
 * dma-buf shared memory provider registry
 */

#include <linux/dma-buf-shmem.h>
#include <linux/mutex.h>
#include <linux/slab.h>

static LIST_HEAD(dma_shmem_providers);
static DEFINE_MUTEX(dma_shmem_lock);

int dma_shmem_register_provider(struct dma_shmem_provider *provider)
{
	if (!provider || !provider->matches || !provider->import)
		return -EINVAL;

	mutex_lock(&dma_shmem_lock);
	INIT_LIST_HEAD(&provider->node);
	list_add(&provider->node, &dma_shmem_providers);
	mutex_unlock(&dma_shmem_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(dma_shmem_register_provider);

void dma_shmem_unregister_provider(struct dma_shmem_provider *provider)
{
	if (!provider)
		return;

	mutex_lock(&dma_shmem_lock);
	list_del(&provider->node);
	mutex_unlock(&dma_shmem_lock);
}
EXPORT_SYMBOL_GPL(dma_shmem_unregister_provider);

int dma_shmem_import(const struct vsock_shmem_desc *desc, struct file **filep)
{
	struct dma_shmem_provider *provider;
	int ret = -ENOENT;

	if (!desc || !filep)
		return -EINVAL;

	mutex_lock(&dma_shmem_lock);
	list_for_each_entry(provider, &dma_shmem_providers, node) {
		if (provider->matches(desc)) {
			if (!try_module_get(provider->owner)) {
				ret = -ENODEV;
				break;
			}
			mutex_unlock(&dma_shmem_lock);
			ret = provider->import(desc, filep);
			module_put(provider->owner);
			return ret;
		}
	}
	mutex_unlock(&dma_shmem_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(dma_shmem_import);

