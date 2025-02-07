// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/mm.h>
#include <linux/arm_ffa.h>
#include <linux/arm_ffa_acs.h>

#include "common.h"

#define THIS "ffa-acs: "

static int ffa_acs_alloc(uint64_t size);
static int ffa_acs_free(void);

static int ffa_acs_open(struct inode *inode, struct file *file);
static int ffa_acs_release(struct inode *inode, struct file *file);
static long ffa_acs_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg);
static ssize_t ffa_acs_read(struct file *file, char __user *buf, size_t count,
				loff_t *offset);
static ssize_t ffa_acs_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *offset);
static int ffa_acs_mmap(struct file *file, struct vm_area_struct *vma);

static const struct file_operations ffa_acs_fops = {
	.owner = THIS_MODULE,
	.open = ffa_acs_open,
	.release = ffa_acs_release,
	.unlocked_ioctl = ffa_acs_ioctl,
	.read = ffa_acs_read,
	.write = ffa_acs_write,
	.mmap = ffa_acs_mmap,
};

static dev_t ffa_acs_dev_t;
static struct cdev *ffa_acs_cdev;
static struct class *ffa_acs_class;

static ffa_fn *invoke_ffa_fn;

/*
 * If we have a big number of cores this might need to be increased
 */
static DECLARE_KFIFO(fifo_irq, struct ffa_acs_irq, 256);

/*
 * Waitqueue used to park caller waiting for an interrupt
 */
static DECLARE_WAIT_QUEUE_HEAD(wq_irq);

static DEFINE_MUTEX(fifo_mutex);

/*
 * Counter of open
 */
static atomic_t num_open;

/*
 * Buffer allocated for mmap
 */
static const void *dma_buffer;
static uint64_t dma_size;

/*
 * Mutex to protect access to dma variables
 */
static DEFINE_MUTEX(dma_mutex);

static int ffa_acs_alloc(uint64_t size)
{
	/* Buffer already allocated */
	if (dma_size != 0)
		return -EBUSY;

	/* size must be aligned to a page */
	if ((size & (PAGE_SIZE - 1)) != 0)
		return -EINVAL;

	dma_buffer = kmalloc(size, GFP_KERNEL);
	if (IS_ERR(dma_buffer)) {
		dma_buffer = NULL;
		return PTR_ERR(dma_buffer);
	}
	dma_size = size;

	/* Mark the pages reserved so the kernel does not swap them out */
	for (uint64_t i = 0; i < dma_size; i += PAGE_SIZE)
		SetPageReserved(virt_to_page(((unsigned long)dma_buffer) + i));

	return 0;
}

static int ffa_acs_free(void)
{
	if (dma_size == 0)
		return -EOPNOTSUPP;

	for (uint64_t i = 0; i < dma_size; i += PAGE_SIZE)
		ClearPageReserved(virt_to_page(((unsigned long)dma_buffer) + i));

	kfree(dma_buffer);
	dma_size = 0;
	dma_buffer = NULL;

	return 0;
}

static void ffa_acs_irq_handler(int irq)
{
	struct ffa_acs_irq entry;

	entry.irq = irq;
	entry.processor_id = smp_processor_id();

	kfifo_in(&fifo_irq, &entry, sizeof(entry));
	wake_up_interruptible(&wq_irq);
}

static int ffa_acs_open(struct inode *inode, struct file *file)
{
	int ret;

	if (!atomic_dec_and_test(&num_open)) {
		/* number of clients reached, reject open */
		atomic_inc(&num_open);

		return -EBUSY;
	}

	/* Unregister all drivers */
	ffa_partitions_cleanup();

	ret = ffa_cleanup_rxtx();
	if (ret != 0) {
		pr_err(THIS "Error unampping rxtx buffer\n");
		atomic_inc(&num_open);

		return ret;
	}

	ret = ffa_register_irq_callback(ffa_acs_irq_handler);
	if (ret != 0) {
		pr_err(THIS "Error registering irq callback\n");
		atomic_inc(&num_open);

		return ret;
	}

	return 0;
}

static int ffa_acs_release(struct inode *inode, struct file *file)
{
	ffa_value_t ffa_ret;
	int ret;

	/* Free dma buffer if any */
	ffa_acs_free();

	/*
	 * In case the application did not cleanup properly:
	 * - try to release RX buffer
	 * - try to unmap RXTX buffer
	 * If the application did it, the calls will just fail and we ignore the
	 * error returned
	 */
	invoke_ffa_fn((ffa_value_t){
				.a0 = FFA_RX_RELEASE,
				}, &ffa_ret);
	invoke_ffa_fn((ffa_value_t){
				.a0 = FFA_RXTX_UNMAP,
				}, &ffa_ret);

	/* remove our irq callback */
	ret = ffa_register_irq_callback(NULL);
	if (ret != 0) {
		pr_err(THIS "Error remove irq callback\n");
		pr_err(THIS "FF-A is not functional and you should reboot the system !!\n");

		return ret;
	}

	/* re-map rxtx buffer */
	ret = ffa_setup_rxtx();
	if (ret != 0) {
		pr_err(THIS "Error re-mapping rxtx buffer\n");
		pr_err(THIS "FF-A is not functional and you should reboot the system !!\n");

		return ret;
	}

	/* re-register drivers */
	ret = ffa_setup_partitions();
	if (ret != 0) {
		pr_err(THIS "Error during re-init of FF-A driver\n");
		pr_err(THIS "FF-A is not functional and you should reboot the system !!\n");

		return ret;
	}

	/* Allow ACS to open again */
	atomic_inc(&num_open);

	return 0;
}

static long ffa_acs_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	ffa_value_t smc_args;
	uint64_t alloc_args;
	int ret;
	struct ffa_acs_irq irq_args;

	switch (cmd) {
	case FFA_ACS_IOC_CALL:

		if (copy_from_user(&smc_args, (void __user *)arg,
							sizeof(smc_args)))
			return -EFAULT;

		/* Check that x0 contains a value within FFA range */
		if (FFA_SMC(ARM_SMCCC_IS_64(smc_args.a0) != 0,
					ARM_SMCCC_FUNC_NUM(smc_args.a0)) != smc_args.a0)
			return -EINVAL;

		invoke_ffa_fn(smc_args, &smc_args);

		if (copy_to_user((void __user *)arg, &smc_args,
						sizeof(smc_args)))
			return -EFAULT;

		break;
	case FFA_ACS_IOC_ISR:

		if (mutex_lock_interruptible(&fifo_mutex))
			return -ERESTARTSYS;

		if (kfifo_is_empty(&fifo_irq)) {
			mutex_unlock(&fifo_mutex);

			if (wait_event_interruptible(wq_irq, !kfifo_is_empty(&fifo_irq)))
				return -ERESTARTSYS;

			if (mutex_lock_interruptible(&fifo_mutex))
				return -ERESTARTSYS;
		}

		ret = kfifo_out(&fifo_irq, &irq_args, sizeof(irq_args));

		mutex_unlock(&fifo_mutex);
		if (ret != sizeof(irq_args))
			return -EFAULT;

		/* if an error occurs here, we lost the interrupt */
		if (copy_to_user((void __user *)arg, &irq_args, sizeof(irq_args)))
			return -EFAULT;

		break;
	case FFA_ACS_IOC_MEM_ALLOC:
		/* Allocate a buffer that can be mmaped later */

		if (copy_from_user(&alloc_args, (void __user *)arg,
							sizeof(alloc_args)))
			return -EFAULT;

		if (mutex_lock_interruptible(&dma_mutex))
			return -ERESTARTSYS;

		ret = ffa_acs_alloc(alloc_args);
		if (ret != 0) {
			mutex_unlock(&dma_mutex);
			return ret;
		}

		alloc_args = virt_to_phys(dma_buffer);
		mutex_unlock(&dma_mutex);

		if (copy_to_user((void __user *)arg, &alloc_args,
							sizeof(alloc_args))) {
			ffa_acs_free();
			return -EFAULT;
		}

		break;
	case FFA_ACS_IOC_MEM_FREE:
		if (mutex_lock_interruptible(&dma_mutex))
			return -ERESTARTSYS;

		/* Free buffer allocated through FFA_ACS_IOC_MEM_ALLOC */
		ret = ffa_acs_free();

		mutex_unlock(&dma_mutex);
		return ret;

		break;
	default:
		pr_err(THIS "Invalid IOCTL\n");
		return -EINVAL;
	}

	return 0;
}

static ssize_t ffa_acs_read(struct file *file, char __user *buf, size_t count,
				loff_t *offset)
{
	pr_err(THIS "read not supported\n");
	return -EOPNOTSUPP;
}

static ssize_t ffa_acs_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *offset)
{
	pr_err(THIS "write not supported\n");
	return -EOPNOTSUPP;
}

static int ffa_acs_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct page *page;
	unsigned long size = (unsigned long)(vma->vm_end - vma->vm_start);

	if (mutex_lock_interruptible(&dma_mutex))
		return -ERESTARTSYS;

	/* ioctl was not called or area was freed */
	if (dma_size == 0) {
		mutex_unlock(&dma_mutex);
		return -ENOMEM;
	}

	/* asking to map more then what we have */
	if (size + (vma->vm_pgoff << PAGE_SHIFT) > dma_size) {
		mutex_unlock(&dma_mutex);
		return -EINVAL;
	}

	page = virt_to_page((unsigned long)dma_buffer +
				(vma->vm_pgoff << PAGE_SHIFT));
	mutex_unlock(&dma_mutex);

	return remap_pfn_range(vma, vma->vm_start, page_to_pfn(page), size,
							vma->vm_page_prot);
}

static int ffa_acs_uevent(const struct device *dev,
			  struct kobj_uevent_env *env)
{
	add_uevent_var(env, "DEVMODE=%#o", 0600);
	return 0;
}

static int __init ffa_acs_init(void)
{
	int ret;
	uint32_t ffa_version;

	ret = ffa_get_version();
	if (ret < 0) {
		pr_err(THIS "cannot load, FF-A version returned an error\n");
		return ret;
	}
	ffa_version = ret;

	ret = ffa_transport_init(&invoke_ffa_fn);
	if (ret)
		return ret;

	ret = alloc_chrdev_region(&ffa_acs_dev_t, 0, 1, "ffa_acs");
	if (ret)
		return ret;

	ffa_acs_cdev = cdev_alloc();
	if (!ffa_acs_cdev)
		return -ENOMEM;

	cdev_init(ffa_acs_cdev, &ffa_acs_fops);

	ret = cdev_add(ffa_acs_cdev, ffa_acs_dev_t, 1);
	if (ret)
		return ret;

	ffa_acs_class = class_create("ffa_acs");
	if (!ffa_acs_class)
		return -EEXIST;

	ffa_acs_class->dev_uevent = ffa_acs_uevent;

	if (!device_create(ffa_acs_class, NULL, ffa_acs_dev_t, NULL, "ffa-acs"))
		return -EINVAL;

	/* limit number of clients to 1 */
	atomic_set(&num_open, 1);

	INIT_KFIFO(fifo_irq);

	pr_info(THIS "loaded: ffa-acs FF-A Version 0x%x\n", ffa_version);

	return 0;
}
module_init(ffa_acs_init);

static void __exit ffa_acs_exit(void)
{
	device_destroy(ffa_acs_class, ffa_acs_dev_t);

	class_unregister(ffa_acs_class);
	class_destroy(ffa_acs_class);
	cdev_del(ffa_acs_cdev);
	unregister_chrdev_region(ffa_acs_dev_t, 1);

	pr_info(THIS "exit\n");
}
module_exit(ffa_acs_exit);

MODULE_ALIAS("ffa-acs");
MODULE_AUTHOR("Bertrand Marquis <bertrand.marquis@arm.com>");
MODULE_DESCRIPTION("ARM FF-A ACS interfaces");
MODULE_LICENSE("GPL");
