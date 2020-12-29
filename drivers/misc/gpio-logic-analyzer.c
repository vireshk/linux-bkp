// SPDX-License-Identifier: GPL-2.0-only
/*
 * Simple logic analyzer using GPIOs (to be run on an isolated CPU)
 *
 * Use the 'gpio-logic-analyzer' script in the 'tools/debugging' folder for
 * easier usage and further documentation. Note that this is still a last resort
 * analyzer which can be affected by latencies and non-determinant code paths.
 * However, for e.g. remote development, it may be useful to get a first view
 * and aid further debugging.
 *
 * Copyright (C) Wolfram Sang <wsa@sang-engineering.com>
 * Copyright (C) Renesas Electronics Corporation
 */

#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/timekeeping.h>
#include <linux/vmalloc.h>

#define GPIO_LA_NAME "gpio-logic-analyzer"
#define GPIO_LA_DEFAULT_BUF_SIZE SZ_256K
/* can be increased if needed */
#define GPIO_LA_MAX_PROBES 8
#define GPIO_LA_PROBES_MASK 7

struct gpio_la_poll_priv {
	unsigned long ndelay;
	u32 buf_idx;
	struct mutex lock;
	struct debugfs_blob_wrapper blob;
	struct gpio_descs *descs;
	struct dentry *debug_dir, *blob_dent;
	struct debugfs_blob_wrapper meta;
	unsigned long gpio_delay;
	unsigned int trigger_len;
	u8 trigger_data[PAGE_SIZE];
};

static struct dentry *gpio_la_poll_debug_dir;

static int fops_capture_set(void *data, u64 val)
{
	struct gpio_la_poll_priv *priv = data;
	u8 *la_buf = priv->blob.data;
	unsigned long state = 0;
	int i, ret;

	if (!la_buf)
		return -ENOMEM;

	if (val) {
		mutex_lock(&priv->lock);
		if (priv->blob_dent) {
			debugfs_remove(priv->blob_dent);
			priv->blob_dent = NULL;
		}

		priv->buf_idx = 0;

		local_irq_disable();
		preempt_disable_notrace();

		for (i = 0; i < priv->trigger_len; i++) {
			u8 data = priv->trigger_data[i];

			do {
				ret = gpiod_get_array_value(priv->descs->ndescs, priv->descs->desc,
							    priv->descs->info, &state);

				if (ret)
					goto gpio_err;
			} while (!!(state & BIT(data & GPIO_LA_PROBES_MASK)) != !!(data & 0x80));
		}

		if (priv->trigger_len) {
			la_buf[priv->buf_idx++] = state;
			ndelay(priv->ndelay);
		}

		while (priv->buf_idx < priv->blob.size && ret == 0) {
			ret = gpiod_get_array_value(priv->descs->ndescs, priv->descs->desc,
					      priv->descs->info, &state);
			la_buf[priv->buf_idx++] = state;
			ndelay(priv->ndelay);
		}
gpio_err:
		preempt_enable_notrace();
		local_irq_enable();
		if (ret)
			pr_err("%s: couldn't read GPIOs: %d\n", __func__, ret);

		priv->blob_dent = debugfs_create_blob("sample_data", 0400, priv->debug_dir, &priv->blob);
		mutex_unlock(&priv->lock);
	}

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_capture, NULL, fops_capture_set, "%llu\n");

static int fops_buf_size_get(void *data, u64 *val)
{
	struct gpio_la_poll_priv *priv = data;

	*val = priv->blob.size;

	return 0;
}

static int fops_buf_size_set(void *data, u64 val)
{
	struct gpio_la_poll_priv *priv = data;
	int ret = 0;
	void *p;

	if (!val)
		return -EINVAL;

	mutex_lock(&priv->lock);

	vfree(priv->blob.data);
	p = vzalloc(val);
	if (!p) {
		/* Try the old value again */
		val = priv->blob.size;
		p = vzalloc(val);
		if (!p) {
			val = 0;
			ret = -ENOMEM;
		}
	}

	priv->blob.data = p;
	priv->blob.size = val;

	mutex_unlock(&priv->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_buf_size, fops_buf_size_get, fops_buf_size_set, "%llu\n");

static int trigger_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static ssize_t trigger_write(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *offset)
{
	struct seq_file *m = file->private_data;
	struct gpio_la_poll_priv *priv = m->private;
	char *buf;
	int i, trigger_len = 0;

	priv->trigger_len = 0;

	if (count & 1)
	    return -EINVAL;

	buf = memdup_user(ubuf, count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	for (i = 0; i < count; i += 2) {
		u8 val;

		if (buf[i] < '1' || buf[i] > '0' + GPIO_LA_MAX_PROBES)
			goto bail_out;

		val = buf[i] - '1';

		switch (toupper(buf[i + 1])) {
		case 'L':
			priv->trigger_data[trigger_len] = val;
			trigger_len++;
			break;
		case 'H':
			priv->trigger_data[trigger_len] = val | 0x80;
			trigger_len++;
			break;
		case 'R':
			priv->trigger_data[trigger_len] = val;
			priv->trigger_data[trigger_len + 1] = val | 0x80;
			trigger_len += 2;
			break;
		case 'F':
			priv->trigger_data[trigger_len] = val | 0x80;
			priv->trigger_data[trigger_len + 1] = val;
			trigger_len += 2;
			break;
		default:
			goto bail_out;
		}

		if (trigger_len > PAGE_SIZE)	/* should never happen */
			goto bail_out;

	}

	priv->trigger_len = trigger_len;

bail_out:
	kfree(buf);
	return priv->trigger_len ? count : -EINVAL;
}

static const struct file_operations fops_trigger = {
	.owner = THIS_MODULE,
	.open = trigger_open,
	.write = trigger_write,
	.llseek = no_llseek,
	.release = single_release,
};

static int gpio_la_poll_probe(struct platform_device *pdev)
{
	struct gpio_la_poll_priv *priv;
	struct device *dev = &pdev->dev;
	char *meta = NULL;
	unsigned long state;
	ktime_t start_time, end_time;
	int ret, i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);

	fops_buf_size_set(priv, GPIO_LA_DEFAULT_BUF_SIZE);

	priv->descs = devm_gpiod_get_array(dev, "probe", GPIOD_IN);
	if (IS_ERR(priv->descs))
		return PTR_ERR(priv->descs);

	/* artificial limit to keep 1 byte per sample for now */
	if (priv->descs->ndescs > GPIO_LA_MAX_PROBES)
		return -ERANGE;

	for (i = 0; i < priv->descs->ndescs; i++) {
		const char *str, *old_meta;
		char *def_name = "ProbeX\0";

		if (gpiod_cansleep(priv->descs->desc[i]))
			return -EREMOTE;

		ret = of_property_read_string_index(pdev->dev.of_node, "probe-names",
						    i, &str);

		/* Hacky way of providing a fallback name if none provided */
		if (ret) {
			def_name[5] = i + '1'; /* assumes GPIO_LA_MAX_PROBES = 8 */
			str = def_name;
		}

		gpiod_set_consumer_name(priv->descs->desc[i], str);

		old_meta = meta;
		meta = devm_kasprintf(dev, GFP_KERNEL, "%sprobe%d=%s\n",
				      old_meta ?: "", i + 1, str);
		if (!meta)
			return -ENOMEM;

		devm_kfree(dev, old_meta);
	}

	platform_set_drvdata(pdev, priv);

	/* Measure delay of reading GPIOs */
	local_irq_disable();
	preempt_disable_notrace();
	start_time = ktime_get();
	for (i = 0, ret = 0; i < 1024 && ret == 0; i++)
		ret = gpiod_get_array_value(priv->descs->ndescs, priv->descs->desc,
				      priv->descs->info, &state);
	end_time = ktime_get();
	preempt_enable_notrace();
	local_irq_enable();
	if (ret) {
		dev_err(dev, "couldn't read GPIOs: %d\n", ret);
		return ret;
	}

	priv->gpio_delay = ktime_sub(end_time, start_time) / 1024;

	priv->debug_dir = debugfs_create_dir(dev_name(dev), gpio_la_poll_debug_dir);
	if (IS_ERR(priv->debug_dir))
		return PTR_ERR(priv->debug_dir);

	priv->meta.data = meta;
	priv->meta.size = strlen(meta);
	debugfs_create_blob("meta_data", 0400, priv->debug_dir, &priv->meta);
	debugfs_create_ulong("delay_ns_acquisition", 0400, priv->debug_dir, &priv->gpio_delay);
	debugfs_create_ulong("delay_ns_user", 0600, priv->debug_dir, &priv->ndelay);
	debugfs_create_file_unsafe("buf_size", 0600, priv->debug_dir, priv, &fops_buf_size);
	debugfs_create_file_unsafe("capture", 0200, priv->debug_dir, priv, &fops_capture);
	debugfs_create_file_unsafe("trigger", 0200, priv->debug_dir, priv, &fops_trigger);

	return 0;
}

static int gpio_la_poll_remove(struct platform_device *pdev)
{
	struct gpio_la_poll_priv *priv = platform_get_drvdata(pdev);

	mutex_lock(&priv->lock);
	debugfs_remove_recursive(priv->debug_dir);
	mutex_unlock(&priv->lock);

	return 0;
}

static const struct of_device_id gpio_la_poll_of_match[] = {
	{ .compatible = GPIO_LA_NAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_la_poll_of_match);

static struct platform_driver gpio_la_poll_device_driver = {
	.probe = gpio_la_poll_probe,
	.remove = gpio_la_poll_remove,
	.driver = {
		.name = GPIO_LA_NAME,
		.of_match_table = gpio_la_poll_of_match,
	}
};

static int __init gpio_la_poll_init(void)
{
	gpio_la_poll_debug_dir = debugfs_create_dir(GPIO_LA_NAME, NULL);
	if (IS_ERR(gpio_la_poll_debug_dir))
		return PTR_ERR(gpio_la_poll_debug_dir);

	return platform_driver_register(&gpio_la_poll_device_driver);
}
late_initcall(gpio_la_poll_init);

static void __exit gpio_la_poll_exit(void)
{
	platform_driver_unregister(&gpio_la_poll_device_driver);
	debugfs_remove_recursive(gpio_la_poll_debug_dir);
}
module_exit(gpio_la_poll_exit);

MODULE_AUTHOR("Wolfram Sang <wsa@sang-engineering.com>");
MODULE_DESCRIPTION("Simple logic analyzer using GPIOs");
MODULE_LICENSE("GPL v2");
