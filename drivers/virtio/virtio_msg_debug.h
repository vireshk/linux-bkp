/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Virtio message subsystem shared logging policy layer.
 *
 * Copyright (c) 2026 Arm Limited or its affiliates. All rights reserved.
 */

#ifndef _VIRTIO_MSG_DEBUG_H
#define _VIRTIO_MSG_DEBUG_H

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/stdarg.h>
#if !defined(VM_LOG_COMPONENT)
#error "VM_LOG_COMPONENT must be defined before including virtio_msg_debug.h"
#endif

#define VM_LOG_COMPONENT_TRANSPORT		1
#define VM_LOG_COMPONENT_BRIDGE			2
#define VM_LOG_COMPONENT_LOOPBACK		3
#define VM_LOG_COMPONENT_DMA			4
#define VM_LOG_COMPONENT_FFA_COMMON		5
#define VM_LOG_COMPONENT_FFA_DRIVER_BINDING	6
#define VM_LOG_COMPONENT_FFA_DEVICE_BINDING	7

/* Per-component policy knobs: 1 enables the policy, 0 disables it. */
#define DBG_PROMOTE_TRANSPORT	0
#define DBG_PROMOTE_BRIDGE	0
#define DBG_PROMOTE_LOOPBACK	0
#define DBG_PROMOTE_DMA		0
#define DBG_PROMOTE_FFA_COMMON	0
#define DBG_PROMOTE_FFA_DRIVER_BINDING	0
#define DBG_PROMOTE_FFA_DEVICE_BINDING	0

#define TRACE_ENABLE_TRANSPORT	0
#define TRACE_ENABLE_BRIDGE	0
#define TRACE_ENABLE_LOOPBACK	0
#define TRACE_ENABLE_DMA	0
#define TRACE_ENABLE_FFA_COMMON	0
#define TRACE_ENABLE_FFA_DRIVER_BINDING	0
#define TRACE_ENABLE_FFA_DEVICE_BINDING	0

#if VM_LOG_COMPONENT == VM_LOG_COMPONENT_TRANSPORT
#define __VM_LOG_PROMOTE	DBG_PROMOTE_TRANSPORT
#define __VM_LOG_TRACE_ENABLE	TRACE_ENABLE_TRANSPORT
#elif VM_LOG_COMPONENT == VM_LOG_COMPONENT_BRIDGE
#define __VM_LOG_PROMOTE	DBG_PROMOTE_BRIDGE
#define __VM_LOG_TRACE_ENABLE	TRACE_ENABLE_BRIDGE
#elif VM_LOG_COMPONENT == VM_LOG_COMPONENT_LOOPBACK
#define __VM_LOG_PROMOTE	DBG_PROMOTE_LOOPBACK
#define __VM_LOG_TRACE_ENABLE	TRACE_ENABLE_LOOPBACK
#elif VM_LOG_COMPONENT == VM_LOG_COMPONENT_DMA
#define __VM_LOG_PROMOTE	DBG_PROMOTE_DMA
#define __VM_LOG_TRACE_ENABLE	TRACE_ENABLE_DMA
#elif VM_LOG_COMPONENT == VM_LOG_COMPONENT_FFA_COMMON
#define __VM_LOG_PROMOTE	DBG_PROMOTE_FFA_COMMON
#define __VM_LOG_TRACE_ENABLE	TRACE_ENABLE_FFA_COMMON
#elif VM_LOG_COMPONENT == VM_LOG_COMPONENT_FFA_DRIVER_BINDING
#define __VM_LOG_PROMOTE	DBG_PROMOTE_FFA_DRIVER_BINDING
#define __VM_LOG_TRACE_ENABLE	TRACE_ENABLE_FFA_DRIVER_BINDING
#elif VM_LOG_COMPONENT == VM_LOG_COMPONENT_FFA_DEVICE_BINDING
#define __VM_LOG_PROMOTE	DBG_PROMOTE_FFA_DEVICE_BINDING
#define __VM_LOG_TRACE_ENABLE	TRACE_ENABLE_FFA_DEVICE_BINDING
#else
#error "VM_LOG_COMPONENT must be one of VM_LOG_COMPONENT_*"
#endif

static inline __printf(2, 3)
void __vm_info(const struct device *dev, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	if (dev)
		dev_info(dev, "%pV", &vaf);
	else
		pr_info("%pV", &vaf);
	va_end(args);
}

static inline __printf(2, 3)
void __vm_warn_rl(const struct device *dev, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	if (dev)
		dev_warn_ratelimited(dev, "%pV", &vaf);
	else
		pr_warn_ratelimited("%pV", &vaf);
	va_end(args);
}

static inline __printf(2, 3)
void __vm_err_rl(const struct device *dev, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	if (dev)
		dev_err_ratelimited(dev, "%pV", &vaf);
	else
		pr_err_ratelimited("%pV", &vaf);
	va_end(args);
}

static inline __printf(2, 3)
void __vm_dbg(const struct device *dev, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	if (dev)
		dev_dbg(dev, "%pV", &vaf);
	else
		pr_debug("%pV", &vaf);
	va_end(args);
}

#define vm_info(dev, fmt, ...)	__vm_info((dev), fmt, ##__VA_ARGS__)
#define vm_warn_rl(dev, fmt, ...)	__vm_warn_rl((dev), fmt, ##__VA_ARGS__)
#define vm_err_rl(dev, fmt, ...)	__vm_err_rl((dev), fmt, ##__VA_ARGS__)

#if __VM_LOG_PROMOTE
#define vm_dbg(dev, fmt, ...)	__vm_info((dev), fmt, ##__VA_ARGS__)
#else
#define vm_dbg(dev, fmt, ...)	__vm_dbg((dev), fmt, ##__VA_ARGS__)
#endif

#if __VM_LOG_TRACE_ENABLE
#define vm_trace(dev, fmt, ...)	__vm_dbg((dev), fmt, ##__VA_ARGS__)
#else
#define vm_trace(dev, fmt, ...)						\
	do {								\
		(void)(dev);						\
		no_printk(fmt, ##__VA_ARGS__);				\
	} while (0)
#endif

#endif /* _VIRTIO_MSG_DEBUG_H */
