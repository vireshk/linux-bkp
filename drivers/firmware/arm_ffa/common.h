/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 ARM Ltd.
 */

#ifndef _FFA_COMMON_H
#define _FFA_COMMON_H

#include <linux/arm_ffa.h>
#include <linux/arm-smccc.h>
#include <linux/err.h>

typedef struct arm_smccc_1_2_regs ffa_value_t;

typedef void (ffa_fn)(ffa_value_t, ffa_value_t *);

typedef void (ffa_irq_callback)(int irq);

bool ffa_device_is_valid(struct ffa_device *ffa_dev);
void ffa_device_match_uuid(struct ffa_device *ffa_dev, const uuid_t *uuid);

#ifdef CONFIG_ARM_FFA_SMCCC
extern int ffa_transport_init(ffa_fn **invoke_ffa_fn);
#else
static inline int ffa_transport_init(ffa_fn **invoke_ffa_fn)
{
	return -EOPNOTSUPP;
}
#endif

extern int ffa_get_version(void);

extern int ffa_setup_partitions(void);
extern void ffa_partitions_cleanup(void);

extern int ffa_setup_rxtx(void);
extern int ffa_cleanup_rxtx(void);

extern int ffa_register_irq_callback(ffa_irq_callback *callback);

#endif /* _FFA_COMMON_H */
