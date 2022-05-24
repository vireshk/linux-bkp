// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 */

#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static struct cpufreq_frequency_table freq_table[] = {
	{0, 0, 1000000},
	{0, 0, 1100000},
	{0, 0, 1200000},
	{0, 0, CPUFREQ_TABLE_END},
};

static unsigned int gindex;

static int set_target(struct cpufreq_policy *policy, unsigned int index)
{
	gindex = index;
	return 0;
}

static int qemu_policy_init(struct cpufreq_policy *policy)
{
	cpumask_setall(policy->cpus);
	policy->freq_table = freq_table;
	return 0;
}

unsigned int qemu_get(unsigned int cpu)
{
	return freq_table[gindex].frequency;
}

static struct cpufreq_driver qemu_cpufreq_driver = {
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = set_target,
	.get = qemu_get,
	.init = qemu_policy_init,
	.name = "cpufreq-qemu",
};

static int qemu_init(void)
{
	return cpufreq_register_driver(&qemu_cpufreq_driver);
}
module_init(qemu_init);
