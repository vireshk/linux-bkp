/*
 * Copyright (C) 2016 Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpu.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

static int pd_power_on(struct generic_pm_domain *domain)
{
	return 0;
}

static int pd_power_off(struct generic_pm_domain *domain)
{
	return 0;
}

static int pd_set_performance(struct generic_pm_domain *domain, unsigned int state)
{
	return 0;
}

static int pd_set_performance1(struct generic_pm_domain *domain, unsigned int state)
{
	pr_info("%s: %d: %d\n", __func__, __LINE__, state);
	return 0;
}

static unsigned int pd_get_performance(struct generic_pm_domain *genpd,
			struct dev_pm_opp *opp)
{
	struct device_node *np;
	unsigned int corner = 0;

	np = dev_pm_opp_get_of_node(opp);
	of_property_read_u32(np, "qcom,level", &corner);
	of_node_put(np);

	return corner;

}

static const struct of_device_id pm_domain_of_match[] __initconst = {
	{
		.compatible = "foo,genpd",
		.data = pd_set_performance,
	},
	{
		.compatible = "foo,genpd1",
		.data = pd_set_performance1,
	},
	{ },
};

static int __init add_domains(void)
{
	struct device_node *np;
	struct generic_pm_domain *pd;
	const struct of_device_id *of_id;

	for_each_matching_node_and_match(np, pm_domain_of_match, &of_id) {
		pd = kzalloc(sizeof(*pd), GFP_KERNEL);
		if (!pd)
			return -ENOMEM;

		pd->name = kstrdup_const(np->full_name, GFP_KERNEL);
		if (!pd->name) {
			of_node_put(np);
			return -1;
		}

		pd->power_off = pd_power_off;
		pd->power_on = pd_power_on;

		pd->set_performance_state = of_id->data;
		pd->opp_to_performance_state = pd_get_performance;

		pm_genpd_init(pd, NULL, false);
		of_genpd_add_provider_simple(np, pd);
	}

	return 0;
}

static int __init genpd_test_init(void)
{
	struct device *dev = get_cpu_device(0), *vdev;

	if (add_domains()) {
		pr_info("%s: %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	vdev = dev_pm_domain_attach_by_id(dev, 0);
	dev_pm_opp_set_genpd_virt_dev(dev, vdev, 0);
	vdev = dev_pm_domain_attach_by_id(dev, 1);
	dev_pm_opp_set_genpd_virt_dev(dev, vdev, 1);
	return 0;
}
device_initcall(genpd_test_init);
