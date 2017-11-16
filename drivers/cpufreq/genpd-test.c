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
#include <linux/pm_runtime.h>
#include <linux/slab.h>

static int pd_power_on(struct generic_pm_domain *domain)
{
	pr_info("%s: %d\n", __func__, __LINE__);
	return 0;
}

static int pd_power_off(struct generic_pm_domain *domain)
{
	pr_info("%s: %d\n", __func__, __LINE__);
	return 0;
}

static int pd_set_performance_parent(struct generic_pm_domain *domain, unsigned int state)
{
	pr_info("%s: %d: %d\n", __func__, __LINE__, state);
	return 0;
}

static int pd_set_performance_multi(struct generic_pm_domain *domain, unsigned int state)
{
	pr_info("%s: %d: %d\n", __func__, __LINE__, state);
	return 0;
}

static int pd_set_performance(struct generic_pm_domain *domain, unsigned int state)
{
	pr_info("%s: %d: %d\n", __func__, __LINE__, state);
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
		.compatible = "foo,genpd_parent",
		.data = pd_set_performance_parent,
	},
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

static const struct of_device_id pm_domain_of_match_multi[] __initconst = {
	{
		.compatible = "foo_multi,genpd",
		.data = pd_set_performance_multi,
	},
	{ },
};

static struct generic_pm_domain *pd_init(struct device_node *np, const struct of_device_id *of_id)
{
	struct generic_pm_domain *pd;

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return NULL;

	pd->name = kstrdup_const(np->full_name, GFP_KERNEL);
	if (!pd->name) {
		of_node_put(np);
		return NULL;
	}

	pd->power_off = pd_power_off;
	pd->power_on = pd_power_on;

	pd->set_performance_state = of_id->data;
	pd->opp_to_performance_state = pd_get_performance;

	pm_genpd_init(pd, NULL, false);
	return pd;
}

static int __init add_domains(void)
{
	struct device_node *np;
	struct generic_pm_domain *pd, *pd_ptr[3];
	const struct of_device_id *of_id;
	struct genpd_onecell_data *data;
	int i = 0;

	for_each_matching_node_and_match(np, pm_domain_of_match, &of_id) {
		pd = pd_init(np, of_id);
		if (!pd)
			return -ENOMEM;

		of_genpd_add_provider_simple(np, pd);

		pd_ptr[i++] = pd;
	}

	if (i > 1) {
		i = pm_genpd_add_subdomain(pd_ptr[0], pd_ptr[1]);
		i = pm_genpd_add_subdomain(pd_ptr[0], pd_ptr[2]);
	}

	i = 0;

	for_each_matching_node_and_match(np, pm_domain_of_match_multi, &of_id) {
		data = kzalloc(sizeof(*data), GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		data->domains = kcalloc(2, sizeof(*data->domains), GFP_KERNEL);
		if (!data->domains)
			return -ENOMEM;

		data->num_domains = 2;

again:
		pd = pd_init(np, of_id);
		if (!pd)
			return -ENOMEM;

		data->domains[i] = pd;

		if (++i == 1)
			goto again;

		of_genpd_add_provider_onecell(np, data);

		pm_genpd_add_subdomain(pd_ptr[0], data->domains[0]);
		pm_genpd_add_subdomain(pd_ptr[0], data->domains[1]);
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
	device_link_add(dev, vdev, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
	dev_pm_opp_set_genpd_virt_dev(dev, vdev, 0);
	pm_runtime_get_sync(vdev);

	vdev = dev_pm_domain_attach_by_id(dev, 1);
	device_link_add(dev, vdev, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
	dev_pm_opp_set_genpd_virt_dev(dev, vdev, 1);
	pm_runtime_get_sync(vdev);
	return 0;
}
device_initcall(genpd_test_init);
