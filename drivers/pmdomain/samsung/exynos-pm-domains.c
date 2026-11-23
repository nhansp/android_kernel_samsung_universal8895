// SPDX-License-Identifier: GPL-2.0
//
// Exynos Generic power domain support.
//
// Copyright (c) 2012 Samsung Electronics Co., Ltd.
//		http://www.samsung.com
//
// Implementation of Exynos specific power domain control which is used in
// conjunction with runtime-pm. Support for both device-tree and non-device-tree
// based power domain support is included.

#include <linux/io.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>

/* One write of a power sequence: a full mask is a plain write, else RMW */
struct exynos_pd_write {
	u32 off, mask, val;
};

/*
 * Steps of a domain's power sequence besides CONFIGURATION and STATUS, selected
 * by the physical address of the CONFIGURATION register (reg 0).
 */
struct exynos_pd_extra {
	phys_addr_t conf;
	const struct exynos_pd_write *pre_on;	/* reg 1, before power-on */
	unsigned int nr_pre_on;
	const struct exynos_pd_write *post_off;	/* reg 1, after power-off */
	unsigned int nr_post_off;
	u32 pmu_off_reg, pmu_off_mask;		/* PMU bits set before power-off */
	u32 sysreg[2];				/* saved before off, restored after on */
	unsigned int nr_sysreg;
	bool sysreg_required;			/* not only DRCG_EN: samsung,sysreg required */
	unsigned int genpd_flags;
};

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	const struct exynos_pd_extra *extra;
	unsigned int nr_extra;
};

/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	struct generic_pm_domain pd;
	u32 local_pwr_cfg;
	const struct exynos_pd_extra *extra;
	void __iomem *seq_base;
	struct regmap *pmu;
	struct regmap *sysreg;
	struct clk *bus_clk;		/* the block's bus clock, on around the SYSREG accesses */
	u32 saved[2];
	bool saved_valid;
};

static void exynos_pd_seq(void __iomem *base, const struct exynos_pd_write *w,
			  unsigned int n)
{
	unsigned int i;
	u32 v;

	for (i = 0; i < n; i++, w++) {
		if (w->mask == U32_MAX) {
			writel_relaxed(w->val, base + w->off);
		} else {
			v = readl_relaxed(base + w->off);
			writel_relaxed((v & ~w->mask) | w->val, base + w->off);
		}
	}
}

static void exynos_pd_before(struct exynos_pm_domain *pd, bool power_on)
{
	const struct exynos_pd_extra *x = pd->extra;
	unsigned int i;

	if (!x)
		return;

	if (power_on) {
		if (pd->seq_base)
			exynos_pd_seq(pd->seq_base, x->pre_on, x->nr_pre_on);
		return;
	}

	if (pd->sysreg) {
		clk_enable(pd->bus_clk);
		for (i = 0; i < x->nr_sysreg; i++)
			regmap_read(pd->sysreg, x->sysreg[i], &pd->saved[i]);
		clk_disable(pd->bus_clk);
		pd->saved_valid = true;
	}
	if (pd->pmu && x->pmu_off_mask)
		regmap_update_bits(pd->pmu, x->pmu_off_reg, x->pmu_off_mask,
				   x->pmu_off_mask);
}

static void exynos_pd_after(struct exynos_pm_domain *pd, bool power_on)
{
	const struct exynos_pd_extra *x = pd->extra;
	unsigned int i;

	if (!x)
		return;

	if (!power_on) {
		if (pd->seq_base)
			exynos_pd_seq(pd->seq_base, x->post_off, x->nr_post_off);
		return;
	}

	/* restore only what a power-off saved */
	if (pd->sysreg && pd->saved_valid) {
		clk_enable(pd->bus_clk);
		for (i = 0; i < x->nr_sysreg; i++)
			regmap_write(pd->sysreg, x->sysreg[i], pd->saved[i]);
		clk_disable(pd->bus_clk);
		pd->saved_valid = false;
	}
}

/*
 * A timed-out power-off leaves the request pending in the PMU while genpd keeps
 * the domain on; a later system power-down then stalls on it. Request the
 * domain on again so that the hardware matches genpd.
 */
static void exynos_pd_off_rollback(struct generic_pm_domain *domain,
				   void __iomem *base, u32 cfg)
{
	unsigned int t;

	writel_relaxed(cfg, base);
	for (t = 0; (readl_relaxed(base + 0x4) & cfg) != cfg; t++) {
		if (t >= 10) {
			pr_err("Power domain %s stuck: CONFIGURATION 0x%x STATUS 0x%x\n",
			       domain->name, readl_relaxed(base) & cfg,
			       readl_relaxed(base + 0x4) & cfg);
			return;
		}
		usleep_range(80, 100);
	}
	pr_err("Power domain %s left on\n", domain->name);
}

static int exynos_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct exynos_pm_domain *pd;
	void __iomem *base;
	u32 timeout, pwr;
	char *op;

	pd = container_of(domain, struct exynos_pm_domain, pd);
	base = pd->base;

	exynos_pd_before(pd, power_on);

	pwr = power_on ? pd->local_pwr_cfg : 0;
	writel_relaxed(pwr, base);

	/* Wait max 1ms */
	timeout = 10;

	while ((readl_relaxed(base + 0x4) & pd->local_pwr_cfg) != pwr) {
		if (!timeout) {
			op = (power_on) ? "enable" : "disable";
			pr_err("Power domain %s %s failed (STATUS 0x%x)\n", domain->name, op,
			       readl_relaxed(base + 0x4) & pd->local_pwr_cfg);
			if (!power_on)
				exynos_pd_off_rollback(domain, base, pd->local_pwr_cfg);
			return -ETIMEDOUT;
		}
		timeout--;
		cpu_relax();
		usleep_range(80, 100);
	}

	exynos_pd_after(pd, power_on);

	return 0;
}

static int exynos_pd_power_on(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, true);
}

static int exynos_pd_power_off(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, false);
}

static const struct exynos_pm_domain_config exynos4210_cfg = {
	.local_pwr_cfg		= 0x7,
};

static const struct exynos_pm_domain_config exynos5433_cfg = {
	.local_pwr_cfg		= 0xf,
};

/* G3D: the CMU_G3D PLL and mux words around power-on and power-off (reg 1) */
static const struct exynos_pd_write exynos8895_g3d_pre_on[] = {
	{ 0x0148, U32_MAX, 0x30000003 },	/* PLL_CON2_PLL_G3D */
	{ 0x0140, BIT(4), BIT(4) },		/* PLL_CON0_PLL_G3D */
	{ 0x0100, BIT(4), BIT(4) },		/* PLL_CON0_MUX_CLKCMU_EMBEDDED_G3D_USER */
};

static const struct exynos_pd_write exynos8895_g3d_post_off[] = {
	{ 0x0100, BIT(4), 0 },			/* PLL_CON0_MUX_CLKCMU_EMBEDDED_G3D_USER */
	{ 0x0140, BIT(4), 0 },			/* PLL_CON0_PLL_G3D */
	{ 0x0148, U32_MAX, 0x10000001 },	/* PLL_CON2_PLL_G3D */
};

#define EXYNOS8895_SYSREG_DRCG_EN	0x0200
#define EXYNOS8895_SYSREG_SNPS_EMA	0x0300
#define EXYNOS8895_ABOX_CA7_OPTION	0x2528
#define EXYNOS8895_ABOX_CA7_USE_STANDBYWFI	BIT(16)

/*
 * ABOX and VTS stay powered through system suspend while a device in them is
 * on the wakeup path.
 */
static const struct exynos_pd_extra exynos8895_pd_extra[] = {
	{	/* g3d */
		.conf		= 0x164840e0,
		.pre_on		= exynos8895_g3d_pre_on,
		.nr_pre_on	= ARRAY_SIZE(exynos8895_g3d_pre_on),
		.post_off	= exynos8895_g3d_post_off,
		.nr_post_off	= ARRAY_SIZE(exynos8895_g3d_post_off),
	},
	{	/* cam */
		.conf = 0x16484020, .sysreg = { EXYNOS8895_SYSREG_DRCG_EN }, .nr_sysreg = 1,
	},
	{	/* g2d */
		.conf = 0x164840c0, .sysreg = { EXYNOS8895_SYSREG_DRCG_EN }, .nr_sysreg = 1,
	},
	{	/* isphq */
		.conf = 0x16484100, .sysreg = { EXYNOS8895_SYSREG_DRCG_EN }, .nr_sysreg = 1,
	},
	{	/* isplp */
		.conf = 0x16484120, .sysreg = { EXYNOS8895_SYSREG_DRCG_EN }, .nr_sysreg = 1,
	},
	{	/* mfc */
		.conf = 0x16484160, .sysreg = { EXYNOS8895_SYSREG_DRCG_EN }, .nr_sysreg = 1,
	},
	{	/* dcam */
		.conf = 0x16485220, .sysreg = { EXYNOS8895_SYSREG_DRCG_EN }, .nr_sysreg = 1,
	},
	{	/* srdz */
		.conf = 0x16485420, .sysreg = { EXYNOS8895_SYSREG_DRCG_EN }, .nr_sysreg = 1,
	},
	{	/* abox */
		.conf		= 0x16484000,
		.pmu_off_reg	= EXYNOS8895_ABOX_CA7_OPTION,
		.pmu_off_mask	= EXYNOS8895_ABOX_CA7_USE_STANDBYWFI,
		.sysreg		= { EXYNOS8895_SYSREG_DRCG_EN, EXYNOS8895_SYSREG_SNPS_EMA },
		.nr_sysreg	= 2,
		.sysreg_required = true,
		.genpd_flags	= GENPD_FLAG_ACTIVE_WAKEUP,
	},
	{	/* vts */
		.conf		= 0x164841a0,
		.sysreg		= { EXYNOS8895_SYSREG_DRCG_EN },
		.nr_sysreg	= 1,
		.genpd_flags	= GENPD_FLAG_ACTIVE_WAKEUP,
	},
};

static const struct exynos_pm_domain_config exynos8895_cfg = {
	.local_pwr_cfg		= 0xf,
	.extra			= exynos8895_pd_extra,
	.nr_extra		= ARRAY_SIZE(exynos8895_pd_extra),
};

static const struct of_device_id exynos_pm_domain_of_match[] = {
	{
		.compatible = "samsung,exynos4210-pd",
		.data = &exynos4210_cfg,
	}, {
		.compatible = "samsung,exynos5433-pd",
		.data = &exynos5433_cfg,
	}, {
		.compatible = "samsung,exynos8895-pd",
		.data = &exynos8895_cfg,
	},
	{ },
};

static const char *exynos_get_domain_name(struct device *dev,
					  struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = kbasename(node->full_name);
	return devm_kstrdup_const(dev, name, GFP_KERNEL);
}

static int exynos_pd_get_extra(struct device *dev, struct exynos_pm_domain *pd,
			       const struct exynos_pm_domain_config *cfg)
{
	struct device_node *np = dev->of_node;
	const struct exynos_pd_extra *x = NULL;
	struct resource res;
	unsigned int i;

	if (!cfg->nr_extra || of_address_to_resource(np, 0, &res))
		return 0;

	for (i = 0; i < cfg->nr_extra; i++)
		if (cfg->extra[i].conf == res.start)
			x = &cfg->extra[i];
	if (!x)
		return 0;

	if (x->pmu_off_mask) {
		pd->pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
		if (IS_ERR(pd->pmu))
			return dev_err_probe(dev, PTR_ERR(pd->pmu),
					     "power sequence needs samsung,pmu-syscon\n");
	}

	if (x->nr_sysreg) {
		if (x->sysreg_required)
			pd->sysreg = syscon_regmap_lookup_by_phandle(np, "samsung,sysreg");
		else
			pd->sysreg = syscon_regmap_lookup_by_phandle_optional(np, "samsung,sysreg");
		if (IS_ERR(pd->sysreg))
			return dev_err_probe(dev, PTR_ERR(pd->sysreg),
					     "samsung,sysreg\n");
	}

	if (!pd->bus_clk) {
		/*
		 * Kept prepared: the clk prepare_lock nests the CMU's genpd, so
		 * only clk_enable()/clk_disable() run under the genpd lock.
		 */
		pd->bus_clk = devm_clk_get_optional_prepared(dev, "bus");
		if (IS_ERR(pd->bus_clk))
			return dev_err_probe(dev, PTR_ERR(pd->bus_clk),
					     "bus clock\n");
	}

	if (x->nr_pre_on || x->nr_post_off) {
		/* not requested: the block's clock driver owns this range */
		pd->seq_base = of_iomap(np, 1);
		if (!pd->seq_base)
			return dev_err_probe(dev, -ENODEV,
					     "power sequence needs reg 1\n");
	}

	pd->extra = x;
	pd->pd.flags |= x->genpd_flags;
	return 0;
}

static int exynos_pd_probe(struct platform_device *pdev)
{
	const struct exynos_pm_domain_config *pm_domain_cfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct of_phandle_args child, parent;
	struct exynos_pm_domain *pd;
	int on, ret;

	pm_domain_cfg = of_device_get_match_data(dev);
	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->pd.name = exynos_get_domain_name(dev, np);
	if (!pd->pd.name)
		return -ENOMEM;

	pd->base = of_iomap(np, 0);
	if (!pd->base)
		return -ENODEV;

	pd->pd.power_off = exynos_pd_power_off;
	pd->pd.power_on = exynos_pd_power_on;
	pd->local_pwr_cfg = pm_domain_cfg->local_pwr_cfg;

	ret = exynos_pd_get_extra(dev, pd, pm_domain_cfg);
	if (ret)
		return ret;

	/*
	 * Some Samsung platforms with bootloaders turning on the splash-screen
	 * and handing it over to the kernel, requires the power-domains to be
	 * reset during boot.
	 */
	if (IS_ENABLED(CONFIG_ARM) &&
	    of_device_is_compatible(np, "samsung,exynos4210-pd"))
		exynos_pd_power_off(&pd->pd);

	on = readl_relaxed(pd->base + 0x4) & pd->local_pwr_cfg;

	pm_genpd_init(&pd->pd, NULL, !on);
	ret = of_genpd_add_provider_simple(np, &pd->pd);

	if (ret == 0 && of_parse_phandle_with_args(np, "power-domains",
				      "#power-domain-cells", 0, &parent) == 0) {
		child.np = np;
		child.args_count = 0;

		if (of_genpd_add_subdomain(&parent, &child))
			pr_warn("%pOF failed to add subdomain: %pOF\n",
				parent.np, child.np);
		else
			pr_info("%pOF has as child subdomain: %pOF.\n",
				parent.np, child.np);
	}

	pm_runtime_enable(dev);
	return ret;
}

static struct platform_driver exynos_pd_driver = {
	.probe	= exynos_pd_probe,
	.driver	= {
		.name		= "exynos-pd",
		.of_match_table	= exynos_pm_domain_of_match,
		.suppress_bind_attrs = true,
	}
};

static __init int exynos4_pm_init_power_domain(void)
{
	return platform_driver_register(&exynos_pd_driver);
}
core_initcall(exynos4_pm_init_power_domain);
