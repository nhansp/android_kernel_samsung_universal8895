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
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>

/* One write of a power sequence: mask ~0 = plain write, else read-modify-write */
struct exynos_pd_write {
	u32 off, mask, val;
};

/*
 * Steps of one domain's power sequence outside CONFIGURATION/STATUS. A row is
 * selected by the physical address of the domain's CONFIGURATION register
 * (reg entry 0) and is inert until the node carries what the row writes
 * through: a second reg for pre_on/post_off, samsung,pmu-syscon for
 * pmu_off_reg, samsung,sysreg for sysreg[]. samsung,sysreg is optional for a
 * row whose words are DRCG_EN only (dynamic clock gating) and required when
 * sysreg_required is set, i.e. when the list carries a word the block needs
 * after a power cycle.
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
	bool sysreg_required;			/* probe fails without samsung,sysreg */
	unsigned int genpd_flags;
};

/*
 * Exynos8895 PMUCAL local lists (exynos8895-pd-cal.h, generated from
 * CAL:pmucal_local_exynos8895.h). Stock ran, per domain, the save list before
 * power-off and wrote the saved words back after power-on
 * (S:drivers/soc/samsung/cal-if/pmucal_local.c:42-43,81-82): the block CMU's
 * USER muxes, dividers, PLL words, every QCH_CON (Q-channel) word and
 * CMU_CONTROLLER_OPTION, and the SYSREG DRCG_EN. Without the write-back a block
 * comes out of a power cycle with its CMU at reset values, and its next
 * power-off never completes (image 0124: STATUS stuck at step 9).
 */
enum exynos8895_pd_seq_type {
	EXYNOS8895_PD_SAVE,		/* PMUCAL_SAVE_RESTORE */
	EXYNOS8895_PD_READ,		/* PMUCAL_READ: read at save, never written back */
	EXYNOS8895_PD_CHECK_SKIP,	/* PMUCAL_CHECK_SKIP: skip the next entry if the value holds */
	EXYNOS8895_PD_WAIT,		/* PMUCAL_WAIT */
	EXYNOS8895_PD_WRITE,		/* PMUCAL_WRITE */
};

struct exynos8895_pd_seq {
	u8 type, base;			/* base: index into exynos8895_pd_cal.bases[] */
	u16 off;
	u32 mask, val;
};

struct exynos8895_pd_cal {
	phys_addr_t conf;		/* CONFIGURATION, selects the row */
	phys_addr_t bases[3];
	u32 sizes[3];
	const struct exynos8895_pd_seq *save;
	unsigned int nr_save;
	const struct exynos8895_pd_seq *pre_off;	/* before CONFIGURATION = 0 */
	unsigned int nr_pre_off;
};

#include "exynos8895-pd-cal.h"

/* escape hatch: exynos_pm_domains.pmucal=0 leaves the CMU/SYSREG words alone */
static bool pmucal = true;
module_param(pmucal, bool, 0444);

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	const struct exynos_pd_extra *extra;
	unsigned int nr_extra;
	const struct exynos8895_pd_cal *cal;
	unsigned int nr_cal;
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
	struct clk *bus_clk;		/* the block's bus clock, on around the sequences */
	u32 saved[2];
	bool saved_valid;
	const struct exynos8895_pd_cal *cal;
	void __iomem *cal_va[3];
	u32 *cal_val;			/* per save entry: the word read before power-off */
	unsigned long *cal_saved;	/* per save entry: read, to be written back */
};

/* pmucal_rae_check_value(), S:drivers/soc/samsung/cal-if/pmucal_rae.c:73-82 */
static bool exynos8895_pd_cal_ok(struct exynos_pm_domain *pd, const struct exynos8895_pd_seq *e)
{
	return (readl_relaxed(pd->cal_va[e->base] + e->off) & e->mask) == e->val;
}

/* pmucal_rae_write(), :112-122 */
static void exynos8895_pd_cal_write(struct exynos_pm_domain *pd,
				    const struct exynos8895_pd_seq *e, u32 v)
{
	void __iomem *a = pd->cal_va[e->base] + e->off;

	writel_relaxed(e->mask == U32_MAX ? v : (readl_relaxed(a) & ~e->mask) | v, a);
}

/* pmucal_rae_save_seq(), :124-149 */
static void exynos8895_pd_cal_save(struct exynos_pm_domain *pd)
{
	const struct exynos8895_pd_seq *e = pd->cal->save;
	unsigned int i;

	for (i = 0; i < pd->cal->nr_save; i++, e++) {
		if (e->type != EXYNOS8895_PD_SAVE && e->type != EXYNOS8895_PD_READ)
			continue;
		pd->cal_val[i] = readl_relaxed(pd->cal_va[e->base] + e->off) & e->mask;
		if (e->type == EXYNOS8895_PD_SAVE)
			set_bit(i, pd->cal_saved);
		else
			clear_bit(i, pd->cal_saved);
	}
}

/* pmucal_rae_restore_seq(), :151-210; a lock timeout is reported, not fatal */
static void exynos8895_pd_cal_restore(struct exynos_pm_domain *pd)
{
	const struct exynos8895_pd_seq *e = pd->cal->save;
	bool skip = false;
	unsigned int i, t;

	for (i = 0; i < pd->cal->nr_save; i++, e++) {
		if (skip) {
			skip = false;
			continue;
		}
		switch (e->type) {
		case EXYNOS8895_PD_SAVE:
			if (test_and_clear_bit(i, pd->cal_saved))
				exynos8895_pd_cal_write(pd, e, pd->cal_val[i]);
			break;
		case EXYNOS8895_PD_CHECK_SKIP:
			skip = exynos8895_pd_cal_ok(pd, e);
			break;
		case EXYNOS8895_PD_WAIT:	/* pmucal_rae_wait(), :84-103 */
			for (t = 0; !exynos8895_pd_cal_ok(pd, e); t++) {
				if (t > 1000) {
					pr_err("Power domain %s: restore wait at 0x%x timed out\n",
					       pd->pd.name, e->off);
					break;
				}
				udelay(1);
			}
			break;
		case EXYNOS8895_PD_WRITE:
			exynos8895_pd_cal_write(pd, e, e->val);
			break;
		default:
			break;
		}
	}
}

/* the <pd>_off entries before CONFIGURATION (only pd_iva has any left) */
static void exynos8895_pd_cal_pre_off(struct exynos_pm_domain *pd)
{
	const struct exynos8895_pd_seq *e = pd->cal->pre_off;
	unsigned int i;

	for (i = 0; i < pd->cal->nr_pre_off; i++, e++)
		exynos8895_pd_cal_write(pd, e, e->val);
}

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
 * A power-off that times out leaves CONFIGURATION at 0 with STATUS still on:
 * the PMU keeps the request pending, and a later system power-down (Exynos8895
 * SLEEP) runs its sequence with that domain half-way. Request the domain on
 * again, so that it is in the state genpd keeps for it after the error (on).
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

	/*
	 * Stock never gated a block's CMU_TOP feed (no vclk holds the
	 * GATE_CLKCMU_* gates, CAL:cmucal-vclk.c), so the block CMU was clocked
	 * for the save/restore and the power-off handshake. Here the block CMU
	 * driver gates it in its runtime suspend; hold it on around the whole
	 * sequence (clk_enable(NULL) does nothing).
	 */
	clk_enable(pd->bus_clk);
	if (pd->cal && !power_on) {
		exynos8895_pd_cal_save(pd);	/* pmucal_local_disable(), pmucal_local.c:81-82 */
		exynos8895_pd_cal_pre_off(pd);
	}

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
			if (!power_on) {
				exynos_pd_off_rollback(domain, base, pd->local_pwr_cfg);
				/* still powered: nothing to write back */
				if (pd->cal)
					bitmap_zero(pd->cal_saved, pd->cal->nr_save);
			}
			clk_disable(pd->bus_clk);
			return -ETIMEDOUT;
		}
		timeout--;
		cpu_relax();
		usleep_range(80, 100);
	}

	if (pd->cal && power_on)
		exynos8895_pd_cal_restore(pd);	/* pmucal_local_enable(), pmucal_local.c:42-43 */

	exynos_pd_after(pd, power_on);
	clk_disable(pd->bus_clk);

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

/*
 * Exynos8895 rows (FULL_IMAGE_PLAN.md 1.2 X-39). CAL: =
 * S:drivers/soc/samsung/cal-if/exynos8895/; ":N" = line of
 * CAL:pmucal_local_exynos8895.h.
 */

/*
 * pd_g3d (L6A REQ-L1-6A-1): stock g3d_on/g3d_off write the CMU_G3D PLL and
 * mux words around CONFIGURATION (CAL:pmucal_local_exynos8895.h:357-374),
 * through reg 1 of the node (CMU_G3D, 0x13800000). Offsets are the PMUCAL
 * ones verbatim.
 */
static const struct exynos_pd_write exynos8895_g3d_pre_on[] = {
	{ 0x0148, U32_MAX, 0x30000003 },	/* :358 PLL_CON2_PLL_G3D */
	{ 0x0140, BIT(4), BIT(4) },		/* :359 PLL_CON0_PLL_G3D */
	{ 0x0100, BIT(4), BIT(4) },		/* :360 PLL_CON0_MUX_CLKCMU_EMBEDDED_G3D_USER */
};

static const struct exynos_pd_write exynos8895_g3d_post_off[] = {
	{ 0x0100, BIT(4), 0 },			/* :371 PLL_CON0_MUX_CLKCMU_EMBEDDED_G3D_USER */
	{ 0x0140, BIT(4), 0 },			/* :372 PLL_CON0_PLL_G3D */
	{ 0x0148, U32_MAX, 0x10000001 },	/* :373 PLL_CON2_PLL_G3D */
};

/*
 * Multimedia domains (L6B X-L1-2, 5.2.3): BUS_COMPONENT_DRCG_EN at SYSREG
 * + 0x200 is a PMUCAL_SAVE_RESTORE word of the domain's save list, i.e. read
 * before power-off and written back after power-on, only if it was saved
 * (S:drivers/soc/samsung/cal-if/pmucal_local.c:42,81;
 * S:drivers/soc/samsung/cal-if/pmucal_rae.c:135,166-167).
 */
/*
 * Audio domains (L6E 5.4): pd_abox sets ABOX_CA7_OPTION bit 16 in the PMU
 * before power-off (:62) and saves/restores BUS_COMPONENT_DRCG_EN and
 * SNPS_EMA (:57-58); pd_vts saves/restores DRCG_EN (:633); its CMUREF mux
 * step (:637-638) belongs to the CMU_VTS driver. Stock vetoes their
 * power-down while ABOX/VTS run (S:drivers/soc/samsung/exynos-pd.c:211-227);
 * here genpd keeps them on through system suspend while a device in them is
 * on the awake path (GENPD_FLAG_ACTIVE_WAKEUP).
 */
static const struct exynos_pd_extra exynos8895_pd_extra[] = {
	{	/* pd_g3d: CAL:pmucal_local_exynos8895.h:361 EMBEDDED_G3D_CONFIGURATION */
		.conf		= 0x164840e0,
		.pre_on		= exynos8895_g3d_pre_on,
		.nr_pre_on	= ARRAY_SIZE(exynos8895_g3d_pre_on),
		.post_off	= exynos8895_g3d_post_off,
		.nr_post_off	= ARRAY_SIZE(exynos8895_g3d_post_off),
	},
	{	/* pd_cam: :72 CAM_CONFIGURATION; :111 DRCG_EN, SYSREG_CAM 0x12c20000 */
		.conf = 0x16484020, .sysreg = { 0x0200 }, .nr_sysreg = 1,
	},
	{	/* pd_g2d: :315 G2D_CONFIGURATION; :345 DRCG_EN, SYSREG_G2D 0x13a20000 */
		.conf = 0x164840c0, .sysreg = { 0x0200 }, .nr_sysreg = 1,
	},
	{	/* pd_isphq: :381 ISPHQ_CONFIGURATION; :399 DRCG_EN, SYSREG_ISPHQ 0x13120000 */
		.conf = 0x16484100, .sysreg = { 0x0200 }, .nr_sysreg = 1,
	},
	{	/* pd_isplp: :412 ISPLP_CONFIGURATION; :434 DRCG_EN, SYSREG_ISPLP 0x13020000 */
		.conf = 0x16484120, .sysreg = { 0x0200 }, .nr_sysreg = 1,
	},
	{	/* pd_mfc: :498 MFC_CONFIGURATION; :519 DRCG_EN, SYSREG_MFC 0x13c20000 */
		.conf = 0x16484160, .sysreg = { 0x0200 }, .nr_sysreg = 1,
	},
	{	/* pd_dcam: :168 DCAM_CONFIGURATION; :186 DRCG_EN, SYSREG_DCAM 0x12e20000 */
		.conf = 0x16485220, .sysreg = { 0x0200 }, .nr_sysreg = 1,
	},
	{	/* pd_srdz: :532 SRDZ_CONFIGURATION; :554 DRCG_EN, SYSREG_SRDZ 0x14220000 */
		.conf = 0x16485420, .sysreg = { 0x0200 }, .nr_sysreg = 1,
	},
	{	/* pd_abox: :2 ABOX_CONFIGURATION; SYSREG_ABOX 0x13e20000 */
		.conf		= 0x16484000,
		.pmu_off_reg	= 0x2528,	/* :62 ABOX_CA7_OPTION */
		.pmu_off_mask	= BIT(16),	/* :62 (0x1 << 16) */
		.sysreg		= { 0x0200, 0x0300 },	/* :57 DRCG_EN, :58 SNPS_EMA */
		.nr_sysreg	= 2,
		.sysreg_required = true,	/* SNPS_EMA: SRAM margin, not clock gating */
		.genpd_flags	= GENPD_FLAG_ACTIVE_WAKEUP,
	},
	{	/* pd_vts: :599 VTS_CONFIGURATION; :633 DRCG_EN, SYSREG_VTS 0x14020000 */
		.conf		= 0x164841a0,
		.sysreg		= { 0x0200 },
		.nr_sysreg	= 1,
		.genpd_flags	= GENPD_FLAG_ACTIVE_WAKEUP,
	},
};

static const struct exynos_pm_domain_config exynos8895_cfg = {
	.local_pwr_cfg		= 0xf,
	.extra			= exynos8895_pd_extra,
	.nr_extra		= ARRAY_SIZE(exynos8895_pd_extra),
	.cal			= exynos8895_pd_cal,
	.nr_cal			= ARRAY_SIZE(exynos8895_pd_cal),
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
		 * The SYSREG sits behind the block's bus clock, which the
		 * block's CMU gates in its runtime suspend (clk-exynos-arm64
		 * exynos_arm64_cmu_suspend()) before genpd powers the domain
		 * off, and which is still gated when genpd powers it back on
		 * before resuming the CMU. Stock PMUCAL saved/restored these
		 * words with the block clocked. Kept prepared: only
		 * clk_enable()/clk_disable() run under the genpd lock, so no
		 * clk prepare_lock (which nests the CMU's genpd) is taken here.
		 */
		pd->bus_clk = devm_clk_get_optional_prepared(dev, "bus");
		if (IS_ERR(pd->bus_clk))
			return dev_err_probe(dev, PTR_ERR(pd->bus_clk),
					     "bus clock\n");
	}

	if (x->nr_pre_on || x->nr_post_off) {
		/*
		 * Non-requesting map: the block's own clock driver requests
		 * this range (devm_platform_ioremap_resource()).
		 */
		pd->seq_base = of_iomap(np, 1);
		if (!pd->seq_base)
			return dev_err_probe(dev, -ENODEV,
					     "power sequence needs reg 1\n");
	}

	pd->extra = x;
	pd->pd.flags |= x->genpd_flags;
	return 0;
}

static int exynos_pd_get_cal(struct device *dev, struct exynos_pm_domain *pd,
			     const struct exynos_pm_domain_config *cfg)
{
	struct device_node *np = dev->of_node;
	const struct exynos8895_pd_cal *c = NULL;
	struct resource res;
	unsigned int i;

	if (!cfg->nr_cal || !pmucal || of_address_to_resource(np, 0, &res))
		return 0;
	for (i = 0; i < cfg->nr_cal; i++)
		if (cfg->cal[i].conf == res.start)
			c = &cfg->cal[i];
	if (!c)
		return 0;

	/* non-requesting maps: the block CMU and SYSREG drivers request their ranges */
	for (i = 0; i < ARRAY_SIZE(c->bases) && c->bases[i]; i++) {
		pd->cal_va[i] = devm_ioremap(dev, c->bases[i], c->sizes[i]);
		if (!pd->cal_va[i])
			return -ENOMEM;
	}
	pd->cal_val = devm_kcalloc(dev, c->nr_save, sizeof(*pd->cal_val), GFP_KERNEL);
	pd->cal_saved = devm_bitmap_zalloc(dev, c->nr_save, GFP_KERNEL);
	if (!pd->cal_val || !pd->cal_saved)
		return -ENOMEM;

	if (!pd->bus_clk) {	/* see exynos_pd_power() */
		pd->bus_clk = devm_clk_get_optional_prepared(dev, "bus");
		if (IS_ERR(pd->bus_clk))
			return dev_err_probe(dev, PTR_ERR(pd->bus_clk), "bus clock\n");
	}
	pd->cal = c;
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
	ret = exynos_pd_get_cal(dev, pd, pm_domain_cfg);
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
