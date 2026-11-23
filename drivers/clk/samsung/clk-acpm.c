// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos ACPM protocol based clock driver.
 *
 * Copyright 2025 Linaro Ltd.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/container_of.h>
#include <linux/cpumask.h>
#include <linux/device/devres.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/thermal.h>
#include <linux/types.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#include <dt-bindings/clock/samsung,exynos8895-acpm.h>

struct acpm_clk_driver_data;

/**
 * struct acpm_clk - one ACPM DVFS clock.
 * @id:		ACPM DVFS domain index.
 * @hw:		clock hardware.
 * @mbox_chan_id: ACPM channel of the DVFS requests.
 * @handle:	ACPM handle.
 * @pll:	PLL_CON0 register the rate is read back from, or NULL.
 * @rate:	rate of the last successful request (clocks without read-back).
 * @data:	variant data.
 */
struct acpm_clk {
	u32 id;
	struct clk_hw hw;
	unsigned int mbox_chan_id;
	struct acpm_handle *handle;
	void __iomem *pll;
	unsigned long rate;
	const struct acpm_clk_driver_data *data;
};

struct acpm_clk_variant {
	const char *name;
};

/**
 * struct acpm_clk_pll_rate - one PLL setting of a table-matched read-back.
 * @p:		P divider.
 * @m:		M divider.
 * @s:		S divider.
 * @khz:	the DVFS level rate this setting stands for.
 */
struct acpm_clk_pll_rate {
	u8 p;
	u16 m;
	u8 s;
	u32 khz;
};

/**
 * struct acpm_clk_pll_table - PLL settings of a table-matched read-back.
 * @rates:	the settings.
 * @nr:		number of settings.
 */
struct acpm_clk_pll_table {
	const struct acpm_clk_pll_rate *rates;
	unsigned int nr;
};

/**
 * struct acpm_clk_cold_desc - one list of domains told about cold temperature.
 * @label:	name used in the log line.
 * @zone:	thermal zone whose temperature decides.
 * @ids:	ACPM DVFS domains that get the COLDTEMP_REQ.
 * @nr_ids:	number of domains.
 * @initial_cold: state assumed before the first evaluation.
 */
struct acpm_clk_cold_desc {
	const char *label;
	const char *zone;
	const u8 *ids;
	unsigned int nr_ids;
	bool initial_cold;
};

/**
 * struct acpm_clk_driver_data - per-SoC data.
 * @clks:	clock names, index = ACPM DVFS domain index.
 * @nr_clks:	number of clocks.
 * @mbox_chan_id: ACPM channel of the DVFS requests.
 * @no_freq_get: the firmware's FREQ_GET is not used; a clock without PLL
 *		read-back reports the last rate it was set to.
 * @pll_con0:	per clock id, physical address of the PLL_CON0 register its
 *		rate is decoded from (0 = none).
 * @nr_pll_con0: entries in @pll_con0, and in @pll_cpus and @pll_tables when
 *		those are set.
 * @pll_cpus:	per clock id with a PLL read-back, the CPUs of the cluster
 *		whose CMU holds the PLL (its registers are read only while
 *		one of them is online); 0 = no such condition.
 * @pll_tables:	per clock id with a PLL read-back, a table of PLL settings
 *		whose rate is returned instead of the decoded rate (NULL =
 *		decode); no match reads 0.
 * @cold:	lists of domains told about cold temperature (COLDTEMP_REQ).
 * @nr_cold:	number of lists.
 * @cold_mc:	below this temperature (millicelsius) a list is cold.
 * @cold_poll_ms: how often the zones are read.
 */
struct acpm_clk_driver_data {
	const struct acpm_clk_variant *clks;
	unsigned int nr_clks;
	unsigned int mbox_chan_id;
	bool no_freq_get;
	const u32 *pll_con0;
	unsigned int nr_pll_con0;
	const unsigned long *pll_cpus;
	const struct acpm_clk_pll_table * const *pll_tables;
	const struct acpm_clk_cold_desc *cold;
	unsigned int nr_cold;
	int cold_mc;
	unsigned int cold_poll_ms;
};

/**
 * struct acpm_clk_cold - state of one cold list.
 * @desc:	the list.
 * @tz:		its thermal zone, once registered.
 * @cold:	the state last sent.
 */
struct acpm_clk_cold {
	const struct acpm_clk_cold_desc *desc;
	struct thermal_zone_device *tz;
	bool cold;
};

/**
 * struct acpm_clk_priv - per-device data of the cold-temperature logic.
 * @dev:	the clocks device.
 * @handle:	ACPM handle.
 * @data:	variant data.
 * @mbox_chan_id: ACPM channel of the DVFS requests.
 * @lock:	serializes the evaluations.
 * @suspended:	between PM_SUSPEND_PREPARE and PM_POST_SUSPEND.
 * @cold:	one state per list.
 * @work:	periodic evaluation.
 * @pm_nb:	system sleep notifier.
 */
struct acpm_clk_priv {
	struct device *dev;
	struct acpm_handle *handle;
	const struct acpm_clk_driver_data *data;
	unsigned int mbox_chan_id;
	struct mutex lock;	/* cold-state evaluations */
	bool suspended;
	struct acpm_clk_cold *cold;
	struct delayed_work work;
	struct notifier_block pm_nb;
};

#define to_acpm_clk(clk) container_of(clk, struct acpm_clk, hw)

#define ACPM_CLK(cname)					\
	{						\
		.name		= cname,		\
	}

static const struct acpm_clk_variant gs101_acpm_clks[] = {
	ACPM_CLK("mif"),
	ACPM_CLK("int"),
	ACPM_CLK("cpucl0"),
	ACPM_CLK("cpucl1"),
	ACPM_CLK("cpucl2"),
	ACPM_CLK("g3d"),
	ACPM_CLK("g3dl2"),
	ACPM_CLK("tpu"),
	ACPM_CLK("intcam"),
	ACPM_CLK("tnr"),
	ACPM_CLK("cam"),
	ACPM_CLK("mfc"),
	ACPM_CLK("disp"),
	ACPM_CLK("bo"),
};

static const struct acpm_clk_driver_data acpm_clk_gs101 = {
	.clks = gs101_acpm_clks,
	.nr_clks = ARRAY_SIZE(gs101_acpm_clks),
	.mbox_chan_id = 0,
};

static const struct acpm_clk_variant exynos8895_acpm_clks[] = {
	ACPM_CLK("mif"),
	ACPM_CLK("int"),
	ACPM_CLK("cpucl0"),
	ACPM_CLK("cpucl1"),
	ACPM_CLK("g3d"),
	ACPM_CLK("intcam"),
	ACPM_CLK("cam"),
	ACPM_CLK("disp"),
	ACPM_CLK("g3dm"),
	ACPM_CLK("cp"),
};

#define EXYNOS8895_ACPM_NR_CLKS		ARRAY_SIZE(exynos8895_acpm_clks)

#define EXYNOS8895_PLL_CON0_PLL_CPUCL0	0x16800120
#define EXYNOS8895_PLL_CON0_PLL_CPUCL1	0x16900120
/* CMU_G3D is outside the G3D power domain: readable with the GPU off. */
#define EXYNOS8895_PLL_CON0_PLL_G3D	0x13800120

static const u32 exynos8895_pll_con0[EXYNOS8895_ACPM_NR_CLKS] = {
	[EXYNOS8895_CLK_ACPM_DVFS_CPUCL0] = EXYNOS8895_PLL_CON0_PLL_CPUCL0,
	[EXYNOS8895_CLK_ACPM_DVFS_CPUCL1] = EXYNOS8895_PLL_CON0_PLL_CPUCL1,
	[EXYNOS8895_CLK_ACPM_DVFS_G3D] = EXYNOS8895_PLL_CON0_PLL_G3D,
};

/*
 * The G3D PLL settings and the DVFS level rate each one stands for. The
 * decoded PLL rate differs from the level rate for four of them.
 */
static const struct acpm_clk_pll_rate exynos8895_g3d_rates[] = {
	{ 4, 129, 0, 839000 }, { 5, 147, 0, 764000 }, { 4, 105, 0, 683000 },
	{ 4, 176, 1, 572000 }, { 4, 168, 1, 546000 }, { 4, 140, 1, 455000 },
	{ 5, 148, 1, 385000 }, { 4, 104, 1, 338000 }, { 4, 160, 2, 260000 },
};

static const struct acpm_clk_pll_table exynos8895_g3d_table = {
	.rates = exynos8895_g3d_rates,
	.nr = ARRAY_SIZE(exynos8895_g3d_rates),
};

static const struct acpm_clk_pll_table * const exynos8895_pll_tables[EXYNOS8895_ACPM_NR_CLKS] = {
	[EXYNOS8895_CLK_ACPM_DVFS_G3D] = &exynos8895_g3d_table,
};

static const unsigned long exynos8895_pll_cpus[EXYNOS8895_ACPM_NR_CLKS] = {
	[EXYNOS8895_CLK_ACPM_DVFS_CPUCL0] = GENMASK(7, 4),	/* Mongoose cpus 4-7 */
	[EXYNOS8895_CLK_ACPM_DVFS_CPUCL1] = GENMASK(3, 0),	/* A53 cpus 0-3 */
};

static const u8 exynos8895_cpu_cold_ids[] = {
	EXYNOS8895_CLK_ACPM_DVFS_MIF, EXYNOS8895_CLK_ACPM_DVFS_INT,
	EXYNOS8895_CLK_ACPM_DVFS_CPUCL0, EXYNOS8895_CLK_ACPM_DVFS_CPUCL1,
	EXYNOS8895_CLK_ACPM_DVFS_INTCAM, EXYNOS8895_CLK_ACPM_DVFS_CAM,
	EXYNOS8895_CLK_ACPM_DVFS_DISP,
};

static const u8 exynos8895_gpu_cold_ids[] = {
	EXYNOS8895_CLK_ACPM_DVFS_G3D,
};

static const struct acpm_clk_cold_desc exynos8895_cold[] = {
	{
		.label = "big",
		.zone = "big-thermal",
		.ids = exynos8895_cpu_cold_ids,
		.nr_ids = ARRAY_SIZE(exynos8895_cpu_cold_ids),
		.initial_cold = false,
	},
	{
		.label = "gpu",
		.zone = "gpu-thermal",
		.ids = exynos8895_gpu_cold_ids,
		.nr_ids = ARRAY_SIZE(exynos8895_gpu_cold_ids),
		/* as the vendor driver: the first warm reading sends "not cold" */
		.initial_cold = true,
	},
};

static const struct acpm_clk_driver_data acpm_clk_exynos8895 = {
	.clks = exynos8895_acpm_clks,
	.nr_clks = ARRAY_SIZE(exynos8895_acpm_clks),
	.mbox_chan_id = 0,
	/* the vendor kernel never sends FREQ_GET */
	.no_freq_get = true,
	.pll_con0 = exynos8895_pll_con0,
	.nr_pll_con0 = ARRAY_SIZE(exynos8895_pll_con0),
	.pll_cpus = exynos8895_pll_cpus,
	.pll_tables = exynos8895_pll_tables,
	.cold = exynos8895_cold,
	.nr_cold = ARRAY_SIZE(exynos8895_cold),
	.cold_mc = 15000,
	.cold_poll_ms = 1000,
};

#define ACPM_PLL_ENABLE		BIT(31)
#define ACPM_PLL_MUX_SEL	BIT(4)
#define ACPM_PLL_M		GENMASK(25, 16)
#define ACPM_PLL_P		GENMASK(13, 8)
#define ACPM_PLL_S		GENMASK(2, 0)
#define ACPM_PLL_FIN_HZ		(26 * HZ_PER_MHZ)

/*
 * Integer PLL: a disabled PLL gives 26 MHz, or 0 with its mux bypass set;
 * otherwise Fout = 26 MHz * M / (P << S).
 */
static unsigned long acpm_clk_pll_decode(u32 v)
{
	u32 p = FIELD_GET(ACPM_PLL_P, v), m = FIELD_GET(ACPM_PLL_M, v);
	u32 s = FIELD_GET(ACPM_PLL_S, v);

	if (!(v & ACPM_PLL_ENABLE))
		return (v & ACPM_PLL_MUX_SEL) ? 0 : ACPM_PLL_FIN_HZ;

	return p ? div_u64((u64)ACPM_PLL_FIN_HZ * m, p << s) : 0;
}

/*
 * Table match: the level whose P/M/S equal the live ones, 0 without a match.
 * The ENABLE bit is ignored, as in the vendor driver.
 */
static unsigned long acpm_clk_pll_match(const struct acpm_clk_pll_table *tbl,
					u32 v)
{
	u32 p = FIELD_GET(ACPM_PLL_P, v), m = FIELD_GET(ACPM_PLL_M, v);
	u32 s = FIELD_GET(ACPM_PLL_S, v);
	unsigned int i;

	for (i = 0; i < tbl->nr; i++)
		if (tbl->rates[i].p == p && tbl->rates[i].m == m &&
		    tbl->rates[i].s == s)
			return (unsigned long)tbl->rates[i].khz * HZ_PER_KHZ;

	return 0;
}

static unsigned long acpm_clk_pll_rate(const struct acpm_clk *clk)
{
	const struct acpm_clk_driver_data *d = clk->data;
	u32 v = readl(clk->pll);

	if (d->pll_tables && clk->id < d->nr_pll_con0 && d->pll_tables[clk->id])
		return acpm_clk_pll_match(d->pll_tables[clk->id], v);

	return acpm_clk_pll_decode(v);
}

/* The PLL may be read: no CPU condition, or a CPU of its cluster is online. */
static bool acpm_clk_pll_readable(const struct acpm_clk *clk)
{
	const struct acpm_clk_driver_data *d = clk->data;
	unsigned long cpus;

	if (!d->pll_cpus || clk->id >= d->nr_pll_con0 || !d->pll_cpus[clk->id])
		return true;

	cpus = d->pll_cpus[clk->id];
	return cpumask_intersects(cpu_online_mask, to_cpumask(&cpus));
}

static unsigned long acpm_clk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct acpm_clk *clk = to_acpm_clk(hw);

	if (clk->pll) {
		if (!acpm_clk_pll_readable(clk))
			return clk->rate;
		return acpm_clk_pll_rate(clk);
	}

	if (clk->data->no_freq_get)
		return clk->rate;

	return clk->handle->ops.dvfs_ops.get_rate(clk->handle,
					clk->mbox_chan_id, clk->id);
}

static int acpm_clk_determine_rate(struct clk_hw *hw,
				   struct clk_rate_request *req)
{
	/*
	 * We can't figure out what rate it will be, so just return the
	 * rate back to the caller. acpm_clk_recalc_rate() will be called
	 * after the rate is set and we'll know what rate the clock is
	 * running at then.
	 */
	return 0;
}

static int acpm_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct acpm_clk *clk = to_acpm_clk(hw);
	int ret;

	ret = clk->handle->ops.dvfs_ops.set_rate(clk->handle,
					clk->mbox_chan_id, clk->id, rate);
	if (!ret)
		clk->rate = rate;

	return ret;
}

static const struct clk_ops acpm_clk_ops = {
	.recalc_rate = acpm_clk_recalc_rate,
	.determine_rate = acpm_clk_determine_rate,
	.set_rate = acpm_clk_set_rate,
};

static int acpm_clk_register(struct device *dev, struct acpm_clk *aclk,
			     const char *name)
{
	struct clk_init_data init = {};

	init.name = name;
	init.ops = &acpm_clk_ops;
	/* the firmware may change the PLL on its own */
	if (aclk->pll)
		init.flags = CLK_GET_RATE_NOCACHE;
	aclk->hw.init = &init;

	return devm_clk_hw_register(dev, &aclk->hw);
}

/*
 * Map the PLL_CON0 register of a clock with read-back. If the DVFS plugin's
 * fvmap names the PLL at another offset, follow it, as the vendor driver
 * does. No region request: the CMU driver owns the region.
 */
static int acpm_clk_map_pll(struct device *dev, struct acpm_clk *aclk)
{
	const struct acpm_clk_driver_data *d = aclk->data;
	u32 pa, fw_pa;

	if (aclk->id >= d->nr_pll_con0 || !d->pll_con0[aclk->id])
		return 0;

	pa = d->pll_con0[aclk->id];
	if (!acpm_get_fvmap_pll_pa(aclk->handle, aclk->id, &fw_pa) &&
	    (fw_pa & 0xffff) != (pa & 0xffff)) {
		u32 moved = pa - (pa & 0xffff) + (fw_pa & 0xffff);

		dev_warn(dev, "%s PLL moved by fvmap: 0x%08x -> 0x%08x\n",
			 d->clks[aclk->id].name, pa, moved);
		pa = moved;
	}

	aclk->pll = devm_ioremap(dev, pa, 4);
	if (!aclk->pll)
		return -ENOMEM;

	return 0;
}

/*
 * Cold means suspending or below cold_mc. The domains are told only when the
 * state changes, and the state is recorded before the requests go out.
 */
static void acpm_clk_cold_eval(struct acpm_clk_priv *p, struct acpm_clk_cold *c)
{
	const struct acpm_clk_cold_desc *d = c->desc;
	char ids[48];
	int t = 0, ret = 0, r, len = 0;
	unsigned int i;
	bool cold;

	if (!c->tz) {
		c->tz = thermal_zone_get_zone_by_name(d->zone);
		if (IS_ERR(c->tz))
			c->tz = NULL;
	}

	if (!p->suspended && (!c->tz || thermal_zone_get_temp(c->tz, &t)))
		return;		/* zone not up (yet) or unreadable */

	cold = p->suspended || t < p->data->cold_mc;
	if (cold == c->cold)
		return;
	c->cold = cold;

	for (i = 0; i < d->nr_ids; i++) {
		r = p->handle->ops.dvfs_ops.set_cold_temp(p->handle,
							  p->mbox_chan_id,
							  d->ids[i], cold);
		if (r && !ret)
			ret = r;
		len += scnprintf(ids + len, sizeof(ids) - len, "%s%u",
				 i ? "," : "", d->ids[i]);
	}

	if (ret)
		dev_err(p->dev, "coldtemp %d (%s %d C) ids %s failed (%d)\n",
			cold, d->label, t / 1000, ids, ret);
	else
		dev_dbg(p->dev, "coldtemp %d (%s %d C) ids %s\n",
			cold, d->label, t / 1000, ids);
}

static void acpm_clk_cold_eval_all(struct acpm_clk_priv *p)
{
	unsigned int i;

	guard(mutex)(&p->lock);
	for (i = 0; i < p->data->nr_cold; i++)
		acpm_clk_cold_eval(p, &p->cold[i]);
}

static void acpm_clk_cold_work(struct work_struct *work)
{
	struct acpm_clk_priv *p = container_of(to_delayed_work(work),
					       struct acpm_clk_priv, work);

	acpm_clk_cold_eval_all(p);
	queue_delayed_work(system_freezable_wq, &p->work,
			   msecs_to_jiffies(p->data->cold_poll_ms));
}

/* System suspend counts as cold. */
static int acpm_clk_pm_notifier(struct notifier_block *nb, unsigned long event,
				void *unused)
{
	struct acpm_clk_priv *p = container_of(nb, struct acpm_clk_priv, pm_nb);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		p->suspended = true;
		acpm_clk_cold_eval_all(p);
		break;
	case PM_POST_SUSPEND:
		p->suspended = false;
		acpm_clk_cold_eval_all(p);
		break;
	}

	return NOTIFY_OK;
}

static void acpm_clk_cold_stop(void *data)
{
	struct acpm_clk_priv *p = data;

	unregister_pm_notifier(&p->pm_nb);
	cancel_delayed_work_sync(&p->work);
}

static int acpm_clk_cold_init(struct device *dev, struct acpm_handle *handle,
			      const struct acpm_clk_driver_data *data)
{
	struct acpm_clk_priv *p;
	unsigned int i;
	int ret;

	if (!data->nr_cold)
		return 0;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->cold = devm_kcalloc(dev, data->nr_cold, sizeof(*p->cold), GFP_KERNEL);
	if (!p->cold)
		return -ENOMEM;

	p->dev = dev;
	p->handle = handle;
	p->data = data;
	p->mbox_chan_id = data->mbox_chan_id;
	mutex_init(&p->lock);
	for (i = 0; i < data->nr_cold; i++) {
		p->cold[i].desc = &data->cold[i];
		p->cold[i].cold = data->cold[i].initial_cold;
	}
	INIT_DELAYED_WORK(&p->work, acpm_clk_cold_work);
	p->pm_nb.notifier_call = acpm_clk_pm_notifier;

	ret = register_pm_notifier(&p->pm_nb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, acpm_clk_cold_stop, p);
	if (ret)
		return ret;

	queue_delayed_work(system_freezable_wq, &p->work,
			   msecs_to_jiffies(data->cold_poll_ms));

	return 0;
}

static int acpm_clk_probe(struct platform_device *pdev)
{
	const struct acpm_clk_driver_data *data;
	struct acpm_handle *acpm_handle;
	struct clk_hw_onecell_data *clk_data;
	struct clk_hw **hws;
	struct device *dev = &pdev->dev;
	struct acpm_clk *aclks;
	unsigned int mbox_chan_id;
	int i, err, count;

	acpm_handle = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(acpm_handle))
		return dev_err_probe(dev, PTR_ERR(acpm_handle),
				     "Failed to get acpm handle\n");

	data = (const struct acpm_clk_driver_data *)
		platform_get_device_id(pdev)->driver_data;
	count = data->nr_clks;
	mbox_chan_id = data->mbox_chan_id;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, count),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = count;
	hws = clk_data->hws;

	aclks = devm_kcalloc(dev, count, sizeof(*aclks), GFP_KERNEL);
	if (!aclks)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		struct acpm_clk *aclk = &aclks[i];

		/*
		 * The code assumes the clock IDs start from zero,
		 * are sequential and do not have gaps.
		 */
		aclk->id = i;
		aclk->handle = acpm_handle;
		aclk->mbox_chan_id = mbox_chan_id;
		aclk->data = data;

		err = acpm_clk_map_pll(dev, aclk);
		if (err)
			return dev_err_probe(dev, err, "Failed to map the %s PLL\n",
					     data->clks[i].name);

		/* the rate a read-back clock is at before its first request */
		if (aclk->pll && acpm_clk_pll_readable(aclk))
			aclk->rate = acpm_clk_pll_rate(aclk);

		hws[i] = &aclk->hw;

		err = acpm_clk_register(dev, aclk, data->clks[i].name);
		if (err)
			return dev_err_probe(dev, err,
					     "Failed to register clock\n");
	}

	err = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_data);
	if (err)
		return err;

	err = acpm_clk_cold_init(dev, acpm_handle, data);
	if (err)
		return dev_err_probe(dev, err, "Failed to start the cold-temperature requests\n");

	return 0;
}

static const struct platform_device_id acpm_clk_id[] = {
	{ "gs101-acpm-clk", (kernel_ulong_t)&acpm_clk_gs101 },
	{ "exynos8895-acpm-clk", (kernel_ulong_t)&acpm_clk_exynos8895 },
	{}
};
MODULE_DEVICE_TABLE(platform, acpm_clk_id);

static struct platform_driver acpm_clk_driver = {
	.driver	= {
		.name = "acpm-clocks",
	},
	.probe = acpm_clk_probe,
	.id_table = acpm_clk_id,
};
module_platform_driver(acpm_clk_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos ACPM clock driver");
MODULE_LICENSE("GPL");
