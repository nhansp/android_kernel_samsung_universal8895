// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 bus DVFS: MIF, INT, INTCAM, CAM and DISP.
 *
 * The APM firmware owns the bus clocks and voltages. A level is requested
 * through the domain's ACPM DVFS clock. The vendor kernel's PM QoS throughput
 * classes are aggregated here, one class per domain, and exported as the
 * exynos_bus_qos_*() API.
 */

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/crc32.h>
#include <linux/devfreq.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/unaligned.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#include <soc/samsung/bts.h>
#include <soc/samsung/exynos8895-bus-qos.h>
#if IS_ENABLED(CONFIG_SND_SOC_SAMSUNG_ABOX)
#include <sound/samsung/abox.h>
#endif

/* the ECT the tables of this driver were taken from: "PARA", version "003j" */
#define E8895_ECT_TOTAL		0xaf68
#define E8895_ECT_CRC		0xb6f123b6

/* ACPM channel of the APM's unsolicited MIF requests */
#define E8895_ACPM_NOTIFY_CHAN	5
/* ACPM channel of the DATA_INIT and RELEASE messages */
#define E8895_ACPM_DEVFREQ_CHAN	1
/* last word of the final DATA_INIT message */
#define E8895_DATA_INIT_SET_CONST	1

/*
 * ECT MINLOCK table of dvfs_mif: {MIF kHz, minimum INT kHz}. The first column
 * is also the ECT dvfs_mif level list that DATA_INIT walks.
 */
static const struct {
	u32 mif_khz;
	u32 int_khz;
} e8895_mif_int[] = {
	{ 2093000, 400000 }, { 2002000, 400000 }, { 1794000, 400000 },
	{ 1540000, 267000 }, { 1352000, 267000 }, { 1014000, 107000 },
	{ 845000, 107000 }, { 676000, 107000 }, { 546000, 107000 },
	{ 421000, 107000 }, { 286000, 107000 }, { 208000, 107000 },
};

enum e8895_qos_state {
	E8895_QOS_UNKNOWN,	/* bus node enabled, probe not finished */
	E8895_QOS_LIVE,		/* devfreq registered */
	E8895_QOS_DEAD,		/* probe finished without a devfreq */
};

/*
 * One PM QoS class. Statically initialised: consumers may add requests from
 * any initcall level, before the bus device probes.
 */
struct e8895_qos_class {
	struct mutex lock;	/* protects reqs, agg updates, df and state */
	struct list_head reqs;
	s32 agg;		/* max of the active requests, kHz; read locklessly */
	struct devfreq *df;
	u8 state;
};

#define E8895_QOS_CLASS(c)						\
	{								\
		.lock = __MUTEX_INITIALIZER(c.lock),			\
		.reqs = LIST_HEAD_INIT(c.reqs),				\
	}

static struct e8895_qos_class e8895_qos[EXYNOS_BUS_QOS_NR] = {
	[EXYNOS_BUS_QOS_MIF] = E8895_QOS_CLASS(e8895_qos[EXYNOS_BUS_QOS_MIF]),
	[EXYNOS_BUS_QOS_INT] = E8895_QOS_CLASS(e8895_qos[EXYNOS_BUS_QOS_INT]),
	[EXYNOS_BUS_QOS_INTCAM] = E8895_QOS_CLASS(e8895_qos[EXYNOS_BUS_QOS_INTCAM]),
	[EXYNOS_BUS_QOS_CAM] = E8895_QOS_CLASS(e8895_qos[EXYNOS_BUS_QOS_CAM]),
	[EXYNOS_BUS_QOS_DISP] = E8895_QOS_CLASS(e8895_qos[EXYNOS_BUS_QOS_DISP]),
};

struct e8895_bus_desc {
	const char *compatible;		/* for exynos_bus_qos_class_live() */
	enum exynos_bus_qos_class cls;
	u32 boot_khz;			/* ASV boot level */
	u32 boot_qos_khz;		/* floor for the first boot_qos_s seconds */
	u32 default_khz;		/* the vendor's default_qos floor */
	unsigned int boot_qos_s;
	unsigned int delay_ms;		/* hold before a decrease, 0 = none */
	unsigned int nr_opps;		/* the DT OPP table this data describes */
	u32 min_opp_khz;
	u32 max_opp_khz;
	bool acpm;			/* DATA_INIT/RELEASE messages over ACPM */
	bool apm_notify;		/* the APM's MIF requests */
	int release_type;		/* domain id of the RELEASE message, -1 = none */
	u32 suspend_khz;		/* ASV resume level, pinned while suspended */
	u32 reboot_min_khz;
	u32 reboot_max_khz;		/* 0 = no maximum */
	bool data_init;			/* the DATA_INIT table at probe */
};

static const struct e8895_bus_desc e8895_bus_mif = {
	.compatible = "samsung,exynos8895-bus-mif",
	.cls = EXYNOS_BUS_QOS_MIF,
	.boot_khz = 1794000,
	.boot_qos_khz = 1794000,
	/* below the lowest OPP, which applies */
	.default_khz = 208000,
	.boot_qos_s = 40,
	.delay_ms = 20,
	.nr_opps = 9,
	.min_opp_khz = 286000,
	.max_opp_khz = 1794000,
	.acpm = true,
	.apm_notify = true,
	.release_type = 0,
	.suspend_khz = 421000,
	.reboot_min_khz = 421000,
	.reboot_max_khz = 421000,
	.data_init = true,
};

static const struct e8895_bus_desc e8895_bus_int = {
	.compatible = "samsung,exynos8895-bus-int",
	.cls = EXYNOS_BUS_QOS_INT,
	.boot_khz = 400000,
	.boot_qos_khz = 667000,
	.default_khz = 178000,
	.boot_qos_s = 40,
	.nr_opps = 7,
	.min_opp_khz = 107000,
	.max_opp_khz = 667000,
	.acpm = true,
	.release_type = 1,
	.suspend_khz = 107000,
	.reboot_min_khz = 667000,
};

static const struct e8895_bus_desc e8895_bus_intcam = {
	.compatible = "samsung,exynos8895-bus-intcam",
	.cls = EXYNOS_BUS_QOS_INTCAM,
	.boot_khz = 640000,
	.boot_qos_khz = 640000,
	.default_khz = 640000,
	.boot_qos_s = 40,
	.nr_opps = 4,
	.min_opp_khz = 640000,
	.max_opp_khz = 690000,
	.release_type = -1,
	.reboot_min_khz = 690000,
};

static const struct e8895_bus_desc e8895_bus_cam = {
	.compatible = "samsung,exynos8895-bus-cam",
	.cls = EXYNOS_BUS_QOS_CAM,
	.boot_khz = 630000,
	.boot_qos_khz = 630000,
	.default_khz = 630000,
	.boot_qos_s = 40,
	.nr_opps = 7,
	.min_opp_khz = 630000,
	.max_opp_khz = 690000,
	.release_type = -1,
	.reboot_min_khz = 690000,
};

/* the vendor's boot and reboot floors are above the highest OPP, which applies */
static const struct e8895_bus_desc e8895_bus_disp = {
	.compatible = "samsung,exynos8895-bus-disp",
	.cls = EXYNOS_BUS_QOS_DISP,
	.boot_khz = 533000,
	.boot_qos_khz = 630000,
	/* below the lowest OPP, which applies */
	.default_khz = 134000,
	.boot_qos_s = 40,
	.nr_opps = 3,
	.min_opp_khz = 214000,
	.max_opp_khz = 533000,
	.release_type = -1,
	.reboot_min_khz = 630000,
};

static const struct e8895_bus_desc * const e8895_bus_descs[EXYNOS_BUS_QOS_NR] = {
	[EXYNOS_BUS_QOS_MIF] = &e8895_bus_mif,
	[EXYNOS_BUS_QOS_INT] = &e8895_bus_int,
	[EXYNOS_BUS_QOS_INTCAM] = &e8895_bus_intcam,
	[EXYNOS_BUS_QOS_CAM] = &e8895_bus_cam,
	[EXYNOS_BUS_QOS_DISP] = &e8895_bus_disp,
};

struct e8895_bus {
	struct device *dev;
	const struct e8895_bus_desc *desc;
	struct clk *clk;
	struct acpm_handle *acpm;
	struct devfreq *df;
	struct devfreq_dev_profile profile;
	struct devfreq_simple_interactive_data gov;

	/* the driver's own limits, in kHz */
	struct dev_pm_qos_request floor_req;
	struct dev_pm_qos_request ceil_req;
	struct dev_pm_qos_request boot_req;
	struct dev_pm_qos_request susp_min_req;
	struct dev_pm_qos_request susp_max_req;
	u32 suspend_khz;
	struct notifier_block pm_nb;
	struct delayed_work boot_work;
	struct notifier_block reboot_nb;

	unsigned long cur_khz;		/* last requested level, 0 = none yet; under df->lock */

	/* MIF: the APM's requests */
	struct exynos_bus_qos_request apm_req;
	struct notifier_block apm_nb;
	struct work_struct apm_work;
	atomic_t apm_khz;
};

/*
 * Recompute and publish the aggregate; when it changed and the class's bus
 * devfreq is live, re-evaluate it. Called with c->lock held.
 */
static void e8895_qos_apply(struct e8895_qos_class *c)
{
	struct exynos_bus_qos_request *req;
	s32 agg = 0;

	lockdep_assert_held(&c->lock);

	list_for_each_entry(req, &c->reqs, node)
		agg = max(agg, req->khz);

	if (agg == c->agg)
		return;

	WRITE_ONCE(c->agg, agg);

	if (c->df) {
		mutex_lock(&c->df->lock);
		update_devfreq(c->df);
		mutex_unlock(&c->df->lock);
	}
}

static void e8895_qos_set(struct exynos_bus_qos_request *req, s32 khz)
{
	struct e8895_qos_class *c = &e8895_qos[req->cls];

	mutex_lock(&c->lock);
	if (req->khz != khz) {
		req->khz = khz;
		e8895_qos_apply(c);
	}
	mutex_unlock(&c->lock);
}

static void e8895_qos_timeout(struct work_struct *work)
{
	struct exynos_bus_qos_request *req =
		container_of(to_delayed_work(work), struct exynos_bus_qos_request, timeout);

	e8895_qos_set(req, 0);
}

int exynos_bus_qos_request_active(struct exynos_bus_qos_request *req)
{
	return req->active;
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_request_active);

void exynos_bus_qos_add_request(struct exynos_bus_qos_request *req,
				enum exynos_bus_qos_class cls, s32 khz)
{
	struct e8895_qos_class *c;

	might_sleep();

	if (!req || WARN_ON(cls >= EXYNOS_BUS_QOS_NR))
		return;

	if (exynos_bus_qos_request_active(req)) {
		WARN(1, "%s() called for already added request\n", __func__);
		return;
	}

	c = &e8895_qos[cls];
	req->cls = cls;
	INIT_DELAYED_WORK(&req->timeout, e8895_qos_timeout);

	mutex_lock(&c->lock);
	req->khz = khz;
	list_add_tail(&req->node, &c->reqs);
	req->active = true;
	e8895_qos_apply(c);
	mutex_unlock(&c->lock);
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_add_request);

void exynos_bus_qos_update_request(struct exynos_bus_qos_request *req, s32 khz)
{
	might_sleep();

	if (!req)
		return;

	if (!exynos_bus_qos_request_active(req)) {
		WARN(1, "%s() called for unknown object\n", __func__);
		return;
	}

	if (delayed_work_pending(&req->timeout))
		cancel_delayed_work_sync(&req->timeout);

	e8895_qos_set(req, khz);
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_update_request);

void exynos_bus_qos_update_request_timeout(struct exynos_bus_qos_request *req,
					   s32 khz, unsigned long timeout_us)
{
	might_sleep();

	if (!req)
		return;
	if (WARN(!exynos_bus_qos_request_active(req),
		 "%s called for unknown object.", __func__))
		return;

	if (delayed_work_pending(&req->timeout))
		cancel_delayed_work_sync(&req->timeout);

	e8895_qos_set(req, khz);

	schedule_delayed_work(&req->timeout, usecs_to_jiffies(timeout_us));
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_update_request_timeout);

void exynos_bus_qos_remove_request(struct exynos_bus_qos_request *req)
{
	struct e8895_qos_class *c;

	might_sleep();

	if (!req)
		return;

	if (!exynos_bus_qos_request_active(req)) {
		WARN(1, "%s() called for unknown object\n", __func__);
		return;
	}

	if (delayed_work_pending(&req->timeout))
		cancel_delayed_work_sync(&req->timeout);

	c = &e8895_qos[req->cls];
	mutex_lock(&c->lock);
	list_del(&req->node);
	req->active = false;
	req->khz = 0;
	e8895_qos_apply(c);
	mutex_unlock(&c->lock);
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_remove_request);

int exynos_bus_qos_class_live(enum exynos_bus_qos_class cls)
{
	struct e8895_qos_class *c;
	struct device_node *np;
	bool avail;
	u8 state;

	if (WARN_ON(cls >= EXYNOS_BUS_QOS_NR))
		return 0;

	c = &e8895_qos[cls];
	mutex_lock(&c->lock);
	state = c->state;
	mutex_unlock(&c->lock);

	if (state == E8895_QOS_LIVE)
		return 1;
	if (state == E8895_QOS_DEAD)
		return 0;

	np = of_find_compatible_node(NULL, NULL, e8895_bus_descs[cls]->compatible);
	avail = np && of_device_is_available(np);
	of_node_put(np);

	return avail ? -EPROBE_DEFER : 0;
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_class_live);

/* INT floor for a MIF level: the last MINLOCK row, from the bottom, at or above it */
static u32 e8895_mif_int_min(unsigned long mif_khz)
{
	int i;

	for (i = ARRAY_SIZE(e8895_mif_int) - 1; i >= 0; i--)
		if (mif_khz <= e8895_mif_int[i].mif_khz)
			return e8895_mif_int[i].int_khz;
	return 0;
}

/*
 * The MIF target holds MIF's df->lock, so the INT floor bypasses the class
 * API. Lock order: MIF df->lock -> e8895_couple_lock -> INT df->lock. The
 * value is kept for an INT device that probes later.
 */
static DEFINE_MUTEX(e8895_couple_lock);
static u32 e8895_couple_khz;
static struct e8895_bus *e8895_couple_int;

static void e8895_bus_couple_set(struct e8895_bus *p, u32 int_khz)
{
	mutex_lock_nested(&p->df->lock, SINGLE_DEPTH_NESTING);
	p->gov.coupled_hz = (unsigned long)int_khz * HZ_PER_KHZ;
	update_devfreq(p->df);
	mutex_unlock(&p->df->lock);
}

/* MIF df->lock held */
static void e8895_bus_couple(unsigned long mif_khz)
{
	u32 int_khz = e8895_mif_int_min(mif_khz);

	mutex_lock(&e8895_couple_lock);
	e8895_couple_khz = int_khz;
	if (e8895_couple_int)
		e8895_bus_couple_set(e8895_couple_int, int_khz);
	mutex_unlock(&e8895_couple_lock);
}

static void e8895_bus_couple_attach(struct e8895_bus *priv)
{
	mutex_lock(&e8895_couple_lock);
	e8895_couple_int = priv;
	e8895_bus_couple_set(priv, e8895_couple_khz);
	mutex_unlock(&e8895_couple_lock);
}

/* minimum MIF level per CPU frequency, {cpu kHz, MIF kHz}, highest first */
struct e8895_cpu_mif_row {
	u32 cpu_khz;
	u32 mif_khz;
};

static const struct e8895_cpu_mif_row e8895_cpu_mif_little[] = {
	{ 2002000, 1014000 }, { 1898000, 1014000 }, { 1794000, 1014000 },
	{ 1690000, 1014000 }, { 1456000, 845000 }, { 1248000, 845000 },
	{ 1053000, 845000 }, { 949000, 845000 }, { 832000, 676000 },
	{ 715000, 676000 }, { 598000, 421000 }, { 455000, 0 },
};

static const struct e8895_cpu_mif_row e8895_cpu_mif_big[] = {
	{ 2808000, 1794000 }, { 2704000, 1794000 }, { 2652000, 1794000 },
	{ 2574000, 1794000 }, { 2496000, 1794000 }, { 2314000, 1794000 },
	{ 2158000, 1540000 }, { 2002000, 1540000 }, { 1937000, 1352000 },
	{ 1807000, 1352000 }, { 1703000, 1014000 }, { 1469000, 845000 },
	{ 1261000, 676000 }, { 1170000, 546000 }, { 1066000, 546000 },
	{ 962000, 421000 }, { 858000, 421000 }, { 741000, 421000 },
};

static u32 e8895_cpu_mif_lookup(int cl, unsigned int cpu_khz)
{
	const struct e8895_cpu_mif_row *t = cl ? e8895_cpu_mif_big : e8895_cpu_mif_little;
	int n = cl ? ARRAY_SIZE(e8895_cpu_mif_big) : ARRAY_SIZE(e8895_cpu_mif_little);
	u32 mif = 0;
	int i;

	if (!cpu_khz)
		return 0;

	for (i = 0; i < n; i++) {
		if (cpu_khz <= t[i].cpu_khz)
			mif = t[i].mif_khz;
		if (cpu_khz >= t[i].cpu_khz)
			break;
	}
	return mif;
}

/* the first CPU of each cluster: 0 (Cortex-A53), 4 (Mongoose) */
static const unsigned int e8895_cpu_mif_cpu[2] = { 0, 4 };

static struct exynos_bus_qos_request e8895_cpu_mif_req[2];

/* MIF goes up before the CPU does and down after it */
static int e8895_cpu_mif_notify(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs = data;
	int cl;

	cl = cpumask_test_cpu(0, freqs->policy->related_cpus) ? 0 : 1;
	if ((val == CPUFREQ_PRECHANGE && freqs->new > freqs->old) ||
	    (val == CPUFREQ_POSTCHANGE && freqs->new < freqs->old))
		exynos_bus_qos_update_request(&e8895_cpu_mif_req[cl],
					      e8895_cpu_mif_lookup(cl, freqs->new));

	return NOTIFY_OK;
}

static struct notifier_block e8895_cpu_mif_nb = {
	.notifier_call = e8895_cpu_mif_notify,
};

static void e8895_cpu_mif_remove(void *data)
{
	int cl;

	cpufreq_unregister_notifier(&e8895_cpu_mif_nb, CPUFREQ_TRANSITION_NOTIFIER);
	for (cl = 0; cl < 2; cl++)
		exynos_bus_qos_remove_request(&e8895_cpu_mif_req[cl]);
}

static int e8895_cpu_mif_init(struct e8895_bus *priv)
{
	int cl, ret;

	for (cl = 0; cl < 2; cl++)
		exynos_bus_qos_add_request(&e8895_cpu_mif_req[cl], EXYNOS_BUS_QOS_MIF, 0);
	ret = cpufreq_register_notifier(&e8895_cpu_mif_nb, CPUFREQ_TRANSITION_NOTIFIER);
	if (ret) {
		for (cl = 0; cl < 2; cl++)
			exynos_bus_qos_remove_request(&e8895_cpu_mif_req[cl]);
		return ret;
	}
	for (cl = 0; cl < 2; cl++) {
		unsigned int cpu_khz = cpufreq_quick_get(e8895_cpu_mif_cpu[cl]);

		exynos_bus_qos_update_request(&e8895_cpu_mif_req[cl],
					      e8895_cpu_mif_lookup(cl, cpu_khz));
	}

	return devm_add_action_or_reset(priv->dev, e8895_cpu_mif_remove, NULL);
}

/* Called by the devfreq core with df->lock held. */
static int e8895_bus_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct e8895_bus *priv = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long new_khz;
	bool up;
	int ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	new_khz = dev_pm_opp_get_freq(opp) / HZ_PER_KHZ;
	dev_pm_opp_put(opp);

	/* cur_khz starts at 0, so the first evaluation always sends the boot level */
	if (new_khz == priv->cur_khz)
		return 0;

	up = new_khz > priv->cur_khz;

	if (priv->desc->cls == EXYNOS_BUS_QOS_MIF && up)
		e8895_bus_couple(new_khz);

	if (priv->desc->cls == EXYNOS_BUS_QOS_MIF && !up)
		bts_update_scen(BS_MIF_CHANGE, new_khz);

	ret = clk_set_rate(priv->clk, new_khz * HZ_PER_KHZ);
	if (ret) {
		dev_err(dev, "FREQ_REQ %lu kHz failed: %d\n", new_khz, ret);
		return ret;
	}

	if (priv->desc->cls == EXYNOS_BUS_QOS_MIF && !up)
		e8895_bus_couple(new_khz);

	if (priv->desc->cls == EXYNOS_BUS_QOS_MIF && up)
		bts_update_scen(BS_MIF_CHANGE, new_khz);

	WRITE_ONCE(priv->cur_khz, new_khz);
	*freq = new_khz * HZ_PER_KHZ;

	return 0;
}

/* the MIF level is not readable from one register: report the last request */
static int e8895_bus_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct e8895_bus *priv = dev_get_drvdata(dev);

	*freq = READ_ONCE(priv->cur_khz) * HZ_PER_KHZ;

	return 0;
}

static int e8895_bus_ect_check(struct device *dev)
{
	struct reserved_mem *rmem;
	struct device_node *np;
	u8 *va;
	u32 crc, total;
	bool ok;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	rmem = np ? of_reserved_mem_lookup(np) : NULL;
	of_node_put(np);
	if (!rmem || rmem->size < E8895_ECT_TOTAL) {
		dev_err(dev, "no ECT memory-region of at least %#x bytes\n",
			E8895_ECT_TOTAL);
		return -ENODEV;
	}

	va = memremap(rmem->base, E8895_ECT_TOTAL, MEMREMAP_WB);
	if (!va) {
		dev_err(dev, "cannot map the ECT at %pa\n", &rmem->base);
		return -ENOMEM;
	}

	total = get_unaligned_le32(va + 8);
	crc = ~crc32_le(~0, va, E8895_ECT_TOTAL);
	ok = !memcmp(va, "PARA", 4) && !memcmp(va + 4, "003j", 4) &&
	     total == E8895_ECT_TOTAL && crc == E8895_ECT_CRC;
	if (!ok)
		dev_err(dev, "unknown ECT: crc 0x%08x ver %.4s total %#x\n",
			crc, (const char *)va + 4, total);

	memunmap(va);

	return ok ? 0 : -EINVAL;
}

static int e8895_bus_check_opps(struct e8895_bus *priv)
{
	const struct e8895_bus_desc *desc = priv->desc;
	struct device *dev = priv->dev;
	unsigned long lo = 0, hi = ULONG_MAX;
	struct dev_pm_opp *opp;
	int count;

	count = dev_pm_opp_get_opp_count(dev);

	opp = dev_pm_opp_find_freq_ceil(dev, &lo);
	if (IS_ERR(opp))
		lo = 0;
	else
		dev_pm_opp_put(opp);

	opp = dev_pm_opp_find_freq_floor(dev, &hi);
	if (IS_ERR(opp))
		hi = 0;
	else
		dev_pm_opp_put(opp);

	if (count != desc->nr_opps || lo != desc->min_opp_khz * HZ_PER_KHZ ||
	    hi != desc->max_opp_khz * HZ_PER_KHZ) {
		dev_err(dev, "OPP table (%d, %lu..%lu Hz) does not match the driver (%u, %u..%u kHz)\n",
			count, lo, hi, desc->nr_opps, desc->min_opp_khz, desc->max_opp_khz);
		return -EINVAL;
	}

	return 0;
}

static void e8895_bus_remove_qos(void *data)
{
	struct e8895_bus *priv = data;

	if (dev_pm_qos_request_active(&priv->susp_max_req))
		dev_pm_qos_remove_request(&priv->susp_max_req);
	if (dev_pm_qos_request_active(&priv->susp_min_req))
		dev_pm_qos_remove_request(&priv->susp_min_req);
	if (dev_pm_qos_request_active(&priv->boot_req))
		dev_pm_qos_remove_request(&priv->boot_req);
	if (dev_pm_qos_request_active(&priv->ceil_req))
		dev_pm_qos_remove_request(&priv->ceil_req);
	if (dev_pm_qos_request_active(&priv->floor_req))
		dev_pm_qos_remove_request(&priv->floor_req);
}

static int e8895_bus_add_qos(struct e8895_bus *priv)
{
	struct device *dev = priv->dev;
	int ret;

	ret = dev_pm_qos_add_request(dev, &priv->floor_req, DEV_PM_QOS_MIN_FREQUENCY,
				     priv->desc->default_khz);
	if (ret < 0)
		return ret;

	ret = dev_pm_qos_add_request(dev, &priv->ceil_req, DEV_PM_QOS_MAX_FREQUENCY,
				     PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE);
	if (ret < 0)
		goto err;

	ret = dev_pm_qos_add_request(dev, &priv->boot_req, DEV_PM_QOS_MIN_FREQUENCY,
				     priv->desc->boot_qos_khz);
	if (ret < 0)
		goto err;

	if (priv->desc->release_type >= 0) {
		ret = dev_pm_qos_add_request(dev, &priv->susp_min_req,
					     DEV_PM_QOS_MIN_FREQUENCY, 0);
		if (ret < 0)
			goto err;
		ret = dev_pm_qos_add_request(dev, &priv->susp_max_req,
					     DEV_PM_QOS_MAX_FREQUENCY,
					     PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE);
		if (ret < 0)
			goto err;
	}

	return devm_add_action_or_reset(dev, e8895_bus_remove_qos, priv);

err:
	e8895_bus_remove_qos(priv);
	return ret;
}

/*
 * MIF: tell the APM the levels inside the OPP range with their INT floors,
 * then the number of levels and the boot level.
 */
static int e8895_bus_data_init(struct e8895_bus *priv)
{
	const struct e8895_bus_desc *desc = priv->desc;
	struct acpm_handle *acpm = priv->acpm;
	unsigned int i, level = 0;
	int ret;

	for (i = 0; i < ARRAY_SIZE(e8895_mif_int); i++) {
		if (e8895_mif_int[i].mif_khz > desc->max_opp_khz ||
		    e8895_mif_int[i].mif_khz < desc->min_opp_khz)
			continue;
		ret = acpm->ops.dvfs_ops.data_init(acpm, E8895_ACPM_DEVFREQ_CHAN, level,
						   e8895_mif_int[i].mif_khz,
						   e8895_mif_int[i].int_khz);
		if (ret)
			goto out;
		level++;
	}

	ret = acpm->ops.dvfs_ops.data_init(acpm, E8895_ACPM_DEVFREQ_CHAN, level,
					   desc->boot_khz, E8895_DATA_INIT_SET_CONST);
out:
	if (ret)
		dev_err(priv->dev, "DATA_INIT of level %u failed: %d\n", level, ret);

	return ret ? -EINVAL : 0;
}

static bool e8895_bus_has_protocol(struct e8895_bus *priv)
{
	return priv && priv->df && priv->desc->release_type >= 0;
}

/* a failed RELEASE is logged, not returned: the vendor driver aborted the suspend */
static void e8895_bus_release(struct e8895_bus *priv, bool running)
{
	int ret;

	mutex_lock(&priv->df->lock);
	ret = priv->acpm->ops.dvfs_ops.sleep_release(priv->acpm, E8895_ACPM_DEVFREQ_CHAN,
						     priv->desc->release_type, running);
	mutex_unlock(&priv->df->lock);
	if (ret)
		dev_err(priv->dev, "RELEASE running=%d failed: %d\n", running, ret);
}

#if IS_ENABLED(CONFIG_SND_SOC_SAMSUNG_ABOX)
/* INT: while A-Box runs, it may need more than the suspend level of INT */
static int e8895_bus_int_pm_notifier(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct e8895_bus *priv = container_of(nb, struct e8895_bus, pm_nb);

	switch (action) {
	case PM_SUSPEND_PREPARE:
		if (abox_is_on()) {
			unsigned int freq = abox_get_requiring_int_freq_in_khz();

			if (freq)
				priv->suspend_khz = freq;
			else
				priv->suspend_khz = priv->desc->suspend_khz;
		} else {
			priv->suspend_khz = priv->desc->suspend_khz;
		}
		break;
	case PM_POST_SUSPEND:
		priv->suspend_khz = priv->desc->suspend_khz;
		break;
	}

	return NOTIFY_OK;
}

static void e8895_bus_unregister_pm(void *data)
{
	struct e8895_bus *priv = data;

	unregister_pm_notifier(&priv->pm_nb);
}
#endif

static int e8895_bus_suspend_late(struct device *dev)
{
	struct e8895_bus *priv = dev_get_drvdata(dev);
	s32 khz;

	if (!e8895_bus_has_protocol(priv))
		return 0;

	e8895_bus_release(priv, false);
	khz = priv->suspend_khz ? priv->suspend_khz : priv->desc->suspend_khz;
	dev_pm_qos_update_request(&priv->susp_max_req, khz);
	dev_pm_qos_update_request(&priv->susp_min_req, khz);

	return 0;
}

static int e8895_bus_resume_early(struct device *dev)
{
	struct e8895_bus *priv = dev_get_drvdata(dev);

	if (!e8895_bus_has_protocol(priv))
		return 0;

	e8895_bus_release(priv, true);
	dev_pm_qos_update_request(&priv->susp_min_req, 0);
	dev_pm_qos_update_request(&priv->susp_max_req,
				  PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE);

	return 0;
}

/* explicit members only: the _PM_OPS helpers would also fill freeze/thaw/poweroff/restore */
static const struct dev_pm_ops e8895_bus_pm_ops = {
	.suspend_late = e8895_bus_suspend_late,
	.resume_early = e8895_bus_resume_early,
};

static int e8895_bus_reboot(struct notifier_block *nb, unsigned long action, void *data)
{
	struct e8895_bus *priv = container_of(nb, struct e8895_bus, reboot_nb);
	const struct e8895_bus_desc *desc = priv->desc;

	if (desc->reboot_max_khz)
		dev_pm_qos_update_request(&priv->ceil_req, desc->reboot_max_khz);
	dev_pm_qos_update_request(&priv->floor_req, desc->reboot_min_khz);

	return NOTIFY_OK;
}

static void e8895_bus_unregister_reboot(void *data)
{
	struct e8895_bus *priv = data;

	unregister_reboot_notifier(&priv->reboot_nb);
}

static void e8895_bus_boot_done(struct work_struct *work)
{
	struct e8895_bus *priv = container_of(to_delayed_work(work),
					      struct e8895_bus, boot_work);

	dev_pm_qos_update_request(&priv->boot_req, 0);
}

static void e8895_bus_cancel_boot(void *data)
{
	struct e8895_bus *priv = data;

	cancel_delayed_work_sync(&priv->boot_work);
}

/* the ACPM notifier runs under the channel's rx_lock, and the update ends in a FREQ_REQ */
static void e8895_bus_apm_work(struct work_struct *work)
{
	struct e8895_bus *priv = container_of(work, struct e8895_bus, apm_work);
	u32 khz = atomic_read(&priv->apm_khz);

	exynos_bus_qos_update_request(&priv->apm_req, khz);
	dev_dbg(priv->dev, "APM MIF request %u kHz\n", khz);
}

static int e8895_bus_apm_notify(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct e8895_bus *priv = container_of(nb, struct e8895_bus, apm_nb);
	const u32 *cmd = data;

	if (action != E8895_ACPM_NOTIFY_CHAN)
		return NOTIFY_DONE;

	atomic_set(&priv->apm_khz, cmd[1]);
	queue_work(system_wq, &priv->apm_work);

	return NOTIFY_OK;
}

static void e8895_bus_remove_apm_req(void *data)
{
	struct e8895_bus *priv = data;

	exynos_bus_qos_remove_request(&priv->apm_req);
}

static void e8895_bus_cancel_apm(void *data)
{
	struct e8895_bus *priv = data;

	cancel_work_sync(&priv->apm_work);
}

static int e8895_bus_apm_init(struct e8895_bus *priv)
{
	struct device *dev = priv->dev;
	int ret;

	exynos_bus_qos_add_request(&priv->apm_req, EXYNOS_BUS_QOS_MIF, 0);
	ret = devm_add_action_or_reset(dev, e8895_bus_remove_apm_req, priv);
	if (ret)
		return ret;

	INIT_WORK(&priv->apm_work, e8895_bus_apm_work);
	ret = devm_add_action_or_reset(dev, e8895_bus_cancel_apm, priv);
	if (ret)
		return ret;

	priv->apm_nb.notifier_call = e8895_bus_apm_notify;
	return devm_acpm_register_notifier(dev, priv->acpm, &priv->apm_nb);
}

static void e8895_bus_set_state(struct e8895_qos_class *c, u8 state)
{
	mutex_lock(&c->lock);
	c->state = state;
	if (state != E8895_QOS_LIVE)
		c->df = NULL;
	mutex_unlock(&c->lock);
}

static int e8895_bus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct e8895_bus_desc *desc = of_device_get_match_data(dev);
	struct e8895_qos_class *c = &e8895_qos[desc->cls];
	struct device_node *acpm_np;
	struct e8895_bus *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		e8895_bus_set_state(c, E8895_QOS_DEAD);
		return -ENOMEM;
	}
	priv->dev = dev;
	priv->desc = desc;
	priv->suspend_khz = desc->suspend_khz;
	platform_set_drvdata(pdev, priv);

	/* the tables below are only valid for this ECT: without it, leave the bus alone */
	if (e8895_bus_ect_check(dev)) {
		e8895_bus_set_state(c, E8895_QOS_DEAD);
		return 0;
	}

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(priv->clk), "no ACPM DVFS clock\n");
		goto err;
	}

	ret = devm_pm_opp_of_add_table(dev);
	if (ret) {
		dev_err_probe(dev, ret, "no OPP table\n");
		goto err;
	}

	ret = e8895_bus_check_opps(priv);
	if (ret)
		goto err;

	if (desc->acpm) {
		acpm_np = of_parse_phandle(dev->of_node, "clocks", 0);
		priv->acpm = devm_acpm_get_by_node(dev, acpm_np);
		of_node_put(acpm_np);
		if (IS_ERR(priv->acpm)) {
			ret = dev_err_probe(dev, PTR_ERR(priv->acpm), "no ACPM handle\n");
			goto err;
		}
	}

	ret = e8895_bus_add_qos(priv);
	if (ret) {
		dev_err(dev, "dev_pm_qos requests: %d\n", ret);
		goto err;
	}

	if (desc->data_init) {
		ret = e8895_bus_data_init(priv);
		if (ret)
			goto err;
	}

	priv->gov.delay_ms = desc->delay_ms;
	priv->gov.qos_khz = &c->agg;
	priv->profile.initial_freq = (unsigned long)desc->boot_khz * HZ_PER_KHZ;
	priv->profile.target = e8895_bus_target;
	priv->profile.get_cur_freq = e8895_bus_get_cur_freq;
	priv->profile.polling_ms = 0;

	priv->df = devm_devfreq_add_device(dev, &priv->profile,
					   DEVFREQ_GOV_SIMPLE_INTERACTIVE, &priv->gov);
	if (IS_ERR(priv->df)) {
		ret = dev_err_probe(dev, PTR_ERR(priv->df), "devfreq\n");
		goto err;
	}

	mutex_lock(&c->lock);
	c->df = priv->df;
	c->state = E8895_QOS_LIVE;
	/* a request that arrived between GOV_START and c->df */
	mutex_lock(&priv->df->lock);
	update_devfreq(priv->df);
	mutex_unlock(&priv->df->lock);
	mutex_unlock(&c->lock);

	INIT_DELAYED_WORK(&priv->boot_work, e8895_bus_boot_done);
	ret = devm_add_action_or_reset(dev, e8895_bus_cancel_boot, priv);
	if (ret)
		goto err;
	schedule_delayed_work(&priv->boot_work, desc->boot_qos_s * HZ);

	if (desc->apm_notify) {
		ret = e8895_bus_apm_init(priv);
		if (ret) {
			dev_err(dev, "APM MIF request notifier: %d\n", ret);
			goto err;
		}
	}

	/* not fatal: MIF keeps its other floors */
	if (desc->cls == EXYNOS_BUS_QOS_MIF) {
		ret = e8895_cpu_mif_init(priv);
		if (ret)
			dev_warn(dev, "CPU -> MIF notifier: %d\n", ret);
	}

	if (desc->cls == EXYNOS_BUS_QOS_INT)
		e8895_bus_couple_attach(priv);

	priv->reboot_nb.notifier_call = e8895_bus_reboot;
	ret = register_reboot_notifier(&priv->reboot_nb);
	if (!ret)
		ret = devm_add_action_or_reset(dev, e8895_bus_unregister_reboot, priv);
	if (ret)
		goto err;

#if IS_ENABLED(CONFIG_SND_SOC_SAMSUNG_ABOX)
	if (desc->cls == EXYNOS_BUS_QOS_INT) {
		priv->pm_nb.notifier_call = e8895_bus_int_pm_notifier;
		ret = register_pm_notifier(&priv->pm_nb);
		if (!ret)
			ret = devm_add_action_or_reset(dev, e8895_bus_unregister_pm, priv);
		if (ret)
			goto err;
	}
#endif

	return 0;

err:
	/* the INT devfreq goes away with devm: MIF must not couple into it */
	mutex_lock(&e8895_couple_lock);
	if (e8895_couple_int == priv)
		e8895_couple_int = NULL;
	mutex_unlock(&e8895_couple_lock);
	e8895_bus_set_state(c, ret == -EPROBE_DEFER ? E8895_QOS_UNKNOWN : E8895_QOS_DEAD);
	return ret;
}

static const struct of_device_id e8895_bus_of_match[] = {
	{ .compatible = "samsung,exynos8895-bus-mif", .data = &e8895_bus_mif },
	{ .compatible = "samsung,exynos8895-bus-int", .data = &e8895_bus_int },
	{ .compatible = "samsung,exynos8895-bus-intcam", .data = &e8895_bus_intcam },
	{ .compatible = "samsung,exynos8895-bus-cam", .data = &e8895_bus_cam },
	{ .compatible = "samsung,exynos8895-bus-disp", .data = &e8895_bus_disp },
	{ }
};

static struct platform_driver e8895_bus_driver = {
	.probe = e8895_bus_probe,
	.driver = {
		.name = "exynos8895-bus",
		.of_match_table = e8895_bus_of_match,
		.pm = pm_sleep_ptr(&e8895_bus_pm_ops),
		/* consumers keep the class's devfreq pointer */
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(e8895_bus_driver);
