// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 bus DVFS (MIF, INT, INTCAM, CAM, DISP), spec L2b D1/D4, 5.1, 5.3.
 *
 * One devfreq device per stock bus domain (S:drivers/devfreq/exynos/
 * exynos-devfreq.c and exynos8895_bus_{mif,int,disp,intcam,cam}.c). The APM
 * firmware owns the bus clocks and voltages; a level is requested through the
 * domain's ACPM DVFS clock, whose set_rate is the stock cal_dfs_set_rate()
 * FREQ_REQ on ACPM channel 0 (S:drivers/devfreq/exynos/exynos8895_bus_mif.c:216-225,
 * WT:drivers/clk/samsung/clk-acpm.c:426-438). The stock PM QoS throughput
 * classes are aggregated here, one class per domain, and exported as the
 * exynos_bus_qos_*() API (include/soc/samsung/exynos8895-bus-qos.h).
 *
 * Stages (module parameter "stage", read once at probe; spec L2b 1.3, 5.3.8):
 * 0 = check the ECT and bind, no devfreq; 1 = devfreq pinned at the ECT boot
 * level, no IPC; 2 = the AP casts its boot-level vote once per domain at probe,
 * floors stay at the boot level and votes may only raise; 3 = 2 plus the stock
 * APM protocol (MIF DATA_INIT at probe, MIF/INT RELEASE and suspend level at
 * suspend) and the reboot pins. Every stage above 0 needs the ECT identity
 * check to pass.
 */

#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/devfreq.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/reboot.h>
#include <linux/unaligned.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#include <dt-bindings/clock/samsung,exynos8895-acpm.h>
#include <soc/samsung/exynos8895-bus-qos.h>

/* The stages this tree implements (M1; spec L2b 5.0: the range grows with K-4c/K-4b/K-4d). */
#define E8895_BUS_STAGE_MAX	3

static unsigned int stage = CONFIG_ARM_EXYNOS8895_BUS_DEVFREQ_STAGE;
module_param(stage, uint, 0444);
MODULE_PARM_DESC(stage, "bus DVFS stage, read at probe (0 diagnose, 1 pinned, 2 AP vote)");

/*
 * Stated addition (spec L2b 5.3.6 item 3, O-B7; stock never re-sent): after an
 * APM "MIF request" notification, send the AP's current MIF level again.
 */
static bool ch5_resend;
module_param(ch5_resend, bool, 0444);
MODULE_PARM_DESC(ch5_resend, "resend the AP MIF level after an APM MIF request");

/*
 * The ECT every static table of this driver was transcribed from: the copy
 * embedded in S-Boot at SB:0x45c00, header "PARA" + "003j" + total 0xaf68, zlib
 * CRC32 of the 0xaf68 bytes = 0xb6f123b6 (spec L2b D2, 5.3.2 step 2;
 * rewrite/work/full/spec-usbc/tmp/L2b/ect_ident.txt).
 */
#define E8895_ECT_TOTAL		0xaf68
#define E8895_ECT_CRC		0xb6f123b6

/* ACPM channel of the APM's unsolicited "MIF request" (WT:drivers/firmware/samsung/exynos-acpm.c:2069) */
#define E8895_ACPM_NOTIFY_CHAN	5
/* channel of the ACPM DVFS clocks' FREQ_REQ (WT:drivers/clk/samsung/clk-acpm.c:311) */
#define E8895_ACPM_DVFS_CHAN	0
/* channel of the MIF/INT devfreq messages: acpm-ipc-channel = <0x01> (FDT:6729,6749) */
#define E8895_ACPM_DEVFREQ_CHAN	1
/* last DATA_INIT word: S:include/soc/samsung/exynos-devfreq.h:23 "#define SET_CONST 1" */
#define E8895_DATA_INIT_SET_CONST	1

/*
 * ECT MINLOCK dvfs_mif, {MIF kHz, INT minimum kHz} (sboot.bin SB:0x50ac0, rows 0-11;
 * rewrite/work/full/spec-usbc/tmp/L2b/ect_minlock.txt, spec L2b 5.3.5). The first
 * column is also the ECT dvfs_mif level list (ECT-DVFS:53-64) that DATA_INIT walks.
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
	E8895_QOS_LIVE,		/* devfreq registered at stage >= 2 */
	E8895_QOS_DEAD,		/* probe finished without a devfreq that takes votes */
};

/*
 * One stock PM QoS class (spec L2b 5.1 rule 1). Statically initialised:
 * consumers may add requests from any initcall level, before the bus device
 * probes. df and state are written by the bus driver's probe under lock.
 */
struct e8895_qos_class {
	struct mutex lock;
	struct list_head reqs;
	s32 agg;		/* max of the active requests, kHz; read lock-free by the governor */
	struct devfreq *df;
	u8 state;
	u8 stage;		/* the probe's effective stage, for the "held" log */
	bool held_logged;
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

/* Per-domain constants (spec L2b 0.4, 5.3.1); every value from the ECT or the shipped DT. */
struct e8895_bus_desc {
	const char *name;
	const char *compatible;		/* for exynos_bus_qos_class_live() */
	enum exynos_bus_qos_class cls;
	u32 boot_khz;			/* ECT/ASV boot level */
	u32 boot_qos_khz;		/* stock boot_info[1], held boot_qos_s seconds */
	unsigned int boot_qos_s;	/* stock boot_info[0] */
	unsigned int delay_ms;		/* stock delay_time_list, 0 = use_delay_time "false" */
	unsigned int nr_opps;		/* the DT OPP table (spec L2b 4.1) */
	u32 min_opp_khz;
	u32 max_opp_khz;
	bool acpm;			/* stock use_acpm "true" + acpm-ipc-channel <1> */
	bool apm_notify;		/* the APM's ch5 MIF requests (stock acpm_dvfs) */
	int release_type;		/* stock devfreq_type of the RELEASE message, -1 = no protocol */
	u32 suspend_khz;		/* use_acpm domains: ASV resume level, pinned while suspended */
	u32 reboot_min_khz;		/* stock freq_info[5] */
	u32 reboot_max_khz;		/* MIF only: stock reboot hook also pins the maximum; 0 = none */
	bool data_init;			/* MIF: the DATA_INIT table at probe */
};

/*
 * MIF: boot = ASV boot idx 2 = 1794000, ASV-enabled L2..L10 = 1794000..286000
 * (ECT-ASV:467, ECT-DVFS:53-64); boot_info = <40 1794000> (FDT:6720);
 * use_delay_time "true", delay_time_list "20" (FDT:6717-6718); use_acpm (FDT:6729-6730).
 */
static const struct e8895_bus_desc e8895_bus_mif = {
	.name = "mif",
	.compatible = "samsung,exynos8895-bus-mif",
	.cls = EXYNOS_BUS_QOS_MIF,
	.boot_khz = 1794000,
	.boot_qos_khz = 1794000,
	.boot_qos_s = 40,
	.delay_ms = 20,
	.nr_opps = 9,
	.min_opp_khz = 286000,
	.max_opp_khz = 1794000,
	.acpm = true,
	.apm_notify = true,
	/* DEVFREQ_MIF (S:include/soc/samsung/exynos-devfreq.h:26) */
	.release_type = 0,
	/* ASV resume idx 9 (ECT-ASV:467) */
	.suspend_khz = 421000,
	/* freq_info[5] (FDT:6719); max too, S:drivers/devfreq/exynos/exynos8895_bus_mif.c:195-202 */
	.reboot_min_khz = 421000,
	.reboot_max_khz = 421000,
	.data_init = true,
};

/*
 * INT: boot = ASV boot idx 2 = 400000, all seven levels enabled (ECT-ASV:574,
 * ECT-DVFS:68-74); boot_info = <40 667000> (FDT:6740); use_delay_time "false"
 * (FDT:6738); use_acpm (FDT:6749-6750).
 */
static const struct e8895_bus_desc e8895_bus_int = {
	.name = "int",
	.compatible = "samsung,exynos8895-bus-int",
	.cls = EXYNOS_BUS_QOS_INT,
	.boot_khz = 400000,
	.boot_qos_khz = 667000,
	.boot_qos_s = 40,
	.nr_opps = 7,
	.min_opp_khz = 107000,
	.max_opp_khz = 667000,
	.acpm = true,
	/* DEVFREQ_INT (S:include/soc/samsung/exynos-devfreq.h:27) */
	.release_type = 1,
	/* ASV resume idx 6 (ECT-ASV:574) */
	.suspend_khz = 107000,
	/* freq_info[5] (FDT:6739) */
	.reboot_min_khz = 667000,
};

/*
 * INTCAM: boot = ASV boot idx 3 = 640000, all four levels (ECT-ASV:641,
 * ECT-DVFS:78-81); boot_info = <40 640000> (FDT:6760); no delay (FDT:6758).
 */
static const struct e8895_bus_desc e8895_bus_intcam = {
	.name = "intcam",
	.compatible = "samsung,exynos8895-bus-intcam",
	.cls = EXYNOS_BUS_QOS_INTCAM,
	.boot_khz = 640000,
	.boot_qos_khz = 640000,
	.boot_qos_s = 40,
	.nr_opps = 4,
	.min_opp_khz = 640000,
	.max_opp_khz = 690000,
	.release_type = -1,
	/* freq_info[5] (FDT:6759) */
	.reboot_min_khz = 690000,
};

/*
 * CAM: boot = ASV boot idx 6 = 630000, all seven levels (ECT-ASV:684,
 * ECT-DVFS:85-91); boot_info = <40 630000> (FDT:6796); no delay (FDT:6794).
 */
static const struct e8895_bus_desc e8895_bus_cam = {
	.name = "cam",
	.compatible = "samsung,exynos8895-bus-cam",
	.cls = EXYNOS_BUS_QOS_CAM,
	.boot_khz = 630000,
	.boot_qos_khz = 630000,
	.boot_qos_s = 40,
	.nr_opps = 7,
	.min_opp_khz = 630000,
	.max_opp_khz = 690000,
	.release_type = -1,
	/* freq_info[5] (FDT:6795) */
	.reboot_min_khz = 690000,
};

/*
 * DISP: boot = ASV boot idx 1 = 533000, ASV-enabled L1..L3 = 533000..214000
 * (ECT-ASV:751, ECT-DVFS:95-99); boot_info = <40 630000> (FDT:6778), above the
 * highest enabled OPP, so the devfreq core clamps it to 533000; no delay (FDT:6776).
 */
static const struct e8895_bus_desc e8895_bus_disp = {
	.name = "disp",
	.compatible = "samsung,exynos8895-bus-disp",
	.cls = EXYNOS_BUS_QOS_DISP,
	.boot_khz = 533000,
	.boot_qos_khz = 630000,
	.boot_qos_s = 40,
	.nr_opps = 3,
	.min_opp_khz = 214000,
	.max_opp_khz = 533000,
	.release_type = -1,
	/* freq_info[5] (FDT:6777), clamped to 533000 by the OPP maximum as stock's DM did */
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
	unsigned int stage;
	struct clk *clk;
	struct acpm_handle *acpm;
	struct devfreq *df;
	struct devfreq_dev_profile profile;
	struct devfreq_simple_interactive_data gov;

	/* the driver's own limits, dev_pm_qos requests on dev (kHz) */
	struct dev_pm_qos_request floor_req;
	struct dev_pm_qos_request ceil_req;
	struct dev_pm_qos_request boot_req;
	struct dev_pm_qos_request susp_min_req;	/* stage >= 3, MIF/INT: the suspend pin */
	struct dev_pm_qos_request susp_max_req;
	struct delayed_work boot_work;
	struct notifier_block reboot_nb;

	unsigned long cur_khz;		/* last requested level; 0 = unknown; written under df->lock */
	unsigned int nreq;		/* FREQ_REQs sent (heartbeat); written under df->lock */

	/* MIF: the APM's ch5 "MIF request" (stock mif_request_from_acpm) */
	struct exynos_bus_qos_request apm_req;
	struct notifier_block apm_nb;
	struct work_struct apm_work;
	atomic_t apm_khz;
	atomic_t apm_count;
};

/* Probed domains, for the heartbeat; set once, never cleared (no unbind). */
static struct e8895_bus *e8895_bus_devs[EXYNOS_BUS_QOS_NR];

/* ---- PM QoS classes (spec L2b 5.1) ------------------------------------------------------ */

/*
 * Recompute and publish the aggregate; when it changed and the class's bus
 * devfreq is live, re-evaluate it (rule 3). Called with c->lock held.
 */
static void e8895_qos_apply(struct e8895_qos_class *c, s32 req_khz)
{
	struct exynos_bus_qos_request *req;
	s32 agg = 0;	/* stock default value of the throughput classes */

	lockdep_assert_held(&c->lock);

	/* stock PM_QOS_MAX / PM_QOS_FORCE_MAX: the largest request (S:kernel/power/qos.c:397-399, no request = 0 :390-391) */
	list_for_each_entry(req, &c->reqs, node)
		agg = max(agg, req->khz);

	if (!c->df && req_khz && !c->held_logged) {
		c->held_logged = true;
		pr_info("exynos8895-bus: %s request %d kHz held (bus stage %u)\n",
			e8895_bus_descs[c - e8895_qos]->name, req_khz,
			c->state == E8895_QOS_UNKNOWN ? stage : c->stage);
	}

	if (agg == c->agg)
		return;

	WRITE_ONCE(c->agg, agg);

	if (c->df) {
		mutex_lock(&c->df->lock);
		update_devfreq(c->df);
		mutex_unlock(&c->df->lock);
	}
}

/* stock __pm_qos_update_request(): only a changed value updates the class */
static void e8895_qos_set(struct exynos_bus_qos_request *req, s32 khz)
{
	struct e8895_qos_class *c = &e8895_qos[req->cls];

	mutex_lock(&c->lock);
	if (req->khz != khz) {
		req->khz = khz;
		e8895_qos_apply(c, khz);
	}
	mutex_unlock(&c->lock);
}

/* stock pm_qos_work_fn(): the timeout falls back to the default value (S:kernel/power/qos.c:752-759) */
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

/* stock pm_qos_add_request_trace() (S:kernel/power/qos.c:774-792) */
void exynos_bus_qos_add_request(struct exynos_bus_qos_request *req,
				enum exynos_bus_qos_class cls, s32 khz)
{
	struct e8895_qos_class *c;

	might_sleep();

	if (!req || WARN_ON(cls >= EXYNOS_BUS_QOS_NR))
		return;

	if (exynos_bus_qos_request_active(req)) {
		WARN(1, "exynos_bus_qos_add_request() called for already added request\n");
		return;
	}

	c = &e8895_qos[cls];
	req->cls = cls;
	INIT_DELAYED_WORK(&req->timeout, e8895_qos_timeout);

	mutex_lock(&c->lock);
	req->khz = khz;
	list_add_tail(&req->node, &c->reqs);
	req->active = true;
	e8895_qos_apply(c, khz);
	mutex_unlock(&c->lock);
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_add_request);

/* stock pm_qos_update_request() (S:kernel/power/qos.c:805-820) */
void exynos_bus_qos_update_request(struct exynos_bus_qos_request *req, s32 khz)
{
	might_sleep();

	if (!req)
		return;

	if (!exynos_bus_qos_request_active(req)) {
		WARN(1, "exynos_bus_qos_update_request() called for unknown object\n");
		return;
	}

	/* before the class lock: the timeout work takes it */
	cancel_delayed_work_sync(&req->timeout);

	e8895_qos_set(req, khz);
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_update_request);

/* stock pm_qos_update_request_timeout() (S:kernel/power/qos.c:860-880) */
void exynos_bus_qos_update_request_timeout(struct exynos_bus_qos_request *req,
					   s32 khz, unsigned long timeout_us)
{
	might_sleep();

	if (!req)
		return;
	if (WARN(!exynos_bus_qos_request_active(req),
		 "%s called for unknown object.", __func__))
		return;

	cancel_delayed_work_sync(&req->timeout);

	e8895_qos_set(req, khz);

	schedule_delayed_work(&req->timeout, usecs_to_jiffies(timeout_us));
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_update_request_timeout);

/* stock pm_qos_remove_request() (S:kernel/power/qos.c:890-909) */
void exynos_bus_qos_remove_request(struct exynos_bus_qos_request *req)
{
	struct e8895_qos_class *c;

	might_sleep();

	if (!req)
		return;

	if (!exynos_bus_qos_request_active(req)) {
		WARN(1, "exynos_bus_qos_remove_request() called for unknown object\n");
		return;
	}

	cancel_delayed_work_sync(&req->timeout);

	c = &e8895_qos[req->cls];
	mutex_lock(&c->lock);
	list_del(&req->node);
	req->active = false;
	req->khz = 0;
	e8895_qos_apply(c, 0);
	mutex_unlock(&c->lock);
}
EXPORT_SYMBOL_GPL(exynos_bus_qos_remove_request);

/* spec L2b 5.1 rule 6 (port): lets a consumer that also drives the clock pick its path once */
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

/* ---- devfreq device (spec L2b 5.3.3, 5.3.4) ----------------------------------------------- */

/* Called by the devfreq core with df->lock held. */
static int e8895_bus_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct e8895_bus *priv = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long new_khz;
	int ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	new_khz = dev_pm_opp_get_freq(opp) / HZ_PER_KHZ;
	dev_pm_opp_put(opp);

	/*
	 * Stock returns when old == new (S:drivers/devfreq/exynos/exynos-devfreq.c:1613-1614).
	 * At stage >= 2 cur_khz starts at 0, so the first evaluation always sends.
	 */
	if (new_khz == priv->cur_khz)
		return 0;

	/* defensive: at stage 1 floor = ceiling = the boot level = cur_khz */
	if (priv->stage < 2) {
		dev_warn_once(dev, "stage %u: FREQ_REQ %lu kHz suppressed\n",
			      priv->stage, new_khz);
		*freq = priv->cur_khz * HZ_PER_KHZ;
		return 0;
	}

	/* FREQ_REQ {id, kHz, 0, 0} on ACPM channel 0 (WT:drivers/firmware/samsung/exynos-acpm-dvfs.c:41-61) */
	ret = clk_set_rate(priv->clk, new_khz * HZ_PER_KHZ);
	if (ret) {
		dev_err(dev, "FREQ_REQ %lu kHz failed: %d\n", new_khz, ret);
		return ret;
	}

	WRITE_ONCE(priv->cur_khz, new_khz);
	WRITE_ONCE(priv->nreq, priv->nreq + 1);
	*freq = new_khz * HZ_PER_KHZ;

	return 0;
}

/*
 * The last rate this driver requested, as stock's old_freq after its own
 * set_freq; the MIF level is not one register field (spec L2b 3.2 item 7, O-B2).
 */
static int e8895_bus_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct e8895_bus *priv = dev_get_drvdata(dev);

	*freq = READ_ONCE(priv->cur_khz) * HZ_PER_KHZ;

	return 0;
}

/* ---- probe helpers (spec L2b 5.3.2) ------------------------------------------------------ */

/* Step 2: the DRAM ECT must be the one the static tables were transcribed from. */
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
		dev_err(dev, "ect: no memory-region of at least %#x bytes: MISMATCH\n",
			E8895_ECT_TOTAL);
		return -ENODEV;
	}

	va = memremap(rmem->base, E8895_ECT_TOTAL, MEMREMAP_WB);
	if (!va) {
		dev_err(dev, "ect: cannot map %pa: MISMATCH\n", &rmem->base);
		return -ENOMEM;
	}

	total = get_unaligned_le32(va + 8);
	crc = ~crc32_le(~0, va, E8895_ECT_TOTAL);
	ok = !memcmp(va, "PARA", 4) && !memcmp(va + 4, "003j", 4) &&
	     total == E8895_ECT_TOTAL && crc == E8895_ECT_CRC;
	dev_info(dev, "ect crc 0x%08x ver %.4s total %#x: %s\n",
		 crc, (const char *)va + 4, total, ok ? "ok" : "MISMATCH");

	memunmap(va);

	return ok ? 0 : -EINVAL;
}

/* Step 4: the DT OPP table must be the one this driver's constants describe. */
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

/* Step 7: the driver's own floor/ceiling/boot limits, before the devfreq device exists. */
static int e8895_bus_add_qos(struct e8895_bus *priv, s32 floor_khz, s32 ceil_khz)
{
	struct device *dev = priv->dev;
	int ret;

	ret = dev_pm_qos_add_request(dev, &priv->floor_req, DEV_PM_QOS_MIN_FREQUENCY,
				     floor_khz);
	if (ret < 0)
		return ret;

	ret = dev_pm_qos_add_request(dev, &priv->ceil_req, DEV_PM_QOS_MAX_FREQUENCY,
				     ceil_khz);
	if (ret < 0)
		goto err;

	if (priv->stage >= 2) {
		ret = dev_pm_qos_add_request(dev, &priv->boot_req,
					     DEV_PM_QOS_MIN_FREQUENCY,
					     priv->desc->boot_qos_khz);
		if (ret < 0)
			goto err;
	}

	if (priv->stage >= 3 && priv->desc->release_type >= 0) {
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
 * Step 8 (stage >= 3, MIF): the stock DATA_INIT table (spec L2b 5.3.6 item 1;
 * S:drivers/devfreq/exynos/exynos8895_bus_mif.c:125-161): per ECT dvfs_mif
 * level inside the ASV range [286000, 1794000] kHz (the CAL min/max passed at
 * :308), {level, kHz, DATA_INIT, INT constraint of that level}, then {count,
 * initial kHz, DATA_INIT, SET_CONST}, initial = the CAL boot level (:305). A
 * send error fails the probe as stock (:142-146,156-160).
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
	dev_info(priv->dev, "DATA_INIT %u levels, init %u kHz: %d\n", level,
		 desc->boot_khz, ret);

	return ret ? -EINVAL : 0;
}

/*
 * Stage >= 3, MIF and INT (stock use_acpm): the APM sleep handshake and the
 * suspend level, stock .suspend_late/.resume_early
 * (S:drivers/devfreq/exynos/exynos-devfreq.c:1866-2016, pm ops :2426-2429).
 * Suspend: RELEASE running=0, then policy min = max = the suspend level (stock
 * str_freq, S:drivers/devfreq/devfreq.c:267-269). Resume: RELEASE running=1,
 * then the pin is released (stock :1988-2010). Stated deviation: a RELEASE send
 * error is logged, not returned (stock :1909-1914 aborts the suspend).
 */
static bool e8895_bus_has_protocol(struct e8895_bus *priv)
{
	return priv && priv->df && priv->stage >= 3 && priv->desc->release_type >= 0;
}

static void e8895_bus_release(struct e8895_bus *priv, bool running)
{
	int ret;

	/* under df->lock like stock (:1885-1914): no FREQ_REQ in between */
	mutex_lock(&priv->df->lock);
	ret = priv->acpm->ops.dvfs_ops.sleep_release(priv->acpm, E8895_ACPM_DEVFREQ_CHAN,
						     priv->desc->release_type, running);
	mutex_unlock(&priv->df->lock);
	dev_info(priv->dev, "release running=%d: %d\n", running, ret);
}

static int e8895_bus_suspend_late(struct device *dev)
{
	struct e8895_bus *priv = dev_get_drvdata(dev);
	s32 khz;

	if (!e8895_bus_has_protocol(priv))
		return 0;

	e8895_bus_release(priv, false);
	khz = priv->desc->suspend_khz;
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
	dev_pm_qos_update_request(&priv->susp_max_req, PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE);

	return 0;
}

/* explicit members only: the _PM_OPS helpers would also fill freeze/thaw/poweroff/restore */
static const struct dev_pm_ops e8895_bus_pm_ops = {
	.suspend_late = e8895_bus_suspend_late,
	.resume_early = e8895_bus_resume_early,
};

/*
 * Stage >= 3: stock reboot notifier (S:drivers/devfreq/exynos/exynos-devfreq.c:1490-1515):
 * floor := freq_info[5]; MIF also caps its maximum there
 * (S:drivers/devfreq/exynos/exynos8895_bus_mif.c:195-202), the others raise their
 * maximum to it (exynos8895_bus_int.c:40-50, exynos8895_bus_disp.c:36-46), which
 * the floor already does here.
 */
static int e8895_bus_reboot(struct notifier_block *nb, unsigned long action, void *data)
{
	struct e8895_bus *priv = container_of(nb, struct e8895_bus, reboot_nb);
	const struct e8895_bus_desc *desc = priv->desc;

	if (desc->reboot_max_khz)
		dev_pm_qos_update_request(&priv->ceil_req, desc->reboot_max_khz);
	dev_pm_qos_update_request(&priv->floor_req, desc->reboot_min_khz);
	dev_info(priv->dev, "reboot pin %d kHz\n", desc->reboot_min_khz);

	return NOTIFY_OK;
}

static void e8895_bus_unregister_reboot(void *data)
{
	struct e8895_bus *priv = data;

	unregister_reboot_notifier(&priv->reboot_nb);
}

/*
 * Step 10: end of the stock 40 s boot QoS (pm_qos_update_request_timeout() of
 * boot_pm_qos, S:drivers/devfreq/exynos/exynos-devfreq.c:2260-2262).
 */
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

/*
 * The APM's MIF request (stock acpm_noti_mif_callback(), CAL:acpm_dvfs.c:151-155,
 * a PM_QOS_BUS_THROUGHPUT request added at probe with 0, :264). The ACPM
 * notifier runs with the channel's rx_lock held
 * (WT:drivers/firmware/samsung/exynos-acpm.c:620-635) and the update ends in a
 * FREQ_REQ, so it is deferred to a work item.
 */
static void e8895_bus_apm_work(struct work_struct *work)
{
	struct e8895_bus *priv = container_of(work, struct e8895_bus, apm_work);
	u32 khz = atomic_read(&priv->apm_khz);
	unsigned long cur;
	int ret = 0;

	exynos_bus_qos_update_request(&priv->apm_req, khz);

	if (!ch5_resend) {
		dev_info_ratelimited(priv->dev, "APM MIF request %u kHz\n", khz);
		return;
	}

	/*
	 * The same FREQ_REQ the clock sends, past the clk core, which drops a set
	 * to the cached rate (A:drivers/clk/clk.c:2605-2607); the cached rate stays
	 * right. Under df->lock: serialised with the target.
	 */
	mutex_lock(&priv->df->lock);
	cur = priv->cur_khz;
	if (cur)
		ret = priv->acpm->ops.dvfs_ops.set_rate(priv->acpm, E8895_ACPM_DVFS_CHAN,
							EXYNOS8895_CLK_ACPM_DVFS_MIF,
							cur * HZ_PER_KHZ);
	mutex_unlock(&priv->df->lock);

	dev_info_ratelimited(priv->dev, "APM MIF request %u kHz, resent %lu kHz: %d\n",
			     khz, cur, ret);
}

static int e8895_bus_apm_notify(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct e8895_bus *priv = container_of(nb, struct e8895_bus, apm_nb);
	const u32 *cmd = data;

	if (action != E8895_ACPM_NOTIFY_CHAN)
		return NOTIFY_DONE;

	/* cmd[1] = kHz (CAL:acpm_dvfs.c:153) */
	atomic_set(&priv->apm_khz, cmd[1]);
	atomic_inc(&priv->apm_count);
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

/*
 * Spec L2b 5.3.6 item 3: request added, work initialised, its cancel action
 * registered, then the notifier (teardown unregisters it first).
 */
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

/* Record the probe result for exynos_bus_qos_class_live() (rule 6). */
static void e8895_bus_set_state(struct e8895_bus *priv, struct e8895_qos_class *c,
				u8 state)
{
	mutex_lock(&c->lock);
	c->stage = priv ? priv->stage : 0;
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
	s32 floor_khz, ceil_khz;
	int ret;

	/* Step 1 */
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		e8895_bus_set_state(NULL, c, E8895_QOS_DEAD);
		return -ENOMEM;
	}
	priv->dev = dev;
	priv->desc = desc;
	priv->stage = min_t(unsigned int, stage, E8895_BUS_STAGE_MAX);
	platform_set_drvdata(pdev, priv);

	/* Step 2 (every stage) */
	if (e8895_bus_ect_check(dev))
		priv->stage = 0;

	/* Step 3 */
	if (priv->stage == 0) {
		dev_info(dev, "stage 0: no devfreq\n");
		e8895_bus_set_state(priv, c, E8895_QOS_DEAD);
		smp_store_release(&e8895_bus_devs[desc->cls], priv);
		return 0;
	}

	/* Step 4 */
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

	/* Step 5: MIF and INT, the stock use_acpm domains */
	if (desc->acpm) {
		acpm_np = of_parse_phandle(dev->of_node, "clocks", 0);
		priv->acpm = devm_acpm_get_by_node(dev, acpm_np);
		of_node_put(acpm_np);
		if (IS_ERR(priv->acpm)) {
			ret = dev_err_probe(dev, PTR_ERR(priv->acpm), "no ACPM handle\n");
			goto err;
		}
	}

	/*
	 * Step 6 (spec L2b V-1): at stage >= 2 the level is "unknown", so the first
	 * evaluation sends the boot-level FREQ_REQ (the AP's vote). Stated deviation:
	 * stock sent no FREQ_REQ at probe (old_freq = initial_freq,
	 * S:drivers/devfreq/exynos/exynos-devfreq.c:2095) and announced the level
	 * with DATA_INIT SET_CONST instead (S:drivers/devfreq/exynos/exynos8895_bus_mif.c:150-161, M2).
	 */
	priv->cur_khz = priv->stage >= 2 ? 0 : desc->boot_khz;

	/* Step 7: floor = boot level (stages 1-2); ceiling = boot level (1) or none (2) */
	floor_khz = desc->boot_khz;
	ceil_khz = priv->stage == 1 ? desc->boot_khz : PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE;
	ret = e8895_bus_add_qos(priv, floor_khz, ceil_khz);
	if (ret) {
		dev_err(dev, "dev_pm_qos requests: %d\n", ret);
		goto err;
	}

	/* Step 8: before the devfreq device, as stock (exynos-devfreq.c:2075,2154) */
	if (priv->stage >= 3 && desc->data_init) {
		ret = e8895_bus_data_init(priv);
		if (ret)
			goto err;
	}

	/* Step 9 */
	priv->gov.delay_ms = desc->delay_ms;
	if (priv->stage >= 2)
		priv->gov.qos_khz = &c->agg;
	priv->profile.initial_freq = (unsigned long)desc->boot_khz * HZ_PER_KHZ;
	priv->profile.target = e8895_bus_target;
	priv->profile.get_cur_freq = e8895_bus_get_cur_freq;
	priv->profile.polling_ms = 0;

	/* DEVFREQ_GOV_START runs the first evaluation (the stage-2 vote) in here */
	priv->df = devm_devfreq_add_device(dev, &priv->profile,
					   DEVFREQ_GOV_SIMPLE_INTERACTIVE, &priv->gov);
	if (IS_ERR(priv->df)) {
		ret = dev_err_probe(dev, PTR_ERR(priv->df), "devfreq\n");
		goto err;
	}

	if (priv->stage >= 2) {
		mutex_lock(&c->lock);
		c->stage = priv->stage;
		c->df = priv->df;
		c->state = E8895_QOS_LIVE;
		/* a request that arrived between GOV_START and c->df */
		mutex_lock(&priv->df->lock);
		update_devfreq(priv->df);
		mutex_unlock(&priv->df->lock);
		mutex_unlock(&c->lock);

		/* Step 10 */
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
	} else {
		e8895_bus_set_state(priv, c, E8895_QOS_DEAD);
	}

	/* Step 11 */
	if (priv->stage >= 3) {
		priv->reboot_nb.notifier_call = e8895_bus_reboot;
		ret = register_reboot_notifier(&priv->reboot_nb);
		if (!ret)
			ret = devm_add_action_or_reset(dev, e8895_bus_unregister_reboot, priv);
		if (ret)
			goto err;
	}

	/* Step 12; the ceiling printed is the effective one */
	dev_info(dev, "stage %u: %d OPPs %u..%u kHz, boot %u kHz, floor %d kHz, ceiling %d kHz\n",
		 priv->stage, dev_pm_opp_get_opp_count(dev), desc->min_opp_khz,
		 desc->max_opp_khz, desc->boot_khz, floor_khz,
		 min_t(s32, ceil_khz, desc->max_opp_khz));

	smp_store_release(&e8895_bus_devs[desc->cls], priv);

	return 0;

err:
	/* every exit but -EPROBE_DEFER leaves the class dead (rule 6); a live df is dropped first */
	if (ret != -EPROBE_DEFER)
		e8895_bus_set_state(priv, c, E8895_QOS_DEAD);
	else
		e8895_bus_set_state(priv, c, E8895_QOS_UNKNOWN);
	return ret;
}

#if IS_ENABLED(CONFIG_GREATLTE_HB)
/*
 * DEBUG: heartbeat "bus" row (spec L2b 1.2, 5.3.9), lock-free:
 * s<stage>m<kHz>/<class kHz>q<n>i<kHz>/<class kHz>q<n>ic<kHz>c<kHz>d<kHz>q<n>n<ch5 count>
 */
int exynos8895_bus_hb(char *buf, size_t size)
{
	unsigned long khz[EXYNOS_BUS_QOS_NR];
	unsigned int nreq[EXYNOS_BUS_QOS_NR];
	struct e8895_bus *p;
	unsigned int n = 0;
	int i;

	for (i = 0; i < EXYNOS_BUS_QOS_NR; i++) {
		p = smp_load_acquire(&e8895_bus_devs[i]);
		khz[i] = p ? READ_ONCE(p->cur_khz) : 0;
		nreq[i] = p ? READ_ONCE(p->nreq) : 0;
		if (p && i == EXYNOS_BUS_QOS_MIF)
			n = atomic_read(&p->apm_count);
	}

	return scnprintf(buf, size, "s%um%lu/%dq%ui%lu/%dq%uic%luc%lud%luq%un%u",
			 stage,
			 khz[EXYNOS_BUS_QOS_MIF], READ_ONCE(e8895_qos[EXYNOS_BUS_QOS_MIF].agg),
			 nreq[EXYNOS_BUS_QOS_MIF],
			 khz[EXYNOS_BUS_QOS_INT], READ_ONCE(e8895_qos[EXYNOS_BUS_QOS_INT].agg),
			 nreq[EXYNOS_BUS_QOS_INT],
			 khz[EXYNOS_BUS_QOS_INTCAM], khz[EXYNOS_BUS_QOS_CAM],
			 khz[EXYNOS_BUS_QOS_DISP], nreq[EXYNOS_BUS_QOS_DISP], n);
}
#endif

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
		/*
		 * Built in, never unbound: consumers hold the class registry's df
		 * pointer and the heartbeat reads the probed devices lock-free.
		 */
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(e8895_bus_driver);
