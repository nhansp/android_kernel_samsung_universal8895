// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 multimedia QoS shim (rewrite/spec/L6B_multimedia.md 5.3).
 *
 * The stock 4.4 camera (FIMC-IS2) and MFC drivers vote on Samsung pm_qos
 * classes (S:include/linux/pm_qos.h:13-45); 6.18 has only cpu_latency_qos and
 * freq_qos. This keeps one request list per class, aggregates it like stock
 * pm_qos (S:kernel/power/qos.c: a MIN class = largest active value, a MAX
 * class = smallest) and applies the result:
 * - INTCAM/CAM: clk_set_rate() on the ACPM DVFS clocks the camera core node
 *   names "dvfs_intcam"/"dvfs_cam", clamped to the stock devfreq range; with
 *   no active request the stock default_qos level (FDT:6759,6795 freq_info =
 *   {initial, default_qos, suspend, min, max, reboot},
 *   S:drivers/devfreq/exynos/exynos-devfreq.c:903-908);
 * - CLUSTER0/1 MIN/MAX: freq_qos requests on the cpufreq policy of cpu 0
 *   (cpus 0-3) and cpu 4 (cpus 4-7) (FDT:4484-4489 domain@0 "0-3"
 *   pm_qos-min-class 3/max 4, FDT:4513-4518 domain@1 "4-7" class 5/6);
 * - INT/MIF/CPU_ONLINE_MIN: aggregated, not applied (no L2 owner of MIF/INT,
 *   L2_power.md Q8; spec L6B 5.3, open item O-MM-MIF).
 * Callers sleep-capable, as stock pm_qos (blocking notifiers).
 */

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/seq_file.h>
#include <soc/samsung/exynos8895-mm-qos.h>

/* stock PM_QOS_DEFAULT_VALUE: the request takes the class default (S:include/linux/pm_qos.h:53) */
#define MM_QOS_DEFAULT_VALUE	(-1)

struct mm_qos_class_desc {
	const char *name;
	bool is_max;		/* MAX class: smallest active value wins */
	s32 default_value;	/* stock <CLASS>_DEFAULT_VALUE, S:include/linux/pm_qos.h:57-84 */
};

static const struct mm_qos_class_desc mm_qos_desc[MM_QOS_NR] = {
	[MM_QOS_INTCAM]		= { "intcam", false, 0 },	/* :58 */
	[MM_QOS_CAM]		= { "cam", false, 0 },		/* :69 */
	[MM_QOS_CL0_MIN]	= { "cl0_min", false, 0 },	/* :83 */
	[MM_QOS_CL0_MAX]	= { "cl0_max", true, INT_MAX },	/* :84 */
	[MM_QOS_CL1_MIN]	= { "cl1_min", false, 0 },	/* :81 */
	[MM_QOS_CL1_MAX]	= { "cl1_max", true, INT_MAX },	/* :82 */
	[MM_QOS_INT]		= { "int", false, 0 },		/* :57 */
	[MM_QOS_MIF]		= { "mif", false, 0 },		/* :63 */
	[MM_QOS_CPU_ONLINE_MIN]	= { "cpu_online_min", false, 1 },	/* :79 */
};

/*
 * INTCAM/CAM levels in kHz: FDT:6759 devfreq_intcam freq_info = <0xa8750
 * 0x9c400 0xa8750 0x9c400 0xa8750 0xa8750>, FDT:6795 devfreq_cam freq_info =
 * <0xa8750 0x99cf0 0xa8750 0x99cf0 0xa8750 0xa8750>.
 */
struct mm_qos_dvfs {
	const char *clk_name;
	u32 default_khz;	/* freq_info[1] default_qos */
	u32 min_khz;		/* freq_info[3] */
	u32 max_khz;		/* freq_info[4] */
	struct clk *clk;
	unsigned long applied_khz;
};

static struct mm_qos_dvfs mm_qos_dvfs[2] = {
	{ "dvfs_intcam", 640000, 640000, 690000 },	/* MM_QOS_INTCAM */
	{ "dvfs_cam", 630000, 630000, 690000 },		/* MM_QOS_CAM */
};

/* cluster n <-> the cpufreq policy of cpu 0 (cpus 0-3) or cpu 4 (cpus 4-7) */
static const unsigned int mm_qos_cluster_cpu[2] = { 0, 4 };

struct mm_qos_cpu {
	enum mm_qos_class cls;
	unsigned int cluster;
	enum freq_qos_req_type type;
	struct freq_qos_request req;
};

static struct mm_qos_cpu mm_qos_cpu[4] = {
	{ MM_QOS_CL0_MIN, 0, FREQ_QOS_MIN },
	{ MM_QOS_CL0_MAX, 0, FREQ_QOS_MAX },
	{ MM_QOS_CL1_MIN, 1, FREQ_QOS_MIN },
	{ MM_QOS_CL1_MAX, 1, FREQ_QOS_MAX },
};

static DEFINE_MUTEX(mm_qos_lock);
#define MM_QOS_LIST(c)	[c] = LIST_HEAD_INIT(mm_qos_list[c])
static struct list_head mm_qos_list[MM_QOS_NR] = {
	MM_QOS_LIST(MM_QOS_INTCAM), MM_QOS_LIST(MM_QOS_CAM),
	MM_QOS_LIST(MM_QOS_CL0_MIN), MM_QOS_LIST(MM_QOS_CL0_MAX),
	MM_QOS_LIST(MM_QOS_CL1_MIN), MM_QOS_LIST(MM_QOS_CL1_MAX),
	MM_QOS_LIST(MM_QOS_INT), MM_QOS_LIST(MM_QOS_MIF),
	MM_QOS_LIST(MM_QOS_CPU_ONLINE_MIN),
};
static bool mm_qos_ready;

static bool mm_qos_has_requests(enum mm_qos_class cls)
{
	return !list_empty(&mm_qos_list[cls]);
}

static s32 mm_qos_aggregate(enum mm_qos_class cls)
{
	const struct mm_qos_class_desc *d = &mm_qos_desc[cls];
	struct mm_qos_request *r;
	s32 v = d->default_value;
	bool first = true;

	list_for_each_entry(r, &mm_qos_list[cls], node) {
		if (first || (d->is_max ? r->value < v : r->value > v))
			v = r->value;
		first = false;
	}

	return v;
}

static void mm_qos_apply_dvfs(enum mm_qos_class cls)
{
	struct mm_qos_dvfs *d = &mm_qos_dvfs[cls - MM_QOS_INTCAM];
	u32 khz;
	int ret;

	if (!d->clk)
		return;

	if (mm_qos_has_requests(cls))
		khz = clamp_t(s32, mm_qos_aggregate(cls), d->min_khz, d->max_khz);
	else
		khz = d->default_khz;

	if (khz == d->applied_khz)
		return;

	ret = clk_set_rate(d->clk, (unsigned long)khz * 1000);
	if (ret) {
		pr_err("mm-qos: %s %u kHz: %d\n", mm_qos_desc[cls].name, khz, ret);
		return;
	}
	d->applied_khz = khz;
}

static struct mm_qos_cpu *mm_qos_cpu_of(enum mm_qos_class cls)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mm_qos_cpu); i++)
		if (mm_qos_cpu[i].cls == cls)
			return &mm_qos_cpu[i];
	return NULL;
}

static s32 mm_qos_cpu_value(struct mm_qos_cpu *c)
{
	if (!mm_qos_has_requests(c->cls))
		return c->type == FREQ_QOS_MIN ? FREQ_QOS_MIN_DEFAULT_VALUE :
						 FREQ_QOS_MAX_DEFAULT_VALUE;
	return mm_qos_aggregate(c->cls);
}

/* add the freq_qos request to @policy if it covers the class's cluster */
static void mm_qos_cpu_attach(struct mm_qos_cpu *c, struct cpufreq_policy *policy)
{
	int ret;

	if (freq_qos_request_active(&c->req) ||
	    !cpumask_test_cpu(mm_qos_cluster_cpu[c->cluster], policy->related_cpus))
		return;

	ret = freq_qos_add_request(&policy->constraints, &c->req, c->type,
				   mm_qos_cpu_value(c));
	if (ret < 0)
		pr_err("mm-qos: %s: %d\n", mm_qos_desc[c->cls].name, ret);
}

static void mm_qos_apply_cpu(enum mm_qos_class cls)
{
	struct mm_qos_cpu *c = mm_qos_cpu_of(cls);
	struct cpufreq_policy *policy;

	if (freq_qos_request_active(&c->req)) {
		freq_qos_update_request(&c->req, mm_qos_cpu_value(c));
		return;
	}

	/* the policy may exist already (cpufreq probed before this request) */
	policy = cpufreq_cpu_get(mm_qos_cluster_cpu[c->cluster]);
	if (policy) {
		mm_qos_cpu_attach(c, policy);
		cpufreq_cpu_put(policy);
	}
}

static void mm_qos_apply(enum mm_qos_class cls)
{
	lockdep_assert_held(&mm_qos_lock);

	switch (cls) {
	case MM_QOS_INTCAM:
	case MM_QOS_CAM:
		mm_qos_apply_dvfs(cls);
		break;
	case MM_QOS_CL0_MIN:
	case MM_QOS_CL0_MAX:
	case MM_QOS_CL1_MIN:
	case MM_QOS_CL1_MAX:
		if (mm_qos_ready)
			mm_qos_apply_cpu(cls);
		break;
	case MM_QOS_INT:
	case MM_QOS_MIF:
		if (mm_qos_has_requests(cls) && mm_qos_aggregate(cls) > 0)
			pr_info_once("mm-qos: INT/MIF vote %d kHz not applied (no L2 owner)\n",
				     mm_qos_aggregate(cls));
		break;
	default:	/* CPU_ONLINE_MIN: aggregated only */
		break;
	}
}

static s32 mm_qos_value(enum mm_qos_class cls, s32 value)
{
	return value == MM_QOS_DEFAULT_VALUE ? mm_qos_desc[cls].default_value : value;
}

/* pm_qos_add_request() (S:kernel/power/qos.c:774-795) */
void mm_qos_add_request(struct mm_qos_request *r, enum mm_qos_class cls, s32 value)
{
	if (!r || WARN_ON(cls >= MM_QOS_NR))
		return;

	mutex_lock(&mm_qos_lock);
	if (r->active) {
		WARN(1, "mm_qos_add_request() called for already added request\n");
		goto out;
	}
	r->cls = cls;
	r->value = mm_qos_value(cls, value);
	r->active = true;
	list_add_tail(&r->node, &mm_qos_list[cls]);
	mm_qos_apply(cls);
out:
	mutex_unlock(&mm_qos_lock);
}
EXPORT_SYMBOL_GPL(mm_qos_add_request);

/* pm_qos_update_request() (S:kernel/power/qos.c:805-819) */
void mm_qos_update_request(struct mm_qos_request *r, s32 value)
{
	if (!r)
		return;

	mutex_lock(&mm_qos_lock);
	if (!r->active) {
		WARN(1, "mm_qos_update_request() called for unknown object\n");
		goto out;
	}
	value = mm_qos_value(r->cls, value);
	if (value != r->value) {
		r->value = value;
		mm_qos_apply(r->cls);
	}
out:
	mutex_unlock(&mm_qos_lock);
}
EXPORT_SYMBOL_GPL(mm_qos_update_request);

/* pm_qos_remove_request() (S:kernel/power/qos.c:890-905) */
void mm_qos_remove_request(struct mm_qos_request *r)
{
	if (!r)
		return;

	mutex_lock(&mm_qos_lock);
	if (!r->active) {
		WARN(1, "mm_qos_remove_request() called for unknown object\n");
		goto out;
	}
	list_del(&r->node);
	r->active = false;
	mm_qos_apply(r->cls);
out:
	mutex_unlock(&mm_qos_lock);
}
EXPORT_SYMBOL_GPL(mm_qos_remove_request);

/* pm_qos_request_active() (S:kernel/power/qos.c:729-732) */
int mm_qos_request_active(struct mm_qos_request *r)
{
	return r->active;
}
EXPORT_SYMBOL_GPL(mm_qos_request_active);

static void mm_qos_put_dvfs(void *data)
{
	unsigned int i;

	mutex_lock(&mm_qos_lock);
	for (i = 0; i < ARRAY_SIZE(mm_qos_dvfs); i++) {
		mm_qos_dvfs[i].clk = NULL;
		mm_qos_dvfs[i].applied_khz = 0;
	}
	mutex_unlock(&mm_qos_lock);
}

/*
 * Called by the camera core probe with its device: takes the two ACPM DVFS
 * clocks and applies the current INTCAM/CAM aggregate (the idle level when no
 * request is active). The clocks are released with the device (the pointers
 * are cleared first, the devm action runs before the clk puts).
 */
int mm_qos_init(struct device *dev)
{
	struct clk *clk[ARRAY_SIZE(mm_qos_dvfs)];
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mm_qos_dvfs); i++) {
		clk[i] = devm_clk_get(dev, mm_qos_dvfs[i].clk_name);
		if (IS_ERR(clk[i]))
			return dev_err_probe(dev, PTR_ERR(clk[i]), "mm-qos: %s\n",
					     mm_qos_dvfs[i].clk_name);
	}

	mutex_lock(&mm_qos_lock);
	if (mm_qos_dvfs[0].clk) {
		mutex_unlock(&mm_qos_lock);
		return -EBUSY;
	}
	for (i = 0; i < ARRAY_SIZE(mm_qos_dvfs); i++)
		mm_qos_dvfs[i].clk = clk[i];
	mutex_unlock(&mm_qos_lock);

	ret = devm_add_action_or_reset(dev, mm_qos_put_dvfs, NULL);
	if (ret)
		return ret;

	mutex_lock(&mm_qos_lock);
	mm_qos_apply(MM_QOS_INTCAM);
	mm_qos_apply(MM_QOS_CAM);
	mutex_unlock(&mm_qos_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(mm_qos_init);

/* requests made before a policy exists are attached when it is created */
static int mm_qos_policy_notifier(struct notifier_block *nb, unsigned long event,
				  void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int i;

	mutex_lock(&mm_qos_lock);
	for (i = 0; i < ARRAY_SIZE(mm_qos_cpu); i++) {
		struct mm_qos_cpu *c = &mm_qos_cpu[i];

		if (event == CPUFREQ_CREATE_POLICY)
			mm_qos_cpu_attach(c, policy);
		else if (event == CPUFREQ_REMOVE_POLICY &&
			 freq_qos_request_active(&c->req) &&
			 c->req.qos == &policy->constraints)
			freq_qos_remove_request(&c->req);
	}
	mutex_unlock(&mm_qos_lock);

	return NOTIFY_OK;
}

static struct notifier_block mm_qos_policy_nb = {
	.notifier_call = mm_qos_policy_notifier,
};

static int mm_qos_show(struct seq_file *s, void *unused)
{
	unsigned int i;

	mutex_lock(&mm_qos_lock);
	for (i = 0; i < MM_QOS_NR; i++) {
		struct mm_qos_request *r;
		unsigned int n = 0;

		list_for_each_entry(r, &mm_qos_list[i], node)
			n++;
		seq_printf(s, "%-15s requests %u aggregate %d", mm_qos_desc[i].name, n,
			   mm_qos_aggregate(i));
		if (i == MM_QOS_INTCAM || i == MM_QOS_CAM)
			seq_printf(s, " applied %lu kHz\n",
				   mm_qos_dvfs[i - MM_QOS_INTCAM].applied_khz);
		else if (mm_qos_cpu_of(i))
			seq_printf(s, " freq_qos %s\n",
				   freq_qos_request_active(&mm_qos_cpu_of(i)->req) ?
				   "attached" : "no policy");
		else
			seq_puts(s, " not applied\n");
	}
	mutex_unlock(&mm_qos_lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mm_qos);

static int __init mm_qos_setup(void)
{
	unsigned int i;
	int ret;

	ret = cpufreq_register_notifier(&mm_qos_policy_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		pr_err("mm-qos: cpufreq notifier: %d\n", ret);

	mutex_lock(&mm_qos_lock);
	mm_qos_ready = true;
	for (i = 0; i < ARRAY_SIZE(mm_qos_cpu); i++)
		mm_qos_apply_cpu(mm_qos_cpu[i].cls);
	mutex_unlock(&mm_qos_lock);

	debugfs_create_file("mm-qos", 0444, NULL, NULL, &mm_qos_fops);

	return 0;
}
core_initcall(mm_qos_setup);
