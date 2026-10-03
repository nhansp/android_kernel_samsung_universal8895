// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos 8895 (Mali-G71) kbase platform sysfs: the port of the stock Samsung
 * platform sysfs that the LineageOS power HAL writes (brief l6agpu; spec
 * L6A_graphics.md 5.1.6 "Samsung sysfs not ported" and 7.4 "/sys/kernel/gpu").
 * K: = the stock drivers/gpu/arm/b_r16p0, S: = the stock 4.4.111 tree.
 *
 * What the power HAL needs:
 *
 *   /sys/devices/platform/13900000.mali/dvfs_table		read
 *   /sys/devices/platform/13900000.mali/dvfs_min_lock		read/write
 *   /sys/devices/platform/13900000.mali/dvfs_max_lock		read/write
 *   /sys/kernel/gpu/gpu_min_clock				read/write
 *   /sys/kernel/gpu/gpu_max_clock				read/write
 *
 * The first three are the stock platform-device files
 * (K:platform/exynos/gpu_custom_interface.c:1322-1346 DEVICE_ATTRs,
 * :1874-1944 device_create_file() list) and the last two the stock kernel
 * kobject (K:platform/exynos/gpu_custom_interface.c:1814-1815,1817-1819
 * gpu_max_lock_attribute / gpu_min_lock_attribute under
 * kobject_create_and_add("gpu", kernel_kobj), :2032). The stock Pie userspace
 * touched exactly these: /sys/kernel/gpu/gpu_{min,max}_clock with the kHz
 * values 546000/455000/385000/338000/260000
 * (recon/trees/legacy8895/android_device_samsung_universal8895-common/
 * configs/power/powerhint.json:101-121) and the three platform files chowned
 * radio:system (same tree, ramdisk/etc/init.samsungexynos8895.rc:310-315).
 *
 * The device directory therefore has to be /sys/devices/platform/13900000.mali/,
 * so the DT node is named mali@13900000 exactly as the stock FDT does (FDT:4636).
 *
 * Semantics. Stock's own DVFS is not ported; kbase devfreq is (spec L6A 5.1.4),
 * so a lock is the devfreq min/max window instead of stock's min_lock/max_lock:
 *
 *   - stock kept one clock per lock owner in user_min_lock[] / user_max_lock[]
 *     and took MAX / MIN over them (K:platform/exynos/gpu_dvfs_api.c:283-292, :312-321,
 *     gpu_dvfs_clock_lock()), and gpu_check_target_clock() clamped every rate
 *     the governor asked for between them, the minimum first and then the
 *     maximum, so the maximum wins when the two cross
 *     (K:platform/exynos/gpu_dvfs_api.c:47-52);
 *   - here the two locks are two PM QoS requests on the GPU device.
 *     devfreq_get_freq_range() folds them into the [min,max] window of every
 *     devfreq update (A:drivers/devfreq/devfreq.c:145-156) and clamps the
 *     minimum to the maximum (A:drivers/devfreq/devfreq.c:155-156), so the maximum
 *     wins as well, and the QoS notifier re-runs that update at once
 *     (A:drivers/devfreq/devfreq.c:699-736) - stock's "raise the rate now if the
 *     current rate is outside the new lock"
 *     (K:platform/exynos/gpu_dvfs_api.c:297-298,326-328). PM QoS aggregates the
 *     maximum-frequency requests with MIN and the minimum-frequency requests
 *     with MAX (A:kernel/power/qos.c:444-465 freq_constraints_init()), the same
 *     as stock's two arrays;
 *   - a rate is sent only while pd_g3d is on, so a lock taken while the GPU is
 *     runtime suspended is applied at the next resume, as stock did
 *     (K:platform/exynos/gpu_control.c:127-131, gpu_notifier.c:249-275).
 *
 * Every clock here is in kHz, the unit of the stock DVFS table
 * (K:platform/exynos/gpu_dvfs_api.c:562-573 gpu_dvfs_get_clock(); FDT:4648
 * gpu_dvfs_table column 0) and of the powerhint.json values above.
 */

#include <linux/devfreq.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/math64.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#include "mali_kbase_config_platform.h"
#include "mali_kbase_exynos8895.h"
#include "exynos8895_gpu.h"

/* The one kbase device, as stock's "static struct kobject *external_kobj"
 * (K:platform/exynos/gpu_custom_interface.c:1865) */
static struct exynos8895_gpu *exynos8895_gpu_kobj;

#define HZ_PER_KHZ_UL	1000UL

/*
 * Stock GPU_LOG() with DVFS_ERROR/DVFS_WARNING is the platform's trace print;
 * use dev_* like the rest of the port (spec L6A 5.1.5).
 */
#define gpu_err(gpu, fmt, ...) dev_err((gpu)->kbdev->dev, fmt, ##__VA_ARGS__)

/* The device attributes live on kbdev->dev, whose drvdata is the kbase_device
 * (K:mali_kbase_core_linux.c:3855-3856). kbdev->platform_context is set before
 * the files are created and cleared after they are removed, so it is never
 * NULL while an attribute can be called. */
static struct exynos8895_gpu *exynos8895_gpu_of_dev(struct device *dev)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	return kbdev ? (struct exynos8895_gpu *)kbdev->platform_context : NULL;
}

/*
 * The DVFS table in kHz, ascending: the OPP table of the DT (FDT:4648
 * gpu_dvfs_table column 0, restricted to [gpu_min_clock FDT:4655 0x3f7a0,
 * gpu_max_clock FDT:4653 0x854d0] by spec L6A 4.1) is this driver's table,
 * and kbase fills the devfreq freq_table with the same levels
 * (K:backend/gpu/mali_kbase_devfreq.c:196-243).
 */
static int exynos8895_gpu_read_levels(struct exynos8895_gpu *gpu)
{
	struct device *dev = gpu->kbdev->dev;
	struct dev_pm_opp *opp;
	unsigned long hz;
	int count, i;

	count = dev_pm_opp_get_opp_count(dev);
	if (count < 1)
		return count < 0 ? count : -ENODEV;

	gpu->level_khz = devm_kcalloc(dev, count, sizeof(*gpu->level_khz),
				      GFP_KERNEL);
	if (!gpu->level_khz)
		return -ENOMEM;

	/* ascending: dev_pm_opp_find_freq_ceil() sets the target to the level it
	 * found, so stepping past it walks the table upwards
	 * (A:drivers/devfreq/devfreq.c:86-98 find_available_min_freq()) */
	hz = 0;
	for (i = 0; i < count; i++) {
		opp = dev_pm_opp_find_freq_ceil(dev, &hz);
		if (IS_ERR(opp))
			break;
		gpu->level_khz[i] = hz / HZ_PER_KHZ_UL;
		dev_pm_opp_put(opp);
		if (!hz)
			break;	/* a 0 Hz level, do not step back onto it */
		hz++;
	}
	gpu->nlevels = i;

	return i ? 0 : -ENODEV;
}

/*
 * Stock gpu_dvfs_get_level() (K:platform/exynos/gpu_dvfs_api.c:494-513): the
 * index of an exact table entry, -1 outside [min, max] or between levels.
 */
static int exynos8895_gpu_get_level(struct exynos8895_gpu *gpu, int clock)
{
	int i;

	if (clock < (int)gpu->level_khz[0] ||
	    clock > (int)gpu->level_khz[gpu->nlevels - 1])
		return -1;

	for (i = 0; i < gpu->nlevels; i++)
		if ((int)gpu->level_khz[i] == clock)
			return i;

	return -1;
}

/*
 * Stock gpu_dvfs_get_level_clock() (K:platform/exynos/gpu_dvfs_api.c:515-531):
 * the highest table entry not above the request, -1 below the table.
 */
static int exynos8895_gpu_level_clock(struct exynos8895_gpu *gpu, int clock)
{
	int i;

	for (i = gpu->nlevels - 1; i >= 0; i--)
		if (clock - (int)gpu->level_khz[i] >= 0)
			return (int)gpu->level_khz[i];

	return -1;
}

/*
 * The current rate in kHz, 0 while the GPU is not runtime active: stock reads
 * gpu_get_cur_clock() only with the domain on
 * (K:platform/exynos/gpu_custom_interface.c:61-97 show_clock(), :1587-1622
 * show_kernel_sysfs_clock()).
 */
static int exynos8895_gpu_cur_clock(struct exynos8895_gpu *gpu)
{
	int khz = 0;

	if (gpu_is_power_on()) {
		mutex_lock(&gpu->lock);
		khz = gpu->cur_rate / HZ_PER_KHZ_UL;
		mutex_unlock(&gpu->lock);
	}

	return khz;
}

/*
 * Stock platform->env_data.utilization: 100 * time_busy / (time_busy +
 * time_idle), -1 while nothing has been collected yet
 * (K:platform/exynos/gpu_integration_callbacks.c:625-635
 * gpu_pm_get_dvfs_utilisation()).
 *
 * kbase_devfreq_status() hands the same two counters to the devfreq core every
 * polling_ms = 100 ms (K:backend/gpu/mali_kbase_devfreq.c:180-194, :359) and
 * devfreq keeps the last window in devfreq->last_status
 * (A:drivers/devfreq/governor.h:121-127 devfreq_update_stats()), which the
 * simple_ondemand governor turns into the DVFS decision
 * (A:drivers/devfreq/governor_simpleondemand.c:26-58). That window is read
 * instead of sampling the kbase metrics again: it is the utilization the DVFS
 * of this port uses, and a second accumulator of its own could wrap.
 */
static int exynos8895_gpu_utilization(struct exynos8895_gpu *gpu)
{
	struct devfreq *devfreq = gpu->kbdev->devfreq;
	unsigned int busy, total;
	u64 ret;

	if (!devfreq)
		return -1;

	mutex_lock(&devfreq->lock);
	busy = devfreq->last_status.busy_time;
	total = devfreq->last_status.total_time;
	mutex_unlock(&devfreq->lock);

	if (!total)
		return -1;

	ret = div64_u64((u64)busy * 100, total);

	return ret > 100 ? 100 : (int)ret;
}

/*
 * The locks: stock set_min_lock_dvfs() / set_max_lock_dvfs()
 * (K:platform/exynos/gpu_custom_interface.c:604-655 and :535-570):
 *
 *   "0"				unlock (:612-614 and :543-545)
 *   kstrtoint()		the raw value is user_*_lock_input (:616-620, :547-551)
 *   gpu_dvfs_get_level_clock()	round down to a level (:624, :555)
 *   range check		-ENOENT outside the table (:626-630, :557-561)
 *   min only: clamp to gpu_max_clock_limit (:632-633), the DVS ceiling
 *   table extreme		unlock instead of lock (:635-638 and :563-566)
 *
 * The clamp to gpu_max_clock_limit has no counterpart here: that is Samsung's
 * DVS/ASV ceiling, not ported (spec L6A 5.1.6). The thermal ceiling of this
 * port is the gpu-thermal cooling map (spec L6A 4.1).
 */
static int exynos8895_gpu_set_lock(struct exynos8895_gpu *gpu, int khz,
				   bool is_min)
{
	int clock;

	if (khz == 0) {
		clock = 0;
	} else {
		clock = exynos8895_gpu_level_clock(gpu, khz);
		if (exynos8895_gpu_get_level(gpu, clock) < 0) {
			gpu_err(gpu, "%s_lock: invalid clock value (%d)\n",
				is_min ? "dvfs_min" : "dvfs_max", khz);
			return -ENOENT;
		}

		/* the extreme of the table means "no lock" (:635-640, :566-569) */
		if (is_min && clock == (int)gpu->level_khz[0])
			clock = 0;
		if (!is_min && clock == (int)gpu->level_khz[gpu->nlevels - 1])
			clock = 0;
	}

	mutex_lock(&gpu->sysfs_lock);
	if (is_min) {
		gpu->lock_min_input = khz;
		gpu->lock_min_khz = clock;
	} else {
		gpu->lock_max_input = khz;
		gpu->lock_max_khz = clock;
	}
	mutex_unlock(&gpu->sysfs_lock);

	/* A minimum-frequency request of 0 and a maximum-frequency request of
	 * PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE both mean "no limit", the values the
	 * devfreq min_freq / max_freq stores use for the same thing
	 * (A:drivers/devfreq/devfreq.c:1594-1596, :1643-1650). */
	return dev_pm_qos_update_request(is_min ? &gpu->min_qos : &gpu->max_qos,
					clock ? clock :
					(is_min ? PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE :
						  PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE));
}

static int exynos8895_gpu_set_lock_min(struct exynos8895_gpu *gpu, int khz)
{
	return exynos8895_gpu_set_lock(gpu, khz, true);
}

static int exynos8895_gpu_set_lock_max(struct exynos8895_gpu *gpu, int khz)
{
	return exynos8895_gpu_set_lock(gpu, khz, false);
}

/*
 * Stock set_clock() (K:platform/exynos/gpu_custom_interface.c:99-155): a
 * non-zero clock pins the rate with the "always_on" PM policy, and "0"
 * restores the policy that was running before (:129-136).
 *
 * The pin is a third and fourth PM QoS pair, so that it wins over the locks
 * exactly as dvfs_status = 0 made gpu_check_target_clock() return the target
 * unchanged (K:platform/exynos/gpu_dvfs_api.c:42-43): the floor and the
 * ceiling both become the pinned clock, and devfreq clamps the floor to the
 * ceiling (A:drivers/devfreq/devfreq.c:155-156).
 */
static int exynos8895_gpu_set_pin(struct exynos8895_gpu *gpu, int khz)
{
	const struct kbase_pm_policy *const *policy_list;
	const struct kbase_pm_policy *policy = NULL;
	int policy_count, i, ret;

	if (khz == 0) {
		mutex_lock(&gpu->sysfs_lock);
		if (gpu->pin_khz) {
			policy = gpu->pin_policy;
			gpu->pin_policy = NULL;
			gpu->pin_khz = 0;
		}
		mutex_unlock(&gpu->sysfs_lock);

		if (policy)
			kbase_pm_set_policy(gpu->kbdev, policy);

		ret = dev_pm_qos_update_request(&gpu->pin_min_qos,
						PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);
		if (ret < 0)
			return ret;

		return dev_pm_qos_update_request(&gpu->pin_max_qos,
						 PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE);
	}

	/* gpu_check_target_clock() rejects a clock that is not a table level
	 * (K:platform/exynos/gpu_dvfs_api.c:38-39); "clock" does not round */
	if (exynos8895_gpu_get_level(gpu, khz) < 0) {
		gpu_err(gpu, "clock: invalid clock value (%d)\n", khz);
		return -ENOENT;
	}

	mutex_lock(&gpu->sysfs_lock);
	if (!gpu->pin_khz)
		gpu->pin_policy = kbase_pm_get_policy(gpu->kbdev);
	gpu->pin_khz = khz;
	mutex_unlock(&gpu->sysfs_lock);

	ret = dev_pm_qos_update_request(&gpu->pin_min_qos, khz);
	if (ret < 0)
		return ret;

	ret = dev_pm_qos_update_request(&gpu->pin_max_qos, khz);
	if (ret < 0)
		return ret;

	/* stock searched the policy list for "always_on", :137-144 */
	policy_count = kbase_pm_list_policies(&policy_list);
	for (i = 0; i < policy_count; i++) {
		if (sysfs_streq(policy_list[i]->name, "always_on")) {
			kbase_pm_set_policy(gpu->kbdev, policy_list[i]);
			return 0;
		}
	}

	gpu_err(gpu, "clock: no always_on power policy, rate pinned by QoS only\n");

	return 0;
}

/*
 * The store path. One trailing newline is stripped first, so that the power
 * HAL (which writes the value without one) and a shell "echo" both work: stock
 * compared the buffer with sysfs_streq() as it came
 * (K:platform/exynos/gpu_custom_interface.c:612), where a trailing newline
 * turns "0" into the "invalid clock value" path.
 */
static ssize_t exynos8895_gpu_store_int(struct exynos8895_gpu *gpu,
					const char *buf, size_t count,
					int (*set)(struct exynos8895_gpu *, int),
					const char *name)
{
	int khz, ret;

	if (count && buf[count - 1] == '\n')
		count--;
	if (!count)
		return -EINVAL;

	khz = 0;
	if (kstrtoint(buf, 0, &khz)) {
		gpu_err(gpu, "%s: invalid value\n", name);
		return -ENOENT;
	}

	ret = set(gpu, khz);
	if (ret < 0)
		return ret;

	return count;
}

/* ---- the platform device files, /sys/devices/platform/13900000.mali/ ---- */

/*
 * Stock show_dvfs_table() (K:platform/exynos/gpu_custom_interface.c:249-283):
 * from the gpu_max_clock level down to the gpu_min_clock level, every entry
 * with a leading space, then the newline of gpu_get_dvfs_table() plus the one
 * show() adds on top of it.
 */
static ssize_t show_dvfs_table(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct exynos8895_gpu *gpu = exynos8895_gpu_of_dev(dev);
	ssize_t ret = 0;
	int i;

	for (i = gpu->nlevels - 1; i >= 0; i--)
		ret += sysfs_emit_at(buf, ret, " %u", gpu->level_khz[i]);
	ret += sysfs_emit_at(buf, ret, "\n\n");

	return ret;
}
static DEVICE_ATTR(dvfs_table, S_IRUGO, show_dvfs_table, NULL);

/*
 * Stock show_min_lock_dvfs() (K:platform/exynos/gpu_custom_interface.c:572-602):
 * "<min_lock> / <user_min_lock_input>", or "-1" while unlocked, an unlock
 * leaving min_lock = 0 (:363-369).
 */
static ssize_t show_min_lock_dvfs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct exynos8895_gpu *gpu = exynos8895_gpu_of_dev(dev);
	int khz, input;

	mutex_lock(&gpu->sysfs_lock);
	khz = gpu->lock_min_khz;
	input = gpu->lock_min_input;
	mutex_unlock(&gpu->sysfs_lock);

	if (khz > 0)
		return sysfs_emit(buf, "%d / %d\n", khz, input);

	return sysfs_emit(buf, "-1\n");
}

static ssize_t set_min_lock_dvfs(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct exynos8895_gpu *gpu = exynos8895_gpu_of_dev(dev);

	return exynos8895_gpu_store_int(gpu, buf, count,
					exynos8895_gpu_set_lock_min,
					"dvfs_min_lock");
}
static DEVICE_ATTR(dvfs_min_lock, S_IRUGO | S_IWUSR, show_min_lock_dvfs,
		   set_min_lock_dvfs);

/* Stock show_max_lock_dvfs(), gpu_custom_interface.c:503-533 */
static ssize_t show_max_lock_dvfs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct exynos8895_gpu *gpu = exynos8895_gpu_of_dev(dev);
	int khz, input;

	mutex_lock(&gpu->sysfs_lock);
	khz = gpu->lock_max_khz;
	input = gpu->lock_max_input;
	mutex_unlock(&gpu->sysfs_lock);

	if (khz > 0)
		return sysfs_emit(buf, "%d / %d\n", khz, input);

	return sysfs_emit(buf, "-1\n");
}

static ssize_t set_max_lock_dvfs(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct exynos8895_gpu *gpu = exynos8895_gpu_of_dev(dev);

	return exynos8895_gpu_store_int(gpu, buf, count,
					exynos8895_gpu_set_lock_max,
					"dvfs_max_lock");
}
static DEVICE_ATTR(dvfs_max_lock, S_IRUGO | S_IWUSR, show_max_lock_dvfs,
		   set_max_lock_dvfs);

/*
 * Stock show_clock() (K:platform/exynos/gpu_custom_interface.c:61-97): the
 * current clock in kHz, 0 while the GPU is off.
 */
static ssize_t show_clock(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct exynos8895_gpu *gpu = exynos8895_gpu_of_dev(dev);

	return sysfs_emit(buf, "%d\n", exynos8895_gpu_cur_clock(gpu));
}

static ssize_t set_clock(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct exynos8895_gpu *gpu = exynos8895_gpu_of_dev(dev);

	return exynos8895_gpu_store_int(gpu, buf, count,
					exynos8895_gpu_set_pin, "clock");
}
static DEVICE_ATTR(clock, S_IRUGO | S_IWUSR, show_clock, set_clock);

/*
 * Stock show_utilization() (K:platform/exynos/gpu_custom_interface.c:314-330):
 * the utilization, 0 while the GPU is off (gpu_control_is_power_on() * util).
 */
static ssize_t show_utilization(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct exynos8895_gpu *gpu = exynos8895_gpu_of_dev(dev);

	return sysfs_emit(buf, "%d\n",
			  gpu_is_power_on() * exynos8895_gpu_utilization(gpu));
}
static DEVICE_ATTR(utilization, S_IRUGO, show_utilization, NULL);

static const struct attribute *exynos8895_gpu_attrs[] = {
	&dev_attr_dvfs_table.attr,
	&dev_attr_dvfs_min_lock.attr,
	&dev_attr_dvfs_max_lock.attr,
	&dev_attr_clock.attr,
	&dev_attr_utilization.attr,
	NULL,
};

/* ---- /sys/kernel/gpu, stock kobject_create_and_add("gpu", kernel_kobj) ---- */

/*
 * Stock show_kernel_sysfs_min_lock_dvfs() (K:platform/exynos/
 * gpu_custom_interface.c:1495-1521): the locked clock, or gpu_min_clock while
 * unlocked (the lowest level, FDT:4655 gpu_min_clock = <0x3f7a0>).
 */
static ssize_t kernel_show_gpu_min_clock(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	int khz;

	mutex_lock(&exynos8895_gpu_kobj->sysfs_lock);
	khz = exynos8895_gpu_kobj->lock_min_khz ?
		exynos8895_gpu_kobj->lock_min_khz :
		(int)exynos8895_gpu_kobj->level_khz[0];
	mutex_unlock(&exynos8895_gpu_kobj->sysfs_lock);

	return sysfs_emit(buf, "%d\n", khz);
}

static ssize_t kernel_set_gpu_min_clock(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	/* stock set_kernel_sysfs_min_lock_dvfs(), gpu_custom_interface.c:1523-1564 */
	return exynos8895_gpu_store_int(exynos8895_gpu_kobj, buf, count,
					exynos8895_gpu_set_lock_min,
					"gpu_min_clock");
}

/* Stock show_kernel_sysfs_max_lock_dvfs(), gpu_custom_interface.c:1402-1430 */
static ssize_t kernel_show_gpu_max_clock(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	int khz;

	mutex_lock(&exynos8895_gpu_kobj->sysfs_lock);
	khz = exynos8895_gpu_kobj->lock_max_khz ?
		exynos8895_gpu_kobj->lock_max_khz :
		(int)exynos8895_gpu_kobj->level_khz[exynos8895_gpu_kobj->nlevels - 1];
	mutex_unlock(&exynos8895_gpu_kobj->sysfs_lock);

	return sysfs_emit(buf, "%d\n", khz);
}

static ssize_t kernel_set_gpu_max_clock(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	/* stock set_kernel_sysfs_max_lock_dvfs(), gpu_custom_interface.c:1432-1466 */
	return exynos8895_gpu_store_int(exynos8895_gpu_kobj, buf, count,
					exynos8895_gpu_set_lock_max,
					"gpu_max_clock");
}

/* Stock show_kernel_sysfs_clock(), gpu_custom_interface.c:1587-1622 */
static ssize_t kernel_show_gpu_clock(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n",
			  exynos8895_gpu_cur_clock(exynos8895_gpu_kobj));
}

/*
 * Stock show_kernel_sysfs_freq_table() (K:platform/exynos/
 * gpu_custom_interface.c:1625-1648): the table ascending, every entry followed
 * by a space.
 */
static ssize_t kernel_show_gpu_freq_table(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  char *buf)
{
	struct exynos8895_gpu *gpu = exynos8895_gpu_kobj;
	ssize_t ret = 0;
	int i;

	for (i = 0; i < gpu->nlevels; i++)
		ret += sysfs_emit_at(buf, ret, "%u ", gpu->level_khz[i]);
	ret += sysfs_emit_at(buf, ret, "\n");

	return ret;
}

/* Stock show_kernel_sysfs_utilization(), gpu_custom_interface.c:1566-1585 */
static ssize_t kernel_show_gpu_busy(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%3d%%\n",
			  exynos8895_gpu_utilization(exynos8895_gpu_kobj));
}

/*
 * Stock show_kernel_sysfs_gpu_model() (K:platform/exynos/
 * gpu_custom_interface.c:1713-1768): the product name of the GPU id. Only the
 * G71 of FDT:4636 is named here (product id 0x6000 = GPU_ID2_PRODUCT_TMIX,
 * K:mali_kbase_gpu_id.h:106); the fall-back string is stock's.
 */
static ssize_t kernel_show_gpu_model(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = exynos8895_gpu_kobj->kbdev;
	u32 gpu_id = kbdev->gpu_props.props.raw_props.gpu_id;
	unsigned int product_id = gpu_id >> GPU_ID_VERSION_PRODUCT_ID_SHIFT;
	const char *name = "(Unknown Mali GPU)";

	if (GPU_ID_IS_NEW_FORMAT(product_id) &&
	    product_id ==
	    (GPU_ID2_PRODUCT_TMIX >> GPU_ID_VERSION_PRODUCT_ID_SHIFT))
		name = "Mali-G71";

	return sysfs_emit(buf, "%s\n", name);
}

static struct kobj_attribute gpu_min_clock_attribute =
	__ATTR(gpu_min_clock, S_IRUGO | S_IWUSR, kernel_show_gpu_min_clock,
	       kernel_set_gpu_min_clock);

static struct kobj_attribute gpu_max_clock_attribute =
	__ATTR(gpu_max_clock, S_IRUGO | S_IWUSR, kernel_show_gpu_max_clock,
	       kernel_set_gpu_max_clock);

static struct kobj_attribute gpu_clock_attribute =
	__ATTR(gpu_clock, S_IRUGO, kernel_show_gpu_clock, NULL);

static struct kobj_attribute gpu_freq_table_attribute =
	__ATTR(gpu_freq_table, S_IRUGO, kernel_show_gpu_freq_table, NULL);

static struct kobj_attribute gpu_busy_attribute =
	__ATTR(gpu_busy, S_IRUGO, kernel_show_gpu_busy, NULL);

static struct kobj_attribute gpu_model_attribute =
	__ATTR(gpu_model, S_IRUGO, kernel_show_gpu_model, NULL);

/* Stock attrs[] is gpu_custom_interface.c:1842-1859; the entries whose Samsung
 * source is not ported (gpu_tmu, gpu_info, the DVFS governor and
 * gpu_available_governor) are left out. */
static struct attribute *exynos8895_kernel_attrs[] = {
	&gpu_min_clock_attribute.attr,
	&gpu_max_clock_attribute.attr,
	&gpu_clock_attribute.attr,
	&gpu_freq_table_attribute.attr,
	&gpu_busy_attribute.attr,
	&gpu_model_attribute.attr,
	NULL,
};

static const struct attribute_group exynos8895_kernel_attr_group = {
	.attrs = exynos8895_kernel_attrs,
};

int exynos8895_gpu_create_sysfs(struct exynos8895_gpu *gpu)
{
	struct device *dev = gpu->kbdev->dev;
	int ret, i;

	gpu->sysfs_ready = false;

	ret = exynos8895_gpu_read_levels(gpu);
	if (ret) {
		dev_err(dev, "exynos8895: no DVFS table, no sysfs (%d)\n", ret);
		return 0;
	}

	/* unlocked: stock starts with min_lock = max_lock = 0 - the context is
	 * kmalloc()ed and memset to 0 (K:platform/exynos/mali_kbase_platform.c:423-428) */
	ret = dev_pm_qos_add_request(dev, &gpu->min_qos,
				     DEV_PM_QOS_MIN_FREQUENCY,
				     PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);
	if (ret)
		return ret;
	ret = dev_pm_qos_add_request(dev, &gpu->max_qos,
				     DEV_PM_QOS_MAX_FREQUENCY,
				     PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE);
	if (ret)
		goto err_min_qos;
	ret = dev_pm_qos_add_request(dev, &gpu->pin_min_qos,
				     DEV_PM_QOS_MIN_FREQUENCY,
				     PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);
	if (ret)
		goto err_max_qos;
	ret = dev_pm_qos_add_request(dev, &gpu->pin_max_qos,
				     DEV_PM_QOS_MAX_FREQUENCY,
				     PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE);
	if (ret)
		goto err_pin_min_qos;

	for (i = 0; exynos8895_gpu_attrs[i]; i++) {
		ret = sysfs_create_file(&dev->kobj, exynos8895_gpu_attrs[i]);
		if (ret) {
			dev_err(dev, "exynos8895: sysfs %s: %d\n",
				exynos8895_gpu_attrs[i]->name, ret);
			goto err_files;
		}
	}

	/* stock gpu_custom_interface.c:2032-2042 */
	exynos8895_gpu_kobj = gpu;
	gpu->gpu_kobj = kobject_create_and_add("gpu", kernel_kobj);
	if (!gpu->gpu_kobj) {
		ret = -ENOMEM;
		goto err_kobj;
	}
	ret = sysfs_create_group(gpu->gpu_kobj, &exynos8895_kernel_attr_group);
	if (ret) {
		kobject_put(gpu->gpu_kobj);
		gpu->gpu_kobj = NULL;
		goto err_kobj;
	}

	gpu->sysfs_ready = true;

	return 0;

err_kobj:
	exynos8895_gpu_kobj = NULL;
err_files:
	while (--i >= 0)
		sysfs_remove_file(&dev->kobj, exynos8895_gpu_attrs[i]);
err_pin_min_qos:
	dev_pm_qos_remove_request(&gpu->pin_min_qos);
err_max_qos:
	dev_pm_qos_remove_request(&gpu->max_qos);
err_min_qos:
	dev_pm_qos_remove_request(&gpu->min_qos);

	return ret;
}

void exynos8895_gpu_delete_sysfs(struct exynos8895_gpu *gpu)
{
	int i;

	/* Nothing to undo when the creation failed: every error path of
	 * exynos8895_gpu_create_sysfs() releases what it had taken. */
	if (!gpu->sysfs_ready)
		return;

	if (gpu->gpu_kobj) {
		sysfs_remove_group(gpu->gpu_kobj, &exynos8895_kernel_attr_group);
		kobject_put(gpu->gpu_kobj);
		gpu->gpu_kobj = NULL;
	}
	exynos8895_gpu_kobj = NULL;

	for (i = 0; exynos8895_gpu_attrs[i]; i++)
		sysfs_remove_file(&gpu->kbdev->dev->kobj,
				exynos8895_gpu_attrs[i]);

	dev_pm_qos_remove_request(&gpu->pin_max_qos);
	dev_pm_qos_remove_request(&gpu->pin_min_qos);
	dev_pm_qos_remove_request(&gpu->max_qos);
	dev_pm_qos_remove_request(&gpu->min_qos);

	gpu->sysfs_ready = false;
}