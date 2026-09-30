// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos 8895 (Mali-G71) kbase platform, spec L6A_graphics.md 5.1.5: power
 * through genpd (pd_g3d) and runtime PM with the DT autosuspend delay, the
 * G3D clock through the ACPM DVFS clock, OPPs from the DT, no regulator (the
 * APM owns the G3D rail). The rate is only sent while the GPU is runtime
 * active, i.e. with pd_g3d on, as stock did (K:platform/exynos/gpu_control.c:127-131;
 * K: = the stock drivers/gpu/arm/b_r16p0). Replaces Samsung's platform/exynos,
 * which is not ported (5.1.6).
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>

#include <dt-bindings/clock/samsung,exynos8895-acpm.h>
#include <soc/samsung/exynos8895-bus-qos.h>

#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#include "mali_kbase_config_platform.h"
#include "mali_kbase_exynos8895.h"

/*
 * Start rate: FDT:4656 gpu_dvfs_start_clock = <0x3f7a0> (260000 kHz); the
 * ECT dvfs_g3d domain has no boot level (L6A 3.1 step 4).
 */
#define EXYNOS8895_G3D_START_HZ		260000000UL

struct exynos8895_gpu {
	struct kbase_device *kbdev;
	struct clk *clk;
	struct mutex lock;		/* cur_rate, want_rate */
	unsigned long cur_rate, want_rate;
	u32 autosuspend_ms;
	struct exynos_bus_qos_request mif_req;
};

/*
 * Stock gpu_dvfs_table column 4 (mem_freq) per GPU frequency (FDT:4648,
 * S:drivers/gpu/arm/b_r16p0/platform/exynos/mali_kbase_platform.c:393;
 * spec L2b 2.2 REQ-2B-L6A-1).
 */
static s32 exynos8895_gpu_to_mif_khz(unsigned long gpu_rate_hz)
{
	unsigned long khz = gpu_rate_hz / 1000;

	if (khz >= 546000)
		return 1794000;
	if (khz >= 455000)
		return 1540000;
	if (khz >= 385000)
		return 1352000;
	if (khz >= 338000)
		return 1014000;
	if (khz >= 260000)
		return 421000;
	return 0;
}

/* Every member NULL: the core tests each one (L6A 5.1.2) */
static struct kbase_vendor_callbacks exynos8895_vendor_cb;

static struct kbase_device *exynos8895_kbdev;

/*
 * The G3D rate. Applied now if the GPU is runtime active (pd_g3d on), else
 * stored and applied by the next runtime resume. Called by kbase devfreq
 * (L6A 5.1.4) in place of clk_set_rate(); always 0, so devfreq accounts the
 * requested rate.
 */
int exynos8895_gpu_set_rate(struct kbase_device *kbdev, unsigned long rate)
{
	struct exynos8895_gpu *gpu = kbdev->platform_context;
	s32 old_mif, new_mif;
	bool active;

	mutex_lock(&gpu->lock);
	old_mif = exynos8895_gpu_to_mif_khz(gpu->cur_rate);
	new_mif = exynos8895_gpu_to_mif_khz(rate);
	gpu->want_rate = rate;
	active = (pm_runtime_get_if_active(kbdev->dev) > 0);
	mutex_unlock(&gpu->lock);

	if (active) {
		/* stock S:drivers/gpu/arm/b_r16p0/platform/exynos/gpu_control.c:184-198:
		 * raise MIF floor before raising clock, lower MIF floor after */
		if (new_mif > old_mif)
			exynos_bus_qos_update_request(&gpu->mif_req, new_mif);
		mutex_lock(&gpu->lock);
		if (!clk_set_rate(gpu->clk, rate))
			gpu->cur_rate = rate;
		mutex_unlock(&gpu->lock);
		if (new_mif < old_mif)
			exynos_bus_qos_update_request(&gpu->mif_req, new_mif);
		pm_runtime_put_autosuspend(kbdev->dev);
	}

	return 0;
}

static int exynos8895_platform_init(struct kbase_device *kbdev)
{
	struct device *dev = kbdev->dev;
	struct exynos8895_gpu *gpu;
	int ret;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;

	gpu->kbdev = kbdev;
	mutex_init(&gpu->lock);

	/* taken by power_control_init() (of_clk_get(np, 0)) */
	gpu->clk = kbdev->clock;
	if (!gpu->clk) {
		gpu->clk = devm_clk_get(dev, "clk_mali");
		if (IS_ERR(gpu->clk))
			return dev_err_probe(dev, PTR_ERR(gpu->clk),
					     "exynos8895: no g3d clock\n");
	}

	gpu->autosuspend_ms = AUTO_SUSPEND_DELAY;
	of_property_read_u32(dev->of_node, "samsung,autosuspend-delay-ms",
			     &gpu->autosuspend_ms);

	/*
	 * pd_g3d is on: genpd powered it at attach and runtime PM is not
	 * enabled yet (this runs before kbase_pm_runtime_init()). Stock set
	 * the start clock with the domain on as well.
	 */
	gpu->want_rate = EXYNOS8895_G3D_START_HZ;
	ret = clk_set_rate(gpu->clk, gpu->want_rate);
	if (ret) {
		dev_warn(dev, "exynos8895: initial g3d rate: %d\n", ret);
		gpu->cur_rate = 0;	/* the next runtime resume retries */
	} else {
		gpu->cur_rate = gpu->want_rate;
	}

	kbdev->platform_context = gpu;
	kbdev->vendor_callbacks = &exynos8895_vendor_cb;
	exynos8895_kbdev = kbdev;

	/*
	 * stock S:drivers/gpu/arm/b_r16p0/platform/exynos/gpu_pmqos.c:62 adds the
	 * request at 0; the first runtime resume or rate change sets the floor
	 * (spec L2b REQ-2B-L6A-1)
	 */
	exynos_bus_qos_add_request(&gpu->mif_req, EXYNOS_BUS_QOS_MIF, 0);

	dev_info(dev, "exynos8895: g3d %lu Hz via ACPM idx %d, %d OPPs, autosuspend %u ms\n",
		 clk_get_rate(gpu->clk), EXYNOS8895_CLK_ACPM_DVFS_G3D,
		 dev_pm_opp_get_opp_count(dev), gpu->autosuspend_ms);

	return 0;
}

static void exynos8895_platform_term(struct kbase_device *kbdev)
{
	struct exynos8895_gpu *gpu = kbdev->platform_context;

	if (gpu)
		exynos_bus_qos_remove_request(&gpu->mif_req);
	exynos8895_kbdev = NULL;
	kbdev->platform_context = NULL;
}

struct kbase_platform_funcs_conf exynos8895_platform_funcs = {
	.platform_init_func = exynos8895_platform_init,
	.platform_term_func = exynos8895_platform_term,
};

/* Power callbacks: the shape of K:platform/devicetree/mali_kbase_runtime_pm.c:28-125 */
static int exynos8895_pm_power_on(struct kbase_device *kbdev)
{
	int err = pm_runtime_get_sync(kbdev->dev);

	/*
	 * On failure the usage count taken by pm_runtime_get_sync() is kept:
	 * the core pairs this call with exynos8895_pm_power_off(), whose put
	 * drops it.
	 */
	if (err < 0) {
		dev_err(kbdev->dev, "pm_runtime_get_sync: %d\n", err);
		return 1;
	}

	/* 1: already active, the GPU was not powered off (:37) */
	return err == 1 ? 0 : 1;
}

static void exynos8895_pm_power_off(struct kbase_device *kbdev)
{
	pm_runtime_mark_last_busy(kbdev->dev);
	pm_runtime_put_autosuspend(kbdev->dev);
}

static int exynos8895_pm_runtime_init(struct kbase_device *kbdev)
{
	struct exynos8895_gpu *gpu = kbdev->platform_context;

	pm_runtime_set_autosuspend_delay(kbdev->dev, gpu->autosuspend_ms);
	pm_runtime_use_autosuspend(kbdev->dev);

	pm_runtime_set_active(kbdev->dev);
	pm_runtime_enable(kbdev->dev);

	if (!pm_runtime_enabled(kbdev->dev)) {
		dev_warn(kbdev->dev, "pm_runtime not enabled");
		return -ENOSYS;
	}

	return 0;
}

static void exynos8895_pm_runtime_term(struct kbase_device *kbdev)
{
	pm_runtime_disable(kbdev->dev);
}

/*
 * Runtime resume, after genpd ran the pd_g3d on-sequence: re-apply a rate
 * that could not be sent while the GPU was off, as stock re-applied the
 * clock in its runtime-on callback (K:platform/exynos/gpu_notifier.c:249-275).
 */
static int exynos8895_pm_runtime_on(struct kbase_device *kbdev)
{
	struct exynos8895_gpu *gpu = kbdev->platform_context;

	/* stock S:drivers/gpu/arm/b_r16p0/platform/exynos/gpu_control.c:188 */
	exynos_bus_qos_update_request(&gpu->mif_req,
				      exynos8895_gpu_to_mif_khz(gpu->want_rate));

	mutex_lock(&gpu->lock);
	if (gpu->want_rate != gpu->cur_rate &&
	    !clk_set_rate(gpu->clk, gpu->want_rate))
		gpu->cur_rate = gpu->want_rate;
	mutex_unlock(&gpu->lock);

	return 0;
}

/* Stock's clock gating and DVFS-timer stop here are Samsung DVFS (:276-300) */
static void exynos8895_pm_runtime_off(struct kbase_device *kbdev)
{
	struct exynos8895_gpu *gpu = kbdev->platform_context;

	/* stock S:drivers/gpu/arm/b_r16p0/platform/exynos/gpu_control.c:274 */
	exynos_bus_qos_update_request(&gpu->mif_req, 0);
}

static void exynos8895_pm_resume(struct kbase_device *kbdev)
{
	int ret = exynos8895_pm_runtime_on(kbdev);

	WARN_ON(ret);
}

static void exynos8895_pm_suspend(struct kbase_device *kbdev)
{
	exynos8895_pm_runtime_off(kbdev);
}

struct kbase_pm_callback_conf exynos8895_pm_callbacks = {
	.power_on_callback = exynos8895_pm_power_on,
	.power_off_callback = exynos8895_pm_power_off,
	.power_suspend_callback = exynos8895_pm_suspend,
	.power_resume_callback = exynos8895_pm_resume,
	.power_runtime_init_callback = exynos8895_pm_runtime_init,
	.power_runtime_term_callback = exynos8895_pm_runtime_term,
	.power_runtime_on_callback = exynos8895_pm_runtime_on,
	.power_runtime_off_callback = exynos8895_pm_runtime_off,
};

/* The vendor interface of L6A 5.1.2 */
int gpu_vendor_dispatch(struct kbase_context *kctx, u32 flags)
{
	/* SLSI CL/VK boost (ioctls 42/43) is not ported: no CPU boost (5.1.6) */
	return 0;
}

int gpu_is_power_on(void)
{
	struct kbase_device *kbdev = exynos8895_kbdev;

	return kbdev ? pm_runtime_active(kbdev->dev) : 0;
}

struct kbase_device *gpu_get_device_structure(void)
{
	return exynos8895_kbdev;
}

/* The same output as K:mali_kbase_gpu_memory_debugfs.c:40-68 */
int gpu_memory_seq_show(struct seq_file *sfile, void *data)
{
	struct list_head *entry;
	const struct list_head *kbdev_list;

	kbdev_list = kbase_dev_list_get();
	list_for_each(entry, kbdev_list) {
		struct kbase_device *kbdev = NULL;
		struct kbasep_kctx_list_element *element;

		kbdev = list_entry(entry, struct kbase_device, entry);
		/* output the total memory usage and cap for this device */
		seq_printf(sfile, "%-16s  %10u\n",
				kbdev->devname,
				atomic_read(&(kbdev->memdev.used_pages)));
		mutex_lock(&kbdev->kctx_list_lock);
		list_for_each_entry(element, &kbdev->kctx_list, link) {
			/* output the memory usage and cap for each kctx
			* opened on this device */
			seq_printf(sfile, "  %s-0x%p %10u\n",
				"kctx",
				element->kctx,
				atomic_read(&(element->kctx->used_pages)));
		}
		mutex_unlock(&kbdev->kctx_list_lock);
	}
	kbase_dev_list_put(kbdev_list);
	return 0;
}
