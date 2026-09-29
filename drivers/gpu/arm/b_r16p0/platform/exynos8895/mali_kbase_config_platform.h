/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos 8895 kbase platform configuration (spec L6A_graphics.md 5.1.5),
 * the interface of K:platform/devicetree/mali_kbase_config_platform.h:29-46
 * (K: = the stock drivers/gpu/arm/b_r16p0). No PROTECTED_CALLBACKS: protected
 * rendering is not ported (5.1.6).
 */

/**
 * Power management configuration
 *
 * Attached value: pointer to @ref kbase_pm_callback_conf
 * Default value: See @ref kbase_pm_callback_conf
 */
#define POWER_MANAGEMENT_CALLBACKS (&exynos8895_pm_callbacks)

/**
 * Platform specific configuration functions
 *
 * Attached value: pointer to @ref kbase_platform_funcs_conf
 * Default value: See @ref kbase_platform_funcs_conf
 */
#define PLATFORM_FUNCS (&exynos8895_platform_funcs)

extern struct kbase_pm_callback_conf exynos8895_pm_callbacks;
extern struct kbase_platform_funcs_conf exynos8895_platform_funcs;

/**
 * Autosuspend delay
 *
 * The delay time (in milliseconds) to be used for autosuspend: FDT:4672
 * gpu_runtime_pm_delay_time = <0x32>. The DT property
 * samsung,autosuspend-delay-ms overrides it at platform init.
 */
#define AUTO_SUSPEND_DELAY (50)
