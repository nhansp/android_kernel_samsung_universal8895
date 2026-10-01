/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Exynos 8895 system power management: the busy predicate of one
 * samsung,sicd-veto entry of the samsung,exynos8895-pm node (spec L7 4.4, 5.5.4).
 */
#ifndef __DT_BINDINGS_SOC_SAMSUNG_EXYNOS8895_PM_H
#define __DT_BINDINGS_SOC_SAMSUNG_EXYNOS8895_PM_H

#define E8895_SICD_VETO_CLK	0	/* busy while the node's first clock is enabled */
#define E8895_SICD_VETO_RPM	1	/* busy unless the device is runtime-suspended */
#define E8895_SICD_VETO_I2C	2	/* busy while an I2C transfer holds the adapter bus lock */
#define E8895_SICD_VETO_PWM	3	/* busy while the PWM framework has a channel enabled */

#endif /* __DT_BINDINGS_SOC_SAMSUNG_EXYNOS8895_PM_H */
