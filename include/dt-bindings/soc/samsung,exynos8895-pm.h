/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Busy predicates of the samsung,sicd-veto entries of the Exynos8895 system
 * power management node.
 */
#ifndef __DT_BINDINGS_SOC_SAMSUNG_EXYNOS8895_PM_H
#define __DT_BINDINGS_SOC_SAMSUNG_EXYNOS8895_PM_H

#define E8895_SICD_VETO_CLK	0	/* busy while the node's first clock is enabled */
#define E8895_SICD_VETO_RPM	1	/* busy unless the device is runtime-suspended */
#define E8895_SICD_VETO_I2C	2	/* busy while an I2C transfer holds the adapter bus lock */
#define E8895_SICD_VETO_PWM	3	/* busy while the PWM framework has a channel enabled */

#endif /* __DT_BINDINGS_SOC_SAMSUNG_EXYNOS8895_PM_H */
