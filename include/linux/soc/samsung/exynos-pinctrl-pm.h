/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung pinctrl interface for the SoC system sleep code
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS_PINCTRL_PM_H
#define __LINUX_SOC_SAMSUNG_EXYNOS_PINCTRL_PM_H

#include <linux/types.h>

u32 exynos_pinctrl_get_eint_wake_mask(void);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS_PINCTRL_PM_H */
