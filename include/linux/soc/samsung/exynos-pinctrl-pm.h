/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung pinctrl interface for the SoC system sleep code
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS_PINCTRL_PM_H
#define __LINUX_SOC_SAMSUNG_EXYNOS_PINCTRL_PM_H

#include <linux/types.h>

struct notifier_block;

u32 exynos_pinctrl_get_eint_wake_mask(void);

#ifdef CONFIG_PINCTRL_SAMSUNG
int samsung_pinctrl_register_resume_notifier(struct notifier_block *nb);
#else
static inline int samsung_pinctrl_register_resume_notifier(struct notifier_block *nb)
{
	return 0;
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS_PINCTRL_PM_H */
