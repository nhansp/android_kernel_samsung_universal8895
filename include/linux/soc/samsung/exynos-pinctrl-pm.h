/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung pinctrl system-sleep hook: a notifier called first in the syscore
 * resume of the pin controllers that save and restore from syscore ops, before
 * any GPIO bank is restored (the position of stock's fingerprint resume SMC,
 * S:drivers/pinctrl/samsung/pinctrl-samsung.c:1315-1323). The callback runs on
 * one CPU with IRQs off and must not sleep. Register from probe only.
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS_PINCTRL_PM_H
#define __LINUX_SOC_SAMSUNG_EXYNOS_PINCTRL_PM_H

struct notifier_block;

#ifdef CONFIG_PINCTRL_SAMSUNG
int samsung_pinctrl_register_resume_notifier(struct notifier_block *nb);
#else
static inline int samsung_pinctrl_register_resume_notifier(struct notifier_block *nb)
{
	return 0;
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS_PINCTRL_PM_H */
