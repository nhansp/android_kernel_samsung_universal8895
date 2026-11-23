/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MAX77865 MUIC AFC/QC high-voltage charger detection, as the charger uses it
 */
#ifndef __LINUX_POWER_MAX77865_MUIC_HV_H
#define __LINUX_POWER_MAX77865_MUIC_HV_H

#include <linux/kconfig.h>
#include <linux/notifier.h>
#include <linux/types.h>

/* notifier actions; data points to the voltage in mV (int), blocking chain */
enum max77865_hv_event {
	MAX77865_HV_EV_NONE,		/* detached / no HV */
	MAX77865_HV_EV_PREPARE,		/* AFC prepare (5 V, ping pending) */
	MAX77865_HV_EV_5V,		/* AFC 5 V ready */
	MAX77865_HV_EV_9V,		/* AFC/QC 9 V on VBUS */
	MAX77865_HV_EV_ERR_V,		/* AFC error voltage (9 V group) */
};

/* max77865_hv_last_mv(): the last notified voltage in mV, 0 = none */
#if IS_ENABLED(CONFIG_CHARGER_MAX77865_AFC)
int max77865_hv_last_mv(void);

void max77865_hv_charger_ready(void);
void max77865_hv_set_voltage(int mv);
int max77865_hv_register_notifier(struct notifier_block *nb);
int max77865_hv_unregister_notifier(struct notifier_block *nb);
#else
static inline void max77865_hv_charger_ready(void) { }
static inline void max77865_hv_set_voltage(int mv) { }

static inline int max77865_hv_register_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline int max77865_hv_unregister_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline int max77865_hv_last_mv(void)
{
	return 0;
}
#endif

#endif /* __LINUX_POWER_MAX77865_MUIC_HV_H */
