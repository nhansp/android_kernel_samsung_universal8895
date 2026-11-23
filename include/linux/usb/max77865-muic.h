/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Maxim MAX77865 MUIC interface for the USB Type-C port controller and the
 * high-voltage charger detection driver.
 */
#ifndef __LINUX_USB_MAX77865_MUIC_H
#define __LINUX_USB_MAX77865_MUIC_H

#include <linux/types.h>

struct auxiliary_device;
struct device_node;
struct mutex;
struct notifier_block;
struct regmap;
struct max77865_muic;

/* attached device types, numbered as in the Samsung MUIC framework */
#define MAX77865_MUIC_ATTACHED_NONE		0
#define MAX77865_MUIC_ATTACHED_USB		1
#define MAX77865_MUIC_ATTACHED_CDP		2
#define MAX77865_MUIC_ATTACHED_OTG		3
#define MAX77865_MUIC_ATTACHED_TA		4
#define MAX77865_MUIC_ATTACHED_JIG_UART_OFF	17
#define MAX77865_MUIC_ATTACHED_JIG_UART_ON	21
#define MAX77865_MUIC_ATTACHED_JIG_UART_ON_VB	22
#define MAX77865_MUIC_ATTACHED_JIG_USB_OFF	23
#define MAX77865_MUIC_ATTACHED_JIG_USB_ON	24

/* notifier actions; data is a pointer to the new value (int) or NULL */
enum max77865_muic_event {
	MAX77865_MUIC_EV_ATTACHED_DEV,	/* the attached device type changed */
	MAX77865_MUIC_EV_RESET,		/* the chip reset its registers */
};

struct max77865_muic *max77865_muic_get(struct device_node *np);
void max77865_muic_put(struct max77865_muic *m);
struct max77865_muic *max77865_muic_from_aux(struct auxiliary_device *adev);

/* Type-C port controller events */
int max77865_muic_attach_rprd(struct max77865_muic *m);
int max77865_muic_attach_sink(struct max77865_muic *m, unsigned int cc_rp);
int max77865_muic_detach(struct max77865_muic *m);
int max77865_muic_rid(struct max77865_muic *m, unsigned int rid);
void max77865_muic_set_water(struct max77865_muic *m, bool water);

/* high-voltage charger detection */
struct regmap *max77865_muic_regmap(struct max77865_muic *m);
struct mutex *max77865_muic_lock(struct max77865_muic *m);
/* hv_irq runs in the MUIC interrupt thread with the MUIC lock held */
int max77865_muic_set_hv_irq(struct max77865_muic *m,
			     void (*hv_irq)(void *ctx, u8 int_bc, u8 int_fc, u8 int_gp),
			     void *ctx);
int max77865_muic_attached_dev(struct max77865_muic *m);
int max77865_muic_run_chgdet(struct max77865_muic *m);
unsigned int max77865_muic_cc_rp(struct max77865_muic *m);
bool max77865_muic_water(struct max77865_muic *m);
int max77865_muic_register_notifier(struct max77865_muic *m, struct notifier_block *nb);
int max77865_muic_unregister_notifier(struct max77865_muic *m, struct notifier_block *nb);

#endif /* __LINUX_USB_MAX77865_MUIC_H */
