/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung S2MM005 USB Type-C / PD port controller: the read-only USB-PD sink status
 * the driver delivers to the charger side (spec L6G 5.1.10, L6H REQ-L6G-3).
 */
#ifndef __LINUX_USB_S2MM005_H
#define __LINUX_USB_S2MM005_H

#include <linux/types.h>

struct notifier_block;

/* S:include/linux/battery/battery_notifier.h:26 */
#define S2MM005_MAX_PDO_NUM	7

/* notifier actions = stock pdic_notifier_event_t (battery_notifier.h:78-84) */
enum s2mm005_pd_event {
	S2MM005_PD_EVENT_DETACH = 0,
	S2MM005_PD_EVENT_CCIC_ATTACH,
	S2MM005_PD_EVENT_PD_SINK,
	S2MM005_PD_EVENT_PD_SOURCE,
	S2MM005_PD_EVENT_PD_SINK_CAP,
};

/* stock RP_CURRENT_LEVEL (battery_notifier.h:91-97) */
enum s2mm005_rp_level {
	S2MM005_RP_CURRENT_LEVEL_NONE = 0,
	S2MM005_RP_CURRENT_LEVEL_DEFAULT,
	S2MM005_RP_CURRENT_LEVEL2,
	S2MM005_RP_CURRENT_LEVEL3,
};

/* stock PDIC_SINK_STATUS (battery_notifier.h:86-105); notifier data is a copy of this */
struct s2mm005_pd_status {
	struct {
		int max_voltage;	/* mV */
		int max_current;	/* mA */
	} power_list[S2MM005_MAX_PDO_NUM + 1];
	int available_pdo_num;
	int selected_pdo_num;
	int current_pdo_num;
	unsigned int rp_currentlvl;
};

/* one port; registration works before the port controller binds */
int s2mm005_register_notifier(struct notifier_block *nb);
int s2mm005_unregister_notifier(struct notifier_block *nb);

/*
 * Request source PDO @num (1-based). Writes the chip only with
 * CONFIG_TYPEC_S2MM005_PD_SELECT (charging path, default n); otherwise logs and returns.
 */
void s2mm005_select_pdo(int num);

#endif /* __LINUX_USB_S2MM005_H */
