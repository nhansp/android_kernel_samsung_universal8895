/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IDT wireless charging receiver: the interface to the MAX77865 charger policy.
 * Pad types and Vout steps follow the vendor driver.
 */

#ifndef __LINUX_POWER_IDT_MFC_WPC_H
#define __LINUX_POWER_IDT_MFC_WPC_H

#include <linux/errno.h>
#include <linux/types.h>

enum idt_mfc_pad {
	IDT_MFC_PAD_NONE = 0,
	IDT_MFC_PAD_WPC,
	IDT_MFC_PAD_WPC_AFC,
	IDT_MFC_PAD_WPC_PACK,
	IDT_MFC_PAD_WPC_PACK_TA,
	IDT_MFC_PAD_WPC_STAND,
	IDT_MFC_PAD_WPC_STAND_HV,
	IDT_MFC_PAD_PMA,
	IDT_MFC_PAD_WPC_VEHICLE,
	IDT_MFC_PAD_WPC_VEHICLE_HV,
	IDT_MFC_PAD_PREPARE_HV,
	IDT_MFC_PAD_A4WP,
};

enum idt_mfc_vout {
	IDT_MFC_VOUT_5V = 0,
	IDT_MFC_VOUT_6V,
	IDT_MFC_VOUT_7V,
	IDT_MFC_VOUT_8V,
	IDT_MFC_VOUT_9V,
	IDT_MFC_VOUT_10V,
	IDT_MFC_VOUT_5_5V,
};

struct idt_mfc_wpc_status {
	bool online;		/* on a pad and the pad has not reported a foreign object */
	bool fod;		/* the pad reported FOD (TX_CHG_ERR_FOD) since it was placed */
	enum idt_mfc_pad pad;
	int vout_mv;		/* requested Vout */
};

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC)
int idt_mfc_wpc_get_status(struct idt_mfc_wpc_status *st);
#else
static inline int idt_mfc_wpc_get_status(struct idt_mfc_wpc_status *st) { return -ENODEV; }
#endif

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_NV)
/* commands of the 5 V charging policy */
void idt_mfc_wpc_rx_update(void);
void idt_mfc_wpc_set_headroom(int headroom);
void idt_mfc_wpc_set_cv_mode(void);
void idt_mfc_wpc_full(void);
void idt_mfc_wpc_send_ept(void);
#endif

#if IS_ENABLED(CONFIG_WIRELESS_CHARGER_IDT_MFC_HV)
/* the charger's 10 V grant */
void idt_mfc_wpc_hv_grant(bool grant);
#endif

/* exported by max77865_charger.c: charge control on and not in fault mode */
bool max77865_charger_wpc_allowed(void);

#endif /* __LINUX_POWER_IDT_MFC_WPC_H */
