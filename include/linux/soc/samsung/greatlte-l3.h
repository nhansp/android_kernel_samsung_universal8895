/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * greatlte L3 (USB) bring-up getter for the heartbeat. DEBUG ONLY.
 * Spec: rewrite/spec/L3_usb.md 1.2 (P1, P2, P11), 8.1, 8.3 (R2, R8).
 * Provided under CONFIG_GREATLTE_USB_DIAG by the Exynos8895 variant of
 * drivers/phy/samsung/phy-exynos5-usbdrd.c; the only caller is
 * drivers/soc/samsung/greatlte-hb-l3.c. It copies stored values; no PHY
 * register is read.
 */
#ifndef __LINUX_SOC_SAMSUNG_GREATLTE_L3_H
#define __LINUX_SOC_SAMSUNG_GREATLTE_L3_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

/* The Exynos8895 USB PHY's CAL state (P1, P2, P11). */
struct greatlte_l3_phy_snap {
	u32 ver[2];		/* offset-0 words of both PHY windows, first init */
	bool probed;		/* the first init ran */
	bool shift;		/* +4 applied: ver[0] != 0 */
	int port;		/* 0, 1 (flipped) or -1 (zero version) */
	unsigned int inits;	/* .init calls */
	unsigned int retunes;	/* re-tunes from the dwc3 soft-reset hook */
	unsigned int cr_timeouts;
	int cr1002_before;	/* CR[0x1002] read before the last re-tune, -1 before any */
	bool armed;		/* re-tune hook armed */
};

#if IS_REACHABLE(CONFIG_PHY_EXYNOS5_USBDRD) && defined(CONFIG_GREATLTE_USB_DIAG)
int exynos8895_usbdrd_get_snap(struct greatlte_l3_phy_snap *s);
#else
static inline int exynos8895_usbdrd_get_snap(struct greatlte_l3_phy_snap *s)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_GREATLTE_L3_H */
