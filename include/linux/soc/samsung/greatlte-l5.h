/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * greatlte L5 (display) heartbeat counters. DEBUG ONLY, never upstream.
 * Spec: rewrite/spec/L5_display.md 5.9 (printer "dpuc"). The display drivers
 * publish their devices and count their events here; the printers of
 * drivers/soc/samsung/greatlte-hb-l5.c read them. A driver passes
 * IS_BUILTIN(<its own symbol>) as @on, so a modular build of it compiles the
 * calls away and needs no export.
 */
#ifndef __LINUX_SOC_SAMSUNG_GREATLTE_L5_H
#define __LINUX_SOC_SAMSUNG_GREATLTE_L5_H

#include <linux/atomic.h>
#include <linux/compiler.h>
#include <linux/types.h>

struct device;
struct gpio_desc;

struct greatlte_l5_stats {
	/* DECON0 (drivers/gpu/drm/exynos/exynos8895_drm_decon.c) */
	struct device *decon_dev;
	void __iomem *decon_regs;	/* DECON0, 0x12860000 */
	void __iomem *decon_dma;	/* DPU_DMA, 0x128b0000 */
	atomic_t fd, fs, to, rc, er, ude, dpe, tec;
	bool ho;			/* S-Boot handover not yet replaced by a modeset */
	atomic_t he, hx;		/* hibernation enters, exits */

	/* DSIM0 (drivers/gpu/drm/bridge/samsung-dsim.c, Exynos 8895 path) */
	struct device *dsim_dev;
	void __iomem *dsim_regs;	/* 0x12870000 */
	const unsigned int *dsim_state;	/* struct samsung_dsim.state */
	atomic_t ur, ecc;

	/* S6E3HA6 panel (drivers/gpu/drm/panel/panel-samsung-s6e3ha6.c) */
	bool panel;			/* probed */
	bool bt, pw;			/* booted from S-Boot's state, rails on */
	struct gpio_desc *det;		/* disp-det */
	u32 lt;				/* lcdtype from the command line */
	const char *var;		/* panel-lut variant */
};

#if IS_ENABLED(CONFIG_GREATLTE_HB_L5)
extern struct greatlte_l5_stats greatlte_l5;
#define greatlte_l5_inc(on, f)		do { if (on) atomic_inc(&greatlte_l5.f); } while (0)
#define greatlte_l5_set(on, f, v)	do { if (on) WRITE_ONCE(greatlte_l5.f, (v)); } while (0)
#else
#define greatlte_l5_inc(on, f)		do { } while (0)
#define greatlte_l5_set(on, f, v)	do { } while (0)
#endif

#endif /* __LINUX_SOC_SAMSUNG_GREATLTE_L5_H */
