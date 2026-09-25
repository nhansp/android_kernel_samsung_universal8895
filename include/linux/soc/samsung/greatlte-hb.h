/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * greatlte bring-up heartbeat: per-layer printer hook. DEBUG ONLY.
 * Contract: T_tooling spec 5.7.6. Callers must be built in (bool Kconfig);
 * a registered printer is never unregistered.
 */
#ifndef __LINUX_SOC_SAMSUNG_GREATLTE_HB_H
#define __LINUX_SOC_SAMSUNG_GREATLTE_HB_H

#include <linux/errno.h>
#include <linux/types.h>

#define GREATLTE_HB_TEXT	2048	/* fmt output per call; '\n' separates records */

struct greatlte_hb_printer {
	const char *name;	/* line type: 1..8 chars of [A-Za-z0-9], unique */
	u8 layer;		/* 0..6: owning layer, for gl-hb accounting */
	bool slow;		/* fmt runs in hb-probe: may sleep, take driver locks, do MMIO */
	unsigned int every_ms;	/* 0: every period; else first period, then once per every_ms */
	int (*fmt)(char *buf, size_t size);	/* bytes written (> 0) or -errno */

	/* private to drivers/soc/samsung/greatlte-hb.c */
	char text[2][GREATLTE_HB_TEXT];
	int ret[2];
	u64 done_ms[2];
	unsigned int cur;
	u64 last_ms;
	bool printed;
};

#ifdef CONFIG_GREATLTE_HB
int greatlte_hb_register(struct greatlte_hb_printer *p);
#else
static inline int greatlte_hb_register(struct greatlte_hb_printer *p)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_GREATLTE_HB_H */
