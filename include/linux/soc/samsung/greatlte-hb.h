/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * greatlte bring-up heartbeat: per-layer printer and row hooks. DEBUG ONLY.
 * Contract: T_tooling spec 5.7.6, extended by FULL_IMAGE_PLAN.md 2.2 T14 (rows
 * registered from each layer's own DEBUG file, plan X-55; layer 7, O7-26).
 * Callers must be built in (bool Kconfig); a registered printer or row is never
 * unregistered.
 */
#ifndef __LINUX_SOC_SAMSUNG_GREATLTE_HB_H
#define __LINUX_SOC_SAMSUNG_GREATLTE_HB_H

#include <linux/errno.h>
#include <linux/types.h>

#define GREATLTE_HB_TEXT	2048	/* fmt output per call; '\n' separates records */
#define GREATLTE_HB_LAYERS	8	/* layer ids 0..7 (L0..L7) */
#define GREATLTE_HB_VALUE	80	/* GREATLTE_HB_FN: value text per call, incl. the NUL */

struct greatlte_hb_printer {
	const char *name;	/* line type: 1..8 chars of [A-Za-z0-9], unique */
	u8 layer;		/* 0..7: owning layer, for gl-hb accounting and hb.skip_layers */
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

/* Row kinds (T 5.7.4 table "Row kinds available to later layers", plus FN). */
enum greatlte_hb_kind {
	GREATLTE_HB_IRQ,	/* .path DT node, .idx interrupt index: kstat_irqs_usr() total */
	GREATLTE_HB_DEV,	/* .path DT node of a platform/i2c/spi device: bound driver name, or U */
	GREATLTE_HB_CLK,	/* .path DT node, .name clock-names entry (else .idx): rate:enable count */
	GREATLTE_HB_REG,	/* .path = the rail's regulator-name: enabled:uV (class, hb-probe) */
	GREATLTE_HB_FREQ,	/* .idx cpu: cpufreq_quick_get() kHz */
	GREATLTE_HB_BLK,	/* block-class devices <all>/<with parent> (class) */
	GREATLTE_HB_PSY,	/* .path power_supply name, .idx property: intval (class, hb-probe) */
	GREATLTE_HB_TZ,		/* .path thermal zone type: milli-degC (class, hb-probe) */
	GREATLTE_HB_FN,		/* .fn writes the value text in 'hb': lock-free reads only (5.7.6 rule 4) */
};

/* One heartbeat row, printed as " <tag>=<value>" in the "L<layer>f" line of every period.
 * greatlte_hb_add_items() rejects a (layer, tag) that is already registered (-EEXIST). */
struct greatlte_hb_item {
	u8 layer;		/* 0..7 */
	const char *tag;	/* 1..8 chars of [A-Za-z0-9_]; unique within the layer */
	enum greatlte_hb_kind kind;
	const char *path;	/* IRQ, DEV, CLK: DT node path; REG: regulator-name; PSY: supply name; TZ: zone type */
	const char *name;	/* CLK: clock-names entry (NULL: use .idx) */
	int idx;		/* IRQ: interrupt index; CLK: clock index; FREQ: cpu; PSY: enum power_supply_property */
	int (*fn)(char *buf, size_t size);	/* FN: bytes written (> 0) or -errno; size is GREATLTE_HB_VALUE.
						 * The text prints as one token: ' ' becomes '_', '=' ':' and a
						 * control character '?' (e.g. "n:2_o:17/340/dev"). */
};

#ifdef CONFIG_GREATLTE_HB
int greatlte_hb_register(struct greatlte_hb_printer *p);
int greatlte_hb_add_items(const struct greatlte_hb_item *items, unsigned int n);
#else
static inline int greatlte_hb_register(struct greatlte_hb_printer *p)
{
	return -ENODEV;
}

static inline int greatlte_hb_add_items(const struct greatlte_hb_item *items, unsigned int n)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_GREATLTE_HB_H */
