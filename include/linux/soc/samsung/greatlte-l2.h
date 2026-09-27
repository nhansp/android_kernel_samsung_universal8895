/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * greatlte L2 (power) bring-up getters for the heartbeat. DEBUG ONLY.
 * Spec: rewrite/spec/L2_power.md 5.2.5, 5.9 (printers "L2" and "L2id").
 * Provided under CONFIG_GREATLTE_HB by exynos-acpm, sec-acpm and clk-acpm;
 * the only caller is drivers/soc/samsung/greatlte-hb-l2.c. Every getter
 * returns stored values or reads always-on registers; none sends an ACPM
 * command.
 */
#ifndef __LINUX_SOC_SAMSUNG_GREATLTE_L2_H
#define __LINUX_SOC_SAMSUNG_GREATLTE_L2_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device_node;

#define GREATLTE_L2_MAX_CHANS	16

/* Plain copy of one channel's counters (exynos-acpm struct acpm_chan_stats). */
struct acpm_chan_stats_snap {
	u32 xfer;
	u32 err;
	u32 notify;
	u32 late;
	u32 ack;
	u32 noack;
	u32 stale;
};

/* One channel of the firmware's channel table, as read at probe (L2 P3). */
struct acpm_chan_info_snap {
	u32 id;
	u32 field;
	u32 poll;
	u32 type;
	u32 qlen;
	u32 mlen;
	u32 stale;
};

/* Identity and probe results of the ACPM interface (L2 P2-P6, I4). */
struct acpm_info_snap {
	u32 initdata_base;
	u32 num_chans;
	u32 nplugins;
	int dvfs_plugin_id;		/* -1: none */
	u8 dvfs_attached;
	u8 dvfs_stay;
	const char *dvfs_dpa;		/* "ok", "fail", "sent", "skip", "none" */
	int dpd_plugin_id;		/* -1: none */
	const char *dpd_res;		/* "ok", "fail", "none" */
	u8 fvmap_lv[2];			/* levels of cpucl0 and cpucl1 */
	bool fwlog_ok;
	u32 fwlog_entries;
	u32 fwlog_bad;
	u32 fwlog_front;
	u32 fwlog_rear;
	u32 fwlog_len;
	struct acpm_chan_info_snap chan[GREATLTE_L2_MAX_CHANS];
};

#if IS_ENABLED(CONFIG_GREATLTE_HB) && IS_REACHABLE(CONFIG_EXYNOS_ACPM_PROTOCOL)
int acpm_get_chan_stats(struct device_node *acpm_np, unsigned int ch,
			struct acpm_chan_stats_snap *out);
int acpm_get_info(struct device_node *acpm_np, struct acpm_info_snap *out);
#else
static inline int acpm_get_chan_stats(struct device_node *acpm_np, unsigned int ch,
				      struct acpm_chan_stats_snap *out)
{
	return -ENODEV;
}

static inline int acpm_get_info(struct device_node *acpm_np,
				struct acpm_info_snap *out)
{
	return -ENODEV;
}
#endif

#if IS_ENABLED(CONFIG_GREATLTE_HB) && IS_REACHABLE(CONFIG_MFD_SEC_ACPM)
/* S2MPS17 TOP PMICID and the revision derived from it at probe (L2 P7). */
int sec_pmic_acpm_get_id(u8 *id, u8 *rev);
#else
static inline int sec_pmic_acpm_get_id(u8 *id, u8 *rev)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_GREATLTE_L2_H */
