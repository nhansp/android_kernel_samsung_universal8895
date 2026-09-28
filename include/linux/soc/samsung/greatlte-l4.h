/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * greatlte L4 (storage) bring-up hooks for the heartbeat. DEBUG ONLY.
 * Spec: rewrite/spec/L4_storage.md 8.1 (printer "L4", field ufs=).
 * Implemented in drivers/soc/samsung/greatlte-hb-l4.c under CONFIG_GREATLTE_HB;
 * the only caller is the ufs-exynos probe.
 */
#ifndef __LINUX_SOC_SAMSUNG_GREATLTE_L4_H
#define __LINUX_SOC_SAMSUNG_GREATLTE_L4_H

#include <linux/kconfig.h>

struct device;

#if IS_ENABLED(CONFIG_GREATLTE_HB)
/* record the return value of the UFS host probe of @dev (0, -EPROBE_DEFER or the failure) */
void greatlte_l4_ufs_probe_done(struct device *dev, int err);
#else
static inline void greatlte_l4_ufs_probe_done(struct device *dev, int err)
{
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_GREATLTE_L4_H */
