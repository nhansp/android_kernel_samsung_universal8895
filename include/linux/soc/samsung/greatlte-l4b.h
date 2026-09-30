/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * greatlte L4b (UFS inline encryption FMP, secure-world TRNG) debug hooks.
 * DEBUG ONLY. Spec: rewrite/spec/L4b_fmp_rng.md 5.5 (heartbeat rows l4bfmp,
 * l4brng) and 5.6 (FMP known-answer test). The row hooks are implemented in
 * drivers/soc/samsung/greatlte-hb-l4b.c under CONFIG_GREATLTE_HB_L4B, the KAT
 * override in drivers/ufs/host/ufs-exynos.c under
 * CONFIG_SCSI_UFS_EXYNOS8895_FMP_KAT.
 */
#ifndef __LINUX_SOC_SAMSUNG_GREATLTE_L4B_H
#define __LINUX_SOC_SAMSUNG_GREATLTE_L4B_H

#include <linux/kconfig.h>
#include <linux/types.h>

struct blk_crypto_key;

#if IS_ENABLED(CONFIG_GREATLTE_HB_L4B)
/* ufs-exynos, 8895 probe: UFSHCD_CAP_CRYPTO set, UFSPRSECURITY read back */
void greatlte_l4b_fmp_probe(bool crypto, unsigned long ufspr_security);
/* exynos-trng, 8895 variant: every SMC_CMD_RANDOM call (x1 = @op, result @ret) */
void greatlte_l4b_rng_smc(unsigned long op, int ret);
#else
static inline void greatlte_l4b_fmp_probe(bool crypto,
					  unsigned long ufspr_security)
{
}

static inline void greatlte_l4b_rng_smc(unsigned long op, int ret)
{
}
#endif

#if IS_ENABLED(CONFIG_SCSI_UFS_EXYNOS8895_FMP_KAT)
/* FMP key layouts the KAT tries: FMP_KEY_LAYOUT_* of drivers/ufs/host/ufs-exynos.h */
#define GREATLTE_L4B_KAT_HALVES		0	/* gs101 order */
#define GREATLTE_L4B_KAT_WHOLE_REVERSED	1	/* stock 8895 order */

/*
 * Use @layout for requests whose key is @key (pointer comparison) until
 * disarmed; every other key keeps the layout of the SoC data.
 */
void exynos8895_ufs_fmp_kat_arm(const struct blk_crypto_key *key, u8 layout);
void exynos8895_ufs_fmp_kat_disarm(void);
#endif

#endif /* __LINUX_SOC_SAMSUNG_GREATLTE_L4B_H */
