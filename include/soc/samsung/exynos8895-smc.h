/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 8895 (greatlte) secure monitor calls to Samsung's EL3 monitor that are
 * not SMCCC services: ids of the stock S:include/linux/smc.h and a plain
 * arm_smccc_smc() wrapper (spec L6F 5.11; users add their ids here).
 */
#ifndef __SOC_SAMSUNG_EXYNOS8895_SMC_H
#define __SOC_SAMSUNG_EXYNOS8895_SMC_H

#define MC_FC_FP_PM_SUSPEND		0x83000021	/* S:drivers/fingerprint/fingerprint.h:57 */
#define MC_FC_FP_PM_RESUME		0x83000022	/* :58 (stock issues it from pinctrl-samsung.c:1322) */
#define MC_FC_FP_PM_SUSPEND_RETAIN	0x83000026	/* :59 */
#define MC_FC_FP_CS_SET		0x83000027	/* :60 */
#define SMC_SRPMB_WSM			0x82003811	/* S:include/linux/smc.h:54; user 6T-R (REQ-6T-L6-3, L6T_tee.draft.md 2.6) */
#define SMC_CMD_SEC_LOG_INFO		0x82000610	/* S:include/linux/smc.h:90; user 6T-S (REQ-6T-L6-4) */

/* ABOX (L6E 5.7.1, F-48) */
#define SMC_CMD_REG			(-101)				/* S:include/linux/smc.h:24 */
#define SMC_REG_CLASS_SFR_W		(0x1 << 30)			/* :94 */
#define SMC_REG_ID_SFR_W(addr)		(SMC_REG_CLASS_SFR_W | ((addr) >> 2))	/* :97 */
#define SMC_CMD_ABOX_MAGIC		0x82000501			/* S:sound/soc/samsung/abox/abox.c:4992 literal */

unsigned long exynos8895_smc(unsigned long id, unsigned long a1, unsigned long a2, unsigned long a3);

#endif /* __SOC_SAMSUNG_EXYNOS8895_SMC_H */
