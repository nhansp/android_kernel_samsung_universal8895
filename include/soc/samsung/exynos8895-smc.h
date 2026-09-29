/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 8895 (greatlte) secure monitor calls to Samsung's EL3 monitor that are
 * not SMCCC services: ids of the stock S:include/linux/smc.h and a plain
 * arm_smccc_smc() wrapper (spec L6F 5.11). Complete for every L6 user (plan X-37: L6F, L6T,
 * L6C, L6E); no later work package edits it.
 */
#ifndef __SOC_SAMSUNG_EXYNOS8895_SMC_H
#define __SOC_SAMSUNG_EXYNOS8895_SMC_H

#define MC_FC_FP_PM_SUSPEND		0x83000021	/* S:drivers/fingerprint/fingerprint.h:57 */
#define MC_FC_FP_PM_RESUME		0x83000022	/* :58 (stock issues it from pinctrl-samsung.c:1322) */
#define MC_FC_FP_PM_SUSPEND_RETAIN	0x83000026	/* :59 */
#define MC_FC_FP_CS_SET		0x83000027	/* :60 */
#define SMC_SRPMB_WSM			0x82003811	/* S:include/linux/smc.h:54; user 6T-R (REQ-6T-L6-3, L6T_tee.draft.md 2.6) */
#define SMC_CMD_SEC_LOG_INFO		0x82000610	/* S:include/linux/smc.h:90; user 6T-S (REQ-6T-L6-4) */

/* modem (L6C 2.5, D-4): CP power control through EL3 and the SSS clock of the secure CP boot */
#define SMC_ID				0x82000700	/* S:include/soc/samsung/pmu-cp.h:114 */
#define READ_CTRL			0x3		/* :115 */
#define WRITE_CTRL			0x4		/* :116 */
#define SMC_ID_CLK			0x82001011	/* S:drivers/misc/modem_v1/modem_v1.h:31 */
#define SSS_CLK_ENABLE			0		/* :32 */
#define SSS_CLK_DISABLE			1		/* :33 */

/* ABOX (L6E 5.7.1, F-48): SFR writes through EL3 and the ABOX magic call */
#define SMC_CMD_REG			(-101)		/* S:include/linux/smc.h:24 */
#define SMC_REG_CLASS_SFR_W		(0x1 << 30)	/* :94 */
#define SMC_REG_ID_SFR_W(addr)		(SMC_REG_CLASS_SFR_W | ((addr) >> 2))	/* :97 */
#define SMC_CMD_ABOX_MAGIC		0x82000501	/* S:sound/soc/samsung/abox/abox.c:4992 literal */

unsigned long exynos8895_smc(unsigned long id, unsigned long a1, unsigned long a2, unsigned long a3);

#endif /* __SOC_SAMSUNG_EXYNOS8895_SMC_H */
