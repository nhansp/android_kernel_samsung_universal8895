/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 secure monitor calls to the Samsung EL3 monitor that are not
 * SMCCC services.
 */
#ifndef __SOC_SAMSUNG_EXYNOS8895_SMC_H
#define __SOC_SAMSUNG_EXYNOS8895_SMC_H

/* fingerprint sensor SPI ownership and power, through the secure world */
#define MC_FC_FP_PM_SUSPEND		0x83000021
#define MC_FC_FP_PM_RESUME		0x83000022
#define MC_FC_FP_PM_SUSPEND_RETAIN	0x83000026
#define MC_FC_FP_CS_SET			0x83000027

#define SMC_SRPMB_WSM			0x82003811
#define SMC_CMD_SEC_LOG_INFO		0x82000610

/* CP power control through the monitor */
#define SMC_ID				0x82000700
#define READ_CTRL			0x3
#define WRITE_CTRL			0x4

/* SFR write through the monitor; negative ids reach EL3 sign-extended */
#define SMC_CMD_REG			(-101)
#define SMC_REG_CLASS_SFR_W		(0x1 << 30)
#define SMC_REG_ID_SFR_W(addr)		(SMC_REG_CLASS_SFR_W | ((addr) >> 2))
#define SMC_CMD_ABOX_MAGIC		0x82000501

unsigned long exynos8895_smc(unsigned long id, unsigned long a1, unsigned long a2,
			     unsigned long a3);

#endif /* __SOC_SAMSUNG_EXYNOS8895_SMC_H */
