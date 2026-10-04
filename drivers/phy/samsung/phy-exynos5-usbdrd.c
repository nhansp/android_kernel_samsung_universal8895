// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos5 SoC series USB DRD PHY driver
 *
 * Phy provider for USB 3.0 DRD controller on Exynos5 SoC series
 *
 * Copyright (C) 2014 Samsung Electronics Co., Ltd.
 * Author: Vivek Gautam <gautam.vivek@samsung.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/iopoll.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/samsung/exynos-regs-pmu.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>
#ifdef CONFIG_ANDROID_VENDOR_HOOKS
#include <trace/hooks/dwc3.h>
#endif
#ifdef CONFIG_GREATLTE_USB_DIAG
#include <linux/soc/samsung/greatlte-l3.h>
#endif

/* Exynos USB PHY registers */
#define EXYNOS5_FSEL_9MHZ6		0x0
#define EXYNOS5_FSEL_10MHZ		0x1
#define EXYNOS5_FSEL_12MHZ		0x2
#define EXYNOS5_FSEL_19MHZ2		0x3
#define EXYNOS5_FSEL_20MHZ		0x4
#define EXYNOS5_FSEL_24MHZ		0x5
#define EXYNOS5_FSEL_26MHZ		0x6
#define EXYNOS5_FSEL_50MHZ		0x7

/* USB 3.2 DRD 4nm PHY link controller registers */
#define EXYNOS2200_DRD_CLKRST			0x0c
#define EXYNOS2200_CLKRST_LINK_PCLK_SEL		BIT(1)

#define EXYNOS2200_DRD_UTMI			0x10
#define EXYNOS2200_UTMI_FORCE_VBUSVALID		BIT(1)
#define EXYNOS2200_UTMI_FORCE_BVALID		BIT(0)

#define EXYNOS2200_DRD_HSP_MISC			0x114
#define HSP_MISC_SET_REQ_IN2			BIT(4)
#define HSP_MISC_RES_TUNE			GENMASK(1, 0)
#define RES_TUNE_PHY1_PHY2			0x1
#define RES_TUNE_PHY1				0x2
#define RES_TUNE_PHY2				0x3

/* Exynos5: USB 3.0 DRD PHY registers */
#define EXYNOS5_DRD_LINKSYSTEM			0x04
#define LINKSYSTEM_XHCI_VERSION_CONTROL		BIT(27)
#define LINKSYSTEM_FORCE_VBUSVALID		BIT(8)
#define LINKSYSTEM_FORCE_BVALID			BIT(7)
#define LINKSYSTEM_FLADJ			GENMASK(6, 1)

#define EXYNOS5_DRD_PHYUTMI			0x08
#define PHYUTMI_UTMI_SUSPEND_COM_N		BIT(12)
#define PHYUTMI_UTMI_L1_SUSPEND_COM_N		BIT(11)
#define PHYUTMI_VBUSVLDEXTSEL			BIT(10)
#define PHYUTMI_VBUSVLDEXT			BIT(9)
#define PHYUTMI_TXBITSTUFFENH			BIT(8)
#define PHYUTMI_TXBITSTUFFEN			BIT(7)
#define PHYUTMI_OTGDISABLE			BIT(6)
#define PHYUTMI_IDPULLUP			BIT(5)
#define PHYUTMI_DRVVBUS				BIT(4)
#define PHYUTMI_DPPULLDOWN			BIT(3)
#define PHYUTMI_DMPULLDOWN			BIT(2)
#define PHYUTMI_FORCESUSPEND			BIT(1)
#define PHYUTMI_FORCESLEEP			BIT(0)

#define EXYNOS5_DRD_PHYPIPE			0x0c

#define EXYNOS5_DRD_PHYCLKRST			0x10
#define PHYCLKRST_EN_UTMISUSPEND		BIT(31)
#define PHYCLKRST_SSC_REFCLKSEL			GENMASK(30, 23)
#define PHYCLKRST_SSC_RANGE			GENMASK(22, 21)
#define PHYCLKRST_SSC_EN			BIT(20)
#define PHYCLKRST_REF_SSP_EN			BIT(19)
#define PHYCLKRST_REF_CLKDIV2			BIT(18)
#define PHYCLKRST_MPLL_MULTIPLIER		GENMASK(17, 11)
#define PHYCLKRST_MPLL_MULTIPLIER_100MHZ_REF	0x19
#define PHYCLKRST_MPLL_MULTIPLIER_50M_REF	0x32
#define PHYCLKRST_MPLL_MULTIPLIER_24MHZ_REF	0x68
#define PHYCLKRST_MPLL_MULTIPLIER_20MHZ_REF	0x7d
#define PHYCLKRST_MPLL_MULTIPLIER_19200KHZ_REF	0x02
#define PHYCLKRST_FSEL_PIPE			GENMASK(10, 8)
#define PHYCLKRST_FSEL_UTMI			GENMASK(7, 5)
#define PHYCLKRST_FSEL_PAD_100MHZ		0x27
#define PHYCLKRST_FSEL_PAD_24MHZ		0x2a
#define PHYCLKRST_FSEL_PAD_20MHZ		0x31
#define PHYCLKRST_FSEL_PAD_19_2MHZ		0x38
#define PHYCLKRST_RETENABLEN			BIT(4)
#define PHYCLKRST_REFCLKSEL			GENMASK(3, 2)
#define PHYCLKRST_REFCLKSEL_PAD_REFCLK		0x2
#define PHYCLKRST_REFCLKSEL_EXT_REFCLK		0x3
#define PHYCLKRST_PORTRESET			BIT(1)
#define PHYCLKRST_COMMONONN			BIT(0)

#define EXYNOS5_DRD_PHYREG0			0x14
#define PHYREG0_SSC_REF_CLK_SEL			BIT(21)
#define PHYREG0_SSC_RANGE			BIT(20)
#define PHYREG0_CR_WRITE			BIT(19)
#define PHYREG0_CR_READ				BIT(18)
#define PHYREG0_CR_DATA_IN			GENMASK(17, 2)
#define PHYREG0_CR_CAP_DATA			BIT(1)
#define PHYREG0_CR_CAP_ADDR			BIT(0)

#define EXYNOS5_DRD_PHYREG1			0x18
#define PHYREG0_CR_DATA_OUT			GENMASK(16, 1)
#define PHYREG1_CR_ACK				BIT(0)

#define EXYNOS5_DRD_PHYPARAM0			0x1c
#define PHYPARAM0_REF_USE_PAD			BIT(31)
#define PHYPARAM0_REF_LOSLEVEL			GENMASK(30, 26)
#define PHYPARAM0_REF_LOSLEVEL_VAL		0x9
#define PHYPARAM0_TXVREFTUNE			GENMASK(25, 22)
#define PHYPARAM0_TXRISETUNE			GENMASK(21, 20)
#define PHYPARAM0_TXRESTUNE			GENMASK(19, 18)
#define PHYPARAM0_TXPREEMPPULSETUNE		BIT(17)
#define PHYPARAM0_TXPREEMPAMPTUNE		GENMASK(16, 15)
#define PHYPARAM0_TXHSXVTUNE			GENMASK(14, 13)
#define PHYPARAM0_TXFSLSTUNE			GENMASK(12, 9)
#define PHYPARAM0_SQRXTUNE			GENMASK(8, 6)
#define PHYPARAM0_OTGTUNE			GENMASK(5, 3)
#define PHYPARAM0_COMPDISTUNE			GENMASK(2, 0)

#define EXYNOS5_DRD_PHYPARAM1			0x20
#define PHYPARAM1_PCS_TXDEEMPH			GENMASK(4, 0)
#define PHYPARAM1_PCS_TXDEEMPH_VAL		0x1c

#define EXYNOS5_DRD_PHYTERM			0x24

#define EXYNOS5_DRD_PHYTEST			0x28
#define PHYTEST_POWERDOWN_SSP			BIT(3)
#define PHYTEST_POWERDOWN_HSP			BIT(2)

#define EXYNOS5_DRD_PHYADP			0x2c

#define EXYNOS5_DRD_PHYUTMICLKSEL		0x30
#define PHYUTMICLKSEL_UTMI_CLKSEL		BIT(2)

#define EXYNOS5_DRD_PHYRESUME			0x34

#define EXYNOS5_DRD_LINKPORT			0x44
#define LINKPORT_HOST_U3_PORT_DISABLE		BIT(8)
#define LINKPORT_HOST_U2_PORT_DISABLE		BIT(7)
#define LINKPORT_HOST_PORT_OVCR_U3		BIT(5)
#define LINKPORT_HOST_PORT_OVCR_U2		BIT(4)
#define LINKPORT_HOST_PORT_OVCR_U3_SEL		BIT(3)
#define LINKPORT_HOST_PORT_OVCR_U2_SEL		BIT(2)

/* USB 3.0 DRD PHY SS Function Control Reg; accessed by CR_PORT */
#define EXYNOS5_DRD_PHYSS_LOSLEVEL_OVRD_IN		(0x15)
#define LOSLEVEL_OVRD_IN_LOS_BIAS_5420			(0x5 << 13)
#define LOSLEVEL_OVRD_IN_LOS_BIAS_DEFAULT		(0x0 << 13)
#define LOSLEVEL_OVRD_IN_EN				(0x1 << 10)
#define LOSLEVEL_OVRD_IN_LOS_LEVEL_DEFAULT		(0x9 << 0)

#define EXYNOS5_DRD_PHYSS_TX_VBOOSTLEVEL_OVRD_IN	(0x12)
#define TX_VBOOSTLEVEL_OVRD_IN_VBOOST_5420		(0x5 << 13)
#define TX_VBOOSTLEVEL_OVRD_IN_VBOOST_DEFAULT		(0x4 << 13)

#define EXYNOS5_DRD_PHYSS_LANE0_TX_DEBUG		(0x1010)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_19M2_20M		(0x4 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_24M		(0x8 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_25M_26M		(0x8 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_48M_50M_52M	(0x20 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_62M5		(0x20 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_96M_100M		(0x40 << 4)

/* Exynos7870: USB DRD PHY registers */
#define EXYNOS7870_DRD_PHYPCSVAL		0x3C
#define PHYPCSVAL_PCS_RX_LOS_MASK		GENMASK(9, 0)

#define EXYNOS7870_DRD_PHYPARAM2		0x50
#define PHYPARAM2_TX_VBOOST_LVL		        GENMASK(6, 4)
#define PHYPARAM2_LOS_BIAS			GENMASK(2, 0)

#define EXYNOS7870_DRD_HSPHYCTRL		0x54
#define HSPHYCTRL_PHYSWRSTALL			BIT(31)
#define HSPHYCTRL_SIDDQ				BIT(6)
#define HSPHYCTRL_PHYSWRST			BIT(0)

#define EXYNOS7870_DRD_HSPHYPLLTUNE		0x70
#define HSPHYPLLTUNE_PLL_B_TUNE			BIT(6)
#define HSPHYPLLTUNE_PLL_I_TUNE			GENMASK(5, 4)
#define HSPHYPLLTUNE_PLL_P_TUNE			GENMASK(3, 0)

/*
 * Exynos8895: USBCON VER_01_1_1 "KC" (S:drivers/phy/phy-samsung-usb-cal.h:24), programmed as
 * the stock CAL does it. Short citation forms used below: cal.c = S:drivers/phy/
 * phy-samsung-usb3-cal.c, cal.h = S:drivers/phy/phy-samsung-usb3-cal.h, usbdrd.c =
 * S:drivers/phy/phy-exynos-usbdrd.c. Offsets are relative to the window base AFTER the
 * conditional +4 version shift (cal.c:855-869, see exynos8895_cal_enable()).
 */
#define EXYNOS8895_LINKSYSTEM_LINK_SW_RESET	BIT(28)		/* cal.h:8 */
#define EXYNOS8895_PHYPIPE_PHY_CLOCK_SEL	BIT(4)		/* cal.h:31; "1 : FREE CLOCK" cal.c:669 */
#define EXYNOS8895_PHYCLKRST_FSEL		GENMASK(10, 5)	/* cal.h:45 */
#define EXYNOS8895_PHYRESUME_DIS_LINKGATE_QACT	BIT(13)		/* cal.h:139 */
#define EXYNOS8895_PHYRESUME_DIS_ID0_QACT	BIT(12)		/* cal.h:140 */
#define EXYNOS8895_PHYRESUME_DIS_VBUSVALID_QACT	BIT(11)		/* cal.h:141 */
#define EXYNOS8895_PHYRESUME_DIS_BVALID_QACT	BIT(10)		/* cal.h:142 */
#define EXYNOS8895_PHYRESUME_FORCE_QACT		BIT(9)		/* cal.h:143 */
#define EXYNOS8895_PHYRESUME_DIS_QACT_ALL	(EXYNOS8895_PHYRESUME_DIS_LINKGATE_QACT | \
						 EXYNOS8895_PHYRESUME_DIS_ID0_QACT | \
						 EXYNOS8895_PHYRESUME_DIS_VBUSVALID_QACT | \
						 EXYNOS8895_PHYRESUME_DIS_BVALID_QACT)	/* cal.c:813-816 */
#define EXYNOS8895_DRD_PHYSELECTION		0x6c		/* cal.h:203 */
#define EXYNOS8895_PHYSEL_PIPE_CLK		BIT(3)		/* cal.h:206 */
#define EXYNOS8895_PHYSEL_PIPE			BIT(1)		/* cal.h:208 */
#define EXYNOS8895_REFCLK_CTRL			0x0c		/* FDT:7902 offset in the 0x10e50070 block */
#define EXYNOS8895_REFCLK_CTRL_SEL		BIT(24)		/* FDT:7903-7904 mask = value = 0x1000000 */

/* 26 MHz CLKCORE reference: cal.c:913-918, 600-615; refclk 0x82 = USBPHY_REFCLK_DIFF_26MHZ */
#define EXYNOS8895_REFCLKSEL_CLKCORE		0x2		/* USBPHY_REFSEL_CLKCORE, S:drivers/phy/phy-samsung-usb-cal.h:77 */
#define EXYNOS8895_FSEL_26M			0x02		/* 0x82 & 0x3f: S:drivers/phy/phy-exynos-usbdrd.h:27, cal.c:918 */
#define EXYNOS8895_MPLL_MULT_26M		0x60		/* cal.c:603 */
/*
 * cal.c:613 shifts the 9-bit value 0x108 into the 8-bit SSC_REFCLKSEL field at bit 23, so
 * bit 8 of it lands on bit 31 (EN_UTMISUSPEND). Reproduced with a raw shift, never with
 * FIELD_PREP() (which would reject 0x108).
 */
#define EXYNOS8895_SSC_REFCLKSEL_26M		0x108
/* PHYCLKRST as cal.c:906-948 writes it after PORTRESET release: every field is set by the code */
#define EXYNOS8895_PHYCLKRST_EXPECT		0x841b0058
#define EXYNOS8895_REFCLK_RATE			26000000	/* OSCCLK: S:drivers/clk/samsung/clk-exynos8895.c:34 */

/* Device-mode HS tune: usbdrd.c:396-411 defaults with the :422-431 device overrides (FDT:7896) */
#define EXYNOS8895_HS_TXVREF			0xc		/* usbdrd.c:426 */
#define EXYNOS8895_HS_TXRISE			0x1		/* usbdrd.c:402 */
#define EXYNOS8895_HS_TXRES			0x3		/* usbdrd.c:428 */
#define EXYNOS8895_HS_TXPREEMPAMP		0x3		/* usbdrd.c:427 */
#define EXYNOS8895_HS_TXHSXV			0x3		/* usbdrd.c:403 */
#define EXYNOS8895_HS_TXFSLS			0x3		/* usbdrd.c:404 */
#define EXYNOS8895_HS_SQRX			0x4		/* usbdrd.c:429 */
#define EXYNOS8895_HS_OTG			0x3		/* usbdrd.c:407 */
#define EXYNOS8895_HS_COMPDIS			0x2		/* usbdrd.c:406 */
/* SS tune: usbdrd.c:513-529 defaults with the :464-471 device overrides; set_crport_level_en = 1 (:528) */
#define EXYNOS8895_SS_TX_VBOOST			0x6		/* usbdrd.c:468 */
#define EXYNOS8895_SS_LOS_BIAS			0x5		/* usbdrd.c:524 */
#define EXYNOS8895_SS_LOS_MASK			0x104		/* usbdrd.c:525 */
#define EXYNOS8895_SS_DEEMPH_3P5DB		0x16		/* usbdrd.c:469 */
#define EXYNOS8895_SS_SWING_FULL		0x7f		/* usbdrd.c:517 */
/* SS CR-port registers and values of the stock late enable, cal.c:1027-1060 */
#define EXYNOS8895_CR_LANE0_TX_DEBUG		0x1010		/* value 0x80, cal.c:1037 */
#define EXYNOS8895_CR_TX_OVRD_DRV_LO		0x1002		/* 0x4000 | deemph << 7 | swing, cal.c:1040-1043 */
#define EXYNOS8895_CR_LOSLEVEL_OVRD_IN		0x0015		/* value 0xa409, cal.c:1056-1057 */
#define EXYNOS8895_CR_TX_VBOOSTLVL_OVRD		0x0012		/* vboost << 13, cal.c:1059-1060 */
/*
 * Host-mode HS tune (usbdrd.c:446-449 overrides on top of the :398-407 fill values; the
 * :422-431 device overrides are not applied) and the host SS boost level (usbdrd.c:515, the
 * fill value; the device override 6 of :465 belongs to device mode only).
 */
#define EXYNOS8895_HS_TXVREF_HOST		0x1		/* usbdrd.c:447 */
#define EXYNOS8895_HS_SQRX_HOST			0x3		/* usbdrd.c:405 (fill) */
#define EXYNOS8895_HS_COMPDIS_HOST		0x7		/* usbdrd.c:449 */
#define EXYNOS8895_SS_TX_VBOOST_HOST		0x4		/* usbdrd.c:515 */
/*
 * greatlte: the MAX77865 MUIC's analog switch sits between the connector D+/D- and this PHY.
 * Its single owner is the MUIC driver (drivers/usb/typec/max77865-muic.c), as in stock:
 * stock connects it from its MUIC driver on every USB attach (S:drivers/muic/universal/
 * muic_apis.c:323-334 switch_to_ap_usb -> :235-245 com_to_usb_ap) and opens it on detach and
 * at every shutdown (S:drivers/muic/universal/muic_task_max77865.c:642-643), while the PHY
 * never writes it (S:drivers/phy/phy-exynos-usbdrd.c only reads gpio_phy_port, :767-770).
 * This PHY therefore never touches CONTROL1: no COM write at .init/.exit or on an
 * orientation re-init, so a PHY exit/init pair (probe, system suspend/resume) can never
 * reopen COM under a live USB data session (spec L6G 5.2.4 fallback).
 */

/* Exynos850: USB DRD PHY registers */
#define EXYNOS850_DRD_LINKCTRL			0x04
#define LINKCTRL_FORCE_RXELECIDLE		BIT(18)
#define LINKCTRL_FORCE_PHYSTATUS		BIT(17)
#define LINKCTRL_FORCE_PIPE_EN			BIT(16)
#define LINKCTRL_FORCE_QACT			BIT(8)
#define LINKCTRL_BUS_FILTER_BYPASS		GENMASK(7, 4)

#define EXYNOS850_DRD_LINKPORT			0x08
#define LINKPORT_HOST_NUM_U3			GENMASK(19, 16)
#define LINKPORT_HOST_NUM_U2			GENMASK(15, 12)

#define EXYNOS850_DRD_CLKRST			0x20
/*
 * On versions without SS ports (like E850), bit 3 is for the 2.0 phy (HS),
 * while on versions with (like gs101), bits 2 and 3 are for the 3.0 phy (SS)
 * and bits 12 & 13 for the 2.0 phy.
 */
#define CLKRST_PHY20_SW_POR			BIT(13)
#define CLKRST_PHY20_SW_POR_SEL			BIT(12)
#define CLKRST_LINK_PCLK_SEL			BIT(7)
#define CLKRST_PHY_SW_RST			BIT(3)
#define CLKRST_PHY_RESET_SEL			BIT(2)
#define CLKRST_PORT_RST				BIT(1)
#define CLKRST_LINK_SW_RST			BIT(0)

#define EXYNOS850_DRD_SSPPLLCTL			0x30
#define SSPPLLCTL_FSEL				GENMASK(2, 0)

#define EXYNOS850_DRD_UTMI			0x50
#define UTMI_FORCE_VBUSVALID			BIT(5)
#define UTMI_FORCE_BVALID			BIT(4)
#define UTMI_DP_PULLDOWN			BIT(3)
#define UTMI_DM_PULLDOWN			BIT(2)
#define UTMI_FORCE_SUSPEND			BIT(1)
#define UTMI_FORCE_SLEEP			BIT(0)

#define EXYNOS850_DRD_HSP			0x54
#define HSP_FSV_OUT_EN				BIT(24)
#define HSP_VBUSVLDEXTSEL			BIT(13)
#define HSP_VBUSVLDEXT				BIT(12)
#define HSP_EN_UTMISUSPEND			BIT(9)
#define HSP_COMMONONN				BIT(8)

#define EXYNOS850_DRD_HSPPARACON		0x58
#define HSPPARACON_TXVREF			GENMASK(31, 28)
#define HSPPARACON_TXRISE			GENMASK(25, 24)
#define HSPPARACON_TXRES			GENMASK(22, 21)
#define HSPPARACON_TXPREEMPPULSE		BIT(20)
#define HSPPARACON_TXPREEMPAMP			GENMASK(19, 18)
#define HSPPARACON_TXHSXV			GENMASK(17, 16)
#define HSPPARACON_TXFSLS			GENMASK(15, 12)
#define HSPPARACON_SQRX				GENMASK(10, 8)
#define HSPPARACON_OTG				GENMASK(6, 4)
#define HSPPARACON_COMPDIS			GENMASK(2, 0)

#define EXYNOS850_DRD_HSP_TEST			0x5c
#define HSP_TEST_SIDDQ				BIT(24)

/* Exynos9 - GS101 */
#define EXYNOS850_DRD_SECPMACTL			0x48
#define SECPMACTL_PMA_ROPLL_REF_CLK_SEL		GENMASK(13, 12)
#define SECPMACTL_PMA_LCPLL_REF_CLK_SEL		GENMASK(11, 10)
#define SECPMACTL_PMA_REF_FREQ_SEL		GENMASK(9, 8)
#define SECPMACTL_PMA_LOW_PWR			BIT(4)
#define SECPMACTL_PMA_TRSV_SW_RST		BIT(3)
#define SECPMACTL_PMA_CMN_SW_RST		BIT(2)
#define SECPMACTL_PMA_INIT_SW_RST		BIT(1)
#define SECPMACTL_PMA_APB_SW_RST		BIT(0)

/* PMA registers */
#define EXYNOS9_PMA_USBDP_CMN_REG0008		0x0020
#define CMN_REG0008_OVRD_AUX_EN			BIT(3)
#define CMN_REG0008_AUX_EN			BIT(2)

#define EXYNOS9_PMA_USBDP_CMN_REG00B8		0x02e0
#define CMN_REG00B8_LANE_MUX_SEL_DP		GENMASK(3, 0)
#define CMN_REG00B8_LANE_MUX_SEL_DP_LANE3	BIT(3)
#define CMN_REG00B8_LANE_MUX_SEL_DP_LANE2	BIT(2)
#define CMN_REG00B8_LANE_MUX_SEL_DP_LANE1	BIT(1)
#define CMN_REG00B8_LANE_MUX_SEL_DP_LANE0	BIT(0)

#define EXYNOS9_PMA_USBDP_CMN_REG01C0		0x0700
#define CMN_REG01C0_ANA_LCPLL_LOCK_DONE		BIT(7)
#define CMN_REG01C0_ANA_LCPLL_AFC_DONE		BIT(6)

/* these have similar register layout, for lanes 0 and 2 */
#define EXYNOS9_PMA_USBDP_TRSV_REG03C3			0x0f0c
#define EXYNOS9_PMA_USBDP_TRSV_REG07C3			0x1f0c
#define TRSV_REG03C3_LN0_MON_RX_CDR_AFC_DONE		BIT(3)
#define TRSV_REG03C3_LN0_MON_RX_CDR_CAL_DONE		BIT(2)
#define TRSV_REG03C3_LN0_MON_RX_CDR_FLD_PLL_MODE_DONE	BIT(1)
#define TRSV_REG03C3_LN0_MON_RX_CDR_LOCK_DONE		BIT(0)

/* TRSV_REG0413 and TRSV_REG0813 have similar register layout */
#define EXYNOS9_PMA_USBDP_TRSV_REG0413		0x104c
#define TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN	BIT(7)
#define TRSV_REG0413_OVRD_LN1_TX_RXD_EN		BIT(5)

#define EXYNOS9_PMA_USBDP_TRSV_REG0813		0x204c
#define TRSV_REG0813_OVRD_LN3_TX_RXD_COMP_EN	BIT(7)
#define TRSV_REG0813_OVRD_LN3_TX_RXD_EN		BIT(5)

/* PCS registers */
#define EXYNOS9_PCS_NS_VEC_PS1_N1		0x010c
#define EXYNOS9_PCS_NS_VEC_PS2_N0		0x0110
#define EXYNOS9_PCS_NS_VEC_PS3_N0		0x0118
#define NS_VEC_NS_REQ				GENMASK(31, 24)
#define NS_VEC_ENABLE_TIMER			BIT(22)
#define NS_VEC_SEL_TIMEOUT			GENMASK(21, 20)
#define NS_VEC_INV_MASK				GENMASK(19, 16)
#define NS_VEC_COND_MASK			GENMASK(11, 8)
#define NS_VEC_EXP_COND				GENMASK(3, 0)

#define EXYNOS9_PCS_OUT_VEC_2			0x014c
#define EXYNOS9_PCS_OUT_VEC_3			0x0150
#define PCS_OUT_VEC_B9_DYNAMIC			BIT(19)
#define PCS_OUT_VEC_B9_SEL_OUT			BIT(18)
#define PCS_OUT_VEC_B8_DYNAMIC			BIT(17)
#define PCS_OUT_VEC_B8_SEL_OUT			BIT(16)
#define PCS_OUT_VEC_B7_DYNAMIC			BIT(15)
#define PCS_OUT_VEC_B7_SEL_OUT			BIT(14)
#define PCS_OUT_VEC_B6_DYNAMIC			BIT(13)
#define PCS_OUT_VEC_B6_SEL_OUT			BIT(12)
#define PCS_OUT_VEC_B5_DYNAMIC			BIT(11)
#define PCS_OUT_VEC_B5_SEL_OUT			BIT(10)
#define PCS_OUT_VEC_B4_DYNAMIC			BIT(9)
#define PCS_OUT_VEC_B4_SEL_OUT			BIT(8)
#define PCS_OUT_VEC_B3_DYNAMIC			BIT(7)
#define PCS_OUT_VEC_B3_SEL_OUT			BIT(6)
#define PCS_OUT_VEC_B2_DYNAMIC			BIT(5)
#define PCS_OUT_VEC_B2_SEL_OUT			BIT(4)
#define PCS_OUT_VEC_B1_DYNAMIC			BIT(3)
#define PCS_OUT_VEC_B1_SEL_OUT			BIT(2)
#define PCS_OUT_VEC_B0_DYNAMIC			BIT(1)
#define PCS_OUT_VEC_B0_SEL_OUT			BIT(0)

#define EXYNOS9_PCS_TIMEOUT_0			0x0170

#define EXYNOS9_PCS_TIMEOUT_3			0x017c

#define EXYNOS9_PCS_EBUF_PARAM			0x0304
#define EBUF_PARAM_SKP_REMOVE_TH_EMPTY_MODE	GENMASK(29, 24)

#define EXYNOS9_PCS_BACK_END_MODE_VEC		0x030c
#define BACK_END_MODE_VEC_FORCE_EBUF_EMPTY_MODE	BIT(1)
#define BACK_END_MODE_VEC_DISABLE_DATA_MASK	BIT(0)

#define EXYNOS9_PCS_RX_CONTROL			0x03f0
#define RX_CONTROL_EN_BLOCK_ALIGNER_TYPE_B	BIT(22)

#define EXYNOS9_PCS_RX_CONTROL_DEBUG		0x03f4
#define RX_CONTROL_DEBUG_EN_TS_CHECK		BIT(5)
#define RX_CONTROL_DEBUG_NUM_COM_FOUND		GENMASK(3, 0)

#define EXYNOS9_PCS_LOCAL_COEF			0x040c
#define LOCAL_COEF_PMA_CENTER_COEF		GENMASK(21, 16)
#define LOCAL_COEF_LF				GENMASK(13, 8)
#define LOCAL_COEF_FS				GENMASK(5, 0)

#define EXYNOS9_PCS_HS_TX_COEF_MAP_0		0x0410
#define HS_TX_COEF_MAP_0_SSTX_DEEMP		GENMASK(17, 12)
#define HS_TX_COEF_MAP_0_SSTX_LEVEL		GENMASK(11, 6)
#define HS_TX_COEF_MAP_0_SSTX_PRE_SHOOT		GENMASK(5, 0)


#define KHZ	1000
#define MHZ	(KHZ * KHZ)

#define PHY_TUNING_ENTRY_PHY(o, m, v) {	\
		.off = (o),		\
		.mask = (m),		\
		.val = (v),		\
		.region = PTR_PHY	\
	}

#define PHY_TUNING_ENTRY_PCS(o, m, v) {	\
		.off = (o),		\
		.mask = (m),		\
		.val = (v),		\
		.region = PTR_PCS	\
	}

#define PHY_TUNING_ENTRY_PMA(o, m, v) {	\
		.off = (o),		\
		.mask = (m),		\
		.val = (v),		\
		.region = PTR_PMA,	\
	}

#define PHY_TUNING_ENTRY_LAST { .region = PTR_INVALID }

#define for_each_phy_tune(tune) \
	for (; (tune)->region != PTR_INVALID; ++(tune))

struct exynos5_usbdrd_phy_tuning {
	u32 off;
	u32 mask;
	u32 val;
	char region;
#define PTR_INVALID	0
#define PTR_PHY		1
#define PTR_PCS		2
#define PTR_PMA		3
};

enum exynos5_usbdrd_phy_tuning_state {
	PTS_UTMI_POSTINIT,
	PTS_PIPE3_PREINIT,
	PTS_PIPE3_INIT,
	PTS_PIPE3_POSTINIT,
	PTS_PIPE3_POSTLOCK,
	PTS_MAX,
};

enum exynos5_usbdrd_phy_id {
	EXYNOS5_DRDPHY_UTMI,
	EXYNOS5_DRDPHY_PIPE3,
	EXYNOS5_DRDPHYS_NUM,
};

struct phy_usb_instance;
struct exynos5_usbdrd_phy;

struct exynos5_usbdrd_phy_config {
	u32 id;
	void (*phy_isol)(struct phy_usb_instance *inst, bool isolate);
	void (*phy_init)(struct exynos5_usbdrd_phy *phy_drd);
	unsigned int (*set_refclk)(struct phy_usb_instance *inst);
};

struct exynos5_usbdrd_phy_drvdata {
	const struct exynos5_usbdrd_phy_config *phy_cfg;
	const struct exynos5_usbdrd_phy_tuning **phy_tunes;
	const struct phy_ops *phy_ops;
	const char * const *clk_names;
	int n_clks;
	const char * const *core_clk_names;
	int n_core_clks;
	const char * const *regulator_names;
	int n_regulators;
	u32 pmu_offset_usbdrd0_phy;
	u32 pmu_offset_usbdrd0_phy_ss;
	u32 pmu_offset_usbdrd1_phy;
	int (*probe_extra)(struct exynos5_usbdrd_phy *phy_drd,
			   struct platform_device *pdev);
	int (*orien_sw_set)(struct typec_switch_dev *sw,
			    enum typec_orientation orientation);
};

/**
 * struct exynos5_usbdrd_phy - driver data for USB 3.0 PHY
 * @dev: pointer to device instance of this platform device
 * @reg_phy: usb phy controller register memory base
 * @reg_pcs: usb phy physical coding sublayer register memory base
 * @reg_pma: usb phy physical media attachment register memory base
 * @clks: clocks for register access
 * @core_clks: core clocks for phy (ref, pipe3, utmi+, ITP, etc. as required)
 * @drv_data: pointer to SoC level driver data structure
 * @hs_phy: pointer to non-Samsung IP high-speed phy controller
 * @phy_mutex: mutex protecting phy_init/exit & TCPC callbacks
 * @phys: array for 'EXYNOS5_DRDPHYS_NUM' number of PHY
 *	    instances each with its 'phy' and 'phy_cfg'.
 * @extrefclk: frequency select settings when using 'separate
 *	       reference clocks' for SS and HS operations
 * @regulators: regulators for phy
 * @sw: TypeC orientation switch handle
 * @orientation: TypeC connector orientation - normal or flipped
 * @reg_phy2: Exynos8895 second PHY window (stock regs_base_2nd)
 * @reg_refclk: Exynos8895 reference-clock select block
 * @cal_base: Exynos8895 window bases after the CAL version shift
 * @port_gpio: Exynos8895 Type-C orientation input
 * @vdd_dp: Exynos8895 optional 1.8 V DP/PHY rail
 * @cal_ver: Exynos8895 offset-0 words of both windows, read at the first init
 * @cal_probed: Exynos8895 CAL version probed (stock hw_version != 0)
 * @cal_zero: Exynos8895 offset 0 of window 0 read 0 (no shift, stock port -1)
 * @cal_port: Exynos8895 port in use: 0, 1 (flipped) or -1 (stock used_phy_port)
 * @sw_port: Exynos8895 port from the Type-C orientation switch, used by the next CAL enable
 * @cal_inited: Exynos8895 PIPE3 .init done and .exit not yet (under @phy_mutex)
 * @cr_timeouts: Exynos8895 CR-port handshake timeouts
 * @init_count: Exynos8895 number of .init calls
 * @retunes: Exynos8895 re-tunes done from the dwc3 soft-reset hook
 * @tune_armed: Exynos8895 set by .power_on, cleared by .exit: the hook re-tunes only when set
 * @host_mode: Exynos8895 last mode dwc3 asked for with phy_set_mode() (L6G 5.3.2)
 */
struct exynos5_usbdrd_phy {
	struct device *dev;
	void __iomem *reg_phy;
	void __iomem *reg_pcs;
	void __iomem *reg_pma;
	struct clk_bulk_data *clks;
	struct clk_bulk_data *core_clks;
	const struct exynos5_usbdrd_phy_drvdata *drv_data;
	struct phy *hs_phy;
	struct mutex phy_mutex;
	struct phy_usb_instance {
		struct phy *phy;
		u32 index;
		struct regmap *reg_pmu;
		u32 pmu_offset;
		const struct exynos5_usbdrd_phy_config *phy_cfg;
	} phys[EXYNOS5_DRDPHYS_NUM];
	u32 extrefclk;
	struct regulator_bulk_data *regulators;

	struct typec_switch_dev *sw;
	enum typec_orientation orientation;

	/* Exynos8895 CAL state (S:drivers/phy/phy-samsung-usb-cal.h:186-222 exynos_usbphy_info) */
	void __iomem *reg_phy2;
	void __iomem *reg_refclk;
	void __iomem *cal_base[2];
	struct gpio_desc *port_gpio;
	struct regulator *vdd_dp;
	u32 cal_ver[2];
	bool cal_probed;
	bool cal_zero;
	int cal_port;
	int sw_port;
	bool cal_inited;
	unsigned int cr_timeouts;
	unsigned int init_count;
	unsigned int retunes;
	bool tune_armed;
	bool host_mode;
};

static inline
struct exynos5_usbdrd_phy *to_usbdrd_phy(struct phy_usb_instance *inst)
{
	return container_of((inst), struct exynos5_usbdrd_phy,
			    phys[(inst)->index]);
}

/*
 * exynos5_rate_to_clk() converts the supplied clock rate to the value that
 * can be written to the phy register.
 */
static unsigned int exynos5_rate_to_clk(unsigned long rate, u32 *reg)
{
	/* EXYNOS5_FSEL_MASK */

	switch (rate) {
	case 9600 * KHZ:
		*reg = EXYNOS5_FSEL_9MHZ6;
		break;
	case 10 * MHZ:
		*reg = EXYNOS5_FSEL_10MHZ;
		break;
	case 12 * MHZ:
		*reg = EXYNOS5_FSEL_12MHZ;
		break;
	case 19200 * KHZ:
		*reg = EXYNOS5_FSEL_19MHZ2;
		break;
	case 20 * MHZ:
		*reg = EXYNOS5_FSEL_20MHZ;
		break;
	case 24 * MHZ:
		*reg = EXYNOS5_FSEL_24MHZ;
		break;
	case 26 * MHZ:
		*reg = EXYNOS5_FSEL_26MHZ;
		break;
	case 50 * MHZ:
		*reg = EXYNOS5_FSEL_50MHZ;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void exynos5_usbdrd_phy_isol(struct phy_usb_instance *inst,
				    bool isolate)
{
	unsigned int val;

	if (!inst->reg_pmu)
		return;

	val = isolate ? 0 : EXYNOS4_PHY_ENABLE;

	regmap_update_bits(inst->reg_pmu, inst->pmu_offset,
			   EXYNOS4_PHY_ENABLE, val);
}

/*
 * Sets the pipe3 phy's clk as EXTREFCLK (XXTI) which is internal clock
 * from clock core. Further sets multiplier values and spread spectrum
 * clock settings for SuperSpeed operations.
 */
static unsigned int
exynos5_usbdrd_pipe3_set_refclk(struct phy_usb_instance *inst)
{
	u32 reg;
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	/* restore any previous reference clock settings */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	/* Use EXTREFCLK as ref clock */
	reg &= ~PHYCLKRST_REFCLKSEL;
	reg |= FIELD_PREP(PHYCLKRST_REFCLKSEL, PHYCLKRST_REFCLKSEL_EXT_REFCLK);

	/* FSEL settings corresponding to reference clock */
	reg &= ~(PHYCLKRST_FSEL_PIPE |
		 PHYCLKRST_MPLL_MULTIPLIER |
		 PHYCLKRST_SSC_REFCLKSEL);
	switch (phy_drd->extrefclk) {
	case EXYNOS5_FSEL_50MHZ:
		reg |= (FIELD_PREP(PHYCLKRST_SSC_REFCLKSEL, 0x00) |
			FIELD_PREP(PHYCLKRST_MPLL_MULTIPLIER,
				   PHYCLKRST_MPLL_MULTIPLIER_50M_REF));
		break;
	case EXYNOS5_FSEL_24MHZ:
		reg |= (FIELD_PREP(PHYCLKRST_SSC_REFCLKSEL, 0x88) |
			FIELD_PREP(PHYCLKRST_MPLL_MULTIPLIER,
				   PHYCLKRST_MPLL_MULTIPLIER_24MHZ_REF));
		break;
	case EXYNOS5_FSEL_20MHZ:
		reg |= (FIELD_PREP(PHYCLKRST_SSC_REFCLKSEL, 0x00) |
			FIELD_PREP(PHYCLKRST_MPLL_MULTIPLIER,
				   PHYCLKRST_MPLL_MULTIPLIER_20MHZ_REF));
		break;
	case EXYNOS5_FSEL_19MHZ2:
		reg |= (FIELD_PREP(PHYCLKRST_SSC_REFCLKSEL, 0x88) |
			FIELD_PREP(PHYCLKRST_MPLL_MULTIPLIER,
				   PHYCLKRST_MPLL_MULTIPLIER_19200KHZ_REF));
		break;
	default:
		dev_dbg(phy_drd->dev, "unsupported ref clk\n");
		break;
	}

	return reg;
}

/*
 * Sets the utmi phy's clk as EXTREFCLK (XXTI) which is internal clock
 * from clock core. Further sets the FSEL values for HighSpeed operations.
 */
static unsigned int
exynos5_usbdrd_utmi_set_refclk(struct phy_usb_instance *inst)
{
	u32 reg;
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	/* restore any previous reference clock settings */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	reg &= ~PHYCLKRST_REFCLKSEL;
	reg |= FIELD_PREP(PHYCLKRST_REFCLKSEL, PHYCLKRST_REFCLKSEL_EXT_REFCLK);

	reg &= ~(PHYCLKRST_FSEL_UTMI |
		 PHYCLKRST_MPLL_MULTIPLIER |
		 PHYCLKRST_SSC_REFCLKSEL);
	reg |= FIELD_PREP(PHYCLKRST_FSEL_UTMI, phy_drd->extrefclk);

	return reg;
}

static void
exynos5_usbdrd_apply_phy_tunes(struct exynos5_usbdrd_phy *phy_drd,
			       enum exynos5_usbdrd_phy_tuning_state state)
{
	const struct exynos5_usbdrd_phy_tuning *tune;

	tune = phy_drd->drv_data->phy_tunes[state];
	if (!tune)
		return;

	for_each_phy_tune(tune) {
		void __iomem *reg_base;
		u32 reg = 0;

		switch (tune->region) {
		case PTR_PHY:
			reg_base = phy_drd->reg_phy;
			break;
		case PTR_PCS:
			reg_base = phy_drd->reg_pcs;
			break;
		case PTR_PMA:
			reg_base = phy_drd->reg_pma;
			break;
		default:
			dev_warn_once(phy_drd->dev,
				      "unknown phy region %d\n", tune->region);
			continue;
		}

		if (~tune->mask) {
			reg = readl(reg_base + tune->off);
			reg &= ~tune->mask;
		}
		reg |= tune->val;
		writel(reg, reg_base + tune->off);
	}
}

static void exynos5_usbdrd_pipe3_init(struct exynos5_usbdrd_phy *phy_drd)
{
	u32 reg;

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM1);
	/* Set Tx De-Emphasis level */
	reg &= ~PHYPARAM1_PCS_TXDEEMPH;
	reg |= FIELD_PREP(PHYPARAM1_PCS_TXDEEMPH, PHYPARAM1_PCS_TXDEEMPH_VAL);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM1);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);
	reg &= ~PHYTEST_POWERDOWN_SSP;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);
}

static void
exynos5_usbdrd_usbdp_g2_v4_ctrl_pma_ready(struct exynos5_usbdrd_phy *phy_drd)
{
	void __iomem *regs_base = phy_drd->reg_phy;
	u32 reg;

	/* link pipe_clock selection to pclk of PMA */
	reg = readl(regs_base + EXYNOS850_DRD_CLKRST);
	reg |= CLKRST_LINK_PCLK_SEL;
	writel(reg, regs_base + EXYNOS850_DRD_CLKRST);

	reg = readl(regs_base + EXYNOS850_DRD_SECPMACTL);
	reg &= ~SECPMACTL_PMA_REF_FREQ_SEL;
	reg |= FIELD_PREP(SECPMACTL_PMA_REF_FREQ_SEL, 1);
	/* SFR reset */
	reg |= (SECPMACTL_PMA_LOW_PWR | SECPMACTL_PMA_APB_SW_RST);
	reg &= ~(SECPMACTL_PMA_ROPLL_REF_CLK_SEL |
		 SECPMACTL_PMA_LCPLL_REF_CLK_SEL);
	/* PMA power off */
	reg |= (SECPMACTL_PMA_TRSV_SW_RST | SECPMACTL_PMA_CMN_SW_RST |
		SECPMACTL_PMA_INIT_SW_RST);
	writel(reg, regs_base + EXYNOS850_DRD_SECPMACTL);

	udelay(1);

	reg = readl(regs_base + EXYNOS850_DRD_SECPMACTL);
	reg &= ~SECPMACTL_PMA_LOW_PWR;
	writel(reg, regs_base + EXYNOS850_DRD_SECPMACTL);

	udelay(1);

	/* release override */
	reg = readl(regs_base + EXYNOS850_DRD_LINKCTRL);
	reg &= ~LINKCTRL_FORCE_PIPE_EN;
	writel(reg, regs_base + EXYNOS850_DRD_LINKCTRL);

	udelay(1);

	/* APB enable */
	reg = readl(regs_base + EXYNOS850_DRD_SECPMACTL);
	reg &= ~SECPMACTL_PMA_APB_SW_RST;
	writel(reg, regs_base + EXYNOS850_DRD_SECPMACTL);
}

static void
exynos5_usbdrd_usbdp_g2_v4_pma_lane_mux_sel(struct exynos5_usbdrd_phy *phy_drd)
{
	void __iomem *regs_base = phy_drd->reg_pma;
	u32 reg;

	/* lane configuration: USB on all lanes */
	reg = readl(regs_base + EXYNOS9_PMA_USBDP_CMN_REG00B8);
	reg &= ~CMN_REG00B8_LANE_MUX_SEL_DP;
	/*
	 * USB on lanes 0 & 1 in normal mode, or 2 & 3 if reversed, DP on the
	 * other ones.
	 */
	reg |= FIELD_PREP(CMN_REG00B8_LANE_MUX_SEL_DP,
			  ((phy_drd->orientation == TYPEC_ORIENTATION_NORMAL)
			   ? (CMN_REG00B8_LANE_MUX_SEL_DP_LANE3
			      | CMN_REG00B8_LANE_MUX_SEL_DP_LANE2)
			   : (CMN_REG00B8_LANE_MUX_SEL_DP_LANE1
			      | CMN_REG00B8_LANE_MUX_SEL_DP_LANE0)));
	writel(reg, regs_base + EXYNOS9_PMA_USBDP_CMN_REG00B8);

	/* override of TX receiver detector and comparator: lane 1 */
	reg = readl(regs_base + EXYNOS9_PMA_USBDP_TRSV_REG0413);
	if (phy_drd->orientation == TYPEC_ORIENTATION_NORMAL) {
		reg &= ~TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN;
		reg &= ~TRSV_REG0413_OVRD_LN1_TX_RXD_EN;
	} else {
		reg |= TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN;
		reg |= TRSV_REG0413_OVRD_LN1_TX_RXD_EN;
	}
	writel(reg, regs_base + EXYNOS9_PMA_USBDP_TRSV_REG0413);

	/* lane 3 */
	reg = readl(regs_base + EXYNOS9_PMA_USBDP_TRSV_REG0813);
	if (phy_drd->orientation == TYPEC_ORIENTATION_NORMAL) {
		reg |= TRSV_REG0813_OVRD_LN3_TX_RXD_COMP_EN;
		reg |= TRSV_REG0813_OVRD_LN3_TX_RXD_EN;
	} else {
		reg &= ~TRSV_REG0813_OVRD_LN3_TX_RXD_COMP_EN;
		reg &= ~TRSV_REG0813_OVRD_LN3_TX_RXD_EN;
	}
	writel(reg, regs_base + EXYNOS9_PMA_USBDP_TRSV_REG0813);
}

static int
exynos5_usbdrd_usbdp_g2_v4_pma_check_pll_lock(struct exynos5_usbdrd_phy *phy_drd)
{
	static const unsigned int timeout_us = 40000;
	static const unsigned int sleep_us = 40;
	static const u32 locked = (CMN_REG01C0_ANA_LCPLL_LOCK_DONE |
				   CMN_REG01C0_ANA_LCPLL_AFC_DONE);
	u32 reg;
	int err;

	err = readl_poll_timeout(
			phy_drd->reg_pma + EXYNOS9_PMA_USBDP_CMN_REG01C0,
			reg, (reg & locked) == locked, sleep_us, timeout_us);
	if (err)
		dev_err(phy_drd->dev,
			"timed out waiting for PLL lock: %#.8x\n", reg);

	return err;
}

static void
exynos5_usbdrd_usbdp_g2_v4_pma_check_cdr_lock(struct exynos5_usbdrd_phy *phy_drd)
{
	static const unsigned int timeout_us = 40000;
	static const unsigned int sleep_us = 40;
	static const u32 locked =
		(TRSV_REG03C3_LN0_MON_RX_CDR_AFC_DONE
		 | TRSV_REG03C3_LN0_MON_RX_CDR_CAL_DONE
		 | TRSV_REG03C3_LN0_MON_RX_CDR_FLD_PLL_MODE_DONE
		 | TRSV_REG03C3_LN0_MON_RX_CDR_LOCK_DONE);
	u32 reg;
	int err;

	err = readl_poll_timeout(
			/* lane depends on cable orientation */
			(phy_drd->reg_pma
			 + ((phy_drd->orientation == TYPEC_ORIENTATION_NORMAL)
			    ? EXYNOS9_PMA_USBDP_TRSV_REG03C3
			    : EXYNOS9_PMA_USBDP_TRSV_REG07C3)),
			reg, (reg & locked) == locked, sleep_us, timeout_us);
	if (err)
		dev_err(phy_drd->dev,
			"timed out waiting for CDR(l%d) lock: %#.8x\n",
			((phy_drd->orientation == TYPEC_ORIENTATION_NORMAL)
			 ? 0
			 : 2), reg);
}

static void exynos5_usbdrd_utmi_init(struct exynos5_usbdrd_phy *phy_drd)
{
	u32 reg;

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM0);
	/* Set Loss-of-Signal Detector sensitivity */
	reg &= ~PHYPARAM0_REF_LOSLEVEL;
	reg |= FIELD_PREP(PHYPARAM0_REF_LOSLEVEL, PHYPARAM0_REF_LOSLEVEL_VAL);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM0);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM1);
	/* Set Tx De-Emphasis level */
	reg &= ~PHYPARAM1_PCS_TXDEEMPH;
	reg |= FIELD_PREP(PHYPARAM1_PCS_TXDEEMPH, PHYPARAM1_PCS_TXDEEMPH_VAL);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM1);

	/* UTMI Power Control */
	writel(PHYUTMI_OTGDISABLE, phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);
	reg &= ~PHYTEST_POWERDOWN_HSP;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);
}

static int exynos5_usbdrd_phy_init(struct phy *phy)
{
	int ret;
	u32 reg;
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;

	/* Reset USB 3.0 PHY */
	writel(0x0, phy_drd->reg_phy + EXYNOS5_DRD_PHYREG0);
	writel(0x0, phy_drd->reg_phy + EXYNOS5_DRD_PHYRESUME);

	/*
	 * Setting the Frame length Adj value[6:1] to default 0x20
	 * See xHCI 1.0 spec, 5.2.4
	 */
	reg =	LINKSYSTEM_XHCI_VERSION_CONTROL |
		FIELD_PREP(LINKSYSTEM_FLADJ, 0x20);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_LINKSYSTEM);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM0);
	/* Select PHY CLK source */
	reg &= ~PHYPARAM0_REF_USE_PAD;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM0);

	/* This bit must be set for both HS and SS operations */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMICLKSEL);
	reg |= PHYUTMICLKSEL_UTMI_CLKSEL;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMICLKSEL);

	/* UTMI or PIPE3 specific init */
	inst->phy_cfg->phy_init(phy_drd);

	/* reference clock settings */
	reg = inst->phy_cfg->set_refclk(inst);

		/* Digital power supply in normal operating mode */
	reg |=	PHYCLKRST_RETENABLEN |
		/* Enable ref clock for SS function */
		PHYCLKRST_REF_SSP_EN |
		/* Enable spread spectrum */
		PHYCLKRST_SSC_EN |
		/* Power down HS Bias and PLL blocks in suspend mode */
		PHYCLKRST_COMMONONN |
		/* Reset the port */
		PHYCLKRST_PORTRESET;

	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	fsleep(10);

	reg &= ~PHYCLKRST_PORTRESET;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	return 0;
}

static int exynos5_usbdrd_phy_exit(struct phy *phy)
{
	int ret;
	u32 reg;
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;

	reg =	PHYUTMI_OTGDISABLE |
		PHYUTMI_FORCESUSPEND |
		PHYUTMI_FORCESLEEP;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);

	/* Resetting the PHYCLKRST enable bits to reduce leakage current */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);
	reg &= ~(PHYCLKRST_REF_SSP_EN |
		 PHYCLKRST_SSC_EN |
		 PHYCLKRST_COMMONONN);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	/* Control PHYTEST to remove leakage current */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);
	reg |=	PHYTEST_POWERDOWN_SSP |
		PHYTEST_POWERDOWN_HSP;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	return 0;
}

static int exynos5_usbdrd_phy_power_on(struct phy *phy)
{
	int ret;
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	dev_dbg(phy_drd->dev, "Request to power_on usbdrd_phy phy\n");

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_core_clks,
				      phy_drd->core_clks);
	if (ret)
		return ret;

	/* Enable VBUS supply */
	ret = regulator_bulk_enable(phy_drd->drv_data->n_regulators,
				    phy_drd->regulators);
	if (ret) {
		dev_err(phy_drd->dev, "Failed to enable PHY regulator(s)\n");
		goto fail_vbus;
	}

	/* Power-on PHY */
	inst->phy_cfg->phy_isol(inst, false);

	return 0;

fail_vbus:
	clk_bulk_disable_unprepare(phy_drd->drv_data->n_core_clks,
				   phy_drd->core_clks);

	return ret;
}

static int exynos5_usbdrd_phy_power_off(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	dev_dbg(phy_drd->dev, "Request to power_off usbdrd_phy phy\n");

	/* Power-off the PHY */
	inst->phy_cfg->phy_isol(inst, true);

	/* Disable VBUS supply */
	regulator_bulk_disable(phy_drd->drv_data->n_regulators,
			       phy_drd->regulators);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_core_clks,
				   phy_drd->core_clks);

	return 0;
}

static int crport_handshake(struct exynos5_usbdrd_phy *phy_drd,
			    u32 val, u32 cmd)
{
	unsigned int result;
	int err;

	writel(val | cmd, phy_drd->reg_phy + EXYNOS5_DRD_PHYREG0);

	err = readl_poll_timeout(phy_drd->reg_phy + EXYNOS5_DRD_PHYREG1,
				 result, (result & PHYREG1_CR_ACK), 1, 100);
	if (err == -ETIMEDOUT) {
		dev_err(phy_drd->dev, "CRPORT handshake timeout1 (0x%08x)\n", val);
		return err;
	}

	writel(val, phy_drd->reg_phy + EXYNOS5_DRD_PHYREG0);

	err = readl_poll_timeout(phy_drd->reg_phy + EXYNOS5_DRD_PHYREG1,
				 result, !(result & PHYREG1_CR_ACK), 1, 100);
	if (err == -ETIMEDOUT) {
		dev_err(phy_drd->dev, "CRPORT handshake timeout2 (0x%08x)\n", val);
		return err;
	}

	return 0;
}

static int crport_ctrl_write(struct exynos5_usbdrd_phy *phy_drd,
			     u32 addr, u32 data)
{
	u32 val;
	int ret;

	/* Write Address */
	val = FIELD_PREP(PHYREG0_CR_DATA_IN, addr);
	writel(val, phy_drd->reg_phy + EXYNOS5_DRD_PHYREG0);
	ret = crport_handshake(phy_drd, val, PHYREG0_CR_CAP_ADDR);
	if (ret)
		return ret;

	/* Write Data */
	val = FIELD_PREP(PHYREG0_CR_DATA_IN, data);
	writel(val, phy_drd->reg_phy + EXYNOS5_DRD_PHYREG0);
	ret = crport_handshake(phy_drd, val, PHYREG0_CR_CAP_DATA);
	if (ret)
		return ret;

	ret = crport_handshake(phy_drd, val, PHYREG0_CR_WRITE);

	return ret;
}

/*
 * Calibrate few PHY parameters using CR_PORT register to meet
 * SuperSpeed requirements on Exynos5420 and Exynos5800 systems,
 * which have 28nm USB 3.0 DRD PHY.
 */
static int exynos5420_usbdrd_phy_calibrate(struct exynos5_usbdrd_phy *phy_drd)
{
	unsigned int temp;
	int ret = 0;

	/*
	 * Change los_bias to (0x5) for 28nm PHY from a
	 * default value (0x0); los_level is set as default
	 * (0x9) as also reflected in los_level[30:26] bits
	 * of PHYPARAM0 register.
	 */
	temp = LOSLEVEL_OVRD_IN_LOS_BIAS_5420 |
		LOSLEVEL_OVRD_IN_EN |
		LOSLEVEL_OVRD_IN_LOS_LEVEL_DEFAULT;
	ret = crport_ctrl_write(phy_drd,
				EXYNOS5_DRD_PHYSS_LOSLEVEL_OVRD_IN,
				temp);
	if (ret) {
		dev_err(phy_drd->dev,
			"Failed setting Loss-of-Signal level for SuperSpeed\n");
		return ret;
	}

	/*
	 * Set tx_vboost_lvl to (0x5) for 28nm PHY Tuning,
	 * to raise Tx signal level from its default value of (0x4)
	 */
	temp = TX_VBOOSTLEVEL_OVRD_IN_VBOOST_5420;
	ret = crport_ctrl_write(phy_drd,
				EXYNOS5_DRD_PHYSS_TX_VBOOSTLEVEL_OVRD_IN,
				temp);
	if (ret) {
		dev_err(phy_drd->dev,
			"Failed setting Tx-Vboost-Level for SuperSpeed\n");
		return ret;
	}

	/*
	 * Set proper time to wait for RxDetect measurement, for
	 * desired reference clock of PHY, by tuning the CR_PORT
	 * register LANE0.TX_DEBUG which is internal to PHY.
	 * This fixes issue with few USB 3.0 devices, which are
	 * not detected (not even generate interrupts on the bus
	 * on insertion) without this change.
	 * e.g. Samsung SUM-TSB16S 3.0 USB drive.
	 */
	switch (phy_drd->extrefclk) {
	case EXYNOS5_FSEL_50MHZ:
		temp = LANE0_TX_DEBUG_RXDET_MEAS_TIME_48M_50M_52M;
		break;
	case EXYNOS5_FSEL_20MHZ:
	case EXYNOS5_FSEL_19MHZ2:
		temp = LANE0_TX_DEBUG_RXDET_MEAS_TIME_19M2_20M;
		break;
	case EXYNOS5_FSEL_24MHZ:
	default:
		temp = LANE0_TX_DEBUG_RXDET_MEAS_TIME_24M;
		break;
	}

	ret = crport_ctrl_write(phy_drd,
				EXYNOS5_DRD_PHYSS_LANE0_TX_DEBUG,
				temp);
	if (ret)
		dev_err(phy_drd->dev,
			"Fail to set RxDet measurement time for SuperSpeed\n");

	return ret;
}

static struct phy *exynos5_usbdrd_phy_xlate(struct device *dev,
					const struct of_phandle_args *args)
{
	struct exynos5_usbdrd_phy *phy_drd = dev_get_drvdata(dev);

	if (WARN_ON(args->args[0] >= EXYNOS5_DRDPHYS_NUM))
		return ERR_PTR(-ENODEV);

	return phy_drd->phys[args->args[0]].phy;
}

static int exynos5_usbdrd_phy_calibrate(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	if (inst->phy_cfg->id == EXYNOS5_DRDPHY_UTMI)
		return exynos5420_usbdrd_phy_calibrate(phy_drd);
	return 0;
}

static const struct phy_ops exynos5_usbdrd_phy_ops = {
	.init		= exynos5_usbdrd_phy_init,
	.exit		= exynos5_usbdrd_phy_exit,
	.power_on	= exynos5_usbdrd_phy_power_on,
	.power_off	= exynos5_usbdrd_phy_power_off,
	.calibrate	= exynos5_usbdrd_phy_calibrate,
	.owner		= THIS_MODULE,
};

static void exynos7870_usbdrd_phy_isol(struct phy_usb_instance *inst,
				       bool isolate)
{
	unsigned int val;

	if (!inst->reg_pmu)
		return;

	val = isolate ? 0 : EXYNOS7870_USB2PHY_ENABLE;

	regmap_update_bits(inst->reg_pmu, inst->pmu_offset,
			   EXYNOS7870_USB2PHY_ENABLE, val);
}

static void exynos7870_usbdrd_utmi_init(struct exynos5_usbdrd_phy *phy_drd)
{
	u32 reg;

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);
	/* Use PADREFCLK as ref clock */
	reg &= ~PHYCLKRST_REFCLKSEL;
	reg |= FIELD_PREP(PHYCLKRST_REFCLKSEL, PHYCLKRST_REFCLKSEL_PAD_REFCLK);
	/* Select ref clock rate */
	reg &= ~PHYCLKRST_FSEL_UTMI;
	reg &= ~PHYCLKRST_FSEL_PIPE;
	reg |= FIELD_PREP(PHYCLKRST_FSEL_UTMI, phy_drd->extrefclk);
	/* Enable suspend and reset the port */
	reg |= PHYCLKRST_EN_UTMISUSPEND;
	reg |= PHYCLKRST_COMMONONN;
	reg |= PHYCLKRST_PORTRESET;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);
	udelay(10);

	/* Clear the port reset bit */
	reg &= ~PHYCLKRST_PORTRESET;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	/* Change PHY PLL tune value */
	reg = readl(phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYPLLTUNE);
	if (phy_drd->extrefclk == EXYNOS5_FSEL_24MHZ)
		reg |= HSPHYPLLTUNE_PLL_B_TUNE;
	else
		reg &= ~HSPHYPLLTUNE_PLL_B_TUNE;
	reg &= ~HSPHYPLLTUNE_PLL_P_TUNE;
	reg |= FIELD_PREP(HSPHYPLLTUNE_PLL_P_TUNE, 14);
	writel(reg, phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYPLLTUNE);

	/* High-Speed PHY control */
	reg = readl(phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYCTRL);
	reg &= ~HSPHYCTRL_SIDDQ;
	reg &= ~HSPHYCTRL_PHYSWRST;
	reg &= ~HSPHYCTRL_PHYSWRSTALL;
	writel(reg, phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYCTRL);
	udelay(500);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_LINKSYSTEM);
	/*
	 * Setting the Frame length Adj value[6:1] to default 0x20
	 * See xHCI 1.0 spec, 5.2.4
	 */
	reg |= LINKSYSTEM_XHCI_VERSION_CONTROL;
	reg &= ~LINKSYSTEM_FLADJ;
	reg |= FIELD_PREP(LINKSYSTEM_FLADJ, 0x20);
	/* Set VBUSVALID signal as the VBUS pad is not used */
	reg |= LINKSYSTEM_FORCE_BVALID;
	reg |= LINKSYSTEM_FORCE_VBUSVALID;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_LINKSYSTEM);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);
	/* Release force_sleep & force_suspend */
	reg &= ~PHYUTMI_FORCESLEEP;
	reg &= ~PHYUTMI_FORCESUSPEND;
	/* DP/DM pull down control */
	reg &= ~PHYUTMI_DMPULLDOWN;
	reg &= ~PHYUTMI_DPPULLDOWN;
	reg &= ~PHYUTMI_DRVVBUS;
	/* Set DP-pull up as the VBUS pad is not used */
	reg |= PHYUTMI_VBUSVLDEXTSEL;
	reg |= PHYUTMI_VBUSVLDEXT;
	/* Disable OTG block and VBUS valid comparator */
	reg |= PHYUTMI_OTGDISABLE;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);

	/* Configure OVC IO usage */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_LINKPORT);
	reg |= LINKPORT_HOST_PORT_OVCR_U3_SEL | LINKPORT_HOST_PORT_OVCR_U2_SEL;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_LINKPORT);

	/* High-Speed PHY swrst */
	reg = readl(phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYCTRL);
	reg |= HSPHYCTRL_PHYSWRST;
	writel(reg, phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYCTRL);
	udelay(20);

	/* Clear the PHY swrst bit */
	reg = readl(phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYCTRL);
	reg &= ~HSPHYCTRL_PHYSWRST;
	writel(reg, phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYCTRL);

	if (phy_drd->drv_data->phy_tunes)
		exynos5_usbdrd_apply_phy_tunes(phy_drd,
					       PTS_UTMI_POSTINIT);
}

static int exynos7870_usbdrd_phy_init(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);
	int ret;

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;

	/* UTMI or PIPE3 specific init */
	inst->phy_cfg->phy_init(phy_drd);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	return 0;
}

static int exynos7870_usbdrd_phy_exit(struct phy *phy)
{
	int ret;
	u32 reg;
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;

	/*
	 * Disable the VBUS signal and the ID pull-up resistor.
	 * Enable force-suspend and force-sleep modes.
	 */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);
	reg &= ~(PHYUTMI_DRVVBUS | PHYUTMI_VBUSVLDEXT | PHYUTMI_VBUSVLDEXTSEL);
	reg &= ~PHYUTMI_IDPULLUP;
	reg |= PHYUTMI_FORCESUSPEND | PHYUTMI_FORCESLEEP;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);

	/* Power down PHY analog blocks */
	reg = readl(phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYCTRL);
	reg |= HSPHYCTRL_SIDDQ;
	writel(reg, phy_drd->reg_phy + EXYNOS7870_DRD_HSPHYCTRL);

	/* Clear VBUSVALID signal as the VBUS pad is not used */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_LINKSYSTEM);
	reg &= ~(LINKSYSTEM_FORCE_BVALID | LINKSYSTEM_FORCE_VBUSVALID);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_LINKSYSTEM);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	return 0;
}

static const struct phy_ops exynos7870_usbdrd_phy_ops = {
	.init		= exynos7870_usbdrd_phy_init,
	.exit		= exynos7870_usbdrd_phy_exit,
	.power_on	= exynos5_usbdrd_phy_power_on,
	.power_off	= exynos5_usbdrd_phy_power_off,
	.owner		= THIS_MODULE,
};

static void exynos2200_usbdrd_utmi_init(struct exynos5_usbdrd_phy *phy_drd)
{
	/* Configure non-Samsung IP PHY, responsible for UTMI */
	phy_init(phy_drd->hs_phy);
}

static void exynos2200_usbdrd_link_init(struct exynos5_usbdrd_phy *phy_drd)
{
	void __iomem *regs_base = phy_drd->reg_phy;
	u32 reg;

	/*
	 * Disable HWACG (hardware auto clock gating control). This will force
	 * QACTIVE signal in Q-Channel interface to HIGH level, to make sure
	 * the PHY clock is not gated by the hardware.
	 */
	reg = readl(regs_base + EXYNOS850_DRD_LINKCTRL);
	reg |= LINKCTRL_FORCE_QACT;
	writel(reg, regs_base + EXYNOS850_DRD_LINKCTRL);

	/* De-assert link reset */
	reg = readl(regs_base + EXYNOS2200_DRD_CLKRST);
	reg &= ~CLKRST_LINK_SW_RST;
	writel(reg, regs_base + EXYNOS2200_DRD_CLKRST);

	/* Set link VBUS Valid */
	reg = readl(regs_base + EXYNOS2200_DRD_UTMI);
	reg |= EXYNOS2200_UTMI_FORCE_BVALID | EXYNOS2200_UTMI_FORCE_VBUSVALID;
	writel(reg, regs_base + EXYNOS2200_DRD_UTMI);
}

static void
exynos2200_usbdrd_link_attach_detach_pipe3_phy(struct phy_usb_instance *inst)
{
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);
	void __iomem *regs_base = phy_drd->reg_phy;
	u32 reg;

	reg = readl(regs_base + EXYNOS850_DRD_LINKCTRL);
	if (inst->phy_cfg->id == EXYNOS5_DRDPHY_UTMI) {
		/* force pipe3 signal for link */
		reg &= ~LINKCTRL_FORCE_PHYSTATUS;
		reg |= LINKCTRL_FORCE_PIPE_EN | LINKCTRL_FORCE_RXELECIDLE;
	} else {
		/* disable forcing pipe interface */
		reg &= ~LINKCTRL_FORCE_PIPE_EN;
	}
	writel(reg, regs_base + EXYNOS850_DRD_LINKCTRL);

	reg = readl(regs_base + EXYNOS2200_DRD_HSP_MISC);
	if (inst->phy_cfg->id == EXYNOS5_DRDPHY_UTMI) {
		/* calibrate only eUSB phy */
		reg |= FIELD_PREP(HSP_MISC_RES_TUNE, RES_TUNE_PHY1);
		reg |= HSP_MISC_SET_REQ_IN2;
	} else {
		/* calibrate for dual phy */
		reg |= FIELD_PREP(HSP_MISC_RES_TUNE, RES_TUNE_PHY1_PHY2);
		reg &= ~HSP_MISC_SET_REQ_IN2;
	}
	writel(reg, regs_base + EXYNOS2200_DRD_HSP_MISC);

	reg = readl(regs_base + EXYNOS2200_DRD_CLKRST);
	if (inst->phy_cfg->id == EXYNOS5_DRDPHY_UTMI)
		reg &= ~EXYNOS2200_CLKRST_LINK_PCLK_SEL;
	else
		reg |= EXYNOS2200_CLKRST_LINK_PCLK_SEL;

	writel(reg, regs_base + EXYNOS2200_DRD_CLKRST);
}

static int exynos2200_usbdrd_phy_init(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);
	int ret;

	if (inst->phy_cfg->id == EXYNOS5_DRDPHY_UTMI) {
		/* Power-on PHY ... */
		ret = regulator_bulk_enable(phy_drd->drv_data->n_regulators,
					    phy_drd->regulators);
		if (ret) {
			dev_err(phy_drd->dev,
				"Failed to enable PHY regulator(s)\n");
			return ret;
		}
	}
	/*
	 * ... and ungate power via PMU. Without this here, we get an SError
	 * trying to access PMA registers
	 */
	exynos5_usbdrd_phy_isol(inst, false);

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;

	/* Set up the link controller */
	exynos2200_usbdrd_link_init(phy_drd);

	/* UTMI or PIPE3 link preparation */
	exynos2200_usbdrd_link_attach_detach_pipe3_phy(inst);

	/* UTMI or PIPE3 specific init */
	inst->phy_cfg->phy_init(phy_drd);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	return 0;
}

static int exynos2200_usbdrd_phy_exit(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);
	void __iomem *regs_base = phy_drd->reg_phy;
	u32 reg;
	int ret;

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;

	reg = readl(regs_base + EXYNOS2200_DRD_UTMI);
	reg &= ~(EXYNOS2200_UTMI_FORCE_BVALID | EXYNOS2200_UTMI_FORCE_VBUSVALID);
	writel(reg, regs_base + EXYNOS2200_DRD_UTMI);

	reg = readl(regs_base + EXYNOS2200_DRD_CLKRST);
	reg |= CLKRST_LINK_SW_RST;
	writel(reg, regs_base + EXYNOS2200_DRD_CLKRST);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	exynos5_usbdrd_phy_isol(inst, true);
	return regulator_bulk_disable(phy_drd->drv_data->n_regulators,
				      phy_drd->regulators);
}

static const struct phy_ops exynos2200_usbdrd_phy_ops = {
	.init		= exynos2200_usbdrd_phy_init,
	.exit		= exynos2200_usbdrd_phy_exit,
	.owner		= THIS_MODULE,
};

static void
exynos5_usbdrd_usb_v3p1_pipe_override(struct exynos5_usbdrd_phy *phy_drd)
{
	void __iomem *regs_base = phy_drd->reg_phy;
	u32 reg;

	/* force pipe3 signal for link */
	reg = readl(regs_base + EXYNOS850_DRD_LINKCTRL);
	reg &= ~LINKCTRL_FORCE_PHYSTATUS;
	reg |= LINKCTRL_FORCE_PIPE_EN | LINKCTRL_FORCE_RXELECIDLE;
	writel(reg, regs_base + EXYNOS850_DRD_LINKCTRL);

	/* PMA disable */
	reg = readl(regs_base + EXYNOS850_DRD_SECPMACTL);
	reg |= SECPMACTL_PMA_LOW_PWR;
	writel(reg, regs_base + EXYNOS850_DRD_SECPMACTL);
}

static void exynos850_usbdrd_utmi_init(struct exynos5_usbdrd_phy *phy_drd)
{
	void __iomem *regs_base = phy_drd->reg_phy;
	u32 reg;
	u32 ss_ports;

	/*
	 * Disable HWACG (hardware auto clock gating control). This will force
	 * QACTIVE signal in Q-Channel interface to HIGH level, to make sure
	 * the PHY clock is not gated by the hardware.
	 */
	reg = readl(regs_base + EXYNOS850_DRD_LINKCTRL);
	reg |= LINKCTRL_FORCE_QACT;
	writel(reg, regs_base + EXYNOS850_DRD_LINKCTRL);

	reg = readl(regs_base + EXYNOS850_DRD_LINKPORT);
	ss_ports = FIELD_GET(LINKPORT_HOST_NUM_U3, reg);

	/* Start PHY Reset (POR=high) */
	reg = readl(regs_base + EXYNOS850_DRD_CLKRST);
	if (ss_ports) {
		reg |= CLKRST_PHY20_SW_POR;
		reg |= CLKRST_PHY20_SW_POR_SEL;
		reg |= CLKRST_PHY_RESET_SEL;
	}
	reg |= CLKRST_PHY_SW_RST;
	writel(reg, regs_base + EXYNOS850_DRD_CLKRST);

	/* Enable UTMI+ */
	reg = readl(regs_base + EXYNOS850_DRD_UTMI);
	reg &= ~(UTMI_FORCE_SUSPEND | UTMI_FORCE_SLEEP | UTMI_DP_PULLDOWN |
		 UTMI_DM_PULLDOWN);
	writel(reg, regs_base + EXYNOS850_DRD_UTMI);

	/* Set PHY clock and control HS PHY */
	reg = readl(regs_base + EXYNOS850_DRD_HSP);
	reg |= HSP_EN_UTMISUSPEND | HSP_COMMONONN;
	writel(reg, regs_base + EXYNOS850_DRD_HSP);

	/* Set VBUS Valid and D+ pull-up control by VBUS pad usage */
	reg = readl(regs_base + EXYNOS850_DRD_LINKCTRL);
	reg |= FIELD_PREP(LINKCTRL_BUS_FILTER_BYPASS, 0xf);
	writel(reg, regs_base + EXYNOS850_DRD_LINKCTRL);

	if (!phy_drd->sw) {
		reg = readl(regs_base + EXYNOS850_DRD_UTMI);
		reg |= UTMI_FORCE_BVALID | UTMI_FORCE_VBUSVALID;
		writel(reg, regs_base + EXYNOS850_DRD_UTMI);

		reg = readl(regs_base + EXYNOS850_DRD_HSP);
		reg |= HSP_VBUSVLDEXT | HSP_VBUSVLDEXTSEL;
		writel(reg, regs_base + EXYNOS850_DRD_HSP);
	}

	reg = readl(regs_base + EXYNOS850_DRD_SSPPLLCTL);
	reg &= ~SSPPLLCTL_FSEL;
	switch (phy_drd->extrefclk) {
	case EXYNOS5_FSEL_50MHZ:
		reg |= FIELD_PREP(SSPPLLCTL_FSEL, 7);
		break;
	case EXYNOS5_FSEL_26MHZ:
		reg |= FIELD_PREP(SSPPLLCTL_FSEL, 6);
		break;
	case EXYNOS5_FSEL_24MHZ:
		reg |= FIELD_PREP(SSPPLLCTL_FSEL, 2);
		break;
	case EXYNOS5_FSEL_20MHZ:
		reg |= FIELD_PREP(SSPPLLCTL_FSEL, 1);
		break;
	case EXYNOS5_FSEL_19MHZ2:
		reg |= FIELD_PREP(SSPPLLCTL_FSEL, 0);
		break;
	default:
		dev_warn(phy_drd->dev, "unsupported ref clk: %#.2x\n",
			 phy_drd->extrefclk);
		break;
	}
	writel(reg, regs_base + EXYNOS850_DRD_SSPPLLCTL);

	if (phy_drd->drv_data->phy_tunes)
		exynos5_usbdrd_apply_phy_tunes(phy_drd,
					       PTS_UTMI_POSTINIT);

	/* Power up PHY analog blocks */
	reg = readl(regs_base + EXYNOS850_DRD_HSP_TEST);
	reg &= ~HSP_TEST_SIDDQ;
	writel(reg, regs_base + EXYNOS850_DRD_HSP_TEST);

	/* Finish PHY reset (POR=low) */
	fsleep(10); /* required before doing POR=low */
	reg = readl(regs_base + EXYNOS850_DRD_CLKRST);
	if (ss_ports) {
		reg |= CLKRST_PHY20_SW_POR_SEL;
		reg &= ~CLKRST_PHY20_SW_POR;
	}
	reg &= ~(CLKRST_PHY_SW_RST | CLKRST_PORT_RST);
	writel(reg, regs_base + EXYNOS850_DRD_CLKRST);
	fsleep(75); /* required after POR=low for guaranteed PHY clock */

	/* Disable single ended signal out */
	reg = readl(regs_base + EXYNOS850_DRD_HSP);
	reg &= ~HSP_FSV_OUT_EN;
	writel(reg, regs_base + EXYNOS850_DRD_HSP);

	if (ss_ports)
		exynos5_usbdrd_usb_v3p1_pipe_override(phy_drd);
}

static int exynos850_usbdrd_phy_init(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);
	int ret;

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;

	/* UTMI or PIPE3 specific init */
	scoped_guard(mutex, &phy_drd->phy_mutex)
		inst->phy_cfg->phy_init(phy_drd);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	return 0;
}

static int exynos850_usbdrd_phy_exit(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);
	void __iomem *regs_base = phy_drd->reg_phy;
	u32 reg;
	int ret;

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;

	guard(mutex)(&phy_drd->phy_mutex);

	/* Set PHY clock and control HS PHY */
	reg = readl(regs_base + EXYNOS850_DRD_UTMI);
	reg &= ~(UTMI_DP_PULLDOWN | UTMI_DM_PULLDOWN);
	reg |= UTMI_FORCE_SUSPEND | UTMI_FORCE_SLEEP;
	writel(reg, regs_base + EXYNOS850_DRD_UTMI);

	/* Power down PHY analog blocks */
	reg = readl(regs_base + EXYNOS850_DRD_HSP_TEST);
	reg |= HSP_TEST_SIDDQ;
	writel(reg, regs_base + EXYNOS850_DRD_HSP_TEST);

	/* Link reset */
	reg = readl(regs_base + EXYNOS850_DRD_CLKRST);
	reg |= CLKRST_LINK_SW_RST;
	writel(reg, regs_base + EXYNOS850_DRD_CLKRST);
	fsleep(10); /* required before doing POR=low */
	reg &= ~CLKRST_LINK_SW_RST;
	writel(reg, regs_base + EXYNOS850_DRD_CLKRST);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	return 0;
}

static const struct phy_ops exynos850_usbdrd_phy_ops = {
	.init		= exynos850_usbdrd_phy_init,
	.exit		= exynos850_usbdrd_phy_exit,
	.power_on	= exynos5_usbdrd_phy_power_on,
	.power_off	= exynos5_usbdrd_phy_power_off,
	.owner		= THIS_MODULE,
};

/*
 * Exynos8895 (spec/L3_usb.md 3.D, 5.1): the stock CAL sequence on the exynos5
 * register layout, window 0 plus the second PHY window of the flipped Type-C port.
 */
enum { E8895_CR_ADDR, E8895_CR_READ, E8895_CR_WRITE };

static void __iomem *exynos8895_port_base(struct exynos5_usbdrd_phy *d)
{
	return d->cal_port == 1 ? d->cal_base[1] : d->cal_base[0];	/* cal.c:60-67 */
}

/*
 * CR-port access, cal.c:51-132: read-modify-write of PHYREG0 that keeps bits 31:20, the
 * port's own window, 100 x 1 us handshake per edge, 5 us after each edge and 50 us after
 * each phase. Stock tests "!usec" after "usec-- > 0" (cal.c:110-124), which can never be
 * true; the timeout is tested here and counted.
 */
static u16 exynos8895_cr_access(struct exynos5_usbdrd_phy *d, int op, u16 data)
{
	void __iomem *base = exynos8895_port_base(d);
	int phase, nphase = op == E8895_CR_ADDR ? 1 : 2, hs, us;
	u32 r0, r1 = 0, trig;

	r0 = readl(base + EXYNOS5_DRD_PHYREG0);
	r0 &= ~0xfffff;					/* cal.c:70-72: clear [19:0] only */
	writel(r0, base + EXYNOS5_DRD_PHYREG0);
	r0 &= ~PHYREG0_CR_DATA_IN;
	r0 |= FIELD_PREP(PHYREG0_CR_DATA_IN, data);	/* cal.c:75-77 */
	writel(r0, base + EXYNOS5_DRD_PHYREG0);

	for (phase = 0; phase < nphase; phase++) {
		if (op == E8895_CR_ADDR)
			trig = PHYREG0_CR_CAP_ADDR;
		else if (phase == 0)
			trig = PHYREG0_CR_CAP_DATA;
		else
			trig = op == E8895_CR_READ ? PHYREG0_CR_READ : PHYREG0_CR_WRITE;
		for (hs = 2; hs > 0; hs--) {		/* cal.c:100-128 */
			bool ack_wanted = hs == 2;	/* trigger raised: wait for ack, then for its release */

			if (ack_wanted)
				r0 |= trig;
			else
				r0 &= ~trig;
			writel(r0, base + EXYNOS5_DRD_PHYREG0);
			for (us = 100; us >= 0; us--) {
				r1 = readl(base + EXYNOS5_DRD_PHYREG1);
				if (!!(r1 & PHYREG1_CR_ACK) == ack_wanted)
					break;
				udelay(1);
			}
			if (us < 0) {
				d->cr_timeouts++;
				dev_err(d->dev, "CRPORT handshake timeout1 (0x%08x)\n", r0);
			}
			udelay(5);
		}
		udelay(50);
	}
	return FIELD_GET(PHYREG0_CR_DATA_OUT, r1);	/* cal.c:131 */
}

static void exynos8895_cr_write(struct exynos5_usbdrd_phy *d, u16 addr, u16 val)
{
	exynos8895_cr_access(d, E8895_CR_ADDR, addr);
	exynos8895_cr_access(d, E8895_CR_WRITE, val);
}

static u16 exynos8895_cr_read(struct exynos5_usbdrd_phy *d, u16 addr)
{
	exynos8895_cr_access(d, E8895_CR_ADDR, addr);
	return exynos8895_cr_access(d, E8895_CR_READ, 0);
}

/* CR read-modify-write: (CR[addr] & ~clr) | set */
static void exynos8895_cr_rmw(struct exynos5_usbdrd_phy *d, u16 addr, u16 clr, u16 set)
{
	exynos8895_cr_write(d, addr, (exynos8895_cr_read(d, addr) & ~clr) | set);
}

/* Q-channel force on/off, cal.c:806-843 */
static void exynos8895_q_ch(void __iomem *base, bool enable)
{
	u32 r;

	if (enable) {
		r = readl(base + EXYNOS5_DRD_PHYRESUME);
		r |= EXYNOS8895_PHYRESUME_DIS_QACT_ALL;		/* :813-816; BIT(14) left alone, :817 */
		r &= ~EXYNOS8895_PHYRESUME_FORCE_QACT;		/* :818 */
		udelay(500);
		writel(r, base + EXYNOS5_DRD_PHYRESUME);
		udelay(500);
		r = readl(base + EXYNOS5_DRD_PHYRESUME);
		r |= EXYNOS8895_PHYRESUME_FORCE_QACT;		/* :823 */
		udelay(500);
		writel(r, base + EXYNOS5_DRD_PHYRESUME);

		r = readl(base + EXYNOS5_DRD_LINKSYSTEM);	/* :827-832 */
		writel(r | EXYNOS8895_LINKSYSTEM_LINK_SW_RESET, base + EXYNOS5_DRD_LINKSYSTEM);
		udelay(10);
		writel(r & ~EXYNOS8895_LINKSYSTEM_LINK_SW_RESET, base + EXYNOS5_DRD_LINKSYSTEM);
	} else {
		r = readl(base + EXYNOS5_DRD_PHYRESUME);
		r &= ~EXYNOS8895_PHYRESUME_FORCE_QACT;		/* :835 */
		r |= EXYNOS8895_PHYRESUME_DIS_QACT_ALL;		/* :836-839 */
		writel(r, base + EXYNOS5_DRD_PHYRESUME);
	}
}

/* SS block of one window, cal.c:541-678 (version 0x0111, refclk DIFF_26MHZ, ss_tune present) */
static void exynos8895_ss_enable(struct exynos5_usbdrd_phy *d, u32 *clkrst, int port)
{
	void __iomem *base = d->cal_base[port];
	u32 c = *clkrst, v, reg0;

	c &= ~PHYCLKRST_COMMONONN;			/* :565-566, common block kept on */
	c &= ~PHYCLKRST_EN_UTMISUSPEND;			/* :569 */
	c |= PHYCLKRST_RETENABLEN;			/* :572 */
	c |= PHYCLKRST_REF_SSP_EN;			/* :575 */

	v = readl(d->cal_base[0] + EXYNOS5_DRD_PHYPARAM0);	/* :577 reads window 0 */
	v &= ~PHYPARAM0_REF_USE_PAD;			/* :581, refsel CLKCORE is not DIFF_PAD */
	writel(v, base + EXYNOS5_DRD_PHYPARAM0);	/* :582 writes the port's window */

	reg0 = readl(base + EXYNOS5_DRD_PHYREG0);	/* :584-585 */

	c &= ~PHYCLKRST_MPLL_MULTIPLIER;
	c |= FIELD_PREP(PHYCLKRST_MPLL_MULTIPLIER, EXYNOS8895_MPLL_MULT_26M);	/* :602-603 */
	c &= ~PHYCLKRST_REF_CLKDIV2;			/* :607, 0x82 & 0x40 == 0 */
	c &= ~PHYCLKRST_SSC_REFCLKSEL;			/* :612 */
	c |= (u32)EXYNOS8895_SSC_REFCLKSEL_26M << 23;	/* :613, raw shift: also sets bit 31 */

	c |= PHYCLKRST_SSC_EN;				/* :645-646, enable_ssc = 1 (usbdrd.c:522) */
	c &= ~PHYCLKRST_SSC_RANGE;			/* :651-652, ssc_range = 0 (usbdrd.c:523) */

	v = readl(base + EXYNOS5_DRD_PHYPIPE);
	v |= EXYNOS8895_PHYPIPE_PHY_CLOCK_SEL;		/* :670-672 */
	writel(v, base + EXYNOS5_DRD_PHYPIPE);

	writel(reg0, base + EXYNOS5_DRD_PHYREG0);	/* :674-675, unchanged write-back */
	*clkrst = c;
}

/* Power-up of the used port and power-down of the unused one, cal.c:680-748 (USB_MUX_UTMI_ENABLE unset, :42) */
static void exynos8895_ss_power_down(struct exynos5_usbdrd_phy *d, bool en)
{
	void __iomem *b0 = d->cal_base[0], *b1 = d->cal_base[1];
	u32 t0, t1, u;

	t0 = readl(b0 + EXYNOS5_DRD_PHYTEST);
	if (en) {
		t0 &= ~(PHYTEST_POWERDOWN_HSP | PHYTEST_POWERDOWN_SSP);
		writel(t0, b0 + EXYNOS5_DRD_PHYTEST);			/* :687-689 */
		t1 = readl(b1 + EXYNOS5_DRD_PHYTEST);
		t1 &= ~(PHYTEST_POWERDOWN_HSP | PHYTEST_POWERDOWN_SSP);
		writel(t1, b1 + EXYNOS5_DRD_PHYTEST);			/* :691-695, version 0x0111 */
		udelay(1000);						/* :697 */
		if (d->cal_port == 0) {
			t1 |= PHYTEST_POWERDOWN_SSP | PHYTEST_POWERDOWN_HSP;
			writel(t1, b1 + EXYNOS5_DRD_PHYTEST);		/* :717-720 */
		} else {						/* port 1 or -1 */
			t0 |= PHYTEST_POWERDOWN_SSP;
			writel(t0, b0 + EXYNOS5_DRD_PHYTEST);		/* :722-723 */
			t1 |= PHYTEST_POWERDOWN_HSP;
			writel(t1, b1 + EXYNOS5_DRD_PHYTEST);		/* :724-725 */
		}
		u = readl(b1 + EXYNOS5_DRD_PHYUTMI);			/* :729-732 "force suspend phy1 hs" */
		u |= PHYUTMI_FORCESLEEP | PHYUTMI_FORCESUSPEND;
		writel(u, b1 + EXYNOS5_DRD_PHYUTMI);
	} else {
		t0 |= PHYTEST_POWERDOWN_HSP | PHYTEST_POWERDOWN_SSP;
		writel(t0, b0 + EXYNOS5_DRD_PHYTEST);			/* :736-738 */
		if (d->cal_port == 1) {
			t1 = readl(b1 + EXYNOS5_DRD_PHYTEST);
			t1 |= PHYTEST_POWERDOWN_HSP | PHYTEST_POWERDOWN_SSP;
			writel(t1, b1 + EXYNOS5_DRD_PHYTEST);		/* :739-746 */
		}
	}
}

/* CAL enable = stock exynos_usbdrd_pipe3_init (usbdrd.c:757-783) + samsung_exynos_cal_usb3phy_enable (cal.c:845-1025) */
static void exynos8895_cal_enable(struct exynos5_usbdrd_phy *d)
{
	bool first = !d->cal_probed;
	void __iomem *b0, *qbase;
	u32 v, c;

	/*
	 * Orientation, usbdrd.c:767-770 "used_phy_port = !gpio_get_value(...)"; the DT flags
	 * the GPIO active-low, so the logical value is the port. Stock reads it at every
	 * attach; here .init runs once per dwc3_core_init (probe and system resume).
	 */
	if (d->sw) {
		d->cal_port = d->sw_port;	/* orientation from the Type-C controller (L6G 5.3.3) */
	} else {
		d->cal_port = d->port_gpio ? gpiod_get_value(d->port_gpio) : 0;
		if (d->cal_port < 0)
			d->cal_port = 0;
	}
	d->init_count++;
	dev_info(d->dev, "8895: init #%u port=%d gpio=%d\n", d->init_count, d->cal_port,
		 d->port_gpio ? gpiod_get_raw_value(d->port_gpio) : -1);

	/* Reference-clock select, usbdrd.c:603-617 (called at :775) */
	v = readl(d->reg_refclk + EXYNOS8895_REFCLK_CTRL);
	v = (v & ~EXYNOS8895_REFCLK_CTRL_SEL) | EXYNOS8895_REFCLK_CTRL_SEL;
	writel(v, d->reg_refclk + EXYNOS8895_REFCLK_CTRL);

	/* Version probe and the conditional +4 shift of BOTH windows, cal.c:855-869, first init only */
	if (first) {
		d->cal_ver[0] = readl(d->reg_phy);
		d->cal_ver[1] = readl(d->reg_phy2);	/* diagnostic, stock never reads it */
		d->cal_probed = true;
		if (!d->cal_ver[0]) {
			d->cal_zero = true;
			d->cal_port = -1;			/* cal.c:860 */
		} else {
			d->cal_base[0] = d->reg_phy + 4;	/* cal.c:863 */
			d->cal_base[1] = d->reg_phy2 + 4;	/* cal.c:866-867 */
		}
		dev_info(d->dev, "8895: ver0=0x%08x ver1=0x%08x shift=%d port=%d\n",
			 d->cal_ver[0], d->cal_ver[1], !d->cal_zero, d->cal_port);
	}
	b0 = d->cal_base[0];

	/* Q-channel force, cal.c:871-877: (0x0111 & 0xf) >= 1 */
	qbase = d->cal_port == 1 ? d->cal_base[1] : b0;
	exynos8895_q_ch(qbase, true);

	/* PHY mux, cal.c:879-904 */
	if (d->cal_port != -1) {
		v = readl(b0 + EXYNOS8895_DRD_PHYSELECTION);
		if (d->cal_port == 0)
			v &= ~(EXYNOS8895_PHYSEL_PIPE | EXYNOS8895_PHYSEL_PIPE_CLK);
		else
			v |= EXYNOS8895_PHYSEL_PIPE | EXYNOS8895_PHYSEL_PIPE_CLK;
		writel(v, b0 + EXYNOS8895_DRD_PHYSELECTION);
	}

	/* PHYCLKRST, cal.c:906-948 */
	c = readl(b0 + EXYNOS5_DRD_PHYCLKRST);
	c |= PHYCLKRST_PORTRESET;
	c &= ~PHYCLKRST_REFCLKSEL;
	c |= FIELD_PREP(PHYCLKRST_REFCLKSEL, EXYNOS8895_REFCLKSEL_CLKCORE);
	c &= ~EXYNOS8895_PHYCLKRST_FSEL;
	c |= FIELD_PREP(EXYNOS8895_PHYCLKRST_FSEL, EXYNOS8895_FSEL_26M);
	exynos8895_ss_enable(d, &c, 0);				/* :923 */
	if (d->cal_port == 1) {					/* :925-939 */
		exynos8895_ss_enable(d, &c, 1);
		writel(c, d->cal_base[1] + EXYNOS5_DRD_PHYCLKRST);
		udelay(10);
		c &= ~PHYCLKRST_PORTRESET;
		writel(c, d->cal_base[1] + EXYNOS5_DRD_PHYCLKRST);
		c |= PHYCLKRST_PORTRESET;
	}
	writel(c, b0 + EXYNOS5_DRD_PHYCLKRST);			/* :944 */
	udelay(10);
	c &= ~PHYCLKRST_PORTRESET;
	writel(c, b0 + EXYNOS5_DRD_PHYCLKRST);			/* :947-948 */
	/* the value WRITTEN is the check; stock never reads PHYCLKRST back */
	if (c != EXYNOS8895_PHYCLKRST_EXPECT)
		dev_warn(d->dev, "8895: PHYCLKRST 0x%08x != 0x%08x\n", c,
			 EXYNOS8895_PHYCLKRST_EXPECT);

	/* cal.c:950-957 */
	exynos8895_ss_power_down(d, true);
	udelay(500);

	/* UTMI and VBUS force, cal.c:959-992 (not_used_vbus_pad: FDT:7886) */
	v = readl(b0 + EXYNOS5_DRD_PHYUTMI);
	v &= ~(PHYUTMI_FORCESLEEP | PHYUTMI_FORCESUSPEND | PHYUTMI_DMPULLDOWN | PHYUTMI_DPPULLDOWN);
	c = readl(b0 + EXYNOS5_DRD_LINKSYSTEM);
	c |= LINKSYSTEM_FORCE_BVALID | LINKSYSTEM_FORCE_VBUSVALID;
	writel(c, b0 + EXYNOS5_DRD_LINKSYSTEM);			/* :972-975 */
	v |= PHYUTMI_VBUSVLDEXTSEL | PHYUTMI_VBUSVLDEXT;	/* :985-986 */
	v &= ~PHYUTMI_DRVVBUS;					/* :990 */
	v |= PHYUTMI_OTGDISABLE;				/* :991 */
	writel(v, b0 + EXYNOS5_DRD_PHYUTMI);			/* :992 */

	/* Over-current select, cal.c:1000-1010 (use_io_for_ovc = false, usbdrd.c:643) */
	v = readl(b0 + EXYNOS5_DRD_LINKPORT);
	v |= LINKPORT_HOST_PORT_OVCR_U3_SEL | LINKPORT_HOST_PORT_OVCR_U2_SEL;
	v &= ~LINKPORT_HOST_U3_PORT_DISABLE;
	writel(v, b0 + EXYNOS5_DRD_LINKPORT);

	dev_info(d->dev, "8895: init done clkrst=0x%08x utmi=0x%08x linksys=0x%08x resume=0x%08x\n",
		 readl(b0 + EXYNOS5_DRD_PHYCLKRST), readl(b0 + EXYNOS5_DRD_PHYUTMI),
		 readl(b0 + EXYNOS5_DRD_LINKSYSTEM), readl(qbase + EXYNOS5_DRD_PHYRESUME));
}

/*
 * Flipped cable only: stock samsung_exynos_cal_usb3phy_dp_altmode_set_ss_disable(info, 0)
 * (cal.c:1075-1077 -> :365-450) stops the SS lane of window 0, through window 0's CR port.
 */
static void exynos8895_dp_altmode_ss_disable_port0(struct exynos5_usbdrd_phy *d)
{
	int port = d->cal_port;
	u32 v;

	d->cal_port = 0;					/* cal.c:372-373 */
	v = readl(d->cal_base[0] + EXYNOS5_DRD_LINKPORT);
	writel(v | LINKPORT_HOST_U3_PORT_DISABLE, d->cal_base[0] + EXYNOS5_DRD_LINKPORT);	/* :385-387 */
	exynos8895_cr_rmw(d, 0x0011, 0x3, 0x2);			/* :392-395 MPLL_EN_OVRD=1 MPLL_EN=0 */
	exynos8895_cr_rmw(d, 0x1000, 0x3 << 8, 0x2 << 8);	/* :400-403 TX_CM_EN_OVRD=1 TX_CM_EN=0 */
	exynos8895_cr_rmw(d, 0x1005, 0x3 << 12, 0x3 << 12);	/* :408-411 RX_LOS_EN_OVRD=1 RX_LOS_EN=1 */
	exynos8895_cr_rmw(d, 0x1000, 0xf << 4, 0xa << 4);	/* :418-421 TX_EN/TX_DATA_EN overrides */
	exynos8895_cr_rmw(d, 0x1005, 0xf << 2, 0xa << 2);	/* :428-431 RX_DATA_EN/RX_PLL_EN overrides */
	exynos8895_cr_rmw(d, 0x1005, 0x2 << 10, 0x2 << 10);	/* :436-439 RX_TERM_EN_OVRD=1 */
	exynos8895_cr_rmw(d, 0x1006, 0x3 << 12, 0x3 << 12);	/* :444-447 RX_RESET_OVRD=1 RX_RESET=1 */
	d->cal_port = port;					/* :449 */
}

/* Device tune = stock exynos_usbdrd_pipe3_tune device branch (usbdrd.c:862-886) */
static void exynos8895_cal_tune_dev(struct exynos5_usbdrd_phy *d)
{
	void __iomem *b0 = d->cal_base[0];
	void __iomem *bs = exynos8895_port_base(d);	/* SS block of the port, cal.c:1414-1419 */
	u32 v;

	/* LINKSYSTEM FLADJ and xHCI version control, cal.c:1362-1366 */
	v = readl(b0 + EXYNOS5_DRD_LINKSYSTEM);
	v &= ~LINKSYSTEM_FLADJ;
	v |= FIELD_PREP(LINKSYSTEM_FLADJ, 0x20) | LINKSYSTEM_XHCI_VERSION_CONTROL;
	writel(v, b0 + EXYNOS5_DRD_LINKSYSTEM);

	/* HS tune on window 0, cal.c:1369-1408; PHYPARAM0[25:0] = 0x031de71a */
	v = readl(b0 + EXYNOS5_DRD_PHYPARAM0);
	v &= ~(PHYPARAM0_TXVREFTUNE | PHYPARAM0_TXRISETUNE | PHYPARAM0_TXRESTUNE |
	       PHYPARAM0_TXPREEMPPULSETUNE | PHYPARAM0_TXPREEMPAMPTUNE | PHYPARAM0_TXHSXVTUNE |
	       PHYPARAM0_TXFSLSTUNE | PHYPARAM0_SQRXTUNE | PHYPARAM0_OTGTUNE |
	       PHYPARAM0_COMPDISTUNE);				/* tx_pre_emp_plus = 0, usbdrd.c:400 */
	v |= FIELD_PREP(PHYPARAM0_TXVREFTUNE, EXYNOS8895_HS_TXVREF) |
	     FIELD_PREP(PHYPARAM0_TXRISETUNE, EXYNOS8895_HS_TXRISE) |
	     FIELD_PREP(PHYPARAM0_TXRESTUNE, EXYNOS8895_HS_TXRES) |
	     FIELD_PREP(PHYPARAM0_TXPREEMPAMPTUNE, EXYNOS8895_HS_TXPREEMPAMP) |
	     FIELD_PREP(PHYPARAM0_TXHSXVTUNE, EXYNOS8895_HS_TXHSXV) |
	     FIELD_PREP(PHYPARAM0_TXFSLSTUNE, EXYNOS8895_HS_TXFSLS) |
	     FIELD_PREP(PHYPARAM0_SQRXTUNE, EXYNOS8895_HS_SQRX) |
	     FIELD_PREP(PHYPARAM0_OTGTUNE, EXYNOS8895_HS_OTG) |
	     FIELD_PREP(PHYPARAM0_COMPDISTUNE, EXYNOS8895_HS_COMPDIS);
	writel(v, b0 + EXYNOS5_DRD_PHYPARAM0);

	/* SS tune on the port's window, cal.c:1411-1451; PHYPARAM1 not written (USB_SS_TX_TUNE_PCS, cal.c:40) */
	v = readl(bs + EXYNOS7870_DRD_PHYPARAM2);
	v &= ~(PHYPARAM2_TX_VBOOST_LVL | PHYPARAM2_LOS_BIAS);
	v |= FIELD_PREP(PHYPARAM2_TX_VBOOST_LVL, EXYNOS8895_SS_TX_VBOOST) |
	     FIELD_PREP(PHYPARAM2_LOS_BIAS, EXYNOS8895_SS_LOS_BIAS);
	writel(v, bs + EXYNOS7870_DRD_PHYPARAM2);
	v = readl(bs + EXYNOS7870_DRD_PHYPCSVAL);
	v &= ~PHYPCSVAL_PCS_RX_LOS_MASK;
	v |= FIELD_PREP(PHYPCSVAL_PCS_RX_LOS_MASK, EXYNOS8895_SS_LOS_MASK);
	writel(v, bs + EXYNOS7870_DRD_PHYPCSVAL);

	/* Late enable, cal.c:1027-1080 */
	exynos8895_cr_write(d, EXYNOS8895_CR_LANE0_TX_DEBUG, 0x80);
	exynos8895_cr_write(d, EXYNOS8895_CR_TX_OVRD_DRV_LO,
			    0x4000 | (EXYNOS8895_SS_DEEMPH_3P5DB << 7) | EXYNOS8895_SS_SWING_FULL);
	/*
	 * rx_decode_mode, set_crport_mpll_charge_pump and decrease_ss_tx_imp are never
	 * assigned by stock (usbdrd.c:513-529, devm_kmalloc'ed): taken as 0 (spec O6).
	 */
	exynos8895_cr_write(d, EXYNOS8895_CR_LOSLEVEL_OVRD_IN, 0xa409);
	exynos8895_cr_write(d, EXYNOS8895_CR_TX_VBOOSTLVL_OVRD, EXYNOS8895_SS_TX_VBOOST << 13);
	if (d->cal_port == 1)
		exynos8895_dp_altmode_ss_disable_port0(d);
}

/*
 * Host tune = stock exynos_usbdrd_pipe3_tune(phy_state >= OTG_STATE_A_IDLE) with
 * use_additional_tuning (FDT:7896): samsung_exynos_cal_usb3phy_tune_host() (cal.c:1454-1557)
 * then samsung_exynos_cal_usb3phy_late_enable() (cal.c:1027-1080), usbdrd.c:862-886.
 *
 * HS (cal.c:1478-1512): the fill values of usbdrd.c:398-407 with the host override of
 * usbdrd.c:446-449 (tx_res 3, tx_vref 1, tx_pre_emp 0, compdis 7). rx_sqrx keeps the fill
 * value 3 of usbdrd.c:405 (the device override of :429 is not applied in host mode).
 * PHYPARAM0[25:0] = vref 1<<22 | rise 1<<20 | res 3<<18 | pre_emp_plus 0 | pre_emp 0<<15 |
 * hsxv 3<<13 | fsls 3<<9 | sqrx 3<<6 | otg 3<<3 | compdis 7 = 0x005c66df.
 * SS (cal.c:1519-1556): the fill values of usbdrd.c:515-528 only - stock has no host SS
 * override (usbdrd.c:476-485) and does not write PHYPARAM1 (USB_SS_TX_TUNE_PCS, cal.c:40);
 * tx_boost_level 4 is the fill value, not the device override 6 (usbdrd.c:465).
 */
static void exynos8895_cal_tune_host(struct exynos5_usbdrd_phy *d)
{
	void __iomem *b0 = d->cal_base[0];
	void __iomem *bs = exynos8895_port_base(d);	/* SS block of the port, cal.c:1520-1522 */
	u32 v;

	/* LINKSYSTEM FLADJ 0x20 and the xHCI version control bit, cal.c:1466-1471 */
	v = readl(b0 + EXYNOS5_DRD_LINKSYSTEM);
	v &= ~LINKSYSTEM_FLADJ;
	v |= FIELD_PREP(LINKSYSTEM_FLADJ, 0x20) | LINKSYSTEM_XHCI_VERSION_CONTROL;
	writel(v, b0 + EXYNOS5_DRD_LINKSYSTEM);

	v = readl(b0 + EXYNOS5_DRD_PHYPARAM0);
	v &= ~(PHYPARAM0_TXVREFTUNE | PHYPARAM0_TXRISETUNE | PHYPARAM0_TXRESTUNE |
	       PHYPARAM0_TXPREEMPPULSETUNE | PHYPARAM0_TXPREEMPAMPTUNE | PHYPARAM0_TXHSXVTUNE |
	       PHYPARAM0_TXFSLSTUNE | PHYPARAM0_SQRXTUNE | PHYPARAM0_OTGTUNE |
	       PHYPARAM0_COMPDISTUNE);
	v |= FIELD_PREP(PHYPARAM0_TXVREFTUNE, EXYNOS8895_HS_TXVREF_HOST) |
	     FIELD_PREP(PHYPARAM0_TXRISETUNE, EXYNOS8895_HS_TXRISE) |
	     FIELD_PREP(PHYPARAM0_TXRESTUNE, EXYNOS8895_HS_TXRES) |
	     FIELD_PREP(PHYPARAM0_TXPREEMPAMPTUNE, 0) |		/* usbdrd.c:448 */
	     FIELD_PREP(PHYPARAM0_TXHSXVTUNE, EXYNOS8895_HS_TXHSXV) |
	     FIELD_PREP(PHYPARAM0_TXFSLSTUNE, EXYNOS8895_HS_TXFSLS) |
	     FIELD_PREP(PHYPARAM0_SQRXTUNE, EXYNOS8895_HS_SQRX_HOST) |
	     FIELD_PREP(PHYPARAM0_OTGTUNE, EXYNOS8895_HS_OTG) |
	     FIELD_PREP(PHYPARAM0_COMPDISTUNE, EXYNOS8895_HS_COMPDIS_HOST);
	writel(v, b0 + EXYNOS5_DRD_PHYPARAM0);

	v = readl(bs + EXYNOS7870_DRD_PHYPARAM2);
	v &= ~(PHYPARAM2_TX_VBOOST_LVL | PHYPARAM2_LOS_BIAS);
	v |= FIELD_PREP(PHYPARAM2_TX_VBOOST_LVL, EXYNOS8895_SS_TX_VBOOST_HOST) |
	     FIELD_PREP(PHYPARAM2_LOS_BIAS, EXYNOS8895_SS_LOS_BIAS);
	writel(v, bs + EXYNOS7870_DRD_PHYPARAM2);
	v = readl(bs + EXYNOS7870_DRD_PHYPCSVAL);
	v &= ~PHYPCSVAL_PCS_RX_LOS_MASK;
	v |= FIELD_PREP(PHYPCSVAL_PCS_RX_LOS_MASK, EXYNOS8895_SS_LOS_MASK);
	writel(v, bs + EXYNOS7870_DRD_PHYPCSVAL);

	/* Late enable, cal.c:1037-1060 (the device values, boost 4 instead of 6) */
	exynos8895_cr_write(d, EXYNOS8895_CR_LANE0_TX_DEBUG, 0x80);
	exynos8895_cr_write(d, EXYNOS8895_CR_TX_OVRD_DRV_LO,
			    0x4000 | (EXYNOS8895_SS_DEEMPH_3P5DB << 7) | EXYNOS8895_SS_SWING_FULL);
	exynos8895_cr_write(d, EXYNOS8895_CR_LOSLEVEL_OVRD_IN, 0xa409);
	exynos8895_cr_write(d, EXYNOS8895_CR_TX_VBOOSTLVL_OVRD, EXYNOS8895_SS_TX_VBOOST_HOST << 13);
	if (d->cal_port == 1)				/* cal.c:1075-1077, power reduction */
		exynos8895_dp_altmode_ss_disable_port0(d);
}

/*
 * Host mode UTMI = stock samsung_exynos_cal_usb3phy_config_host_mode() (cal.c:1138-1150):
 * PHYUTMI |= DMPULLDOWN | DPPULLDOWN, &= ~VBUSVLDEXTSEL | ~VBUSVLDEXT. Stock runs it on
 * window 0 (cal.c:1141 usbphy_info->regs_base), after dwc3 added the xHCI platform device
 * (S:drivers/usb/dwc3/otg.c:245-250, S:drivers/usb/host/xhci-plat.c:68-73).
 * PHYUTMI_DMPULLDOWN/DPPULLDOWN/VBUSVLDEXTSEL/VBUSVLDEXT = bits 2, 3, 10, 9
 * (S:drivers/phy/phy-samsung-usb3-cal.h:15-26).
 */
static void exynos8895_cal_host_mode(struct exynos5_usbdrd_phy *d)
{
	void __iomem *b0 = d->cal_base[0];
	u32 v;

	v = readl(b0 + EXYNOS5_DRD_PHYUTMI);
	v |= PHYUTMI_DMPULLDOWN | PHYUTMI_DPPULLDOWN;
	v &= ~(PHYUTMI_VBUSVLDEXTSEL | PHYUTMI_VBUSVLDEXT);
	writel(v, b0 + EXYNOS5_DRD_PHYUTMI);
}

/*
 * Device mode UTMI = what the CAL enable of .init left there: the pull-downs are cleared by
 * samsung_exynos_cal_usb3phy_enable (cal.c:965-966) and VBUSVLDEXTSEL/VBUSVLDEXT are set by
 * it for not_used_vbus_pad (cal.c:976-977, this board has no VBUS pad, FDT:7886; L3 P2
 * "U & 0x65f == 0x640"). The device tune itself already ran from .power_on (L3 5.1.6b), so
 * set_mode only restores these bits.
 */
static void exynos8895_cal_device_mode(struct exynos5_usbdrd_phy *d)
{
	void __iomem *b0 = d->cal_base[0];
	u32 v;

	v = readl(b0 + EXYNOS5_DRD_PHYUTMI);
	v &= ~(PHYUTMI_DMPULLDOWN | PHYUTMI_DPPULLDOWN);
	v |= PHYUTMI_VBUSVLDEXTSEL | PHYUTMI_VBUSVLDEXT;
	writel(v, b0 + EXYNOS5_DRD_PHYUTMI);
}

/*
 * dwc3 calls phy_set_mode() after dwc3_host_init() (A:drivers/usb/dwc3/core.c:250-254) and
 * after its core soft reset on every switch to device (:263-271). Host mode is the point
 * where stock re-tuned the PHY after the xHCI reset, so the tune and the pull-downs are
 * (re)written here. Nothing of this runs while the controller stays in device mode, so the
 * peripheral-only image behaves exactly as before.
 */
static int exynos8895_usbdrd_phy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *d = to_usbdrd_phy(inst);
	void __iomem *b0 = d->cal_base[0];

	if (inst->phy_cfg->id != EXYNOS5_DRDPHY_PIPE3)
		return 0;
	if (mode != PHY_MODE_USB_HOST && mode != PHY_MODE_USB_DEVICE)
		return 0;

	guard(mutex)(&d->phy_mutex);
	if (mode == PHY_MODE_USB_HOST) {
		WRITE_ONCE(d->host_mode, true);
		exynos8895_cal_tune_host(d);
		exynos8895_cal_host_mode(d);
	} else {
		WRITE_ONCE(d->host_mode, false);
		exynos8895_cal_device_mode(d);
	}
	dev_info(d->dev, "8895: %s mode, utmi=0x%08x\n",
		 READ_ONCE(d->host_mode) ? "host" : "device",
		 readl(b0 + EXYNOS5_DRD_PHYUTMI));

	return 0;
}

/* CAL disable = stock samsung_exynos_cal_usb3phy_disable, cal.c:1082-1136 */
static void exynos8895_cal_disable(struct exynos5_usbdrd_phy *d)
{
	void __iomem *b0 = d->cal_base[0];
	u32 v;

	v = readl(b0 + EXYNOS5_DRD_PHYCLKRST);
	v |= PHYCLKRST_EN_UTMISUSPEND | PHYCLKRST_COMMONONN | PHYCLKRST_RETENABLEN;	/* :1090-1092 */
	v &= ~(PHYCLKRST_REF_SSP_EN | PHYCLKRST_SSC_EN);				/* :1093-1094 */
	v &= ~PHYCLKRST_REFCLKSEL;
	v |= FIELD_PREP(PHYCLKRST_REFCLKSEL, EXYNOS8895_REFCLKSEL_CLKCORE);		/* :1096-1097 */
	v &= ~EXYNOS8895_PHYCLKRST_FSEL;
	v |= FIELD_PREP(EXYNOS8895_PHYCLKRST_FSEL, EXYNOS8895_FSEL_26M);		/* :1100-1101 */
	writel(v, b0 + EXYNOS5_DRD_PHYCLKRST);

	v = readl(b0 + EXYNOS5_DRD_PHYUTMI);
	v &= ~(PHYUTMI_IDPULLUP | PHYUTMI_DRVVBUS | PHYUTMI_VBUSVLDEXTSEL | PHYUTMI_VBUSVLDEXT);
	v |= PHYUTMI_FORCESUSPEND | PHYUTMI_FORCESLEEP;					/* :1104-1113 */
	writel(v, b0 + EXYNOS5_DRD_PHYUTMI);

	exynos8895_ss_power_down(d, false);						/* :1115-1117 */

	v = readl(b0 + EXYNOS5_DRD_LINKSYSTEM);
	v &= ~(LINKSYSTEM_FORCE_BVALID | LINKSYSTEM_FORCE_VBUSVALID);			/* :1124-1130 */
	writel(v, b0 + EXYNOS5_DRD_LINKSYSTEM);

	exynos8895_q_ch(b0, false);							/* :1133-1135 */
}

/*
 * Stock order (spec 5.1.8): the PMU de-isolation (phy_power_on) runs before the CAL enable
 * (phy_init) and a full tune follows every DCTL.CSFTRST (S:drivers/usb/dwc3/core.c:191-231,
 * 768-787). ACK dwc3 calls phy_init, CSFTRST, phy_power_on (A:drivers/usb/dwc3/core.c:1381,
 * 1385,1411) and on exit phy_power_off before phy_exit (:995-1002). So .init = rails, clocks,
 * de-isolate, CAL enable; .power_on = tune; .exit = CAL disable, isolate, clocks, rail. The
 * UTMI instance (index 0) is a no-op in every op, as in stock (usbdrd.c:295-299, 805-808).
 */
static int exynos8895_usbdrd_phy_init(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *d = to_usbdrd_phy(inst);
	unsigned int pmu = 0;
	int ret;

	if (inst->phy_cfg->id != EXYNOS5_DRDPHY_PIPE3)
		return 0;
	if (d->vdd_dp) {
		ret = regulator_enable(d->vdd_dp);			/* S:drivers/usb/notify/usb_notifier.c:276 */
		if (ret)
			return ret;
	}
	ret = clk_bulk_prepare_enable(d->drv_data->n_clks, d->clks);
	if (ret)
		goto err_rail;
	ret = clk_bulk_prepare_enable(d->drv_data->n_core_clks, d->core_clks);
	if (ret)
		goto err_clk;
	if (clk_get_rate(d->core_clks[0].clk) != EXYNOS8895_REFCLK_RATE) {
		dev_err(d->dev, "8895: ref clock %lu Hz, CAL needs 26 MHz\n",
			clk_get_rate(d->core_clks[0].clk));
		ret = -EINVAL;
		goto err_core;
	}
	inst->phy_cfg->phy_isol(inst, false);			/* stock power_on, usbdrd.c:966 */
	scoped_guard(mutex, &d->phy_mutex) {			/* vs the orientation switch */
		exynos8895_cal_enable(d);
		/* D+/D- reach the AP through the MUIC driver's own attach path (single owner) */
		d->cal_inited = true;
	}
	regmap_read(inst->reg_pmu, inst->pmu_offset, &pmu);
	dev_info(d->dev, "8895: pmu704=0x%08x\n", pmu);
	return 0;

err_core:
	clk_bulk_disable_unprepare(d->drv_data->n_core_clks, d->core_clks);
err_clk:
	clk_bulk_disable_unprepare(d->drv_data->n_clks, d->clks);
err_rail:
	if (d->vdd_dp)
		regulator_disable(d->vdd_dp);
	return ret;
}

static int exynos8895_usbdrd_phy_power_on(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *d;

	if (inst->phy_cfg->id != EXYNOS5_DRDPHY_PIPE3)
		return 0;
	d = to_usbdrd_phy(inst);
	/* after the dwc3_core_init CSFTRST (A:drivers/usb/dwc3/core.c:1385 -> 1411) */
	guard(mutex)(&d->phy_mutex);
	exynos8895_cal_tune_dev(d);
	WRITE_ONCE(d->tune_armed, true);	/* later CSFTRSTs re-tune from the dwc3 hook */
	return 0;
}

static int exynos8895_usbdrd_phy_exit(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *d = to_usbdrd_phy(inst);

	if (inst->phy_cfg->id != EXYNOS5_DRDPHY_PIPE3)
		return 0;
	scoped_guard(mutex, &d->phy_mutex) {
		WRITE_ONCE(d->tune_armed, false);	/* no re-tune of a PHY that goes down */
		d->cal_inited = false;
		exynos8895_cal_disable(d);
	}
	inst->phy_cfg->phy_isol(inst, true);			/* usbdrd.c:989 */
	clk_bulk_disable_unprepare(d->drv_data->n_core_clks, d->core_clks);
	clk_bulk_disable_unprepare(d->drv_data->n_clks, d->clks);
	if (d->vdd_dp)
		regulator_disable(d->vdd_dp);
	return 0;
}

/*
 * Type-C orientation switch (spec L6G 5.3.3, D-6; closes L3 O22). Stock re-read gpi1-7 in
 * pipe3_init at every attach (S:drivers/phy/phy-exynos-usbdrd.c:767-770, called per attach
 * from S:drivers/usb/dwc3/otg.c:310-316), used_phy_port = !level: NORMAL (level 0) -> port 1,
 * REVERSE (level 1) -> port 0. The S2MM005 driver owns the pin and calls this before it sets
 * the USB role. Not initialised: the next .init uses the port. Initialised on the other
 * port: stock's per-attach exit + init pair on the CAL (clocks and rails stay on), then the
 * device tune if .power_on had tuned, or the host tune and config_host_mode
 * when dwc3 has the controller in host mode (L6G 5.3.2).
 */
static int exynos8895_orien_sw_set(struct typec_switch_dev *sw,
				   enum typec_orientation orientation)
{
	struct exynos5_usbdrd_phy *d = typec_switch_get_drvdata(sw);
	bool armed;
	int port;

	guard(mutex)(&d->phy_mutex);
	d->orientation = orientation;
	if (orientation == TYPEC_ORIENTATION_NONE)
		return 0;			/* the last port stays until the next attach */

	port = orientation == TYPEC_ORIENTATION_NORMAL ? 1 : 0;
	d->sw_port = port;
	if (!d->cal_inited)
		return 0;

	/*
	 * The dwc3 CSFTRST re-tune runs from a tracepoint with preemption disabled, cannot
	 * take phy_mutex and briefly rewrites cal_port: disarm it and wait until a re-tune
	 * already past its check has finished before cal_port is read.
	 */
	armed = READ_ONCE(d->tune_armed);
	WRITE_ONCE(d->tune_armed, false);
	synchronize_rcu();
	if (port != d->cal_port) {
		dev_info(d->dev, "8895: orientation %s, re-init on port %d\n",
			 orientation == TYPEC_ORIENTATION_NORMAL ? "normal" : "reverse", port);
		exynos8895_cal_disable(d);
		exynos8895_cal_enable(d);
		if (armed) {
			if (READ_ONCE(d->host_mode)) {
				exynos8895_cal_tune_host(d);
				exynos8895_cal_host_mode(d);
			} else {
				exynos8895_cal_tune_dev(d);
			}
		}
	}
	if (armed)
		WRITE_ONCE(d->tune_armed, true);
	return 0;
}

static const struct phy_ops exynos8895_usbdrd_phy_ops = {
	.init		= exynos8895_usbdrd_phy_init,
	.exit		= exynos8895_usbdrd_phy_exit,
	.power_on	= exynos8895_usbdrd_phy_power_on,
	.set_mode	= exynos8895_usbdrd_phy_set_mode,
	.owner		= THIS_MODULE,
};

#ifdef CONFIG_GREATLTE_USB_DIAG
/* DEBUG: the heartbeat's read-only view of the one Exynos8895 USB PHY (spec 8.1, 8.3 R2/R8) */
static struct exynos5_usbdrd_phy *exynos8895_diag_phy;
static int exynos8895_diag_cr1002 = -1;

int exynos8895_usbdrd_get_snap(struct greatlte_l3_phy_snap *s)
{
	struct exynos5_usbdrd_phy *d = READ_ONCE(exynos8895_diag_phy);

	if (!d)
		return -ENODEV;
	s->ver[0] = d->cal_ver[0];
	s->ver[1] = d->cal_ver[1];
	s->probed = d->cal_probed;
	s->shift = d->cal_probed && !d->cal_zero;
	s->port = d->cal_port;
	s->inits = d->init_count;
	s->retunes = READ_ONCE(d->retunes);
	s->cr_timeouts = READ_ONCE(d->cr_timeouts);
	s->cr1002_before = READ_ONCE(exynos8895_diag_cr1002);
	s->armed = READ_ONCE(d->tune_armed);
	return 0;
}

static void exynos8895_diag_clear(void *data)
{
	WRITE_ONCE(exynos8895_diag_phy, NULL);	/* probe failed or deferred after probe_extra */
}
#endif

#ifdef CONFIG_ANDROID_VENDOR_HOOKS
/*
 * Re-tune after every dwc3 CSFTRST (spec 5.1.6b). Stock keeps "every CSFTRST is followed by a
 * full PHY tune before RUN_STOP": at attach (S:drivers/usb/dwc3/core.c:768-787) and at every
 * gadget pullup(1) (S:drivers/usb/dwc3/gadget.c:1962-1978). ACK issues a CSFTRST at every
 * soft-connect (A:drivers/usb/dwc3/gadget.c:2782) and sets RUN_STOP right after it (:2788)
 * with no PHY op in between; its only callback there is this vendor hook, fired with
 * POST_SOFT_RESET after the CSFTRST poll (A:drivers/usb/dwc3/core.c:369-371, 384-386).
 * Called with preemption disabled (tracepoint): the tune only uses readl/writel/udelay.
 */
static void exynos8895_vh_dwc3_soft_reset(void *data, struct device *dwc3_dev,
					  void __iomem *dwc3_regs, struct phy **usb3_phy,
					  enum soft_reset_phase phase)
{
	struct exynos5_usbdrd_phy *d = data;

	if (phase != POST_SOFT_RESET || !usb3_phy ||
	    usb3_phy[0] != d->phys[EXYNOS5_DRDPHY_PIPE3].phy)
		return;				/* not this PHY */
	if (!READ_ONCE(d->tune_armed))
		return;				/* core-init CSFTRST: .power_on tunes after it */
#ifdef CONFIG_GREATLTE_USB_DIAG
	/*
	 * DEBUG, spec 8.3 R8 / O21: CR[0x1002] after the CSFTRST and before the re-tune (the
	 * previous tune wrote 0x4b7f): an extra CR read stock never does at this point.
	 */
	WRITE_ONCE(exynos8895_diag_cr1002, exynos8895_cr_read(d, EXYNOS8895_CR_TX_OVRD_DRV_LO));
#endif
	exynos8895_cal_tune_dev(d);		/* as S:drivers/usb/dwc3/gadget.c:1965-1966 */
	d->retunes++;
#ifdef CONFIG_GREATLTE_USB_DIAG
	dev_info(d->dev, "8895: re-tune #%u after CSFTRST (cr1002 before=0x%04x) cr_timeouts=%u\n",
		 d->retunes, exynos8895_diag_cr1002, d->cr_timeouts);
#else
	dev_info(d->dev, "8895: re-tune #%u after CSFTRST cr_timeouts=%u\n",
		 d->retunes, d->cr_timeouts);
#endif
}

static void exynos8895_unregister_vh(void *data)
{
	unregister_trace_android_vh_dwc3_core_soft_reset(exynos8895_vh_dwc3_soft_reset, data);
	tracepoint_synchronize_unregister();
}
#endif

static int exynos8895_usbdrd_probe_extra(struct exynos5_usbdrd_phy *d,
					 struct platform_device *pdev)
{
	int ret = 0;

	d->reg_phy2 = devm_platform_ioremap_resource(pdev, 1);		/* usbdrd.c:1118-1123 */
	if (IS_ERR(d->reg_phy2))
		return PTR_ERR(d->reg_phy2);
	d->reg_refclk = devm_platform_ioremap_resource(pdev, 2);	/* usbdrd.c:568-578 */
	if (IS_ERR(d->reg_refclk))
		return PTR_ERR(d->reg_refclk);
	d->cal_base[0] = d->reg_phy;					/* usbdrd.c:656 */
	d->cal_base[1] = d->reg_phy2;					/* usbdrd.c:646 */
	d->cal_port = 0;						/* usbdrd.c:647 */

	/* with an orientation switch the Type-C controller owns gpi1-7 (L6G 5.3.3) */
	if (!device_property_present(d->dev, "orientation-switch")) {
		d->port_gpio = devm_gpiod_get_optional(d->dev, "samsung,phy-port", GPIOD_IN);
		if (IS_ERR(d->port_gpio))
			return dev_err_probe(d->dev, PTR_ERR(d->port_gpio), "phy-port gpio\n");
	}

	d->vdd_dp = devm_regulator_get_optional(d->dev, "vdd18-dp");
	if (IS_ERR(d->vdd_dp)) {
		if (PTR_ERR(d->vdd_dp) != -ENODEV)
			return dev_err_probe(d->dev, PTR_ERR(d->vdd_dp), "vdd18-dp\n");
		d->vdd_dp = NULL;
	}

#ifdef CONFIG_ANDROID_VENDOR_HOOKS
	/*
	 * Registered once per PHY instance; the handler compares against phys[PIPE3].phy at
	 * call time (the two struct phy are created after this hook runs).
	 */
	ret = register_trace_android_vh_dwc3_core_soft_reset(exynos8895_vh_dwc3_soft_reset, d);
	if (ret)
		return dev_err_probe(d->dev, ret, "dwc3 soft-reset hook\n");
	ret = devm_add_action_or_reset(d->dev, exynos8895_unregister_vh, d);
	if (ret)
		return ret;
#else
	dev_warn(d->dev, "8895: no vendor hooks, PHY not re-tuned after soft-connect CSFTRST\n");
#endif
#ifdef CONFIG_GREATLTE_USB_DIAG
	WRITE_ONCE(exynos8895_diag_phy, d);	/* one USB PHY on this SoC */
	ret = devm_add_action_or_reset(d->dev, exynos8895_diag_clear, NULL);
#endif
	return ret;
}

static void exynos5_usbdrd_gs101_pipe3_init(struct exynos5_usbdrd_phy *phy_drd)
{
	void __iomem *regs_pma = phy_drd->reg_pma;
	void __iomem *regs_phy = phy_drd->reg_phy;
	u32 reg;

	exynos5_usbdrd_usbdp_g2_v4_ctrl_pma_ready(phy_drd);

	/* force aux off */
	reg = readl(regs_pma + EXYNOS9_PMA_USBDP_CMN_REG0008);
	reg &= ~CMN_REG0008_AUX_EN;
	reg |= CMN_REG0008_OVRD_AUX_EN;
	writel(reg, regs_pma + EXYNOS9_PMA_USBDP_CMN_REG0008);

	exynos5_usbdrd_apply_phy_tunes(phy_drd, PTS_PIPE3_PREINIT);
	exynos5_usbdrd_apply_phy_tunes(phy_drd, PTS_PIPE3_INIT);
	exynos5_usbdrd_apply_phy_tunes(phy_drd, PTS_PIPE3_POSTINIT);

	exynos5_usbdrd_usbdp_g2_v4_pma_lane_mux_sel(phy_drd);

	/* reset release from port */
	reg = readl(regs_phy + EXYNOS850_DRD_SECPMACTL);
	reg &= ~(SECPMACTL_PMA_TRSV_SW_RST | SECPMACTL_PMA_CMN_SW_RST |
		 SECPMACTL_PMA_INIT_SW_RST);
	writel(reg, regs_phy + EXYNOS850_DRD_SECPMACTL);

	if (!exynos5_usbdrd_usbdp_g2_v4_pma_check_pll_lock(phy_drd))
		exynos5_usbdrd_usbdp_g2_v4_pma_check_cdr_lock(phy_drd);
}

static int exynos5_usbdrd_gs101_phy_init(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);
	int ret;

	if (inst->phy_cfg->id == EXYNOS5_DRDPHY_UTMI) {
		/* Power-on PHY ... */
		ret = regulator_bulk_enable(phy_drd->drv_data->n_regulators,
					    phy_drd->regulators);
		if (ret) {
			dev_err(phy_drd->dev,
				"Failed to enable PHY regulator(s)\n");
			return ret;
		}
	}
	/*
	 * ... and ungate power via PMU. Without this here, we get an SError
	 * trying to access PMA registers
	 */
	exynos5_usbdrd_phy_isol(inst, false);

	return exynos850_usbdrd_phy_init(phy);
}

static int exynos5_usbdrd_gs101_phy_exit(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);
	int ret;

	if (inst->phy_cfg->id == EXYNOS5_DRDPHY_UTMI) {
		ret = exynos850_usbdrd_phy_exit(phy);
		if (ret)
			return ret;
	}

	exynos5_usbdrd_phy_isol(inst, true);

	if (inst->phy_cfg->id != EXYNOS5_DRDPHY_UTMI)
		return 0;

	return regulator_bulk_disable(phy_drd->drv_data->n_regulators,
				      phy_drd->regulators);
}

static const struct phy_ops gs101_usbdrd_phy_ops = {
	.init		= exynos5_usbdrd_gs101_phy_init,
	.exit		= exynos5_usbdrd_gs101_phy_exit,
	.owner		= THIS_MODULE,
};

static int exynos5_usbdrd_phy_clk_handle(struct exynos5_usbdrd_phy *phy_drd)
{
	int ret;
	struct clk *ref_clk;
	unsigned long ref_rate;

	phy_drd->clks = devm_kcalloc(phy_drd->dev, phy_drd->drv_data->n_clks,
				     sizeof(*phy_drd->clks), GFP_KERNEL);
	if (!phy_drd->clks)
		return -ENOMEM;

	for (int i = 0; i < phy_drd->drv_data->n_clks; ++i)
		phy_drd->clks[i].id = phy_drd->drv_data->clk_names[i];

	ret = devm_clk_bulk_get(phy_drd->dev, phy_drd->drv_data->n_clks,
				phy_drd->clks);
	if (ret)
		return dev_err_probe(phy_drd->dev, ret,
				     "failed to get phy clock(s)\n");

	phy_drd->core_clks = devm_kcalloc(phy_drd->dev,
					  phy_drd->drv_data->n_core_clks,
					  sizeof(*phy_drd->core_clks),
					  GFP_KERNEL);
	if (!phy_drd->core_clks)
		return -ENOMEM;

	for (int i = 0; i < phy_drd->drv_data->n_core_clks; ++i)
		phy_drd->core_clks[i].id = phy_drd->drv_data->core_clk_names[i];

	ret = devm_clk_bulk_get(phy_drd->dev, phy_drd->drv_data->n_core_clks,
				phy_drd->core_clks);
	if (ret)
		return dev_err_probe(phy_drd->dev, ret,
				     "failed to get phy core clock(s)\n");

	if (phy_drd->drv_data->n_core_clks) {
		ref_clk = NULL;
		for (int i = 0; i < phy_drd->drv_data->n_core_clks; ++i) {
			if (!strcmp(phy_drd->core_clks[i].id, "ref")) {
				ref_clk = phy_drd->core_clks[i].clk;
				break;
			}
		}
		if (!ref_clk)
			return dev_err_probe(phy_drd->dev, -ENODEV,
					     "failed to find phy reference clock\n");

		ref_rate = clk_get_rate(ref_clk);
		ret = exynos5_rate_to_clk(ref_rate, &phy_drd->extrefclk);
		if (ret)
			return dev_err_probe(phy_drd->dev, ret,
					     "clock rate (%ld) not supported\n",
					     ref_rate);
	}

	return 0;
}

static const struct exynos5_usbdrd_phy_config phy_cfg_exynos2200[] = {
	{
		.id		= EXYNOS5_DRDPHY_UTMI,
		.phy_isol	= exynos5_usbdrd_phy_isol,
		.phy_init	= exynos2200_usbdrd_utmi_init,
	},
};

static int exynos5_usbdrd_orien_sw_set(struct typec_switch_dev *sw,
				       enum typec_orientation orientation)
{
	struct exynos5_usbdrd_phy *phy_drd = typec_switch_get_drvdata(sw);
	int ret;

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret) {
		dev_err(phy_drd->dev, "Failed to enable PHY clocks(s)\n");
		return ret;
	}

	scoped_guard(mutex, &phy_drd->phy_mutex) {
		void __iomem * const regs_base = phy_drd->reg_phy;
		unsigned int reg;

		if (orientation == TYPEC_ORIENTATION_NONE) {
			reg = readl(regs_base + EXYNOS850_DRD_UTMI);
			reg &= ~(UTMI_FORCE_VBUSVALID | UTMI_FORCE_BVALID);
			writel(reg, regs_base +  EXYNOS850_DRD_UTMI);

			reg = readl(regs_base + EXYNOS850_DRD_HSP);
			reg |= HSP_VBUSVLDEXTSEL;
			reg &= ~HSP_VBUSVLDEXT;
			writel(reg, regs_base + EXYNOS850_DRD_HSP);
		} else {
			reg = readl(regs_base + EXYNOS850_DRD_UTMI);
			reg |= UTMI_FORCE_VBUSVALID | UTMI_FORCE_BVALID;
			writel(reg, regs_base +  EXYNOS850_DRD_UTMI);

			reg = readl(regs_base + EXYNOS850_DRD_HSP);
			reg |= HSP_VBUSVLDEXTSEL | HSP_VBUSVLDEXT;
			writel(reg, regs_base + EXYNOS850_DRD_HSP);
		}

		phy_drd->orientation = orientation;
	}

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	return 0;
}

static void exynos5_usbdrd_orien_switch_unregister(void *data)
{
	struct exynos5_usbdrd_phy *phy_drd = data;

	typec_switch_unregister(phy_drd->sw);
}

static int exynos5_usbdrd_setup_notifiers(struct exynos5_usbdrd_phy *phy_drd)
{
	int ret;

	if (!IS_ENABLED(CONFIG_TYPEC))
		return 0;

	if (device_property_present(phy_drd->dev, "orientation-switch")) {
		struct typec_switch_desc sw_desc = { };

		sw_desc.drvdata = phy_drd;
		sw_desc.fwnode = dev_fwnode(phy_drd->dev);
		sw_desc.set = phy_drd->drv_data->orien_sw_set ?: exynos5_usbdrd_orien_sw_set;

		phy_drd->sw = typec_switch_register(phy_drd->dev, &sw_desc);
		if (IS_ERR(phy_drd->sw))
			return dev_err_probe(phy_drd->dev,
					     PTR_ERR(phy_drd->sw),
					     "Failed to register TypeC orientation switch\n");

		ret = devm_add_action_or_reset(phy_drd->dev,
					       exynos5_usbdrd_orien_switch_unregister,
					       phy_drd);
		if (ret)
			return dev_err_probe(phy_drd->dev, ret,
					     "Failed to register TypeC orientation devm action\n");
	}

	return 0;
}

static const struct exynos5_usbdrd_phy_config phy_cfg_exynos5[] = {
	{
		.id		= EXYNOS5_DRDPHY_UTMI,
		.phy_isol	= exynos5_usbdrd_phy_isol,
		.phy_init	= exynos5_usbdrd_utmi_init,
		.set_refclk	= exynos5_usbdrd_utmi_set_refclk,
	},
	{
		.id		= EXYNOS5_DRDPHY_PIPE3,
		.phy_isol	= exynos5_usbdrd_phy_isol,
		.phy_init	= exynos5_usbdrd_pipe3_init,
		.set_refclk	= exynos5_usbdrd_pipe3_set_refclk,
	},
};

static const struct exynos5_usbdrd_phy_config phy_cfg_exynos7870[] = {
	{
		.id		= EXYNOS5_DRDPHY_UTMI,
		.phy_isol	= exynos7870_usbdrd_phy_isol,
		.phy_init	= exynos7870_usbdrd_utmi_init,
	},
};

static const struct exynos5_usbdrd_phy_config phy_cfg_exynos850[] = {
	{
		.id		= EXYNOS5_DRDPHY_UTMI,
		.phy_isol	= exynos5_usbdrd_phy_isol,
		.phy_init	= exynos850_usbdrd_utmi_init,
	},
};

/* both instances: the probe fills phys[i].phy_cfg = &phy_cfg[i] for i = 0, 1 */
static const struct exynos5_usbdrd_phy_config phy_cfg_exynos8895[] = {
	{
		.id		= EXYNOS5_DRDPHY_UTMI,
		.phy_isol	= exynos5_usbdrd_phy_isol,
	},
	{
		.id		= EXYNOS5_DRDPHY_PIPE3,
		.phy_isol	= exynos5_usbdrd_phy_isol,
	},
};

static
const struct exynos5_usbdrd_phy_tuning exynos7870_tunes_utmi_postinit[] = {
	PHY_TUNING_ENTRY_PHY(EXYNOS5_DRD_PHYPARAM0,
			     (PHYPARAM0_TXVREFTUNE | PHYPARAM0_TXRISETUNE |
			      PHYPARAM0_TXRESTUNE | PHYPARAM0_TXPREEMPPULSETUNE |
			      PHYPARAM0_TXPREEMPAMPTUNE | PHYPARAM0_TXHSXVTUNE |
			      PHYPARAM0_TXFSLSTUNE | PHYPARAM0_SQRXTUNE |
			      PHYPARAM0_OTGTUNE | PHYPARAM0_COMPDISTUNE),
			     (FIELD_PREP_CONST(PHYPARAM0_TXVREFTUNE, 14) |
			      FIELD_PREP_CONST(PHYPARAM0_TXRISETUNE, 1) |
			      FIELD_PREP_CONST(PHYPARAM0_TXRESTUNE, 3) |
			      FIELD_PREP_CONST(PHYPARAM0_TXPREEMPAMPTUNE, 0) |
			      FIELD_PREP_CONST(PHYPARAM0_TXHSXVTUNE, 0) |
			      FIELD_PREP_CONST(PHYPARAM0_TXFSLSTUNE, 3) |
			      FIELD_PREP_CONST(PHYPARAM0_SQRXTUNE, 6) |
			      FIELD_PREP_CONST(PHYPARAM0_OTGTUNE, 2) |
			      FIELD_PREP_CONST(PHYPARAM0_COMPDISTUNE, 3))),
	PHY_TUNING_ENTRY_LAST
};

static const struct exynos5_usbdrd_phy_tuning *exynos7870_tunes[PTS_MAX] = {
	[PTS_UTMI_POSTINIT] = exynos7870_tunes_utmi_postinit,
};

static const char * const exynos5_clk_names[] = {
	"phy",
};

static const char * const exynos5_core_clk_names[] = {
	"ref",
};

static const char * const exynos5433_core_clk_names[] = {
	"ref", "phy_pipe", "phy_utmi", "itp",
};

static const char * const exynos5_regulator_names[] = {
	"vbus", "vbus-boost",
};

static const struct exynos5_usbdrd_phy_drvdata exynos2200_usb32drd_phy = {
	.phy_cfg		= phy_cfg_exynos2200,
	.phy_ops		= &exynos2200_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy	= EXYNOS2200_PHY_CTRL_USB20,
	.clk_names		= exynos5_clk_names,
	.n_clks			= ARRAY_SIZE(exynos5_clk_names),
	/* clocks and regulators are specific to the underlying PHY blocks */
	.core_clk_names		= NULL,
	.n_core_clks		= 0,
	.regulator_names	= NULL,
	.n_regulators		= 0,
};

static const struct exynos5_usbdrd_phy_drvdata exynos5420_usbdrd_phy = {
	.phy_cfg		= phy_cfg_exynos5,
	.phy_ops		= &exynos5_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy	= EXYNOS5_USBDRD_PHY_CONTROL,
	.pmu_offset_usbdrd1_phy	= EXYNOS5420_USBDRD1_PHY_CONTROL,
	.clk_names		= exynos5_clk_names,
	.n_clks			= ARRAY_SIZE(exynos5_clk_names),
	.core_clk_names		= exynos5_core_clk_names,
	.n_core_clks		= ARRAY_SIZE(exynos5_core_clk_names),
	.regulator_names	= exynos5_regulator_names,
	.n_regulators		= ARRAY_SIZE(exynos5_regulator_names),
};

static const struct exynos5_usbdrd_phy_drvdata exynos5250_usbdrd_phy = {
	.phy_cfg		= phy_cfg_exynos5,
	.phy_ops		= &exynos5_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy	= EXYNOS5_USBDRD_PHY_CONTROL,
	.clk_names		= exynos5_clk_names,
	.n_clks			= ARRAY_SIZE(exynos5_clk_names),
	.core_clk_names		= exynos5_core_clk_names,
	.n_core_clks		= ARRAY_SIZE(exynos5_core_clk_names),
	.regulator_names	= exynos5_regulator_names,
	.n_regulators		= ARRAY_SIZE(exynos5_regulator_names),
};

static const struct exynos5_usbdrd_phy_drvdata exynos5433_usbdrd_phy = {
	.phy_cfg		= phy_cfg_exynos5,
	.phy_ops		= &exynos5_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy	= EXYNOS5_USBDRD_PHY_CONTROL,
	.pmu_offset_usbdrd1_phy	= EXYNOS5433_USBHOST30_PHY_CONTROL,
	.clk_names		= exynos5_clk_names,
	.n_clks			= ARRAY_SIZE(exynos5_clk_names),
	.core_clk_names		= exynos5433_core_clk_names,
	.n_core_clks		= ARRAY_SIZE(exynos5433_core_clk_names),
	.regulator_names	= exynos5_regulator_names,
	.n_regulators		= ARRAY_SIZE(exynos5_regulator_names),
};

static const struct exynos5_usbdrd_phy_drvdata exynos7_usbdrd_phy = {
	.phy_cfg		= phy_cfg_exynos5,
	.phy_ops		= &exynos5_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy	= EXYNOS5_USBDRD_PHY_CONTROL,
	.clk_names		= exynos5_clk_names,
	.n_clks			= ARRAY_SIZE(exynos5_clk_names),
	.core_clk_names		= exynos5433_core_clk_names,
	.n_core_clks		= ARRAY_SIZE(exynos5433_core_clk_names),
	.regulator_names	= exynos5_regulator_names,
	.n_regulators		= ARRAY_SIZE(exynos5_regulator_names),
};

static const struct exynos5_usbdrd_phy_drvdata exynos7870_usbdrd_phy = {
	.phy_cfg		= phy_cfg_exynos7870,
	.phy_tunes		= exynos7870_tunes,
	.phy_ops		= &exynos7870_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy	= EXYNOS5_USBDRD_PHY_CONTROL,
	.clk_names		= exynos5_clk_names,
	.n_clks			= ARRAY_SIZE(exynos5_clk_names),
	.core_clk_names		= exynos5_core_clk_names,
	.n_core_clks		= ARRAY_SIZE(exynos5_core_clk_names),
	.regulator_names	= exynos5_regulator_names,
	.n_regulators		= ARRAY_SIZE(exynos5_regulator_names),
};

static const struct exynos5_usbdrd_phy_drvdata exynos850_usbdrd_phy = {
	.phy_cfg		= phy_cfg_exynos850,
	.phy_ops		= &exynos850_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy	= EXYNOS5_USBDRD_PHY_CONTROL,
	.clk_names		= exynos5_clk_names,
	.n_clks			= ARRAY_SIZE(exynos5_clk_names),
	.core_clk_names		= exynos5_core_clk_names,
	.n_core_clks		= ARRAY_SIZE(exynos5_core_clk_names),
	.regulator_names	= exynos5_regulator_names,
	.n_regulators		= ARRAY_SIZE(exynos5_regulator_names),
};

static const struct exynos5_usbdrd_phy_drvdata exynos8895_usbdrd_phy = {
	.phy_cfg		= phy_cfg_exynos8895,
	.phy_ops		= &exynos8895_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy	= EXYNOS5_USBDRD_PHY_CONTROL,	/* 0x704 = S:drivers/phy/phy-exynos-usbdrd.h:16 */
	.clk_names		= exynos5_clk_names,		/* "phy" */
	.n_clks			= ARRAY_SIZE(exynos5_clk_names),
	.core_clk_names		= exynos5_core_clk_names,	/* "ref" */
	.n_core_clks		= ARRAY_SIZE(exynos5_core_clk_names),
	.probe_extra		= exynos8895_usbdrd_probe_extra,
	.orien_sw_set		= exynos8895_orien_sw_set,
	/* no regulator_names: this PHY switches no VBUS (no VBUS pad, FDT:7886) */
};

static const struct exynos5_usbdrd_phy_tuning exynos990_tunes_utmi_postinit[] = {
	PHY_TUNING_ENTRY_PHY(EXYNOS850_DRD_HSPPARACON,
			     (HSPPARACON_TXVREF |
			      HSPPARACON_TXPREEMPAMP | HSPPARACON_SQRX |
			      HSPPARACON_COMPDIS),
			     (FIELD_PREP_CONST(HSPPARACON_TXVREF, 7) |
			      FIELD_PREP_CONST(HSPPARACON_TXPREEMPAMP, 3) |
			      FIELD_PREP_CONST(HSPPARACON_SQRX, 5) |
			      FIELD_PREP_CONST(HSPPARACON_COMPDIS, 7))),
	PHY_TUNING_ENTRY_LAST
};

static const struct exynos5_usbdrd_phy_tuning *exynos990_tunes[PTS_MAX] = {
	[PTS_UTMI_POSTINIT] = exynos990_tunes_utmi_postinit,
};

static const struct exynos5_usbdrd_phy_drvdata exynos990_usbdrd_phy = {
	.phy_cfg		= phy_cfg_exynos850,
	.phy_ops		= &exynos850_usbdrd_phy_ops,
	.phy_tunes		= exynos990_tunes,
	.pmu_offset_usbdrd0_phy	= EXYNOS990_PHY_CTRL_USB20,
	.clk_names		= exynos5_clk_names,
	.n_clks			= ARRAY_SIZE(exynos5_clk_names),
	.core_clk_names		= exynos5_core_clk_names,
	.n_core_clks		= ARRAY_SIZE(exynos5_core_clk_names),
	.regulator_names	= exynos5_regulator_names,
	.n_regulators		= ARRAY_SIZE(exynos5_regulator_names),
};

static const struct exynos5_usbdrd_phy_config phy_cfg_gs101[] = {
	{
		.id		= EXYNOS5_DRDPHY_UTMI,
		.phy_isol	= exynos5_usbdrd_phy_isol,
		.phy_init	= exynos850_usbdrd_utmi_init,
	},
	{
		.id		= EXYNOS5_DRDPHY_PIPE3,
		.phy_isol	= exynos5_usbdrd_phy_isol,
		.phy_init	= exynos5_usbdrd_gs101_pipe3_init,
	},
};

static const struct exynos5_usbdrd_phy_tuning gs101_tunes_utmi_postinit[] = {
	PHY_TUNING_ENTRY_PHY(EXYNOS850_DRD_HSPPARACON,
			     (HSPPARACON_TXVREF | HSPPARACON_TXRES |
			      HSPPARACON_TXPREEMPAMP | HSPPARACON_SQRX |
			      HSPPARACON_COMPDIS),
			     (FIELD_PREP_CONST(HSPPARACON_TXVREF, 6) |
			      FIELD_PREP_CONST(HSPPARACON_TXRES, 1) |
			      FIELD_PREP_CONST(HSPPARACON_TXPREEMPAMP, 3) |
			      FIELD_PREP_CONST(HSPPARACON_SQRX, 5) |
			      FIELD_PREP_CONST(HSPPARACON_COMPDIS, 7))),
	PHY_TUNING_ENTRY_LAST
};

static const struct exynos5_usbdrd_phy_tuning gs101_tunes_pipe3_preinit[] = {
	/* preinit */
	/* CDR data mode exit GEN1 ON / GEN2 OFF */
	PHY_TUNING_ENTRY_PMA(0x0c8c, -1, 0xff),
	PHY_TUNING_ENTRY_PMA(0x1c8c, -1, 0xff),
	PHY_TUNING_ENTRY_PMA(0x0c9c, -1, 0x7d),
	PHY_TUNING_ENTRY_PMA(0x1c9c, -1, 0x7d),
	/* improve EDS distribution */
	PHY_TUNING_ENTRY_PMA(0x0e7c, -1, 0x06),
	PHY_TUNING_ENTRY_PMA(0x09e0, -1, 0x00),
	PHY_TUNING_ENTRY_PMA(0x09e4, -1, 0x36),
	PHY_TUNING_ENTRY_PMA(0x1e7c, -1, 0x06),
	PHY_TUNING_ENTRY_PMA(0x19e0, -1, 0x00),
	PHY_TUNING_ENTRY_PMA(0x19e4, -1, 0x36),
	/* fix bootloader bug */
	PHY_TUNING_ENTRY_PMA(0x1e90, -1, 0x02),
	PHY_TUNING_ENTRY_PMA(0x1e94, -1, 0x0b),
	/* improve LVCC */
	PHY_TUNING_ENTRY_PMA(0x08f0, -1, 0x30),
	PHY_TUNING_ENTRY_PMA(0x18f0, -1, 0x30),
	/* LFPS RX VIH shmoo hole */
	PHY_TUNING_ENTRY_PMA(0x0a08, -1, 0x0c),
	PHY_TUNING_ENTRY_PMA(0x1a08, -1, 0x0c),
	/* remove unrelated option for v4 phy */
	PHY_TUNING_ENTRY_PMA(0x0a0c, -1, 0x05),
	PHY_TUNING_ENTRY_PMA(0x1a0c, -1, 0x05),
	/* improve Gen2 LVCC */
	PHY_TUNING_ENTRY_PMA(0x00f8, -1, 0x1c),
	PHY_TUNING_ENTRY_PMA(0x00fc, -1, 0x54),
	/* Change Vth of RCV_DET because of TD 7.40 Polling Retry Test */
	PHY_TUNING_ENTRY_PMA(0x104c, -1, 0x07),
	PHY_TUNING_ENTRY_PMA(0x204c, -1, 0x07),
	/* reduce Ux Exit time, assuming 26MHz clock */
	/* Gen1 */
	PHY_TUNING_ENTRY_PMA(0x0ca8, -1, 0x00),
	PHY_TUNING_ENTRY_PMA(0x0cac, -1, 0x04),
	PHY_TUNING_ENTRY_PMA(0x1ca8, -1, 0x00),
	PHY_TUNING_ENTRY_PMA(0x1cac, -1, 0x04),
	/* Gen2 */
	PHY_TUNING_ENTRY_PMA(0x0cb8, -1, 0x00),
	PHY_TUNING_ENTRY_PMA(0x0cbc, -1, 0x04),
	PHY_TUNING_ENTRY_PMA(0x1cb8, -1, 0x00),
	PHY_TUNING_ENTRY_PMA(0x1cbc, -1, 0x04),
	/* RX impedance setting */
	PHY_TUNING_ENTRY_PMA(0x0bb0, 0x03, 0x01),
	PHY_TUNING_ENTRY_PMA(0x0bb4, 0xf0, 0xa0),
	PHY_TUNING_ENTRY_PMA(0x1bb0, 0x03, 0x01),
	PHY_TUNING_ENTRY_PMA(0x1bb4, 0xf0, 0xa0),

	PHY_TUNING_ENTRY_LAST
};

static const struct exynos5_usbdrd_phy_tuning gs101_tunes_pipe3_init[] = {
	/* init */
	/* abnormal common pattern mask */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_BACK_END_MODE_VEC,
			     BACK_END_MODE_VEC_DISABLE_DATA_MASK, 0),
	/* de-serializer enabled when U2 */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_OUT_VEC_2, PCS_OUT_VEC_B4_DYNAMIC,
			     PCS_OUT_VEC_B4_SEL_OUT),
	/* TX Keeper Disable, Squelch on when U3 */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_OUT_VEC_3, PCS_OUT_VEC_B7_DYNAMIC,
			     PCS_OUT_VEC_B7_SEL_OUT | PCS_OUT_VEC_B2_SEL_OUT),
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_NS_VEC_PS1_N1, -1,
			     (FIELD_PREP_CONST(NS_VEC_NS_REQ, 5) |
			      NS_VEC_ENABLE_TIMER |
			      FIELD_PREP_CONST(NS_VEC_SEL_TIMEOUT, 3))),
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_NS_VEC_PS2_N0, -1,
			     (FIELD_PREP_CONST(NS_VEC_NS_REQ, 1) |
			      NS_VEC_ENABLE_TIMER |
			      FIELD_PREP_CONST(NS_VEC_SEL_TIMEOUT, 3) |
			      FIELD_PREP_CONST(NS_VEC_COND_MASK, 2) |
			      FIELD_PREP_CONST(NS_VEC_EXP_COND, 2))),
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_NS_VEC_PS3_N0, -1,
			     (FIELD_PREP_CONST(NS_VEC_NS_REQ, 1) |
			      NS_VEC_ENABLE_TIMER |
			      FIELD_PREP_CONST(NS_VEC_SEL_TIMEOUT, 3) |
			      FIELD_PREP_CONST(NS_VEC_COND_MASK, 7) |
			      FIELD_PREP_CONST(NS_VEC_EXP_COND, 7))),
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_TIMEOUT_0, -1, 112),
	/* Block Aligner Type B */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_RX_CONTROL, 0,
			     RX_CONTROL_EN_BLOCK_ALIGNER_TYPE_B),
	/* Block align at TS1/TS2 for Gen2 stability (Gen2 only) */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_RX_CONTROL_DEBUG,
		RX_CONTROL_DEBUG_NUM_COM_FOUND,
		(RX_CONTROL_DEBUG_EN_TS_CHECK |
		 /*
		  * increase pcs ts1 adding packet-cnt 1 --> 4
		  * lnx_rx_valid_rstn_delay_rise_sp/ssp :
		  * 19.6us(0x200) -> 15.3us(0x4)
		  */
		 FIELD_PREP_CONST(RX_CONTROL_DEBUG_NUM_COM_FOUND, 4))),
	/* Gen1 Tx DRIVER pre-shoot, de-emphasis, level ctrl */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_HS_TX_COEF_MAP_0,
		(HS_TX_COEF_MAP_0_SSTX_DEEMP | HS_TX_COEF_MAP_0_SSTX_LEVEL |
		 HS_TX_COEF_MAP_0_SSTX_PRE_SHOOT),
		(FIELD_PREP_CONST(HS_TX_COEF_MAP_0_SSTX_DEEMP, 8) |
		 FIELD_PREP_CONST(HS_TX_COEF_MAP_0_SSTX_LEVEL, 0xb) |
		 FIELD_PREP_CONST(HS_TX_COEF_MAP_0_SSTX_PRE_SHOOT, 0))),
	/* Gen2 Tx DRIVER level ctrl */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_LOCAL_COEF,
		LOCAL_COEF_PMA_CENTER_COEF,
		FIELD_PREP_CONST(LOCAL_COEF_PMA_CENTER_COEF, 0xb)),
	/* Gen2 U1 exit LFPS duration : 900ns ~ 1.2us */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_TIMEOUT_3, -1, 4096),
	/* set skp_remove_th 0x2 -> 0x7 for avoiding retry problem. */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_EBUF_PARAM,
		EBUF_PARAM_SKP_REMOVE_TH_EMPTY_MODE,
		FIELD_PREP_CONST(EBUF_PARAM_SKP_REMOVE_TH_EMPTY_MODE, 0x7)),

	PHY_TUNING_ENTRY_LAST
};

static const struct exynos5_usbdrd_phy_tuning gs101_tunes_pipe3_postlock[] = {
	/* Squelch off when U3 */
	PHY_TUNING_ENTRY_PCS(EXYNOS9_PCS_OUT_VEC_3, PCS_OUT_VEC_B2_SEL_OUT, 0),

	PHY_TUNING_ENTRY_LAST
};

static const struct exynos5_usbdrd_phy_tuning *gs101_tunes[PTS_MAX] = {
	[PTS_UTMI_POSTINIT] = gs101_tunes_utmi_postinit,
	[PTS_PIPE3_PREINIT] = gs101_tunes_pipe3_preinit,
	[PTS_PIPE3_INIT] = gs101_tunes_pipe3_init,
	[PTS_PIPE3_POSTLOCK] = gs101_tunes_pipe3_postlock,
};

static const char * const gs101_clk_names[] = {
	"phy", "ctrl_aclk", "ctrl_pclk", "scl_pclk",
};

static const char * const gs101_regulator_names[] = {
	"pll",
	"dvdd-usb20", "vddh-usb20", "vdd33-usb20",
	"vdda-usbdp", "vddh-usbdp",
};

static const struct exynos5_usbdrd_phy_drvdata gs101_usbd31rd_phy = {
	.phy_cfg			= phy_cfg_gs101,
	.phy_tunes			= gs101_tunes,
	.phy_ops			= &gs101_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy		= GS101_PHY_CTRL_USB20,
	.pmu_offset_usbdrd0_phy_ss	= GS101_PHY_CTRL_USBDP,
	.clk_names			= gs101_clk_names,
	.n_clks				= ARRAY_SIZE(gs101_clk_names),
	.core_clk_names			= exynos5_core_clk_names,
	.n_core_clks			= ARRAY_SIZE(exynos5_core_clk_names),
	.regulator_names		= gs101_regulator_names,
	.n_regulators			= ARRAY_SIZE(gs101_regulator_names),
};

static const struct of_device_id exynos5_usbdrd_phy_of_match[] = {
	{
		.compatible = "google,gs101-usb31drd-phy",
		.data = &gs101_usbd31rd_phy
	}, {
		.compatible = "samsung,exynos2200-usb32drd-phy",
		.data = &exynos2200_usb32drd_phy,
	}, {
		.compatible = "samsung,exynos5250-usbdrd-phy",
		.data = &exynos5250_usbdrd_phy
	}, {
		.compatible = "samsung,exynos5420-usbdrd-phy",
		.data = &exynos5420_usbdrd_phy
	}, {
		.compatible = "samsung,exynos5433-usbdrd-phy",
		.data = &exynos5433_usbdrd_phy
	}, {
		.compatible = "samsung,exynos7-usbdrd-phy",
		.data = &exynos7_usbdrd_phy
	}, {
		.compatible = "samsung,exynos7870-usbdrd-phy",
		.data = &exynos7870_usbdrd_phy
	}, {
		.compatible = "samsung,exynos850-usbdrd-phy",
		.data = &exynos850_usbdrd_phy
	}, {
		.compatible = "samsung,exynos8895-usbdrd-phy",
		.data = &exynos8895_usbdrd_phy
	}, {
		.compatible = "samsung,exynos990-usbdrd-phy",
		.data = &exynos990_usbdrd_phy
	},
	{ },
};
MODULE_DEVICE_TABLE(of, exynos5_usbdrd_phy_of_match);

static int exynos5_usbdrd_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct exynos5_usbdrd_phy *phy_drd;
	struct phy_provider *phy_provider;
	const struct exynos5_usbdrd_phy_drvdata *drv_data;
	struct regmap *reg_pmu;
	u32 pmu_offset;
	int i, ret;
	int channel;

	phy_drd = devm_kzalloc(dev, sizeof(*phy_drd), GFP_KERNEL);
	if (!phy_drd)
		return -ENOMEM;

	dev_set_drvdata(dev, phy_drd);
	phy_drd->dev = dev;

	drv_data = of_device_get_match_data(dev);
	if (!drv_data)
		return -EINVAL;
	phy_drd->drv_data = drv_data;

	ret = devm_mutex_init(dev, &phy_drd->phy_mutex);
	if (ret)
		return ret;

	if (of_property_present(dev->of_node, "reg-names")) {
		void __iomem *reg;

		reg = devm_platform_ioremap_resource_byname(pdev, "phy");
		if (IS_ERR(reg))
			return PTR_ERR(reg);
		phy_drd->reg_phy = reg;

		reg = devm_platform_ioremap_resource_byname(pdev, "pcs");
		if (IS_ERR(reg))
			return PTR_ERR(reg);
		phy_drd->reg_pcs = reg;

		reg = devm_platform_ioremap_resource_byname(pdev, "pma");
		if (IS_ERR(reg))
			return PTR_ERR(reg);
		phy_drd->reg_pma = reg;
	} else {
		/* DTB with just a single region */
		phy_drd->reg_phy = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(phy_drd->reg_phy))
			return PTR_ERR(phy_drd->reg_phy);
	}

	/*
	 * USB32DRD 4nm controller implements Synopsys eUSB2.0 PHY
	 * and Synopsys SS/USBDP COMBOPHY, managed by external code.
	 */
	if (of_property_present(dev->of_node, "phy-names")) {
		phy_drd->hs_phy = devm_of_phy_get(dev, dev->of_node, "hs");
		if (IS_ERR(phy_drd->hs_phy))
			return dev_err_probe(dev, PTR_ERR(phy_drd->hs_phy),
					     "failed to get hs_phy\n");
	}

	ret = exynos5_usbdrd_phy_clk_handle(phy_drd);
	if (ret)
		return ret;

	if (drv_data->probe_extra) {
		ret = drv_data->probe_extra(phy_drd, pdev);
		if (ret)
			return ret;
	}

	reg_pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						   "samsung,pmu-syscon");
	if (IS_ERR(reg_pmu))
		return dev_err_probe(dev, PTR_ERR(reg_pmu),
				     "Failed to lookup PMU regmap\n");

	/*
	 * Exynos5420 SoC has multiple channels for USB 3.0 PHY, with
	 * each having separate power control registers.
	 * 'channel' facilitates to set such registers.
	 */
	channel = of_alias_get_id(node, "usbdrdphy");
	if (channel < 0)
		dev_dbg(dev, "Not a multi-controller usbdrd phy\n");

	/* Get regulators */
	phy_drd->regulators = devm_kcalloc(dev,
					   drv_data->n_regulators,
					   sizeof(*phy_drd->regulators),
					   GFP_KERNEL);
	if (!phy_drd->regulators)
		return -ENOMEM;
	regulator_bulk_set_supply_names(phy_drd->regulators,
					drv_data->regulator_names,
					drv_data->n_regulators);
	ret = devm_regulator_bulk_get(dev, drv_data->n_regulators,
				      phy_drd->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = exynos5_usbdrd_setup_notifiers(phy_drd);
	if (ret)
		return ret;

	dev_vdbg(dev, "Creating usbdrd_phy phy\n");

	for (i = 0; i < EXYNOS5_DRDPHYS_NUM; i++) {
		struct phy *phy = devm_phy_create(dev, NULL, drv_data->phy_ops);

		if (IS_ERR(phy))
			return dev_err_probe(dev, PTR_ERR(phy),
					     "Failed to create usbdrd_phy phy\n");

		phy_drd->phys[i].phy = phy;
		phy_drd->phys[i].index = i;
		phy_drd->phys[i].reg_pmu = reg_pmu;
		switch (channel) {
		case 1:
			pmu_offset = drv_data->pmu_offset_usbdrd1_phy;
			break;
		case 0:
		default:
			pmu_offset = drv_data->pmu_offset_usbdrd0_phy;
			if (i == EXYNOS5_DRDPHY_PIPE3 && drv_data
						->pmu_offset_usbdrd0_phy_ss)
				pmu_offset = drv_data->pmu_offset_usbdrd0_phy_ss;
			break;
		}
		phy_drd->phys[i].pmu_offset = pmu_offset;
		phy_drd->phys[i].phy_cfg = &drv_data->phy_cfg[i];
		phy_set_drvdata(phy, &phy_drd->phys[i]);
	}

	phy_provider = devm_of_phy_provider_register(dev,
						     exynos5_usbdrd_phy_xlate);
	if (IS_ERR(phy_provider))
		return dev_err_probe(phy_drd->dev, PTR_ERR(phy_provider),
				     "Failed to register phy provider\n");

	return 0;
}

static struct platform_driver exynos5_usb3drd_phy = {
	.probe	= exynos5_usbdrd_phy_probe,
	.driver = {
		.of_match_table	= exynos5_usbdrd_phy_of_match,
		.name		= "exynos5_usb3drd_phy",
		.suppress_bind_attrs = true,
	}
};

module_platform_driver(exynos5_usb3drd_phy);
MODULE_DESCRIPTION("Samsung Exynos5 SoCs USB 3.0 DRD controller PHY driver");
MODULE_AUTHOR("Vivek Gautam <gautam.vivek@samsung.com>");
MODULE_LICENSE("GPL v2");
