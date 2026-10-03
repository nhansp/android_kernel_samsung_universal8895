// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung MIPI DSIM bridge driver.
 *
 * Copyright (C) 2021 Amarula Solutions(India)
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 *
 * Based on exynos_drm_dsi from
 * Tomasz Figa <t.figa@samsung.com>
 */

#include <linux/unaligned.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/irq.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/samsung/greatlte-l5.h>
#include <linux/regmap.h>
#include <linux/units.h>

#include <video/mipi_display.h>

#include <drm/bridge/samsung-dsim.h>
#include <drm/display/drm_dsc.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

/* DEBUG: greatlte heartbeat counters (greatlte-hb-l5.c), only when built in */
#define GL5_ON	IS_BUILTIN(CONFIG_DRM_SAMSUNG_DSIM)

/* returns true iff both arguments logically differs */
#define NEQV(a, b) (!(a) ^ !(b))

/* DSIM_STATUS or DSIM_DPHY_STATUS */
#define DSIM_STOP_STATE_DAT(x)		(((x) & 0xf) << 0)
#define DSIM_STOP_STATE_CLK		BIT(8)
#define DSIM_TX_READY_HS_CLK		BIT(10)

/* DSIM_SWRST */
#define DSIM_FUNCRST			BIT(16)
#define DSIM_SWRST			BIT(0)

/* DSIM_TIMEOUT */
#define DSIM_LPDR_TIMEOUT(x)		((x) << 0)
#define DSIM_BTA_TIMEOUT(x)		((x) << 16)

/* DSIM_CLKCTRL */
#define DSIM_ESC_PRESCALER(x)			(((x) & 0xffff) << 0)
#define DSIM_ESC_PRESCALER_MASK			(0xffff << 0)
#define DSIM_LANE_ESC_CLK_EN_DATA(x, offset)	(((x) & 0xf) << offset)
#define DSIM_LANE_ESC_CLK_EN_DATA_MASK(offset)	(0xf << offset)
#define DSIM_BYTE_CLK_SRC(x)			(((x) & 0x3) << 25)
#define DSIM_BYTE_CLK_SRC_MASK			(0x3 << 25)
#define DSIM_PLL_BYPASS				BIT(27)

/* DSIM_CONFIG */
#define DSIM_LANE_EN_CLK		BIT(0)
#define DSIM_LANE_EN(x)			(((x) & 0xf) << 1)
#define DSIM_NUM_OF_DATA_LANE(x)	(((x) & 0x3) << 5)
#define DSIM_SUB_PIX_FORMAT(x)		(((x) & 0x7) << 8)
#define DSIM_MAIN_PIX_FORMAT_MASK	(0x7 << 12)
#define DSIM_MAIN_PIX_FORMAT_RGB888	(0x7 << 12)
#define DSIM_MAIN_PIX_FORMAT_RGB666	(0x6 << 12)
#define DSIM_MAIN_PIX_FORMAT_RGB666_P	(0x5 << 12)
#define DSIM_MAIN_PIX_FORMAT_RGB565	(0x4 << 12)
#define DSIM_SUB_VC			(((x) & 0x3) << 16)
#define DSIM_MAIN_VC			(((x) & 0x3) << 18)
#define DSIM_HSA_DISABLE_MODE		BIT(20)
#define DSIM_HBP_DISABLE_MODE		BIT(21)
#define DSIM_HFP_DISABLE_MODE		BIT(22)
/*
 * The i.MX 8M Mini Applications Processor Reference Manual,
 * Rev. 3, 11/2020 Page 4091
 * The i.MX 8M Nano Applications Processor Reference Manual,
 * Rev. 2, 07/2022 Page 3058
 * The i.MX 8M Plus Applications Processor Reference Manual,
 * Rev. 1, 06/2021 Page 5436
 * all claims this bit is 'HseDisableMode' with the definition
 * 0 = Disables transfer
 * 1 = Enables transfer
 *
 * This clearly states that HSE is not a disabled bit.
 *
 * The naming convention follows as per the manual and the
 * driver logic is based on the MIPI_DSI_MODE_VIDEO_HSE flag.
 */
#define DSIM_HSE_DISABLE_MODE		BIT(23)
#define DSIM_AUTO_MODE			BIT(24)
#define DSIM_BURST_MODE			BIT(26)
#define DSIM_SYNC_INFORM		BIT(27)
#define DSIM_EOT_DISABLE		BIT(28)
#define DSIM_MFLUSH_VS			BIT(29)
/* This flag is valid only for exynos3250/3472/5260/5430 */
#define DSIM_CLKLANE_STOP		BIT(30)
#define DSIM_NON_CONTINUOUS_CLKLANE	BIT(31)

/* DSIM_ESCMODE */
#define DSIM_TX_TRIGGER_RST		BIT(4)
#define DSIM_TX_LPDT_LP			BIT(6)
#define DSIM_CMD_LPDT_LP		BIT(7)
#define DSIM_FORCE_BTA			BIT(16)
#define DSIM_FORCE_STOP_STATE		BIT(20)
#define DSIM_STOP_STATE_CNT(x)		(((x) & 0x7ff) << 21)
#define DSIM_STOP_STATE_CNT_MASK	(0x7ff << 21)

/* DSIM_MDRESOL */
#define DSIM_MAIN_STAND_BY		BIT(31)
#define DSIM_MAIN_VRESOL(x, num_bits)	(((x) & ((1 << (num_bits)) - 1)) << 16)
#define DSIM_MAIN_HRESOL(x, num_bits)	(((x) & ((1 << (num_bits)) - 1)) << 0)

/* DSIM_MVPORCH */
#define DSIM_CMD_ALLOW(x)		((x) << 28)
#define DSIM_STABLE_VFP(x)		((x) << 16)
#define DSIM_MAIN_VBP(x)		((x) << 0)
#define DSIM_CMD_ALLOW_MASK		(0xf << 28)
#define DSIM_STABLE_VFP_MASK		(0x7ff << 16)
#define DSIM_MAIN_VBP_MASK		(0x7ff << 0)

/* DSIM_MHPORCH */
#define DSIM_MAIN_HFP(x)		((x) << 16)
#define DSIM_MAIN_HBP(x)		((x) << 0)
#define DSIM_MAIN_HFP_MASK		((0xffff) << 16)
#define DSIM_MAIN_HBP_MASK		((0xffff) << 0)

/* DSIM_MSYNC */
#define DSIM_MAIN_VSA(x, offset)	((x) << offset)
#define DSIM_MAIN_HSA(x)		((x) << 0)
#define DSIM_MAIN_VSA_MASK(offset)	((0x3ff) << offset)
#define DSIM_MAIN_HSA_MASK		((0xffff) << 0)

/* DSIM_SDRESOL */
#define DSIM_SUB_STANDY(x)		((x) << 31)
#define DSIM_SUB_VRESOL(x)		((x) << 16)
#define DSIM_SUB_HRESOL(x)		((x) << 0)
#define DSIM_SUB_STANDY_MASK		((0x1) << 31)
#define DSIM_SUB_VRESOL_MASK		((0x7ff) << 16)
#define DSIM_SUB_HRESOL_MASK		((0x7ff) << 0)

/* DSIM_INTSRC */
#define DSIM_INT_PLL_STABLE		BIT(31)
#define DSIM_INT_SW_RST_RELEASE		BIT(30)
#define DSIM_INT_SFR_FIFO_EMPTY		BIT(29)
#define DSIM_INT_SFR_HDR_FIFO_EMPTY	BIT(28)
#define DSIM_INT_BTA			BIT(25)
#define DSIM_INT_FRAME_DONE		BIT(24)
#define DSIM_INT_RX_TIMEOUT		BIT(21)
#define DSIM_INT_BTA_TIMEOUT		BIT(20)
#define DSIM_INT_RX_DONE		BIT(18)
#define DSIM_INT_RX_TE			BIT(17)
#define DSIM_INT_RX_ACK			BIT(16)
#define DSIM_INT_RX_ECC_ERR		BIT(15)
#define DSIM_INT_RX_CRC_ERR		BIT(14)

/* DSIM_SFRCTRL */
#define DSIM_SFR_CTRL_STAND_BY		BIT(4)
#define DSIM_SFR_CTRL_SHADOW_UPDATE	BIT(1)
#define DSIM_SFR_CTRL_SHADOW_EN		BIT(0)

/* DSIM_FIFOCTRL */
#define DSIM_RX_DATA_FULL		BIT(25)
#define DSIM_RX_DATA_EMPTY		BIT(24)
#define DSIM_SFR_HEADER_FULL		BIT(23)
#define DSIM_SFR_HEADER_EMPTY		BIT(22)
#define DSIM_SFR_PAYLOAD_FULL		BIT(21)
#define DSIM_SFR_PAYLOAD_EMPTY		BIT(20)
#define DSIM_I80_HEADER_FULL		BIT(19)
#define DSIM_I80_HEADER_EMPTY		BIT(18)
#define DSIM_I80_PAYLOAD_FULL		BIT(17)
#define DSIM_I80_PAYLOAD_EMPTY		BIT(16)
#define DSIM_SD_HEADER_FULL		BIT(15)
#define DSIM_SD_HEADER_EMPTY		BIT(14)
#define DSIM_SD_PAYLOAD_FULL		BIT(13)
#define DSIM_SD_PAYLOAD_EMPTY		BIT(12)
#define DSIM_MD_HEADER_FULL		BIT(11)
#define DSIM_MD_HEADER_EMPTY		BIT(10)
#define DSIM_MD_PAYLOAD_FULL		BIT(9)
#define DSIM_MD_PAYLOAD_EMPTY		BIT(8)
#define DSIM_RX_FIFO			BIT(4)
#define DSIM_SFR_FIFO			BIT(3)
#define DSIM_I80_FIFO			BIT(2)
#define DSIM_SD_FIFO			BIT(1)
#define DSIM_MD_FIFO			BIT(0)

/* DSIM_PHYACCHR */
#define DSIM_AFC_EN			BIT(14)
#define DSIM_AFC_CTL(x)			(((x) & 0x7) << 5)

/* DSIM_PLLCTRL */
#define DSIM_PLL_DPDNSWAP_CLK		(1 << 25)
#define DSIM_PLL_DPDNSWAP_DAT		(1 << 24)
#define DSIM_FREQ_BAND(x)		((x) << 24)
#define DSIM_PLL_EN			BIT(23)
#define DSIM_PLL(x, offset)		((x) << (offset))

/* DSIM_PHYCTRL */
#define DSIM_PHYCTRL_ULPS_EXIT(x)	(((x) & 0x1ff) << 0)
#define DSIM_PHYCTRL_B_DPHYCTL_VREG_LP	BIT(30)
#define DSIM_PHYCTRL_B_DPHYCTL_SLEW_UP	BIT(14)

/* DSIM_PHYTIMING */
#define DSIM_PHYTIMING_LPX(x)		((x) << 8)
#define DSIM_PHYTIMING_HS_EXIT(x)	((x) << 0)

/* DSIM_PHYTIMING1 */
#define DSIM_PHYTIMING1_CLK_PREPARE(x)	((x) << 24)
#define DSIM_PHYTIMING1_CLK_ZERO(x)	((x) << 16)
#define DSIM_PHYTIMING1_CLK_POST(x)	((x) << 8)
#define DSIM_PHYTIMING1_CLK_TRAIL(x)	((x) << 0)

/* DSIM_PHYTIMING2 */
#define DSIM_PHYTIMING2_HS_PREPARE(x)	((x) << 16)
#define DSIM_PHYTIMING2_HS_ZERO(x)	((x) << 8)
#define DSIM_PHYTIMING2_HS_TRAIL(x)	((x) << 0)

#define DSI_MAX_BUS_WIDTH		4
#define DSI_NUM_VIRTUAL_CHANNELS	4
#define DSI_TX_FIFO_SIZE		2048
#define DSI_RX_FIFO_SIZE		256
#define DSI_XFER_TIMEOUT_MS		100
#define DSI_RX_FIFO_EMPTY		0x30800002

#define PS_TO_CYCLE(ps, hz) DIV64_U64_ROUND_CLOSEST(((ps) * (hz)), 1000000000000ULL)

enum samsung_dsim_transfer_type {
	EXYNOS_DSI_TX,
	EXYNOS_DSI_RX,
};

static struct clk_bulk_data exynos3_clk_bulk_data[] = {
	{ .id = "bus_clk" },
	{ .id = "pll_clk" },
};

static struct clk_bulk_data exynos4_clk_bulk_data[] = {
	{ .id = "bus_clk" },
	{ .id = "sclk_mipi" },
};

static struct clk_bulk_data exynos5433_clk_bulk_data[] = {
	{ .id = "bus_clk" },
	{ .id = "sclk_mipi" },
	{ .id = "phyclk_mipidphy0_bitclkdiv8" },
	{ .id = "phyclk_mipidphy0_rxclkesc0" },
	{ .id = "sclk_rgb_vclk_to_dsim0" },
};

static struct clk_bulk_data exynos7870_clk_bulk_data[] = {
	{ .id = "bus" },
	{ .id = "pll" },
	{ .id = "byte" },
	{ .id = "esc" },
};

enum reg_idx {
	DSIM_STATUS_REG,	/* Status register (legacy) */
	DSIM_LINK_STATUS_REG,	/* Link status register */
	DSIM_DPHY_STATUS_REG,	/* D-PHY status register */
	DSIM_SWRST_REG,		/* Software reset register */
	DSIM_CLKCTRL_REG,	/* Clock control register */
	DSIM_TIMEOUT_REG,	/* Time out register */
	DSIM_CONFIG_REG,	/* Configuration register */
	DSIM_ESCMODE_REG,	/* Escape mode register */
	DSIM_MDRESOL_REG,
	DSIM_MVPORCH_REG,	/* Main display Vporch register */
	DSIM_MHPORCH_REG,	/* Main display Hporch register */
	DSIM_MSYNC_REG,		/* Main display sync area register */
	DSIM_INTSRC_REG,	/* Interrupt source register */
	DSIM_INTMSK_REG,	/* Interrupt mask register */
	DSIM_PKTHDR_REG,	/* Packet Header FIFO register */
	DSIM_PAYLOAD_REG,	/* Payload FIFO register */
	DSIM_RXFIFO_REG,	/* Read FIFO register */
	DSIM_SFRCTRL_REG,	/* SFR standby and shadow control register */
	DSIM_FIFOCTRL_REG,	/* FIFO status and control register */
	DSIM_PLLCTRL_REG,	/* PLL control register */
	DSIM_PHYCTRL_REG,
	DSIM_PHYTIMING_REG,
	DSIM_PHYTIMING1_REG,
	DSIM_PHYTIMING2_REG,
	NUM_REGS
};

static const unsigned int exynos_reg_ofs[] = {
	[DSIM_STATUS_REG] =  0x00,
	[DSIM_SWRST_REG] =  0x04,
	[DSIM_CLKCTRL_REG] =  0x08,
	[DSIM_TIMEOUT_REG] =  0x0c,
	[DSIM_CONFIG_REG] =  0x10,
	[DSIM_ESCMODE_REG] =  0x14,
	[DSIM_MDRESOL_REG] =  0x18,
	[DSIM_MVPORCH_REG] =  0x1c,
	[DSIM_MHPORCH_REG] =  0x20,
	[DSIM_MSYNC_REG] =  0x24,
	[DSIM_INTSRC_REG] =  0x2c,
	[DSIM_INTMSK_REG] =  0x30,
	[DSIM_PKTHDR_REG] =  0x34,
	[DSIM_PAYLOAD_REG] =  0x38,
	[DSIM_RXFIFO_REG] =  0x3c,
	[DSIM_FIFOCTRL_REG] =  0x44,
	[DSIM_PLLCTRL_REG] =  0x4c,
	[DSIM_PHYCTRL_REG] =  0x5c,
	[DSIM_PHYTIMING_REG] =  0x64,
	[DSIM_PHYTIMING1_REG] =  0x68,
	[DSIM_PHYTIMING2_REG] =  0x6c,
};

static const unsigned int exynos5433_reg_ofs[] = {
	[DSIM_STATUS_REG] = 0x04,
	[DSIM_SWRST_REG] = 0x0C,
	[DSIM_CLKCTRL_REG] = 0x10,
	[DSIM_TIMEOUT_REG] = 0x14,
	[DSIM_CONFIG_REG] = 0x18,
	[DSIM_ESCMODE_REG] = 0x1C,
	[DSIM_MDRESOL_REG] = 0x20,
	[DSIM_MVPORCH_REG] = 0x24,
	[DSIM_MHPORCH_REG] = 0x28,
	[DSIM_MSYNC_REG] = 0x2C,
	[DSIM_INTSRC_REG] = 0x34,
	[DSIM_INTMSK_REG] = 0x38,
	[DSIM_PKTHDR_REG] = 0x3C,
	[DSIM_PAYLOAD_REG] = 0x40,
	[DSIM_RXFIFO_REG] = 0x44,
	[DSIM_FIFOCTRL_REG] = 0x4C,
	[DSIM_PLLCTRL_REG] = 0x94,
	[DSIM_PHYCTRL_REG] = 0xA4,
	[DSIM_PHYTIMING_REG] = 0xB4,
	[DSIM_PHYTIMING1_REG] = 0xB8,
	[DSIM_PHYTIMING2_REG] = 0xBC,
};

static const unsigned int exynos7870_reg_ofs[] = {
	[DSIM_LINK_STATUS_REG] = 0x04,
	[DSIM_DPHY_STATUS_REG] = 0x08,
	[DSIM_SWRST_REG] = 0x0C,
	[DSIM_CLKCTRL_REG] = 0x10,
	[DSIM_TIMEOUT_REG] = 0x14,
	[DSIM_ESCMODE_REG] = 0x1C,
	[DSIM_MDRESOL_REG] = 0x20,
	[DSIM_MVPORCH_REG] = 0x24,
	[DSIM_MHPORCH_REG] = 0x28,
	[DSIM_MSYNC_REG] = 0x2C,
	[DSIM_CONFIG_REG] = 0x30,
	[DSIM_INTSRC_REG] = 0x34,
	[DSIM_INTMSK_REG] = 0x38,
	[DSIM_PKTHDR_REG] = 0x3C,
	[DSIM_PAYLOAD_REG] = 0x40,
	[DSIM_RXFIFO_REG] = 0x44,
	[DSIM_SFRCTRL_REG] = 0x48,
	[DSIM_FIFOCTRL_REG] = 0x4C,
	[DSIM_PLLCTRL_REG] = 0x94,
	[DSIM_PHYCTRL_REG] = 0xA4,
	[DSIM_PHYTIMING_REG] = 0xB4,
	[DSIM_PHYTIMING1_REG] = 0xB8,
	[DSIM_PHYTIMING2_REG] = 0xBC,
};

enum reg_value_idx {
	RESET_TYPE,
	PLL_TIMER,
	STOP_STATE_CNT,
	PHYCTRL_ULPS_EXIT,
	PHYCTRL_VREG_LP,
	PHYCTRL_SLEW_UP,
	PHYTIMING_LPX,
	PHYTIMING_HS_EXIT,
	PHYTIMING_CLK_PREPARE,
	PHYTIMING_CLK_ZERO,
	PHYTIMING_CLK_POST,
	PHYTIMING_CLK_TRAIL,
	PHYTIMING_HS_PREPARE,
	PHYTIMING_HS_ZERO,
	PHYTIMING_HS_TRAIL
};

static const unsigned int reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,
	[PLL_TIMER] = 500,
	[STOP_STATE_CNT] = 0xf,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0x0af),
	[PHYCTRL_VREG_LP] = 0,
	[PHYCTRL_SLEW_UP] = 0,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x06),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0b),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x07),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x27),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0d),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x08),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x09),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x0d),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0b),
};

static const unsigned int exynos5422_reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,
	[PLL_TIMER] = 500,
	[STOP_STATE_CNT] = 0xf,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0xaf),
	[PHYCTRL_VREG_LP] = 0,
	[PHYCTRL_SLEW_UP] = 0,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x08),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0d),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x09),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x30),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0e),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x0a),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x0c),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x11),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0d),
};

static const unsigned int exynos5433_reg_values[] = {
	[RESET_TYPE] = DSIM_FUNCRST,
	[PLL_TIMER] = 22200,
	[STOP_STATE_CNT] = 0xa,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0x190),
	[PHYCTRL_VREG_LP] = DSIM_PHYCTRL_B_DPHYCTL_VREG_LP,
	[PHYCTRL_SLEW_UP] = DSIM_PHYCTRL_B_DPHYCTL_SLEW_UP,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x07),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0c),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x09),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x2d),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0e),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x09),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x0b),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x10),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0c),
};

static const unsigned int exynos7870_reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,
	[PLL_TIMER] = 80000,
	[STOP_STATE_CNT] = 0xa,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0x177),
	[PHYCTRL_VREG_LP] = 0,
	[PHYCTRL_SLEW_UP] = 0,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x07),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0c),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x08),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x2b),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0d),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x09),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x09),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x0f),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0c),
};

static const unsigned int imx8mm_dsim_reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,
	[PLL_TIMER] = 500,
	[STOP_STATE_CNT] = 0xf,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0xaf),
	[PHYCTRL_VREG_LP] = 0,
	[PHYCTRL_SLEW_UP] = 0,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x06),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0b),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x07),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x26),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0d),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x08),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x08),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x0d),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0b),
};

static const struct samsung_dsim_driver_data exynos3_dsi_driver_data = {
	.reg_ofs = exynos_reg_ofs,
	.plltmr_reg = 0x50,
	.has_legacy_status_reg = 1,
	.has_freqband = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos3_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos3_clk_bulk_data),
	.max_freq = 1000,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 1,
	.num_bits_resol = 11,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
	.has_broken_fifoctrl_emptyhdr = 1,
};

static const struct samsung_dsim_driver_data exynos4_dsi_driver_data = {
	.reg_ofs = exynos_reg_ofs,
	.plltmr_reg = 0x50,
	.has_legacy_status_reg = 1,
	.has_freqband = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos4_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos4_clk_bulk_data),
	.max_freq = 1000,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 1,
	.num_bits_resol = 11,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
	.has_broken_fifoctrl_emptyhdr = 1,
};

static const struct samsung_dsim_driver_data exynos5_dsi_driver_data = {
	.reg_ofs = exynos_reg_ofs,
	.plltmr_reg = 0x58,
	.has_legacy_status_reg = 1,
	.clk_data = exynos3_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos3_clk_bulk_data),
	.max_freq = 1000,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 1,
	.num_bits_resol = 11,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
};

static const struct samsung_dsim_driver_data exynos5433_dsi_driver_data = {
	.reg_ofs = exynos5433_reg_ofs,
	.plltmr_reg = 0xa0,
	.has_legacy_status_reg = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos5433_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos5433_clk_bulk_data),
	.max_freq = 1500,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 0,
	.num_bits_resol = 12,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = exynos5433_reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
};

static const struct samsung_dsim_driver_data exynos5422_dsi_driver_data = {
	.reg_ofs = exynos5433_reg_ofs,
	.plltmr_reg = 0xa0,
	.has_legacy_status_reg = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos3_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos3_clk_bulk_data),
	.max_freq = 1500,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 1,
	.num_bits_resol = 12,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = exynos5422_reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
};

static const struct samsung_dsim_driver_data exynos7870_dsi_driver_data = {
	.reg_ofs = exynos7870_reg_ofs,
	.plltmr_reg = 0xa0,
	.has_clklane_stop = 1,
	.has_sfrctrl = 1,
	.clk_data = exynos7870_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos7870_clk_bulk_data),
	.max_freq = 1500,
	.wait_for_hdr_fifo = 0,
	.wait_for_reset = 1,
	.num_bits_resol = 12,
	.video_mode_bit = 18,
	.pll_stable_bit = 24,
	.esc_clken_bit = 16,
	.byte_clken_bit = 17,
	.tx_req_hsclk_bit = 20,
	.lane_esc_clk_bit = 8,
	.lane_esc_data_offset = 9,
	.pll_p_offset = 13,
	.pll_m_offset = 3,
	.pll_s_offset = 0,
	.main_vsa_offset = 16,
	.reg_values = exynos7870_reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
};

static const struct samsung_dsim_driver_data imx8mm_dsi_driver_data = {
	.reg_ofs = exynos5433_reg_ofs,
	.plltmr_reg = 0xa0,
	.has_legacy_status_reg = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos4_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos4_clk_bulk_data),
	.max_freq = 2100,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 0,
	.num_bits_resol = 12,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	/*
	 * Unlike Exynos, PLL_P(PMS_P) offset 14 is used in i.MX8M Mini/Nano/Plus
	 * downstream driver - drivers/gpu/drm/bridge/sec-dsim.c
	 */
	.pll_p_offset = 14,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = imx8mm_dsim_reg_values,
	.pll_fin_min = 2,
	.pll_fin_max = 30,
	.m_min = 64,
	.m_max = 1023,
	.min_freq = 1050,
};

static struct clk_bulk_data exynos8895_clk_bulk_data[] = {
	{ .id = "bus" },
	{ .id = "sysreg" },	/* SYSREG_DPU0 PCLK: LPMUX_CFG in samsung_dsim_init_exynos8895() */
};

/*
 * Exynos 8895 DSIM (S:drivers/video/fbdev/exynos/dpu/regs-dsim.h). The generic
 * indices point at the 8895 registers of the same function; the 8895-only
 * registers are the DSIM8895_* offsets below.
 */
static const unsigned int exynos8895_reg_ofs[] = {
	[DSIM_SWRST_REG] = 0x04,	/* regs-dsim.h:17 */
	[DSIM_LINK_STATUS_REG] = 0x14,	/* :32 LINK_STATUS3, PLL_STABLE bit 0 */
	[DSIM_DPHY_STATUS_REG] = 0x1c,	/* :42 */
	[DSIM_CLKCTRL_REG] = 0x20,	/* :50 */
	[DSIM_TIMEOUT_REG] = 0x28,	/* :70 */
	[DSIM_ESCMODE_REG] = 0x2c,	/* :77 */
	[DSIM_MDRESOL_REG] = 0x3c,	/* :103 RESOL */
	[DSIM_MVPORCH_REG] = 0x40,	/* :112 */
	[DSIM_MHPORCH_REG] = 0x44,	/* :123 */
	[DSIM_MSYNC_REG] = 0x48,	/* :130 */
	[DSIM_CONFIG_REG] = 0x4c,	/* :137 */
	[DSIM_INTSRC_REG] = 0x50,	/* :158 */
	[DSIM_INTMSK_REG] = 0x54,	/* :177 */
	[DSIM_PKTHDR_REG] = 0x58,	/* :196 */
	[DSIM_PAYLOAD_REG] = 0x5c,	/* :204 */
	[DSIM_RXFIFO_REG] = 0x60,	/* :207 */
	[DSIM_SFRCTRL_REG] = 0x64,	/* :210 */
	[DSIM_FIFOCTRL_REG] = 0x68,	/* :215 */
	[DSIM_PLLCTRL_REG] = 0xa0,	/* :316 */
	[DSIM_PHYCTRL_REG] = 0xb0,	/* :334 PHYCTRL_B1 */
	[DSIM_PHYTIMING_REG] = 0xd0,	/* :367 */
	[DSIM_PHYTIMING1_REG] = 0xd4,	/* :373 */
	[DSIM_PHYTIMING2_REG] = 0xd8,	/* :383 */
};

static const unsigned int exynos8895_reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,	/* regs-dsim.h:20 DSIM_SWRST_RESET (1 << 0) */
	[PLL_TIMER] = 0x13880,		/* dsim_reg.c:19 DSIM_PLL_STABLE_TIME */
	[STOP_STATE_CNT] = 0xa,		/* dsim_reg.c:15 DSIM_STOP_STATE_CNT */
};

/*
 * Fixed link parameters of an Exynos 8895 DSIM: the PMS comes from the panel
 * timing of the vendor DT (made by a vendor tool, dsim_reg.c:1435-1451), the
 * D-PHY timings from the vendor lookup table, and the analog characteristic
 * words from the vendor driver.
 */
struct samsung_dsim_fixed_link {
	unsigned int fin_mhz;		/* PLL reference, MHz */
	unsigned int p, m, s;		/* PLL PMS */
	unsigned int word_clk_div;	/* word clock = HS clock / div */
	const u32 (*dphy_timing)[10];	/* bps, clk_prepare, clk_zero, clk_post,
					 * clk_trail, hs_prepare, hs_zero, hs_trail,
					 * lpx, hs_exit; row = (row0 bps - hs) / 10 */
	unsigned int dphy_timing_rows;
	const u32 *b_dphyctl;		/* index = escape clock MHz - 7 */
	unsigned int b_dphyctl_n;
	const u32 *phy_charic;		/* MPLL_CTRL1/2, B_DPHY_CTRL2/3/4, M_DPHY_CTRL1..4 */
};

/* S:drivers/video/fbdev/exynos/dpu/dsim_reg.c:48-254 */
static const u32 exynos8895_dphy_timing[][10] = {
	{ 2500, 25, 43, 14, 20, 24, 19, 24, 18, 16 },
	{ 2490, 25, 43, 14, 20, 24, 19, 24, 18, 16 },
	{ 2480, 24, 43, 14, 20, 24, 19, 24, 18, 16 },
	{ 2470, 24, 43, 13, 20, 24, 18, 24, 18, 15 },
	{ 2460, 24, 42, 13, 20, 24, 18, 24, 18, 15 },
	{ 2450, 24, 42, 13, 20, 24, 18, 24, 18, 15 },
	{ 2440, 24, 42, 13, 20, 23, 18, 24, 18, 15 },
	{ 2430, 24, 42, 13, 20, 23, 18, 24, 18, 15 },
	{ 2420, 24, 41, 13, 20, 23, 18, 24, 18, 15 },
	{ 2410, 24, 41, 13, 20, 23, 18, 24, 18, 15 },
	{ 2400, 24, 41, 13, 20, 23, 18, 24, 17, 15 },
	{ 2390, 23, 41, 13, 19, 23, 18, 23, 17, 15 },
	{ 2380, 23, 41, 13, 19, 23, 18, 23, 17, 15 },
	{ 2370, 23, 41, 13, 19, 23, 18, 23, 17, 15 },
	{ 2360, 23, 41, 13, 19, 23, 18, 23, 17, 15 },
	{ 2350, 23, 40, 13, 19, 23, 18, 23, 17, 15 },
	{ 2340, 23, 40, 13, 19, 22, 18, 23, 17, 15 },
	{ 2330, 23, 40, 13, 19, 22, 17, 23, 17, 15 },
	{ 2320, 23, 40, 13, 19, 22, 17, 23, 17, 14 },
	{ 2310, 23, 39, 13, 19, 22, 17, 23, 17, 14 },
	{ 2300, 23, 39, 13, 19, 22, 17, 23, 17, 14 },
	{ 2290, 22, 39, 13, 19, 22, 17, 23, 17, 14 },
	{ 2280, 22, 39, 13, 18, 22, 17, 22, 17, 14 },
	{ 2270, 22, 39, 13, 18, 22, 17, 22, 17, 14 },
	{ 2260, 22, 39, 13, 18, 22, 17, 22, 16, 14 },
	{ 2250, 22, 39, 13, 18, 22, 17, 22, 16, 14 },
	{ 2240, 22, 38, 12, 18, 21, 17, 22, 16, 14 },
	{ 2230, 22, 38, 12, 18, 21, 17, 22, 16, 14 },
	{ 2220, 22, 38, 12, 18, 21, 16, 22, 16, 14 },
	{ 2210, 22, 38, 12, 18, 21, 16, 22, 16, 14 },
	{ 2200, 22, 38, 12, 18, 21, 16, 22, 16, 14 },
	{ 2190, 21, 38, 12, 18, 21, 16, 22, 16, 14 },
	{ 2180, 21, 37, 12, 18, 21, 16, 22, 16, 13 },
	{ 2170, 21, 37, 12, 18, 21, 16, 22, 16, 13 },
	{ 2160, 21, 37, 12, 17, 21, 16, 21, 16, 13 },
	{ 2150, 21, 37, 12, 17, 21, 16, 21, 16, 13 },
	{ 2140, 21, 36, 12, 17, 20, 16, 21, 16, 13 },
	{ 2130, 21, 36, 12, 17, 20, 16, 21, 15, 13 },
	{ 2120, 21, 36, 12, 17, 20, 16, 21, 15, 13 },
	{ 2110, 21, 36, 12, 17, 20, 15, 21, 15, 13 },
	{ 2100, 21, 36, 12, 17, 20, 15, 21, 15, 13 },
	{ 2090, 20, 36, 12, 17, 20, 15, 21, 15, 13 },
	{ 2080, 20, 36, 12, 17, 20, 15, 21, 15, 13 },
	{ 2070, 20, 35, 12, 17, 20, 15, 21, 15, 13 },
	{ 2060, 20, 35, 12, 17, 20, 15, 21, 15, 13 },
	{ 2050, 20, 35, 12, 16, 20, 15, 20, 15, 13 },
	{ 2040, 20, 35, 12, 16, 20, 15, 20, 15, 13 },
	{ 2030, 20, 34, 12, 16, 19, 15, 20, 15, 12 },
	{ 2020, 20, 34, 12, 16, 19, 15, 20, 15, 12 },
	{ 2010, 20, 34, 11, 16, 19, 15, 20, 15, 12 },
	{ 2000, 20, 34, 11, 16, 19, 14, 20, 14, 12 },
	{ 1990, 19, 34, 11, 16, 19, 14, 20, 14, 12 },
	{ 1980, 19, 34, 11, 16, 19, 14, 20, 14, 12 },
	{ 1970, 19, 33, 11, 16, 19, 14, 20, 14, 12 },
	{ 1960, 19, 33, 11, 16, 19, 14, 20, 14, 12 },
	{ 1950, 19, 33, 11, 16, 19, 14, 20, 14, 12 },
	{ 1940, 19, 33, 11, 15, 19, 14, 19, 14, 12 },
	{ 1930, 19, 33, 11, 15, 18, 14, 19, 14, 12 },
	{ 1920, 19, 32, 11, 15, 18, 14, 19, 14, 12 },
	{ 1910, 19, 32, 11, 15, 18, 14, 19, 14, 12 },
	{ 1900, 18, 32, 11, 15, 18, 14, 19, 14, 12 },
	{ 1890, 18, 32, 11, 15, 18, 14, 19, 14, 11 },
	{ 1880, 18, 32, 11, 15, 18, 13, 19, 14, 11 },
	{ 1870, 18, 32, 11, 15, 18, 13, 19, 14, 11 },
	{ 1860, 18, 31, 11, 15, 18, 13, 19, 13, 11 },
	{ 1850, 18, 31, 11, 15, 18, 13, 19, 13, 11 },
	{ 1840, 18, 31, 11, 15, 18, 13, 19, 13, 11 },
	{ 1830, 18, 31, 11, 14, 17, 13, 18, 13, 11 },
	{ 1820, 18, 30, 11, 14, 17, 13, 18, 13, 11 },
	{ 1810, 18, 30, 11, 14, 17, 13, 18, 13, 11 },
	{ 1800, 17, 30, 11, 14, 17, 13, 18, 13, 11 },
	{ 1790, 17, 30, 11, 14, 17, 13, 18, 13, 11 },
	{ 1780, 17, 30, 10, 14, 17, 13, 18, 13, 11 },
	{ 1770, 17, 30, 10, 14, 17, 12, 18, 13, 11 },
	{ 1760, 17, 30, 10, 14, 17, 12, 18, 13, 11 },
	{ 1750, 17, 29, 10, 14, 17, 12, 18, 13, 11 },
	{ 1740, 17, 29, 10, 14, 17, 12, 18, 13, 10 },
	{ 1730, 17, 29, 10, 14, 16, 12, 18, 12, 10 },
	{ 1720, 17, 29, 10, 14, 16, 12, 18, 12, 10 },
	{ 1710, 17, 29, 10, 13, 16, 12, 17, 12, 10 },
	{ 1700, 16, 29, 10, 13, 16, 12, 17, 12, 10 },
	{ 1690, 16, 28, 10, 13, 16, 12, 17, 12, 10 },
	{ 1680, 16, 28, 10, 13, 16, 12, 17, 12, 10 },
	{ 1670, 16, 28, 10, 13, 16, 12, 17, 12, 10 },
	{ 1660, 16, 28, 10, 13, 16, 11, 17, 12, 10 },
	{ 1650, 16, 28, 10, 13, 16, 11, 17, 12, 10 },
	{ 1640, 16, 27, 10, 13, 16, 11, 17, 12, 10 },
	{ 1630, 16, 27, 10, 13, 16, 11, 17, 12, 10 },
	{ 1620, 16, 27, 10, 13, 15, 11, 17, 12, 10 },
	{ 1610, 16, 27, 10, 13, 15, 11, 17, 12, 10 },
	{ 1600, 15, 27, 10, 12, 15, 11, 16, 11, 9 },
	{ 1590, 15, 27, 10, 12, 15, 11, 16, 11, 9 },
	{ 1580, 15, 26, 10, 12, 15, 11, 16, 11, 9 },
	{ 1570, 15, 26, 10, 12, 15, 11, 16, 11, 9 },
	{ 1560, 15, 26, 10, 12, 15, 11, 16, 11, 9 },
	{ 1550, 15, 26, 9, 12, 15, 10, 16, 11, 9 },
	{ 1540, 15, 25, 9, 12, 15, 10, 16, 11, 9 },
	{ 1530, 15, 25, 9, 12, 15, 10, 16, 11, 9 },
	{ 1520, 15, 25, 9, 12, 14, 10, 16, 11, 9 },
	{ 1510, 14, 25, 9, 12, 14, 10, 16, 11, 9 },
	{ 1500, 14, 25, 9, 12, 14, 10, 16, 11, 9 },
	{ 1490, 14, 25, 9, 11, 14, 10, 15, 11, 9 },
	{ 1480, 14, 25, 9, 11, 14, 10, 15, 11, 9 },
	{ 1470, 14, 24, 9, 11, 14, 10, 15, 11, 9 },
	{ 1460, 14, 24, 9, 11, 14, 10, 15, 10, 9 },
	{ 1450, 14, 24, 9, 11, 14, 10, 15, 10, 8 },
	{ 1440, 14, 24, 9, 11, 14, 10, 15, 10, 8 },
	{ 1430, 14, 23, 9, 11, 14, 10, 15, 10, 8 },
	{ 1420, 14, 23, 9, 11, 13, 10, 15, 10, 8 },
	{ 1410, 13, 23, 9, 11, 13, 9, 15, 10, 8 },
	{ 1400, 13, 23, 9, 11, 13, 9, 15, 10, 8 },
	{ 1390, 13, 23, 9, 11, 13, 9, 15, 10, 8 },
	{ 1380, 13, 23, 9, 10, 13, 9, 14, 10, 8 },
	{ 1370, 13, 23, 9, 10, 13, 9, 14, 10, 8 },
	{ 1360, 13, 22, 9, 10, 13, 9, 14, 10, 8 },
	{ 1350, 13, 22, 9, 10, 13, 9, 14, 10, 8 },
	{ 1340, 13, 22, 9, 10, 13, 9, 14, 10, 8 },
	{ 1330, 13, 22, 9, 10, 13, 9, 14, 9, 8 },
	{ 1320, 13, 22, 8, 10, 12, 9, 14, 9, 8 },
	{ 1310, 12, 22, 8, 10, 12, 9, 14, 9, 8 },
	{ 1300, 12, 21, 8, 10, 12, 8, 14, 9, 7 },
	{ 1290, 12, 21, 8, 10, 12, 8, 14, 9, 7 },
	{ 1280, 12, 21, 8, 10, 12, 8, 14, 9, 7 },
	{ 1270, 12, 21, 8, 10, 12, 8, 14, 9, 7 },
	{ 1260, 12, 20, 8, 9, 12, 8, 13, 9, 7 },
	{ 1250, 12, 20, 8, 9, 12, 8, 13, 9, 7 },
	{ 1240, 12, 20, 8, 9, 12, 8, 13, 9, 7 },
	{ 1230, 12, 20, 8, 9, 12, 8, 13, 9, 7 },
	{ 1220, 12, 20, 8, 9, 12, 8, 13, 9, 7 },
	{ 1210, 11, 20, 8, 9, 11, 8, 13, 9, 7 },
	{ 1200, 11, 20, 8, 9, 11, 8, 13, 8, 7 },
	{ 1190, 11, 19, 8, 9, 11, 7, 13, 8, 7 },
	{ 1180, 11, 19, 8, 9, 11, 7, 13, 8, 7 },
	{ 1170, 11, 19, 8, 9, 11, 7, 13, 8, 7 },
	{ 1160, 11, 19, 8, 9, 11, 7, 13, 8, 6 },
	{ 1150, 11, 18, 8, 8, 11, 7, 12, 8, 6 },
	{ 1140, 11, 18, 8, 8, 11, 7, 12, 8, 6 },
	{ 1130, 11, 18, 8, 8, 11, 7, 12, 8, 6 },
	{ 1120, 10, 18, 8, 8, 11, 7, 12, 8, 6 },
	{ 1110, 10, 18, 8, 8, 10, 7, 12, 8, 6 },
	{ 1100, 10, 18, 8, 8, 10, 7, 12, 8, 6 },
	{ 1090, 10, 17, 7, 8, 10, 7, 12, 8, 6 },
	{ 1080, 10, 17, 7, 8, 10, 6, 12, 8, 6 },
	{ 1070, 10, 17, 7, 8, 10, 6, 12, 8, 6 },
	{ 1060, 10, 17, 7, 8, 10, 6, 12, 7, 6 },
	{ 1050, 10, 17, 7, 8, 10, 6, 12, 7, 6 },
	{ 1040, 10, 16, 7, 7, 10, 6, 11, 7, 6 },
	{ 1030, 10, 16, 7, 7, 10, 6, 11, 7, 6 },
	{ 1020, 9, 16, 7, 7, 10, 6, 11, 7, 6 },
	{ 1010, 9, 16, 7, 7, 9, 6, 11, 7, 5 },
	{ 1000, 9, 16, 7, 7, 9, 6, 11, 7, 5 },
	{ 990, 9, 16, 7, 7, 9, 6, 11, 7, 5 },
	{ 980, 9, 15, 7, 7, 9, 6, 11, 7, 5 },
	{ 970, 9, 15, 7, 7, 9, 5, 11, 7, 5 },
	{ 960, 9, 15, 7, 7, 9, 5, 11, 7, 5 },
	{ 950, 9, 15, 7, 7, 9, 5, 11, 7, 5 },
	{ 940, 9, 14, 7, 7, 9, 5, 11, 7, 5 },
	{ 930, 9, 14, 7, 7, 9, 5, 11, 6, 5 },
	{ 920, 8, 14, 7, 6, 9, 5, 10, 6, 5 },
	{ 910, 8, 14, 7, 6, 8, 5, 10, 6, 5 },
	{ 900, 8, 14, 7, 6, 8, 5, 10, 6, 5 },
	{ 890, 8, 14, 7, 6, 8, 5, 10, 6, 5 },
	{ 880, 8, 14, 7, 6, 8, 5, 10, 6, 5 },
	{ 870, 8, 13, 6, 6, 8, 5, 10, 6, 4 },
	{ 860, 8, 13, 6, 6, 8, 4, 10, 6, 4 },
	{ 850, 8, 13, 6, 6, 8, 4, 10, 6, 4 },
	{ 840, 8, 13, 6, 6, 8, 4, 10, 6, 4 },
	{ 830, 8, 13, 6, 6, 8, 4, 10, 6, 4 },
	{ 820, 7, 13, 6, 6, 8, 4, 10, 6, 4 },
	{ 810, 7, 12, 6, 5, 8, 4, 9, 6, 4 },
	{ 800, 7, 12, 6, 5, 7, 4, 9, 5, 4 },
	{ 790, 7, 12, 6, 5, 7, 4, 9, 5, 4 },
	{ 780, 7, 12, 6, 5, 7, 4, 9, 5, 4 },
	{ 770, 7, 12, 6, 5, 7, 4, 9, 5, 4 },
	{ 760, 7, 11, 6, 5, 7, 4, 9, 5, 4 },
	{ 750, 7, 11, 6, 5, 7, 3, 9, 5, 4 },
	{ 740, 7, 11, 6, 5, 7, 3, 9, 5, 4 },
	{ 730, 6, 11, 6, 5, 7, 3, 9, 5, 4 },
	{ 720, 6, 11, 6, 5, 7, 3, 9, 5, 3 },
	{ 710, 6, 11, 6, 5, 7, 3, 9, 5, 3 },
	{ 700, 6, 10, 6, 4, 6, 3, 8, 5, 3 },
	{ 690, 6, 10, 6, 4, 6, 3, 8, 5, 3 },
	{ 680, 6, 10, 6, 4, 6, 3, 8, 5, 3 },
	{ 670, 6, 10, 6, 4, 6, 3, 8, 5, 3 },
	{ 660, 6, 9, 6, 4, 6, 3, 8, 4, 3 },
	{ 650, 6, 9, 6, 4, 6, 3, 8, 4, 3 },
	{ 640, 6, 9, 5, 4, 6, 2, 8, 4, 3 },
	{ 630, 5, 9, 5, 4, 6, 2, 8, 4, 3 },
	{ 620, 5, 9, 5, 4, 6, 2, 8, 4, 3 },
	{ 610, 5, 9, 5, 4, 6, 2, 8, 4, 3 },
	{ 600, 5, 9, 5, 4, 5, 2, 8, 4, 3 },
	{ 590, 5, 8, 5, 3, 5, 2, 7, 4, 3 },
	{ 580, 5, 8, 5, 3, 5, 2, 7, 4, 2 },
	{ 570, 5, 8, 5, 3, 5, 2, 7, 4, 2 },
	{ 560, 5, 8, 5, 3, 5, 2, 7, 4, 2 },
	{ 550, 5, 7, 5, 3, 5, 2, 7, 4, 2 },
	{ 540, 5, 7, 5, 3, 5, 2, 7, 4, 2 },
	{ 530, 4, 7, 5, 3, 5, 1, 7, 3, 2 },
	{ 520, 4, 7, 5, 3, 5, 1, 7, 3, 2 },
	{ 510, 4, 7, 5, 3, 5, 1, 7, 3, 2 },
	{ 500, 4, 7, 5, 3, 5, 1, 7, 3, 2 },
	{ 490, 4, 6, 5, 3, 4, 1, 7, 3, 2 },
	{ 480, 4, 6, 5, 3, 4, 1, 7, 3, 2 },
};

/* S:drivers/video/fbdev/exynos/dpu/dsim_reg.c:256-260, escape clock 7..20 MHz */
static const u32 exynos8895_b_dphyctl[] = {
	0x0af, 0x0c8, 0x0e1, 0x0fa,
	0x113, 0x12c, 0x145, 0x15e, 0x177,
	0x190, 0x1a9, 0x1c2, 0x1db, 0x1f4,
};

/* S:drivers/video/fbdev/exynos/dpu/dsim_reg.c:39 (DSIM0), order dsim.h:153-163 */
static const u32 exynos8895_phy_charic[9] = {
	0x0, 0x3800000, 0x800, 0x10001249, 0x500000, 0x1980, 0x0, 0x0, 0x0,
};

static const struct samsung_dsim_fixed_link exynos8895_fixed_link = {
	.fin_mhz = 26,			/* dsim_reg.c:1459 "u32 fin = 26;" */
	.p = 5, .m = 691, .s = 2,	/* FDT:10088 timing,pms = <0x05 0x2b3 0x02> */
	.word_clk_div = 16,		/* dsim_reg.c:1486-1491 hs_clk / 16 */
	.dphy_timing = exynos8895_dphy_timing,
	.dphy_timing_rows = ARRAY_SIZE(exynos8895_dphy_timing),
	.b_dphyctl = exynos8895_b_dphyctl,
	.b_dphyctl_n = ARRAY_SIZE(exynos8895_b_dphyctl),
	.phy_charic = exynos8895_phy_charic,
};

static const struct samsung_dsim_driver_data exynos8895_dsi_driver_data = {
	.reg_ofs = exynos8895_reg_ofs,
	.plltmr_reg = 0xac,		/* regs-dsim.h:331 DSIM_PLLTMR */
	.has_8895_link = 1,
	.te_on_long_packet = 1,
	.handover_hold = 1,
	.fixed_link = &exynos8895_fixed_link,
	.clk_data = exynos8895_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos8895_clk_bulk_data),
	.wait_for_hdr_fifo = 0,
	.wait_for_reset = 0,
	.num_bits_resol = 13,		/* regs-dsim.h:104-106 0x1fff */
	.video_mode_bit = 18,		/* :146 */
	.pll_stable_bit = 0,		/* :33 */
	.esc_clken_bit = 16,		/* :55 */
	.byte_clken_bit = 17,		/* :54 WORDCLK_EN */
	.tx_req_hsclk_bit = 20,		/* :53 */
	.lane_esc_clk_bit = 12,		/* :56-57 LANE_ESCCLK_EN [12:8], clock lane bit 12 */
	.lane_esc_data_offset = 8,
	.pll_p_offset = 13,		/* dsim_reg.c:350-355 */
	.pll_m_offset = 3,
	.pll_s_offset = 0,
	.main_vsa_offset = 16,		/* regs-dsim.h:131 */
	.reg_values = exynos8895_reg_values,
};

static const struct samsung_dsim_driver_data *
samsung_dsim_types[DSIM_TYPE_COUNT] = {
	[DSIM_TYPE_EXYNOS3250] = &exynos3_dsi_driver_data,
	[DSIM_TYPE_EXYNOS4210] = &exynos4_dsi_driver_data,
	[DSIM_TYPE_EXYNOS5410] = &exynos5_dsi_driver_data,
	[DSIM_TYPE_EXYNOS5422] = &exynos5422_dsi_driver_data,
	[DSIM_TYPE_EXYNOS5433] = &exynos5433_dsi_driver_data,
	[DSIM_TYPE_EXYNOS7870] = &exynos7870_dsi_driver_data,
	[DSIM_TYPE_EXYNOS8895] = &exynos8895_dsi_driver_data,
	[DSIM_TYPE_IMX8MM] = &imx8mm_dsi_driver_data,
	[DSIM_TYPE_IMX8MP] = &imx8mm_dsi_driver_data,
};

static inline struct samsung_dsim *host_to_dsi(struct mipi_dsi_host *h)
{
	return container_of(h, struct samsung_dsim, dsi_host);
}

static inline struct samsung_dsim *bridge_to_dsi(struct drm_bridge *b)
{
	return container_of(b, struct samsung_dsim, bridge);
}

static inline void samsung_dsim_write(struct samsung_dsim *dsi,
				      enum reg_idx idx, u32 val)
{
	writel(val, dsi->reg_base + dsi->driver_data->reg_ofs[idx]);
}

static inline u32 samsung_dsim_read(struct samsung_dsim *dsi, enum reg_idx idx)
{
	return readl(dsi->reg_base + dsi->driver_data->reg_ofs[idx]);
}

static void samsung_dsim_wait_for_reset(struct samsung_dsim *dsi)
{
	if (wait_for_completion_timeout(&dsi->completed, msecs_to_jiffies(300)))
		return;

	dev_err(dsi->dev, "timeout waiting for reset\n");
}

static void samsung_dsim_reset(struct samsung_dsim *dsi)
{
	u32 reset_val = dsi->driver_data->reg_values[RESET_TYPE];

	reinit_completion(&dsi->completed);
	samsung_dsim_write(dsi, DSIM_SWRST_REG, reset_val);
}

static unsigned long samsung_dsim_pll_find_pms(struct samsung_dsim *dsi,
					       unsigned long fin,
					       unsigned long fout,
					       u8 *p, u16 *m, u8 *s)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	unsigned long best_freq = 0;
	u32 min_delta = 0xffffffff;
	u8 p_min, p_max;
	u8 _p, best_p;
	u16 _m, best_m;
	u8 _s, best_s;

	p_min = DIV_ROUND_UP(fin, (driver_data->pll_fin_max * HZ_PER_MHZ));
	p_max = fin / (driver_data->pll_fin_min * HZ_PER_MHZ);

	for (_p = p_min; _p <= p_max; ++_p) {
		for (_s = 0; _s <= 5; ++_s) {
			u64 tmp;
			u32 delta;

			tmp = (u64)fout * (_p << _s);
			do_div(tmp, fin);
			_m = tmp;
			if (_m < driver_data->m_min || _m > driver_data->m_max)
				continue;

			tmp = (u64)_m * fin;
			do_div(tmp, _p);
			if (tmp < driver_data->min_freq  * HZ_PER_MHZ ||
			    tmp > driver_data->max_freq * HZ_PER_MHZ)
				continue;

			tmp = (u64)_m * fin;
			do_div(tmp, _p << _s);

			delta = abs(fout - tmp);
			if (delta < min_delta) {
				best_p = _p;
				best_m = _m;
				best_s = _s;
				min_delta = delta;
				best_freq = tmp;
			}
		}
	}

	if (best_freq) {
		*p = best_p;
		*m = best_m;
		*s = best_s;
	}

	return best_freq;
}

static unsigned long samsung_dsim_set_pll(struct samsung_dsim *dsi,
					  unsigned long freq)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	unsigned long fin, fout;
	int timeout;
	u8 p, s;
	u16 m;
	u32 reg;

	if (dsi->pll_clk) {
		/*
		 * Ensure that the reference clock is generated with a power of
		 * two divider from its parent, but close to the PLLs upper
		 * limit.
		 */
		fin = clk_get_rate(clk_get_parent(dsi->pll_clk));
		while (fin > driver_data->pll_fin_max * HZ_PER_MHZ)
			fin /= 2;
		clk_set_rate(dsi->pll_clk, fin);

		fin = clk_get_rate(dsi->pll_clk);
	} else {
		fin = dsi->pll_clk_rate;
	}
	dev_dbg(dsi->dev, "PLL ref clock freq %lu\n", fin);

	fout = samsung_dsim_pll_find_pms(dsi, fin, freq, &p, &m, &s);
	if (!fout) {
		dev_err(dsi->dev,
			"failed to find PLL PMS for requested frequency\n");
		return 0;
	}
	dev_dbg(dsi->dev, "PLL freq %lu, (p %d, m %d, s %d)\n", fout, p, m, s);

	writel(driver_data->reg_values[PLL_TIMER],
	       dsi->reg_base + driver_data->plltmr_reg);

	reg = DSIM_PLL_EN | DSIM_PLL(p, driver_data->pll_p_offset)
			  | DSIM_PLL(m, driver_data->pll_m_offset)
			  | DSIM_PLL(s, driver_data->pll_s_offset);

	if (driver_data->has_freqband) {
		static const unsigned long freq_bands[] = {
			100 * HZ_PER_MHZ, 120 * HZ_PER_MHZ, 160 * HZ_PER_MHZ,
			200 * HZ_PER_MHZ, 270 * HZ_PER_MHZ, 320 * HZ_PER_MHZ,
			390 * HZ_PER_MHZ, 450 * HZ_PER_MHZ, 510 * HZ_PER_MHZ,
			560 * HZ_PER_MHZ, 640 * HZ_PER_MHZ, 690 * HZ_PER_MHZ,
			770 * HZ_PER_MHZ, 870 * HZ_PER_MHZ, 950 * HZ_PER_MHZ,
		};
		int band;

		for (band = 0; band < ARRAY_SIZE(freq_bands); ++band)
			if (fout < freq_bands[band])
				break;

		dev_dbg(dsi->dev, "band %d\n", band);

		reg |= DSIM_FREQ_BAND(band);
	}

	if (dsi->swap_dn_dp_clk)
		reg |= DSIM_PLL_DPDNSWAP_CLK;
	if (dsi->swap_dn_dp_data)
		reg |= DSIM_PLL_DPDNSWAP_DAT;

	samsung_dsim_write(dsi, DSIM_PLLCTRL_REG, reg);

	timeout = 3000;
	do {
		if (timeout-- == 0) {
			dev_err(dsi->dev, "PLL failed to stabilize\n");
			return 0;
		}
		if (driver_data->has_legacy_status_reg)
			reg = samsung_dsim_read(dsi, DSIM_STATUS_REG);
		else
			reg = samsung_dsim_read(dsi, DSIM_LINK_STATUS_REG);
	} while ((reg & BIT(driver_data->pll_stable_bit)) == 0);

	dsi->hs_clock = fout;

	return fout;
}

static int samsung_dsim_enable_clock(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	unsigned long hs_clk, byte_clk, esc_clk, pix_clk;
	unsigned long esc_div;
	u32 reg;
	struct drm_display_mode *m = &dsi->mode;
	int bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	/* m->clock is in KHz */
	pix_clk = m->clock * 1000;

	/* Use burst_clk_rate if available, otherwise use the pix_clk */
	if (dsi->burst_clk_rate)
		hs_clk = samsung_dsim_set_pll(dsi, dsi->burst_clk_rate);
	else
		hs_clk = samsung_dsim_set_pll(dsi, DIV_ROUND_UP(pix_clk * bpp, dsi->lanes));

	if (!hs_clk) {
		dev_err(dsi->dev, "failed to configure DSI PLL\n");
		return -EFAULT;
	}

	byte_clk = hs_clk / 8;
	esc_div = DIV_ROUND_UP(byte_clk, dsi->esc_clk_rate);
	esc_clk = byte_clk / esc_div;

	if (esc_clk > 20 * HZ_PER_MHZ) {
		++esc_div;
		esc_clk = byte_clk / esc_div;
	}

	dev_dbg(dsi->dev, "hs_clk = %lu, byte_clk = %lu, esc_clk = %lu\n",
		hs_clk, byte_clk, esc_clk);

	reg = samsung_dsim_read(dsi, DSIM_CLKCTRL_REG);
	reg &= ~(DSIM_ESC_PRESCALER_MASK | BIT(driver_data->lane_esc_clk_bit)
		| DSIM_LANE_ESC_CLK_EN_DATA_MASK(driver_data->lane_esc_data_offset)
		| DSIM_PLL_BYPASS
		| DSIM_BYTE_CLK_SRC_MASK);
	reg |= BIT(driver_data->esc_clken_bit) | BIT(driver_data->byte_clken_bit)
		| DSIM_ESC_PRESCALER(esc_div)
		| BIT(driver_data->lane_esc_clk_bit)
		| DSIM_LANE_ESC_CLK_EN_DATA(BIT(dsi->lanes) - 1,
					    driver_data->lane_esc_data_offset)
		| DSIM_BYTE_CLK_SRC(0)
		| BIT(driver_data->tx_req_hsclk_bit);
	samsung_dsim_write(dsi, DSIM_CLKCTRL_REG, reg);

	return 0;
}

static void samsung_dsim_set_phy_ctrl(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	const unsigned int *reg_values = driver_data->reg_values;
	u32 reg;
	struct phy_configure_opts_mipi_dphy cfg;
	int clk_prepare, lpx, clk_zero, clk_post, clk_trail;
	int hs_exit, hs_prepare, hs_zero, hs_trail;
	unsigned long long byte_clock = dsi->hs_clock / 8;

	if (driver_data->has_freqband)
		return;

	phy_mipi_dphy_get_default_config_for_hsclk(dsi->hs_clock,
						   dsi->lanes, &cfg);

	/*
	 * TODO:
	 * The tech Applications Processor manuals for i.MX8M Mini, Nano,
	 * and Plus don't state what the definition of the PHYTIMING
	 * bits are beyond their address and bit position.
	 * After reviewing NXP's downstream code, it appears
	 * that the various PHYTIMING registers take the number
	 * of cycles and use various dividers on them.  This
	 * calculation does not result in an exact match to the
	 * downstream code, but it is very close to the values
	 * generated by their lookup table, and it appears
	 * to sync at a variety of resolutions. If someone
	 * can get a more accurate mathematical equation needed
	 * for these registers, this should be updated.
	 */

	lpx = PS_TO_CYCLE(cfg.lpx, byte_clock);
	hs_exit = PS_TO_CYCLE(cfg.hs_exit, byte_clock);
	clk_prepare = PS_TO_CYCLE(cfg.clk_prepare, byte_clock);
	clk_zero = PS_TO_CYCLE(cfg.clk_zero, byte_clock);
	clk_post = PS_TO_CYCLE(cfg.clk_post, byte_clock);
	clk_trail = PS_TO_CYCLE(cfg.clk_trail, byte_clock);
	hs_prepare = PS_TO_CYCLE(cfg.hs_prepare, byte_clock);
	hs_zero = PS_TO_CYCLE(cfg.hs_zero, byte_clock);
	hs_trail = PS_TO_CYCLE(cfg.hs_trail, byte_clock);

	/* B D-PHY: D-PHY Master & Slave Analog Block control */
	reg = reg_values[PHYCTRL_ULPS_EXIT] | reg_values[PHYCTRL_VREG_LP] |
		reg_values[PHYCTRL_SLEW_UP];

	samsung_dsim_write(dsi, DSIM_PHYCTRL_REG, reg);

	/*
	 * T LPX: Transmitted length of any Low-Power state period
	 * T HS-EXIT: Time that the transmitter drives LP-11 following a HS
	 *	burst
	 */

	reg  = DSIM_PHYTIMING_LPX(lpx) | DSIM_PHYTIMING_HS_EXIT(hs_exit);

	samsung_dsim_write(dsi, DSIM_PHYTIMING_REG, reg);

	/*
	 * T CLK-PREPARE: Time that the transmitter drives the Clock Lane LP-00
	 *	Line state immediately before the HS-0 Line state starting the
	 *	HS transmission
	 * T CLK-ZERO: Time that the transmitter drives the HS-0 state prior to
	 *	transmitting the Clock.
	 * T CLK_POST: Time that the transmitter continues to send HS clock
	 *	after the last associated Data Lane has transitioned to LP Mode
	 *	Interval is defined as the period from the end of T HS-TRAIL to
	 *	the beginning of T CLK-TRAIL
	 * T CLK-TRAIL: Time that the transmitter drives the HS-0 state after
	 *	the last payload clock bit of a HS transmission burst
	 */

	reg = DSIM_PHYTIMING1_CLK_PREPARE(clk_prepare)	|
	      DSIM_PHYTIMING1_CLK_ZERO(clk_zero)	|
	      DSIM_PHYTIMING1_CLK_POST(clk_post)	|
	      DSIM_PHYTIMING1_CLK_TRAIL(clk_trail);

	samsung_dsim_write(dsi, DSIM_PHYTIMING1_REG, reg);

	/*
	 * T HS-PREPARE: Time that the transmitter drives the Data Lane LP-00
	 *	Line state immediately before the HS-0 Line state starting the
	 *	HS transmission
	 * T HS-ZERO: Time that the transmitter drives the HS-0 state prior to
	 *	transmitting the Sync sequence.
	 * T HS-TRAIL: Time that the transmitter drives the flipped differential
	 *	state after last payload data bit of a HS transmission burst
	 */

	reg = DSIM_PHYTIMING2_HS_PREPARE(hs_prepare) |
	      DSIM_PHYTIMING2_HS_ZERO(hs_zero) |
	      DSIM_PHYTIMING2_HS_TRAIL(hs_trail);

	samsung_dsim_write(dsi, DSIM_PHYTIMING2_REG, reg);
}

static void samsung_dsim_disable_clock(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	u32 reg;

	reg = samsung_dsim_read(dsi, DSIM_CLKCTRL_REG);
	reg &= ~(BIT(driver_data->lane_esc_clk_bit)
		| DSIM_LANE_ESC_CLK_EN_DATA_MASK(driver_data->lane_esc_data_offset)
		| BIT(driver_data->esc_clken_bit)
		| BIT(driver_data->byte_clken_bit));
	samsung_dsim_write(dsi, DSIM_CLKCTRL_REG, reg);

	reg = samsung_dsim_read(dsi, DSIM_PLLCTRL_REG);
	reg &= ~DSIM_PLL_EN;
	samsung_dsim_write(dsi, DSIM_PLLCTRL_REG, reg);
}

static void samsung_dsim_enable_lane(struct samsung_dsim *dsi, u32 lane)
{
	u32 reg = samsung_dsim_read(dsi, DSIM_CONFIG_REG);

	reg |= (DSIM_NUM_OF_DATA_LANE(dsi->lanes - 1) | DSIM_LANE_EN_CLK |
			DSIM_LANE_EN(lane));
	samsung_dsim_write(dsi, DSIM_CONFIG_REG, reg);
}

static int samsung_dsim_init_link(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	int timeout;
	u32 reg;
	u32 lanes_mask;

	/* Initialize FIFO pointers */
	reg = samsung_dsim_read(dsi, DSIM_FIFOCTRL_REG);
	reg &= ~0x1f;
	samsung_dsim_write(dsi, DSIM_FIFOCTRL_REG, reg);

	usleep_range(9000, 11000);

	reg |= 0x1f;
	samsung_dsim_write(dsi, DSIM_FIFOCTRL_REG, reg);
	usleep_range(9000, 11000);

	/* DSI configuration */
	reg = 0;

	/*
	 * The first bit of mode_flags specifies display configuration.
	 * If this bit is set[= MIPI_DSI_MODE_VIDEO], dsi will support video
	 * mode, otherwise it will support command mode.
	 */
	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) {
		reg |= BIT(driver_data->video_mode_bit);

		/*
		 * The user manual describes that following bits are ignored in
		 * command mode.
		 */
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			reg |= DSIM_SYNC_INFORM;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			reg |= DSIM_BURST_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_AUTO_VERT)
			reg |= DSIM_AUTO_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_HSE)
			reg |= DSIM_HSE_DISABLE_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_NO_HFP)
			reg |= DSIM_HFP_DISABLE_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_NO_HBP)
			reg |= DSIM_HBP_DISABLE_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_NO_HSA)
			reg |= DSIM_HSA_DISABLE_MODE;
	}

	if (dsi->mode_flags & MIPI_DSI_MODE_NO_EOT_PACKET)
		reg |= DSIM_EOT_DISABLE;

	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		reg |= DSIM_MAIN_PIX_FORMAT_RGB888;
		break;
	case MIPI_DSI_FMT_RGB666:
		reg |= DSIM_MAIN_PIX_FORMAT_RGB666;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		reg |= DSIM_MAIN_PIX_FORMAT_RGB666_P;
		break;
	case MIPI_DSI_FMT_RGB565:
		reg |= DSIM_MAIN_PIX_FORMAT_RGB565;
		break;
	default:
		dev_err(dsi->dev, "invalid pixel format\n");
		return -EINVAL;
	}

	/*
	 * Use non-continuous clock mode if the periparal wants and
	 * host controller supports
	 *
	 * In non-continous clock mode, host controller will turn off
	 * the HS clock between high-speed transmissions to reduce
	 * power consumption.
	 */
	if (driver_data->has_clklane_stop &&
	    dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		if (!samsung_dsim_hw_is_exynos(dsi->plat_data->hw_type))
			reg |= DSIM_NON_CONTINUOUS_CLKLANE;

		reg |= DSIM_CLKLANE_STOP;
	}
	samsung_dsim_write(dsi, DSIM_CONFIG_REG, reg);

	lanes_mask = BIT(dsi->lanes) - 1;
	samsung_dsim_enable_lane(dsi, lanes_mask);

	/* Check clock and data lane state are stop state */
	timeout = 100;
	do {
		if (timeout-- == 0) {
			dev_err(dsi->dev, "waiting for bus lanes timed out\n");
			return -EFAULT;
		}

		if (driver_data->has_legacy_status_reg)
			reg = samsung_dsim_read(dsi, DSIM_STATUS_REG);
		else
			reg = samsung_dsim_read(dsi, DSIM_DPHY_STATUS_REG);
		if ((reg & DSIM_STOP_STATE_DAT(lanes_mask))
		    != DSIM_STOP_STATE_DAT(lanes_mask))
			continue;
	} while (!(reg & (DSIM_STOP_STATE_CLK | DSIM_TX_READY_HS_CLK)));

	reg = samsung_dsim_read(dsi, DSIM_ESCMODE_REG);
	reg &= ~DSIM_STOP_STATE_CNT_MASK;
	reg |= DSIM_STOP_STATE_CNT(driver_data->reg_values[STOP_STATE_CNT]);
	samsung_dsim_write(dsi, DSIM_ESCMODE_REG, reg);

	reg = DSIM_BTA_TIMEOUT(0xff) | DSIM_LPDR_TIMEOUT(0xffff);
	samsung_dsim_write(dsi, DSIM_TIMEOUT_REG, reg);

	return 0;
}

static void samsung_dsim_set_display_mode(struct samsung_dsim *dsi)
{
	struct drm_display_mode *m = &dsi->mode;
	unsigned int num_bits_resol = dsi->driver_data->num_bits_resol;
	unsigned int main_vsa_offset = dsi->driver_data->main_vsa_offset;
	u32 reg;

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) {
		u64 byte_clk = dsi->hs_clock / 8;
		u64 pix_clk = m->clock * 1000;

		int hfp = DIV64_U64_ROUND_UP((m->hsync_start - m->hdisplay) * byte_clk, pix_clk);
		int hbp = DIV64_U64_ROUND_UP((m->htotal - m->hsync_end) * byte_clk, pix_clk);
		int hsa = DIV64_U64_ROUND_UP((m->hsync_end - m->hsync_start) * byte_clk, pix_clk);

		/* remove packet overhead when possible */
		hfp = max(hfp - 6, 0);
		hbp = max(hbp - 6, 0);
		hsa = max(hsa - 6, 0);

		dev_dbg(dsi->dev, "calculated hfp: %u, hbp: %u, hsa: %u",
			hfp, hbp, hsa);

		reg = DSIM_CMD_ALLOW(0xf)
			| DSIM_STABLE_VFP(m->vsync_start - m->vdisplay)
			| DSIM_MAIN_VBP(m->vtotal - m->vsync_end);
		samsung_dsim_write(dsi, DSIM_MVPORCH_REG, reg);

		reg = DSIM_MAIN_HFP(hfp) | DSIM_MAIN_HBP(hbp);
		samsung_dsim_write(dsi, DSIM_MHPORCH_REG, reg);

		reg = DSIM_MAIN_VSA(m->vsync_end - m->vsync_start, main_vsa_offset)
			| DSIM_MAIN_HSA(hsa);
		samsung_dsim_write(dsi, DSIM_MSYNC_REG, reg);
	}
	reg =  DSIM_MAIN_HRESOL(m->hdisplay, num_bits_resol) |
		DSIM_MAIN_VRESOL(m->vdisplay, num_bits_resol);

	samsung_dsim_write(dsi, DSIM_MDRESOL_REG, reg);

	dev_dbg(dsi->dev, "LCD size = %dx%d\n", m->hdisplay, m->vdisplay);
}

static void samsung_dsim_set_display_enable(struct samsung_dsim *dsi, bool enable)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	u32 reg;

	reg = samsung_dsim_read(dsi, DSIM_MDRESOL_REG);
	if (enable)
		reg |= DSIM_MAIN_STAND_BY;
	else
		reg &= ~DSIM_MAIN_STAND_BY;
	samsung_dsim_write(dsi, DSIM_MDRESOL_REG, reg);

	if (driver_data->has_sfrctrl) {
		reg = samsung_dsim_read(dsi, DSIM_SFRCTRL_REG);
		if (enable)
			reg |= DSIM_SFR_CTRL_STAND_BY;
		else
			reg &= ~DSIM_SFR_CTRL_STAND_BY;
		samsung_dsim_write(dsi, DSIM_SFRCTRL_REG, reg);
	}
}

static int samsung_dsim_wait_for_hdr_fifo(struct samsung_dsim *dsi)
{
	int timeout = 2000;

	do {
		u32 reg = samsung_dsim_read(dsi, DSIM_FIFOCTRL_REG);

		if (!dsi->driver_data->has_broken_fifoctrl_emptyhdr) {
			if (reg & DSIM_SFR_HEADER_EMPTY)
				return 0;
		} else {
			if (!(reg & DSIM_SFR_HEADER_FULL)) {
				/*
				 * Wait a little bit, so the pending data can
				 * actually leave the FIFO to avoid overflow.
				 */
				if (!cond_resched())
					usleep_range(950, 1050);
				return 0;
			}
		}

		if (!cond_resched())
			usleep_range(950, 1050);
	} while (--timeout);

	return -ETIMEDOUT;
}

static void samsung_dsim_set_cmd_lpm(struct samsung_dsim *dsi, bool lpm)
{
	u32 v = samsung_dsim_read(dsi, DSIM_ESCMODE_REG);

	if (lpm)
		v |= DSIM_CMD_LPDT_LP;
	else
		v &= ~DSIM_CMD_LPDT_LP;

	samsung_dsim_write(dsi, DSIM_ESCMODE_REG, v);
}

static void samsung_dsim_force_bta(struct samsung_dsim *dsi)
{
	u32 v = samsung_dsim_read(dsi, DSIM_ESCMODE_REG);

	v |= DSIM_FORCE_BTA;
	samsung_dsim_write(dsi, DSIM_ESCMODE_REG, v);
}

static void samsung_dsim_send_to_fifo(struct samsung_dsim *dsi,
				      struct samsung_dsim_transfer *xfer)
{
	struct device *dev = dsi->dev;
	struct mipi_dsi_packet *pkt = &xfer->packet;
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	const u8 *payload = pkt->payload + xfer->tx_done;
	u16 length = pkt->payload_length - xfer->tx_done;
	bool first = !xfer->tx_done;
	u32 reg;

	dev_dbg(dev, "< xfer %p: tx len %u, done %u, rx len %u, done %u\n",
		xfer, length, xfer->tx_done, xfer->rx_len, xfer->rx_done);

	if (length > DSI_TX_FIFO_SIZE)
		length = DSI_TX_FIFO_SIZE;

	xfer->tx_done += length;

	/* Send payload */
	while (length >= 4) {
		reg = get_unaligned_le32(payload);
		samsung_dsim_write(dsi, DSIM_PAYLOAD_REG, reg);
		payload += 4;
		length -= 4;
	}

	reg = 0;
	switch (length) {
	case 3:
		reg |= payload[2] << 16;
		fallthrough;
	case 2:
		reg |= payload[1] << 8;
		fallthrough;
	case 1:
		reg |= payload[0];
		samsung_dsim_write(dsi, DSIM_PAYLOAD_REG, reg);
		break;
	}

	/* Send packet header */
	if (!first)
		return;

	reg = get_unaligned_le32(pkt->header);
	if (driver_data->wait_for_hdr_fifo) {
		if (samsung_dsim_wait_for_hdr_fifo(dsi)) {
			dev_err(dev, "waiting for header FIFO timed out\n");
			return;
		}
	}

	if (NEQV(xfer->flags & MIPI_DSI_MSG_USE_LPM,
		 dsi->state & DSIM_STATE_CMD_LPM)) {
		samsung_dsim_set_cmd_lpm(dsi, xfer->flags & MIPI_DSI_MSG_USE_LPM);
		dsi->state ^= DSIM_STATE_CMD_LPM;
	}

	samsung_dsim_write(dsi, DSIM_PKTHDR_REG, reg);

	if (xfer->flags & MIPI_DSI_MSG_REQ_ACK)
		samsung_dsim_force_bta(dsi);
}

static void samsung_dsim_read_from_fifo(struct samsung_dsim *dsi,
					struct samsung_dsim_transfer *xfer)
{
	u8 *payload = xfer->rx_payload + xfer->rx_done;
	bool first = !xfer->rx_done;
	struct device *dev = dsi->dev;
	u16 length;
	u32 reg;

	if (first) {
		reg = samsung_dsim_read(dsi, DSIM_RXFIFO_REG);

		switch (reg & 0x3f) {
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
			if (xfer->rx_len >= 2) {
				payload[1] = reg >> 16;
				++xfer->rx_done;
			}
			fallthrough;
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
			payload[0] = reg >> 8;
			++xfer->rx_done;
			xfer->rx_len = xfer->rx_done;
			xfer->result = 0;
			goto clear_fifo;
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			dev_err(dev, "DSI Error Report: 0x%04x\n", (reg >> 8) & 0xffff);
			xfer->result = 0;
			goto clear_fifo;
		}

		length = (reg >> 8) & 0xffff;
		if (length > xfer->rx_len) {
			dev_err(dev,
				"response too long (%u > %u bytes), stripping\n",
				xfer->rx_len, length);
			length = xfer->rx_len;
		} else if (length < xfer->rx_len) {
			xfer->rx_len = length;
		}
	}

	length = xfer->rx_len - xfer->rx_done;
	xfer->rx_done += length;

	/* Receive payload */
	while (length >= 4) {
		reg = samsung_dsim_read(dsi, DSIM_RXFIFO_REG);
		payload[0] = (reg >>  0) & 0xff;
		payload[1] = (reg >>  8) & 0xff;
		payload[2] = (reg >> 16) & 0xff;
		payload[3] = (reg >> 24) & 0xff;
		payload += 4;
		length -= 4;
	}

	if (length) {
		reg = samsung_dsim_read(dsi, DSIM_RXFIFO_REG);
		switch (length) {
		case 3:
			payload[2] = (reg >> 16) & 0xff;
			fallthrough;
		case 2:
			payload[1] = (reg >> 8) & 0xff;
			fallthrough;
		case 1:
			payload[0] = reg & 0xff;
		}
	}

	if (xfer->rx_done == xfer->rx_len)
		xfer->result = 0;

clear_fifo:
	length = DSI_RX_FIFO_SIZE / 4;
	do {
		reg = samsung_dsim_read(dsi, DSIM_RXFIFO_REG);
		if (reg == DSI_RX_FIFO_EMPTY)
			break;
	} while (--length);
}

static void samsung_dsim_transfer_start(struct samsung_dsim *dsi)
{
	unsigned long flags;
	struct samsung_dsim_transfer *xfer;

	spin_lock_irqsave(&dsi->transfer_lock, flags);

	while (!list_empty(&dsi->transfer_list)) {
		xfer = list_first_entry(&dsi->transfer_list,
					struct samsung_dsim_transfer, list);

		spin_unlock_irqrestore(&dsi->transfer_lock, flags);

		if (xfer->packet.payload_length &&
		    xfer->tx_done == xfer->packet.payload_length)
			/* waiting for RX */
			return;

		samsung_dsim_send_to_fifo(dsi, xfer);

		if (xfer->packet.payload_length || xfer->rx_len)
			return;

		xfer->result = 0;
		complete(&xfer->completed);

		spin_lock_irqsave(&dsi->transfer_lock, flags);

		list_del_init(&xfer->list);
	}

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);
}

static bool samsung_dsim_transfer_finish(struct samsung_dsim *dsi)
{
	struct samsung_dsim_transfer *xfer;
	unsigned long flags;
	bool start = true;

	spin_lock_irqsave(&dsi->transfer_lock, flags);

	if (list_empty(&dsi->transfer_list)) {
		spin_unlock_irqrestore(&dsi->transfer_lock, flags);
		return false;
	}

	xfer = list_first_entry(&dsi->transfer_list,
				struct samsung_dsim_transfer, list);

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);

	dev_dbg(dsi->dev,
		"> xfer %p, tx_len %zu, tx_done %u, rx_len %u, rx_done %u\n",
		xfer, xfer->packet.payload_length, xfer->tx_done, xfer->rx_len,
		xfer->rx_done);

	if (xfer->tx_done != xfer->packet.payload_length)
		return true;

	if (xfer->rx_done != xfer->rx_len)
		samsung_dsim_read_from_fifo(dsi, xfer);

	if (xfer->rx_done != xfer->rx_len)
		return true;

	spin_lock_irqsave(&dsi->transfer_lock, flags);

	list_del_init(&xfer->list);
	start = !list_empty(&dsi->transfer_list);

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);

	if (!xfer->rx_len)
		xfer->result = 0;
	complete(&xfer->completed);

	return start;
}

static void samsung_dsim_remove_transfer(struct samsung_dsim *dsi,
					 struct samsung_dsim_transfer *xfer)
{
	unsigned long flags;
	bool start;

	spin_lock_irqsave(&dsi->transfer_lock, flags);

	if (!list_empty(&dsi->transfer_list) &&
	    xfer == list_first_entry(&dsi->transfer_list,
				     struct samsung_dsim_transfer, list)) {
		list_del_init(&xfer->list);
		start = !list_empty(&dsi->transfer_list);
		spin_unlock_irqrestore(&dsi->transfer_lock, flags);
		if (start)
			samsung_dsim_transfer_start(dsi);
		return;
	}

	list_del_init(&xfer->list);

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);
}

static int samsung_dsim_transfer(struct samsung_dsim *dsi,
				 struct samsung_dsim_transfer *xfer)
{
	unsigned long flags;
	bool stopped;

	xfer->tx_done = 0;
	xfer->rx_done = 0;
	xfer->result = -ETIMEDOUT;
	init_completion(&xfer->completed);

	spin_lock_irqsave(&dsi->transfer_lock, flags);

	stopped = list_empty(&dsi->transfer_list);
	list_add_tail(&xfer->list, &dsi->transfer_list);

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);

	if (stopped)
		samsung_dsim_transfer_start(dsi);

	wait_for_completion_timeout(&xfer->completed,
				    msecs_to_jiffies(DSI_XFER_TIMEOUT_MS));
	if (xfer->result == -ETIMEDOUT) {
		struct mipi_dsi_packet *pkt = &xfer->packet;

		samsung_dsim_remove_transfer(dsi, xfer);
		dev_err(dsi->dev, "xfer timed out: %*ph %*ph\n", 4, pkt->header,
			(int)pkt->payload_length, pkt->payload);
		return -ETIMEDOUT;
	}

	/* Also covers hardware timeout condition */
	return xfer->result;
}

static irqreturn_t samsung_dsim_irq_exynos8895(struct samsung_dsim *dsi);

static irqreturn_t samsung_dsim_irq(int irq, void *dev_id)
{
	struct samsung_dsim *dsi = dev_id;
	u32 status;

	if (dsi->driver_data->has_8895_link)
		return samsung_dsim_irq_exynos8895(dsi);

	status = samsung_dsim_read(dsi, DSIM_INTSRC_REG);
	if (!status) {
		static unsigned long j;

		if (printk_timed_ratelimit(&j, 500))
			dev_warn(dsi->dev, "spurious interrupt\n");
		return IRQ_HANDLED;
	}
	samsung_dsim_write(dsi, DSIM_INTSRC_REG, status);

	if (status & DSIM_INT_SW_RST_RELEASE) {
		unsigned long mask = ~(DSIM_INT_RX_DONE |
				       DSIM_INT_SFR_FIFO_EMPTY |
				       DSIM_INT_SFR_HDR_FIFO_EMPTY |
				       DSIM_INT_RX_ECC_ERR |
				       DSIM_INT_SW_RST_RELEASE);
		samsung_dsim_write(dsi, DSIM_INTMSK_REG, mask);
		complete(&dsi->completed);
		return IRQ_HANDLED;
	}

	if (!(status & (DSIM_INT_RX_DONE | DSIM_INT_SFR_FIFO_EMPTY |
			DSIM_INT_PLL_STABLE)))
		return IRQ_HANDLED;

	if (samsung_dsim_transfer_finish(dsi))
		samsung_dsim_transfer_start(dsi);

	return IRQ_HANDLED;
}

static void samsung_dsim_enable_irq(struct samsung_dsim *dsi)
{
	enable_irq(dsi->irq);

	if (dsi->te_gpio)
		enable_irq(gpiod_to_irq(dsi->te_gpio));
}

static void samsung_dsim_disable_irq(struct samsung_dsim *dsi)
{
	if (dsi->te_gpio)
		disable_irq(gpiod_to_irq(dsi->te_gpio));

	disable_irq(dsi->irq);
}

/*
 * ---- Exynos 8895 link (has_8895_link) --------------------------------------
 * The vendor sequence of the Exynos 8895 kernel: dsim_reg_ready() +
 * dsim_reg_init() + dsim_reg_start() for the cold init
 * (S:drivers/video/fbdev/exynos/dpu/dsim_drv.c:1041-1100,
 * S:drivers/video/fbdev/exynos/dpu/dsim_reg.c:1308-1402,1452-1551,1822-1826),
 * dsim_reg_stop() for the stop (dsim_reg.c:1829-1844), dsim_write_data() for
 * the transfer (dsim_drv.c:115-279) and dsim_irq_handler() (dsim_drv.c:941-992).
 */
#define DSIM8895_DPHY_RST		BIT(16)	/* SWRST, active low (regs-dsim.h:18) */
#define DSIM8895_LINK_STATUS2		0x10	/* regs-dsim.h:29-30 */
#define DSIM8895_CLK_LANE_STATUS(x)	(((x) >> 4) & 0x3)
#define DSIM8895_DESKEW_CTRL		0x24	/* regs-dsim.h:61-62 */
#define DSIM8895_DESKEW_HW_EN		BIT(15)
#define DSIM8895_NUM_OF_TRANSFER	0x30	/* regs-dsim.h:89-91, [19:0] */
#define DSIM8895_UNDERRUN_CTRL		0x34	/* regs-dsim.h:94-96, [15:0] */
#define DSIM8895_THRESHOLD		0x38	/* regs-dsim.h:98-100, [15:0] */
#define DSIM8895_CPRS_CTRL		0x74	/* regs-dsim.h:233-236 */
#define DSIM8895_MULTI_SLICE_PACKET	BIT(3)
#define DSIM8895_SLICE01		0x78	/* regs-dsim.h:240-246 */
#define DSIM8895_SLICE23		0x7c	/* regs-dsim.h:249-255 */
#define DSIM8895_CMD_CONFIG		0x80	/* regs-dsim.h:258-260 */
#define DSIM8895_PKT_GO_EN		BIT(16)
#define DSIM8895_CMD_TE_CTRL0		0x84	/* regs-dsim.h:266-268 */
#define DSIM8895_CMD_TE_CTRL1		0x88	/* regs-dsim.h:273-277 */
#define DSIM8895_PLL_CTRL1		0xa4	/* regs-dsim.h:323 */
#define DSIM8895_PHYCTRL_B2		0xb4	/* regs-dsim.h:339, then B3 0xb8, B4 0xbc, M1..M4 0xc0..0xcc */
/* CLK_CTRL (regs-dsim.h:50-59) */
#define DSIM8895_NONCONT_CLOCK_LANE	BIT(25)
#define DSIM8895_TX_REQUEST_HSCLK	BIT(20)
#define DSIM8895_WORDCLK_EN		BIT(17)
#define DSIM8895_ESCCLK_EN		BIT(16)
#define DSIM8895_LANE_ESCCLK_EN_MASK	(0x1f << 8)
/* CONFIG (regs-dsim.h:137-155) */
#define DSIM8895_CPRS_EN		BIT(19)
#define DSIM8895_VIDEO_MODE		BIT(18)
#define DSIM8895_VC_ID_MASK		(0x3 << 15)
#define DSIM8895_PIXEL_FORMAT_MASK	(0x3 << 13)	/* 0 = RGB24 (dsim.h:38) */
#define DSIM8895_PER_FRAME_READ_EN	BIT(8)
#define DSIM8895_EOTP_EN		BIT(7)
#define DSIM8895_NUM_OF_DATA_LANE_MASK	(0x3 << 5)
#define DSIM8895_LANES_EN_MASK		0x1f
/* SFR_CTRL (regs-dsim.h:210-212) */
#define DSIM8895_SHADOW_REG_READ_EN	BIT(1)
#define DSIM8895_SHADOW_EN		BIT(0)
/* FIFOCTRL EMPTY_PH_SFR (regs-dsim.h:220) */
#define DSIM8895_EMPTY_PH_SFR		BIT(10)
/* PKTHDR BTA type (regs-dsim.h:197) */
#define DSIM8895_PKTHDR_BTA		BIT(24)
/* INTSRC/INTMSK bits (regs-dsim.h:158-193) */
#define DSIM8895_INT_SW_RST_RELEASE	BIT(30)
#define DSIM8895_INT_SFR_PL_FIFO_EMPTY	BIT(29)
#define DSIM8895_INT_SFR_PH_FIFO_EMPTY	BIT(28)
#define DSIM8895_INT_FRAME_DONE		BIT(24)
#define DSIM8895_INT_INVALID_SFR_VALUE	BIT(23)
#define DSIM8895_INT_UNDER_RUN		BIT(19)
#define DSIM8895_INT_RX_DATA_DONE	BIT(18)
#define DSIM8895_INT_ERR_RX_ECC		BIT(15)
#define DSIM8895_INT_VT_STATUS		BIT(13)
#define DSIM8895_INT_PLL_STABLE		BIT(31)
/* dsim_reg_set_int() (dsim_reg.c:1584-1601) */
#define DSIM8895_INTMSK_RUN	(DSIM8895_INT_SW_RST_RELEASE | DSIM8895_INT_SFR_PL_FIFO_EMPTY | \
				 DSIM8895_INT_SFR_PH_FIFO_EMPTY | DSIM8895_INT_FRAME_DONE | \
				 DSIM8895_INT_INVALID_SFR_VALUE | DSIM8895_INT_UNDER_RUN | \
				 DSIM8895_INT_RX_DATA_DONE | DSIM8895_INT_ERR_RX_ECC | \
				 DSIM8895_INT_VT_STATUS)
/* dsim_reg.c:15-27 */
#define DSIM8895_BTA_TIMEOUT		0xff
#define DSIM8895_LP_RX_TIMEOUT		0xffff
#define DSIM8895_STABLE_VFP_VALUE	2
#define DSIM8895_TE_PROTECT_ON_TIME	158	/* 15.8 ms */
#define DSIM8895_TE_TIMEOUT_TIME	180	/* 18 ms */
#define DSIM8895_WR_TIMEOUT_MS		33	/* dsim.h:50 MIPI_WR_TIMEOUT */
#define DSIM8895_RD_TIMEOUT_MS		100	/* dsim.h:51 MIPI_RD_TIMEOUT */
/* SYSREG_DPU0 +0x1000 DISP_LPMUX_CFG (regs-decon.h:21, FDT:7465 base 0x12821000) */
#define DSIM8895_SYSREG_LPMUX_CFG	0x1000
#define DSIM8895_LPMUX_VAL		0x1201	/* decon_reg.c:667-670 */
/* SYSREG +0x200 BUS_COMPONENT_DRCG_EN (pmucal_local_exynos8895.h:267) */
#define DSIM8895_SYSREG_DRCG_EN		0x200

static void dsim8895_write(struct samsung_dsim *dsi, u32 ofs, u32 val)
{
	writel(val, dsi->reg_base + ofs);
}

static u32 dsim8895_read(struct samsung_dsim *dsi, u32 ofs)
{
	return readl(dsi->reg_base + ofs);
}

/* dsim_write_mask() of the vendor driver */
static void dsim8895_mask(struct samsung_dsim *dsi, u32 ofs, u32 val, u32 mask)
{
	u32 v = readl(dsi->reg_base + ofs);

	writel((v & ~mask) | (val & mask), dsi->reg_base + ofs);
}

static unsigned int dsim8895_hs_mhz(const struct samsung_dsim_fixed_link *l)
{
	/* dsim_reg.c:1480 pll_freq = (m * fin) / (p * (1 << s)) */
	return (l->m * l->fin_mhz) / (l->p * (1 << l->s));
}

/* dsim_reg_sw_reset(): set bit 0, poll it clear, 10 us x 1000 (dsim_reg.c:265-279) */
static void dsim8895_sw_reset(struct samsung_dsim *dsi)
{
	u32 cnt = 1000, state;

	dsim8895_mask(dsi, 0x04, DSIM_SWRST, DSIM_SWRST);
	do {
		state = dsim8895_read(dsi, 0x04) & DSIM_SWRST;
		cnt--;
		udelay(10);
	} while (state && cnt);
	if (!cnt)
		dev_err(dsi->dev, "sw reset timeout\n");
}

/* dsim_reg_set_clocks(en = 1) (dsim_reg.c:1452-1542) */
static int dsim8895_set_clocks(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_fixed_link *l = dsi->driver_data->fixed_link;
	unsigned int hs = dsim8895_hs_mhz(l);
	unsigned int word = hs / l->word_clk_div;
	unsigned int esc_req = dsi->esc_clk_rate / HZ_PER_MHZ;
	unsigned int esc_div, esc, row, i;
	const u32 *t;
	int cnt;

	if (!esc_req)
		return -EINVAL;
	/* escape clock divider with the vendor rounding (:1497-1503) */
	esc_div = word / esc_req;
	if (!esc_div)
		esc_div = 1;
	if ((word / esc_div) > esc_req)
		esc_div += 1;
	esc = word / esc_div;
	/* ESCCLK_EN + prescaler (:482-489) */
	dsim8895_mask(dsi, 0x20, DSIM8895_ESCCLK_EN | esc_div,
		      DSIM8895_ESCCLK_EN | 0xff);

	/* D-PHY timing row = (row0 bps - hs) / 10 (:1267-1306) */
	row = (l->dphy_timing[0][0] - hs) / 10;
	if (row >= l->dphy_timing_rows || esc < 7 || esc - 7 >= l->b_dphyctl_n) {
		dev_err(dsi->dev, "no D-PHY timing for hs %u MHz esc %u MHz\n", hs, esc);
		return -EINVAL;
	}
	t = l->dphy_timing[row];
	/* dsim_reg_set_dphy_timing_values() (:404-425) */
	dsim8895_write(dsi, 0xd0, (t[8] << 8) | t[9]);
	dsim8895_write(dsi, 0xd4, (t[1] << 24) | (t[2] << 16) | (t[3] << 8) | t[4]);
	dsim8895_write(dsi, 0xd8, (t[5] << 16) | (t[6] << 8) | t[7]);
	dsim8895_write(dsi, 0xb0, l->b_dphyctl[esc - 7]);

	/* characteristic words (:1527-1535): PLL_CTRL1/2, then B2..B4, M1..M4 */
	dsim8895_write(dsi, DSIM8895_PLL_CTRL1, l->phy_charic[0]);
	dsim8895_write(dsi, DSIM8895_PLL_CTRL1 + 4, l->phy_charic[1]);
	for (i = 2; i < 9; i++)
		dsim8895_write(dsi, DSIM8895_PHYCTRL_B2 + (i - 2) * 4, l->phy_charic[i]);

	/* PMS (:350-355), lock time (:357-360) */
	dsim8895_mask(dsi, 0xa0, ((l->p & 0x3f) << 13) | ((l->m & 0x3ff) << 3) | (l->s & 0x7),
		      0x7ffff);
	dsim8895_write(dsi, dsi->driver_data->plltmr_reg,
		       dsi->driver_data->reg_values[PLL_TIMER]);

	/* dsim_reg_enable_pll(1) (:455-480): the counter drops twice per pass */
	dsim8895_write(dsi, 0x50, DSIM8895_INT_PLL_STABLE);
	dsim8895_mask(dsi, 0xa0, DSIM_PLL_EN, DSIM_PLL_EN);
	cnt = 1000;
	while (cnt--) {
		cnt--;
		if (dsim8895_read(dsi, 0x14) & BIT(0))
			goto stable;
		if (cnt == 0)
			break;
		udelay(10);
	}
	dev_err(dsi->dev, "PLL lock timeout, LINK_STATUS3=0x%08x PLLCTRL=0x%08x\n",
		dsim8895_read(dsi, 0x14), dsim8895_read(dsi, 0xa0));
	return -EBUSY;

stable:
	dsi->hs_clock = hs * HZ_PER_MHZ;
	dev_info(dsi->dev, "PLL stable, hs %u MHz (PMS %u/%u/%u, esc %u MHz)\n",
		 hs, l->p, l->m, l->s, esc);
	return 0;
}

/* dsim_reg_set_config() for MIPI command mode (dsim_reg.c:1308-1402) */
static int dsim8895_set_config(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_fixed_link *l = dsi->driver_data->fixed_link;
	const struct drm_display_mode *m = &dsi->mode;
	unsigned int xres = m->hdisplay, yres = m->vdisplay;
	unsigned int hs = dsim8895_hs_mhz(l);
	bool dsc = dsi->dsc;
	unsigned int threshold, slices = 0, slice_w = 0;
	u32 val;

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) {
		dev_err(dsi->dev, "video mode is not supported on this DSIM variant\n");
		return -EINVAL;
	}
	if (dsc) {
		slices = dsi->dsc->slice_count;
		if (slices != 2 && slices != 4) {
			dev_err(dsi->dev, "unsupported DSC slice count %u\n", slices);
			return -EINVAL;
		}
		slice_w = xres / slices;
	}

	/* non-continuous clock lane in command mode (:1317-1318, :877-882) */
	dsim8895_mask(dsi, 0x20, DSIM8895_NONCONT_CLOCK_LANE, DSIM8895_NONCONT_CLOCK_LANE);
	/* dsim_set_hw_deskew(id, 0): HW_EN = 0 only (:1320, :1209-1211, :1090-1095) */
	dsim8895_mask(dsi, DSIM8895_DESKEW_CTRL, 0, DSIM8895_DESKEW_HW_EN);
	/* BTA/LPDR timeouts (:530-542), stop state count (:523-528) */
	dsim8895_write(dsi, 0x28, (DSIM8895_BTA_TIMEOUT << 16) | DSIM8895_LP_RX_TIMEOUT);
	dsim8895_mask(dsi, 0x2c, DSIM_STOP_STATE_CNT(dsi->driver_data->reg_values[STOP_STATE_CNT]),
		      DSIM_STOP_STATE_CNT_MASK);
	/* command mode underrun LP reference (:1326-1327, :1097-1103) */
	dsim8895_mask(dsi, DSIM8895_UNDERRUN_CTRL, dsi->cmd_underrun_lp_ref, 0xffff);
	/* threshold (:1329-1335, :1105-1111) */
	threshold = dsc ? xres / 3 : xres;
	dsim8895_mask(dsi, DSIM8895_THRESHOLD, threshold, 0xffff);
	/* resolution; horizontal is xres / 3 with DSC (:658-677) */
	dsim8895_mask(dsi, 0x3c, (yres & 0x1fff) << 16, 0x1fff << 16);
	dsim8895_mask(dsi, 0x3c, (dsc ? xres / 3 : xres) & 0x1fff, 0x1fff);
	/* no porches in command mode (:679-691); transfers per frame (:1341-1348) */
	val = dsc ? xres * yres / threshold / 3 : xres * yres / threshold;
	dsim8895_mask(dsi, DSIM8895_NUM_OF_TRANSFER, val, 0xfffff);
	/* lanes, EOTP on, per-frame read off, RGB24, VC 0, command mode (:1350-1358) */
	dsim8895_mask(dsi, 0x4c, (dsi->lanes - 1) << 5, DSIM8895_NUM_OF_DATA_LANE_MASK);
	dsim8895_mask(dsi, 0x4c, DSIM8895_EOTP_EN, DSIM8895_EOTP_EN);
	dsim8895_mask(dsi, 0x4c, 0, DSIM8895_PER_FRAME_READ_EN);
	dsim8895_mask(dsi, 0x4c, 0, DSIM8895_PIXEL_FORMAT_MASK);
	dsim8895_mask(dsi, 0x4c, 0, DSIM8895_VC_ID_MASK);
	dsim8895_mask(dsi, 0x4c, 0, DSIM8895_VIDEO_MODE);
	/* DSC (:1360, :1372-1381) */
	dsim8895_mask(dsi, 0x4c, dsc ? DSIM8895_CPRS_EN : 0, DSIM8895_CPRS_EN);
	if (dsc) {
		dsim8895_mask(dsi, DSIM8895_CPRS_CTRL, slices, 0x7);
		/* command mode: one packet header for all slices (:751-767) */
		dsim8895_mask(dsi, DSIM8895_CPRS_CTRL, DSIM8895_MULTI_SLICE_PACKET,
			      DSIM8895_MULTI_SLICE_PACKET);
		/* slice 0..3 widths; 2 slices on 2 encoders use both words (:769-813) */
		val = ((slice_w & 0x1fff) << 16) | (slice_w & 0x1fff);
		dsim8895_write(dsi, DSIM8895_SLICE01, val);
		dsim8895_write(dsi, DSIM8895_SLICE23, val);
	}
	/* packet go off (:1387, :891-896) */
	dsim8895_mask(dsi, DSIM8895_CMD_CONFIG, 0, DSIM8895_PKT_GO_EN);
	/* TE based command timing (:863-875) */
	dsim8895_mask(dsi, DSIM8895_CMD_TE_CTRL0, xres * DSIM8895_STABLE_VFP_VALUE * 3 / 100,
		      0xffff);
	dsim8895_write(dsi, DSIM8895_CMD_TE_CTRL1,
		       (((hs * DSIM8895_TE_PROTECT_ON_TIME) / 16) << 16) |
		       ((hs * DSIM8895_TE_TIMEOUT_TIME) / 16));
	/* shadow read + shadow enable (:1397-1398) */
	dsim8895_mask(dsi, 0x64, DSIM8895_SHADOW_REG_READ_EN | DSIM8895_SHADOW_EN,
		      DSIM8895_SHADOW_REG_READ_EN | DSIM8895_SHADOW_EN);

	return 0;
}

/* dsim_reg_start(): HS clock request, unmask (dsim_reg.c:1822-1826,1568-1601) */
static void dsim8895_start(struct samsung_dsim *dsi)
{
	dsim8895_mask(dsi, 0x20, DSIM8895_TX_REQUEST_HSCLK, DSIM8895_TX_REQUEST_HSCLK);
	/* non-continuous clock: the vendor driver does not wait for TX_READY_HS_CLK */
	dsim8895_mask(dsi, 0x54, 0, DSIM8895_INTMSK_RUN);
}

static int samsung_dsim_init_exynos8895(struct samsung_dsim *dsi)
{
	int ret;

	if (dsi->state & DSIM_STATE_INITIALIZED)
		return 0;

	/* DSIM0 -> D-PHY mux (dsim_drv.c:1051, decon_reg.c:667-670) */
	if (dsi->disp_sysreg)
		regmap_write(dsi->disp_sysreg, DSIM8895_SYSREG_LPMUX_CFG, DSIM8895_LPMUX_VAL);

	samsung_dsim_enable_irq(dsi);

	if (dsi->handover) {
		/*
		 * The boot loader left the link running: skip link/D-PHY init as
		 * the vendor driver does ("PLL is stabled in bootloader, so skip
		 * DSIM link/DPHY init.", dsim_drv.c:1058-1064), then dsim_reg_start()
		 * (:1120) and no panel SLEEP_OUT (:1126-1130).
		 */
		dsim8895_start(dsi);
		dsi->hs_clock = dsim8895_hs_mhz(dsi->driver_data->fixed_link) * HZ_PER_MHZ;
		dsi->state |= DSIM_STATE_INITIALIZED;
		dsi->handover = false;
		pm_runtime_put_noidle(dsi->dev);
		dev_info(dsi->dev, "S-Boot DSIM link kept (PLL stable), HS clock requested\n");
		return 0;
	}

	/* D-PHY reset assert (dsim_drv.c:1068, dsim_reg.c:287-291) */
	dsim8895_mask(dsi, 0x04, 0, DSIM8895_DPHY_RST);
	dsim8895_sw_reset(dsi);
	ret = dsim8895_set_clocks(dsi);
	if (ret)
		goto err;
	/* lanes: clock + data (dsim_reg.c:343-348,1553-1559), then 300 us (dsim_drv.c:1080) */
	dsim8895_mask(dsi, 0x4c, (BIT(dsi->lanes + 1) - 1), DSIM8895_LANES_EN_MASK);
	udelay(400);
	udelay(300);
	/* lane escape clocks: data lanes + clock lane bit 4 (dsim_reg.c:491-521) */
	dsim8895_mask(dsi, 0x20, ((BIT(dsi->lanes) - 1) | BIT(4)) << 8,
		      DSIM8895_LANE_ESCCLK_EN_MASK);
	dsim8895_mask(dsi, 0x20, DSIM8895_WORDCLK_EN, DSIM8895_WORDCLK_EN);
	/* D-PHY reset release (dsim_drv.c:1086) */
	dsim8895_mask(dsi, 0x04, DSIM8895_DPHY_RST, DSIM8895_DPHY_RST);
	ret = dsim8895_set_config(dsi);
	if (ret)
		goto err;
	dsim8895_start(dsi);
	dsi->state |= DSIM_STATE_INITIALIZED;
	return 0;

err:
	samsung_dsim_disable_irq(dsi);
	return ret;
}

/* dsim_reg_stop() (dsim_reg.c:1829-1844) */
static void samsung_dsim_stop_exynos8895(struct samsung_dsim *dsi)
{
	u32 cnt = 1000;

	dsim8895_write(dsi, 0x50, 0xffffffff);
	dsim8895_mask(dsi, 0x54, DSIM8895_INTMSK_RUN, DSIM8895_INTMSK_RUN);
	/* HS clock off, wait for the clock lane to be in LP (:924-929, :986-1003) */
	dsim8895_mask(dsi, 0x20, 0, DSIM8895_TX_REQUEST_HSCLK);
	while (DSIM8895_CLK_LANE_STATUS(dsim8895_read(dsi, DSIM8895_LINK_STATUS2)) && --cnt)
		udelay(10);
	if (!cnt)
		dev_err(dsi->dev, "clock lane did not return to LP\n");
	dsim8895_mask(dsi, 0x4c, 0, DSIM8895_LANES_EN_MASK);
	udelay(400);
	dsim8895_mask(dsi, 0x20, 0, DSIM8895_LANE_ESCCLK_EN_MASK);
	dsim8895_mask(dsi, 0x20, 0, DSIM8895_WORDCLK_EN);
	/* dsim_reg_set_clocks(en = 0) (:1543-1548) */
	dsim8895_write(dsi, DSIM8895_PLL_CTRL1, 0);
	dsim8895_write(dsi, DSIM8895_PLL_CTRL1 + 4, 0);
	dsim8895_mask(dsi, 0x20, 0xff, DSIM8895_ESCCLK_EN | 0xff);
	dsim8895_mask(dsi, 0xa0, 0, DSIM_PLL_EN);
	dsim8895_sw_reset(dsi);
}

/*
 * One packet: payload words, header, wait for PH_FIFO_EMPTY. The completion is
 * re-armed for every packet, as the vendor driver re-inits ph_wr_comp and clears
 * DSIM_INTSRC_SFR_PH_FIFO_EMPTY in dsim_write_data() (dsim_drv.c:209-210).
 */
static int samsung_dsim_8895_send(struct samsung_dsim *dsi, u32 hdr,
				  const void *payload, size_t len)
{
	const u8 *p = payload;
	u32 reg;
	int i;

	reinit_completion(&dsi->ph_empty);

	while (len >= 4) {
		dsim8895_write(dsi, 0x5c, get_unaligned_le32(p));
		p += 4;
		len -= 4;
	}
	if (len) {
		reg = 0;
		for (i = 0; i < len; i++)
			reg |= p[i] << (8 * i);
		dsim8895_write(dsi, 0x5c, reg);
	}

	dsim8895_mask(dsi, 0x58, hdr, 0x1ffffff);

	if (!wait_for_completion_timeout(&dsi->ph_empty,
					 msecs_to_jiffies(DSIM8895_WR_TIMEOUT_MS))) {
		/* a timeout with an empty header FIFO is success (dsim_drv.c:131-136) */
		if (!(dsim8895_read(dsi, 0x68) & DSIM8895_EMPTY_PH_SFR)) {
			dev_err(dsi->dev, "cmd write timeout, INTSRC=0x%08x FIFOCTRL=0x%08x\n",
				dsim8895_read(dsi, 0x50), dsim8895_read(dsi, 0x68));
			return -ETIMEDOUT;
		}
		dsim8895_write(dsi, 0x50, DSIM8895_INT_SFR_PH_FIFO_EMPTY);
	}

	return 0;
}

/*
 * One packet in flight, as the vendor driver ("Single Packet Transmission",
 * dsim_drv.c:168-187): clear PH_FIFO_EMPTY, payload words, header, wait for
 * PH_FIFO_EMPTY; a read then waits for RX_DATA_DONE.
 */
static ssize_t samsung_dsim_xfer_exynos8895(struct samsung_dsim *dsi,
					  const struct mipi_dsi_msg *msg)
{
	struct mipi_dsi_msg m = *msg;
	struct samsung_dsim_transfer xfer = { };
	u32 hdr;
	int ret;

	/*
	 * "When send TE_ON(35h) command with 1 para type, DSI goes to BTA. To
	 * prevent unintended BTA in that case, tx TE_ON(35h) command with 1 para
	 * will be sent with long packet type." (dsim_drv.c:405-413)
	 */
	if (dsi->driver_data->te_on_long_packet &&
	    m.type == MIPI_DSI_DCS_SHORT_WRITE_PARAM && m.tx_len == 2 &&
	    ((const u8 *)m.tx_buf)[0] == MIPI_DCS_SET_TEAR_ON)
		m.type = MIPI_DSI_DCS_LONG_WRITE;

	ret = mipi_dsi_create_packet(&xfer.packet, &m);
	if (ret < 0)
		return ret;

	mutex_lock(&dsi->cmd_lock);

	dsim8895_write(dsi, 0x50, DSIM8895_INT_SFR_PH_FIFO_EMPTY |
		       (m.rx_len ? DSIM8895_INT_RX_DATA_DONE : 0));

	/*
	 * dsim_read_data() writes the "set maximum return packet size" packet
	 * before the read itself (dsim_drv.c:312-313), with the same header
	 * builder as the read request below (dsim_reg.c:1026-1033,
	 * regs-dsim.h:196-201): data type 0x37 (video/mipi_display.h:42),
	 * data0 = size & 0xff, data1 = size >> 8, no BTA. Without it the DDI
	 * answers the read request with its previous maximum return packet size
	 * (one byte after reset) and mipi_dsi_dcs_read() returns a short read.
	 */
	if (m.rx_len) {
		hdr = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE |
		      (m.rx_len & 0xff) << 8 | ((m.rx_len >> 8) & 0xff) << 16;
		ret = samsung_dsim_8895_send(dsi, hdr, NULL, 0);
		if (ret)
			goto out;
	}

	hdr = get_unaligned_le32(xfer.packet.header) & 0xffffff;
	if (m.rx_len)
		hdr |= DSIM8895_PKTHDR_BTA;	/* dsim_drv.c:250 bta_type = true */
	ret = samsung_dsim_8895_send(dsi, hdr, xfer.packet.payload,
				     xfer.packet.payload_length);
	if (ret)
		goto out;

	/*
	 * The host has to return the number of bytes transmitted for a write
	 * and the number of bytes received for a read (drm_mipi_dsi.h:77-82);
	 * mipi_dsi_dcs_write_buffer() passes its return value on unchanged and
	 * mipi_dsi_dcs_read() documents it as "the number of bytes read"
	 * (drm_mipi_dsi.c:920-947, :1074-1082). Not payload_length: a short
	 * write carries its two parameters in the header and has no payload at
	 * all (drm_mipi_dsi.c:567-578), so a two byte DCS short write would
	 * report 0. The vendor driver returns size as well
	 * (dsim_drv.c:273) and compares it with size
	 * (dsim_drv.c:411, :319-323).
	 */
	ret = m.tx_len;
	if (!m.rx_len)
		goto out;

	if (!wait_for_completion_timeout(&dsi->rx_done,
					 msecs_to_jiffies(DSIM8895_RD_TIMEOUT_MS))) {
		dev_err(dsi->dev, "read timeout, INTSRC=0x%08x\n", dsim8895_read(dsi, 0x50));
		ret = -ETIMEDOUT;
		goto out;
	}
	xfer.rx_payload = m.rx_buf;
	xfer.rx_len = m.rx_len;
	xfer.result = -EIO;
	samsung_dsim_read_from_fifo(dsi, &xfer);
	ret = xfer.result ? xfer.result : xfer.rx_done;

out:
	mutex_unlock(&dsi->cmd_lock);
	return ret;
}

/*
 * mipi_read_cmd() (dsim_drv.c:780-806) is the only read entry point of the
 * vendor DSIM: it runs the read above again up to five times when the number of
 * bytes it got is not the requested one. Retries are for reads only.
 */
static ssize_t samsung_dsim_transfer_exynos8895(struct samsung_dsim *dsi,
						const struct mipi_dsi_msg *msg)
{
	ssize_t ret;
	int retry = 5;

	if (!msg->rx_len)
		return samsung_dsim_xfer_exynos8895(dsi, msg);

	for (;;) {
		ret = samsung_dsim_xfer_exynos8895(dsi, msg);
		if (ret >= 0 && (size_t)ret == msg->rx_len)
			return ret;
		if (!retry) {
			dev_err(dsi->dev, "failed to read addr 0x%02x, %zd of %zu bytes\n",
				((const u8 *)msg->tx_buf)[0], ret, msg->rx_len);
			return ret;
		}
		retry--;
	}
}

/* dsim_irq_handler() (dsim_drv.c:941-992); INTMSK is never rewritten here */
static irqreturn_t samsung_dsim_irq_exynos8895(struct samsung_dsim *dsi)
{
	u32 status = dsim8895_read(dsi, 0x50);

	if (!status)
		return IRQ_HANDLED;
	/* write back exactly what was read (dsim_drv.c:987, dsim_reg.c:427-430) */
	dsim8895_write(dsi, 0x50, status);

	if (status & DSIM8895_INT_SW_RST_RELEASE)
		complete(&dsi->completed);
	if (status & DSIM8895_INT_SFR_PH_FIFO_EMPTY)
		complete(&dsi->ph_empty);
	if (status & DSIM8895_INT_RX_DATA_DONE)
		complete(&dsi->rx_done);
	if (status & DSIM8895_INT_ERR_RX_ECC) {
		WRITE_ONCE(dsi->ecc_cnt, dsi->ecc_cnt + 1);
		greatlte_l5_inc(GL5_ON, ecc);
		dev_err_ratelimited(dsi->dev, "RX ECC multibit error\n");
	}
	if (status & DSIM8895_INT_UNDER_RUN) {
		WRITE_ONCE(dsi->underrun_cnt, dsi->underrun_cnt + 1);
		greatlte_l5_inc(GL5_ON, ur);
		dev_info_ratelimited(dsi->dev, "underrun (%u)\n", dsi->underrun_cnt);
	}

	return IRQ_HANDLED;
}

static int samsung_dsim_init(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;

	if (dsi->state & DSIM_STATE_INITIALIZED)
		return 0;

	if (driver_data->has_8895_link)
		return samsung_dsim_init_exynos8895(dsi);

	samsung_dsim_reset(dsi);
	samsung_dsim_enable_irq(dsi);

	if (driver_data->reg_values[RESET_TYPE] == DSIM_FUNCRST)
		samsung_dsim_enable_lane(dsi, BIT(dsi->lanes) - 1);

	samsung_dsim_enable_clock(dsi);
	if (driver_data->wait_for_reset)
		samsung_dsim_wait_for_reset(dsi);
	samsung_dsim_set_phy_ctrl(dsi);
	samsung_dsim_init_link(dsi);

	dsi->state |= DSIM_STATE_INITIALIZED;

	return 0;
}

static void samsung_dsim_atomic_pre_enable(struct drm_bridge *bridge,
					   struct drm_atomic_state *state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);
	int ret;

	if (dsi->state & DSIM_STATE_ENABLED)
		return;

	ret = pm_runtime_resume_and_get(dsi->dev);
	if (ret < 0) {
		dev_err(dsi->dev, "failed to enable DSI device.\n");
		return;
	}

	dsi->state |= DSIM_STATE_ENABLED;

	/*
	 * For Exynos-DSIM the downstream bridge, or panel are expecting
	 * the host initialization during DSI transfer.
	 */
	if (!samsung_dsim_hw_is_exynos(dsi->plat_data->hw_type)) {
		ret = samsung_dsim_init(dsi);
		if (ret)
			return;
	}
}

static void samsung_dsim_atomic_enable(struct drm_bridge *bridge,
				       struct drm_atomic_state *state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	/* Exynos 8895: resolution is set by the link init, there is no stand-by bit */
	if (!dsi->driver_data->has_8895_link) {
		samsung_dsim_set_display_mode(dsi);
		samsung_dsim_set_display_enable(dsi, true);
	}

	dsi->state |= DSIM_STATE_VIDOUT_AVAILABLE;
}

static void samsung_dsim_atomic_disable(struct drm_bridge *bridge,
					struct drm_atomic_state *state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	if (!(dsi->state & DSIM_STATE_ENABLED))
		return;

	if (!dsi->driver_data->has_8895_link)
		samsung_dsim_set_display_enable(dsi, false);
	dsi->state &= ~DSIM_STATE_VIDOUT_AVAILABLE;
}

static void samsung_dsim_atomic_post_disable(struct drm_bridge *bridge,
					     struct drm_atomic_state *state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	dsi->state &= ~DSIM_STATE_ENABLED;
	pm_runtime_put_sync(dsi->dev);
}

/*
 * This pixel output formats list referenced from,
 * AN13573 i.MX 8/RT MIPI DSI/CSI-2, Rev. 0, 21 March 2022
 * 3.7.4 Pixel formats
 * Table 14. DSI pixel packing formats
 */
static const u32 samsung_dsim_pixel_output_fmts[] = {
	MEDIA_BUS_FMT_YUYV10_1X20,
	MEDIA_BUS_FMT_YUYV12_1X24,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_RGB101010_1X30,
	MEDIA_BUS_FMT_RGB121212_1X36,
	MEDIA_BUS_FMT_RGB565_1X16,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB888_1X24,
};

static bool samsung_dsim_pixel_output_fmt_supported(u32 fmt)
{
	int i;

	if (fmt == MEDIA_BUS_FMT_FIXED)
		return false;

	for (i = 0; i < ARRAY_SIZE(samsung_dsim_pixel_output_fmts); i++) {
		if (samsung_dsim_pixel_output_fmts[i] == fmt)
			return true;
	}

	return false;
}

static u32 *
samsung_dsim_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
				       struct drm_bridge_state *bridge_state,
				       struct drm_crtc_state *crtc_state,
				       struct drm_connector_state *conn_state,
				       u32 output_fmt,
				       unsigned int *num_input_fmts)
{
	u32 *input_fmts;

	input_fmts = kmalloc(sizeof(*input_fmts), GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	if (!samsung_dsim_pixel_output_fmt_supported(output_fmt))
		/*
		 * Some bridge/display drivers are still not able to pass the
		 * correct format, so handle those pipelines by falling back
		 * to the default format till the supported formats finalized.
		 */
		output_fmt = MEDIA_BUS_FMT_RGB888_1X24;

	input_fmts[0] = output_fmt;
	*num_input_fmts = 1;

	return input_fmts;
}

static int samsung_dsim_atomic_check(struct drm_bridge *bridge,
				     struct drm_bridge_state *bridge_state,
				     struct drm_crtc_state *crtc_state,
				     struct drm_connector_state *conn_state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);
	struct drm_display_mode *adjusted_mode = &crtc_state->adjusted_mode;

	/*
	 * The i.MX8M Mini/Nano glue logic between LCDIF and DSIM
	 * inverts HS/VS/DE sync signals polarity, therefore, while
	 * i.MX 8M Mini Applications Processor Reference Manual Rev. 3, 11/2020
	 * 13.6.3.5.2 RGB interface
	 * i.MX 8M Nano Applications Processor Reference Manual Rev. 2, 07/2022
	 * 13.6.2.7.2 RGB interface
	 * both claim "Vsync, Hsync, and VDEN are active high signals.", the
	 * LCDIF must generate inverted HS/VS/DE signals, i.e. active LOW.
	 *
	 * The i.MX8M Plus glue logic between LCDIFv3 and DSIM does not
	 * implement the same behavior, therefore LCDIFv3 must generate
	 * HS/VS/DE signals active HIGH.
	 */
	if (dsi->plat_data->hw_type == DSIM_TYPE_IMX8MM) {
		adjusted_mode->flags |= (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
		adjusted_mode->flags &= ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	} else if (dsi->plat_data->hw_type == DSIM_TYPE_IMX8MP) {
		adjusted_mode->flags &= ~(DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
		adjusted_mode->flags |= (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	}

	/*
	 * When using video sync pulses, the HFP, HBP, and HSA are divided between
	 * the available lanes if there is more than one lane.  For certain
	 * timings and lane configurations, the HFP may not be evenly divisible.
	 * If the HFP is rounded down, it ends up being too small which can cause
	 * some monitors to not sync properly. In these instances, adjust htotal
	 * and hsync to round the HFP up, and recalculate the htotal. Through trial
	 * and error, it appears that the HBP and HSA do not appearto need the same
	 * correction that HFP does.
	 */
	if (dsi->lanes > 1) {
		int hfp = adjusted_mode->hsync_start - adjusted_mode->hdisplay;
		int remainder = hfp % dsi->lanes;

		if (remainder) {
			adjusted_mode->hsync_start += remainder;
			adjusted_mode->hsync_end   += remainder;
			adjusted_mode->htotal      += remainder;
		}
	}

	return 0;
}

static void samsung_dsim_mode_set(struct drm_bridge *bridge,
				  const struct drm_display_mode *mode,
				  const struct drm_display_mode *adjusted_mode)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	drm_mode_copy(&dsi->mode, adjusted_mode);
}

static int samsung_dsim_attach(struct drm_bridge *bridge,
			       struct drm_encoder *encoder,
			       enum drm_bridge_attach_flags flags)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	return drm_bridge_attach(encoder, dsi->out_bridge, bridge,
				 flags);
}

static const struct drm_bridge_funcs samsung_dsim_bridge_funcs = {
	.atomic_duplicate_state		= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state		= drm_atomic_helper_bridge_destroy_state,
	.atomic_reset			= drm_atomic_helper_bridge_reset,
	.atomic_get_input_bus_fmts	= samsung_dsim_atomic_get_input_bus_fmts,
	.atomic_check			= samsung_dsim_atomic_check,
	.atomic_pre_enable		= samsung_dsim_atomic_pre_enable,
	.atomic_enable			= samsung_dsim_atomic_enable,
	.atomic_disable			= samsung_dsim_atomic_disable,
	.atomic_post_disable		= samsung_dsim_atomic_post_disable,
	.mode_set			= samsung_dsim_mode_set,
	.attach				= samsung_dsim_attach,
};

static irqreturn_t samsung_dsim_te_irq_handler(int irq, void *dev_id)
{
	struct samsung_dsim *dsi = (struct samsung_dsim *)dev_id;
	const struct samsung_dsim_plat_data *pdata = dsi->plat_data;

	if (pdata->host_ops && pdata->host_ops->te_irq_handler)
		return pdata->host_ops->te_irq_handler(dsi);

	return IRQ_HANDLED;
}

static int samsung_dsim_register_te_irq(struct samsung_dsim *dsi, struct device *dev)
{
	int te_gpio_irq;
	int ret;

	dsi->te_gpio = devm_gpiod_get_optional(dev, "te", GPIOD_IN);
	if (!dsi->te_gpio)
		return 0;
	else if (IS_ERR(dsi->te_gpio))
		return dev_err_probe(dev, PTR_ERR(dsi->te_gpio), "failed to get te GPIO\n");

	te_gpio_irq = gpiod_to_irq(dsi->te_gpio);

	ret = request_threaded_irq(te_gpio_irq, samsung_dsim_te_irq_handler, NULL,
				   IRQF_TRIGGER_RISING | IRQF_NO_AUTOEN, "TE", dsi);
	if (ret) {
		dev_err(dsi->dev, "request interrupt failed with %d\n", ret);
		gpiod_put(dsi->te_gpio);
		return ret;
	}

	return 0;
}

static void samsung_dsim_unregister_te_irq(struct samsung_dsim *dsi)
{
	if (dsi->te_gpio) {
		free_irq(gpiod_to_irq(dsi->te_gpio), dsi);
		gpiod_put(dsi->te_gpio);
	}
}

static int samsung_dsim_host_attach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *device)
{
	struct samsung_dsim *dsi = host_to_dsi(host);
	const struct samsung_dsim_plat_data *pdata = dsi->plat_data;
	struct device *dev = dsi->dev;
	struct device_node *np = dev->of_node;
	struct device_node *remote;
	struct drm_panel *panel;
	int ret;

	/*
	 * Devices can also be child nodes when we also control that device
	 * through the upstream device (ie, MIPI-DCS for a MIPI-DSI device).
	 *
	 * Lookup for a child node of the given parent that isn't either port
	 * or ports.
	 */
	for_each_available_child_of_node(np, remote) {
		if (of_node_name_eq(remote, "port") ||
		    of_node_name_eq(remote, "ports"))
			continue;

		goto of_find_panel_or_bridge;
	}

	/*
	 * of_graph_get_remote_node() produces a noisy error message if port
	 * node isn't found and the absence of the port is a legit case here,
	 * so at first we silently check whether graph presents in the
	 * device-tree node.
	 */
	if (!of_graph_is_present(np))
		return -ENODEV;

	remote = of_graph_get_remote_node(np, 1, 0);

of_find_panel_or_bridge:
	if (!remote)
		return -ENODEV;

	panel = of_drm_find_panel(remote);
	if (!IS_ERR(panel)) {
		dsi->out_bridge = devm_drm_panel_bridge_add(dev, panel);
	} else {
		dsi->out_bridge = of_drm_find_bridge(remote);
		if (!dsi->out_bridge)
			dsi->out_bridge = ERR_PTR(-EINVAL);
	}

	of_node_put(remote);

	if (IS_ERR(dsi->out_bridge)) {
		ret = PTR_ERR(dsi->out_bridge);
		DRM_DEV_ERROR(dev, "failed to find the bridge: %d\n", ret);
		return ret;
	}

	DRM_DEV_INFO(dev, "Attached %s device (lanes:%d bpp:%d mode-flags:0x%lx)\n",
		     device->name, device->lanes,
		     mipi_dsi_pixel_format_to_bpp(device->format),
		     device->mode_flags);

	drm_bridge_add(&dsi->bridge);

	/*
	 * This is a temporary solution and should be made by more generic way.
	 *
	 * If attached panel device is for command mode one, dsi should register
	 * TE interrupt handler.
	 */
	if (!(device->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		ret = samsung_dsim_register_te_irq(dsi, &device->dev);
		if (ret)
			goto err_remove_bridge;
	}

	if (pdata->host_ops && pdata->host_ops->attach) {
		ret = pdata->host_ops->attach(dsi, device);
		if (ret)
			goto err_unregister_te_irq;
	}

	dsi->lanes = device->lanes;
	dsi->format = device->format;
	dsi->mode_flags = device->mode_flags;
	dsi->dsc = device->dsc;

	return 0;

err_unregister_te_irq:
	if (!(device->mode_flags & MIPI_DSI_MODE_VIDEO))
		samsung_dsim_unregister_te_irq(dsi);
err_remove_bridge:
	drm_bridge_remove(&dsi->bridge);
	return ret;
}

static int samsung_dsim_host_detach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *device)
{
	struct samsung_dsim *dsi = host_to_dsi(host);
	const struct samsung_dsim_plat_data *pdata = dsi->plat_data;

	dsi->out_bridge = NULL;

	if (pdata->host_ops && pdata->host_ops->detach)
		pdata->host_ops->detach(dsi, device);

	samsung_dsim_unregister_te_irq(dsi);

	drm_bridge_remove(&dsi->bridge);

	return 0;
}

static ssize_t samsung_dsim_host_transfer(struct mipi_dsi_host *host,
					  const struct mipi_dsi_msg *msg)
{
	struct samsung_dsim *dsi = host_to_dsi(host);
	struct samsung_dsim_transfer xfer;
	int ret;

	if (!(dsi->state & DSIM_STATE_ENABLED))
		return -EINVAL;

	ret = samsung_dsim_init(dsi);
	if (ret)
		return ret;

	if (dsi->driver_data->has_8895_link)
		return samsung_dsim_transfer_exynos8895(dsi, msg);

	ret = mipi_dsi_create_packet(&xfer.packet, msg);
	if (ret < 0)
		return ret;

	xfer.rx_len = msg->rx_len;
	xfer.rx_payload = msg->rx_buf;
	xfer.flags = msg->flags;

	ret = samsung_dsim_transfer(dsi, &xfer);
	return (ret < 0) ? ret : xfer.rx_done;
}

static const struct mipi_dsi_host_ops samsung_dsim_ops = {
	.attach = samsung_dsim_host_attach,
	.detach = samsung_dsim_host_detach,
	.transfer = samsung_dsim_host_transfer,
};

static int samsung_dsim_of_read_u32(const struct device_node *np,
				    const char *propname, u32 *out_value, bool optional)
{
	int ret = of_property_read_u32(np, propname, out_value);

	if (ret < 0 && !optional)
		pr_err("%pOF: failed to get '%s' property\n", np, propname);

	return ret;
}

static int samsung_dsim_parse_dt(struct samsung_dsim *dsi)
{
	struct device *dev = dsi->dev;
	struct device_node *node = dev->of_node;
	u32 lane_polarities[5] = { 0 };
	struct device_node *endpoint;
	int i, nr_lanes, ret;

	ret = samsung_dsim_of_read_u32(node, "samsung,pll-clock-frequency",
				       &dsi->pll_clk_rate, 1);
	/* If it doesn't exist, read it from the clock instead of failing */
	if (ret < 0) {
		dev_dbg(dev, "Using sclk_mipi for pll clock frequency\n");
		dsi->pll_clk = devm_clk_get(dev, "sclk_mipi");
		if (IS_ERR(dsi->pll_clk))
			return PTR_ERR(dsi->pll_clk);
	}

	/* If it doesn't exist, use pixel clock instead of failing */
	ret = samsung_dsim_of_read_u32(node, "samsung,burst-clock-frequency",
				       &dsi->burst_clk_rate, 1);
	if (ret < 0) {
		dev_dbg(dev, "Using pixel clock for HS clock frequency\n");
		dsi->burst_clk_rate = 0;
	}

	ret = samsung_dsim_of_read_u32(node, "samsung,esc-clock-frequency",
				       &dsi->esc_clk_rate, 0);
	if (ret < 0)
		return ret;

	endpoint = of_graph_get_endpoint_by_regs(node, 1, -1);
	nr_lanes = of_property_count_u32_elems(endpoint, "data-lanes");
	if (nr_lanes > 0 && nr_lanes <= 4) {
		/* Polarity 0 is clock lane, 1..4 are data lanes. */
		of_property_read_u32_array(endpoint, "lane-polarities",
					   lane_polarities, nr_lanes + 1);
		for (i = 1; i <= nr_lanes; i++) {
			if (lane_polarities[1] != lane_polarities[i])
				DRM_DEV_ERROR(dsi->dev, "Data lanes polarities do not match");
		}
		if (lane_polarities[0])
			dsi->swap_dn_dp_clk = true;
		if (lane_polarities[1])
			dsi->swap_dn_dp_data = true;
	}

	return 0;
}

static int generic_dsim_register_host(struct samsung_dsim *dsi)
{
	return mipi_dsi_host_register(&dsi->dsi_host);
}

static void generic_dsim_unregister_host(struct samsung_dsim *dsi)
{
	mipi_dsi_host_unregister(&dsi->dsi_host);
}

static const struct samsung_dsim_host_ops generic_dsim_host_ops = {
	.register_host = generic_dsim_register_host,
	.unregister_host = generic_dsim_unregister_host,
};

static const struct drm_bridge_timings samsung_dsim_bridge_timings_de_high = {
	.input_bus_flags = DRM_BUS_FLAG_DE_HIGH,
};

static const struct drm_bridge_timings samsung_dsim_bridge_timings_de_low = {
	.input_bus_flags = DRM_BUS_FLAG_DE_LOW,
};

static int samsung_dsim_suspend(struct device *dev);
static int samsung_dsim_resume(struct device *dev);

static int samsung_dsim_parse_dt_exynos8895(struct samsung_dsim *dsi)
{
	struct device_node *np = dsi->dev->of_node;

	dsi->disp_sysreg = syscon_regmap_lookup_by_phandle(np, "samsung,disp-sysreg");
	if (IS_ERR(dsi->disp_sysreg))
		return dev_err_probe(dsi->dev, PTR_ERR(dsi->disp_sysreg),
				     "no samsung,disp-sysreg\n");

	dsi->dpu1_sysreg = syscon_regmap_lookup_by_phandle_optional(np, "samsung,dpu1-sysreg");
	if (IS_ERR(dsi->dpu1_sysreg))
		return PTR_ERR(dsi->dpu1_sysreg);

	of_property_read_u32(np, "samsung,cmd-underrun-lp-ref", &dsi->cmd_underrun_lp_ref);

	mutex_init(&dsi->cmd_lock);
	init_completion(&dsi->ph_empty);
	init_completion(&dsi->rx_done);

	return 0;
}

/*
 * The boot loader may leave the link running (panel on, PLL locked). The vendor
 * driver then takes a runtime-PM reference in dsim_reg_ready() that it keeps
 * through boot (dsim_drv.c:1046) and skips the link init; do the same: power
 * the DSIM up by hand before runtime PM is enabled, and if LINK_STATUS3
 * PLL_STABLE is set, keep it active with a reference that the first link init
 * drops.
 */
static void samsung_dsim_handover_exynos8895(struct samsung_dsim *dsi)
{
	if (samsung_dsim_resume(dsi->dev))
		return;

	if (dsim8895_read(dsi, 0x14) & BIT(0)) {
		dev_info(dsi->dev, "S-Boot left DSIM PLL running, holding pd_dpu1\n");
		dsi->handover = true;
		pm_runtime_set_active(dsi->dev);
		pm_runtime_get_noresume(dsi->dev);
		return;
	}

	samsung_dsim_suspend(dsi->dev);
}

int samsung_dsim_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct samsung_dsim *dsi;
	int ret;

	dsi = devm_drm_bridge_alloc(dev, struct samsung_dsim, bridge, &samsung_dsim_bridge_funcs);
	if (IS_ERR(dsi))
		return PTR_ERR(dsi);

	init_completion(&dsi->completed);
	spin_lock_init(&dsi->transfer_lock);
	INIT_LIST_HEAD(&dsi->transfer_list);

	dsi->dsi_host.ops = &samsung_dsim_ops;
	dsi->dsi_host.dev = dev;

	dsi->dev = dev;
	dsi->plat_data = of_device_get_match_data(dev);
	dsi->driver_data = samsung_dsim_types[dsi->plat_data->hw_type];

	dsi->supplies[0].supply = "vddcore";
	dsi->supplies[1].supply = "vddio";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(dsi->supplies),
				      dsi->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = devm_clk_bulk_get(dev, dsi->driver_data->num_clks,
				dsi->driver_data->clk_data);
	if (ret) {
		dev_err(dev, "failed to get clocks in bulk (%d)\n", ret);
		return ret;
	}

	dsi->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsi->reg_base))
		return PTR_ERR(dsi->reg_base);

	dsi->phy = devm_phy_optional_get(dev, "dsim");
	if (IS_ERR(dsi->phy)) {
		dev_info(dev, "failed to get dsim phy\n");
		return PTR_ERR(dsi->phy);
	}

	dsi->irq = platform_get_irq(pdev, 0);
	if (dsi->irq < 0)
		return dsi->irq;

	ret = devm_request_threaded_irq(dev, dsi->irq, NULL,
					samsung_dsim_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), dsi);
	if (ret) {
		dev_err(dev, "failed to request dsi irq\n");
		return ret;
	}

	ret = samsung_dsim_parse_dt(dsi);
	if (ret)
		return ret;

	if (dsi->driver_data->has_8895_link) {
		ret = samsung_dsim_parse_dt_exynos8895(dsi);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, dsi);

	if (dsi->driver_data->has_8895_link) {
		/* DEBUG: greatlte heartbeat dpuc (greatlte-hb-l5.c) */
		greatlte_l5_set(GL5_ON, dsim_regs, dsi->reg_base);
		greatlte_l5_set(GL5_ON, dsim_state, &dsi->state);
		greatlte_l5_set(GL5_ON, dsim_dev, dev);
	}

	if (dsi->driver_data->handover_hold)
		samsung_dsim_handover_exynos8895(dsi);

	pm_runtime_enable(dev);

	dsi->bridge.of_node = dev->of_node;
	dsi->bridge.type = DRM_MODE_CONNECTOR_DSI;

	/* DE_LOW: i.MX8M Mini/Nano LCDIF-DSIM glue logic inverts HS/VS/DE */
	if (dsi->plat_data->hw_type == DSIM_TYPE_IMX8MM)
		dsi->bridge.timings = &samsung_dsim_bridge_timings_de_low;
	else
		dsi->bridge.timings = &samsung_dsim_bridge_timings_de_high;

	if (dsi->plat_data->host_ops && dsi->plat_data->host_ops->register_host) {
		ret = dsi->plat_data->host_ops->register_host(dsi);
		if (ret)
			goto err_disable_runtime;
	}

	return 0;

err_disable_runtime:
	pm_runtime_disable(dev);

	return ret;
}
EXPORT_SYMBOL_GPL(samsung_dsim_probe);

void samsung_dsim_remove(struct platform_device *pdev)
{
	struct samsung_dsim *dsi = platform_get_drvdata(pdev);

	if (dsi->driver_data->has_8895_link) {
		greatlte_l5_set(GL5_ON, dsim_dev, NULL);
		greatlte_l5_set(GL5_ON, dsim_state, NULL);
		greatlte_l5_set(GL5_ON, dsim_regs, NULL);
	}
	pm_runtime_disable(&pdev->dev);

	if (dsi->plat_data->host_ops && dsi->plat_data->host_ops->unregister_host)
		dsi->plat_data->host_ops->unregister_host(dsi);
}
EXPORT_SYMBOL_GPL(samsung_dsim_remove);

static int samsung_dsim_suspend(struct device *dev)
{
	struct samsung_dsim *dsi = dev_get_drvdata(dev);
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	int ret;

	usleep_range(10000, 20000);

	if (dsi->state & DSIM_STATE_INITIALIZED) {
		dsi->state &= ~DSIM_STATE_INITIALIZED;

		if (driver_data->has_8895_link)
			samsung_dsim_stop_exynos8895(dsi);
		else
			samsung_dsim_disable_clock(dsi);

		samsung_dsim_disable_irq(dsi);
	}

	dsi->state &= ~DSIM_STATE_CMD_LPM;

	phy_power_off(dsi->phy);

	clk_bulk_disable_unprepare(driver_data->num_clks, driver_data->clk_data);

	ret = regulator_bulk_disable(ARRAY_SIZE(dsi->supplies), dsi->supplies);
	if (ret < 0)
		dev_err(dsi->dev, "cannot disable regulators %d\n", ret);

	return 0;
}

static int samsung_dsim_resume(struct device *dev)
{
	struct samsung_dsim *dsi = dev_get_drvdata(dev);
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(dsi->supplies), dsi->supplies);
	if (ret < 0) {
		dev_err(dsi->dev, "cannot enable regulators %d\n", ret);
		return ret;
	}

	ret = clk_bulk_prepare_enable(driver_data->num_clks, driver_data->clk_data);
	if (ret < 0)
		goto err_clk;

	ret = phy_power_on(dsi->phy);
	if (ret < 0) {
		dev_err(dsi->dev, "cannot enable phy %d\n", ret);
		goto err_clk;
	}

	/*
	 * Exynos 8895: the DSIM is the only display device in pd_dpu1, so it
	 * restores SYSREG_DPU1 BUS_COMPONENT_DRCG_EN, which the vendor PMUCAL
	 * restores after every pd_dpu1 power-on (0xffffffff at boot).
	 */
	if (driver_data->has_8895_link && dsi->dpu1_sysreg)
		regmap_write(dsi->dpu1_sysreg, DSIM8895_SYSREG_DRCG_EN, 0xffffffff);

	return 0;

err_clk:
	clk_bulk_disable_unprepare(driver_data->num_clks, driver_data->clk_data);
	regulator_bulk_disable(ARRAY_SIZE(dsi->supplies), dsi->supplies);

	return ret;
}

const struct dev_pm_ops samsung_dsim_pm_ops = {
	RUNTIME_PM_OPS(samsung_dsim_suspend, samsung_dsim_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};
EXPORT_SYMBOL_GPL(samsung_dsim_pm_ops);

static const struct samsung_dsim_plat_data samsung_dsim_imx8mm_pdata = {
	.hw_type = DSIM_TYPE_IMX8MM,
	.host_ops = &generic_dsim_host_ops,
};

static const struct samsung_dsim_plat_data samsung_dsim_imx8mp_pdata = {
	.hw_type = DSIM_TYPE_IMX8MP,
	.host_ops = &generic_dsim_host_ops,
};

static const struct of_device_id samsung_dsim_of_match[] = {
	{
		.compatible = "fsl,imx8mm-mipi-dsim",
		.data = &samsung_dsim_imx8mm_pdata,
	},
	{
		.compatible = "fsl,imx8mp-mipi-dsim",
		.data = &samsung_dsim_imx8mp_pdata,
	},
	{ /* sentinel. */ }
};
MODULE_DEVICE_TABLE(of, samsung_dsim_of_match);

static struct platform_driver samsung_dsim_driver = {
	.probe = samsung_dsim_probe,
	.remove = samsung_dsim_remove,
	.driver = {
		   .name = "samsung-dsim",
		   .pm = pm_ptr(&samsung_dsim_pm_ops),
		   .of_match_table = samsung_dsim_of_match,
	},
};

module_platform_driver(samsung_dsim_driver);

MODULE_AUTHOR("Jagan Teki <jagan@amarulasolutions.com>");
MODULE_DESCRIPTION("Samsung MIPI DSIM controller bridge");
MODULE_LICENSE("GPL");
