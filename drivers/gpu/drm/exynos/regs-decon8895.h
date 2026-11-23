/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos8895 DECON0, DSC and SYSREG_DPU0 registers
 *
 * From the Samsung vendor kernel, Copyright (c) 2014-2016 Samsung
 * Electronics, with every name prefixed D8895_.
 */

#ifndef _REGS_DECON8895_H
#define _REGS_DECON8895_H

/* SYSREG_DPU0 */
#define D8895_DISP_LPMUX_CFG					0x0000
/* _v : [0,1] */
#define D8895_SEL_DPHY2DSIM_MASK(_v)		(0x3 << (12 + (_v) * 4))
/* _v : [0,2] */
#define D8895_SEL_DSIM2DPHY_MASK(_v)		(0x3 << (0 + (_v) * 4))

#define D8895_DISP_DLMUX_CFG					0x0004
/* _v : [0,1] */
#define D8895_SEL_DISPIF2DSIM_MASK(_v)		(0x3 << (12 + (_v) * 4))
/* _x(dsim_if_id): [0,2] (0,1: for decon0, 2: for decon1), _y(dsim_id): [0,1] */
#define D8895_SEL_DISPIF2DSIM(_x, _y)		((_x) << (12 + (_y) * 4))

/* _v : [0,2] */
#define D8895_SEL_DSIM2DISPIF_MASK(_v)		(0x7 << (0 + (_v) * 4))
/* _x(dsim_if_id): [0,2] (0,1: for decon0, 2: for decon1), _y(dsim_id): [0,1] */
#define D8895_SEL_DSIM2DISPIF(_y, _x)		(((_y) + 1) << (0 + (_x) * 4))

#define D8895_DISP_DPU_MIPI_PHY_CON				0x0008
/* _v : [0,1] */
#define D8895_SEL_RESET_DPHY_MASK(_v)		(0x1 << (4 + (_v)))
#define D8895_M_RESETN_M1S2S2_MODULE_MASK		(0x1 << 2)
#define D8895_M_RESETN_M4S4_MODULE_MASK		(0x1 << 1)
#define D8895_M_RESETN_M4S4_TOP_MASK		(0x1 << 0)

#define D8895_DISP_DPU_TE_SEL				0x000C
/* _v : [0,2] */
#define D8895_SEL_TE_MASK(_n)			(0x1 << (_n))
/* _v : [0,1](0=speedy, 1=gpio),  _n (TE id) : [0,2] */
#define D8895_SEL_TE_PATH(_n, _v)			((_v) << (_n))
#define D8895_TE_FROM_SPEEDY				0
#define D8895_TE_FROM_GPIO				1
#define D8895_GLOBAL_CONTROL					0x0000
#define D8895_GLOBAL_CONTROL_SRESET				(1 << 28)
#define D8895_GLOBAL_CONTROL_OPERATION_MODE_F		(1 << 8)
#define D8895_GLOBAL_CONTROL_OPERATION_MODE_RGBIF_F	(0 << 8)
#define D8895_GLOBAL_CONTROL_OPERATION_MODE_I80IF_F	(1 << 8)
#define D8895_GLOBAL_CONTROL_IDLE_STATUS			(1 << 5)
#define D8895_GLOBAL_CONTROL_RUN_STATUS			(1 << 4)
#define D8895_GLOBAL_CONTROL_DECON_EN			(1 << 1)
#define D8895_GLOBAL_CONTROL_DECON_EN_F			(1 << 0)

#define D8895_RESOURCE_OCCUPANCY_INFO_0			0x0010
#define D8895_RESOURCE_OCCUPANCY_INFO_1			0x0014
#define D8895_RESOURCE_OCCUPANCY_INFO_2			0x0018

#define D8895_SRAM_SHARE_ENABLE				0x0030
#define D8895_FLOATING_SRAM_SHARE_ENABLE_F		(1 << 24)
#define D8895_FF2_SRAM_SHARE_ENABLE_F		(1 << 20)
#define D8895_FF1_SRAM_SHARE_ENABLE_F		(1 << 16)
#define D8895_FF0_SRAM_SHARE_ENABLE_F		(1 << 12)
#define D8895_ALL_SRAM_SHARE_ENABLE			(0x1111 << 12)
#define D8895_ALL_SRAM_SHARE_DISABLE		(0x0000 << 12)

#define D8895_INTERRUPT_ENABLE				0x0040
#define D8895_DPU_MDNIE_DIMMING_END_INT_EN		(1 << 25)
#define D8895_DPU_MDNIE_DIMMING_START_INT_EN	(1 << 24)
#define D8895_DPU_DQE_DIMMING_END_INT_EN		(1 << 21)
#define D8895_DPU_DQE_DIMMING_START_INT_EN		(1 << 20)
#define D8895_DPU_FRAME_DONE_INT_EN			(1 << 13)
#define D8895_DPU_FRAME_START_INT_EN		(1 << 12)
#define D8895_DPU_FIFO_SEL_FIFO0			(0 << 9)
#define D8895_DPU_FIFO_SEL_FIFO1			(1 << 9)
#define D8895_DPU_UNDER_FLOW_INT_EN			(1 << 8)
#define D8895_DPU_EXTRA_INT_EN			(1 << 4)
#define D8895_DPU_INT_EN				(1 << 0)
#define D8895_INTERRUPT_ENABLE_MASK			0x3303711

#define D8895_EXTRA_INTERRUPT_ENABLE			0x0044
#define D8895_DPU_RESOURCE_CONFLICT_INT_EN		(1 << 8)
#define D8895_DPU_TIME_OUT_INT_EN			(1 << 4)
#define D8895_DPU_ERROR_INT_EN			(1 << 0)

#define D8895_TIME_OUT_VALUE					0x0048

#define D8895_INTERRUPT_PENDING				0x004C
#define D8895_DPU_MDNIE_DIMMING_END_INT_PEND	(1 << 25)
#define D8895_DPU_MDNIE_DIMMING_START_INT_PEND	(1 << 24)
#define D8895_DPU_DQE_DIMMING_END_INT_PEND		(1 << 21)
#define D8895_DPU_DQE_DIMMING_START_INT_PEND	(1 << 20)
#define D8895_DPU_FRAME_DONE_INT_PEND		(1 << 13)
#define D8895_DPU_FRAME_START_INT_PEND		(1 << 12)
#define D8895_DPU_UNDER_FLOW_INT_PEND		(1 << 8)
#define D8895_DPU_EXTRA_INT_PEND			(1 << 4)

#define D8895_EXTRA_INTERRUPT_PENDING			0x0050
#define D8895_DPU_RESOURCE_CONFLICT_INT_PEND	(1 << 8)
#define D8895_DPU_TIME_OUT_INT_PEND			(1 << 4)
#define D8895_DPU_ERROR_INT_PEND			(1 << 0)

#define D8895_SHADOW_REG_UPDATE_REQ				0x0060
#define D8895_SHADOW_REG_UPDATE_REQ_GLOBAL		(1 << 31)
#define D8895_SHADOW_REG_UPDATE_REQ_DQE		(1 << 28)
#define D8895_SHADOW_REG_UPDATE_REQ_MDNIE		(1 << 24)
#define D8895_SHADOW_REG_UPDATE_REQ_WIN(_win)	(1 << (_win))
#define D8895_SHADOW_REG_UPDATE_REQ_FOR_DECON0	(0x3f)
#define D8895_SHADOW_REG_UPDATE_REQ_FOR_DECON2	(0x1f)

#define D8895_HW_SW_TRIG_CONTROL				0x0070
#define D8895_HW_TRIG_SEL(_v)			((_v) << 24)
#define D8895_HW_TRIG_SEL_MASK			(0x3 << 24)
#define D8895_HW_TRIG_SEL_FROM_USB			(3 << 24)
#define D8895_HW_TRIG_SEL_FROM_DDI2			(2 << 24)
#define D8895_HW_TRIG_SEL_FROM_DDI1			(1 << 24)
#define D8895_HW_TRIG_SEL_FROM_DDI0			(0 << 24)
#define D8895_HW_TRIG_SKIP(_v)			((_v) << 16)
#define D8895_HW_TRIG_SKIP_MASK			(0xff << 16)
#define D8895_TRIG_AUTO_MASK_EN			(1 << 12)
#define D8895_SW_TRIG_EN				(1 << 8)
#define D8895_HW_TRIG_EDGE_POLARITY			(1 << 7)
#define D8895_HW_TRIG_MASK_DECON			(1 << 4)
#define D8895_HW_SW_TRIG_TIMER_CLEAR		(1 << 3)
#define D8895_HW_SW_TRIG_TIMER_EN			(1 << 2)
#define D8895_HW_TRIG_EN				(1 << 0)

#define D8895_HW_SW_TRIG_TIMER				0x0074

#define D8895_CLOCK_CONTROL_0				0x00F0
/* [24] QACTIVE_VALUE = 0
 * 0: QACTIVE is dynamically changed by DECON h/w,
 * 1: QACTIVE is stuck to 1'b1
 * [22][20][16][12]+[8][0] AUTO_CG_EN_xxx
 */
#define D8895_CLOCK_CONTROL_0_F_MASK		(0x00511101)
#define D8895_CLOCK_CONTROL_0_S_MASK		(0x00511000)
#define D8895_CLOCK_CONTROL_0_T_MASK		(0x00511000)

#define D8895_SPLITTER_SIZE_CONTROL_0			0x0100
#define D8895_SPLITTER_HEIGHT_F(_v)			((_v) << 16)
#define D8895_SPLITTER_HEIGHT_MASK			(0x3fff << 16)
#define D8895_SPLITTER_HEIGHT_GET(_v)		(((_v) >> 16) & 0x3fff)
#define D8895_SPLITTER_WIDTH_F(_v)			((_v) << 0)
#define D8895_SPLITTER_WIDTH_MASK			(0x3fff << 0)
#define D8895_SPLITTER_WIDTH_GET(_v)		(((_v) >> 0) & 0x3fff)

#define D8895_SPLITTER_SIZE_CONTROL_1			0x0104

#define D8895_SPLITTER_SPLIT_IDX_CONTROL			0x0108
#define D8895_SPLITTER_SPLIT_IDX_F(_v)		((_v) << 0)
#define D8895_SPLITTER_SPLIT_IDX_MASK		(0x3fff << 0)

#define D8895_SPLITTER_OVERLAP_CONTROL			0x010C
#define D8895_SPLITTER_OVERLAP_F(_v)		((_v) << 0)
#define D8895_SPLITTER_OVERLAP_MASK			(0x3fff << 0)

#define D8895_FRAME_FIFO_0_MONITOR_CONTROL			0x0114
#define D8895_FRAME_FIFO_1_MONITOR_CONTROL			0x0118

#define D8895_FRAME_FIFO_0_SIZE_CONTROL_0			0x0120
#define D8895_FRAME_FIFO_HEIGHT_F(_v)		((_v) << 16)
#define D8895_FRAME_FIFO_HEIGHT_MASK		(0x3fff << 16)
#define D8895_FRAME_FIFO_HEIGHT_GET(_v)		(((_v) >> 16) & 0x3fff)
#define D8895_FRAME_FIFO_WIDTH_F(_v)		((_v) << 0)
#define D8895_FRAME_FIFO_WIDTH_MASK			(0x3fff << 0)
#define D8895_FRAME_FIFO_WIDTH_GET(_v)		(((_v) >> 0) & 0x3fff)

#define D8895_FRAME_FIFO_0_SIZE_CONTROL_1			0x0124
/* same field with D8895_FRAME_FIFO_0_SIZE_CONTROL_0 */
#define D8895_FRAME_FIFO_1_SIZE_CONTROL_0			0x0128
#define D8895_FRAME_FIFO_1_SIZE_CONTROL_1			0x012C

#define D8895_FRAME_FIFO_TH_CONTROL_0			0x0130
#define D8895_FRAME_FIFO_1_TH_F(_v)			((_v) << 16)
#define D8895_FRAME_FIFO_1_TH_MASK			(0xffff << 16)
#define D8895_FRAME_FIFO_1_TH_GET(_v)		(((_v) >> 16) & 0xffff)
#define D8895_FRAME_FIFO_0_TH_F(_v)			((_v) << 0)
#define D8895_FRAME_FIFO_0_TH_MASK			(0xffff << 0)
#define D8895_FRAME_FIFO_0_TH_GET(_v)		(((_v) >> 0) & 0xffff)

#define D8895_FRAME_FIFO_0_LEVEL				0x0134
#define D8895_FRAME_FIFO_1_LEVEL				0x0138

/*
 * DECON2 Only
 */
#define D8895_DPU_DISPIF_VSTATUS_INT_SEL_MASK		(3 << 17)	/* D8895_INTERRUPT_ENABLE */
#define D8895_DPU_DISPIF_VSTATUS_INT_SEL(_v)		((_v) << 17)
#define D8895_DPU_DISPIF_VSTATUS_INT_EN		(1 << 16)
#define D8895_DPU_FIFO_SEL_DISPIF			(2 << 9)
#define D8895_DPU_FIFO_SEL_MASK			(3 << 9)
#define D8895_DPU_VERTICAL_VSYNC_START		0x01
#define D8895_INTERRUPT_ENABLE_MASK_DECON2		0x00073711

#define D8895_DPU_DISPIF_VSTATUS_INT_PEND		(1 << 16)

#define D8895_DISPIF_CONTROL				0x0140
#define D8895_DISPIF_CLOCK_UNDERRUN_SCHEME_F(_v)	((_v) << 12)
#define D8895_DISPIF_CLOCK_UNDERRUN_SCHEME_MASK	(0x3 << 12)
#define D8895_DISPIF_RGB_ORDER_F(_v)		((_v) << 8)
#define D8895_DISPIF_RGB_ORDER_MASK			(0x7 << 8)
#define D8895_DISPIF_CLOCK_FREE_RUN_EN		(1 << 4)
#define D8895_DISPIF_VSYNC_ACTIVE_VALUE(_v)		((_v) << 2)
#define D8895_DISPIF_VSYNC_ACTIVE_VALUE_MASK		(1 << 2)
#define D8895_DISPIF_HSYNC_ACTIVE_VALUE(_v)		((_v) << 1)
#define D8895_DISPIF_HSYNC_ACTIVE_VALUE_MASK		(1 << 1)
#define D8895_DISPIF_VDEN_ACTIVE_VALUE		(1 << 0)

#define D8895_DISPIF_TIMING_CONTROL_0			0x0150
#define D8895_DISPIF_VBPD_F(_v)			((_v) << 16)
#define D8895_DISPIF_VBPD_MASK			(0xffff << 16)
#define D8895_DISPIF_VBPD_GET(_v)			(((_v) >> 16) & 0xffff)
#define D8895_DISPIF_VFPD_F(_v)			((_v) << 0)
#define D8895_DISPIF_VFPD_MASK			(0xffff << 0)
#define D8895_DISPIF_VFPD_GET(_v)			(((_v) >> 0) & 0xffff)

#define D8895_DISPIF_TIMING_CONTROL_1			0x0154
#define D8895_DISPIF_VSPD_F(_v)			((_v) << 0)
#define D8895_DISPIF_VSPD_MASK			(0xffff << 0)
#define D8895_DISPIF_VSPD_GET(_v)			(((_v) >> 0) & 0xffff)

#define D8895_DISPIF_TIMING_CONTROL_2			0x0158
#define D8895_DISPIF_HBPD_F(_v)			((_v) << 16)
#define D8895_DISPIF_HBPD_MASK			(0xffff << 16)
#define D8895_DISPIF_HBPD_GET(_v)			(((_v) >> 16) & 0xffff)
#define D8895_DISPIF_HFPD_F(_v)			((_v) << 0)
#define D8895_DISPIF_HFPD_MASK			(0xffff << 0)
#define D8895_DISPIF_HFPD_GET(_v)			(((_v) >> 0) & 0xffff)

#define D8895_DISPIF_TIMING_CONTROL_3			0x015C
#define D8895_DISPIF_HSPD_F(_v)			((_v) << 0)
#define D8895_DISPIF_HSPD_MASK			(0xffff << 0)
#define D8895_DISPIF_HSPD_GET(_v)			(((_v) >> 0) & 0xffff)

#define D8895_DISPIF_SIZE_CONTROL_0				0x0160
#define D8895_DISPIF_HEIGHT_F(_v)			((_v) << 16)
#define D8895_DISPIF_HEIGHT_MASK			(0xffff << 16)
#define D8895_DISPIF_HEIGHT_GET(_v)			(((_v) >> 16) & 0x3fff)
#define D8895_DISPIF_WIDTH_F(_v)			((_v) << 0)
#define D8895_DISPIF_WIDTH_MASK			(0xffff << 0)
#define D8895_DISPIF_WIDTH_GET(_v)			(((_v) >> 0) & 0x3fff)

#define D8895_DISPIF_SIZE_CONTROL_1				0x0164
#define D8895_DISPIF_PIXEL_COUNT_F(_v)		((_v) << 0)

/*************************************************************/

#define D8895_HYSTERESIS_CONTROL				0x0180
#define D8895_HYSTERESIS_ENABLE_F			(1 << 0)

#define D8895_HYSTERESIS_THRESHOLD				0x0184
#define D8895_HYSTERESIS_HIGH_TH_F(_v)		((_v) << 16)
#define D8895_HYSTERESIS_HIGH_TH_MASK		(0xffff << 16)
#define D8895_HYSTERESIS_HIGH_TH_GET(_v)		(((_v) >> 16) & 0xffff)
#define D8895_HYSTERESIS_LOW_TH_F(_v)		((_v) << 0)
#define D8895_HYSTERESIS_LOW_TH_MASK		(0xffff << 0)
#define D8895_HYSTERESIS_LOW_TH_GET(_v)		(((_v) >> 0) & 0xffff)

#define D8895_FORMATTER_CONTROL				0x0190
#define D8895_FORMATTER_PIXEL0123_ORDER_SWAP_F	(1 << 16)
#define D8895_FORMATTER_PIXEL23_ORDER_SWAP_F	(1 << 12)
#define D8895_FORMATTER_PIXEL01_ORDER_SWAP_F	(1 << 8)
#define D8895_FORMATTER_PIXEL_ORDER_SWAP(_a, _b, _c)	\
	(((_a) << 16) | ((_b) << 12) | ((_c) << 8))
#define D8895_FORMATTER_PIXEL_ORDER_SWAP_MASK	((1 << 16) | (1 << 12) | (1 << 8))
#define D8895_FORMATTER_OUT_RGB_ORDER_F(_v)	((_v) << 4)
#define D8895_FORMATTER_OUT_RGB_ORDER_MASK		(0x7 << 4)

#define D8895_FORMATTER0_SIZE_CONTROL_0			0x01A0
#define D8895_FORMATTER_HEIGHT_F(_v)		((_v) << 16)
#define D8895_FORMATTER_HEIGHT_MASK		(0x3fff << 16)
#define D8895_FORMATTER_HEIGHT_GET(_v)		(((_v) >> 16) & 0x3fff)
#define D8895_FORMATTER_WIDTH_F(_v)		((_v) << 0)
#define D8895_FORMATTER_WIDTH_MASK			(0x3fff << 0)
#define D8895_FORMATTER_WIDTH_GET(_v)		(((_v) >> 0) & 0x3fff)

#define D8895_FORMATTER0_SIZE_CONTROL_1			0x01A4
/* same field with D8895_FORMATTER0_SIZE_CONTROL_0 */
#define D8895_FORMATTER1_SIZE_CONTROL_0			0x01B0
#define D8895_FORMATTER1_SIZE_CONTROL_1			0x01B4

#define D8895_BLENDER_BG_IMAGE_SIZE_0			0x0200
#define D8895_BLENDER_BG_HEIGHT_F(_v)		((_v) << 16)
#define D8895_BLENDER_BG_HEIGHT_MASK		(0x3fff << 16)
#define D8895_BLENDER_BG_HEIGHT_GET(_v)		(((_v) >> 16) & 0x3fff)
#define D8895_BLENDER_BG_WIDTH_F(_v)		((_v) << 0)
#define D8895_BLENDER_BG_WIDTH_MASK			(0x3fff << 0)
#define D8895_BLENDER_BG_WIDTH_GET(_v)		(((_v) >> 0) & 0x3fff)

#define D8895_BLENDER_BG_IMAGE_SIZE_1			0x0204

#define D8895_BLENDER_BG_IMAGE_COLOR_0			0x0208
#define D8895_BLENDER_BG_A_F(_v)			((_v) << 16)
#define D8895_BLENDER_BG_A_MASK			(0xff << 16)
#define D8895_BLENDER_BG_A_GET(_v)			(((_v) >> 16) & 0xff)
#define D8895_BLENDER_BG_R_F(_v)			((_v) << 0)
#define D8895_BLENDER_BG_R_MASK			(0xff << 0)
#define D8895_BLENDER_BG_R_GET(_v)			(((_v) >> 0) & 0xff)

#define D8895_BLENDER_BG_IMAGE_COLOR_1			0x020C
#define D8895_BLENDER_BG_G_F(_v)			((_v) << 16)
#define D8895_BLENDER_BG_G_MASK			(0xff << 16)
#define D8895_BLENDER_BG_G_GET(_v)			(((_v) >> 16) & 0xff)
#define D8895_BLENDER_BG_B_F(_v)			((_v) << 0)
#define D8895_BLENDER_BG_B_MASK			(0xff << 0)
#define D8895_BLENDER_BG_B_GET(_v)			(((_v) >> 0) & 0xff)

#define D8895_LRMERGER_MODE_CONTROL				0x0210
#define D8895_LRM23_MODE_F(_v)			((_v) << 16)
#define D8895_LRM23_MODE_MASK			(0x7 << 16)
#define D8895_LRM01_MODE_F(_v)			((_v) << 0)
#define D8895_LRM01_MODE_MASK			(0x7 << 0)

#define D8895_DATA_PATH_CONTROL_0				0x0214
#define D8895_WIN_MAPCOLOR_EN_F(_win)		(1 << (4*_win + 1))
#define D8895_WIN_EN_F(_win)				(1 << (4*_win + 0))

#define D8895_DATA_PATH_CONTROL_1				0x0218
/* WIN5_CHMAP_F [22:20]=5 :  dedicated to channel5 */
#define D8895_WIN_CHMAP_F(_win, _ch)		(((_ch) & 0x7) << (4*_win))
#define D8895_WIN_CHMAP_MASK(_win)			(0x7 << (4*_win))

#define D8895_DATA_PATH_CONTROL_2				0x0230
#define D8895_DQE_LPD_EXIT_CTRL			(1 << 24)
#define D8895_HSC_PATH_F(_v)				((_v) << 20)
#define D8895_HSC_PATH_MASK				(0x7 << 20)
#define D8895_APS_PATH_F(_v)				((_v) << 16)
#define D8895_APS_PATH_MASK				(0x7 << 16)
#define D8895_ENHANCE_LOGIC_PATH_F(_v)		((_v) << 12)
#define D8895_ENHANCE_LOGIC_PATH_MASK		(0x7 << 12)
#define D8895_ENHANCE_LOGIC_PATH_GET(_v)		(((_v) >> 12) & 0x7)
#define D8895_COMP_LINKIF_WB_PATH_F(_v)		((_v) << 0)
#define D8895_COMP_LINKIF_WB_PATH_MASK		(0x1ff << 0)
#define D8895_COMP_LINKIF_WB_PATH_GET(_v)		(((_v) >> 0) & 0x1ff)

#define D8895_CRC_DATA					0x0280
#define D8895_CRC_DATA_WCLK1_GET(_v)		(((_v) >> 16) & 0xffff)
#define D8895_CRC_DATA_WCLK0_GET(_v)		(((_v) >> 0) & 0xffff)

#define D8895_CRC_CONTROL					0x0288
#define D8895_CRC_BITS_SEL(_v)			((_v) << 16)
#define D8895_CRC_BITS_SEL_MASK			(0xf << 16)
#define D8895_CRC_START				(1 << 0)

#define D8895_FRAME_ID					0x02A0

/* hidden for customer */
#define D8895_WAIT_CYCLE_AFTER_SFR_UPDATE			0x02A4

#define D8895_DITHER_CONTROL					0x0300
#define D8895_DITHER_CONTROL_F(_v)			((_v) << 0)
#define D8895_DITHER_CONTROL_MASK			(0x3 << 0)

#define D8895_WIN_CONTROL_0(_win)			(0x1000 + ((_win) * 0x30))
#define D8895_WIN_ALPHA1_F(_v)			(((_v) & 0xFF) << 24)
#define D8895_WIN_ALPHA1_MASK			(0xFF << 24)
#define D8895_WIN_ALPHA0_F(_v)			(((_v) & 0xFF) << 16)
#define D8895_WIN_ALPHA0_MASK			(0xFF << 16)
#define D8895_WIN_ALPHA_GET(_v, _n)		(((_v) >> (16 + 8 * (_n))) & 0xFF)
#define D8895_WIN_FUNC_F(_v)				(((_v) & 0xF) << 8)
#define D8895_WIN_FUNC_MASK				(0xF << 8)
#define D8895_WIN_FUNC_GET(_v)			(((_v) >> 8) & 0xf)
#define D8895_WIN_SRESET				(1 << 4)
#define D8895_WIN_ALPHA_MULT_SRC_SEL_F(_v)		(((_v) & 0x3) << 0)
#define D8895_WIN_ALPHA_MULT_SRC_SEL_MASK		(0x3 << 0)

#define D8895_WIN_CONTROL_1(_win)			(0x1004 + ((_win) * 0x30))
#define D8895_WIN_FG_ALPHA_D_SEL_F(_v)		(((_v) & 0xF) << 24)
#define D8895_WIN_FG_ALPHA_D_SEL_MASK		(0xF << 24)
#define D8895_WIN_BG_ALPHA_D_SEL_F(_v)		(((_v) & 0xF) << 16)
#define D8895_WIN_BG_ALPHA_D_SEL_MASK		(0xF << 16)
#define D8895_WIN_FG_ALPHA_A_SEL_F(_v)		(((_v) & 0xF) << 8)
#define D8895_WIN_FG_ALPHA_A_SEL_MASK		(0xF << 8)
#define D8895_WIN_BG_ALPHA_A_SEL_F(_v)		(((_v) & 0xF) << 0)
#define D8895_WIN_BG_ALPHA_A_SEL_MASK		(0xF << 0)

#define D8895_WIN_START_POSITION(_win)		(0x1008 + ((_win) * 0x30))
#define D8895_WIN_STRPTR_Y_F(_v)			(((_v) & 0x3FFF) << 16)
#define D8895_WIN_STRPTR_X_F(_v)			(((_v) & 0x3FFF) << 0)

#define D8895_WIN_END_POSITION(_win)		(0x100C + ((_win) * 0x30))
#define D8895_WIN_ENDPTR_Y_F(_v)			(((_v) & 0x3FFF) << 16)
#define D8895_WIN_ENDPTR_X_F(_v)			(((_v) & 0x3FFF) << 0)

#define D8895_WIN_COLORMAP_0(_win)			(0x1010 + ((_win) * 0x30))
#define D8895_WIN_MAPCOLOR_A_F(_v)			((_v) << 16)
#define D8895_WIN_MAPCOLOR_A_MASK			(0xff << 16)
#define D8895_WIN_MAPCOLOR_R_F(_v)			((_v) << 0)
#define D8895_WIN_MAPCOLOR_R_MASK			(0xff << 0)

#define D8895_WIN_COLORMAP_1(_win)			(0x1014 + ((_win) * 0x30))
#define D8895_WIN_MAPCOLOR_G_F(_v)			((_v) << 16)
#define D8895_WIN_MAPCOLOR_G_MASK			(0xff << 16)
#define D8895_WIN_MAPCOLOR_B_F(_v)			((_v) << 0)
#define D8895_WIN_MAPCOLOR_B_MASK			(0xff << 0)

#define D8895_WIN_START_TIME_CONTROL(_win)		(0x1018 + ((_win) * 0x30))
#define D8895_WIN_START_TIME_CONTROL_F(_v)		((_v) << 0)
#define D8895_WIN_START_TIME_CONTROL_MASK		(0x3fff << 0)

#define D8895_WIN_PIXEL_COUNT(_win)			(0x101C + ((_win) * 0x30))

#define D8895_DSC0_OFFSET					0x4000
#define D8895_DSC1_OFFSET					0x5000
#define D8895_DSC_NUM_EXTRA_MUX_BIT				246
#define D8895_DSC_MIN_SLICE_SIZE				15000
#define D8895_DSC_INIT_TRANSMIT_DELAY				0x200
#define D8895_DSC_INIT_SCALE_VALUE				0x20
#define D8895_DSC_BIT_PER_PIXEL				0x80
#define D8895_DSC_FIRST_LINE_BPG_OFFSET			0xC
#define D8895_DSC_INIT_OFFSET					0x1800
#define D8895_DSC_RC_MODE_SIZE				0x2000

#define D8895_DSC_CONTROL0					0x0000
#define D8895_DSC_SW_RESET				(0x1 << 28)
#define D8895_DSC_DCG_EN_REF(_v)			((_v) << 19)
#define D8895_DSC_DCG_EN_SSM(_v)			((_v) << 18)
#define D8895_DSC_DCG_EN_ICH(_v)			((_v) << 17)
#define D8895_DSC_DCG_EN_BYPASS(_v)			((_v) << 16)
#define D8895_DSC_DCG_EN_ALL_OFF			(0x0 << 16)
#define D8895_DSC_DCG_EN_ALL_MASK			(0xf << 16)
#define D8895_DSC_BIT_SWAP(_v)			((_v) << 10)
#define D8895_DSC_BYTE_SWAP(_v)			((_v) << 9)
#define D8895_DSC_WORD_SWAP(_v)			((_v) << 8)
#define D8895_DSC_SWAP(_b, _c, _w)	\
	(((_b) << 10) | ((_c) << 9) | ((_w) << 8))
#define D8895_DSC_SWAP_MASK				((1 << 10) | (1 << 9) | (1 << 8))
#define D8895_DSC_FLATNESS_DET_TH_MASK		(0xf << 4)
#define D8895_DSC_FLATNESS_DET_TH_F(_v)		((_v) << 4)
#define D8895_DSC_SLICE_MODE_CH_MASK		(0x1 << 3)
#define D8895_DSC_SLICE_MODE_CH_F(_v)		((_v) << 3)
#define D8895_DSC_BYPASS_MASK			(0x1 << 2)
#define D8895_DSC_BYPASS_F(_v)			((_v) << 2)
#define D8895_DSC_CG_EN_MASK				(0x1 << 1)
#define D8895_DSC_CG_EN_F(_v)			((_v) << 1)
#define D8895_DSC_DUAL_SLICE_EN_MASK		(0x1 << 0)
#define D8895_DSC_DUAL_SLICE_EN_F(_v)		((_v) << 0)

#define D8895_DSC_CONTROL1					0x0004
#define D8895_DSC_V_FRONT_F(_v)			((_v) << 16)
#define D8895_DSC_V_SYNC_F(_v)			((_v) << 8)
#define D8895_DSC_V_BACK_F(_v)			((_v) << 0)
#define D8895_DSC_V_BLANK_MASK			(0xFFFFFF)

#define D8895_DSC_CONTROL2					0x0008
#define D8895_DSC_H_FRONT_F(_v)			((_v) << 16)
#define D8895_DSC_H_SYNC_F(_v)			((_v) << 8)
#define D8895_DSC_H_BACK_F(_v)			((_v) << 0)
#define D8895_DSC_H_BLANK_MASK			(0xFFFFFF)

#define D8895_DSC_PPS00_03					0x0020

#define D8895_DSC_PPS04_07					0x0024
#define D8895_PPS04_COMP_CFG(_v)			(_v << 24)
#define D8895_PPS04_COMP_CFG_MASK			(0x3F << 24)
#define D8895_PPS05_BPP(_v)				(_v << 16)
#define D8895_PPS05_BPP_MASK				(0xFF << 16)
#define D8895_PPS06_07_PIC_HEIGHT_MASK		(0xFFFF << 0)
#define D8895_PPS06_07_PIC_HEIGHT(_v)		(_v << 0)

#define D8895_DSC_PPS08_11					0x0028
#define D8895_PPS08_09_PIC_WIDHT_MASK		(0xFFFF << 16)
#define D8895_PPS08_09_PIC_WIDHT(_v)		((_v) << 16)
#define D8895_PPS10_11_SLICE_HEIGHT_MASK		(0xFFFF << 0)
#define D8895_PPS10_11_SLICE_HEIGHT(_v)		(_v << 0)

#define D8895_DSC_PPS12_15					0x002C
#define D8895_PPS12_13_SLICE_WIDTH_MASK		(0xFFFF << 16)
#define D8895_PPS12_13_SLICE_WIDTH(_v)		((_v) << 16)
#define D8895_PPS14_15_CHUNK_SIZE_MASK		(0xFFFF << 0)
#define D8895_PPS14_15_CHUNK_SIZE(_v)		(_v << 0)

#define D8895_DSC_PPS16_19					0x0030
#define D8895_PPS16_17_INIT_XMIT_DELAY_MASK	(0x3FF << 16)
#define D8895_PPS16_17_INIT_XMIT_DELAY(_v)		((_v) << 16)
#define D8895_PPS18_19_INIT_DEC_DELAY_MASK		(0xFFFF << 0)
#define D8895_PPS18_19_INIT_DEC_DELAY(_v)		((_v) << 0)

#define D8895_DSC_PPS20_23					0x0034
#define D8895_PPS21_INIT_SCALE_VALUE_MASK		(0x3F << 16)
#define D8895_PPS21_INIT_SCALE_VALUE(_v)		((_v) << 16)
#define D8895_PPS22_23_SCALE_INC_INTERVAL_MASK	(0xFFFF << 0)
#define D8895_PPS22_23_SCALE_INC_INTERVAL(_v)	(_v << 0)

#define D8895_DSC_PPS24_27					0x0038
#define D8895_PPS24_25_SCALE_DEC_INTERVAL_MASK	(0xFFF << 16)
#define D8895_PPS24_25_SCALE_DEC_INTERVAL(_v)	((_v) << 16)
/* FL : First Line */
#define D8895_PPS27_FL_BPG_OFFSET_MASK		(0x1F << 0)
#define D8895_PPS27_FL_BPG_OFFSET(_v)		(_v << 0)

#define D8895_DSC_PPS28_31					0x003C
/* NFL : Not First Line */
#define D8895_PPS28_29_NFL_BPG_OFFSET_MASK		(0xFFFF << 16)
#define D8895_PPS28_29_NFL_BPG_OFFSET(_v)		((_v) << 16)
#define D8895_PPS30_31_SLICE_BPG_OFFSET_MASK	(0xFFFF << 0)
#define D8895_PPS30_31_SLICE_BPG_OFFSET(_v)	(_v << 0)

#define D8895_DSC_PPS32_35					0x0040
#define D8895_PPS32_33_INIT_OFFSET_MASK		(0xFFFF << 16)
#define D8895_PPS32_33_INIT_OFFSET(_v)		((_v) << 16)
#define D8895_PPS34_35_FINAL_OFFSET_MASK		(0xFFFF << 0)
#define D8895_PPS34_35_FINAL_OFFSET(_v)		(_v << 0)

#define D8895_DSC_PPS56_59					0x0058
#define D8895_PPS56_RC_BUF_THRESH_C_MASK		(0xFF << 24)
#define D8895_PPS56_RC_BUF_THRESH_C(_v)		((_v) << 24)
#define D8895_PPS57_RC_BUF_THRESH_D_MASK		(0xFF << 16)
#define D8895_PPS57_RC_BUF_THRESH_D(_v)		((_v) << 16)
#define D8895_PPS58_RC_RANGE_PARAM_MASK		(0xFF << 8)
#define D8895_PPS58_RC_RANGE_PARAM(_v)		(_v << 8)
#define D8895_PPS59_RC_RANGE_PARAM_MASK		(0xFF << 0)
#define D8895_PPS59_RC_RANGE_PARAM(_v)		(_v << 0)
#define D8895_PPS58_59_RC_RANGE_PARAM_MASK		(0xFFFF << 0)
#define D8895_PPS58_59_RC_RANGE_PARAM(_v)		(_v << 0)

#endif /* _REGS_DECON8895_H */
