/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 8895 DPP and DPU_DMA registers
 *
 * Copied from the Exynos 8895 vendor kernel
 * (S:drivers/video/fbdev/exynos/dpu/, Copyright (c) 2014-2016 Samsung
 * Electronics), every name prefixed D8895_. Line ranges are marked.
 */

#ifndef _REGS_DPP8895_H
#define _REGS_DPP8895_H

/* regs-dpp.h:24-547 */
#define D8895_DPU_DMA_VERSION			0x0000

#define D8895_DPU_DMA_CH_MAP				0x0004
#define D8895_DMA_CH_MAP(_v, _ch)			(((_v) & 0x3) << (2 * _ch))
#define D8895_DMA_CH_MAP_MASK(_ch)			(0x3 << (2 * _ch))

#define D8895_DPU_DMA_LPI_PERIOD			0x0008
#define D8895_DMA_LPI_PERIOD(_v)			((_v) << 0)
#define D8895_DMA_LPI_PERIOD_MASK			(0x1f << 0)

#define D8895_DPU_DMA_QCH_EN				0x000C
#define D8895_DMA_QCH_EN				(1 << 0)

#define D8895_DPU_DMA_SWRST				0x0010
#define D8895_DMA_CH2_SWRST				(1 << 6)
#define D8895_DMA_CH1_SWRST				(1 << 5)
#define D8895_DMA_CH0_SWRST				(1 << 4)
#define D8895_DMA_ALL_SWRST				(1 << 0)
#define D8895_DMA_CH_SWRST(_ch)			(1 << (4 + (_ch)))


#define D8895_DPU_DMA_GLB_CGEN			0x0014
#define D8895_DMA_SFR_CGEN(_v)			((_v) << 16)
#define D8895_DMA_SFR_CGEN_MASK			(1 << 16)
#define D8895_DMA_INT_CGEN(_v, _sb)			((_v) << (_sb))
#define D8895_DMA_INT_CGEN_MASK(_sb)		(0xFFF << (_sb))
#define D8895_DMA_ALL_CGEN_MASK			(0x10FFF)

#define D8895_DPU_DMA_TEST_PATTERN0_3		0x0020
#define D8895_DPU_DMA_TEST_PATTERN0_2		0x0024
#define D8895_DPU_DMA_TEST_PATTERN0_1		0x0028
#define D8895_DPU_DMA_TEST_PATTERN0_0		0x002C
#define D8895_DPU_DMA_TEST_PATTERN1_3		0x0030
#define D8895_DPU_DMA_TEST_PATTERN1_2		0x0034
#define D8895_DPU_DMA_TEST_PATTERN1_1		0x0038
#define D8895_DPU_DMA_TEST_PATTERN1_0		0x003C
#define D8895_DPU_DMA_GLB_CONTROL		0x0040
#define D8895_DPU_DMA_GLB_DATA		0x0044

/*
 * 1.1 - IDMA SFR
 * < DMA.offset >
 *  G0      G1      VG0     VG1     VGF0    VGF1    WB
 *  0x1000  0x2000  0x3000  0x4000  0x5000  0x6000  0x7000
 */
#define D8895_IDMA_ENABLE				0x0000
#define D8895_IDMA_SRSET				(1 << 24)
#define D8895_IDMA_SFR_CLOCK_GATE_EN		(1 << 10)
#define D8895_IDMA_SRAM_CLOCK_GATE_EN		(1 << 9)
#define D8895_IDMA_ALL_CLOCK_GATE_EN_MASK		(0x3 << 9)
#define D8895_IDMA_FRAME_START_FORCE		(1 << 5)
#define D8895_IDMA_SFR_UPDATE_FORCE			(1 << 4)
#define D8895_IDMA_OP_STATUS				(1 << 2)
#define D8895_OP_STATUS_IDLE				(0)
#define D8895_OP_STATUS_BUSY				(1)
#define D8895_IDMA_SECURE_MODE			(1 << 0)
#define D8895_SECURE_MODE_DISABLE			(0)
#define D8895_SECURE_MODE_ENABLE			(1)

#define D8895_IDMA_IRQ				0x0004
#define D8895_IDMA_AFBC_TIMEOUT_IRQ			(1 << 23)
#define D8895_IDMA_RECOVERY_START_IRQ		(1 << 22)
#define D8895_IDMA_CONFIG_ERROR			(1 << 21)
#define D8895_IDMA_LOCAL_HW_RESET_DONE		(1 << 20)
#define D8895_IDMA_READ_SLAVE_ERROR			(1 << 19)
#define D8895_IDMA_STATUS_DEADLOCK_IRQ		(1 << 17)
#define D8895_IDMA_STATUS_FRAMEDONE_IRQ		(1 << 16)
#define D8895_IDMA_ALL_IRQ_CLEAR			(0xfb << 16)
#define D8895_IDMA_AFBC_TIMEOUT_MASK		(1 << 8)
#define D8895_IDMA_RECOVERY_START_MASK		(1 << 7)
#define D8895_IDMA_CONFIG_ERROR_MASK		(1 << 6)
#define D8895_IDMA_LOCAL_HW_RESET_DONE_MASK	(1 << 5)
#define D8895_IDMA_READ_SLAVE_ERROR_MASK		(1 << 4)
#define D8895_IDMA_IRQ_DEADLOCK_MASK		(1 << 2)
#define D8895_IDMA_IRQ_FRAMEDONE_MASK		(1 << 1)
#define D8895_IDMA_ALL_IRQ_MASK			(0xfb << 1)
#define D8895_IDMA_IRQ_ENABLE			(1 << 0)

#define D8895_IDMA_IN_CON				0x0008
#define D8895_IDMA_IN_IC_MAX(_v)			((_v) << 19)
#define D8895_IDMA_IN_IC_MAX_MASK			(0x7f << 19)
#define D8895_IDMA_IMG_FORMAT(_v)			((_v) << 11)
#define D8895_IDMA_IMG_FORMAT_MASK			(0x1f << 11)
#define D8895_IDMA_IMG_FORMAT_ARGB8888		(0)
#define D8895_IDMA_IMG_FORMAT_ABGR8888		(1)
#define D8895_IDMA_IMG_FORMAT_RGBA8888		(2)
#define D8895_IDMA_IMG_FORMAT_BGRA8888		(3)
#define D8895_IDMA_IMG_FORMAT_XRGB8888		(4)
#define D8895_IDMA_IMG_FORMAT_XBGR8888		(5)
#define D8895_IDMA_IMG_FORMAT_RGBX8888		(6)
#define D8895_IDMA_IMG_FORMAT_BGRX8888		(7)
#define D8895_IDMA_IMG_FORMAT_RGB565		(8)
#define D8895_IDMA_IMG_FORMAT_ARGB1555		(12)
#define D8895_IDMA_IMG_FORMAT_ARGB4444		(13)
#define D8895_IDMA_IMG_FORMAT_YUV420_2P		(24)
#define D8895_IDMA_IMG_FORMAT_YVU420_2P		(25)
#define D8895_IDMA_IN_FLIP(_v)			((_v) << 8)
#define D8895_IDMA_IN_FLIP_MASK			(0x3 << 8)
#define D8895_IDMA_AFBC_EN				(1 << 7)
#define D8895_IDMA_AFBC_TO_EN			(1 << 6)
#define D8895_IDMA_IN_CHROMINANCE_STRIDE_SEL	(1 << 4)
#define D8895_IDMA_BLOCK_EN				(1 << 3)

#define D8895_IDMA_OUT_CON				0x000C
#define D8895_IDMA_OUT_FRAME_ALPHA(_v)		((_v) << 24)
#define D8895_IDMA_OUT_FRAME_ALPHA_MASK		(0xff << 24)

#define D8895_IDMA_SRC_SIZE				0x0010
#define D8895_IDMA_SRC_HEIGHT(_v)			((_v) << 16)
#define D8895_IDMA_SRC_HEIGHT_MASK			(0x3FFF << 16)
#define D8895_IDMA_SRC_WIDTH(_v)			((_v) << 0)
#define D8895_IDMA_SRC_WIDTH_MASK			(0xFFFF << 0)

#define D8895_IDMA_SRC_OFFSET			0x0014
#define D8895_IDMA_SRC_OFFSET_Y(_v)			((_v) << 16)
#define D8895_IDMA_SRC_OFFSET_Y_MASK		(0x1FFF << 16)
#define D8895_IDMA_SRC_OFFSET_X(_v)			((_v) << 0)
#define D8895_IDMA_SRC_OFFSET_X_MASK		(0x1FFF << 0)

#define D8895_IDMA_IMG_SIZE				0x0018
#define D8895_IDMA_IMG_HEIGHT(_v)			((_v) << 16)
#define D8895_IDMA_IMG_HEIGHT_MASK			(0x1FFF << 16)
#define D8895_IDMA_IMG_WIDTH(_v)			((_v) << 0)
#define D8895_IDMA_IMG_WIDTH_MASK			(0x1FFF << 0)

#define D8895_IDMA_CHROMINANCE_STRIDE		0x0020
#define D8895_IDMA_CHROMA_STRIDE(_v)		((_v) << 0)
#define D8895_IDMA_CHROMA_STRIDE_MASK		(0xFFFF << 0)

#define D8895_IDMA_BLOCK_OFFSET			0x0024
#define D8895_IDMA_BLK_OFFSET_Y(_v)			((_v) << 16)
#define D8895_IDMA_BLK_OFFSET_Y_MASK		(0x1FFF << 16)
#define D8895_IDMA_BLK_OFFSET_X(_v)			((_v) << 0)
#define D8895_IDMA_BLK_OFFSET_X_MASK		(0x1FFF << 0)

#define D8895_IDMA_BLOCK_SIZE			0x0028
#define D8895_IDMA_BLK_HEIGHT(_v)			((_v) << 16)
#define D8895_IDMA_BLK_HEIGHT_MASK			(0x1FFF << 16)
#define D8895_IDMA_BLK_WIDTH(_v)			((_v) << 0)
#define D8895_IDMA_BLK_WIDTH_MASK			(0x1FFF << 0)

#define D8895_IDMA_PERFORMANCE_CON0			0x0030
#define D8895_IDMA_DEGRADATION_TIME(_v)		((_v) << 16)
#define D8895_IDMA_DEGRADATION_TIME_MASK		(0xFFFF << 16)
#define D8895_IDMA_DEGRADATION_EN			(1 << 15)
#define D8895_IDMA_IN_IC_MAX_DEG(_v)		((_v) << 0)
#define D8895_IDMA_IN_IC_MAX_DEG_MASK		(0x7F << 0)

/* _n: [0,7], _v: [0x0, 0xF] */
#define D8895_IDMA_IN_QOS_LUT07_00			0x0034
#define D8895_IDMA_IN_QOS_LUT15_08			0x0038
#define D8895_IDMA_IN_QOS_LUT(_n, _v)		((_v) << (4*(_n)))
#define D8895_IDMA_IN_QOS_LUT_MASK(_n)		(0xF << (4*(_n)))

#define D8895_IDMA_IN_BASE_ADDR_Y			0x0040
#define D8895_IDMA_IN_BASE_ADDR_C			0x0044

#define D8895_IDMA_DEADLOCK_NUM			0x0050
#define D8895_IDMA_DEADLOCK_VAL(_v)			((_v) << 1)
#define D8895_IDMA_DEADLOCK_VAL_MASK		(0x7FFFFFFF << 1)
#define D8895_IDMA_DEADLOCK_EN			(1 << 0)

#define D8895_IDMA_BUS_CON				0x0054

#define D8895_IDMA_DYNAMIC_GATING_EN		0x0058
#define D8895_IDMA_DG_EN(_n, _v)			((_v) << (_n))
#define D8895_IDMA_DG_EN_MASK(_n)			(1 << (_n))
#define D8895_IDMA_DG_EN_ALL				(0x7F << 0)

#define D8895_IDMA_RECOVERY_CTRL			0x005C
#define D8895_IDMA_RECOVERY_NUM(_v)			((_v) << 1)
#define D8895_IDMA_RECOVERY_NUM_MASK		(0x7FFFFFFF << 1)
#define D8895_IDMA_RECOVERY_EN			(1 << 0)

#define D8895_IDMA_CHAN_CONTROL				0x0060
#define D8895_IDMA_CHAN_DATA				0x0064

#define D8895_IDMA_CHAN_DEBUG_ENABLE		(1 << 0)

#define D8895_IDMA_CFG_ERR_STATE			0x0870
#define D8895_IDMA_CFG_ERR_SRC_WIDTH		(1 << 10)
#define D8895_IDMA_CFG_ERR_CHROM_STRIDE		(1 << 9)
#define D8895_IDMA_CFG_ERR_BASE_ADDR_Y		(1 << 8)
#define D8895_IDMA_CFG_ERR_BASE_ADDR_C		(1 << 7)
#define D8895_IDMA_CFG_ERR_IMG_WIDTH_AFBC		(1 << 6)
#define D8895_IDMA_CFG_ERR_IMG_WIDTH		(1 << 5)
#define D8895_IDMA_CFG_ERR_IMG_HEIGHT_ROTATION	(1 << 4)
#define D8895_IDMA_CFG_ERR_IMG_HEIGHT		(1 << 3)
#define D8895_IDMA_CFG_ERR_BLOCKING			(1 << 2)
#define D8895_IDMA_CFG_ERR_SRC_OFFSET_X		(1 << 1)
#define D8895_IDMA_CFG_ERR_SRC_OFFSET_Y		(1 << 0)
#define D8895_IDMA_CFG_ERR_GET(_v)			(((_v) >> 0) & 0x7FF)

/*
 * 1.2 - ODMA(WB) SFR
 * < DMA.offset >
 *  WB = 0x7000
 */
#define D8895_ODMA_ENABLE				0x0000
#define D8895_ODMA_SRSET				(1 << 24)
#define D8895_ODMA_SFR_CLOCK_GATE_EN		(1 << 10)
#define D8895_ODMA_SRAM_CLOCK_GATE_EN		(1 << 9)
#define D8895_ODMA_ALL_CLOCK_GATE_EN_MASK		(0x3 << 9)
#define D8895_ODMA_FRAME_START_FORCE		(1 << 5)
#define D8895_ODMA_SFR_UPDATE_FORCE			(1 << 4)
#define D8895_ODMA_OP_STATUS				(1 << 2)

#define D8895_ODMA_IRQ				0x0004
#define D8895_ODMA_CONFIG_ERROR			(1 << 28)
#define D8895_ODMA_SLICE_DONE(_n)			(1 << (21 + (_n)))
#define D8895_ODMA_ALL_SLICE_DONE_CLEAR		(0x7F << 21)
#define D8895_ODMA_LOCAL_HW_RESET_DONE		(1 << 20)
#define D8895_ODMA_WRITE_SLAVE_ERROR		(1 << 19)
#define D8895_ODMA_STATUS_DEADLOCK_IRQ		(1 << 17)
#define D8895_ODMA_STATUS_FRAMEDONE_IRQ		(1 << 16)
#define D8895_ODMA_ALL_IRQ_CLEAR			(0x1FFB << 16)

#define D8895_ODMA_CONFIG_ERROR_MASK		(1 << 13)
#define D8895_ODMA_SLICE_DONE_MASK(_n)		(1 << (6 + (_n)))
#define D8895_ODMA_ALL_SLICE_DONE_MASK		(0x7F << 6)
#define D8895_ODMA_LOCAL_HW_RESET_DONE_MASK	(1 << 5)
#define D8895_ODMA_WRITE_SLAVE_ERROR_MASK		(1 << 4)
#define D8895_ODMA_IRQ_DEADLOCK_MASK		(1 << 2)
#define D8895_ODMA_IRQ_FRAMEDONE_MASK		(1 << 1)
#define D8895_ODMA_ALL_IRQ_MASK			(0x1FFB << 1)
#define D8895_ODMA_IRQ_ENABLE			(1 << 0)

#define D8895_ODMA_CHROMINANCE_STRIDE		0x0020
#define D8895_ODMA_CHROMA_STRIDE(_v)		((_v) << 0)
#define D8895_ODMA_CHROMA_STRIDE_MASK		(0xFFFF << 0)

#define D8895_ODMA_PERFORMANCE_CON0			0x0030
#define D8895_ODMA_DEGRADATION_TIME(_v)		((_v) << 16)
#define D8895_ODMA_DEGRADATION_TIME_MASK		(0xFFFF << 16)
#define D8895_ODMA_DEGRADATION_EN			(1 << 15)
#define D8895_ODMA_IN_IC_MAX_DEG(_v)		((_v) << 0)
#define D8895_ODMA_IN_IC_MAX_DEG_MASK		(0x7F << 0)

#define D8895_ODMA_OUT_CON0				0x004C
#define D8895_ODMA_IN_IC_MAX(_v)			((_v) << 19)
#define D8895_ODMA_IN_IC_MAX_MASK			(0x7f << 19)
#define D8895_ODMA_IMG_FORMAT(_v)			((_v) << 11)
#define D8895_ODMA_IMG_FORMAT_MASK			(0x1f << 11)
#define D8895_ODMA_IN_CHROMINANCE_STRIDE_SEL	(1 << 4)

#define D8895_ODMA_OUT_CON1				0x0050
#define D8895_ODMA_OUT_FRAME_ALPHA(_v)		((_v) << 24)
#define D8895_ODMA_OUT_FRAME_ALPHA_MASK		(0xff << 24)

#define D8895_ODMA_DST_SIZE				0x0054
#define D8895_ODMA_DST_HEIGHT(_v)			((_v) << 16)
#define D8895_ODMA_DST_HEIGHT_MASK			(0x3FFF << 16)
#define D8895_ODMA_DST_WIDTH(_v)			((_v) << 0)
#define D8895_ODMA_DST_WIDTH_MASK			(0x3FFF << 0)

#define D8895_ODMA_DST_OFFSET			0x0058
#define D8895_ODMA_DST_OFFSET_Y(_v)			((_v) << 16)
#define D8895_ODMA_DST_OFFSET_Y_MASK		(0x1FFF << 16)
#define D8895_ODMA_DST_OFFSET_X(_v)			((_v) << 0)
#define D8895_ODMA_DST_OFFSET_X_MASK		(0x1FFF << 0)

#define D8895_ODMA_OUT_IMG_SIZE			0x005C
#define D8895_ODMA_OUT_IMG_HEIGHT(_v)		((_v) << 16)
#define D8895_ODMA_OUT_IMG_HEIGHT_MASK		(0x1FFF << 16)
#define D8895_ODMA_OUT_IMG_WIDTH(_v)		((_v) << 0)
#define D8895_ODMA_OUT_IMG_WIDTH_MASK		(0x1FFF << 0)

#define D8895_ODMA_OUT_QOS_LUT07_00			0x0060
#define D8895_ODMA_OUT_QOS_LUT15_08			0x0064
#define D8895_ODMA_OUT_QOS_LUT(_n, _v)		((_v) << (4*(_n)))
#define D8895_ODMA_OUT_QOS_LUT_MASK(_n)		(0xF << (4*(_n)))

#define D8895_ODMA_IN_BASE_ADDR_Y			0x0074
#define D8895_ODMA_IN_BASE_ADDR_C			0x0094

#define D8895_ODMA_SLICE0_BYTE_CNT			0x0100
#define D8895_ODMA_SLICE1_BYTE_CNT			0x0104
#define D8895_ODMA_SLICE2_BYTE_CNT			0x0108
#define D8895_ODMA_SLICE3_BYTE_CNT			0x010C
#define D8895_ODMA_SLICE4_BYTE_CNT			0x0110
#define D8895_ODMA_SLICE5_BYTE_CNT			0x0114
#define D8895_ODMA_SLICE6_BYTE_CNT			0x0118
#define D8895_ODMA_FRAME_BYTE_CNT			0x011C
#define D8895_ODMA_SLICE_BYTE_CNT(_n)		(0x0100 + ((_n) * 0x4))

#define D8895_ODMA_USB_TV_WB_CON			0x0120
#define D8895_ODMA_USB_WB_PATH_SEL			(1 << 3)
#define D8895_USB_WB_PATH_MEM			(0)
#define D8895_USB_WB_PATH_OTF			(1)
#define D8895_ODMA_USB_WB_EN				(1 << 2)

#define D8895_ODMA_DEADLOCK_NUM			0x0300
#define D8895_ODMA_DEADLOCK_VAL(_v)			((_v) << 1)
#define D8895_ODMA_DEADLOCK_VAL_MASK		(0x7FFFFFFF << 1)
#define D8895_ODMA_DEADLOCK_EN			(1 << 0)

#define D8895_ODMA_BUS_CON				0x0304

/* _n: [0,4], v: [0,1] */
#define D8895_ODMA_DYNAMIC_GATING_EN		0x0354
#define D8895_ODMA_DG_EN(_n, _v)			((_v) << (_n))
#define D8895_ODMA_DG_EN_MASK(_n)			(1 << (_n))
#define D8895_ODMA_DG_EN_ALL				(0x1F << 0)

#define D8895_ODMA_CHAN_CONTROL			0x0360
#define D8895_ODMA_CHAN_DATA				0x0364

#define D8895_ODMA_CFG_ERR_STATE			0x0C00
#define D8895_ODMA_CFG_ERR_GET(_v)			(((_v) >> 0) & 0x7FF)
// ADD field def

/*
 * 2 - DPU_WB_MUX.base
 *  Non-secure        : 0x1289_0000
 */
#define D8895_DPU_WB_ENABLE				0x0000
#define D8895_WB_SRSET				(1 << 24)
#define D8895_WB_SFR_CLOCK_GATE_EN			(1 << 10)
#define D8895_WB_SRAM_CLOCK_GATE_EN			(1 << 9)
#define D8895_WB_INT_CLOCK_GATE_EN			(1 << 8)
#define D8895_WB_ALL_CLOCK_GATE_EN_MASK		(0x7 << 8)
#define D8895_WB_SFR_UPDATE_FORCE			(1 << 4)
#define D8895_WB_QCHANNEL_EN				(1 << 3)
#define D8895_WB_OP_STATUS				(1 << 2)

#define D8895_DPU_WB_OUT_CON0			0x004C
#define D8895_WB_RGB_TYPE_MASK			(0x3 << 17)
#define D8895_WB_RGB_TYPE(_v)			((_v) << 17)
#define D8895_WB_CSC_R2Y_MASK			(0x1 << 0)
#define D8895_WB_CSC_R2Y(_v)				((_v) << 0)

#define D8895_DPU_WB_OUT_CON1			0x0050
#define D8895_WB_OUT_FRAME_ALPHA(_v)		((_v) << 24)
#define D8895_WB_OUT_FRAME_ALPHA_MASK		(0xff << 24)
#define D8895_WB_UV_OFFSET_Y(_v)			((_v) << 5)
#define D8895_WB_UV_OFFSET_Y_MASK			(0x7 << 5)
#define D8895_WB_UV_OFFSET_X(_v)			((_v) << 0)
#define D8895_WB_UV_OFFSET_X_MASK			(0x7 << 0)

#define D8895_DPU_WB_DST_SIZE			0x0054
#define D8895_WB_DST_HEIGHT(_v)			((_v) << 16)
#define D8895_WB_DST_HEIGHT_MASK			(0x1FFF << 16)
#define D8895_WB_DST_WIDTH(_v)			((_v) << 0)
#define D8895_WB_DST_WIDTH_MASK			(0x1FFF << 0)

#define D8895_DPU_WB_USB_TV_WB_SIZE			0x091C
#define D8895_WB_FRAME_BYTE_CNT(_v)			((_v) << 0)
#define D8895_WB_FRAME_BYTE_CNT_MASK		(0xFFFFFFFF << 0)

#define D8895_DPU_WB_USB_TV_WB_CON			0x0920
#define D8895_WB_USB_WB_EN(_v)			((_v) << 2)
#define D8895_WB_USB_WB_EN_MASK			(0x1 << 2)
#define D8895_WB_SWAP_OPTION(_v)			((_v) << 0)
#define D8895_WB_SWAP_OPTION_MASK			(0x3 << 0)

/* _n: [0,6], v: [0,1] */
#define D8895_DPU_WB_DYNAMIC_GATING_EN		0x0A54
#define D8895_WB_DG_EN(_n, _v)			((_v) << (_n))
#define D8895_WB_DG_EN_MASK(_n)			(1 << (_n))
#define D8895_WB_DG_EN_ALL				(0x7F << 0)

#define D8895_DPU_WB_CFG_ERR_STATE			0x0D08
#define D8895_WB_CFG_ERR_GET(_v)			(((_v) >> 0) & 0xF)
#define D8895_WB_CFG_ERR_WRONG_PATH			(1 << 3)
#define D8895_WB_CFG_ERR_ODD_SIZE			(1 << 2)
#define D8895_WB_CFG_ERR_MAX_SIZE			(1 << 1)
#define D8895_WB_CFG_ERR_MIN_SIZE			(1 << 0)


/*
 * 3 - DPP.base
 *  Secure (G0 only)  : 0x1283_0000
 *  Non-secure        : 0x1285_0000
 */
#define D8895_DPP_ENABLE				0x0000
#define D8895_DPP_SRSET				(1 << 24)
#define D8895_DPP_SFR_CLOCK_GATE_EN			(1 << 10)
#define D8895_DPP_SRAM_CLOCK_GATE_EN		(1 << 9)
#define D8895_DPP_INT_CLOCK_GATE_EN			(1 << 8)
#define D8895_DPP_ALL_CLOCK_GATE_EN_MASK		(0x7 << 8)
#define D8895_DPP_SFR_UPDATE_FORCE			(1 << 4)
#define D8895_DPP_QCHANNEL_EN			(1 << 3)
#define D8895_DPP_OP_STATUS				(1 << 2)
#define D8895_DPP_SECURE_MODE			(1 << 0)

#define D8895_DPP_IRQ					0x0004
#define D8895_DPP_CONFIG_ERROR			(1 << 21)
#define D8895_DPP_STATUS_FRAMEDONE_IRQ		(1 << 16)
#define D8895_DPP_ALL_IRQ_CLEAR			(0x21 << 16)
#define D8895_DPP_CONFIG_ERROR_MASK			(1 << 6)
#define D8895_DPP_IRQ_FRAMEDONE_MASK		(1 << 1)
#define D8895_DPP_ALL_IRQ_MASK			(0x21 << 1)
#define D8895_DPP_IRQ_ENABLE				(1 << 0)

/* VG & VGF only */
#define D8895_DPP_IN_CON				0x0008
#define D8895_DPP_CSC_TYPE(_v)			((_v) << 18)
#define D8895_DPP_CSC_TYPE_MASK			(1 << 18)
#define D8895_DPP_CSC_RANGE(_v)			((_v) << 17)
#define D8895_DPP_CSC_RANGE_MASK			(1 << 17)
#define D8895_DPP_CSC_MODE(_v)			((_v) << 16)
#define D8895_DPP_CSC_MODE_MASK			(1 << 16)
#define D8895_DPP_IMG_FORMAT(_v)			((_v) << 11)
#define D8895_DPP_IMG_FORMAT_MASK			(1 << 11)
#define D8895_DPP_ALPHA_SEL(_v)			((_v) << 10)
#define D8895_DPP_ALPHA_SEL_MASK			(1 << 10)

#define D8895_DPP_IMG_SIZE				0x0018
#define D8895_DPP_IMG_HEIGHT(_v)			((_v) << 16)
#define D8895_DPP_IMG_HEIGHT_MASK			(0x1FFF << 16)
#define D8895_DPP_IMG_WIDTH(_v)			((_v) << 0)
#define D8895_DPP_IMG_WIDTH_MASK			(0x1FFF << 0)

/* VGF only */
#define D8895_DPP_SCALED_IMG_SIZE			0x002C
#define D8895_DPP_SCALED_IMG_HEIGHT(_v)		((_v) << 16)
#define D8895_DPP_SCALED_IMG_HEIGHT_MASK		(0x1FFF << 16)
#define D8895_DPP_SCALED_IMG_WIDTH(_v)		((_v) << 0)
#define D8895_DPP_SCALED_IMG_WIDTH_MASK		(0x1FFF << 0)

/*
 * VG & VGF only
 * (00-01-02) : Reg0.L-Reg0.H-Reg1.L
 * (10-11-12) : Reg1.H-Reg2.L-Reg2.H
 * (20-21-22) : Reg3.L-Reg3.H-Reg4.L
 */
#define D8895_DPP_CSC_COEF0				0x0030
#define D8895_DPP_CSC_COEF1				0x0034
#define D8895_DPP_CSC_COEF2				0x0038
#define D8895_DPP_CSC_COEF3				0x003C
#define D8895_DPP_CSC_COEF4				0x0040
#define D8895_DPP_CSC_COEF_H(_v)			((_v) << 16)
#define D8895_DPP_CSC_COEF_H_MASK			(0xFFF << 16)
#define D8895_DPP_CSC_COEF_L(_v)			((_v) << 0)
#define D8895_DPP_CSC_COEF_L_MASK			(0xFFF << 0)
#define D8895_DPP_CSC_COEF_XX(_n, _v)		((_v) << (0 + (16 * (_n))))
#define D8895_DPP_CSC_COEF_XX_MASK(_n)		(0xFFF << (0 + (16 * (_n))))

#define D8895_DPP_MAIN_H_RATIO			0x0044
#define D8895_DPP_H_RATIO(_v)			((_v) << 0)
#define D8895_DPP_H_RATIO_MASK			(0xFFFFFF << 0)

#define D8895_DPP_MAIN_V_RATIO			0x0048
#define D8895_DPP_V_RATIO(_v)			((_v) << 0)
#define D8895_DPP_V_RATIO_MASK			(0xFFFFFF << 0)

#define D8895_DPP_Y_VCOEF_0A				0x0200
#define D8895_DPP_Y_HCOEF_0A				0x0290
#define D8895_DPP_C_VCOEF_0A				0x0400
#define D8895_DPP_C_HCOEF_0A				0x0490
#define D8895_DPP_SCL_COEF(_v)			((_v) << 0)
#define D8895_DPP_SCL_COEF_MASK			(0x1FF << 0)
#define D8895_DPP_H_COEF(n, s, x)	(0x290 + (n) * 0x4 + (s) * 0x24 + (x) * 0x200)
#define D8895_DPP_V_COEF(n, s, x)	(0x200 + (n) * 0x4 + (s) * 0x24 + (x) * 0x200)

#define D8895_DPP_YHPOSITION				0x05B0
#define D8895_DPP_YVPOSITION				0x05B4
#define D8895_DPP_CHPOSITION				0x05B8
#define D8895_DPP_CVPOSITION				0x05BC
#define D8895_DPP_POS_I(_v)				((_v) << 20)
#define D8895_DPP_POS_I_MASK				(0xFFF << 20)
#define D8895_DPP_POS_I_GET(_v)			(((_v) >> 20) & 0xFFF)
#define D8895_DPP_POS_F(_v)				((_v) << 0)
#define D8895_DPP_POS_F_MASK				(0xFFFFF << 0)
#define D8895_DPP_POS_F_GET(_v)			(((_v) >> 0) & 0xFFFFF)

#define D8895_DPP_SHE_CON0				0x0A0C
#define D8895_DPP_SHE_ON(_v)				((_v) << 31)
#define D8895_DPP_SHE_ON_MASK			(1 << 31)
#define D8895_DPP_GAIN(_v)				((_v) << 16)
#define D8895_DPP_GAIN_MASK				(0x1FF << 16)
#define D8895_DPP_EC_EN(_v)				((_v) << 8)
#define D8895_DPP_EC_EN_MASK				(1 << 8)
#define D8895_DPP_EC_TH_N32(_v)			((_v) << 0)
#define D8895_DPP_EC_TH_N32_MASK			(0xFF << 0)

#define D8895_DPP_SHE_CON1				0x0A10
#define D8895_DPP_EC_TH_N16(_v)			((_v) << 24)
#define D8895_DPP_EC_TH_N16_MASK			(0xFF << 24)
#define D8895_DPP_EC_TH_0(_v)			((_v) << 16)
#define D8895_DPP_EC_TH_0_MASK			(0xFF << 16)
#define D8895_DPP_EC_TH_P16(_v)			((_v) << 8)
#define D8895_DPP_EC_TH_P16_MASK			(0xFF << 8)
#define D8895_DPP_EC_TH_P32(_v)			((_v) << 0)
#define D8895_DPP_EC_TH_P32_MASK			(0xFF << 0)

#define D8895_DPP_SHE_CON2				0x0A14
#define D8895_DPP_SC_EN(_v)				((_v) << 24)
#define D8895_DPP_SC_EN_MASK				(1 << 24)
#define D8895_DPP_SC_TH1(_v)				((_v) << 16)
#define D8895_DPP_SC_TH1_MASK			(0xFF << 16)
#define D8895_DPP_SC_TH2(_v)				((_v) << 8)
#define D8895_DPP_SC_TH2_MASK			(0xFF << 8)
#define D8895_DPP_SC_TH3(_v)				((_v) << 0)
#define D8895_DPP_SC_TH3_MASK			(0xFF << 0)

#define D8895_DPP_DYNAMIC_GATING_EN			0x0A54
#define D8895_DPP_DG_EN(_n, _v)			((_v) << (_n))
#define D8895_DPP_DG_EN_MASK(_n)			(1 << (_n))
#define D8895_DPP_DG_EN_ALL				(0x5F << 0)

#define D8895_DPP_CHAN_CONTROL				0x0C04
#define D8895_DPP_CHAN_DATA				0x0C10

#define D8895_DPP_LINECNT_CON			0x0d00
#define D8895_DPP_LC_CAPTURE(_v)			((_v) << 2)
#define D8895_DPP_LC_CAPTURE_MASK			(1 << 2)
#define D8895_DPP_LC_ENABLE(_v)			((_v) << 0)
#define D8895_DPP_LC_ENABLE_MASK			(1 << 0)

#define D8895_DPP_LINECNT_VAL			0x0d04
#define D8895_DPP_LC_COUNTER(_v)			((_v) << 0)
#define D8895_DPP_LC_COUNTER_MASK			(0x1FFF << 0)
#define D8895_DPP_LC_COUNTER_GET(_v)		(((_v) >> 0) & 0x1FFF)

#define D8895_DPP_CFG_ERR_STATE			0x0d08
/*
#define D8895_DPP_CFG_ERR_SCL_POS			(1 << 4)
#define D8895_DPP_CFG_ERR_SCALE_RATIO		(1 << 3)
#define D8895_DPP_CFG_ERR_ODD_SIZE			(1 << 2)
#define D8895_DPP_CFG_ERR_MAX_SIZE			(1 << 1)
#define D8895_DPP_CFG_ERR_MIN_SIZE			(1 << 0)
 */
#define D8895_DPP_CFG_ERR_GET(_v)			(((_v) >> 0) & 0x1F)

#endif /* _REGS_DPP8895_H */
