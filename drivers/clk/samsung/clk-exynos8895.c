// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Ivaylo Ivanov <ivo.ivanov.ivanov1@gmail.com>
 * Author: Ivaylo Ivanov <ivo.ivanov.ivanov1@gmail.com>
 *
 * Common Clock Framework support for Exynos8895 SoC.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <dt-bindings/clock/samsung,exynos8895.h>

#include "clk.h"
#include "clk-exynos-arm64.h"

/* NOTE: Must be equal to the last clock ID increased by one */
#define CLKS_NR_TOP	(CLK_GOUT_CMU_VPU_BUS + 1)
#define CLKS_NR_FSYS0	(CLK_GOUT_FSYS0_XIU_P_FSYS0_ACLK + 1)
#define CLKS_NR_FSYS1	(CLK_GOUT_FSYS1_XIU_P_FSYS1_ACLK + 1)
#define CLKS_NR_PERIC0	(CLK_GOUT_PERIC0_USI00_QCH + 1)
#define CLKS_NR_PERIC1	(CLK_GOUT_PERIC1_XIU_P_PERIC1_ACLK + 1)
#define CLKS_NR_PERIS	(CLK_GOUT_PERIS_XIU_P_PERIS_ACLK + 1)
#define CLKS_NR_APM	(CLK_GOUT_APM_WDT_APM_PCLK + 1)
#define CLKS_NR_CPUCL0	(CLK_GOUT_CPUCL0_SYSREG_CPUCL0_PCLK + 1)
#define CLKS_NR_CPUCL1	(CLK_GOUT_CPUCL1_SYSREG_CPUCL1_PCLK + 1)
#define CLKS_NR_DPU0	(CLK_GOUT_DPU0_XIU_P0_DPU_ACLK + 1)
#define CLKS_NR_DPU1	(CLK_GOUT_DPU1_SYSREG_DPU1_PCLK + 1)
#define CLKS_NR_MFC	(CLK_GOUT_MFC_SYSREG_MFC_PCLK + 1)
#define CLKS_NR_CAM	(CLK_DOUT_CAM_BUSD_DIV2 + 1)
#define CLKS_NR_G2D	(CLK_GOUT_G2D_XIU_P_G2D_ACLK + 1)
#define CLKS_NR_ISPHQ	(CLK_GOUT_ISPHQ_SYSREG_ISPHQ_PCLK + 1)
#define CLKS_NR_ISPLP	(CLK_GOUT_ISPLP_SYSREG_ISPLP_PCLK + 1)
#define CLKS_NR_DCAM	(CLK_GOUT_DCAM_XIU_P_DCAM_ACLK + 1)
#define CLKS_NR_SRDZ	(CLK_GOUT_SRDZ_XIU_P_SRDZ_ACLK + 1)
#define CLKS_NR_BUSC	(CLK_GOUT_BUSC_TREX_P_BUSC_PCLK_BUSC + 1)
#define CLKS_NR_G3D	(CLK_GOUT_G3D_XIU_P_G3D_ACLK + 1)
#define CLKS_NR_ABOX	(CLK_GOUT_ABOX_CODEC_MCLK + 1)
#define CLKS_NR_VTS	(CLK_GOUT_VTS_DMIC + 1)

/*
 * The gates of the always-on CMUs stay under hardware auto-gating, as the
 * vendor kernel left them; see samsung_cmu_info.hwacg_gates.
 */

/* ---- CMU_TOP ------------------------------------------------------------- */

/* Register Offset definitions for CMU_TOP (0x15a80000) */
#define PLL_LOCKTIME_PLL_SHARED0			0x0000
#define PLL_LOCKTIME_PLL_SHARED1			0x0004
#define PLL_LOCKTIME_PLL_SHARED2			0x0008
#define PLL_LOCKTIME_PLL_SHARED3			0x000c
#define PLL_LOCKTIME_PLL_SHARED4			0x0010
#define PLL_CON0_MUX_CP2AP_MIF_CLK_USER			0x0100
#define PLL_CON2_MUX_CP2AP_MIF_CLK_USER			0x0108
#define PLL_CON0_PLL_SHARED0				0x0120
#define PLL_CON0_PLL_SHARED1				0x0140
#define PLL_CON0_PLL_SHARED2				0x0160
#define PLL_CON0_PLL_SHARED3				0x0180
#define PLL_CON0_PLL_SHARED4				0x01a0
#define CLK_CON_MUX_MUX_CLKCMU_ABOX_CPUABOX		0x1000
#define CLK_CON_MUX_MUX_CLKCMU_APM_BUS			0x1004
#define CLK_CON_MUX_MUX_CLKCMU_BUS1_BUS			0x1008
#define CLK_CON_MUX_MUX_CLKCMU_BUSC_BUS			0x100c
#define CLK_CON_MUX_MUX_CLKCMU_BUSC_BUSPHSI2C		0x1010
#define CLK_CON_MUX_MUX_CLKCMU_CAM_BUS			0x1014
#define CLK_CON_MUX_MUX_CLKCMU_CAM_TPU0			0x1018
#define CLK_CON_MUX_MUX_CLKCMU_CAM_TPU1			0x101c
#define CLK_CON_MUX_MUX_CLKCMU_CAM_VRA			0x1020
#define CLK_CON_MUX_MUX_CLKCMU_CIS_CLK0			0x1024
#define CLK_CON_MUX_MUX_CLKCMU_CIS_CLK1			0x1028
#define CLK_CON_MUX_MUX_CLKCMU_CIS_CLK2			0x102c
#define CLK_CON_MUX_MUX_CLKCMU_CIS_CLK3			0x1030
#define CLK_CON_MUX_MUX_CLKCMU_CORE_BUS			0x1034
#define CLK_CON_MUX_MUX_CLKCMU_CPUCL0_SWITCH		0x1038
#define CLK_CON_MUX_MUX_CLKCMU_CPUCL1_SWITCH		0x103c
#define CLK_CON_MUX_MUX_CLKCMU_DBG_BUS			0x1040
#define CLK_CON_MUX_MUX_CLKCMU_DCAM_BUS			0x1044
#define CLK_CON_MUX_MUX_CLKCMU_DCAM_IMGD		0x1048
#define CLK_CON_MUX_MUX_CLKCMU_DPU_BUS			0x104c
#define CLK_CON_MUX_MUX_CLKCMU_DROOPDETECTOR		0x1050
#define CLK_CON_MUX_MUX_CLKCMU_DSP_BUS			0x1054
#define CLK_CON_MUX_MUX_CLKCMU_FSYS0_BUS		0x1058
#define CLK_CON_MUX_MUX_CLKCMU_FSYS0_DPGTC		0x105c
#define CLK_CON_MUX_MUX_CLKCMU_FSYS0_MMC_EMBD		0x1060
#define CLK_CON_MUX_MUX_CLKCMU_FSYS0_UFS_EMBD		0x1064
#define CLK_CON_MUX_MUX_CLKCMU_FSYS0_USBDRD30		0x1068
#define CLK_CON_MUX_MUX_CLKCMU_FSYS1_BUS		0x106c
#define CLK_CON_MUX_MUX_CLKCMU_FSYS1_MMC_CARD		0x1070
#define CLK_CON_MUX_MUX_CLKCMU_FSYS1_PCIE		0x1074
#define CLK_CON_MUX_MUX_CLKCMU_FSYS1_UFS_CARD		0x1078
#define CLK_CON_MUX_MUX_CLKCMU_G2D_G2D			0x107c
#define CLK_CON_MUX_MUX_CLKCMU_G2D_JPEG			0x1080
#define CLK_CON_MUX_MUX_CLKCMU_HPM			0x1084
#define CLK_CON_MUX_MUX_CLKCMU_IMEM_BUS			0x1088
#define CLK_CON_MUX_MUX_CLKCMU_ISPHQ_BUS		0x108c
#define CLK_CON_MUX_MUX_CLKCMU_ISPLP_BUS		0x1090
#define CLK_CON_MUX_MUX_CLKCMU_IVA_BUS			0x1094
#define CLK_CON_MUX_MUX_CLKCMU_MFC_BUS			0x1098
#define CLK_CON_MUX_MUX_CLKCMU_MIF_SWITCH		0x109c
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_BUS		0x10a0
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_UART_DBG		0x10a4
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI00		0x10a8
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI01		0x10ac
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI02		0x10b0
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI03		0x10b4
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_BUS		0x10b8
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_SPEEDY2		0x10bc
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_SPI_CAM0		0x10c0
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_SPI_CAM1		0x10c4
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_UART_BT		0x10c8
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI04		0x10cc
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI05		0x10d0
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI06		0x10d4
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI07		0x10d8
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI08		0x10dc
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI09		0x10e0
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI10		0x10e4
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI11		0x10e8
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI12		0x10ec
#define CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI13		0x10f0
#define CLK_CON_MUX_MUX_CLKCMU_PERIS_BUS		0x10f4
#define CLK_CON_MUX_MUX_CLKCMU_SRDZ_BUS			0x10f8
#define CLK_CON_MUX_MUX_CLKCMU_SRDZ_IMGD		0x10fc
#define CLK_CON_MUX_MUX_CLKCMU_VPU_BUS			0x1100
#define CLK_CON_MUX_MUX_CLK_CMU_CMUREF			0x1104
#define CLK_CON_MUX_MUX_CMU_CMUREF			0x1108
#define CLK_CON_DIV_CLKCMU_ABOX_CPUABOX			0x1800
#define CLK_CON_DIV_CLKCMU_APM_BUS			0x1804
#define CLK_CON_DIV_CLKCMU_BUS1_BUS			0x1808
#define CLK_CON_DIV_CLKCMU_BUSC_BUS			0x180c
#define CLK_CON_DIV_CLKCMU_BUSC_BUSPHSI2C		0x1810
#define CLK_CON_DIV_CLKCMU_CAM_BUS			0x1814
#define CLK_CON_DIV_CLKCMU_CAM_TPU0			0x1818
#define CLK_CON_DIV_CLKCMU_CAM_TPU1			0x181c
#define CLK_CON_DIV_CLKCMU_CAM_VRA			0x1820
#define CLK_CON_DIV_CLKCMU_CIS_CLK0			0x1824
#define CLK_CON_DIV_CLKCMU_CIS_CLK1			0x1828
#define CLK_CON_DIV_CLKCMU_CIS_CLK2			0x182c
#define CLK_CON_DIV_CLKCMU_CIS_CLK3			0x1830
#define CLK_CON_DIV_CLKCMU_CORE_BUS			0x1834
#define CLK_CON_DIV_CLKCMU_CPUCL0_SWITCH		0x1838
#define CLK_CON_DIV_CLKCMU_CPUCL1_SWITCH		0x183c
#define CLK_CON_DIV_CLKCMU_DBG_BUS			0x1840
#define CLK_CON_DIV_CLKCMU_DCAM_BUS			0x1844
#define CLK_CON_DIV_CLKCMU_DCAM_IMGD			0x1848
#define CLK_CON_DIV_CLKCMU_DPU_BUS			0x184c
#define CLK_CON_DIV_CLKCMU_DSP_BUS			0x1850
#define CLK_CON_DIV_CLKCMU_FSYS0_BUS			0x1854
#define CLK_CON_DIV_CLKCMU_FSYS0_DPGTC			0x1858
#define CLK_CON_DIV_CLKCMU_FSYS0_MMC_EMBD		0x185c
#define CLK_CON_DIV_CLKCMU_FSYS0_UFS_EMBD		0x1860
#define CLK_CON_DIV_CLKCMU_FSYS0_USBDRD30		0x1864
#define CLK_CON_DIV_CLKCMU_FSYS1_BUS			0x1868
#define CLK_CON_DIV_CLKCMU_FSYS1_MMC_CARD		0x186c
#define CLK_CON_DIV_CLKCMU_FSYS1_PCIE			0x1870
#define CLK_CON_DIV_CLKCMU_FSYS1_UFS_CARD		0x1874
#define CLK_CON_DIV_CLKCMU_G2D_G2D			0x1878
#define CLK_CON_DIV_CLKCMU_G2D_JPEG			0x187c
#define CLK_CON_DIV_CLKCMU_G3D_SWITCH			0x1880
#define CLK_CON_DIV_CLKCMU_HPM				0x1884
#define CLK_CON_DIV_CLKCMU_IMEM_BUS			0x1888
#define CLK_CON_DIV_CLKCMU_ISPHQ_BUS			0x188c
#define CLK_CON_DIV_CLKCMU_ISPLP_BUS			0x1890
#define CLK_CON_DIV_CLKCMU_IVA_BUS			0x1894
#define CLK_CON_DIV_CLKCMU_MFC_BUS			0x1898
#define CLK_CON_DIV_CLKCMU_MODEM_SHARED0		0x189c
#define CLK_CON_DIV_CLKCMU_MODEM_SHARED1		0x18a0
#define CLK_CON_DIV_CLKCMU_OTP				0x18a4
#define CLK_CON_DIV_CLKCMU_PERIC0_BUS			0x18a8
#define CLK_CON_DIV_CLKCMU_PERIC0_UART_DBG		0x18ac
#define CLK_CON_DIV_CLKCMU_PERIC0_USI00			0x18b0
#define CLK_CON_DIV_CLKCMU_PERIC0_USI01			0x18b4
#define CLK_CON_DIV_CLKCMU_PERIC0_USI02			0x18b8
#define CLK_CON_DIV_CLKCMU_PERIC0_USI03			0x18bc
#define CLK_CON_DIV_CLKCMU_PERIC1_BUS			0x18c0
#define CLK_CON_DIV_CLKCMU_PERIC1_SPEEDY2		0x18c4
#define CLK_CON_DIV_CLKCMU_PERIC1_SPI_CAM0		0x18c8
#define CLK_CON_DIV_CLKCMU_PERIC1_SPI_CAM1		0x18cc
#define CLK_CON_DIV_CLKCMU_PERIC1_UART_BT		0x18d0
#define CLK_CON_DIV_CLKCMU_PERIC1_USI04			0x18d4
#define CLK_CON_DIV_CLKCMU_PERIC1_USI05			0x18d8
#define CLK_CON_DIV_CLKCMU_PERIC1_USI06			0x18dc
#define CLK_CON_DIV_CLKCMU_PERIC1_USI07			0x18e0
#define CLK_CON_DIV_CLKCMU_PERIC1_USI08			0x18e4
#define CLK_CON_DIV_CLKCMU_PERIC1_USI09			0x18e8
#define CLK_CON_DIV_CLKCMU_PERIC1_USI10			0x18ec
#define CLK_CON_DIV_CLKCMU_PERIC1_USI11			0x18f0
#define CLK_CON_DIV_CLKCMU_PERIC1_USI12			0x18f4
#define CLK_CON_DIV_CLKCMU_PERIC1_USI13			0x18f8
#define CLK_CON_DIV_CLKCMU_PERIS_BUS			0x18fc
#define CLK_CON_DIV_CLKCMU_SRDZ_BUS			0x1900
#define CLK_CON_DIV_CLKCMU_SRDZ_IMGD			0x1904
#define CLK_CON_DIV_CLKCMU_VPU_BUS			0x1908
#define CLK_CON_DIV_DIV_CLK_CMU_CMUREF			0x190c
#define CLK_CON_DIV_DIV_CP2AP_MIF_CLK_DIV2		0x1910
#define CLK_CON_DIV_DIV_PLL_SHARED0_DIV2		0x1914
#define CLK_CON_DIV_DIV_PLL_SHARED0_DIV4		0x1918
#define CLK_CON_DIV_DIV_PLL_SHARED1_DIV2		0x191c
#define CLK_CON_DIV_DIV_PLL_SHARED1_DIV4		0x1920
#define CLK_CON_DIV_DIV_PLL_SHARED2_DIV2		0x1924
#define CLK_CON_DIV_DIV_PLL_SHARED3_DIV2		0x1928
#define CLK_CON_DIV_DIV_PLL_SHARED4_DIV2		0x192c
#define CLK_CON_GAT_CLKCMU_DROOPDETECTOR		0x2000
#define CLK_CON_GAT_CLKCMU_MIF_SWITCH			0x2004
#define CLK_CON_GAT_GATE_CLKCMU_ABOX_CPUABOX		0x2008
#define CLK_CON_GAT_GATE_CLKCMU_APM_BUS			0x200c
#define CLK_CON_GAT_GATE_CLKCMU_BUS1_BUS		0x2010
#define CLK_CON_GAT_GATE_CLKCMU_BUSC_BUS		0x2014
#define CLK_CON_GAT_GATE_CLKCMU_BUSC_BUSPHSI2C		0x2018
#define CLK_CON_GAT_GATE_CLKCMU_CAM_BUS			0x201c
#define CLK_CON_GAT_GATE_CLKCMU_CAM_TPU0		0x2020
#define CLK_CON_GAT_GATE_CLKCMU_CAM_TPU1		0x2024
#define CLK_CON_GAT_GATE_CLKCMU_CAM_VRA			0x2028
#define CLK_CON_GAT_GATE_CLKCMU_CIS_CLK0		0x202c
#define CLK_CON_GAT_GATE_CLKCMU_CIS_CLK1		0x2030
#define CLK_CON_GAT_GATE_CLKCMU_CIS_CLK2		0x2034
#define CLK_CON_GAT_GATE_CLKCMU_CIS_CLK3		0x2038
#define CLK_CON_GAT_GATE_CLKCMU_CORE_BUS		0x203c
#define CLK_CON_GAT_GATE_CLKCMU_CPUCL0_SWITCH		0x2040
#define CLK_CON_GAT_GATE_CLKCMU_CPUCL1_SWITCH		0x2044
#define CLK_CON_GAT_GATE_CLKCMU_DBG_BUS			0x2048
#define CLK_CON_GAT_GATE_CLKCMU_DCAM_BUS		0x204c
#define CLK_CON_GAT_GATE_CLKCMU_DCAM_IMGD		0x2050
#define CLK_CON_GAT_GATE_CLKCMU_DPU_BUS			0x2054
#define CLK_CON_GAT_GATE_CLKCMU_DSP_BUS			0x2058
#define CLK_CON_GAT_GATE_CLKCMU_FSYS0_BUS		0x205c
#define CLK_CON_GAT_GATE_CLKCMU_FSYS0_DPGTC		0x2060
#define CLK_CON_GAT_GATE_CLKCMU_FSYS0_MMC_EMBD		0x2064
#define CLK_CON_GAT_GATE_CLKCMU_FSYS0_UFS_EMBD		0x2068
#define CLK_CON_GAT_GATE_CLKCMU_FSYS0_USBDRD30		0x206c
#define CLK_CON_GAT_GATE_CLKCMU_FSYS1_BUS		0x2070
#define CLK_CON_GAT_GATE_CLKCMU_FSYS1_MMC_CARD		0x2074
#define CLK_CON_GAT_GATE_CLKCMU_FSYS1_PCIE		0x2078
#define CLK_CON_GAT_GATE_CLKCMU_FSYS1_UFS_CARD		0x207c
#define CLK_CON_GAT_GATE_CLKCMU_G2D_G2D			0x2080
#define CLK_CON_GAT_GATE_CLKCMU_G2D_JPEG		0x2084
#define CLK_CON_GAT_GATE_CLKCMU_G3D_SWITCH		0x2088
#define CLK_CON_GAT_GATE_CLKCMU_HPM			0x208c
#define CLK_CON_GAT_GATE_CLKCMU_IMEM_BUS		0x2090
#define CLK_CON_GAT_GATE_CLKCMU_ISPHQ_BUS		0x2094
#define CLK_CON_GAT_GATE_CLKCMU_ISPLP_BUS		0x2098
#define CLK_CON_GAT_GATE_CLKCMU_IVA_BUS			0x209c
#define CLK_CON_GAT_GATE_CLKCMU_MFC_BUS			0x20a0
#define CLK_CON_GAT_GATE_CLKCMU_MODEM_SHARED0		0x20a4
#define CLK_CON_GAT_GATE_CLKCMU_MODEM_SHARED1		0x20a8
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_BUS		0x20ac
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_UART_DBG		0x20b0
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_USI00		0x20b4
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_USI01		0x20b8
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_USI02		0x20bc
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_USI03		0x20c0
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_BUS		0x20c4
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_SPEEDY2		0x20c8
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_SPI_CAM0		0x20cc
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_SPI_CAM1		0x20d0
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_UART_BT		0x20d4
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI04		0x20d8
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI05		0x20dc
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI06		0x20e0
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI07		0x20e4
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI08		0x20e8
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI09		0x20ec
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI10		0x20f0
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI11		0x20f4
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI12		0x20f8
#define CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI13		0x20fc
#define CLK_CON_GAT_GATE_CLKCMU_PERIS_BUS		0x2100
#define CLK_CON_GAT_GATE_CLKCMU_SRDZ_BUS		0x2104
#define CLK_CON_GAT_GATE_CLKCMU_SRDZ_IMGD		0x2108
#define CLK_CON_GAT_GATE_CLKCMU_VPU_BUS			0x210c

/*
 * The bus clock registers the APM firmware sets for DVFS (PLL_SHARED1 and the
 * MIF, INT, INTCAM, CAM and DISP bus muxes and dividers) are not listed here:
 * a restore after sleep would undo the level the firmware has set since.
 */
static const unsigned long top_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_SHARED0,
	PLL_LOCKTIME_PLL_SHARED1,
	PLL_LOCKTIME_PLL_SHARED2,
	PLL_LOCKTIME_PLL_SHARED3,
	PLL_LOCKTIME_PLL_SHARED4,
	PLL_CON0_MUX_CP2AP_MIF_CLK_USER,
	PLL_CON2_MUX_CP2AP_MIF_CLK_USER,
	PLL_CON0_PLL_SHARED0,
	PLL_CON0_PLL_SHARED2,
	PLL_CON0_PLL_SHARED3,
	PLL_CON0_PLL_SHARED4,
	CLK_CON_MUX_MUX_CLKCMU_ABOX_CPUABOX,
	CLK_CON_MUX_MUX_CLKCMU_APM_BUS,
	CLK_CON_MUX_MUX_CLKCMU_BUSC_BUSPHSI2C,
	CLK_CON_MUX_MUX_CLKCMU_CIS_CLK0,
	CLK_CON_MUX_MUX_CLKCMU_CIS_CLK1,
	CLK_CON_MUX_MUX_CLKCMU_CIS_CLK2,
	CLK_CON_MUX_MUX_CLKCMU_CIS_CLK3,
	CLK_CON_MUX_MUX_CLKCMU_CPUCL0_SWITCH,
	CLK_CON_MUX_MUX_CLKCMU_CPUCL1_SWITCH,
	CLK_CON_MUX_MUX_CLKCMU_DBG_BUS,
	CLK_CON_MUX_MUX_CLKCMU_DROOPDETECTOR,
	CLK_CON_MUX_MUX_CLKCMU_FSYS0_DPGTC,
	CLK_CON_MUX_MUX_CLKCMU_FSYS0_MMC_EMBD,
	CLK_CON_MUX_MUX_CLKCMU_FSYS0_UFS_EMBD,
	CLK_CON_MUX_MUX_CLKCMU_FSYS0_USBDRD30,
	CLK_CON_MUX_MUX_CLKCMU_FSYS1_MMC_CARD,
	CLK_CON_MUX_MUX_CLKCMU_FSYS1_PCIE,
	CLK_CON_MUX_MUX_CLKCMU_FSYS1_UFS_CARD,
	CLK_CON_MUX_MUX_CLKCMU_HPM,
	CLK_CON_MUX_MUX_CLKCMU_MIF_SWITCH,
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_BUS,
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_UART_DBG,
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI00,
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI01,
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI02,
	CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI03,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_BUS,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_SPEEDY2,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_SPI_CAM0,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_SPI_CAM1,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_UART_BT,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI04,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI05,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI06,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI07,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI08,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI09,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI10,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI11,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI12,
	CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI13,
	CLK_CON_MUX_MUX_CLKCMU_PERIS_BUS,
	CLK_CON_MUX_MUX_CLK_CMU_CMUREF,
	CLK_CON_MUX_MUX_CMU_CMUREF,
	CLK_CON_DIV_CLKCMU_ABOX_CPUABOX,
	CLK_CON_DIV_CLKCMU_APM_BUS,
	CLK_CON_DIV_CLKCMU_BUSC_BUSPHSI2C,
	CLK_CON_DIV_CLKCMU_CIS_CLK0,
	CLK_CON_DIV_CLKCMU_CIS_CLK1,
	CLK_CON_DIV_CLKCMU_CIS_CLK2,
	CLK_CON_DIV_CLKCMU_CIS_CLK3,
	CLK_CON_DIV_CLKCMU_CPUCL0_SWITCH,
	CLK_CON_DIV_CLKCMU_CPUCL1_SWITCH,
	CLK_CON_DIV_CLKCMU_DBG_BUS,
	CLK_CON_DIV_CLKCMU_FSYS0_DPGTC,
	CLK_CON_DIV_CLKCMU_FSYS0_MMC_EMBD,
	CLK_CON_DIV_CLKCMU_FSYS0_UFS_EMBD,
	CLK_CON_DIV_CLKCMU_FSYS0_USBDRD30,
	CLK_CON_DIV_CLKCMU_FSYS1_MMC_CARD,
	CLK_CON_DIV_CLKCMU_FSYS1_PCIE,
	CLK_CON_DIV_CLKCMU_FSYS1_UFS_CARD,
	CLK_CON_DIV_CLKCMU_G3D_SWITCH,
	CLK_CON_DIV_CLKCMU_HPM,
	CLK_CON_DIV_CLKCMU_MODEM_SHARED0,
	CLK_CON_DIV_CLKCMU_MODEM_SHARED1,
	CLK_CON_DIV_CLKCMU_OTP,
	CLK_CON_DIV_CLKCMU_PERIC0_BUS,
	CLK_CON_DIV_CLKCMU_PERIC0_UART_DBG,
	CLK_CON_DIV_CLKCMU_PERIC0_USI00,
	CLK_CON_DIV_CLKCMU_PERIC0_USI01,
	CLK_CON_DIV_CLKCMU_PERIC0_USI02,
	CLK_CON_DIV_CLKCMU_PERIC0_USI03,
	CLK_CON_DIV_CLKCMU_PERIC1_BUS,
	CLK_CON_DIV_CLKCMU_PERIC1_SPEEDY2,
	CLK_CON_DIV_CLKCMU_PERIC1_SPI_CAM0,
	CLK_CON_DIV_CLKCMU_PERIC1_SPI_CAM1,
	CLK_CON_DIV_CLKCMU_PERIC1_UART_BT,
	CLK_CON_DIV_CLKCMU_PERIC1_USI04,
	CLK_CON_DIV_CLKCMU_PERIC1_USI05,
	CLK_CON_DIV_CLKCMU_PERIC1_USI06,
	CLK_CON_DIV_CLKCMU_PERIC1_USI07,
	CLK_CON_DIV_CLKCMU_PERIC1_USI08,
	CLK_CON_DIV_CLKCMU_PERIC1_USI09,
	CLK_CON_DIV_CLKCMU_PERIC1_USI10,
	CLK_CON_DIV_CLKCMU_PERIC1_USI11,
	CLK_CON_DIV_CLKCMU_PERIC1_USI12,
	CLK_CON_DIV_CLKCMU_PERIC1_USI13,
	CLK_CON_DIV_CLKCMU_PERIS_BUS,
	CLK_CON_DIV_DIV_CLK_CMU_CMUREF,
	CLK_CON_DIV_DIV_CP2AP_MIF_CLK_DIV2,
	CLK_CON_DIV_DIV_PLL_SHARED0_DIV2,
	CLK_CON_DIV_DIV_PLL_SHARED0_DIV4,
	CLK_CON_DIV_DIV_PLL_SHARED1_DIV2,
	CLK_CON_DIV_DIV_PLL_SHARED1_DIV4,
	CLK_CON_DIV_DIV_PLL_SHARED2_DIV2,
	CLK_CON_DIV_DIV_PLL_SHARED3_DIV2,
	CLK_CON_DIV_DIV_PLL_SHARED4_DIV2,
};

static const struct samsung_pll_rate_table pll_shared0_rate_table[] __initconst = {
	PLL_35XX_RATE(26 * MHZ, 2132000000U, 328, 4, 0),
};

static const struct samsung_pll_rate_table pll_shared1_rate_table[] __initconst = {
	PLL_35XX_RATE(26 * MHZ, 1865500000U, 287, 4, 0),
};

static const struct samsung_pll_rate_table pll_shared2_rate_table[] __initconst = {
	PLL_35XX_RATE(26 * MHZ, 800000000U, 400, 13, 0),
};

static const struct samsung_pll_rate_table pll_shared3_rate_table[] __initconst = {
	PLL_35XX_RATE(26 * MHZ, 630000000U, 315, 13, 0),
};

static const struct samsung_pll_rate_table pll_shared4_rate_table[] __initconst = {
	PLL_35XX_RATE(26 * MHZ, 667333333U, 154, 6, 0),
};

static const struct samsung_pll_clock top_pll_clks[] __initconst = {
	/* CMU_TOP_PURECLKCOMP */
	PLL(pll_1051x, CLK_FOUT_SHARED0_PLL, "fout_shared0_pll", "oscclk",
	    PLL_LOCKTIME_PLL_SHARED0, PLL_CON0_PLL_SHARED0,
	    pll_shared0_rate_table),
	PLL(pll_1051x, CLK_FOUT_SHARED1_PLL, "fout_shared1_pll", "oscclk",
	    PLL_LOCKTIME_PLL_SHARED1, PLL_CON0_PLL_SHARED1,
	    pll_shared1_rate_table),
	PLL(pll_1052x, CLK_FOUT_SHARED2_PLL, "fout_shared2_pll", "oscclk",
	    PLL_LOCKTIME_PLL_SHARED2, PLL_CON0_PLL_SHARED2,
	    pll_shared2_rate_table),
	PLL(pll_1052x, CLK_FOUT_SHARED3_PLL, "fout_shared3_pll", "oscclk",
	    PLL_LOCKTIME_PLL_SHARED3, PLL_CON0_PLL_SHARED3,
	    pll_shared3_rate_table),
	PLL(pll_1052x, CLK_FOUT_SHARED4_PLL, "fout_shared4_pll", "oscclk",
	    PLL_LOCKTIME_PLL_SHARED4, PLL_CON0_PLL_SHARED4,
	    pll_shared4_rate_table),
};

/* List of parent clocks for Muxes in CMU_TOP */
PNAME(mout_pll_shared0_p)		= { "oscclk", "fout_shared0_pll" };
PNAME(mout_pll_shared1_p)		= { "oscclk", "fout_shared1_pll" };
PNAME(mout_pll_shared2_p)		= { "oscclk", "fout_shared2_pll" };
PNAME(mout_pll_shared3_p)		= { "oscclk", "fout_shared3_pll" };
PNAME(mout_pll_shared4_p)		= { "oscclk", "fout_shared4_pll" };
PNAME(mout_cp2ap_mif_clk_user_p)	= { "oscclk", "cp2ap_mif_clk" };
PNAME(mout_cmu_abox_cpuabox_p)		= { "dout_cmu_shared0_div2",
					    "dout_cmu_shared1_div2",
					    "fout_shared2_pll",
					    "fout_shared4_pll" };
PNAME(mout_cmu_apm_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_bus1_bus_p)		= { "fout_shared4_pll",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_busc_bus_p)		= { "fout_shared4_pll",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_cp2ap_mif_clk_div2",
					    "oscclk", "oscclk", "oscclk" };
PNAME(mout_cmu_busc_busphsi2c_p)	= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_cp2ap_mif_clk_div2",
					    "oscclk" };
PNAME(mout_cmu_cam_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_cam_tpu0_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_cam_tpu1_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_cam_vra_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_cis_clk0_p)		= { "oscclk",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_cis_clk1_p)		= { "oscclk",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_cis_clk2_p)		= { "oscclk",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_cis_clk3_p)		= { "oscclk",
					    "dout_cmu_shared2_div2" };
PNAME(mout_core_bus_p)			= { "dout_cmu_shared0_div2",
					    "dout_cmu_shared1_div2",
					    "fout_shared2_pll",
					    "fout_shared4_pll",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_cp2ap_mif_clk_div2" };
PNAME(mout_cmu_cpucl0_switch_p)		= { "dout_cmu_shared0_div2",
					    "dout_cmu_shared1_div2",
					    "fout_shared2_pll",
					    "fout_shared4_pll" };
PNAME(mout_cmu_cpucl1_switch_p)		= { "dout_cmu_shared0_div2",
					    "dout_cmu_shared1_div2",
					    "fout_shared2_pll",
					    "fout_shared4_pll" };
PNAME(mout_cmu_dbg_bus_p)		= { "fout_shared2_pll",
					    "fout_shared4_pll",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4" };
PNAME(mout_cmu_dcam_bus_p)		= { "fout_shared4_pll",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_dcam_imgd_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_dpu_bus_p)		= { "dout_cmu_shared0_div2",
					    "fout_shared3_pll",
					    "fout_shared4_pll",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "oscclk", "oscclk", "oscclk" };
PNAME(mout_cmu_droopdetector_p)		= { "oscclk", "dout_cmu_shared0_div2",
					    "dout_cmu_shared1_div2",
					    "fout_shared2_pll" };
PNAME(mout_cmu_dsp_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_fsys0_bus_p)			= { "dout_cmu_shared1_div2",
					    "fout_shared2_pll",
					    "fout_shared4_pll",
					    "dout_cmu_shared0_div4" };
PNAME(mout_fsys0_dpgtc_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_fsys0_mmc_embd_p)		= { "oscclk", "fout_shared2_pll",
					    "fout_shared4_pll",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "oscclk", "oscclk", "oscclk" };
PNAME(mout_fsys0_ufs_embd_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_fsys0_usbdrd30_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_fsys1_bus_p)		= { "fout_shared2_pll",
					    "dout_cmu_shared0_div4" };
PNAME(mout_cmu_fsys1_mmc_card_p)	= { "oscclk", "fout_shared2_pll",
					    "fout_shared4_pll",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "oscclk", "oscclk", "oscclk" };
PNAME(mout_cmu_fsys1_pcie_p)		= { "oscclk", "fout_shared2_pll" };
PNAME(mout_cmu_fsys1_ufs_card_p)	= { "oscclk",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_g2d_g2d_p)		= { "fout_shared4_pll",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_g2d_jpeg_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_hpm_p)			= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_imem_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_isphq_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_isplp_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_iva_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_mfc_bus_p)		= { "fout_shared4_pll",
					    "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_mif_switch_p)		= { "fout_shared0_pll",
					    "fout_shared1_pll",
					    "dout_cmu_shared0_div2",
					    "dout_cmu_shared1_div2",
					    "fout_shared2_pll",
					    "mout_cp2ap_mif_clk_user",
					    "oscclk", "oscclk" };
PNAME(mout_cmu_peric0_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_peric0_uart_dbg_p)	= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric0_usi00_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric0_usi01_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric0_usi02_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric0_usi03_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_peric1_speedy2_p)	= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "oscclk" };
PNAME(mout_cmu_peric1_spi_cam0_p)	= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_spi_cam1_p)	= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_uart_bt_p)	= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi04_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi05_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi06_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi07_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi08_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi09_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi10_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi11_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi12_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peric1_usi13_p)		= { "oscclk", "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_peris_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared2_div2" };
PNAME(mout_cmu_srdz_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_srdz_imgd_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };
PNAME(mout_cmu_vpu_bus_p)		= { "dout_cmu_shared0_div4",
					    "dout_cmu_shared1_div4",
					    "dout_cmu_shared2_div2",
					    "dout_cmu_shared4_div2" };

/*
 * Register name to clock name mangling strategy used in this file
 *
 * Replace PLL_CON0_PLL	           with CLK_MOUT_PLL and mout_pll
 * Replace CLK_CON_MUX_MUX_CLKCMU  with CLK_MOUT_CMU and mout_cmu
 * Replace CLK_CON_DIV_CLKCMU      with CLK_DOUT_CMU and dout_cmu
 * Replace CLK_CON_DIV_DIV_CLKCMU  with CLK_DOUT_CMU and dout_cmu
 * Replace CLK_CON_GAT_CLKCMU      with CLK_GOUT_CMU and gout_cmu
 * Replace CLK_CON_GAT_GATE_CLKCMU with CLK_GOUT_CMU and gout_cmu
 *
 * For gates remove _UID _BLK _IPCLKPORT and _RSTNSYNC
 */

static const struct samsung_mux_clock top_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PLL_SHARED0, "mout_pll_shared0", mout_pll_shared0_p,
	    PLL_CON0_PLL_SHARED0, 4, 1),
	MUX(CLK_MOUT_PLL_SHARED1, "mout_pll_shared1", mout_pll_shared1_p,
	    PLL_CON0_PLL_SHARED1, 4, 1),
	MUX(CLK_MOUT_PLL_SHARED2, "mout_pll_shared2", mout_pll_shared2_p,
	    PLL_CON0_PLL_SHARED2, 4, 1),
	MUX(CLK_MOUT_PLL_SHARED3, "mout_pll_shared3", mout_pll_shared3_p,
	    PLL_CON0_PLL_SHARED3, 4, 1),
	MUX(CLK_MOUT_PLL_SHARED4, "mout_pll_shared4", mout_pll_shared4_p,
	    PLL_CON0_PLL_SHARED4, 4, 1),
	MUX(CLK_MOUT_CP2AP_MIF_CLK_USER, "mout_cp2ap_mif_clk_user",
	    mout_cp2ap_mif_clk_user_p, PLL_CON0_MUX_CP2AP_MIF_CLK_USER, 4, 1),
	MUX(CLK_MOUT_CMU_ABOX_CPUABOX, "mout_cmu_abox_cpuabox",
	    mout_cmu_abox_cpuabox_p, CLK_CON_MUX_MUX_CLKCMU_ABOX_CPUABOX,
	    0, 2),
	MUX(CLK_MOUT_CMU_APM_BUS, "mout_cmu_apm_bus", mout_cmu_apm_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_APM_BUS, 0, 1),
	MUX(CLK_MOUT_CMU_BUS1_BUS, "mout_cmu_bus1_bus", mout_cmu_bus1_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_BUS1_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_BUSC_BUS, "mout_cmu_busc_bus", mout_cmu_busc_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_BUSC_BUS, 0, 3),
	MUX(CLK_MOUT_CMU_BUSC_BUSPHSI2C, "mout_cmu_busc_busphsi2c",
	    mout_cmu_busc_busphsi2c_p, CLK_CON_MUX_MUX_CLKCMU_BUSC_BUSPHSI2C,
	    0, 2),
	MUX(CLK_MOUT_CMU_CAM_BUS, "mout_cmu_cam_bus", mout_cmu_cam_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_CAM_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_CAM_TPU0, "mout_cmu_cam_tpu0", mout_cmu_cam_tpu0_p,
	    CLK_CON_MUX_MUX_CLKCMU_CAM_TPU0, 0, 2),
	MUX(CLK_MOUT_CMU_CAM_TPU1, "mout_cmu_cam_tpu1", mout_cmu_cam_tpu1_p,
	    CLK_CON_MUX_MUX_CLKCMU_CAM_TPU1, 0, 2),
	MUX(CLK_MOUT_CMU_CAM_VRA, "mout_cmu_cam_vra", mout_cmu_cam_vra_p,
	    CLK_CON_MUX_MUX_CLKCMU_CAM_VRA, 0, 2),
	MUX(CLK_MOUT_CMU_CIS_CLK0, "mout_cmu_cis_clk0", mout_cmu_cis_clk0_p,
	    CLK_CON_MUX_MUX_CLKCMU_CIS_CLK0, 0, 1),
	MUX(CLK_MOUT_CMU_CIS_CLK1, "mout_cmu_cis_clk1", mout_cmu_cis_clk1_p,
	    CLK_CON_MUX_MUX_CLKCMU_CIS_CLK1, 0, 1),
	MUX(CLK_MOUT_CMU_CIS_CLK2, "mout_cmu_cis_clk2", mout_cmu_cis_clk2_p,
	    CLK_CON_MUX_MUX_CLKCMU_CIS_CLK2, 0, 1),
	MUX(CLK_MOUT_CMU_CIS_CLK3, "mout_cmu_cis_clk3", mout_cmu_cis_clk3_p,
	    CLK_CON_MUX_MUX_CLKCMU_CIS_CLK3, 0, 1),
	MUX(CLK_MOUT_CMU_CORE_BUS, "mout_core_bus", mout_core_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_CORE_BUS, 0, 3),
	MUX(CLK_MOUT_CMU_CPUCL0_SWITCH, "mout_cmu_cpucl0_switch",
	    mout_cmu_cpucl0_switch_p, CLK_CON_MUX_MUX_CLKCMU_CPUCL0_SWITCH,
	    0, 2),
	MUX(CLK_MOUT_CMU_CPUCL1_SWITCH, "mout_cmu_cpucl1_switch",
	    mout_cmu_cpucl1_switch_p, CLK_CON_MUX_MUX_CLKCMU_CPUCL1_SWITCH,
	    0, 2),
	MUX(CLK_MOUT_CMU_DBG_BUS, "mout_cmu_dbg_bus", mout_cmu_dbg_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_DBG_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_DCAM_BUS, "mout_cmu_dcam_bus", mout_cmu_dcam_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_DCAM_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_DCAM_IMGD, "mout_cmu_dcam_imgd", mout_cmu_dcam_imgd_p,
	    CLK_CON_MUX_MUX_CLKCMU_DCAM_IMGD, 0, 2),
	MUX(CLK_MOUT_CMU_DPU_BUS, "mout_cmu_dpu_bus", mout_cmu_dpu_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_DPU_BUS, 0, 3),
	MUX(CLK_MOUT_CMU_DROOPDETECTOR, "mout_cmu_droopdetector",
	    mout_cmu_droopdetector_p, CLK_CON_MUX_MUX_CLKCMU_DROOPDETECTOR,
	    0, 2),
	MUX(CLK_MOUT_CMU_DSP_BUS, "mout_cmu_dsp_bus", mout_cmu_dsp_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_DSP_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_FSYS0_BUS, "mout_fsys0_bus", mout_fsys0_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_FSYS0_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_FSYS0_DPGTC, "mout_fsys0_dpgtc", mout_fsys0_dpgtc_p,
	    CLK_CON_MUX_MUX_CLKCMU_FSYS0_DPGTC, 0, 2),
	MUX(CLK_MOUT_CMU_FSYS0_MMC_EMBD, "mout_fsys0_mmc_embd",
	    mout_fsys0_mmc_embd_p, CLK_CON_MUX_MUX_CLKCMU_FSYS0_MMC_EMBD,
	    0, 3),
	MUX(CLK_MOUT_CMU_FSYS0_UFS_EMBD, "mout_fsys0_ufs_embd",
	    mout_fsys0_ufs_embd_p, CLK_CON_MUX_MUX_CLKCMU_FSYS0_UFS_EMBD,
	    0, 2),
	MUX(CLK_MOUT_CMU_FSYS0_USBDRD30, "mout_fsys0_usbdrd30",
	    mout_fsys0_usbdrd30_p, CLK_CON_MUX_MUX_CLKCMU_FSYS0_USBDRD30,
	    0, 2),
	MUX(CLK_MOUT_CMU_FSYS1_BUS, "mout_cmu_fsys1_bus", mout_cmu_fsys1_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_FSYS1_BUS, 0, 1),
	MUX(CLK_MOUT_CMU_FSYS1_MMC_CARD, "mout_cmu_fsys1_mmc_card",
	    mout_cmu_fsys1_mmc_card_p, CLK_CON_MUX_MUX_CLKCMU_FSYS1_MMC_CARD,
	    0, 3),
	MUX(CLK_MOUT_CMU_FSYS1_PCIE, "mout_cmu_fsys1_pcie",
	    mout_cmu_fsys1_pcie_p, CLK_CON_MUX_MUX_CLKCMU_FSYS1_PCIE, 0, 1),
	MUX(CLK_MOUT_CMU_FSYS1_UFS_CARD, "mout_cmu_fsys1_ufs_card",
	    mout_cmu_fsys1_ufs_card_p, CLK_CON_MUX_MUX_CLKCMU_FSYS1_UFS_CARD,
	    0, 2),
	MUX(CLK_MOUT_CMU_G2D_G2D, "mout_cmu_g2d_g2d", mout_cmu_g2d_g2d_p,
	    CLK_CON_MUX_MUX_CLKCMU_G2D_G2D, 0, 2),
	MUX(CLK_MOUT_CMU_G2D_JPEG, "mout_cmu_g2d_jpeg", mout_cmu_g2d_jpeg_p,
	    CLK_CON_MUX_MUX_CLKCMU_G2D_JPEG, 0, 2),
	MUX(CLK_MOUT_CMU_HPM, "mout_cmu_hpm", mout_cmu_hpm_p,
	    CLK_CON_MUX_MUX_CLKCMU_HPM, 0, 2),
	MUX(CLK_MOUT_CMU_IMEM_BUS, "mout_cmu_imem_bus", mout_cmu_imem_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_IMEM_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_ISPHQ_BUS, "mout_cmu_isphq_bus", mout_cmu_isphq_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_ISPHQ_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_ISPLP_BUS, "mout_cmu_isplp_bus", mout_cmu_isplp_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_ISPLP_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_IVA_BUS, "mout_cmu_iva_bus", mout_cmu_iva_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_IVA_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_MFC_BUS, "mout_cmu_mfc_bus", mout_cmu_mfc_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_MFC_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_MIF_SWITCH, "mout_cmu_mif_switch",
	    mout_cmu_mif_switch_p, CLK_CON_MUX_MUX_CLKCMU_MIF_SWITCH, 0, 3),
	MUX(CLK_MOUT_CMU_PERIC0_BUS, "mout_cmu_peric0_bus",
	    mout_cmu_peric0_bus_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_BUS, 0, 1),
	MUX(CLK_MOUT_CMU_PERIC0_UART_DBG, "mout_cmu_peric0_uart_dbg",
	    mout_cmu_peric0_uart_dbg_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_UART_DBG,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC0_USI00, "mout_cmu_peric0_usi00",
	    mout_cmu_peric0_usi00_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI00,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC0_USI01, "mout_cmu_peric0_usi01",
	    mout_cmu_peric0_usi01_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI01,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC0_USI02, "mout_cmu_peric0_usi02",
	    mout_cmu_peric0_usi02_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI02,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC0_USI03, "mout_cmu_peric0_usi03",
	    mout_cmu_peric0_usi03_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_USI03,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_BUS, "mout_cmu_peric1_bus",
	    mout_cmu_peric1_bus_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_BUS, 0, 1),
	MUX(CLK_MOUT_CMU_PERIC1_SPEEDY2, "mout_cmu_peric1_speedy2",
	    mout_cmu_peric1_speedy2_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_SPEEDY2,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_SPI_CAM0, "mout_cmu_peric1_spi_cam0",
	    mout_cmu_peric1_spi_cam0_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_SPI_CAM0,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_SPI_CAM1, "mout_cmu_peric1_spi_cam1",
	    mout_cmu_peric1_spi_cam1_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_SPI_CAM1,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_UART_BT, "mout_cmu_peric1_uart_bt",
	    mout_cmu_peric1_uart_bt_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_UART_BT,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI04, "mout_cmu_peric1_usi04",
	    mout_cmu_peric1_usi04_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI04,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI05, "mout_cmu_peric1_usi05",
	    mout_cmu_peric1_usi05_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI05,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI06, "mout_cmu_peric1_usi06",
	    mout_cmu_peric1_usi06_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI06,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI07, "mout_cmu_peric1_usi07",
	    mout_cmu_peric1_usi07_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI07,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI08, "mout_cmu_peric1_usi08",
	    mout_cmu_peric1_usi08_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI08,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI09, "mout_cmu_peric1_usi09",
	    mout_cmu_peric1_usi09_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI09,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI10, "mout_cmu_peric1_usi10",
	    mout_cmu_peric1_usi10_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI10,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI11, "mout_cmu_peric1_usi11",
	    mout_cmu_peric1_usi11_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI11,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI12, "mout_cmu_peric1_usi12",
	    mout_cmu_peric1_usi12_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI12,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIC1_USI13, "mout_cmu_peric1_usi13",
	    mout_cmu_peric1_usi13_p, CLK_CON_MUX_MUX_CLKCMU_PERIC1_USI13,
	    0, 2),
	MUX(CLK_MOUT_CMU_PERIS_BUS, "mout_cmu_peris_bus", mout_cmu_peris_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERIS_BUS, 0, 1),
	MUX(CLK_MOUT_CMU_SRDZ_BUS, "mout_cmu_srdz_bus", mout_cmu_srdz_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_SRDZ_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_SRDZ_IMGD, "mout_cmu_srdz_imgd", mout_cmu_srdz_imgd_p,
	    CLK_CON_MUX_MUX_CLKCMU_SRDZ_IMGD, 0, 2),
	MUX(CLK_MOUT_CMU_VPU_BUS, "mout_cmu_vpu_bus", mout_cmu_vpu_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_VPU_BUS, 0, 2),
};

static const struct samsung_div_clock top_div_clks[] __initconst = {
	DIV(CLK_DOUT_CMU_ABOX_CPUABOX, "dout_cmu_cmu_abox_cpuabox",
	    "gout_cmu_abox_cpuabox", CLK_CON_DIV_CLKCMU_ABOX_CPUABOX, 0, 3),
	DIV(CLK_DOUT_CMU_APM_BUS, "dout_cmu_apm_bus", "gout_cmu_apm_bus",
	    CLK_CON_DIV_CLKCMU_APM_BUS, 0, 3),
	DIV(CLK_DOUT_CMU_BUS1_BUS, "dout_cmu_bus1_bus", "gout_cmu_bus1_bus",
	    CLK_CON_DIV_CLKCMU_BUS1_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_BUSC_BUS, "dout_cmu_busc_bus",
	    "gout_cmu_busc_bus", CLK_CON_DIV_CLKCMU_BUSC_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_BUSC_BUSPHSI2C, "dout_cmu_busc_busphsi2c",
	    "gout_cmu_busc_busphsi2c", CLK_CON_DIV_CLKCMU_BUSC_BUSPHSI2C,
	    0, 4),
	DIV(CLK_DOUT_CMU_CAM_BUS, "dout_cmu_cam_bus", "gout_cmu_cam_bus",
	    CLK_CON_DIV_CLKCMU_CAM_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_CAM_TPU0, "dout_cmu_cam_tpu0", "gout_cmu_cam_tpu0",
	    CLK_CON_DIV_CLKCMU_CAM_TPU0, 0, 4),
	DIV(CLK_DOUT_CMU_CAM_TPU1, "dout_cmu_cam_tpu1", "gout_cmu_cam_tpu1",
	    CLK_CON_DIV_CLKCMU_CAM_TPU1, 0, 4),
	DIV(CLK_DOUT_CMU_CAM_VRA, "dout_cmu_cam_vra", "gout_cmu_cam_vra",
	    CLK_CON_DIV_CLKCMU_CAM_VRA, 0, 4),
	DIV(CLK_DOUT_CMU_CIS_CLK0, "dout_cmu_cis_clk0", "gout_cmu_cis_clk0",
	    CLK_CON_DIV_CLKCMU_CIS_CLK0, 0, 5),
	DIV(CLK_DOUT_CMU_CIS_CLK1, "dout_cmu_cis_clk1", "gout_cmu_cis_clk1",
	    CLK_CON_DIV_CLKCMU_CIS_CLK1, 0, 5),
	DIV(CLK_DOUT_CMU_CIS_CLK2, "dout_cmu_cis_clk2", "gout_cmu_cis_clk2",
	    CLK_CON_DIV_CLKCMU_CIS_CLK2, 0, 5),
	DIV(CLK_DOUT_CMU_CIS_CLK3, "dout_cmu_cis_clk3", "gout_cmu_cis_clk3",
	    CLK_CON_DIV_CLKCMU_CIS_CLK3, 0, 5),
	DIV(CLK_DOUT_CMU_CORE_BUS, "dout_cmu_core_bus", "gout_cmu_core_bus",
	    CLK_CON_DIV_CLKCMU_CORE_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_CPUCL0_SWITCH, "dout_cmu_cpucl0_switch",
	    "gout_cmu_cpucl0_switch", CLK_CON_DIV_CLKCMU_CPUCL0_SWITCH, 0, 3),
	DIV(CLK_DOUT_CMU_CPUCL1_SWITCH, "dout_cmu_cpucl1_switch",
	    "gout_cmu_cpucl1_switch", CLK_CON_DIV_CLKCMU_CPUCL1_SWITCH, 0, 3),
	DIV(CLK_DOUT_CMU_DBG_BUS, "dout_cmu_dbg_bus", "gout_cmu_dbg_bus",
	    CLK_CON_DIV_CLKCMU_DBG_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_DCAM_BUS, "dout_cmu_dcam_bus", "gout_cmu_dcam_bus",
	    CLK_CON_DIV_CLKCMU_DCAM_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_DCAM_IMGD, "dout_cmu_dcam_imgd", "gout_cmu_dcam_imgd",
	    CLK_CON_DIV_CLKCMU_DCAM_IMGD, 0, 4),
	DIV(CLK_DOUT_CMU_DPU_BUS, "dout_cmu_dpu_bus", "gout_cmu_dpu_bus",
	    CLK_CON_DIV_CLKCMU_DPU_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_DSP_BUS, "dout_cmu_dsp_bus", "gout_cmu_dsp_bus",
	    CLK_CON_DIV_CLKCMU_DSP_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_FSYS0_BUS, "dout_cmu_fsys0_bus", "gout_cmu_fsys0_bus",
	    CLK_CON_DIV_CLKCMU_FSYS0_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_FSYS0_DPGTC, "dout_cmu_fsys0_dpgtc",
	    "gout_cmu_fsys0_dpgtc", CLK_CON_DIV_CLKCMU_FSYS0_DPGTC, 0, 3),
	DIV(CLK_DOUT_CMU_FSYS0_MMC_EMBD, "dout_cmu_fsys0_mmc_embd",
	    "gout_cmu_fsys0_mmc_embd", CLK_CON_DIV_CLKCMU_FSYS0_MMC_EMBD,
	    0, 9),
	DIV(CLK_DOUT_CMU_FSYS0_UFS_EMBD, "dout_cmu_fsys0_ufs_embd",
	    "gout_cmu_fsys0_ufs_embd", CLK_CON_DIV_CLKCMU_FSYS0_UFS_EMBD,
	    0, 3),
	DIV(CLK_DOUT_CMU_FSYS0_USBDRD30, "dout_cmu_fsys0_usbdrd30",
	    "gout_cmu_fsys0_usbdrd30", CLK_CON_DIV_CLKCMU_FSYS0_USBDRD30,
	    0, 4),
	DIV(CLK_DOUT_CMU_FSYS1_BUS, "dout_cmu_fsys1_bus", "gout_cmu_fsys1_bus",
	    CLK_CON_DIV_CLKCMU_FSYS1_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_FSYS1_MMC_CARD, "dout_cmu_fsys1_mmc_card",
	    "gout_cmu_fsys1_mmc_card", CLK_CON_DIV_CLKCMU_FSYS1_MMC_CARD,
	    0, 9),
	DIV(CLK_DOUT_CMU_FSYS1_UFS_CARD, "dout_cmu_fsys1_ufs_card",
	    "gout_cmu_fsys1_ufs_card", CLK_CON_DIV_CLKCMU_FSYS1_UFS_CARD,
	    0, 4),
	DIV(CLK_DOUT_CMU_G2D_G2D, "dout_cmu_g2d_g2d", "gout_cmu_g2d_g2d",
	    CLK_CON_DIV_CLKCMU_G2D_G2D, 0, 4),
	DIV(CLK_DOUT_CMU_G2D_JPEG, "dout_cmu_g2d_jpeg", "gout_cmu_g2d_jpeg",
	    CLK_CON_DIV_CLKCMU_G2D_JPEG, 0, 4),
	DIV(CLK_DOUT_CMU_G3D_SWITCH, "dout_cmu_g3d_switch",
	    "gout_cmu_g3d_switch", CLK_CON_DIV_CLKCMU_G3D_SWITCH, 0, 3),
	DIV(CLK_DOUT_CMU_HPM, "dout_cmu_hpm", "gout_cmu_hpm",
	    CLK_CON_DIV_CLKCMU_HPM, 0, 2),
	DIV(CLK_DOUT_CMU_IMEM_BUS, "dout_cmu_imem_bus", "gout_cmu_imem_bus",
	    CLK_CON_DIV_CLKCMU_IMEM_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_ISPHQ_BUS, "dout_cmu_isphq_bus", "gout_cmu_isphq_bus",
	    CLK_CON_DIV_CLKCMU_ISPHQ_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_ISPLP_BUS, "dout_cmu_isplp_bus", "gout_cmu_isplp_bus",
	    CLK_CON_DIV_CLKCMU_ISPLP_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_IVA_BUS, "dout_cmu_iva_bus", "gout_cmu_iva_bus",
	    CLK_CON_DIV_CLKCMU_IVA_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_MFC_BUS, "dout_cmu_mfc_bus", "gout_cmu_mfc_bus",
	    CLK_CON_DIV_CLKCMU_MFC_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_MODEM_SHARED0, "dout_cmu_modem_shared0",
	    "gout_cmu_modem_shared0", CLK_CON_DIV_CLKCMU_MODEM_SHARED0, 0, 3),
	DIV(CLK_DOUT_CMU_MODEM_SHARED1, "dout_cmu_modem_shared1",
	    "gout_cmu_modem_shared1", CLK_CON_DIV_CLKCMU_MODEM_SHARED1, 0, 3),
	DIV(CLK_DOUT_CMU_PERIC0_BUS, "dout_cmu_peric0_bus",
	    "gout_cmu_peric0_bus", CLK_CON_DIV_CLKCMU_PERIC0_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC0_UART_DBG, "dout_cmu_peric0_uart_dbg",
	    "gout_cmu_peric0_uart_dbg", CLK_CON_DIV_CLKCMU_PERIC0_UART_DBG,
	    0, 4),
	DIV(CLK_DOUT_CMU_PERIC0_USI00, "dout_cmu_peric0_usi00",
	    "gout_cmu_peric0_usi00", CLK_CON_DIV_CLKCMU_PERIC0_USI00, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC0_USI01, "dout_cmu_peric0_usi01",
	    "gout_cmu_peric0_usi01", CLK_CON_DIV_CLKCMU_PERIC0_USI01, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC0_USI02, "dout_cmu_peric0_usi02",
	    "gout_cmu_peric0_usi02", CLK_CON_DIV_CLKCMU_PERIC0_USI02, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC0_USI03, "dout_cmu_peric0_usi03",
	    "gout_cmu_peric0_usi03", CLK_CON_DIV_CLKCMU_PERIC0_USI03, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_BUS, "dout_cmu_peric1_bus",
	    "gout_cmu_peric1_bus", CLK_CON_DIV_CLKCMU_PERIC1_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_SPEEDY2, "dout_cmu_peric1_speedy2",
	    "gout_cmu_peric1_speedy2", CLK_CON_DIV_CLKCMU_PERIC1_SPEEDY2,
	    0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_SPI_CAM0, "dout_cmu_peric1_spi_cam0",
	    "gout_cmu_peric1_spi_cam0", CLK_CON_DIV_CLKCMU_PERIC1_SPI_CAM0,
	    0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_SPI_CAM1, "dout_cmu_peric1_spi_cam1",
	    "gout_cmu_peric1_spi_cam1", CLK_CON_DIV_CLKCMU_PERIC1_SPI_CAM1,
	    0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_UART_BT, "dout_cmu_peric1_uart_bt",
	    "gout_cmu_peric1_uart_bt", CLK_CON_DIV_CLKCMU_PERIC1_UART_BT,
	    0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI04, "dout_cmu_peric1_usi04",
	    "gout_cmu_peric1_usi04", CLK_CON_DIV_CLKCMU_PERIC1_USI04, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI05, "dout_cmu_peric1_usi05",
	    "gout_cmu_peric1_usi05", CLK_CON_DIV_CLKCMU_PERIC1_USI05, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI06, "dout_cmu_peric1_usi06",
	    "gout_cmu_peric1_usi06", CLK_CON_DIV_CLKCMU_PERIC1_USI06, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI07, "dout_cmu_peric1_usi07",
	    "gout_cmu_peric1_usi07", CLK_CON_DIV_CLKCMU_PERIC1_USI07, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI08, "dout_cmu_peric1_usi08",
	    "gout_cmu_peric1_usi08", CLK_CON_DIV_CLKCMU_PERIC1_USI08, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI09, "dout_cmu_peric1_usi09",
	    "gout_cmu_peric1_usi09", CLK_CON_DIV_CLKCMU_PERIC1_USI09, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI10, "dout_cmu_peric1_usi10",
	    "gout_cmu_peric1_usi10", CLK_CON_DIV_CLKCMU_PERIC1_USI10, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI11, "dout_cmu_peric1_usi11",
	    "gout_cmu_peric1_usi11", CLK_CON_DIV_CLKCMU_PERIC1_USI11, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI12, "dout_cmu_peric1_usi12",
	    "gout_cmu_peric1_usi12", CLK_CON_DIV_CLKCMU_PERIC1_USI12, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC1_USI13, "dout_cmu_peric1_usi13",
	    "gout_cmu_peric1_usi13", CLK_CON_DIV_CLKCMU_PERIC1_USI13, 0, 4),
	DIV(CLK_DOUT_CMU_PERIS_BUS, "dout_cmu_peris_bus", "gout_cmu_peris_bus",
	    CLK_CON_DIV_CLKCMU_PERIS_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_SRDZ_BUS, "dout_cmu_srdz_bus", "gout_cmu_srdz_bus",
	    CLK_CON_DIV_CLKCMU_SRDZ_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_SRDZ_IMGD, "dout_cmu_srdz_imgd", "gout_cmu_srdz_imgd",
	    CLK_CON_DIV_CLKCMU_SRDZ_IMGD, 0, 4),
	DIV(CLK_DOUT_CMU_VPU_BUS, "dout_cmu_vpu_bus", "gout_cmu_vpu_bus",
	    CLK_CON_DIV_CLKCMU_VPU_BUS, 0, 4),
};

/* MIF clock from the modem: nominal rate, the modem sets the real one */
static const struct samsung_fixed_rate_clock top_fixed_clks[] __initconst = {
	FRATE(0, "cp2ap_mif_clk", NULL, 0, 1066000000),
};

static const struct samsung_fixed_factor_clock top_fixed_factor_clks[] __initconst = {
	FFACTOR(CLK_DOUT_CMU_SHARED0_DIV2, "dout_cmu_shared0_div2",
		"mout_pll_shared0", 1, 2, 0),
	FFACTOR(CLK_DOUT_CMU_SHARED0_DIV4, "dout_cmu_shared0_div4",
		"mout_pll_shared0", 1, 4, 0),
	FFACTOR(CLK_DOUT_CMU_SHARED1_DIV2, "dout_cmu_shared1_div2",
		"mout_pll_shared1", 1, 2, 0),
	FFACTOR(CLK_DOUT_CMU_SHARED1_DIV4, "dout_cmu_shared1_div4",
		"mout_pll_shared1", 1, 4, 0),
	FFACTOR(CLK_DOUT_CMU_SHARED2_DIV2, "dout_cmu_shared2_div2",
		"mout_pll_shared2", 1, 2, 0),
	FFACTOR(CLK_DOUT_CMU_SHARED3_DIV2, "dout_cmu_shared3_div2",
		"mout_pll_shared3", 1, 2, 0),
	FFACTOR(CLK_DOUT_CMU_SHARED4_DIV2, "dout_cmu_shared4_div2",
		"mout_pll_shared4", 1, 2, 0),
	FFACTOR(CLK_DOUT_CMU_FSYS1_PCIE, "dout_cmu_fsys1_pcie",
		"gout_cmu_fsys1_pcie", 1, 8, 0),
	FFACTOR(CLK_DOUT_CMU_CP2AP_MIF_CLK_DIV2, "dout_cmu_cp2ap_mif_clk_div2",
		"mout_cp2ap_mif_clk_user", 1, 2, 0),
	FFACTOR(CLK_DOUT_CMU_CMU_OTP, "dout_cmu_cmu_otp", "oscclk", 1, 8, 0),
};

static const struct samsung_gate_clock top_gate_clks[] __initconst = {
	GATE(CLK_GOUT_CMU_DROOPDETECTOR, "gout_droopdetector",
	     "mout_cmu_droopdetector", CLK_CON_GAT_CLKCMU_DROOPDETECTOR,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_MIF_SWITCH, "gout_cmu_mif_switch",
	     "mout_cmu_mif_switch", CLK_CON_GAT_CLKCMU_MIF_SWITCH, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_ABOX_CPUABOX, "gout_cmu_abox_cpuabox",
	     "mout_cmu_abox_cpuabox", CLK_CON_GAT_GATE_CLKCMU_ABOX_CPUABOX,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_APM_BUS, "gout_cmu_apm_bus", "mout_cmu_apm_bus",
	     CLK_CON_GAT_GATE_CLKCMU_APM_BUS, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_BUS1_BUS, "gout_cmu_bus1_bus", "mout_cmu_bus1_bus",
	     CLK_CON_GAT_GATE_CLKCMU_BUS1_BUS, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_BUSC_BUS, "gout_cmu_busc_bus", "mout_cmu_busc_bus",
	     CLK_CON_GAT_GATE_CLKCMU_BUSC_BUS, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_BUSC_BUSPHSI2C, "gout_cmu_busc_busphsi2c",
	     "mout_cmu_busc_busphsi2c", CLK_CON_GAT_GATE_CLKCMU_BUSC_BUSPHSI2C,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_CAM_BUS, "gout_cmu_cam_bus", "mout_cmu_cam_bus",
	     CLK_CON_GAT_GATE_CLKCMU_CAM_BUS, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_CAM_TPU0, "gout_cmu_cam_tpu0", "mout_cmu_cam_tpu0",
	     CLK_CON_GAT_GATE_CLKCMU_CAM_TPU0, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_CAM_TPU1, "gout_cmu_cam_tpu1", "mout_cmu_cam_tpu1",
	     CLK_CON_GAT_GATE_CLKCMU_CAM_TPU1, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_CAM_VRA, "gout_cmu_cam_vra", "mout_cmu_cam_vra",
	     CLK_CON_GAT_GATE_CLKCMU_CAM_VRA, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_CIS_CLK0, "gout_cmu_cis_clk0", "mout_cmu_cis_clk0",
	     CLK_CON_GAT_GATE_CLKCMU_CIS_CLK0, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_CIS_CLK1, "gout_cmu_cis_clk1", "mout_cmu_cis_clk1",
	     CLK_CON_GAT_GATE_CLKCMU_CIS_CLK1, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_CIS_CLK2, "gout_cmu_cis_clk2", "mout_cmu_cis_clk2",
	     CLK_CON_GAT_GATE_CLKCMU_CIS_CLK2, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_CIS_CLK3, "gout_cmu_cis_clk3", "mout_cmu_cis_clk3",
	     CLK_CON_GAT_GATE_CLKCMU_CIS_CLK3, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_CORE_BUS, "gout_cmu_core_bus", "mout_core_bus",
	     CLK_CON_GAT_GATE_CLKCMU_CORE_BUS, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_CPUCL0_SWITCH, "gout_cmu_cpucl0_switch",
	     "mout_cmu_cpucl0_switch", CLK_CON_GAT_GATE_CLKCMU_CPUCL0_SWITCH,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_CPUCL1_SWITCH, "gout_cmu_cpucl1_switch",
	     "mout_cmu_cpucl1_switch", CLK_CON_GAT_GATE_CLKCMU_CPUCL1_SWITCH,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_DBG_BUS, "gout_cmu_dbg_bus", "mout_cmu_dbg_bus",
	     CLK_CON_GAT_GATE_CLKCMU_DBG_BUS, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_DCAM_BUS, "gout_cmu_dcam_bus", "mout_cmu_dcam_bus",
	     CLK_CON_GAT_GATE_CLKCMU_DCAM_BUS, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_DCAM_IMGD, "gout_cmu_dcam_imgd",
	     "mout_cmu_dcam_imgd", CLK_CON_GAT_GATE_CLKCMU_DCAM_IMGD,
	     21, CLK_IGNORE_UNUSED, 0),
	/*
	 * CMU_DPU0 runtime-suspends right after it registers and drops its bus
	 * clock: keep this feed counted on while the boot splash is scanned out.
	 */
	GATE(CLK_GOUT_CMU_DPU_BUS, "gout_cmu_dpu_bus", "mout_cmu_dpu_bus",
	     CLK_CON_GAT_GATE_CLKCMU_DPU_BUS, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_DSP_BUS, "gout_cmu_dsp_bus", "mout_cmu_dsp_bus",
	     CLK_CON_GAT_GATE_CLKCMU_DSP_BUS, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_FSYS0_BUS, "gout_cmu_fsys0_bus", "mout_fsys0_bus",
	     CLK_CON_GAT_GATE_CLKCMU_FSYS0_BUS, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_FSYS0_DPGTC, "gout_cmu_fsys0_dpgtc",
	     "mout_fsys0_dpgtc", CLK_CON_GAT_GATE_CLKCMU_FSYS0_DPGTC,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_FSYS0_MMC_EMBD, "gout_cmu_fsys0_mmc_embd",
	     "mout_fsys0_mmc_embd", CLK_CON_GAT_GATE_CLKCMU_FSYS0_MMC_EMBD,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_FSYS0_UFS_EMBD, "gout_cmu_fsys0_ufs_embd",
	     "mout_fsys0_ufs_embd", CLK_CON_GAT_GATE_CLKCMU_FSYS0_UFS_EMBD,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_FSYS0_USBDRD30, "gout_cmu_fsys0_usbdrd30",
	     "mout_fsys0_usbdrd30", CLK_CON_GAT_GATE_CLKCMU_FSYS0_USBDRD30,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_FSYS1_BUS, "gout_cmu_fsys1_bus",
	     "mout_cmu_fsys1_bus", CLK_CON_GAT_GATE_CLKCMU_FSYS1_BUS,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_FSYS1_MMC_CARD, "gout_cmu_fsys1_mmc_card",
	     "mout_cmu_fsys1_mmc_card", CLK_CON_GAT_GATE_CLKCMU_FSYS1_MMC_CARD,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_FSYS1_PCIE, "gout_cmu_fsys1_pcie",
	     "mout_cmu_fsys1_pcie", CLK_CON_GAT_GATE_CLKCMU_FSYS1_PCIE,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_FSYS1_UFS_CARD, "gout_cmu_fsys1_ufs_card",
	     "mout_cmu_fsys1_ufs_card", CLK_CON_GAT_GATE_CLKCMU_FSYS1_UFS_CARD,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_G2D_G2D, "gout_cmu_g2d_g2d", "mout_cmu_g2d_g2d",
	     CLK_CON_GAT_GATE_CLKCMU_G2D_G2D, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_G2D_JPEG, "gout_cmu_g2d_jpeg", "mout_cmu_g2d_jpeg",
	     CLK_CON_GAT_GATE_CLKCMU_G2D_JPEG, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_G3D_SWITCH, "gout_cmu_g3d_switch",
	     "fout_shared2_pll", CLK_CON_GAT_GATE_CLKCMU_G3D_SWITCH, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_HPM, "gout_cmu_hpm", "mout_cmu_hpm",
	     CLK_CON_GAT_GATE_CLKCMU_HPM, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_IMEM_BUS, "gout_cmu_imem_bus", "mout_cmu_imem_bus",
	     CLK_CON_GAT_GATE_CLKCMU_IMEM_BUS, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_ISPHQ_BUS, "gout_cmu_isphq_bus",
	     "mout_cmu_isphq_bus", CLK_CON_GAT_GATE_CLKCMU_ISPHQ_BUS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_ISPLP_BUS, "gout_cmu_isplp_bus",
	     "mout_cmu_isplp_bus", CLK_CON_GAT_GATE_CLKCMU_ISPLP_BUS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_IVA_BUS, "gout_cmu_iva_bus", "mout_cmu_iva_bus",
	     CLK_CON_GAT_GATE_CLKCMU_IVA_BUS, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_MFC_BUS, "gout_cmu_mfc_bus", "mout_cmu_mfc_bus",
	     CLK_CON_GAT_GATE_CLKCMU_MFC_BUS, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_MODEM_SHARED0, "gout_cmu_modem_shared0",
	     "dout_cmu_shared0_div2", CLK_CON_GAT_GATE_CLKCMU_MODEM_SHARED0,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_MODEM_SHARED1, "gout_cmu_modem_shared1",
	     "fout_shared2_pll", CLK_CON_GAT_GATE_CLKCMU_MODEM_SHARED1,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_PERIC0_BUS, "gout_cmu_peric0_bus",
	     "mout_cmu_peric0_bus", CLK_CON_GAT_GATE_CLKCMU_PERIC0_BUS,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_PERIC0_UART_DBG, "gout_cmu_peric0_uart_dbg",
	     "mout_cmu_peric0_uart_dbg",
	     CLK_CON_GAT_GATE_CLKCMU_PERIC0_UART_DBG, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_PERIC0_USI00, "gout_cmu_peric0_usi00",
	     "mout_cmu_peric0_usi00", CLK_CON_GAT_GATE_CLKCMU_PERIC0_USI00,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC0_USI01, "gout_cmu_peric0_usi01",
	     "mout_cmu_peric0_usi01", CLK_CON_GAT_GATE_CLKCMU_PERIC0_USI01,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC0_USI02, "gout_cmu_peric0_usi02",
	     "mout_cmu_peric0_usi02", CLK_CON_GAT_GATE_CLKCMU_PERIC0_USI02,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC0_USI03, "gout_cmu_peric0_usi03",
	     "mout_cmu_peric0_usi03", CLK_CON_GAT_GATE_CLKCMU_PERIC0_USI03,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_BUS, "gout_cmu_peric1_bus",
	     "mout_cmu_peric1_bus", CLK_CON_GAT_GATE_CLKCMU_PERIC1_BUS,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_PERIC1_SPEEDY2, "gout_cmu_peric1_speedy2",
	     "mout_cmu_peric1_speedy2", CLK_CON_GAT_GATE_CLKCMU_PERIC1_SPEEDY2,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_SPI_CAM0, "gout_cmu_peric1_spi_cam0",
	     "mout_cmu_peric1_spi_cam0",
	     CLK_CON_GAT_GATE_CLKCMU_PERIC1_SPI_CAM0, 21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_SPI_CAM1, "gout_cmu_peric1_spi_cam1",
	     "mout_cmu_peric1_spi_cam1",
	     CLK_CON_GAT_GATE_CLKCMU_PERIC1_SPI_CAM1, 21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_UART_BT, "gout_cmu_peric1_uart_bt",
	     "mout_cmu_peric1_uart_bt", CLK_CON_GAT_GATE_CLKCMU_PERIC1_UART_BT,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI04, "gout_cmu_peric1_usi04",
	     "mout_cmu_peric1_usi04", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI04,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI05, "gout_cmu_peric1_usi05",
	     "mout_cmu_peric1_usi05", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI05,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI06, "gout_cmu_peric1_usi06",
	     "mout_cmu_peric1_usi06", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI06,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI07, "gout_cmu_peric1_usi07",
	     "mout_cmu_peric1_usi07", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI07,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI08, "gout_cmu_peric1_usi08",
	     "mout_cmu_peric1_usi08", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI08,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI09, "gout_cmu_peric1_usi09",
	     "mout_cmu_peric1_usi09", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI09,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI10, "gout_cmu_peric1_usi10",
	     "mout_cmu_peric1_usi10", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI10,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI11, "gout_cmu_peric1_usi11",
	     "mout_cmu_peric1_usi11", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI11,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI12, "gout_cmu_peric1_usi12",
	     "mout_cmu_peric1_usi12", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI12,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIC1_USI13, "gout_cmu_peric1_usi13",
	     "mout_cmu_peric1_usi13", CLK_CON_GAT_GATE_CLKCMU_PERIC1_USI13,
	     21, 0, 0),
	GATE(CLK_GOUT_CMU_PERIS_BUS, "gout_cmu_peris_bus",
	     "mout_cmu_peris_bus", CLK_CON_GAT_GATE_CLKCMU_PERIS_BUS,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_SRDZ_BUS, "gout_cmu_srdz_bus", "mout_cmu_srdz_bus",
	     CLK_CON_GAT_GATE_CLKCMU_SRDZ_BUS, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_SRDZ_IMGD, "gout_cmu_srdz_imgd",
	     "mout_cmu_srdz_imgd", CLK_CON_GAT_GATE_CLKCMU_SRDZ_IMGD,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CMU_VPU_BUS, "gout_cmu_vpu_bus", "mout_cmu_vpu_bus",
	     CLK_CON_GAT_GATE_CLKCMU_VPU_BUS, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info top_cmu_info __initconst = {
	.pll_clks		= top_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(top_pll_clks),
	.mux_clks		= top_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(top_mux_clks),
	.div_clks		= top_div_clks,
	.nr_div_clks		= ARRAY_SIZE(top_div_clks),
	.fixed_clks		= top_fixed_clks,
	.nr_fixed_clks		= ARRAY_SIZE(top_fixed_clks),
	.fixed_factor_clks	= top_fixed_factor_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(top_fixed_factor_clks),
	.gate_clks		= top_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(top_gate_clks),
	.nr_clk_ids		= CLKS_NR_TOP,
	.clk_regs		= top_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(top_clk_regs),
	.hwacg_gates		= true,
};

static void __init exynos8895_cmu_top_init(struct device_node *np)
{
	exynos_arm64_register_cmu(NULL, np, &top_cmu_info);
}

/* Register CMU_TOP early, as it's a dependency for other early domains */
CLK_OF_DECLARE(exynos8895_cmu_top, "samsung,exynos8895-cmu-top",
	       exynos8895_cmu_top_init);

/* ---- CMU_PERIS ----------------------------------------------------------- */

/* Register Offset definitions for CMU_PERIS (0x10010000) */
#define PLL_CON0_MUX_CLKCMU_PERIS_BUS_USER					0x0100
#define PLL_CON2_MUX_CLKCMU_PERIS_BUS_USER					0x0108
#define CLK_CON_MUX_MUX_CLK_PERIS_GIC						0x1000
#define CLK_CON_GAT_CLK_BLK_PERIS_UID_PERIS_CMU_PERIS_IPCLKPORT_PCLK		0x2000
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_AD_AXI_P_PERIS_IPCLKPORT_ACLKM		0x2010
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_AD_AXI_P_PERIS_IPCLKPORT_ACLKS		0x2014
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_AXI2APB_PERISP0_IPCLKPORT_ACLK		0x2018
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_AXI2APB_PERISP1_IPCLKPORT_ACLK		0x201c
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_BUSIF_TMU_IPCLKPORT_PCLK			0x2020
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_GIC_IPCLKPORT_CLK			0x2024
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_LHM_AXI_P_PERIS_IPCLKPORT_I_CLK		0x2028
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_MCT_IPCLKPORT_PCLK			0x202c
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_OTP_CON_BIRA_IPCLKPORT_PCLK		0x2030
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_OTP_CON_TOP_IPCLKPORT_PCLK		0x2034
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_PMU_PERIS_IPCLKPORT_PCLK			0x2038
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_RSTNSYNC_CLK_PERIS_BUSP_IPCLKPORT_CLK	0x203c
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_RSTNSYNC_CLK_PERIS_GIC_IPCLKPORT_CLK	0x2040
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_SYSREG_PERIS_IPCLKPORT_PCLK		0x2044
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC00_IPCLKPORT_PCLK			0x2048
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC01_IPCLKPORT_PCLK			0x204c
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC02_IPCLKPORT_PCLK			0x2050
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC03_IPCLKPORT_PCLK			0x2054
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC04_IPCLKPORT_PCLK			0x2058
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC05_IPCLKPORT_PCLK			0x205c
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC06_IPCLKPORT_PCLK			0x2060
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC07_IPCLKPORT_PCLK			0x2064
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC08_IPCLKPORT_PCLK			0x2068
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC09_IPCLKPORT_PCLK			0x206c
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC10_IPCLKPORT_PCLK			0x2070
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC11_IPCLKPORT_PCLK			0x2074
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC12_IPCLKPORT_PCLK			0x2078
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC13_IPCLKPORT_PCLK			0x207c
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC14_IPCLKPORT_PCLK			0x2080
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC15_IPCLKPORT_PCLK			0x2084
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_WDT_CLUSTER0_IPCLKPORT_PCLK		0x2088
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_WDT_CLUSTER1_IPCLKPORT_PCLK		0x208c
#define CLK_CON_GAT_GOUT_BLK_PERIS_UID_XIU_P_PERIS_IPCLKPORT_ACLK		0x2090

static const unsigned long peris_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_PERIS_BUS_USER,
	PLL_CON2_MUX_CLKCMU_PERIS_BUS_USER,
	CLK_CON_MUX_MUX_CLK_PERIS_GIC,
};

/* List of parent clocks for Muxes in CMU_PERIS */
PNAME(mout_peris_bus_user_p)	= { "oscclk", "dout_cmu_peris_bus" };
PNAME(mout_peris_gic_p)		= { "mout_peris_bus_user", "oscclk" };

static const struct samsung_mux_clock peris_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PERIS_BUS_USER, "mout_peris_bus_user",
	    mout_peris_bus_user_p, PLL_CON0_MUX_CLKCMU_PERIS_BUS_USER, 4, 1),
	MUX(CLK_MOUT_PERIS_GIC, "mout_peris_gic",
	    mout_peris_gic_p, CLK_CON_MUX_MUX_CLK_PERIS_GIC, 0, 5),
};

static const struct samsung_gate_clock peris_gate_clks[] __initconst = {
	GATE(CLK_GOUT_PERIS_CMU_PERIS_PCLK, "gout_peris_cmu_peris_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_CLK_BLK_PERIS_UID_PERIS_CMU_PERIS_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_AD_AXI_P_PERIS_ACLKM,
	     "gout_peris_ad_axi_p_peris_aclkm", "mout_peris_gic",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_AD_AXI_P_PERIS_IPCLKPORT_ACLKM,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_AD_AXI_P_PERIS_ACLKS,
	     "gout_peris_ad_axi_p_peris_aclks", "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_AD_AXI_P_PERIS_IPCLKPORT_ACLKS,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_AXI2APB_PERISP0_ACLK,
	     "gout_peris_axi2apb_perisp0_aclk", "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_AXI2APB_PERISP0_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_AXI2APB_PERISP1_ACLK,
	     "gout_peris_axi2apb_perisp1_aclk", "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_AXI2APB_PERISP1_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_BUSIF_TMU_PCLK, "gout_peris_busif_tmu_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_BUSIF_TMU_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	/* GIC (interrupt controller) clock must be always running */
	GATE(CLK_GOUT_PERIS_GIC_CLK, "gout_peris_gic_clk",
	     "mout_peris_gic",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_GIC_IPCLKPORT_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_LHM_AXI_P_PERIS_I_CLK,
	     "gout_peris_lhm_axi_p_peris_i_clk", "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_LHM_AXI_P_PERIS_IPCLKPORT_I_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_MCT_PCLK, "gout_peris_mct_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_MCT_IPCLKPORT_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIS_OTP_CON_BIRA_PCLK, "gout_peris_otp_con_bira_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_OTP_CON_BIRA_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_OTP_CON_TOP_PCLK, "gout_peris_otp_con_top_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_OTP_CON_TOP_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_PMU_PERIS_PCLK, "gout_peris_pmu_peris_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_PMU_PERIS_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_RSTNSYNC_CLK_PERIS_BUSP_CLK,
	     "gout_peris_rstnsync_clk_peris_busp_clk", "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_RSTNSYNC_CLK_PERIS_BUSP_IPCLKPORT_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_RSTNSYNC_CLK_PERIS_GIC_CLK,
	     "gout_peris_rstnsync_clk_peris_gic_clk", "mout_peris_gic",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_RSTNSYNC_CLK_PERIS_GIC_IPCLKPORT_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_SYSREG_PERIS_PCLK, "gout_peris_sysreg_peris_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_SYSREG_PERIS_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC00_PCLK, "gout_peris_tzpc00_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC00_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC01_PCLK, "gout_peris_tzpc01_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC01_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC02_PCLK, "gout_peris_tzpc02_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC02_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC03_PCLK, "gout_peris_tzpc03_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC03_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC04_PCLK, "gout_peris_tzpc04_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC04_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC05_PCLK, "gout_peris_tzpc05_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC05_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC06_PCLK, "gout_peris_tzpc06_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC06_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC07_PCLK, "gout_peris_tzpc07_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC07_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC08_PCLK, "gout_peris_tzpc08_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC08_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC09_PCLK, "gout_peris_tzpc09_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC09_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC10_PCLK, "gout_peris_tzpc10_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC10_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC11_PCLK, "gout_peris_tzpc11_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC11_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC12_PCLK, "gout_peris_tzpc12_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC12_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC13_PCLK, "gout_peris_tzpc13_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC13_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC14_PCLK, "gout_peris_tzpc14_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC14_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_TZPC15_PCLK, "gout_peris_tzpc15_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_TZPC15_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_WDT_CLUSTER0_PCLK, "gout_peris_wdt_cluster0_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_WDT_CLUSTER0_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_WDT_CLUSTER1_PCLK, "gout_peris_wdt_cluster1_pclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_WDT_CLUSTER1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIS_XIU_P_PERIS_ACLK, "gout_peris_xiu_p_peris_aclk",
	     "mout_peris_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIS_UID_XIU_P_PERIS_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info peris_cmu_info __initconst = {
	.mux_clks		= peris_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(peris_mux_clks),
	.gate_clks		= peris_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(peris_gate_clks),
	.nr_clk_ids		= CLKS_NR_PERIS,
	.clk_regs		= peris_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(peris_clk_regs),
	.clk_name		= "bus",
	.hwacg_gates		= true,
};

static void __init exynos8895_cmu_peris_init(struct device_node *np)
{
	exynos_arm64_register_cmu(NULL, np, &peris_cmu_info);
}

/* Register CMU_PERIS early, as it's needed for MCT timer */
CLK_OF_DECLARE(exynos8895_cmu_peris, "samsung,exynos8895-cmu-peris",
	       exynos8895_cmu_peris_init);

/* ---- CMU_FSYS0 ---------------------------------------------------------- */

/* Register Offset definitions for CMU_FSYS0 (0x11000000) */
#define PLL_CON0_MUX_CLKCMU_FSYS0_BUS_USER					0x0100
#define PLL_CON2_MUX_CLKCMU_FSYS0_BUS_USER					0x0108
#define PLL_CON0_MUX_CLKCMU_FSYS0_DPGTC_USER					0x0120
#define PLL_CON2_MUX_CLKCMU_FSYS0_DPGTC_USER					0x0128
#define PLL_CON0_MUX_CLKCMU_FSYS0_MMC_EMBD_USER					0x0140
#define PLL_CON2_MUX_CLKCMU_FSYS0_MMC_EMBD_USER					0x0148
#define PLL_CON0_MUX_CLKCMU_FSYS0_UFS_EMBD_USER					0x0160
#define PLL_CON2_MUX_CLKCMU_FSYS0_UFS_EMBD_USER					0x0168
#define PLL_CON0_MUX_CLKCMU_FSYS0_USBDRD30_USER					0x0180
#define PLL_CON2_MUX_CLKCMU_FSYS0_USBDRD30_USER					0x0188
#define CLK_CON_GAT_CLK_BLK_FSYS0_UID_FSYS0_CMU_FSYS0_IPCLKPORT_PCLK		0x2000
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_AHBBR_FSYS0_IPCLKPORT_HCLK		0x2010
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_AXI2AHB_FSYS0_IPCLKPORT_ACLK		0x2014
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_AXI2AHB_USB_FSYS0_IPCLKPORT_ACLK		0x2018
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_AXI2APB_FSYS0_IPCLKPORT_ACLK		0x201c
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_BTM_FSYS0_IPCLKPORT_I_ACLK		0x2020
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_BTM_FSYS0_IPCLKPORT_I_PCLK		0x2024
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_DP_LINK_IPCLKPORT_I_GTC_EXT_CLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_DP_LINK_IPCLKPORT_I_PCLK			0x2030
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_ETR_MIU_IPCLKPORT_I_ACLK			0x2034
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_ETR_MIU_IPCLKPORT_I_PCLK			0x2038
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_GPIO_FSYS0_IPCLKPORT_PCLK		0x203c
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_LHM_AXI_D_USBTV_IPCLKPORT_I_CLK		0x2040
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_LHM_AXI_G_ETR_IPCLKPORT_I_CLK		0x2044
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_LHM_AXI_P_FSYS0_IPCLKPORT_I_CLK		0x2048
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_LHS_ACEL_D_FSYS0_IPCLKPORT_I_CLK		0x204c
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_MMC_EMBD_IPCLKPORT_I_ACLK		0x2050
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_MMC_EMBD_IPCLKPORT_SDCLKIN		0x2054
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_PMU_FSYS0_IPCLKPORT_PCLK			0x2058
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_BCM_FSYS0_IPCLKPORT_ACLK			0x205c
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_BCM_FSYS0_IPCLKPORT_PCLK			0x2060
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_RSTNSYNC_CLK_FSYS0_BUS_IPCLKPORT_CLK	0x2064
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_SYSREG_FSYS0_IPCLKPORT_PCLK		0x2068
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_UFS_EMBD_IPCLKPORT_I_ACLK		0x206c
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_UFS_EMBD_IPCLKPORT_I_CLK_UNIPRO		0x2070
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_UFS_EMBD_IPCLKPORT_I_FMP_CLK		0x2074
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USB30DRD_ACLK		0x2078
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USB30DRD_REF_CLK	0x207c
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USB30DRD_SUSPEND_CLK	0x2080
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USBTVH_AHB_CLK		0x2084
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USBTVH_CORE_CLK	0x2088
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USBTVH_XIU_CLK		0x208c
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_US_D_FSYS0_USB_IPCLKPORT_ACLK		0x2090
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_XIU_D_FSYS0_IPCLKPORT_ACLK		0x2094
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_XIU_D_FSYS0_USB_IPCLKPORT_ACLK		0x2098
#define CLK_CON_GAT_GOUT_BLK_FSYS0_UID_XIU_P_FSYS0_IPCLKPORT_ACLK		0x209c

static const unsigned long fsys0_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_FSYS0_BUS_USER,
	PLL_CON2_MUX_CLKCMU_FSYS0_BUS_USER,
	PLL_CON0_MUX_CLKCMU_FSYS0_DPGTC_USER,
	PLL_CON2_MUX_CLKCMU_FSYS0_DPGTC_USER,
	PLL_CON0_MUX_CLKCMU_FSYS0_MMC_EMBD_USER,
	PLL_CON2_MUX_CLKCMU_FSYS0_MMC_EMBD_USER,
	PLL_CON0_MUX_CLKCMU_FSYS0_UFS_EMBD_USER,
	PLL_CON2_MUX_CLKCMU_FSYS0_UFS_EMBD_USER,
	PLL_CON0_MUX_CLKCMU_FSYS0_USBDRD30_USER,
	PLL_CON2_MUX_CLKCMU_FSYS0_USBDRD30_USER,
};

/* List of parent clocks for Muxes in CMU_FSYS0 */
PNAME(mout_fsys0_bus_user_p)		= { "oscclk", "dout_cmu_fsys0_bus" };
PNAME(mout_fsys0_dpgtc_user_p)		= { "oscclk", "dout_cmu_fsys0_dpgtc" };
PNAME(mout_fsys0_mmc_embd_user_p)	= { "oscclk",
					    "dout_cmu_fsys0_mmc_embd" };
PNAME(mout_fsys0_ufs_embd_user_p)	= { "oscclk",
					    "dout_cmu_fsys0_ufs_embd" };
PNAME(mout_fsys0_usbdrd30_user_p)	= { "oscclk",
					    "dout_cmu_fsys0_usbdrd30" };

static const struct samsung_mux_clock fsys0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_FSYS0_BUS_USER, "mout_fsys0_bus_user",
	    mout_fsys0_bus_user_p, PLL_CON0_MUX_CLKCMU_FSYS0_BUS_USER, 4, 1),
	MUX(CLK_MOUT_FSYS0_DPGTC_USER, "mout_fsys0_dpgtc_user",
	    mout_fsys0_dpgtc_user_p, PLL_CON0_MUX_CLKCMU_FSYS0_DPGTC_USER,
	    4, 1),
	MUX_F(CLK_MOUT_FSYS0_MMC_EMBD_USER, "mout_fsys0_mmc_embd_user",
	      mout_fsys0_mmc_embd_user_p, PLL_CON0_MUX_CLKCMU_FSYS0_MMC_EMBD_USER,
	      4, 1, CLK_SET_RATE_PARENT, 0),
	MUX(CLK_MOUT_FSYS0_UFS_EMBD_USER, "mout_fsys0_ufs_embd_user",
	    mout_fsys0_ufs_embd_user_p, PLL_CON0_MUX_CLKCMU_FSYS0_UFS_EMBD_USER,
	    4, 1),
	MUX(CLK_MOUT_FSYS0_USBDRD30_USER, "mout_fsys0_usbdrd30_user",
	    mout_fsys0_usbdrd30_user_p, PLL_CON0_MUX_CLKCMU_FSYS0_USBDRD30_USER,
	    4, 1),
};

static const struct samsung_gate_clock fsys0_gate_clks[] __initconst = {
	/* Disabling this clock makes the system hang. Mark the clock as critical. */
	GATE(CLK_GOUT_FSYS0_FSYS0_CMU_FSYS0_PCLK,
	     "gout_fsys0_fsys0_cmu_fsys0_pclk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_CLK_BLK_FSYS0_UID_FSYS0_CMU_FSYS0_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_AHBBR_FSYS0_HCLK, "gout_fsys0_ahbbr_fsys0_hclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_AHBBR_FSYS0_IPCLKPORT_HCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_AXI2AHB_FSYS0_ACLK,
	     "gout_fsys0_axi2ahb_fsys0_aclk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_AXI2AHB_FSYS0_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_AXI2AHB_USB_FSYS0_ACLK,
	     "gout_fsys0_axi2ahb_usb_fsys0_aclk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_AXI2AHB_USB_FSYS0_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_AXI2APB_FSYS0_ACLK,
	     "gout_fsys0_axi2apb_fsys0_aclk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_AXI2APB_FSYS0_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_BTM_FSYS0_I_ACLK, "gout_fsys0_btm_fsys0_i_aclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_BTM_FSYS0_IPCLKPORT_I_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_BTM_FSYS0_I_PCLK, "gout_fsys0_btm_fsys0_i_pclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_BTM_FSYS0_IPCLKPORT_I_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_DP_LINK_I_GTC_EXT_CLK,
	     "gout_fsys0_dp_link_i_gtc_ext_clk", "mout_fsys0_dpgtc_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_DP_LINK_IPCLKPORT_I_GTC_EXT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS0_DP_LINK_I_PCLK, "gout_fsys0_dp_link_i_pclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_DP_LINK_IPCLKPORT_I_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS0_ETR_MIU_I_ACLK, "gout_fsys0_etr_miu_i_aclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_ETR_MIU_IPCLKPORT_I_ACLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_ETR_MIU_I_PCLK, "gout_fsys0_etr_miu_i_pclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_ETR_MIU_IPCLKPORT_I_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_GPIO_FSYS0_PCLK, "gout_fsys0_gpio_fsys0_pclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_GPIO_FSYS0_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_LHM_AXI_D_USBTV_I_CLK,
	     "gout_fsys0_lhm_axi_d_usbtv_i_clk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_LHM_AXI_D_USBTV_IPCLKPORT_I_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_LHM_AXI_G_ETR_I_CLK,
	     "gout_fsys0_lhm_axi_g_etr_i_clk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_LHM_AXI_G_ETR_IPCLKPORT_I_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_LHM_AXI_P_FSYS0_I_CLK,
	     "gout_fsys0_lhm_axi_p_fsys0_i_clk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_LHM_AXI_P_FSYS0_IPCLKPORT_I_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_LHS_ACEL_D_FSYS0_I_CLK,
	     "gout_fsys0_lhs_acel_d_fsys0_i_clk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_LHS_ACEL_D_FSYS0_IPCLKPORT_I_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_MMC_EMBD_I_ACLK, "gout_fsys0_mmc_embd_i_aclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_MMC_EMBD_IPCLKPORT_I_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS0_MMC_EMBD_SDCLKIN, "gout_fsys0_mmc_embd_sdclkin",
	     "mout_fsys0_mmc_embd_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_MMC_EMBD_IPCLKPORT_SDCLKIN,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_PMU_FSYS0_PCLK, "gout_fsys0_pmu_fsys0_pclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_PMU_FSYS0_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_BCM_FSYS0_ACLK, "gout_fsys0_bcm_fsys0_aclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_BCM_FSYS0_IPCLKPORT_ACLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_BCM_FSYS0_PCLK, "gout_fsys0_bcm_fsys0_pclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_BCM_FSYS0_IPCLKPORT_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_RSTNSYNC_CLK_FSYS0_BUS_CLK,
	     "gout_fsys0_rstnsync_clk_fsys0_bus_clk", "mout_fsys0_bus_user",
	      CLK_CON_GAT_GOUT_BLK_FSYS0_UID_RSTNSYNC_CLK_FSYS0_BUS_IPCLKPORT_CLK,
	      21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_SYSREG_FSYS0_PCLK, "gout_fsys0_sysreg_fsys0_pclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_SYSREG_FSYS0_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_UFS_EMBD_I_ACLK, "gout_fsys0_ufs_embd_i_aclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_UFS_EMBD_IPCLKPORT_I_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS0_UFS_EMBD_I_CLK_UNIPRO,
	     "gout_fsys0_ufs_embd_i_clk_unipro", "mout_fsys0_ufs_embd_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_UFS_EMBD_IPCLKPORT_I_CLK_UNIPRO,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_UFS_EMBD_I_FMP_CLK,
	     "gout_fsys0_ufs_embd_i_fmp_clk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_UFS_EMBD_IPCLKPORT_I_FMP_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_USBTV_I_USB30DRD_ACLK,
	     "gout_fsys0_usbtv_i_usb30drd_aclk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USB30DRD_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS0_USBTV_I_USB30DRD_REF_CLK,
	     "gout_fsys0_usbtv_i_usb30drd_ref_clk", "mout_fsys0_usbdrd30_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USB30DRD_REF_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS0_USBTV_I_USB30DRD_SUSPEND_CLK,
	     "gout_fsys0_usbtv_i_usb30drd_suspend_clk",
	     "mout_fsys0_usbdrd30_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USB30DRD_SUSPEND_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS0_USBTV_I_USBTVH_AHB_CLK,
	     "gout_fsys0_usbtv_i_usbtvh_ahb_clk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USBTVH_AHB_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS0_USBTV_I_USBTVH_CORE_CLK,
	     "gout_fsys0_usbtv_i_usbtvh_core_clk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USBTVH_CORE_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS0_USBTV_I_USBTVH_XIU_CLK,
	     "gout_fsys0_usbtv_i_usbtvh_xiu_clk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_USBTV_IPCLKPORT_I_USBTVH_XIU_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_US_D_FSYS0_USB_ACLK,
	     "gout_fsys0_us_d_fsys0_usb_aclk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_US_D_FSYS0_USB_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_XIU_D_FSYS0_ACLK, "gout_fsys0_xiu_d_fsys0_aclk",
	     "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_XIU_D_FSYS0_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_XIU_D_FSYS0_USB_ACLK,
	     "gout_fsys0_xiu_d_fsys0_usb_aclk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_XIU_D_FSYS0_USB_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_XIU_P_FSYS0_ACLK,
	     "gout_fsys0_xiu_p_fsys0_aclk", "mout_fsys0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS0_UID_XIU_P_FSYS0_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
};

/*
 * USER muxes the vendor kernel has on OSCCLK when the SoC enters SLEEP, where
 * each followed the last clk_disable() of its consumers.
 */
static const struct samsung_clk_reg_dump fsys0_suspend_regs[] = {
	{ PLL_CON0_MUX_CLKCMU_FSYS0_BUS_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_FSYS0_UFS_EMBD_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_FSYS0_USBDRD30_USER, 0x0 },
};

static const struct samsung_cmu_info fsys0_cmu_info __initconst = {
	.mux_clks		= fsys0_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(fsys0_mux_clks),
	.gate_clks		= fsys0_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(fsys0_gate_clks),
	.nr_clk_ids		= CLKS_NR_FSYS0,
	.clk_regs		= fsys0_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(fsys0_clk_regs),
	.suspend_regs		= fsys0_suspend_regs,
	.nr_suspend_regs	= ARRAY_SIZE(fsys0_suspend_regs),
	.clk_name		= "bus",
	.hwacg_gates		= true,
};

/* ---- CMU_FSYS1 ---------------------------------------------------------- */

/* Register Offset definitions for CMU_FSYS1 (0x11400000) */
#define PLL_CON0_MUX_CLKCMU_FSYS1_BUS_USER									0x0100
#define PLL_CON2_MUX_CLKCMU_FSYS1_BUS_USER									0x0108
#define PLL_CON0_MUX_CLKCMU_FSYS1_MMC_CARD_USER									0x0120
#define PLL_CON2_MUX_CLKCMU_FSYS1_MMC_CARD_USER									0x0128
#define PLL_CON0_MUX_CLKCMU_FSYS1_PCIE_USER									0x0140
#define PLL_CON2_MUX_CLKCMU_FSYS1_PCIE_USER									0x0148
#define PLL_CON0_MUX_CLKCMU_FSYS1_UFS_CARD_USER									0x0160
#define PLL_CON2_MUX_CLKCMU_FSYS1_UFS_CARD_USER									0x0168
#define CLK_CON_GAT_CLK_BLK_FSYS1_UID_PCIE_IPCLKPORT_PHY_REF_CLK_IN						0x2000
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_ADM_AHB_SSS_IPCLKPORT_HCLKM						0x2004
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_AHBBR_FSYS1_IPCLKPORT_HCLK						0x2008
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_AXI2AHB_FSYS1_IPCLKPORT_ACLK						0x200c
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_AXI2APB_FSYS1P0_IPCLKPORT_ACLK						0x2010
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_AXI2APB_FSYS1P1_IPCLKPORT_ACLK						0x2014
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_BTM_FSYS1_IPCLKPORT_I_ACLK						0x2018
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_BTM_FSYS1_IPCLKPORT_I_PCLK						0x201c
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_FSYS1_CMU_FSYS1_IPCLKPORT_PCLK						0x2024
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_GPIO_FSYS1_IPCLKPORT_PCLK						0x2028
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_LHM_AXI_P_FSYS1_IPCLKPORT_I_CLK						0x202c
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_LHS_ACEL_D_FSYS1_IPCLKPORT_I_CLK						0x2030
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_MMC_CARD_IPCLKPORT_I_ACLK						0x2034
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_MMC_CARD_IPCLKPORT_SDCLKIN						0x2038
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_DBI_ACLK_0						0x203c
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_DBI_ACLK_1						0x2040
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_IEEE1500_WRAPPER_FOR_PCIE_PHY_LC_X2_INST_0_I_SCL_APB_PCLK	0x2044
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_MSTR_ACLK_0						0x2048
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_MSTR_ACLK_1						0x204c
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_PCIE_SUB_CTRL_INST_0_I_DRIVER_APB_CLK			0x2050
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_PCIE_SUB_CTRL_INST_1_I_DRIVER_APB_CLK			0x2054
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_PIPE2_DIGITAL_X2_WRAP_INST_0_I_APB_PCLK_SCL		0x2058
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_SLV_ACLK_0						0x205c
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_SLV_ACLK_1						0x2060
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PMU_FSYS1_IPCLKPORT_PCLK							0x2068
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_BCM_FSYS1_IPCLKPORT_ACLK							0x206c
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_BCM_FSYS1_IPCLKPORT_PCLK							0x2070
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_RSTNSYNC_CLK_FSYS1_BUS_IPCLKPORT_CLK					0x2074
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_RTIC_IPCLKPORT_I_ACLK							0x207c
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_RTIC_IPCLKPORT_I_PCLK							0x2080
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_SSS_IPCLKPORT_I_ACLK							0x2084
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_SSS_IPCLKPORT_I_PCLK							0x2088
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_SYSREG_FSYS1_IPCLKPORT_PCLK						0x2090
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_TOE_WIFI0_IPCLKPORT_I_CLK						0x2094
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_TOE_WIFI1_IPCLKPORT_I_CLK						0x2098
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_UFS_CARD_IPCLKPORT_I_ACLK						0x209c
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_UFS_CARD_IPCLKPORT_I_CLK_UNIPRO						0x20a0
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_UFS_CARD_IPCLKPORT_I_FMP_CLK						0x20a4
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_XIU_D_FSYS1_IPCLKPORT_ACLK						0x20a8
#define CLK_CON_GAT_GOUT_BLK_FSYS1_UID_XIU_P_FSYS1_IPCLKPORT_ACLK						0x20ac

static const unsigned long fsys1_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_FSYS1_BUS_USER,
	PLL_CON2_MUX_CLKCMU_FSYS1_BUS_USER,
	PLL_CON0_MUX_CLKCMU_FSYS1_MMC_CARD_USER,
	PLL_CON2_MUX_CLKCMU_FSYS1_MMC_CARD_USER,
	PLL_CON0_MUX_CLKCMU_FSYS1_PCIE_USER,
	PLL_CON2_MUX_CLKCMU_FSYS1_PCIE_USER,
	PLL_CON0_MUX_CLKCMU_FSYS1_UFS_CARD_USER,
	PLL_CON2_MUX_CLKCMU_FSYS1_UFS_CARD_USER,
};

/* List of parent clocks for Muxes in CMU_FSYS1 */
PNAME(mout_fsys1_bus_user_p)		= { "oscclk", "dout_cmu_fsys1_bus" };
PNAME(mout_fsys1_mmc_card_user_p)	= { "oscclk",
					    "dout_cmu_fsys1_mmc_card" };
PNAME(mout_fsys1_pcie_user_p)		= { "oscclk", "dout_cmu_fsys1_pcie" };
PNAME(mout_fsys1_ufs_card_user_p)	= { "oscclk",
					    "dout_cmu_fsys1_ufs_card" };

static const struct samsung_mux_clock fsys1_mux_clks[] __initconst = {
	MUX(CLK_MOUT_FSYS1_BUS_USER, "mout_fsys1_bus_user",
	    mout_fsys1_bus_user_p, PLL_CON0_MUX_CLKCMU_FSYS1_BUS_USER, 4, 1),
	MUX_F(CLK_MOUT_FSYS1_MMC_CARD_USER, "mout_fsys1_mmc_card_user",
	      mout_fsys1_mmc_card_user_p,
	      PLL_CON0_MUX_CLKCMU_FSYS1_MMC_CARD_USER,
	      4, 1, CLK_SET_RATE_PARENT, 0),
	MUX(CLK_MOUT_FSYS1_PCIE_USER, "mout_fsys1_pcie_user",
	    mout_fsys1_pcie_user_p, PLL_CON0_MUX_CLKCMU_FSYS1_PCIE_USER, 4, 1),
	MUX(CLK_MOUT_FSYS1_UFS_CARD_USER, "mout_fsys1_ufs_card_user",
	    mout_fsys1_ufs_card_user_p,
	    PLL_CON0_MUX_CLKCMU_FSYS1_UFS_CARD_USER, 4, 1),
};

static const struct samsung_gate_clock fsys1_gate_clks[] __initconst = {
	GATE(CLK_GOUT_FSYS1_PCIE_PHY_REF_CLK_IN,
	     "gout_clk_blk_fsys1_pcie_phy_ref_clk_in", "mout_fsys1_pcie_user",
	     CLK_CON_GAT_CLK_BLK_FSYS1_UID_PCIE_IPCLKPORT_PHY_REF_CLK_IN,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_ADM_AHB_SSS_HCLKM, "gout_fsys1_adm_ahb_sss_hclkm",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_ADM_AHB_SSS_IPCLKPORT_HCLKM,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_AHBBR_FSYS1_HCLK, "gout_fsys1_ahbbr_fsys1_hclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_AHBBR_FSYS1_IPCLKPORT_HCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_AXI2AHB_FSYS1_ACLK,
	     "gout_fsys1_axi2ahb_fsys1_aclk", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_AXI2AHB_FSYS1_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_AXI2APB_FSYS1P0_ACLK,
	     "gout_fsys1_axi2apb_fsys1p0_aclk", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_AXI2APB_FSYS1P0_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_AXI2APB_FSYS1P1_ACLK,
	     "gout_fsys1_axi2apb_fsys1p1_aclk", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_AXI2APB_FSYS1P1_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_BTM_FSYS1_I_ACLK, "gout_fsys1_btm_fsys1_i_aclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_BTM_FSYS1_IPCLKPORT_I_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_BTM_FSYS1_I_PCLK, "gout_fsys1_btm_fsys1_i_pclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_BTM_FSYS1_IPCLKPORT_I_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_FSYS1_CMU_FSYS1_PCLK,
	     "gout_fsys1_fsys1_cmu_fsys1_pclk", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_FSYS1_CMU_FSYS1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_GPIO_FSYS1_PCLK, "gout_fsys1_gpio_fsys1_pclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_GPIO_FSYS1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_LHM_AXI_P_FSYS1_I_CLK,
	     "gout_fsys1_lhm_axi_p_fsys1_i_clk", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_LHM_AXI_P_FSYS1_IPCLKPORT_I_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_LHS_ACEL_D_FSYS1_I_CLK,
	     "gout_fsys1_lhs_acel_d_fsys1_i_clk", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_LHS_ACEL_D_FSYS1_IPCLKPORT_I_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_MMC_CARD_I_ACLK, "gout_fsys1_mmc_card_i_aclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_MMC_CARD_IPCLKPORT_I_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_MMC_CARD_SDCLKIN, "gout_fsys1_mmc_card_sdclkin",
	     "mout_fsys1_mmc_card_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_MMC_CARD_IPCLKPORT_SDCLKIN,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_DBI_ACLK_0, "gout_fsys1_pcie_dbi_aclk_0",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_DBI_ACLK_0,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_DBI_ACLK_1, "gout_fsys1_pcie_dbi_aclk_1",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_DBI_ACLK_1,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_IEEE1500_WRAPPER_FOR_PCIE_PHY_LC_X2_INST_0_I_SCL_APB_PCLK,
	     "gout_fsys1_pcie_ieee1500_wrapper_for_pcie_phy_lc_x2_inst_0_i_scl_apb_pclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_IEEE1500_WRAPPER_FOR_PCIE_PHY_LC_X2_INST_0_I_SCL_APB_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_MSTR_ACLK_0,
	     "gout_fsys1_pcie_mstr_aclk_0", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_MSTR_ACLK_0,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_MSTR_ACLK_1,
	     "gout_fsys1_pcie_mstr_aclk_1", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_MSTR_ACLK_1,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_PCIE_SUB_CTRL_INST_0_I_DRIVER_APB_CLK,
	     "gout_fsys1_pcie_pcie_sub_ctrl_inst_0_i_driver_apb_clk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_PCIE_SUB_CTRL_INST_0_I_DRIVER_APB_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_PCIE_SUB_CTRL_INST_1_I_DRIVER_APB_CLK,
	     "gout_fsys1_pcie_pcie_sub_ctrl_inst_1_i_driver_apb_clk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_PCIE_SUB_CTRL_INST_1_I_DRIVER_APB_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_PIPE2_DIGITAL_X2_WRAP_INST_0_I_APB_PCLK_SCL,
	     "gout_fsys1_pcie_pipe2_digital_x2_wrap_inst_0_i_apb_pclk_scl",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_PIPE2_DIGITAL_X2_WRAP_INST_0_I_APB_PCLK_SCL,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_SLV_ACLK_0, "gout_fsys1_pcie_slv_aclk_0",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_SLV_ACLK_0,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PCIE_SLV_ACLK_1, "gout_fsys1_pcie_slv_aclk_1",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PCIE_IPCLKPORT_SLV_ACLK_1,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_PMU_FSYS1_PCLK, "gout_fsys1_pmu_fsys1_pclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_PMU_FSYS1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_BCM_FSYS1_ACLK, "gout_fsys1_bcm_fsys1_aclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_BCM_FSYS1_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_BCM_FSYS1_PCLK, "gout_fsys1_bcm_fsys1_pclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_BCM_FSYS1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_RSTNSYNC_CLK_FSYS1_BUS_CLK,
	     "gout_fsys1_rstnsync_clk_fsys1_bus_clk", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_RSTNSYNC_CLK_FSYS1_BUS_IPCLKPORT_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_RTIC_I_ACLK, "gout_fsys1_rtic_i_aclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_RTIC_IPCLKPORT_I_ACLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_RTIC_I_PCLK, "gout_fsys1_rtic_i_pclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_RTIC_IPCLKPORT_I_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_SSS_I_ACLK, "gout_fsys1_sss_i_aclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_SSS_IPCLKPORT_I_ACLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_SSS_I_PCLK, "gout_fsys1_sss_i_pclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_SSS_IPCLKPORT_I_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_SYSREG_FSYS1_PCLK, "gout_fsys1_sysreg_fsys1_pclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_SYSREG_FSYS1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_TOE_WIFI0_I_CLK, "gout_fsys1_toe_wifi0_i_clk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_TOE_WIFI0_IPCLKPORT_I_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_TOE_WIFI1_I_CLK, "gout_fsys1_toe_wifi1_i_clk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_TOE_WIFI1_IPCLKPORT_I_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_UFS_CARD_I_ACLK, "gout_fsys1_ufs_card_i_aclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_UFS_CARD_IPCLKPORT_I_ACLK,
	     21, 0, 0),
	GATE(CLK_GOUT_FSYS1_UFS_CARD_I_CLK_UNIPRO,
	     "gout_fsys1_ufs_card_i_clk_unipro", "mout_fsys1_ufs_card_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_UFS_CARD_IPCLKPORT_I_CLK_UNIPRO,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_FSYS1_UFS_CARD_I_FMP_CLK,
	     "gout_fsys1_ufs_card_i_fmp_clk", "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_UFS_CARD_IPCLKPORT_I_FMP_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_XIU_D_FSYS1_ACLK, "gout_fsys1_xiu_d_fsys1_aclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_XIU_D_FSYS1_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_XIU_P_FSYS1_ACLK, "gout_fsys1_xiu_p_fsys1_aclk",
	     "mout_fsys1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_FSYS1_UID_XIU_P_FSYS1_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info fsys1_cmu_info __initconst = {
	.mux_clks		= fsys1_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(fsys1_mux_clks),
	.gate_clks		= fsys1_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(fsys1_gate_clks),
	.nr_clk_ids		= CLKS_NR_FSYS1,
	.clk_regs		= fsys1_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(fsys1_clk_regs),
	.clk_name		= "bus",
	.hwacg_gates		= true,
};

/* ---- CMU_PERIC0 ---------------------------------------------------------- */

/* Register Offset definitions for CMU_PERIC0 (0x10400000) */
#define PLL_CON0_MUX_CLKCMU_PERIC0_BUS_USER						0x0100
#define PLL_CON2_MUX_CLKCMU_PERIC0_BUS_USER						0x0108
#define PLL_CON0_MUX_CLKCMU_PERIC0_UART_DBG_USER					0x0120
#define PLL_CON2_MUX_CLKCMU_PERIC0_UART_DBG_USER					0x0128
#define PLL_CON0_MUX_CLKCMU_PERIC0_USI00_USER						0x0140
#define PLL_CON2_MUX_CLKCMU_PERIC0_USI00_USER						0x0148
#define PLL_CON0_MUX_CLKCMU_PERIC0_USI01_USER						0x0160
#define PLL_CON2_MUX_CLKCMU_PERIC0_USI01_USER						0x0168
#define PLL_CON0_MUX_CLKCMU_PERIC0_USI02_USER						0x0180
#define PLL_CON2_MUX_CLKCMU_PERIC0_USI02_USER						0x0188
#define PLL_CON0_MUX_CLKCMU_PERIC0_USI03_USER						0x01a0
#define PLL_CON2_MUX_CLKCMU_PERIC0_USI03_USER						0x01a8
#define CLK_CON_GAT_CLK_BLK_PERIC0_UID_PERIC0_CMU_PERIC0_IPCLKPORT_PCLK			0x2000
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_AXI2APB_PERIC0_IPCLKPORT_ACLK			0x2014
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_GPIO_PERIC0_IPCLKPORT_PCLK			0x2018
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_LHM_AXI_P_PERIC0_IPCLKPORT_I_CLK		0x201c
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_PMU_PERIC0_IPCLKPORT_PCLK			0x2020
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_PWM_IPCLKPORT_I_PCLK_S0				0x2028
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_BUSP_IPCLKPORT_CLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_SPEEDY2_TSP_IPCLKPORT_CLK			0x2030
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_SYSREG_PERIC0_IPCLKPORT_PCLK			0x2034
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_UART_DBG_IPCLKPORT_EXT_UCLK			0x2038
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_UART_DBG_IPCLKPORT_PCLK				0x203c
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI00_IPCLKPORT_I_PCLK				0x2040
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI00_IPCLKPORT_I_SCLK_USI			0x2044
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI01_IPCLKPORT_I_PCLK				0x2048
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI01_IPCLKPORT_I_SCLK_USI			0x204c
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI02_IPCLKPORT_I_PCLK				0x2050
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI02_IPCLKPORT_I_SCLK_USI			0x2054
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI03_IPCLKPORT_I_PCLK				0x2058
#define CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI03_IPCLKPORT_I_SCLK_USI			0x205c
/* Q-channel controls: ENABLE is bit 0, CLOCK_REQ bit 1 */
#define QCH_CON_USI00_QCH							0x3020
#define QCH_CON_USI01_QCH							0x3024
#define QCH_CON_USI02_QCH							0x3028
#define QCH_CON_USI03_QCH							0x302c

static const unsigned long peric0_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_PERIC0_BUS_USER,
	PLL_CON2_MUX_CLKCMU_PERIC0_BUS_USER,
	PLL_CON0_MUX_CLKCMU_PERIC0_UART_DBG_USER,
	PLL_CON2_MUX_CLKCMU_PERIC0_UART_DBG_USER,
	PLL_CON0_MUX_CLKCMU_PERIC0_USI00_USER,
	PLL_CON2_MUX_CLKCMU_PERIC0_USI00_USER,
	PLL_CON0_MUX_CLKCMU_PERIC0_USI01_USER,
	PLL_CON2_MUX_CLKCMU_PERIC0_USI01_USER,
	PLL_CON0_MUX_CLKCMU_PERIC0_USI02_USER,
	PLL_CON2_MUX_CLKCMU_PERIC0_USI02_USER,
	PLL_CON0_MUX_CLKCMU_PERIC0_USI03_USER,
	PLL_CON2_MUX_CLKCMU_PERIC0_USI03_USER,
	/* Q-channel controls, outside the gate range: save and restore only */
	QCH_CON_USI00_QCH,
	QCH_CON_USI01_QCH,
	QCH_CON_USI02_QCH,
	QCH_CON_USI03_QCH,
};

/* List of parent clocks for Muxes in CMU_PERIC0 */
PNAME(mout_peric0_bus_user_p)		= { "oscclk", "dout_cmu_peric0_bus" };
PNAME(mout_peric0_uart_dbg_user_p)	= { "oscclk",
					    "dout_cmu_peric0_uart_dbg" };
PNAME(mout_peric0_usi00_user_p)		= { "oscclk",
					    "dout_cmu_peric0_usi00" };
PNAME(mout_peric0_usi01_user_p)		= { "oscclk",
					    "dout_cmu_peric0_usi01" };
PNAME(mout_peric0_usi02_user_p)		= { "oscclk",
					    "dout_cmu_peric0_usi02" };
PNAME(mout_peric0_usi03_user_p)		= { "oscclk",
					    "dout_cmu_peric0_usi03" };

static const struct samsung_mux_clock peric0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PERIC0_BUS_USER, "mout_peric0_bus_user",
	    mout_peric0_bus_user_p, PLL_CON0_MUX_CLKCMU_PERIC0_BUS_USER, 4, 1),
	MUX(CLK_MOUT_PERIC0_UART_DBG_USER, "mout_peric0_uart_dbg_user",
	    mout_peric0_uart_dbg_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC0_UART_DBG_USER, 4, 1),
	MUX(CLK_MOUT_PERIC0_USI00_USER, "mout_peric0_usi00_user",
	    mout_peric0_usi00_user_p, PLL_CON0_MUX_CLKCMU_PERIC0_USI00_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC0_USI01_USER, "mout_peric0_usi01_user",
	    mout_peric0_usi01_user_p, PLL_CON0_MUX_CLKCMU_PERIC0_USI01_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC0_USI02_USER, "mout_peric0_usi02_user",
	    mout_peric0_usi02_user_p, PLL_CON0_MUX_CLKCMU_PERIC0_USI02_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC0_USI03_USER, "mout_peric0_usi03_user",
	    mout_peric0_usi03_user_p, PLL_CON0_MUX_CLKCMU_PERIC0_USI03_USER,
	    4, 1),
};

static const struct samsung_gate_clock peric0_gate_clks[] __initconst = {
	GATE(CLK_GOUT_PERIC0_PERIC0_CMU_PERIC0_PCLK,
	     "gout_cperic0_peric0_cmu_peric0_pclk", "mout_peric0_bus_user",
	     CLK_CON_GAT_CLK_BLK_PERIC0_UID_PERIC0_CMU_PERIC0_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC0_AXI2APB_PERIC0_ACLK,
	     "gout_peric0_axi2apb_peric0_aclk", "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_AXI2APB_PERIC0_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC0_GPIO_PERIC0_PCLK, "gout_peric0_gpio_peric0_pclk",
	     "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_GPIO_PERIC0_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC0_LHM_AXI_P_PERIC0_I_CLK,
	     "gout_peric0_lhm_axi_p_peric0_i_clk", "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_LHM_AXI_P_PERIC0_IPCLKPORT_I_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC0_PMU_PERIC0_PCLK, "gout_peric0_pmu_peric0_pclk",
	     "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_PMU_PERIC0_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC0_PWM_I_PCLK_S0, "gout_peric0_pwm_i_pclk_s0",
	     "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_PWM_IPCLKPORT_I_PCLK_S0,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC0_RSTNSYNC_CLK_PERIC0_BUSP_CLK,
	     "gout_peric0_rstnsync_clk_peric0_busp_clk",
	     "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_RSTNSYNC_CLK_PERIC0_BUSP_IPCLKPORT_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC0_SPEEDY2_TSP_CLK, "gout_peric0_speedy2_tsp_clk",
	     "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_SPEEDY2_TSP_IPCLKPORT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC0_SYSREG_PERIC0_PCLK,
	     "gout_peric0_sysreg_peric0_pclk", "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_SYSREG_PERIC0_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC0_UART_DBG_EXT_UCLK,
	     "gout_peric0_uart_dbg_ext_uclk", "mout_peric0_uart_dbg_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_UART_DBG_IPCLKPORT_EXT_UCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_UART_DBG_PCLK, "gout_peric0_uart_dbg_pclk",
	     "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_UART_DBG_IPCLKPORT_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI00_I_PCLK, "gout_peric0_usi00_i_pclk",
	     "mout_peric0_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI00_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI00_I_SCLK_USI, "gout_peric0_usi00_i_sclk_usi",
	     "mout_peric0_usi00_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI00_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	/*
	 * Each USI's Q-channel clock request is the parent of its PCLK, so the
	 * USI clock is requested exactly while a consumer holds the PCLK.
	 */
	GATE(0, "gout_peric0_usi01_qch", "mout_peric0_bus_user",
	     QCH_CON_USI01_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI01_I_PCLK, "gout_peric0_usi01_i_pclk",
	     "gout_peric0_usi01_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI01_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI01_I_SCLK_USI, "gout_peric0_usi01_i_sclk_usi",
	     "mout_peric0_usi01_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI01_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric0_usi02_qch", "mout_peric0_bus_user",
	     QCH_CON_USI02_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI02_I_PCLK, "gout_peric0_usi02_i_pclk",
	     "gout_peric0_usi02_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI02_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI02_I_SCLK_USI, "gout_peric0_usi02_i_sclk_usi",
	     "mout_peric0_usi02_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI02_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric0_usi03_qch", "mout_peric0_bus_user",
	     QCH_CON_USI03_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI03_I_PCLK, "gout_peric0_usi03_i_pclk",
	     "gout_peric0_usi03_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI03_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI03_I_SCLK_USI, "gout_peric0_usi03_i_sclk_usi",
	     "mout_peric0_usi03_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC0_UID_USI03_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	/*
	 * USI00 (the fingerprint SPI) has no Linux USI consumer: its request
	 * gate is exported and sits below the PCLK, like the vendor fp-spi-pclk.
	 */
	GATE(CLK_GOUT_PERIC0_USI00_QCH, "gout_peric0_usi00_qch",
	     "gout_peric0_usi00_i_pclk", QCH_CON_USI00_QCH, 1, 0, 0),
};

/* USER muxes the vendor kernel has on OSCCLK in SLEEP, as in CMU_FSYS0 */
static const struct samsung_clk_reg_dump peric0_suspend_regs[] = {
	{ PLL_CON0_MUX_CLKCMU_PERIC0_USI00_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC0_USI01_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC0_USI02_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC0_USI03_USER, 0x0 },
};

static const struct samsung_cmu_info peric0_cmu_info __initconst = {
	.mux_clks		= peric0_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(peric0_mux_clks),
	.gate_clks		= peric0_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(peric0_gate_clks),
	.nr_clk_ids		= CLKS_NR_PERIC0,
	.clk_regs		= peric0_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(peric0_clk_regs),
	.suspend_regs		= peric0_suspend_regs,
	.nr_suspend_regs	= ARRAY_SIZE(peric0_suspend_regs),
	.clk_name		= "bus",
	.hwacg_gates		= true,
};

/* ---- CMU_PERIC1 ---------------------------------------------------------- */

/* Register Offset definitions for CMU_PERIC1 (0x10800000) */
#define PLL_CON0_MUX_CLKCMU_PERIC1_BUS_USER						0x0100
#define PLL_CON2_MUX_CLKCMU_PERIC1_BUS_USER						0x0108
#define PLL_CON0_MUX_CLKCMU_PERIC1_SPEEDY2_USER						0x0120
#define PLL_CON2_MUX_CLKCMU_PERIC1_SPEEDY2_USER						0x0128
#define PLL_CON0_MUX_CLKCMU_PERIC1_SPI_CAM0_USER					0x0140
#define PLL_CON2_MUX_CLKCMU_PERIC1_SPI_CAM0_USER					0x0148
#define PLL_CON0_MUX_CLKCMU_PERIC1_SPI_CAM1_USER					0x0160
#define PLL_CON2_MUX_CLKCMU_PERIC1_SPI_CAM1_USER					0x0168
#define PLL_CON0_MUX_CLKCMU_PERIC1_UART_BT_USER						0x0180
#define PLL_CON2_MUX_CLKCMU_PERIC1_UART_BT_USER						0x0188
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI04_USER						0x01a0
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI04_USER						0x01a8
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI05_USER						0x01c0
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI05_USER						0x01c8
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI06_USER						0x01e0
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI06_USER						0x01e8
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI07_USER						0x0200
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI07_USER						0x0208
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI08_USER						0x0220
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI08_USER						0x0228
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI09_USER						0x0240
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI09_USER						0x0248
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI10_USER						0x0260
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI10_USER						0x0268
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI11_USER						0x0280
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI11_USER						0x0288
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI12_USER						0x02a0
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI12_USER						0x02a8
#define PLL_CON0_MUX_CLKCMU_PERIC1_USI13_USER						0x02c0
#define PLL_CON2_MUX_CLKCMU_PERIC1_USI13_USER						0x02c8
#define CLK_CON_GAT_CLK_BLK_PERIC1_UID_PERIC1_CMU_PERIC1_IPCLKPORT_PCLK			0x2000
#define CLK_CON_GAT_CLK_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_SPEEDY2_IPCLKPORT_CLK	0x200c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_AXI2APB_PERIC1P0_IPCLKPORT_ACLK			0x201c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_AXI2APB_PERIC1P1_IPCLKPORT_ACLK			0x2020
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_AXI2APB_PERIC1P2_IPCLKPORT_ACLK			0x2024
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_GPIO_PERIC1_IPCLKPORT_PCLK			0x2028
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_HSI2C_CAM0_IPCLKPORT_IPCLK			0x202c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_HSI2C_CAM1_IPCLKPORT_IPCLK			0x2030
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_HSI2C_CAM2_IPCLKPORT_IPCLK			0x2034
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_HSI2C_CAM3_IPCLKPORT_IPCLK			0x2038
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_LHM_AXI_P_PERIC1_IPCLKPORT_I_CLK		0x203c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_PMU_PERIC1_IPCLKPORT_PCLK			0x2040
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_BUSP_IPCLKPORT_CLK		0x2044
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI1_IPCLKPORT_CLK			0x2048
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI1_IPCLKPORT_SCLK			0x204c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI2_IPCLKPORT_CLK			0x2050
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI2_IPCLKPORT_SCLK			0x2054
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI_IPCLKPORT_CLK			0x2058
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI_IPCLKPORT_SCLK			0x205c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_TSP1_IPCLKPORT_CLK			0x2060
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_TSP2_IPCLKPORT_CLK			0x2064
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPI_CAM0_IPCLKPORT_PCLK				0x2068
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPI_CAM0_IPCLKPORT_SPI_EXT_CLK			0x206c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPI_CAM1_IPCLKPORT_PCLK				0x2070
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPI_CAM1_IPCLKPORT_SPI_EXT_CLK			0x2074
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SYSREG_PERIC1_IPCLKPORT_PCLK			0x2078
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_UART_BT_IPCLKPORT_EXT_UCLK			0x207c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_UART_BT_IPCLKPORT_PCLK				0x2080
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI04_IPCLKPORT_I_PCLK				0x2084
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI04_IPCLKPORT_I_SCLK_USI			0x2088
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI05_IPCLKPORT_I_PCLK				0x208c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI05_IPCLKPORT_I_SCLK_USI			0x2090
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI06_IPCLKPORT_I_PCLK				0x2094
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI06_IPCLKPORT_I_SCLK_USI			0x2098
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI07_IPCLKPORT_I_PCLK				0x209c
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI07_IPCLKPORT_I_SCLK_USI			0x20a0
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI08_IPCLKPORT_I_PCLK				0x20a4
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI08_IPCLKPORT_I_SCLK_USI			0x20a8
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI09_IPCLKPORT_I_PCLK				0x20ac
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI09_IPCLKPORT_I_SCLK_USI			0x20b0
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI10_IPCLKPORT_I_PCLK				0x20b4
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI10_IPCLKPORT_I_SCLK_USI			0x20b8
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI11_IPCLKPORT_I_PCLK				0x20bc
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI11_IPCLKPORT_I_SCLK_USI			0x20c0
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI12_IPCLKPORT_I_PCLK				0x20c4
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI12_IPCLKPORT_I_SCLK_USI			0x20c8
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI13_IPCLKPORT_I_PCLK				0x20cc
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI13_IPCLKPORT_I_SCLK_USI			0x20d0
#define CLK_CON_GAT_GOUT_BLK_PERIC1_UID_XIU_P_PERIC1_IPCLKPORT_ACLK			0x20d4
/* Q-channel controls: ENABLE is bit 0, CLOCK_REQ bit 1 */
#define QCH_CON_USI04_QCH							0x3044
#define QCH_CON_USI05_QCH							0x3048
#define QCH_CON_USI06_QCH							0x304c
#define QCH_CON_USI07_QCH							0x3050
#define QCH_CON_USI08_QCH							0x3054
#define QCH_CON_USI09_QCH							0x3058
#define QCH_CON_USI10_QCH							0x305c
#define QCH_CON_USI11_QCH							0x3060
#define QCH_CON_USI12_QCH							0x3064
#define QCH_CON_USI13_QCH							0x3068

static const unsigned long peric1_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_PERIC1_BUS_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_BUS_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_SPEEDY2_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_SPEEDY2_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_SPI_CAM0_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_SPI_CAM0_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_SPI_CAM1_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_SPI_CAM1_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_UART_BT_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_UART_BT_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI04_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI04_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI05_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI05_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI06_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI06_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI07_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI07_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI08_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI08_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI09_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI09_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI10_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI10_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI11_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI11_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI12_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI12_USER,
	PLL_CON0_MUX_CLKCMU_PERIC1_USI13_USER,
	PLL_CON2_MUX_CLKCMU_PERIC1_USI13_USER,
	/* Q-channel controls, outside the gate range: save and restore only */
	QCH_CON_USI04_QCH,
	QCH_CON_USI05_QCH,
	QCH_CON_USI06_QCH,
	QCH_CON_USI07_QCH,
	QCH_CON_USI08_QCH,
	QCH_CON_USI09_QCH,
	QCH_CON_USI10_QCH,
	QCH_CON_USI11_QCH,
	QCH_CON_USI12_QCH,
	QCH_CON_USI13_QCH,
};

/* List of parent clocks for Muxes in CMU_PERIC1 */
PNAME(mout_peric1_bus_user_p)		= { "oscclk", "dout_cmu_peric1_bus" };
PNAME(mout_peric1_speedy2_user_p)	= { "oscclk",
					    "dout_cmu_peric1_speedy2" };
PNAME(mout_peric1_spi_cam0_user_p)	= { "oscclk",
					    "dout_cmu_peric1_spi_cam0" };
PNAME(mout_peric1_spi_cam1_user_p)	= { "oscclk",
					    "dout_cmu_peric1_spi_cam1" };
PNAME(mout_peric1_uart_bt_user_p)	= { "oscclk",
					    "dout_cmu_peric1_uart_bt" };
PNAME(mout_peric1_usi04_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi04" };
PNAME(mout_peric1_usi05_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi05" };
PNAME(mout_peric1_usi06_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi06" };
PNAME(mout_peric1_usi07_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi07" };
PNAME(mout_peric1_usi08_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi08" };
PNAME(mout_peric1_usi09_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi09" };
PNAME(mout_peric1_usi10_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi10" };
PNAME(mout_peric1_usi11_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi11" };
PNAME(mout_peric1_usi12_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi12" };
PNAME(mout_peric1_usi13_user_p)		= { "oscclk",
					    "dout_cmu_peric1_usi13" };

static const struct samsung_mux_clock peric1_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PERIC1_BUS_USER, "mout_peric1_bus_user",
	    mout_peric1_bus_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_BUS_USER, 4, 1),
	MUX(CLK_MOUT_PERIC1_SPEEDY2_USER, "mout_peric1_speedy2_user",
	    mout_peric1_speedy2_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC1_SPEEDY2_USER, 4, 1),
	MUX(CLK_MOUT_PERIC1_SPI_CAM0_USER, "mout_peric1_spi_cam0_user",
	    mout_peric1_spi_cam0_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC1_SPI_CAM0_USER, 4, 1),
	MUX(CLK_MOUT_PERIC1_SPI_CAM1_USER, "mout_peric1_spi_cam1_user",
	    mout_peric1_spi_cam1_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC1_SPI_CAM1_USER, 4, 1),
	MUX(CLK_MOUT_PERIC1_UART_BT_USER, "mout_peric1_uart_bt_user",
	    mout_peric1_uart_bt_user_p,
	    PLL_CON0_MUX_CLKCMU_PERIC1_UART_BT_USER, 4, 1),
	MUX(CLK_MOUT_PERIC1_USI04_USER, "mout_peric1_usi04_user",
	    mout_peric1_usi04_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI04_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI05_USER, "mout_peric1_usi05_user",
	    mout_peric1_usi05_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI05_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI06_USER, "mout_peric1_usi06_user",
	    mout_peric1_usi06_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI06_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI07_USER, "mout_peric1_usi07_user",
	    mout_peric1_usi07_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI07_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI08_USER, "mout_peric1_usi08_user",
	    mout_peric1_usi08_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI08_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI09_USER, "mout_peric1_usi09_user",
	    mout_peric1_usi09_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI09_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI10_USER, "mout_peric1_usi10_user",
	    mout_peric1_usi10_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI10_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI11_USER, "mout_peric1_usi11_user",
	    mout_peric1_usi11_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI11_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI12_USER, "mout_peric1_usi12_user",
	    mout_peric1_usi12_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI12_USER,
	    4, 1),
	MUX(CLK_MOUT_PERIC1_USI13_USER, "mout_peric1_usi13_user",
	    mout_peric1_usi13_user_p, PLL_CON0_MUX_CLKCMU_PERIC1_USI13_USER,
	    4, 1),
};

static const struct samsung_gate_clock peric1_gate_clks[] __initconst = {
	GATE(CLK_GOUT_PERIC1_PERIC1_CMU_PERIC1_PCLK,
	     "gout_peric1_peric1_cmu_peric1_pclk", "mout_peric1_bus_user",
	     CLK_CON_GAT_CLK_BLK_PERIC1_UID_PERIC1_CMU_PERIC1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC1_RSTNSYNC_CLK_PERIC1_SPEEDY2_CLK,
	     "gout_peric1_rstnsync_clk_peric1_speedy2_clk",
	     "mout_peric1_speedy2_user",
	     CLK_CON_GAT_CLK_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_SPEEDY2_IPCLKPORT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_AXI2APB_PERIC1P0_ACLK,
	     "gout_peric1_axi2apb_peric1p0_aclk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_AXI2APB_PERIC1P0_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC1_AXI2APB_PERIC1P1_ACLK,
	     "gout_peric1_axi2apb_peric1p1_aclk", "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_AXI2APB_PERIC1P1_IPCLKPORT_ACLK,
	    21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC1_AXI2APB_PERIC1P2_ACLK,
	     "gout_peric1_axi2apb_peric1p2_aclk", "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_AXI2APB_PERIC1P2_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC1_GPIO_PERIC1_PCLK,
	     "gout_peric1_gpio_peric1_pclk", "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_GPIO_PERIC1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC1_HSI2C_CAM0_IPCLK, "gout_peric1_hsi2c_cam0_ipclk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_HSI2C_CAM0_IPCLKPORT_IPCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_HSI2C_CAM1_IPCLK,
	     "gout_peric1_hsi2c_cam1_ipclk", "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_HSI2C_CAM1_IPCLKPORT_IPCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_HSI2C_CAM2_IPCLK,
	     "gout_peric1_hsi2c_cam2_ipclk", "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_HSI2C_CAM2_IPCLKPORT_IPCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_HSI2C_CAM3_IPCLK, "gout_peric1_hsi2c_cam3_ipclk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_HSI2C_CAM3_IPCLKPORT_IPCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_LHM_AXI_P_PERIC1_I_CLK,
	     "gout_peric1_lhm_axi_p_peric1_i_clk", "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_LHM_AXI_P_PERIC1_IPCLKPORT_I_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC1_PMU_PERIC1_PCLK, "gout_peric1_pmu_peric1_pclk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_PMU_PERIC1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC1_RSTNSYNC_CLK_PERIC1_BUSP_CLK,
	     "gout_peric1_rstnsync_clk_peric1_busp_clk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_RSTNSYNC_CLK_PERIC1_BUSP_IPCLKPORT_CLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC1_SPEEDY2_DDI1_CLK, "gout_peric1_speedy2_ddi1_clk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI1_IPCLKPORT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPEEDY2_DDI1_SCLK,
	     "gout_peric1_speedy2_ddi1_sclk", "mout_peric1_speedy2_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI1_IPCLKPORT_SCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPEEDY2_DDI2_CLK, "gout_peric1_speedy2_ddi2_clk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI2_IPCLKPORT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPEEDY2_DDI2_SCLK,
	     "gout_peric1_speedy2_ddi2_sclk", "mout_peric1_speedy2_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI2_IPCLKPORT_SCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPEEDY2_DDI_CLK, "gout_peric1_speedy2_ddi_clk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI_IPCLKPORT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPEEDY2_DDI_SCLK, "gout_peric1_speedy2_ddi_sclk",
	     "mout_peric1_speedy2_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_DDI_IPCLKPORT_SCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPEEDY2_TSP1_CLK, "gout_peric1_speedy2_tsp1_clk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_TSP1_IPCLKPORT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPEEDY2_TSP2_CLK, "gout_peric1_speedy2_tsp2_clk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPEEDY2_TSP2_IPCLKPORT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPI_CAM0_PCLK, "gout_peric1_spi_cam0_pclk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPI_CAM0_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPI_CAM0_SPI_EXT_CLK,
	     "gout_peric1_spi_cam0_spi_ext_clk", "mout_peric1_spi_cam0_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPI_CAM0_IPCLKPORT_SPI_EXT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPI_CAM1_PCLK, "gout_peric1_spi_cam1_pclk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPI_CAM1_IPCLKPORT_PCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SPI_CAM1_SPI_EXT_CLK,
	     "gout_peric1_spi_cam1_spi_ext_clk", "mout_peric1_spi_cam1_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SPI_CAM1_IPCLKPORT_SPI_EXT_CLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_SYSREG_PERIC1_PCLK,
	     "gout_peric1_sysreg_peric1_pclk", "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_SYSREG_PERIC1_IPCLKPORT_PCLK,
	     21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC1_UART_BT_EXT_UCLK, "gout_peric1_uart_bt_ext_uclk",
	     "mout_peric1_uart_bt_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_UART_BT_IPCLKPORT_EXT_UCLK,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_UART_BT_PCLK, "gout_peric1_uart_bt_pclk",
	     "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_UART_BT_IPCLKPORT_PCLK, 21, 0, 0),
	/* As in CMU_PERIC0, each USI's Q-channel request is its PCLK's parent */
	GATE(0, "gout_peric1_usi04_qch", "mout_peric1_bus_user",
	     QCH_CON_USI04_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI04_I_PCLK, "gout_peric1_usi04_i_pclk",
	     "gout_peric1_usi04_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI04_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI04_I_SCLK_USI, "gout_peric1_usi04_i_sclk_usi",
	     "mout_peric1_usi04_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI04_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric1_usi05_qch", "mout_peric1_bus_user",
	     QCH_CON_USI05_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI05_I_PCLK, "gout_peric1_usi05_i_pclk",
	     "gout_peric1_usi05_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI05_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI05_I_SCLK_USI, "gout_peric1_usi05_i_sclk_usi",
	     "mout_peric1_usi05_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI05_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric1_usi06_qch", "mout_peric1_bus_user",
	     QCH_CON_USI06_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI06_I_PCLK, "gout_peric1_usi06_i_pclk",
	     "gout_peric1_usi06_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI06_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI06_I_SCLK_USI, "gout_peric1_usi06_i_sclk_usi",
	     "mout_peric1_usi06_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI06_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric1_usi07_qch", "mout_peric1_bus_user",
	     QCH_CON_USI07_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI07_I_PCLK, "gout_peric1_usi07_i_pclk",
	     "gout_peric1_usi07_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI07_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI07_I_SCLK_USI, "gout_peric1_usi07_i_sclk_usi",
	     "mout_peric1_usi07_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI07_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric1_usi08_qch", "mout_peric1_bus_user",
	     QCH_CON_USI08_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI08_I_PCLK, "gout_peric1_usi08_i_pclk",
	     "gout_peric1_usi08_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI08_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI08_I_SCLK_USI, "gout_peric1_usi08_i_sclk_usi",
	     "mout_peric1_usi08_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI08_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric1_usi09_qch", "mout_peric1_bus_user",
	     QCH_CON_USI09_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI09_I_PCLK, "gout_peric1_usi09_i_pclk",
	     "gout_peric1_usi09_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI09_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI09_I_SCLK_USI, "gout_peric1_usi09_i_sclk_usi",
	     "mout_peric1_usi09_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI09_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric1_usi10_qch", "mout_peric1_bus_user",
	     QCH_CON_USI10_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI10_I_PCLK, "gout_peric1_usi10_i_pclk",
	     "gout_peric1_usi10_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI10_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI10_I_SCLK_USI, "gout_peric1_usi10_i_sclk_usi",
	     "mout_peric1_usi10_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI10_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric1_usi11_qch", "mout_peric1_bus_user",
	     QCH_CON_USI11_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI11_I_PCLK, "gout_peric1_usi11_i_pclk",
	     "gout_peric1_usi11_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI11_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI11_I_SCLK_USI, "gout_peric1_usi11_i_sclk_usi",
	     "mout_peric1_usi11_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI11_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric1_usi12_qch", "mout_peric1_bus_user",
	     QCH_CON_USI12_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI12_I_PCLK, "gout_peric1_usi12_i_pclk",
	     "gout_peric1_usi12_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI12_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI12_I_SCLK_USI, "gout_peric1_usi12_i_sclk_usi",
	     "mout_peric1_usi12_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI12_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(0, "gout_peric1_usi13_qch", "mout_peric1_bus_user",
	     QCH_CON_USI13_QCH, 1, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI13_I_PCLK, "gout_peric1_usi13_i_pclk",
	     "gout_peric1_usi13_qch",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI13_IPCLKPORT_I_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI13_I_SCLK_USI, "gout_peric1_usi13_i_sclk_usi",
	     "mout_peric1_usi13_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_USI13_IPCLKPORT_I_SCLK_USI,
	     21, 0, 0),
	GATE(CLK_GOUT_PERIC1_XIU_P_PERIC1_ACLK,
	     "gout_peric1_xiu_p_peric1_aclk", "mout_peric1_bus_user",
	     CLK_CON_GAT_GOUT_BLK_PERIC1_UID_XIU_P_PERIC1_IPCLKPORT_ACLK,
	     21, CLK_IS_CRITICAL, 0),
};

/* USER muxes the vendor kernel has on OSCCLK in SLEEP, as in CMU_FSYS0 */
static const struct samsung_clk_reg_dump peric1_suspend_regs[] = {
	{ PLL_CON0_MUX_CLKCMU_PERIC1_BUS_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_SPI_CAM0_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_SPI_CAM1_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_UART_BT_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_USI05_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_USI06_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_USI08_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_USI09_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_USI10_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_USI11_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_USI12_USER, 0x0 },
	{ PLL_CON0_MUX_CLKCMU_PERIC1_USI13_USER, 0x0 },
};

static const struct samsung_cmu_info peric1_cmu_info __initconst = {
	.mux_clks		= peric1_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(peric1_mux_clks),
	.gate_clks		= peric1_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(peric1_gate_clks),
	.nr_clk_ids		= CLKS_NR_PERIC1,
	.clk_regs		= peric1_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(peric1_clk_regs),
	.suspend_regs		= peric1_suspend_regs,
	.nr_suspend_regs	= ARRAY_SIZE(peric1_suspend_regs),
	.clk_name		= "bus",
	.hwacg_gates		= true,
};

/* ---- CMU_APM ------------------------------------------------------------- */

/* Register Offset definitions for CMU_APM (0x16400000) */
#define PLL_CON0_MUX_CLKCMU_APM_BUS_USER					0x0100
#define CLK_CON_GAT_CLK_BLK_APM_UID_APM_CMU_APM_IPCLKPORT_PCLK			0x2000
#define CLK_CON_GAT_GOUT_BLK_APM_UID_APM_IPCLKPORT_ACLK_CPU			0x2018
#define CLK_CON_GAT_GOUT_BLK_APM_UID_APM_IPCLKPORT_ACLK_SYS			0x201c
#define CLK_CON_GAT_GOUT_BLK_APM_UID_APM_IPCLKPORT_ACLK_XIU_D_ALIVE		0x2020
#define CLK_CON_GAT_GOUT_BLK_APM_UID_LHM_AXI_P_ALIVE_IPCLKPORT_I_CLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_APM_UID_LHS_AXI_D_ALIVE_IPCLKPORT_I_CLK		0x2030
#define CLK_CON_GAT_GOUT_BLK_APM_UID_MAILBOX_APM2AP_IPCLKPORT_PCLK		0x2038
#define CLK_CON_GAT_GOUT_BLK_APM_UID_MAILBOX_APM2CP_IPCLKPORT_PCLK		0x203c
#define CLK_CON_GAT_GOUT_BLK_APM_UID_MAILBOX_APM2GNSS_IPCLKPORT_PCLK		0x2040
#define CLK_CON_GAT_GOUT_BLK_APM_UID_RSTNSYNC_CLK_APM_BUS_IPCLKPORT_CLK		0x2044
#define CLK_CON_GAT_GOUT_BLK_APM_UID_SCAN2AXI_IPCLKPORT_ACLK			0x2048
#define CLK_CON_GAT_GOUT_BLK_APM_UID_SYSREG_APM_IPCLKPORT_PCLK			0x204c
#define CLK_CON_GAT_GOUT_BLK_APM_UID_WDT_APM_IPCLKPORT_PCLK			0x2050

/* List of parent clocks for Muxes in CMU_APM */
PNAME(mout_apm_bus_user_p)		= { "oscclk", "dout_cmu_apm_bus" };

static const struct samsung_mux_clock apm_mux_clks[] __initconst = {
	MUX(CLK_MOUT_APM_BUS_USER, "mout_apm_bus_user", mout_apm_bus_user_p,
	    PLL_CON0_MUX_CLKCMU_APM_BUS_USER, 4, 1),
};

static const struct samsung_gate_clock apm_gate_clks[] __initconst = {
	GATE(CLK_GOUT_APM_CMU_APM_PCLK, "gout_apm_cmu_apm_pclk",
	     "mout_apm_bus_user",
	     CLK_CON_GAT_CLK_BLK_APM_UID_APM_CMU_APM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_ACLK_CPU, "gout_apm_aclk_cpu", "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_APM_IPCLKPORT_ACLK_CPU, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_ACLK_SYS, "gout_apm_aclk_sys", "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_APM_IPCLKPORT_ACLK_SYS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_ACLK_XIU_D_ALIVE, "gout_apm_aclk_xiu_d_alive",
	     "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_APM_IPCLKPORT_ACLK_XIU_D_ALIVE, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_LHM_AXI_P_ALIVE_I_CLK,
	     "gout_apm_lhm_axi_p_alive_i_clk", "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_LHM_AXI_P_ALIVE_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_LHS_AXI_D_ALIVE_I_CLK,
	     "gout_apm_lhs_axi_d_alive_i_clk", "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_LHS_AXI_D_ALIVE_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_MAILBOX_APM2AP_PCLK, "gout_apm_mailbox_apm2ap_pclk",
	     "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_MAILBOX_APM2AP_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_MAILBOX_APM2CP_PCLK, "gout_apm_mailbox_apm2cp_pclk",
	     "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_MAILBOX_APM2CP_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_MAILBOX_APM2GNSS_PCLK,
	     "gout_apm_mailbox_apm2gnss_pclk", "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_MAILBOX_APM2GNSS_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_RSTNSYNC_CLK_APM_BUS_CLK,
	     "gout_apm_rstnsync_clk_apm_bus_clk", "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_RSTNSYNC_CLK_APM_BUS_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_SCAN2AXI_ACLK, "gout_apm_scan2axi_aclk",
	     "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_SCAN2AXI_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_SYSREG_APM_PCLK, "gout_apm_sysreg_apm_pclk",
	     "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_SYSREG_APM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_WDT_APM_PCLK, "gout_apm_wdt_apm_pclk",
	     "mout_apm_bus_user",
	     CLK_CON_GAT_GOUT_BLK_APM_UID_WDT_APM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info apm_cmu_info __initconst = {
	.mux_clks		= apm_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(apm_mux_clks),
	.gate_clks		= apm_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(apm_gate_clks),
	.nr_clk_ids		= CLKS_NR_APM,
	.clk_name		= "bus",
};

/* ---- CMU_CPUCL0 ---------------------------------------------------------- */

/* Register Offset definitions for CMU_CPUCL0 (0x16800000) */
#define PLL_LOCKTIME_PLL_CPUCL0							0x0000
#define PLL_CON0_MUX_CLKCMU_CPUCL0_SWITCH_USER					0x0100
#define PLL_CON0_PLL_CPUCL0							0x0120
#define CLK_CON_MUX_MUX_CLK_CPUCL0_PLL						0x1000
#define CLK_CON_DIV_DIV_CLK_CLUSTER0_ACLK					0x1800
#define CLK_CON_DIV_DIV_CLK_CLUSTER0_ATCLK					0x1804
#define CLK_CON_DIV_DIV_CLK_CPUCL0_CMUREF					0x1808
#define CLK_CON_DIV_DIV_CLK_CPUCL0_CPU						0x180c
#define CLK_CON_DIV_DIV_CLK_CPUCL0_PCLK						0x1810
#define CLK_CON_DIV_DIV_CLK_CPUCL0_PCLKDBG					0x1814
#define CLK_CON_GAT_CLK_BLK_CPUCL0_UID_CPUCL0_CMU_CPUCL0_IPCLKPORT_PCLK		0x2000
#define CLK_CON_GAT_CLK_BLK_CPUCL0_UID_DROOP_DETECTOR_CPUCL0_GRP0_IPCLKPORT_CK_IN	0x2004
#define CLK_CON_GAT_CLK_BLK_CPUCL0_UID_DROOP_DETECTOR_CPUCL0_GRP1_IPCLKPORT_CK_IN	0x2008
#define CLK_CON_GAT_CLK_BLK_CPUCL0_UID_HPM_CPUCL0_IPCLKPORT_HPM_TARGETCLK_C	0x200c
#define CLK_CON_GAT_GATE_CLK_CPUCL0_CPU						0x2020
#define CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_AXI2APB_CPUCL0_IPCLKPORT_ACLK		0x2024
#define CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_BUSIF_DROOPDETECTOR_CPUCL0_IPCLKPORT_PCLK	0x2028
#define CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_BUSIF_HPMCPUCL0_IPCLKPORT_PCLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_LHM_AXI_P_CPUCL0_IPCLKPORT_I_CLK	0x2034
#define CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_PMU_CPUCL0_IPCLKPORT_PCLK		0x2038
#define CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_RSTNSYNC_CLK_CPUCL0_PCLK_IPCLKPORT_CLK	0x203c
#define CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_SYSREG_CPUCL0_IPCLKPORT_PCLK		0x2040

/* List of parent clocks for Muxes in CMU_CPUCL0 */
/* PLL_CON0 bit 4 selects between OSCCLK and the PLL output */
PNAME(mout_pll_cpucl0_p)		= { "oscclk", "fout_cpucl0_pll" };
PNAME(mout_cpucl0_switch_user_p)	= { "oscclk", "dout_cmu_cpucl0_switch" };
PNAME(mout_cpucl0_pll_p)		= { "mout_pll_cpucl0", "mout_cpucl0_switch_user" };

static const struct samsung_pll_clock cpucl0_pll_clks[] __initconst = {
	PLL(pll_1051x, CLK_FOUT_CPUCL0_PLL, "fout_cpucl0_pll", "oscclk",
	    PLL_LOCKTIME_PLL_CPUCL0, PLL_CON0_PLL_CPUCL0, NULL),
};

static const struct samsung_mux_clock cpucl0_mux_clks[] __initconst = {
	MUX_F(CLK_MOUT_PLL_CPUCL0, "mout_pll_cpucl0", mout_pll_cpucl0_p,
	      PLL_CON0_PLL_CPUCL0, 4, 1, CLK_GET_RATE_NOCACHE,
	      CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_CPUCL0_SWITCH_USER, "mout_cpucl0_switch_user",
	      mout_cpucl0_switch_user_p, PLL_CON0_MUX_CLKCMU_CPUCL0_SWITCH_USER,
	      4, 1, CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_CPUCL0_PLL, "mout_cpucl0_pll", mout_cpucl0_pll_p,
	      CLK_CON_MUX_MUX_CLK_CPUCL0_PLL, 0, 1, CLK_GET_RATE_NOCACHE,
	      CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock cpucl0_div_clks[] __initconst = {
	DIV_F(CLK_DOUT_CPUCL0_CLUSTER0_ACLK, "dout_cpucl0_cluster0_aclk",
	      "gout_cpucl0_cpu", CLK_CON_DIV_DIV_CLK_CLUSTER0_ACLK, 0, 4,
	      CLK_GET_RATE_NOCACHE, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL0_CLUSTER0_ATCLK, "dout_cpucl0_cluster0_atclk",
	      "gout_cpucl0_cpu", CLK_CON_DIV_DIV_CLK_CLUSTER0_ATCLK, 0, 4,
	      CLK_GET_RATE_NOCACHE, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL0_CMUREF, "dout_cpucl0_cmuref", "dout_cpucl0_cpu",
	      CLK_CON_DIV_DIV_CLK_CPUCL0_CMUREF, 0, 3, CLK_GET_RATE_NOCACHE,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL0_CPU, "dout_cpucl0_cpu", "mout_cpucl0_pll",
	      CLK_CON_DIV_DIV_CLK_CPUCL0_CPU, 0, 12, CLK_GET_RATE_NOCACHE,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL0_PCLK, "dout_cpucl0_pclk", "dout_cpucl0_cpu",
	      CLK_CON_DIV_DIV_CLK_CPUCL0_PCLK, 0, 4, CLK_GET_RATE_NOCACHE,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL0_PCLKDBG, "dout_cpucl0_pclkdbg", "gout_cpucl0_cpu",
	      CLK_CON_DIV_DIV_CLK_CPUCL0_PCLKDBG, 0, 4, CLK_GET_RATE_NOCACHE,
	      CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock cpucl0_gate_clks[] __initconst = {
	GATE(CLK_GOUT_CPUCL0_CMU_CPUCL0_PCLK, "gout_cpucl0_cmu_cpucl0_pclk",
	     "dout_cpucl0_pclk",
	     CLK_CON_GAT_CLK_BLK_CPUCL0_UID_CPUCL0_CMU_CPUCL0_IPCLKPORT_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_DROOP_DETECTOR_CPUCL0_GRP0_CK_IN,
	     "gout_cpucl0_droop_detector_cpucl0_grp0_ck_in",
	     "gout_droopdetector",
	     CLK_CON_GAT_CLK_BLK_CPUCL0_UID_DROOP_DETECTOR_CPUCL0_GRP0_IPCLKPORT_CK_IN,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_DROOP_DETECTOR_CPUCL0_GRP1_CK_IN,
	     "gout_cpucl0_droop_detector_cpucl0_grp1_ck_in",
	     "gout_droopdetector",
	     CLK_CON_GAT_CLK_BLK_CPUCL0_UID_DROOP_DETECTOR_CPUCL0_GRP1_IPCLKPORT_CK_IN,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_HPM_CPUCL0_HPM_TARGETCLK_C,
	     "gout_cpucl0_hpm_cpucl0_hpm_targetclk_c", "dout_cmu_hpm",
	     CLK_CON_GAT_CLK_BLK_CPUCL0_UID_HPM_CPUCL0_IPCLKPORT_HPM_TARGETCLK_C,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_CPU, "gout_cpucl0_cpu", "dout_cpucl0_cpu",
	     CLK_CON_GAT_GATE_CLK_CPUCL0_CPU, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_AXI2APB_CPUCL0_ACLK,
	     "gout_cpucl0_axi2apb_cpucl0_aclk", "dout_cpucl0_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_AXI2APB_CPUCL0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_BUSIF_DROOPDETECTOR_CPUCL0_PCLK,
	     "gout_cpucl0_busif_droopdetector_cpucl0_pclk", "dout_cpucl0_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_BUSIF_DROOPDETECTOR_CPUCL0_IPCLKPORT_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_BUSIF_HPMCPUCL0_PCLK,
	     "gout_cpucl0_busif_hpmcpucl0_pclk", "dout_cpucl0_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_BUSIF_HPMCPUCL0_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_LHM_AXI_P_CPUCL0_I_CLK,
	     "gout_cpucl0_lhm_axi_p_cpucl0_i_clk", "dout_cpucl0_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_LHM_AXI_P_CPUCL0_IPCLKPORT_I_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_PMU_CPUCL0_PCLK, "gout_cpucl0_pmu_cpucl0_pclk",
	     "dout_cpucl0_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_PMU_CPUCL0_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_RSTNSYNC_CLK_CPUCL0_PCLK_CLK,
	     "gout_cpucl0_rstnsync_clk_cpucl0_pclk_clk", "dout_cpucl0_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_RSTNSYNC_CLK_CPUCL0_PCLK_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL0_SYSREG_CPUCL0_PCLK,
	     "gout_cpucl0_sysreg_cpucl0_pclk", "dout_cpucl0_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL0_UID_SYSREG_CPUCL0_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info cpucl0_cmu_info __initconst = {
	.pll_clks		= cpucl0_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(cpucl0_pll_clks),
	.mux_clks		= cpucl0_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cpucl0_mux_clks),
	.div_clks		= cpucl0_div_clks,
	.nr_div_clks		= ARRAY_SIZE(cpucl0_div_clks),
	.gate_clks		= cpucl0_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(cpucl0_gate_clks),
	.nr_clk_ids		= CLKS_NR_CPUCL0,
};

/* ---- CMU_CPUCL1 ---------------------------------------------------------- */

/* Register Offset definitions for CMU_CPUCL1 (0x16900000) */
#define PLL_LOCKTIME_PLL_CPUCL1							0x0000
#define PLL_CON0_MUX_CLKCMU_CPUCL1_SWITCH_USER					0x0100
#define PLL_CON0_PLL_CPUCL1							0x0120
#define CLK_CON_MUX_MUX_CLK_CPUCL1_PLL						0x1000
#define CLK_CON_DIV_DIV_CLK_CLUSTER1_ACLK					0x1800
#define CLK_CON_DIV_DIV_CLK_CLUSTER1_ATCLK					0x1804
#define CLK_CON_DIV_DIV_CLK_CLUSTER1_CNTCLK					0x1808
#define CLK_CON_DIV_DIV_CLK_CLUSTER1_PCLKDBG					0x180c
#define CLK_CON_DIV_DIV_CLK_CPUCL1_CMUREF					0x1810
#define CLK_CON_DIV_DIV_CLK_CPUCL1_CPU						0x1814
#define CLK_CON_DIV_DIV_CLK_CPUCL1_PCLK						0x1818
#define CLK_CON_GAT_CLK_BLK_CPUCL1_UID_CPUCL1_CMU_CPUCL1_IPCLKPORT_PCLK		0x2000
#define CLK_CON_GAT_CLK_BLK_CPUCL1_UID_HPM_CPUCL1_IPCLKPORT_HPM_TARGETCLK_C	0x2004
#define CLK_CON_GAT_GATE_CLK_CPUCL1_CPU						0x2018
#define CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_AXI2APB_CPUCL1_IPCLKPORT_ACLK		0x201c
#define CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_BUSIF_HPMCPUCL1_IPCLKPORT_PCLK		0x2020
#define CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_LHM_AXI_P_CPUCL1_IPCLKPORT_I_CLK	0x2028
#define CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_PMU_CPUCL1_IPCLKPORT_PCLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_RSTNSYNC_CLK_CPUCL1_PCLK_IPCLKPORT_CLK	0x2030
#define CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_SYSREG_CPUCL1_IPCLKPORT_PCLK		0x2034

/* List of parent clocks for Muxes in CMU_CPUCL1 */
/* PLL_CON0 bit 4 selects between OSCCLK and the PLL output */
PNAME(mout_pll_cpucl1_p)		= { "oscclk", "fout_cpucl1_pll" };
PNAME(mout_cpucl1_switch_user_p)	= { "oscclk", "dout_cmu_cpucl1_switch" };
PNAME(mout_cpucl1_pll_p)		= { "mout_pll_cpucl1", "mout_cpucl1_switch_user" };

static const struct samsung_pll_clock cpucl1_pll_clks[] __initconst = {
	PLL(pll_1051x, CLK_FOUT_CPUCL1_PLL, "fout_cpucl1_pll", "oscclk",
	    PLL_LOCKTIME_PLL_CPUCL1, PLL_CON0_PLL_CPUCL1, NULL),
};

static const struct samsung_mux_clock cpucl1_mux_clks[] __initconst = {
	MUX_F(CLK_MOUT_PLL_CPUCL1, "mout_pll_cpucl1", mout_pll_cpucl1_p,
	      PLL_CON0_PLL_CPUCL1, 4, 1, CLK_GET_RATE_NOCACHE,
	      CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_CPUCL1_SWITCH_USER, "mout_cpucl1_switch_user",
	      mout_cpucl1_switch_user_p, PLL_CON0_MUX_CLKCMU_CPUCL1_SWITCH_USER,
	      4, 1, CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_CPUCL1_PLL, "mout_cpucl1_pll", mout_cpucl1_pll_p,
	      CLK_CON_MUX_MUX_CLK_CPUCL1_PLL, 0, 1, CLK_GET_RATE_NOCACHE,
	      CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock cpucl1_div_clks[] __initconst = {
	DIV_F(CLK_DOUT_CPUCL1_CLUSTER1_ACLK, "dout_cpucl1_cluster1_aclk",
	      "gout_cpucl1_cpu", CLK_CON_DIV_DIV_CLK_CLUSTER1_ACLK, 0, 4,
	      CLK_GET_RATE_NOCACHE, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL1_CLUSTER1_ATCLK, "dout_cpucl1_cluster1_atclk",
	      "gout_cpucl1_cpu", CLK_CON_DIV_DIV_CLK_CLUSTER1_ATCLK, 0, 4,
	      CLK_GET_RATE_NOCACHE, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL1_CLUSTER1_CNTCLK, "dout_cpucl1_cluster1_cntclk",
	      "gout_cpucl1_cpu", CLK_CON_DIV_DIV_CLK_CLUSTER1_CNTCLK, 0, 4,
	      CLK_GET_RATE_NOCACHE, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL1_CLUSTER1_PCLKDBG, "dout_cpucl1_cluster1_pclkdbg",
	      "gout_cpucl1_cpu", CLK_CON_DIV_DIV_CLK_CLUSTER1_PCLKDBG, 0, 4,
	      CLK_GET_RATE_NOCACHE, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL1_CMUREF, "dout_cpucl1_cmuref", "dout_cpucl1_cpu",
	      CLK_CON_DIV_DIV_CLK_CPUCL1_CMUREF, 0, 3, CLK_GET_RATE_NOCACHE,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL1_CPU, "dout_cpucl1_cpu", "mout_cpucl1_pll",
	      CLK_CON_DIV_DIV_CLK_CPUCL1_CPU, 0, 12, CLK_GET_RATE_NOCACHE,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_CPUCL1_PCLK, "dout_cpucl1_pclk", "dout_cpucl1_cpu",
	      CLK_CON_DIV_DIV_CLK_CPUCL1_PCLK, 0, 4, CLK_GET_RATE_NOCACHE,
	      CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock cpucl1_gate_clks[] __initconst = {
	GATE(CLK_GOUT_CPUCL1_CMU_CPUCL1_PCLK, "gout_cpucl1_cmu_cpucl1_pclk",
	     "dout_cpucl1_pclk",
	     CLK_CON_GAT_CLK_BLK_CPUCL1_UID_CPUCL1_CMU_CPUCL1_IPCLKPORT_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL1_HPM_CPUCL1_HPM_TARGETCLK_C,
	     "gout_cpucl1_hpm_cpucl1_hpm_targetclk_c", "dout_cmu_hpm",
	     CLK_CON_GAT_CLK_BLK_CPUCL1_UID_HPM_CPUCL1_IPCLKPORT_HPM_TARGETCLK_C,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL1_CPU, "gout_cpucl1_cpu", "dout_cpucl1_cpu",
	     CLK_CON_GAT_GATE_CLK_CPUCL1_CPU, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL1_AXI2APB_CPUCL1_ACLK,
	     "gout_cpucl1_axi2apb_cpucl1_aclk", "dout_cpucl1_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_AXI2APB_CPUCL1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL1_BUSIF_HPMCPUCL1_PCLK,
	     "gout_cpucl1_busif_hpmcpucl1_pclk", "dout_cpucl1_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_BUSIF_HPMCPUCL1_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL1_LHM_AXI_P_CPUCL1_I_CLK,
	     "gout_cpucl1_lhm_axi_p_cpucl1_i_clk", "dout_cpucl1_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_LHM_AXI_P_CPUCL1_IPCLKPORT_I_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL1_PMU_CPUCL1_PCLK, "gout_cpucl1_pmu_cpucl1_pclk",
	     "dout_cpucl1_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_PMU_CPUCL1_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL1_RSTNSYNC_CLK_CPUCL1_PCLK_CLK,
	     "gout_cpucl1_rstnsync_clk_cpucl1_pclk_clk", "dout_cpucl1_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_RSTNSYNC_CLK_CPUCL1_PCLK_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL1_SYSREG_CPUCL1_PCLK,
	     "gout_cpucl1_sysreg_cpucl1_pclk", "dout_cpucl1_pclk",
	     CLK_CON_GAT_GOUT_BLK_CPUCL1_UID_SYSREG_CPUCL1_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info cpucl1_cmu_info __initconst = {
	.pll_clks		= cpucl1_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(cpucl1_pll_clks),
	.mux_clks		= cpucl1_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cpucl1_mux_clks),
	.div_clks		= cpucl1_div_clks,
	.nr_div_clks		= ARRAY_SIZE(cpucl1_div_clks),
	.gate_clks		= cpucl1_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(cpucl1_gate_clks),
	.nr_clk_ids		= CLKS_NR_CPUCL1,
};

/* ---- CMU_DPU0 ------------------------------------------------------------ */

/* Register Offset definitions for CMU_DPU0 (0x12800000) */
#define PLL_CON0_MUX_CLKCMU_DPU_BUS_USER					0x0100
#define DPU0_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_DIV_DIV_CLK_DPU0_BUSP						0x1808
#define CLK_CON_GAT_CLKCMU_DPU1_BUSD						0x2000
#define CLK_CON_GAT_CLKCMU_DPU1_BUSP						0x2004
#define CLK_CON_GAT_CLK_BLK_DPU0_UID_DPU0_CMU_DPU0_IPCLKPORT_PCLK		0x2008
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_IPCLKPORT_PCLKM		0x2014
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_IPCLKPORT_PCLKS		0x2018
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_SECURE_IPCLKPORT_PCLKM	0x201c
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_SECURE_IPCLKPORT_PCLKS	0x2020
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_IPCLKPORT_PCLKM		0x2024
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_IPCLKPORT_PCLKS		0x2028
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_SECURE_IPCLKPORT_PCLKM		0x202c
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_SECURE_IPCLKPORT_PCLKS		0x2030
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM		0x2034
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKS		0x2038
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_SECURE_IPCLKPORT_PCLKM	0x203c
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_SECURE_IPCLKPORT_PCLKS	0x2040
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_WB_MUX_IPCLKPORT_PCLKM		0x2044
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_WB_MUX_IPCLKPORT_PCLKS		0x2048
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_MIPI_DSIM0_IPCLKPORT_PCLKS		0x204c
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_MIPI_DSIM1_IPCLKPORT_PCLKS		0x2050
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AXI2APB_DPU0P0_IPCLKPORT_ACLK		0x2054
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_AXI2APB_DPU0P1_IPCLKPORT_ACLK		0x2058
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD0_IPCLKPORT_I_ACLK		0x205c
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD0_IPCLKPORT_I_PCLK		0x2060
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD1_IPCLKPORT_I_ACLK		0x2064
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD1_IPCLKPORT_I_PCLK		0x2068
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD2_IPCLKPORT_I_ACLK		0x206c
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD2_IPCLKPORT_I_PCLK		0x2070
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DECON0_IPCLKPORT_ACLK			0x207c
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPP_IPCLKPORT_DPP_G0_ACLK			0x2080
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPP_IPCLKPORT_DPP_G1_ACLK			0x2084
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPP_IPCLKPORT_DPP_VGR_ACLK		0x2088
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH0		0x208c
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH1		0x2090
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH2		0x2094
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH_INTF		0x2098
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_SFR		0x209c
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_WB			0x20a0
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_WB_MUX_IPCLKPORT_DPP_G1_ACLK		0x20a4
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHM_AXI_P0_DPU_IPCLKPORT_I_CLK		0x20a8
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D0_DPU_IPCLKPORT_I_CLK		0x20ac
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D1_DPU_IPCLKPORT_I_CLK		0x20b0
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D2_DPU_IPCLKPORT_I_CLK		0x20b4
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D_USBTV_IPCLKPORT_I_CLK		0x20b8
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_PMU_DPU0_IPCLKPORT_PCLK			0x20bc
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD0_IPCLKPORT_ACLK			0x20c0
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD0_IPCLKPORT_PCLK			0x20c4
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD1_IPCLKPORT_ACLK			0x20c8
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD1_IPCLKPORT_PCLK			0x20cc
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD2_IPCLKPORT_ACLK			0x20d0
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD2_IPCLKPORT_PCLK			0x20d4
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_RSTNSYNC_CLK_DPU0_BUSD_IPCLKPORT_CLK	0x20d8
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_RSTNSYNC_CLK_DPU0_BUSP_IPCLKPORT_CLK	0x20dc
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD0_IPCLKPORT_ACLK		0x20e0
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD0_IPCLKPORT_PCLK_NONSECURE	0x20e4
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD0_IPCLKPORT_PCLK_SECURE	0x20e8
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD1_IPCLKPORT_ACLK		0x20ec
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD1_IPCLKPORT_PCLK_NONSECURE	0x20f0
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD1_IPCLKPORT_PCLK_SECURE	0x20f4
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD2_IPCLKPORT_ACLK		0x20f8
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD2_IPCLKPORT_PCLK_NONSECURE	0x20fc
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD2_IPCLKPORT_PCLK_SECURE	0x2100
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSREG_DPU0_IPCLKPORT_PCLK		0x2108
#define CLK_CON_GAT_GOUT_BLK_DPU0_UID_XIU_P0_DPU_IPCLKPORT_ACLK			0x210c
#define QCH_CON_BTM_DPUD0_QCH							0x3004
#define QCH_CON_BTM_DPUD1_QCH							0x3008
#define QCH_CON_BTM_DPUD2_QCH							0x300c
#define QCH_CON_DECON0_QCH							0x3014
#define QCH_CON_DPP_QCH_DPP_G0							0x3018
#define QCH_CON_DPP_QCH_DPP_G1							0x301c
#define QCH_CON_DPP_QCH_DPP_VGR							0x3020
#define QCH_CON_DPU0_CMU_DPU0_QCH						0x3024
#define QCH_CON_DPU_DMA_QCH							0x3028
#define QCH_CON_DPU_WB_MUX_QCH							0x302c
#define QCH_CON_LHM_AXI_P0_DPU_QCH						0x3030
#define QCH_CON_LHS_AXI_D0_DPU_QCH						0x3034
#define QCH_CON_LHS_AXI_D1_DPU_QCH						0x3038
#define QCH_CON_LHS_AXI_D2_DPU_QCH						0x303c
#define QCH_CON_LHS_AXI_D_USBTV_QCH						0x3040
#define QCH_CON_PMU_DPU0_QCH							0x304c
#define QCH_CON_BCM_DPUD0_QCH							0x3050
#define QCH_CON_BCM_DPUD1_QCH							0x3054
#define QCH_CON_BCM_DPUD2_QCH							0x3058
#define QCH_CON_SYSMMU_DPUD0_QCH						0x305c
#define QCH_CON_SYSMMU_DPUD1_QCH						0x3060
#define QCH_CON_SYSMMU_DPUD2_QCH						0x3064
#define QCH_CON_SYSREG_DPU0_QCH							0x3068

static const unsigned long dpu0_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_DPU_BUS_USER,
	CLK_CON_DIV_DIV_CLK_DPU0_BUSP,
	CLK_CON_GAT_CLKCMU_DPU1_BUSD,
	CLK_CON_GAT_CLKCMU_DPU1_BUSP,
	CLK_CON_GAT_CLK_BLK_DPU0_UID_DPU0_CMU_DPU0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_SECURE_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_SECURE_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_SECURE_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_SECURE_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_SECURE_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_SECURE_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_WB_MUX_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_WB_MUX_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_MIPI_DSIM0_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_MIPI_DSIM1_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AXI2APB_DPU0P0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_AXI2APB_DPU0P1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD0_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD0_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD1_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD1_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD2_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD2_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DECON0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPP_IPCLKPORT_DPP_G0_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPP_IPCLKPORT_DPP_G1_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPP_IPCLKPORT_DPP_VGR_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH0,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH1,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH2,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH_INTF,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_SFR,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_WB,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_WB_MUX_IPCLKPORT_DPP_G1_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHM_AXI_P0_DPU_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D0_DPU_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D1_DPU_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D2_DPU_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D_USBTV_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_PMU_DPU0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD1_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD2_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD2_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_RSTNSYNC_CLK_DPU0_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_RSTNSYNC_CLK_DPU0_BUSP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD0_IPCLKPORT_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD0_IPCLKPORT_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD1_IPCLKPORT_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD1_IPCLKPORT_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD2_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD2_IPCLKPORT_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD2_IPCLKPORT_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSREG_DPU0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU0_UID_XIU_P0_DPU_IPCLKPORT_ACLK,
	QCH_CON_SYSREG_DPU0_QCH,
	QCH_CON_SYSMMU_DPUD2_QCH,
	QCH_CON_SYSMMU_DPUD1_QCH,
	QCH_CON_SYSMMU_DPUD0_QCH,
	QCH_CON_BCM_DPUD2_QCH,
	QCH_CON_BCM_DPUD1_QCH,
	QCH_CON_BCM_DPUD0_QCH,
	QCH_CON_PMU_DPU0_QCH,
	QCH_CON_LHS_AXI_D_USBTV_QCH,
	QCH_CON_LHS_AXI_D2_DPU_QCH,
	QCH_CON_LHS_AXI_D1_DPU_QCH,
	QCH_CON_LHS_AXI_D0_DPU_QCH,
	QCH_CON_LHM_AXI_P0_DPU_QCH,
	QCH_CON_DPU_WB_MUX_QCH,
	QCH_CON_DPU_DMA_QCH,
	QCH_CON_DPU0_CMU_DPU0_QCH,
	QCH_CON_DPP_QCH_DPP_VGR,
	QCH_CON_DPP_QCH_DPP_G1,
	QCH_CON_DPP_QCH_DPP_G0,
	QCH_CON_DECON0_QCH,
	QCH_CON_BTM_DPUD2_QCH,
	QCH_CON_BTM_DPUD1_QCH,
	QCH_CON_BTM_DPUD0_QCH,
	DPU0_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_DPU0 */
PNAME(mout_dpu0_dpu_bus_user_p)		= { "oscclk", "dout_cmu_dpu_bus" };

static const struct samsung_mux_clock dpu0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_DPU0_DPU_BUS_USER, "mout_dpu0_dpu_bus_user",
	    mout_dpu0_dpu_bus_user_p, PLL_CON0_MUX_CLKCMU_DPU_BUS_USER, 4, 1),
};

static const struct samsung_div_clock dpu0_div_clks[] __initconst = {
	DIV(CLK_DOUT_DPU0_BUSP, "dout_dpu0_busp", "mout_dpu0_dpu_bus_user",
	    CLK_CON_DIV_DIV_CLK_DPU0_BUSP, 0, 3),
};

static const struct samsung_gate_clock dpu0_gate_clks[] __initconst = {
	GATE(CLK_GOUT_DPU0_CLKCMU_DPU1_BUSD, "gout_dpu0_clkcmu_dpu1_busd",
	     "mout_dpu0_dpu_bus_user", CLK_CON_GAT_CLKCMU_DPU1_BUSD, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_CLKCMU_DPU1_BUSP, "gout_dpu0_clkcmu_dpu1_busp",
	     "dout_dpu0_busp", CLK_CON_GAT_CLKCMU_DPU1_BUSP, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_CMU_DPU0_PCLK, "gout_dpu0_cmu_dpu0_pclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_CLK_BLK_DPU0_UID_DPU0_CMU_DPU0_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DECON0_PCLKM, "gout_dpu0_ad_apb_decon0_pclkm",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DECON0_PCLKS, "gout_dpu0_ad_apb_decon0_pclks",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DECON0_SECURE_PCLKM,
	     "gout_dpu0_ad_apb_decon0_secure_pclkm", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_SECURE_IPCLKPORT_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DECON0_SECURE_PCLKS,
	     "gout_dpu0_ad_apb_decon0_secure_pclks", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DECON0_SECURE_IPCLKPORT_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPP_PCLKM, "gout_dpu0_ad_apb_dpp_pclkm",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPP_PCLKS, "gout_dpu0_ad_apb_dpp_pclks",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPP_SECURE_PCLKM,
	     "gout_dpu0_ad_apb_dpp_secure_pclkm", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_SECURE_IPCLKPORT_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPP_SECURE_PCLKS,
	     "gout_dpu0_ad_apb_dpp_secure_pclks", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPP_SECURE_IPCLKPORT_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPU_DMA_PCLKM,
	     "gout_dpu0_ad_apb_dpu_dma_pclkm", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPU_DMA_PCLKS,
	     "gout_dpu0_ad_apb_dpu_dma_pclks", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPU_DMA_SECURE_PCLKM,
	     "gout_dpu0_ad_apb_dpu_dma_secure_pclkm", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_SECURE_IPCLKPORT_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPU_DMA_SECURE_PCLKS,
	     "gout_dpu0_ad_apb_dpu_dma_secure_pclks", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_DMA_SECURE_IPCLKPORT_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPU_WB_MUX_PCLKM,
	     "gout_dpu0_ad_apb_dpu_wb_mux_pclkm", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_WB_MUX_IPCLKPORT_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_DPU_WB_MUX_PCLKS,
	     "gout_dpu0_ad_apb_dpu_wb_mux_pclks", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_DPU_WB_MUX_IPCLKPORT_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_MIPI_DSIM0_PCLKS,
	     "gout_dpu0_ad_apb_mipi_dsim0_pclks", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_MIPI_DSIM0_IPCLKPORT_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AD_APB_MIPI_DSIM1_PCLKS,
	     "gout_dpu0_ad_apb_mipi_dsim1_pclks", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AD_APB_MIPI_DSIM1_IPCLKPORT_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AXI2APB_DPU0P0_ACLK, "gout_dpu0_axi2apb_dpu0p0_aclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AXI2APB_DPU0P0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_AXI2APB_DPU0P1_ACLK, "gout_dpu0_axi2apb_dpu0p1_aclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_AXI2APB_DPU0P1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BTM_DPUD0_I_ACLK, "gout_dpu0_btm_dpud0_i_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD0_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BTM_DPUD0_I_PCLK, "gout_dpu0_btm_dpud0_i_pclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD0_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BTM_DPUD1_I_ACLK, "gout_dpu0_btm_dpud1_i_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD1_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BTM_DPUD1_I_PCLK, "gout_dpu0_btm_dpud1_i_pclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD1_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BTM_DPUD2_I_ACLK, "gout_dpu0_btm_dpud2_i_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD2_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BTM_DPUD2_I_PCLK, "gout_dpu0_btm_dpud2_i_pclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BTM_DPUD2_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DECON0_ACLK, "gout_dpu0_decon0_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DECON0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPP_DPP_G0_ACLK, "gout_dpu0_dpp_dpp_g0_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPP_IPCLKPORT_DPP_G0_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPP_DPP_G1_ACLK, "gout_dpu0_dpp_dpp_g1_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPP_IPCLKPORT_DPP_G1_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPP_DPP_VGR_ACLK, "gout_dpu0_dpp_dpp_vgr_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPP_IPCLKPORT_DPP_VGR_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPU_DMA_ACLK_CH0, "gout_dpu0_dpu_dma_aclk_ch0",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPU_DMA_ACLK_CH1, "gout_dpu0_dpu_dma_aclk_ch1",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPU_DMA_ACLK_CH2, "gout_dpu0_dpu_dma_aclk_ch2",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH2, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPU_DMA_ACLK_CH_INTF,
	     "gout_dpu0_dpu_dma_aclk_ch_intf", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_CH_INTF, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPU_DMA_ACLK_SFR, "gout_dpu0_dpu_dma_aclk_sfr",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_SFR, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPU_DMA_ACLK_WB, "gout_dpu0_dpu_dma_aclk_wb",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_DMA_IPCLKPORT_ACLK_WB, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_DPU_WB_MUX_DPP_G1_ACLK,
	     "gout_dpu0_dpu_wb_mux_dpp_g1_aclk", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_DPU_WB_MUX_IPCLKPORT_DPP_G1_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_LHM_AXI_P0_DPU_I_CLK,
	     "gout_dpu0_lhm_axi_p0_dpu_i_clk", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHM_AXI_P0_DPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_LHS_AXI_D0_DPU_I_CLK,
	     "gout_dpu0_lhs_axi_d0_dpu_i_clk", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D0_DPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_LHS_AXI_D1_DPU_I_CLK,
	     "gout_dpu0_lhs_axi_d1_dpu_i_clk", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D1_DPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_LHS_AXI_D2_DPU_I_CLK,
	     "gout_dpu0_lhs_axi_d2_dpu_i_clk", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D2_DPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_LHS_AXI_D_USBTV_I_CLK,
	     "gout_dpu0_lhs_axi_d_usbtv_i_clk", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_LHS_AXI_D_USBTV_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_PMU_DPU0_PCLK, "gout_dpu0_pmu_dpu0_pclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_PMU_DPU0_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BCM_DPUD0_ACLK, "gout_dpu0_bcm_dpud0_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BCM_DPUD0_PCLK, "gout_dpu0_bcm_dpud0_pclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD0_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BCM_DPUD1_ACLK, "gout_dpu0_bcm_dpud1_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BCM_DPUD1_PCLK, "gout_dpu0_bcm_dpud1_pclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD1_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BCM_DPUD2_ACLK, "gout_dpu0_bcm_dpud2_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD2_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_BCM_DPUD2_PCLK, "gout_dpu0_bcm_dpud2_pclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_BCM_DPUD2_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_RSTNSYNC_CLK_DPU0_BUSD_CLK,
	     "gout_dpu0_rstnsync_clk_dpu0_busd_clk", "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_RSTNSYNC_CLK_DPU0_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_RSTNSYNC_CLK_DPU0_BUSP_CLK,
	     "gout_dpu0_rstnsync_clk_dpu0_busp_clk", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_RSTNSYNC_CLK_DPU0_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSMMU_DPUD0_ACLK, "gout_dpu0_sysmmu_dpud0_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSMMU_DPUD0_PCLK_NONSECURE,
	     "gout_dpu0_sysmmu_dpud0_pclk_nonsecure", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD0_IPCLKPORT_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSMMU_DPUD0_PCLK_SECURE,
	     "gout_dpu0_sysmmu_dpud0_pclk_secure", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD0_IPCLKPORT_PCLK_SECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSMMU_DPUD1_ACLK, "gout_dpu0_sysmmu_dpud1_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSMMU_DPUD1_PCLK_NONSECURE,
	     "gout_dpu0_sysmmu_dpud1_pclk_nonsecure", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD1_IPCLKPORT_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSMMU_DPUD1_PCLK_SECURE,
	     "gout_dpu0_sysmmu_dpud1_pclk_secure", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD1_IPCLKPORT_PCLK_SECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSMMU_DPUD2_ACLK, "gout_dpu0_sysmmu_dpud2_aclk",
	     "mout_dpu0_dpu_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD2_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSMMU_DPUD2_PCLK_NONSECURE,
	     "gout_dpu0_sysmmu_dpud2_pclk_nonsecure", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD2_IPCLKPORT_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSMMU_DPUD2_PCLK_SECURE,
	     "gout_dpu0_sysmmu_dpud2_pclk_secure", "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSMMU_DPUD2_IPCLKPORT_PCLK_SECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_SYSREG_DPU0_PCLK, "gout_dpu0_sysreg_dpu0_pclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_SYSREG_DPU0_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU0_XIU_P0_DPU_ACLK, "gout_dpu0_xiu_p0_dpu_aclk",
	     "dout_dpu0_busp",
	     CLK_CON_GAT_GOUT_BLK_DPU0_UID_XIU_P0_DPU_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info dpu0_cmu_info __initconst = {
	.mux_clks		= dpu0_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(dpu0_mux_clks),
	.div_clks		= dpu0_div_clks,
	.nr_div_clks		= ARRAY_SIZE(dpu0_div_clks),
	.gate_clks		= dpu0_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(dpu0_gate_clks),
	.nr_clk_ids		= CLKS_NR_DPU0,
	.clk_regs		= dpu0_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(dpu0_clk_regs),
	.clk_name		= "bus",
};

/* ---- CMU_DPU1 ------------------------------------------------------------ */

/* Register Offset definitions for CMU_DPU1 (0x12a00000) */
#define PLL_LOCKTIME_PLL_DPU							0x0000
#define PLL_CON0_MUX_CLKCMU_DPU1_BUSD_USER					0x0100
#define PLL_CON0_MUX_CLKCMU_DPU1_BUSP_USER					0x0120
#define PLL_CON0_PLL_DPU							0x0140
#define DPU1_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_DIV_DIV_CLKCMU_DPU1_DECON2					0x1800
#define CLK_CON_GAT_CLK_BLK_DPU1_UID_DPU1_CMU_DPU1_IPCLKPORT_PCLK		0x2000
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON1_IPCLKPORT_PCLKM		0x2008
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON1_IPCLKPORT_PCLKS		0x200c
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON2_IPCLKPORT_PCLKM		0x2010
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON2_IPCLKPORT_PCLKS		0x2014
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_AXI2APB_DPU1P0_IPCLKPORT_ACLK		0x2018
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_DECON1_IPCLKPORT_ACLK			0x201c
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_DECON2_IPCLKPORT_ACLK			0x2020
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_DECON2_IPCLKPORT_VCLK			0x2024
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_LHM_AXI_P_DPU1_IPCLKPORT_I_CLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_PMU_DPU1_IPCLKPORT_PCLK			0x2034
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_RSTNSYNC_CLK_DPU1_BUSD_IPCLKPORT_CLK	0x2038
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_RSTNSYNC_CLK_DPU1_BUSP_IPCLKPORT_CLK	0x203c
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_RSTNSYNC_CLK_DPU1_DECON2_IPCLKPORT_CLK	0x2040
#define CLK_CON_GAT_GOUT_BLK_DPU1_UID_SYSREG_DPU1_IPCLKPORT_PCLK		0x2048
#define QCH_CON_DECON1_QCH							0x3004
#define QCH_CON_DECON2_QCH_ACLK							0x3008
#define QCH_CON_DECON2_QCH_VCLK							0x300c
#define QCH_CON_DPU1_CMU_DPU1_QCH						0x3010
#define QCH_CON_LHM_AXI_P_DPU1_QCH						0x3014
#define QCH_CON_LHS_ATB_DPTX_QCH						0x3018
#define QCH_CON_PMU_DPU1_QCH							0x301c
#define QCH_CON_SYSREG_DPU1_QCH							0x3020

static const unsigned long dpu1_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_DPU,
	PLL_CON0_MUX_CLKCMU_DPU1_BUSD_USER,
	PLL_CON0_MUX_CLKCMU_DPU1_BUSP_USER,
	PLL_CON0_PLL_DPU,
	CLK_CON_DIV_DIV_CLKCMU_DPU1_DECON2,
	CLK_CON_GAT_CLK_BLK_DPU1_UID_DPU1_CMU_DPU1_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON1_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON1_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON2_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON2_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_AXI2APB_DPU1P0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_DECON1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_DECON2_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_DECON2_IPCLKPORT_VCLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_LHM_AXI_P_DPU1_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_PMU_DPU1_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_RSTNSYNC_CLK_DPU1_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_RSTNSYNC_CLK_DPU1_BUSP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_RSTNSYNC_CLK_DPU1_DECON2_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DPU1_UID_SYSREG_DPU1_IPCLKPORT_PCLK,
	QCH_CON_SYSREG_DPU1_QCH,
	QCH_CON_PMU_DPU1_QCH,
	QCH_CON_LHS_ATB_DPTX_QCH,
	QCH_CON_LHM_AXI_P_DPU1_QCH,
	QCH_CON_DPU1_CMU_DPU1_QCH,
	QCH_CON_DECON2_QCH_VCLK,
	QCH_CON_DECON2_QCH_ACLK,
	QCH_CON_DECON1_QCH,
	DPU1_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_DPU1 */
/* PLL_CON0 bit 4 selects between OSCCLK and the PLL output */
PNAME(mout_pll_dpu_p)			= { "oscclk", "fout_dpu_pll" };
PNAME(mout_dpu1_busd_user_p)		= { "oscclk", "gout_dpu0_clkcmu_dpu1_busd" };
PNAME(mout_dpu1_busp_user_p)		= { "oscclk", "gout_dpu0_clkcmu_dpu1_busp" };

static const struct samsung_pll_clock dpu1_pll_clks[] __initconst = {
	PLL(pll_1052x, CLK_FOUT_DPU_PLL, "fout_dpu_pll", "oscclk",
	    PLL_LOCKTIME_PLL_DPU, PLL_CON0_PLL_DPU, NULL),
};

static const struct samsung_mux_clock dpu1_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PLL_DPU, "mout_pll_dpu", mout_pll_dpu_p, PLL_CON0_PLL_DPU,
	    4, 1),
	MUX(CLK_MOUT_DPU1_BUSD_USER, "mout_dpu1_busd_user",
	    mout_dpu1_busd_user_p, PLL_CON0_MUX_CLKCMU_DPU1_BUSD_USER, 4, 1),
	MUX(CLK_MOUT_DPU1_BUSP_USER, "mout_dpu1_busp_user",
	    mout_dpu1_busp_user_p, PLL_CON0_MUX_CLKCMU_DPU1_BUSP_USER, 4, 1),
};

static const struct samsung_div_clock dpu1_div_clks[] __initconst = {
	DIV(CLK_DOUT_DPU1_CLKCMU_DPU1_DECON2, "dout_dpu1_clkcmu_dpu1_decon2",
	    "mout_pll_dpu", CLK_CON_DIV_DIV_CLKCMU_DPU1_DECON2, 0, 3),
};

static const struct samsung_gate_clock dpu1_gate_clks[] __initconst = {
	GATE(CLK_GOUT_DPU1_CMU_DPU1_PCLK, "gout_dpu1_cmu_dpu1_pclk",
	     "mout_dpu1_busp_user",
	     CLK_CON_GAT_CLK_BLK_DPU1_UID_DPU1_CMU_DPU1_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_AD_APB_DECON1_PCLKM, "gout_dpu1_ad_apb_decon1_pclkm",
	     "mout_dpu1_busd_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON1_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_AD_APB_DECON1_PCLKS, "gout_dpu1_ad_apb_decon1_pclks",
	     "mout_dpu1_busp_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON1_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_AD_APB_DECON2_PCLKM, "gout_dpu1_ad_apb_decon2_pclkm",
	     "mout_dpu1_busd_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON2_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_AD_APB_DECON2_PCLKS, "gout_dpu1_ad_apb_decon2_pclks",
	     "mout_dpu1_busp_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_AD_APB_DECON2_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_AXI2APB_DPU1P0_ACLK, "gout_dpu1_axi2apb_dpu1p0_aclk",
	     "mout_dpu1_busp_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_AXI2APB_DPU1P0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_DECON1_ACLK, "gout_dpu1_decon1_aclk",
	     "mout_dpu1_busd_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_DECON1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_DECON2_ACLK, "gout_dpu1_decon2_aclk",
	     "mout_dpu1_busd_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_DECON2_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_DECON2_VCLK, "gout_dpu1_decon2_vclk",
	     "dout_dpu1_clkcmu_dpu1_decon2",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_DECON2_IPCLKPORT_VCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_LHM_AXI_P_DPU1_I_CLK,
	     "gout_dpu1_lhm_axi_p_dpu1_i_clk", "mout_dpu1_busp_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_LHM_AXI_P_DPU1_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_PMU_DPU1_PCLK, "gout_dpu1_pmu_dpu1_pclk",
	     "mout_dpu1_busp_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_PMU_DPU1_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_RSTNSYNC_CLK_DPU1_BUSD_CLK,
	     "gout_dpu1_rstnsync_clk_dpu1_busd_clk", "mout_dpu1_busd_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_RSTNSYNC_CLK_DPU1_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_RSTNSYNC_CLK_DPU1_BUSP_CLK,
	     "gout_dpu1_rstnsync_clk_dpu1_busp_clk", "mout_dpu1_busp_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_RSTNSYNC_CLK_DPU1_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_RSTNSYNC_CLK_DPU1_DECON2_CLK,
	     "gout_dpu1_rstnsync_clk_dpu1_decon2_clk",
	     "dout_dpu1_clkcmu_dpu1_decon2",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_RSTNSYNC_CLK_DPU1_DECON2_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DPU1_SYSREG_DPU1_PCLK, "gout_dpu1_sysreg_dpu1_pclk",
	     "mout_dpu1_busp_user",
	     CLK_CON_GAT_GOUT_BLK_DPU1_UID_SYSREG_DPU1_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info dpu1_cmu_info __initconst = {
	.pll_clks		= dpu1_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(dpu1_pll_clks),
	.mux_clks		= dpu1_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(dpu1_mux_clks),
	.div_clks		= dpu1_div_clks,
	.nr_div_clks		= ARRAY_SIZE(dpu1_div_clks),
	.gate_clks		= dpu1_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(dpu1_gate_clks),
	.nr_clk_ids		= CLKS_NR_DPU1,
	.clk_regs		= dpu1_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(dpu1_clk_regs),
	.clk_name		= "busp",
};

/* ---- CMU_MFC ------------------------------------------------------------- */

/* Register Offset definitions for CMU_MFC (0x13c00000) */
#define PLL_CON0_MUX_CLKCMU_MFC_BUS_USER					0x0100
#define MFC_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_DIV_DIV_CLK_MFC_BUSP						0x1800
#define CLK_CON_GAT_CLK_BLK_MFC_UID_MFC_CMU_MFC_IPCLKPORT_PCLK			0x2000
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_AS_P_MFCP_IPCLKPORT_PCLKM			0x2010
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_AS_P_MFCP_IPCLKPORT_PCLKS			0x2014
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_AXI2APB_MFC_IPCLKPORT_ACLK			0x2018
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD0_IPCLKPORT_I_ACLK			0x201c
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD0_IPCLKPORT_I_PCLK			0x2020
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD1_IPCLKPORT_I_ACLK			0x2024
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD1_IPCLKPORT_I_PCLK			0x2028
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_LHM_AXI_P_MFC_IPCLKPORT_I_CLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_LHS_AXI_D0_MFC_IPCLKPORT_I_CLK		0x2030
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_LHS_AXI_D1_MFC_IPCLKPORT_I_CLK		0x2034
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_MFC_IPCLKPORT_CLK				0x2038
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_PMU_MFC_IPCLKPORT_PCLK			0x203c
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD0_IPCLKPORT_ACLK			0x2040
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD0_IPCLKPORT_PCLK			0x2044
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD1_IPCLKPORT_ACLK			0x2048
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD1_IPCLKPORT_PCLK			0x204c
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_RSTNSYNC_CLK_MFC_BUSD_IPCLKPORT_CLK	0x2050
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_RSTNSYNC_CLK_MFC_BUSP_IPCLKPORT_CLK	0x2054
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD0_IPCLKPORT_ACLK			0x2058
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD0_IPCLKPORT_PCLK_NONSECURE	0x205c
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD0_IPCLKPORT_PCLK_SECURE		0x2060
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD1_IPCLKPORT_ACLK			0x2064
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD1_IPCLKPORT_PCLK_NONSECURE	0x2068
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD1_IPCLKPORT_PCLK_SECURE		0x206c
#define CLK_CON_GAT_GOUT_BLK_MFC_UID_SYSREG_MFC_IPCLKPORT_PCLK			0x2070
#define QCH_CON_BTM_MFCD0_QCH							0x3000
#define QCH_CON_BTM_MFCD1_QCH							0x3004
#define QCH_CON_LHM_AXI_P_MFC_QCH						0x3010
#define QCH_CON_LHS_AXI_D0_MFC_QCH						0x3014
#define QCH_CON_LHS_AXI_D1_MFC_QCH						0x3018
#define QCH_CON_MFC_CMU_MFC_QCH							0x301c
#define QCH_CON_MFC_QCH								0x3020
#define QCH_CON_PMU_MFC_QCH							0x3024
#define QCH_CON_BCM_MFCD0_QCH							0x3028
#define QCH_CON_BCM_MFCD1_QCH							0x302c
#define QCH_CON_SMMU_MFCD0_QCH							0x3030
#define QCH_CON_SMMU_MFCD1_QCH							0x3034
#define QCH_CON_SYSREG_MFC_QCH							0x3038

static const unsigned long mfc_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_MFC_BUS_USER,
	CLK_CON_DIV_DIV_CLK_MFC_BUSP,
	CLK_CON_GAT_CLK_BLK_MFC_UID_MFC_CMU_MFC_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_AS_P_MFCP_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_AS_P_MFCP_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_AXI2APB_MFC_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD0_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD0_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD1_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD1_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_LHM_AXI_P_MFC_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_LHS_AXI_D0_MFC_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_LHS_AXI_D1_MFC_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_MFC_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_PMU_MFC_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD1_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_RSTNSYNC_CLK_MFC_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_RSTNSYNC_CLK_MFC_BUSP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD0_IPCLKPORT_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD0_IPCLKPORT_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD1_IPCLKPORT_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD1_IPCLKPORT_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_MFC_UID_SYSREG_MFC_IPCLKPORT_PCLK,
	QCH_CON_SYSREG_MFC_QCH,
	QCH_CON_SMMU_MFCD1_QCH,
	QCH_CON_SMMU_MFCD0_QCH,
	QCH_CON_BCM_MFCD1_QCH,
	QCH_CON_BCM_MFCD0_QCH,
	QCH_CON_PMU_MFC_QCH,
	QCH_CON_MFC_QCH,
	QCH_CON_MFC_CMU_MFC_QCH,
	QCH_CON_LHS_AXI_D1_MFC_QCH,
	QCH_CON_LHS_AXI_D0_MFC_QCH,
	QCH_CON_LHM_AXI_P_MFC_QCH,
	QCH_CON_BTM_MFCD1_QCH,
	QCH_CON_BTM_MFCD0_QCH,
	MFC_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_MFC */
PNAME(mout_mfc_bus_user_p)		= { "oscclk", "dout_cmu_mfc_bus" };

static const struct samsung_mux_clock mfc_mux_clks[] __initconst = {
	MUX(CLK_MOUT_MFC_BUS_USER, "mout_mfc_bus_user", mout_mfc_bus_user_p,
	    PLL_CON0_MUX_CLKCMU_MFC_BUS_USER, 4, 1),
};

static const struct samsung_div_clock mfc_div_clks[] __initconst = {
	DIV(CLK_DOUT_MFC_BUSP, "dout_mfc_busp", "mout_mfc_bus_user",
	    CLK_CON_DIV_DIV_CLK_MFC_BUSP, 0, 3),
};

static const struct samsung_gate_clock mfc_gate_clks[] __initconst = {
	GATE(CLK_GOUT_MFC_CMU_MFC_PCLK, "gout_mfc_cmu_mfc_pclk",
	     "dout_mfc_busp",
	     CLK_CON_GAT_CLK_BLK_MFC_UID_MFC_CMU_MFC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_AS_P_MFCP_PCLKM, "gout_mfc_as_p_mfcp_pclkm",
	     "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_AS_P_MFCP_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_AS_P_MFCP_PCLKS, "gout_mfc_as_p_mfcp_pclks",
	     "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_AS_P_MFCP_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_AXI2APB_MFC_ACLK, "gout_mfc_axi2apb_mfc_aclk",
	     "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_AXI2APB_MFC_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_BTM_MFCD0_I_ACLK, "gout_mfc_btm_mfcd0_i_aclk",
	     "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD0_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_BTM_MFCD0_I_PCLK, "gout_mfc_btm_mfcd0_i_pclk",
	     "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD0_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_BTM_MFCD1_I_ACLK, "gout_mfc_btm_mfcd1_i_aclk",
	     "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD1_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_BTM_MFCD1_I_PCLK, "gout_mfc_btm_mfcd1_i_pclk",
	     "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_BTM_MFCD1_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_LHM_AXI_P_MFC_I_CLK, "gout_mfc_lhm_axi_p_mfc_i_clk",
	     "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_LHM_AXI_P_MFC_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_LHS_AXI_D0_MFC_I_CLK, "gout_mfc_lhs_axi_d0_mfc_i_clk",
	     "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_LHS_AXI_D0_MFC_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_LHS_AXI_D1_MFC_I_CLK, "gout_mfc_lhs_axi_d1_mfc_i_clk",
	     "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_LHS_AXI_D1_MFC_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_CLK, "gout_mfc_clk", "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_MFC_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_PMU_MFC_PCLK, "gout_mfc_pmu_mfc_pclk",
	     "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_PMU_MFC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_BCM_MFCD0_ACLK, "gout_mfc_bcm_mfcd0_aclk",
	     "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_BCM_MFCD0_PCLK, "gout_mfc_bcm_mfcd0_pclk",
	     "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD0_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_BCM_MFCD1_ACLK, "gout_mfc_bcm_mfcd1_aclk",
	     "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_BCM_MFCD1_PCLK, "gout_mfc_bcm_mfcd1_pclk",
	     "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_BCM_MFCD1_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_RSTNSYNC_CLK_MFC_BUSD_CLK,
	     "gout_mfc_rstnsync_clk_mfc_busd_clk", "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_RSTNSYNC_CLK_MFC_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_RSTNSYNC_CLK_MFC_BUSP_CLK,
	     "gout_mfc_rstnsync_clk_mfc_busp_clk", "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_RSTNSYNC_CLK_MFC_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_SMMU_MFCD0_ACLK, "gout_mfc_smmu_mfcd0_aclk",
	     "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_SMMU_MFCD0_PCLK_NONSECURE,
	     "gout_mfc_smmu_mfcd0_pclk_nonsecure", "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD0_IPCLKPORT_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_SMMU_MFCD0_PCLK_SECURE,
	     "gout_mfc_smmu_mfcd0_pclk_secure", "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD0_IPCLKPORT_PCLK_SECURE, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_SMMU_MFCD1_ACLK, "gout_mfc_smmu_mfcd1_aclk",
	     "mout_mfc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_SMMU_MFCD1_PCLK_NONSECURE,
	     "gout_mfc_smmu_mfcd1_pclk_nonsecure", "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD1_IPCLKPORT_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_SMMU_MFCD1_PCLK_SECURE,
	     "gout_mfc_smmu_mfcd1_pclk_secure", "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_SMMU_MFCD1_IPCLKPORT_PCLK_SECURE, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_MFC_SYSREG_MFC_PCLK, "gout_mfc_sysreg_mfc_pclk",
	     "dout_mfc_busp",
	     CLK_CON_GAT_GOUT_BLK_MFC_UID_SYSREG_MFC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info mfc_cmu_info __initconst = {
	.mux_clks		= mfc_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(mfc_mux_clks),
	.div_clks		= mfc_div_clks,
	.nr_div_clks		= ARRAY_SIZE(mfc_div_clks),
	.gate_clks		= mfc_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(mfc_gate_clks),
	.nr_clk_ids		= CLKS_NR_MFC,
	.clk_regs		= mfc_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(mfc_clk_regs),
	.clk_name		= "bus",
};

/* ---- CMU_CAM ------------------------------------------------------------- */

/* Register Offset definitions for CMU_CAM (0x12c00000) */
#define PLL_CON0_MUX_CLKCMU_CAM_BUS_USER					0x0100
#define PLL_CON0_MUX_CLKCMU_CAM_TPU0_USER					0x0120
#define PLL_CON0_MUX_CLKCMU_CAM_TPU1_USER					0x0140
#define PLL_CON0_MUX_CLKCMU_CAM_VRA_USER					0x0160
#define CAM_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_DIV_DIV_CLK_CAM_BUSP						0x1804
#define CLK_CON_GAT_CLK_BLK_CAM_UID_CAM_CMU_CAM_IPCLKPORT_PCLK			0x2000
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD0_IPCLKPORT_I_ACLK			0x2010
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD0_IPCLKPORT_I_PCLK			0x2014
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD1_IPCLKPORT_I_ACLK			0x2018
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD1_IPCLKPORT_I_PCLK			0x201c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_ISP_EWGEN_CAM_IPCLKPORT_I_PCLK		0x2020
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_BNS	0x2024
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS0	0x2028
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS1	0x202c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS2	0x2030
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS3	0x2034
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSISX4_DMA	0x2038
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_MC_SCALER	0x203c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_TPU0	0x2040
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_TPU1	0x2044
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_VRA	0x2048
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_AXI2APB_IS_CAM_0	0x204c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_AXI2APB_IS_CAM_1	0x2050
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_BNS			0x2054
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS0		0x2058
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS0_DIV2		0x205c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS1		0x2060
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS1_DIV2		0x2064
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS2		0x2068
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS2_DIV2		0x206c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS3		0x2070
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS3_DIV2		0x2074
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSISX4_DMA		0x2078
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_MC_SCALER		0x207c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_BCM_CAM0		0x2080
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_BCM_CAM1		0x2084
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_1TO2_TPU0_MCSC	0x2088
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_1TO2_TPU1_MCSC	0x208c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_CAM_ISPHQ	0x2090
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_CAM_ISPLP	0x2094
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_ISP_TPU0	0x2098
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_ISP_TPU1	0x209c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_MCSC_VRA	0x20a0
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_2TO1_ISP_TPU0	0x20a4
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_2TO1_ISP_TPU1	0x20a8
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_2TO1_VRA	0x20ac
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_ISPHQ_CAM	0x20b0
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_ISPLP_CAM	0x20b4
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_TPU0_MCSC	0x20b8
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_TPU1_MCSC	0x20bc
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_CSISX4		0x20c0
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_TPU0		0x20c4
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_TPU1		0x20c8
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_VRA		0x20cc
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_SYSMMU_CAM0		0x20d0
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_SYSMMU_CAM1		0x20d4
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_TPU0			0x20d8
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_TPU1			0x20dc
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_VRA			0x20e0
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCM_TPU0	0x20e4
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCM_TPU1	0x20e8
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCM_VRA	0x20ec
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCS_TPU0	0x20f0
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCS_TPU1	0x20f4
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCS_VRA	0x20f8
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_D_CAM0_4X1	0x20fc
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_P_CAM_1X2	0x2100
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_BNS	0x2104
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS0	0x2108
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS1	0x210c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS2	0x2110
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS3	0x2114
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSISX4_DMA	0x2118
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_MC_SCALER	0x211c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_TPU0	0x2120
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_TPU1	0x2124
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_VRA	0x2128
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_CFW_CAM		0x212c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_BCM_CAM0		0x2130
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_BCM_CAM1		0x2134
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_CSISX4		0x2138
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_TPU0		0x213c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_TPU1		0x2140
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_VRA		0x2144
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_SYSMMU_CAM0		0x2148
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_SYSMMU_CAM1		0x214c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_LHM_ATB_SRDZCAM_IPCLKPORT_I_CLK		0x2150
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_LHM_AXI_P_CAM_IPCLKPORT_I_CLK		0x2154
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_LHS_AXI_D0_CAM_IPCLKPORT_I_CLK		0x2158
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_LHS_AXI_D1_CAM_IPCLKPORT_I_CLK		0x215c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_PMU_CAM_IPCLKPORT_PCLK			0x2160
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_BUSD_DIV2_IPCLKPORT_CLK	0x2164
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_BUSD_IPCLKPORT_CLK	0x2168
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_BUSP_IPCLKPORT_CLK	0x216c
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_TPU0_IPCLKPORT_CLK	0x2170
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_TPU1_IPCLKPORT_CLK	0x2174
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_VRA_IPCLKPORT_CLK		0x2178
#define CLK_CON_GAT_GOUT_BLK_CAM_UID_SYSREG_CAM_IPCLKPORT_PCLK			0x217c
#define QCH_CON_BTM_CAMD0_QCH							0x3004
#define QCH_CON_BTM_CAMD1_QCH							0x3008
#define QCH_CON_CAM_CMU_CAM_QCH							0x300c
#define QCH_CON_ISP_EWGEN_CAM_QCH						0x3010
#define QCH_CON_IS_CAM_QCH_BNS							0x3014
#define QCH_CON_IS_CAM_QCH_CSIS0						0x3018
#define QCH_CON_IS_CAM_QCH_CSIS1						0x301c
#define QCH_CON_IS_CAM_QCH_CSIS2						0x3020
#define QCH_CON_IS_CAM_QCH_CSIS3						0x3024
#define QCH_CON_IS_CAM_QCH_CSISX4_DMA						0x3028
#define QCH_CON_IS_CAM_QCH_MC_SCALER						0x302c
#define QCH_CON_IS_CAM_QCH_BCM_CAM0						0x3030
#define QCH_CON_IS_CAM_QCH_BCM_CAM1						0x3034
#define QCH_CON_IS_CAM_QCH_QE_CSISX4						0x3038
#define QCH_CON_IS_CAM_QCH_QE_TPU0						0x303c
#define QCH_CON_IS_CAM_QCH_QE_TPU1						0x3040
#define QCH_CON_IS_CAM_QCH_QE_VRA						0x3044
#define QCH_CON_IS_CAM_QCH_SYSMMU_CAM0						0x3048
#define QCH_CON_IS_CAM_QCH_SYSMMU_CAM1						0x304c
#define QCH_CON_IS_CAM_QCH_TPU0							0x3050
#define QCH_CON_IS_CAM_QCH_TPU1							0x3054
#define QCH_CON_IS_CAM_QCH_VRA							0x3058
#define QCH_CON_LHM_ATB_SRDZCAM_QCH						0x305c
#define QCH_CON_LHM_AXI_P_CAM_QCH						0x3060
#define QCH_CON_LHS_AXI_D0_CAM_QCH						0x3064
#define QCH_CON_LHS_AXI_D1_CAM_QCH						0x3068
#define QCH_CON_PMU_CAM_QCH							0x306c
#define QCH_CON_SYSREG_CAM_QCH							0x3070

static const unsigned long cam_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_CAM_BUS_USER,
	PLL_CON0_MUX_CLKCMU_CAM_TPU0_USER,
	PLL_CON0_MUX_CLKCMU_CAM_TPU1_USER,
	PLL_CON0_MUX_CLKCMU_CAM_VRA_USER,
	CLK_CON_DIV_DIV_CLK_CAM_BUSP,
	CLK_CON_GAT_CLK_BLK_CAM_UID_CAM_CMU_CAM_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD0_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD0_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD1_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD1_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_ISP_EWGEN_CAM_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_BNS,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS2,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS3,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSISX4_DMA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_MC_SCALER,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_TPU0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_TPU1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_VRA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_AXI2APB_IS_CAM_0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_AXI2APB_IS_CAM_1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_BNS,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS0_DIV2,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS1_DIV2,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS2,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS2_DIV2,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS3,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS3_DIV2,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSISX4_DMA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_MC_SCALER,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_BCM_CAM0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_BCM_CAM1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_1TO2_TPU0_MCSC,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_1TO2_TPU1_MCSC,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_CAM_ISPHQ,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_CAM_ISPLP,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_ISP_TPU0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_ISP_TPU1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_MCSC_VRA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_2TO1_ISP_TPU0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_2TO1_ISP_TPU1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_2TO1_VRA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_ISPHQ_CAM,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_ISPLP_CAM,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_TPU0_MCSC,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_TPU1_MCSC,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_CSISX4,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_TPU0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_TPU1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_VRA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_SYSMMU_CAM0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_SYSMMU_CAM1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_TPU0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_TPU1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_VRA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCM_TPU0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCM_TPU1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCM_VRA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCS_TPU0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCS_TPU1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCS_VRA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_D_CAM0_4X1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_P_CAM_1X2,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_BNS,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS2,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS3,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSISX4_DMA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_MC_SCALER,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_TPU0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_TPU1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_VRA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_CFW_CAM,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_BCM_CAM0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_BCM_CAM1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_CSISX4,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_TPU0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_TPU1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_VRA,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_SYSMMU_CAM0,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_SYSMMU_CAM1,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_LHM_ATB_SRDZCAM_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_LHM_AXI_P_CAM_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_LHS_AXI_D0_CAM_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_LHS_AXI_D1_CAM_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_PMU_CAM_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_BUSD_DIV2_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_BUSP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_TPU0_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_TPU1_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_VRA_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_CAM_UID_SYSREG_CAM_IPCLKPORT_PCLK,
	QCH_CON_SYSREG_CAM_QCH,
	QCH_CON_PMU_CAM_QCH,
	QCH_CON_LHS_AXI_D1_CAM_QCH,
	QCH_CON_LHS_AXI_D0_CAM_QCH,
	QCH_CON_LHM_AXI_P_CAM_QCH,
	QCH_CON_LHM_ATB_SRDZCAM_QCH,
	QCH_CON_IS_CAM_QCH_VRA,
	QCH_CON_IS_CAM_QCH_TPU1,
	QCH_CON_IS_CAM_QCH_TPU0,
	QCH_CON_IS_CAM_QCH_SYSMMU_CAM1,
	QCH_CON_IS_CAM_QCH_SYSMMU_CAM0,
	QCH_CON_IS_CAM_QCH_QE_VRA,
	QCH_CON_IS_CAM_QCH_QE_TPU1,
	QCH_CON_IS_CAM_QCH_QE_TPU0,
	QCH_CON_IS_CAM_QCH_QE_CSISX4,
	QCH_CON_IS_CAM_QCH_BCM_CAM1,
	QCH_CON_IS_CAM_QCH_BCM_CAM0,
	QCH_CON_IS_CAM_QCH_MC_SCALER,
	QCH_CON_IS_CAM_QCH_CSISX4_DMA,
	QCH_CON_IS_CAM_QCH_CSIS3,
	QCH_CON_IS_CAM_QCH_CSIS2,
	QCH_CON_IS_CAM_QCH_CSIS1,
	QCH_CON_IS_CAM_QCH_CSIS0,
	QCH_CON_IS_CAM_QCH_BNS,
	QCH_CON_ISP_EWGEN_CAM_QCH,
	QCH_CON_CAM_CMU_CAM_QCH,
	QCH_CON_BTM_CAMD1_QCH,
	QCH_CON_BTM_CAMD0_QCH,
	CAM_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_CAM */
PNAME(mout_cam_bus_user_p)		= { "oscclk", "dout_cmu_cam_bus" };
PNAME(mout_cam_tpu0_user_p)		= { "oscclk", "dout_cmu_cam_tpu0" };
PNAME(mout_cam_tpu1_user_p)		= { "oscclk", "dout_cmu_cam_tpu1" };
PNAME(mout_cam_vra_user_p)		= { "oscclk", "dout_cmu_cam_vra" };

static const struct samsung_mux_clock cam_mux_clks[] __initconst = {
	MUX(CLK_MOUT_CAM_BUS_USER, "mout_cam_bus_user", mout_cam_bus_user_p,
	    PLL_CON0_MUX_CLKCMU_CAM_BUS_USER, 4, 1),
	MUX(CLK_MOUT_CAM_TPU0_USER, "mout_cam_tpu0_user", mout_cam_tpu0_user_p,
	    PLL_CON0_MUX_CLKCMU_CAM_TPU0_USER, 4, 1),
	MUX(CLK_MOUT_CAM_TPU1_USER, "mout_cam_tpu1_user", mout_cam_tpu1_user_p,
	    PLL_CON0_MUX_CLKCMU_CAM_TPU1_USER, 4, 1),
	MUX(CLK_MOUT_CAM_VRA_USER, "mout_cam_vra_user", mout_cam_vra_user_p,
	    PLL_CON0_MUX_CLKCMU_CAM_VRA_USER, 4, 1),
};

static const struct samsung_div_clock cam_div_clks[] __initconst = {
	DIV(CLK_DOUT_CAM_BUSP, "dout_cam_busp", "mout_cam_bus_user",
	    CLK_CON_DIV_DIV_CLK_CAM_BUSP, 0, 3),
};

static const struct samsung_gate_clock cam_gate_clks[] __initconst = {
	GATE(CLK_GOUT_CAM_CMU_CAM_PCLK, "gout_cam_cmu_cam_pclk",
	     "dout_cam_busp",
	     CLK_CON_GAT_CLK_BLK_CAM_UID_CAM_CMU_CAM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_BTM_CAMD0_I_ACLK, "gout_cam_btm_camd0_i_aclk",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD0_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_BTM_CAMD0_I_PCLK, "gout_cam_btm_camd0_i_pclk",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD0_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_BTM_CAMD1_I_ACLK, "gout_cam_btm_camd1_i_aclk",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD1_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_BTM_CAMD1_I_PCLK, "gout_cam_btm_camd1_i_pclk",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_BTM_CAMD1_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_ISP_EWGEN_CAM_I_PCLK, "gout_cam_isp_ewgen_cam_i_pclk",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_ISP_EWGEN_CAM_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_BNS,
	     "gout_cam_is_cam_aclk_apb_asyncm_bns", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_BNS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_CSIS0,
	     "gout_cam_is_cam_aclk_apb_asyncm_csis0", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS0,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_CSIS1,
	     "gout_cam_is_cam_aclk_apb_asyncm_csis1", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_CSIS2,
	     "gout_cam_is_cam_aclk_apb_asyncm_csis2", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS2,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_CSIS3,
	     "gout_cam_is_cam_aclk_apb_asyncm_csis3", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSIS3,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_CSISX4_DMA,
	     "gout_cam_is_cam_aclk_apb_asyncm_csisx4_dma", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_CSISX4_DMA,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_MC_SCALER,
	     "gout_cam_is_cam_aclk_apb_asyncm_mc_scaler", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_MC_SCALER,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_TPU0,
	     "gout_cam_is_cam_aclk_apb_asyncm_tpu0", "mout_cam_tpu0_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_TPU0,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_TPU1,
	     "gout_cam_is_cam_aclk_apb_asyncm_tpu1", "mout_cam_tpu1_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_TPU1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_APB_ASYNCM_VRA,
	     "gout_cam_is_cam_aclk_apb_asyncm_vra", "mout_cam_vra_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_APB_ASYNCM_VRA,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_AXI2APB_IS_CAM_0,
	     "gout_cam_is_cam_aclk_axi2apb_is_cam_0", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_AXI2APB_IS_CAM_0,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_AXI2APB_IS_CAM_1,
	     "gout_cam_is_cam_aclk_axi2apb_is_cam_1", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_AXI2APB_IS_CAM_1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_BNS, "gout_cam_is_cam_aclk_bns",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_BNS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_CSIS0, "gout_cam_is_cam_aclk_csis0",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_CSIS0_DIV2,
	     "gout_cam_is_cam_aclk_csis0_div2", "dout_cam_busd_div2",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS0_DIV2, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_CSIS1, "gout_cam_is_cam_aclk_csis1",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_CSIS1_DIV2,
	     "gout_cam_is_cam_aclk_csis1_div2", "dout_cam_busd_div2",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS1_DIV2, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_CSIS2, "gout_cam_is_cam_aclk_csis2",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS2, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_CSIS2_DIV2,
	     "gout_cam_is_cam_aclk_csis2_div2", "dout_cam_busd_div2",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS2_DIV2, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_CSIS3, "gout_cam_is_cam_aclk_csis3",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS3, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_CSIS3_DIV2,
	     "gout_cam_is_cam_aclk_csis3_div2", "dout_cam_busd_div2",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSIS3_DIV2, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_CSISX4_DMA,
	     "gout_cam_is_cam_aclk_csisx4_dma", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_CSISX4_DMA, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_MC_SCALER,
	     "gout_cam_is_cam_aclk_mc_scaler", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_MC_SCALER, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_BCM_CAM0, "gout_cam_is_cam_aclk_bcm_cam0",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_BCM_CAM0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_BCM_CAM1, "gout_cam_is_cam_aclk_bcm_cam1",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_BCM_CAM1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBM_1TO2_TPU0_MCSC,
	     "gout_cam_is_cam_aclk_pxl_asbm_1to2_tpu0_mcsc",
	     "mout_cam_tpu0_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_1TO2_TPU0_MCSC,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBM_1TO2_TPU1_MCSC,
	     "gout_cam_is_cam_aclk_pxl_asbm_1to2_tpu1_mcsc",
	     "mout_cam_tpu1_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_1TO2_TPU1_MCSC,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBM_CAM_ISPHQ,
	     "gout_cam_is_cam_aclk_pxl_asbm_cam_isphq", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_CAM_ISPHQ,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBM_CAM_ISPLP,
	     "gout_cam_is_cam_aclk_pxl_asbm_cam_isplp", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_CAM_ISPLP,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBM_ISP_TPU0,
	     "gout_cam_is_cam_aclk_pxl_asbm_isp_tpu0", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_ISP_TPU0,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBM_ISP_TPU1,
	     "gout_cam_is_cam_aclk_pxl_asbm_isp_tpu1", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_ISP_TPU1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBM_MCSC_VRA,
	     "gout_cam_is_cam_aclk_pxl_asbm_mcsc_vra", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBM_MCSC_VRA,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBS_2TO1_ISP_TPU0,
	     "gout_cam_is_cam_aclk_pxl_asbs_2to1_isp_tpu0",
	     "mout_cam_tpu0_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_2TO1_ISP_TPU0,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBS_2TO1_ISP_TPU1,
	     "gout_cam_is_cam_aclk_pxl_asbs_2to1_isp_tpu1",
	     "mout_cam_tpu1_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_2TO1_ISP_TPU1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBS_2TO1_VRA,
	     "gout_cam_is_cam_aclk_pxl_asbs_2to1_vra", "mout_cam_vra_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_2TO1_VRA,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBS_ISPHQ_CAM,
	     "gout_cam_is_cam_aclk_pxl_asbs_isphq_cam", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_ISPHQ_CAM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBS_ISPLP_CAM,
	     "gout_cam_is_cam_aclk_pxl_asbs_isplp_cam", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_ISPLP_CAM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBS_TPU0_MCSC,
	     "gout_cam_is_cam_aclk_pxl_asbs_tpu0_mcsc", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_TPU0_MCSC,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_PXL_ASBS_TPU1_MCSC,
	     "gout_cam_is_cam_aclk_pxl_asbs_tpu1_mcsc", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_PXL_ASBS_TPU1_MCSC,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_QE_CSISX4,
	     "gout_cam_is_cam_aclk_qe_csisx4", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_CSISX4, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_QE_TPU0, "gout_cam_is_cam_aclk_qe_tpu0",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_TPU0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_QE_TPU1, "gout_cam_is_cam_aclk_qe_tpu1",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_TPU1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_QE_VRA, "gout_cam_is_cam_aclk_qe_vra",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_QE_VRA, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_SYSMMU_CAM0,
	     "gout_cam_is_cam_aclk_sysmmu_cam0", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_SYSMMU_CAM0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_SYSMMU_CAM1,
	     "gout_cam_is_cam_aclk_sysmmu_cam1", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_SYSMMU_CAM1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_TPU0, "gout_cam_is_cam_aclk_tpu0",
	     "mout_cam_tpu0_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_TPU0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_TPU1, "gout_cam_is_cam_aclk_tpu1",
	     "mout_cam_tpu1_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_TPU1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_VRA, "gout_cam_is_cam_aclk_vra",
	     "mout_cam_vra_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_VRA, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_XIU_ASYNCM_TPU0,
	     "gout_cam_is_cam_aclk_xiu_asyncm_tpu0", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCM_TPU0,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_XIU_ASYNCM_TPU1,
	     "gout_cam_is_cam_aclk_xiu_asyncm_tpu1", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCM_TPU1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_XIU_ASYNCM_VRA,
	     "gout_cam_is_cam_aclk_xiu_asyncm_vra", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCM_VRA,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_XIU_ASYNCS_TPU0,
	     "gout_cam_is_cam_aclk_xiu_asyncs_tpu0", "mout_cam_tpu0_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCS_TPU0,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_XIU_ASYNCS_TPU1,
	     "gout_cam_is_cam_aclk_xiu_asyncs_tpu1", "mout_cam_tpu1_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCS_TPU1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_XIU_ASYNCS_VRA,
	     "gout_cam_is_cam_aclk_xiu_asyncs_vra", "mout_cam_vra_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_ASYNCS_VRA,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_XIU_D_CAM0_4X1,
	     "gout_cam_is_cam_aclk_xiu_d_cam0_4x1", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_D_CAM0_4X1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_ACLK_XIU_P_CAM_1X2,
	     "gout_cam_is_cam_aclk_xiu_p_cam_1x2", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_ACLK_XIU_P_CAM_1X2,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_BNS,
	     "gout_cam_is_cam_pclk_asyncs_apb_bns", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_BNS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_CSIS0,
	     "gout_cam_is_cam_pclk_asyncs_apb_csis0", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS0,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_CSIS1,
	     "gout_cam_is_cam_pclk_asyncs_apb_csis1", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_CSIS2,
	     "gout_cam_is_cam_pclk_asyncs_apb_csis2", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS2,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_CSIS3,
	     "gout_cam_is_cam_pclk_asyncs_apb_csis3", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSIS3,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_CSISX4_DMA,
	     "gout_cam_is_cam_pclk_asyncs_apb_csisx4_dma", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_CSISX4_DMA,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_MC_SCALER,
	     "gout_cam_is_cam_pclk_asyncs_apb_mc_scaler", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_MC_SCALER,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_TPU0,
	     "gout_cam_is_cam_pclk_asyncs_apb_tpu0", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_TPU0,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_TPU1,
	     "gout_cam_is_cam_pclk_asyncs_apb_tpu1", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_TPU1,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_ASYNCS_APB_VRA,
	     "gout_cam_is_cam_pclk_asyncs_apb_vra", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_ASYNCS_APB_VRA,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_CFW_CAM, "gout_cam_is_cam_pclk_cfw_cam",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_CFW_CAM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_BCM_CAM0, "gout_cam_is_cam_pclk_bcm_cam0",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_BCM_CAM0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_BCM_CAM1, "gout_cam_is_cam_pclk_bcm_cam1",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_BCM_CAM1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_QE_CSISX4,
	     "gout_cam_is_cam_pclk_qe_csisx4", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_CSISX4, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_QE_TPU0, "gout_cam_is_cam_pclk_qe_tpu0",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_TPU0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_QE_TPU1, "gout_cam_is_cam_pclk_qe_tpu1",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_TPU1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_QE_VRA, "gout_cam_is_cam_pclk_qe_vra",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_QE_VRA, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_SYSMMU_CAM0,
	     "gout_cam_is_cam_pclk_sysmmu_cam0", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_SYSMMU_CAM0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_IS_CAM_PCLK_SYSMMU_CAM1,
	     "gout_cam_is_cam_pclk_sysmmu_cam1", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_IS_CAM_IPCLKPORT_PCLK_SYSMMU_CAM1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_LHM_ATB_SRDZCAM_I_CLK,
	     "gout_cam_lhm_atb_srdzcam_i_clk", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_LHM_ATB_SRDZCAM_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_LHM_AXI_P_CAM_I_CLK, "gout_cam_lhm_axi_p_cam_i_clk",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_LHM_AXI_P_CAM_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_LHS_AXI_D0_CAM_I_CLK, "gout_cam_lhs_axi_d0_cam_i_clk",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_LHS_AXI_D0_CAM_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_LHS_AXI_D1_CAM_I_CLK, "gout_cam_lhs_axi_d1_cam_i_clk",
	     "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_LHS_AXI_D1_CAM_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_PMU_CAM_PCLK, "gout_cam_pmu_cam_pclk",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_PMU_CAM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_RSTNSYNC_CLK_CAM_BUSD_DIV2_CLK,
	     "gout_cam_rstnsync_clk_cam_busd_div2_clk", "dout_cam_busd_div2",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_BUSD_DIV2_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_RSTNSYNC_CLK_CAM_BUSD_CLK,
	     "gout_cam_rstnsync_clk_cam_busd_clk", "mout_cam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_RSTNSYNC_CLK_CAM_BUSP_CLK,
	     "gout_cam_rstnsync_clk_cam_busp_clk", "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_RSTNSYNC_CLK_CAM_TPU0_CLK,
	     "gout_cam_rstnsync_clk_cam_tpu0_clk", "mout_cam_tpu0_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_TPU0_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_RSTNSYNC_CLK_CAM_TPU1_CLK,
	     "gout_cam_rstnsync_clk_cam_tpu1_clk", "mout_cam_tpu1_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_TPU1_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_RSTNSYNC_CLK_CAM_VRA_CLK,
	     "gout_cam_rstnsync_clk_cam_vra_clk", "mout_cam_vra_user",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_RSTNSYNC_CLK_CAM_VRA_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CAM_SYSREG_CAM_PCLK, "gout_cam_sysreg_cam_pclk",
	     "dout_cam_busp",
	     CLK_CON_GAT_GOUT_BLK_CAM_UID_SYSREG_CAM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_fixed_factor_clock cam_fixed_factor_clks[] __initconst = {
	FFACTOR(CLK_DOUT_CAM_BUSD_DIV2, "dout_cam_busd_div2",
		"mout_cam_bus_user", 1, 2, 0),
};

static const struct samsung_cmu_info cam_cmu_info __initconst = {
	.mux_clks		= cam_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cam_mux_clks),
	.div_clks		= cam_div_clks,
	.nr_div_clks		= ARRAY_SIZE(cam_div_clks),
	.gate_clks		= cam_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(cam_gate_clks),
	.fixed_factor_clks	= cam_fixed_factor_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(cam_fixed_factor_clks),
	.nr_clk_ids		= CLKS_NR_CAM,
	.clk_regs		= cam_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(cam_clk_regs),
	.clk_name		= "bus",
};

/* ---- CMU_G2D ------------------------------------------------------------- */

/* Register Offset definitions for CMU_G2D (0x13a00000) */
#define PLL_CON0_MUX_CLKCMU_G2D_G2D_USER					0x0100
#define PLL_CON0_MUX_CLKCMU_G2D_JPEG_USER					0x0120
#define G2D_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_DIV_DIV_CLK_G2D_BUSP						0x1804
#define CLK_CON_GAT_CLK_BLK_G2D_UID_G2D_CMU_G2D_IPCLKPORT_PCLK			0x2000
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_G2DP_IPCLKPORT_PCLKM			0x2010
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_G2DP_IPCLKPORT_PCLKS			0x2014
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_JPEGP_IPCLKPORT_PCLKM			0x2018
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_JPEGP_IPCLKPORT_PCLKS			0x201c
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_M2MSCALERP_IPCLKPORT_PCLKM		0x2020
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_M2MSCALERP_IPCLKPORT_PCLKS		0x2024
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_AXI2APB_G2DP0_IPCLKPORT_ACLK		0x2028
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_AXI2APB_G2DP1_IPCLKPORT_ACLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD0_IPCLKPORT_I_ACLK			0x2030
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD0_IPCLKPORT_I_PCLK			0x2034
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD1_IPCLKPORT_I_ACLK			0x2038
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD1_IPCLKPORT_I_PCLK			0x203c
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD2_IPCLKPORT_I_ACLK			0x2040
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD2_IPCLKPORT_I_PCLK			0x2044
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_G2D_IPCLKPORT_CLK				0x2048
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_JPEG_IPCLKPORT_I_SMFC_CLK			0x204c
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_LHM_AXI_P_G2D_IPCLKPORT_I_CLK		0x2050
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_LHS_ACEL_D0_G2D_IPCLKPORT_I_CLK		0x2054
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_LHS_ACEL_D1_G2D_IPCLKPORT_I_CLK		0x2058
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_LHS_ACEL_D2_G2D_IPCLKPORT_I_CLK		0x205c
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_M2MSCALER_IPCLKPORT_ACLK			0x2060
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_PMU_G2D_IPCLKPORT_PCLK			0x2064
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD0_IPCLKPORT_ACLK			0x2068
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD0_IPCLKPORT_PCLK			0x206c
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD1_IPCLKPORT_ACLK			0x2070
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD1_IPCLKPORT_PCLK			0x2074
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD2_IPCLKPORT_ACLK			0x2078
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD2_IPCLKPORT_PCLK			0x207c
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_JPEG_IPCLKPORT_ACLK			0x2080
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_JPEG_IPCLKPORT_PCLK			0x2084
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_M2MSCALER_IPCLKPORT_ACLK		0x2088
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_M2MSCALER_IPCLKPORT_PCLK		0x208c
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_RSTNSYNC_CLK_G2D_BUSP_IPCLKPORT_CLK	0x2090
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_RSTNSYNC_CLK_G2D_G2D_BUSD_IPCLKPORT_CLK	0x2094
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_RSTNSYNC_CLK_G2D_JPEG_BUSD_IPCLKPORT_CLK	0x2098
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD0_IPCLKPORT_ACLK			0x209c
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD0_IPCLKPORT_PCLK_NONSECURE	0x20a0
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD0_IPCLKPORT_PCLK_SECURE		0x20a4
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD1_IPCLKPORT_ACLK			0x20a8
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD1_IPCLKPORT_PCLK_NONSECURE	0x20ac
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD1_IPCLKPORT_PCLK_SECURE		0x20b0
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD2_IPCLKPORT_ACLK			0x20b4
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD2_IPCLKPORT_PCLK_NONSECURE	0x20b8
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD2_IPCLKPORT_PCLK_SECURE		0x20bc
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_SYSREG_G2D_IPCLKPORT_PCLK			0x20c0
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_XIU_D_G2D_IPCLKPORT_ACLK			0x20c4
#define CLK_CON_GAT_GOUT_BLK_G2D_UID_XIU_P_G2D_IPCLKPORT_ACLK			0x20c8
#define QCH_CON_BTM_G2DD0_QCH							0x3000
#define QCH_CON_BTM_G2DD1_QCH							0x3004
#define QCH_CON_BTM_G2DD2_QCH							0x3008
#define QCH_CON_G2D_CMU_G2D_QCH							0x300c
#define QCH_CON_G2D_QCH								0x3010
#define QCH_CON_JPEG_QCH							0x3014
#define QCH_CON_LHM_AXI_P_G2D_QCH						0x3018
#define QCH_CON_LHS_ACEL_D0_G2D_QCH						0x301c
#define QCH_CON_LHS_ACEL_D1_G2D_QCH						0x3020
#define QCH_CON_LHS_ACEL_D2_G2D_QCH						0x3024
#define QCH_CON_M2MSCALER_QCH							0x3028
#define QCH_CON_PMU_G2D_QCH							0x302c
#define QCH_CON_BCM_G2DD0_QCH							0x3030
#define QCH_CON_BCM_G2DD1_QCH							0x3034
#define QCH_CON_BCM_G2DD2_QCH							0x3038
#define QCH_CON_QE_JPEG_QCH							0x303c
#define QCH_CON_QE_M2MSCALER_QCH						0x3040
#define QCH_CON_SMMU_G2DD0_QCH							0x3044
#define QCH_CON_SMMU_G2DD1_QCH							0x3048
#define QCH_CON_SMMU_G2DD2_QCH							0x304c
#define QCH_CON_SYSREG_G2D_QCH							0x3050

static const unsigned long g2d_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_G2D_G2D_USER,
	PLL_CON0_MUX_CLKCMU_G2D_JPEG_USER,
	CLK_CON_DIV_DIV_CLK_G2D_BUSP,
	CLK_CON_GAT_CLK_BLK_G2D_UID_G2D_CMU_G2D_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_G2DP_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_G2DP_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_JPEGP_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_JPEGP_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_M2MSCALERP_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_M2MSCALERP_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_AXI2APB_G2DP0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_AXI2APB_G2DP1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD0_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD0_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD1_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD1_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD2_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD2_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_G2D_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_JPEG_IPCLKPORT_I_SMFC_CLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_LHM_AXI_P_G2D_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_LHS_ACEL_D0_G2D_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_LHS_ACEL_D1_G2D_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_LHS_ACEL_D2_G2D_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_M2MSCALER_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_PMU_G2D_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD0_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD1_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD2_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD2_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_JPEG_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_JPEG_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_M2MSCALER_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_M2MSCALER_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_RSTNSYNC_CLK_G2D_BUSP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_RSTNSYNC_CLK_G2D_G2D_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_RSTNSYNC_CLK_G2D_JPEG_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD0_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD0_IPCLKPORT_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD0_IPCLKPORT_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD1_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD1_IPCLKPORT_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD1_IPCLKPORT_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD2_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD2_IPCLKPORT_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD2_IPCLKPORT_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_SYSREG_G2D_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_XIU_D_G2D_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_G2D_UID_XIU_P_G2D_IPCLKPORT_ACLK,
	QCH_CON_SYSREG_G2D_QCH,
	QCH_CON_SMMU_G2DD2_QCH,
	QCH_CON_SMMU_G2DD1_QCH,
	QCH_CON_SMMU_G2DD0_QCH,
	QCH_CON_QE_M2MSCALER_QCH,
	QCH_CON_QE_JPEG_QCH,
	QCH_CON_BCM_G2DD2_QCH,
	QCH_CON_BCM_G2DD1_QCH,
	QCH_CON_BCM_G2DD0_QCH,
	QCH_CON_PMU_G2D_QCH,
	QCH_CON_M2MSCALER_QCH,
	QCH_CON_LHS_ACEL_D2_G2D_QCH,
	QCH_CON_LHS_ACEL_D1_G2D_QCH,
	QCH_CON_LHS_ACEL_D0_G2D_QCH,
	QCH_CON_LHM_AXI_P_G2D_QCH,
	QCH_CON_JPEG_QCH,
	QCH_CON_G2D_QCH,
	QCH_CON_G2D_CMU_G2D_QCH,
	QCH_CON_BTM_G2DD2_QCH,
	QCH_CON_BTM_G2DD1_QCH,
	QCH_CON_BTM_G2DD0_QCH,
	G2D_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_G2D */
PNAME(mout_g2d_g2d_user_p)		= { "oscclk", "dout_cmu_g2d_g2d" };
PNAME(mout_g2d_jpeg_user_p)		= { "oscclk", "dout_cmu_g2d_jpeg" };

static const struct samsung_mux_clock g2d_mux_clks[] __initconst = {
	MUX(CLK_MOUT_G2D_G2D_USER, "mout_g2d_g2d_user", mout_g2d_g2d_user_p,
	    PLL_CON0_MUX_CLKCMU_G2D_G2D_USER, 4, 1),
	MUX(CLK_MOUT_G2D_JPEG_USER, "mout_g2d_jpeg_user", mout_g2d_jpeg_user_p,
	    PLL_CON0_MUX_CLKCMU_G2D_JPEG_USER, 4, 1),
};

static const struct samsung_div_clock g2d_div_clks[] __initconst = {
	DIV(CLK_DOUT_G2D_BUSP, "dout_g2d_busp", "mout_g2d_jpeg_user",
	    CLK_CON_DIV_DIV_CLK_G2D_BUSP, 0, 3),
};

static const struct samsung_gate_clock g2d_gate_clks[] __initconst = {
	GATE(CLK_GOUT_G2D_CMU_G2D_PCLK, "gout_g2d_cmu_g2d_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_CLK_BLK_G2D_UID_G2D_CMU_G2D_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_AS_P_G2DP_PCLKM, "gout_g2d_as_p_g2dp_pclkm",
	     "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_G2DP_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_AS_P_G2DP_PCLKS, "gout_g2d_as_p_g2dp_pclks",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_G2DP_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_AS_P_JPEGP_PCLKM, "gout_g2d_as_p_jpegp_pclkm",
	     "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_JPEGP_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_AS_P_JPEGP_PCLKS, "gout_g2d_as_p_jpegp_pclks",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_JPEGP_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_AS_P_M2MSCALERP_PCLKM,
	     "gout_g2d_as_p_m2mscalerp_pclkm", "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_M2MSCALERP_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_AS_P_M2MSCALERP_PCLKS,
	     "gout_g2d_as_p_m2mscalerp_pclks", "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_AS_P_M2MSCALERP_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_AXI2APB_G2DP0_ACLK, "gout_g2d_axi2apb_g2dp0_aclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_AXI2APB_G2DP0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_AXI2APB_G2DP1_ACLK, "gout_g2d_axi2apb_g2dp1_aclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_AXI2APB_G2DP1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BTM_G2DD0_I_ACLK, "gout_g2d_btm_g2dd0_i_aclk",
	     "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD0_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BTM_G2DD0_I_PCLK, "gout_g2d_btm_g2dd0_i_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD0_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BTM_G2DD1_I_ACLK, "gout_g2d_btm_g2dd1_i_aclk",
	     "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD1_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BTM_G2DD1_I_PCLK, "gout_g2d_btm_g2dd1_i_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD1_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BTM_G2DD2_I_ACLK, "gout_g2d_btm_g2dd2_i_aclk",
	     "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD2_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BTM_G2DD2_I_PCLK, "gout_g2d_btm_g2dd2_i_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BTM_G2DD2_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_CLK, "gout_g2d_clk", "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_G2D_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_JPEG_I_SMFC_CLK, "gout_g2d_jpeg_i_smfc_clk",
	     "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_JPEG_IPCLKPORT_I_SMFC_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_LHM_AXI_P_G2D_I_CLK, "gout_g2d_lhm_axi_p_g2d_i_clk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_LHM_AXI_P_G2D_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_LHS_ACEL_D0_G2D_I_CLK,
	     "gout_g2d_lhs_acel_d0_g2d_i_clk", "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_LHS_ACEL_D0_G2D_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_LHS_ACEL_D1_G2D_I_CLK,
	     "gout_g2d_lhs_acel_d1_g2d_i_clk", "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_LHS_ACEL_D1_G2D_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_LHS_ACEL_D2_G2D_I_CLK,
	     "gout_g2d_lhs_acel_d2_g2d_i_clk", "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_LHS_ACEL_D2_G2D_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_M2MSCALER_ACLK, "gout_g2d_m2mscaler_aclk",
	     "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_M2MSCALER_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_PMU_G2D_PCLK, "gout_g2d_pmu_g2d_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_PMU_G2D_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BCM_G2DD0_ACLK, "gout_g2d_bcm_g2dd0_aclk",
	     "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BCM_G2DD0_PCLK, "gout_g2d_bcm_g2dd0_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD0_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BCM_G2DD1_ACLK, "gout_g2d_bcm_g2dd1_aclk",
	     "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BCM_G2DD1_PCLK, "gout_g2d_bcm_g2dd1_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD1_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BCM_G2DD2_ACLK, "gout_g2d_bcm_g2dd2_aclk",
	     "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD2_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_BCM_G2DD2_PCLK, "gout_g2d_bcm_g2dd2_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_BCM_G2DD2_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_QE_JPEG_ACLK, "gout_g2d_qe_jpeg_aclk",
	     "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_JPEG_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_QE_JPEG_PCLK, "gout_g2d_qe_jpeg_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_JPEG_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_QE_M2MSCALER_ACLK, "gout_g2d_qe_m2mscaler_aclk",
	     "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_M2MSCALER_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_QE_M2MSCALER_PCLK, "gout_g2d_qe_m2mscaler_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_QE_M2MSCALER_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_RSTNSYNC_CLK_G2D_BUSP_CLK,
	     "gout_g2d_rstnsync_clk_g2d_busp_clk", "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_RSTNSYNC_CLK_G2D_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_RSTNSYNC_CLK_G2D_G2D_BUSD_CLK,
	     "gout_g2d_rstnsync_clk_g2d_g2d_busd_clk", "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_RSTNSYNC_CLK_G2D_G2D_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_RSTNSYNC_CLK_G2D_JPEG_BUSD_CLK,
	     "gout_g2d_rstnsync_clk_g2d_jpeg_busd_clk", "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_RSTNSYNC_CLK_G2D_JPEG_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SMMU_G2DD0_ACLK, "gout_g2d_smmu_g2dd0_aclk",
	     "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SMMU_G2DD0_PCLK_NONSECURE,
	     "gout_g2d_smmu_g2dd0_pclk_nonsecure", "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD0_IPCLKPORT_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SMMU_G2DD0_PCLK_SECURE,
	     "gout_g2d_smmu_g2dd0_pclk_secure", "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD0_IPCLKPORT_PCLK_SECURE, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SMMU_G2DD1_ACLK, "gout_g2d_smmu_g2dd1_aclk",
	     "mout_g2d_g2d_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SMMU_G2DD1_PCLK_NONSECURE,
	     "gout_g2d_smmu_g2dd1_pclk_nonsecure", "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD1_IPCLKPORT_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SMMU_G2DD1_PCLK_SECURE,
	     "gout_g2d_smmu_g2dd1_pclk_secure", "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD1_IPCLKPORT_PCLK_SECURE, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SMMU_G2DD2_ACLK, "gout_g2d_smmu_g2dd2_aclk",
	     "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD2_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SMMU_G2DD2_PCLK_NONSECURE,
	     "gout_g2d_smmu_g2dd2_pclk_nonsecure", "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD2_IPCLKPORT_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SMMU_G2DD2_PCLK_SECURE,
	     "gout_g2d_smmu_g2dd2_pclk_secure", "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SMMU_G2DD2_IPCLKPORT_PCLK_SECURE, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_SYSREG_G2D_PCLK, "gout_g2d_sysreg_g2d_pclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_SYSREG_G2D_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_XIU_D_G2D_ACLK, "gout_g2d_xiu_d_g2d_aclk",
	     "mout_g2d_jpeg_user",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_XIU_D_G2D_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G2D_XIU_P_G2D_ACLK, "gout_g2d_xiu_p_g2d_aclk",
	     "dout_g2d_busp",
	     CLK_CON_GAT_GOUT_BLK_G2D_UID_XIU_P_G2D_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info g2d_cmu_info __initconst = {
	.mux_clks		= g2d_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(g2d_mux_clks),
	.div_clks		= g2d_div_clks,
	.nr_div_clks		= ARRAY_SIZE(g2d_div_clks),
	.gate_clks		= g2d_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(g2d_gate_clks),
	.nr_clk_ids		= CLKS_NR_G2D,
	.clk_regs		= g2d_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(g2d_clk_regs),
	.clk_name		= "jpeg",
};

/* ---- CMU_ISPHQ ----------------------------------------------------------- */

/* Register Offset definitions for CMU_ISPHQ (0x13100000) */
#define PLL_CON0_MUX_CLKCMU_ISPHQ_BUS_USER					0x0100
#define ISPHQ_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_DIV_DIV_CLK_ISPHQ_BUSP						0x1800
#define CLK_CON_GAT_CLK_BLK_ISPHQ_UID_ISPHQ_CMU_ISPHQ_IPCLKPORT_PCLK		0x2004
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_ISP_EWGEN_ISPHQ_IPCLKPORT_I_PCLK		0x2014
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_3AA_PCLKM	0x2018
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_3AA_PCLKS	0x201c
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_ISPHQ_PCLKM	0x2020
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_ISPHQ_PCLKS	0x2024
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_AXI2APB_BRIDGE_ISPHQ_ACLK	0x2028
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_ISPHQ_ACLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_3AA_ACLK		0x2030
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_3AA_PCLK		0x2034
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_ISPHQ_ACLK		0x2038
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_ISPHQ_PCLK		0x203c
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_TAA_ACLK		0x2040
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_XIU_D_ISPHQ_ACLK	0x2044
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_LHM_AXI_P_ISPHQ_IPCLKPORT_I_CLK		0x2048
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_LHS_AXI_LD_ISPHQ_IPCLKPORT_I_CLK		0x204c
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_PMU_ISPHQ_IPCLKPORT_PCLK			0x2050
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_RSTNSYNC_CLK_ISPHQ_BUSD_IPCLKPORT_CLK	0x2054
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_RSTNSYNC_CLK_ISPHQ_BUSP_IPCLKPORT_CLK	0x2058
#define CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_SYSREG_ISPHQ_IPCLKPORT_PCLK		0x205c
#define QCH_CON_ISPHQ_CMU_ISPHQ_QCH						0x3000
#define QCH_CON_ISP_EWGEN_ISPHQ_QCH						0x3004
#define QCH_CON_IS_ISPHQ_QCH_3AA						0x3008
#define QCH_CON_IS_ISPHQ_QCH_ISPHQ						0x300c
#define QCH_CON_IS_ISPHQ_QCH_QE_3AA						0x3010
#define QCH_CON_IS_ISPHQ_QCH_QE_ISPHQ						0x3014
#define QCH_CON_LHM_AXI_P_ISPHQ_QCH						0x3018
#define QCH_CON_LHS_AXI_LD_ISPHQ_QCH						0x301c
#define QCH_CON_PMU_ISPHQ_QCH							0x3020
#define QCH_CON_SYSREG_ISPHQ_QCH						0x3024

static const unsigned long isphq_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_ISPHQ_BUS_USER,
	CLK_CON_DIV_DIV_CLK_ISPHQ_BUSP,
	CLK_CON_GAT_CLK_BLK_ISPHQ_UID_ISPHQ_CMU_ISPHQ_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_ISP_EWGEN_ISPHQ_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_3AA_PCLKM,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_3AA_PCLKS,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_ISPHQ_PCLKM,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_ISPHQ_PCLKS,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_AXI2APB_BRIDGE_ISPHQ_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_ISPHQ_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_3AA_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_3AA_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_ISPHQ_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_ISPHQ_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_TAA_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_XIU_D_ISPHQ_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_LHM_AXI_P_ISPHQ_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_LHS_AXI_LD_ISPHQ_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_PMU_ISPHQ_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_RSTNSYNC_CLK_ISPHQ_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_RSTNSYNC_CLK_ISPHQ_BUSP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_SYSREG_ISPHQ_IPCLKPORT_PCLK,
	QCH_CON_SYSREG_ISPHQ_QCH,
	QCH_CON_PMU_ISPHQ_QCH,
	QCH_CON_LHS_AXI_LD_ISPHQ_QCH,
	QCH_CON_LHM_AXI_P_ISPHQ_QCH,
	QCH_CON_IS_ISPHQ_QCH_QE_ISPHQ,
	QCH_CON_IS_ISPHQ_QCH_QE_3AA,
	QCH_CON_IS_ISPHQ_QCH_ISPHQ,
	QCH_CON_IS_ISPHQ_QCH_3AA,
	QCH_CON_ISP_EWGEN_ISPHQ_QCH,
	QCH_CON_ISPHQ_CMU_ISPHQ_QCH,
	ISPHQ_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_ISPHQ */
PNAME(mout_isphq_bus_user_p)		= { "oscclk", "dout_cmu_isphq_bus" };

static const struct samsung_mux_clock isphq_mux_clks[] __initconst = {
	MUX(CLK_MOUT_ISPHQ_BUS_USER, "mout_isphq_bus_user",
	    mout_isphq_bus_user_p, PLL_CON0_MUX_CLKCMU_ISPHQ_BUS_USER, 4, 1),
};

static const struct samsung_div_clock isphq_div_clks[] __initconst = {
	DIV(CLK_DOUT_ISPHQ_BUSP, "dout_isphq_busp", "mout_isphq_bus_user",
	    CLK_CON_DIV_DIV_CLK_ISPHQ_BUSP, 0, 3),
};

static const struct samsung_gate_clock isphq_gate_clks[] __initconst = {
	GATE(CLK_GOUT_ISPHQ_CMU_ISPHQ_PCLK, "gout_isphq_cmu_isphq_pclk",
	     "dout_isphq_busp",
	     CLK_CON_GAT_CLK_BLK_ISPHQ_UID_ISPHQ_CMU_ISPHQ_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_ISP_EWGEN_ISPHQ_I_PCLK,
	     "gout_isphq_isp_ewgen_isphq_i_pclk", "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_ISP_EWGEN_ISPHQ_IPCLKPORT_I_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_APB_ASYNC_TOP_3AA_PCLKM,
	     "gout_isphq_is_isphq_apb_async_top_3aa_pclkm",
	     "mout_isphq_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_3AA_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_APB_ASYNC_TOP_3AA_PCLKS,
	     "gout_isphq_is_isphq_apb_async_top_3aa_pclks", "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_3AA_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_APB_ASYNC_TOP_ISPHQ_PCLKM,
	     "gout_isphq_is_isphq_apb_async_top_isphq_pclkm",
	     "mout_isphq_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_ISPHQ_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_APB_ASYNC_TOP_ISPHQ_PCLKS,
	     "gout_isphq_is_isphq_apb_async_top_isphq_pclks", "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_APB_ASYNC_TOP_ISPHQ_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_AXI2APB_BRIDGE_ISPHQ_ACLK,
	     "gout_isphq_is_isphq_axi2apb_bridge_isphq_aclk", "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_AXI2APB_BRIDGE_ISPHQ_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_ISPHQ_ACLK,
	     "gout_isphq_is_isphq_isphq_aclk", "mout_isphq_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_ISPHQ_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_QE_3AA_ACLK,
	     "gout_isphq_is_isphq_qe_3aa_aclk", "mout_isphq_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_3AA_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_QE_3AA_PCLK,
	     "gout_isphq_is_isphq_qe_3aa_pclk", "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_3AA_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_QE_ISPHQ_ACLK,
	     "gout_isphq_is_isphq_qe_isphq_aclk", "mout_isphq_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_ISPHQ_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_QE_ISPHQ_PCLK,
	     "gout_isphq_is_isphq_qe_isphq_pclk", "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_QE_ISPHQ_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_TAA_ACLK, "gout_isphq_is_isphq_taa_aclk",
	     "mout_isphq_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_TAA_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_IS_ISPHQ_XIU_D_ISPHQ_ACLK,
	     "gout_isphq_is_isphq_xiu_d_isphq_aclk", "mout_isphq_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_IS_ISPHQ_IPCLKPORT_XIU_D_ISPHQ_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_LHM_AXI_P_ISPHQ_I_CLK,
	     "gout_isphq_lhm_axi_p_isphq_i_clk", "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_LHM_AXI_P_ISPHQ_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_LHS_AXI_LD_ISPHQ_I_CLK,
	     "gout_isphq_lhs_axi_ld_isphq_i_clk", "mout_isphq_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_LHS_AXI_LD_ISPHQ_IPCLKPORT_I_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_PMU_ISPHQ_PCLK, "gout_isphq_pmu_isphq_pclk",
	     "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_PMU_ISPHQ_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_RSTNSYNC_CLK_ISPHQ_BUSD_CLK,
	     "gout_isphq_rstnsync_clk_isphq_busd_clk", "mout_isphq_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_RSTNSYNC_CLK_ISPHQ_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_RSTNSYNC_CLK_ISPHQ_BUSP_CLK,
	     "gout_isphq_rstnsync_clk_isphq_busp_clk", "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_RSTNSYNC_CLK_ISPHQ_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPHQ_SYSREG_ISPHQ_PCLK, "gout_isphq_sysreg_isphq_pclk",
	     "dout_isphq_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPHQ_UID_SYSREG_ISPHQ_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info isphq_cmu_info __initconst = {
	.mux_clks		= isphq_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(isphq_mux_clks),
	.div_clks		= isphq_div_clks,
	.nr_div_clks		= ARRAY_SIZE(isphq_div_clks),
	.gate_clks		= isphq_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(isphq_gate_clks),
	.nr_clk_ids		= CLKS_NR_ISPHQ,
	.clk_regs		= isphq_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(isphq_clk_regs),
	.clk_name		= "bus",
};

/* ---- CMU_ISPLP ----------------------------------------------------------- */

/* Register Offset definitions for CMU_ISPLP (0x13000000) */
#define PLL_CON0_MUX_CLKCMU_ISPLP_BUS_USER					0x0100
#define ISPLP_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_DIV_DIV_CLK_ISPLP_BUSP						0x1800
#define CLK_CON_GAT_CLK_BLK_ISPLP_UID_ISPLP_CMU_ISPLP_IPCLKPORT_PCLK		0x2000
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_BTM_ISPLP_IPCLKPORT_I_ACLK		0x2010
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_BTM_ISPLP_IPCLKPORT_I_PCLK		0x2014
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_ISP_EWGEN_ISPLP_IPCLKPORT_I_PCLK		0x2018
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_3AAW_PCLKM	0x201c
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_3AAW_PCLKS	0x2020
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_ISPLP_PCLKM	0x2024
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_ISPLP_PCLKS	0x2028
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_AXI2APB_BRIDGE_ISPLP_ACLK	0x202c
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_ISPLP_ACLK		0x2030
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_BCM_ISPLP_ACLK	0x2034
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_BCM_ISPLP_PCLK	0x2038
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_3AAW_ACLK		0x203c
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_3AAW_PCLK		0x2040
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_ISPLP_ACLK		0x2044
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_ISPLP_PCLK		0x2048
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_SMMU_ISPLP_ACLK	0x204c
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_SMMU_ISPLP_PCLK_NONSECURE	0x2050
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_SMMU_ISPLP_PCLK_SECURE	0x2054
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_TAAW_ACLK		0x2058
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_XIU_D_ISPLP_ACLK	0x205c
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_LHM_AXI_LD_ISPHQ_IPCLKPORT_I_CLK		0x2060
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_LHM_AXI_P_ISPLP_IPCLKPORT_I_CLK		0x2064
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_LHS_AXI_D_ISPLP_IPCLKPORT_I_CLK		0x2068
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_PMU_ISPLP_IPCLKPORT_PCLK			0x206c
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_RSTNSYNC_CLK_ISPLP_BUSD_IPCLKPORT_CLK	0x2070
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_RSTNSYNC_CLK_ISPLP_BUSP_IPCLKPORT_CLK	0x2074
#define CLK_CON_GAT_GOUT_BLK_ISPLP_UID_SYSREG_ISPLP_IPCLKPORT_PCLK		0x2078
#define QCH_CON_BTM_ISPLP_QCH							0x3000
#define QCH_CON_ISPLP_CMU_ISPLP_QCH						0x3004
#define QCH_CON_ISP_EWGEN_ISPLP_QCH						0x3008
#define QCH_CON_IS_ISPLP_QCH_3AAW						0x300c
#define QCH_CON_IS_ISPLP_QCH_ISPLP						0x3010
#define QCH_CON_IS_ISPLP_QCH_BCM_ISPLP						0x3014
#define QCH_CON_IS_ISPLP_QCH_QE_3AAW						0x3018
#define QCH_CON_IS_ISPLP_QCH_QE_ISPLP						0x301c
#define QCH_CON_IS_ISPLP_QCH_SMMU_ISPLP						0x3020
#define QCH_CON_LHM_AXI_LD_ISPHQ_QCH						0x3024
#define QCH_CON_LHM_AXI_P_ISPLP_QCH						0x3028
#define QCH_CON_LHS_AXI_D_ISPLP_QCH						0x302c
#define QCH_CON_PMU_ISPLP_QCH							0x3030
#define QCH_CON_SYSREG_ISPLP_QCH						0x3034

static const unsigned long isplp_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_ISPLP_BUS_USER,
	CLK_CON_DIV_DIV_CLK_ISPLP_BUSP,
	CLK_CON_GAT_CLK_BLK_ISPLP_UID_ISPLP_CMU_ISPLP_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_BTM_ISPLP_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_BTM_ISPLP_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_ISP_EWGEN_ISPLP_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_3AAW_PCLKM,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_3AAW_PCLKS,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_ISPLP_PCLKM,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_ISPLP_PCLKS,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_AXI2APB_BRIDGE_ISPLP_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_ISPLP_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_BCM_ISPLP_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_BCM_ISPLP_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_3AAW_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_3AAW_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_ISPLP_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_ISPLP_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_SMMU_ISPLP_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_SMMU_ISPLP_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_SMMU_ISPLP_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_TAAW_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_XIU_D_ISPLP_ACLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_LHM_AXI_LD_ISPHQ_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_LHM_AXI_P_ISPLP_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_LHS_AXI_D_ISPLP_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_PMU_ISPLP_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_RSTNSYNC_CLK_ISPLP_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_RSTNSYNC_CLK_ISPLP_BUSP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_ISPLP_UID_SYSREG_ISPLP_IPCLKPORT_PCLK,
	QCH_CON_SYSREG_ISPLP_QCH,
	QCH_CON_PMU_ISPLP_QCH,
	QCH_CON_LHS_AXI_D_ISPLP_QCH,
	QCH_CON_LHM_AXI_P_ISPLP_QCH,
	QCH_CON_LHM_AXI_LD_ISPHQ_QCH,
	QCH_CON_IS_ISPLP_QCH_SMMU_ISPLP,
	QCH_CON_IS_ISPLP_QCH_QE_ISPLP,
	QCH_CON_IS_ISPLP_QCH_QE_3AAW,
	QCH_CON_IS_ISPLP_QCH_BCM_ISPLP,
	QCH_CON_IS_ISPLP_QCH_ISPLP,
	QCH_CON_IS_ISPLP_QCH_3AAW,
	QCH_CON_ISP_EWGEN_ISPLP_QCH,
	QCH_CON_ISPLP_CMU_ISPLP_QCH,
	QCH_CON_BTM_ISPLP_QCH,
	ISPLP_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_ISPLP */
PNAME(mout_isplp_bus_user_p)		= { "oscclk", "dout_cmu_isplp_bus" };

static const struct samsung_mux_clock isplp_mux_clks[] __initconst = {
	MUX(CLK_MOUT_ISPLP_BUS_USER, "mout_isplp_bus_user",
	    mout_isplp_bus_user_p, PLL_CON0_MUX_CLKCMU_ISPLP_BUS_USER, 4, 1),
};

static const struct samsung_div_clock isplp_div_clks[] __initconst = {
	DIV(CLK_DOUT_ISPLP_BUSP, "dout_isplp_busp", "mout_isplp_bus_user",
	    CLK_CON_DIV_DIV_CLK_ISPLP_BUSP, 0, 3),
};

static const struct samsung_gate_clock isplp_gate_clks[] __initconst = {
	GATE(CLK_GOUT_ISPLP_CMU_ISPLP_PCLK, "gout_isplp_cmu_isplp_pclk",
	     "dout_isplp_busp",
	     CLK_CON_GAT_CLK_BLK_ISPLP_UID_ISPLP_CMU_ISPLP_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_BTM_ISPLP_I_ACLK, "gout_isplp_btm_isplp_i_aclk",
	     "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_BTM_ISPLP_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_BTM_ISPLP_I_PCLK, "gout_isplp_btm_isplp_i_pclk",
	     "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_BTM_ISPLP_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_ISP_EWGEN_ISPLP_I_PCLK,
	     "gout_isplp_isp_ewgen_isplp_i_pclk", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_ISP_EWGEN_ISPLP_IPCLKPORT_I_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_APB_ASYNC_TOP_3AAW_PCLKM,
	     "gout_isplp_is_isplp_apb_async_top_3aaw_pclkm",
	     "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_3AAW_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_APB_ASYNC_TOP_3AAW_PCLKS,
	     "gout_isplp_is_isplp_apb_async_top_3aaw_pclks", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_3AAW_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_APB_ASYNC_TOP_ISPLP_PCLKM,
	     "gout_isplp_is_isplp_apb_async_top_isplp_pclkm",
	     "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_ISPLP_PCLKM,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_APB_ASYNC_TOP_ISPLP_PCLKS,
	     "gout_isplp_is_isplp_apb_async_top_isplp_pclks", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_APB_ASYNC_TOP_ISPLP_PCLKS,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_AXI2APB_BRIDGE_ISPLP_ACLK,
	     "gout_isplp_is_isplp_axi2apb_bridge_isplp_aclk", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_AXI2APB_BRIDGE_ISPLP_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_ISPLP_ACLK,
	     "gout_isplp_is_isplp_isplp_aclk", "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_ISPLP_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_BCM_ISPLP_ACLK,
	     "gout_isplp_is_isplp_bcm_isplp_aclk", "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_BCM_ISPLP_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_BCM_ISPLP_PCLK,
	     "gout_isplp_is_isplp_bcm_isplp_pclk", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_BCM_ISPLP_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_QE_3AAW_ACLK,
	     "gout_isplp_is_isplp_qe_3aaw_aclk", "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_3AAW_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_QE_3AAW_PCLK,
	     "gout_isplp_is_isplp_qe_3aaw_pclk", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_3AAW_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_QE_ISPLP_ACLK,
	     "gout_isplp_is_isplp_qe_isplp_aclk", "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_ISPLP_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_QE_ISPLP_PCLK,
	     "gout_isplp_is_isplp_qe_isplp_pclk", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_QE_ISPLP_PCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_SMMU_ISPLP_ACLK,
	     "gout_isplp_is_isplp_smmu_isplp_aclk", "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_SMMU_ISPLP_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_SMMU_ISPLP_PCLK_NONSECURE,
	     "gout_isplp_is_isplp_smmu_isplp_pclk_nonsecure", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_SMMU_ISPLP_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_SMMU_ISPLP_PCLK_SECURE,
	     "gout_isplp_is_isplp_smmu_isplp_pclk_secure", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_SMMU_ISPLP_PCLK_SECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_TAAW_ACLK, "gout_isplp_is_isplp_taaw_aclk",
	     "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_TAAW_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_IS_ISPLP_XIU_D_ISPLP_ACLK,
	     "gout_isplp_is_isplp_xiu_d_isplp_aclk", "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_IS_ISPLP_IPCLKPORT_XIU_D_ISPLP_ACLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_LHM_AXI_LD_ISPHQ_I_CLK,
	     "gout_isplp_lhm_axi_ld_isphq_i_clk", "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_LHM_AXI_LD_ISPHQ_IPCLKPORT_I_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_LHM_AXI_P_ISPLP_I_CLK,
	     "gout_isplp_lhm_axi_p_isplp_i_clk", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_LHM_AXI_P_ISPLP_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_LHS_AXI_D_ISPLP_I_CLK,
	     "gout_isplp_lhs_axi_d_isplp_i_clk", "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_LHS_AXI_D_ISPLP_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_PMU_ISPLP_PCLK, "gout_isplp_pmu_isplp_pclk",
	     "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_PMU_ISPLP_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_RSTNSYNC_CLK_ISPLP_BUSD_CLK,
	     "gout_isplp_rstnsync_clk_isplp_busd_clk", "mout_isplp_bus_user",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_RSTNSYNC_CLK_ISPLP_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_RSTNSYNC_CLK_ISPLP_BUSP_CLK,
	     "gout_isplp_rstnsync_clk_isplp_busp_clk", "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_RSTNSYNC_CLK_ISPLP_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPLP_SYSREG_ISPLP_PCLK, "gout_isplp_sysreg_isplp_pclk",
	     "dout_isplp_busp",
	     CLK_CON_GAT_GOUT_BLK_ISPLP_UID_SYSREG_ISPLP_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info isplp_cmu_info __initconst = {
	.mux_clks		= isplp_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(isplp_mux_clks),
	.div_clks		= isplp_div_clks,
	.nr_div_clks		= ARRAY_SIZE(isplp_div_clks),
	.gate_clks		= isplp_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(isplp_gate_clks),
	.nr_clk_ids		= CLKS_NR_ISPLP,
	.clk_regs		= isplp_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(isplp_clk_regs),
	.clk_name		= "bus",
};

/* ---- CMU_DCAM ------------------------------------------------------------ */

/* Register Offset definitions for CMU_DCAM (0x12e00000) */
#define PLL_CON0_MUX_CLKCMU_DCAM_BUS_USER					0x0100
#define PLL_CON0_MUX_CLKCMU_DCAM_IMGD_USER					0x0120
#define DCAM_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_DIV_DIV_CLK_DCAM_BUSP						0x1800
#define CLK_CON_GAT_CLK_BLK_DCAM_UID_DCAM_CMU_DCAM_IPCLKPORT_PCLK		0x2000
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_AD_APB_DCAM_IPCLKPORT_PCLKM		0x2010
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_AD_APB_DCAM_IPCLKPORT_PCLKS		0x2014
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_AXI2APB_DCAM_IPCLKPORT_ACLK		0x2018
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_AXI2APB_DCAM_SYS_IPCLKPORT_ACLK		0x201c
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_BTM_DCAM_IPCLKPORT_I_ACLK			0x2020
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_BTM_DCAM_IPCLKPORT_I_PCLK			0x2024
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_DCP_IPCLKPORT_CLK				0x2028
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_LHM_AXI_P_SRDZDCAM_IPCLKPORT_I_CLK	0x202c
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_LHS_ATB_DCAMSRDZ_IPCLKPORT_I_CLK		0x2030
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_LHS_AXI_D_DCAMSRDZ_IPCLKPORT_I_CLK	0x2034
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_PMU_DCAM_IPCLKPORT_PCLK			0x2038
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_BCM_DCAM_IPCLKPORT_ACLK			0x203c
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_BCM_DCAM_IPCLKPORT_PCLK			0x2040
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_PXL_ASBM_DCAM_IPCLKPORT_CLK		0x2044
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_PXL_ASBS_DCAM_IPCLKPORT_CLK		0x2048
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_RSTNSYNC_CLK_DCAM_BUSD_IPCLKPORT_CLK	0x204c
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_RSTNSYNC_CLK_DCAM_BUSP_IPCLKPORT_CLK	0x2050
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_RSTNSYNC_CLK_DCAM_IMGD_IPCLKPORT_CLK	0x2054
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_SYSREG_DCAM_IPCLKPORT_PCLK		0x2058
#define CLK_CON_GAT_GOUT_BLK_DCAM_UID_XIU_P_DCAM_IPCLKPORT_ACLK			0x205c
#define QCH_CON_BTM_DCAM_QCH							0x3004
#define QCH_CON_DCAM_CMU_DCAM_QCH						0x3008
#define QCH_CON_DCP_QCH								0x300c
#define QCH_CON_LHM_AXI_P_SRDZDCAM_QCH						0x3010
#define QCH_CON_LHS_ATB_DCAMSRDZ_QCH						0x3014
#define QCH_CON_LHS_AXI_D_DCAMSRDZ_QCH						0x3018
#define QCH_CON_PMU_DCAM_QCH							0x301c
#define QCH_CON_BCM_DCAM_QCH							0x3020
#define QCH_CON_SYSREG_DCAM_QCH							0x3024

static const unsigned long dcam_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_DCAM_BUS_USER,
	PLL_CON0_MUX_CLKCMU_DCAM_IMGD_USER,
	CLK_CON_DIV_DIV_CLK_DCAM_BUSP,
	CLK_CON_GAT_CLK_BLK_DCAM_UID_DCAM_CMU_DCAM_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_AD_APB_DCAM_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_AD_APB_DCAM_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_AXI2APB_DCAM_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_AXI2APB_DCAM_SYS_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_BTM_DCAM_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_BTM_DCAM_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_DCP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_LHM_AXI_P_SRDZDCAM_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_LHS_ATB_DCAMSRDZ_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_LHS_AXI_D_DCAMSRDZ_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_PMU_DCAM_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_BCM_DCAM_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_BCM_DCAM_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_PXL_ASBM_DCAM_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_PXL_ASBS_DCAM_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_RSTNSYNC_CLK_DCAM_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_RSTNSYNC_CLK_DCAM_BUSP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_RSTNSYNC_CLK_DCAM_IMGD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_SYSREG_DCAM_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_DCAM_UID_XIU_P_DCAM_IPCLKPORT_ACLK,
	QCH_CON_SYSREG_DCAM_QCH,
	QCH_CON_BCM_DCAM_QCH,
	QCH_CON_PMU_DCAM_QCH,
	QCH_CON_LHS_AXI_D_DCAMSRDZ_QCH,
	QCH_CON_LHS_ATB_DCAMSRDZ_QCH,
	QCH_CON_LHM_AXI_P_SRDZDCAM_QCH,
	QCH_CON_DCP_QCH,
	QCH_CON_DCAM_CMU_DCAM_QCH,
	QCH_CON_BTM_DCAM_QCH,
	DCAM_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_DCAM */
PNAME(mout_dcam_bus_user_p)		= { "oscclk", "dout_cmu_dcam_bus" };
PNAME(mout_dcam_imgd_user_p)		= { "oscclk", "dout_cmu_dcam_imgd" };

static const struct samsung_mux_clock dcam_mux_clks[] __initconst = {
	MUX(CLK_MOUT_DCAM_BUS_USER, "mout_dcam_bus_user", mout_dcam_bus_user_p,
	    PLL_CON0_MUX_CLKCMU_DCAM_BUS_USER, 4, 1),
	MUX(CLK_MOUT_DCAM_IMGD_USER, "mout_dcam_imgd_user",
	    mout_dcam_imgd_user_p, PLL_CON0_MUX_CLKCMU_DCAM_IMGD_USER, 4, 1),
};

static const struct samsung_div_clock dcam_div_clks[] __initconst = {
	DIV(CLK_DOUT_DCAM_BUSP, "dout_dcam_busp", "mout_dcam_bus_user",
	    CLK_CON_DIV_DIV_CLK_DCAM_BUSP, 0, 3),
};

static const struct samsung_gate_clock dcam_gate_clks[] __initconst = {
	GATE(CLK_GOUT_DCAM_CMU_DCAM_PCLK, "gout_dcam_cmu_dcam_pclk",
	     "dout_dcam_busp",
	     CLK_CON_GAT_CLK_BLK_DCAM_UID_DCAM_CMU_DCAM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_AD_APB_DCAM_PCLKM, "gout_dcam_ad_apb_dcam_pclkm",
	     "mout_dcam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_AD_APB_DCAM_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_AD_APB_DCAM_PCLKS, "gout_dcam_ad_apb_dcam_pclks",
	     "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_AD_APB_DCAM_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_AXI2APB_DCAM_ACLK, "gout_dcam_axi2apb_dcam_aclk",
	     "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_AXI2APB_DCAM_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_AXI2APB_DCAM_SYS_ACLK,
	     "gout_dcam_axi2apb_dcam_sys_aclk", "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_AXI2APB_DCAM_SYS_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_BTM_DCAM_I_ACLK, "gout_dcam_btm_dcam_i_aclk",
	     "mout_dcam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_BTM_DCAM_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_BTM_DCAM_I_PCLK, "gout_dcam_btm_dcam_i_pclk",
	     "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_BTM_DCAM_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_DCP_CLK, "gout_dcam_dcp_clk", "mout_dcam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_DCP_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_LHM_AXI_P_SRDZDCAM_I_CLK,
	     "gout_dcam_lhm_axi_p_srdzdcam_i_clk", "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_LHM_AXI_P_SRDZDCAM_IPCLKPORT_I_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_LHS_ATB_DCAMSRDZ_I_CLK,
	     "gout_dcam_lhs_atb_dcamsrdz_i_clk", "mout_dcam_imgd_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_LHS_ATB_DCAMSRDZ_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_LHS_AXI_D_DCAMSRDZ_I_CLK,
	     "gout_dcam_lhs_axi_d_dcamsrdz_i_clk", "mout_dcam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_LHS_AXI_D_DCAMSRDZ_IPCLKPORT_I_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_PMU_DCAM_PCLK, "gout_dcam_pmu_dcam_pclk",
	     "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_PMU_DCAM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_BCM_DCAM_ACLK, "gout_dcam_bcm_dcam_aclk",
	     "mout_dcam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_BCM_DCAM_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_BCM_DCAM_PCLK, "gout_dcam_bcm_dcam_pclk",
	     "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_BCM_DCAM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_PXL_ASBM_DCAM_CLK, "gout_dcam_pxl_asbm_dcam_clk",
	     "mout_dcam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_PXL_ASBM_DCAM_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_PXL_ASBS_DCAM_CLK, "gout_dcam_pxl_asbs_dcam_clk",
	     "mout_dcam_imgd_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_PXL_ASBS_DCAM_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_RSTNSYNC_CLK_DCAM_BUSD_CLK,
	     "gout_dcam_rstnsync_clk_dcam_busd_clk", "mout_dcam_bus_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_RSTNSYNC_CLK_DCAM_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_RSTNSYNC_CLK_DCAM_BUSP_CLK,
	     "gout_dcam_rstnsync_clk_dcam_busp_clk", "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_RSTNSYNC_CLK_DCAM_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_RSTNSYNC_CLK_DCAM_IMGD_CLK,
	     "gout_dcam_rstnsync_clk_dcam_imgd_clk", "mout_dcam_imgd_user",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_RSTNSYNC_CLK_DCAM_IMGD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_SYSREG_DCAM_PCLK, "gout_dcam_sysreg_dcam_pclk",
	     "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_SYSREG_DCAM_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_DCAM_XIU_P_DCAM_ACLK, "gout_dcam_xiu_p_dcam_aclk",
	     "dout_dcam_busp",
	     CLK_CON_GAT_GOUT_BLK_DCAM_UID_XIU_P_DCAM_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info dcam_cmu_info __initconst = {
	.mux_clks		= dcam_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(dcam_mux_clks),
	.div_clks		= dcam_div_clks,
	.nr_div_clks		= ARRAY_SIZE(dcam_div_clks),
	.gate_clks		= dcam_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(dcam_gate_clks),
	.nr_clk_ids		= CLKS_NR_DCAM,
	.clk_regs		= dcam_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(dcam_clk_regs),
	.clk_name		= "bus",
};

/* ---- CMU_SRDZ ------------------------------------------------------------ */

/* Register Offset definitions for CMU_SRDZ (0x14200000) */
#define PLL_CON0_MUX_CLKCMU_SRDZ_BUS_USER					0x0100
#define PLL_CON0_MUX_CLKCMU_SRDZ_IMGD_USER					0x0120
#define SRDZ_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_DIV_DIV_CLK_SRDZ_BUSP						0x1800
#define CLK_CON_GAT_CLK_BLK_SRDZ_UID_SRDZ_CMU_SRDZ_IPCLKPORT_PCLK		0x2008
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AD_APB_SRDZ_IPCLKPORT_PCLKM		0x2010
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AD_APB_SRDZ_IPCLKPORT_PCLKS		0x2014
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AXI2APB_SRDZ_IPCLKPORT_ACLK		0x2018
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AXI2APB_SRDZ_SYS_IPCLKPORT_ACLK		0x201c
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BTM_SRDZ_IPCLKPORT_I_ACLK			0x2020
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BTM_SRDZ_IPCLKPORT_I_PCLK			0x2024
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHM_ATB_DCAMSRDZ_IPCLKPORT_I_CLK		0x2028
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHM_AXI_D_DCAMSRDZ_IPCLKPORT_I_CLK	0x202c
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHM_AXI_P_SRDZ_IPCLKPORT_I_CLK		0x2030
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHS_ATB_SRDZCAM_IPCLKPORT_I_CLK		0x2034
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHS_AXI_D_SRDZ_IPCLKPORT_I_CLK		0x2038
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHS_AXI_P_SRDZDCAM_IPCLKPORT_I_CLK	0x203c
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_PMU_SRDZ_IPCLKPORT_PCLK			0x2040
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BCM_SRDZ_IPCLKPORT_ACLK			0x2044
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BCM_SRDZ_IPCLKPORT_PCLK			0x2048
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_PXL_ASBM_1TO2_SRDZ_IPCLKPORT_CLK		0x204c
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_PXL_ASBS_SRDZ_IPCLKPORT_CLK		0x2050
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_RSTNSYNC_CLK_SRDZ_BUSD_IPCLKPORT_CLK	0x2054
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_RSTNSYNC_CLK_SRDZ_BUSP_IPCLKPORT_CLK	0x2058
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_RSTNSYNC_CLK_SRDZ_IMGD_IPCLKPORT_CLK	0x205c
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SMMU_SRDZ_IPCLKPORT_ACLK			0x2060
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SMMU_SRDZ_IPCLKPORT_PCLK_NONSECURE	0x2064
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SMMU_SRDZ_IPCLKPORT_PCLK_SECURE		0x2068
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SRDZ_IPCLKPORT_CLK			0x206c
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SYSREG_SRDZ_IPCLKPORT_PCLK		0x2070
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_XIU_D_SRDZ_IPCLKPORT_ACLK			0x2074
#define CLK_CON_GAT_GOUT_BLK_SRDZ_UID_XIU_P_SRDZ_IPCLKPORT_ACLK			0x2078
#define QCH_CON_BTM_SRDZ_QCH							0x3008
#define QCH_CON_LHM_ATB_DCAMSRDZ_QCH						0x300c
#define QCH_CON_LHM_AXI_D_DCAMSRDZ_QCH						0x3010
#define QCH_CON_LHM_AXI_P_SRDZ_QCH						0x3014
#define QCH_CON_LHS_ATB_SRDZCAM_QCH						0x3018
#define QCH_CON_LHS_AXI_D_SRDZ_QCH						0x301c
#define QCH_CON_LHS_AXI_P_SRDZDCAM_QCH						0x3020
#define QCH_CON_PMU_SRDZ_QCH							0x3024
#define QCH_CON_BCM_SRDZ_QCH							0x3028
#define QCH_CON_SMMU_SRDZ_QCH							0x302c
#define QCH_CON_SRDZ_CMU_SRDZ_QCH						0x3030
#define QCH_CON_SRDZ_QCH							0x3034
#define QCH_CON_SYSREG_SRDZ_QCH							0x3038

static const unsigned long srdz_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_SRDZ_BUS_USER,
	PLL_CON0_MUX_CLKCMU_SRDZ_IMGD_USER,
	CLK_CON_DIV_DIV_CLK_SRDZ_BUSP,
	CLK_CON_GAT_CLK_BLK_SRDZ_UID_SRDZ_CMU_SRDZ_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AD_APB_SRDZ_IPCLKPORT_PCLKM,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AD_APB_SRDZ_IPCLKPORT_PCLKS,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AXI2APB_SRDZ_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AXI2APB_SRDZ_SYS_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BTM_SRDZ_IPCLKPORT_I_ACLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BTM_SRDZ_IPCLKPORT_I_PCLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHM_ATB_DCAMSRDZ_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHM_AXI_D_DCAMSRDZ_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHM_AXI_P_SRDZ_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHS_ATB_SRDZCAM_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHS_AXI_D_SRDZ_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHS_AXI_P_SRDZDCAM_IPCLKPORT_I_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_PMU_SRDZ_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BCM_SRDZ_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BCM_SRDZ_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_PXL_ASBM_1TO2_SRDZ_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_PXL_ASBS_SRDZ_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_RSTNSYNC_CLK_SRDZ_BUSD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_RSTNSYNC_CLK_SRDZ_BUSP_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_RSTNSYNC_CLK_SRDZ_IMGD_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SMMU_SRDZ_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SMMU_SRDZ_IPCLKPORT_PCLK_NONSECURE,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SMMU_SRDZ_IPCLKPORT_PCLK_SECURE,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SRDZ_IPCLKPORT_CLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SYSREG_SRDZ_IPCLKPORT_PCLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_XIU_D_SRDZ_IPCLKPORT_ACLK,
	CLK_CON_GAT_GOUT_BLK_SRDZ_UID_XIU_P_SRDZ_IPCLKPORT_ACLK,
	QCH_CON_SYSREG_SRDZ_QCH,
	QCH_CON_SRDZ_QCH,
	QCH_CON_SRDZ_CMU_SRDZ_QCH,
	QCH_CON_SMMU_SRDZ_QCH,
	QCH_CON_BCM_SRDZ_QCH,
	QCH_CON_PMU_SRDZ_QCH,
	QCH_CON_LHS_AXI_P_SRDZDCAM_QCH,
	QCH_CON_LHS_AXI_D_SRDZ_QCH,
	QCH_CON_LHS_ATB_SRDZCAM_QCH,
	QCH_CON_LHM_AXI_P_SRDZ_QCH,
	QCH_CON_LHM_AXI_D_DCAMSRDZ_QCH,
	QCH_CON_LHM_ATB_DCAMSRDZ_QCH,
	QCH_CON_BTM_SRDZ_QCH,
	SRDZ_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_SRDZ */
PNAME(mout_srdz_bus_user_p)		= { "oscclk", "dout_cmu_srdz_bus" };
PNAME(mout_srdz_imgd_user_p)		= { "oscclk", "dout_cmu_srdz_imgd" };

static const struct samsung_mux_clock srdz_mux_clks[] __initconst = {
	MUX(CLK_MOUT_SRDZ_BUS_USER, "mout_srdz_bus_user", mout_srdz_bus_user_p,
	    PLL_CON0_MUX_CLKCMU_SRDZ_BUS_USER, 4, 1),
	MUX(CLK_MOUT_SRDZ_IMGD_USER, "mout_srdz_imgd_user",
	    mout_srdz_imgd_user_p, PLL_CON0_MUX_CLKCMU_SRDZ_IMGD_USER, 4, 1),
};

static const struct samsung_div_clock srdz_div_clks[] __initconst = {
	DIV(CLK_DOUT_SRDZ_BUSP, "dout_srdz_busp", "mout_srdz_bus_user",
	    CLK_CON_DIV_DIV_CLK_SRDZ_BUSP, 0, 3),
};

static const struct samsung_gate_clock srdz_gate_clks[] __initconst = {
	GATE(CLK_GOUT_SRDZ_CMU_SRDZ_PCLK, "gout_srdz_cmu_srdz_pclk",
	     "dout_srdz_busp",
	     CLK_CON_GAT_CLK_BLK_SRDZ_UID_SRDZ_CMU_SRDZ_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_AD_APB_SRDZ_PCLKM, "gout_srdz_ad_apb_srdz_pclkm",
	     "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AD_APB_SRDZ_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_AD_APB_SRDZ_PCLKS, "gout_srdz_ad_apb_srdz_pclks",
	     "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AD_APB_SRDZ_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_AXI2APB_SRDZ_ACLK, "gout_srdz_axi2apb_srdz_aclk",
	     "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AXI2APB_SRDZ_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_AXI2APB_SRDZ_SYS_ACLK,
	     "gout_srdz_axi2apb_srdz_sys_aclk", "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_AXI2APB_SRDZ_SYS_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_BTM_SRDZ_I_ACLK, "gout_srdz_btm_srdz_i_aclk",
	     "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BTM_SRDZ_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_BTM_SRDZ_I_PCLK, "gout_srdz_btm_srdz_i_pclk",
	     "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BTM_SRDZ_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_LHM_ATB_DCAMSRDZ_I_CLK,
	     "gout_srdz_lhm_atb_dcamsrdz_i_clk", "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHM_ATB_DCAMSRDZ_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_LHM_AXI_D_DCAMSRDZ_I_CLK,
	     "gout_srdz_lhm_axi_d_dcamsrdz_i_clk", "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHM_AXI_D_DCAMSRDZ_IPCLKPORT_I_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_LHM_AXI_P_SRDZ_I_CLK,
	     "gout_srdz_lhm_axi_p_srdz_i_clk", "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHM_AXI_P_SRDZ_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_LHS_ATB_SRDZCAM_I_CLK,
	     "gout_srdz_lhs_atb_srdzcam_i_clk", "mout_srdz_imgd_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHS_ATB_SRDZCAM_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_LHS_AXI_D_SRDZ_I_CLK,
	     "gout_srdz_lhs_axi_d_srdz_i_clk", "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHS_AXI_D_SRDZ_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_LHS_AXI_P_SRDZDCAM_I_CLK,
	     "gout_srdz_lhs_axi_p_srdzdcam_i_clk", "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_LHS_AXI_P_SRDZDCAM_IPCLKPORT_I_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_PMU_SRDZ_PCLK, "gout_srdz_pmu_srdz_pclk",
	     "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_PMU_SRDZ_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_BCM_SRDZ_ACLK, "gout_srdz_bcm_srdz_aclk",
	     "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BCM_SRDZ_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_BCM_SRDZ_PCLK, "gout_srdz_bcm_srdz_pclk",
	     "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_BCM_SRDZ_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_PXL_ASBM_1TO2_SRDZ_CLK,
	     "gout_srdz_pxl_asbm_1to2_srdz_clk", "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_PXL_ASBM_1TO2_SRDZ_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_PXL_ASBS_SRDZ_CLK, "gout_srdz_pxl_asbs_srdz_clk",
	     "mout_srdz_imgd_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_PXL_ASBS_SRDZ_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_RSTNSYNC_CLK_SRDZ_BUSD_CLK,
	     "gout_srdz_rstnsync_clk_srdz_busd_clk", "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_RSTNSYNC_CLK_SRDZ_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_RSTNSYNC_CLK_SRDZ_BUSP_CLK,
	     "gout_srdz_rstnsync_clk_srdz_busp_clk", "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_RSTNSYNC_CLK_SRDZ_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_RSTNSYNC_CLK_SRDZ_IMGD_CLK,
	     "gout_srdz_rstnsync_clk_srdz_imgd_clk", "mout_srdz_imgd_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_RSTNSYNC_CLK_SRDZ_IMGD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_SMMU_SRDZ_ACLK, "gout_srdz_smmu_srdz_aclk",
	     "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SMMU_SRDZ_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_SMMU_SRDZ_PCLK_NONSECURE,
	     "gout_srdz_smmu_srdz_pclk_nonsecure", "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SMMU_SRDZ_IPCLKPORT_PCLK_NONSECURE,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_SMMU_SRDZ_PCLK_SECURE,
	     "gout_srdz_smmu_srdz_pclk_secure", "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SMMU_SRDZ_IPCLKPORT_PCLK_SECURE, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_CLK, "gout_srdz_clk", "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SRDZ_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_SYSREG_SRDZ_PCLK, "gout_srdz_sysreg_srdz_pclk",
	     "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_SYSREG_SRDZ_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_XIU_D_SRDZ_ACLK, "gout_srdz_xiu_d_srdz_aclk",
	     "mout_srdz_bus_user",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_XIU_D_SRDZ_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SRDZ_XIU_P_SRDZ_ACLK, "gout_srdz_xiu_p_srdz_aclk",
	     "dout_srdz_busp",
	     CLK_CON_GAT_GOUT_BLK_SRDZ_UID_XIU_P_SRDZ_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info srdz_cmu_info __initconst = {
	.mux_clks		= srdz_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(srdz_mux_clks),
	.div_clks		= srdz_div_clks,
	.nr_div_clks		= ARRAY_SIZE(srdz_div_clks),
	.gate_clks		= srdz_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(srdz_gate_clks),
	.nr_clk_ids		= CLKS_NR_SRDZ,
	.clk_regs		= srdz_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(srdz_clk_regs),
	.clk_name		= "bus",
};

/* ---- CMU_BUSC ------------------------------------------------------------ */

/* Register Offset definitions for CMU_BUSC (0x15a00000) */
#define PLL_CON0_MUX_CLKCMU_BUSC_BUSPHSI2C_USER					0x0120
#define PLL_CON0_MUX_CLKCMU_BUSC_BUS_USER					0x0140
#define CLK_CON_DIV_DIV_CLK_BUSC_BUSP						0x1800
#define CLK_CON_GAT_CLK_BLK_BUSC_UID_BUSC_CMU_BUSC_IPCLKPORT_PCLK		0x2000
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_ADCIF_BUSC_IPCLKPORT_PCLK_S0		0x2010
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_ADCIF_BUSC_IPCLKPORT_PCLK_S1		0x2014
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_HSI2CDF_IPCLKPORT_PCLKM		0x2018
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_HSI2CDF_IPCLKPORT_PCLKS		0x201c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_PDMA0_IPCLKPORT_PCLKM		0x2020
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_PDMA0_IPCLKPORT_PCLKS		0x2024
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_SPDMA_IPCLKPORT_PCLKM		0x2028
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_SPDMA_IPCLKPORT_PCLKS		0x202c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_ASYNCSFR_WR_SCI_IPCLKPORT_I_PCLK		0x2030
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_ASYNCSFR_WR_SMC_IPCLKPORT_I_PCLK		0x2034
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_AXI2APB_BUSCP0_IPCLKPORT_ACLK		0x2038
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_AXI2APB_BUSCP1_IPCLKPORT_ACLK		0x203c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_AXI2APB_BUSC_TDP_IPCLKPORT_ACLK		0x2040
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_BUSIF_CMUTOPC_IPCLKPORT_PCLK		0x2044
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_GNSSMBOX_IPCLKPORT_PCLK			0x2048
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_GPIO_BUSC_IPCLKPORT_PCLK			0x204c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_HSI2CDF_IPCLKPORT_IPCLK			0x2050
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D0_G2D_IPCLKPORT_I_CLK		0x2054
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D1_G2D_IPCLKPORT_I_CLK		0x2058
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D2_G2D_IPCLKPORT_I_CLK		0x205c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D_DSP_IPCLKPORT_I_CLK		0x2060
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D_FSYS0_IPCLKPORT_I_CLK		0x2064
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D_IVA_IPCLKPORT_I_CLK		0x2068
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D_VPU_IPCLKPORT_I_CLK		0x206c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D0_CAM_IPCLKPORT_I_CLK		0x2070
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D0_DPU_IPCLKPORT_I_CLK		0x2074
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D0_MFC_IPCLKPORT_I_CLK		0x2078
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D1_CAM_IPCLKPORT_I_CLK		0x207c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D1_DPU_IPCLKPORT_I_CLK		0x2080
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D1_MFC_IPCLKPORT_I_CLK		0x2084
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D2_DPU_IPCLKPORT_I_CLK		0x2088
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D_ABOX_IPCLKPORT_I_CLK		0x208c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D_ISPLP_IPCLKPORT_I_CLK		0x2090
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D_SRDZ_IPCLKPORT_I_CLK		0x2094
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D_VTS_IPCLKPORT_I_CLK		0x2098
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_G_CSSYS_IPCLKPORT_I_CLK		0x209c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_D_IVASC_IPCLKPORT_I_CLK		0x20a0
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P0_DPU_IPCLKPORT_I_CLK		0x20a4
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P1_DPU_IPCLKPORT_I_CLK		0x20a8
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_ABOX_IPCLKPORT_I_CLK		0x20ac
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_CAM_IPCLKPORT_I_CLK		0x20b0
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_DSP_IPCLKPORT_I_CLK		0x20b4
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_FSYS0_IPCLKPORT_I_CLK		0x20b8
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_G2D_IPCLKPORT_I_CLK		0x20bc
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_ISPHQ_IPCLKPORT_I_CLK		0x20c0
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_ISPLP_IPCLKPORT_I_CLK		0x20c4
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_IVA_IPCLKPORT_I_CLK		0x20c8
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MFC_IPCLKPORT_I_CLK		0x20cc
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MIF0_IPCLKPORT_I_CLK		0x20d0
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MIF1_IPCLKPORT_I_CLK		0x20d4
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MIF2_IPCLKPORT_I_CLK		0x20d8
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MIF3_IPCLKPORT_I_CLK		0x20dc
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_PERIC0_IPCLKPORT_I_CLK		0x20e0
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_PERIC1_IPCLKPORT_I_CLK		0x20e4
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_PERIS_IPCLKPORT_I_CLK		0x20e8
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_SRDZ_IPCLKPORT_I_CLK		0x20ec
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_VPU_IPCLKPORT_I_CLK		0x20f0
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_VTS_IPCLKPORT_I_CLK		0x20f4
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_MBOX_IPCLKPORT_PCLK			0x20f8
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_PDMA0_IPCLKPORT_ACLK			0x20fc
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_PMU_BUSC_IPCLKPORT_PCLK			0x2100
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_RSTNSYNC_CLK_BUSC_BUSD_IPCLKPORT_CLK	0x2104
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_RSTNSYNC_CLK_BUSC_BUSPHSI2C_IPCLKPORT_CLK	0x2108
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_RSTNSYNC_CLK_BUSC_BUSP_IPCLKPORT_CLK	0x210c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_SECMBOX_IPCLKPORT_PCLK			0x2110
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPDMA_IPCLKPORT_ACLK			0x2114
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPEEDY_BATCHER_WRAP_IPCLKPORT_I_PCLK_BATCHER_AP	0x2118
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPEEDY_BATCHER_WRAP_IPCLKPORT_I_PCLK_BATCHER_CP	0x211c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPEEDY_BATCHER_WRAP_IPCLKPORT_PCLK_BATCHER_SPEEDY	0x2120
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPEEDY_IPCLKPORT_PCLK			0x2124
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_SYSREG_BUSC_IPCLKPORT_PCLK		0x2128
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_D_BUSC_IPCLKPORT_ACLK		0x212c
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_D_BUSC_IPCLKPORT_PCLK		0x2130
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_P_BUSC_IPCLKPORT_ACLK_BUSC		0x2134
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_P_BUSC_IPCLKPORT_PCLK		0x2138
#define CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_P_BUSC_IPCLKPORT_PCLK_BUSC		0x213c

/* List of parent clocks for Muxes in CMU_BUSC */
PNAME(mout_busc_busphsi2c_user_p)	= { "oscclk", "dout_cmu_busc_busphsi2c" };
PNAME(mout_busc_bus_user_p)		= { "oscclk", "dout_cmu_busc_bus" };

static const struct samsung_mux_clock busc_mux_clks[] __initconst = {
	MUX_F(CLK_MOUT_BUSC_BUSPHSI2C_USER, "mout_busc_busphsi2c_user",
	      mout_busc_busphsi2c_user_p,
	      PLL_CON0_MUX_CLKCMU_BUSC_BUSPHSI2C_USER, 4, 1,
	      CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_BUSC_BUS_USER, "mout_busc_bus_user",
	      mout_busc_bus_user_p, PLL_CON0_MUX_CLKCMU_BUSC_BUS_USER, 4, 1,
	      CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock busc_div_clks[] __initconst = {
	DIV_F(CLK_DOUT_BUSC_BUSP, "dout_busc_busp", "mout_busc_bus_user",
	      CLK_CON_DIV_DIV_CLK_BUSC_BUSP, 0, 3, CLK_GET_RATE_NOCACHE,
	      CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock busc_gate_clks[] __initconst = {
	GATE(CLK_GOUT_BUSC_CMU_BUSC_PCLK, "gout_busc_cmu_busc_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_CLK_BLK_BUSC_UID_BUSC_CMU_BUSC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_ADCIF_BUSC_PCLK_S0, "gout_busc_adcif_busc_pclk_s0",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_ADCIF_BUSC_IPCLKPORT_PCLK_S0, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_ADCIF_BUSC_PCLK_S1, "gout_busc_adcif_busc_pclk_s1",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_ADCIF_BUSC_IPCLKPORT_PCLK_S1, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_AD_APB_HSI2CDF_PCLKM,
	     "gout_busc_ad_apb_hsi2cdf_pclkm", "mout_busc_busphsi2c_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_HSI2CDF_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_AD_APB_HSI2CDF_PCLKS,
	     "gout_busc_ad_apb_hsi2cdf_pclks", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_HSI2CDF_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_AD_APB_PDMA0_PCLKM, "gout_busc_ad_apb_pdma0_pclkm",
	     "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_PDMA0_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_AD_APB_PDMA0_PCLKS, "gout_busc_ad_apb_pdma0_pclks",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_PDMA0_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_AD_APB_SPDMA_PCLKM, "gout_busc_ad_apb_spdma_pclkm",
	     "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_SPDMA_IPCLKPORT_PCLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_AD_APB_SPDMA_PCLKS, "gout_busc_ad_apb_spdma_pclks",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_AD_APB_SPDMA_IPCLKPORT_PCLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_ASYNCSFR_WR_SCI_I_PCLK,
	     "gout_busc_asyncsfr_wr_sci_i_pclk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_ASYNCSFR_WR_SCI_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_ASYNCSFR_WR_SMC_I_PCLK,
	     "gout_busc_asyncsfr_wr_smc_i_pclk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_ASYNCSFR_WR_SMC_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_AXI2APB_BUSCP0_ACLK, "gout_busc_axi2apb_buscp0_aclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_AXI2APB_BUSCP0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_AXI2APB_BUSCP1_ACLK, "gout_busc_axi2apb_buscp1_aclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_AXI2APB_BUSCP1_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_AXI2APB_BUSC_TDP_ACLK,
	     "gout_busc_axi2apb_busc_tdp_aclk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_AXI2APB_BUSC_TDP_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_BUSIF_CMUTOPC_PCLK, "gout_busc_busif_cmutopc_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_BUSIF_CMUTOPC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_GNSSMBOX_PCLK, "gout_busc_gnssmbox_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_GNSSMBOX_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_GPIO_BUSC_PCLK, "gout_busc_gpio_busc_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_GPIO_BUSC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_HSI2CDF_IPCLK, "gout_busc_hsi2cdf_ipclk",
	     "mout_busc_busphsi2c_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_HSI2CDF_IPCLKPORT_IPCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_ACEL_D0_G2D_I_CLK,
	     "gout_busc_lhm_acel_d0_g2d_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D0_G2D_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_ACEL_D1_G2D_I_CLK,
	     "gout_busc_lhm_acel_d1_g2d_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D1_G2D_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_ACEL_D2_G2D_I_CLK,
	     "gout_busc_lhm_acel_d2_g2d_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D2_G2D_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_ACEL_D_DSP_I_CLK,
	     "gout_busc_lhm_acel_d_dsp_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D_DSP_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_ACEL_D_FSYS0_I_CLK,
	     "gout_busc_lhm_acel_d_fsys0_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D_FSYS0_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_ACEL_D_IVA_I_CLK,
	     "gout_busc_lhm_acel_d_iva_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D_IVA_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_ACEL_D_VPU_I_CLK,
	     "gout_busc_lhm_acel_d_vpu_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_ACEL_D_VPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D0_CAM_I_CLK,
	     "gout_busc_lhm_axi_d0_cam_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D0_CAM_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D0_DPU_I_CLK,
	     "gout_busc_lhm_axi_d0_dpu_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D0_DPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D0_MFC_I_CLK,
	     "gout_busc_lhm_axi_d0_mfc_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D0_MFC_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D1_CAM_I_CLK,
	     "gout_busc_lhm_axi_d1_cam_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D1_CAM_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D1_DPU_I_CLK,
	     "gout_busc_lhm_axi_d1_dpu_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D1_DPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D1_MFC_I_CLK,
	     "gout_busc_lhm_axi_d1_mfc_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D1_MFC_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D2_DPU_I_CLK,
	     "gout_busc_lhm_axi_d2_dpu_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D2_DPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D_ABOX_I_CLK,
	     "gout_busc_lhm_axi_d_abox_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D_ABOX_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D_ISPLP_I_CLK,
	     "gout_busc_lhm_axi_d_isplp_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D_ISPLP_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D_SRDZ_I_CLK,
	     "gout_busc_lhm_axi_d_srdz_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D_SRDZ_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_D_VTS_I_CLK, "gout_busc_lhm_axi_d_vts_i_clk",
	     "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_D_VTS_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHM_AXI_G_CSSYS_I_CLK,
	     "gout_busc_lhm_axi_g_cssys_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHM_AXI_G_CSSYS_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_D_IVASC_I_CLK,
	     "gout_busc_lhs_axi_d_ivasc_i_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_D_IVASC_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P0_DPU_I_CLK,
	     "gout_busc_lhs_axi_p0_dpu_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P0_DPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P1_DPU_I_CLK,
	     "gout_busc_lhs_axi_p1_dpu_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P1_DPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_ABOX_I_CLK,
	     "gout_busc_lhs_axi_p_abox_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_ABOX_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_CAM_I_CLK, "gout_busc_lhs_axi_p_cam_i_clk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_CAM_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_DSP_I_CLK, "gout_busc_lhs_axi_p_dsp_i_clk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_DSP_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_FSYS0_I_CLK,
	     "gout_busc_lhs_axi_p_fsys0_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_FSYS0_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_G2D_I_CLK, "gout_busc_lhs_axi_p_g2d_i_clk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_G2D_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_ISPHQ_I_CLK,
	     "gout_busc_lhs_axi_p_isphq_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_ISPHQ_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_ISPLP_I_CLK,
	     "gout_busc_lhs_axi_p_isplp_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_ISPLP_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_IVA_I_CLK, "gout_busc_lhs_axi_p_iva_i_clk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_IVA_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_MFC_I_CLK, "gout_busc_lhs_axi_p_mfc_i_clk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MFC_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_MIF0_I_CLK,
	     "gout_busc_lhs_axi_p_mif0_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MIF0_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_MIF1_I_CLK,
	     "gout_busc_lhs_axi_p_mif1_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MIF1_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_MIF2_I_CLK,
	     "gout_busc_lhs_axi_p_mif2_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MIF2_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_MIF3_I_CLK,
	     "gout_busc_lhs_axi_p_mif3_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_MIF3_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_PERIC0_I_CLK,
	     "gout_busc_lhs_axi_p_peric0_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_PERIC0_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_PERIC1_I_CLK,
	     "gout_busc_lhs_axi_p_peric1_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_PERIC1_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_PERIS_I_CLK,
	     "gout_busc_lhs_axi_p_peris_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_PERIS_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_SRDZ_I_CLK,
	     "gout_busc_lhs_axi_p_srdz_i_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_SRDZ_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_VPU_I_CLK, "gout_busc_lhs_axi_p_vpu_i_clk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_VPU_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_LHS_AXI_P_VTS_I_CLK, "gout_busc_lhs_axi_p_vts_i_clk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_LHS_AXI_P_VTS_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_MBOX_PCLK, "gout_busc_mbox_pclk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_MBOX_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_PDMA0_ACLK, "gout_busc_pdma0_aclk",
	     "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_PDMA0_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_PMU_BUSC_PCLK, "gout_busc_pmu_busc_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_PMU_BUSC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_RSTNSYNC_CLK_BUSC_BUSD_CLK,
	     "gout_busc_rstnsync_clk_busc_busd_clk", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_RSTNSYNC_CLK_BUSC_BUSD_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_RSTNSYNC_CLK_BUSC_BUSPHSI2C_CLK,
	     "gout_busc_rstnsync_clk_busc_busphsi2c_clk",
	     "mout_busc_busphsi2c_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_RSTNSYNC_CLK_BUSC_BUSPHSI2C_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_RSTNSYNC_CLK_BUSC_BUSP_CLK,
	     "gout_busc_rstnsync_clk_busc_busp_clk", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_RSTNSYNC_CLK_BUSC_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_SECMBOX_PCLK, "gout_busc_secmbox_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_SECMBOX_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_SPDMA_ACLK, "gout_busc_spdma_aclk",
	     "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPDMA_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_SPEEDY_BATCHER_WRAP_I_PCLK_BATCHER_AP,
	     "gout_busc_speedy_batcher_wrap_i_pclk_batcher_ap",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPEEDY_BATCHER_WRAP_IPCLKPORT_I_PCLK_BATCHER_AP,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_SPEEDY_BATCHER_WRAP_I_PCLK_BATCHER_CP,
	     "gout_busc_speedy_batcher_wrap_i_pclk_batcher_cp",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPEEDY_BATCHER_WRAP_IPCLKPORT_I_PCLK_BATCHER_CP,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_SPEEDY_BATCHER_WRAP_PCLK_BATCHER_SPEEDY,
	     "gout_busc_speedy_batcher_wrap_pclk_batcher_speedy",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPEEDY_BATCHER_WRAP_IPCLKPORT_PCLK_BATCHER_SPEEDY,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_SPEEDY_PCLK, "gout_busc_speedy_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_SPEEDY_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_SYSREG_BUSC_PCLK, "gout_busc_sysreg_busc_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_SYSREG_BUSC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_TREX_D_BUSC_ACLK, "gout_busc_trex_d_busc_aclk",
	     "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_D_BUSC_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_TREX_D_BUSC_PCLK, "gout_busc_trex_d_busc_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_D_BUSC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_TREX_P_BUSC_ACLK_BUSC,
	     "gout_busc_trex_p_busc_aclk_busc", "mout_busc_bus_user",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_P_BUSC_IPCLKPORT_ACLK_BUSC, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_TREX_P_BUSC_PCLK, "gout_busc_trex_p_busc_pclk",
	     "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_P_BUSC_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_BUSC_TREX_P_BUSC_PCLK_BUSC,
	     "gout_busc_trex_p_busc_pclk_busc", "dout_busc_busp",
	     CLK_CON_GAT_GOUT_BLK_BUSC_UID_TREX_P_BUSC_IPCLKPORT_PCLK_BUSC, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info busc_cmu_info __initconst = {
	.mux_clks		= busc_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(busc_mux_clks),
	.div_clks		= busc_div_clks,
	.nr_div_clks		= ARRAY_SIZE(busc_div_clks),
	.gate_clks		= busc_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(busc_gate_clks),
	.nr_clk_ids		= CLKS_NR_BUSC,
	.clk_name		= "bus",
	.hwacg_gates		= true,
};

/* ---- CMU_G3D ------------------------------------------------------------- */

/* Register Offset definitions for CMU_G3D (0x13800000) */
#define PLL_LOCKTIME_PLL_G3D							0x0000
#define PLL_CON0_MUX_CLKCMU_G3D_SWITCH_USER					0x0100
#define PLL_CON0_PLL_G3D							0x0120
#define CLK_CON_MUX_MUX_CLK_G3D_BUSD						0x1000
#define CLK_CON_DIV_DIV_CLK_G3D_BUSP						0x1804
#define CLK_CON_GAT_CLK_BLK_G3D_UID_G3D_CMU_G3D_IPCLKPORT_PCLK			0x2004
#define CLK_CON_GAT_CLK_BLK_G3D_UID_HPM_G3D_IPCLKPORT_HPM_TARGETCLK_C		0x2008
#define CLK_CON_GAT_GATE_CLK_G3D_AGPU						0x201c
#define CLK_CON_GAT_GOUT_BLK_G3D_UID_AXI2APB_G3D_IPCLKPORT_ACLK			0x2020
#define CLK_CON_GAT_GOUT_BLK_G3D_UID_BUSIF_HPMG3D_IPCLKPORT_PCLK		0x2024
#define CLK_CON_GAT_GOUT_BLK_G3D_UID_LHM_AXI_P_G3D_IPCLKPORT_I_CLK		0x2028
#define CLK_CON_GAT_GOUT_BLK_G3D_UID_LHS_AXI_G3DSFR_IPCLKPORT_I_CLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_G3D_UID_PMU_G3D_IPCLKPORT_PCLK			0x2030
#define CLK_CON_GAT_GOUT_BLK_G3D_UID_RSTNSYNC_CLK_G3D_BUSP_IPCLKPORT_CLK	0x2034
#define CLK_CON_GAT_GOUT_BLK_G3D_UID_SYSREG_G3D_IPCLKPORT_PCLK			0x2038
#define CLK_CON_GAT_GOUT_BLK_G3D_UID_XIU_P_G3D_IPCLKPORT_ACLK			0x203c

/* List of parent clocks for Muxes in CMU_G3D */
PNAME(mout_g3d_switch_user_p)		= { "oscclk", "dout_cmu_g3d_switch" };
PNAME(mout_g3d_busd_p)			= { "fout_g3d_pll", "mout_g3d_switch_user" };

static const struct samsung_pll_clock g3d_pll_clks[] __initconst = {
	PLL(pll_1052x, CLK_FOUT_G3D_PLL, "fout_g3d_pll", "oscclk",
	    PLL_LOCKTIME_PLL_G3D, PLL_CON0_PLL_G3D, NULL),
};

static const struct samsung_mux_clock g3d_mux_clks[] __initconst = {
	MUX_F(CLK_MOUT_G3D_SWITCH_USER, "mout_g3d_switch_user",
	      mout_g3d_switch_user_p, PLL_CON0_MUX_CLKCMU_G3D_SWITCH_USER, 4, 1,
	      CLK_GET_RATE_NOCACHE, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_BUSD, "mout_g3d_busd", mout_g3d_busd_p,
	      CLK_CON_MUX_MUX_CLK_G3D_BUSD, 0, 1, CLK_GET_RATE_NOCACHE,
	      CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock g3d_div_clks[] __initconst = {
	DIV_F(CLK_DOUT_G3D_BUSP, "dout_g3d_busp", "mout_g3d_busd",
	      CLK_CON_DIV_DIV_CLK_G3D_BUSP, 0, 3, CLK_GET_RATE_NOCACHE,
	      CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock g3d_gate_clks[] __initconst = {
	GATE(CLK_GOUT_G3D_CMU_G3D_PCLK, "gout_g3d_cmu_g3d_pclk",
	     "dout_g3d_busp",
	     CLK_CON_GAT_CLK_BLK_G3D_UID_G3D_CMU_G3D_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_HPM_G3D_HPM_TARGETCLK_C,
	     "gout_g3d_hpm_g3d_hpm_targetclk_c", "dout_cmu_hpm",
	     CLK_CON_GAT_CLK_BLK_G3D_UID_HPM_G3D_IPCLKPORT_HPM_TARGETCLK_C, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_AGPU, "gout_g3d_agpu", "mout_g3d_busd",
	     CLK_CON_GAT_GATE_CLK_G3D_AGPU, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_AXI2APB_G3D_ACLK, "gout_g3d_axi2apb_g3d_aclk",
	     "dout_g3d_busp",
	     CLK_CON_GAT_GOUT_BLK_G3D_UID_AXI2APB_G3D_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_BUSIF_HPMG3D_PCLK, "gout_g3d_busif_hpmg3d_pclk",
	     "dout_g3d_busp",
	     CLK_CON_GAT_GOUT_BLK_G3D_UID_BUSIF_HPMG3D_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_LHM_AXI_P_G3D_I_CLK, "gout_g3d_lhm_axi_p_g3d_i_clk",
	     "dout_g3d_busp",
	     CLK_CON_GAT_GOUT_BLK_G3D_UID_LHM_AXI_P_G3D_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_LHS_AXI_G3DSFR_I_CLK, "gout_g3d_lhs_axi_g3dsfr_i_clk",
	     "dout_g3d_busp",
	     CLK_CON_GAT_GOUT_BLK_G3D_UID_LHS_AXI_G3DSFR_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_PMU_G3D_PCLK, "gout_g3d_pmu_g3d_pclk",
	     "dout_g3d_busp",
	     CLK_CON_GAT_GOUT_BLK_G3D_UID_PMU_G3D_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_RSTNSYNC_CLK_G3D_BUSP_CLK,
	     "gout_g3d_rstnsync_clk_g3d_busp_clk", "dout_g3d_busp",
	     CLK_CON_GAT_GOUT_BLK_G3D_UID_RSTNSYNC_CLK_G3D_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_SYSREG_G3D_PCLK, "gout_g3d_sysreg_g3d_pclk",
	     "dout_g3d_busp",
	     CLK_CON_GAT_GOUT_BLK_G3D_UID_SYSREG_G3D_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_XIU_P_G3D_ACLK, "gout_g3d_xiu_p_g3d_aclk",
	     "dout_g3d_busp",
	     CLK_CON_GAT_GOUT_BLK_G3D_UID_XIU_P_G3D_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info g3d_cmu_info __initconst = {
	.pll_clks		= g3d_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(g3d_pll_clks),
	.mux_clks		= g3d_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(g3d_mux_clks),
	.div_clks		= g3d_div_clks,
	.nr_div_clks		= ARRAY_SIZE(g3d_div_clks),
	.gate_clks		= g3d_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(g3d_gate_clks),
	.nr_clk_ids		= CLKS_NR_G3D,
};

/* ---- CMU_ABOX ------------------------------------------------------------ */

/* Register Offset definitions for CMU_ABOX (0x13e00000) */
#define PLL_LOCKTIME_PLL_AUD							0x0000
#define PLL_CON0_MUX_CLKCMU_ABOX_CPUABOX_USER					0x0100
#define PLL_CON0_PLL_AUD							0x0120
#define PLL_CON1_PLL_AUD							0x0124
#define PLL_CON2_PLL_AUD							0x0128
#define PLL_CON3_PLL_AUD							0x012c
#define PLL_CON4_PLL_AUD							0x0130
#define ABOX_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_MUX_MUX_CLK_ABOX_CPU						0x1000
#define CLK_CON_MUX_MUX_CLK_ABOX_UAIF0						0x1004
#define CLK_CON_MUX_MUX_CLK_ABOX_UAIF1						0x1008
#define CLK_CON_MUX_MUX_CLK_ABOX_UAIF2						0x100c
#define CLK_CON_MUX_MUX_CLK_ABOX_UAIF3						0x1010
#define CLK_CON_MUX_MUX_CLK_ABOX_UAIF4						0x1014
#define CLK_CON_DIV_DIV_CLK_ABOX_AUDIF						0x1800
#define CLK_CON_DIV_DIV_CLK_ABOX_BUS						0x1804
#define CLK_CON_DIV_DIV_CLK_ABOX_BUSP						0x1808
#define CLK_CON_DIV_DIV_CLK_ABOX_CPU_ACLK					0x180c
#define CLK_CON_DIV_DIV_CLK_ABOX_CPU_ATCLK					0x1810
#define CLK_CON_DIV_DIV_CLK_ABOX_CPU_PCLKDBG					0x1814
#define CLK_CON_DIV_DIV_CLK_ABOX_DMIC						0x1818
#define CLK_CON_DIV_DIV_CLK_ABOX_DSIF						0x181c
#define CLK_CON_DIV_DIV_CLK_ABOX_PLL						0x1820
#define CLK_CON_DIV_DIV_CLK_ABOX_UAIF0						0x1828
#define CLK_CON_DIV_DIV_CLK_ABOX_UAIF1						0x182c
#define CLK_CON_DIV_DIV_CLK_ABOX_UAIF2						0x1830
#define CLK_CON_DIV_DIV_CLK_ABOX_UAIF3						0x1834
#define CLK_CON_DIV_DIV_CLK_ABOX_UAIF4						0x1838
#define CLK_CON_GAT_CLK_BLK_ABOX_UID_ABOX_CMU_ABOX_IPCLKPORT_PCLK		0x2000
#define CLK_CON_GAT_CLK_BLK_ABOX_UID_DFTMUX_ABOX_IPCLKPORT_AUD_CODEC_MCLK	0x2004
#define CLK_CON_GAT_CLK_BLK_ABOX_UID_DMIC_IPCLKPORT_CLK				0x2008
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_DAP_IPCLKPORT_DAPCLK			0x2018
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_ACLK			0x201c
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_DSIF		0x2020
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF0		0x2024
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF1		0x2028
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF2		0x202c
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF3		0x2030
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF4		0x2034
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_CCLK_ASB		0x2038
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_CCLK_ATB		0x203c
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_CCLK_CA7		0x2040
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_AXI_US_32TO128_IPCLKPORT_ACLK		0x2048
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_BTM_ABOX_IPCLKPORT_I_ACLK			0x204c
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_BTM_ABOX_IPCLKPORT_I_PCLK			0x2050
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_GPIO_ABOX_IPCLKPORT_PCLK			0x2058
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_LHM_AXI_P_ABOX_IPCLKPORT_I_CLK		0x205c
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_LHS_ATB_ABOX_IPCLKPORT_I_CLK		0x2060
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_LHS_AXI_D_ABOX_IPCLKPORT_I_CLK		0x2064
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_PERI_AXI_ASB_IPCLKPORT_ACLKM		0x2068
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_PERI_AXI_ASB_IPCLKPORT_ACLKS		0x206c
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_PERI_AXI_ASB_IPCLKPORT_PCLK		0x2070
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_PMU_ABOX_IPCLKPORT_PCLK			0x2074
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_BCM_ABOX_IPCLKPORT_ACLK			0x2078
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_BCM_ABOX_IPCLKPORT_PCLK			0x207c
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_BUSP_IPCLKPORT_CLK	0x2080
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_BUS_IPCLKPORT_CLK	0x2084
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_CPU_ACLK_IPCLKPORT_CLK	0x2088
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_CPU_ATCLK_IPCLKPORT_CLK	0x208c
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_CPU_IPCLKPORT_CLK	0x2090
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_CPU_PCLKDBG_IPCLKPORT_CLK	0x2094
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_DSIF_IPCLKPORT_CLK	0x2098
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF0_IPCLKPORT_CLK	0x209c
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF1_IPCLKPORT_CLK	0x20a0
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF2_IPCLKPORT_CLK	0x20a4
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF3_IPCLKPORT_CLK	0x20a8
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF4_IPCLKPORT_CLK	0x20ac
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_SMMU_ABOX_IPCLKPORT_ACLK			0x20b0
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_SMMU_ABOX_IPCLKPORT_PCLK			0x20b4
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_SYSREG_ABOX_IPCLKPORT_PCLK		0x20b8
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_TREX_ABOX_IPCLKPORT_CLK			0x20bc
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_WDT_ABOXCPU_IPCLKPORT_PCLK		0x20c0
#define CLK_CON_GAT_GOUT_BLK_ABOX_UID_WRAP2_CONV_ABOX_IPCLKPORT_I_CLK		0x20c4
#define DMYQCH_CON_ABOX_TOP_QCH							0x3000
#define DMYQCH_CON_DMIC_QCH							0x3004
#define QCH_CON_ABOX_CMU_ABOX_QCH						0x300c
#define QCH_CON_BTM_ABOX_QCH							0x3010
#define QCH_CON_GPIO_ABOX_QCH							0x3014
#define QCH_CON_LHM_AXI_P_ABOX_QCH						0x3018
#define QCH_CON_LHS_ATB_ABOX_QCH						0x301c
#define QCH_CON_LHS_AXI_D_ABOX_QCH						0x3020
#define QCH_CON_PMU_ABOX_QCH							0x3024
#define QCH_CON_BCM_ABOX_QCH							0x3028
#define QCH_CON_SMMU_ABOX_QCH							0x302c
#define QCH_CON_SYSREG_ABOX_QCH							0x3030
#define QCH_CON_TREX_ABOX_QCH							0x3034
#define QCH_CON_WDT_ABOXCPU_QCH							0x3038

/* Saved and restored across pd_abox power cycles, in the vendor order */
static const unsigned long abox_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_AUD,
	PLL_CON1_PLL_AUD,
	PLL_CON2_PLL_AUD,
	PLL_CON3_PLL_AUD,
	PLL_CON4_PLL_AUD,
	CLK_CON_DIV_DIV_CLK_ABOX_UAIF4,
	CLK_CON_DIV_DIV_CLK_ABOX_UAIF3,
	CLK_CON_DIV_DIV_CLK_ABOX_UAIF2,
	CLK_CON_DIV_DIV_CLK_ABOX_UAIF1,
	CLK_CON_DIV_DIV_CLK_ABOX_UAIF0,
	CLK_CON_DIV_DIV_CLK_ABOX_PLL,
	CLK_CON_DIV_DIV_CLK_ABOX_DSIF,
	CLK_CON_DIV_DIV_CLK_ABOX_DMIC,
	CLK_CON_DIV_DIV_CLK_ABOX_CPU_PCLKDBG,
	CLK_CON_DIV_DIV_CLK_ABOX_CPU_ATCLK,
	CLK_CON_DIV_DIV_CLK_ABOX_CPU_ACLK,
	CLK_CON_DIV_DIV_CLK_ABOX_BUSP,
	CLK_CON_DIV_DIV_CLK_ABOX_BUS,
	CLK_CON_DIV_DIV_CLK_ABOX_AUDIF,
	CLK_CON_MUX_MUX_CLK_ABOX_UAIF4,
	CLK_CON_MUX_MUX_CLK_ABOX_UAIF3,
	CLK_CON_MUX_MUX_CLK_ABOX_UAIF2,
	CLK_CON_MUX_MUX_CLK_ABOX_UAIF1,
	CLK_CON_MUX_MUX_CLK_ABOX_UAIF0,
	CLK_CON_MUX_MUX_CLK_ABOX_CPU,
	CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF4,
	CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF3,
	CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF2,
	CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF1,
	CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF0,
	CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_DSIF,
	PLL_CON0_MUX_CLKCMU_ABOX_CPUABOX_USER,
	PLL_CON0_PLL_AUD,
	QCH_CON_WDT_ABOXCPU_QCH,
	QCH_CON_TREX_ABOX_QCH,
	QCH_CON_SYSREG_ABOX_QCH,
	QCH_CON_SMMU_ABOX_QCH,
	QCH_CON_BCM_ABOX_QCH,
	QCH_CON_PMU_ABOX_QCH,
	QCH_CON_LHS_AXI_D_ABOX_QCH,
	QCH_CON_LHS_ATB_ABOX_QCH,
	QCH_CON_LHM_AXI_P_ABOX_QCH,
	QCH_CON_GPIO_ABOX_QCH,
	QCH_CON_BTM_ABOX_QCH,
	QCH_CON_ABOX_CMU_ABOX_QCH,
	DMYQCH_CON_DMIC_QCH,
	DMYQCH_CON_ABOX_TOP_QCH,
	ABOX_CMU_CONTROLLER_OPTION,
};

/* The vendor's 1179.648 and 1083.8016 MHz rows, at the exact rates M, P, S, K give */
static const struct samsung_pll_rate_table pll_aud_rate_table[] __initconst = {
	PLL_36XX_RATE(26 * MHZ, 1179648040U, 45, 1, 0, 24319),
	PLL_36XX_RATE(26 * MHZ, 1083801605U, 42, 1, 0, -20665),
};

/* List of parent clocks for Muxes in CMU_ABOX */
/* PLL_CON0 bit 4 selects between OSCCLK and the PLL output */
PNAME(mout_abox_pll_aud_p)		= { "oscclk", "fout_pll_aud" };
PNAME(mout_abox_cpuabox_user_p)		= { "oscclk", "dout_cmu_cmu_abox_cpuabox" };
PNAME(mout_abox_cpu_p)			= { "dout_abox_pll", "mout_abox_cpuabox_user" };
PNAME(mout_abox_uaif0_p)		= { "dout_abox_uaif0", "clkio_abox_uaif0" };
PNAME(mout_abox_uaif1_p)		= { "dout_abox_uaif1", "clkio_abox_uaif1" };
PNAME(mout_abox_uaif2_p)		= { "dout_abox_uaif2", "clkio_abox_uaif2" };
PNAME(mout_abox_uaif3_p)		= { "dout_abox_uaif3", "clkio_abox_uaif3" };
PNAME(mout_abox_uaif4_p)		= { "dout_abox_uaif4", "clkio_abox_uaif4" };

static const struct samsung_fixed_rate_clock abox_fixed_clks[] __initconst = {
	FRATE(0, "clkio_abox_uaif0", NULL, 0, 0),
	FRATE(0, "clkio_abox_uaif1", NULL, 0, 0),
	FRATE(0, "clkio_abox_uaif2", NULL, 0, 0),
	FRATE(0, "clkio_abox_uaif3", NULL, 0, 0),
	FRATE(0, "clkio_abox_uaif4", NULL, 0, 0),
};

static const struct samsung_pll_clock abox_pll_clks[] __initconst = {
	PLL(pll_1061x, CLK_FOUT_PLL_AUD, "fout_pll_aud", "oscclk",
	    PLL_LOCKTIME_PLL_AUD, PLL_CON0_PLL_AUD, pll_aud_rate_table),
};

static const struct samsung_mux_clock abox_mux_clks[] __initconst = {
	MUX_F(CLK_MOUT_ABOX_PLL_AUD, "mout_abox_pll_aud", mout_abox_pll_aud_p,
	      PLL_CON0_PLL_AUD, 4, 1, CLK_SET_RATE_PARENT, 0),
	MUX(CLK_MOUT_ABOX_CPUABOX_USER, "mout_abox_cpuabox_user",
	    mout_abox_cpuabox_user_p, PLL_CON0_MUX_CLKCMU_ABOX_CPUABOX_USER, 4,
	    1),
	MUX(CLK_MOUT_ABOX_CPU, "mout_abox_cpu", mout_abox_cpu_p,
	    CLK_CON_MUX_MUX_CLK_ABOX_CPU, 0, 1),
	MUX(CLK_MOUT_ABOX_UAIF0, "mout_abox_uaif0", mout_abox_uaif0_p,
	    CLK_CON_MUX_MUX_CLK_ABOX_UAIF0, 0, 1),
	MUX(CLK_MOUT_ABOX_UAIF1, "mout_abox_uaif1", mout_abox_uaif1_p,
	    CLK_CON_MUX_MUX_CLK_ABOX_UAIF1, 0, 1),
	MUX(CLK_MOUT_ABOX_UAIF2, "mout_abox_uaif2", mout_abox_uaif2_p,
	    CLK_CON_MUX_MUX_CLK_ABOX_UAIF2, 0, 1),
	MUX(CLK_MOUT_ABOX_UAIF3, "mout_abox_uaif3", mout_abox_uaif3_p,
	    CLK_CON_MUX_MUX_CLK_ABOX_UAIF3, 0, 1),
	MUX(CLK_MOUT_ABOX_UAIF4, "mout_abox_uaif4", mout_abox_uaif4_p,
	    CLK_CON_MUX_MUX_CLK_ABOX_UAIF4, 0, 1),
};

static const struct samsung_div_clock abox_div_clks[] __initconst = {
	DIV_F(CLK_DOUT_ABOX_PLL, "dout_abox_pll", "mout_abox_pll_aud",
	      CLK_CON_DIV_DIV_CLK_ABOX_PLL, 0, 4, 0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_AUDIF, "dout_abox_audif", "mout_abox_pll_aud",
	      CLK_CON_DIV_DIV_CLK_ABOX_AUDIF, 0, 9, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_BUS, "dout_abox_bus", "mout_abox_pll_aud",
	      CLK_CON_DIV_DIV_CLK_ABOX_BUS, 0, 3, 0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_BUSP, "dout_abox_busp", "dout_abox_bus",
	      CLK_CON_DIV_DIV_CLK_ABOX_BUSP, 0, 2, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_CPU_ACLK, "dout_abox_cpu_aclk", "mout_abox_cpu",
	      CLK_CON_DIV_DIV_CLK_ABOX_CPU_ACLK, 0, 3, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_CPU_ATCLK, "dout_abox_cpu_atclk", "mout_abox_cpu",
	      CLK_CON_DIV_DIV_CLK_ABOX_CPU_ATCLK, 0, 3, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_CPU_PCLKDBG, "dout_abox_cpu_pclkdbg",
	      "mout_abox_cpu", CLK_CON_DIV_DIV_CLK_ABOX_CPU_PCLKDBG, 0, 3, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_DSIF, "dout_abox_dsif", "dout_abox_audif",
	      CLK_CON_DIV_DIV_CLK_ABOX_DSIF, 0, 5, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_DMIC, "dout_abox_dmic", "dout_abox_dsif",
	      CLK_CON_DIV_DIV_CLK_ABOX_DMIC, 0, 2, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_UAIF0, "dout_abox_uaif0", "dout_abox_audif",
	      CLK_CON_DIV_DIV_CLK_ABOX_UAIF0, 0, 5, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_UAIF1, "dout_abox_uaif1", "dout_abox_audif",
	      CLK_CON_DIV_DIV_CLK_ABOX_UAIF1, 0, 5, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_UAIF2, "dout_abox_uaif2", "dout_abox_audif",
	      CLK_CON_DIV_DIV_CLK_ABOX_UAIF2, 0, 5, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_UAIF3, "dout_abox_uaif3", "dout_abox_audif",
	      CLK_CON_DIV_DIV_CLK_ABOX_UAIF3, 0, 5, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_ABOX_UAIF4, "dout_abox_uaif4", "dout_abox_audif",
	      CLK_CON_DIV_DIV_CLK_ABOX_UAIF4, 0, 5, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
};

static const struct samsung_gate_clock abox_gate_clks[] __initconst = {
	GATE(CLK_GOUT_ABOX_BCLK_UAIF0, "gout_abox_top_bclk_uaif0",
	     "mout_abox_uaif0",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF0, 21, 0,
	     0),
	GATE(CLK_GOUT_ABOX_BCLK_UAIF1, "gout_abox_top_bclk_uaif1",
	     "mout_abox_uaif1",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF1, 21, 0,
	     0),
	GATE(CLK_GOUT_ABOX_BCLK_UAIF2, "gout_abox_top_bclk_uaif2",
	     "mout_abox_uaif2",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF2, 21, 0,
	     0),
	GATE(CLK_GOUT_ABOX_BCLK_UAIF3, "gout_abox_top_bclk_uaif3",
	     "mout_abox_uaif3",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF3, 21, 0,
	     0),
	GATE(CLK_GOUT_ABOX_BCLK_UAIF4, "gout_abox_top_bclk_uaif4",
	     "mout_abox_uaif4",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF4, 21, 0,
	     0),
	GATE(CLK_GOUT_ABOX_BCLK_DSIF, "gout_abox_top_bclk_dsif",
	     "dout_abox_dsif",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_DSIF, 21, 0,
	     0),
	GATE(CLK_GOUT_ABOX_CCLK_CA7, "gout_abox_top_cclk_ca7", "mout_abox_cpu",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_CCLK_CA7, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ABOX_SMMU_ACLK, "gout_abox_smmu_abox_aclk",
	     "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_SMMU_ABOX_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ABOX_SMMU_PCLK, "gout_abox_smmu_abox_pclk",
	     "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_SMMU_ABOX_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ABOX_DMIC, "gout_abox_dmic_clk", "dout_abox_dmic",
	     CLK_CON_GAT_CLK_BLK_ABOX_UID_DMIC_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ABOX_CODEC_MCLK, "gout_abox_dftmux_abox_aud_codec_mclk",
	     "dout_abox_audif",
	     CLK_CON_GAT_CLK_BLK_ABOX_UID_DFTMUX_ABOX_IPCLKPORT_AUD_CODEC_MCLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_cmu_abox_pclk", "dout_abox_bus",
	     CLK_CON_GAT_CLK_BLK_ABOX_UID_ABOX_CMU_ABOX_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_dap_dapclk", "dout_abox_cpu_pclkdbg",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_DAP_IPCLKPORT_DAPCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_top_aclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_top_cclk_asb", "dout_abox_cpu_aclk",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_CCLK_ASB, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_top_cclk_atb", "dout_abox_cpu_atclk",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_CCLK_ATB, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_axi_us_32to128_aclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_AXI_US_32TO128_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_btm_abox_i_aclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_BTM_ABOX_IPCLKPORT_I_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_btm_abox_i_pclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_BTM_ABOX_IPCLKPORT_I_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_gpio_abox_pclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_GPIO_ABOX_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_lhm_axi_p_abox_i_clk", "dout_abox_busp",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_LHM_AXI_P_ABOX_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_lhs_atb_abox_i_clk", "dout_abox_cpu_atclk",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_LHS_ATB_ABOX_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_lhs_axi_d_abox_i_clk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_LHS_AXI_D_ABOX_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_peri_axi_asb_aclkm", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_PERI_AXI_ASB_IPCLKPORT_ACLKM, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_peri_axi_asb_aclks", "dout_abox_busp",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_PERI_AXI_ASB_IPCLKPORT_ACLKS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_peri_axi_asb_pclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_PERI_AXI_ASB_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_pmu_abox_pclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_PMU_ABOX_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_bcm_abox_aclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_BCM_ABOX_IPCLKPORT_ACLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_bcm_abox_pclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_BCM_ABOX_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_busp_clk", "dout_abox_busp",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_BUSP_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_bus_clk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_BUS_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_cpu_aclk_clk",
	     "dout_abox_cpu_aclk",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_CPU_ACLK_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_cpu_atclk_clk",
	     "dout_abox_cpu_atclk",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_CPU_ATCLK_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_cpu_clk", "mout_abox_cpu",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_CPU_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_cpu_pclkdbg_clk",
	     "dout_abox_cpu_pclkdbg",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_CPU_PCLKDBG_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_dsif_clk", "dout_abox_dsif",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_DSIF_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_uaif0_clk", "mout_abox_uaif0",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF0_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_uaif1_clk", "mout_abox_uaif1",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF1_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_uaif2_clk", "mout_abox_uaif2",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF2_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_uaif3_clk", "mout_abox_uaif3",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF3_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_rstnsync_clk_abox_uaif4_clk", "mout_abox_uaif4",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_RSTNSYNC_CLK_ABOX_UAIF4_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_sysreg_abox_pclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_SYSREG_ABOX_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_trex_abox_clk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_TREX_ABOX_IPCLKPORT_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_wdt_aboxcpu_pclk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_WDT_ABOXCPU_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_abox_wrap2_conv_abox_i_clk", "dout_abox_bus",
	     CLK_CON_GAT_GOUT_BLK_ABOX_UID_WRAP2_CONV_ABOX_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info abox_cmu_info __initconst = {
	.pll_clks		= abox_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(abox_pll_clks),
	.mux_clks		= abox_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(abox_mux_clks),
	.div_clks		= abox_div_clks,
	.nr_div_clks		= ARRAY_SIZE(abox_div_clks),
	.gate_clks		= abox_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(abox_gate_clks),
	.fixed_clks		= abox_fixed_clks,
	.nr_fixed_clks		= ARRAY_SIZE(abox_fixed_clks),
	.nr_clk_ids		= CLKS_NR_ABOX,
	.clk_regs		= abox_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(abox_clk_regs),
	.clk_name		= "cpuabox",
};

/* ---- CMU_VTS ------------------------------------------------------------- */

/* Register Offset definitions for CMU_VTS (0x14000000) */
#define OSC_LOCKTIME_OSC_VTS							0x0000
#define OSC_CON0_OSC_VTS							0x0100
#define OSC_CON1_OSC_VTS							0x0104
#define OSC_CON2_OSC_VTS							0x0108
#define OSC_CON4_OSC_VTS							0x0110
#define VTS_CMU_CONTROLLER_OPTION						0x0800
#define CLK_CON_MUX_MUX_CLK_VTS_CMUREF						0x1000
#define CLK_CON_DIV_DIV_CLK_VTS_BUS						0x1800
#define CLK_CON_DIV_DIV_CLK_VTS_CMUREF						0x1804
#define CLK_CON_DIV_DIV_CLK_VTS_DMIC						0x1808
#define CLK_CON_DIV_DIV_CLK_VTS_DMICIF						0x180c
#define CLK_CON_DIV_DIV_CLK_VTS_DMIC_DIV2					0x1810
#define CLK_CON_GAT_CLK_BLK_VTS_UID_DMIC_IF_IPCLKPORT_DMIC_CLK			0x2000
#define CLK_CON_GAT_CLK_BLK_VTS_UID_VTS_CMU_VTS_IPCLKPORT_PCLK			0x200c
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_AHB_IPCLKPORT_HCLK			0x2014
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_AHB_IPCLKPORT_PCLK			0x2018
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_IF_IPCLKPORT_DMICIF_CLK		0x201c
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_IF_IPCLKPORT_DMIC_DIV2_CLK		0x2020
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_IF_IPCLKPORT_PCLK			0x2024
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_GPIO_VTS_IPCLKPORT_PCLK			0x2028
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_LHM_AXI_P_VTS_IPCLKPORT_I_CLK		0x202c
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_LHS_AXI_D_VTS_IPCLKPORT_I_CLK		0x2030
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_MAILBOX_VTS2AP_IPCLKPORT_PCLK		0x2034
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_RSTNSYNC_CLK_VTS_BUS_IPCLKPORT_CLK		0x2038
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_RSTNSYNC_CLK_VTS_DMICIF_IPCLKPORT_CLK	0x203c
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_SYSREG_VTS_IPCLKPORT_PCLK			0x2040
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_VTS_IPCLKPORT_ACLK_CPU			0x2044
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_VTS_IPCLKPORT_ACLK_SYS			0x2048
#define CLK_CON_GAT_GOUT_BLK_VTS_UID_WDT_VTS_IPCLKPORT_PCLK			0x204c
#define DMYQCH_CON_CMU_VTS_CMUREF_QCH						0x3000
#define DMYQCH_CON_DMIC_AHB_QCH_HCLK						0x3004
#define DMYQCH_CON_DMIC_IF_QCH_DMIC_CLK						0x3008
#define QCH_CON_DMIC_AHB_QCH_PCLK						0x300c
#define QCH_CON_DMIC_IF_QCH_PCLK						0x3010
#define QCH_CON_GPIO_VTS_QCH							0x3014
#define QCH_CON_LHM_AXI_P_VTS_QCH						0x3018
#define QCH_CON_LHS_AXI_D_VTS_QCH						0x301c
#define QCH_CON_MAILBOX_VTS2AP_QCH						0x3020
#define QCH_CON_SYSREG_VTS_QCH							0x3024
#define QCH_CON_VTS_CMU_VTS_QCH							0x3028
#define QCH_CON_VTS_QCH_CPU							0x302c
#define QCH_CON_VTS_QCH_SYS							0x3030
#define QCH_CON_VTS_QCH_SYS_DMIC						0x3034
#define QCH_CON_WDT_VTS_QCH							0x3038

/* Saved and restored across pd_vts power cycles, in the vendor order */
static const unsigned long vts_clk_regs[] __initconst = {
	OSC_LOCKTIME_OSC_VTS,
	OSC_CON1_OSC_VTS,
	OSC_CON2_OSC_VTS,
	OSC_CON4_OSC_VTS,
	CLK_CON_DIV_DIV_CLK_VTS_DMIC_DIV2,
	CLK_CON_DIV_DIV_CLK_VTS_DMICIF,
	CLK_CON_DIV_DIV_CLK_VTS_DMIC,
	CLK_CON_DIV_DIV_CLK_VTS_CMUREF,
	CLK_CON_DIV_DIV_CLK_VTS_BUS,
	CLK_CON_MUX_MUX_CLK_VTS_CMUREF,
	OSC_CON0_OSC_VTS,
	QCH_CON_WDT_VTS_QCH,
	QCH_CON_VTS_QCH_SYS_DMIC,
	QCH_CON_VTS_QCH_SYS,
	QCH_CON_VTS_QCH_CPU,
	QCH_CON_VTS_CMU_VTS_QCH,
	QCH_CON_SYSREG_VTS_QCH,
	QCH_CON_MAILBOX_VTS2AP_QCH,
	QCH_CON_LHS_AXI_D_VTS_QCH,
	QCH_CON_LHM_AXI_P_VTS_QCH,
	QCH_CON_GPIO_VTS_QCH,
	QCH_CON_DMIC_IF_QCH_PCLK,
	QCH_CON_DMIC_AHB_QCH_PCLK,
	DMYQCH_CON_DMIC_IF_QCH_DMIC_CLK,
	DMYQCH_CON_DMIC_AHB_QCH_HCLK,
	DMYQCH_CON_CMU_VTS_CMUREF_QCH,
	VTS_CMU_CONTROLLER_OPTION,
};

/* List of parent clocks for Muxes in CMU_VTS */
PNAME(mout_vts_cmuref_p)		= { "oscclk", "vts_cmuref" };

static const struct samsung_fixed_rate_clock vts_fixed_clks[] __initconst = {
	FRATE(CLK_VTS_OSC, "vts_osc", NULL, 0, 49152000),
};

static const struct samsung_fixed_factor_clock vts_fixed_factor_clks[] __initconst = {
	FFACTOR(0, "vts_cmuref", "vts_osc", 1, 2, 0),
};

static const struct samsung_mux_clock vts_mux_clks[] __initconst = {
	MUX(CLK_MOUT_VTS_CMUREF, "mout_vts_cmuref", mout_vts_cmuref_p,
	    CLK_CON_MUX_MUX_CLK_VTS_CMUREF, 0, 1),
};

static const struct samsung_div_clock vts_div_clks[] __initconst = {
	DIV_F(CLK_DOUT_VTS_BUS, "dout_vts_bus", "vts_osc",
	      CLK_CON_DIV_DIV_CLK_VTS_BUS, 0, 6, 0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_VTS_DMICIF, "dout_vts_dmicif", "vts_osc",
	      CLK_CON_DIV_DIV_CLK_VTS_DMICIF, 0, 6, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_VTS_DMIC, "dout_vts_dmic", "dout_vts_dmicif",
	      CLK_CON_DIV_DIV_CLK_VTS_DMIC, 0, 1, 0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_F(CLK_DOUT_VTS_DMIC_DIV2, "dout_vts_dmic_div2", "dout_vts_dmicif",
	      CLK_CON_DIV_DIV_CLK_VTS_DMIC_DIV2, 0, 1, 0,
	      CLK_DIVIDER_ROUND_CLOSEST),
};

static const struct samsung_gate_clock vts_gate_clks[] __initconst = {
	GATE(CLK_GOUT_VTS_DMICIF, "gout_vts_dmic_if_dmicif_clk",
	     "dout_vts_dmicif",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_IF_IPCLKPORT_DMICIF_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_VTS_DMIC_DIV2, "gout_vts_dmic_if_dmic_div2_clk",
	     "dout_vts_dmic_div2",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_IF_IPCLKPORT_DMIC_DIV2_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_VTS_DMIC, "gout_vts_dmic_if_dmic_clk", "dout_vts_dmic",
	     CLK_CON_GAT_CLK_BLK_VTS_UID_DMIC_IF_IPCLKPORT_DMIC_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_cmu_vts_pclk", "dout_vts_bus",
	     CLK_CON_GAT_CLK_BLK_VTS_UID_VTS_CMU_VTS_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_dmic_ahb_hclk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_AHB_IPCLKPORT_HCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_dmic_ahb_pclk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_AHB_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_dmic_if_pclk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_DMIC_IF_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_gpio_vts_pclk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_GPIO_VTS_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_lhm_axi_p_vts_i_clk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_LHM_AXI_P_VTS_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_lhs_axi_d_vts_i_clk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_LHS_AXI_D_VTS_IPCLKPORT_I_CLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_mailbox_vts2ap_pclk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_MAILBOX_VTS2AP_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_rstnsync_clk_vts_bus_clk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_RSTNSYNC_CLK_VTS_BUS_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_rstnsync_clk_vts_dmicif_clk", "dout_vts_dmicif",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_RSTNSYNC_CLK_VTS_DMICIF_IPCLKPORT_CLK,
	     21, CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_sysreg_vts_pclk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_SYSREG_VTS_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_aclk_cpu", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_VTS_IPCLKPORT_ACLK_CPU, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_aclk_sys", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_VTS_IPCLKPORT_ACLK_SYS, 21,
	     CLK_IGNORE_UNUSED, 0),
	GATE(0, "gout_vts_wdt_vts_pclk", "dout_vts_bus",
	     CLK_CON_GAT_GOUT_BLK_VTS_UID_WDT_VTS_IPCLKPORT_PCLK, 21,
	     CLK_IGNORE_UNUSED, 0),
};

/* The vendor powers pd_vts off with MUX_CLK_VTS_CMUREF on its first input */
static const struct samsung_clk_reg_dump vts_suspend_regs[] = {
	{ CLK_CON_MUX_MUX_CLK_VTS_CMUREF, 0x0 },
};

static const struct samsung_cmu_info vts_cmu_info __initconst = {
	.mux_clks		= vts_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(vts_mux_clks),
	.div_clks		= vts_div_clks,
	.nr_div_clks		= ARRAY_SIZE(vts_div_clks),
	.gate_clks		= vts_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(vts_gate_clks),
	.fixed_clks		= vts_fixed_clks,
	.nr_fixed_clks		= ARRAY_SIZE(vts_fixed_clks),
	.fixed_factor_clks	= vts_fixed_factor_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(vts_fixed_factor_clks),
	.nr_clk_ids		= CLKS_NR_VTS,
	.clk_regs		= vts_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(vts_clk_regs),
	.suspend_regs		= vts_suspend_regs,
	.nr_suspend_regs	= ARRAY_SIZE(vts_suspend_regs),
};

/* ---- CMU probe glue ------------------------------------------------------ */

#define CMU_QCH_ENABLE			BIT(0)
#define CMU_QCH_CLOCK_REQ		BIT(1)

/*
 * Clear ENABLE and CLOCK_REQ of the USI Q-channels before the clocks are
 * registered, as the vendor kernel does; the request gates drive CLOCK_REQ.
 */
static const u16 exynos8895_peric0_usi_qch[] __initconst = {
	QCH_CON_USI00_QCH, QCH_CON_USI01_QCH, QCH_CON_USI02_QCH,
	QCH_CON_USI03_QCH,
};

static const u16 exynos8895_peric1_usi_qch[] __initconst = {
	QCH_CON_USI04_QCH, QCH_CON_USI05_QCH, QCH_CON_USI06_QCH,
	QCH_CON_USI07_QCH, QCH_CON_USI08_QCH, QCH_CON_USI09_QCH,
	QCH_CON_USI10_QCH, QCH_CON_USI11_QCH, QCH_CON_USI12_QCH,
	QCH_CON_USI13_QCH,
};

static void __init exynos8895_usi_qch_init(struct device_node *np,
					   const u16 *qch, unsigned int n)
{
	void __iomem *base = of_iomap(np, 0);
	unsigned int i;

	if (!base)
		return;

	for (i = 0; i < n; i++)
		writel(readl(base + qch[i]) & ~(CMU_QCH_CLOCK_REQ | CMU_QCH_ENABLE),
		       base + qch[i]);
	iounmap(base);
}

static int __init exynos8895_cmu_probe(struct platform_device *pdev)
{
	const struct samsung_cmu_info *info;
	struct device *dev = &pdev->dev;

	info = of_device_get_match_data(dev);

	if (info == &peric0_cmu_info)
		exynos8895_usi_qch_init(dev->of_node, exynos8895_peric0_usi_qch,
					ARRAY_SIZE(exynos8895_peric0_usi_qch));
	else if (info == &peric1_cmu_info)
		exynos8895_usi_qch_init(dev->of_node, exynos8895_peric1_usi_qch,
					ARRAY_SIZE(exynos8895_peric1_usi_qch));

	exynos_arm64_register_cmu(dev, dev->of_node, info);

	return 0;
}

static const struct of_device_id exynos8895_cmu_of_match[] = {
	{
		.compatible = "samsung,exynos8895-cmu-fsys0",
		.data = &fsys0_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-fsys1",
		.data = &fsys1_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-peric0",
		.data = &peric0_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-peric1",
		.data = &peric1_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-apm",
		.data = &apm_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-cpucl0",
		.data = &cpucl0_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-cpucl1",
		.data = &cpucl1_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-busc",
		.data = &busc_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-g3d",
		.data = &g3d_cmu_info,
	},
	{ }
};

static struct platform_driver exynos8895_cmu_driver __refdata = {
	.driver = {
		.name = "exynos8895-cmu",
		.of_match_table = exynos8895_cmu_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = exynos8895_cmu_probe,
};

/*
 * The vendor kernel sets the BUSP divider of these CMUs once at boot, to a
 * fixed ratio of their bus clock, which DVFS scales.
 */
struct exynos8895_busp_ratio {
	const char *compat;
	unsigned int id, div;
};

static const struct exynos8895_busp_ratio exynos8895_busp[] __initconst = {
	{ "samsung,exynos8895-cmu-mfc", CLK_DOUT_MFC_BUSP, 4 },
	{ "samsung,exynos8895-cmu-cam", CLK_DOUT_CAM_BUSP, 2 },
	{ "samsung,exynos8895-cmu-g2d", CLK_DOUT_G2D_BUSP, 2 },
	{ "samsung,exynos8895-cmu-isphq", CLK_DOUT_ISPHQ_BUSP, 2 },
	{ "samsung,exynos8895-cmu-isplp", CLK_DOUT_ISPLP_BUSP, 2 },
	{ "samsung,exynos8895-cmu-dcam", CLK_DOUT_DCAM_BUSP, 3 },
	{ "samsung,exynos8895-cmu-srdz", CLK_DOUT_SRDZ_BUSP, 2 },
};

static void __init exynos8895_cmu_set_busp(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct of_phandle_args spec = { .np = np, .args_count = 1 };
	unsigned long prate;
	struct clk *clk;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(exynos8895_busp); i++) {
		if (!of_device_is_compatible(np, exynos8895_busp[i].compat))
			continue;

		spec.args[0] = exynos8895_busp[i].id;
		clk = of_clk_get_from_provider(&spec);
		if (IS_ERR(clk))
			return;

		/* Round up: divider_get_val() then picks exactly div - 1 */
		prate = clk_get_rate(clk_get_parent(clk));
		if (prate) {
			ret = clk_set_rate(clk,
					   DIV_ROUND_UP(prate, exynos8895_busp[i].div));
			if (ret)
				dev_warn(&pdev->dev, "BUSP 1/%u: %d\n",
					 exynos8895_busp[i].div, ret);
		}
		clk_put(clk);
		return;
	}
}

/*
 * The vendor kernel programs the audio blocks once at boot, outside its clock
 * framework; pd_abox and pd_vts only restore what was written since.
 */
#define CMU_PLL_ENABLE			BIT(31)
#define CMU_PLL_STABLE			BIT(29)
#define CMU_PLL_MUX_BUSY		BIT(7)
#define CMU_PLL_MUX_SEL			BIT(4)
#define CMU_PLL_PMS_MASK		(GENMASK(25, 16) | GENMASK(13, 8) | GENMASK(2, 0))
#define CMU_PLL_PMS(m, p, s)		(((m) << 16) | ((p) << 8) | (s))
#define CMU_BUSY			BIT(16)
#define CMU_GATE_CG_VAL			BIT(21)
#define CMU_GATE_MANUAL			BIT(20)

static void __iomem *cmu_abox_base, *cmu_vts_base;

static int exynos8895_cmu_wait_bit(void __iomem *reg, u32 bit, bool set, int us)
{
	while (us--) {
		if (!!(readl(reg) & bit) == set)
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

static void exynos8895_cmu_rmw(void __iomem *reg, u32 mask, u32 val)
{
	writel((readl(reg) & ~mask) | val, reg);
}

/*
 * Vendor boot setup of BLK_ABOX (pd_abox on, ABOX CPU off): PLL_AUD at
 * 1179.648 MHz, the CPU/BUSP/AUDIF/DSIF dividers, BCLK gates manual and off.
 */
static void __init exynos8895_cmu_abox_boot_init(void __iomem *b)
{
	int i;

	exynos8895_cmu_rmw(b + PLL_CON0_PLL_AUD, CMU_PLL_MUX_SEL, 0);
	exynos8895_cmu_wait_bit(b + PLL_CON0_PLL_AUD, CMU_PLL_MUX_BUSY, false, 100);
	exynos8895_cmu_rmw(b + PLL_CON0_PLL_AUD, CMU_PLL_ENABLE, 0);
	writel(1 * 1500, b + PLL_LOCKTIME_PLL_AUD);
	exynos8895_cmu_rmw(b + PLL_CON3_PLL_AUD, 0xffff, 24319);
	exynos8895_cmu_rmw(b + PLL_CON0_PLL_AUD,
			   CMU_PLL_PMS_MASK | CMU_PLL_ENABLE | CMU_PLL_MUX_SEL,
			   CMU_PLL_PMS(45, 1, 0) | CMU_PLL_ENABLE | CMU_PLL_MUX_SEL);
	if (exynos8895_cmu_wait_bit(b + PLL_CON0_PLL_AUD, CMU_PLL_STABLE, true, 100))
		pr_err("cmu_abox: PLL_AUD lock timeout\n");

	exynos8895_cmu_rmw(b + CLK_CON_DIV_DIV_CLK_ABOX_CPU_ATCLK, 0x7, 1);
	exynos8895_cmu_wait_bit(b + CLK_CON_DIV_DIV_CLK_ABOX_CPU_ATCLK, CMU_BUSY, false, 100);
	exynos8895_cmu_rmw(b + CLK_CON_DIV_DIV_CLK_ABOX_BUSP, 0x3, 2);
	exynos8895_cmu_wait_bit(b + CLK_CON_DIV_DIV_CLK_ABOX_BUSP, CMU_BUSY, false, 100);
	exynos8895_cmu_rmw(b + CLK_CON_DIV_DIV_CLK_ABOX_AUDIF, 0x1ff, 23);
	exynos8895_cmu_wait_bit(b + CLK_CON_DIV_DIV_CLK_ABOX_AUDIF, CMU_BUSY, false, 100);
	exynos8895_cmu_rmw(b + CLK_CON_DIV_DIV_CLK_ABOX_DSIF, 0x1f, 1);
	exynos8895_cmu_wait_bit(b + CLK_CON_DIV_DIV_CLK_ABOX_DSIF, CMU_BUSY, false, 100);

	for (i = CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_DSIF;
	     i <= CLK_CON_GAT_GOUT_BLK_ABOX_UID_ABOX_TOP_IPCLKPORT_BCLK_UAIF4; i += 4)
		exynos8895_cmu_rmw(b + i, CMU_GATE_MANUAL | CMU_GATE_CG_VAL,
				   CMU_GATE_MANUAL);
}

/*
 * Vendor boot setup of BLK_VTS: the BUS and DMICIF dividers, the CMUREF mux,
 * and the OSC_VTS oscillator, which the VTS driver enables once for good.
 */
static void __init exynos8895_cmu_vts_boot_init(void __iomem *b)
{
	exynos8895_cmu_rmw(b + CLK_CON_DIV_DIV_CLK_VTS_BUS, 0x3f, 0);
	exynos8895_cmu_wait_bit(b + CLK_CON_DIV_DIV_CLK_VTS_BUS, CMU_BUSY, false, 100);
	exynos8895_cmu_rmw(b + CLK_CON_MUX_MUX_CLK_VTS_CMUREF, BIT(0), 0);
	exynos8895_cmu_wait_bit(b + CLK_CON_MUX_MUX_CLK_VTS_CMUREF, CMU_BUSY, false, 100);
	exynos8895_cmu_rmw(b + CLK_CON_DIV_DIV_CLK_VTS_DMICIF, 0x3f, 11);
	exynos8895_cmu_wait_bit(b + CLK_CON_DIV_DIV_CLK_VTS_DMICIF, CMU_BUSY, false, 100);
	exynos8895_cmu_rmw(b + OSC_CON0_OSC_VTS, CMU_PLL_ENABLE | CMU_PLL_MUX_SEL,
			   CMU_PLL_ENABLE | CMU_PLL_MUX_SEL);
	if (exynos8895_cmu_wait_bit(b + OSC_CON0_OSC_VTS, CMU_PLL_STABLE, true, 400))
		pr_err("cmu_vts: OSC_VTS stable timeout\n");
}

/* CMUs in power domains keep their gates in the mode the boot loader left */
static int __init exynos8895_cmu_pd_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	int ret;

	if (of_device_is_compatible(np, "samsung,exynos8895-cmu-abox")) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			return -EINVAL;
		cmu_abox_base = ioremap(res->start, resource_size(res));
		if (!cmu_abox_base)
			return -ENOMEM;
		exynos8895_cmu_abox_boot_init(cmu_abox_base);
	} else if (of_device_is_compatible(np, "samsung,exynos8895-cmu-vts")) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			return -EINVAL;
		cmu_vts_base = ioremap(res->start, resource_size(res));
		if (!cmu_vts_base)
			return -ENOMEM;
		exynos8895_cmu_vts_boot_init(cmu_vts_base);
	}

	ret = exynos_arm64_register_cmu_pm(pdev, false);
	if (ret)
		return ret;

	exynos8895_cmu_set_busp(pdev);

	return 0;
}

static bool exynos8895_cmu_pd_is_audio(struct device *dev)
{
	return of_device_is_compatible(dev->of_node, "samsung,exynos8895-cmu-abox") ||
	       of_device_is_compatible(dev->of_node, "samsung,exynos8895-cmu-vts");
}

/* After a restore, wait for an enabled PLL_AUD or OSC_VTS to lock again */
static int exynos8895_cmu_pd_runtime_resume(struct device *dev)
{
	int ret = exynos_arm64_cmu_resume(dev);
	void __iomem *r = NULL;

	if (cmu_abox_base &&
	    of_device_is_compatible(dev->of_node, "samsung,exynos8895-cmu-abox"))
		r = cmu_abox_base + PLL_CON0_PLL_AUD;
	else if (cmu_vts_base &&
		 of_device_is_compatible(dev->of_node, "samsung,exynos8895-cmu-vts"))
		r = cmu_vts_base + OSC_CON0_OSC_VTS;
	if (r && (readl(r) & CMU_PLL_ENABLE) &&
	    exynos8895_cmu_wait_bit(r, CMU_PLL_STABLE, true, 1000))
		dev_err(dev, "PLL/OSC relock timeout\n");

	return ret;
}

/* vts_suspend_regs switch MUX_CLK_VTS_CMUREF: wait for the mux to settle */
static int exynos8895_cmu_pd_runtime_suspend(struct device *dev)
{
	int ret = exynos_arm64_cmu_suspend(dev);

	if (cmu_vts_base &&
	    of_device_is_compatible(dev->of_node, "samsung,exynos8895-cmu-vts"))
		exynos8895_cmu_wait_bit(cmu_vts_base + CLK_CON_MUX_MUX_CLK_VTS_CMUREF,
					CMU_BUSY, false, 1000);

	return ret;
}

/*
 * genpd powers these domains off in suspend_noirq even with the CMU runtime
 * active; CMU_DPU0 always is, as the DPU0 syscon keeps its PCLK prepared.
 */
static bool exynos8895_cmu_pd_sleep_save(struct device *dev)
{
	return exynos8895_cmu_pd_is_audio(dev) ||
	       of_device_is_compatible(dev->of_node, "samsung,exynos8895-cmu-dpu0");
}

static int exynos8895_cmu_pd_suspend_noirq(struct device *dev)
{
	return exynos8895_cmu_pd_sleep_save(dev) ? pm_runtime_force_suspend(dev) : 0;
}

static int exynos8895_cmu_pd_resume_noirq(struct device *dev)
{
	return exynos8895_cmu_pd_sleep_save(dev) ? pm_runtime_force_resume(dev) : 0;
}

static const struct of_device_id exynos8895_cmu_pd_of_match[] = {
	{
		.compatible = "samsung,exynos8895-cmu-dpu0",
		.data = &dpu0_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-dpu1",
		.data = &dpu1_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-mfc",
		.data = &mfc_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-cam",
		.data = &cam_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-g2d",
		.data = &g2d_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-isphq",
		.data = &isphq_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-isplp",
		.data = &isplp_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-dcam",
		.data = &dcam_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-srdz",
		.data = &srdz_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-abox",
		.data = &abox_cmu_info,
	}, {
		.compatible = "samsung,exynos8895-cmu-vts",
		.data = &vts_cmu_info,
	},
	{ }
};

static const struct dev_pm_ops exynos8895_cmu_pd_pm_ops = {
	SET_RUNTIME_PM_OPS(exynos8895_cmu_pd_runtime_suspend,
			   exynos8895_cmu_pd_runtime_resume, NULL)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(exynos8895_cmu_pd_suspend_noirq,
				      exynos8895_cmu_pd_resume_noirq)
};

static struct platform_driver exynos8895_cmu_pd_driver __refdata = {
	.driver = {
		.name = "exynos8895-cmu-pd",
		.of_match_table = exynos8895_cmu_pd_of_match,
		.suppress_bind_attrs = true,
		.pm = &exynos8895_cmu_pd_pm_ops,
	},
	.probe = exynos8895_cmu_pd_probe,
};

static int __init exynos8895_cmu_init(void)
{
	int ret;

	ret = platform_driver_register(&exynos8895_cmu_driver);
	if (ret)
		return ret;

	return platform_driver_register(&exynos8895_cmu_pd_driver);
}
core_initcall(exynos8895_cmu_init);
