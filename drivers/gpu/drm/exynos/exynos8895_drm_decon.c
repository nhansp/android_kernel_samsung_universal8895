// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 8895 DECON0 ("decon_f") with its DPP/IDMA channels, driven as one
 * CRTC for a MIPI DSI command-mode panel with DSC.
 *
 * The register programming reproduces the vendor driver of the Exynos 8895
 * kernel (S:drivers/video/fbdev/exynos/dpu/decon_reg.c, dpp_reg.c,
 * decon_core.c, dpp_drv.c); the DRM structure follows exynos5433_drm_decon.c.
 * No register value is borrowed from another DECON generation.
 */

#include <linux/aperture.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soc/samsung/greatlte-l5.h>

#include <soc/samsung/bts.h>
#include <soc/samsung/exynos8895-bus-qos.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_connector.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_panic.h>
#include <drm/drm_vblank.h>
#include <drm/exynos8895_hiber.h>
#include <drm/exynos8895_panel.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos8895_drm_decon.h"
#include "regs-decon8895.h"
#include "regs-dpp8895.h"

#define D8895_NR_CH		6	/* G0 G1 VG0 VG1 VGF0 VGF1 */

/* DEBUG: heartbeat counters of greatlte-hb-l5.c, only when built in */
#define GL5_ON			IS_BUILTIN(CONFIG_DRM_EXYNOS)
#define D8895_NR_WIN		6	/* FDT:7538 max_win = <0x06> */
#define D8895_CH_G0		0	/* S:.../dpu/decon.h:310 IDMA_G0 = 0 */
#define D8895_WIN_G0		5	/* FDT:7539 default_win = <0x05> */

/* SYSREG_DPU0: the syscon maps 0x12820000; the DISP_SS registers are at +0x1000 */
#define D8895_SS(r)		(0x1000 + (r))
#define D8895_SYSREG_DRCG_EN	0x200	/* CAL:pmucal_local_exynos8895.h:230 */

/* DPU_DMA channel n at +0x1000 * (n + 1), DPP channel n at +0x1000 * n */
#define D8895_DMA_CH(n)		(0x1000 * ((n) + 1))
#define D8895_DPP_CH(n)		(0x1000 * (n))

#define D8895_SHADOW_TIMEOUT_US	(100 * 1000)	/* S:.../dpu/decon.h:112 */
#define D8895_IDLE_TIMEOUT_US	(50 * 1000)	/* S:.../dpu/decon.h:113 IDLE_WAIT_TIMEOUT */
#define D8895_RUN_TIMEOUT_US	(20 * 1000)	/* decon_reg.c:2105,2199 */
#define D8895_ACLK_KHZ_DEFAULT	630000		/* decon_core.c:1640 */
#define D8895_INIT_RCV_NUM	630000		/* S:.../dpu/dpp.h:45 */

/* bus-vote arithmetic (S:drivers/video/fbdev/exynos/dpu/bts.c, REQ-2B-L5-1) */
#define D8895_BTS_BUS_UTIL	75	/* S:dpu/bts.c:91 BUS_UTIL */
#define D8895_BTS_LCD_REFRESH	63	/* S:dpu/bts.c:19 LCD_REFRESH_RATE */
#define D8895_BTS_PPC		2	/* S:dpu/bts.c:34-38 PPC, dsc_cnt == 2 */
#define D8895_BTS_MULT		(1UL << 10)	/* S:dpu/bts.c:20 MULTI_FACTOR */
#define D8895_BTS_DFACTOR	100UL	/* S:dpu/bts.c:17 DISP_FACTOR */

/* S:.../dpu/decon.h:208-225 (8895 enum decon_win_func), :248-268 */
#define D8895_PD_FUNC_COPY		0x1
#define D8895_PD_FUNC_USER_DEFINED	0xd
#define D8895_BND_COEF_PLNAE_ALPHA0	0x6
#define D8895_BND_COEF_ALPHA_MULT	0xa
#define D8895_BND_COEF_1_M_ALPHA_MULT	0xb
#define D8895_ALPHA_MULT_SRC_SEL_AF	2

/* S:.../dpu/decon.h:370, :385, :509, :518, :201, :204 */
#define D8895_DPATH_NOCOMP_FF0_FORMATTER0_DSIMIF0		0x001
#define D8895_DPATH_DSCC_DSCENC01_FF01_FORMATTER0_DSIMIF0	0x0b1
#define D8895_ENHANCEPATH_ENHANCE_ALL_OFF			0x0
#define D8895_SHAREPATH_DQE_USE					0x0
#define D8895_DECON_RGB						0x0
#define D8895_DECON_BGR						0x4

/* DPP scaler ratio thresholds, S:.../dpu/dpp_reg.c:20-25 */
#define D8895_SC_RATIO_MAX	((1 << 20) * 8 / 8)
#define D8895_SC_RATIO_7_8	((1 << 20) * 8 / 7)
#define D8895_SC_RATIO_6_8	((1 << 20) * 8 / 6)
#define D8895_SC_RATIO_5_8	((1 << 20) * 8 / 5)
#define D8895_SC_RATIO_4_8	((1 << 20) * 8 / 4)
#define D8895_SC_RATIO_3_8	((1 << 20) * 8 / 3)

/* S:.../dpu/dpp.h:47-56 */
#define D8895_SRC_WIDTH_MIN	16
#define D8895_SRC_WIDTH_MAX	65534
#define D8895_SRC_HEIGHT_MIN	16
#define D8895_SRC_HEIGHT_MAX	8190
#define D8895_IMG_WIDTH_MIN	16
#define D8895_IMG_WIDTH_MAX	4096
#define D8895_IMG_HEIGHT_MIN	16
#define D8895_IMG_HEIGHT_MAX	4096

/* S:drivers/video/fbdev/exynos/dpu/dpp_coef.h:16-137, phases 0-8 of each set */
static const s16 d8895_h_coef_8t[7][9][8] = {
	{	/* Ratio <= 65536 (~8:8) (:17) */
		{    0,    0,    0,  128,    0,    0,    0,    0 },
		{   -1,    2,   -6,  127,    7,   -2,    1,    0 },
		{   -1,    4,  -12,  125,   16,   -5,    1,    0 },
		{   -1,    5,  -15,  120,   25,   -8,    2,    0 },
		{   -1,    6,  -18,  114,   35,  -10,    3,   -1 },
		{   -1,    6,  -20,  107,   46,  -13,    4,   -1 },
		{   -2,    7,  -21,   99,   57,  -16,    5,   -1 },
		{   -1,    6,  -20,   89,   68,  -18,    5,   -1 },
		{   -1,    6,  -20,   79,   79,  -20,    6,   -1 },
	},
	{	/* 65536 < Ratio <= 74898 (~8:7) (:34) */
		{    3,   -8,   14,  111,   13,   -8,    3,    0 },
		{    2,   -6,    7,  112,   21,  -10,    3,   -1 },
		{    2,   -4,    1,  110,   28,  -12,    4,   -1 },
		{    1,   -2,   -3,  106,   36,  -13,    4,   -1 },
		{    1,   -1,   -7,  103,   44,  -15,    4,   -1 },
		{    1,    1,  -11,   97,   53,  -16,    4,   -1 },
		{    0,    2,  -13,   91,   61,  -16,    4,   -1 },
		{    0,    3,  -15,   85,   69,  -17,    4,   -1 },
		{    0,    3,  -16,   77,   77,  -16,    3,    0 },
	},
	{	/* 74898 < Ratio <= 87381 (~8:6) (:51) */
		{    2,  -11,   25,   96,   25,  -11,    2,    0 },
		{    2,  -10,   19,   96,   31,  -12,    2,    0 },
		{    2,   -9,   14,   94,   37,  -12,    2,    0 },
		{    2,   -8,   10,   92,   43,  -12,    1,    0 },
		{    2,   -7,    5,   90,   49,  -12,    1,    0 },
		{    2,   -5,    1,   86,   55,  -12,    0,    1 },
		{    2,   -4,   -2,   82,   61,  -11,   -1,    1 },
		{    1,   -3,   -5,   77,   67,   -9,   -1,    1 },
		{    1,   -2,   -7,   72,   72,   -7,   -2,    1 },
	},
	{	/* 87381 < Ratio <= 104857 (~8:5) (:68) */
		{   -1,   -8,   33,   80,   33,   -8,   -1,    0 },
		{   -1,   -8,   28,   80,   37,   -7,   -2,    1 },
		{    0,   -8,   24,   79,   41,   -7,   -2,    1 },
		{    0,   -8,   20,   78,   46,   -6,   -3,    1 },
		{    0,   -8,   16,   76,   50,   -4,   -3,    1 },
		{    0,   -7,   13,   74,   54,   -3,   -4,    1 },
		{    1,   -7,   10,   71,   58,   -1,   -5,    1 },
		{    1,   -6,    6,   68,   62,    1,   -5,    1 },
		{    1,   -6,    4,   65,   65,    4,   -6,    1 },
	},
	{	/* 104857 < Ratio <= 131072 (~8:4) (:85) */
		{   -3,    0,   35,   64,   35,    0,   -3,    0 },
		{   -3,   -1,   32,   64,   38,    1,   -3,    0 },
		{   -2,   -2,   29,   63,   41,    2,   -3,    0 },
		{   -2,   -3,   27,   63,   43,    4,   -4,    0 },
		{   -2,   -3,   24,   61,   46,    6,   -4,    0 },
		{   -2,   -3,   21,   60,   49,    7,   -4,    0 },
		{   -1,   -4,   19,   59,   51,    9,   -4,   -1 },
		{   -1,   -4,   16,   57,   53,   12,   -4,   -1 },
		{   -1,   -4,   14,   55,   55,   14,   -4,   -1 },
	},
	{	/* 131072 < Ratio <= 174762 (~8:3) (:102) */
		{   -1,    8,   33,   48,   33,    8,   -1,    0 },
		{   -1,    7,   31,   49,   35,    9,   -1,   -1 },
		{   -1,    6,   30,   49,   36,   10,   -1,   -1 },
		{   -1,    5,   28,   48,   38,   12,   -1,   -1 },
		{   -1,    4,   26,   48,   39,   13,    0,   -1 },
		{   -1,    3,   24,   47,   41,   15,    0,   -1 },
		{   -1,    2,   23,   47,   42,   16,    0,   -1 },
		{   -1,    2,   21,   45,   43,   18,    1,   -1 },
		{   -1,    1,   19,   45,   45,   19,    1,   -1 },
	},
	{	/* 174762 < Ratio <= 262144 (~8:2) (:119) */
		{    2,   13,   30,   38,   30,   13,    2,    0 },
		{    2,   12,   29,   38,   30,   14,    3,    0 },
		{    2,   11,   28,   38,   31,   15,    3,    0 },
		{    2,   10,   26,   38,   32,   16,    4,    0 },
		{    1,   10,   26,   37,   33,   17,    4,    0 },
		{    1,    9,   24,   37,   34,   18,    5,    0 },
		{    1,    8,   24,   37,   34,   19,    5,    0 },
		{    1,    7,   22,   36,   35,   20,    6,    1 },
		{    1,    6,   21,   36,   36,   21,    6,    1 },
	},
};

/* S:drivers/video/fbdev/exynos/dpu/dpp_coef.h:140-261, phases 0-8 of each set */
static const s16 d8895_v_coef_4t[7][9][4] = {
	{	/* Ratio <= 65536 (~8:8) (:141) */
		{    0,  128,    0,    0 },
		{   -4,  127,    5,    0 },
		{   -6,  124,   11,   -1 },
		{   -8,  118,   19,   -1 },
		{   -8,  111,   27,   -2 },
		{   -8,  102,   37,   -3 },
		{   -8,   92,   48,   -4 },
		{   -7,   81,   59,   -5 },
		{   -6,   70,   70,   -6 },
	},
	{	/* 65536 < Ratio <= 74898 (~8:7) (:158) */
		{    8,  112,    8,    0 },
		{    4,  111,   14,   -1 },
		{    1,  109,   20,   -2 },
		{   -2,  105,   27,   -2 },
		{   -3,  100,   34,   -3 },
		{   -5,   93,   43,   -3 },
		{   -5,   86,   51,   -4 },
		{   -5,   77,   60,   -4 },
		{   -5,   69,   69,   -5 },
	},
	{	/* 74898 < Ratio <= 87381 (~8:6) (:175) */
		{   16,   96,   16,    0 },
		{   12,   97,   21,   -2 },
		{    8,   96,   26,   -2 },
		{    5,   93,   32,   -2 },
		{    2,   89,   39,   -2 },
		{    0,   84,   46,   -2 },
		{   -1,   79,   53,   -3 },
		{   -2,   73,   59,   -2 },
		{   -2,   66,   66,   -2 },
	},
	{	/* 87381 < Ratio <= 104857 (~8:5) (:192) */
		{   22,   84,   22,    0 },
		{   18,   85,   26,   -1 },
		{   14,   84,   31,   -1 },
		{   11,   82,   36,   -1 },
		{    8,   79,   42,   -1 },
		{    6,   76,   47,   -1 },
		{    4,   72,   52,    0 },
		{    2,   68,   58,    0 },
		{    1,   63,   63,    1 },
	},
	{	/* 104857 < Ratio <= 131072 (~8:4) (:209) */
		{   26,   76,   26,    0 },
		{   22,   76,   30,    0 },
		{   19,   75,   34,    0 },
		{   16,   73,   38,    1 },
		{   13,   71,   43,    1 },
		{   10,   69,   47,    2 },
		{    8,   66,   51,    3 },
		{    6,   63,   55,    4 },
		{    5,   59,   59,    5 },
	},
	{	/* 131072 < Ratio <= 174762 (~8:3) (:226) */
		{   29,   70,   29,    0 },
		{   26,   68,   32,    2 },
		{   23,   67,   36,    2 },
		{   20,   66,   39,    3 },
		{   17,   65,   43,    3 },
		{   15,   63,   46,    4 },
		{   12,   61,   50,    5 },
		{   10,   58,   53,    7 },
		{    8,   56,   56,    8 },
	},
	{	/* 174762 < Ratio <= 262144 (~8:2) (:243) */
		{   32,   64,   32,    0 },
		{   28,   63,   34,    3 },
		{   25,   62,   37,    4 },
		{   22,   62,   40,    4 },
		{   19,   61,   43,    5 },
		{   17,   59,   46,    6 },
		{   15,   58,   48,    7 },
		{   13,   55,   51,    9 },
		{   11,   53,   53,   11 },
	},
};

/*
 * Channel table: stock enum decon_idma_type order (S:.../dpu/decon.h:309-318),
 * DECON channel code of dpu_dma_type_to_channel() (S:.../dpu/helper.c:598-628).
 */
static const struct {
	const char *name;
	u8 chmap;
	u8 yuv:1;	/* YUV input and CSC: VG and VGF (dpp_drv.c:444-449, dpp_reg.c:939-940) */
	u8 scale:1;	/* scaler and AFBC decoder: VGF only (dpp_drv.c:451-462) */
} d8895_ch[D8895_NR_CH] = {
	{ "g0", 5, 0, 0 }, { "g1", 3, 0, 0 }, { "vg0", 0, 1, 0 },
	{ "vg1", 4, 1, 0 }, { "vgf0", 1, 1, 1 }, { "vgf1", 2, 1, 1 },
};

/* "sysreg": SYSREG_DPU0 PCLK, on while the DECON is (TE_SEL, DLMUX, DRCG_EN) */
static const char * const d8895_clk_names[] = { "aclk", "dpp_g0", "dma_sfr", "dma_ch1", "sysreg" };

/* DRM fourcc -> IDMA_IMG_FORMAT code (S:.../dpu/regs-dpp.h:107-117), alpha_sel, YUV */
struct d8895_fmt {
	u32 fourcc;
	u8 idma;
	u8 alpha;	/* DPP ALPHA_SEL: per-pixel alpha */
	u8 yuv;		/* DPP IMG_FORMAT: YUV, two planes */
};

/*
 * dpp_reg_set_format() (S:.../dpu/dpp_reg.c:1266-1344). The IDMA format
 * codes name the components from bit 31 down, as DRM does: measured on
 * SM-N950N (image 0050), DRM XBGR8888 (bytes R,G,B,X) scanned out with code
 * RGBX8888 showed the X byte as red (black -> red, purple -> yellow), so
 * each DRM format takes the code of the same name. Stock passes its own
 * DECON format straight to the same-named code (dpp_reg.c:1279-1302); the
 * HAL-to-DECON step is in the composer. The formats the vendor DPP accepts (dpp_check_format(),
 * dpp_drv.c:465-485) minus RGBA_5551 and the 10-bit NV12N, which the stock
 * composer never scans out on this panel; NV12/NV12M and NV21/NV21M are
 * the same DRM formats. The same MSB-first naming holds for the two-plane
 * YUV codes: DRM NV12 (Cb in the low byte of each chroma pair) is
 * YVU420_2P, which is what stock scanned an NV12 buffer with (DECON NV12 ->
 * dpu_translate_fmt_to_dpp() NV21, S:.../dpu/helper.c:84-85 -> YVU420_2P,
 * S:.../dpu/dpp_reg.c:1320-1324), and DRM NV21 is YUV420_2P (:1314-1318).
 * The first D8895_NR_RGB entries are the G formats.
 */
static const struct d8895_fmt d8895_formats[] = {
	{ DRM_FORMAT_ARGB8888, D8895_IDMA_IMG_FORMAT_ARGB8888, 1, 0 },
	{ DRM_FORMAT_ABGR8888, D8895_IDMA_IMG_FORMAT_ABGR8888, 1, 0 },
	{ DRM_FORMAT_RGBA8888, D8895_IDMA_IMG_FORMAT_RGBA8888, 1, 0 },
	{ DRM_FORMAT_BGRA8888, D8895_IDMA_IMG_FORMAT_BGRA8888, 1, 0 },
	{ DRM_FORMAT_XRGB8888, D8895_IDMA_IMG_FORMAT_XRGB8888, 0, 0 },
	{ DRM_FORMAT_XBGR8888, D8895_IDMA_IMG_FORMAT_XBGR8888, 0, 0 },
	{ DRM_FORMAT_RGBX8888, D8895_IDMA_IMG_FORMAT_RGBX8888, 0, 0 },
	{ DRM_FORMAT_BGRX8888, D8895_IDMA_IMG_FORMAT_BGRX8888, 0, 0 },
	{ DRM_FORMAT_RGB565, D8895_IDMA_IMG_FORMAT_RGB565, 0, 0 },
	{ DRM_FORMAT_NV12, D8895_IDMA_IMG_FORMAT_YVU420_2P, 0, 1 },
	{ DRM_FORMAT_NV21, D8895_IDMA_IMG_FORMAT_YUV420_2P, 0, 1 },
};
#define D8895_NR_RGB		9

/* S:.../dpu/decon.h:568-571 enum dpp_flip; :576-584 CSC standard/range */
#define D8895_DPP_FLIP_X	1
#define D8895_DPP_FLIP_Y	2
#define D8895_CSC_BT_601	0
#define D8895_CSC_BT_709	1
#define D8895_CSC_BT_2020	2
#define D8895_CSC_RANGE_LIMITED	0
#define D8895_CSC_RANGE_FULL	1
/* S:.../dpu/dpp.h:192-196 */
#define D8895_CSC_COEF_HARDWIRED	0
#define D8895_CSC_COEF_CUSTOMIZED	1

/* 3x3 CSC for BT.2020, limited and full range, 3.9 format (S:.../dpu/dpp_coef.h:263-277) */
static const u16 d8895_csc_bt2020[2][3][3] = {
	{ { 0x254, 0x000, 0x36F }, { 0x254, 0xF9E, 0xEAC }, { 0x254, 0x461, 0x000 } },
	{ { 0x200, 0x000, 0x2F3 }, { 0x200, 0xFAC, 0xEDB }, { 0x200, 0x3C3, 0x000 } },
};

struct d8895_plane {
	struct drm_plane base;
	unsigned int ch;
};

/*
 * Plane state: the DECON window a plane is blended in. The stock composer
 * names the window of every layer itself (WIN_CONFIG slot = window); an
 * in-kernel client sets @win, plain DRM userspace gets a window derived from
 * the normalized zpos (L6A REQ-L5-3).
 */
struct d8895_plane_state {
	struct drm_plane_state base;
	int win;		/* requested window 0..5, or -1: derive from zpos */
	bool afbc;		/* AFBC stream (VGF only), set by an in-kernel client */
	int hw_win;		/* window chosen by atomic_check, -1 when not visible */
};

static inline struct d8895_plane_state *to_d8895_pstate(struct drm_plane_state *st)
{
	return container_of(st, struct d8895_plane_state, base);
}

struct decon8895_ctx {
	struct device *dev;
	struct drm_device *drm;
	void *dma_priv;
	struct exynos_drm_crtc *crtc;
	struct d8895_plane planes[D8895_NR_CH];
	unsigned int nr_planes;

	void __iomem *regs;
	void __iomem *dpp;
	void __iomem *dma;
	struct regmap *sysreg;
	struct clk_bulk_data clks[ARRAY_SIZE(d8895_clk_names)];
	struct resource fb_res;		/* boot loader framebuffer (memory-region) */

	int irq_fs, irq_fd, irq_extra;
	int irq_dma[D8895_NR_CH];
	int irq_dpp[D8895_NR_CH];

	bool handover;		/* S-Boot left the DECON running */
	bool rpm_held;		/* runtime-PM reference of an enabled CRTC */
	bool needs_init;	/* run the full init before the next start */
	bool hiber_init;	/* hibernation workqueue and CAM status are ready */
	bool hiber_on;		/* DECON + DSIM powered down between frames */
	bool hiber_ulps;	/* hiber_on with the DSIM link in ULPS */
	bool hiber_timeout;	/* DECON timeout IRQ since the last commit */
	void __iomem *hiber_cam;	/* CAM running status for the enter check */
	struct mutex hiber_lock;	/* serialises a hibernation enter and exit */
	struct work_struct hiber_work;	/* the TE-queued enter check */
	struct workqueue_struct *hiber_wq;
	atomic_t hiber_trig;	/* TEs since the last frame done */
	atomic_t hiber_block;	/* enters blocked while > 0 */
	bool irqs_on;
	bool vblank_on;
	bool first_frame_done;
	u32 ch_on;		/* channels initialised since the last enable */
	u32 ch_used;		/* channels of the frame last started */
	spinlock_t vblank_lock;

	/* HW_SW_TRIG_CONTROL is changed from process and IRQ context */
	spinlock_t trig_lock;

	/* one-shot trigger (L6A REQ-L5-8): 1 = armed by a flush, 2 = its frame started */
	bool oneshot;
	atomic_t oneshot_state;

	/* vsync callback of an in-kernel client (L6A REQ-L5-5) */
	spinlock_t vsync_lock;
	void (*vsync_cb)(void *data, ktime_t ts);
	void *vsync_data;
	void (*fs_cb)(void *data);	/* frame start of a flushed frame, vsync_lock */
	void *fs_data;

	/* colour windows of an in-kernel client (L6A REQ-L5-4) */
	spinlock_t cw_lock;
	u8 cw_on;
	struct exynos8895_color_win cw[D8895_NR_WIN];

	/* DSU mode state and staged update (S:.../dpu/decon.h:768, 1251) */
	spinlock_t dsu_lock;
	struct dsu_info dsu;
	struct dsu_info pending_dsu;

	/*
	 * Bus votes (spec L2b REQ-2B-L5-1, S:drivers/video/fbdev/exynos/dpu/
	 * bts.c): the DISP floor of every frame plus the MIF/INT floors the
	 * BTS core derives from the frame bandwidth. DECON0 only
	 * (BTS_BW_DECON0); channel N is BTS_DPP N (S:.../dpu/decon.h:310
	 * IDMA_G0 = 0, same order as d8895_ch[]). The other DECONs do not
	 * exist in this tree, so the cross-DECON sums of stock
	 * (dpu_bts_sum_all_decon_bw(), dpu_bts_share_bw_info()) are no-ops.
	 */
	struct exynos_bus_qos_request disp_req;
	bool disp_qos_added;
	u32 bts_xres, bts_yres;		/* panel size of the last hw_init */
	u32 bts_resol_clk;		/* kHz (S:dpu/bts.c:410-421) */
	u32 bts_total_bw;		/* KB/s of the last computed frame */
	u32 bts_peak;
	u32 bts_max_disp_freq;		/* kHz */
	u32 bts_prev_total_bw;
	u32 bts_prev_max_disp_freq;
	u32 bts_bw[BTS_DPP_MAX];	/* per-DPP KB/s */

	/*
	 * drm_panic scanout buffer: XRGB8888 of the mode size, allocated at
	 * atomic_enable; the pointers change under drm_panic_lock().
	 */
	void *panic_vaddr;
	dma_addr_t panic_dma;
	size_t panic_size;
	u32 panic_w, panic_h;

	/* counters (heartbeat / debugfs) */
	atomic_t fd_cnt, fs_cnt, to_cnt, rc_cnt, er_cnt, ude_cnt, dpe_cnt, te_cnt;
};

static inline struct d8895_plane *to_d8895_plane(struct drm_plane *p)
{
	return container_of(p, struct d8895_plane, base);
}

static inline u32 d_read(struct decon8895_ctx *ctx, u32 reg)
{
	return readl(ctx->regs + reg);
}

static inline void d_write(struct decon8895_ctx *ctx, u32 reg, u32 val)
{
	writel(val, ctx->regs + reg);
}

/* decon_write_mask() of the vendor driver */
static inline void d_mask(struct decon8895_ctx *ctx, u32 reg, u32 val, u32 mask)
{
	u32 old = readl(ctx->regs + reg);

	writel((old & ~mask) | (val & mask), ctx->regs + reg);
}

static inline void io_mask(void __iomem *addr, u32 val, u32 mask)
{
	u32 old = readl(addr);

	writel((old & ~mask) | (val & mask), addr);
}

static inline void __iomem *d_dma(struct decon8895_ctx *ctx, unsigned int ch)
{
	return ctx->dma + D8895_DMA_CH(ch);
}

static inline void __iomem *d_dpp(struct decon8895_ctx *ctx, unsigned int ch)
{
	return ctx->dpp + D8895_DPP_CH(ch);
}

/* trig_lock held */
static void __d8895_trigger(struct decon8895_ctx *ctx, bool enable)
{
	d_mask(ctx, D8895_HW_SW_TRIG_CONTROL,
	       enable ? D8895_HW_TRIG_EN : D8895_HW_TRIG_MASK_DECON,
	       D8895_HW_TRIG_EN | D8895_HW_TRIG_MASK_DECON);
}

/* decon_reg_set_trigger(), HW trigger (S:.../dpu/decon_reg.c:1006-1024) */
static void d8895_trigger(struct decon8895_ctx *ctx, bool enable)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->trig_lock, flags);
	__d8895_trigger(ctx, enable);
	spin_unlock_irqrestore(&ctx->trig_lock, flags);
}

/* decon_reg_clear_int_all() for DECON0 (decon_reg.c:2222-2250) */
static void d8895_clear_int_all(struct decon8895_ctx *ctx)
{
	u32 mask = D8895_DPU_MDNIE_DIMMING_END_INT_EN | D8895_DPU_MDNIE_DIMMING_START_INT_EN |
		   D8895_DPU_DQE_DIMMING_END_INT_EN | D8895_DPU_DQE_DIMMING_START_INT_EN |
		   D8895_DPU_FRAME_DONE_INT_EN | D8895_DPU_FRAME_START_INT_EN |
		   D8895_DPU_FIFO_SEL_FIFO0 | D8895_DPU_UNDER_FLOW_INT_EN | D8895_DPU_EXTRA_INT_EN;

	d_mask(ctx, D8895_INTERRUPT_PENDING, ~0, mask);
	d_mask(ctx, D8895_EXTRA_INTERRUPT_PENDING, ~0,
	       D8895_DPU_RESOURCE_CONFLICT_INT_EN | D8895_DPU_TIME_OUT_INT_EN |
	       D8895_DPU_ERROR_INT_EN);
}

/* decon_reg_set_int(en = 1) in MIPI command mode (decon_reg.c:2252-2289) */
static void d8895_int_enable(struct decon8895_ctx *ctx)
{
	d8895_clear_int_all(ctx);
	/* no underflow interrupt with a DSIM output (:2265-2267) */
	d_mask(ctx, D8895_INTERRUPT_ENABLE,
	       D8895_DPU_DQE_DIMMING_END_INT_EN | D8895_DPU_DQE_DIMMING_START_INT_EN |
	       D8895_DPU_FRAME_DONE_INT_EN | D8895_DPU_FRAME_START_INT_EN |
	       D8895_DPU_EXTRA_INT_EN | D8895_DPU_INT_EN,
	       D8895_INTERRUPT_ENABLE_MASK);
	d_write(ctx, D8895_EXTRA_INTERRUPT_ENABLE,
		D8895_DPU_RESOURCE_CONFLICT_INT_EN | D8895_DPU_TIME_OUT_INT_EN |
		D8895_DPU_ERROR_INT_EN);
}

/* decon_reg_set_int(en = 0) (decon_reg.c:2290-2293) */
static void d8895_int_disable(struct decon8895_ctx *ctx)
{
	d8895_clear_int_all(ctx);
	d_mask(ctx, D8895_INTERRUPT_ENABLE, 0, D8895_DPU_EXTRA_INT_EN | D8895_DPU_INT_EN);
}

/* decon_reg_wait_idle_status_timeout() (S:.../dpu/decon_reg.c:840-869) */
static int d8895_wait_idle(struct decon8895_ctx *ctx)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(ctx->regs + D8895_GLOBAL_CONTROL, val,
				 val & D8895_GLOBAL_CONTROL_IDLE_STATUS, 10,
				 D8895_IDLE_TIMEOUT_US);
	if (ret)
		dev_err(ctx->dev, "decon0 wait timeout decon idle status(%u)\n",
			!!(val & D8895_GLOBAL_CONTROL_IDLE_STATUS));
	return ret;
}

/*
 * decon_reg_stop() in HW-trigger command mode (decon_reg.c:2181-2213): mask
 * the trigger, per-frame off, global update, wait until RUN_STATUS drops.
 */
static int d8895_stop(struct decon8895_ctx *ctx)
{
	u32 val;
	int ret;

	d8895_trigger(ctx, false);
	d_mask(ctx, D8895_GLOBAL_CONTROL, 0, D8895_GLOBAL_CONTROL_DECON_EN_F);
	d_mask(ctx, D8895_SHADOW_REG_UPDATE_REQ, ~0, D8895_SHADOW_REG_UPDATE_REQ_GLOBAL);
	ret = readl_poll_timeout(ctx->regs + D8895_GLOBAL_CONTROL, val,
				 !(val & D8895_GLOBAL_CONTROL_RUN_STATUS), 10,
				 D8895_RUN_TIMEOUT_US);
	if (ret) {
		/* the vendor instant-off fallback (S:.../dpu/decon_core.c:788-790) */
		dev_err(ctx->dev, "decon stop: RUN_STATUS timeout, GC=0x%08x\n", val);
		d_mask(ctx, D8895_GLOBAL_CONTROL, 0,
		       D8895_GLOBAL_CONTROL_DECON_EN | D8895_GLOBAL_CONTROL_DECON_EN_F);
	}

	return ret;
}

/* ---- DPP / IDMA channel (S:.../dpu/dpp_reg.c) ---------------------------- */

/* dpp_reg_init() (dpp_reg.c:1522-1540) */
static void d8895_dpp_init_regs(struct decon8895_ctx *ctx, unsigned int ch)
{
	void __iomem *dma = d_dma(ctx, ch), *dpp = d_dpp(ctx, ch);

	io_mask(dma + D8895_IDMA_IRQ, 0, D8895_IDMA_ALL_IRQ_MASK);
	io_mask(dpp + D8895_DPP_IRQ, 0, D8895_DPP_ALL_IRQ_MASK);
	io_mask(dma + D8895_IDMA_IRQ, ~0, D8895_IDMA_IRQ_ENABLE);
	io_mask(dpp + D8895_DPP_IRQ, ~0, D8895_DPP_IRQ_ENABLE);

	io_mask(dma + D8895_IDMA_ENABLE, ~0, D8895_IDMA_ALL_CLOCK_GATE_EN_MASK);
	io_mask(dpp + D8895_DPP_ENABLE, ~0, D8895_DPP_ALL_CLOCK_GATE_EN_MASK);

	/* lookup table (:1410-1414) */
	writel(0x44444444, dma + D8895_IDMA_IN_QOS_LUT07_00);
	writel(0x44444444, dma + D8895_IDMA_IN_QOS_LUT15_08);
	/* dynamic clock gating (:1416-1422) */
	io_mask(dma + D8895_IDMA_DYNAMIC_GATING_EN, ~0, D8895_IDMA_DG_EN_ALL);
	io_mask(dpp + D8895_DPP_DYNAMIC_GATING_EN, ~0, D8895_DPP_DG_EN_ALL);
	/* plane alpha fixed 0xff (:1403-1408) */
	io_mask(dma + D8895_IDMA_OUT_CON, D8895_IDMA_OUT_FRAME_ALPHA(0xff),
		D8895_IDMA_OUT_FRAME_ALPHA_MASK);
	/* line counter: capture off, enable (:1261-1265) */
	io_mask(dpp + D8895_DPP_LINECNT_CON, 0, D8895_DPP_LC_CAPTURE_MASK);
	io_mask(dpp + D8895_DPP_LINECNT_CON, ~0, D8895_DPP_LC_ENABLE_MASK);
	/* recovery number (:719-726) */
	io_mask(dma + D8895_IDMA_RECOVERY_CTRL, D8895_IDMA_RECOVERY_NUM(D8895_INIT_RCV_NUM),
		D8895_IDMA_RECOVERY_NUM_MASK);
}

/* dpp_reg_init() + enable_irq (S:.../dpu/dpp_drv.c:558-563), once per enable */
static void d8895_dpp_init(struct decon8895_ctx *ctx, unsigned int ch)
{
	if (ctx->ch_on & BIT(ch))
		return;

	d8895_dpp_init_regs(ctx, ch);
	if (ctx->irq_dma[ch] > 0)
		enable_irq(ctx->irq_dma[ch]);
	if (ctx->irq_dpp[ch] > 0)
		enable_irq(ctx->irq_dpp[ch]);
	ctx->ch_on |= BIT(ch);
}

/* dpp_reg_deinit() (dpp_reg.c:1542-1562) with the IRQ lines of dpp_stop() */
static void d8895_dpp_deinit_regs(struct decon8895_ctx *ctx, unsigned int ch, bool reset)
{
	void __iomem *dma = d_dma(ctx, ch), *dpp = d_dpp(ctx, ch);
	u32 v1, v2;

	io_mask(dma + D8895_IDMA_IRQ, D8895_IDMA_ALL_IRQ_CLEAR, D8895_IDMA_ALL_IRQ_CLEAR);
	io_mask(dpp + D8895_DPP_IRQ, D8895_DPP_ALL_IRQ_CLEAR, D8895_DPP_ALL_IRQ_CLEAR);
	io_mask(dma + D8895_IDMA_IRQ, ~0, D8895_IDMA_ALL_IRQ_MASK);
	io_mask(dpp + D8895_DPP_IRQ, ~0, D8895_DPP_ALL_IRQ_MASK);

	if (!reset)
		return;
	io_mask(dma + D8895_IDMA_ENABLE, ~0, D8895_IDMA_SRSET);
	io_mask(dpp + D8895_DPP_ENABLE, ~0, D8895_DPP_SRSET);
	/* 10 us x 100000 each (:290-310, :789-810) */
	if (readl_poll_timeout(dpp + D8895_DPP_ENABLE, v1, !(v1 & D8895_DPP_SRSET), 10,
			       1000000) ||
	    readl_poll_timeout(dma + D8895_IDMA_ENABLE, v2, !(v2 & D8895_IDMA_SRSET), 10,
			       1000000))
		dev_err(ctx->dev, "%s: sw reset timeout\n", d8895_ch[ch].name);
}

static void d8895_dpp_deinit(struct decon8895_ctx *ctx, unsigned int ch, bool reset)
{
	if (ctx->ch_on & BIT(ch)) {
		if (ctx->irq_dma[ch] > 0)
			disable_irq(ctx->irq_dma[ch]);
		if (ctx->irq_dpp[ch] > 0)
			disable_irq(ctx->irq_dpp[ch]);
		ctx->ch_on &= ~BIT(ch);
	}
	d8895_dpp_deinit_regs(ctx, ch, reset);
}

static unsigned int d8895_coef_set(u32 ratio)
{
	/* dpp_reg_set_h_coef()/_v_coef() ratio classes (dpp_reg.c:1014-1031) */
	if (ratio <= D8895_SC_RATIO_MAX)
		return 0;
	if (ratio <= D8895_SC_RATIO_7_8)
		return 1;
	if (ratio <= D8895_SC_RATIO_6_8)
		return 2;
	if (ratio <= D8895_SC_RATIO_5_8)
		return 3;
	if (ratio <= D8895_SC_RATIO_4_8)
		return 4;
	if (ratio <= D8895_SC_RATIO_3_8)
		return 5;
	return 6;
}

/* dpp_reg_set_scale_ratio() (dpp_reg.c:998-1087) */
static void d8895_dpp_set_scale(struct decon8895_ctx *ctx, unsigned int ch,
				u32 h_ratio, u32 v_ratio)
{
	void __iomem *dpp = d_dpp(ctx, ch);
	unsigned int hs = d8895_coef_set(h_ratio), vs = d8895_coef_set(v_ratio);
	int i, j, k;

	io_mask(dpp + D8895_DPP_MAIN_H_RATIO, D8895_DPP_H_RATIO(h_ratio), D8895_DPP_H_RATIO_MASK);
	for (i = 0; i < 9; i++)
		for (j = 0; j < 8; j++)
			for (k = 0; k < 2; k++)
				writel((u32)(s32)d8895_h_coef_8t[hs][i][j],
				       dpp + D8895_DPP_H_COEF(i, j, k));
	io_mask(dpp + D8895_DPP_MAIN_V_RATIO, D8895_DPP_V_RATIO(v_ratio), D8895_DPP_V_RATIO_MASK);
	for (i = 0; i < 9; i++)
		for (j = 0; j < 4; j++)
			for (k = 0; k < 2; k++)
				writel((u32)(s32)d8895_v_coef_4t[vs][i][j],
				       dpp + D8895_DPP_V_COEF(i, j, k));
}

static const struct d8895_fmt *d8895_find_fmt(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(d8895_formats); i++)
		if (d8895_formats[i].fourcc == fourcc)
			return &d8895_formats[i];
	return NULL;
}

/*
 * dpp_reg_set_csc_config() for a VG/VGF channel (S:.../dpu/dpp_reg.c:
 * 937-966): type, range and coefficient mode in DPP_IN_CON (the mask is the
 * vendor's), and the BT.2020 3x3 coefficients (dpp_reg_set_csc_coef(),
 * :880-927). BT.601/709 use the hardwired matrices.
 */
static void d8895_dpp_set_csc(struct decon8895_ctx *ctx, unsigned int ch,
			      enum drm_color_encoding enc, enum drm_color_range rng)
{
	void __iomem *dpp = d_dpp(ctx, ch);
	u32 std = enc == DRM_COLOR_YCBCR_BT709 ? D8895_CSC_BT_709 :
		  enc == DRM_COLOR_YCBCR_BT2020 ? D8895_CSC_BT_2020 : D8895_CSC_BT_601;
	u32 range = rng == DRM_COLOR_YCBCR_FULL_RANGE ? D8895_CSC_RANGE_FULL :
							D8895_CSC_RANGE_LIMITED;
	u32 mode = std < D8895_CSC_BT_2020 ? D8895_CSC_COEF_HARDWIRED : D8895_CSC_COEF_CUSTOMIZED;
	const u16 (*c)[3];

	io_mask(dpp + D8895_DPP_IN_CON,
		D8895_DPP_CSC_TYPE(std) | D8895_DPP_CSC_RANGE(range) | D8895_DPP_CSC_MODE(mode),
		D8895_DPP_CSC_TYPE_MASK | D8895_DPP_CSC_RANGE_MASK | D8895_DPP_CSC_MODE_MASK);
	if (mode == D8895_CSC_COEF_HARDWIRED)
		return;

	c = d8895_csc_bt2020[range];
	io_mask(dpp + D8895_DPP_CSC_COEF0, D8895_DPP_CSC_COEF_H(c[0][1]) | D8895_DPP_CSC_COEF_L(c[0][0]),
		D8895_DPP_CSC_COEF_H_MASK | D8895_DPP_CSC_COEF_L_MASK);
	io_mask(dpp + D8895_DPP_CSC_COEF1, D8895_DPP_CSC_COEF_H(c[1][0]) | D8895_DPP_CSC_COEF_L(c[0][2]),
		D8895_DPP_CSC_COEF_H_MASK | D8895_DPP_CSC_COEF_L_MASK);
	io_mask(dpp + D8895_DPP_CSC_COEF2, D8895_DPP_CSC_COEF_H(c[1][2]) | D8895_DPP_CSC_COEF_L(c[1][1]),
		D8895_DPP_CSC_COEF_H_MASK | D8895_DPP_CSC_COEF_L_MASK);
	io_mask(dpp + D8895_DPP_CSC_COEF3, D8895_DPP_CSC_COEF_H(c[2][1]) | D8895_DPP_CSC_COEF_L(c[2][0]),
		D8895_DPP_CSC_COEF_H_MASK | D8895_DPP_CSC_COEF_L_MASK);
	io_mask(dpp + D8895_DPP_CSC_COEF4, D8895_DPP_CSC_COEF_L(c[2][2]), D8895_DPP_CSC_COEF_L_MASK);
}

/* the input of one DPP channel: from a plane state, or the panic buffer */
struct d8895_dpp_cfg {
	const struct d8895_fmt *fmt;
	dma_addr_t y, c;	/* base addresses; C = Y for AFBC, 0 for one-plane RGB */
	u32 sx, sy, sw, sh;	/* source rectangle in the buffer */
	u32 fw, fh;		/* buffer size in pixels (pitch / cpp, height) */
	u32 dw, dh;		/* window size */
	u32 flip;		/* D8895_DPP_FLIP_X | D8895_DPP_FLIP_Y */
	bool afbc;
	enum drm_color_encoding enc;
	enum drm_color_range range;
};

/*
 * dpp_reg_configure_params() (S:.../dpu/dpp_reg.c:1565-1577), in its order:
 * CSC (VG/VGF, :937-966), scale ratio and coefficients (every channel,
 * :998-1087), sizes (:1363-1377, scaled size on VGF), flip (:453-462),
 * base addresses (:1351-1361: AFBC uses the Y address for C), no block area
 * (:1379-1384; the composer's block area is a bandwidth hint only), format,
 * alpha select, RGB/YUV, AFBC, HW recovery off (:1266-1344).
 */
static void d8895_dpp_program(struct decon8895_ctx *ctx, unsigned int ch,
			      const struct d8895_dpp_cfg *cfg)
{
	void __iomem *dma = d_dma(ctx, ch), *dpp = d_dpp(ctx, ch);
	const struct d8895_fmt *f = cfg->fmt;

	if (d8895_ch[ch].yuv)
		d8895_dpp_set_csc(ctx, ch, cfg->enc, cfg->range);

	/* ratio = (src << 20) / dst (S:.../dpu/dpp_drv.c:335-336) */
	d8895_dpp_set_scale(ctx, ch, (u32)(((u64)cfg->sw << 20) / cfg->dw),
			    (u32)(((u64)cfg->sh << 20) / cfg->dh));

	writel(D8895_IDMA_SRC_OFFSET_Y(cfg->sy) | D8895_IDMA_SRC_OFFSET_X(cfg->sx),
	       dma + D8895_IDMA_SRC_OFFSET);
	writel(D8895_IDMA_SRC_HEIGHT(cfg->fh) | D8895_IDMA_SRC_WIDTH(cfg->fw),
	       dma + D8895_IDMA_SRC_SIZE);
	writel(D8895_IDMA_IMG_HEIGHT(cfg->sh) | D8895_IDMA_IMG_WIDTH(cfg->sw),
	       dma + D8895_IDMA_IMG_SIZE);
	writel(D8895_DPP_IMG_HEIGHT(cfg->sh) | D8895_DPP_IMG_WIDTH(cfg->sw),
	       dpp + D8895_DPP_IMG_SIZE);
	if (d8895_ch[ch].scale)
		writel(D8895_DPP_SCALED_IMG_HEIGHT(cfg->dh) | D8895_DPP_SCALED_IMG_WIDTH(cfg->dw),
		       dpp + D8895_DPP_SCALED_IMG_SIZE);

	io_mask(dma + D8895_IDMA_IN_CON, D8895_IDMA_IN_FLIP(cfg->flip), D8895_IDMA_IN_FLIP_MASK);

	writel(lower_32_bits(cfg->y), dma + D8895_IDMA_IN_BASE_ADDR_Y);
	writel(lower_32_bits(cfg->c), dma + D8895_IDMA_IN_BASE_ADDR_C);
	io_mask(dma + D8895_IDMA_IN_CON, 0, D8895_IDMA_BLOCK_EN);

	io_mask(dma + D8895_IDMA_IN_CON, D8895_IDMA_IMG_FORMAT(f->idma), D8895_IDMA_IMG_FORMAT_MASK);
	io_mask(dpp + D8895_DPP_IN_CON, D8895_DPP_ALPHA_SEL(f->alpha), D8895_DPP_ALPHA_SEL_MASK);
	io_mask(dpp + D8895_DPP_IN_CON, D8895_DPP_IMG_FORMAT(f->yuv), D8895_DPP_IMG_FORMAT_MASK);
	io_mask(dma + D8895_IDMA_IN_CON, cfg->afbc ? D8895_IDMA_AFBC_EN : 0, D8895_IDMA_AFBC_EN);
	io_mask(dma + D8895_IDMA_RECOVERY_CTRL, 0, D8895_IDMA_RECOVERY_EN);
}

/* d8895_dpp_program() with the input of a checked plane state */
static void d8895_dpp_configure(struct decon8895_ctx *ctx, unsigned int ch,
				struct drm_plane_state *st)
{
	struct d8895_plane_state *ps = to_d8895_pstate(st);
	struct drm_framebuffer *fb = st->fb;
	struct d8895_dpp_cfg cfg = {
		.fmt	= d8895_find_fmt(fb->format->format),
		.y	= exynos_drm_fb_dma_addr(fb, 0),
		.sx	= st->src.x1 >> 16,
		.sy	= st->src.y1 >> 16,
		.sw	= drm_rect_width(&st->src) >> 16,
		.sh	= drm_rect_height(&st->src) >> 16,
		.fw	= fb->pitches[0] / fb->format->cpp[0],
		.fh	= fb->height,
		.dw	= drm_rect_width(&st->dst),
		.dh	= drm_rect_height(&st->dst),
		.afbc	= ps->afbc,
		.enc	= st->color_encoding,
		.range	= st->color_range,
	};

	if (st->rotation & DRM_MODE_REFLECT_X)
		cfg.flip |= D8895_DPP_FLIP_X;
	if (st->rotation & DRM_MODE_REFLECT_Y)
		cfg.flip |= D8895_DPP_FLIP_Y;
	if (cfg.afbc)
		cfg.c = cfg.y;
	else if (cfg.fmt->yuv)
		cfg.c = exynos_drm_fb_dma_addr(fb, 1);

	d8895_dpp_program(ctx, ch, &cfg);
}

/* ---- DSC encoders (S:.../dpu/decon_reg.c:1391-1662, 1879-1900) ---------- */

struct d8895_dsc {
	u32 slice_width, slice_height, pic_width, pic_height, chunk_size;
	u32 scale_increment_interval, scale_decrement_interval;
	u32 nfl_bpg_offset, slice_bpg_offset, final_offset;
	u32 width_per_enc;
	u32 dual_slice_en, slice_mode_ch;
};

/*
 * dsc_calc_pps_info() for two encoders (dscc_en = 1), overlap 0
 * (decon_reg.c:1411-1563). dsc_cnt = 2 (FDT:10095).
 */
static void d8895_dsc_calc(u32 xres, u32 yres, u32 slices, u32 slice_h, struct d8895_dsc *d)
{
	const u32 rc_model_size = 0x2000, initial_xmit_delay = 0x200;
	const u32 initial_scale_value = 0x20, first_line_bpg_offset = 0x0c;
	const u32 initial_offset = 0x1800;
	u32 num_extra_mux_bits = 246;
	u32 width_eff = xres >> 1;	/* dscc_en */
	u32 slice_bits, groups_per_line, groups_total, final_scale, flag;
	u32 comp_byte, comp_pixel;

	/* dsc_get_dual_slice_mode()/_slice_mode_change() for dsc_cnt 2 (:1302-1330) */
	d->dual_slice_en = slices == 4;
	d->slice_mode_ch = slices == 2;

	d->pic_width = width_eff;
	d->slice_width = d->dual_slice_en ? width_eff >> 1 : width_eff;
	d->pic_height = yres;
	d->slice_height = slice_h;
	d->chunk_size = d->slice_width;
	slice_bits = 8 * d->chunk_size * d->slice_height;
	while ((slice_bits - num_extra_mux_bits) % 48)
		num_extra_mux_bits--;
	groups_per_line = (d->slice_width + 2) / 3;
	groups_total = groups_per_line * d->slice_height;
	d->final_offset = rc_model_size - ((initial_xmit_delay * (8 << 4) + 8) >> 4) +
			  num_extra_mux_bits;
	final_scale = 8 * rc_model_size / (rc_model_size - d->final_offset);
	flag = (first_line_bpg_offset * 2048) % (d->slice_height - 1);
	d->nfl_bpg_offset = (first_line_bpg_offset * 2048) / (d->slice_height - 1) + !!flag;
	flag = 2048 * (rc_model_size - initial_offset + num_extra_mux_bits) % groups_total;
	d->slice_bpg_offset = 2048 * (rc_model_size - initial_offset + num_extra_mux_bits) /
			      groups_total + !!flag;
	d->scale_increment_interval = (2048 * d->final_offset) /
		((final_scale - 9) * (d->nfl_bpg_offset + d->slice_bpg_offset));
	d->scale_decrement_interval = groups_per_line / (initial_scale_value - 8);

	/* compressed width per encoder in pixels (:1497-1540) */
	comp_byte = d->slice_width * 3 / 3;
	comp_pixel = comp_byte / 3;
	if (comp_byte % 3 == 0)
		d->width_per_enc = comp_pixel + (comp_pixel % 2);
	else
		d->width_per_enc = comp_pixel + 1 + ((comp_pixel + 1) % 2);
	if (d->dual_slice_en)
		d->width_per_enc *= 2;
}

/* dsc_reg_config_control() + dsc_reg_set_pps() for one encoder */
static void d8895_dsc_program(struct decon8895_ctx *ctx, u32 base, const struct d8895_dsc *d)
{
	/* control (:1391-1401): DCG all, byte swap, flatness 2, CG, dual slice, no bypass, SMC */
	d_mask(ctx, base + D8895_DSC_CONTROL0, D8895_DSC_DCG_EN_ALL_MASK, D8895_DSC_DCG_EN_ALL_MASK);
	d_mask(ctx, base + D8895_DSC_CONTROL0, D8895_DSC_SWAP(0x0, 0x1, 0x0), D8895_DSC_SWAP_MASK);
	d_mask(ctx, base + D8895_DSC_CONTROL0, D8895_DSC_FLATNESS_DET_TH_F(0x2),
	       D8895_DSC_FLATNESS_DET_TH_MASK);
	d_mask(ctx, base + D8895_DSC_CONTROL0, D8895_DSC_CG_EN_F(1), D8895_DSC_CG_EN_MASK);
	d_mask(ctx, base + D8895_DSC_CONTROL0, D8895_DSC_DUAL_SLICE_EN_F(d->dual_slice_en),
	       D8895_DSC_DUAL_SLICE_EN_MASK);
	d_mask(ctx, base + D8895_DSC_CONTROL0, D8895_DSC_BYPASS_F(0), D8895_DSC_BYPASS_MASK);
	d_mask(ctx, base + D8895_DSC_CONTROL0, D8895_DSC_SLICE_MODE_CH_F(d->slice_mode_ch),
	       D8895_DSC_SLICE_MODE_CH_MASK);

	/* PPS (:1615-1662); comp_cfg 0x30, bpp 8 << 4 (:1543-1544) */
	d_mask(ctx, base + D8895_DSC_PPS04_07, D8895_PPS04_COMP_CFG(0x30), D8895_PPS04_COMP_CFG_MASK);
	d_mask(ctx, base + D8895_DSC_PPS04_07, D8895_PPS05_BPP(8 << 4), D8895_PPS05_BPP_MASK);
	d_mask(ctx, base + D8895_DSC_PPS04_07, D8895_PPS06_07_PIC_HEIGHT(d->pic_height),
	       D8895_PPS06_07_PIC_HEIGHT_MASK);
	d_mask(ctx, base + D8895_DSC_PPS08_11, D8895_PPS08_09_PIC_WIDHT(d->pic_width),
	       D8895_PPS08_09_PIC_WIDHT_MASK);
	d_mask(ctx, base + D8895_DSC_PPS08_11, D8895_PPS10_11_SLICE_HEIGHT(d->slice_height),
	       D8895_PPS10_11_SLICE_HEIGHT_MASK);
	d_mask(ctx, base + D8895_DSC_PPS12_15, D8895_PPS12_13_SLICE_WIDTH(d->slice_width),
	       D8895_PPS12_13_SLICE_WIDTH_MASK);
	d_mask(ctx, base + D8895_DSC_PPS12_15, D8895_PPS14_15_CHUNK_SIZE(d->chunk_size),
	       D8895_PPS14_15_CHUNK_SIZE_MASK);
	d_mask(ctx, base + D8895_DSC_PPS16_19, D8895_PPS16_17_INIT_XMIT_DELAY(0x200),
	       D8895_PPS16_17_INIT_XMIT_DELAY_MASK);
	/* VESA_SCR_V4 is defined nowhere: initial_dec_delay 0x01B4 (:1629-1630) */
	d_mask(ctx, base + D8895_DSC_PPS16_19, D8895_PPS18_19_INIT_DEC_DELAY(0x01b4),
	       D8895_PPS18_19_INIT_DEC_DELAY_MASK);
	d_mask(ctx, base + D8895_DSC_PPS20_23, D8895_PPS21_INIT_SCALE_VALUE(0x20),
	       D8895_PPS21_INIT_SCALE_VALUE_MASK);
	d_mask(ctx, base + D8895_DSC_PPS20_23,
	       D8895_PPS22_23_SCALE_INC_INTERVAL(d->scale_increment_interval),
	       D8895_PPS22_23_SCALE_INC_INTERVAL_MASK);
	d_mask(ctx, base + D8895_DSC_PPS24_27,
	       D8895_PPS24_25_SCALE_DEC_INTERVAL(d->scale_decrement_interval),
	       D8895_PPS24_25_SCALE_DEC_INTERVAL_MASK);
	d_mask(ctx, base + D8895_DSC_PPS24_27, D8895_PPS27_FL_BPG_OFFSET(0x0c),
	       D8895_PPS27_FL_BPG_OFFSET_MASK);
	d_mask(ctx, base + D8895_DSC_PPS28_31, D8895_PPS28_29_NFL_BPG_OFFSET(d->nfl_bpg_offset),
	       D8895_PPS28_29_NFL_BPG_OFFSET_MASK);
	d_mask(ctx, base + D8895_DSC_PPS28_31, D8895_PPS30_31_SLICE_BPG_OFFSET(d->slice_bpg_offset),
	       D8895_PPS30_31_SLICE_BPG_OFFSET_MASK);
	d_mask(ctx, base + D8895_DSC_PPS32_35, D8895_PPS32_33_INIT_OFFSET(0x1800),
	       D8895_PPS32_33_INIT_OFFSET_MASK);
	d_mask(ctx, base + D8895_DSC_PPS32_35, D8895_PPS34_35_FINAL_OFFSET(d->final_offset),
	       D8895_PPS34_35_FINAL_OFFSET_MASK);
	/* min_qp0 = 0, max_qp0 = 4, bpg_off0 = 2 (:1652-1654, rc 0x0102 :1432) */
	d_mask(ctx, base + D8895_DSC_PPS56_59, D8895_PPS58_59_RC_RANGE_PARAM(0x0102),
	       D8895_PPS58_59_RC_RANGE_PARAM_MASK);
	/* PPS79-87 (:1655-1660) */
	d_write(ctx, base + 0x006c, 0x1ab62af6);
	d_write(ctx, base + 0x0070, 0x2b342b74);
	d_write(ctx, base + 0x0074, 0x3b746bf4);
}

/*
 * decon_reg_config_data_path_size() for DSIM_IF0 (decon_reg.c:596-645)
 * with width = width_per_enc when dual_dsc is true, or xres when false.
 */
static void d8895_data_path_size(struct decon8895_ctx *ctx, u32 width, u32 height,
				 bool dual_dsc)
{
	u32 fmt_w = dual_dsc ? width * 2 : width;

	/* splitter (:252-270): width * 2, split index = width, no overlap */
	d_write(ctx, D8895_SPLITTER_SIZE_CONTROL_0,
		D8895_SPLITTER_HEIGHT_F(height) | D8895_SPLITTER_WIDTH_F(width * 2));
	d_write(ctx, D8895_SPLITTER_SIZE_CONTROL_1, height * width);
	d_write(ctx, D8895_SPLITTER_SPLIT_IDX_CONTROL, width);
	d_write(ctx, D8895_SPLITTER_OVERLAP_CONTROL, D8895_SPLITTER_OVERLAP_F(0));
	/* frame FIFO 0, and 1 when dual DSC (:297-318, :618-620) */
	d_write(ctx, D8895_FRAME_FIFO_0_SIZE_CONTROL_0,
		D8895_FRAME_FIFO_HEIGHT_F(height) | D8895_FRAME_FIFO_WIDTH_F(width));
	d_write(ctx, D8895_FRAME_FIFO_0_SIZE_CONTROL_1, height * width);
	d_mask(ctx, D8895_FRAME_FIFO_TH_CONTROL_0, D8895_FRAME_FIFO_0_TH_F(width),
	       D8895_FRAME_FIFO_0_TH_MASK);
	if (dual_dsc) {
		d_write(ctx, D8895_FRAME_FIFO_1_SIZE_CONTROL_0,
			D8895_FRAME_FIFO_HEIGHT_F(height) | D8895_FRAME_FIFO_WIDTH_F(width));
		d_write(ctx, D8895_FRAME_FIFO_1_SIZE_CONTROL_1, height * width);
		d_mask(ctx, D8895_FRAME_FIFO_TH_CONTROL_0, D8895_FRAME_FIFO_1_TH_F(width),
		       D8895_FRAME_FIFO_1_TH_MASK);
	}
	/* formatter 0 (:403-420, :623-628) */
	d_write(ctx, D8895_FORMATTER0_SIZE_CONTROL_0,
		D8895_FORMATTER_HEIGHT_F(height) | D8895_FORMATTER_WIDTH_F(fmt_w));
	d_write(ctx, D8895_FORMATTER0_SIZE_CONTROL_1, height * fmt_w);
}

/*
 * decon_set_dsu_update() for DECON0, single DSI, dsc_cnt = 2
 * (S:.../dpu/decon_reg.c:2653-2697; dsc_reg_init() :1879-1900).
 */
static void d8895_set_dsu_update(struct decon8895_ctx *ctx, const struct dsu_info *dsu)
{
	const struct dsu_info_dt *dt = exynos8895_panel_get_dsu_dt();
	const struct dsu_res_info *res;
	u32 width, height, rgb, d_path;
	unsigned long flags = 0;
	struct d8895_dsc d;
	int idx, i;

	if (!dt || !dt->dsu_en)
		return;
	idx = dsu->mode - DSU_MODE_1;
	if (idx < 0 || idx >= dt->dsu_number)
		return;
	res = &dt->res_info[idx];
	width = res->width;
	height = res->height;

	/* decon_reg_set_blender_bg_image_size() (:439-455, :2665) */
	d_mask(ctx, D8895_BLENDER_BG_IMAGE_SIZE_0,
	       D8895_BLENDER_BG_HEIGHT_F(height) | D8895_BLENDER_BG_WIDTH_F(width),
	       D8895_BLENDER_BG_HEIGHT_MASK | D8895_BLENDER_BG_WIDTH_MASK);
	d_write(ctx, D8895_BLENDER_BG_IMAGE_SIZE_1, height * width);
	/* decon_reg_set_dispif_porch() is DECON2-only (:50-66, :2666) */

	if (res->dsc_en) {
		rgb = D8895_DECON_RGB;
		d_path = D8895_DPATH_DSCC_DSCENC01_FF01_FORMATTER0_DSIMIF0;
		d_mask(ctx, D8895_DATA_PATH_CONTROL_2,
		       D8895_ENHANCE_LOGIC_PATH_F(D8895_ENHANCEPATH_ENHANCE_ALL_OFF) |
		       D8895_COMP_LINKIF_WB_PATH_F(d_path),
		       D8895_ENHANCE_LOGIC_PATH_MASK | D8895_COMP_LINKIF_WB_PATH_MASK);
		d8895_dsc_calc(width, height, 2, res->dsc_height, &d);
		for (i = 0; i < 2; i++)
			d8895_dsc_program(ctx, i ? D8895_DSC1_OFFSET : D8895_DSC0_OFFSET, &d);
		d8895_data_path_size(ctx, d.width_per_enc, height, true);
	} else {
		rgb = D8895_DECON_BGR;
		d_path = D8895_DPATH_NOCOMP_FF0_FORMATTER0_DSIMIF0;
		d_mask(ctx, D8895_DATA_PATH_CONTROL_2,
		       D8895_ENHANCE_LOGIC_PATH_F(D8895_ENHANCEPATH_ENHANCE_ALL_OFF) |
		       D8895_COMP_LINKIF_WB_PATH_F(d_path),
		       D8895_ENHANCE_LOGIC_PATH_MASK | D8895_COMP_LINKIF_WB_PATH_MASK);
		d8895_data_path_size(ctx, width, height, false);
		/* decon_reg_set_dispif_size() is DECON2-only (:69-81, :2691) */
	}

	/* decon_reg_set_rgb_order() (:382-395, :2694) */
	d_mask(ctx, D8895_FORMATTER_CONTROL, D8895_FORMATTER_OUT_RGB_ORDER_F(rgb),
	       D8895_FORMATTER_OUT_RGB_ORDER_MASK);

	/* BTS lcd_w, lcd_h, resol_clk for the new resolution (S:dpu/bts.c:41, 128-130, 256-258) */
	ctx->bts_xres = width;
	ctx->bts_yres = height;
	ctx->bts_resol_clk = (u32)((u64)width * (u64)height * D8895_BTS_LCD_REFRESH *
				   11 / 10 / 1000 + 1);

	if (IS_ENABLED(CONFIG_DRM_PANIC) && ctx->panic_vaddr) {
		drm_panic_lock(ctx->drm, flags);
		ctx->panic_w = width;
		ctx->panic_h = height;
		drm_panic_unlock(ctx->drm, flags);
	}
}

/*
 * decon_reg_init() full path for DECON0, MIPI command mode, HW trigger,
 * single DSI, dual DSC (decon_reg.c:2047-2094). Uses the active DSU mode
 * dimensions and DSC state when DSU is enabled, matching stock's shared
 * decon->lcd_info across hibernation and power-off/on
 * (S:drivers/video/fbdev/exynos/dpu/decon_reg.c:1902-1965, 2047-2094).
 */
static void d8895_hw_init(struct decon8895_ctx *ctx, const struct drm_display_mode *m,
			  const struct drm_dsc_config *dsc)
{
	const struct dsu_info_dt *dt = exynos8895_panel_get_dsu_dt();
	u32 xres = m->hdisplay, yres = m->vdisplay;
	u32 slice_h = dsc->slice_height, val;
	bool dsc_en = true;
	struct d8895_dsc d;
	int idx, i;

	if (dt && dt->dsu_en) {
		idx = ctx->dsu.mode - DSU_MODE_1;
		if (idx >= 0 && idx < dt->dsu_number) {
			xres = dt->res_info[idx].width;
			yres = dt->res_info[idx].height;
			dsc_en = dt->res_info[idx].dsc_en != 0;
			slice_h = dt->res_info[idx].dsc_height;
		}
	}

	/*
	 * Resolution clock for the bus votes (S:dpu/bts.c:410-421
	 * dpu_bts_init(), non-DP branch: xres * yres * 63 * 11 / 10 / 1000 + 1).
	 */
	ctx->bts_xres = xres;
	ctx->bts_yres = yres;
	ctx->bts_resol_clk = (u32)((u64)xres * (u64)yres * D8895_BTS_LCD_REFRESH *
				   11 / 10 / 1000 + 1);

	/* TE from GPIO for TE0..2 (:678-688) */
	regmap_write(ctx->sysreg, D8895_SS(D8895_DISP_DPU_TE_SEL),
		     D8895_SEL_TE_PATH(0, D8895_TE_FROM_GPIO) |
		     D8895_SEL_TE_PATH(1, D8895_TE_FROM_GPIO) |
		     D8895_SEL_TE_PATH(2, D8895_TE_FROM_GPIO));
	/* clock gating (:152-165), SRAM share 8K for DECON0 (:216-250) */
	d_mask(ctx, D8895_CLOCK_CONTROL_0, ~0, D8895_CLOCK_CONTROL_0_F_MASK);
	d_write(ctx, D8895_SRAM_SHARE_ENABLE,
		D8895_FLOATING_SRAM_SHARE_ENABLE_F | D8895_FF0_SRAM_SHARE_ENABLE_F);
	/* I80 (command mode) (:126-136) */
	d_mask(ctx, D8895_GLOBAL_CONTROL, D8895_GLOBAL_CONTROL_OPERATION_MODE_I80IF_F,
	       D8895_GLOBAL_CONTROL_OPERATION_MODE_F);
	/* blender background size (:439-455) */
	d_mask(ctx, D8895_BLENDER_BG_IMAGE_SIZE_0,
	       D8895_BLENDER_BG_HEIGHT_F(yres) | D8895_BLENDER_BG_WIDTH_F(xres),
	       D8895_BLENDER_BG_HEIGHT_MASK | D8895_BLENDER_BG_WIDTH_MASK);
	d_write(ctx, D8895_BLENDER_BG_IMAGE_SIZE_1, yres * xres);

	/* decon_reg_configure_lcd() (:1902-1965): RGB with DSC, BGR without (:1921-1925) */
	if (dsc_en) {
		d_mask(ctx, D8895_FORMATTER_CONTROL,
		       D8895_FORMATTER_OUT_RGB_ORDER_F(D8895_DECON_RGB),
		       D8895_FORMATTER_OUT_RGB_ORDER_MASK);
		d_mask(ctx, D8895_DATA_PATH_CONTROL_2,
		       D8895_ENHANCE_LOGIC_PATH_F(D8895_ENHANCEPATH_ENHANCE_ALL_OFF) |
		       D8895_COMP_LINKIF_WB_PATH_F(D8895_DPATH_DSCC_DSCENC01_FF01_FORMATTER0_DSIMIF0),
		       D8895_ENHANCE_LOGIC_PATH_MASK | D8895_COMP_LINKIF_WB_PATH_MASK);
		d8895_dsc_calc(xres, yres, dsc->slice_count, slice_h, &d);
		for (i = 0; i < 2; i++)
			d8895_dsc_program(ctx, i ? D8895_DSC1_OFFSET : D8895_DSC0_OFFSET, &d);
		d8895_data_path_size(ctx, d.width_per_enc, yres, true);
	} else {
		d_mask(ctx, D8895_FORMATTER_CONTROL,
		       D8895_FORMATTER_OUT_RGB_ORDER_F(D8895_DECON_BGR),
		       D8895_FORMATTER_OUT_RGB_ORDER_MASK);
		d_mask(ctx, D8895_DATA_PATH_CONTROL_2,
		       D8895_ENHANCE_LOGIC_PATH_F(D8895_ENHANCEPATH_ENHANCE_ALL_OFF) |
		       D8895_COMP_LINKIF_WB_PATH_F(D8895_DPATH_NOCOMP_FF0_FORMATTER0_DSIMIF0),
		       D8895_ENHANCE_LOGIC_PATH_MASK | D8895_COMP_LINKIF_WB_PATH_MASK);
		d8895_data_path_size(ctx, xres, yres, false);
	}
	d_mask(ctx, D8895_DATA_PATH_CONTROL_2, D8895_APS_PATH_F(D8895_SHAREPATH_DQE_USE),
	       D8895_APS_PATH_MASK);
	d_mask(ctx, D8895_DATA_PATH_CONTROL_2, D8895_HSC_PATH_F(D8895_SHAREPATH_DQE_USE),
	       D8895_HSC_PATH_MASK);
	d_mask(ctx, D8895_GLOBAL_CONTROL, 0, D8895_GLOBAL_CONTROL_DECON_EN_F);

	/* trigger disable (:2079-2081), clear pendings (:2089) */
	d8895_trigger(ctx, false);
	d8895_clear_int_all(ctx);

	/*
	 * decon_reg_set_disp_ss_cfg() for DECON0 (:700-771): dsim_if0 -> dsim0,
	 * dsim_if1 unused -> reserved value; then TE_SEL again.
	 */
	regmap_read(ctx->sysreg, D8895_SS(D8895_DISP_DLMUX_CFG), &val);
	val &= ~(D8895_SEL_DISPIF2DSIM_MASK(0) | D8895_SEL_DSIM2DISPIF_MASK(0) |
		 D8895_SEL_DISPIF2DSIM_MASK(1) | D8895_SEL_DSIM2DISPIF_MASK(1));
	val |= D8895_SEL_DISPIF2DSIM(0, 0) | D8895_SEL_DSIM2DISPIF(0, 0);
	val |= D8895_SEL_DISPIF2DSIM(1, 1) | D8895_SEL_DSIM2DISPIF(3, 1);
	regmap_write(ctx->sysreg, D8895_SS(D8895_DISP_DLMUX_CFG), val);
	regmap_write(ctx->sysreg, D8895_SS(D8895_DISP_DPU_TE_SEL),
		     D8895_SEL_TE_PATH(0, D8895_TE_FROM_GPIO) |
		     D8895_SEL_TE_PATH(1, D8895_TE_FROM_GPIO) |
		     D8895_SEL_TE_PATH(2, D8895_TE_FROM_GPIO));
}

/* ---- window (S:.../dpu/decon_reg.c:2296-2480) ---------------------------- */

static void d8895_win_disable(struct decon8895_ctx *ctx, unsigned int win)
{
	d_mask(ctx, D8895_DATA_PATH_CONTROL_0, 0, D8895_WIN_EN_F(win));
}

/*
 * decon_reg_set_win_bnd_function() with the vendor's plane-alpha branch
 * (S:.../dpu/decon_reg.c:2407-2459; the composer always passes a plane alpha
 * of 0..0xff, S:.../dpu/decon_core.c:1330): alpha0 = plane alpha, alpha1 = 0,
 * alpha multiplier from the pixel alpha, and per blend mode:
 * none -> PD_FUNC_COPY; coverage -> ALPHA_MULT / 1 - ALPHA_MULT;
 * premultiplied -> PLANE_ALPHA0 / 1 - ALPHA_MULT (:2433-2447). A buffer
 * without per-pixel alpha at full plane alpha is copied: the same output,
 * and the register state of stage 5b.
 */
static void d8895_win_blend(struct decon8895_ctx *ctx, unsigned int win, u16 alpha,
			    u16 mode, bool pixel_alpha)
{
	u32 fg = D8895_BND_COEF_PLNAE_ALPHA0;

	d_mask(ctx, D8895_WIN_CONTROL_0(win), D8895_WIN_ALPHA1_F(0) | D8895_WIN_ALPHA0_F(alpha >> 8),
	       D8895_WIN_ALPHA1_MASK | D8895_WIN_ALPHA0_MASK);
	d_mask(ctx, D8895_WIN_CONTROL_0(win),
	       D8895_WIN_ALPHA_MULT_SRC_SEL_F(D8895_ALPHA_MULT_SRC_SEL_AF),
	       D8895_WIN_ALPHA_MULT_SRC_SEL_MASK);
	if (mode == DRM_MODE_BLEND_PIXEL_NONE || (!pixel_alpha && alpha == DRM_BLEND_ALPHA_OPAQUE)) {
		d_mask(ctx, D8895_WIN_CONTROL_0(win), D8895_WIN_FUNC_F(D8895_PD_FUNC_COPY),
		       D8895_WIN_FUNC_MASK);
		return;
	}
	if (mode == DRM_MODE_BLEND_COVERAGE)
		fg = D8895_BND_COEF_ALPHA_MULT;
	d_mask(ctx, D8895_WIN_CONTROL_0(win), D8895_WIN_FUNC_F(D8895_PD_FUNC_USER_DEFINED),
	       D8895_WIN_FUNC_MASK);
	d_mask(ctx, D8895_WIN_CONTROL_1(win),
	       D8895_WIN_FG_ALPHA_D_SEL_F(fg) |
	       D8895_WIN_BG_ALPHA_D_SEL_F(D8895_BND_COEF_1_M_ALPHA_MULT) |
	       D8895_WIN_FG_ALPHA_A_SEL_F(fg) |
	       D8895_WIN_BG_ALPHA_A_SEL_F(D8895_BND_COEF_1_M_ALPHA_MULT),
	       D8895_WIN_FG_ALPHA_D_SEL_MASK | D8895_WIN_BG_ALPHA_D_SEL_MASK |
	       D8895_WIN_FG_ALPHA_A_SEL_MASK | D8895_WIN_BG_ALPHA_A_SEL_MASK);
}

/*
 * decon_reg_set_window_control() (S:.../dpu/decon_reg.c:2461-2480): blend,
 * position (S:.../dpu/decon.h:1427-1435), start time 0, pixel count, enable,
 * colour map (decon_reg_set_winmap(), :2329-2340: on only for a colour
 * window, whose ARGB then fills it), channel map (:923-934).
 */
static void d8895_win_program(struct decon8895_ctx *ctx, unsigned int win, unsigned int chmap,
			      const struct drm_rect *dst, u16 alpha, u16 mode, bool pixel_alpha,
			      bool colormap, u32 argb)
{
	u32 x = dst->x1, y = dst->y1, w = drm_rect_width(dst), h = drm_rect_height(dst);

	d8895_win_blend(ctx, win, alpha, mode, pixel_alpha);

	d_write(ctx, D8895_WIN_START_POSITION(win),
		D8895_WIN_STRPTR_Y_F(y) | D8895_WIN_STRPTR_X_F(x));
	d_write(ctx, D8895_WIN_END_POSITION(win),
		D8895_WIN_ENDPTR_Y_F(y + h - 1) | D8895_WIN_ENDPTR_X_F(x + w - 1));
	d_write(ctx, D8895_WIN_START_TIME_CONTROL(win), 0);
	d_write(ctx, D8895_WIN_PIXEL_COUNT(win), w * h);

	d_mask(ctx, D8895_DATA_PATH_CONTROL_0, ~0, D8895_WIN_EN_F(win));
	/* decon_reg_set_win_mapcolor(): A[31:24] R[23:16] G[15:8] B[7:0] (:2305-2327) */
	d_mask(ctx, D8895_DATA_PATH_CONTROL_0, colormap ? ~0 : 0, D8895_WIN_MAPCOLOR_EN_F(win));
	d_mask(ctx, D8895_WIN_COLORMAP_0(win),
	       D8895_WIN_MAPCOLOR_A_F((argb >> 24) & 0xff) | D8895_WIN_MAPCOLOR_R_F((argb >> 16) & 0xff),
	       D8895_WIN_MAPCOLOR_A_MASK | D8895_WIN_MAPCOLOR_R_MASK);
	d_mask(ctx, D8895_WIN_COLORMAP_1(win),
	       D8895_WIN_MAPCOLOR_G_F((argb >> 8) & 0xff) | D8895_WIN_MAPCOLOR_B_F(argb & 0xff),
	       D8895_WIN_MAPCOLOR_G_MASK | D8895_WIN_MAPCOLOR_B_MASK);
	d_mask(ctx, D8895_DATA_PATH_CONTROL_1, D8895_WIN_CHMAP_F(win, chmap),
	       D8895_WIN_CHMAP_MASK(win));
}

static void d8895_win_program_plane(struct decon8895_ctx *ctx, unsigned int win, unsigned int ch,
				    struct drm_plane_state *st)
{
	const struct d8895_fmt *f = d8895_find_fmt(st->fb->format->format);

	d8895_win_program(ctx, win, d8895_ch[ch].chmap, &st->dst, st->alpha,
			  st->pixel_blend_mode, f->alpha, false, 0);
}

/* ---- drm_panic buffer ---------------------------------------------------- */

/*
 * The planes carry the composer's buffers (several overlapping windows,
 * possibly AFBC), so the panic screen is not drawn into them: it goes into
 * a dedicated XRGB8888 buffer of the mode size (XRGB8888 is the format whose
 * IDMA code was measured on hardware, see d8895_formats), which the panic
 * flush scans out on G0 alone. Sleepable; reallocated only when the size
 * changes.
 */
static void d8895_panic_alloc(struct decon8895_ctx *ctx, const struct drm_display_mode *m)
{
	struct device *dma_dev = to_dma_dev(ctx->drm);
	size_t size = (size_t)m->hdisplay * m->vdisplay * 4;
	u32 pw = ctx->dsu.right > ctx->dsu.left ?
		 (u32)(ctx->dsu.right - ctx->dsu.left) : m->hdisplay;
	u32 ph = ctx->dsu.bottom > ctx->dsu.top ?
		 (u32)(ctx->dsu.bottom - ctx->dsu.top) : m->vdisplay;
	void *vaddr = NULL, *old_vaddr;
	dma_addr_t dma = 0, old_dma;
	unsigned long flags = 0;
	size_t old_size;

	if (!IS_ENABLED(CONFIG_DRM_PANIC))
		return;

	if (ctx->panic_vaddr && ctx->panic_size == size) {
		drm_panic_lock(ctx->drm, flags);
		ctx->panic_w = pw;
		ctx->panic_h = ph;
		drm_panic_unlock(ctx->drm, flags);
		return;
	}

	if (size) {
		vaddr = dma_alloc_attrs(dma_dev, size, &dma, GFP_KERNEL, DMA_ATTR_WRITE_COMBINE);
		if (!vaddr)
			dev_warn(ctx->dev, "no panic buffer (%zu bytes)\n", size);
	}

	drm_panic_lock(ctx->drm, flags);
	old_vaddr = ctx->panic_vaddr;
	old_dma = ctx->panic_dma;
	old_size = ctx->panic_size;
	ctx->panic_vaddr = vaddr;
	ctx->panic_dma = dma;
	ctx->panic_size = vaddr ? size : 0;
	ctx->panic_w = pw;
	ctx->panic_h = ph;
	drm_panic_unlock(ctx->drm, flags);

	if (old_vaddr)
		dma_free_attrs(dma_dev, old_size, old_vaddr, old_dma, DMA_ATTR_WRITE_COMBINE);
}

static void d8895_panic_free(struct decon8895_ctx *ctx)
{
	unsigned long flags = 0;
	dma_addr_t dma;
	size_t size;
	void *vaddr;

	drm_panic_lock(ctx->drm, flags);
	vaddr = ctx->panic_vaddr;
	dma = ctx->panic_dma;
	size = ctx->panic_size;
	ctx->panic_vaddr = NULL;
	ctx->panic_size = 0;
	drm_panic_unlock(ctx->drm, flags);

	if (vaddr)
		dma_free_attrs(to_dma_dev(ctx->drm), size, vaddr, dma, DMA_ATTR_WRITE_COMBINE);
}

/* ---- panel hooks (include/drm/exynos8895_panel.h) ---------------------- */

/* the panel's DECON event hooks (include/drm/exynos8895_panel.h); one DECON0 */
static DEFINE_SPINLOCK(d8895_panel_hooks_lock);
static const struct exynos8895_panel_hooks *d8895_panel_hooks;

void exynos8895_decon_set_panel_hooks(const struct exynos8895_panel_hooks *hooks)
{
	unsigned long flags;

	spin_lock_irqsave(&d8895_panel_hooks_lock, flags);
	d8895_panel_hooks = hooks;
	spin_unlock_irqrestore(&d8895_panel_hooks_lock, flags);
}
EXPORT_SYMBOL_GPL(exynos8895_decon_set_panel_hooks);

/*
 * A power request of the S3CFB front-end to the panel (the vendor DECON's
 * DSIM/panel subdev calls, include/drm/exynos8895_panel.h); 0 without hook.
 * The hooks, once set, are never cleared.
 */
int exynos8895_decon_panel_power(enum exynos8895_panel_power req, bool active)
{
	const struct exynos8895_panel_hooks *hooks;
	unsigned long flags;

	spin_lock_irqsave(&d8895_panel_hooks_lock, flags);
	hooks = d8895_panel_hooks;
	spin_unlock_irqrestore(&d8895_panel_hooks_lock, flags);

	if (!hooks || !hooks->power)
		return 0;
	return hooks->power(hooks->data, req, active);
}

/* the panel is in HMD mode (include/drm/exynos8895_panel.h) */
static bool d8895_panel_hmd_on(void)
{
	unsigned long flags;
	bool on = false;

	spin_lock_irqsave(&d8895_panel_hooks_lock, flags);
	if (d8895_panel_hooks && d8895_panel_hooks->hmd_on)
		on = d8895_panel_hooks->hmd_on(d8895_panel_hooks->data);
	spin_unlock_irqrestore(&d8895_panel_hooks_lock, flags);

	return on;
}

/*
 * The vendor frame-done path: decon_irq_handler() stamps the frame done and
 * wakes the decon0-fsync thread (S:.../dpu/decon_dsi.c:60-68), which sends
 * V4L2_EVENT_DECON_FRAME_DONE (:1252-1272) and decon_notify() turns into
 * the panel's PANEL_IOC_EVT_FRAME_DONE (decon_core.c:3475-3481). Here the
 * panel's hook is called from the interrupt; it only counts and wakes.
 */
static void d8895_panel_frame_done(void)
{
	spin_lock(&d8895_panel_hooks_lock);
	if (d8895_panel_hooks && d8895_panel_hooks->frame_done)
		d8895_panel_hooks->frame_done(d8895_panel_hooks->data);
	spin_unlock(&d8895_panel_hooks_lock);
}

static int d8895_panel_set_dsu(const struct dsu_info *dsu)
{
	const struct exynos8895_panel_hooks *hooks;
	unsigned long flags;

	spin_lock_irqsave(&d8895_panel_hooks_lock, flags);
	hooks = d8895_panel_hooks;
	spin_unlock_irqrestore(&d8895_panel_hooks_lock, flags);

	if (!hooks || !hooks->set_dsu)
		return 0;
	return hooks->set_dsu(hooks->data, dsu);
}

/* ---- display hibernation (DESIGN 2.5; S:.../dpu/decon_dsi.c:1037-1250) --- */

#define D8895_HIBER_ON	IS_ENABLED(CONFIG_DRM_EXYNOS8895_DECON_HIBER)
#define D8895_ENTER_HIBER_CNT	3	/* decon.h:59 DECON_ENTER_HIBER_CNT */

/* the DSIM's ULPS entry points (include/drm/exynos8895_hiber.h); one DECON0 */
static DEFINE_SPINLOCK(d8895_dsim_hooks_lock);
static const struct exynos8895_dsim_hooks *d8895_dsim_hooks;

void exynos8895_decon_set_dsim_hooks(const struct exynos8895_dsim_hooks *hooks)
{
	unsigned long flags;

	spin_lock_irqsave(&d8895_dsim_hooks_lock, flags);
	d8895_dsim_hooks = hooks;
	spin_unlock_irqrestore(&d8895_dsim_hooks_lock, flags);
}
EXPORT_SYMBOL_GPL(exynos8895_decon_set_dsim_hooks);

static int d8895_dsim_set_dsu(const struct dsu_info *dsu)
{
	const struct exynos8895_dsim_hooks *hooks;
	unsigned long flags;

	spin_lock_irqsave(&d8895_dsim_hooks_lock, flags);
	hooks = d8895_dsim_hooks;
	spin_unlock_irqrestore(&d8895_dsim_hooks_lock, flags);

	if (!hooks || !hooks->set_dsu)
		return 0;
	return hooks->set_dsu(hooks->data, dsu);
}

/* the one DECON0 context, for the hibernation exports below */
static struct decon8895_ctx *d8895_hiber_ctx;

/*
 * Runtime kill switches for display hibernation, both at the stock setting:
 * stock can only turn hibernation off with a rebuild (CONFIG_DECON_HIBER,
 * S:drivers/video/fbdev/exynos/dpu/Kconfig:55-58, default n, used in
 * decon_core.c:4194-4199) and its only other gate, decon->hiber.init_status
 * (S:.../dpu/decon.h:1062), is set once at probe (decon_dsi.c:1231-1248);
 * ULPS was reachable from userspace only through the vendor ioctl
 * DSIM_IOC_ENTER_ULPS (S:.../dpu/dsim_drv.c:1456-1474). The parameters are
 * writable so a display problem can be bisected without a rebuild, as
 * decon_log_level is (module_param(decon_log_level, int, 0644),
 * S:.../dpu/decon_core.c:50).
 */
static bool d8895_hiber_enabled = true;
static bool d8895_ulps_enabled = true;

static int d8895_hiber_enable_set(const char *val, const struct kernel_param *kp)
{
	struct decon8895_ctx *ctx = READ_ONCE(d8895_hiber_ctx);
	bool off;
	int ret;

	ret = kstrtobool(val, &off);
	if (ret)
		return ret;

	/*
	 * Leave hibernation before the switch goes off: a store of 0 must not
	 * leave the pipe powered down until the next panel command brings it
	 * back. A refused exit keeps the switch on (the write fails), so the
	 * enter stays armed and a later panel command can retry the exit.
	 */
	if (off && ctx && READ_ONCE(ctx->hiber_init)) {
		ret = exynos8895_decon_hiber_block_exit();
		exynos8895_decon_hiber_unblock();
		if (ret) {
			dev_warn(ctx->dev, "hibernation exit failed: %d, still enabled\n",
				 ret);
			return ret;
		}
	}

	return param_set_bool(val, kp);
}

static const struct kernel_param_ops d8895_hiber_enable_ops = {
	.set = d8895_hiber_enable_set,
	.get = param_get_bool,
};
module_param_cb(hiber_enable, &d8895_hiber_enable_ops,
		&d8895_hiber_enabled, 0644);
MODULE_PARM_DESC(hiber_enable,
		 "DECON display hibernation; 0 leaves hibernation and stops entering it (default: Y)");

module_param_named(ulps_enable, d8895_ulps_enabled, bool, 0644);
MODULE_PARM_DESC(ulps_enable,
		 "put the DSIM link into ULPS together with display hibernation (default: Y)");

/* the bus votes are released at the hibernation enter, defined below */
static void d8895_bts_release(struct decon8895_ctx *ctx);

static void d8895_hiber_block(struct decon8895_ctx *ctx)
{
	atomic_inc(&ctx->hiber_block);
}

static bool d8895_hiber_blocked(struct decon8895_ctx *ctx)
{
	return atomic_read(&ctx->hiber_block) > 0;
}

static void d8895_hiber_unblock(struct decon8895_ctx *ctx)
{
	if (d8895_hiber_blocked(ctx))
		atomic_dec(&ctx->hiber_block);
}

/* decon_hiber_trig_reset() (decon.h:1476-1480) */
static void d8895_hiber_trig_reset(struct decon8895_ctx *ctx)
{
	atomic_set(&ctx->hiber_trig, 0);
}

/*
 * decon_hiber_enter_cond() (decon.h:1514-1529): 3 counted TEs without a frame
 * done, no blocker, the CAM PD idle (cam-stat 0x16484024, FDT:7555-7557),
 * no timeout IRQ since the last commit, the panel not in HMD mode. The DSR
 * (!dsr_on) term of stock is omitted: s->dsr_on is stored but unused in this
 * tree (L6A:909); hibernation during self-refresh is safe, the exit re-inits
 * the pipe while the panel holds its frame memory.
 */
static bool d8895_hiber_enter_cond(struct decon8895_ctx *ctx)
{
	u32 cam = ctx->hiber_cam ? readl(ctx->hiber_cam) : 0xf;

	if (d8895_hiber_blocked(ctx))
		return false;
	if (cam & 0xf)
		return false;
	if (READ_ONCE(ctx->hiber_timeout))
		return false;
	if (atomic_inc_return(&ctx->hiber_trig) <= D8895_ENTER_HIBER_CNT)
		return false;
	if (d8895_panel_hmd_on())
		return false;
	return true;
}

/*
 * decon_enter_hiber() (decon_dsi.c:1113-1199): stop the DECON, power the DPPs
 * and the DISP vote down, enter DSIM ULPS and put both runtime-PM references
 * so pd_dpu0 and pd_dpu1 can power off. The full register init is deferred to
 * the next commit (needs_init); the exit only powers the pipe back on.
 */
static int d8895_hiber_enter(struct decon8895_ctx *ctx)
{
	const struct exynos8895_dsim_hooks *hooks;
	unsigned long flags;
	int ret = 0, ch;

	if (!D8895_HIBER_ON || !READ_ONCE(ctx->hiber_init) ||
	    !READ_ONCE(d8895_hiber_enabled))
		return 0;
	mutex_lock(&ctx->hiber_lock);
	if (!ctx->rpm_held || READ_ONCE(ctx->hiber_on) || d8895_hiber_blocked(ctx))
		goto out;

	spin_lock_irqsave(&d8895_dsim_hooks_lock, flags);
	hooks = d8895_dsim_hooks;
	spin_unlock_irqrestore(&d8895_dsim_hooks_lock, flags);
	if (!hooks || (READ_ONCE(d8895_ulps_enabled) &&
		       (!hooks->enter_ulps || !hooks->exit_ulps)))
		goto out;

	d8895_hiber_block(ctx);
	if (atomic_read(&ctx->oneshot_state) != 0)
		goto unblock;	/* a frame is armed or started: try the next TE */

	d8895_hiber_trig_reset(ctx);

	d8895_int_disable(ctx);	/* decon_reg_set_int(..., 0) */
	/* the frame IRQs must not run on gated clocks */
	disable_irq(ctx->irq_fs);
	disable_irq(ctx->irq_fd);
	disable_irq(ctx->irq_extra);

	ret = d8895_stop(ctx);
	if (ret)
		dev_err(ctx->dev, "hibernation enter with the DECON not stopped\n");
	d8895_clear_int_all(ctx);

	/* cur_using_dpp = 0, decon_dpp_stop(false): every channel off */
	for (ch = 0; ch < D8895_NR_CH; ch++)
		if (ctx->ch_on & BIT(ch))
			d8895_dpp_deinit(ctx, ch, false);

	/* dpu_bts_release_bw() once the DPPs are stopped (decon_core.c:896-901) */
	d8895_bts_release(ctx);

	WRITE_ONCE(ctx->hiber_on, true);
	greatlte_l5_inc(GL5_ON, he);
	if (READ_ONCE(d8895_ulps_enabled)) {
		ret = hooks->enter_ulps(hooks->data);
		WRITE_ONCE(ctx->hiber_ulps, ret == 0);
		if (ret)
			dev_warn(ctx->dev, "DSIM ULPS enter failed: %d\n", ret);
	}

	ctx->needs_init = true;
	pm_runtime_put_sync(ctx->dev);
unblock:
	d8895_hiber_unblock(ctx);
out:
	mutex_unlock(&ctx->hiber_lock);
	return ret;
}

/*
 * decon_exit_hiber() (decon_dsi.c:1037-1111): power the DECON back on, leave
 * ULPS with a re-initialised link and re-enable the interrupts; the register
 * init runs with the next commit (needs_init). Stays hibernating on a resume
 * failure so the next commit retries.
 */
static int d8895_hiber_exit(struct decon8895_ctx *ctx)
{
	const struct exynos8895_dsim_hooks *hooks;
	unsigned long flags;
	int ret = 0;

	/* the exit runs whatever hiber_enable says: the pipe must come back */
	if (!D8895_HIBER_ON || !READ_ONCE(ctx->hiber_init))
		return 0;
	d8895_hiber_block(ctx);
	/* a queued enter runs first (decon_dsi.c:1048, flush before the lock) */
	flush_workqueue(ctx->hiber_wq);
	mutex_lock(&ctx->hiber_lock);
	if (!READ_ONCE(ctx->hiber_on))
		goto out;

	ret = pm_runtime_get_sync(ctx->dev);
	if (ret < 0) {
		dev_err(ctx->dev, "hibernation exit without the DECON: %d\n", ret);
		goto out;
	}
	ret = 0;	/* already active (1) is success too */

	spin_lock_irqsave(&d8895_dsim_hooks_lock, flags);
	hooks = d8895_dsim_hooks;
	spin_unlock_irqrestore(&d8895_dsim_hooks_lock, flags);
	/*
	 * hiber_ulps, not ulps_enable: the link is in ULPS whenever the enter
	 * put it there, also when the switch was turned off in between.
	 */
	if (hooks && hooks->exit_ulps && READ_ONCE(ctx->hiber_ulps)) {
		WRITE_ONCE(ctx->hiber_ulps, false);
		ret = hooks->exit_ulps(hooks->data);
		if (ret)
			dev_warn(ctx->dev, "DSIM ULPS exit failed: %d\n", ret);
	}

	enable_irq(ctx->irq_fs);
	enable_irq(ctx->irq_fd);
	enable_irq(ctx->irq_extra);
	d8895_int_enable(ctx);

	WRITE_ONCE(ctx->hiber_on, false);
	greatlte_l5_inc(GL5_ON, hx);
	d8895_hiber_trig_reset(ctx);
out:
	d8895_hiber_unblock(ctx);
	mutex_unlock(&ctx->hiber_lock);
	return ret;
}

static void d8895_hiber_work(struct work_struct *work)
{
	struct decon8895_ctx *ctx = container_of(work, struct decon8895_ctx, hiber_work);

	if (!D8895_HIBER_ON || !READ_ONCE(ctx->hiber_init) ||
	    !READ_ONCE(d8895_hiber_enabled))
		return;
	if (d8895_hiber_enter_cond(ctx))
		d8895_hiber_enter(ctx);
}

/* decon_hiber_block_exit() (decon_dsi.c:1201-1212) */
int exynos8895_decon_hiber_block_exit(void)
{
	struct decon8895_ctx *ctx = READ_ONCE(d8895_hiber_ctx);
	int ret = 0;

	if (!D8895_HIBER_ON || !ctx || !READ_ONCE(ctx->hiber_init))
		return 0;
	d8895_hiber_block(ctx);
	ret = d8895_hiber_exit(ctx);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos8895_decon_hiber_block_exit);

void exynos8895_decon_hiber_unblock(void)
{
	struct decon8895_ctx *ctx = READ_ONCE(d8895_hiber_ctx);

	if (!D8895_HIBER_ON || !ctx || !READ_ONCE(ctx->hiber_init))
		return;
	d8895_hiber_unblock(ctx);
}
EXPORT_SYMBOL_GPL(exynos8895_decon_hiber_unblock);

/* ---- CRTC ops ------------------------------------------------------------ */

/*
 * Window of every visible plane (L6A REQ-L5-3). Explicit windows (set by an
 * in-kernel client for all its planes) are taken as given; otherwise the
 * planes are stacked bottom-up in normalized zpos order from window 0, the
 * way the vendor composer numbers the windows of its layers (bottom layer =
 * lowest window, SLSI:graphics/base/libhwc2/libresource/
 * ExynosResourceManager.cpp:1220-1275), and G0, which the DECON only blends
 * in window 5, the top one (S:.../dpu/decon_core.c:1312-1318 "IDMA_G0
 * channel is dedicated to WIN5"), must be the top-most plane.
 */
static int d8895_assign_windows(struct decon8895_ctx *ctx, struct drm_crtc_state *cs)
{
	struct d8895_plane_state *vis[D8895_NR_CH];
	struct drm_plane *plane;
	unsigned int n = 0, n_explicit = 0, i, j, used = 0;
	int ret;

	ret = drm_atomic_add_affected_planes(cs->state, cs->crtc);
	if (ret)
		return ret;

	drm_for_each_plane_mask(plane, ctx->drm, cs->plane_mask) {
		struct drm_plane_state *st = drm_atomic_get_new_plane_state(cs->state, plane);
		struct d8895_plane_state *ps;

		if (!st)
			return -EINVAL;
		ps = to_d8895_pstate(st);
		ps->hw_win = -1;
		if (!st->visible || !st->fb)
			continue;
		vis[n++] = ps;
		if (ps->win >= 0)
			n_explicit++;
	}

	/* the DECON never starts without a window (decon_core.c:1714-1719) */
	if (!n)
		return -EINVAL;
	if (n_explicit && n_explicit != n)
		return -EINVAL;

	/* bottom-up by normalized zpos */
	for (i = 1; i < n; i++)
		for (j = i; j > 0 && vis[j - 1]->base.normalized_zpos >
				     vis[j]->base.normalized_zpos; j--)
			swap(vis[j - 1], vis[j]);

	for (i = 0, j = 0; i < n; i++) {
		unsigned int ch = to_d8895_plane(vis[i]->base.plane)->ch;
		int win;

		if (n_explicit)
			win = vis[i]->win;
		else if (ch == D8895_CH_G0)
			win = i == n - 1 ? D8895_WIN_G0 : -1;
		else
			win = j++;
		if (win < 0 || win >= D8895_NR_WIN || (used & BIT(win)) ||
		    (ch == D8895_CH_G0 && win != D8895_WIN_G0))
			return -EINVAL;
		used |= BIT(win);
		vis[i]->hw_win = win;
	}

	return 0;
}

static int d8895_atomic_check(struct exynos_drm_crtc *crtc, struct drm_crtc_state *state)
{
	struct decon8895_ctx *ctx = crtc->ctx;
	const struct drm_display_mode *m = &state->adjusted_mode;
	const struct drm_dsc_config *dsc = crtc->dsc;

	if (!state->active)
		return 0;

	/*
	 * Any of the six planes starts the DECON, as the vendor driver starts
	 * it with any configured window (decon_get_active_win_count(),
	 * S:.../dpu/decon_core.c:2446-2471); only no window at all is refused
	 * (L6A REQ-L5-9).
	 */
	if (!state->plane_mask)
		return -EINVAL;

	/* the one configuration the encoders and the DSIM are programmed for */
	if (!dsc || dsc->slice_count != 2 || !dsc->slice_height ||
	    dsc->pic_width != m->hdisplay || dsc->pic_height != m->vdisplay ||
	    dsc->slice_width * dsc->slice_count != m->hdisplay) {
		dev_dbg(ctx->dev, "mode %dx%d without a matching 2-slice DSC config\n",
			m->hdisplay, m->vdisplay);
		return -EINVAL;
	}

	return d8895_assign_windows(ctx, state);
}

/* bus votes released at disable; defined with the other vote helpers below */
static void d8895_bts_release(struct decon8895_ctx *ctx);

static void d8895_atomic_enable(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;
	int ret;

	if (!ctx->rpm_held) {
		ret = pm_runtime_resume_and_get(ctx->dev);
		if (ret < 0) {
			dev_err(ctx->dev, "cannot resume: %d\n", ret);
			return;
		}
		ctx->rpm_held = true;
	}
	/* a failed disable may have left hibernation on: leave it first */
	d8895_hiber_block(ctx);
	d8895_hiber_exit(ctx);
	d8895_hiber_unblock(ctx);
	if (READ_ONCE(ctx->hiber_on))
		return;	/* the begin/flush below retry the exit */

	pinctrl_pm_select_default_state(ctx->dev);	/* hw_te_on */

	/* S-Boot's running DECON, or a previous start: stop it (decon_reg_stop) */
	if (d_read(ctx, D8895_GLOBAL_CONTROL) & D8895_GLOBAL_CONTROL_RUN_STATUS)
		d8895_stop(ctx);
	d8895_dpp_deinit(ctx, D8895_CH_G0, false);

	ctx->needs_init = true;
	ctx->handover = false;
	greatlte_l5_set(GL5_ON, ho, false);
	if (!ctx->irqs_on) {
		enable_irq(ctx->irq_fs);
		enable_irq(ctx->irq_fd);
		enable_irq(ctx->irq_extra);
		ctx->irqs_on = true;
	}

	/* the new CRTC state is current here */
	d8895_panic_alloc(ctx, &crtc->base.state->adjusted_mode);
}

static void d8895_atomic_disable(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;
	int ch;

	/* blanking leaves hibernation first; the pipe is reachable below */
	d8895_hiber_block(ctx);
	d8895_hiber_exit(ctx);
	if (READ_ONCE(ctx->hiber_on)) {
		dev_err(ctx->dev, "disable while hibernating: pipe left on\n");
		d8895_hiber_unblock(ctx);
		return;
	}
	d8895_hiber_unblock(ctx);

	if (ctx->rpm_held)
		d8895_int_disable(ctx);
	if (ctx->irqs_on) {
		disable_irq(ctx->irq_fs);
		disable_irq(ctx->irq_fd);
		disable_irq(ctx->irq_extra);
		ctx->irqs_on = false;
	}
	atomic_set(&ctx->oneshot_state, 0);
	if (!ctx->rpm_held)
		return;
	d8895_stop(ctx);
	d8895_clear_int_all(ctx);
	for (ch = 0; ch < D8895_NR_CH; ch++)
		if (ctx->ch_on & BIT(ch))
			d8895_dpp_deinit(ctx, ch, false);

	d8895_bts_release(ctx);	/* S:dpu/bts.c:368-379: votes to 0, BTS to 0 */

	pinctrl_pm_select_sleep_state(ctx->dev);	/* hw_te_off */
	if (ctx->rpm_held) {
		ctx->rpm_held = false;
		pm_runtime_put_sync(ctx->dev);
	}
}

/*
 * Bus votes, per frame (spec L2b REQ-2B-L5-1; S:drivers/video/fbdev/exynos/dpu/
 * bts.c:93-154 dpu_bts_find_max_disp_freq(), :22-45 dpu_bts_calc_aclk_disp(),
 * :224-273 dpu_bts_calc_bw(), :275-310 dpu_bts_update_bw()). The DISP floor is
 * the bandwidth term over the DPU ports, raised to the widest scaled window's
 * aclk need; MIF/INT go through the BTS core (spec L2b C-1). Rising votes run
 * before the hardware update, falling ones after it; both may sleep (bus class
 * mutex, BTS media_mutex), so they run in the commit path outside any spinlock
 * (spec L2b 5.1 rule 5). Stated deviations: DECON2-only paths dropped
 * (dpu_bts_update_qos_{mif,int,disp,scen}, disp_freq_minlock, the DP fps: L6G
 * REQ-2B-L6G-1 owns them); ppc is 2, the dual-DSC panel (FDT:10095,10113
 * dsc_cnt = <0x02>; this driver only accepts 2-slice DSC, d8895_atomic_check);
 * refresh is LCD_REFRESH_RATE; resol_clk has no comp_ratio
 * or ppc divisor, verbatim (S:dpu/bts.c:410-421).
 */
static u32 d8895_bpp_of(u32 fourcc)
{
	switch (fourcc) {
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRX8888:
		return 32;
	case DRM_FORMAT_RGB565:
		return 16;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		return 12;
	default:
		return 0;
	}
}

/* aclk need of one scaled window (S:dpu/bts.c:22-45 dpu_bts_calc_aclk_disp) */
static u64 d8895_bts_aclk(u32 src_w, u32 src_h, u32 dst_w, u32 dst_h,
			  u64 resol_clk, u32 xres)
{
	u64 s_ratio_h, s_ratio_v, aclk;

	s_ratio_h = (src_w <= dst_w) ?
		D8895_BTS_MULT : D8895_BTS_MULT * (u64)src_w / (u64)dst_w;
	s_ratio_v = (src_h <= dst_h) ?
		D8895_BTS_MULT : D8895_BTS_MULT * (u64)src_h / (u64)dst_h;

	aclk = resol_clk * s_ratio_h * s_ratio_v * D8895_BTS_DFACTOR / 100UL
		/ D8895_BTS_PPC * (D8895_BTS_MULT * (u64)dst_w / (u64)xres)
		/ (D8895_BTS_MULT * D8895_BTS_MULT * D8895_BTS_MULT);

	return aclk;
}

/*
 * Per-frame bandwidth and DISP floor (S:dpu/bts.c:224-273 dpu_bts_calc_bw(),
 * :93-154 dpu_bts_find_max_disp_freq()). Only windows a plane scans out count,
 * same condition as the programming loop of d8895_atomic_flush().
 */
static void d8895_bts_calc(struct decon8895_ctx *ctx, struct drm_crtc_state *cs)
{
	struct bts_decon_info info;
	struct drm_plane *plane;
	u32 disp_ch_bw[BTS_DPU_MAX], max_ch, op_freq = 0, freq;
	u64 resol = ctx->bts_resol_clk;
	int i;

	memset(&info, 0, sizeof(info));
	drm_for_each_plane_mask(plane, ctx->drm, cs->plane_mask) {
		struct drm_plane_state *st = plane->state;
		struct bts_dpp_info *dpp;
		unsigned int ch;
		int win;

		if (!st->visible || !st->fb)
			continue;
		win = to_d8895_pstate(st)->hw_win;
		if (win < 0)
			continue;
		ch = to_d8895_plane(plane)->ch;
		if (ch >= BTS_DPP_MAX)
			continue;
		dpp = &info.dpp[ch];
		dpp->used = true;
		dpp->bpp = d8895_bpp_of(st->fb->format->format);
		dpp->src_w = drm_rect_width(&st->src) >> 16;
		dpp->src_h = drm_rect_height(&st->src) >> 16;
		dpp->dst.x1 = st->dst.x1;
		dpp->dst.x2 = st->dst.x2;
		dpp->dst.y1 = st->dst.y1;
		dpp->dst.y2 = st->dst.y2;
	}

	info.vclk = ctx->bts_resol_clk;
	info.lcd_w = ctx->bts_xres;
	info.lcd_h = ctx->bts_yres;
	ctx->bts_total_bw = bts_calc_bw(BTS_BW_DECON0, &info);
	for (i = 0; i < BTS_DPP_MAX; i++)
		ctx->bts_bw[i] = info.dpp[i].bw;

	/* S:dpu/bts.c:106-108: the DPU ports each DPP pair drains through */
	disp_ch_bw[BTS_DPU0] = ctx->bts_bw[BTS_DPP4] + ctx->bts_bw[BTS_DPP5];
	disp_ch_bw[BTS_DPU1] = ctx->bts_bw[BTS_DPP0] + ctx->bts_bw[BTS_DPP2];
	disp_ch_bw[BTS_DPU2] = ctx->bts_bw[BTS_DPP1] + ctx->bts_bw[BTS_DPP3];

	max_ch = disp_ch_bw[0];
	for (i = 1; i < BTS_DPU_MAX; i++)
		if (max_ch < disp_ch_bw[i])
			max_ch = disp_ch_bw[i];
	ctx->bts_peak = max_ch;
	ctx->bts_max_disp_freq = max_ch * 100 / (16 * D8895_BTS_BUS_UTIL) + 1;

	drm_for_each_plane_mask(plane, ctx->drm, cs->plane_mask) {
		struct drm_plane_state *st = plane->state;
		u32 sw, sh, dw, dh;
		int win;

		if (!st->visible || !st->fb)
			continue;
		win = to_d8895_pstate(st)->hw_win;
		if (win < 0)
			continue;
		sw = drm_rect_width(&st->src) >> 16;
		sh = drm_rect_height(&st->src) >> 16;
		dw = drm_rect_width(&st->dst);
		dh = drm_rect_height(&st->dst);
		if (!dw || !dh || !ctx->bts_xres)
			continue;
		freq = (u32)d8895_bts_aclk(sw, sh, dw, dh, resol, ctx->bts_xres);
		if (op_freq < freq)
			op_freq = freq;
	}

	if (ctx->bts_max_disp_freq < op_freq)
		ctx->bts_max_disp_freq = op_freq;
}

/* rising/falling votes around the hardware update (S:dpu/bts.c:275-310) */
static void d8895_bts_update(struct decon8895_ctx *ctx, bool after)
{
	struct bts_bw bw = {
		.peak = ctx->bts_peak,
		.read = ctx->bts_total_bw,
	};

	if (bw.read == 0)
		bw.peak = 0;

	if (after) {
		if (ctx->bts_total_bw <= ctx->bts_prev_total_bw)
			bts_update_bw(BTS_BW_DECON0, bw);
		if (ctx->bts_max_disp_freq <= ctx->bts_prev_max_disp_freq)
			exynos_bus_qos_update_request(&ctx->disp_req,
						      ctx->bts_max_disp_freq);
		ctx->bts_prev_total_bw = ctx->bts_total_bw;
		ctx->bts_prev_max_disp_freq = ctx->bts_max_disp_freq;
	} else {
		if (ctx->bts_total_bw > ctx->bts_prev_total_bw)
			bts_update_bw(BTS_BW_DECON0, bw);
		if (ctx->bts_max_disp_freq > ctx->bts_prev_max_disp_freq)
			exynos_bus_qos_update_request(&ctx->disp_req,
						      ctx->bts_max_disp_freq);
	}
}

/* votes released when the DECON stops (S:dpu/bts.c:368-379 dpu_bts_release_bw) */
static void d8895_bts_release(struct decon8895_ctx *ctx)
{
	struct bts_bw bw = { 0, };

	if (!ctx->disp_qos_added)
		return;
	bts_update_bw(BTS_BW_DECON0, bw);
	ctx->bts_prev_total_bw = 0;
	exynos_bus_qos_update_request(&ctx->disp_req, 0);
	ctx->bts_prev_max_disp_freq = 0;
}

/* wait for the shadow update, then mask the trigger (decon_core.c:1663-1672) */
static void d8895_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;
	u32 val;

	if (!ctx->rpm_held)	/* atomic_enable could not resume: never touch the block */
		return;
	/*
	 * Every commit leaves hibernation first (decon_set_win_info(),
	 * S:.../dpu/decon_dsi.c:761); the block is held to the end of the
	 * flush, so a TE cannot power the pipe down under the new frame.
	 */
	d8895_hiber_block(ctx);
	d8895_hiber_exit(ctx);
	if (READ_ONCE(ctx->hiber_on))	/* the exit failed: touch nothing below */
		return;
	if (readl_poll_timeout(ctx->regs + D8895_SHADOW_REG_UPDATE_REQ, val, !val, 100,
			       D8895_SHADOW_TIMEOUT_US)) {
		dev_warn(ctx->dev, "shadow update request not cleared (0x%08x)\n", val);
		d_write(ctx, D8895_SHADOW_REG_UPDATE_REQ, 0);
		if (readl_poll_timeout(ctx->regs + D8895_SHADOW_REG_UPDATE_REQ, val, !val, 100,
				       1000))
			dev_err(ctx->dev, "failed to clear the shadow update request\n");
	}
	d8895_trigger(ctx, false);
}

static void d8895_dump(struct decon8895_ctx *ctx)
{
	void __iomem *g0 = d_dma(ctx, D8895_CH_G0);

	dev_err(ctx->dev, "d0=0x%08x d1=0x%08x d2=0x%08x g0e=0x%08x g0c=0x%08x g0y=0x%08x\n",
		d_read(ctx, D8895_DATA_PATH_CONTROL_0), d_read(ctx, D8895_DATA_PATH_CONTROL_1),
		d_read(ctx, D8895_DATA_PATH_CONTROL_2), readl(g0 + D8895_IDMA_ENABLE),
		readl(g0 + D8895_IDMA_IN_CON), readl(g0 + D8895_IDMA_IN_BASE_ADDR_Y));
}

/*
 * decon_reg_start() (decon_reg.c:2097-2111). @oneshot arms the one-shot
 * state together with the unmask, under the lock the frame-done mask takes:
 * the frame done of the previous frame either masks before the unmask or
 * finds the new state, and the new frame cannot start before its state is
 * armed.
 */
static int d8895_start(struct decon8895_ctx *ctx, bool oneshot)
{
	unsigned long flags;
	u32 val;
	int ret;

	d_mask(ctx, D8895_GLOBAL_CONTROL, ~0,
	       D8895_GLOBAL_CONTROL_DECON_EN | D8895_GLOBAL_CONTROL_DECON_EN_F);
	d_mask(ctx, D8895_SHADOW_REG_UPDATE_REQ, ~0, D8895_SHADOW_REG_UPDATE_REQ_GLOBAL);
	ret = readl_poll_timeout_atomic(ctx->regs + D8895_GLOBAL_CONTROL, val,
					val & D8895_GLOBAL_CONTROL_RUN_STATUS, 10,
					D8895_RUN_TIMEOUT_US);
	if (ret) {
		/* stock BUG()s here (decon_core.c:1735-1740) */
		dev_err(ctx->dev, "decon start: RUN_STATUS timeout, GC=0x%08x TRIG=0x%08x SHADOW=0x%08x\n",
			val, d_read(ctx, D8895_HW_SW_TRIG_CONTROL),
			d_read(ctx, D8895_SHADOW_REG_UPDATE_REQ));
		d8895_dump(ctx);
		return ret;
	}
	spin_lock_irqsave(&ctx->trig_lock, flags);
	if (oneshot)
		atomic_set(&ctx->oneshot_state, 1);
	__d8895_trigger(ctx, true);
	spin_unlock_irqrestore(&ctx->trig_lock, flags);

	return 0;
}

/*
 * dpu_set_dsu_update_config() (S:.../dpu/dsu_update.c:114-156), called after
 * the HW trigger is masked in __decon_update_regs() (decon_core.c:1671-1677):
 * wait for DECON idle, send PANEL_IOC_SET_DSU, update DSIM (DSIM_IOC_SET_DSU),
 * update DECON (decon_set_dsu_update), and commit the new mode state.
 */
static void d8895_dsu_update_config(struct decon8895_ctx *ctx)
{
	struct dsu_info cur_dsu;
	unsigned long flags;

	spin_lock_irqsave(&ctx->dsu_lock, flags);
	cur_dsu = ctx->pending_dsu;
	ctx->pending_dsu.needupdate = 0;
	spin_unlock_irqrestore(&ctx->dsu_lock, flags);

	if (!cur_dsu.needupdate)
		return;

	if (ctx->dsu.mode == cur_dsu.mode) {
		dev_info(ctx->dev, "DECON:INFO:%s:already set dsu mode : %d\n",
			 __func__, cur_dsu.mode);
		return;
	}

	d8895_wait_idle(ctx);
	d8895_panel_set_dsu(&cur_dsu);
	d8895_dsim_set_dsu(&cur_dsu);
	d8895_set_dsu_update(ctx, &cur_dsu);

	spin_lock_irqsave(&ctx->dsu_lock, flags);
	ctx->dsu.mode = cur_dsu.mode;
	ctx->dsu.top = cur_dsu.top;
	ctx->dsu.bottom = cur_dsu.bottom;
	ctx->dsu.left = cur_dsu.left;
	ctx->dsu.right = cur_dsu.right;
	spin_unlock_irqrestore(&ctx->dsu_lock, flags);
}

static void d8895_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;
	struct drm_crtc_state *cs = crtc->base.state;
	struct drm_plane *plane;
	unsigned long flags, aclk_khz;
	struct exynos8895_color_win cw[D8895_NR_WIN];
	u32 used_win = 0, used_ch = 0;
	u8 cw_on;
	int win, ch;

	if (!ctx->rpm_held)
		goto out_event;
	if (READ_ONCE(ctx->hiber_on))	/* begin could not leave hibernation: no hardware */
		goto out_unblock;
	/* a programmed update clears the timeout (decon_core.c:2013, 2028) */
	if (D8895_HIBER_ON)
		WRITE_ONCE(ctx->hiber_timeout, false);

	if (ctx->needs_init) {
		d8895_hw_init(ctx, &cs->adjusted_mode, crtc->dsc);
		ctx->needs_init = false;
		d8895_int_enable(ctx);
	}

	d8895_dsu_update_config(ctx);

	drm_for_each_plane_mask(plane, ctx->drm, cs->plane_mask) {
		struct d8895_plane *dp = to_d8895_plane(plane);
		struct drm_plane_state *st = plane->state;

		win = to_d8895_pstate(st)->hw_win;
		if (!st->visible || !st->fb || win < 0)
			continue;
		d8895_dpp_init(ctx, dp->ch);
		d8895_dpp_configure(ctx, dp->ch, st);
		d8895_win_program_plane(ctx, win, dp->ch, st);
		used_win |= BIT(win);
		used_ch |= BIT(dp->ch);

		if (!ctx->first_frame_done) {
			dma_addr_t a = exynos_drm_fb_dma_addr(st->fb, 0);

			ctx->first_frame_done = true;
			dev_info(ctx->dev, "first frame: win%d ch%d dma %pad fmt %p4cc pitch %u\n",
				 win, d8895_ch[dp->ch].chmap, &a, &st->fb->format->format,
				 st->fb->pitches[0]);
		}
	}
	/*
	 * A window no plane uses is a colour window if the client asked for one
	 * (decon_prepare_win_config(), S:.../dpu/decon_core.c:2566-2575: colour
	 * map on, alpha forced to 0xff, the named channel in CHMAP), else off.
	 */
	spin_lock_irqsave(&ctx->cw_lock, flags);
	cw_on = ctx->cw_on;
	memcpy(cw, ctx->cw, sizeof(cw));
	spin_unlock_irqrestore(&ctx->cw_lock, flags);
	for (win = 0; win < D8895_NR_WIN; win++) {
		if (used_win & BIT(win))
			continue;
		if (cw_on & BIT(win))
			d8895_win_program(ctx, win, d8895_ch[cw[win].plane].chmap, &cw[win].dst,
					  cw[win].alpha, cw[win].blend, true, true,
					  cw[win].argb | 0xff000000);
		else
			d8895_win_disable(ctx, win);
	}
	/*
	 * A channel whose window goes away in this update: mask its interrupts
	 * (the vendor stops the DPPs the new frame no longer uses,
	 * decon_dpp_stop(), S:.../dpu/decon_core.c:428-448). Its window is
	 * disabled by the same shadow update, so the fetch of the frame on
	 * screen is not disturbed.
	 */
	for (ch = 0; ch < D8895_NR_CH; ch++)
		if ((ctx->ch_on & BIT(ch)) && !(used_ch & BIT(ch)))
			d8895_dpp_deinit(ctx, ch, false);
	ctx->ch_used = used_ch;

 	/* aclk_khz * (1000 / 60) * 2 ms (decon_core.c:1638-1652,1724-1731) */
	aclk_khz = clk_get_rate(ctx->clks[0].clk) / 1000;
	if (!aclk_khz)
		aclk_khz = D8895_ACLK_KHZ_DEFAULT;
	d_write(ctx, D8895_TIME_OUT_VALUE, aclk_khz * ((1000 / 60) * 2));

	/*
	 * Rising bus votes before the hardware takes the new frame
	 * (S:dpu/bts.c:290-306, is_after = 0).
	 */
	d8895_bts_calc(ctx, cs);
	d8895_bts_update(ctx, false);

	/* decon_reg_all_win_shadow_update_req() (decon_reg.c:825-838) */
	d_mask(ctx, D8895_SHADOW_REG_UPDATE_REQ, ~0, D8895_SHADOW_REG_UPDATE_REQ_FOR_DECON0);
	/*
	 * The event is armed before the trigger is unmasked. The next TE may
	 * start the frame right away, and the falling votes below can sleep in
	 * a bus DVFS change for longer than a frame. A frame start ahead of
	 * the event counts its vblank without it, the frame done then masks
	 * the one-shot trigger, and no later frame start completes the flip.
	 */
	spin_lock_irqsave(&ctx->vblank_lock, flags);
	exynos_crtc_handle_event(crtc);
	spin_unlock_irqrestore(&ctx->vblank_lock, flags);
	/*
	 * One-shot unless the panel is in HMD mode: the vendor then keeps the
	 * trigger running ("video_emul", S:.../dpu/decon_core.c:2224-2228,
	 * 2320-2323).
	 */
	d8895_start(ctx, READ_ONCE(ctx->oneshot) && !d8895_panel_hmd_on());

	/* falling votes after the hardware update (S:dpu/bts.c:292-307) */
	d8895_bts_update(ctx, true);
	d8895_hiber_unblock(ctx);
	return;

out_unblock:
	d8895_hiber_unblock(ctx);
out_event:
	spin_lock_irqsave(&ctx->vblank_lock, flags);
	exynos_crtc_handle_event(crtc);
	spin_unlock_irqrestore(&ctx->vblank_lock, flags);
}

static int d8895_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;

	WRITE_ONCE(ctx->vblank_on, true);
	return 0;
}

static void d8895_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;

	WRITE_ONCE(ctx->vblank_on, false);
}

/*
 * HW-trigger mode: the TE only timestamps (S:.../dpu/decon_dsi.c:503-535,
 * ktime_get() first) and hands the time to the client's vsync callback
 * (L6A REQ-L5-5).
 */
static void d8895_te_handler(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;
	ktime_t ts = ktime_get();
	unsigned long flags;

	atomic_inc(&ctx->te_cnt);
	greatlte_l5_inc(GL5_ON, tec);
	spin_lock_irqsave(&ctx->vsync_lock, flags);
	if (ctx->vsync_cb)
		ctx->vsync_cb(ctx->vsync_data, ts);
	spin_unlock_irqrestore(&ctx->vsync_lock, flags);
	/*
	 * Hibernation: every TE queues the enter check while the CRTC is on
	 * (decon_dsi.c:518-523); the check counts TEs without a frame done.
	 */
	if (D8895_HIBER_ON && ctx->rpm_held && READ_ONCE(ctx->hiber_init) &&
	    READ_ONCE(d8895_hiber_enabled) && !d8895_hiber_blocked(ctx))
		queue_work(ctx->hiber_wq, &ctx->hiber_work);
}

static enum drm_mode_status d8895_mode_valid(struct exynos_drm_crtc *crtc,
					     const struct drm_display_mode *mode)
{
	/* one DSC-compressed command-mode panel; sizes the registers can hold */
	if (mode->hdisplay > 0x3fff || mode->vdisplay > 0x3fff)
		return MODE_BAD;
	return MODE_OK;
}

static const struct exynos_drm_crtc_ops d8895_crtc_ops = {
	.atomic_enable	= d8895_atomic_enable,
	.atomic_disable	= d8895_atomic_disable,
	.enable_vblank	= d8895_enable_vblank,
	.disable_vblank	= d8895_disable_vblank,
	.mode_valid	= d8895_mode_valid,
	.atomic_check	= d8895_atomic_check,
	.atomic_begin	= d8895_atomic_begin,
	.atomic_flush	= d8895_atomic_flush,
	.te_handler	= d8895_te_handler,
};

/* ---- planes -------------------------------------------------------------- */

/*
 * The vendor DPP limits (L6A REQ-L5-2): dpp_check_limitation() and
 * dpp_check_size() (S:.../dpu/dpp_drv.c:351-532) with the size constraints
 * of dpp_constraints_params() (S:.../dpu/dpp_reg.c:1436-1477, S:.../dpu/
 * dpp.h:47-60): YUV only on VG/VGF, scaling (1/2x down .. 8x up) and AFBC
 * only on VGF, no AFBC with flip or YUV 4:2:0; YUV offsets and sizes even;
 * source 16 (32 for YUV) .. 65534 x 16 .. 8190, window 16 .. 4096; plus the
 * DECON window rules of decon_check_limitation() (S:.../dpu/
 * decon_core.c:1309-1351): on screen, at least 16 wide.
 */
static int d8895_plane_atomic_check(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct drm_plane_state *st = drm_atomic_get_new_plane_state(state, plane);
	struct d8895_plane_state *ps = to_d8895_pstate(st);
	struct d8895_plane *dp = to_d8895_plane(plane);
	unsigned int ch = dp->ch;
	struct decon8895_ctx *ctx = container_of(dp, struct decon8895_ctx, planes[ch]);
	struct drm_framebuffer *fb = st->fb;
	int min_scale = DRM_PLANE_NO_SCALING, max_scale = DRM_PLANE_NO_SCALING;
	const struct d8895_fmt *f;
	struct drm_crtc_state *cs;
	u32 sx, sy, sw, sh, dw, dh, fw, al;
	unsigned long flags;
	int max_w, max_h;
	int ret;

	if (!st->crtc || !fb)
		return 0;

	cs = drm_atomic_get_new_crtc_state(state, st->crtc);
	if (!cs)
		return -EINVAL;

	if (d8895_ch[ch].scale) {
		min_scale = DRM_PLANE_NO_SCALING / 8;	/* 8x up */
		max_scale = DRM_PLANE_NO_SCALING * 2;	/* 1/2x down */
	}
	ret = drm_atomic_helper_check_plane_state(st, cs, min_scale, max_scale, true, true);
	if (ret)
		return ret;
	if (!st->visible)
		return 0;

	spin_lock_irqsave(&ctx->dsu_lock, flags);
	max_w = ctx->pending_dsu.needupdate ? ctx->pending_dsu.right : ctx->dsu.right;
	max_h = ctx->pending_dsu.needupdate ? ctx->pending_dsu.bottom : ctx->dsu.bottom;
	spin_unlock_irqrestore(&ctx->dsu_lock, flags);

	/* the whole window on the screen (active DSU resolution), whole source pixels */
	if (st->dst.x1 != st->crtc_x || st->dst.y1 != st->crtc_y ||
	    drm_rect_width(&st->dst) != st->crtc_w || drm_rect_height(&st->dst) != st->crtc_h ||
	    (max_w > 0 && st->dst.x2 > max_w) || (max_h > 0 && st->dst.y2 > max_h) ||
	    ((st->src.x1 | st->src.y1 | st->src.x2 | st->src.y2) & 0xffff))
		return -EINVAL;

	f = d8895_find_fmt(fb->format->format);
	if (!f || (f->yuv && !d8895_ch[ch].yuv) || fb->modifier != DRM_FORMAT_MOD_LINEAR)
		return -EINVAL;
	if (ps->afbc && (!d8895_ch[ch].scale || f->yuv ||
			 (st->rotation & (DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y))))
		return -EINVAL;
	if (fb->pitches[0] % fb->format->cpp[0] ||
	    (f->yuv && fb->pitches[1] != fb->pitches[0]))
		return -EINVAL;

	sx = st->src.x1 >> 16;
	sy = st->src.y1 >> 16;
	sw = drm_rect_width(&st->src) >> 16;
	sh = drm_rect_height(&st->src) >> 16;
	dw = drm_rect_width(&st->dst);
	dh = drm_rect_height(&st->dst);
	fw = fb->pitches[0] / fb->format->cpp[0];
	al = f->yuv ? 2 : 1;

	if (!IS_ALIGNED(sx, al) || !IS_ALIGNED(sy, al) || !IS_ALIGNED(fw, al) ||
	    !IS_ALIGNED(fb->height, al) || !IS_ALIGNED(sw, al) || !IS_ALIGNED(sh, al))
		return -EINVAL;
	if (sw < D8895_SRC_WIDTH_MIN * al || sw > D8895_SRC_WIDTH_MAX ||
	    sh < D8895_SRC_HEIGHT_MIN || sh > D8895_SRC_HEIGHT_MAX ||
	    fw > D8895_SRC_WIDTH_MAX || fb->height > D8895_SRC_HEIGHT_MAX)
		return -EINVAL;
	if (dw < D8895_IMG_WIDTH_MIN || dw > D8895_IMG_WIDTH_MAX ||
	    dh < D8895_IMG_HEIGHT_MIN || dh > D8895_IMG_HEIGHT_MAX)
		return -EINVAL;
	/* dpp_check_scale_ratio() (dpp_drv.c:398-423) */
	if (sw > dw * 2 || sh > dh * 2 || sw < (dw + 7) / 8 || sh < (dh + 7) / 8)
		return -EINVAL;

	return 0;
}

/*
 * drm_atomic_helper_commit_planes() calls .atomic_update for every plane with a CRTC,
 * without a NULL check (A:drivers/gpu/drm/drm_atomic_helper.c:3035-3036). The window and
 * its DPP channel are programmed from d8895_atomic_flush(), under the shadow update.
 */
static void d8895_plane_atomic_update(struct drm_plane *plane, struct drm_atomic_state *state)
{
}

static const struct drm_plane_helper_funcs d8895_plane_helper_funcs = {
	.atomic_check = d8895_plane_atomic_check,
	.atomic_update = d8895_plane_atomic_update,
};

static struct decon8895_ctx *d8895_ctx_of_primary(struct drm_plane *plane)
{
	return container_of(plane, struct decon8895_ctx, planes[D8895_CH_G0].base);
}

/*
 * drm_panic (A:drivers/gpu/drm/drm_panic.c:874-902), with the panic lock
 * held: the panic buffer, only while the CRTC is on and initialised.
 */
static int d8895_get_scanout_buffer(struct drm_plane *plane, struct drm_scanout_buffer *sb)
{
	struct decon8895_ctx *ctx = d8895_ctx_of_primary(plane);

	if (!ctx->panic_vaddr || !READ_ONCE(ctx->rpm_held) || READ_ONCE(ctx->needs_init))
		return -ENODEV;

	sb->format = drm_format_info(DRM_FORMAT_XRGB8888);
	sb->width = ctx->panic_w;
	sb->height = ctx->panic_h;
	sb->pitch[0] = ctx->panic_w * 4;
	iosys_map_set_vaddr(&sb->map[0], ctx->panic_vaddr);

	return 0;
}

/*
 * Put the panic buffer on screen from a panic: no lock but the held panic
 * lock (trig_lock may be held by a CPU the panic stopped), no IRQ
 * enable/disable, no sleep, no allocation. G0 1:1 over the whole screen in
 * window 5 with every other window off, then the start of d8895_atomic_flush()
 * and d8895_start() (S:.../dpu/decon_reg.c:825-838, :2097-2111) without the
 * RUN_STATUS poll.
 */
static void d8895_panic_flush(struct drm_plane *plane)
{
	struct decon8895_ctx *ctx = d8895_ctx_of_primary(plane);
	u32 w = ctx->panic_w, h = ctx->panic_h;
	const struct drm_rect full = DRM_RECT_INIT(0, 0, w, h);
	const struct d8895_dpp_cfg cfg = {
		.fmt	= d8895_find_fmt(DRM_FORMAT_XRGB8888),
		.y	= ctx->panic_dma,
		.sw	= w,
		.sh	= h,
		.fw	= w,
		.fh	= h,
		.dw	= w,
		.dh	= h,
	};
	unsigned int win;

	/*
	 * Best effort only: with the pipe hibernating the clocks are gated
	 * and every register access below would hang the crashing CPU.
	 */
	if (D8895_HIBER_ON && READ_ONCE(ctx->hiber_init) && READ_ONCE(ctx->hiber_on))
		return;

	/* the panic screen out of the write-combining buffer before the fetch */
	wmb();

	if (!(ctx->ch_on & BIT(D8895_CH_G0)))
		d8895_dpp_init_regs(ctx, D8895_CH_G0);
	d8895_dpp_program(ctx, D8895_CH_G0, &cfg);
	for (win = 0; win < D8895_WIN_G0; win++)
		d8895_win_disable(ctx, win);
	d8895_win_program(ctx, D8895_WIN_G0, d8895_ch[D8895_CH_G0].chmap, &full,
			  DRM_BLEND_ALPHA_OPAQUE, DRM_MODE_BLEND_PIXEL_NONE, false, false, 0);

	d_mask(ctx, D8895_SHADOW_REG_UPDATE_REQ, ~0, D8895_SHADOW_REG_UPDATE_REQ_FOR_DECON0);
	d_mask(ctx, D8895_GLOBAL_CONTROL, ~0,
	       D8895_GLOBAL_CONTROL_DECON_EN | D8895_GLOBAL_CONTROL_DECON_EN_F);
	d_mask(ctx, D8895_SHADOW_REG_UPDATE_REQ, ~0, D8895_SHADOW_REG_UPDATE_REQ_GLOBAL);
	/* decon_reg_set_trigger() enable (S:.../dpu/decon_reg.c:1006-1024), without trig_lock */
	d_mask(ctx, D8895_HW_SW_TRIG_CONTROL, D8895_HW_TRIG_EN,
	       D8895_HW_TRIG_EN | D8895_HW_TRIG_MASK_DECON);
	if (READ_ONCE(ctx->oneshot))
		atomic_set(&ctx->oneshot_state, 1);
}

/* G0, the primary plane, also carries the drm_panic hooks */
static const struct drm_plane_helper_funcs d8895_primary_helper_funcs = {
	.atomic_check = d8895_plane_atomic_check,
	.atomic_update = d8895_plane_atomic_update,
	.get_scanout_buffer = d8895_get_scanout_buffer,
	.panic_flush = d8895_panic_flush,
};

static void d8895_plane_reset(struct drm_plane *plane)
{
	struct d8895_plane_state *ps;

	if (plane->state) {
		__drm_atomic_helper_plane_destroy_state(plane->state);
		kfree(to_d8895_pstate(plane->state));
		plane->state = NULL;
	}
	ps = kzalloc(sizeof(*ps), GFP_KERNEL);
	if (!ps)
		return;
	__drm_atomic_helper_plane_reset(plane, &ps->base);
	ps->win = -1;
	ps->hw_win = -1;
}

static struct drm_plane_state *d8895_plane_duplicate_state(struct drm_plane *plane)
{
	struct d8895_plane_state *old = to_d8895_pstate(plane->state), *ps;

	ps = kmemdup(old, sizeof(*ps), GFP_KERNEL);
	if (!ps)
		return NULL;
	__drm_atomic_helper_plane_duplicate_state(plane, &ps->base);

	return &ps->base;
}

static void d8895_plane_destroy_state(struct drm_plane *plane, struct drm_plane_state *st)
{
	__drm_atomic_helper_plane_destroy_state(st);
	kfree(to_d8895_pstate(st));
}

static const struct drm_plane_funcs d8895_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= d8895_plane_reset,
	.atomic_duplicate_state	= d8895_plane_duplicate_state,
	.atomic_destroy_state	= d8895_plane_destroy_state,
};

/*
 * One plane per DPP channel, plane index = stock enum decon_idma_type
 * (S:.../dpu/decon.h:309-318; L6A REQ-L5-1). G0 is the primary plane (the
 * boot pairing with window 5); its default zpos puts it on top, where the
 * window-5 rule of d8895_assign_windows() wants it.
 */
static int d8895_plane_init(struct decon8895_ctx *ctx, unsigned int ch)
{
	static const u8 zpos_default[D8895_NR_CH] = { 5, 0, 1, 2, 3, 4 };
	struct d8895_plane *p = &ctx->planes[ch];
	unsigned int nfmt = d8895_ch[ch].yuv ? ARRAY_SIZE(d8895_formats) : D8895_NR_RGB;
	u32 formats[ARRAY_SIZE(d8895_formats)];
	int i, ret;

	for (i = 0; i < nfmt; i++)
		formats[i] = d8895_formats[i].fourcc;

	ret = drm_universal_plane_init(ctx->drm, &p->base, 1 << ctx->drm->mode_config.num_crtc,
				       &d8895_plane_funcs, formats, nfmt, NULL,
				       ch == D8895_CH_G0 ? DRM_PLANE_TYPE_PRIMARY :
							   DRM_PLANE_TYPE_OVERLAY,
				       "%s", d8895_ch[ch].name);
	if (ret)
		return ret;
	drm_plane_helper_add(&p->base, ch == D8895_CH_G0 ? &d8895_primary_helper_funcs :
							   &d8895_plane_helper_funcs);
	p->ch = ch;
	ret = drm_plane_create_zpos_property(&p->base, zpos_default[ch], 0, D8895_NR_WIN - 1);
	if (ret)
		return ret;

	/* per-plane stock capabilities (L6A REQ-L5-2) */
	ret = drm_plane_create_alpha_property(&p->base);
	if (ret)
		return ret;
	ret = drm_plane_create_blend_mode_property(&p->base, BIT(DRM_MODE_BLEND_PIXEL_NONE) |
						   BIT(DRM_MODE_BLEND_PREMULTI) |
						   BIT(DRM_MODE_BLEND_COVERAGE));
	if (ret)
		return ret;
	/* IDMA_IN_FLIP on every input channel (S:.../dpu/dpp_reg.c:453-462) */
	ret = drm_plane_create_rotation_property(&p->base, DRM_MODE_ROTATE_0,
						 DRM_MODE_ROTATE_0 | DRM_MODE_REFLECT_X |
						 DRM_MODE_REFLECT_Y);
	if (ret)
		return ret;
	if (d8895_ch[ch].yuv) {
		ret = drm_plane_create_color_properties(&p->base,
							BIT(DRM_COLOR_YCBCR_BT601) |
							BIT(DRM_COLOR_YCBCR_BT709) |
							BIT(DRM_COLOR_YCBCR_BT2020),
							BIT(DRM_COLOR_YCBCR_LIMITED_RANGE) |
							BIT(DRM_COLOR_YCBCR_FULL_RANGE),
							DRM_COLOR_YCBCR_BT601,
							DRM_COLOR_YCBCR_LIMITED_RANGE);
		if (ret)
			return ret;
	}
	ctx->nr_planes++;

	return 0;
}

/* ---- IRQs ---------------------------------------------------------------- */

/* decon_reg_get_interrupt_and_clear() (decon_reg.c:2504-2564) */
static irqreturn_t d8895_irq(int irq, void *dev_id)
{
	struct decon8895_ctx *ctx = dev_id;
	u32 pend, ext;

	pend = d_read(ctx, D8895_INTERRUPT_PENDING);
	if (pend & D8895_DPU_UNDER_FLOW_INT_PEND)
		d_write(ctx, D8895_INTERRUPT_PENDING, D8895_DPU_UNDER_FLOW_INT_PEND);
	if (pend & D8895_DPU_FRAME_START_INT_PEND) {
		d_write(ctx, D8895_INTERRUPT_PENDING, D8895_DPU_FRAME_START_INT_PEND);
		atomic_inc(&ctx->fs_cnt);
		greatlte_l5_inc(GL5_ON, fs);
		/* the first frame after the flush latched its shadow registers */
		if (atomic_cmpxchg(&ctx->oneshot_state, 1, 2) == 1) {
			spin_lock(&ctx->vsync_lock);
			if (ctx->fs_cb)
				ctx->fs_cb(ctx->fs_data);
			spin_unlock(&ctx->vsync_lock);
		}
		/*
		 * The flip completes when the new frame starts: its shadow registers
		 * latched and the DMA reads the new buffers. decon_update_regs()
		 * returns at that point too (decon_wait_for_vsync() and
		 * decon_wait_for_vstatus(), S:.../dpu/decon_core.c:2279-2283), so
		 * the next update is written while this frame is still transferring
		 * and latches at the next TE. Completing at frame done held the
		 * commit for the whole transfer: the next flush often missed the
		 * following TE (33 ms commits) and frames queued behind it.
		 */
		if (READ_ONCE(ctx->vblank_on)) {
			spin_lock(&ctx->vblank_lock);
			drm_crtc_handle_vblank(&ctx->crtc->base);
			spin_unlock(&ctx->vblank_lock);
		}
	}
	if (pend & D8895_DPU_FRAME_DONE_INT_PEND) {
		d_write(ctx, D8895_INTERRUPT_PENDING, D8895_DPU_FRAME_DONE_INT_PEND);
		atomic_inc(&ctx->fd_cnt);
		greatlte_l5_inc(GL5_ON, fd);
		/* a frame went out: hibernation counts from here (decon_dsi.c:63) */
		if (D8895_HIBER_ON && READ_ONCE(ctx->hiber_init))
			d8895_hiber_trig_reset(ctx);
		/*
		 * One-shot: mask the HW trigger once the updated frame is out, as
		 * the vendor does after each update (S:.../dpu/decon_core.c:
		 * 2316-2326); the next commit re-enables it.
		 */
		if (atomic_read(&ctx->oneshot_state) == 2) {
			spin_lock(&ctx->trig_lock);
			if (!(d_read(ctx, D8895_SHADOW_REG_UPDATE_REQ) &
			      D8895_SHADOW_REG_UPDATE_REQ_GLOBAL) &&
			    atomic_cmpxchg(&ctx->oneshot_state, 2, 0) == 2)
				__d8895_trigger(ctx, false);
			spin_unlock(&ctx->trig_lock);
		}
		d8895_panel_frame_done();
	}
	if (pend & D8895_DPU_DQE_DIMMING_START_INT_PEND)
		d_write(ctx, D8895_INTERRUPT_PENDING, D8895_DPU_DQE_DIMMING_START_INT_PEND);
	if (pend & D8895_DPU_DQE_DIMMING_END_INT_PEND)
		d_write(ctx, D8895_INTERRUPT_PENDING, D8895_DPU_DQE_DIMMING_END_INT_PEND);
	if (pend & D8895_DPU_MDNIE_DIMMING_START_INT_PEND)
		d_write(ctx, D8895_INTERRUPT_PENDING, D8895_DPU_MDNIE_DIMMING_START_INT_PEND);
	if (pend & D8895_DPU_MDNIE_DIMMING_END_INT_PEND)
		d_write(ctx, D8895_INTERRUPT_PENDING, D8895_DPU_MDNIE_DIMMING_END_INT_PEND);
	if (pend & D8895_DPU_EXTRA_INT_PEND) {
		d_write(ctx, D8895_INTERRUPT_PENDING, D8895_DPU_EXTRA_INT_PEND);
		ext = d_read(ctx, D8895_EXTRA_INTERRUPT_PENDING);
		if (ext & D8895_DPU_RESOURCE_CONFLICT_INT_PEND) {
			d_write(ctx, D8895_EXTRA_INTERRUPT_PENDING,
				D8895_DPU_RESOURCE_CONFLICT_INT_PEND);
			atomic_inc(&ctx->rc_cnt);
			greatlte_l5_inc(GL5_ON, rc);
			dev_err_ratelimited(ctx->dev, "resource conflict: %08x %08x %08x\n",
					    d_read(ctx, D8895_RESOURCE_OCCUPANCY_INFO_0),
					    d_read(ctx, D8895_RESOURCE_OCCUPANCY_INFO_1),
					    d_read(ctx, D8895_RESOURCE_OCCUPANCY_INFO_2));
		}
		if (ext & D8895_DPU_TIME_OUT_INT_PEND) {
			d_write(ctx, D8895_EXTRA_INTERRUPT_PENDING, D8895_DPU_TIME_OUT_INT_PEND);
			atomic_inc(&ctx->to_cnt);
			greatlte_l5_inc(GL5_ON, to);
			/* hibernation stays off until the next programmed update */
			if (D8895_HIBER_ON && READ_ONCE(ctx->hiber_init))
				WRITE_ONCE(ctx->hiber_timeout, true);
			dev_err_ratelimited(ctx->dev, "timeout irq\n");
		}
		if (ext & D8895_DPU_ERROR_INT_PEND) {
			d_write(ctx, D8895_EXTRA_INTERRUPT_PENDING, D8895_DPU_ERROR_INT_PEND);
			atomic_inc(&ctx->er_cnt);
			greatlte_l5_inc(GL5_ON, er);
		}
	}

	return IRQ_HANDLED;
}

static int d8895_ch_of_irq(struct decon8895_ctx *ctx, int irq, bool dma)
{
	int ch;

	for (ch = 0; ch < D8895_NR_CH; ch++)
		if ((dma ? ctx->irq_dma[ch] : ctx->irq_dpp[ch]) == irq)
			return ch;
	return -1;
}

/* dma_irq_handler(): error causes only (S:.../dpu/dpp_drv.c:776-850) */
static irqreturn_t d8895_dma_irq(int irq, void *dev_id)
{
	struct decon8895_ctx *ctx = dev_id;
	int ch = d8895_ch_of_irq(ctx, irq, true);
	void __iomem *dma;
	u32 st, cfg = 0;

	if (ch < 0)
		return IRQ_NONE;
	dma = d_dma(ctx, ch);
	st = readl(dma + D8895_IDMA_IRQ) & D8895_IDMA_ALL_IRQ_CLEAR;
	if (st & D8895_IDMA_CONFIG_ERROR)
		cfg = readl(dma + D8895_IDMA_CFG_ERR_STATE);
	io_mask(dma + D8895_IDMA_IRQ, ~0, st);
	if (st & (D8895_IDMA_READ_SLAVE_ERROR | D8895_IDMA_STATUS_DEADLOCK_IRQ |
		  D8895_IDMA_CONFIG_ERROR | D8895_IDMA_AFBC_TIMEOUT_IRQ)) {
		atomic_inc(&ctx->ude_cnt);
		greatlte_l5_inc(GL5_ON, ude);
		dev_err_ratelimited(ctx->dev, "%s dma irq 0x%08x cfg_err 0x%03x\n",
				    d8895_ch[ch].name, st, D8895_IDMA_CFG_ERR_GET(cfg));
	}

	return IRQ_HANDLED;
}

/* dpp_irq_handler() (S:.../dpu/dpp_drv.c:852-915) */
static irqreturn_t d8895_dpp_irq(int irq, void *dev_id)
{
	struct decon8895_ctx *ctx = dev_id;
	int ch = d8895_ch_of_irq(ctx, irq, false);
	void __iomem *dpp;
	u32 st, cfg = 0;

	if (ch < 0)
		return IRQ_NONE;
	dpp = d_dpp(ctx, ch);
	st = readl(dpp + D8895_DPP_IRQ) & D8895_DPP_ALL_IRQ_CLEAR;
	if (st & D8895_DPP_CONFIG_ERROR)
		cfg = readl(dpp + D8895_DPP_CFG_ERR_STATE);
	io_mask(dpp + D8895_DPP_IRQ, ~0, st);
	if (st & D8895_DPP_CONFIG_ERROR) {
		atomic_inc(&ctx->dpe_cnt);
		greatlte_l5_inc(GL5_ON, dpe);
		dev_err_ratelimited(ctx->dev, "%s dpp config error 0x%02x\n",
				    d8895_ch[ch].name, D8895_DPP_CFG_ERR_GET(cfg));
	}

	return IRQ_HANDLED;
}

static int d8895_get_irq(struct decon8895_ctx *ctx, const char *name, irq_handler_t h,
			 bool optional)
{
	struct platform_device *pdev = to_platform_device(ctx->dev);
	int irq, ret;

	irq = optional ? platform_get_irq_byname_optional(pdev, name) :
			 platform_get_irq_byname(pdev, name);
	if (irq < 0)
		return (optional && irq == -ENXIO) ? 0 : irq;
	ret = devm_request_irq(ctx->dev, irq, h, IRQF_NO_AUTOEN, "exynos8895-decon", ctx);
	if (ret)
		return dev_err_probe(ctx->dev, ret, "IRQ %s request failed\n", name);
	return irq;
}

/* ---- component ----------------------------------------------------------- */

static int d8895_bind(struct device *dev, struct device *master, void *data)
{
	struct decon8895_ctx *ctx = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	int ch, ret;

	ctx->drm = drm;

	for (ch = 0; ch < D8895_NR_CH; ch++) {
		ret = d8895_plane_init(ctx, ch);
		if (ret)
			return ret;
	}

	ctx->crtc = exynos_drm_crtc_create(drm, &ctx->planes[D8895_CH_G0].base, EXYNOS_DISPLAY_TYPE_LCD,
					   &d8895_crtc_ops, ctx);
	if (IS_ERR(ctx->crtc))
		return PTR_ERR(ctx->crtc);
	ctx->crtc->i80_mode = true;	/* FDT:10081 mode = <0x02> (command mode) */

	/*
	 * Bus votes (S:dpu/bts.c:425-427 dpu_bts_init(): the three pm_qos
	 * requests at 0; only the DISP one exists here, MIF/INT go through
	 * the BTS core, and the DP-only requests are L6G's). The class lists
	 * are static, so binding before the bus driver probed is safe.
	 */
	exynos_bus_qos_add_request(&ctx->disp_req, EXYNOS_BUS_QOS_DISP, 0);
	ctx->disp_qos_added = true;
	/*
	 * No channel clearing on handover: the trigger is already masked and
	 * clearing would blank S-Boot's logo in the panel's frame memory.
	 * Hand the boot framebuffer's aperture over from simplefb, if any.
	 */
	if (ctx->fb_res.end)
		aperture_remove_conflicting_devices(ctx->fb_res.start,
						    resource_size(&ctx->fb_res), "exynos");

	ret = exynos_drm_register_dma(drm, dev, &ctx->dma_priv);
	if (ret) {
		exynos_bus_qos_remove_request(&ctx->disp_req);
		ctx->disp_qos_added = false;
	}
	return ret;
}

static void d8895_unbind(struct device *dev, struct device *master, void *data)
{
	struct decon8895_ctx *ctx = dev_get_drvdata(dev);

	if (ctx->rpm_held)
		d8895_atomic_disable(ctx->crtc);
if (ctx->disp_qos_added) {	/* S:dpu/bts.c:429-435 dpu_bts_deinit() */
		exynos_bus_qos_remove_request(&ctx->disp_req);
		ctx->disp_qos_added = false;
	}
	d8895_panic_free(ctx);
	exynos_drm_unregister_dma(ctx->drm, ctx->dev, &ctx->dma_priv);
	ctx->crtc = NULL;	/* the client interface refuses an unbound DECON */
}

static const struct component_ops d8895_component_ops = {
	.bind	= d8895_bind,
	.unbind	= d8895_unbind,
};

/* ---- interface for in-kernel clients (exynos8895_drm_decon.h) ----------- */

static struct decon8895_ctx *d8895_ctx_of(struct device *dev)
{
	struct decon8895_ctx *ctx = dev ? dev_get_drvdata(dev) : NULL;

	return ctx && ctx->crtc ? ctx : NULL;
}

/*
 * The bound DECON of @np (NULL: the first samsung,exynos8895-decon node):
 * DRM device, CRTC, the six planes in enum decon_idma_type order and the
 * connector of the output that can be driven from this CRTC (L6A REQ-L5-7).
 */
int exynos8895_decon_get(struct device_node *np, struct exynos8895_decon_handle *h)
{
	struct drm_connector_list_iter iter;
	struct drm_connector *connector;
	struct platform_device *pdev;
	struct decon8895_ctx *ctx;
	struct device_node *node = np;
	int i, ret = 0;

	if (!node)
		node = of_find_compatible_node(NULL, NULL, "samsung,exynos8895-decon");
	pdev = node ? of_find_device_by_node(node) : NULL;
	if (!np)
		of_node_put(node);
	if (!pdev)
		return -ENODEV;

	ctx = d8895_ctx_of(&pdev->dev);
	if (!ctx) {
		ret = -EPROBE_DEFER;
		goto out;
	}

	memset(h, 0, sizeof(*h));
	h->dev = ctx->dev;
	h->drm = ctx->drm;
	h->crtc = &ctx->crtc->base;
	for (i = 0; i < D8895_NR_CH; i++)
		h->planes[i] = &ctx->planes[i].base;

	drm_connector_list_iter_begin(ctx->drm, &iter);
	drm_for_each_connector_iter(connector, &iter) {
		struct drm_encoder *encoder;

		drm_connector_for_each_possible_encoder(connector, encoder) {
			if (encoder->possible_crtcs & drm_crtc_mask(h->crtc)) {
				h->connector = connector;
				break;
			}
		}
		if (h->connector)
			break;
	}
	drm_connector_list_iter_end(&iter);
	if (!h->connector)
		ret = -ENODEV;
out:
	put_device(&pdev->dev);
	return ret;
}

/*
 * The window (0..5, or -1 to derive it from zpos) and the AFBC flag of a
 * plane in an atomic state being built by a client (L6A REQ-L5-3, the AFBC
 * part of REQ-L5-2); atomic_check validates both.
 */
int exynos8895_decon_plane_set_win(struct drm_plane_state *st, int win, bool afbc)
{
	struct d8895_plane_state *ps;

	if (!st || st->plane->funcs != &d8895_plane_funcs ||
	    win < -1 || win >= D8895_NR_WIN)
		return -EINVAL;
	ps = to_d8895_pstate(st);
	ps->win = win;
	ps->afbc = afbc;

	return 0;
}

/*
 * Colour window @win (@cw == NULL: none) for the next commit; a window a
 * visible plane uses is a buffer window whatever is set here (L6A REQ-L5-4).
 */
int exynos8895_decon_set_color_win(struct device *dev, unsigned int win,
				   const struct exynos8895_color_win *cw)
{
	struct decon8895_ctx *ctx = d8895_ctx_of(dev);
	unsigned long flags;

	if (!ctx || win >= D8895_NR_WIN || (cw && cw->plane >= D8895_NR_CH))
		return -EINVAL;
	if (cw && (drm_rect_width(&cw->dst) < D8895_IMG_WIDTH_MIN ||
		   drm_rect_height(&cw->dst) <= 0 || cw->dst.x1 < 0 || cw->dst.y1 < 0))
		return -EINVAL;

	spin_lock_irqsave(&ctx->cw_lock, flags);
	if (cw) {
		ctx->cw[win] = *cw;
		ctx->cw_on |= BIT(win);
	} else {
		ctx->cw_on &= ~BIT(win);
	}
	spin_unlock_irqrestore(&ctx->cw_lock, flags);

	return 0;
}

/*
 * Called from the TE interrupt handler (hard interrupt context: keep it
 * short, no sleeping) with the TE time, which is where the vendor
 * timestamps vsync (L6A REQ-L5-5); @cb NULL removes it.
 */
void exynos8895_decon_set_vsync_cb(struct device *dev,
				   void (*cb)(void *data, ktime_t ts), void *data)
{
	struct decon8895_ctx *ctx = d8895_ctx_of(dev);
	unsigned long flags;

	if (!ctx)
		return;
	spin_lock_irqsave(&ctx->vsync_lock, flags);
	ctx->vsync_cb = cb;
	ctx->vsync_data = data;
	spin_unlock_irqrestore(&ctx->vsync_lock, flags);
}

/*
 * Called from the DECON interrupt handler (hard interrupt context) when the
 * first frame after a flush starts, i.e. its shadow registers latched and the
 * DMA reads the new buffers: the point where decon_update_regs() is done
 * waiting (decon_wait_for_vsync() and decon_wait_for_vstatus(),
 * S:.../dpu/decon_core.c:2279-2283) and releases
 * the old buffers (:2336-2338). One-shot mode only; @cb NULL removes it.
 */
void exynos8895_decon_set_frame_start_cb(struct device *dev,
					 void (*cb)(void *data), void *data)
{
	struct decon8895_ctx *ctx = d8895_ctx_of(dev);
	unsigned long flags;

	if (!ctx)
		return;
	spin_lock_irqsave(&ctx->vsync_lock, flags);
	ctx->fs_cb = cb;
	ctx->fs_data = data;
	spin_unlock_irqrestore(&ctx->vsync_lock, flags);
}

/*
 * One-shot trigger (L6A REQ-L5-8, O-6A-13): with @oneshot, each commit sends
 * its frame and the HW trigger is masked again after it (the panel refreshes
 * from its frame memory), as the vendor driver ran; without it, the DECON
 * sends a frame on every TE.
 */
void exynos8895_decon_set_oneshot(struct device *dev, bool oneshot)
{
	struct decon8895_ctx *ctx = d8895_ctx_of(dev);

	if (ctx)
		WRITE_ONCE(ctx->oneshot, oneshot);
}

/*
 * Stage a DSU mode switch (@dsu == NULL clears any pending update) for the
 * next atomic_flush (S:.../dpu/dsu_update.c:73-78, 114-156).
 */
void exynos8895_decon_set_dsu(struct device *dev, const struct dsu_info *dsu)
{
	struct decon8895_ctx *ctx = d8895_ctx_of(dev);
	unsigned long flags;

	if (!ctx)
		return;
	spin_lock_irqsave(&ctx->dsu_lock, flags);
	if (dsu && dsu->needupdate)
		ctx->pending_dsu = *dsu;
	else
		ctx->pending_dsu.needupdate = 0;
	spin_unlock_irqrestore(&ctx->dsu_lock, flags);
}

/* ---- probe / PM ---------------------------------------------------------- */

static int d8895_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct decon8895_ctx *ctx;
	struct device_node *mr;
	u32 gc;
	int i, ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = dev;
	spin_lock_init(&ctx->vblank_lock);
	spin_lock_init(&ctx->cw_lock);
	spin_lock_init(&ctx->dsu_lock);
	spin_lock_init(&ctx->trig_lock);
	spin_lock_init(&ctx->vsync_lock);
	/* initial DSU mode 1 (1440x2960, S:.../dpu/decon_core.c:4223-4228) */
	ctx->dsu.left = 0;
	ctx->dsu.top = 0;
	ctx->dsu.right = 1440;
	ctx->dsu.bottom = 2960;
	ctx->dsu.mode = DSU_MODE_1;
	platform_set_drvdata(pdev, ctx);

	/* 1. resources */
	ctx->regs = devm_platform_ioremap_resource_byname(pdev, "decon");
	if (IS_ERR(ctx->regs))
		return PTR_ERR(ctx->regs);
	ctx->dpp = devm_platform_ioremap_resource_byname(pdev, "dpp");
	if (IS_ERR(ctx->dpp))
		return PTR_ERR(ctx->dpp);
	ctx->dma = devm_platform_ioremap_resource_byname(pdev, "dma");
	if (IS_ERR(ctx->dma))
		return PTR_ERR(ctx->dma);
	ctx->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,disp-sysreg");
	if (IS_ERR(ctx->sysreg))
		return dev_err_probe(dev, PTR_ERR(ctx->sysreg), "no samsung,disp-sysreg\n");
	mr = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (mr) {
		if (of_address_to_resource(mr, 0, &ctx->fb_res))
			memset(&ctx->fb_res, 0, sizeof(ctx->fb_res));
		of_node_put(mr);
	}

	/* 2. clocks before any register access */
	for (i = 0; i < ARRAY_SIZE(d8895_clk_names); i++)
		ctx->clks[i].id = d8895_clk_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(ctx->clks), ctx->clks);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get clocks\n");
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(ctx->clks), ctx->clks);
	if (ret)
		return ret;

	/*
	 * 3. S-Boot left DECON0 running: mask the HW trigger as the vendor
	 * takeover does (decon_reg.c:2039-2043) and let an in-flight frame end
	 * (two frames at 60 Hz) before any SysMMU can turn translation on.
	 */
	gc = d_read(ctx, D8895_GLOBAL_CONTROL);
	if (gc & D8895_GLOBAL_CONTROL_RUN_STATUS) {
		d8895_trigger(ctx, false);
		msleep(34);
		ctx->handover = true;
		dev_info(dev, "S-Boot left DECON running (GLOBAL_CONTROL=0x%08x), HW trigger masked\n",
			 gc);
	} else {
		dev_info(dev, "DECON idle at probe (GLOBAL_CONTROL=0x%08x)\n", gc);
	}

	/* 4. runtime PM: keep pd_dpu0 (and the SysMMUs through the device links) on */
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	clk_bulk_disable_unprepare(ARRAY_SIZE(ctx->clks), ctx->clks);
	if (ret < 0)
		goto err_rpm;
	ctx->rpm_held = true;

	/*
	 * 4b. hibernation (DESIGN 2.5; decon_register_hiber_work(),
	 * S:.../dpu/decon_dsi.c:1231-1250): the lock, counters and work before
	 * any IRQ that queues it; the workqueue fails the probe like stock.
	 */
	if (D8895_HIBER_ON) {
		struct device_node *cam;

		mutex_init(&ctx->hiber_lock);
		INIT_WORK(&ctx->hiber_work, d8895_hiber_work);
		atomic_set(&ctx->hiber_trig, 0);
		atomic_set(&ctx->hiber_block, 0);
		ctx->hiber_wq = alloc_ordered_workqueue("decon_hiber", 0);
		if (!ctx->hiber_wq) {
			ret = -ENOMEM;
			goto err_put;
		}
		/* CAM running status for the enter check (decon_core.c:3869-3876) */
		cam = of_get_child_by_name(dev->of_node, "cam-stat");
		if (!cam) {
			dev_info(dev, "no cam-stat node: hibernation stays off\n");
		} else {
			ctx->hiber_cam = of_iomap(cam, 0);
			of_node_put(cam);
			if (!ctx->hiber_cam)
				dev_info(dev, "no cam-stat mapping: hibernation stays off\n");
		}
	}

	/* 5. IRQs, requested disabled */
	ret = d8895_get_irq(ctx, "frame_start", d8895_irq, false);
	if (ret < 0)
		goto err_put;
	ctx->irq_fs = ret;
	ret = d8895_get_irq(ctx, "frame_done", d8895_irq, false);
	if (ret < 0)
		goto err_put;
	ctx->irq_fd = ret;
	ret = d8895_get_irq(ctx, "extra", d8895_irq, false);
	if (ret < 0)
		goto err_put;
	ctx->irq_extra = ret;
	/*
	 * "<ch>_dma" and "<ch>_dpp" of every channel (vendor order dma, dpp:
	 * S:.../dpu/dpp_drv.c:1033-1065); G0's are required, a channel without
	 * its interrupts in the DT still scans out, its errors are not counted.
	 */
	for (i = 0; i < D8895_NR_CH; i++) {
		char name[16];

		snprintf(name, sizeof(name), "%s_dma", d8895_ch[i].name);
		ret = d8895_get_irq(ctx, name, d8895_dma_irq, i != D8895_CH_G0);
		if (ret < 0)
			goto err_put;
		ctx->irq_dma[i] = ret;
		snprintf(name, sizeof(name), "%s_dpp", d8895_ch[i].name);
		ret = d8895_get_irq(ctx, name, d8895_dpp_irq, i != D8895_CH_G0);
		if (ret < 0)
			goto err_put;
		ctx->irq_dpp[i] = ret;
	}

	/* 6. component */
	ret = component_add(dev, &d8895_component_ops);
	if (ret)
		goto err_put;

	greatlte_l5_set(GL5_ON, decon_regs, ctx->regs);
	greatlte_l5_set(GL5_ON, decon_dma, ctx->dma);
	greatlte_l5_set(GL5_ON, ho, ctx->handover);
	greatlte_l5_set(GL5_ON, decon_dev, dev);

	if (D8895_HIBER_ON && ctx->hiber_wq && ctx->hiber_cam) {
		WRITE_ONCE(ctx->hiber_init, true);
		WRITE_ONCE(d8895_hiber_ctx, ctx);
	}

	return 0;

err_put:
	ctx->rpm_held = false;
	pm_runtime_put_sync(dev);
err_rpm:
	pm_runtime_disable(dev);
	if (D8895_HIBER_ON && ctx->hiber_wq)
		destroy_workqueue(ctx->hiber_wq);
	return ret;
}

static void d8895_remove(struct platform_device *pdev)
{
	struct decon8895_ctx *ctx = platform_get_drvdata(pdev);

	if (D8895_HIBER_ON && READ_ONCE(ctx->hiber_init)) {
		WRITE_ONCE(ctx->hiber_init, false);
		WRITE_ONCE(d8895_hiber_ctx, NULL);
		cancel_work_sync(&ctx->hiber_work);
		destroy_workqueue(ctx->hiber_wq);
		iounmap(ctx->hiber_cam);
	}
	greatlte_l5_set(GL5_ON, decon_dev, NULL);
	greatlte_l5_set(GL5_ON, decon_regs, NULL);
	greatlte_l5_set(GL5_ON, decon_dma, NULL);
	component_del(&pdev->dev, &d8895_component_ops);
	if (ctx->rpm_held)
		pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static int d8895_runtime_suspend(struct device *dev)
{
	struct decon8895_ctx *ctx = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(ARRAY_SIZE(ctx->clks), ctx->clks);
	return 0;
}

static int d8895_runtime_resume(struct device *dev)
{
	struct decon8895_ctx *ctx = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(ctx->clks), ctx->clks);
	if (ret)
		return ret;

	/*
	 * SYSREG_DPU0 BUS_COMPONENT_DRCG_EN: 0xffffffff at lpm_init and after
	 * every pd_dpu0 power-on (CAL:pmucal_system_exynos8895.h:103,
	 * CAL:pmucal_local_exynos8895.h:230); the DECON owns the word here.
	 */
	regmap_write(ctx->sysreg, D8895_SYSREG_DRCG_EN, 0xffffffff);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(d8895_pm_ops, d8895_runtime_suspend, d8895_runtime_resume,
				 NULL);

static const struct of_device_id d8895_of_match[] = {
	{ .compatible = "samsung,exynos8895-decon" },
	{ }
};
MODULE_DEVICE_TABLE(of, d8895_of_match);

struct platform_driver exynos8895_decon_driver = {
	.probe		= d8895_probe,
	.remove		= d8895_remove,
	.driver		= {
		.name	= "exynos8895-decon",
		.pm	= pm_ptr(&d8895_pm_ops),
		.of_match_table = d8895_of_match,
	},
};
