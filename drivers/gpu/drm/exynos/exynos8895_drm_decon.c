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
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_vblank.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "regs-decon8895.h"
#include "regs-dpp8895.h"

#define D8895_NR_CH		6	/* G0 G1 VG0 VG1 VGF0 VGF1 */
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
#define D8895_RUN_TIMEOUT_US	(20 * 1000)	/* decon_reg.c:2105,2199 */
#define D8895_ACLK_KHZ_DEFAULT	630000		/* decon_core.c:1640 */
#define D8895_INIT_RCV_NUM	630000		/* S:.../dpu/dpp.h:45 */

/* S:.../dpu/decon.h:208-225 (8895 enum decon_win_func), :248-268 */
#define D8895_PD_FUNC_COPY		0x1
#define D8895_PD_FUNC_USER_DEFINED	0xd
#define D8895_BND_COEF_PLNAE_ALPHA0	0x6
#define D8895_BND_COEF_1_M_ALPHA_MULT	0xb
#define D8895_ALPHA_MULT_SRC_SEL_AF	2

/* S:.../dpu/decon.h:385, :509, :518, :201 */
#define D8895_DPATH_DSCC_DSCENC01_FF01_FORMATTER0_DSIMIF0	0x0b1
#define D8895_ENHANCEPATH_ENHANCE_ALL_OFF			0x0
#define D8895_SHAREPATH_DQE_USE					0x0
#define D8895_DECON_RGB						0x0

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
} d8895_ch[D8895_NR_CH] = {
	{ "g0", 5 }, { "g1", 3 }, { "vg0", 0 }, { "vg1", 4 }, { "vgf0", 1 }, { "vgf1", 2 },
};

static const char * const d8895_clk_names[] = { "aclk", "dpp_g0", "dma_sfr", "dma_ch1" };

/* DRM fourcc -> IDMA_IMG_FORMAT code (S:.../dpu/regs-dpp.h:107-117), alpha_sel */
struct d8895_fmt {
	u32 fourcc;
	u8 idma;
	u8 alpha;
};

/*
 * DRM names give the component order of a little-endian 32-bit word, the
 * vendor (Android HAL) names the byte order in memory: DRM ARGB8888 is bytes
 * B,G,R,A = HAL/DECON BGRA_8888 (S:.../dpu/dpp_reg.c:1279-1309).
 */
static const struct d8895_fmt d8895_rgb_formats[] = {
	{ DRM_FORMAT_ARGB8888, 3, 1 },	/* BGRA8888, dpp_reg.c:1291-1294 */
	{ DRM_FORMAT_XRGB8888, 7, 0 },	/* BGRX8888, :1304-1306 */
	{ DRM_FORMAT_ABGR8888, 2, 1 },	/* RGBA8888, :1287-1290 */
	{ DRM_FORMAT_XBGR8888, 6, 0 },	/* RGBX8888, :1301-1303 */
	{ DRM_FORMAT_RGB565, 8, 0 },	/* RGB565, :1307-1309 */
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
	bool irqs_on;
	bool vblank_on;
	bool first_frame_done;
	u32 ch_on;		/* channels initialised since the last enable */
	u32 ch_used;		/* channels of the frame last started */
	spinlock_t vblank_lock;

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

/* decon_reg_set_trigger(), HW trigger (S:.../dpu/decon_reg.c:1006-1024) */
static void d8895_trigger(struct decon8895_ctx *ctx, bool enable)
{
	d_mask(ctx, D8895_HW_SW_TRIG_CONTROL,
	       enable ? D8895_HW_TRIG_EN : D8895_HW_TRIG_MASK_DECON,
	       D8895_HW_TRIG_EN | D8895_HW_TRIG_MASK_DECON);
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

/* dpp_reg_init() (dpp_reg.c:1522-1540) + enable_irq (dpp_drv.c:558-563) */
static void d8895_dpp_init(struct decon8895_ctx *ctx, unsigned int ch)
{
	void __iomem *dma = d_dma(ctx, ch), *dpp = d_dpp(ctx, ch);

	if (ctx->ch_on & BIT(ch))
		return;

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

	for (i = 0; i < ARRAY_SIZE(d8895_rgb_formats); i++)
		if (d8895_rgb_formats[i].fourcc == fourcc)
			return &d8895_rgb_formats[i];
	return NULL;
}

/*
 * dpp_reg_configure_params() for an RGB buffer without scaling
 * (dpp_reg.c:1565-1577): CSC skipped on G channels (:939-940), scale ratio
 * and coefficients, sizes, flip, base address, block area, format.
 */
static void d8895_dpp_configure(struct decon8895_ctx *ctx, unsigned int ch,
				struct drm_plane_state *st)
{
	void __iomem *dma = d_dma(ctx, ch), *dpp = d_dpp(ctx, ch);
	struct drm_framebuffer *fb = st->fb;
	const struct d8895_fmt *f = d8895_find_fmt(fb->format->format);
	u32 sx = st->src.x1 >> 16, sy = st->src.y1 >> 16;
	u32 sw = drm_rect_width(&st->src) >> 16, sh = drm_rect_height(&st->src) >> 16;
	u32 dw = drm_rect_width(&st->dst), dh = drm_rect_height(&st->dst);
	u32 cpp = fb->format->cpp[0];

	/* ratio = (src << 20) / dst (S:.../dpu/dpp_drv.c:335-336) */
	d8895_dpp_set_scale(ctx, ch, (u32)(((u64)sw << 20) / dw), (u32)(((u64)sh << 20) / dh));

	/* sizes (dpp_reg.c:1363-1377): offset, full buffer, crop */
	writel(D8895_IDMA_SRC_OFFSET_Y(sy) | D8895_IDMA_SRC_OFFSET_X(sx),
	       dma + D8895_IDMA_SRC_OFFSET);
	writel(D8895_IDMA_SRC_HEIGHT(fb->height) | D8895_IDMA_SRC_WIDTH(fb->pitches[0] / cpp),
	       dma + D8895_IDMA_SRC_SIZE);
	writel(D8895_IDMA_IMG_HEIGHT(sh) | D8895_IDMA_IMG_WIDTH(sw), dma + D8895_IDMA_IMG_SIZE);
	writel(D8895_DPP_IMG_HEIGHT(sh) | D8895_DPP_IMG_WIDTH(sw), dpp + D8895_DPP_IMG_SIZE);

	/* no flip (:453-462) */
	io_mask(dma + D8895_IDMA_IN_CON, D8895_IDMA_IN_FLIP(0), D8895_IDMA_IN_FLIP_MASK);
	/* base address (:1351-1361, :628-637) */
	writel(lower_32_bits(exynos_drm_fb_dma_addr(fb, 0)), dma + D8895_IDMA_IN_BASE_ADDR_Y);
	writel(0, dma + D8895_IDMA_IN_BASE_ADDR_C);
	/* no block area (:1379-1384) */
	io_mask(dma + D8895_IDMA_IN_CON, 0, D8895_IDMA_BLOCK_EN);
	/* format (:1331-1333), no AFBC on G channels, HW recovery off (:1344-1346) */
	io_mask(dma + D8895_IDMA_IN_CON, D8895_IDMA_IMG_FORMAT(f->idma), D8895_IDMA_IMG_FORMAT_MASK);
	io_mask(dpp + D8895_DPP_IN_CON, D8895_DPP_ALPHA_SEL(f->alpha), D8895_DPP_ALPHA_SEL_MASK);
	io_mask(dpp + D8895_DPP_IN_CON, D8895_DPP_IMG_FORMAT(0), D8895_DPP_IMG_FORMAT_MASK);
	io_mask(dma + D8895_IDMA_RECOVERY_CTRL, 0, D8895_IDMA_RECOVERY_EN);
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
 * decon_reg_config_data_path_size() for dual DSC on DSIM_IF0
 * (decon_reg.c:596-645) with width = width_per_enc.
 */
static void d8895_data_path_size(struct decon8895_ctx *ctx, u32 width, u32 height)
{
	/* splitter (:252-270): width * 2, split index = width, no overlap */
	d_write(ctx, D8895_SPLITTER_SIZE_CONTROL_0,
		D8895_SPLITTER_HEIGHT_F(height) | D8895_SPLITTER_WIDTH_F(width * 2));
	d_write(ctx, D8895_SPLITTER_SIZE_CONTROL_1, height * width);
	d_write(ctx, D8895_SPLITTER_SPLIT_IDX_CONTROL, width);
	d_write(ctx, D8895_SPLITTER_OVERLAP_CONTROL, D8895_SPLITTER_OVERLAP_F(0));
	/* frame FIFO 0 and 1 (dual DSC) (:297-318) */
	d_write(ctx, D8895_FRAME_FIFO_0_SIZE_CONTROL_0,
		D8895_FRAME_FIFO_HEIGHT_F(height) | D8895_FRAME_FIFO_WIDTH_F(width));
	d_write(ctx, D8895_FRAME_FIFO_0_SIZE_CONTROL_1, height * width);
	d_mask(ctx, D8895_FRAME_FIFO_TH_CONTROL_0, D8895_FRAME_FIFO_0_TH_F(width),
	       D8895_FRAME_FIFO_0_TH_MASK);
	d_write(ctx, D8895_FRAME_FIFO_1_SIZE_CONTROL_0,
		D8895_FRAME_FIFO_HEIGHT_F(height) | D8895_FRAME_FIFO_WIDTH_F(width));
	d_write(ctx, D8895_FRAME_FIFO_1_SIZE_CONTROL_1, height * width);
	d_mask(ctx, D8895_FRAME_FIFO_TH_CONTROL_0, D8895_FRAME_FIFO_1_TH_F(width),
	       D8895_FRAME_FIFO_1_TH_MASK);
	/* formatter 0, dual DSC -> width * 2 (:403-420, :623-628) */
	d_write(ctx, D8895_FORMATTER0_SIZE_CONTROL_0,
		D8895_FORMATTER_HEIGHT_F(height) | D8895_FORMATTER_WIDTH_F(width * 2));
	d_write(ctx, D8895_FORMATTER0_SIZE_CONTROL_1, height * width * 2);
}

/*
 * decon_reg_init() full path for DECON0, MIPI command mode, HW trigger,
 * single DSI, dual DSC (decon_reg.c:2047-2094).
 */
static void d8895_hw_init(struct decon8895_ctx *ctx, const struct drm_display_mode *m,
			  const struct drm_dsc_config *dsc)
{
	u32 xres = m->hdisplay, yres = m->vdisplay, val;
	struct d8895_dsc d;
	int i;

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

	/* decon_reg_configure_lcd() (:1902-1965): RGB order with DSC (:1921-1925) */
	d_mask(ctx, D8895_FORMATTER_CONTROL, D8895_FORMATTER_OUT_RGB_ORDER_F(D8895_DECON_RGB),
	       D8895_FORMATTER_OUT_RGB_ORDER_MASK);
	d_mask(ctx, D8895_DATA_PATH_CONTROL_2,
	       D8895_ENHANCE_LOGIC_PATH_F(D8895_ENHANCEPATH_ENHANCE_ALL_OFF) |
	       D8895_COMP_LINKIF_WB_PATH_F(D8895_DPATH_DSCC_DSCENC01_FF01_FORMATTER0_DSIMIF0),
	       D8895_ENHANCE_LOGIC_PATH_MASK | D8895_COMP_LINKIF_WB_PATH_MASK);
	d8895_dsc_calc(xres, yres, dsc->slice_count, dsc->slice_height, &d);
	for (i = 0; i < 2; i++)
		d8895_dsc_program(ctx, i ? D8895_DSC1_OFFSET : D8895_DSC0_OFFSET, &d);
	d8895_data_path_size(ctx, d.width_per_enc, yres);
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
 * decon_reg_set_window_control() for a buffer window fed by channel ch:
 * blending with stock's plane-alpha branch at plane alpha 0xff, position,
 * start time, pixel count, enable, colour map off, channel map.
 */
static void d8895_win_program(struct decon8895_ctx *ctx, unsigned int win, unsigned int ch,
			      struct drm_plane_state *st)
{
	const struct d8895_fmt *f = d8895_find_fmt(st->fb->format->format);
	u32 x = st->dst.x1, y = st->dst.y1;
	u32 w = drm_rect_width(&st->dst), h = drm_rect_height(&st->dst);

	/* decon_reg_set_win_bnd_function() (:2407-2459) */
	d_mask(ctx, D8895_WIN_CONTROL_0(win), D8895_WIN_ALPHA1_F(0) | D8895_WIN_ALPHA0_F(0xff),
	       D8895_WIN_ALPHA1_MASK | D8895_WIN_ALPHA0_MASK);
	d_mask(ctx, D8895_WIN_CONTROL_0(win),
	       D8895_WIN_ALPHA_MULT_SRC_SEL_F(D8895_ALPHA_MULT_SRC_SEL_AF),
	       D8895_WIN_ALPHA_MULT_SRC_SEL_MASK);
	if (f->alpha) {
		/* DRM default blend mode: premultiplied, plane alpha (:2443-2447) */
		d_mask(ctx, D8895_WIN_CONTROL_0(win), D8895_WIN_FUNC_F(D8895_PD_FUNC_USER_DEFINED),
		       D8895_WIN_FUNC_MASK);
		d_mask(ctx, D8895_WIN_CONTROL_1(win),
		       D8895_WIN_FG_ALPHA_D_SEL_F(D8895_BND_COEF_PLNAE_ALPHA0) |
		       D8895_WIN_BG_ALPHA_D_SEL_F(D8895_BND_COEF_1_M_ALPHA_MULT) |
		       D8895_WIN_FG_ALPHA_A_SEL_F(D8895_BND_COEF_PLNAE_ALPHA0) |
		       D8895_WIN_BG_ALPHA_A_SEL_F(D8895_BND_COEF_1_M_ALPHA_MULT),
		       D8895_WIN_FG_ALPHA_D_SEL_MASK | D8895_WIN_BG_ALPHA_D_SEL_MASK |
		       D8895_WIN_FG_ALPHA_A_SEL_MASK | D8895_WIN_BG_ALPHA_A_SEL_MASK);
	} else {
		/* DECON_BLENDING_NONE -> PD_FUNC_COPY (:2419-2420) */
		d_mask(ctx, D8895_WIN_CONTROL_0(win), D8895_WIN_FUNC_F(D8895_PD_FUNC_COPY),
		       D8895_WIN_FUNC_MASK);
	}

	/* S:.../dpu/decon.h:1427-1435 */
	d_write(ctx, D8895_WIN_START_POSITION(win),
		D8895_WIN_STRPTR_Y_F(y) | D8895_WIN_STRPTR_X_F(x));
	d_write(ctx, D8895_WIN_END_POSITION(win),
		D8895_WIN_ENDPTR_Y_F(y + h - 1) | D8895_WIN_ENDPTR_X_F(x + w - 1));
	d_write(ctx, D8895_WIN_START_TIME_CONTROL(win), 0);
	d_write(ctx, D8895_WIN_PIXEL_COUNT(win), w * h);

	d_mask(ctx, D8895_DATA_PATH_CONTROL_0, ~0, D8895_WIN_EN_F(win));
	/* colour map off, colour 0 (decon_reg_set_winmap(), :2329-2340) */
	d_mask(ctx, D8895_DATA_PATH_CONTROL_0, 0, D8895_WIN_MAPCOLOR_EN_F(win));
	d_mask(ctx, D8895_WIN_COLORMAP_0(win), 0, D8895_WIN_MAPCOLOR_A_MASK | D8895_WIN_MAPCOLOR_R_MASK);
	d_mask(ctx, D8895_WIN_COLORMAP_1(win), 0, D8895_WIN_MAPCOLOR_G_MASK | D8895_WIN_MAPCOLOR_B_MASK);
	/* channel map (:923-934) */
	d_mask(ctx, D8895_DATA_PATH_CONTROL_1, D8895_WIN_CHMAP_F(win, d8895_ch[ch].chmap),
	       D8895_WIN_CHMAP_MASK(win));
}

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

	pinctrl_pm_select_default_state(ctx->dev);	/* hw_te_on */

	/* S-Boot's running DECON, or a previous start: stop it (decon_reg_stop) */
	if (d_read(ctx, D8895_GLOBAL_CONTROL) & D8895_GLOBAL_CONTROL_RUN_STATUS)
		d8895_stop(ctx);
	d8895_dpp_deinit(ctx, D8895_CH_G0, false);

	ctx->needs_init = true;
	ctx->handover = false;
	if (!ctx->irqs_on) {
		enable_irq(ctx->irq_fs);
		enable_irq(ctx->irq_fd);
		enable_irq(ctx->irq_extra);
		ctx->irqs_on = true;
	}
}

static void d8895_atomic_disable(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;
	int ch;

	if (ctx->rpm_held)
		d8895_int_disable(ctx);
	if (ctx->irqs_on) {
		disable_irq(ctx->irq_fs);
		disable_irq(ctx->irq_fd);
		disable_irq(ctx->irq_extra);
		ctx->irqs_on = false;
	}
	if (!ctx->rpm_held)
		return;
	d8895_stop(ctx);
	d8895_clear_int_all(ctx);
	for (ch = 0; ch < D8895_NR_CH; ch++)
		if (ctx->ch_on & BIT(ch))
			d8895_dpp_deinit(ctx, ch, false);

	pinctrl_pm_select_sleep_state(ctx->dev);	/* hw_te_off */
	if (ctx->rpm_held) {
		ctx->rpm_held = false;
		pm_runtime_put_sync(ctx->dev);
	}
}

/* wait for the shadow update, then mask the trigger (decon_core.c:1663-1672) */
static void d8895_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;
	u32 val;

	if (!ctx->rpm_held)	/* atomic_enable could not resume: never touch the block */
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

/* decon_reg_start() (decon_reg.c:2097-2111) */
static int d8895_start(struct decon8895_ctx *ctx)
{
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
	d8895_trigger(ctx, true);

	return 0;
}

static void d8895_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;
	struct drm_crtc_state *cs = crtc->base.state;
	struct drm_plane *plane;
	unsigned long flags, aclk_khz;
	u32 used_win = 0, used_ch = 0;
	int win, ch;

	if (!ctx->rpm_held)
		goto out_event;

	if (ctx->needs_init) {
		d8895_hw_init(ctx, &cs->adjusted_mode, crtc->dsc);
		ctx->needs_init = false;
		d8895_int_enable(ctx);
	}

	drm_for_each_plane_mask(plane, ctx->drm, cs->plane_mask) {
		struct d8895_plane *dp = to_d8895_plane(plane);
		struct drm_plane_state *st = plane->state;

		win = to_d8895_pstate(st)->hw_win;
		if (!st->visible || !st->fb || win < 0)
			continue;
		d8895_dpp_init(ctx, dp->ch);
		d8895_dpp_configure(ctx, dp->ch, st);
		d8895_win_program(ctx, win, dp->ch, st);
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
	for (win = 0; win < D8895_NR_WIN; win++)
		if (!(used_win & BIT(win)))
			d8895_win_disable(ctx, win);
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

	/* decon_reg_all_win_shadow_update_req() (decon_reg.c:825-838) */
	d_mask(ctx, D8895_SHADOW_REG_UPDATE_REQ, ~0, D8895_SHADOW_REG_UPDATE_REQ_FOR_DECON0);
	d8895_start(ctx);

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

/* HW-trigger mode: the TE only timestamps (S:.../dpu/decon_dsi.c:504-535) */
static void d8895_te_handler(struct exynos_drm_crtc *crtc)
{
	struct decon8895_ctx *ctx = crtc->ctx;

	atomic_inc(&ctx->te_cnt);
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

static int d8895_plane_atomic_check(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct drm_plane_state *st = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs;
	struct drm_framebuffer *fb = st->fb;
	u32 sw, sh;
	int ret;

	if (!st->crtc || !fb)
		return 0;

	cs = drm_atomic_get_new_crtc_state(state, st->crtc);
	if (!cs)
		return -EINVAL;

	/* G channels do not scale (S:.../dpu/dpp_drv.c:432-443) */
	ret = drm_atomic_helper_check_plane_state(st, cs, DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING, false, true);
	if (ret)
		return ret;
	if (!st->visible)
		return 0;

	if (!d8895_find_fmt(fb->format->format) || fb->modifier != DRM_FORMAT_MOD_LINEAR)
		return -EINVAL;

	/* dpp_check_size() limits (S:.../dpu/dpp.h:47-56, dpp_drv.c:351-394) */
	sw = drm_rect_width(&st->src) >> 16;
	sh = drm_rect_height(&st->src) >> 16;
	if (fb->pitches[0] % fb->format->cpp[0] ||
	    fb->pitches[0] / fb->format->cpp[0] > D8895_SRC_WIDTH_MAX ||
	    fb->height > D8895_SRC_HEIGHT_MAX ||
	    sw < D8895_IMG_WIDTH_MIN || sw > D8895_IMG_WIDTH_MAX ||
	    sh < D8895_IMG_HEIGHT_MIN || sh > D8895_IMG_HEIGHT_MAX)
		return -EINVAL;

	return 0;
}

static const struct drm_plane_helper_funcs d8895_plane_helper_funcs = {
	.atomic_check = d8895_plane_atomic_check,
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
	u32 formats[ARRAY_SIZE(d8895_rgb_formats)];
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(d8895_rgb_formats); i++)
		formats[i] = d8895_rgb_formats[i].fourcc;

	ret = drm_universal_plane_init(ctx->drm, &p->base, 1 << ctx->drm->mode_config.num_crtc,
				       &d8895_plane_funcs, formats,
				       ARRAY_SIZE(formats), NULL,
				       ch == D8895_CH_G0 ? DRM_PLANE_TYPE_PRIMARY :
							   DRM_PLANE_TYPE_OVERLAY,
				       "%s", d8895_ch[ch].name);
	if (ret)
		return ret;
	drm_plane_helper_add(&p->base, &d8895_plane_helper_funcs);
	p->ch = ch;
	ret = drm_plane_create_zpos_property(&p->base, zpos_default[ch], 0, D8895_NR_WIN - 1);
	if (ret)
		return ret;
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
	}
	if (pend & D8895_DPU_FRAME_DONE_INT_PEND) {
		d_write(ctx, D8895_INTERRUPT_PENDING, D8895_DPU_FRAME_DONE_INT_PEND);
		atomic_inc(&ctx->fd_cnt);
		if (READ_ONCE(ctx->vblank_on)) {
			spin_lock(&ctx->vblank_lock);
			drm_crtc_handle_vblank(&ctx->crtc->base);
			spin_unlock(&ctx->vblank_lock);
		}
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
			dev_err_ratelimited(ctx->dev, "resource conflict: %08x %08x %08x\n",
					    d_read(ctx, D8895_RESOURCE_OCCUPANCY_INFO_0),
					    d_read(ctx, D8895_RESOURCE_OCCUPANCY_INFO_1),
					    d_read(ctx, D8895_RESOURCE_OCCUPANCY_INFO_2));
		}
		if (ext & D8895_DPU_TIME_OUT_INT_PEND) {
			d_write(ctx, D8895_EXTRA_INTERRUPT_PENDING, D8895_DPU_TIME_OUT_INT_PEND);
			atomic_inc(&ctx->to_cnt);
			dev_err_ratelimited(ctx->dev, "timeout irq\n");
		}
		if (ext & D8895_DPU_ERROR_INT_PEND) {
			d_write(ctx, D8895_EXTRA_INTERRUPT_PENDING, D8895_DPU_ERROR_INT_PEND);
			atomic_inc(&ctx->er_cnt);
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
	 * No channel clearing on handover: the trigger is already masked and
	 * clearing would blank S-Boot's logo in the panel's frame memory.
	 * Hand the boot framebuffer's aperture over from simplefb, if any.
	 */
	if (ctx->fb_res.end)
		aperture_remove_conflicting_devices(ctx->fb_res.start,
						    resource_size(&ctx->fb_res), "exynos");

	return exynos_drm_register_dma(drm, dev, &ctx->dma_priv);
}

static void d8895_unbind(struct device *dev, struct device *master, void *data)
{
	struct decon8895_ctx *ctx = dev_get_drvdata(dev);

	if (ctx->rpm_held)
		d8895_atomic_disable(ctx->crtc);
	exynos_drm_unregister_dma(ctx->drm, ctx->dev, &ctx->dma_priv);
}

static const struct component_ops d8895_component_ops = {
	.bind	= d8895_bind,
	.unbind	= d8895_unbind,
};

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

	return 0;

err_put:
	ctx->rpm_held = false;
	pm_runtime_put_sync(dev);
err_rpm:
	pm_runtime_disable(dev);
	return ret;
}

static void d8895_remove(struct platform_device *pdev)
{
	struct decon8895_ctx *ctx = platform_get_drvdata(pdev);

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
