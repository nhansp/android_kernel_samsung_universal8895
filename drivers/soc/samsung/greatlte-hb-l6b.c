// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6B multimedia heartbeat rows (spec L6B 1.2, 5.10). DEBUG ONLY.
 *
 * mfc/mfci/mfcv (MFC), fimc/fimci/fss0..3/fimcl (camera), smfc, mscl and
 * busp (the BUSP DIVRATIO fields of the seven multimedia CMUs).
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/soc/samsung/greatlte-hb-l6b.h>

#include <dt-bindings/clock/samsung,exynos8895.h>

#define L6B_MFC_PATH	"/soc@0/codec@13ce0000"
#define L6B_FIMC_PATH	"/soc@0/fimc_is@13140000"
#define L6B_SMFC_PATH	"/soc@0/jpeg@13b00000"
#define L6B_MSCL_PATH	"/soc@0/scaler@13b10000"

u32 greatlte_hb_l6b_mfcv;
unsigned long greatlte_hb_l6b_fimcl;

/* lock-free: one word the MFC driver writes once */
static int l6b_mfcv_fn(char *buf, size_t size)
{
	return scnprintf(buf, size, "0x%x", READ_ONCE(greatlte_hb_l6b_mfcv));
}

/* lock-free: the bits the camera driver sets */
static int l6b_fimcl_fn(char *buf, size_t size)
{
	return scnprintf(buf, size, "0x%lx", READ_ONCE(greatlte_hb_l6b_fimcl));
}

/*
 * busp: the BUSP DIVRATIO field of each multimedia CMU, in the order of spec
 * L6B 5.10 (CAM, DCAM, G2D, ISPHQ, ISPLP, MFC, SRDZ; expected 1,2,1,1,1,3,1).
 * The value is what the clock framework read back from the divider register
 * when the CMU probe set the ratio (exynos8895_cmu_set_busp(), the divider's
 * recalc after clk_set_rate()): parent rate / rate - 1 of the cached rates,
 * taken once at late_initcall_sync; no register is read here, so a CMU whose
 * domain is off is never touched. '-' = CMU absent or disabled.
 */
static const struct {
	const char *path;
	unsigned int id;
} l6b_busp[] __initconst = {
	{ "/soc@0/clock-controller@12c00000", CLK_DOUT_CAM_BUSP },
	{ "/soc@0/clock-controller@12e00000", CLK_DOUT_DCAM_BUSP },
	{ "/soc@0/clock-controller@13a00000", CLK_DOUT_G2D_BUSP },
	{ "/soc@0/clock-controller@13100000", CLK_DOUT_ISPHQ_BUSP },
	{ "/soc@0/clock-controller@13000000", CLK_DOUT_ISPLP_BUSP },
	{ "/soc@0/clock-controller@13c00000", CLK_DOUT_MFC_BUSP },
	{ "/soc@0/clock-controller@14200000", CLK_DOUT_SRDZ_BUSP },
};

static char l6b_busp_text[GREATLTE_HB_VALUE] = "-";

static int l6b_busp_fn(char *buf, size_t size)
{
	return scnprintf(buf, size, "%s", l6b_busp_text);
}

static const struct greatlte_hb_item l6b_rows[] = {
	/* L6.8 MFC: "mfc0" alias (FDT:4234), SPI 208 (FDT:8307) */
	{ .layer = 6, .tag = "mfc",   .kind = GREATLTE_HB_DEV, .path = L6B_MFC_PATH },
	{ .layer = 6, .tag = "mfci",  .kind = GREATLTE_HB_IRQ, .path = L6B_MFC_PATH, .idx = 0 },
	{ .layer = 6, .tag = "mfcv",  .kind = GREATLTE_HB_FN, .fn = l6b_mfcv_fn },
	/* L6.10 camera: IRQ idx 0 = SPI 294 "3AAW_0" (FDT:8529) */
	{ .layer = 6, .tag = "fimc",  .kind = GREATLTE_HB_DEV, .path = L6B_FIMC_PATH },
	{ .layer = 6, .tag = "fimci", .kind = GREATLTE_HB_IRQ, .path = L6B_FIMC_PATH, .idx = 0 },
	{ .layer = 6, .tag = "fss0",  .kind = GREATLTE_HB_DEV, .path = "/soc@0/fimc_is_sensor@12ca0000" },
	{ .layer = 6, .tag = "fss1",  .kind = GREATLTE_HB_DEV, .path = "/soc@0/fimc_is_sensor@12cb0000" },
	{ .layer = 6, .tag = "fss2",  .kind = GREATLTE_HB_DEV, .path = "/soc@0/fimc_is_sensor@12cc0000" },
	{ .layer = 6, .tag = "fss3",  .kind = GREATLTE_HB_DEV, .path = "/soc@0/fimc_is_sensor@12cd0000" },
	{ .layer = 6, .tag = "fimcl", .kind = GREATLTE_HB_FN, .fn = l6b_fimcl_fn },
	/* L6.10 JPEG and scaler */
	{ .layer = 6, .tag = "smfc",  .kind = GREATLTE_HB_DEV, .path = L6B_SMFC_PATH },
	{ .layer = 6, .tag = "mscl",  .kind = GREATLTE_HB_DEV, .path = L6B_MSCL_PATH },
	{ .layer = 6, .tag = "busp",  .kind = GREATLTE_HB_FN, .fn = l6b_busp_fn },
};

static void __init l6b_busp_capture(void)
{
	struct of_phandle_args spec = { .args_count = 1 };
	size_t len = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(l6b_busp); i++) {
		unsigned long rate = 0, prate = 0;
		struct clk *clk;

		spec.np = of_find_node_by_path(l6b_busp[i].path);
		spec.args[0] = l6b_busp[i].id;
		clk = spec.np && of_device_is_available(spec.np) ?
			of_clk_get_from_provider(&spec) : ERR_PTR(-ENODEV);
		of_node_put(spec.np);

		if (!IS_ERR(clk)) {
			rate = clk_get_rate(clk);
			prate = clk_get_rate(clk_get_parent(clk));
			clk_put(clk);
		}

		if (rate && prate)
			len += scnprintf(l6b_busp_text + len,
					 sizeof(l6b_busp_text) - len, "%s%lu",
					 i ? "," : "", prate / rate - 1);
		else
			len += scnprintf(l6b_busp_text + len,
					 sizeof(l6b_busp_text) - len, "%s-",
					 i ? "," : "");
	}
}

static int __init l6b_hb_init(void)
{
	l6b_busp_capture();
	greatlte_hb_add_items(l6b_rows, ARRAY_SIZE(l6b_rows));
	return 0;
}
/* _sync: after the deferred probes of late_initcall, so the CMUs are bound */
late_initcall_sync(l6b_hb_init);
