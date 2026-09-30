// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L2b (bus DVFS) heartbeat rows, the MIF/DISP hardware read-back and
 * the INT test request (spec L2b 1.2, 5.8). DEBUG ONLY.
 *
 * The read-back decodes the CMU registers the ECT DVFS clock lists name
 * (ECT-DVFS:52 MIF, :94 DISP) into the ECT level, as stock's
 * exynos_devfreq_get_freq sysfs did through cal_dfs_get_rate() (table match,
 * CAL:vclk.c:211-245). It works at every bus stage, 0 included.
 *
 * Citation keys as in the spec: ECT-DVFS = work/scratch/L2_power/ect_dvfs_asv_dump.txt,
 * ECT-PLL = work/scratch/L2_power/ect_pll_dump.txt (L2's ect.py on sboot.bin),
 * CAL = S:drivers/soc/samsung/cal-if/.
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <soc/samsung/bts.h>
#include <soc/samsung/exynos8895-bus-qos.h>

/* Register addresses: SFR_BLOCK CMU 0x15a80000 / MIF 0x16000000 (CAL:exynos8895/cmucal-sfr.c:13,31) */
#define L2B_PLL_CON0_PLL_MIF		0x16000100	/* :1571 */
#define L2B_MUX_MIF_DDRPHY2X		0x16001000	/* :1573 */
#define L2B_PLL_CON0_PLL_SHARED1	0x15a80140	/* :480 */
#define L2B_MUX_CLKCMU_CORE_BUS		0x15a81034	/* :591 */
#define L2B_DIV_CLKCMU_CORE_BUS		0x15a81834	/* :589 */
#define L2B_MUX_CLKCMU_DPU_BUS		0x15a8104c	/* :478 */
#define L2B_DIV_CLKCMU_DPU_BUS		0x15a8184c	/* :479 */

/* Fields (CAL:exynos8895/cmucal-sfr.c) */
#define L2B_PLL_ENABLE		BIT(31)		/* PLL_CON0_PLL_MIF_ENABLE :6557 */
#define L2B_PLL_M		GENMASK(25, 16)	/* DIV_M :6555, :3394 */
#define L2B_PLL_P		GENMASK(13, 8)	/* DIV_P :6554, :3393 */
#define L2B_PLL_S		GENMASK(2, 0)	/* DIV_S :6556, :3395 */
#define L2B_MUX_SELECT		GENMASK(2, 0)	/* CORE_BUS :3715, DPU_BUS :3389 */
#define L2B_DIV_RATIO		GENMASK(3, 0)	/* CORE_BUS :3709, DPU_BUS :3392 */

struct l2b_pms {
	u16 p, m, s;
};

/* ECT PLL_MIF entries 0..11 (ECT-PLL:45-56); entry 12 is P = M = S = 0 (:57), see l2b_pll_mif_idx() */
static const struct l2b_pms l2b_pll_mif[] = {
	{ 3, 483, 0 }, { 3, 462, 0 }, { 3, 414, 0 }, { 5, 592, 0 },
	{ 3, 312, 0 }, { 3, 468, 1 }, { 3, 390, 1 }, { 3, 312, 1 },
	{ 3, 504, 2 }, { 4, 518, 2 }, { 3, 264, 2 }, { 3, 384, 3 },
};

/* ECT PLL_SHARED1 entries 0..11 (ECT-PLL:59-70); entries 0..4 are identical */
static const struct l2b_pms l2b_pll_shared1[] = {
	{ 4, 287, 0 }, { 4, 287, 0 }, { 4, 287, 0 }, { 4, 287, 0 },
	{ 4, 287, 0 }, { 3, 234, 0 }, { 3, 195, 0 }, { 3, 156, 0 },
	{ 3, 252, 1 }, { 4, 259, 1 }, { 3, 264, 2 }, { 3, 192, 2 },
};

/* ECT dvfs_mif L0..L11: kHz, [PLL_MIF idx, PLL_SHARED1 idx, CORE_BUS mux, CORE_BUS div] (ECT-DVFS:53-64) */
static const struct {
	u32 khz;
	u8 pll_mif, pll_shared1, mux, div;
} l2b_mif_lv[] = {
	{ 2093000, 0, 0, 0, 0 }, { 2002000, 1, 0, 0, 0 }, { 1794000, 2, 0, 1, 0 },
	{ 1540000, 3, 0, 2, 0 }, { 1352000, 4, 0, 3, 0 }, { 1014000, 5, 0, 4, 0 },
	{ 845000, 12, 6, 5, 0 }, { 676000, 12, 7, 5, 0 }, { 546000, 12, 8, 5, 0 },
	{ 421000, 12, 9, 5, 0 }, { 286000, 12, 10, 5, 0 }, { 208000, 12, 11, 5, 0 },
};

/* ECT dvfs_disp L0..L4: kHz, [DPU_BUS mux, DPU_BUS div] (ECT-DVFS:95-99) */
static const struct {
	u32 khz;
	u8 mux, div;
} l2b_disp_lv[] = {
	{ 630000, 1, 0 }, { 533000, 0, 1 }, { 356000, 0, 2 }, { 214000, 0, 3 }, { 134000, 0, 5 },
};

static void __iomem *l2b_pll_mif_va, *l2b_ddrphy_va, *l2b_shared1_va;
static void __iomem *l2b_core_mux_va, *l2b_core_div_va;
static void __iomem *l2b_dpu_mux_va, *l2b_dpu_div_va;

static int l2b_pms_idx(u32 v, const struct l2b_pms *t, int n)
{
	int i;

	for (i = 0; i < n; i++)
		if (FIELD_GET(L2B_PLL_P, v) == t[i].p && FIELD_GET(L2B_PLL_M, v) == t[i].m &&
		    FIELD_GET(L2B_PLL_S, v) == t[i].s)
			return i;
	return -1;
}

/*
 * PLL_MIF index 12 when the PLL is off, whatever P/M/S say: the ECT gives
 * entry 12 as P = M = S = 0 and levels L6..L11 use it, i.e. PLL_MIF is off
 * there. Inference (spec L2b 5.8, O-B6): stock's ra_get_pll_idx() compared
 * P/M/S only (CAL:ra.c:349-376).
 */
static int l2b_pll_mif_idx(u32 v)
{
	if (!(v & L2B_PLL_ENABLE))
		return 12;
	return l2b_pms_idx(v, l2b_pll_mif, ARRAY_SIZE(l2b_pll_mif));
}

struct l2b_mif_hw {
	u32 pll_mif, ddrphy, shared1, mux, div;
	int lv;
};

static void l2b_mif_read(struct l2b_mif_hw *h)
{
	int pm, ps, i;

	h->pll_mif = readl(l2b_pll_mif_va);
	h->ddrphy = readl(l2b_ddrphy_va);
	h->shared1 = readl(l2b_shared1_va);
	h->mux = FIELD_GET(L2B_MUX_SELECT, readl(l2b_core_mux_va));
	h->div = FIELD_GET(L2B_DIV_RATIO, readl(l2b_core_div_va));

	pm = l2b_pll_mif_idx(h->pll_mif);
	/* the first entry with these P/M/S (spec L2b 5.8) */
	ps = l2b_pms_idx(h->shared1, l2b_pll_shared1, ARRAY_SIZE(l2b_pll_shared1));

	h->lv = -1;
	for (i = 0; i < ARRAY_SIZE(l2b_mif_lv); i++) {
		if (l2b_mif_lv[i].pll_mif == pm && l2b_mif_lv[i].pll_shared1 == ps &&
		    l2b_mif_lv[i].mux == h->mux && l2b_mif_lv[i].div == h->div) {
			h->lv = i;
			break;
		}
	}
}

struct l2b_disp_hw {
	u32 mux_raw, div_raw;
	int lv;
};

static void l2b_disp_read(struct l2b_disp_hw *h)
{
	u32 mux, div;
	int i;

	h->mux_raw = readl(l2b_dpu_mux_va);
	h->div_raw = readl(l2b_dpu_div_va);
	mux = FIELD_GET(L2B_MUX_SELECT, h->mux_raw);
	div = FIELD_GET(L2B_DIV_RATIO, h->div_raw);

	h->lv = -1;
	for (i = 0; i < ARRAY_SIZE(l2b_disp_lv); i++) {
		if (l2b_disp_lv[i].mux == mux && l2b_disp_lv[i].div == div) {
			h->lv = i;
			break;
		}
	}
}

static u32 l2b_mif_khz(int lv)
{
	return lv < 0 ? 0 : l2b_mif_lv[lv].khz;
}

static u32 l2b_disp_khz(int lv)
{
	return lv < 0 ? 0 : l2b_disp_lv[lv].khz;
}

/* FN "mifsfr": PLL_MIF/DDRPHY2X/PLL_SHARED1 raw, CORE_BUS mux/div, level, kHz */
static int l2b_hb_mifsfr(char *buf, size_t size)
{
	struct l2b_mif_hw h;

	l2b_mif_read(&h);
	return scnprintf(buf, size, "%08x/%08x/%08x/%x/%x/%d/%u", h.pll_mif, h.ddrphy,
			 h.shared1, h.mux, h.div, h.lv, l2b_mif_khz(h.lv));
}

/* FN "dispsfr": DPU_BUS mux/div raw, level, kHz */
static int l2b_hb_dispsfr(char *buf, size_t size)
{
	struct l2b_disp_hw h;

	l2b_disp_read(&h);
	return scnprintf(buf, size, "%x/%x/%d/%u", h.mux_raw, h.div_raw, h.lv,
			 l2b_disp_khz(h.lv));
}

static const struct greatlte_hb_item l2b_rows[] = {
	/* spec L2b 4.1 nodes, driver "exynos8895-bus" */
	{ .layer = 2, .tag = "bmif", .kind = GREATLTE_HB_DEV, .path = "/bus-mif" },
	{ .layer = 2, .tag = "bint", .kind = GREATLTE_HB_DEV, .path = "/bus-int" },
	{ .layer = 2, .tag = "bicam", .kind = GREATLTE_HB_DEV, .path = "/bus-intcam" },
	{ .layer = 2, .tag = "bcam", .kind = GREATLTE_HB_DEV, .path = "/bus-cam" },
	{ .layer = 2, .tag = "bdisp", .kind = GREATLTE_HB_DEV, .path = "/bus-disp" },
	/* the ACPM DVFS clock's cached rate = the last FREQ_REQ (WT:drivers/clk/samsung/clk-acpm.c:396-412) */
	{ .layer = 2, .tag = "fmif", .kind = GREATLTE_HB_CLK, .path = "/bus-mif", .idx = 0 },
	{ .layer = 2, .tag = "fint", .kind = GREATLTE_HB_CLK, .path = "/bus-int", .idx = 0 },
	{ .layer = 2, .tag = "ficam", .kind = GREATLTE_HB_CLK, .path = "/bus-intcam", .idx = 0 },
	{ .layer = 2, .tag = "fcam", .kind = GREATLTE_HB_CLK, .path = "/bus-cam", .idx = 0 },
	{ .layer = 2, .tag = "fdisp", .kind = GREATLTE_HB_CLK, .path = "/bus-disp", .idx = 0 },
	{ .layer = 2, .tag = "bus", .kind = GREATLTE_HB_FN, .fn = exynos8895_bus_hb },
#if IS_ENABLED(CONFIG_EXYNOS8895_BTS)
	/* BTS register state (spec L2b 1.2); "off" without bts_exynos8895.program=1 */
	{ .layer = 2, .tag = "bts", .kind = GREATLTE_HB_FN, .fn = bts_hb },
#endif
	{ .layer = 2, .tag = "mifsfr", .kind = GREATLTE_HB_FN, .fn = l2b_hb_mifsfr },
	{ .layer = 2, .tag = "dispsfr", .kind = GREATLTE_HB_FN, .fn = l2b_hb_dispsfr },
};

/* debugfs exynos8895-bus/{mif,disp}_hw_khz: a fresh read on demand (spec L2b 5.8, V-2) */
static int mif_hw_khz_show(struct seq_file *s, void *unused)
{
	struct l2b_mif_hw h;

	l2b_mif_read(&h);
	seq_printf(s, "%u lv=%d raw=%08x/%08x/%08x/%x/%x\n", l2b_mif_khz(h.lv), h.lv,
		   h.pll_mif, h.ddrphy, h.shared1, h.mux, h.div);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mif_hw_khz);

static int disp_hw_khz_show(struct seq_file *s, void *unused)
{
	struct l2b_disp_hw h;

	l2b_disp_read(&h);
	seq_printf(s, "%u lv=%d raw=%x/%x\n", l2b_disp_khz(h.lv), h.lv, h.mux_raw, h.div_raw);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(disp_hw_khz);

/*
 * int_test_khz (0644, default 0): one INT class request set to the written
 * value (spec L2b 8.2 T-B2a).
 */
static DEFINE_MUTEX(l2b_int_test_lock);
static struct exynos_bus_qos_request l2b_int_test_req;
static int l2b_int_test_khz;

static int l2b_int_test_set(const char *val, const struct kernel_param *kp)
{
	int khz, ret;

	ret = kstrtoint(val, 0, &khz);
	if (ret)
		return ret;
	if (khz < 0)
		return -EINVAL;

	mutex_lock(&l2b_int_test_lock);
	if (!exynos_bus_qos_request_active(&l2b_int_test_req))
		exynos_bus_qos_add_request(&l2b_int_test_req, EXYNOS_BUS_QOS_INT, khz);
	else
		exynos_bus_qos_update_request(&l2b_int_test_req, khz);
	l2b_int_test_khz = khz;
	mutex_unlock(&l2b_int_test_lock);

	return 0;
}

static const struct kernel_param_ops l2b_int_test_ops = {
	.set = l2b_int_test_set,
	.get = param_get_int,
};
module_param_cb(int_test_khz, &l2b_int_test_ops, &l2b_int_test_khz, 0644);
MODULE_PARM_DESC(int_test_khz, "DEBUG: INT bus floor request in kHz (0 = none)");

static void __iomem *l2b_map(phys_addr_t pa)
{
	return ioremap(pa, sizeof(u32));
}

static int __init l2b_hb_init(void)
{
	struct dentry *dir;

	/* single words, read-only, kept mapped; CMU_MIF and CMU_TOP are always on while Linux runs */
	l2b_pll_mif_va = l2b_map(L2B_PLL_CON0_PLL_MIF);
	l2b_ddrphy_va = l2b_map(L2B_MUX_MIF_DDRPHY2X);
	l2b_shared1_va = l2b_map(L2B_PLL_CON0_PLL_SHARED1);
	l2b_core_mux_va = l2b_map(L2B_MUX_CLKCMU_CORE_BUS);
	l2b_core_div_va = l2b_map(L2B_DIV_CLKCMU_CORE_BUS);
	l2b_dpu_mux_va = l2b_map(L2B_MUX_CLKCMU_DPU_BUS);
	l2b_dpu_div_va = l2b_map(L2B_DIV_CLKCMU_DPU_BUS);
	if (!l2b_pll_mif_va || !l2b_ddrphy_va || !l2b_shared1_va || !l2b_core_mux_va ||
	    !l2b_core_div_va || !l2b_dpu_mux_va || !l2b_dpu_div_va) {
		pr_err("greatlte-hb-l2b: ioremap failed\n");
		/* the rows before mifsfr/dispsfr (the last two) need no mapping */
		greatlte_hb_add_items(l2b_rows, ARRAY_SIZE(l2b_rows) - 2);
		return 0;
	}

	greatlte_hb_add_items(l2b_rows, ARRAY_SIZE(l2b_rows));

	dir = debugfs_create_dir("exynos8895-bus", NULL);
	debugfs_create_file("mif_hw_khz", 0400, dir, NULL, &mif_hw_khz_fops);
	debugfs_create_file("disp_hw_khz", 0400, dir, NULL, &disp_hw_khz_fops);

	return 0;
}
late_initcall(l2b_hb_init);
