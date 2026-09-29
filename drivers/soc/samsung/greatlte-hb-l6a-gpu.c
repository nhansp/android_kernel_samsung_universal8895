// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6A GPU (Mali-G71 kbase, pd_g3d, ACPM G3D clock) heartbeat rows
 * (spec L6A_graphics.md 1.2 L6A.1 (c), 8.1). DEBUG ONLY.
 *
 * Rows: gpu (bound driver), g3dhz (the clk_mali rate in Hz : enable count),
 * pdg3d (pd_g3d STATUS[3:0]), gpuj (JOB interrupt count). gpum/gpug (MMU and
 * GPU fault counts) need a counter inside kbase and are not rows here.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/soc/samsung/greatlte-hb.h>

#define L6A_GPU_PATH	"/soc@0/gpu@13900000"
#define L6A_PDG3D_PATH	"/soc@0/power-domain@164840e0"

/*
 * pd_g3d reg 0 = EMBEDDED_G3D_CONFIGURATION 0x164840e0; STATUS is at +0x4,
 * power state in [3:0]: 0xf on, 0x0 off (CAL:pmucal_local_exynos8895.h:362,370,
 * CAL: = S:drivers/soc/samsung/cal-if/exynos8895/).
 */
#define L6A_PD_STATUS		0x4
#define L6A_PD_STATUS_MASK	0xf

static void __iomem *l6a_pdg3d;

/* lock-free: one read of an always-on PMU register */
static int l6a_pdg3d_fn(char *buf, size_t size)
{
	void __iomem *base = READ_ONCE(l6a_pdg3d);

	if (!base)
		return -ENODEV;
	return scnprintf(buf, size, "0x%x",
			 readl_relaxed(base + L6A_PD_STATUS) & L6A_PD_STATUS_MASK);
}

static const struct greatlte_hb_item l6a_gpu_rows[] = {
	/* driver "mali": K:mali_kbase_strings.c:24,27 kbase_drv_name */
	{ .layer = 6, .tag = "gpu",   .kind = GREATLTE_HB_DEV, .path = L6A_GPU_PATH },
	/* ACPM DVFS id 4, read back from PLL_G3D (REQ-L2-6A-1) */
	{ .layer = 6, .tag = "g3dhz", .kind = GREATLTE_HB_CLK, .path = L6A_GPU_PATH,
	  .name = "clk_mali" },
	{ .layer = 6, .tag = "pdg3d", .kind = GREATLTE_HB_FN, .fn = l6a_pdg3d_fn },
	/* "JOB", SPI 60: FDT:4639-4640 */
	{ .layer = 6, .tag = "gpuj",  .kind = GREATLTE_HB_IRQ, .path = L6A_GPU_PATH, .idx = 0 },
};

static int __init l6a_gpu_hb_init(void)
{
	struct device_node *np = of_find_node_by_path(L6A_PDG3D_PATH);

	/* non-requesting map: the PD driver maps the same word with of_iomap() */
	if (np && of_device_is_available(np))
		WRITE_ONCE(l6a_pdg3d, of_iomap(np, 0));
	of_node_put(np);

	greatlte_hb_add_items(l6a_gpu_rows, ARRAY_SIZE(l6a_gpu_rows));
	return 0;
}
late_initcall(l6a_gpu_hb_init);
