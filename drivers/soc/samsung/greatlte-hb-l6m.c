// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6M (SMFC JPEG codec + MSCL scaler) heartbeat rows
 * (spec L6B_multimedia.md 5.10). DEBUG ONLY.
 *
 * Rows: smfc (bound state of jpeg@13b00000), mscl (bound state of
 * scaler@13b10000).
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

#define L6M_SMFC_PATH	"/soc@0/jpeg@13b00000"
#define L6M_MSCL_PATH	"/soc@0/scaler@13b10000"

static const struct greatlte_hb_item l6m_rows[] = {
	{ .layer = 6, .tag = "smfc", .kind = GREATLTE_HB_DEV, .path = L6M_SMFC_PATH },
	{ .layer = 6, .tag = "mscl", .kind = GREATLTE_HB_DEV, .path = L6M_MSCL_PATH },
};

static int __init l6m_hb_init(void)
{
	greatlte_hb_add_items(l6m_rows, ARRAY_SIZE(l6m_rows));
	return 0;
}
late_initcall(l6m_hb_init);
