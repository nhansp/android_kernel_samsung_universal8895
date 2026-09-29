// SPDX-License-Identifier: GPL-2.0-only
/*
 * greatlte L6A slice ion (legacy ION ABI shim) heartbeat rows (spec L6A 8.1,
 * 5.2.8 item 5). DEBUG ONLY. Both rows are FN rows that read the shim's
 * counters lock-free (T_tooling 5.7.6 rule 4).
 */

#include <linux/atomic.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/soc/samsung/greatlte-hb.h>

#include "../../staging/android/ion_legacy/ion_legacy.h"

/* "ion": ion-legacy probe result, 0 or -errno; -ENODEV until a probe ran (L6A 8.1) */
static int l6a_ion_probe(char *buf, size_t size)
{
	return scnprintf(buf, size, "%d", READ_ONCE(ionl_stats.probe));
}

/* "ionb": live ION buffers, back at its idle value after T-6A-1 (L6A 1.2, 8.1) */
static int l6a_ion_buffers(char *buf, size_t size)
{
	return scnprintf(buf, size, "%d", atomic_read(&ionl_stats.buffers));
}

static const struct greatlte_hb_item l6a_ion_rows[] = {
	{ .layer = 6, .tag = "ion",  .kind = GREATLTE_HB_FN, .fn = l6a_ion_probe },
	{ .layer = 6, .tag = "ionb", .kind = GREATLTE_HB_FN, .fn = l6a_ion_buffers },
};

static int __init l6a_ion_hb_init(void)
{
	greatlte_hb_add_items(l6a_ion_rows, ARRAY_SIZE(l6a_ion_rows));
	return 0;
}
late_initcall(l6a_ion_hb_init);
