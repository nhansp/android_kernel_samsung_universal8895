// SPDX-License-Identifier: GPL-2.0-only
/*
 * sound/soc/samsung/abox/abox_util.c - Samsung ABOX utility
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 *
 * Transplanted from S:sound/soc/samsung/abox/abox_util.c of the SM-N950N
 * 4.4 kernel (Samsung N950NKSU5DZE2 OSRC 4.4.111), spec L6E 5.7.2. The
 * resource and clock helpers are replaced by the 6.18 devm helpers at their
 * call sites; the chip-id test that is_secure_gic() did with an ioremap per
 * call is done once at ABOX probe.
 */

#include <linux/errno.h>
#include <linux/io.h>
#include <linux/printk.h>

#include "abox_util.h"

#define ABOX_CHIPID_BASE	0x10000000	/* stock readl_phys(0x10000000) :146 */
#define ABOX_CHIPID_SIZE	0x14		/* covers the word at 0x10000010 :147 */
#define ABOX_CHIPID_WORD_0	0x0
#define ABOX_CHIPID_WORD_10	0x10
#define ABOX_CHIPID_8895	0xE8895000	/* :146 */

static bool abox_secure_gic;

int abox_probe_secure_gic(void)
{
	void __iomem *base;
	u32 w0, w10;

	/*
	 * The ACK chipid driver owns 0x10000000 with a region request
	 * (exynos8895.dtsi chipid@10000000), so map without requesting it.
	 */
	base = ioremap(ABOX_CHIPID_BASE, ABOX_CHIPID_SIZE);
	if (!base)
		return -ENOMEM;

	w0 = readl(base + ABOX_CHIPID_WORD_0);
	w10 = readl(base + ABOX_CHIPID_WORD_10);
	iounmap(base);

	pr_debug("%s: %08x, %08x\n", __func__, w0, w10);
	abox_secure_gic = (w0 == ABOX_CHIPID_8895) && (w10 == 0x0);

	return 0;
}

bool is_secure_gic(void)
{
	return abox_secure_gic;
}
