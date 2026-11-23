// SPDX-License-Identifier: GPL-2.0-only
/*
 * sound/soc/samsung/abox/abox_util.c - Samsung ABOX utility
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 */

#include <linux/errno.h>
#include <linux/io.h>
#include <linux/printk.h>

#include "abox_util.h"

#define ABOX_CHIPID_BASE	0x10000000
#define ABOX_CHIPID_SIZE	0x14
#define ABOX_CHIPID_WORD_0	0x0
#define ABOX_CHIPID_WORD_10	0x10
#define ABOX_CHIPID_8895	0xE8895000

static bool abox_secure_gic;

int abox_probe_secure_gic(void)
{
	void __iomem *base;
	u32 w0, w10;

	/* The chipid driver holds the region request, so map it plainly */
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
