/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * sound/soc/samsung/abox/abox_util.h - Samsung ABOX utility
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 */

#ifndef __SND_SOC_ABOX_UTIL_H
#define __SND_SOC_ABOX_UTIL_H

#include <linux/types.h>

/**
 * Read the chip id once (sleeping function, called from the ABOX probe)
 * @return	error code if any
 */
extern int abox_probe_secure_gic(void);

/**
 * Check whether the GIC is secure
 * @return	true if the GIC is secure, false on otherwise
 */
extern bool is_secure_gic(void);

#endif /* __SND_SOC_ABOX_UTIL_H */
