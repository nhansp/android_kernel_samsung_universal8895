/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ALSA SoC - Samsung VTS driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 */

#ifndef __VTS_H
#define __VTS_H

#include <linux/platform_device.h>

#ifdef CONFIG_SND_SOC_SAMSUNG_VTS
/**
 * Acquire SRAM in VTS
 * @param[in]	pdev	pointer to struct platform_device of a VTS device
 * @param[in]	vts	1, if requester is vts. 0 for others.
 * @return		error code if any
 */
extern int vts_acquire_sram(struct platform_device *pdev, int vts);

/**
 * Release SRAM in VTS
 * @param[in]	pdev	pointer to struct platform_device of a VTS device
 * @param[in]	vts	1, if requester is vts. 0 for others.
 * @return		error code if any
 */
extern int vts_release_sram(struct platform_device *pdev, int vts);

/**
 * Clear SRAM in VTS
 * @param[in]	pdev	pointer to struct platform_device of a VTS device
 * @return		error code if any
 */
extern int vts_clear_sram(struct platform_device *pdev);

/**
 * Check VTS is on
 * @return		true if VTS is on, false on otherwise
 */
extern bool vts_is_on(void);
extern bool vts_is_recognitionrunning(void);
#else /* !CONFIG_SND_SOC_SAMSUNG_VTS */
static inline int vts_acquire_sram(struct platform_device *pdev, int vts)
{ return -ENODEV; }
static inline int vts_release_sram(struct platform_device *pdev, int vts)
{ return -ENODEV; }
static inline int vts_clear_sram(struct platform_device *pdev) { return -ENODEV; }
static inline bool vts_is_on(void) { return false; }
static inline bool vts_is_recognitionrunning(void) { return false; }
#endif /* !CONFIG_SND_SOC_SAMSUNG_VTS */

#endif /* __VTS_H */
