/* linux/arch/arm/mach-exynos/include/mach/secos_booster.h
*
* Copyright (c) 2014 Samsung Electronics Co., Ltd.
*              http://www.samsung.com/
*
* Header file for secure OS booster API
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/


#ifndef __LINUX_SECOS_BOOST_H__
#define __LINUX_SECOS_BOOST_H__

/*
 * Secure OS Boost Policy
 */
enum secos_boost_policy {
	MAX_PERFORMANCE,
	MID_PERFORMANCE,
	MIN_PERFORMANCE,
	STB_PERFORMANCE,
	PERFORMANCE_MAX_CNT,
};

#ifdef CONFIG_SECURE_OS_BOOSTER_API
int secos_booster_start(enum secos_boost_policy policy);
int secos_booster_stop(void);
#else
/* L6T 5.4: callers (L6F fingerprint CPU_SPEEDUP) build without the booster */
static inline int secos_booster_start(enum secos_boost_policy policy)
{
	return 0;
}

static inline int secos_booster_stop(void)
{
	return 0;
}
#endif

#endif
