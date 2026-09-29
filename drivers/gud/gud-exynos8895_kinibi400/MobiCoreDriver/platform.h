/*
 * Copyright (c) 2013-2018 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
/*
 * Header file of MobiCore Driver Kernel Module Platform
 * specific structures
 *
 * Internal structures of the McDrvModule
 *
 * Header file the MobiCore Driver Kernel Module,
 * its internal structures and defines.
 *
 * greatlte (Exynos 8895) only (L6T 5.1): the stock CONFIG_SOC_EXYNOS* chains
 * never match on 6.18; each value keeps its stock line as S:.../platform.h:N.
 * Dropped: MC_INTR_SSIQ (the S-SIQ comes from DT only, L6T 0 "IRQ"), IRQ_SPI and
 * MC_INTR_LOCAL_TIMER (MCT-disable only, not set), MC_PM_RUNTIME, MC_NO_UIDGIT_H
 * (other SoCs); CONFIG_TRUSTONIC_TEE_LPAE comes from Kconfig.
 */
#ifndef _MC_DRV_PLATFORM_H_
#define _MC_DRV_PLATFORM_H_

#define TBASE_CORE_SWITCHER			/* S:.../platform.h:49 */

#define COUNT_OF_CPUS 8				/* S:.../platform.h:56 */

/* Values of MPIDR regs (L0 numbering: cpu0-3 A53 0x100-0x103, cpu4-7 M2 0x0-0x3) */
#define CPU_IDS {0x0100, 0x0101, 0x0102, 0x0103, 0x0000, 0x0001, 0x0002, 0x0003}	/* S:.../platform.h:74 */

/* Enable Fastcall worker thread */
#define MC_FASTCALL_WORKER_THREAD		/* S:.../platform.h:94 */

/* Set Parameters for Secure OS Boosting */
#define DEFAULT_LITTLE_CORE		0	/* S:.../platform.h:97 */
#define NONBOOT_LITTLE_CORE		1	/* S:.../platform.h:98 */
#define DEFAULT_BIG_CORE		4	/* S:.../platform.h:99 */
#define MIGRATE_TARGET_CORE     DEFAULT_BIG_CORE	/* S:.../platform.h:100 */

#define LOCAL_TIMER_PERIOD             50	/* S:.../platform.h:104; sec_os_booster.c mc_timer_work_func() */

#define DEFAULT_SECOS_BOOST_TIME       5000	/* S:.../platform.h:106 */
#define MAX_SECOS_BOOST_TIME           600000  /* 600 sec, S:.../platform.h:107 */

#define DUMP_TBASE_HALT_STATUS			/* S:.../platform.h:109 */

#endif /* _MC_DRV_PLATFORM_H_ */
