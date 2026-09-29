/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Entry points shared by the greatlte Wi-Fi glue (dhd_custom_exynos8895.c),
 * the static-buffer preallocator (dhd_custom_memprealloc.c) and dhd_linux.c
 * (L6D 5.3.2). The stock glue declared them as local externs
 * (S:.../dhd_custom_exynos.c:72); a shared header keeps the definitions
 * checked against one prototype (6.18 -Wmissing-prototypes).
 */
#ifndef _DHD_CUSTOM_EXYNOS8895_H_
#define _DHD_CUSTOM_EXYNOS8895_H_

int dhd_module_init(void);				/* dhd_linux.c, DHD_INIT_FROM_PLATFORM_GLUE */
int dhd_init_wlan_mem(void);				/* S:.../dhd_custom_memprealloc.c:287-288 */
void dhd_exit_wlan_mem(void);				/* :484-485 */
void *dhd_wlan_mem_prealloc(int section, unsigned long size);	/* :160-161 */

#endif /* _DHD_CUSTOM_EXYNOS8895_H_ */
