/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOUND_SAMSUNG_ABOX_DSM_H
#define __SOUND_SAMSUNG_ABOX_DSM_H

#if IS_ENABLED(CONFIG_SND_SOC_SAMSUNG_ABOX)
int abox_dsm_set_rdc_temp(int rdc, int temp);
#else
static inline int abox_dsm_set_rdc_temp(int rdc, int temp) { return -ENODEV; }
#endif

#endif /* __SOUND_SAMSUNG_ABOX_DSM_H */
