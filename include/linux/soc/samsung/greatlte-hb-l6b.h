/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * greatlte L6B heartbeat hooks (spec L6B 5.10). DEBUG ONLY.
 * The MFC and camera drivers report one-shot states here; the rows are in
 * drivers/soc/samsung/greatlte-hb-l6b.c. Without CONFIG_GREATLTE_HB_L6B the
 * hooks compile to nothing.
 */
#ifndef __LINUX_SOC_SAMSUNG_GREATLTE_HB_L6B_H
#define __LINUX_SOC_SAMSUNG_GREATLTE_HB_L6B_H

#include <linux/bitops.h>
#include <linux/compiler.h>
#include <linux/types.h>

/* fimcl bits (spec L6B 1.2 L6.10 table) */
#define GREATLTE_HB_L6B_FIMCL_WINDOW	0	/* library window mapped (5.6.1) */
#define GREATLTE_HB_L6B_FIMCL_DDK	1	/* DDK loaded + ROX */
#define GREATLTE_HB_L6B_FIMCL_DDK_UP	2	/* DDK start_up returned */
#define GREATLTE_HB_L6B_FIMCL_RTA	3	/* RTA loaded */
#define GREATLTE_HB_L6B_FIMCL_RTA_UP	4	/* RTA start_up returned */

#ifdef CONFIG_GREATLTE_HB_L6B
extern u32 greatlte_hb_l6b_mfcv;
extern unsigned long greatlte_hb_l6b_fimcl;

/* MFC: s5p_mfc_get_mfc_version() once the F/W version check passed */
static inline void greatlte_hb_l6b_set_mfcv(u32 ver)
{
	WRITE_ONCE(greatlte_hb_l6b_mfcv, ver);
}

static inline void greatlte_hb_l6b_set_fimcl(unsigned int bit)
{
	set_bit(bit, &greatlte_hb_l6b_fimcl);
}
#else
static inline void greatlte_hb_l6b_set_mfcv(u32 ver) { }
static inline void greatlte_hb_l6b_set_fimcl(unsigned int bit) { }
#endif

#endif /* __LINUX_SOC_SAMSUNG_GREATLTE_HB_L6B_H */
