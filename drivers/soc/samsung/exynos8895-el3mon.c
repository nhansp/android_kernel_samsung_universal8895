// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 8895 (greatlte) EL3 monitor hooks the stock kernel registers on every boot:
 * S:drivers/soc/samsung/exynos-el3_mon.c, built for every ARCH_EXYNOS kernel
 * (S:drivers/soc/samsung/Makefile:10). Ported:
 *  - SMC_CMD_SET_DEBUG_MEM (-120, S:include/linux/smc.h:27): one page the monitor may
 *    write debug data into (S:drivers/soc/samsung/exynos-el3_mon.c:83-114)
 *  - SMC_CMD_SET_SEH_ADDRESS (-123, :30): the EL1 "smart exception handler" the monitor
 *    jumps to on an unexpected exception in EL3 or in a LDFW (:219-270)
 *  - SMC_CMD_GET_LOCKUP_REASON (-121, :28): per-CPU records of cores stuck in the
 *    monitor (:116-216; stock asks from its hard-lockup detector)
 * Not ported: SMC_CMD_PROTECT_KERNEL_TEXT (-125, CONFIG_EXYNOS_KERNEL_PROTECTION, :17-80),
 * a kernel-protection feature (out of scope).
 *
 * Deviations (DEBUG, L7 deep suspend: the SLEEP SMC never returns and leaves nothing):
 *  - the debug page and the lockup buffer are a fixed reserved region (DT el3-debug@91808000)
 *    instead of kmalloc() pages, and the debug page as the previous boot left it is copied
 *    (and its non-zero rows printed) before it is cleared and handed to the monitor again,
 *    so whatever the monitor wrote before a hang and a forced reset can be read back;
 *  - the SEH handler panics instead of BUG() (:258), so the report always reaches pstore.
 * el3=1 prints the live debug page, el3=2 the previous boot's copy, el3=3 asks
 * GET_LOCKUP_REASON (/sys/module/exynos8895_el3mon/parameters/el3).
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/panic.h>
#include <linux/printk.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/cacheflush.h>
#include <soc/samsung/exynos8895-smc.h>

#define SMC_CMD_SET_DEBUG_MEM		(-120)	/* S:include/linux/smc.h:27 */
#define SMC_CMD_GET_LOCKUP_REASON	(-121)	/* :28 */
#define SMC_CMD_SET_SEH_ADDRESS		(-123)	/* :30 */

#define E8895_EL3_PAGE		SZ_4K	/* stock PAGE_SIZE (IKC:403 CONFIG_ARM64_4K_PAGES) */
#define E8895_EL3_NR_CPUS	8	/* stock NR_CPUS (IKC:421): the monitor's record count */

/* S:drivers/soc/samsung/exynos-el3_mon.c:123-133 struct __exception_info / __lockup_info */
struct e8895_el3_exc {
	unsigned long exception_type;
	unsigned long sp_el1;
	unsigned long sp_el3;
	unsigned long elr_el3;
	unsigned long esr_el3;
};

static u8 *e8895_el3_dbg;		/* debug page, cacheable like stock's kmalloc() page */
static struct e8895_el3_exc *e8895_el3_lockup;	/* lockup buffer, the next page */
static phys_addr_t e8895_el3_pa;
static u8 *e8895_el3_last;		/* the debug page as the previous boot left it */

/*
 * S:drivers/soc/samsung/exynos-el3_mon.c:219-260. Entered from the monitor by an ERET
 * to EL1 with the arguments in x0-x6, never called by the kernel.
 */
static void __noreturn __used e8895_el3_seh(unsigned int id, unsigned long elr,
					    unsigned long esr, unsigned long sctlr,
					    unsigned long ttbr, unsigned long tcr,
					    unsigned long x6)
{
	if (id) {
		pr_emerg("exynos8895-el3: unexpected exception from a LDFW, smc id 0x%x\n", id);
		pr_emerg("exynos8895-el3: elr_el1 0x%016lx esr_el1 0x%016lx sctlr_el1 0x%016lx\n",
			 elr, esr, sctlr);
		pr_emerg("exynos8895-el3: ttbr_el1 0x%016lx tcr_el1 0x%016lx lr 0x%016lx\n",
			 ttbr, tcr, x6);
	} else {
		pr_emerg("exynos8895-el3: unexpected exception from the EL3 monitor\n");
		pr_emerg("exynos8895-el3: elr_el3 0x%016lx esr_el3 0x%016lx sctlr_el3 0x%016lx\n",
			 elr, esr, sctlr);
		pr_emerg("exynos8895-el3: ttbr_el3 0x%016lx tcr_el3 0x%016lx scr_el3 0x%016lx\n",
			 ttbr, tcr, x6);
	}
	panic("exynos8895-el3: monitor exception");
}

static void e8895_el3_rows(const char *tag, const u8 *p)
{
	unsigned int off;

	for (off = 0; off < E8895_EL3_PAGE; off += 32)
		if (memchr_inv(p + off, 0, 32))
			pr_info("exynos8895-el3: %s +%03x: %32ph\n", tag, off, p + off);
}

/* S:drivers/soc/samsung/exynos-el3_mon.c:180-216 exynos_check_hardlockup_reason() */
static void e8895_el3_lockup_query(void)
{
	unsigned long ret, etype;
	int i, n = 0;

	memset(e8895_el3_lockup, 0, E8895_EL3_PAGE);
	dcache_clean_inval_poc((unsigned long)e8895_el3_lockup,
			       (unsigned long)e8895_el3_lockup + E8895_EL3_PAGE);
	ret = exynos8895_smc(SMC_CMD_GET_LOCKUP_REASON, e8895_el3_pa + E8895_EL3_PAGE,
			     E8895_EL3_PAGE, 0);
	if (ret) {
		pr_info("exynos8895-el3: GET_LOCKUP_REASON returned 0x%lx\n", ret);
		return;
	}
	/* S:drivers/soc/samsung/exynos-el3_mon.c:142-178 exynos_parse_reason() */
	for (i = 0; i < E8895_EL3_NR_CPUS; i++) {
		etype = e8895_el3_lockup[i].exception_type;
		if (!etype)
			continue;
		n++;
		pr_emerg("exynos8895-el3: core %d stuck in the monitor: type 0x%lx (from %s)\n",
			 i, etype, (etype >> 63) & 1 ? "EL3" : "EL1");
		pr_emerg("exynos8895-el3: elr_el3 0x%lx sp_el1 0x%lx sp_el3 0x%lx esr_el3 0x%lx\n",
			 e8895_el3_lockup[i].elr_el3, e8895_el3_lockup[i].sp_el1,
			 e8895_el3_lockup[i].sp_el3, e8895_el3_lockup[i].esr_el3);
	}
	pr_info("exynos8895-el3: GET_LOCKUP_REASON: %d core(s) stuck in the monitor\n", n);
}

static int e8895_el3_set(const char *val, const struct kernel_param *kp)
{
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);

	if (ret)
		return ret;
	if (!e8895_el3_dbg)
		return -ENODEV;
	switch (v) {
	case 1:
		e8895_el3_rows("live", e8895_el3_dbg);
		break;
	case 2:
		if (e8895_el3_last)
			e8895_el3_rows("last", e8895_el3_last);
		break;
	case 3:
		e8895_el3_lockup_query();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct kernel_param_ops e8895_el3_ops = { .set = e8895_el3_set };
module_param_cb(el3, &e8895_el3_ops, NULL, 0200);

static int __init e8895_el3_init(void)
{
	struct reserved_mem *rmem = NULL;
	struct device_node *np;
	unsigned int i, used = 0;
	unsigned long ret;

	/* S:drivers/soc/samsung/exynos-el3_mon.c:262-280 exynos_set_seh_address() */
	ret = exynos8895_smc(SMC_CMD_SET_SEH_ADDRESS, (unsigned long)e8895_el3_seh, 0, 0);
	pr_info("exynos8895-el3: SEH %ps @%px: %s (0x%lx)\n", e8895_el3_seh, e8895_el3_seh,
		ret ? "refused" : "ok", ret);

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos8895-el3-debug");
	if (np)
		rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem || rmem->size < 2 * E8895_EL3_PAGE) {
		pr_err("exynos8895-el3: no el3-debug reserved region, no debug page\n");
		return 0;
	}
	e8895_el3_pa = rmem->base;
	e8895_el3_dbg = memremap(rmem->base, 2 * E8895_EL3_PAGE, MEMREMAP_WB);
	if (!e8895_el3_dbg) {
		pr_err("exynos8895-el3: cannot map %pa\n", &e8895_el3_pa);
		return 0;
	}
	e8895_el3_lockup = (struct e8895_el3_exc *)(e8895_el3_dbg + E8895_EL3_PAGE);

	/* what the monitor left there before the reset that started this boot */
	e8895_el3_last = kmemdup(e8895_el3_dbg, E8895_EL3_PAGE, GFP_KERNEL);
	for (i = 0; i < E8895_EL3_PAGE; i++)
		used += !!e8895_el3_dbg[i];
	pr_info("exynos8895-el3: debug page %pa: %u non-zero bytes from the previous boot\n",
		&e8895_el3_pa, used);
	if (used && e8895_el3_last)
		e8895_el3_rows("last", e8895_el3_last);

	/* S:drivers/soc/samsung/exynos-el3_mon.c:93-113 exynos_set_debug_mem() */
	memset(e8895_el3_dbg, 0, 2 * E8895_EL3_PAGE);
	dcache_clean_inval_poc((unsigned long)e8895_el3_dbg,
			       (unsigned long)e8895_el3_dbg + 2 * E8895_EL3_PAGE);
	ret = exynos8895_smc(SMC_CMD_SET_DEBUG_MEM, e8895_el3_pa, E8895_EL3_PAGE, 0);
	/* "correct return value is input size" (:106) */
	pr_info("exynos8895-el3: debug mem %pa: %s (0x%lx)\n", &e8895_el3_pa,
		ret == E8895_EL3_PAGE ? "ok" : "refused", ret);
	return 0;
}
arch_initcall(e8895_el3_init);
