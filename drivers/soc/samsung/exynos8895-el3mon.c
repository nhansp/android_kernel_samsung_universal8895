// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 EL3 monitor hooks the vendor kernel registers on every boot: a
 * page the monitor writes its debug data into, and an EL1 exception handler
 * the monitor jumps to on an unexpected exception in EL3 or in a loadable
 * firmware.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/panic.h>
#include <linux/printk.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <asm/cacheflush.h>
#include <soc/samsung/exynos8895-smc.h>

#define SMC_CMD_SET_DEBUG_MEM		(-120)
#define SMC_CMD_SET_SEH_ADDRESS		(-123)

#define E8895_EL3_PAGE		SZ_4K

/* Entered from the monitor by an ERET to EL1, with the arguments in x0-x6 */
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
	/* the vendor handler uses BUG(); panic() always reaches the crash log */
	panic("exynos8895-el3: monitor exception");
}

static int __init e8895_el3_init(void)
{
	struct reserved_mem *rmem = NULL;
	struct device_node *np;
	phys_addr_t pa;
	unsigned long ret;
	u8 *page;

	ret = exynos8895_smc(SMC_CMD_SET_SEH_ADDRESS, (unsigned long)e8895_el3_seh, 0, 0);
	if (ret)
		pr_warn("exynos8895-el3: exception handler refused (0x%lx)\n", ret);

	/* a reserved region, unlike the vendor's kmalloc() page: readable after a reset */
	np = of_find_compatible_node(NULL, NULL, "samsung,exynos8895-el3-debug");
	if (np)
		rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem || rmem->size < E8895_EL3_PAGE) {
		pr_err("exynos8895-el3: no el3-debug reserved region, no debug page\n");
		return 0;
	}
	pa = rmem->base;
	page = memremap(pa, E8895_EL3_PAGE, MEMREMAP_WB);
	if (!page) {
		pr_err("exynos8895-el3: cannot map %pa\n", &pa);
		return 0;
	}

	memset(page, 0, E8895_EL3_PAGE);
	dcache_clean_inval_poc((unsigned long)page, (unsigned long)page + E8895_EL3_PAGE);
	ret = exynos8895_smc(SMC_CMD_SET_DEBUG_MEM, pa, E8895_EL3_PAGE, 0);
	/* the monitor returns the size it accepted */
	if (ret != E8895_EL3_PAGE)
		pr_warn("exynos8895-el3: debug page %pa refused (0x%lx)\n", &pa, ret);
	return 0;
}
arch_initcall(e8895_el3_init);
