// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Galaxy Note 8 (greatlte) DEBUG: program counter of every other CPU at
 * panic, sampled through the CoreSight external debug interface (EDPCSR).
 *
 * The Exynos 8895 GIC-400 has no pseudo-NMI, so a CPU that hangs with its
 * interrupts masked (a hard lockup) cannot be backtraced by an IPI. Stock sampled
 * its PC through the debug APB instead (S:drivers/trace/exynos-coresight.c:150-207,
 * CONFIG_EXYNOS_CORESIGHT_PC_INFO=y and CONFIG_PC_ITERATION=5, IKC:4711-4713).
 * This is that panic handler, plus a debugfs trigger to test it on a live system.
 * Never enable in a release build.
 */

#include <linux/cpumask.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kallsyms.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/panic_notifier.h>
#include <linux/printk.h>
#include <linux/smp.h>
#include <linux/spinlock.h>

#include <asm/cputype.h>
#include <asm/smp_plat.h>

/* S:arch/arm64/include/asm/core_regs.h:16-73 */
#define CSPC_OSLOCK_MAGIC	0xc5acce55
#define CSPC_SJTAG_OFFSET	0x8000		/* default of the "sj-offset" property */
#define CSPC_SJTAG_STATUS	0x4
#define CSPC_SJTAG_SOFT_LOCK	BIT(2)
#define CSPC_DBGPCSRLO		0x0a0
#define CSPC_DBGPCSRHI		0x0ac
#define CSPC_DBGOSLAR		0x300
#define CSPC_DBGLAR		0xfb0
#define CSPC_DBGDEVID1		0xfc4		/* PC offset field [3:0] */

/* S:drivers/trace/exynos-coresight.c:41-45 */
#define CSPC_ITERATION		5		/* CONFIG_PC_ITERATION, IKC:4713 */
#define CSPC_NR			8
#define CSPC_MSB_PADDING	0xFFFFFFC000000000UL
#define CSPC_MSB_MASKING	0x0001ffc000000000UL

/* debug block of physical core aff1 * 4 + aff0 (DT nodes cl<aff1>_cpu<aff0>) */
static void __iomem *cspc_base[CSPC_NR];
static unsigned long cspc_pc[NR_CPUS][CSPC_ITERATION];
static DEFINE_RAW_SPINLOCK(cspc_lock);

static void __iomem *cspc_base_of(unsigned int cpu)
{
	u64 mpidr = cpu_logical_map(cpu);
	unsigned int phys = MPIDR_AFFINITY_LEVEL(mpidr, 1) * 4 + MPIDR_AFFINITY_LEVEL(mpidr, 0);

	return phys < CSPC_NR ? cspc_base[phys] : NULL;
}

/* one EDPCSR sample, as S:drivers/trace/exynos-coresight.c:173-189 */
static unsigned long cspc_sample(void __iomem *base)
{
	unsigned long val;

	mb();
	__raw_writel(CSPC_OSLOCK_MAGIC, base + CSPC_DBGLAR);	/* DBG_UNLOCK, :33-34 */
	isb();
	mb();
	__raw_writel(0, base + CSPC_DBGOSLAR);			/* dbg_os_unlock, :87-98 */

	val = __raw_readl(base + CSPC_DBGPCSRLO);
	val |= (unsigned long)__raw_readl(base + CSPC_DBGPCSRHI) << 32;
	if (!(__raw_readl(base + CSPC_DBGDEVID1) & 0xf))	/* have_pc_offset(), :110-113 */
		val -= 0x8;

	__raw_writel(1, base + CSPC_DBGOSLAR);			/* dbg_os_lock, :74-85 */
	mb();
	isb();
	__raw_writel(1, base + CSPC_DBGLAR);			/* DBG_LOCK, :35-36 */
	mb();

	if ((val & CSPC_MSB_MASKING) == CSPC_MSB_MASKING)
		val |= CSPC_MSB_PADDING;
	return val;
}

/*
 * Every online CPU except the caller, CSPC_ITERATION rounds (:159-205). CPUs the
 * panic already stopped are offline and skipped. Stock also skips a powered-down
 * core (exynos_cpu.power_state(), :170): without cpuidle power-down states (L7 not
 * merged) an online core is always powered; L7 must add that check here.
 */
static void cspc_dump(const char *why)
{
	unsigned int cpu, iter, self;
	unsigned long flags;

	raw_spin_lock_irqsave(&cspc_lock, flags);
	self = raw_smp_processor_id();
	for (iter = 0; iter < CSPC_ITERATION; iter++) {
		for_each_possible_cpu(cpu) {
			void __iomem *base = cspc_base_of(cpu);

			cspc_pc[cpu][iter] = 0;
			if (!base || cpu == self || !cpu_online(cpu))
				continue;
			cspc_pc[cpu][iter] = cspc_sample(base);
		}
	}

	pr_emerg("cspc: %s: PCs of the other CPUs (from CPU%u)\n", why, self);
	for_each_possible_cpu(cpu) {
		for (iter = 0; iter < CSPC_ITERATION; iter++) {
			if (!cspc_pc[cpu][iter])
				continue;
			pr_emerg("cspc: CPU%u %pS\n", cpu, (void *)cspc_pc[cpu][iter]);
		}
	}
	raw_spin_unlock_irqrestore(&cspc_lock, flags);
}

static int cspc_panic(struct notifier_block *nb, unsigned long ev, void *msg)
{
	if (num_online_cpus() > 1)
		cspc_dump("panic");
	return NOTIFY_DONE;
}

/* before the other panic notifiers print (the PCIe one): the pstore record is 16 KiB */
static struct notifier_block cspc_panic_nb = {
	.notifier_call	= cspc_panic,
	.priority	= INT_MAX,
};

static int cspc_trigger_set(void *data, u64 val)
{
	if (val)
		cspc_dump("debugfs");
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(cspc_trigger_fops, NULL, cspc_trigger_set, "%llu\n");

/* S:drivers/trace/exynos-coresight.c:507-549, nodes FDT:7758-7830 */
static int __init cspc_init(void)
{
	struct device_node *np, *child;
	void __iomem *sj;
	u32 base, sj_off, off, status;
	int n = 0;

	np = of_find_compatible_node(NULL, NULL, "exynos,coresight");
	if (!np)
		return 0;
	if (of_property_read_u32(np, "base", &base))
		goto out;
	if (of_property_read_u32(np, "sj-offset", &sj_off))
		sj_off = CSPC_SJTAG_OFFSET;

	sj = ioremap(base + sj_off, SZ_8);
	if (!sj)
		goto out;
	status = readl(sj + CSPC_SJTAG_STATUS);
	iounmap(sj);
	if (status & CSPC_SJTAG_SOFT_LOCK) {
		pr_info("cspc: secure JTAG soft lock set (0x%08x), PC sampling off\n", status);
		goto out;
	}

	for_each_child_of_node(np, child) {
		if (!of_node_is_type(child, "cs"))
			continue;
		if (n >= CSPC_NR || of_property_read_u32(child, "dbg-offset", &off)) {
			of_node_put(child);
			break;
		}
		cspc_base[n] = ioremap(base + off, SZ_4K);
		n++;
	}

	atomic_notifier_chain_register(&panic_notifier_list, &cspc_panic_nb);
	debugfs_create_file_unsafe("greatlte-cspc", 0200, NULL, NULL, &cspc_trigger_fops);
	pr_info("cspc: %d CPU debug blocks at 0x%08x, sjtag 0x%08x\n", n, base, status);
out:
	of_node_put(np);
	return 0;
}
late_initcall(cspc_init);
