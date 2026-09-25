// SPDX-License-Identifier: GPL-2.0-only
/* greatlte bring-up heartbeat. DEBUG ONLY - see T_tooling spec 5.7. */
#define pr_fmt(fmt) "hb: v1 " fmt

#include <linux/atomic.h>
#include <linux/blkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpufreq.h>
#include <linux/cpuhotplug.h>
#include <linux/device/bus.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/initrd.h>
#include <linux/kernel_stat.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/random.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/sched/stat.h>
#include <linux/sched/task.h>
#include <linux/soc/samsung/greatlte-hb.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/thermal.h>
#include <linux/utsname.h>
#include <asm/memory.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/sysreg.h>
#include <trace/events/initcall.h>

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "hb."
static unsigned int period_ms = 2000;       module_param(period_ms, uint, 0644);
static unsigned int slow_after = 120;       module_param(slow_after, uint, 0644);
static unsigned int slow_period_ms = 10000; module_param(slow_period_ms, uint, 0644);
static unsigned int banner_every = 15;      module_param(banner_every, uint, 0644);
static unsigned int panic_after;            module_param(panic_after, uint, 0644);
static unsigned int tick_ms = 10;           module_param(tick_ms, uint, 0444);	/* 0 = no per-CPU ticks */

#define HB_LINE		200
#define HB_CHUNK	140			/* payload per physical line; prefix fits in the rest */
#define HB_LAYERS	7			/* L0..L6 */
#define HB_MAX_PRINTERS	16

enum hb_kind { HB_IRQ, HB_DEV, HB_CLK, HB_REG, HB_FREQ, HB_BLK, HB_PSY, HB_TZ };

struct hb_item {
	u8 layer;
	const char *tag;
	enum hb_kind kind;
	const char *path;	/* DT node path (IRQ, DEV, CLK); the rail's regulator-name (REG);
				 * power_supply name (PSY); thermal zone type (TZ) */
	const char *name;	/* clock-names entry (CLK) */
	int idx;		/* irq index (IRQ), clock index if !name (CLK), cpu (FREQ), property (PSY) */
	/* resolved by hb-probe; read by hb only after 'ok' is set (release/acquire) */
	bool ok;
	int err;		/* last resolution error; 0 = not tried yet (printed as !-115) */
	int virq;
	struct clk *clk;
	struct regulator *reg;
	struct power_supply *psy;
	struct device *dev;	/* HB_DEV: referenced device (platform, i2c or spi bus) */
	struct device_node *np;
	char slow[2][24];	/* REG/PSY/TZ value text, double-buffered, written by hb-probe */
	unsigned int scur;	/* index of the last complete slow[] text */
};

static struct hb_item hb_items[] = {
	/* L0 rows (T_tooling). Paths are ACK exynos8895.dtsi paths. */
	{ .layer = 0, .tag = "ptmr", .kind = HB_IRQ, .path = "/timer", .idx = 1 },
		/* A:arch/arm64/boot/dts/exynos/exynos8895.dtsi:1361-1364, 2nd entry = PPI 14 (non-secure phys);
		 * stock S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:577-580 "<1 13 0xff01>, <1 14 0xff01>, <1 11 0xff01>, <1 10 0xff01>" */
	{ .layer = 0, .tag = "vtmr", .kind = HB_IRQ, .path = "/timer", .idx = 2 },
		/* 3rd entry = PPI 11 (virtual); EL1 without hyp uses it: A:drivers/clocksource/arm_arch_timer.c:1108-1120 */
	{ .layer = 0, .tag = "mct",  .kind = HB_IRQ, .path = "/soc@0/timer@10040000", .idx = 0 },
		/* A:arch/arm64/boot/dts/exynos/exynos8895.dtsi:182,188 "<GIC_SPI 455 IRQ_TYPE_LEVEL_HIGH>";
		 * stock S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:603,620 mct@10040000 "interrupt-map = <0 &gic 0 455 0>",
		 * shipped DTB (dtb0_plat22bf_rev6-255.dts:4581,4596) first map entry 0x1c7 = 455 */
	{ .layer = 0, .tag = "blk",  .kind = HB_BLK },
	/* L1..L6 rows: appended by each layer spec, see 5.7.4 */
};

static struct greatlte_hb_printer *hb_printers[HB_MAX_PRINTERS];
static atomic_t hb_nprinters = ATOMIC_INIT(0);

static DEFINE_PER_CPU(struct hrtimer, hb_timer);
static DEFINE_PER_CPU(u32, hb_ticks);
static u32 hb_nonce;
static unsigned int hb_seq;
static phys_addr_t hb_initrd_start;	/* copies: phys_initrd_* are __initdata (init/do_mounts_initrd.c:20-21) */
static unsigned long hb_initrd_size;
static initcall_t hb_ic_cur, hb_ic_fail;
static int hb_ic_fail_ret;
static bool hb_classes_ready;		/* block/regulator/power_supply classes exist (subsys_initcall_sync) */
static struct task_struct *hb_probe_task;
static atomic_t hb_probe_kick = ATOMIC_INIT(0);
static bool hb_probe_busy = true;	/* true until hb-probe first goes to sleep */
static unsigned int hb_probe_seq;		/* completed probe passes */
static u64 hb_probe_done_ms;

/* ---- initcall tracking (tracepoints fire in do_one_initcall) ---- */
static void hb_ic_start(void *d, initcall_t fn) { WRITE_ONCE(hb_ic_cur, fn); }
static void hb_ic_finish(void *d, initcall_t fn, int ret)
{
	WRITE_ONCE(hb_ic_cur, NULL);
	if (ret && ret != -ENODEV) { WRITE_ONCE(hb_ic_fail, fn); WRITE_ONCE(hb_ic_fail_ret, ret); }
}

/* ---- per-CPU liveness ---- */
static enum hrtimer_restart hb_tick(struct hrtimer *t)
{
	this_cpu_inc(hb_ticks);
	hrtimer_forward_now(t, ms_to_ktime(tick_ms));
	return HRTIMER_RESTART;
}
static int hb_cpu_online(unsigned int cpu)	/* runs on 'cpu' (CPUHP_AP_ONLINE_DYN) */
{
	struct hrtimer *t = this_cpu_ptr(&hb_timer);
	hrtimer_setup(t, hb_tick, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
	hrtimer_start(t, ms_to_ktime(tick_ms), HRTIMER_MODE_REL_PINNED);
	return 0;
}
static int hb_cpu_offline(unsigned int cpu)
{
	hrtimer_cancel(this_cpu_ptr(&hb_timer));
	return 0;
}

static u64 hb_up_ms(void) { return ktime_to_ms(ktime_get_boottime()); }
static unsigned int hb_period(u64 up)
{
	return up >= (u64)slow_after * 1000 ? slow_period_ms : period_ms;
}

/* ---- printer registration (include/linux/soc/samsung/greatlte-hb.h) ---- */
static unsigned int hb_nprn(void)
{
	return min_t(unsigned int, atomic_read(&hb_nprinters), HB_MAX_PRINTERS);
}

int greatlte_hb_register(struct greatlte_hb_printer *p)
{
	int i;

	if (!p || !p->name || !p->fmt || p->layer >= HB_LAYERS)
		return -EINVAL;
	i = atomic_fetch_inc(&hb_nprinters);
	if (i >= HB_MAX_PRINTERS) {
		pr_err("reg %s !%d\n", p->name, -ENOSPC);
		return -ENOSPC;
	}
	smp_store_release(&hb_printers[i], p);
	pr_err("reg %s layer=%u slow=%d every=%u\n", p->name, p->layer, p->slow, p->every_ms);
	return 0;
}

/*
 * One logical output = '\n'-separated records. Each record starts a line
 * "<name> seq=<n>[ age=<ms>] <text>"; a record longer than HB_CHUNK is
 * wrapped at a space into "<name>+ seq=<n> <rest>" lines.
 */
static void hb_emit(const char *name, const char *text, int ret, s64 age)
{
	char agebuf[24] = "";
	const char *s = text, *end;
	size_t len, n;
	bool first;

	if (age >= 0)
		scnprintf(agebuf, sizeof(agebuf), " age=%lld", age);
	if (ret <= 0 || !text || !*text) {
		pr_err("%s seq=%u%s !%d\n", name, hb_seq, agebuf, ret < 0 ? ret : -ENODATA);
		return;
	}
	while (*s) {
		end = strchrnul(s, '\n');
		len = end - s;
		first = true;
		while (len) {
			n = min_t(size_t, len, HB_CHUNK);
			if (n < len) {			/* break at the last space in the chunk */
				size_t k = n;

				while (k > 1 && s[k] != ' ')
					k--;
				if (k > 1)
					n = k;
			}
			pr_err("%s%s seq=%u%s %.*s\n", name, first ? "" : "+", hb_seq,
			       first ? agebuf : "", (int)n, s);
			first = false;
			s += n;
			len -= n;
			while (len && *s == ' ') {
				s++;
				len--;
			}
		}
		s = *end ? end + 1 : end;
	}
}

/* ---- lock-free counters used by 'hb' ---- */
static int hb_unbound_one(struct device *dev, void *data)
{
	if (dev->of_node && of_device_is_available(dev->of_node) && !READ_ONCE(dev->driver))
		(*(unsigned int *)data)++;
	return 0;
}
static int hb_blk_one(struct device *dev, void *data)
{
	int *c = data;

	c[0]++;				/* every block-class device */
	if (dev->parent)		/* not /sys/devices/virtual (loop, ram): disks with a parent and all partitions */
		c[1]++;
	return 0;
}

struct hb_ulist { char *b; size_t sz; int n; unsigned int more; };
static int hb_unbound_name(struct device *dev, void *data)
{
	struct hb_ulist *u = data;
	const char *name = dev_name(dev);

	if (!dev->of_node || !of_device_is_available(dev->of_node) || READ_ONCE(dev->driver))
		return 0;
	if (u->n + strlen(name) + 2 < u->sz)
		u->n += scnprintf(u->b + u->n, u->sz - u->n, " %s", name);
	else
		u->more++;
	return 0;
}

static bool hb_is_slow(enum hb_kind k) { return k == HB_REG || k == HB_PSY || k == HB_TZ; }
/* kinds that walk a class: class_for_each/find_device WARN before class_register (A:drivers/base/class.c:414-417) */
static bool hb_needs_class(enum hb_kind k) { return k == HB_BLK || hb_is_slow(k); }

static int hb_fmt_fast(char *b, size_t sz, struct hb_item *it)
{
	const struct device_driver *drv;
	int e, cnt[2] = { 0, 0 };

	if (hb_needs_class(it->kind) && !smp_load_acquire(&hb_classes_ready))
		return scnprintf(b, sz, " %s=!%d", it->tag, -EAGAIN);
	if (hb_is_slow(it->kind)) {		/* value or "!<errno>" text produced by hb-probe */
		const char *s = it->slow[smp_load_acquire(&it->scur)];

		return scnprintf(b, sz, " %s=%s", it->tag, *s ? s : "!-115");
	}
	if (it->kind != HB_FREQ && it->kind != HB_BLK && !smp_load_acquire(&it->ok)) {
		e = READ_ONCE(it->err);
		return scnprintf(b, sz, " %s=!%d", it->tag, e ? e : -EINPROGRESS);
	}
	switch (it->kind) {
	case HB_IRQ:	/* RCU only: A:kernel/irq/irqdesc.c:977-985 */
		return scnprintf(b, sz, " %s=%u", it->tag, kstat_irqs_usr(it->virq));
	case HB_DEV:	/* cached device; lock-free read of ->driver, never device_lock() */
		drv = READ_ONCE(smp_load_acquire(&it->dev)->driver);
		return scnprintf(b, sz, " %s=%s", it->tag, drv ? drv->name : "U");
	case HB_CLK:	/* software state only: no prepare_lock, no register read */
		return scnprintf(b, sz, " %s=%lu:%u", it->tag,
				 clk_hw_get_rate(__clk_get_hw(it->clk)),
				 __clk_get_enable_count(it->clk));
	case HB_FREQ:
		return scnprintf(b, sz, " %s=%u", it->tag, cpufreq_quick_get(it->idx));
	case HB_BLK:
		class_for_each_device(&block_class, NULL, cnt, hb_blk_one);
		return scnprintf(b, sz, " %s=%d/%d", it->tag, cnt[0], cnt[1]);
	default:
		break;
	}
	return 0;
}

/* ---- hb-probe: resolution and anything that may sleep on driver locks ---- */
static void hb_set_slow(struct hb_item *it, const char *fmt, int v)
{
	unsigned int w = !it->scur;		/* only hb-probe writes scur */

	scnprintf(it->slow[w], sizeof(it->slow[w]), fmt, v);
	smp_store_release(&it->scur, w);
}

/* A node's device may sit on the platform, i2c or spi bus (A:include/linux/device/bus.h:184). */
static struct device *hb_find_dev(struct device_node *np)
{
	struct device *dev = bus_find_device_by_of_node(&platform_bus_type, np);

#if IS_REACHABLE(CONFIG_I2C)
	if (!dev)
		dev = bus_find_device_by_of_node(&i2c_bus_type, np);	/* A:include/linux/i2c.h:26 */
#endif
#if IS_REACHABLE(CONFIG_SPI)
	if (!dev)
		dev = bus_find_device_by_of_node(&spi_bus_type, np);	/* A:include/linux/spi/spi.h:42 */
#endif
	return dev;
}

static void hb_resolve(struct hb_item *it)
{
	struct regulator *r;
	struct device *d;
	struct clk *c;
	int v;

	if (it->kind == HB_FREQ || it->kind == HB_BLK || it->kind == HB_TZ)
		return;				/* nothing to resolve */
	if (it->ok) {
		/* a device that was deleted (e.g. an MFD cell re-created by a deferred parent probe) is looked up
		 * again; the stale reference is leaked on purpose because hb may still be reading it */
		if (it->kind == HB_DEV && !device_is_registered(it->dev)) {
			d = hb_find_dev(it->np);
			if (d)
				smp_store_release(&it->dev, d);
		}
		return;
	}
	if (it->kind == HB_REG) {
		r = regulator_get_optional(NULL, it->path);	/* deviceless: matches regulator-name only, A:drivers/regulator/core.c:2037 */
		if (IS_ERR(r)) { WRITE_ONCE(it->err, PTR_ERR(r)); return; }
		it->reg = r;
		smp_store_release(&it->ok, true);
		return;
	}
	if (it->kind == HB_PSY) {
#if IS_ENABLED(CONFIG_POWER_SUPPLY)
		struct power_supply *psy = power_supply_get_by_name(it->path);

		if (!psy) { WRITE_ONCE(it->err, -ENODEV); return; }
		it->psy = psy;
		smp_store_release(&it->ok, true);
#else
		WRITE_ONCE(it->err, -ENOSYS);
#endif
		return;
	}
	if (!it->np) {
		it->np = of_find_node_by_path(it->path);
		if (!it->np) { WRITE_ONCE(it->err, -ENOENT); return; }
	}
	if (it->kind == HB_IRQ) {
		v = of_irq_get(it->np, it->idx);		/* A:drivers/of/irq.c:458 */
		if (v <= 0) { WRITE_ONCE(it->err, v ? v : -ENXIO); return; }
		it->virq = v;
	} else if (it->kind == HB_CLK) {
		c = it->name ? of_clk_get_by_name(it->np, it->name) : of_clk_get(it->np, it->idx);
		if (IS_ERR(c)) { WRITE_ONCE(it->err, PTR_ERR(c)); return; }
		it->clk = c;
	} else if (it->kind == HB_DEV) {
		d = hb_find_dev(it->np);
		if (!d) { WRITE_ONCE(it->err, -ENODEV); return; }
		smp_store_release(&it->dev, d);
	}
	smp_store_release(&it->ok, true);
}

static void hb_probe_items(void)
{
	bool ready = smp_load_acquire(&hb_classes_ready);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hb_items); i++) {
		struct hb_item *it = &hb_items[i];

		if (hb_needs_class(it->kind) && !ready)
			continue;			/* hb prints !-11 until the classes exist */
		hb_resolve(it);
		if (!hb_is_slow(it->kind))
			continue;
		if (it->kind == HB_TZ) {
#if IS_ENABLED(CONFIG_THERMAL)
			/* looked up on every pass: the core hands out no reference (A:drivers/thermal/thermal_core.c:1755-1779) */
			struct thermal_zone_device *tz = thermal_zone_get_zone_by_name(it->path);
			int t = 0, r = IS_ERR(tz) ? PTR_ERR(tz) : thermal_zone_get_temp(tz, &t);

			if (r)
				hb_set_slow(it, "!%d", r);
			else
				hb_set_slow(it, "%d", t);	/* milli-degC */
#else
			hb_set_slow(it, "!%d", -ENOSYS);
#endif
		} else if (!it->ok) {
			hb_set_slow(it, "!%d", it->err ? it->err : -EINPROGRESS);
		} else if (it->kind == HB_REG) {
			int en = regulator_is_enabled(it->reg);
			int uv = regulator_get_voltage(it->reg);
			unsigned int w = !it->scur;

			scnprintf(it->slow[w], sizeof(it->slow[w]), "%d:%d", en, uv);
			smp_store_release(&it->scur, w);
		} else {
#if IS_ENABLED(CONFIG_POWER_SUPPLY)
			union power_supply_propval pv = {};
			int r = power_supply_get_property(it->psy, it->idx, &pv);

			if (r)
				hb_set_slow(it, "!%d", r);
			else
				hb_set_slow(it, "%d", pv.intval);
#endif
		}
	}
}

static void hb_probe_printers(void)
{
	unsigned int i, n = hb_nprn();

	for (i = 0; i < n; i++) {
		struct greatlte_hb_printer *p = smp_load_acquire(&hb_printers[i]);
		u64 now = hb_up_ms();
		unsigned int w;

		if (!p || !p->slow)
			continue;
		/* produce if never produced, or if hb will print it at its next period */
		if (p->done_ms[p->cur] && READ_ONCE(p->printed) && p->every_ms &&
		    now + hb_period(now) < READ_ONCE(p->last_ms) + p->every_ms)
			continue;
		w = !p->cur;			/* only hb-probe writes cur */
		p->ret[w] = p->fmt(p->text[w], sizeof(p->text[w]));
		p->done_ms[w] = hb_up_ms();
		smp_store_release(&p->cur, w);
	}
}

/*
 * One pass per kick from 'hb', started only after 'hb' has finished printing
 * (that is what makes the double buffers tear-free). The state is set before
 * 'busy' is cleared and the kick is tested after it, so a kick cannot be lost;
 * a spurious wake-up finds no kick and sleeps again.
 */
static int hb_probe_fn(void *unused)
{
	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		WRITE_ONCE(hb_probe_busy, false);
		if (!atomic_xchg(&hb_probe_kick, 0)) {
			schedule();
			continue;
		}
		__set_current_state(TASK_RUNNING);
		WRITE_ONCE(hb_probe_busy, true);
		hb_probe_items();
		hb_probe_printers();
		WRITE_ONCE(hb_probe_done_ms, hb_up_ms());
		WRITE_ONCE(hb_probe_seq, hb_probe_seq + 1);
	}
	return 0;
}

/* ---- output ---- */
static void hb_banner(void)
{
	const char *model = "?";
	struct device_node *np;
	struct resource r;
	u32 rec = 0, con = 0;
	size_t len = saved_command_line_len, off;
	char ps[48] = "!ENOENT", prn[HB_CHUNK] = "";
	unsigned int i, n = hb_nprn(), k = 0;

	of_property_read_string(of_root, "model", &model);
	np = of_find_compatible_node(NULL, NULL, "ramoops");
	if (np) {
		if (!of_address_to_resource(np, 0, &r)) {
			of_property_read_u32(np, "record-size", &rec);
			of_property_read_u32(np, "console-size", &con);
			scnprintf(ps, sizeof(ps), "%pa+%#llx/%#x/%#x", &r.start,
				  (unsigned long long)resource_size(&r), rec, con);
		} else {
			strscpy(ps, "!EINVAL");
		}
		of_node_put(np);
	}
	for (i = 0; i < n; i++) {
		struct greatlte_hb_printer *p = smp_load_acquire(&hb_printers[i]);

		if (p)
			k += scnprintf(prn + k, sizeof(prn) - k, "%s%s/%u", k ? "," : "",
				       p->name, p->layer);
	}
	pr_err("banner rel=%s ver=%s nonce=%08x el=%llu load=%pa kaslr=%#lx\n",
	       init_utsname()->release, init_utsname()->version, hb_nonce,
	       read_sysreg(CurrentEL) >> 2, &(phys_addr_t){ __pa_symbol(_text) },
	       kaslr_offset());
	pr_err("banner initrd=%pa+%#lx model=\"%s\" pstore=%s cpus=%u\n",
	       &hb_initrd_start, hb_initrd_size, model, ps, num_possible_cpus());
	pr_err("banner prn=%s items=%zu\n", k ? prn : "-", ARRAY_SIZE(hb_items));
	for (off = 0; off < len; off += 150)
		pr_err("cmd %zu/%zu %.150s\n", off / 150, (len + 149) / 150, saved_command_line + off);
	pr_err("cmdlen=%zu max=%d trunc=%d\n", len, COMMAND_LINE_SIZE - 1,
	       len >= COMMAND_LINE_SIZE - 1);
}

static void hb_line_L0(u64 up)
{
	char b[HB_LINE];
	struct task_struct *p1;
	unsigned int unb = 0, cpu;
	int n;

	bus_for_each_dev(&platform_bus_type, NULL, &unb, hb_unbound_one);
	n = scnprintf(b, sizeof(b), "L0 seq=%u up=%llu st=%d on=%*pbl tk=", hb_seq, up,
		      system_state, cpumask_pr_args(cpu_online_mask));
	for_each_possible_cpu(cpu)
		n += (tick_ms && cpu_online(cpu)) ? scnprintf(b + n, sizeof(b) - n, "%u/", per_cpu(hb_ticks, cpu))
				     : scnprintf(b + n, sizeof(b) - n, "-/");
	pr_err("%s\n", b);
	rcu_read_lock();
	p1 = find_task_by_vpid(1);
	pr_err("L0b seq=%u ic=%ps fail=%ps:%d p1=%s thr=%d unb=%u mem=%lu pb=%u/%llu%s\n",
	       hb_seq, READ_ONCE(hb_ic_cur), READ_ONCE(hb_ic_fail), READ_ONCE(hb_ic_fail_ret),
	       p1 ? p1->comm : "-", nr_threads, unb,
	       si_mem_available() >> (20 - PAGE_SHIFT),
	       READ_ONCE(hb_probe_seq), up - READ_ONCE(hb_probe_done_ms),
	       READ_ONCE(hb_probe_busy) ? "B" : "");
	rcu_read_unlock();
}

static void hb_line_unbound(void)		/* names of the devices counted in unb= */
{
	char b[HB_LINE];
	struct hb_ulist u = { .b = b, .sz = sizeof(b) - 16 };

	u.n = scnprintf(b, u.sz, "L0u seq=%u", hb_seq);
	bus_for_each_dev(&platform_bus_type, NULL, &u, hb_unbound_name);
	pr_err("%s +%u\n", b, u.more);
}

static void hb_line_layers(void)
{
	char b[HB_LINE];
	unsigned int l, i;
	int n;

	for (l = 0; l < HB_LAYERS; l++) {
		n = 0;
		for (i = 0; i < ARRAY_SIZE(hb_items); i++) {
			if (hb_items[i].layer != l)
				continue;
			if (!n)
				n = scnprintf(b, sizeof(b), "L%uf seq=%u", l, hb_seq);
			if (n > HB_LINE - 40) {			/* continuation line */
				pr_err("%s +\n", b);
				n = scnprintf(b, sizeof(b), "L%uf seq=%u", l, hb_seq);
			}
			n += hb_fmt_fast(b + n, sizeof(b) - n, &hb_items[i]);
		}
		if (n)
			pr_err("%s\n", b);
	}
}

static void hb_line_printers(u64 up)
{
	unsigned int i, n = hb_nprn(), c;

	for (i = 0; i < n; i++) {
		struct greatlte_hb_printer *p = smp_load_acquire(&hb_printers[i]);

		if (!p)
			continue;
		if (p->printed && p->every_ms && up < p->last_ms + p->every_ms)
			continue;
		if (p->slow) {
			c = smp_load_acquire(&p->cur);
			if (!p->done_ms[c]) {			/* hb-probe has not run it yet */
				hb_emit(p->name, NULL, -EINPROGRESS, -1);
				continue;			/* stays due: text follows next period */
			}
			hb_emit(p->name, p->text[c], p->ret[c], up - p->done_ms[c]);
		} else {
			/* fast printers run here, in 'hb': text[0] is scratch */
			hb_emit(p->name, p->text[0], p->fmt(p->text[0], sizeof(p->text[0])), -1);
		}
		WRITE_ONCE(p->last_ms, up);
		WRITE_ONCE(p->printed, true);
	}
}

static int hb_fn(void *unused)
{
	u64 up;

	sched_set_fifo_low(current);
	for (;;) {
		up = hb_up_ms();
		if (banner_every && !(hb_seq % banner_every)) {
			hb_banner();
			hb_line_unbound();
		}
		hb_line_L0(up);
		hb_line_layers();
		hb_line_printers(up);
		if (hb_probe_task && !READ_ONCE(hb_probe_busy)) {
			atomic_set(&hb_probe_kick, 1);		/* consumed by hb_probe_fn */
			wake_up_process(hb_probe_task);
		}
		if (panic_after && up >= (u64)panic_after * 1000)
			panic("hb: end of run after %us", panic_after);
		hb_seq++;
		schedule_timeout_interruptible(msecs_to_jiffies(hb_period(up)));
	}
	return 0;
}

static int __init hb_early(void)		/* before SMP: banner + threads + initcall hooks */
{
	struct task_struct *t;

	hb_nonce = get_random_u32();
	hb_initrd_start = phys_initrd_start;	/* __initdata: copy while it exists */
	hb_initrd_size = phys_initrd_size;
	register_trace_initcall_start(hb_ic_start, NULL);
	register_trace_initcall_finish(hb_ic_finish, NULL);
	t = kthread_run(hb_probe_fn, NULL, "hb-probe");
	if (IS_ERR(t))
		pr_err("probe !%ld\n", PTR_ERR(t));	/* fast lines keep running; pb= stays 0 */
	else
		hb_probe_task = t;
	t = kthread_run(hb_fn, NULL, "hb");
	if (IS_ERR(t))
		pr_err("thread !%ld\n", PTR_ERR(t));
	return 0;
}
early_initcall(hb_early);

static int __init hb_cpus(void)			/* after smp_init(): per-CPU ticks */
{
	if (!tick_ms)
		return 0;
	cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "greatlte/hb:online", hb_cpu_online, hb_cpu_offline);
	return 0;
}
core_initcall(hb_cpus);

/* block_class (subsys, A:block/genhd.c:1011), regulator_class (core, A:drivers/regulator/core.c:6490) and
 * power_supply_class (subsys, A:drivers/power/supply/power_supply_core.c:2021) all exist from here on */
static int __init hb_classes(void)
{
	smp_store_release(&hb_classes_ready, true);
	return 0;
}
subsys_initcall_sync(hb_classes);

#ifdef CONFIG_GREATLTE_RAMCON
#include <linux/io.h>
#include <linux/serial_core.h>

#define GL_PRZ_SIG	0x43474244	/* PERSISTENT_RAM_SIG, A:fs/pstore/ram_core.c:40; zone sig 0 */
#define GL_HDR		12		/* sig, start, size: A:fs/pstore/ram_core.c:33-38 */
#define GL_CAP		(PAGE_SIZE - GL_HDR)	/* one fixmap page; never wraps */

static void __iomem *gl_ramcon;
static bool gl_ramcon_off;

static void gl_ramcon_write(struct console *con, const char *s, unsigned int n)
{
	u32 used;

	if (READ_ONCE(gl_ramcon_off) || !gl_ramcon)
		return;
	used = readl_relaxed(gl_ramcon + 8);
	if (used >= GL_CAP)
		return;
	n = min_t(u32, n, GL_CAP - used);
	memcpy_toio(gl_ramcon + GL_HDR + used, s, n);
	writel_relaxed(used + n, gl_ramcon + 4);	/* start: next write offset */
	writel_relaxed(used + n, gl_ramcon + 8);	/* size: valid bytes */
}

static int __init gl_ramcon_setup(struct earlycon_device *dev, const char *opt)
{
	static const char hdr[] = "====1.000000-D\n";

	gl_ramcon = dev->port.membase;
	if (!gl_ramcon)
		return -ENODEV;
	writel_relaxed(0, gl_ramcon);			/* invalidate while (re)initialising */
	memcpy_toio(gl_ramcon + GL_HDR, hdr, sizeof(hdr) - 1);
	writel_relaxed(sizeof(hdr) - 1, gl_ramcon + 4);
	writel_relaxed(sizeof(hdr) - 1, gl_ramcon + 8);
	writel(GL_PRZ_SIG, gl_ramcon);			/* 0 ^ PERSISTENT_RAM_SIG */
	dev->con->write = gl_ramcon_write;
	return 0;
}
EARLYCON_DECLARE(gl_ram, gl_ramcon_setup);

static int __init gl_ramcon_stop(void)
{
	WRITE_ONCE(gl_ramcon_off, true);
	return 0;
}
subsys_initcall(gl_ramcon_stop);
#endif

#ifdef CONFIG_GREATLTE_LASTKMSG
/*
 * Transport R-K (T spec 5.11): mirror the console into the exynos-snapshot
 * "log_kernel" item of the pinned TWRP kernel (MINIMIZED_MODE layout), which
 * that kernel publishes as /proc/last_kmsg on its next boot. Unproven on
 * hardware (O-20); the ramoops console stays the primary channel.
 */
#include <linux/console.h>
#include <linux/of_reserved_mem.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#define GL_ESS_BASE	0x91200000UL	/* S-Boot " ess_setup=0x%x" value: SB+0x7be94 "mov w2, #-0x6ee00000" */
#define GL_ESS_LASTBUF	0x200		/* S:drivers/trace/exynos-ss.c:110 ESS_OFFSET_LAST_LOGBUF; kept by TW (:74, :2510) */
#define GL_LK_BASE	0x91400000UL	/* 2nd MINIMIZED item "log_kernel" = base + SZ_2M (S:drivers/trace/exynos-ss.c:563-564) */
#define GL_LK_SIZE	SZ_2M		/* S:drivers/trace/exynos-ss.c:564 */
#define GL_LK_CAP	(GL_LK_SIZE - 8)	/* last 8 bytes hold the "Full" magic (S:drivers/staging/samsung/sec_debug.c:430,433; writer S:drivers/trace/exynos-ss.c:1447) */
#define GL_LK_MAGIC	0x0000000a6c6c7546ULL	/* S:include/linux/sec_debug.h:298 SEC_LKMSG_MAGICKEY */

static u8 *gl_lk_hdr, *gl_lk_buf;
static u32 gl_lk_pos;

/* Legacy console ->write: serialised by the console lock, no own lock needed. */
static void gl_lk_write(struct console *con, const char *s, unsigned int n)
{
	while (n) {
		u32 c = min_t(u32, n, GL_LK_CAP - gl_lk_pos);

		memcpy(gl_lk_buf + gl_lk_pos, s, c);
		gl_lk_pos += c;
		s += c;
		n -= c;
		if (gl_lk_pos == GL_LK_CAP) {	/* wrapped: TW returns [pos, end) then [head, pos), sec_debug.c:433-439 */
			WRITE_ONCE(*(u64 *)(gl_lk_buf + GL_LK_CAP), GL_LK_MAGIC);
			gl_lk_pos = 0;
		}
	}
	wmb();					/* text before the pointer that publishes it */
	WRITE_ONCE(*(u32 *)(gl_lk_hdr + GL_ESS_LASTBUF), (u32)(GL_LK_BASE + gl_lk_pos));
}

static struct console gl_lk_con = {
	.name	= "gllk",
	.write	= gl_lk_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,	/* pre-enabled like pstore, A:fs/pstore/platform.c:423 */
	.index	= -1,
};

/* Map only a region the DTB reserves for exactly this purpose (gl-pack --lastkmsg adds the nodes). */
static u8 *gl_lk_map(const char *path, phys_addr_t base, size_t size)
{
	struct device_node *np = of_find_node_by_path(path);
	struct reserved_mem *rm = np ? of_reserved_mem_lookup(np) : NULL;
	unsigned int i, n = size >> PAGE_SHIFT;
	struct page **pages;
	void *va;

	of_node_put(np);
	if (!rm || rm->base != base || rm->size != size)
		return NULL;
	pages = kmalloc_array(n, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return NULL;
	for (i = 0; i < n; i++)
		pages[i] = pfn_to_page(PHYS_PFN(base) + i);
	/* write-combine alias of mapped reserved RAM, as ramoops: A:fs/pstore/ram_core.c:414-457 */
	va = vmap(pages, n, VM_MAP | VM_IOREMAP, pgprot_writecombine(PAGE_KERNEL));
	kfree(pages);
	return va;
}

static int __init gl_lk_init(void)
{
	gl_lk_hdr = gl_lk_map("/reserved-memory/gl-ess-hdr@91200000", GL_ESS_BASE, PAGE_SIZE);
	gl_lk_buf = gl_lk_map("/reserved-memory/gl-ess-lk@91400000", GL_LK_BASE, GL_LK_SIZE);
	if (!gl_lk_hdr || !gl_lk_buf) {
		pr_err("lk !%d\n", -ENOENT);
		return 0;
	}
	WRITE_ONCE(*(u64 *)(gl_lk_buf + GL_LK_CAP), 0);	/* no stale "Full" magic from TWRP's own log */
	register_console(&gl_lk_con);
	pr_err("lk on buf=%#lx+%#lx ptr=%#lx\n", GL_LK_BASE, (unsigned long)GL_LK_SIZE,
	       GL_ESS_BASE + GL_ESS_LASTBUF);
	return 0;
}
early_initcall(gl_lk_init);
#endif
