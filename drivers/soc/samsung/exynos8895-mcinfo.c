// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 DRAM temperature trip (spec L6K 5.2): reads the MR4 temperature
 * range field of the four DMC channels on demand (sysfs ref_rate/ref_rate)
 * and panics on any of the four DMC over-temperature interrupts.
 *
 * Port of S:drivers/soc/samsung/exynos-mcinfo.c (stock 4.4). The stock DT
 * properties bit_field/basecnt/irqcnt (FDT:6706-6708) become the variant data
 * below and the number of reg/interrupts entries. Not ported: the
 * get_mcinfo_base_count()/get_refresh_rate() exports, whose only user logged
 * to exynos-snapshot (S:drivers/thermal/samsung/exynos_tmu.c:718-769).
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/panic.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

struct e8895_mcinfo_variant {
	u32 start;	/* first bit of the field */
	u32 width;	/* stock "width"; the stock formula keeps width + 1 bits */
};

struct e8895_mcinfo {
	const struct e8895_mcinfo_variant *v;
	void __iomem **base;
	int nbase;
};

/* FDT:6706 bit_field = <0x14 0x04> "start bit, width" (S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:2447-2448) */
static const struct e8895_mcinfo_variant e8895_mcinfo = {
	.start = 20,
	.width = 4,
};

/*
 * Stock's exact expression on an unsigned int (S:drivers/soc/samsung/exynos-mcinfo.c:44-46,
 * :76-78): with start 20 and width 4 it is << 7 >> 27, i.e. bits [24:20] (five bits).
 * __raw_readl becomes readl_relaxed: the same 32-bit access on little-endian arm64.
 */
static u32 e8895_mcinfo_field(struct e8895_mcinfo *d, int i)
{
	return (readl_relaxed(d->base[i]) << (31 - d->v->start - d->v->width)) >>
	       (31 - d->v->width);
}

/* S:drivers/soc/samsung/exynos-mcinfo.c:33-52 */
static ssize_t ref_rate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct e8895_mcinfo *d = dev_get_drvdata(dev);
	int len, i;

	len = sysfs_emit(buf, "HW Refreshing rate\n");
	for (i = 0; i < d->nbase; i++)
		len += sysfs_emit_at(buf, len, "HwTempRange#%d: 0x%x\n", i,
				     e8895_mcinfo_field(d, i));

	return len;
}
static DEVICE_ATTR_RO(ref_rate);	/* stock 0644 without a store (:53) */

static struct attribute *e8895_mcinfo_attrs[] = {
	&dev_attr_ref_rate.attr,
	NULL,
};

/* stock group "ref_rate" (:60-63) */
static const struct attribute_group e8895_mcinfo_group = {
	.name = "ref_rate",
	.attrs = e8895_mcinfo_attrs,
};

static const struct attribute_group *e8895_mcinfo_groups[] = {
	&e8895_mcinfo_group,
	NULL,
};

/*
 * S:drivers/soc/samsung/exynos-mcinfo.c:66-85. Stock never clears or masks the source; the
 * first interrupt panics. pr_info (:79) is raised to pr_emerg so that the words reach the
 * ramoops console before the panic.
 */
static irqreturn_t e8895_mcinfo_irq(int irq, void *p)
{
	struct e8895_mcinfo *d = p;
	int i;

	for (i = 0; i < d->nbase; i++)
		pr_emerg("[SW Trip] HwTempRange#%d: 0x%x\n", i, e8895_mcinfo_field(d, i));

	panic("[SW Trip] Memory temperature is too high (irqnum: %d)\n", irq);

	return IRQ_HANDLED;
}

static int e8895_mcinfo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct e8895_mcinfo *d;
	int nirq, irq, i, ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->v = device_get_match_data(dev);
	if (!d->v)
		return -ENODEV;

	/* stock basecnt = 4 (FDT:6707): one word per reg entry (S:.../exynos-mcinfo.c:171-188) */
	while (platform_get_resource(pdev, IORESOURCE_MEM, d->nbase))
		d->nbase++;
	if (d->nbase < 1)
		return -EINVAL;

	d->base = devm_kcalloc(dev, d->nbase, sizeof(*d->base), GFP_KERNEL);
	if (!d->base)
		return -ENOMEM;

	for (i = 0; i < d->nbase; i++) {
		d->base[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(d->base[i]))
			return PTR_ERR(d->base[i]);
	}

	/* everything the handler and ref_rate_show() read exists from here on */
	platform_set_drvdata(pdev, d);

	/*
	 * stock irqcnt = 4 (FDT:6708), one handler per interrupt (S:.../exynos-mcinfo.c:190-202).
	 * Stock passed IRQF_SHARED; no other device uses these SPIs, and under
	 * CONFIG_DEBUG_SHIRQ a shared handler that always panics would run at request time.
	 */
	nirq = platform_irq_count(pdev);
	if (nirq < 0)
		return nirq;

	for (i = 0; i < nirq; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return irq;

		ret = devm_request_irq(dev, irq, e8895_mcinfo_irq, 0, dev_name(dev), d);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request IRQ %d\n", irq);
	}

	dev_info(dev, "probe finished!\n");	/* stock text :211 */

	return 0;
}

static const struct of_device_id e8895_mcinfo_of_match[] = {
	{ .compatible = "samsung,exynos8895-mcinfo", .data = &e8895_mcinfo },
	{ }
};

static struct platform_driver e8895_mcinfo_driver = {
	.probe = e8895_mcinfo_probe,
	.driver = {
		.name = "exynos8895-mcinfo",
		.of_match_table = e8895_mcinfo_of_match,
		.dev_groups = e8895_mcinfo_groups,
		/* a hardware-protection driver must not be unbound from sysfs */
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(e8895_mcinfo_driver);
