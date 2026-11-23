// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 DRAM temperature trip: reports the MR4 temperature range field
 * of the four DMC channels (sysfs ref_rate/ref_rate) and panics on any of the
 * four DMC over-temperature interrupts, as the vendor exynos-mcinfo driver.
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
	u32 width;	/* the field is width + 1 bits wide */
};

struct e8895_mcinfo {
	const struct e8895_mcinfo_variant *v;
	void __iomem **base;
	int nbase;
};

static const struct e8895_mcinfo_variant e8895_mcinfo = {
	.start = 20,
	.width = 4,
};

/* the vendor expression: bits [start + width:start] */
static u32 e8895_mcinfo_field(struct e8895_mcinfo *d, int i)
{
	return (readl_relaxed(d->base[i]) << (31 - d->v->start - d->v->width)) >>
	       (31 - d->v->width);
}

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
static DEVICE_ATTR_RO(ref_rate);

static struct attribute *e8895_mcinfo_attrs[] = {
	&dev_attr_ref_rate.attr,
	NULL,
};

static const struct attribute_group e8895_mcinfo_group = {
	.name = "ref_rate",
	.attrs = e8895_mcinfo_attrs,
};

static const struct attribute_group *e8895_mcinfo_groups[] = {
	&e8895_mcinfo_group,
	NULL,
};

/* the source is never cleared: the first interrupt panics */
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
	 * Not IRQF_SHARED, unlike the vendor driver: nothing else uses these
	 * interrupts, and CONFIG_DEBUG_SHIRQ would call the panicking handler.
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
