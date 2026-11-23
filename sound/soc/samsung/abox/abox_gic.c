// SPDX-License-Identifier: GPL-2.0-only
/*
 * sound/soc/samsung/abox/abox_gic.c - Samsung ABOX GIC driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 */
/* #define DEBUG */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/irqchip/arm-gic.h>
#include <soc/samsung/exynos8895-smc.h>

#include "abox_util.h"
#include "abox_gic.h"

void abox_gic_generate_interrupt(struct platform_device *pdev, int hw_irq)
{
	struct abox_gic_data *data = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s(%d)\n", __func__, hw_irq);
	/* the vendor kernel was built with GIC_IS_SECURE_FREE */
	writel((0x1 << 16) | (hw_irq & 0xF),
			data->gicd_base + GIC_DIST_SOFTINT);
}
EXPORT_SYMBOL(abox_gic_generate_interrupt);

void abox_gic_register_irq_handler(struct platform_device *pdev,
		irq_handler_t irq_handler, void *dev_id)
{
	struct abox_gic_data *data = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s\n", __func__);

	data->abox_gic_irq_handler.irq_handler = irq_handler;
	data->abox_gic_irq_handler.dev_id = dev_id;
}
EXPORT_SYMBOL(abox_gic_register_irq_handler);

void abox_gic_unregister_irq_handler(struct platform_device *pdev)
{
	struct abox_gic_data *data = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s\n", __func__);

	data->abox_gic_irq_handler.irq_handler = NULL;
	data->abox_gic_irq_handler.dev_id = NULL;
}
EXPORT_SYMBOL(abox_gic_unregister_irq_handler);

static irqreturn_t abox_gic_irq_handler(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct abox_gic_data *data = platform_get_drvdata(pdev);
	struct abox_gic_irq_handler_t *irq_handler =
			&data->abox_gic_irq_handler;
	irqreturn_t result = IRQ_HANDLED;
	u32 irqstat, irqnr;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	do {
		irqstat = readl(data->gicc_base + GIC_CPU_INTACK);
		irqnr = irqstat & GICC_IAR_INT_ID_MASK;
		dev_dbg(&pdev->dev, "IAR: %08X\n", irqstat);

		if (likely(irqnr < 16)) {
			writel(irqstat, data->gicc_base + GIC_CPU_EOI);
			writel(irqstat, data->gicc_base + GIC_CPU_DEACTIVATE);
			if (irq_handler->irq_handler) {
				result = irq_handler->irq_handler(irqnr,
						irq_handler->dev_id);
			}
			continue;
		} else if (unlikely(irqnr > 15 && irqnr < 1021)) {
			writel(irqstat, data->gicc_base + GIC_CPU_EOI);
			continue;
		}
		break;
	} while (1);

	return result;
}

void abox_gic_init_gic(struct platform_device *pdev)
{
	struct abox_gic_data *data = platform_get_drvdata(pdev);
	unsigned long arg;
	int i;

	dev_info(&pdev->dev, "%s\n", __func__);

	arg = SMC_REG_ID_SFR_W(data->gicc_base_phys + GIC_CPU_PRIMASK);
	exynos8895_smc(SMC_CMD_REG, arg, 0x000000FF, 0);

	arg = SMC_REG_ID_SFR_W(data->gicd_base_phys + GIC_DIST_CTRL);
	exynos8895_smc(SMC_CMD_REG, arg, 0x3, 0);
	if (is_secure_gic()) {
		for (i = 0; i < 1; i++) {
			arg = SMC_REG_ID_SFR_W(data->gicd_base_phys +
					GIC_DIST_IGROUP + (i * 4));
			exynos8895_smc(SMC_CMD_REG, arg, 0xFFFFFFFF, 0);
		}
	}
	for (i = 0; i < 40; i++) {
		arg = SMC_REG_ID_SFR_W(data->gicd_base_phys +
				GIC_DIST_PRI + (i * 4));
		exynos8895_smc(SMC_CMD_REG, arg, 0x10101010, 0);
	}

	writel(0x3, data->gicc_base + GIC_CPU_CTRL);
}
EXPORT_SYMBOL(abox_gic_init_gic);

int abox_gic_enable_irq(struct device *dev)
{
	struct abox_gic_data *data = dev_get_drvdata(dev);

	if (likely(data->disabled)) {
		dev_info(dev, "%s\n", __func__);

		data->disabled = false;
		enable_irq(data->irq);
	}
	return 0;
}

int abox_gic_disable_irq(struct device *dev)
{
	struct abox_gic_data *data = dev_get_drvdata(dev);

	if (likely(!data->disabled)) {
		dev_info(dev, "%s\n", __func__);

		data->disabled = true;
		disable_irq(data->irq);
	}

	return 0;
}

static int samsung_abox_gic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct abox_gic_data *data;
	struct resource *res;
	int result;

	dev_info(dev, "%s\n", __func__);

	data = devm_kzalloc(dev, sizeof(struct abox_gic_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	platform_set_drvdata(pdev, data);

	data->gicd_base = devm_platform_ioremap_resource_byname(pdev, "gicd");
	if (IS_ERR(data->gicd_base))
		return PTR_ERR(data->gicd_base);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gicd");
	data->gicd_base_phys = res->start;	/* SMC argument */

	data->gicc_base = devm_platform_ioremap_resource_byname(pdev, "gicc");
	if (IS_ERR(data->gicc_base))
		return PTR_ERR(data->gicc_base);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gicc");
	data->gicc_base_phys = res->start;	/* SMC argument */

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0)
		return data->irq;

	/*
	 * The trigger type comes from DT. The GIC CPU interface is in the ABOX
	 * power domain, so the line stays off until abox_enable() enables it.
	 */
	data->disabled = true;
	result = devm_request_irq(dev, data->irq, abox_gic_irq_handler,
		IRQF_NO_AUTOEN, pdev->name, pdev);
	if (result < 0) {
		dev_err(dev, "Failed to request irq\n");
		return result;
	}

	dev_info(dev, "%s: probe complete\n", __func__);

	return 0;
}

static void samsung_abox_gic_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s\n", __func__);
}

static const struct of_device_id samsung_abox_gic_of_match[] = {
	{
		.compatible = "samsung,exynos8895-abox-gic",
	},
	{},
};
MODULE_DEVICE_TABLE(of, samsung_abox_gic_of_match);

static struct platform_driver samsung_abox_gic_driver = {
	.probe  = samsung_abox_gic_probe,
	.remove = samsung_abox_gic_remove,
	.driver = {
		.name = "samsung-abox-gic",
		.of_match_table = samsung_abox_gic_of_match,
	},
};

module_platform_driver(samsung_abox_gic_driver);

/* Module information */
MODULE_AUTHOR("Gyeongtaek Lee, <gt82.lee@samsung.com>");
MODULE_DESCRIPTION("Samsung ASoC A-Box GIC Driver");
MODULE_ALIAS("platform:samsung-abox-gic");
MODULE_LICENSE("GPL");
