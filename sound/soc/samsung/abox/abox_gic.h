/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * sound/soc/samsung/abox/abox_gic.h - Samsung ABOX GIC driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 *
 * Transplanted from S:sound/soc/samsung/abox/abox_gic.h (Samsung
 * N950NKSU5DZE2 OSRC 4.4.111), spec L6E 5.7.1.
 */

#ifndef __SND_SOC_ABOX_GIC_H
#define __SND_SOC_ABOX_GIC_H

#include <linux/interrupt.h>
#include <linux/platform_device.h>

struct abox_gic_irq_handler_t {
	irq_handler_t irq_handler;
	void *dev_id;
};

struct abox_gic_data {
	void __iomem *gicd_base;
	void __iomem *gicc_base;
	phys_addr_t gicd_base_phys;
	phys_addr_t gicc_base_phys;
	int irq;
	struct abox_gic_irq_handler_t abox_gic_irq_handler;
	bool disabled;

};

/**
 * Generate interrupt
 * @param[in]	pdev	pointer to platform_device of abox_gic device
 * @param[in]	hw_irq	hardware irq number
 */
extern void abox_gic_generate_interrupt(struct platform_device *pdev,
		int hw_irq);

/**
 * Register interrupt handler
 * @param[in]	pdev		pointer to platform_device of abox_gic device
 * @param[in]	irq_handler	function to be called on interrupt
 * @param[in]	dev_id		Cookie for interrupt.
 */
extern void abox_gic_register_irq_handler(struct platform_device *pdev,
		irq_handler_t irq_handler, void *dev_id);

/**
 * Unregister interrupt handler
 * @param[in]	pdev	pointer to platform_device of abox_gic device
 */
extern void abox_gic_unregister_irq_handler(struct platform_device *pdev);

/**
 * Enable ABOX GIC irq
 * @param[in]	dev	pointer to platform_device of abox_gic device
 */
extern int abox_gic_enable_irq(struct device *dev);

/**
 * Disable ABOX GIC irq
 * @param[in]	dev	pointer to platform_device of abox_gic device
 */
extern int abox_gic_disable_irq(struct device *dev);

/**
 * Initialize A-Box GIC
 * @param[in]	pdev	pointer to platform_device of abox_gic device
 */
extern void abox_gic_init_gic(struct platform_device *pdev);

#endif /* __SND_SOC_ABOX_GIC_H */
