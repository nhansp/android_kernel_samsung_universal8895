// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxim MAX77865 companion PMIC core
 *
 * Top-level page at I2C address 0x66: the register map and the INTSRC interrupt
 * demultiplexer (CHG, TOP, FG). The charger (0x69), the fuel gauge (0x36) and
 * the MUIC (0x25) are separate I2C devices.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77865-private.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regmap.h>

static const struct regmap_range max77865_readable_ranges[] = {
	regmap_reg_range(MAX77865_PMIC_REG_PMICID1, 0x03),
	regmap_reg_range(MAX77865_PMIC_REG_MCONFIG, MAX77865_PMIC_REG_MCONFIG),
	regmap_reg_range(MAX77865_PMIC_REG_INTSRC, 0x29),
	regmap_reg_range(MAX77865_RGBLED_REG_LEDEN, MAX77865_RGBLED_REG_LEDBLNK),
	regmap_reg_range(MAX77865_PMIC_REG_BOOSTCONTROL1, MAX77865_PMIC_REG_BOOSTCONTROL1),
	regmap_reg_range(MAX77865_PMIC_REG_BOOSTCONTROL2, MAX77865_PMIC_REG_USBC_RESET),
};

static const struct regmap_range max77865_writable_ranges[] = {
	regmap_reg_range(MAX77865_PMIC_REG_MAINCTRL1, MAX77865_PMIC_REG_MAINCTRL1),
	regmap_reg_range(MAX77865_PMIC_REG_MCONFIG, MAX77865_PMIC_REG_MCONFIG),
	regmap_reg_range(MAX77865_PMIC_REG_INTSRC_MASK, MAX77865_PMIC_REG_INTSRC_MASK),
	regmap_reg_range(MAX77865_PMIC_REG_SYSTEM_INT_MASK, MAX77865_PMIC_REG_SYSTEM_INT_MASK),
	regmap_reg_range(MAX77865_RGBLED_REG_LEDEN, MAX77865_RGBLED_REG_LEDBLNK),
	regmap_reg_range(MAX77865_PMIC_REG_BOOSTCONTROL1, MAX77865_PMIC_REG_BOOSTCONTROL1),
	regmap_reg_range(MAX77865_PMIC_REG_BOOSTCONTROL2, MAX77865_PMIC_REG_BOOSTCONTROL2),
};

static const struct regmap_range max77865_volatile_ranges[] = {
	regmap_reg_range(MAX77865_PMIC_REG_INTSRC, MAX77865_PMIC_REG_INTSRC),
	regmap_reg_range(MAX77865_PMIC_REG_SYSTEM_INT, MAX77865_PMIC_REG_SYSTEM_INT),
};

static const struct regmap_access_table max77865_readable_table = {
	.yes_ranges = max77865_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77865_readable_ranges),
};

static const struct regmap_access_table max77865_writable_table = {
	.yes_ranges = max77865_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77865_writable_ranges),
};

static const struct regmap_access_table max77865_volatile_table = {
	.yes_ranges = max77865_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77865_volatile_ranges),
};

static const struct regmap_config max77865_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.rd_table = &max77865_readable_table,
	.wr_table = &max77865_writable_table,
	.volatile_table = &max77865_volatile_table,
	.max_register = MAX77865_PMIC_REG_USBC_RESET,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_irq max77865_irqs[] = {
	REGMAP_IRQ_REG(0, 0, MAX77865_IRQSRC_CHG),
	REGMAP_IRQ_REG(1, 0, MAX77865_IRQSRC_TOP),
	REGMAP_IRQ_REG(2, 0, MAX77865_IRQSRC_FG),
};

/* INTSRC is not acked: each source clears when its own interrupt register is read */
static const struct regmap_irq_chip max77865_irq_chip = {
	.name		= "max77865",
	.status_base	= MAX77865_PMIC_REG_INTSRC,
	.mask_base	= MAX77865_PMIC_REG_INTSRC_MASK,
	.num_regs	= 1,
	.irqs		= max77865_irqs,
	.num_irqs	= ARRAY_SIZE(max77865_irqs),
};

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
/*
 * SYSTEM_INT behind the TOP source. All its sources (BSTEN, SYSUVLO, SYSOVLO,
 * TSHDN, TM) are declared so that regmap-irq masks them before TOP is unmasked;
 * only SYSOVLO is requested, by the charger.
 */
static const struct regmap_irq max77865_sys_irqs[] = {
	REGMAP_IRQ_REG(3, 0, BIT(3)),
	REGMAP_IRQ_REG(4, 0, BIT(4)),
	REGMAP_IRQ_REG(5, 0, MAX77865_SYSTEM_INT_SYSOVLO),
	REGMAP_IRQ_REG(6, 0, BIT(6)),
	REGMAP_IRQ_REG(7, 0, BIT(7)),
};

static const struct regmap_irq_chip max77865_sys_irq_chip = {
	.name		= "max77865-sys",
	.status_base	= MAX77865_PMIC_REG_SYSTEM_INT,
	.mask_base	= MAX77865_PMIC_REG_SYSTEM_INT_MASK,
	.num_regs	= 1,
	.irqs		= max77865_sys_irqs,
	.num_irqs	= ARRAY_SIZE(max77865_sys_irqs),
};
#endif

static const struct mfd_cell max77865_cells[] = {
	MFD_CELL_OF("max77865-haptic", NULL, NULL, 0, 0, "maxim,max77865-haptic"),
	MFD_CELL_OF("max77865-rgb", NULL, NULL, 0, 0, "maxim,max77865-rgb"),
};

static int max77865_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regmap_irq_chip_data *irq_data;
	struct max77693_dev *max;
	unsigned int id, rev, src_mask, sys_mask;
	int ret;

	max = devm_kzalloc(dev, sizeof(*max), GFP_KERNEL);
	if (!max)
		return -ENOMEM;

	max->dev = dev;
	max->i2c = i2c;
	max->type = TYPE_MAX77865;
	max->irq = i2c->irq;

	max->regmap = devm_regmap_init_i2c(i2c, &max77865_regmap_config);
	if (IS_ERR(max->regmap))
		return dev_err_probe(dev, PTR_ERR(max->regmap), "regmap init failed\n");

	i2c_set_clientdata(i2c, max);

	/* the masks are reported as the bootloader left them, before regmap-irq writes them */
	ret = regmap_read(max->regmap, MAX77865_PMIC_REG_PMICID1, &id);
	if (!ret)
		ret = regmap_read(max->regmap, MAX77865_PMIC_REG_PMICREV, &rev);
	if (!ret)
		ret = regmap_read(max->regmap, MAX77865_PMIC_REG_INTSRC_MASK, &src_mask);
	if (!ret)
		ret = regmap_read(max->regmap, MAX77865_PMIC_REG_SYSTEM_INT_MASK, &sys_mask);
	if (ret)
		return dev_err_probe(dev, ret, "device not found\n");

	dev_info(dev, "PMIC ID 0x%02x REV 0x%02x INTSRC_MASK 0x%02x SYS_INT_MASK 0x%02x\n",
		 id, rev, src_mask, sys_mask);

	ret = devm_regmap_add_irq_chip(dev, max->regmap, i2c->irq, IRQF_ONESHOT, 0,
				       &max77865_irq_chip, &irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add IRQ chip\n");

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
	ret = devm_regmap_add_irq_chip(dev, max->regmap,
				       regmap_irq_get_virq(irq_data, 1),
				       IRQF_ONESHOT, 0,
				       &max77865_sys_irq_chip,
				       &max->sys_irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add SYS IRQ chip\n");
#endif

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, max77865_cells,
				   ARRAY_SIZE(max77865_cells), NULL, 0,
				   regmap_irq_get_domain(irq_data));
	if (ret)
		return dev_err_probe(dev, ret, "failed to add MFD cells\n");

	/* the I2C core only does this for an interrupt named "wakeup" */
	if (of_property_read_bool(dev->of_node, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return dev_err_probe(dev, ret, "failed to init wakeup\n");
	}

	/* the 5.2 V boost of the motor; the haptic cell switches it around suspend */
	ret = regmap_update_bits(max->regmap, MAX77865_PMIC_REG_BOOSTCONTROL1,
				 MAX77865_BOOSTCONTROL1_BSTOUT, 0x00);
	if (!ret)
		ret = regmap_update_bits(max->regmap, MAX77865_PMIC_REG_BOOSTCONTROL2,
					 MAX77865_BOOSTCONTROL2_FORCE_EN,
					 MAX77865_BOOSTCONTROL2_FORCE_EN);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable 5.2V booster\n");

	return 0;
}

static int max77865_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(i2c->irq);

	disable_irq(i2c->irq);

	return 0;
}

static int max77865_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(i2c->irq);

	enable_irq(i2c->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(max77865_pm_ops, max77865_suspend, max77865_resume);

static const struct of_device_id max77865_i2c_of_match[] = {
	{ .compatible = "maxim,max77865" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77865_i2c_of_match);

static struct i2c_driver max77865_i2c_driver = {
	.driver = {
		.name		= "max77865",
		.of_match_table	= max77865_i2c_of_match,
		.pm		= pm_sleep_ptr(&max77865_pm_ops),
	},
	.probe = max77865_i2c_probe,
};
module_i2c_driver(max77865_i2c_driver);

MODULE_DESCRIPTION("Maxim MAX77865 PMIC core driver");
MODULE_LICENSE("GPL");
