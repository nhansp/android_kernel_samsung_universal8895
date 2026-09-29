// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxim MAX77865 companion PMIC core (Samsung Galaxy Note 8, greatlte)
 *
 * Top-level page at I2C 0x66: register map and the INTSRC interrupt demultiplexer
 * (CHG, TOP, FG) as a regmap-irq domain. The charger (0x69) and the fuel gauge
 * (0x36) are their own DT I2C nodes; the MUIC (0x25) is not bound here.
 * Structure of drivers/mfd/max77705.c; spec rewrite/spec/L6F_power_input_misc.md 5.2.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77865-private.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regmap.h>

/* L6F 5.2 step 2: register ranges of 5.1 */
static const struct regmap_range max77865_readable_ranges[] = {
	regmap_reg_range(MAX77865_PMIC_REG_PMICID1, 0x03),
	regmap_reg_range(MAX77865_PMIC_REG_MCONFIG, MAX77865_PMIC_REG_MCONFIG),
	/* 0x29 = RESERVED_29, S:include/linux/mfd/max77865-private.h:46 */
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
	.cache_type = REGCACHE_NONE,	/* every read goes to the chip, as stock */
};

/* INTSRC bits, S:include/linux/mfd/max77865-private.h:30-32 */
static const struct regmap_irq max77865_irqs[] = {
	REGMAP_IRQ_REG(0, 0, MAX77865_IRQSRC_CHG),
	REGMAP_IRQ_REG(1, 0, MAX77865_IRQSRC_TOP),
	REGMAP_IRQ_REG(2, 0, MAX77865_IRQSRC_FG),
};

/*
 * No .ack_base: stock only reads INTSRC (S:drivers/mfd/max77865-irq.c:160-170); the
 * CHG source clears when the charger reads CHG_INT. regmap-irq masks the three bits
 * at registration and unmasks one when its nested IRQ is requested (L6F 5.2 step 4).
 */
static const struct regmap_irq_chip max77865_irq_chip = {
	.name		= "max77865",
	.status_base	= MAX77865_PMIC_REG_INTSRC,
	.mask_base	= MAX77865_PMIC_REG_INTSRC_MASK,
	.num_regs	= 1,
	.irqs		= max77865_irqs,
	.num_irqs	= ARRAY_SIZE(max77865_irqs),
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

	/* the charger finds the core regmap through this (L6F 5.3.2 step 12) */
	i2c_set_clientdata(i2c, max);

	/*
	 * L6F 5.2 step 3: the identity and the two masks as S-Boot/reset left them,
	 * before regmap-irq writes INTSRC_MASK (O-J2, O-J19). No revision gate.
	 */
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

	/* trigger from DT: level low, S:drivers/mfd/max77865-irq.c:286-287 */
	ret = devm_regmap_add_irq_chip(dev, max->regmap, i2c->irq, IRQF_ONESHOT, 0,
				       &max77865_irq_chip, &irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add IRQ chip\n");

	/*
	 * The haptic and RGB LED cells, and the 5.2 V boost the vibrator needs
	 * (L6F 5.2 steps 5 and 5a), belong to the haptic/LED work (L6F 5.7-5.9).
	 * Wakeup: "wakeup-source" (FDT:6173 max77865,wakeup) makes the I2C core
	 * set up the device wakeup and the wake IRQ (i2c_device_probe()).
	 */
	return 0;
}

/* as max77705 (A:drivers/mfd/max77705.c:137-160); wake arming is the I2C core's */
static int max77865_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);

	disable_irq(i2c->irq);

	return 0;
}

static int max77865_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);

	enable_irq(i2c->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(max77865_pm_ops, max77865_suspend, max77865_resume);

static const struct of_device_id max77865_i2c_of_match[] = {
	{ .compatible = "maxim,max77865" },	/* FDT:6166 */
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
