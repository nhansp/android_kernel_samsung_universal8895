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
#include <linux/mfd/core.h>
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

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
/*
 * TOP sub-chip on SYSTEM_INT for SYSOVLO. Every SYS_INT source stock declared is
 * listed (Samsung's drivers/mfd/max77865-irq.c: BSTEN 3, SYSUVLO 4,
 * SYSOVLO 5, TSHDN 6, TM 7), so regmap-irq masks all of them at registration
 * (it only touches the bits of declared irqs) before it unmasks TOP in INTSRC_MASK;
 * only SYSOVLO is ever requested (by the charger).
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

/* L6F 5.2 step 5: haptic and RGB LED child cells */
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

#if IS_ENABLED(CONFIG_CHARGER_MAX77865_SYSOVLO)
	ret = devm_regmap_add_irq_chip(dev, max->regmap,
				       regmap_irq_get_virq(irq_data, 1),
				       IRQF_ONESHOT, 0,
				       &max77865_sys_irq_chip,
				       &max->sys_irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add SYS IRQ chip\n");
#endif

	/*
	 * L6F 5.2 step 5: haptic and RGB LED child cells.
	 */
	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, max77865_cells,
				   ARRAY_SIZE(max77865_cells), NULL, 0,
				   regmap_irq_get_domain(irq_data));
	if (ret)
		return dev_err_probe(dev, ret, "failed to add MFD cells\n");

	/*
	 * Wakeup source for the PMIC INT pad (gpa1-6, FDT:467-474/6166-6173 "max77865,wakeup";
	 * the board spells it wakeup-source, arch/arm64/boot/dts/exynos/exynos8895-greatlte-l6f.dtsi:237):
	 * stock arms it from the same DT flag, device_init_wakeup(max77865->dev, pdata->wakeup)
	 * S:drivers/mfd/max77865.c:196,306. The I2C core only does this for a node with
	 * interrupt-names = "wakeup" (A:drivers/i2c/i2c-core-base.c:547-569), which this node
	 * has not, so the driver has to do it itself. Without it the whole max77865 interrupt
	 * tree (the charger's CHGIN source, the detach trigger) is silent across suspend.
	 */
	if (of_property_read_bool(dev->of_node, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return dev_err_probe(dev, ret, "failed to init wakeup\n");
	}

	/*
	 * L6F 5.2 step 5a: 5.2 V boost on, stock order (after mfd_add_devices,
	 * S:drivers/mfd/max77865.c:301-310 "Enable booster 5.2v for vibrator").
	 * The haptic driver switches it off/on around suspend and at shutdown (L6F 5.7 item 7).
	 */
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

/*
 * The disable_irq() pair is ACK's max77705 (A:drivers/mfd/max77705.c:137-160); the wakeup
 * half of that pattern, which max77705 also has, was missing here and is stock's:
 * enable_irq_wake() before disable_irq() at suspend, disable_irq_wake() before enable_irq()
 * at resume, S:drivers/mfd/max77865.c:351-364 and :366-379. Without it the PMIC INT pad
 * cannot wake the system, so a charger/MUIC detach during suspend stays unreported.
 */
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
