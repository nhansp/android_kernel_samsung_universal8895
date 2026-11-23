// SPDX-License-Identifier: GPL-2.0+
//
// Samsung S2DOS03 display power regulators
//
// The S2DOS03 is an I2C PMIC with four LDOs and one buck, used on Exynos8895
// phones to supply the display panel. It is not register compatible with
// the S2DOS05.

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define S2DOS03_REG_DEV_ID		0x00
#define S2DOS03_REG_EN			0x03
#define S2DOS03_REG_LDO1_CFG		0x10
#define S2DOS03_REG_LDO2_CFG		0x11
#define S2DOS03_REG_LDO3_CFG		0x12
#define S2DOS03_REG_LDO4_CFG		0x13
#define S2DOS03_REG_BUCK_CFG		0x20
#define S2DOS03_REG_BUCK_VOUT		0x21

#define S2DOS03_BUCK_MIN1		500000
#define S2DOS03_LDO_MIN1		600000
#define S2DOS03_BUCK_STEP1		6250
#define S2DOS03_LDO_STEP1		25000
#define S2DOS03_LDO_VSEL_MASK		0x7f
#define S2DOS03_BUCK_VSEL_MASK		0xff
#define S2DOS03_ENABLE_MASK_L1		BIT(0)
#define S2DOS03_ENABLE_MASK_L2		BIT(1)
#define S2DOS03_ENABLE_MASK_L3		BIT(2)
#define S2DOS03_ENABLE_MASK_L4		BIT(3)
#define S2DOS03_ENABLE_MASK_B1		BIT(4)
#define S2DOS03_RAMP_DELAY		12000	/* uV/us */
#define S2DOS03_ENABLE_TIME_LDO		50	/* us */
#define S2DOS03_ENABLE_TIME_BUCK	350	/* us */

enum s2dos03_regulators {
	S2DOS03_LDO1,
	S2DOS03_LDO2,
	S2DOS03_LDO3,
	S2DOS03_LDO4,
	S2DOS03_BUCK1,
	S2DOS03_REGULATOR_MAX,
};

/* As the vendor driver, only wait when the voltage rises. */
static int s2dos03_set_voltage_time_sel(struct regulator_dev *rdev,
					unsigned int old_selector,
					unsigned int new_selector)
{
	int old_uV = regulator_list_voltage_linear(rdev, old_selector);
	int new_uV = regulator_list_voltage_linear(rdev, new_selector);

	if (old_uV < 0 || new_uV < 0)
		return -EINVAL;
	if (old_selector < new_selector)
		return DIV_ROUND_UP(new_uV - old_uV, S2DOS03_RAMP_DELAY);

	return 0;
}

static const struct regulator_ops s2dos03_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2dos03_set_voltage_time_sel,
};

#define S2DOS03_DESC(_name, _id, _min, _step, _n, _vreg, _vmask, _emask, _t) { \
	.name		= _name,					\
	.of_match	= of_match_ptr(_name),				\
	.regulators_node = of_match_ptr("regulators"),			\
	.id		= _id,						\
	.ops		= &s2dos03_ops,					\
	.type		= REGULATOR_VOLTAGE,				\
	.owner		= THIS_MODULE,					\
	.min_uV		= _min,						\
	.uV_step	= _step,					\
	.n_voltages	= _n,						\
	.vsel_reg	= _vreg,					\
	.vsel_mask	= _vmask,					\
	.enable_reg	= S2DOS03_REG_EN,				\
	.enable_mask	= _emask,					\
	.enable_time	= _t,						\
}

static const struct regulator_desc s2dos03_regulators[S2DOS03_REGULATOR_MAX] = {
	S2DOS03_DESC("ldo1", S2DOS03_LDO1, S2DOS03_LDO_MIN1, S2DOS03_LDO_STEP1,
		     S2DOS03_LDO_VSEL_MASK + 1, S2DOS03_REG_LDO1_CFG,
		     S2DOS03_LDO_VSEL_MASK, S2DOS03_ENABLE_MASK_L1,
		     S2DOS03_ENABLE_TIME_LDO),
	S2DOS03_DESC("ldo2", S2DOS03_LDO2, S2DOS03_LDO_MIN1, S2DOS03_LDO_STEP1,
		     S2DOS03_LDO_VSEL_MASK + 1, S2DOS03_REG_LDO2_CFG,
		     S2DOS03_LDO_VSEL_MASK, S2DOS03_ENABLE_MASK_L2,
		     S2DOS03_ENABLE_TIME_LDO),
	S2DOS03_DESC("ldo3", S2DOS03_LDO3, S2DOS03_LDO_MIN1, S2DOS03_LDO_STEP1,
		     S2DOS03_LDO_VSEL_MASK + 1, S2DOS03_REG_LDO3_CFG,
		     S2DOS03_LDO_VSEL_MASK, S2DOS03_ENABLE_MASK_L3,
		     S2DOS03_ENABLE_TIME_LDO),
	S2DOS03_DESC("ldo4", S2DOS03_LDO4, S2DOS03_LDO_MIN1, S2DOS03_LDO_STEP1,
		     S2DOS03_LDO_VSEL_MASK + 1, S2DOS03_REG_LDO4_CFG,
		     S2DOS03_LDO_VSEL_MASK, S2DOS03_ENABLE_MASK_L4,
		     S2DOS03_ENABLE_TIME_LDO),
	S2DOS03_DESC("buck1", S2DOS03_BUCK1, S2DOS03_BUCK_MIN1, S2DOS03_BUCK_STEP1,
		     S2DOS03_BUCK_VSEL_MASK + 1, S2DOS03_REG_BUCK_VOUT,
		     S2DOS03_BUCK_VSEL_MASK, S2DOS03_ENABLE_MASK_B1,
		     S2DOS03_ENABLE_TIME_BUCK),
};

static const struct regmap_config s2dos03_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= S2DOS03_REG_BUCK_VOUT,
};

static int s2dos03_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regulator_config config = { };
	struct regmap *regmap;
	unsigned int en;
	int i, ret;

	regmap = devm_regmap_init_i2c(i2c, &s2dos03_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "regmap init failed\n");

	ret = regmap_read(regmap, S2DOS03_REG_EN, &en);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read EN\n");
	dev_dbg(dev, "EN 0x%02x\n", en);

	config.dev = dev;
	config.regmap = regmap;

	for (i = 0; i < S2DOS03_REGULATOR_MAX; i++) {
		const struct regulator_desc *desc = &s2dos03_regulators[i];
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n", desc->name);
	}

	return 0;
}

static const struct of_device_id s2dos03_of_match[] = {
	{ .compatible = "samsung,s2dos03" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2dos03_of_match);

static const struct i2c_device_id s2dos03_i2c_id[] = {
	{ "s2dos03" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2dos03_i2c_id);

static struct i2c_driver s2dos03_i2c_driver = {
	.driver = {
		.name = "s2dos03",
		.of_match_table = s2dos03_of_match,
	},
	.probe = s2dos03_probe,
	.id_table = s2dos03_i2c_id,
};
module_i2c_driver(s2dos03_i2c_driver);

MODULE_DESCRIPTION("Samsung S2DOS03 display power regulator driver");
MODULE_LICENSE("GPL");
