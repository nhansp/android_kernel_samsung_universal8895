// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS PHY driver data for Samsung Exynos8895 SoC
 */
#include <linux/delay.h>
#include <linux/iopoll.h>

#include "phy-samsung-ufs.h"

#define EXYNOS8895_EMBEDDED_COMBO_PHY_CTRL	0x724
#define EXYNOS8895_EMBEDDED_COMBO_PHY_CTRL_MASK	0x1
#define EXYNOS8895_EMBEDDED_COMBO_PHY_CTRL_EN	BIT(0)
#define EXYNOS8895_PHY_CDR_LOCK_STATUS		0x7f
#define EXYNOS8895_PHY_CDR_LOCK_BIT		BIT(0)
#define EXYNOS8895_PHY_CDR_TIMEOUT_US		100000
#define EXYNOS8895_PHY_CAL_WAIT_US		200

#define PHY_TRSV_REG_CFG_8895(o, v, d) \
	PHY_TRSV_REG_CFG_OFFSET(o, v, d, 0x50)

/* Calibration for phy initialization */
static const struct samsung_ufs_phy_cfg exynos8895_pre_init_cfg[] = {
	/* vendor "SQ on" rows, which the vendor driver skips before the first link startup */
	PHY_COMN_REG_CFG(0x001, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_8895(0x031, 0xd9, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_8895(0x03a, 0x77, PWR_MODE_ANY),

	PHY_COMN_REG_CFG(0x023, 0x80, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x01d, 0x10, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_8895(0x044, 0xb5, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_8895(0x04d, 0x43, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_8895(0x05b, 0x20, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_8895(0x05e, 0xc0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_8895(0x038, 0x12, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_8895(0x059, 0x58, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x023, 0xc0, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x023, 0x00, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

/* Calibration for HS mode series B, only the rows that apply to HS-G3 x2 */
static const struct samsung_ufs_phy_cfg exynos8895_pre_pwr_hs_cfg[] = {
	PHY_TRSV_REG_CFG_8895(0x032, 0xbc, PWR_MODE_HS_ANY),
	PHY_TRSV_REG_CFG_8895(0x03c, 0x7f, PWR_MODE_HS_ANY),
	PHY_TRSV_REG_CFG_8895(0x048, 0xc0, PWR_MODE_HS_ANY),
	PHY_TRSV_REG_CFG_8895(0x04a, 0x00, PWR_MODE_HS_G3_SER_B),	/* TX amplitude */
	PHY_TRSV_REG_CFG_8895(0x04b, 0x00, PWR_MODE_HS_G3_SER_B),	/* TX pre-emphasis */
	PHY_TRSV_REG_CFG_8895(0x04d, 0x63, PWR_MODE_HS_G3_SER_B),	/* TX aux cap */
	PHY_TRSV_REG_CFG_8895(0x042, 0x5d, PWR_MODE_HS_ANY),
	PHY_TRSV_REG_CFG_8895(0x043, 0x90, PWR_MODE_HS_ANY),
	END_UFS_PHY_CFG,
};

/* No register to write; the table must exist for wait_for_cdr to run */
static const struct samsung_ufs_phy_cfg exynos8895_post_pwr_hs_cfg[] = {
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg *exynos8895_ufs_phy_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT]		= exynos8895_pre_init_cfg,
	[CFG_PRE_PWR_HS]	= exynos8895_pre_pwr_hs_cfg,
	[CFG_POST_PWR_HS]	= exynos8895_post_pwr_hs_cfg,
};

static int exynos8895_ufs_phy_wait_for_cal(struct phy *phy, u8 lane)
{
	if (lane == 0)
		udelay(EXYNOS8895_PHY_CAL_WAIT_US);

	return 0;
}

static int exynos8895_ufs_phy_wait_for_cdr(struct phy *phy, u8 lane)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	u32 val;
	int err;

	if (lane != 0)
		return 0;

	err = readl_poll_timeout(ufs_phy->reg_pma +
				 PHY_APB_ADDR(EXYNOS8895_PHY_CDR_LOCK_STATUS),
				 val, val & EXYNOS8895_PHY_CDR_LOCK_BIT,
				 10, EXYNOS8895_PHY_CDR_TIMEOUT_US);
	if (err)
		dev_err(ufs_phy->dev, "failed to get phy cdr lock (%#x)\n", val);

	return err;
}

static const char * const exynos8895_ufs_phy_clks[] = {
	"ref_clk",
};

const struct samsung_ufs_phy_drvdata exynos8895_ufs_phy = {
	.cfgs = exynos8895_ufs_phy_cfgs,
	.isol = {
		.offset = EXYNOS8895_EMBEDDED_COMBO_PHY_CTRL,
		.mask = EXYNOS8895_EMBEDDED_COMBO_PHY_CTRL_MASK,
		.en = EXYNOS8895_EMBEDDED_COMBO_PHY_CTRL_EN,
	},
	.clk_list = exynos8895_ufs_phy_clks,
	.num_clks = ARRAY_SIZE(exynos8895_ufs_phy_clks),
	.wait_for_cal = exynos8895_ufs_phy_wait_for_cal,
	.wait_for_cdr = exynos8895_ufs_phy_wait_for_cdr,
};
