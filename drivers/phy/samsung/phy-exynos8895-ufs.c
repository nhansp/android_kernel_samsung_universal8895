// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS PHY driver data for Samsung Exynos 8895 SoC
 *
 * Every value comes from the ufs node of the stock SM-N950N device tree
 * (arch/arm64/boot/dts/exynos/exynos8895.dtsi of the 4.4.111 kernel, tables
 * post-clk-on, phy-init, calib-of-hs-rate-b and post-calib-of-hs-rate-b),
 * which are identical in the shipped DTB. PMA registers are 32-bit words at
 * byte offset (index << 2); lane 1 of a TRSV register is 0x140 bytes above
 * lane 0 (stock PHY_PMA_TRSV_ADDR(reg, lane) = reg + 0x140 * lane).
 *
 * Citations: dtsi:N = S:arch/arm64/boot/dts/exynos/exynos8895.dtsi:N,
 * ufs-exynos.c:N = S:drivers/scsi/ufs/ufs-exynos.c:N (stock 4.4.111 tree).
 */
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/moduleparam.h>

#include "phy-samsung-ufs.h"

#define EXYNOS8895_EMBEDDED_COMBO_PHY_CTRL	0x724	/* PMU 0x16480724, dtsi:1072 */
#define EXYNOS8895_EMBEDDED_COMBO_PHY_CTRL_MASK	0x1	/* ufs-exynos.c:291-295 writel(!!en) */
#define EXYNOS8895_EMBEDDED_COMBO_PHY_CTRL_EN	BIT(0)	/* ufs-exynos.c:291-295 writel(!!en) */
#define EXYNOS8895_PHY_CDR_LOCK_STATUS		0x7f	/* TRSV byte 0x1fc PHY_CDR_WAIT, dtsi:1035 */
#define EXYNOS8895_PHY_CDR_LOCK_BIT		BIT(0)	/* PHY_CDR_WAIT value 0x01, dtsi:1035 */
#define EXYNOS8895_PHY_CDR_TIMEOUT_US		100000	/* ufs-exynos.c:903 msecs_to_jiffies(100) */
#define EXYNOS8895_PHY_CAL_WAIT_US		200	/* COMMON_WAIT 0xc8, dtsi:920; ufs-exynos.c:1063-1066 */

/* TRSV lane stride 0x140 bytes = 0x50 words: S:drivers/scsi/ufs/ufs-exynos.h:465 */
#define PHY_TRSV_REG_CFG_8895(o, v, d) \
	PHY_TRSV_REG_CFG_OFFSET(o, v, d, 0x50)

/*
 * Before every link startup (and right after the isolation release on
 * resume): post-clk-on "SQ on", which stock writes before every resume
 * link-up, then the PMA rows of phy-init in stock order. The UniPro and
 * PCS rows of phy-init are written by the host driver.
 */
static const struct samsung_ufs_phy_cfg exynos8895_pre_init_cfg[] = {
	/* post-clk-on */
	PHY_COMN_REG_CFG(0x001, 0x00, PWR_MODE_ANY),		/* 0x004 dtsi:1050 */
	PHY_TRSV_REG_CFG_8895(0x031, 0xd9, PWR_MODE_ANY),	/* 0x0c4 dtsi:1051 */
	PHY_TRSV_REG_CFG_8895(0x03a, 0x77, PWR_MODE_ANY),	/* 0x0e8 dtsi:1052 */

	/* phy-init, PMA part */
	PHY_COMN_REG_CFG(0x023, 0x80, PWR_MODE_ANY),		/* 0x08c dtsi:910 */
	PHY_COMN_REG_CFG(0x01d, 0x10, PWR_MODE_ANY),		/* 0x074 dtsi:911: PWM clock from refclk */
	PHY_TRSV_REG_CFG_8895(0x044, 0xb5, PWR_MODE_ANY),	/* 0x110 dtsi:912 */
	PHY_TRSV_REG_CFG_8895(0x04d, 0x43, PWR_MODE_ANY),	/* 0x134 dtsi:913 */
	PHY_TRSV_REG_CFG_8895(0x05b, 0x20, PWR_MODE_ANY),	/* 0x16c dtsi:914 */
	PHY_TRSV_REG_CFG_8895(0x05e, 0xc0, PWR_MODE_ANY),	/* 0x178 dtsi:915 */
	PHY_TRSV_REG_CFG_8895(0x038, 0x12, PWR_MODE_ANY),	/* 0x0e0 dtsi:916 */
	PHY_TRSV_REG_CFG_8895(0x059, 0x58, PWR_MODE_ANY),	/* 0x164 dtsi:917 */
	PHY_COMN_REG_CFG(0x023, 0xc0, PWR_MODE_ANY),		/* 0x08c dtsi:918 */
	PHY_COMN_REG_CFG(0x023, 0x00, PWR_MODE_ANY),		/* 0x08c dtsi:919 */
	END_UFS_PHY_CFG,
};

/*
 * calib-of-hs-rate-b, the PMA rows that apply to HS-G3 with 2 lanes (the
 * only mode this host requests). The framework writes every row whatever
 * its .desc, so the G1/G2 rows of the stock table are not listed.
 */
static const struct samsung_ufs_phy_cfg exynos8895_pre_pwr_hs_cfg[] = {
	PHY_TRSV_REG_CFG_8895(0x032, 0xbc, PWR_MODE_HS_ANY),		/* 0x0c8 dtsi:1011 */
	PHY_TRSV_REG_CFG_8895(0x03c, 0x7f, PWR_MODE_HS_ANY),		/* 0x0f0 dtsi:1012 */
	PHY_TRSV_REG_CFG_8895(0x048, 0xc0, PWR_MODE_HS_ANY),		/* 0x120 dtsi:1013 */
	PHY_TRSV_REG_CFG_8895(0x04a, 0x00, PWR_MODE_HS_G3_SER_B),	/* 0x128 dtsi:1016: G3 amplitude */
	PHY_TRSV_REG_CFG_8895(0x04b, 0x00, PWR_MODE_HS_G3_SER_B),	/* 0x12c dtsi:1017: G1/G3 pre-emphasis */
	PHY_TRSV_REG_CFG_8895(0x04d, 0x63, PWR_MODE_HS_G3_SER_B),	/* 0x134 dtsi:1021: G3 aux cap */
	PHY_TRSV_REG_CFG_8895(0x042, 0x5d, PWR_MODE_HS_ANY),		/* 0x108 dtsi:1023 */
	PHY_TRSV_REG_CFG_8895(0x043, 0x90, PWR_MODE_HS_ANY),		/* 0x10c dtsi:1024 */
	END_UFS_PHY_CFG,
};

/* post-calib-of-hs-rate-b (dtsi:1034-1036) has no register row, only the CDR wait below. The
 * table must exist: a NULL table skips the wait. */
static const struct samsung_ufs_phy_cfg exynos8895_post_pwr_hs_cfg[] = {
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg *exynos8895_ufs_phy_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT]		= exynos8895_pre_init_cfg,
	[CFG_PRE_PWR_HS]	= exynos8895_pre_pwr_hs_cfg,
	[CFG_POST_PWR_HS]	= exynos8895_post_pwr_hs_cfg,
};

#if IS_ENABLED(CONFIG_GREATLTE_HB)
/*
 * DEBUG: deviation D-3 test (spec/L4_storage.md 5.2): phy_exynos_ufs.d3_no_t0=1
 * starts CFG_PRE_INIT after the three post-clk-on rows, i.e. the PMA state
 * before the first link-up is the one stock left untouched. Parsed before any
 * initcall, so the PHY probe already sees the shortened table.
 */
#define EXYNOS8895_POST_CLK_ON_ROWS	3	/* first rows of exynos8895_pre_init_cfg */

static bool exynos8895_d3_no_t0;

static int exynos8895_d3_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (ret)
		return ret;
	exynos8895_ufs_phy_cfgs[CFG_PRE_INIT] = exynos8895_pre_init_cfg +
		(exynos8895_d3_no_t0 ? EXYNOS8895_POST_CLK_ON_ROWS : 0);
	return 0;
}

static const struct kernel_param_ops exynos8895_d3_ops = {
	.set = exynos8895_d3_set,
	.get = param_get_bool,
};
module_param_cb(d3_no_t0, &exynos8895_d3_ops, &exynos8895_d3_no_t0, 0444);
MODULE_PARM_DESC(d3_no_t0, "DEBUG 8895: no post-clk-on rows before link startup (D-3)");
#endif

/* COMMON_WAIT of phy-init: one 200 us delay after the whole table (lane 0 only). */
static int exynos8895_ufs_phy_wait_for_cal(struct phy *phy, u8 lane)
{
	if (lane == 0)
		udelay(EXYNOS8895_PHY_CAL_WAIT_US);

	return 0;
}

/* PHY_CDR_WAIT: bit 0 of TRSV 0x1fc of lane 0, up to 100 ms. */
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
		dev_err(ufs_phy->dev, "8895: CDR lock timeout (0x1fc=%#x)\n", val);

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
