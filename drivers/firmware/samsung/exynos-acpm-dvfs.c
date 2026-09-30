// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC.
 * Copyright 2025 Linaro Ltd.
 */

#include <linux/bitfield.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/ktime.h>
#include <linux/types.h>
#include <linux/units.h>

#include "exynos-acpm.h"
#include "exynos-acpm-dvfs.h"

#define ACPM_DVFS_ID			GENMASK(11, 0)
#define ACPM_DVFS_REQ_TYPE		GENMASK(15, 0)

#define ACPM_DVFS_FREQ_REQ		0
#define ACPM_DVFS_FREQ_GET		1
#define ACPM_DVFS_COLDTEMP_REQ		3	/* Exynos8895: S:drivers/soc/samsung/cal-if/acpm_dvfs.h:22 */

/* Exynos8895 devfreq requests (S:include/soc/samsung/exynos-devfreq.h:22,24) */
#define ACPM_DVFS_DATA_INIT		5
#define ACPM_DVFS_RELEASE		2

static void acpm_dvfs_set_xfer(struct acpm_xfer *xfer, u32 *cmd, size_t cmdlen,
			       unsigned int acpm_chan_id, bool response)
{
	xfer->acpm_chan_id = acpm_chan_id;
	xfer->txd = cmd;
	xfer->txlen = cmdlen;

	if (response) {
		xfer->rxd = cmd;
		xfer->rxlen = cmdlen;
	}
}

static void acpm_dvfs_init_set_rate_cmd(u32 cmd[4], unsigned int clk_id,
					unsigned long rate)
{
	cmd[0] = FIELD_PREP(ACPM_DVFS_ID, clk_id);
	cmd[1] = rate / HZ_PER_KHZ;
	cmd[2] = FIELD_PREP(ACPM_DVFS_REQ_TYPE, ACPM_DVFS_FREQ_REQ);
}

int acpm_dvfs_set_rate(struct acpm_handle *handle,
		       unsigned int acpm_chan_id, unsigned int clk_id,
		       unsigned long rate)
{
	struct acpm_xfer xfer = {0};
	u32 cmd[4];

	acpm_dvfs_init_set_rate_cmd(cmd, clk_id, rate);
	cmd[3] = acpm_cmd_timestamp(handle);
	acpm_dvfs_set_xfer(&xfer, cmd, sizeof(cmd), acpm_chan_id, false);

	return acpm_do_xfer(handle, &xfer);
}

static void acpm_dvfs_init_get_rate_cmd(u32 cmd[4], unsigned int clk_id)
{
	cmd[0] = FIELD_PREP(ACPM_DVFS_ID, clk_id);
	cmd[2] = FIELD_PREP(ACPM_DVFS_REQ_TYPE, ACPM_DVFS_FREQ_GET);
}

/*
 * Tell the firmware that a DVFS domain is cold (below 15 C, or the system is
 * suspending) or no longer cold. Exynos8895 stock exynos_acpm_set_cold_temp()
 * (S:drivers/soc/samsung/cal-if/acpm_dvfs.c:99-123): cmd = {domain, is_cold,
 * COLDTEMP_REQ, 0}, response expected.
 */
int acpm_dvfs_set_cold_temp(struct acpm_handle *handle,
			    unsigned int acpm_chan_id, unsigned int clk_id,
			    bool cold)
{
	struct acpm_xfer xfer = {0};
	u32 cmd[4] = {0};

	cmd[0] = FIELD_PREP(ACPM_DVFS_ID, clk_id);
	cmd[1] = cold;
	cmd[2] = FIELD_PREP(ACPM_DVFS_REQ_TYPE, ACPM_DVFS_COLDTEMP_REQ);
	cmd[3] = acpm_cmd_timestamp(handle);
	acpm_dvfs_set_xfer(&xfer, cmd, sizeof(cmd), acpm_chan_id, true);

	return acpm_do_xfer(handle, &xfer);
}

/*
 * Tell the firmware whether the AP is running or going to sleep, for one
 * devfreq domain (Exynos8895 MIF 0, INT 1). Stock exynos-devfreq sends it on
 * its channel at suspend with "running" false and at resume with true
 * (S:drivers/devfreq/exynos/exynos-devfreq.c:1901-1914, 1988-2000):
 * cmd = {type, running, DATA_INIT, RELEASE}, response expected. Stock only
 * checks the send result; so does the caller.
 */
int acpm_dvfs_sleep_release(struct acpm_handle *handle,
			    unsigned int acpm_chan_id, unsigned int type,
			    bool running)
{
	u32 cmd[4] = { type, running, ACPM_DVFS_DATA_INIT, ACPM_DVFS_RELEASE };
	struct acpm_xfer xfer = {0};

	acpm_dvfs_set_xfer(&xfer, cmd, sizeof(cmd), acpm_chan_id, true);

	return acpm_do_xfer(handle, &xfer);
}

/*
 * One DATA_INIT message of a devfreq domain's table (Exynos8895 MIF): stock
 * exynos8895_mif_constraint_parse() sends {level, kHz, DATA_INIT, INT
 * constraint kHz} per enabled MIF level and then {count, initial kHz,
 * DATA_INIT, SET_CONST} on the domain's channel, response expected, indirection
 * off (S:drivers/devfreq/exynos/exynos8895_bus_mif.c:121-123,130-133,151-154).
 * Stock only checks the send result (:142-146,156-160); so does the caller.
 */
int acpm_dvfs_data_init(struct acpm_handle *handle, unsigned int acpm_chan_id,
			u32 cmd0, u32 cmd1, u32 cmd3)
{
	u32 cmd[4] = { cmd0, cmd1, ACPM_DVFS_DATA_INIT, cmd3 };
	struct acpm_xfer xfer = {0};

	acpm_dvfs_set_xfer(&xfer, cmd, sizeof(cmd), acpm_chan_id, true);

	return acpm_do_xfer(handle, &xfer);
}

unsigned long acpm_dvfs_get_rate(struct acpm_handle *handle,
				 unsigned int acpm_chan_id, unsigned int clk_id)
{
	struct acpm_xfer xfer;
	unsigned int cmd[4] = {0};
	int ret;

	acpm_dvfs_init_get_rate_cmd(cmd, clk_id);
	cmd[3] = acpm_cmd_timestamp(handle);
	acpm_dvfs_set_xfer(&xfer, cmd, sizeof(cmd), acpm_chan_id, true);

	ret = acpm_do_xfer(handle, &xfer);
	if (ret)
		return 0;

	return xfer.rxd[1] * HZ_PER_KHZ;
}
