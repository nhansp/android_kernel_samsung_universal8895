// SPDX-License-Identifier: GPL-2.0-only
/*
 * The vendor DSIM command path of the panel framework (panel->mipi_drv) on the
 * DRM panel's mipi_dsi_device. Each vendor packet is one host transfer.
 */

#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/exynos8895_hiber.h>
#include <video/mipi_display.h>

#include "panel_dpu.h"
#include "panel.h"

#define DSIM_FIFO_SIZE		512
#define SIDE_RAM_ALIGN_CNT	12

/* the vendor ops take a DSIM id; there is one DSIM */
static struct panel_device *panel_dsi_panel;

/* one packet as the vendor dsim_write_data() sends it, in HS mode */
static int panel_dsi_write_packet(struct mipi_dsi_device *dsi, u8 type,
				  const u8 *cmd, int size)
{
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.type = type,
		.tx_buf = cmd,
		.tx_len = size,
	};
	ssize_t ret;

	if (!ops || !ops->transfer)
		return -ENOSYS;

	ret = ops->transfer(dsi->host, &msg);
	return ret < 0 ? ret : size;
}

static int __mipi_write_data(struct mipi_dsi_device *dsi, u8 cmd_id,
			     const u8 *cmd, int size)
{
	int ret = 0;
	int retry = 5;
	u8 dsi_cmd;

	if (cmd_id == MIPI_DSI_WRITE) {
		if (size == 1)
			dsi_cmd = MIPI_DSI_DCS_SHORT_WRITE;
		/*
		 * TE_ON (35h) with one parameter goes as a long packet: as a
		 * short one the DSI would turn the bus around (BTA).
		 */
		else if (size == 2 && cmd && cmd[0] != 0x35)
			dsi_cmd = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		else
			dsi_cmd = MIPI_DSI_DCS_LONG_WRITE;
	} else {
		dsi_cmd = cmd_id;
	}

write_command:
	ret = panel_dsi_write_packet(dsi, dsi_cmd, cmd, size);
	if (ret != size) {
		retry--;
		if (retry) {
			pr_info("INFO:%s:retry dsim write command : %d\n", __func__, retry);
			goto write_command;
		} else {
			pr_err("ERR:%s:failed to dsim write command\n", __func__);
		}
	}
	return ret;
}

static int mipi_write_gen_cmd(struct mipi_dsi_device *dsi, u8 dsi_type,
			      const u8 *cmd, int size)
{
	u8 cmd_buf[DSIM_FIFO_SIZE];
	u8 g_param[2] = {0xB0, 0x00};
	int ret, remained, tx_size, data_offset, tx_data_size;
	int fifo_size = DSIM_FIFO_SIZE;

	data_offset = 1;
	remained = size - data_offset;

	do {
		if (g_param[1] != 0) {
			ret = __mipi_write_data(dsi, MIPI_DSI_WRITE, g_param, 2);
			if (ret != 2) {
				pr_err("DISP:ERR:%s:failed to write global command\n", __func__);
				goto fail_write_command;
			}

			cmd_buf[0] = cmd[0];
			tx_size = min(remained + 1, fifo_size);

			tx_data_size = (tx_size - 1);
			memcpy(cmd_buf + 1, cmd + data_offset + g_param[1], tx_data_size);
			ret = __mipi_write_data(dsi, MIPI_DSI_WRITE, cmd_buf, tx_size);
			if (ret != tx_size) {
				pr_err("DISP:ERR:%s:failed to write command\n", __func__);
				goto fail_write_command;
			}
		} else {
			tx_size = min(remained + data_offset, fifo_size);
			tx_data_size = (tx_size - data_offset);
			ret = __mipi_write_data(dsi, dsi_type, cmd, tx_size);
			if (ret != tx_size) {
				pr_err("DISP:ERR:%s:failed to write command\n", __func__);
				goto fail_write_command;
			}
		}

		g_param[1] += tx_data_size;
		remained -= tx_data_size;
	} while (remained > 0);

fail_write_command:
	return ret;
}

static int mipi_write_pps_cmd(struct mipi_dsi_device *dsi, const u8 *cmd, int size)
{
	int ret;
	u8 cmd_buf[DSIM_FIFO_SIZE];
	u8 g_param[2] = {0xB0, 0x00};
	s32 t_tx_size, tx_size, remind_size;
	s32 fifo_size = DSIM_FIFO_SIZE;

	tx_size = (size > fifo_size) ? fifo_size : size;

	ret = __mipi_write_data(dsi, MIPI_DSI_DSC_PPS, cmd, tx_size);
	if (ret != tx_size) {
		pr_err("DISP:ERR:%s:failed to write command\n", __func__);
		goto err_write_gen_cmd;
	}
	t_tx_size = tx_size;
	remind_size = size - t_tx_size;

	while (remind_size) {
		g_param[1] = (u8)t_tx_size;
		ret = __mipi_write_data(dsi, MIPI_DSI_WRITE, g_param, 2);
		if (ret != 2) {
			pr_err("DISP:ERR:%s:failed to write global command\n", __func__);
			goto err_write_gen_cmd;
		}

		cmd_buf[0] = 0x9E;

		tx_size = (remind_size > fifo_size - 1) ? fifo_size - 1 : remind_size;
		memcpy(cmd_buf + 1, cmd + t_tx_size, tx_size);

		ret = __mipi_write_data(dsi, MIPI_DSI_WRITE, cmd_buf, tx_size + 1);
		if (ret != tx_size + 1) {
			pr_err("DISP:ERR:%s:failed to write command\n", __func__);
			goto err_write_gen_cmd;
		}
		t_tx_size += tx_size;
		remind_size -= tx_size;
	}

	return size;

err_write_gen_cmd:
	return ret;
}

static int mipi_write_side_ram(struct mipi_dsi_device *dsi, const u8 *cmd, int size)
{
	int ret;
	u8 cmd_buf[DSIM_FIFO_SIZE];
	u32 t_tx_size, tx_size, remind_size;
	u32 fifo_size, tmp;

	pr_debug("%s : was called : %d\n", __func__, size);

	fifo_size = DSIM_FIFO_SIZE;

	/* for 12byte align */
	tmp = (fifo_size - 1) / SIDE_RAM_ALIGN_CNT;
	fifo_size = (tmp * SIDE_RAM_ALIGN_CNT) + 1;

	pr_debug("%s : fifo size : %d\n", __func__, fifo_size);

	t_tx_size = 0;
	remind_size = size;

	while (remind_size) {
		if (remind_size == size)
			cmd_buf[0] = MIPI_DSI_OEM1_WR_SIDE_RAM;
		else
			cmd_buf[0] = MIPI_DSI_OEM1_WR_SIDE_RAM2;

		tx_size = min(remind_size, fifo_size - 1);

		memcpy(cmd_buf + 1, cmd + t_tx_size, tx_size);

		ret = __mipi_write_data(dsi, MIPI_DSI_WRITE, cmd_buf, tx_size + 1);
		if (ret != tx_size + 1) {
			pr_err("DISP:ERR(2):%s:failed to write command : %d\n", __func__, ret);
			goto err_write_side_ram;
		}

		t_tx_size += tx_size;
		remind_size -= tx_size;
	}

	return size;

err_write_side_ram:
	return ret;
}

static int mipi_write_gram(struct mipi_dsi_device *dsi, const u8 *cmd, int size)
{
	int ret;
	u8 buf[DSIM_FIFO_SIZE];
	u32 t_tx_size, tx_size, remind_size;
	u32 fifo_size, tmp;

	pr_debug("%s : was called : %d\n", __func__, size);

	fifo_size = DSIM_FIFO_SIZE;

	/* for 12byte align */
	tmp = (fifo_size - 1) / SIDE_RAM_ALIGN_CNT;
	fifo_size = (tmp * SIDE_RAM_ALIGN_CNT) + 1;

	t_tx_size = 0;
	remind_size = size;

	while (remind_size > 0) {
		buf[0] = (remind_size == size) ?
			MIPI_DCS_WRITE_MEMORY_START : MIPI_DCS_WRITE_MEMORY_CONTINUE;

		tx_size = min(remind_size, fifo_size - 1);
		if (tx_size < 12) {
			pr_debug("%s tx_size %d align 12bytes\n", __func__, tx_size);
			tx_size = 12;
		}

		memcpy(buf + 1, cmd + t_tx_size, tx_size);

		ret = __mipi_write_data(dsi, MIPI_DSI_WRITE, buf, tx_size + 1);
		if (ret != tx_size + 1) {
			pr_err("DISP:ERR(2):%s:failed to write command : %d\n", __func__, ret);
			goto err_write_gram;
		}

		t_tx_size += tx_size;
		remind_size -= tx_size;
	}
	pr_debug("%s fifo %d write %d bytes\n", __func__, fifo_size, t_tx_size);

	return size;

err_write_gram:
	return ret;
}

static int mipi_write_cmd(u32 id, u8 cmd_id, const u8 *cmd, int size)
{
	struct panel_device *panel = panel_dsi_panel;
	struct mipi_dsi_device *dsi;
	int ret = 0;

	if (!panel || !panel->dsi) {
		pr_err("ERR:DSIM:%s:dsim is NULL\n", __func__);
		return -ENODEV;
	}

	/* as the vendor dsim_write_data(), leave hibernation for every command */
	ret = exynos8895_decon_hiber_block_exit();
	if (ret) {
		pr_err("ERR:DSIM:%s:hibernation exit failed: %d\n", __func__, ret);
		/* the block is taken even when the exit is refused */
		exynos8895_decon_hiber_unblock();
		return ret;
	}
	dsi = panel->dsi;

	if (!panel->link_on) {
		pr_warn("DSIM:WARN:%s:dsim is not on\n", __func__);
		ret = -EINVAL;
		goto err_write_cmd;
	}

	switch (cmd_id) {
	case MIPI_DSI_DSC_PPS:
		ret = mipi_write_pps_cmd(dsi, cmd, size);
		if (ret != size) {
			pr_err("DSIM:ERR:%s:failed to write pps cmd\n", __func__);
			goto err_write_cmd;
		}
		break;
	case MIPI_DSI_DSC_PRA:
	case MIPI_DSI_WRITE:
		ret = mipi_write_gen_cmd(dsi, cmd_id, cmd, size);
		if (ret != size) {
			pr_err("DSIM:ERR:%s:failed to write gen cmd\n", __func__);
			goto err_write_cmd;
		}
		break;
	case MIPI_DSI_WR_MEM:
		ret = mipi_write_gram(dsi, cmd, size);
		if (ret != size) {
			pr_err("DSIM:ERR:%s:failed to write gram\n", __func__);
			goto err_write_cmd;
		}
		break;
	case MIPI_DSI_OEM1_WR_SIDE_RAM:
		ret = mipi_write_side_ram(dsi, cmd, size);
		if (ret != size) {
			pr_err("DSIM:ERR:%s:failed to write side ram\n", __func__);
			goto err_write_cmd;
		}
		break;
	default:
		/* returns 0, as the vendor driver does */
		pr_err("DSIM:ERR:%s:invalid id : %d\n", __func__, cmd_id);
		goto err_write_cmd;
	}
	ret = size;

err_write_cmd:
	exynos8895_decon_hiber_unblock();
	return ret;
}

static int dsim_read_data(struct mipi_dsi_device *dsi, u8 addr, u8 *buf, u16 size)
{
	ssize_t ret;

	ret = mipi_dsi_set_maximum_return_packet_size(dsi, size);
	if (ret < 0) {
		pr_err("ERR:%s:failed to write maximum read size\n", __func__);
		return -EIO;
	}

	ret = mipi_dsi_dcs_read(dsi, addr, buf, size);
	if (ret < 0)
		pr_err("ERR:%s:failed to read addr 0x%02x: %zd\n", __func__, addr, ret);
	return ret;
}

static int mipi_read_cmd(u32 id, u8 addr, u8 *buf, int size)
{
	struct panel_device *panel = panel_dsi_panel;
	int ret = 0;
	int retry = 5;

	if (!panel || !panel->dsi) {
		pr_err("ERR:DSIM:%s:dsim is NULL\n", __func__);
		return -ENODEV;
	}

	ret = exynos8895_decon_hiber_block_exit();
	if (ret) {
		pr_err("ERR:%s:hibernation exit failed: %d\n", __func__, ret);
		exynos8895_decon_hiber_unblock();
		return ret;
	}

read_command:
	/* the vendor dsim_read_data() refuses a DSIM that is not on */
	if (!panel->link_on) {
		pr_err("DSIM is not ready\n");
		ret = -EINVAL;
	} else {
		ret = dsim_read_data(panel->dsi, addr, buf, size);
	}
	if (ret != size) {
		if (retry == 0) {
			pr_err("ERR:%s:failed to read addr : %x\n", __func__, addr);
		} else {
			retry--;
			goto read_command;
		}
	}
	exynos8895_decon_hiber_unblock();
	return ret;
}

static enum dsim_state get_dsim_state(u32 id)
{
	struct panel_device *panel = panel_dsi_panel;

	if (!panel || !panel->dsi || !panel->link_on)
		return DSIM_STATE_OFF;
	return DSIM_STATE_ON;
}

/* the ops the vendor DSIM passed with PANEL_IOC_DSIM_PUT_MIPI_OPS */
void panel_dsi_init(struct panel_device *panel, struct mipi_dsi_device *dsi)
{
	panel_dsi_panel = panel;
	panel->dsi = dsi;
	panel->dsi_id = 0;
	panel->mipi_drv.read = mipi_read_cmd;
	panel->mipi_drv.write = mipi_write_cmd;
	panel->mipi_drv.get_state = get_dsim_state;
}
