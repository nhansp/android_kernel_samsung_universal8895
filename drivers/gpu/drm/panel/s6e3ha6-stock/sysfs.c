/*
 * linux/drivers/video/fbdev/exynos/panel/sysfs.c
 *
 * Samsung MIPI-DSI Panel SYSFS driver.
 *
 * Copyright (c) 2016 Samsung Electronics
 * JiHoon Kim <jihoonn.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/ctype.h>
#include <linux/lcd.h>

#include "../dpu/dsim.h"
#include "panel.h"
#include "panel_drv.h"
#include "panel_bl.h"
#include "../dpu/decon.h"
#include "copr.h"
#include "mdnie.h"
#include "dimming.h"
#include "panel_poc.h"

extern struct kset *devices_kset;

int fingerprint_value = -1;
static ssize_t fingerprint_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%u\n", fingerprint_value);

	return strlen(buf);
}

static ssize_t fingerprint_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int rc;

	rc = kstrtouint(buf, (unsigned int)0, &fingerprint_value);
	if (rc < 0)
		return rc;

	dev_info(dev, "%s: %d\n", __func__, fingerprint_value);

	return size;
}

static ssize_t lcd_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "SDC_%02X%02X%02X\n",
			panel_data->id[0], panel_data->id[1], panel_data->id[2]);

	return strlen(buf);
}

static ssize_t window_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}

	if (panel->state.connect_panel == PANEL_DISCONNECT) {
		panel_warn("PANEL:WANR:%s:panel disconnected\n", __func__);
		return -ENODEV;
	}

	panel_data = &panel->panel_data;

	sprintf(buf, "%02x %02x %02x\n",
			panel_data->id[0], panel_data->id[1], panel_data->id[2]);
	pr_info("%s %02x %02x %02x\n",
			__func__, panel_data->id[0], panel_data->id[1], panel_data->id[2]);
	return strlen(buf);
}

static ssize_t manufacture_code_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 code[5] = { 0, };
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	resource_copy_by_name(panel_data, code, "code");

	sprintf(buf, "%02X%02X%02X%02X%02X\n",
		code[0], code[1], code[2], code[3], code[4]);

	return strlen(buf);
}

static ssize_t cell_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 date[7] = { 0, }, coordinate[4] = { 0, };
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	resource_copy_by_name(panel_data, date, "date");
	resource_copy_by_name(panel_data, coordinate, "coordinate");

	sprintf(buf, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
		date[0] , date[1], date[2], date[3], date[4], date[5], date[6],
		coordinate[0], coordinate[1], coordinate[2], coordinate[3]);

	return strlen(buf);
}

static ssize_t SVC_OCTA_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return cell_id_show(dev, attr, buf);
}

static ssize_t octa_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i, site, rework, poc;
	u8 cell_id[16], octa_id[PANEL_OCTA_ID_LEN] = { 0, };
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);
	char *p = buf;
	bool cell_id_exist = true;

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;
	resource_copy_by_name(panel_data, octa_id, "octa_id");

	site = (octa_id[0] >> 4) & 0x0F;
	rework = octa_id[0] & 0x0F;
	poc = octa_id[1] & 0x0F;

	panel_dbg("site (%d), rework (%d), poc (%d)\n",
			site, rework, poc);

	panel_dbg("<CELL ID>\n");
	for (i = 0; i < 16; i++) {
		cell_id[i] = isalnum(octa_id[i + 4]) ? octa_id[i + 4] : '\0';
		panel_dbg("%x -> %c\n", octa_id[i + 4], cell_id[i]);
		if (cell_id[i] == '\0') {
			cell_id_exist = false;
			break;
		}
	}

	p += sprintf(p, "%d%d%d%02x%02x", site, rework, poc, octa_id[2], octa_id[3]);
	if (cell_id_exist) {
		for (i = 0; i < 16; i++)
			p += sprintf(p, "%c", cell_id[i]);
	}
	p += sprintf(p, "\n");

	return strlen(buf);
}

static ssize_t SVC_OCTA_CHIPID_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return octa_id_show(dev, attr, buf);
}

static ssize_t color_coordinate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 coordinate[4] = { 0, };
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	resource_copy_by_name(panel_data, coordinate, "coordinate");

	sprintf(buf, "%u, %u\n", /* X, Y */
			coordinate[0] << 8 | coordinate[1],
			coordinate[2] << 8 | coordinate[3]);
	return strlen(buf);
}

static ssize_t manufacture_date_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u16 year;
	u8 month, day, hour, min, date[4] = { 0, };
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	resource_copy_by_name(panel_data, date, "date");

	year = ((date[0] & 0xF0) >> 4) + 2011;
	month = date[0] & 0xF;
	day = date[1] & 0x1F;
	hour = date[2] & 0x1F;
	min = date[3] & 0x3F;

	sprintf(buf, "%d, %d, %d, %d:%d\n", year, month, day, hour, min);
	return strlen(buf);
}

static ssize_t brightness_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_device *panel = dev_get_drvdata(dev);
	struct panel_bl_device *panel_bl;
	char *pos = buf;
	int br;

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_bl = &panel->panel_bl;

	for (br = 0; br <= EXTEND_BRIGHTNESS; br += BRT_SCALE)
		pos += sprintf(pos, "%5d %3d\n",
				br, get_actual_brightness(panel_bl, br));

	return pos - buf;
}

static ssize_t adaptive_control_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "%d\n", panel_data->props.adaptive_control);

	return strlen(buf);
}

static ssize_t adaptive_control_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);
	struct panel_bl_device *panel_bl;
	struct backlight_device *bd;
	int value, rc;

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}

	panel_data = &panel->panel_data;
	panel_bl = &panel->panel_bl;
	bd = panel_bl->bd;

	if (bd == NULL) {
		panel_err("PANEL:ERR:%s:backlight device is null\n", __func__);
		return -EINVAL;
	}

	rc = kstrtouint(buf, (unsigned int)0, &value);
	if (rc < 0)
		return rc;

	if (panel_data->props.adaptive_control == value)
		return size;

	mutex_lock(&panel_bl->lock);
	panel_data->props.adaptive_control = value;
	mutex_unlock(&panel_bl->lock);
	backlight_update_status(bd);

	dev_info(dev, "%s, adaptive_control %d\n",
			__func__, panel_data->props.adaptive_control);

	return size;
}

static ssize_t siop_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "%u\n", panel_data->props.siop_enable);

	return strlen(buf);
}

static ssize_t siop_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);
	struct backlight_device *bd;
	int value, rc;

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	bd = panel->panel_bl.bd;
	if (bd == NULL) {
		panel_err("PANEL:ERR:%s:backlight device is null\n", __func__);
		return -EINVAL;
	}

	rc = kstrtouint(buf, (unsigned int)0, &value);
	if (rc < 0)
		return rc;

	if (panel_data->props.siop_enable == value)
		return size;

	mutex_lock(&panel->op_lock);
	panel_data->props.siop_enable = value;
	mutex_unlock(&panel->op_lock);
	backlight_update_status(bd);

	dev_info(dev, "%s, siop_enable %d\n",
			__func__, panel_data->props.siop_enable);
	return size;
}

static ssize_t temperature_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char temp[] = "-15, -14, 0, 1\n";

	strcat(buf, temp);
	return strlen(buf);
}

static ssize_t temperature_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);
	struct backlight_device *bd;
	int value, rc;

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	bd = panel->panel_bl.bd;
	if (bd == NULL) {
		panel_err("PANEL:ERR:%s:backlight device is null\n", __func__);
		return -EINVAL;
	}

	rc = kstrtoint(buf, 10, &value);
	if (rc < 0)
		return rc;

	mutex_lock(&panel->op_lock);
	panel_data->props.temperature = value;
	mutex_unlock(&panel->op_lock);
	backlight_update_status(bd);

	dev_info(dev, "%s, temperature %d\n",
			__func__, panel_data->props.temperature);

	return size;
}

unsigned char readbuf[256] = { 0xff, };
unsigned int readreg, readpos, readlen;

static ssize_t read_mtp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int len = 0, i;

	if (!readreg || !readlen || readreg > 0xff || readlen > 255 || readpos > 255)
		return -EINVAL;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"addr:0x%02X pos:%d size:%d\n",
			readreg, readpos, readlen);
	for (i = 0; i < readlen; i++)
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02X%s", readbuf[i],
				(((i + 1) % 16) == 0) || (i == readlen - 1) ? "\n" : " ");
	return strlen(buf);
}

static ssize_t read_mtp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct panel_device *panel = dev_get_drvdata(dev);
	int ret, i;

	sscanf(buf, "%x %d %d", &readreg, &readpos, &readlen);

	if (!readreg || !readlen || readreg > 0xff || readlen > 255 || readpos > 255)
		return -EINVAL;

	if (!IS_PANEL_ACTIVE(panel))
		return -EINVAL;

	mutex_lock(&panel->op_lock);
	panel_set_key(panel, 3, true);
	ret = panel_rx_nbytes(panel, DSI_PKT_TYPE_RD, readbuf, readreg, readpos, readlen);
	panel_set_key(panel, 3, false);

	if (unlikely(ret != readlen)) {
		pr_err("%s, failed to read reg %02Xh pos %d len %d\n",
				__func__, readreg, readpos, readlen);
		ret = -EIO;
		goto store_err;
	}

	pr_info("READ_Reg addr: %02x, pos : %d len : %d\n",
			readreg, readpos, readlen);
	for (i = 0; i < readlen; i++)
		pr_info("READ_Reg %dth : %02x \n", i + 1, readbuf[i]);

store_err:
	mutex_unlock(&panel->op_lock);
	return size;
}

static ssize_t mcd_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "%u\n", panel_data->props.mcd_on);

	return strlen(buf);
}

static ssize_t mcd_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int value, rc, ret;
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	rc = kstrtouint(buf, (unsigned int)0, &value);
	if (rc < 0)
		return rc;

	if (panel_data->props.mcd_on == value)
		return size;

	mutex_lock(&panel->op_lock);
	panel_data->props.mcd_on = value;
	mutex_unlock(&panel->op_lock);

	ret = panel_do_seqtbl_by_index(panel,
			value ? PANEL_MCD_ON_SEQ : PANEL_MCD_OFF_SEQ);
	if (unlikely(ret < 0)) {
		pr_err("%s, failed to write mcd seq\n", __func__);
		return ret;
	}
	dev_info(dev, "%s, mcd %s\n",
			__func__, panel_data->props.mcd_on ? "on" : "off");

	return size;
}

static ssize_t mst_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "%u\n", panel_data->props.mst_on);

	return strlen(buf);
}

static ssize_t mst_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int value, rc, ret;
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	rc = kstrtouint(buf, (unsigned int)0, &value);
	if (rc < 0)
		return rc;

	if (panel_data->props.mst_on == value)
		return size;

	mutex_lock(&panel->op_lock);
	panel_data->props.mst_on = value;
	mutex_unlock(&panel->op_lock);

	ret = panel_do_seqtbl_by_index(panel,
			value ? PANEL_MST_ON_SEQ : PANEL_MST_OFF_SEQ);
	if (unlikely(ret < 0)) {
		pr_err("%s, failed to write mcd seq\n", __func__);
		return ret;
	}
	dev_info(dev, "%s, mst %s\n",
			__func__, panel_data->props.mst_on ? "on" : "off");

	return size;
}






static ssize_t poc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_device *panel = dev_get_drvdata(dev);
	struct panel_poc_device *poc_dev;
	struct panel_poc_info *poc_info;
	int ret;

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}

	if (!IS_PANEL_ACTIVE(panel)) {
		panel_err("%s:panel is not active\n", __func__);
		return -EAGAIN;
	}

	poc_dev = &panel->poc_dev;
	poc_info = &poc_dev->poc_info;

	ret = set_panel_poc(poc_dev, POC_OP_CHECKPOC);
	if (unlikely(ret < 0)) {
		pr_err("%s, failed to chkpoc (ret %d)\n", __func__, ret);
		return ret;
	}

	ret = set_panel_poc(poc_dev, POC_OP_CHECKSUM);
	if (unlikely(ret < 0)) {
		pr_err("%s, failed to chksum (ret %d)\n", __func__, ret);
		return ret;
	}

	snprintf(buf, PAGE_SIZE, "%d %d %02x\n", poc_info->poc,
			poc_info->poc_chksum[4], poc_info->poc_ctrl[3]);

	dev_info(dev, "%s poc:%d chk:%d gray:%02x\n", __func__, poc_info->poc,
			poc_info->poc_chksum[4], poc_info->poc_ctrl[3]);

	return strlen(buf);
}

static ssize_t poc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct panel_device *panel = dev_get_drvdata(dev);
	struct panel_info *panel_data;
	struct panel_poc_device *poc_dev;
	struct panel_poc_info *poc_info;
	int rc, ret;
	unsigned int value;

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;
	poc_dev = &panel->poc_dev;
	poc_info = &poc_dev->poc_info;

	rc = kstrtouint(buf, (unsigned int)0, &value);
	if (rc < 0)
		return rc;

	if (!IS_VALID_POC_OP(value)) {
		pr_warn("%s invalid poc_op %d\n", __func__, value);
		return -EINVAL;
	}

	if (value == POC_OP_WRITE || value == POC_OP_READ) {
		pr_warn("%s unsupported poc_op %d\n", __func__, value);
		return size;
	}

	if (value == POC_OP_CANCEL) {
		atomic_set(&poc_dev->cancel, 1);
	} else {
		mutex_lock(&panel->io_lock);
		ret = set_panel_poc(poc_dev, value);
		if (unlikely(ret < 0)) {
			pr_err("%s, failed to poc_op %d(ret %d)\n", __func__, value, ret);
			mutex_unlock(&panel->io_lock);
			return -EINVAL;
		}
		mutex_unlock(&panel->io_lock);
	}

	mutex_lock(&panel->op_lock);
	panel_data->props.poc_op = value;
	mutex_unlock(&panel->op_lock);

	dev_info(dev, "%s poc_op %d\n",
			__func__, panel_data->props.poc_op);

	return size;
}

static ssize_t grayspot_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "%u\n", panel_data->props.grayspot);

	return strlen(buf);
}

static ssize_t grayspot_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int value, rc, ret;
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	rc = kstrtouint(buf, (unsigned int)0, &value);
	if (rc < 0)
		return rc;

	if (panel_data->props.grayspot == value)
		return size;

	mutex_lock(&panel->op_lock);
	panel_data->props.grayspot = value;
	mutex_unlock(&panel->op_lock);

	ret = panel_do_seqtbl_by_index(panel,
			value ? PANEL_GRAYSPOT_ON_SEQ : PANEL_GRAYSPOT_OFF_SEQ);
	if (unlikely(ret < 0)) {
		pr_err("%s, failed to write grayspot on/off seq\n", __func__);
		return ret;
	}
	dev_info(dev, "%s, grayspot %s\n",
			__func__, panel_data->props.grayspot ? "on" : "off");

	return size;
}

static ssize_t hmt_bright_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_bl_device *panel_bl;
	struct panel_device *panel = dev_get_drvdata(dev);
	int size;

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}

	panel_bl = &panel->panel_bl;
	mutex_lock(&panel_bl->lock);
	if (panel_bl->props.id == PANEL_BL_SUBDEV_TYPE_DISP) {
		size = snprintf(buf, 30, "HMD off state\n");
	} else {
		size = sprintf(buf, "index : %d, brightenss : %d\n",
				panel_bl->props.actual_brightness_index,
				BRT_USER(panel_bl->props.brightness));
	}
	mutex_unlock(&panel_bl->lock);
	return size;
}

static ssize_t hmt_bright_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int value, rc, ret;
	struct panel_bl_device *panel_bl;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_bl = &panel->panel_bl;

	rc = kstrtouint(buf, (unsigned int)0, &value);
	if (rc < 0)
		return rc;

	panel_info("PANEL:INFO:%s: brightness : %d\n",
			__func__, value);

	mutex_lock(&panel_bl->lock);
	if (panel_bl->subdev[PANEL_BL_SUBDEV_TYPE_HMD].brightness != BRT(value)) {
		panel_bl->subdev[PANEL_BL_SUBDEV_TYPE_HMD].brightness = BRT(value);
	}

	if (panel->state.hmd_on != PANEL_HMD_ON) {
		panel_info("PANEL:WARN:%s: hmd off\n", __func__);
		goto exit_store;
	}

	ret = panel_bl_set_brightness(panel_bl, PANEL_BL_SUBDEV_TYPE_HMD, 1);
	if (ret) {
		pr_err("%s : fail to set brightness\n", __func__);
		goto exit_store;
	}

exit_store:
	mutex_unlock(&panel_bl->lock);
	return size;
}

static ssize_t hmt_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_state *state;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	state = &panel->state;

	sprintf(buf, "%u\n", state->hmd_on);

	return strlen(buf);
}

static ssize_t hmt_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	int value, rc;
	struct backlight_device *bd;
	struct panel_state *state;
	struct panel_device *panel = dev_get_drvdata(dev);
	struct panel_bl_device *panel_bl;

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}

	panel_bl = &panel->panel_bl;
	bd = panel_bl->bd;
	state = &panel->state;

	rc = kstrtoint(buf, 0, &value);
	if (rc < 0)
		return rc;

	panel_info("PANEL:INFO:%s: hmt %s\n",
			__func__, value ? "on" : "off");

	if (value == state->hmd_on) {
		panel_warn("PANEL:WARN:%s:already set : %d\n",
			__func__, value);
		return size;
	}

	mutex_lock(&panel_bl->lock);
	if (value == PANEL_HMD_ON) {
		ret = panel_do_seqtbl_by_index(panel, PANEL_HMD_ON_SEQ);
		if (ret) {
			panel_err("PANEL:ERR:%s:failed to set hmd on seq\n", __func__);
			goto exit_store;
		}
		ret = panel_bl_set_brightness(panel_bl, PANEL_BL_SUBDEV_TYPE_HMD, 1);
		if (ret) {
			pr_err("%s : fail to set brightness\n", __func__);
			goto exit_store;
		}
	} else {
		ret = panel_do_seqtbl_by_index(panel, PANEL_HMD_OFF_SEQ);
		if (ret) {
			panel_err("PANEL:ERR:%s:failed to set hmd off seq\n",
				__func__);
			goto exit_store;
		}
		ret = panel_bl_set_brightness(panel_bl, PANEL_BL_SUBDEV_TYPE_DISP, 1);
		if (ret) {
			pr_err("%s : fail to set brightness\n", __func__);
			goto exit_store;
		}
	}
	state->hmd_on = value;

exit_store:
	mutex_unlock(&panel_bl->lock);
	return size;
}



static int set_alpm_mode(struct panel_device *panel, int mode)
{
	int ret = 0;
	struct panel_info *panel_data = &panel->panel_data;

	switch (mode) {
		case ALPM_LOW_BR:
		case HLPM_LOW_BR:
		case ALPM_HIGH_BR:
		case HLPM_HIGH_BR:
			if (panel->state.cur_state != PANEL_STATE_ALPM) {
				panel_info("PANEL:INFO:%s:panel state : %d\n", __func__,
					panel->state.cur_state);
				return 0;
			}
			if (mode != panel_data->props.alpm_mode) {
				panel_data->props.alpm_mode = mode;
				ret = panel_do_seqtbl_by_index(panel, PANEL_ALPM_ENTER_SEQ);
				if (ret) {
					panel_err("PANEL:ERR:%s, failed to write init seqtbl\n",
						__func__);
				}
			}
			break;
		default:
			panel_err("PANEL:ERR:%s:Invalid ALPM Mode : %d\n", __func__, mode);
			break;
	}
	return ret;
}

static ssize_t alpm_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	struct panel_device *panel = dev_get_drvdata(dev);
	struct panel_info *panel_data = &panel->panel_data;

	mutex_lock(&panel->io_lock);

	sscanf(buf, "%9d", &value);

	panel_info("PANEL:INFO:%s:mode : %d\n", __func__, value);

	if (set_alpm_mode(panel, value)) {
		panel_err("PANEL:ERR:%s:failed to set alpm\n", __func__);
		goto exit_store;
	}
	panel_data->props.alpm_mode = value;

exit_store:
	mutex_unlock(&panel->io_lock);
	return size;
}

static ssize_t alpm_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "%d\n", panel_data->props.alpm_mode);
	panel_dbg("%s: %d\n", __func__, panel_data->props.alpm_mode);

	return strlen(buf);
}

static void show_aid_log(struct panel_info *panel_data, int id)
{
	struct dimming_info *dim_info;
	struct maptbl *tbl = NULL, *aor_tbl = NULL, *irc_tbl = NULL;
	int layer, row, col, i;
	char buf[1024], *p;
	struct panel_device *panel = to_panel_device(panel_data);
	struct panel_bl_device *panel_bl = &panel->panel_bl;
	struct panel_bl_sub_dev *subdev;

	if (unlikely(!panel_data)) {
		panel_err("%s:panel is NULL\n", __func__);
		return;
	}

	if (id >= MAX_PANEL_BL_SUBDEV) {
		panel_err("%s invalid bl-%d\n", __func__, id);
		return;
	}

	subdev = &panel_bl->subdev[id];

	pr_info("[====================== [%s] ======================]\n",
			(id == PANEL_BL_SUBDEV_TYPE_HMD ? "HMD" : "DISP"));
	dim_info = panel_data->dim_info[id];
	if (!dim_info) {
		panel_warn("%s bl-%d dim_info is null\n", __func__, id);
		return;
	}

	print_dimming_info(dim_info, TAG_MTP_OFFSET_START);
	print_dimming_info(dim_info, TAG_TP_VOLT_START);
	print_dimming_info(dim_info, TAG_GRAY_SCALE_VOLT_START);
	print_dimming_info(dim_info, TAG_GAMMA_CENTER_START);

	/* TODO : 0 means GAMMA_MAPTBL.
	          To use commonly in panel driver some maptbl index should be same
	 */
	if (id == PANEL_BL_SUBDEV_TYPE_HMD)
		tbl = find_panel_maptbl_by_name(panel_data, "hmd_gamma_table");
	else
		tbl = find_panel_maptbl_by_name(panel_data, "gamma_table");

	if (tbl) {
		pr_info("[GAMMA MAPTBL] HEXA-DECIMAL\n");
		for_each_layer(tbl, layer) {
			for_each_row(tbl, row) {
				p = buf;
				p += sprintf(p, "gamma[%3d] : ",
						subdev->brt_tbl.lum[row]);
				for_each_col(tbl, i)
					p += sprintf(p, "%02X ",
							tbl->arr[row * sizeof_row(tbl) + i]);
				pr_info("%s\n", buf);
			}
		}
	}

	if (id == PANEL_BL_SUBDEV_TYPE_HMD)
		aor_tbl = find_panel_maptbl_by_name(panel_data, "hmd_aor_table");
	else
		aor_tbl = find_panel_maptbl_by_name(panel_data, "aor_table");

	if (id != PANEL_BL_SUBDEV_TYPE_HMD)
		irc_tbl = find_panel_maptbl_by_name(panel_data, "irc_table");

	panel_info("\n[BRIGHTNESS, %s %s table]\n",
			aor_tbl ? "AOR" : "", irc_tbl ? "IRC" : "");
	p = buf;
	p += sprintf(p, "[idx] platform   luminance  ");
	if (aor_tbl)
		p += sprintf(p, "|  aor  ");
	if (irc_tbl)
		p += sprintf(p, "| =========================== irc ===========================");
	pr_info("%s\n", buf);

	for (i = 0; i < subdev->brt_tbl.sz_brt_to_step; i++) {
		p = buf;
		p += sprintf(p, "[%3d]   %-5d   %3d(%3d.%02d) ", i,
				subdev->brt_tbl.brt_to_step[i],
				get_subdev_actual_brightness(panel_bl, id,
					subdev->brt_tbl.brt_to_step[i]),
				get_subdev_actual_brightness_interpolation(panel_bl, id,
					subdev->brt_tbl.brt_to_step[i]) / 100,
				get_subdev_actual_brightness_interpolation(panel_bl, id,
					subdev->brt_tbl.brt_to_step[i]) % 100);
		if (aor_tbl) {
			p += sprintf(p, "| ");
			for_each_col(aor_tbl, col)
				p += sprintf(p, "%02X ", aor_tbl->arr[i * sizeof_row(aor_tbl) + col]);
		}

		if (irc_tbl) {
			p += sprintf(p, "| ");
			for_each_col(irc_tbl, col)
				p += sprintf(p, "%02X ", irc_tbl->arr[i * sizeof_row(irc_tbl) + col]);
		}
		pr_info("%s\n", buf);
	}
}

static ssize_t aid_log_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	print_panel_resource(panel);

	show_aid_log(panel_data, PANEL_BL_SUBDEV_TYPE_DISP);
	show_aid_log(panel_data, PANEL_BL_SUBDEV_TYPE_HMD);

	return strlen(buf);
}


static ssize_t lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "%d\n", panel_data->props.lux);

	return strlen(buf);
}

static ssize_t lux_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int rc;
	int value;
	struct mdnie_info *mdnie;
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	mdnie = &panel->mdnie;
	panel_data = &panel->panel_data;

	rc = kstrtoint(buf, 0, &value);

	if (rc < 0)
		return rc;

	if (panel_data->props.lux != value) {
		mutex_lock(&panel->op_lock);
		panel_data->props.lux = value;
		mutex_unlock(&panel->op_lock);
		attr_store_for_each(mdnie->class, attr->attr.name, buf, size);
	}

	return size;
}

static ssize_t copr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_device *panel = dev_get_drvdata(dev);
	struct copr_info *copr = &panel->copr;
	int copr_avg, brt_avg, ret;

	if (!copr_is_enabled(copr)) {
		panel_err("%s copr is off state\n", __func__);
		return -EIO;
	}

	ret = copr_get_average(copr, &copr_avg, &brt_avg);
	if (ret < 0) {
		panel_err("%s failed to get average\n", __func__);
		return ret;
	}

	sprintf(buf, "%d %d\n", copr_avg, brt_avg);

	return strlen(buf);
}

static ssize_t copr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct panel_device *panel = dev_get_drvdata(dev);
	struct copr_info *copr = &panel->copr;
	int copr_en, copr_gamma, copr_er, copr_eg, copr_eb;
	int roi_on, roi_xs, roi_ys, roi_xe, roi_ye;
	int rc;

	rc = sscanf(buf, "%i %i %i %i %i %i %i %i %i %i",
			&copr_en, &copr_gamma, &copr_er, &copr_eg, &copr_eb,
			&roi_on, &roi_xs, &roi_ys, &roi_xe, &roi_ye);
	if (rc < 0)
		return rc;

	mutex_lock(&copr->lock);
	if (copr->props.reg.copr_en != !!copr_en) {
		copr->props.reg.copr_en = !!copr_en;
		copr->props.state = COPR_UNINITIALIZED;
		pr_info("%s copr %s\n", __func__, copr_en ? "enable" : "disable");
	}

	if ((copr_gamma == 0 || copr_gamma == 1) &&
			(copr_er > 0 && copr_er < 0xFF) &&
			(copr_eg > 0 && copr_eg < 0xFF) &&
			(copr_eb > 0 && copr_eb < 0xFF) &&
			((copr_er + copr_eg + copr_eb) == 384) &&
			(roi_on == 0 || roi_on == 1)) {
		copr->props.reg.copr_gamma = !!copr_gamma;
		copr->props.reg.copr_er = copr_er;
		copr->props.reg.copr_eg = copr_eg;
		copr->props.reg.copr_eb = copr_eb;
		copr->props.reg.roi_on = !!roi_on;

		if (roi_on && (roi_xs % 4 == 0) && (roi_xe % 4 == 3) && (roi_xe - roi_xs + 1 >= 12) &&
				(roi_ys % 4 == 0) && (roi_ye % 4 == 3) && (roi_ye - roi_ys + 1 >= 12)) {
			copr->props.reg.roi_xs = roi_xs;
			copr->props.reg.roi_ys = roi_ys;
			copr->props.reg.roi_xe = roi_xe;
			copr->props.reg.roi_ye = roi_ye;
		}
		pr_info("%s copr en %d gamma %d er 0x%02X eg 0x%02X eb 0x%02X\n",
				__func__, copr_en, copr_gamma, copr_er, copr_eg, copr_eb);
		copr->props.state = COPR_UNINITIALIZED;
	}
	mutex_unlock(&copr->lock);
	copr_update_start(&panel->copr, 1);

	return size;
}

static ssize_t read_copr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_device *panel = dev_get_drvdata(dev);
	struct copr_info *copr = &panel->copr;
	int cur_copr;

	if (!copr_is_enabled(copr)) {
		panel_err("%s copr is off state\n", __func__);
		return -EIO;
	}

	cur_copr = copr_get_value(copr);
	if (cur_copr < 0) {
		panel_err("%s failed to get copr\n", __func__);
		return -EIO;
	}

	sprintf(buf, "%d\n", cur_copr);

	return strlen(buf);
}

static ssize_t poc_onoff_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "%d\n", panel_data->props.poc_onoff);

	return strlen(buf);
}

static ssize_t poc_onoff_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int rc;
	int value;
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	rc = kstrtoint(buf, 0, &value);

	if (rc < 0)
		return rc;

	pr_info("%s: %d -> %d\n", __func__, panel_data->props.poc_onoff, value);

	if (panel_data->props.poc_onoff != value) {
		mutex_lock(&panel->panel_bl.lock);
		panel_data->props.poc_onoff = value;
		mutex_unlock(&panel->panel_bl.lock);
		backlight_update_status(panel->panel_bl.bd);
	}

	return size;
}

static ssize_t irc_onoff_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	sprintf(buf, "%d\n", panel_data->props.irc_onoff);

	return strlen(buf);
}

static ssize_t irc_onoff_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int rc;
	int value;
	struct panel_info *panel_data;
	struct panel_device *panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		panel_err("PANEL:ERR:%s:panel is null\n", __func__);
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	rc = kstrtoint(buf, 0, &value);

	if (rc < 0)
		return rc;

	pr_info("%s: %d -> %d\n", __func__, panel_data->props.irc_onoff, value);

	if (panel_data->props.irc_onoff != value) {
		mutex_lock(&panel->panel_bl.lock);
		panel_data->props.irc_onoff = value;
		mutex_unlock(&panel->panel_bl.lock);
		backlight_update_status(panel->panel_bl.bd);
	}

	return size;
}

struct device_attribute panel_attrs[] = {
	__PANEL_ATTR_RO(lcd_type, 0444),
	__PANEL_ATTR_RO(window_type, 0444),
	__PANEL_ATTR_RO(manufacture_code, 0444),
	__PANEL_ATTR_RO(cell_id, 0444),
	__PANEL_ATTR_RO(octa_id, 0444),
	__PANEL_ATTR_RO(SVC_OCTA, 0444),
	__PANEL_ATTR_RO(SVC_OCTA_CHIPID, 0444),
	__PANEL_ATTR_RW(mst, 0664),
	__PANEL_ATTR_RW(poc, 0660),
	__PANEL_ATTR_RW(grayspot, 0664),
	__PANEL_ATTR_RO(color_coordinate, 0444),
	__PANEL_ATTR_RO(manufacture_date, 0444),
	__PANEL_ATTR_RO(brightness_table, 0444),
	__PANEL_ATTR_RW(adaptive_control, 0664),
	__PANEL_ATTR_RW(siop_enable, 0664),
	__PANEL_ATTR_RW(temperature, 0664),
	__PANEL_ATTR_RW(read_mtp, 0644),
	__PANEL_ATTR_RW(mcd_mode, 0664),
	__PANEL_ATTR_RO(aid_log, 0440),
	__PANEL_ATTR_RW(lux, 0644),
	__PANEL_ATTR_RW(copr, 0600),
	__PANEL_ATTR_RO(read_copr, 0440),
	__PANEL_ATTR_RW(alpm, 0664),
	__PANEL_ATTR_RW(fingerprint, 0644),
	__PANEL_ATTR_RW(hmt_bright, 0664),
	__PANEL_ATTR_RW(hmt_on, 0664),
	__PANEL_ATTR_RW(poc_onoff, 0664),
	__PANEL_ATTR_RW(irc_onoff, 0664),
};

int panel_sysfs_probe(struct panel_device *panel)
{
	struct lcd_device *lcd;
	int i, ret;
	struct kernfs_node *svc_sd;
	struct kobject *svc;

	lcd = panel->lcd;
	if (unlikely(!lcd)) {
		pr_err("%s, lcd device not exist\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(panel_attrs); i++) {
		ret = device_create_file(&lcd->dev, &panel_attrs[i]);
		if (ret < 0) {
			dev_err(&lcd->dev, "%s, failed to add %s sysfs entries, %d\n",
					__func__, panel_attrs[i].attr.name, ret);
			return -ENODEV;
		}
	}

	/* to /sys/devices/svc/ */
	svc_sd = sysfs_get_dirent(devices_kset->kobj.sd, "svc");
	if (IS_ERR_OR_NULL(svc_sd)) {
		svc = kobject_create_and_add("svc", &devices_kset->kobj);
		if (IS_ERR_OR_NULL(svc))
			pr_err("failed to create /sys/devices/svc already exist svc : 0x%p\n", svc);
		else
			pr_err("success to create /sys/devices/svc svc : 0x%p\n", svc);
	} else {
		svc = (struct kobject*)svc_sd->priv;
		pr_info("success to find svc_sd : 0x%p  svc : 0x%p\n", svc_sd, svc);
	}

	if (!IS_ERR_OR_NULL(svc)) {
		ret = sysfs_create_link(svc, &lcd->dev.kobj, "OCTA");
		if (ret)
			pr_err("failed to create svc/OCTA/ \n");
		else
			pr_info("success to create svc/OCTA/ \n");
	} else {
		pr_err("failed to find svc kobject\n");
	}

	return 0;
}
