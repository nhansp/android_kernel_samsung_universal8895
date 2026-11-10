/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The definitions the vendor panel framework took from the vendor DPU and its
 * MIPI header (S:drivers/video/fbdev/exynos/dpu/decon.h, dsim.h,
 * S:include/video/mipi_display.h), values unchanged.
 */
#ifndef __PANEL_DPU_H__
#define __PANEL_DPU_H__

/* command ids of the panel's mipi_drv.write() (S:.../dpu/dsim.h:54-55) */
#define MIPI_DSI_WRITE	0x00
#define MIPI_DSI_WR_MEM 0x99

/* vendor data types (S:include/video/mipi_display.h:63-66) */
#define MIPI_DSI_DSC_PRA		0x07	/* = MIPI_DSI_COMPRESSION_MODE */
#define MIPI_DSI_DSC_PPS		0x0a	/* = MIPI_DSI_PICTURE_PARAMETER_SET */
#define MIPI_DSI_OEM1_WR_SIDE_RAM	0x4C
#define MIPI_DSI_OEM1_WR_SIDE_RAM2	0x5C

/* S:.../dpu/dsim.h:145-150 */
enum dsim_state {
	DSIM_STATE_INIT,
	DSIM_STATE_ON,		/* HS clock was enabled. */
	DSIM_STATE_ULPS,	/* DSIM was entered ULPS state */
	DSIM_STATE_OFF		/* DSIM is suspend state */
};

#include <drm/exynos8895_panel.h>

/* what the vendor dsim.h pulled in for its includers (S:.../dpu/dsim.h:28-29) */
#include "decon_lcd.h"
#include "panel_drv.h"

#endif /* __PANEL_DPU_H__ */
