/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * What the vendor panel framework took from the vendor DPU driver and its MIPI
 * header, with the vendor values.
 */
#ifndef __PANEL_DPU_H__
#define __PANEL_DPU_H__

/* command ids of mipi_drv.write() */
#define MIPI_DSI_WRITE	0x00
#define MIPI_DSI_WR_MEM 0x99

/* vendor MIPI data types */
#define MIPI_DSI_DSC_PRA		0x07	/* = MIPI_DSI_COMPRESSION_MODE */
#define MIPI_DSI_DSC_PPS		0x0a	/* = MIPI_DSI_PICTURE_PARAMETER_SET */
#define MIPI_DSI_OEM1_WR_SIDE_RAM	0x4C
#define MIPI_DSI_OEM1_WR_SIDE_RAM2	0x5C

enum dsim_state {
	DSIM_STATE_INIT,
	DSIM_STATE_ON,		/* HS clock was enabled. */
	DSIM_STATE_ULPS,	/* DSIM was entered ULPS state */
	DSIM_STATE_OFF		/* DSIM is suspend state */
};

#include <drm/exynos8895_panel.h>

/* the vendor dsim.h included these for its users */
#include "decon_lcd.h"
#include "panel_drv.h"

#endif /* __PANEL_DPU_H__ */
