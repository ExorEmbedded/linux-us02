/*
 * Data structures, function prototypes and registers layout
 *
 * Copyright (C) 2016 Exor International S.p.a.
 * Author: Giovanni Pavoni (Exor Int. S.p.a.)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#ifndef ALTEVCH_H
#define ALTEVCH_H

#include <video/displayconfig.h>
#define PALETTE_SIZE	256
#define ALTEVC_INTERNAL_BUS_WIDTH 64
#define ALTEVC_BRIGHTNESS_LEVELS 26

/* -----------------------------------------------------------
   Function prototypes
   -----------------------------------------------------------*/
int altevc_setvideomode(void); /* Configures the core according with the selected display and activates it */
int altevc_bl_update_status(struct backlight_device *bd);
void altevc_bl_init(struct fb_info* sinfo, int initial_brightness);
void altevc_bl_deinit(void);
/* -----------------------------------------------------------
   Altevc regster mapping
   -----------------------------------------------------------*/

/* Frame Reader */
#define ALTVIPFB_CONTROL		0
#define ALTVIPFB_FRAME_SELECT		12
#define ALTVIPFB_FRAME0_BASE_ADDRESS	16
#define ALTVIPFB_FRAME0_NUM_WORDS	20
#define ALTVIPFB_FRAME0_SAMPLES		24
#define ALTVIPFB_FRAME0_WIDTH		32
#define ALTVIPFB_FRAME0_HEIGHT		36
#define ALTVIPFB_FRAME0_INTERLACED	40

/* Control and power registers */
#define DATACTRL_REG 			0x80
#define PWRCTRL_REG			0x90

/* Backlight dimming registers*/
#define BL_FREQ_REG			0xa0
#define BL_PWM_REG			0xb0

/* PLL Reconfig for pixclk */
#define PLL_RECONFIG_MODE		0x100 + (0x00 << 2)
#define PLL_RECONFIG_STATUS		0x100 + (0x01 << 2)
#define PLL_RECONFIG_START		0x100 + (0x02 << 2)
#define PLL_RECONFIG_N_COUNT		0x100 + (0x03 << 2)
#define PLL_RECONFIG_M_COUNT		0x100 + (0x04 << 2)
#define PLL_RECONFIG_C_COUNT		0x100 + (0x05 << 2)
#define PLL_RECONFIG_PHASE		0x100 + (0x06 << 2)
#define PLL_RECONFIG_K_COUNT		0x100 + (0x07 << 2)
#define PLL_RECONFIG_BW			0x100 + (0x08 << 2)
#define PLL_RECONFIG_CHARGEPUMP		0x100 + (0x09 << 2)
#define PLL_RECONFIG_VCODIV		0x100 + (0x1C << 2)

/* Clocked Video Output */
#define CVO_CONTROL         		0x400+(0x00 << 2)
#define CVO_STATUS          		0x400+(0x01 << 2)
#define CVO_MODEX_CTRL      		0x400+(0x05 << 2)
#define CVO_MODEX_XRES      		0x400+(0x06 << 2)
#define CVO_MODEX_VRES      		0x400+(0x07 << 2)
#define CVO_MODEX_HSFP      		0x400+(0x09 << 2)
#define CVO_MODEX_HSW       		0x400+(0x0a << 2)
#define CVO_MODEX_HSSUM     		0x400+(0x0b << 2)
#define CVO_MODEX_VSFP      		0x400+(0x0c << 2)
#define CVO_MODEX_VSW       		0x400+(0x0d << 2)
#define CVO_MODEX_VSSUM     		0x400+(0x0e << 2)
#define CVO_MODEX_VALID     		0x400+(0x1c << 2)

/* Gamma Correction */
#define ALTGAMMA_LUT2			0x808
#define ALTGAMMA_LUT1			0x1008
#define ALTGAMMA_LUT0			0x1808
#define ALTGAMMA_CONTROL		0x1800

/* Register fields */
#define PWR_VIDEO_ENA   (0x1 << 0)
#define PWR_VDD_ENA     (0x1 << 1)
#define PWR_BL_ENA      (0x1 << 2)

#define DCTRL_VSYNC_EN  (0x1 << 0)
#define DCTRL_VSYNC_INV (0x1 << 1)
#define DCTRL_HSYNC_EN  (0x1 << 2)
#define DCTRL_HSYNC_INV (0x1 << 3)
#define DCTRL_BLANK_EN  (0x1 << 4)
#define DCTRL_BLANK_INV (0x1 << 5)
#define DCTRL_VCLK_INV  (0x1 << 6)

#endif
