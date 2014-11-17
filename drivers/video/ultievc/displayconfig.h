/*
 * Defines array containing list of all available/known displays and related physical parameters.
 *
 * Copyright (C) 2013 Exor International
 * Author: Giovanni Pavoni (Exor)
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
#ifndef DISPLAYCONFIG_H
#define DISPLAYCONFIG_H

#define NOTUSED 0xffff
/* 
 * Add to this list any further display device description
 * NOTE: Please update the MAXNRDISPLAYS accordingly
 */
#define MAXNRDISPLAYS 50
static struct t_DisplayParams displayconfig[MAXNRDISPLAYS+1] = {
    /* 0: Dummy, not used*/
    {
        .rezx      = NOTUSED, 
        .rezy      = NOTUSED, 
        .bpp       = NOTUSED,
        
        .pclk_freq = NOTUSED, 
        .pclk_inv  = NOTUSED,
        
        .hs_fp     = NOTUSED, 
        .hs_bp     = NOTUSED, 
        .hs_w      = NOTUSED, 
        .hs_inv    = NOTUSED,
        
        .vs_fp     = NOTUSED, 
        .vs_bp     = NOTUSED, 
        .vs_w      = NOTUSED, 
        .vs_inv    = NOTUSED,
        
        .blank_inv      = NOTUSED,
        
        .pwmfreq        = NOTUSED,
        .brightness_min = NOTUSED,
        .brightness_max = NOTUSED,
    },
    /* 1: TX18D16VM1CBA 800x480 */
    {
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 32000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 2: Dummy, not used*/
    {
        .rezx      = NOTUSED, 
        .rezy      = NOTUSED, 
        .bpp       = NOTUSED,
        
        .pclk_freq = NOTUSED, 
        .pclk_inv  = NOTUSED,
        
        .hs_fp     = NOTUSED, 
        .hs_bp     = NOTUSED, 
        .hs_w      = NOTUSED, 
        .hs_inv    = NOTUSED,
        
        .vs_fp     = NOTUSED, 
        .vs_bp     = NOTUSED, 
        .vs_w      = NOTUSED, 
        .vs_inv    = NOTUSED,
        
        .blank_inv      = NOTUSED,
        
        .pwmfreq        = NOTUSED,
        .brightness_min = NOTUSED,
        .brightness_max = NOTUSED,
    },
    /* 3: Dummy, not used*/
    {
        .rezx      = NOTUSED, 
        .rezy      = NOTUSED, 
        .bpp       = NOTUSED,
        
        .pclk_freq = NOTUSED, 
        .pclk_inv  = NOTUSED,
        
        .hs_fp     = NOTUSED, 
        .hs_bp     = NOTUSED, 
        .hs_w      = NOTUSED, 
        .hs_inv    = NOTUSED,
        
        .vs_fp     = NOTUSED, 
        .vs_bp     = NOTUSED, 
        .vs_w      = NOTUSED, 
        .vs_inv    = NOTUSED,
        
        .blank_inv      = NOTUSED,
        
        .pwmfreq        = NOTUSED,
        .brightness_min = NOTUSED,
        .brightness_max = NOTUSED,
    },
    /* 4: TX14D14VM1BAA 640x480 */
    {
        .rezx      = 640, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 24000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 144, 
        .hs_w      = 96, 
        .hs_inv    = 1,
        
        .vs_fp     = 32, 
        .vs_bp     = 12, 
        .vs_w      = 2, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 5: Dummy, not used*/
    {
        .rezx      = NOTUSED, 
        .rezy      = NOTUSED, 
        .bpp       = NOTUSED,
        
        .pclk_freq = NOTUSED, 
        .pclk_inv  = NOTUSED,
        
        .hs_fp     = NOTUSED, 
        .hs_bp     = NOTUSED, 
        .hs_w      = NOTUSED, 
        .hs_inv    = NOTUSED,
        
        .vs_fp     = NOTUSED, 
        .vs_bp     = NOTUSED, 
        .vs_w      = NOTUSED, 
        .vs_inv    = NOTUSED,
        
        .blank_inv      = NOTUSED,
        
        .pwmfreq        = NOTUSED,
        .brightness_min = NOTUSED,
        .brightness_max = NOTUSED,
    },
    /* 6: Dummy, not used*/
    {
        .rezx      = NOTUSED, 
        .rezy      = NOTUSED, 
        .bpp       = NOTUSED,
        
        .pclk_freq = NOTUSED, 
        .pclk_inv  = NOTUSED,
        
        .hs_fp     = NOTUSED, 
        .hs_bp     = NOTUSED, 
        .hs_w      = NOTUSED, 
        .hs_inv    = NOTUSED,
        
        .vs_fp     = NOTUSED, 
        .vs_bp     = NOTUSED, 
        .vs_w      = NOTUSED, 
        .vs_inv    = NOTUSED,
        
        .blank_inv      = NOTUSED,
        
        .pwmfreq        = NOTUSED,
        .brightness_min = NOTUSED,
        .brightness_max = NOTUSED,
    },
    /* 7: Dummy, not used*/
    {
        .rezx      = NOTUSED, 
        .rezy      = NOTUSED, 
        .bpp       = NOTUSED,
        
        .pclk_freq = NOTUSED, 
        .pclk_inv  = NOTUSED,
        
        .hs_fp     = NOTUSED, 
        .hs_bp     = NOTUSED, 
        .hs_w      = NOTUSED, 
        .hs_inv    = NOTUSED,
        
        .vs_fp     = NOTUSED, 
        .vs_bp     = NOTUSED, 
        .vs_w      = NOTUSED, 
        .vs_inv    = NOTUSED,
        
        .blank_inv      = NOTUSED,
        
        .pwmfreq        = NOTUSED,
        .brightness_min = NOTUSED,
        .brightness_max = NOTUSED,
    },
    /* 8: LQ150X1LGN2A 1024x768*/
    {
        .rezx      = 1024, 
        .rezy      = 768, 
        .bpp       = 16,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 9: SHARP LQ104V1DG61 640x480*/
    
    {
        .rezx      = 640, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 24000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 65, 
        .hs_bp     = 45, 
        .hs_w      = 97, 
        .hs_inv    = 1,
        
        .vs_fp     = 11, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 10: NEC NL10276BC30 1024x768*/ 
    
    {
        .rezx      = 1024, 
        .rezy      = 768, 
        .bpp       = 16,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 11: Sharp LQ200T3LZ18 1366x768*/
    {
        .rezx      = 1366, 
        .rezy      = 768, 
        .bpp       = 16,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 11, 
        .hs_bp     = 221, 
        .hs_w      = 101, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 28, 
        .vs_w      = 11, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 12: Sharp LQ43T3DX0A 480x272*/
    {
        .rezx      = 480, 
        .rezy      = 272, 
        .bpp       = 16,
        
        .pclk_freq = 9500, 
        .pclk_inv  = 0,
        
        .hs_fp     = 3, 
        .hs_bp     = 3, 
        .hs_w      = 42, 
        .hs_inv    = 1,
        
        .vs_fp     = 3, 
        .vs_bp     = 3, 
        .vs_w      = 11, 
        .vs_inv    = 1,
        
        .blank_inv      = 1,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 13: Powertip PS480272T-001 480x272 */
    {
        .rezx      = 480, 
        .rezy      = 272, 
        .bpp       = 16,
        
        .pclk_freq = 9500, 
        .pclk_inv  = 0,
        
        .hs_fp     = 3, 
        .hs_bp     = 3, 
        .hs_w      = 42, 
        .hs_inv    = 1,
        
        .vs_fp     = 3, 
        .vs_bp     = 3, 
        .vs_w      = 11, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 14: Dummy, not used*/
    {
        .rezx      = NOTUSED, 
        .rezy      = NOTUSED, 
        .bpp       = NOTUSED,
        
        .pclk_freq = NOTUSED, 
        .pclk_inv  = NOTUSED,
        
        .hs_fp     = NOTUSED, 
        .hs_bp     = NOTUSED, 
        .hs_w      = NOTUSED, 
        .hs_inv    = NOTUSED,
        
        .vs_fp     = NOTUSED, 
        .vs_bp     = NOTUSED, 
        .vs_w      = NOTUSED, 
        .vs_inv    = NOTUSED,
        
        .blank_inv      = NOTUSED,
        
        .pwmfreq        = NOTUSED,
        .brightness_min = NOTUSED,
        .brightness_max = NOTUSED,
    },
    /* 15: DataImage 640x480 TFT*/
    {
        .rezx      = 640, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 48000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 16: Chimei G133I1-L02 1280x800*/
    {
        .rezx      = 1280, 
        .rezy      = 800, 
        .bpp       = 16,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 11, 
        .hs_bp     = 221, 
        .hs_w      = 101, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 28, 
        .vs_w      = 11, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 17: Innolux AT070-83-TT-12 800x480*/
    {
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 38000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 210, 
        .hs_bp     = 45, 
        .hs_w      = 1, 
        .hs_inv    = 1,
        
        .vs_fp     = 132, 
        .vs_bp     = 22, 
        .vs_w      = 1, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 18: PowerView T070W2D2 800x480*/
    {
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 32000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 19: Dummy, not used*/
    {
        .rezx      = NOTUSED, 
        .rezy      = NOTUSED, 
        .bpp       = NOTUSED,
        
        .pclk_freq = NOTUSED, 
        .pclk_inv  = NOTUSED,
        
        .hs_fp     = NOTUSED, 
        .hs_bp     = NOTUSED, 
        .hs_w      = NOTUSED, 
        .hs_inv    = NOTUSED,
        
        .vs_fp     = NOTUSED, 
        .vs_bp     = NOTUSED, 
        .vs_w      = NOTUSED, 
        .vs_inv    = NOTUSED,
        
        .blank_inv      = NOTUSED,
        
        .pwmfreq        = NOTUSED,
        .brightness_min = NOTUSED,
        .brightness_max = NOTUSED,
    },
    /* 20: Evervision VGG804806 800x480*/
    {
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 32000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 21: Chimei G121S1-L01 800x600*/
    {
        .rezx      = 800, 
        .rezy      = 600, 
        .bpp       = 16,
        
        .pclk_freq = 48000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 22: Chimei G104V1_T01_CH01_002 640x480*/
    {
        .rezx      = 640, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 24000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 144, 
        .hs_w      = 96, 
        .hs_inv    = 1,
        
        .vs_fp     = 32, 
        .vs_bp     = 12, 
        .vs_w      = 2, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 23: Sharp LQ075V3DG01 640x480*/
    {
        .rezx      = 640, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 24000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 15, 
        .hs_bp     = 47, 
        .hs_w      = 95, 
        .hs_inv    = 1,
        
        .vs_fp     = 11, 
        .vs_bp     = 30, 
        .vs_w      = 4, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 24: INNOLUX AT102TN03 TFT 640x480*/
    {
        .rezx      = 640, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 38000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 112, 
        .hs_bp     = 112, 
        .hs_w      = 32, 
        .hs_inv    = 1,
        
        .vs_fp     = 44, 
        .vs_bp     = 0, 
        .vs_w      = 1, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 25: Powertip PS480272T-005-I11Q 480x272*/
    {
        .rezx      = 480, 
        .rezy      = 272, 
        .bpp       = 16,
        
        .pclk_freq = 12000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 3, 
        .hs_bp     = 1, 
        .hs_w      = 41, 
        .hs_inv    = 1,
        
        .vs_fp     = 1, 
        .vs_bp     = 3, 
        .vs_w      = 10, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 26: Hitachi TX14D17VM1BPB 640x480*/
    {
        .rezx      = 640, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 24000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 144, 
        .hs_w      = 96, 
        .hs_inv    = 1,
        
        .vs_fp     = 32, 
        .vs_bp     = 12, 
        .vs_w      = 2, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 27: Toshiba LT084AC27500 800x600*/
    {
        .rezx      = 800, 
        .rezy      = 600, 
        .bpp       = 16,
        
        .pclk_freq = 38000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 28: NEC NL6448BC26 640x480*/
    {
        .rezx      = 640, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 24000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 144, 
        .hs_w      = 96, 
        .hs_inv    = 1,
        
        .vs_fp     = 32, 
        .vs_bp     = 12, 
        .vs_w      = 2, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 29: Evervision VGG804806_PWM 800x480*/
    {
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 38000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 30: Dummy, not used*/
    {
        .rezx      = NOTUSED, 
        .rezy      = NOTUSED, 
        .bpp       = NOTUSED,
        
        .pclk_freq = NOTUSED, 
        .pclk_inv  = NOTUSED,
        
        .hs_fp     = NOTUSED, 
        .hs_bp     = NOTUSED, 
        .hs_w      = NOTUSED, 
        .hs_inv    = NOTUSED,
        
        .vs_fp     = NOTUSED, 
        .vs_bp     = NOTUSED, 
        .vs_w      = NOTUSED, 
        .vs_inv    = NOTUSED,
        
        .blank_inv      = NOTUSED,
        
        .pwmfreq        = NOTUSED,
        .brightness_min = NOTUSED,
        .brightness_max = NOTUSED,
    },
    /* 31: Evervision VGG804806_HSE03_PWM 800x480*/
    {
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 30000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 32: CHIMEI TG070Y2-L01 800x480*/
    {
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 30000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 33: Optrex T-55335D150J-FW 1024x768*/
    {
        .rezx      = 1024, 
        .rezy      = 768, 
        .bpp       = 16,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 34: LG LB104S10-TL02 800x600*/
    {
        .rezx      = 800, 
        .rezy      = 600, 
        .bpp       = 16,
        
        .pclk_freq = 38000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 35: CHIMEI G133I1-L02_PWM 1280x800*/
    {
        .rezx      = 1280, 
        .rezy      = 800, 
        .bpp       = 16,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 11, 
        .hs_bp     = 221, 
        .hs_w      = 101, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 28, 
        .vs_w      = 11, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 36: AUO G104SN02_V2 800x600*/
    {
        .rezx      = 800, 
        .rezy      = 600, 
        .bpp       = 16,
        
        .pclk_freq = 38000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 37: Powertip 320x240 */
    {
        .rezx      = 320, 
        .rezy      = 240, 
        .bpp       = 16,
        
        .pclk_freq = 8000,  
        .pclk_inv  = 1,
        
        .hs_fp     = 20, 
        .hs_bp     = 38, 
        .hs_w      = 30, 
        .hs_inv    = 1,
        
        .vs_fp     = 5, 
        .vs_bp     = 15, 
        .vs_w      = 4, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 38: AUO G121SN01V4 800x600*/
    {
        .rezx      = 800, 
        .rezy      = 600, 
        .bpp       = 16,
        
        .pclk_freq = 36000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 39: AUO G150XG01 1024x768*/
    {
        .rezx      = 1024, 
        .rezy      = 768, 
        .bpp       = 16,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 40: Innolux AT050TN33 480x272*/
    {
        .rezx      = 480, 
        .rezy      = 272, 
        .bpp       = 16,
        
        .pclk_freq = 12000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 3, 
        .hs_bp     = 1, 
        .hs_w      = 41, 
        .hs_inv    = 1,
        
        .vs_fp     = 1, 
        .vs_bp     = 3, 
        .vs_w      = 10, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 41: Chimei G133I1-L03 1280x800*/
    {
        .rezx      = 1280, 
        .rezy      = 800, 
        .bpp       = 16,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 11, 
        .hs_bp     = 110, 
        .hs_w      = 50, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 28, 
        .vs_w      = 11, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 42: Chimei G121I1-L01 1280x800*/
    {
        .rezx      = 1280, 
        .rezy      = 800, 
        .bpp       = 16,
        
        .pclk_freq = 72000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 11, 
        .hs_bp     = 221, 
        .hs_w      = 101, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 28, 
        .vs_w      = 11, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 43: Ampire AM-800480R2TMQW-T02H 800x480*/
    {
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 28000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 44: Tianma TM043NBH02 480x272*/
    {
        .rezx      = 480, 
        .rezy      = 272, 
        .bpp       = 16,
        
        .pclk_freq = 12000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 3, 
        .hs_bp     = 1, 
        .hs_w      = 41, 
        .hs_inv    = 1,
        
        .vs_fp     = 1, 
        .vs_bp     = 3, 
        .vs_w      = 10, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 45: AGL VM15B2 V4 1024x768 15" */
    {
        .rezx      = 1024, 
        .rezy      = 768, 
        .bpp       = 16,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 46: TIANMA TM050RDH03 800x480 */
    {
      .rezx      = 800, 
      .rezy      = 480, 
      .bpp       = 16,
      
      .pclk_freq = 27000, 
      .pclk_inv  = 0,
      
      .hs_fp     = 40, 
      .hs_bp     = 40, 
      .hs_w      = 48, 
      .hs_inv    = 1,
      
      .vs_fp     = 13, 
      .vs_bp     = 30, 
      .vs_w      = 3, 
      .vs_inv    = 1,
      
      .blank_inv      = 0,
      
      .pwmfreq        = 10000,
      .brightness_min = 1,
      .brightness_max = 100,
    },
    /* 47: AUO G101EVN01.0 1280x800 */
    {
      .rezx      = 1280, 
      .rezy      = 800, 
      .bpp       = 16,
      
      .pclk_freq = 64000, 
      .pclk_inv  = 0,
      
      .hs_fp     = 11, 
      .hs_bp     = 110, 
      .hs_w      = 50, 
      .hs_inv    = 1,
      
      .vs_fp     = 2, 
      .vs_bp     = 28, 
      .vs_w      = 11, 
      .vs_inv    = 1,
      
      .blank_inv      = 0,
      
      .pwmfreq        = 4000,
      .brightness_min = 5,
      .brightness_max = 100,
    },    
    /* 48: Evervision VGG804806 for eTOP607 800x480 */
    {
      .rezx      = 800, 
      .rezy      = 480, 
      .bpp       = 16,
      
      .pclk_freq = 30000, 
      .pclk_inv  = 0,
      
      .hs_fp     = 41, 
      .hs_bp     = 35, 
      .hs_w      = 129, 
      .hs_inv    = 1,
      
      .vs_fp     = 12, 
      .vs_bp     = 35, 
      .vs_w      = 3, 
      .vs_inv    = 1,
      
      .blank_inv      = 0,
      
      .pwmfreq        = 10000,
      .brightness_min = 1,
      .brightness_max = 70,
    },   
    /* 49: Rocktech RK070EH1401-T 800x480*/
    {
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 30000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 205, 
        .hs_bp     = 46, 
        .hs_w      = 3, 
        .hs_inv    = 1,
        
        .vs_fp     = 20, 
        .vs_bp     = 23, 
        .vs_w      = 2, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 100,
    },     
    /* 50: Rocktech RK101EH1401-T 1024x600*/
    {
        .rezx      = 1024, 
        .rezy      = 600, 
        .bpp       = 16,
        
        .pclk_freq = 51000, 
        .pclk_inv  = 1,
        
        .hs_fp     = 10, 
        .hs_bp     = 320, 
        .hs_w      = 10, 
        .hs_inv    = 0,
        
        .vs_fp     = 10, 
        .vs_bp     = 35, 
        .vs_w      = 10, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 100,
    },     
};

#endif
