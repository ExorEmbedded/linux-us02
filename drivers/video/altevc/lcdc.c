/*
 * AltEVC controller handling : contains the low level functions for interfacing with the AltEVC FPGA 
 * core instantiated on the ARM SOCFPGA CPU  
 *
 * Copyright (C) 2016 Exor International S.p.a.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/backlight.h>

#include <linux/fb.h>
#include <linux/init.h>

#include "altevc.h"

extern dma_addr_t    videomemoryphys;
extern u32           displayindex;
extern void __iomem* evcbase;
extern struct mutex  lock;
struct backlight_device	*bl = NULL;

/* 
 * This is the global variable containing the display id value passed from cmdline
 */
extern int hw_dispid;

/*
 * This is the external variable indicating if the DVI plugin module is installed and the actual dvi cfg.
 */
extern int dvi_dispid; 

typedef struct pll_reconfig_data
{
  u32 f_khz;
  u32 pll_mode;
  u32 pll_ncount;
  u32 pll_mcount;
  u32 pll_ccount;
  u32 pll_phase;
  u32 pll_kfrac;
  u32 pll_bandwidth;    
  u32 pll_chargepump;   
  u32 pll_vcodiv;       
} _pll_reconfig_data;

/*
 * Helper function to perform the powerup sequence of the display output subsystem
 */
static void altevc_powerup(void)
{
    u32 pwctrl = readl(evcbase + PWRCTRL_REG);
 
    pwctrl |= PWR_VDD_ENA;
    writel(pwctrl, evcbase + PWRCTRL_REG);
    msleep(150);
    
    pwctrl |= PWR_VIDEO_ENA;
    writel(pwctrl, evcbase + PWRCTRL_REG);
    msleep(100);
    
    pwctrl |= PWR_BL_ENA;
    // The backlight will be eventually enabled by the backlight subsystem writel(pwctrl, evcbase + PWRCTRL_REG);
}

/*
 * Helper function to set the reconfigurable PLL in order best fit the desired pixel clock frequency.
 */
static void altevc_setclkctrl(u32 desired_freq)
{
    struct pll_reconfig_data pll_cfg[] = {
      //f_khz  PLL_MODE PLL_NCOUNT PLL_MCOUNT PLL_CCOUNT PLL_PHASE PLL_KFRAC PLL_BANDWIDTH PLL_CHARGEPUMP PLL_VCODIV
      {8000,   0x01,    0x20302,   0x808,     0x1414,    0x200001, 0xffffff, 0x07,         0x01,          0x01},
      {9000,   0x01,    0x20302,   0x909,     0x1414,    0x200001, 0xffffff, 0x07,         0x01,          0x01},
      {12000,  0x01,    0x3ffff,   0x20201,   0x20d0c,   0x200001, 0xffffff, 0x09,         0x02,          0x01},
      {27000,  0x01,    0x20302,   0x20e0d,   0x10a,     0x200001, 0xffffff, 0x07,         0x01,          0x00},
      {28000,  0x01,    0x20302,   0x20b0a,   0x20807,   0x200001, 0xffffff, 0x07,         0x01,          0x00},
      {30000,  0x01,    0x3ffff,   0x20201,   0x505,     0x200001, 0xffffff, 0x09,         0x02,          0x01},
      {36000,  0x01,    0x20302,   0x909,     0x505,     0x200001, 0xffffff, 0x07,         0x01,          0x00},
      {41000,  0x01,    0xa0a,     0x23e3d,   0x20807,   0x200001, 0xffffff, 0x04,         0x01,          0x00},
      {51000,  0x01,    0xa0a,     0x24d4c,   0x20807,   0x200001, 0xffffff, 0x03,         0x01,          0x00},
      {61000,  0x01,    0x505,     0x21f1e,   0x505,     0x200001, 0xffffff, 0x06,         0x01,          0x00},
      {64000,  0x01,    0x20302,   0x808,     0x20302,   0x200001, 0xffffff, 0x07,         0x01,          0x01},
      {72000,  0x01,    0x20302,   0x909,     0x20302,   0x200001, 0xffffff, 0x07,         0x01,          0x01},
      {0,      0,       0,         0,         0,         0,        0,        0,            0,             0},
    };

    u32 min_err = 0xfffffff;
    u32 err;
    int i;
    int j = 0;
    
    for(i=0; pll_cfg[i].f_khz != 0; i++)
    {
      if(pll_cfg[i].f_khz > desired_freq)
	err = pll_cfg[i].f_khz - desired_freq;
      else
	err = desired_freq - pll_cfg[i].f_khz;
      
      if(err < min_err)
      {
	min_err = err;
	j = i;
      }
    }
    
    printk("*** Setting pclk freq to %d Khz...\n",pll_cfg[j].f_khz);
    
    writel(pll_cfg[j].pll_mode , evcbase + PLL_RECONFIG_MODE);
    writel(pll_cfg[j].pll_ncount , evcbase + PLL_RECONFIG_N_COUNT);
    writel(pll_cfg[j].pll_mcount , evcbase + PLL_RECONFIG_M_COUNT);
    writel(pll_cfg[j].pll_ccount , evcbase + PLL_RECONFIG_C_COUNT);
    writel(pll_cfg[j].pll_phase , evcbase + PLL_RECONFIG_PHASE);
    writel(pll_cfg[j].pll_kfrac , evcbase + PLL_RECONFIG_K_COUNT);
    writel(pll_cfg[j].pll_bandwidth , evcbase + PLL_RECONFIG_BW);
    writel(pll_cfg[j].pll_chargepump , evcbase + PLL_RECONFIG_CHARGEPUMP);
    writel(pll_cfg[j].pll_vcodiv , evcbase + PLL_RECONFIG_VCODIV);
    
    writel(0 , evcbase + PLL_RECONFIG_START);
    msleep(1);
}

/*
 * Configures the altevc core according with the selected display and activates it 
 */
int altevc_setvideomode(void)
{
  u32 dctrl;
  struct t_DisplayParams* pdispconfig = NULL;
  
  mutex_lock(&lock);
 
  if((hw_dispid == NODISPLAY)&&(dvi_dispid != NODISPLAY))  
  {
    pdispconfig = dviconfig;
  }
  else
  {
    pdispconfig = displayconfig;
  }
  
  printk(KERN_INFO "*** Configuring the LCD display according with the following cfg:\n");
  printk(KERN_INFO "*** Resolution....: %ld x %ld\n",pdispconfig[displayindex].rezx,pdispconfig[displayindex].rezy);
  printk(KERN_INFO "*** Pclk..........: Freq=%ld Inv=%ld\n",pdispconfig[displayindex].pclk_freq,pdispconfig[displayindex].pclk_inv);
  printk(KERN_INFO "*** Hsync.........: FP=%ld BP=%ld W=%ld Inv=%ld\n",pdispconfig[displayindex].hs_fp,pdispconfig[displayindex].hs_bp,pdispconfig[displayindex].hs_w,pdispconfig[displayindex].hs_inv);
  printk(KERN_INFO "*** Vsync.........: FP=%ld BP=%ld W=%ld Inv=%ld\n",pdispconfig[displayindex].vs_fp,pdispconfig[displayindex].vs_bp,pdispconfig[displayindex].vs_w,pdispconfig[displayindex].vs_inv);
  printk(KERN_INFO "*** Blank.........: Inv=%ld\n",pdispconfig[displayindex].blank_inv);
  
  /* Start with cores in OFF state */
  writel(0, evcbase + CVO_CONTROL);
  writel(0, evcbase + ALTVIPFB_CONTROL);
  
  /* Setup sync polarities */
  dctrl = DCTRL_VSYNC_EN | DCTRL_HSYNC_EN | DCTRL_BLANK_EN;
  
  if(dvi_dispid != NODISPLAY)
  { /* If we have the DVI plugin module, fixed sync polarities need to be used */
    dctrl |= DCTRL_VCLK_INV;
  }
  else
  {
    if(pdispconfig[displayindex].hs_inv)
      dctrl |= DCTRL_HSYNC_INV;
    
    if(pdispconfig[displayindex].vs_inv)
      dctrl |= DCTRL_VSYNC_INV;
    
    if(pdispconfig[displayindex].blank_inv)
      dctrl |= DCTRL_BLANK_INV;

    if(pdispconfig[displayindex].pclk_inv)
      dctrl |= DCTRL_VCLK_INV;
  }
  
  writel(dctrl, evcbase + DATACTRL_REG);
  
  /* Pixel clock cfg (use fixed 64Mhz freq, if we have no local dispaly and we use the DVI resolution as system resolution*/
  if((hw_dispid == NODISPLAY)&&(dvi_dispid != NODISPLAY))
    altevc_setclkctrl(DVI_FIXEDCLKFREQ);
  else
    altevc_setclkctrl(pdispconfig[displayindex].pclk_freq);
   
  /* Frame Reader Setup */
  writel(videomemoryphys, evcbase + ALTVIPFB_FRAME0_BASE_ADDRESS);
  writel(pdispconfig[displayindex].rezx * pdispconfig[displayindex].rezy / (ALTEVC_INTERNAL_BUS_WIDTH/16), evcbase + ALTVIPFB_FRAME0_NUM_WORDS);
  writel(pdispconfig[displayindex].rezx * pdispconfig[displayindex].rezy, evcbase + ALTVIPFB_FRAME0_SAMPLES);
  writel(pdispconfig[displayindex].rezx, evcbase + ALTVIPFB_FRAME0_WIDTH);
  writel(pdispconfig[displayindex].rezy, evcbase + ALTVIPFB_FRAME0_HEIGHT);
  writel(3, evcbase + ALTVIPFB_FRAME0_INTERLACED);
  writel(0, evcbase + ALTVIPFB_FRAME_SELECT);
  /* Finally set the control register to 1 to start streaming */
  writel(1, evcbase + ALTVIPFB_CONTROL);

  /* CVO core init */
  writel(0, evcbase + CVO_MODEX_VALID);     // Mode 1 invalid
  writel(0, evcbase + CVO_MODEX_CTRL);      // Progressive, parallel
  writel(pdispconfig[displayindex].rezx, evcbase + CVO_MODEX_XRES); 
  writel(pdispconfig[displayindex].rezy, evcbase + CVO_MODEX_VRES);
  writel(pdispconfig[displayindex].hs_fp, evcbase + CVO_MODEX_HSFP);     
  writel(pdispconfig[displayindex].hs_w, evcbase + CVO_MODEX_HSW);     
  writel(pdispconfig[displayindex].hs_fp + pdispconfig[displayindex].hs_w + pdispconfig[displayindex].hs_bp, evcbase + CVO_MODEX_HSSUM);     
  writel(pdispconfig[displayindex].vs_fp, evcbase + CVO_MODEX_VSFP);     
  writel(pdispconfig[displayindex].vs_w, evcbase + CVO_MODEX_VSW);     
  writel(pdispconfig[displayindex].vs_fp + pdispconfig[displayindex].vs_w + pdispconfig[displayindex].vs_bp, evcbase + CVO_MODEX_VSSUM);     
  writel(1, evcbase + CVO_MODEX_VALID);     // Mode 1 valid
  /* Finally ON CVO */
  writel(1, evcbase + CVO_CONTROL);     // ON CVO
  
  /* Backlight dimming init (for now we set 100% duty) */
  if((pdispconfig[displayindex].pwmfreq < 100000) && (pdispconfig[displayindex].pwmfreq > 1))
    writel(100000 / pdispconfig[displayindex].pwmfreq, evcbase + BL_FREQ_REG); 
  else
    writel(0, evcbase + BL_FREQ_REG);
  
  writel(255, evcbase + BL_PWM_REG);
  
  /* Perform powerup sequence */
  altevc_powerup();
  
  mutex_unlock(&lock);
  return 0;
}

/*----------------------------------------------------------------------------------------------------
 * Backlight dimming related functions
 *----------------------------------------------------------------------------------------------------*/

int altevc_bl_update_status(struct backlight_device *bd)
{
  int level;
  u32 pwctrl;
  
  /* Exit if no local display*/
  if(hw_dispid == NODISPLAY)
    return 0;
  
  mutex_lock(&lock);
  
  /* Get the level */
  if (bd->props.power != FB_BLANK_UNBLANK || bd->props.fb_blank != FB_BLANK_UNBLANK)
    level = 0;
  else
    level = bd->props.brightness;
  
  /* Sanity check */
  if(level >= ALTEVC_BRIGHTNESS_LEVELS)
  {
    bd->props.brightness = ALTEVC_BRIGHTNESS_LEVELS - 1;
    level = ALTEVC_BRIGHTNESS_LEVELS - 1;
  }
  
  if(level < 0)
    level = 0;
  
  /* BL off if level ==0, or set the proper BL dimming otherwise */
  pwctrl = readl(evcbase + PWRCTRL_REG);
  
  if(level == 0)
  {
    pwctrl &= ~ PWR_BL_ENA;
    writel(pwctrl, evcbase + PWRCTRL_REG);
    writel(0, evcbase + BL_PWM_REG);
  }
  else
  {
    int min_duty = displayconfig[displayindex].brightness_min;
    int max_duty = displayconfig[displayindex].brightness_max;
    int duty = min_duty + ((max_duty - min_duty) * (level-1)) / (ALTEVC_BRIGHTNESS_LEVELS - 2);
    duty = (255 * duty)/100;
    if(duty > 255)
      duty = 255;
    
    writel(duty & 0xff, evcbase + BL_PWM_REG);
    pwctrl |= PWR_BL_ENA;
    writel(pwctrl, evcbase + PWRCTRL_REG);
  }
  
  mutex_unlock(&lock);
  return 0;
}

static const struct backlight_ops altevc_bl_data = {
	.update_status	= altevc_bl_update_status,
};

void altevc_bl_init(struct fb_info* sinfo, int initial_brightness)
{
	struct backlight_properties props;
	
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = ALTEVC_BRIGHTNESS_LEVELS - 1;
	bl = backlight_device_register("backlight", sinfo->dev , sinfo, &altevc_bl_data, &props);
	if (IS_ERR(bl)) {
		printk("Error %ld on backlight register\n", PTR_ERR(bl));
		return;
	}

	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.brightness = initial_brightness;
	altevc_bl_update_status(bl);
}

void altevc_bl_deinit(void)
{
  if(bl)
    backlight_device_unregister(bl);
  bl=NULL;
}

/*
 * Returns if the backlight is on (true) or off (false)
 */
bool altevc_bl_get_status(void)
{
  int level;

  if(bl == NULL)
    return false;

  /* Get the level */
  if (bl->props.power != FB_BLANK_UNBLANK || bl->props.fb_blank != FB_BLANK_UNBLANK)
    level = 0;
  else
    level = bl->props.brightness;
  
  if(level < 0)
    level = 0;
  
  if(level)
    return true;
  
  return false;
}
EXPORT_SYMBOL(altevc_bl_get_status);