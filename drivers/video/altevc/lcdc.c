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

#include <linux/fb.h>
#include <linux/init.h>

#include "altevc.h"

extern dma_addr_t    videomemoryphys;
extern u32           displayindex;
extern void __iomem* evcbase;
extern struct mutex  lock;

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
    writel(pwctrl, evcbase + PWRCTRL_REG);
}

/*
 * Helper function to set the reconfigurable PLL in order best fit the desired pixel clock frequency.
 */
static void altevc_setclkctrl(void)
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

    u32 desired_freq = displayconfig[displayindex].pclk_freq;
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
  int i;
  u32 dctrl;
  
  mutex_lock(&lock);
    
  printk(KERN_INFO "*** Configuring the LCD display according with the following cfg:\n");
  printk(KERN_INFO "*** Resolution....: %ld x %ld\n",displayconfig[displayindex].rezx,displayconfig[displayindex].rezy);
  printk(KERN_INFO "*** Pclk..........: Freq=%ld Inv=%ld\n",displayconfig[displayindex].pclk_freq,displayconfig[displayindex].pclk_inv);
  printk(KERN_INFO "*** Bpp...........: %ld\n",displayconfig[displayindex].bpp);
  printk(KERN_INFO "*** Hsync.........: FP=%ld BP=%ld W=%ld Inv=%ld\n",displayconfig[displayindex].hs_fp,displayconfig[displayindex].hs_bp,displayconfig[displayindex].hs_w,displayconfig[displayindex].hs_inv);
  printk(KERN_INFO "*** Vsync.........: FP=%ld BP=%ld W=%ld Inv=%ld\n",displayconfig[displayindex].vs_fp,displayconfig[displayindex].vs_bp,displayconfig[displayindex].vs_w,displayconfig[displayindex].vs_inv);
  printk(KERN_INFO "*** Blank.........: Inv=%ld\n",displayconfig[displayindex].blank_inv);
  
  /* Start with cores in OFF state */
  writel(0, evcbase + CVO_CONTROL);
  writel(0, evcbase + ALTGAMMA_CONTROL);
  writel(0, evcbase + ALTVIPFB_CONTROL);
  
  /* Setup sync polarities */
  dctrl = DCTRL_VSYNC_EN | DCTRL_HSYNC_EN | DCTRL_BLANK_EN;
  
  if(displayconfig[displayindex].hs_inv)
    dctrl |= DCTRL_HSYNC_INV;
  
  if(displayconfig[displayindex].vs_inv)
    dctrl |= DCTRL_VSYNC_INV;
  
  if(displayconfig[displayindex].blank_inv)
    dctrl |= DCTRL_BLANK_INV;

  if(displayconfig[displayindex].pclk_inv)
    dctrl |= DCTRL_VCLK_INV;
  
  writel(dctrl, evcbase + DATACTRL_REG);
  
  /* Pixel clock cfg */
  altevc_setclkctrl();
  
  /* Frame Reader Setup */
  writel(videomemoryphys, evcbase + ALTVIPFB_FRAME0_BASE_ADDRESS);
  writel(displayconfig[displayindex].rezx * displayconfig[displayindex].rezy / (ALTEVC_INTERNAL_BUS_WIDTH/32), evcbase + ALTVIPFB_FRAME0_NUM_WORDS);
  writel(displayconfig[displayindex].rezx * displayconfig[displayindex].rezy, evcbase + ALTVIPFB_FRAME0_SAMPLES);
  writel(displayconfig[displayindex].rezx, evcbase + ALTVIPFB_FRAME0_WIDTH);
  writel(displayconfig[displayindex].rezy, evcbase + ALTVIPFB_FRAME0_HEIGHT);
  writel(3, evcbase + ALTVIPFB_FRAME0_INTERLACED);
  writel(0, evcbase + ALTVIPFB_FRAME_SELECT);
  /* Finally set the control register to 1 to start streaming */
  writel(1, evcbase + ALTVIPFB_CONTROL);
  
  /* Gamma correction setup (start with 1:1 mapping) */
  for(i=0; i<256; i++)
  {
    writel(i, evcbase + ALTGAMMA_LUT0 + (i<<2));
    writel(i, evcbase + ALTGAMMA_LUT1 + (i<<2));
    writel(i, evcbase + ALTGAMMA_LUT2 + (i<<2));
  }
  /* Finally enable the GAMMA correction engine */
  writel(1, evcbase + ALTGAMMA_CONTROL);
  
  /* CVO core init */
  writel(0, evcbase + CVO_MODEX_VALID);     // Mode 1 invalid
  writel(0, evcbase + CVO_MODEX_CTRL);      // Progressive, parallel
  writel(displayconfig[displayindex].rezx, evcbase + CVO_MODEX_XRES); 
  writel(displayconfig[displayindex].rezy, evcbase + CVO_MODEX_VRES);
  writel(displayconfig[displayindex].hs_fp, evcbase + CVO_MODEX_HSFP);     
  writel(displayconfig[displayindex].hs_w, evcbase + CVO_MODEX_HSW);     
  writel(displayconfig[displayindex].hs_fp + displayconfig[displayindex].hs_w + displayconfig[displayindex].hs_bp, evcbase + CVO_MODEX_HSSUM);     
  writel(displayconfig[displayindex].vs_fp, evcbase + CVO_MODEX_VSFP);     
  writel(displayconfig[displayindex].vs_w, evcbase + CVO_MODEX_VSW);     
  writel(displayconfig[displayindex].vs_fp + displayconfig[displayindex].vs_w + displayconfig[displayindex].vs_bp, evcbase + CVO_MODEX_VSSUM);     
  writel(1, evcbase + CVO_MODEX_VALID);     // Mode 1 valid
  /* Finally ON CVO */
  writel(1, evcbase + CVO_CONTROL);     // ON CVO
  
  /* Backlight dimming init (for now we set 100% duty) */
  if((displayconfig[displayindex].pwmfreq < 100000) && (displayconfig[displayindex].pwmfreq > 1))
    writel(100000 / displayconfig[displayindex].pwmfreq - 1, evcbase + BL_FREQ_REG); 
  else
    writel(0, evcbase + BL_FREQ_REG);
  
  writel(255, evcbase + BL_PWM_REG);
  
  
  /* Perform powerup sequence */
  altevc_powerup();
  
  mutex_unlock(&lock);
  return 0;
}


