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
  u32 f_Khz;
  u32 pll_mode;
  u32 pll_ncount;
  u32 pll_mcount;
  u32 pll_ccount;
  u32 pll_phase;
  u32 pll_kfrac;
  u32 pll_bandwidth;    
  u32 pll_chargepump;   
  u32 pll_vcodiv;       
};

/*
 * Helper function to perform the powerup sequence
 */
static void altevc_powerup(void)
{
  printk("*** TODO: Enable Vdd\n");
  msleep(150);
  
  printk("*** TODO: Enable LCD output\n");
  msleep(150);
    
  printk("*** TODO: Enable Backlight\n");
}

/*
 * Helper function to set the best pixclk frequency
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
    u32 min_err = 0xffffffff;
    u32 err;
    int i,j;

    printk("*** TODO: Choose the nearest PLL frequency and set PLL registers accordingly\n");
}

/*
 * Configures the altevc core according with the selected display and activates it 
 */
int altevc_setvideomode(void)
{
        volatile u32 sctrl;
        
        mutex_lock(&lock);

        printk(KERN_INFO "*** Configuring the LCD display according with the following cfg:\n");
        printk(KERN_INFO "*** Resolution....: %ld x %ld\n",displayconfig[displayindex].rezx,displayconfig[displayindex].rezy);
        printk(KERN_INFO "*** Pclk..........: Freq=%ld Inv=%ld\n",displayconfig[displayindex].pclk_freq,displayconfig[displayindex].pclk_inv);
        printk(KERN_INFO "*** Bpp...........: %ld\n",displayconfig[displayindex].bpp);
        printk(KERN_INFO "*** Hsync.........: FP=%ld BP=%ld W=%ld Inv=%ld\n",displayconfig[displayindex].hs_fp,displayconfig[displayindex].hs_bp,displayconfig[displayindex].hs_w,displayconfig[displayindex].hs_inv);
        printk(KERN_INFO "*** Vsync.........: FP=%ld BP=%ld W=%ld Inv=%ld\n",displayconfig[displayindex].vs_fp,displayconfig[displayindex].vs_bp,displayconfig[displayindex].vs_w,displayconfig[displayindex].vs_inv);
        printk(KERN_INFO "*** Blank.........: Inv=%ld\n",displayconfig[displayindex].blank_inv);
        
        /* Horizontal resolution and Hsync */
        /* Vertical resolution and Vsync */
        /* Sync signals polarity */
        /* Set phy address for framebuffer memory */
        /* Set stripe size (our framebuffer is always 32bpp)*/
	printk("*** TODO: Set CORE registers accordingly\n");
        
        /* Pixel clock cfg */
        altevc_setclkctrl();
        
        /* Perform powerup sequence */
        altevc_powerup();

        mutex_unlock(&lock);
        return 0;
}


