/*
 * Ultievc controller handling : contains the low level functions for interfacing with the UltiEVC FPGA 
 * core instantiated on the ARM SOCFPGA CPU  
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

#include "ultievc.h"
#include "displayconfig.h"


extern dma_addr_t    videomemoryphys;
extern u32           displayindex;
extern void __iomem* evcbase;
extern struct mutex  lock;

/*
 * Helper function to perform the powerup sequence
 */
static void ultievc_powerup(void)
{
    u32 pwctrl = readl(evcbase + PWRCTRL);
    
    pwctrl |= PWRCTRL_VDEN;
    writel(pwctrl, evcbase + PWRCTRL);
    msleep(100);
    
    pwctrl |= PWRCTRL_VEEN;
    writel(pwctrl, evcbase + PWRCTRL);
    msleep(100);
    
    pwctrl |= PWRCTRL_VEN;
    writel(pwctrl, evcbase + PWRCTRL);
    msleep(100);
    
    pwctrl |= PWRCTRL_BLEN;
    writel(pwctrl, evcbase + PWRCTRL);
}

/*
 * Helper function to write the CLKCTRL register according with the required pixel clock specifications
 */
static void ultievc_setclkctrl(void)
{
    u32 base_clk_freq[7] = {24000, 32000, 38000, 48000, 64000, 76000, 96000};
    u32 base_clk_div[4] = {1, 2, 4, 8};
    u32 min_err = 0xffffffff;
    u32 err;
    u32 freq;
    int i,j;
    u32 clk_sel = 0;
    u32 clk_div = 0;
    u32 desired_freq = displayconfig[displayindex].pclk_freq;
    u32 clkctrl = 0;
    
    /* Loops for all the possible configurations and take the one which minimizes the frequency error */
    for(i=0; i<7; i++)
        for(j=0; j<4; j++)
        {
            freq = base_clk_freq[i] / base_clk_div[j];
            if(freq > desired_freq)
                err = freq - desired_freq;
            else
                err = desired_freq - freq;
            
            if(err < min_err)
            {
                min_err = err;
                clk_sel = i;
                clk_div  = j;
            }
        }
    
    printk(KERN_INFO "*** ultievc_setclkctrl clk_sel=0x%x clk_div=0x%x\n",clk_sel, clk_div);
    
    /* Now compute the register fields and write to register */
    clkctrl = clk_sel | (clk_div << 3);
    if(displayconfig[displayindex].pclk_inv)
        clkctrl |= (1 << 7);
    
    writel(clkctrl, evcbase + CLKCTRL);
}

/*
 * Configures the ultievc core according with the selected display and activates it 
 */
int ultievc_setvideomode(void)
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
        writel(displayconfig[displayindex].hs_fp, evcbase + HSY_FP);
        writel(displayconfig[displayindex].hs_bp, evcbase + HSY_BP);
        writel(displayconfig[displayindex].hs_w, evcbase + HSY_W);
        writel(displayconfig[displayindex].rezx, evcbase + HRES);
        
        /* Vertical resolution and Vsync */
        writel(displayconfig[displayindex].vs_fp, evcbase + VSY_FP);
        writel(displayconfig[displayindex].vs_bp, evcbase + VSY_BP);
        writel(displayconfig[displayindex].vs_w, evcbase + VSY_W);
        writel(displayconfig[displayindex].rezy, evcbase + VRES);
        
        /* Sync signals polarity */
        sctrl = SCTRL_VSEN | SCTRL_HSEN | SCTRL_ENEN;
        
        if(!displayconfig[displayindex].hs_inv)
            sctrl |= SCTRL_HSPOL;
        
        if(!displayconfig[displayindex].vs_inv)
            sctrl |= SCTRL_VSPOL;

        if(!displayconfig[displayindex].blank_inv)
            sctrl |= SCTRL_ENPOL;
        
        writel(sctrl, evcbase + SCTRL);
        
        /* Pixel clock cfg */
        ultievc_setclkctrl();
        
        /* Pixel data scrambling settings*/
        if(displayconfig[displayindex].bpp == 16)
            writel(DCTRL_16BOUT, evcbase + DCTRL);
        else
            writel(DCTRL_24BOUT, evcbase + DCTRL);
        
        /* Set phy address for framebuffer memory */
        writel(videomemoryphys, evcbase + LBADDR);
        
        /* Set stripe size (our framebuffer is always 16bpp)*/
        writel(2 * displayconfig[displayindex].rezx - 1, evcbase + LSTRIPE);

        /* Enable master layer*/
        writel(LCTRL_ENABLE, evcbase + LCTRL);
        
        /* Perform powerup sequence */
        ultievc_powerup();
        
        printk(KERN_INFO "*** ULTIEVC regs dump\n"); 
        sctrl =  readl(evcbase + HSY_FP);
        printk(KERN_INFO "*** HSY_FP...........: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + HSY_W); 
        printk(KERN_INFO "*** HSY_W............: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + HSY_BP); 
        printk(KERN_INFO "*** HSY_BP...........: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + HRES); 
        printk(KERN_INFO "*** HRES.............: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + VSY_FP); 
        printk(KERN_INFO "*** VSY_FP...........: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + VSY_W); 
        printk(KERN_INFO "*** VSY_W............: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + VSY_BP); 
        printk(KERN_INFO "*** VSY_BP...........: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + VRES); 
        printk(KERN_INFO "*** VRES.............: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + SCTRL); 
        printk(KERN_INFO "*** SCTRL............: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + CLKCTRL); 
        printk(KERN_INFO "*** CLKCTRL..........: 0x%x\n",sctrl);
        sctrl =  readl(evcbase + DCTRL);
        printk(KERN_INFO "*** DCTRL............: 0x%x\n",sctrl);
        sctrl =  readl(evcbase + PWRCTRL); 
        printk(KERN_INFO "*** PWRCTRL..........: 0x%x\n",sctrl); 
        sctrl =  readl(evcbase + LBADDR); 
        printk(KERN_INFO "*** LBADDR...........: 0x%x\n",sctrl);
        sctrl =  readl(evcbase + LSTRIPE);
        printk(KERN_INFO "*** LSTRIPE..........: 0x%x\n",sctrl);
        sctrl =  readl(evcbase + LCTRL); 
        printk(KERN_INFO "*** LCTRL............: 0x%x\n",sctrl);

        mutex_unlock(&lock);
        return 0;
}


