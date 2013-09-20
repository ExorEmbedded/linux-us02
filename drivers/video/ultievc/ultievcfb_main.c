/*
 *  UltiEVC frame buffer device for the Altera SOC FPGA device
 * 
 * This is a basic driver with following features:
 *  - 1 layer only
 *  - RGB565 16 bit only
 *  - NO scroll
 *  - NO rotation
 *  - NO HW acceleration
 *  - Fixed video memory size to 4MB
 *  - Display parameters (resolution/timings) based on display ID passed at probe level
 * 
 * Copyright (C) 2013 Exor International
 * Author: Giovanni Pavoni (Exor)
 * 
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
#include <linux/io.h>

#include "ultievc.h"
#include "displayconfig.h"

/*
 *  RAM we reserve for the frame buffer. This defines the maximum screen
 *  size
 */
#define VIDEOMEMSIZE	(4*1024*1024)	/* 4 MB */
static void*      videomemory;
static u_long     videomemorysize = VIDEOMEMSIZE;
dma_addr_t        videomemoryphys;

/*
 * Base address for accessing the UltiEVC registers
 */
void __iomem*     evcbase = NULL;

/*
 * Global variable indicating the display identifier
 */
u32 displayid = 0;

/*
 * Mutex for locking
 */
struct mutex lock;

/*
 * Function for allocating the framebuffer memory for UltiEVC DMA operation
 */
static void *alloc_fbmem(struct device* dev, unsigned long size)
{
    void *mem;
    size = PAGE_ALIGN(size);
 
    printk(KERN_INFO "*** Trying to allocate framebuffer memory: Size=%ld\n", size);
    
    mem = dma_alloc_writecombine(dev, size, &videomemoryphys, GFP_KERNEL);
    if (!mem)
        return NULL;

    memset(mem, 0, size); /* Clear the ram out, no junk to the user */
    return mem;
}

/*
 *  function for deallocating the framebuffer memory
 */
static void free_fbmem(struct device* dev, unsigned long size)
{
    dma_free_writecombine(dev, size, videomemory, videomemoryphys);
}

/* We set default configuration for the 1024x768 15" AGL TFT
 */ 
static struct fb_var_screeninfo ultievcfb_default = {
    .xres =		1024,
    .yres =		768,
    .xres_virtual =	1024,
    .yres_virtual =	768,
    .bits_per_pixel = 16,
    .red =   { 11, 5, 0 },
    .green = { 5 , 6, 0 },
    .blue =  { 0 , 5, 0 },
    .activate =	FB_ACTIVATE_TEST,
    .height= -1,
    .width = -1,
    .pixclock =	64000,
    .left_margin =	16,
    .right_margin =	149,
    .upper_margin =	2,
    .lower_margin =	36,
    .hsync_len =   69,
    .vsync_len =   7,
    .vmode =FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo ultievcfb_fix = {
    .id =		"ultievcfb",
    .type =		FB_TYPE_PACKED_PIXELS,
    .visual =	FB_VISUAL_TRUECOLOR,
    .xpanstep =	0,
    .ypanstep =	0,
    .ywrapstep =	0,
    .accel =	FB_ACCEL_NONE,
};

static int ultievcfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int ultievcfb_set_par(struct fb_info *info);
static int ultievcfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue, u_int transp, struct fb_info *info);

static struct fb_ops ultievcfb_ops = {
    .fb_check_var  = ultievcfb_check_var,
    .fb_set_par    = ultievcfb_set_par,
    .fb_setcolreg  = ultievcfb_setcolreg,
    .fb_fillrect   = cfb_fillrect,
    .fb_copyarea   = cfb_copyarea,
    .fb_imageblit  = cfb_imageblit,
};

/*
 * Helper function to get the length in bytes of the framebuffer line 
 */
static u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;
	length = xres_virtual * bpp / 8;
	return (length);
}

/*
 * Since we can not modify the screen resolution at runtime, we just need to  
 * copy to var the current settings (which can not be changed)
 */
static int ultievcfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
    *var = info -> var;
	return 0;
}

/*
 * This routine sets the video mode and initializes
 * the videocontroller.
 */
static int ultievcfb_set_par(struct fb_info *info)
{
	info->fix.line_length = get_line_length(info->var.xres_virtual, info->var.bits_per_pixel);
    return ultievc_setvideomode();
}

/*
 *  Set a single color register. The values supplied are already
 *  rounded down to the hardware's capabilities (according to the
 *  entries in the var structure). Return != 0 for invalid regno.
 */
static int ultievcfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info)
{
	if (regno >= 256)	/* no. of hw registers */
		return 1;

    /*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue =
		    (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Directcolor:
	 *   var->{color}.offset contains start of bitfield
	 *   var->{color}.length contains length of bitfield
	 *   {hardwarespecific} contains width of RAMDAC
	 *   cmap[X] is programmed to (X << red.offset) | (X << green.offset) | (X << blue.offset)
	 *   RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Pseudocolor:
	 *    var->{color}.offset is 0 unless the palette index takes less than
	 *                        bits_per_pixel bits and is stored in the upper
	 *                        bits of the pixel value
	 *    var->{color}.length is set so that 1 << length is the number of available
	 *                        palette entries
	 *    cmap is not used
	 *    RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Truecolor:
	 *    does not use DAC. Usually 3 are present.
	 *    var->{color}.offset contains start of bitfield
	 *    var->{color}.length contains length of bitfield
	 *    cmap is programmed to (red << red.offset) | (green << green.offset) |
	 *                      (blue << blue.offset) | (transp << transp.offset)
	 *    RAMDAC does not exist
	 */
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);
		switch (info->var.bits_per_pixel) {
		case 8:
			break;
		case 16:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		return 0;
	}
	return 0;
}


/*
 *  Initialisation
 */
static int ultievcfb_probe(struct platform_device *dev)
{
	struct fb_info *info;
	int retval = -ENOMEM;
    static int f_probed;
    struct resource *res;
    struct device devx;
    
    mutex_init(&lock);
    
    /*
     * Sanity check to avoid void pointers and multiple allocations
     */
    devx= dev->dev;
    if(!devx.platform_data)
        return -ENODEV;
    
    if(f_probed)
        return -ENODEV;
    
    f_probed = 1;
    
    /*
     * Get the display ID and perform the required sanity checks
     */
    displayid = *((u32*)devx.platform_data);
    printk(KERN_INFO "*** ultievcfb_probe. DisplayId=%d\n", displayid);
    
    if(displayid > MAXNRDISPLAYS)
    {
        printk(KERN_ERR"*** ultievcfb_probe: ERROR : displayid value out of range\n");
        return -ENODEV;
    }
    
    if((displayconfig[displayid].rezx == NOTUSED)||(displayconfig[displayid].rezy == NOTUSED))
    {
        printk(KERN_ERR"*** ultievcfb_probe: ERROR : invalid display\n");
        return -ENODEV;
    }
    
    /* 
     * Allocates resources for FPGA registers
     */
    res = platform_get_resource(dev, IORESOURCE_MEM, 0);
    if (!res)
    {
        printk(KERN_ERR"*** ultievcfb_probe: Failed to allocate FPGA resources: no specified resources\n");
        return retval;
    }
    
    if (!devm_request_mem_region(&dev->dev, res->start, resource_size(res), dev->name))
    {
        printk(KERN_ERR"*** ultievcfb_probe: Failed to allocate FPGA resources (1) \n");
        return retval;
    }
    
    evcbase = devm_ioremap_nocache(&dev->dev, res->start, resource_size(res));
    if (!evcbase)
    {
        printk(KERN_ERR"*** ultievcfb_probe: Failed to allocate FPGA resources (1) \n");
        return retval;
    }
    
    printk(KERN_INFO "*** ultievcfb_probe: FPGA phybase=0x%x mapped at evcbase=0x%x\n",(unsigned int)res->start, (unsigned int)evcbase);

	/*  
	 *  Memory allocation for framebuffer
	 */
	if (!(videomemory = alloc_fbmem(&dev->dev, videomemorysize)))
    {
        printk(KERN_ERR"*** ultievcfb_probe: Failed to allocate framebuffer memory\n");
		return retval;
    }
    
	/*
	 * ultievcfb must clear memory to prevent kernel info
	 */
	memset(videomemory, 0, videomemorysize);

	info = framebuffer_alloc(sizeof(u32) * 256, &dev->dev);
	if (!info)
		goto err;

	info->screen_base = (char __iomem *)videomemory;
	info->fbops = &ultievcfb_ops;
    
    /*
     * Configuration based on the selected display 
     */
	info->var = ultievcfb_default;

    info->var.xres = displayconfig[displayid].rezx;
    info->var.yres = displayconfig[displayid].rezy;
    info->var.xres_virtual = displayconfig[displayid].rezx;
    info->var.yres_virtual = displayconfig[displayid].rezy;

    info->var.pixclock = displayconfig[displayid].pclk_freq;
 
    info->var.left_margin = displayconfig[displayid].hs_fp;
    info->var.right_margin = displayconfig[displayid].hs_bp;
    info->var.hsync_len = displayconfig[displayid].hs_w;
    
    info->var.upper_margin = displayconfig[displayid].vs_fp;
    info->var.lower_margin = displayconfig[displayid].vs_bp;
    info->var.vsync_len = displayconfig[displayid].vs_w;

    ultievcfb_fix.smem_start = (unsigned long) videomemoryphys;
	ultievcfb_fix.smem_len = videomemorysize;
	info->fix = ultievcfb_fix;
	info->pseudo_palette = info->par;
	info->par = NULL;
	info->flags = FBINFO_FLAG_DEFAULT;
    info->screen_base = videomemory;
    info->fix.line_length = get_line_length(info->var.xres_virtual, info->var.bits_per_pixel);

	retval = fb_alloc_cmap(&info->cmap, 256, 0);
	if (retval < 0)
		goto err1;

	retval = register_framebuffer(info);
	if (retval < 0)
		goto err2;
	platform_set_drvdata(dev, info);

	printk(KERN_INFO "*** fb%d: Virtual frame buffer device allocated OK. Size=%ld Vaddr=0x%x Phyaddr=0x%x\n", info->node, 
           videomemorysize, (unsigned int)videomemory, videomemoryphys);
    
    /*
     *  All OK, here we configure and activate the videocontroller
     */
    return ultievc_setvideomode();
err2:
	fb_dealloc_cmap(&info->cmap);
err1:
	framebuffer_release(info);
err:
	free_fbmem(&dev->dev, videomemorysize);
	return retval;
}

static int ultievcfb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);

	if (info) {
		unregister_framebuffer(info);
		free_fbmem(&dev->dev, videomemorysize);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}
	return 0;
}

static struct platform_driver ultievcfb_driver = {
	.probe	= ultievcfb_probe,
	.remove = ultievcfb_remove,
	.driver = {
	.name	= "ultievcfb",
	},
};

static struct platform_device *ultievcfb_device;

static int __init ultievcfb_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&ultievcfb_driver);

	if (!ret) {
		ultievcfb_device = platform_device_alloc("ultievcfb", 0);

		if (ultievcfb_device)
			ret = platform_device_add(ultievcfb_device);
		else
			ret = -ENOMEM;

		if (ret) {
			platform_device_put(ultievcfb_device);
			platform_driver_unregister(&ultievcfb_driver);
		}
	}

	return ret;
}

module_init(ultievcfb_init);

