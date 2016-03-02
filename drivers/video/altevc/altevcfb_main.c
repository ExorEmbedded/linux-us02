/*
 *  AltEVC frame buffer driver
 * 
 * Features are as follows
 *  - 1 layer only
 *  - RGB888 32 bit/pixel only
 *  - NO scroll
 *  - NO HW rotation
 *  - NO HW acceleration
 *  - Max resolution 1920x1080
 *  - Display parameters (resolution/timings) based on display ID passed at probe level
 * 
 * Copyright (C) 2014 Exor International S.p.a.
 * Author: Giovanni Pavoni (Exor Int. s.p.a.)
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

#include "altevc.h"

/*
 *  RAM we reserve for the frame buffer. This defines the maximum screen
 *  size
 */
#define VIDEOMEMSIZE	(8*1024*1024)	/* 8 MB */
static void*      videomemory;
static u_long     videomemorysize = VIDEOMEMSIZE;
dma_addr_t        videomemoryphys;

/* 
 * This is the global variable containing the display id value passed from cmdline
 */
extern int hw_dispid;

/*
 * Base address for accessing the altevc registers
 */
void __iomem*     evcbase = NULL;

/*
 * Global variable indicating the display identifier
 */
u32 displayid = 0;
u32 displayindex = 0;

/*
 * Mutex for locking
 */
struct mutex lock;

/*
 * Function for allocating the framebuffer memory for AltEVC DMA operation
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

/* Default display configuration
 */ 
static struct fb_var_screeninfo altevcfb_default = {
  .xres = 1024,
  .yres = 768,
  .xres_virtual = 1024,
  .yres_virtual = 768,
  .bits_per_pixel = 32,
  .blue =   { 16, 8, 0 },
  .green = { 8 , 8, 0 },
  .red =  { 0 , 8, 0 },
  .activate = FB_ACTIVATE_TEST,
  .height= -1,
  .width = -1,
  .pixclock = 64000,
  .left_margin = 16,
  .right_margin = 149,
  .upper_margin = 2,
  .lower_margin = 36,
  .hsync_len = 69,
  .vsync_len =  7,
  .vmode = FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo altevcfb_fix = {
  .id = "altevcfb",
  .type = FB_TYPE_PACKED_PIXELS,
  .visual = FB_VISUAL_TRUECOLOR,
  .xpanstep = 0,
  .ypanstep = 0,
  .ywrapstep = 0,
  .accel = FB_ACCEL_NONE,
};

static int altevcfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue, u_int transp, struct fb_info *info);

static struct fb_ops altevcfb_ops = {
    .fb_setcolreg  = altevcfb_setcolreg,
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
 *  Set a single color register. The values supplied have a 32 bit
 *  magnitude.
 *  Return != 0 for invalid regno.
 */
static int altevcfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue, u_int transp, struct fb_info *info)
{
  u32 v;

  if (regno >= PALETTE_SIZE)
    return 1;

#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
  red = CNVT_TOHW(red, info->var.red.length);
  green = CNVT_TOHW(green, info->var.green.length);
  blue = CNVT_TOHW(blue, info->var.blue.length);
  transp = CNVT_TOHW(transp, info->var.transp.length);
#undef CNVT_TOHW
  v = (red << info->var.red.offset) | (green << info->var.green.offset) | (blue << info->var.blue.offset) | (transp << info->var.transp.offset);
  ((u32 *) (info->pseudo_palette))[regno] = v;
  return 0;
}


/*
 *  Initialisation
 */
static int altevcfb_probe(struct platform_device *dev)
{
  struct fb_info *info;
  int retval = -ENOMEM;
  static int f_probed;
  struct resource *res;
  struct device devx;
  int ret;
  
  mutex_init(&lock);
  
  /*
   * Sanity check to avoid void pointers and multiple allocations
   */
  devx= dev->dev;
  
  if(f_probed)
    return -ENODEV;
  
  f_probed = 1;
  
  /*
   * Get the display ID and perform the required sanity checks
   */
  displayid = hw_dispid;
  printk(KERN_INFO "*** altevcfb_probe. DisplayId=%d\n", displayid);
  
  displayindex = 0;
  while ((displayconfig[displayindex].dispid != displayid) && (displayconfig[displayindex].dispid != NODISPLAY))
    displayindex ++;
  
  if (displayconfig[displayindex].dispid != displayid)
  {
    displayindex = 0;
    printk(KERN_ERR"*** altevcfb_probe: ERROR : invalid display\n");
    return -ENODEV;
  }
  
  /* 
   * Allocates resources for FPGA registers
   */
  res = platform_get_resource(dev, IORESOURCE_MEM, 0);
  if (!res)
  {
    printk(KERN_ERR"*** altevcfb_probe: Failed to allocate FPGA resources: no specified resources\n");
    return retval;
  }
  
  if (!devm_request_mem_region(&dev->dev, res->start, resource_size(res), dev->name))
  {
    printk(KERN_ERR"*** altevcfb_probe: Failed to allocate FPGA resources (1) \n");
    return retval;
  }
  
  evcbase = devm_ioremap_nocache(&dev->dev, res->start, resource_size(res));
  if (!evcbase)
  {
    printk(KERN_ERR"*** altevcfb_probe: Failed to allocate FPGA resources (1) \n");
    return retval;
  }
  
  printk(KERN_INFO "*** altevcfb_probe: FPGA phybase=0x%x mapped at evcbase=0x%x\n",(unsigned int)res->start, (unsigned int)evcbase);
  
  /*  
   *  Memory allocation for framebuffer
   */
  if (!(videomemory = alloc_fbmem(&dev->dev, videomemorysize)))
  {
    printk(KERN_ERR"*** altevcfb_probe: Failed to allocate framebuffer memory\n");
    return retval;
  }
  
  /*
   * altevcfb must clear memory to prevent kernel info
   */
  memset(videomemory, 0, videomemorysize);
  
  info = framebuffer_alloc(sizeof(u32) * PALETTE_SIZE, &dev->dev);
  if (!info)
    goto err;
  
  info->screen_base = (char __iomem *)videomemory;
  info->fbops = &altevcfb_ops;
  
  /*
   * Configuration based on the selected display 
   */
  info->var = altevcfb_default;
  
  info->var.xres = displayconfig[displayindex].rezx;
  info->var.yres = displayconfig[displayindex].rezy;
  info->var.xres_virtual = displayconfig[displayindex].rezx;
  info->var.yres_virtual = displayconfig[displayindex].rezy;
  
  info->var.pixclock = displayconfig[displayindex].pclk_freq;
  
  info->var.left_margin = displayconfig[displayindex].hs_fp;
  info->var.right_margin = displayconfig[displayindex].hs_bp;
  info->var.hsync_len = displayconfig[displayindex].hs_w;
  info->var.upper_margin = displayconfig[displayindex].vs_fp;
  info->var.lower_margin = displayconfig[displayindex].vs_bp;
  info->var.vsync_len = displayconfig[displayindex].vs_w;
  
  altevcfb_fix.smem_start = (unsigned long) videomemoryphys;
  altevcfb_fix.smem_len = videomemorysize;
  info->fix = altevcfb_fix;
  info->pseudo_palette = info->par;
  info->par = NULL;
  info->flags = FBINFO_FLAG_DEFAULT;
  info->screen_base = videomemory;
  info->fix.line_length = get_line_length(info->var.xres_virtual, info->var.bits_per_pixel);
  
  retval = fb_alloc_cmap(&info->cmap, PALETTE_SIZE, 0);
  if (retval < 0)
    goto err1;
  
  retval = register_framebuffer(info);
  if (retval < 0)
    goto err2;
  platform_set_drvdata(dev, info);
  
  printk(KERN_INFO "*** fb%d: Virtual frame buffer device allocated OK. Size=%ld Vaddr=0x%x Phyaddr=0x%x\n", info->node, videomemorysize, (unsigned int)videomemory, videomemoryphys);
  
  /*
   *  All OK, here we configure and activate the videocontroller
   */
  ret = altevc_setvideomode();
  
  /*
   * Init the backlight dimming subsystem
   */
  altevc_bl_init(info);
  return ret;
  
err2:
  fb_dealloc_cmap(&info->cmap);
err1:
  framebuffer_release(info);
err:
  free_fbmem(&dev->dev, videomemorysize);
  return retval;
}

static int altevcfb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);

	if (info) {
		unregister_framebuffer(info);
		altevc_bl_deinit();
		free_fbmem(&dev->dev, videomemorysize);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}
	return 0;
}

static struct of_device_id altevcfb_match[] = {
	{ .compatible = "exor,altevcfb" },
	{},
};
MODULE_DEVICE_TABLE(of, altevcfb_match);

static struct platform_driver altevcfb_driver = {
	.probe	= altevcfb_probe,
	.remove = altevcfb_remove,
	.driver = {
	.name	= "altevcfb",
	.owner = THIS_MODULE,
	.of_match_table = altevcfb_match,
	},
};

static struct platform_device *altevcfb_device;

static int __init altevcfb_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&altevcfb_driver);
	if (!ret) {
		altevcfb_device = platform_device_alloc("altevcfb", 0);

		if (altevcfb_device)
			ret = platform_device_add(altevcfb_device);
		else
			ret = -ENOMEM;

		if (ret) {
			platform_device_put(altevcfb_device);
			platform_driver_unregister(&altevcfb_driver);
		}
	}
	return ret;
}

module_init(altevcfb_init);

