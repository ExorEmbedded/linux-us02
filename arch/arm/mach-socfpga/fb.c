/*
 * File: arch/arm/mach-socfpga/fb.c
 *
 * Ultievc Framebuffer device registration for Altera SOC platforms
 *
 * Copyright (C) 2013 Exor International
 * Author: Giovanni Pavoni <imre.deak@nokia.com>
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
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/memblock.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <asm/mach/map.h>

#if defined(CONFIG_FB_ULTIEVC)


/* 
 * Modify this parameter to adjust to the actual ULTIEVC phy register base address, as seen by the cpu
 */
#define ULTIEVCBASE 0xFF200000

/*
 * This is the only parameter we pass to the fb buffer, in order to univocally identify
 * the connected LCD display and its corresponding parameters (resolution and timings)
 */
static u32 display_id = 45;
static u64 ultievc_fb_dma_mask = ~(u32)0;

static struct resource ultievc_resources[] = {
    [0] = {
        .start  = ULTIEVCBASE, /* Physical */
        .end    = ULTIEVCBASE + 0xfff,
        .flags  = IORESOURCE_MEM,
    },
};

static struct platform_device ultievc_fb_device = {
	.name		= "ultievcfb",
	.id		= -1,
	.dev = {
		.dma_mask		= &ultievc_fb_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &display_id,
	},
    .num_resources  = ARRAY_SIZE(ultievc_resources),
    .resource   = ultievc_resources,
};

int ultievc_init_fb(u32 id)
{
    void __iomem *sdram_ctrl_base_addr;
    /* Enable ports from SDRAM to FPGA*/
    sdram_ctrl_base_addr = ioremap(0xffc25000,4096);
    __raw_writel(0xffff, sdram_ctrl_base_addr + 0x80);
    
    display_id = id;
	return platform_device_register(&ultievc_fb_device);
}

#else

int ultievc_init_fb(u32 id)
{
    return -1;
}

#endif
