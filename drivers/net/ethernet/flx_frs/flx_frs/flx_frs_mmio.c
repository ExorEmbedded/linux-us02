/** @file
 */

/*

   FRS Linux driver

   Copyright (C) 2013 Flexibilis Oy

   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License version 2
   as published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include "flx_frs_main.h"
#include "flx_frs_mmio.h"

/**
 * Get pointer to given register for use with memory mapped I/O.
 * @param regs Register access context.
 * @param reg Register number.
 * @return Memory mapped IO address of register.
 */
static inline void __iomem *flx_frs_reg_addr(struct flx_frs_reg_access *regs,
                                             int reg)
{
    // Byte addressing: x2 is needed
    return regs->ioaddr + (reg << 1);
}

/**
 * Read FRS register via MMIO.
 * @param regs Register access context.
 * @param reg Register to read.
 * @return Read value.
 */
static inline int flx_frs_read_reg_mmio(struct flx_frs_reg_access *regs,
                                        int reg)
{
    return ioread16(flx_frs_reg_addr(regs, reg));
}

/**
 * Write FRS register via MMIO.
 * @param regs Register access context.
 * @param reg Register to write.
 * @param value Value to be written.
 */
static inline int flx_frs_write_reg_mmio(struct flx_frs_reg_access *regs,
                                         int reg, uint16_t value)
{
    iowrite16(value, flx_frs_reg_addr(regs, reg));
    return 0;
}

/**
 * Read port register via MMIO.
 * @param pp Port private
 * @param reg Register number.
 * @return Read value.
 */
static int flx_frs_read_port_reg_mmio(struct flx_frs_port_priv *pp,
                                      int reg)
{
    return flx_frs_read_reg_mmio(&pp->regs, reg);
}

/**
 * Write port register via MMIO.
 * @param pp Port private
 * @param reg Register number.
 * @param value Value to be written.
 * @return 0 on success.
 */
static int flx_frs_write_port_reg_mmio(struct flx_frs_port_priv *pp,
                                       int reg, uint16_t value)
{
    return flx_frs_write_reg_mmio(&pp->regs, reg, value);
}

/**
 * Read port adapter register via MMIO.
 * @param pp Port private
 * @param reg Register number.
 * @return Read value.
 */
static int flx_frs_read_adapter_reg_mmio(struct flx_frs_port_priv *pp,
                                         int reg)
{
    struct flx_frs_reg_access *regs = &pp->adapter.regs;

    // Adapters are optional.
    if (!regs->ioaddr)
        return -ENODEV;

    return flx_frs_read_reg_mmio(regs, reg);
}

/**
 * Write port adapter register via MMIO.
 * @param pp Port private
 * @param reg Register number.
 * @param value Value to be written.
 * @return 0 on success.
 */
static int flx_frs_write_adapter_reg_mmio(struct flx_frs_port_priv *pp,
                                          int reg, uint16_t value)
{
    struct flx_frs_reg_access *regs = &pp->adapter.regs;

    // Adapters are optional.
    if (!regs->ioaddr)
        return -ENODEV;

    return flx_frs_write_reg_mmio(regs, reg, value);
}

/**
 * Read switch register via MMIO.
 * @param dp Device private
 * @param reg Register number.
 * @return Read value.
 */
static int flx_frs_read_switch_reg_mmio(struct flx_frs_dev_priv *dp,
                                        int reg)
{
    return flx_frs_read_reg_mmio(&dp->regs, reg);
}

/**
 * Write switch register via MMIO.
 * @param dp Device private
 * @param reg Register number.
 * @param value Value to be written.
 * @return 0 on success.
 */
static int flx_frs_write_switch_reg_mmio(struct flx_frs_dev_priv *dp,
                                         int reg, uint16_t value)
{
    return flx_frs_write_reg_mmio(&dp->regs, reg, value);
}

/**
 * Init MMIO register access.
 * @param dp Device private
 * @param pdev Platform device
 * @return 0 on success.
 */
int __devinit flx_frs_mmio_init_device(struct flx_frs_dev_priv *dp,
                                       struct platform_device *pdev,
                                       struct flx_frs_cfg *frs_cfg)
{
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    int i;
    int ret = 0;

    dev_dbg(dp->this_dev, "Setup device for memory mapped access\n");

    if (!res) {
        dev_err(dp->this_dev, "No I/O memory defined\n");
        ret = -ENXIO;
        goto out;
    }

    dp->regs.ioaddr = ioremap_nocache(res->start, resource_size(res));
    if (!dp->regs.ioaddr) {
        dev_err(dp->this_dev, "ioremap failed for switch address 0x%llx\n",
                (unsigned long long int) res->start);
        ret = -ENXIO;
        goto out;
    }

    dev_printk(KERN_DEBUG, dp->this_dev,
               "Device uses memory mapped access: 0x%llx/0x%llx\n",
               (unsigned long long int) res->start,
               (unsigned long long int) resource_size(res));

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct flx_frs_port_priv *pp = dp->port[i];
        struct flx_frs_port_cfg *port_cfg = &frs_cfg->port[i];
        resource_size_t port_base;
        resource_size_t adapter_base;

        if (!pp)
            continue;

        port_base = port_cfg->baseaddr;
        if (!port_base) {
            dev_warn(dp->this_dev, "No base address for port %i\n",
                     pp->port_num);
            continue;
        }

        pp->regs.ioaddr = ioremap_nocache(port_base, FLX_FRS_PORT_IOSIZE);
        if (!pp->regs.ioaddr) {
            dev_warn(dp->this_dev, "ioremap failed for port %i address 0x%llx\n",
                     pp->port_num, (unsigned long long int) port_base);
            // TODO: fail?
            continue;
        }

        dev_printk(KERN_DEBUG, dp->this_dev,
                   "Port %i uses memory mapped access: 0x%llx/0x%x\n",
                   pp->port_num, (unsigned long long int) port_base,
                   FLX_FRS_PORT_IOSIZE);

        adapter_base = port_cfg->adapter_baseaddr;
        // Adapter is optional.
        if (!adapter_base)
            continue;

        pp->adapter.regs.ioaddr =
            ioremap_nocache(adapter_base, FLX_FRS_ADAPTER_IOSIZE);
        if (!pp->adapter.regs.ioaddr) {
            dev_warn(dp->this_dev,
                     "ioremap failed for port %i adapter address 0x%llx\n",
                     pp->port_num, (unsigned long long int) adapter_base);
            // TODO: fail?
            continue;
        }

        dev_printk(KERN_DEBUG, dp->this_dev,
                   "Port %i adapter uses memory mapped access:"
                   " 0x%llx/0x%x\n",
                   pp->port_num, (unsigned long long int) adapter_base,
                   FLX_FRS_ADAPTER_IOSIZE);
    }

    dp->ops = (struct flx_frs_ops) {
        .read_port_reg = &flx_frs_read_port_reg_mmio,
        .write_port_reg = &flx_frs_write_port_reg_mmio,
        .read_adapter_reg = &flx_frs_read_adapter_reg_mmio,
        .write_adapter_reg = &flx_frs_write_adapter_reg_mmio,
        .read_switch_reg = &flx_frs_read_switch_reg_mmio,
        .write_switch_reg = &flx_frs_write_switch_reg_mmio,
    };

out:
    return ret;
}

/**
 * Cleanup MMIO register access.
 * @param dp Device private
 */
void flx_frs_mmio_cleanup_device(struct flx_frs_dev_priv *dp)
{
    int i;

    dev_dbg(dp->this_dev, "Cleanup device memory mapped access\n");

    dp->ops = (struct flx_frs_ops) {
        .read_port_reg = NULL,
    };

    iounmap(dp->regs.ioaddr);
    dp->regs.ioaddr = NULL;

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct flx_frs_port_priv *pp = dp->port[i];

        if (!pp)
            continue;

        if (pp->adapter.regs.ioaddr) {
            iounmap(pp->adapter.regs.ioaddr);
            pp->adapter.regs.ioaddr = NULL;
        }

        if (pp->regs.ioaddr) {
            iounmap(pp->regs.ioaddr);
            pp->regs.ioaddr = NULL;
        }
    }

    return;
}
