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
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <flx_bus/flx_bus.h>

#include "flx_frs_main.h"
#include "flx_frs_indirect.h"

/**
 * Determine bus address for given FRS register.
 * @param regs Register access context.
 * @param reg Register number.
 * @return Bus address for register.
 */
static inline uint32_t flx_frs_bus_addr(struct flx_frs_reg_access *regs,
                                        int reg)
{
    // Byte addressing: x2 is needed
    return regs->addr + (reg << 1);
}

/**
 * Read FRS register via flx_bus.
 * @param regs Register access context.
 * @param reg Register to read.
 * @return Read value.
 */
static inline int flx_frs_read_reg_indirect(struct flx_frs_reg_access *regs,
                                            int reg)
{
    uint16_t value = 0xffff;
    int ret;

    ret = flx_bus_read16(regs->flx_bus, flx_frs_bus_addr(regs, reg), &value);
    if (ret < 0)
        return ret;
    return value;
}

/**
 * Write FRS register via flx_bus.
 * @param regs Register access context.
 * @param reg Register to write.
 * @param value Value to be written.
 */
static inline int flx_frs_write_reg_indirect(struct flx_frs_reg_access *regs,
                                             int reg, uint16_t value)
{
    return flx_bus_write16(regs->flx_bus, flx_frs_bus_addr(regs, reg), value);
}

/**
 * Read port register via flx_bus.
 * @param pp Port private
 * @param reg Register number.
 * @return Read value.
 */
static int flx_frs_read_port_reg_indirect(struct flx_frs_port_priv *pp,
                                          int reg)
{
    return flx_frs_read_reg_indirect(&pp->regs, reg);
}

/**
 * Write port register via flx_bus.
 * @param pp Port private
 * @param reg Register number.
 * @param value Value to be written.
 * @return 0 on success.
 */
static int flx_frs_write_port_reg_indirect(struct flx_frs_port_priv *pp,
                                           int reg, uint16_t value)
{
    return flx_frs_write_reg_indirect(&pp->regs, reg, value);
}

/**
 * Read port adapter register via flx_bus.
 * @param pp Port private
 * @param reg Register number.
 * @return Read value.
 */
static int flx_frs_read_adapter_reg_indirect(struct flx_frs_port_priv *pp,
                                             int reg)
{
    struct flx_frs_reg_access *regs = &pp->adapter.regs;

    // Adapters are optional.
    if (!regs->flx_bus)
        return -ENODEV;

    return flx_frs_read_reg_indirect(regs, reg);
}

/**
 * Write port adapter register via flx_bus.
 * @param pp Port private
 * @param reg Register number.
 * @param value Value to be written.
 * @return 0 on success.
 */
static int flx_frs_write_adapter_reg_indirect(struct flx_frs_port_priv *pp,
                                              int reg, uint16_t value)
{
    struct flx_frs_reg_access *regs = &pp->adapter.regs;

    // Adapters are optional.
    if (!regs->flx_bus)
        return -ENODEV;

    return flx_frs_write_reg_indirect(regs, reg, value);
}

/**
 * Read switch register via flx_bus.
 * @param dp Device private
 * @param reg Register number.
 * @return Read value.
 */
static int flx_frs_read_switch_reg_indirect(struct flx_frs_dev_priv *dp,
                                            int reg)
{
    return flx_frs_read_reg_indirect(&dp->regs, reg);
}

/**
 * Write switch register via flx_bus.
 * @param dp Device private
 * @param reg Register number.
 * @param value Value to be written.
 * @return 0 on success.
 */
static int flx_frs_write_switch_reg_indirect(struct flx_frs_dev_priv *dp,
                                             int reg, uint16_t value)
{
    return flx_frs_write_reg_indirect(&dp->regs, reg, value);
}

/**
 * Initialize indirect register access.
 * @param dp Device private
 * @param pdev Platform device
 * @param frs_cfg FRS config
 * @return 0 on success.
 */
int __devinit flx_frs_indirect_init_device(struct flx_frs_dev_priv *dp,
                                           struct platform_device *pdev,
                                           struct flx_frs_cfg *frs_cfg)
{
    struct resource *res = platform_get_resource(pdev, IORESOURCE_REG, 0);
    int i;

    dev_dbg(dp->this_dev, "Setup device for indirect register access\n");

    if (!res) {
        dev_err(dp->this_dev, "No I/O registers defined\n");
        return -ENXIO;
    }

    dp->regs.addr = res->start;

    dev_printk(KERN_DEBUG, dp->this_dev,
               "Device uses indirect access: 0x%llx/0x%llx\n",
               (unsigned long long int) res->start,
               (unsigned long long int) resource_size(res));

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct flx_frs_port_priv *pp = dp->port[i];
        struct flx_frs_port_cfg *port_cfg = &frs_cfg->port[i];

        if (!pp)
            continue;

        pp->regs.addr = port_cfg->baseaddr;
        if (port_cfg->flags & FLX_FRS_PORT_ADDR_VALID) {
            pp->regs.flx_bus = dp->regs.flx_bus;
            dev_printk(KERN_DEBUG, dp->this_dev,
                       "Port %i uses indirect access: 0x%llx/0x%x\n",
                       pp->port_num,
                       (unsigned long long int) pp->regs.addr,
                       FLX_FRS_PORT_IOSIZE);
        }

        pp->adapter.regs.addr = port_cfg->adapter_baseaddr;
        if (port_cfg->flags & FLX_FRS_ADAPTER_ADDR_VALID) {
            pp->adapter.regs.flx_bus = dp->regs.flx_bus;
            dev_printk(KERN_DEBUG, dp->this_dev,
                       "Port %i adapter uses indirect access: 0x%llx/0x%x\n",
                       pp->port_num,
                       (unsigned long long int) pp->adapter.regs.addr,
                       FLX_FRS_ADAPTER_IOSIZE);
        }
    }

    dp->ops = (struct flx_frs_ops) {
        .read_port_reg = &flx_frs_read_port_reg_indirect,
        .write_port_reg = &flx_frs_write_port_reg_indirect,
        .read_adapter_reg = &flx_frs_read_adapter_reg_indirect,
        .write_adapter_reg = &flx_frs_write_adapter_reg_indirect,
        .read_switch_reg = &flx_frs_read_switch_reg_indirect,
        .write_switch_reg = &flx_frs_write_switch_reg_indirect,
    };

    return 0;
}

/**
 * Cleanup indirect register access.
 * @param dp Device private
 */
void flx_frs_indirect_cleanup_device(struct flx_frs_dev_priv *dp)
{
    int i;

    dev_dbg(dp->this_dev, "Cleanup device indirect register access\n");

    dp->ops = (struct flx_frs_ops) {
        .read_port_reg = NULL,
    };

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct flx_frs_port_priv *pp = dp->port[i];

        if (!pp)
            continue;

        if (!pp->regs.flx_bus)
            continue;

        pp->regs.flx_bus = NULL;
        pp->regs.addr = 0;
    }

    if (dp->regs.flx_bus) {
        flx_bus_put(dp->regs.flx_bus);
        dp->regs.flx_bus = NULL;
    }

    return;
}

