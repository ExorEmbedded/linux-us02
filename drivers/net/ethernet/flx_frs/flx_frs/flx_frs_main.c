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
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/hardirq.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_net.h>
#endif

#ifdef CONFIG_FLX_BUS
# include <flx_bus/flx_bus.h>
#endif

#include "flx_frs_types.h"
#include "flx_frs_proc.h"
#include "flx_frs_if.h"
#include "flx_frs_indirect.h"
#include "flx_frs_mmio.h"
#include "flx_frs_netdev.h"
#include "flx_frs_hw.h"
#include "flx_frs_netdevif.h"
#include "flx_frs_aux_netdev.h"
#include "flx_frs_switchdev.h"
#include "flx_frs_main.h"

// Module description information
MODULE_DESCRIPTION("Flexibilis Ethernet Switch (FES/FRS/FDS/RS) driver");
MODULE_AUTHOR("Flexibilis Oy");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);

// Module parameters
static char *ifacemodes = NULL; ///< list of bitmasks, 0 means switched
module_param(ifacemodes, charp, S_IRUGO);
MODULE_PARM_DESC(ifacemodes,
                 "Comma separated list of switch interface mode bitmasks:"
                 " 0=switched (default), 1=independent");

// of_property_read_string appeared in Linux 3.1
#ifdef CONFIG_OF
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
static int of_property_read_string(struct device_node *np,
                                   const char *propname,
                                   const char **out_string)
{
        struct property *prop = of_find_property(np, propname, NULL);
        if (!prop)
                return -EINVAL;
        if (!prop->value)
                return -ENODATA;
        if (strnlen(prop->value, prop->length) >= prop->length)
                return -EILSEQ;
        *out_string = prop->value;
        return 0;
}
#endif
#endif

// of_property_read_bool appeared in Linux 3.4
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#ifdef CONFIG_OF
static inline bool of_property_read_bool(const struct device_node *np,
                                         const char *propname)
{
    struct property *prop = of_find_property(np, propname, NULL);

    return prop ? true : false;
}
#endif
#endif

// Driver private data
static struct flx_frs_drv_priv flx_frs_drv_priv = {
    .wq = NULL,
};

/**
 * Get access to driver privates.
 */
struct flx_frs_drv_priv *get_drv_priv(void)
{
    return &flx_frs_drv_priv;
}

/*
 * Platform Device init and clean function declaration.
 * These are declared before function body.
 */
static int __devinit flx_frs_device_init(struct platform_device *pdev);
static void __devexit flx_frs_device_cleanup(struct flx_frs_dev_priv *dp);

/*
 * Platform device driver match table.
 */
#ifdef CONFIG_OF
static const struct of_device_id flx_frs_match[] = {
    { .compatible = "flx,fes", },
    { .compatible = "flx,xrs7004-rs", },
    { .compatible = "flx,xrs7003-rs", },
    { .compatible = "flx,xrs3003-rs", },

    // Legacy, for compatibility, may be removed in the future.
    { .compatible = "flx,frs", },
    { .compatible = "flx,fds", },
    { .compatible = "flx,rs", },

    { },
};
#endif

/**
 * Platform Driver definition for Linux core.
 */
static struct platform_driver flx_frs_dev_driver = {
    .driver = {
        .name = "flx_frs",
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = flx_frs_match,
#endif
    },
    .probe = &flx_frs_device_init,
};

/**
 * Write common registers that have inter-device dependencies.
 * The correct value of these registers may depend on the
 * total amount of devices, not known at the moment when
 * a single device is initialized.
 */
static void flx_frs_init_common_registers(void)
{
    int i;
    uint16_t frs_gen;

    for (i = 0; i < ARRAY_SIZE(flx_frs_drv_priv.dev_priv); i++) {
        struct flx_frs_dev_priv *dp = flx_frs_drv_priv.dev_priv[i];
        if (!dp)
            continue;

        dev_dbg(dp->this_dev,
                "Writing trailer bits to general switch register\n");

        frs_gen = flx_frs_read_switch_reg(dp, FRS_REG_GEN);

#ifdef USE_FRS_TIMETRAILER
        if (flx_frs_dev_has_cpu_port(dp)) {
            frs_gen |= FRS_GEN_TIME_TRAILER;
        }
        else {
            frs_gen &= ~FRS_GEN_TIME_TRAILER;
        }
#else
        frs_gen &= ~FRS_GEN_TIME_TRAILER;
#endif

        frs_gen &= ~(FRS_GEN_MGMT_TRAILER_LEN | FRS_GEN_MGMT_TRAILER_OFFSET);

        if (dp->trailer_len > 1) {
            frs_gen |= FRS_GEN_MGMT_TRAILER_LEN;
            if (dp->trailer_offset > 0)
                frs_gen |= FRS_GEN_MGMT_TRAILER_OFFSET;
        }

        flx_frs_write_switch_reg(dp, FRS_REG_GEN, frs_gen);
    }

    return;
}

/**
 * Module init.
 * @return 0 if success.
 */
static int __init flx_frs_init(void)
{
    struct flx_frs_drv_priv *drv = &flx_frs_drv_priv;
    int ret = 0;

    // Make sure that ioctl data definition is valid.
    BUILD_BUG_ON(sizeof(struct frs_ioctl_data) >
                 sizeof(((struct ifreq *)NULL)->ifr_ifru));

    ret = flx_frs_init_crc40(drv);
    if (ret)
        return ret;

    drv->wq = create_singlethread_workqueue(DRV_NAME);
    if (!drv->wq) {
        pr_err(DRV_NAME ": Failed to create work queue\n");
        ret = -ENOMEM;
        goto err_workqueue;
    }

    ret = flx_frs_proc_init_driver();
    if (ret)
        goto err_proc_init;

    ret = platform_driver_register(&flx_frs_dev_driver);
    if (ret)
        goto err_reg_driver;

    flx_frs_init_common_registers();

    ret = flx_frs_aux_init(drv);
    if (ret)
        goto err_aux_init;

    ret = flx_frs_switchdev_init_driver();
    if (ret)
        goto err_init_switchdev;

    return 0;

err_init_switchdev:
    flx_frs_aux_cleanup(drv);

err_aux_init:
err_reg_driver:
    flx_frs_proc_cleanup_driver();

err_proc_init:
    destroy_workqueue(drv->wq);
    drv->wq = NULL;

err_workqueue:
    flx_frs_cleanup_crc40(drv);

    return ret;
}

/**
 * Module exit.
 * Cleanup everything.
 */
static void __exit flx_frs_cleanup(void)
{
    struct flx_frs_drv_priv *drv = &flx_frs_drv_priv;
    int i;

    printk(KERN_DEBUG DRV_NAME ": Module cleanup\n");

    flx_frs_switchdev_cleanup_driver();
    flx_frs_aux_cleanup(drv);

    // Cleanup devices
    for (i = 0; i < ARRAY_SIZE(drv->dev_priv); i++) {
        struct flx_frs_dev_priv *dp = drv->dev_priv[i];

        if (!dp)
            continue;

        flx_frs_device_cleanup(dp);
    }

    flx_frs_proc_cleanup_driver();

    platform_driver_unregister(&flx_frs_dev_driver);

    destroy_workqueue(drv->wq);
    drv->wq = NULL;

    flx_frs_cleanup_crc40(drv);

    pr_debug(DRV_NAME ": %s() done\n", __func__);

    return;
}

// Module init and exit function
module_init(flx_frs_init);
module_exit(flx_frs_cleanup);

/**
 * Init device private data.
 * @param dp Device private data
 * @param id Device id
 */
static void __devinit init_dev_privates(struct flx_frs_dev_priv *dp, int id)
{
    int i;

    dp->drv = &flx_frs_drv_priv;
    dp->dev_num = id;
    dp->dev_num_with_cpu = FLX_FRS_MAX_DEVICES;
    flx_frs_drv_priv.dev_priv[id] = dp;

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        dp->port[i] = NULL;
    }

    spin_lock_init(&dp->link_mask_lock);
    mutex_init(&dp->common_reg_lock);
    mutex_init(&dp->smac_table_lock);

    INIT_LIST_HEAD(&dp->aux_netdev_list);

    return;
}

#ifdef CONFIG_OF

/**
 * Table of compatible strings and associated predefined FES features.
 */
static const struct {
    const char *compat;                 ///< compatible string
    struct flx_frs_features features;   ///< associated features
} flx_frs_compatible_features[] = {
    { "flx,xrs7004-rs", {
                            .flags =
                                FLX_FRS_FEAT_AUTO |
                                FLX_FRS_FEAT_GIGABIT |
                                FLX_FRS_FEAT_STATS |
                                FLX_FRS_FEAT_VLAN |
                                FLX_FRS_FEAT_MAC_TABLE,
                            .prio_queues = 4,
                            .hsr_ports = 0xe,
                            .prp_ports = 0xe,
                        },
    },
    { "flx,xrs7003-rs", {
                            .flags =
                                FLX_FRS_FEAT_AUTO |
                                FLX_FRS_FEAT_GIGABIT |
                                FLX_FRS_FEAT_STATS |
                                FLX_FRS_FEAT_VLAN |
                                FLX_FRS_FEAT_MAC_TABLE,
                            .prio_queues = 4,
                            .hsr_ports = 0x6,
                            .prp_ports = 0x6,
                        },
    },
    { "flx,xrs3003-rs", {
                            .flags =
                                FLX_FRS_FEAT_AUTO |
                                FLX_FRS_FEAT_GIGABIT,
                            .prio_queues = 4,
                            .hsr_ports = 0x6,
                            .prp_ports = 0x6,
                        },
    },
};

/**
 * Set features from predefined table based on compatible string.
 * @param dp Device privates.
 * @return Zero on success, or negative error code.
 */
static int flx_frs_of_get_compatible_features(struct flx_frs_dev_priv *dp)
{
    struct device_node *node = dp->this_dev->of_node;
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(flx_frs_compatible_features); i++) {
        if (of_device_is_compatible(node,
                                    flx_frs_compatible_features[i].compat)) {
            dp->features = flx_frs_compatible_features[i].features;
            return 0;
        }
    }

    return -ENOENT;
}

/**
 * Get switch features from device tree.
 * @param dp Device privates whose features to set.
 */
static int flx_frs_of_get_features(struct flx_frs_dev_priv *dp)
{
    struct device_node *frs_node = dp->this_dev->of_node;
    struct device_node *node = NULL;
    uint32_t value = 0;
    int ret = -ENOENT;

    // Older kernels do not have of_get_child_by_name.
    for_each_child_of_node(frs_node, node) {
        if (strcmp("features", node->name) == 0)
            break;
    }

    if (!node) {
        // features node is missing, try to set based on compatible string.
        ret = flx_frs_of_get_compatible_features(dp);
        if (ret == 0)
            return 0;

        return -ENOENT;
    }

    if (of_property_read_u32(node, "clock-frequency", &value) == 0)
        dp->features.clock_freq = value;
    else
        dp->features.clock_freq = 0;

    dp->features.flags = 0;
    if (of_property_read_bool(node, "device-id"))
        dp->features.flags |= FLX_FRS_FEAT_DEV_ID;
    if (of_property_read_bool(node, "gigabit"))
        dp->features.flags |= FLX_FRS_FEAT_GIGABIT;
    if (of_property_read_bool(node, "statistics-counters"))
        dp->features.flags |= FLX_FRS_FEAT_STATS;
    if (of_property_read_bool(node, "mac-address-table"))
        dp->features.flags |= FLX_FRS_FEAT_MAC_TABLE;
    if (of_property_read_bool(node, "vlan"))
        dp->features.flags |= FLX_FRS_FEAT_VLAN;
    if (of_property_read_bool(node, "traffic-shaper"))
        dp->features.flags |= FLX_FRS_FEAT_SHAPER;

    if (of_property_read_u32(node, "priority-queues", &value) == 0)
        dp->features.prio_queues = value;
    else
        dp->features.prio_queues = 0;
    if (of_property_read_u32(node, "traffic-policers", &value) == 0)
        dp->features.policers = value;
    else
        dp->features.policers = 0;

    if (of_property_read_u32(node,
                             "static-mac-address-table-rows",
                             &value) == 0)
        dp->features.smac_rows = value;
    else
        dp->features.smac_rows = 0;
    if (of_property_read_bool(node, "static-mac-address-table-hash"))
        dp->features.flags |= FLX_FRS_FEAT_SMAC_HASH;

    if (of_property_read_u32(node, "hsr-ports", &value) == 0)
        dp->features.hsr_ports = value;
    else
        dp->features.hsr_ports = 0;
    if (of_property_read_u32(node, "prp-ports", &value) == 0)
        dp->features.prp_ports = value;
    else
        dp->features.prp_ports = 0;
    if (of_property_read_u32(node, "macsec-ports", &value) == 0)
        dp->features.macsec_ports = value;
    else
        dp->features.macsec_ports = 0;
    if (of_property_read_u32(node, "scheduled-ports", &value) == 0)
        dp->features.sched_ports = value;
    else
        dp->features.sched_ports = 0;

    return 0;
}

#endif

/**
 * Sanitize switch features.
 * @param dp FRS device privates.
 */
static void flx_frs_sanitize_features(struct flx_frs_dev_priv *dp)
{
    unsigned int column;

    if (dp->features.prio_queues > FLX_FRS_MAX_PRIO_QUEUES)
        dp->features.prio_queues = FLX_FRS_MAX_PRIO_QUEUES;
    if (dp->features.prio_queues == 0)
        dp->features.flags &= ~FLX_FRS_FEAT_SHAPER;

    if (dp->features.policers > FLX_FRS_MAX_POLICERS)
        dp->features.policers = FLX_FRS_MAX_POLICERS;

    if (dp->features.smac_rows == 0)
        dp->features.flags &= ~FLX_FRS_FEAT_SMAC_HASH;
    else if (dp->features.smac_rows > FRS_SMAC_TABLE_MAX_ROWS)
        dp->features.smac_rows = FRS_SMAC_TABLE_MAX_ROWS;
    for (column = 0; column < FRS_SMAC_TABLE_COLS; column++) {
        if (dp->features.flags & FLX_FRS_FEAT_SMAC_HASH)
            dp->smac.cfg.row_sel[column] = FLX_FRS_SMAC_ROW_SEL_NO_VLAN;
        else
            dp->smac.cfg.row_sel[column] = FLX_FRS_SMAC_ROW_SEL_ADDR;
    }

    return;
}

/**
 * Configure FRS device.
 * @param dp FRS device privates.
 * @param pdev FRS platform_device.
 * @param frs_cfg Temporary storage for building FRS config.
 * @return Pointer for acquired FRS config, or NULL.
 */
static struct flx_frs_cfg *flx_frs_device_config(struct flx_frs_dev_priv *dp,
                                                 struct platform_device *pdev,
                                                 struct flx_frs_cfg *frs_cfg)
{
    struct flx_frs_cfg *frs_cfg_pdata = dev_get_platdata(&pdev->dev);

    if (frs_cfg_pdata) {
        dev_printk(KERN_DEBUG, dp->this_dev, "Config via platform_data\n");
        return frs_cfg_pdata;
    }

    if (!frs_cfg_pdata) {
#ifdef CONFIG_OF
        bool indirect = false;
        struct resource *res = NULL;
        struct device *dev = &pdev->dev;
        struct device_node *child_n = NULL;
        const __be32 *reg = NULL;
        int length = 0;
        const char *svalue = NULL;
        uint32_t ivalue = 0;
        struct resource *irq_res =
            platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        int ret = 0;

        if (!irq_res) {
            // Allow operation without IRQ.
            dev_warn(dp->this_dev, "No IRQ defined\n");
            dp->irq = 0;
        }
        else {
            dp->irq = irq_res->start;
        }

        ret = flx_frs_of_get_features(dp);
        if (ret == -ENOENT) {
            dev_warn(dev, "Missing features in device tree\n");
        }

        // Underlying Ethernet MAC name
        if (of_property_read_string(dev->of_node, "mac_name", &svalue)) {
            dev_printk(KERN_DEBUG, dp->this_dev, "unable to get MAC name\n");
        } else {
            frs_cfg->mac_name = svalue;
        }

        // Register access
#ifdef CONFIG_FLX_BUS
        dp->regs.flx_bus = of_flx_bus_get_by_device(dev->of_node);
        if (dp->regs.flx_bus) {
            indirect = true;
            // Registers accessed indirectly
            res = platform_get_resource(pdev, IORESOURCE_REG, 0);
            if (!res) {
                dev_err(dev, "No I/O registers defined\n");
                flx_bus_put(dp->regs.flx_bus);
                dp->regs.flx_bus = NULL;
                return NULL;
            }
            frs_cfg->baseaddr = res->start;
        }
#endif
        if (!indirect) {
            // Memory mapped registers
            res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
            if (!res) {
                dev_err(dev, "No I/O memory defined\n");
                return NULL;
            }
            else {
                frs_cfg->baseaddr = res->start;
            }
        }

        // Config all ports
        frs_cfg->num_of_ports = 0;
        for_each_child_of_node(dev->of_node, child_n) {
            struct flx_frs_port_cfg *port_cfg = NULL;
            struct flx_frs_port_priv *pp = NULL;
            int port_num = -1;
            int phy_mode = -1;

            if (strncmp("port", child_n->name, 4) != 0) {
                continue;
            }

            if (kstrtoint(child_n->name + 4, 10, &port_num) ||
                port_num < 0 || port_num >= ARRAY_SIZE(frs_cfg->port)) {
                dev_warn(dp->this_dev, "Invalid port %s\n",
                         child_n->name);
                continue;
            }

            port_cfg = &frs_cfg->port[port_num];
            pp = dp->port[port_num];

            if (port_cfg->medium_type != FLX_FRS_MEDIUM_NONE) {
                dev_warn(dp->this_dev, "Already defined port %i\n",
                         port_num);
                continue;
            }

            dev_dbg(dp->this_dev, "Reading node %s index %i port %i\n",
                       child_n->name, port_num, port_num);

            // Interface name for port
            if (of_property_read_string(child_n, "if_name", &svalue)) {
                dev_printk(KERN_DEBUG, dp->this_dev,
                           "unable to get interface name\n");
            } else {
                port_cfg->if_name = svalue;
            }

            // Port medium type
            if (of_property_read_u32(child_n, "medium_type", &ivalue)) {
                dev_err(dp->this_dev, "unable to get port %i medium type\n",
                        port_num);
                if (port_num == 0) {
                    port_cfg->medium_type = FLX_FRS_MEDIUM_NOPHY;
                    port_cfg->flags |= FLX_FRS_PORT_CPU;
                } else {
                    port_cfg->medium_type = FLX_FRS_MEDIUM_PHY;
                }
            } else {
                switch (ivalue) {
                case FLX_FRS_MEDIUM_NOPHY:
                    port_cfg->medium_type = FLX_FRS_MEDIUM_NOPHY;
                    break;
                case FLX_FRS_MEDIUM_NONE:
                    port_cfg->medium_type = FLX_FRS_MEDIUM_NONE;
                    break;
                case FLX_FRS_MEDIUM_PHY:
                    port_cfg->medium_type = FLX_FRS_MEDIUM_PHY;
                    break;
                case FLX_FRS_MEDIUM_SFP:
                    port_cfg->medium_type = FLX_FRS_MEDIUM_SFP;
                    break;
                }
            }
            if (of_property_read_bool(child_n, "cpu-port")) {
                port_cfg->flags |= FLX_FRS_PORT_CPU;
            }
            if (of_property_read_bool(child_n, "interlink-port")) {
                port_cfg->flags |= FLX_FRS_PORT_IE;
            }
            if (of_property_read_bool(child_n, "auto-speed-select")) {
                port_cfg->flags |= FLX_FRS_PORT_SPEED_EXT;
            }

            // Port and port adapter register addresses
            reg = of_get_property(child_n, "reg", &length);
            if (!reg ||
                (length != 2*sizeof(uint32_t) &&
                 length != 4*sizeof(uint32_t))) {
                dev_printk(KERN_DEBUG, dp->this_dev,
                           "unable to get port %i"
                           " memory mapped I/O addresses\n",
                           port_num);
            } else {
                port_cfg->baseaddr = be32_to_cpu(reg[0]);
                if (length == 16) {
                    port_cfg->adapter_baseaddr = be32_to_cpu(reg[2]);
                    port_cfg->flags |=
                        FLX_FRS_PORT_ADDR_VALID |
                        FLX_FRS_ADAPTER_ADDR_VALID;
                }
                else {
                    port_cfg->flags |= FLX_FRS_PORT_ADDR_VALID;
                }
            }

            // Ext PHY handle (not required with all medium types).
            // Try phy-id if phy-handle is missing.
            if (port_cfg->medium_type == FLX_FRS_MEDIUM_SFP ||
                port_cfg->medium_type == FLX_FRS_MEDIUM_PHY) {
                port_cfg->ext_phy_node = of_parse_phandle(child_n,
                                                          "phy-handle", 0);
                if (!port_cfg->ext_phy_node) {
                    ret = of_property_read_string(child_n,
                                                  "phy-id",
                                                  &port_cfg->ext_phy_id);
                    if (ret) {
                        dev_printk(KERN_DEBUG, dp->this_dev,
                                   "unable to get FRS port %i external PHY\n",
                                   port_num);
                    }
                }
            }

            // SGMII mode
            if (of_property_read_bool(child_n, "sgmii-phy-mode")) {
                port_cfg->flags |= FLX_FRS_ADAPTER_SGMII_PHY_MODE;
                dev_printk(KERN_DEBUG, dp->this_dev,
                           "port %i SGMII %s mode\n",
                           port_num, "PHY");
            }

            // Ext PHY interface type
            phy_mode = of_get_phy_mode(child_n);
            if (phy_mode < 0)
                port_cfg->ext_phy_if = PHY_INTERFACE_MODE_NA;
            else
                port_cfg->ext_phy_if = phy_mode;

            // SFP EEPROM for SFP type detection
            if (port_cfg->medium_type == FLX_FRS_MEDIUM_SFP) {
                port_cfg->sfp_eeprom_node = of_parse_phandle(child_n,
                                                             "sfp-eeprom", 0);
                if (port_cfg->sfp_eeprom_node) {
                    port_cfg->flags |= FLX_FRS_SFP_EEPROM;
                }
                else {
                    ret = of_property_read_string(child_n, "sfp-eeprom-id",
                                                  &port_cfg->sfp_eeprom_name);
                    if (ret == 0) {
                        port_cfg->flags |= FLX_FRS_SFP_EEPROM;
                    }
                    else {
                        dev_printk(KERN_DEBUG, dp->this_dev,
                                   "unable to get FRS port %i SFP EEPROM\n",
                                   port_num);
                    }
                }
            }

            // SFP PHY handle (not required with all medium types).
            // Try phy-id if sfp-phy-handle is missing.
            if (port_cfg->medium_type == FLX_FRS_MEDIUM_SFP) {
                // Always use SGMII with SFP PHY.
                port_cfg->sfp_phy_if = PHY_INTERFACE_MODE_SGMII;
                port_cfg->sfp_phy_node = of_parse_phandle(child_n,
                                                          "sfp-phy-handle", 0);
                if (!port_cfg->sfp_phy_node) {
                    ret = of_property_read_string(child_n,
                                                  "sfp-phy-id",
                                                  &port_cfg->sfp_phy_id);
                    if (ret) {
                        dev_printk(KERN_DEBUG, dp->this_dev,
                                   "unable to get FRS port %i SFP PHY\n",
                                   port_num);
                    }
                }
            }

            if (port_cfg->ext_phy_node || port_cfg->ext_phy_id)
                port_cfg->flags |= FLX_FRS_HAS_PHY;
            if (port_cfg->sfp_phy_node || port_cfg->sfp_phy_id)
                port_cfg->flags |= FLX_FRS_HAS_SFP_PHY;

            if ((port_cfg->flags & FLX_FRS_HAS_PHY) &&
                (port_cfg->flags & FLX_FRS_HAS_SFP_PHY))
                port_cfg->flags |= FLX_FRS_HAS_SEPARATE_SFP;

            // Compatibility: Treat sole PHY like SFP PHY.
            if (port_cfg->medium_type == FLX_FRS_MEDIUM_SFP &&
                (port_cfg->flags & FLX_FRS_HAS_PHY) &&
                !(port_cfg->flags & FLX_FRS_HAS_SFP_PHY)) {

                port_cfg->sfp_phy_node = port_cfg->ext_phy_node;
                port_cfg->sfp_phy_id = port_cfg->ext_phy_id;
                port_cfg->sfp_phy_if = port_cfg->ext_phy_if;

                port_cfg->ext_phy_node = NULL;
                port_cfg->ext_phy_id = NULL;

                port_cfg->flags &= ~FLX_FRS_HAS_PHY;
                port_cfg->flags |= FLX_FRS_HAS_SFP_PHY;
            }

            if (port_num + 1 > frs_cfg->num_of_ports)
                frs_cfg->num_of_ports = port_num + 1;
        }

#else
        dev_printk(KERN_DEBUG, dp->this_dev, "No platform_data\n");
        return NULL;
#endif
    } else {
        dev_printk(KERN_DEBUG, dp->this_dev, "Config via platform_data\n");
        // Config provided via platform_data.
        frs_cfg = frs_cfg_pdata;
    }

    flx_frs_sanitize_features(dp);

    return frs_cfg;
}

/**
 * Initialize FRS dev config using FRS config.
 * @param dp FRS device privates.
 * @param frs_cfg FRS config.
 */
static int flx_frs_init_ports(struct flx_frs_dev_priv *dp,
                              struct flx_frs_cfg *frs_cfg)
{
    int i = 0;
    struct flx_frs_port_priv *pp = NULL;
    struct flx_frs_port_cfg *port_cfg = NULL;

    for (i = 0; i < frs_cfg->num_of_ports; i++) {
        port_cfg = &frs_cfg->port[i];

        // Ignore unconfigured ports.
        if (!port_cfg->if_name)
            continue;

        // Port addresses are required.
        if (!(port_cfg->flags & FLX_FRS_PORT_ADDR_VALID))
            continue;

        pp = kzalloc(sizeof(*pp), GFP_KERNEL);
        if (!pp) {
            dev_err(dp->this_dev, "kmalloc failed\n");
            goto fail;
        }

        dp->port[i] = pp;
        dp->port[i]->dp = dp;
        strncpy(pp->if_name, port_cfg->if_name, IFNAMSIZ);
        pp->port_num = i;
        pp->medium_type = port_cfg->medium_type;
        pp->flags = port_cfg->flags;
        // Port mask may be adjusted later, when all FRS devices are known.
        pp->port_mask = 1u << pp->port_num;
        if (pp->flags & FLX_FRS_PORT_CPU) {
            dp->cpu_port_mask |= pp->port_mask;
            pp->port_mask = 0;
        }
#ifdef CONFIG_OF
        pp->ext_phy.node = port_cfg->ext_phy_node;
        pp->sfp.phy.node = port_cfg->sfp_phy_node;
        pp->sfp.eeprom_node = port_cfg->sfp_eeprom_node;
#endif
        pp->ext_phy.interface = port_cfg->ext_phy_if;
        if (port_cfg->ext_phy_id) {
            pp->ext_phy.bus_id = kstrdup(port_cfg->ext_phy_id, GFP_KERNEL);
        }
        if (port_cfg->sfp_phy_id) {
            pp->sfp.phy.bus_id = kstrdup(port_cfg->sfp_phy_id, GFP_KERNEL);
        }
        if (port_cfg->sfp_eeprom_name) {
            pp->sfp.eeprom_name = kstrdup(port_cfg->sfp_eeprom_name,
                                          GFP_KERNEL);
        }
        pp->sfp.supported = FLX_FRS_ETHTOOL_SUPPORTED;
        pp->sfp.phy.interface = port_cfg->sfp_phy_if;

        pp->rx_delay = 0;
        pp->tx_delay = 0;
        pp->p2p_delay = 0;

        mutex_init(&pp->stats_lock);
        mutex_init(&pp->port_reg_lock);
    }

    strncpy(dp->mac_name, frs_cfg->mac_name, IFNAMSIZ);

    dp->num_of_ports = frs_cfg->num_of_ports;
    // Trailer length may be adjusted later, when all FRS devices are known.
    dp->trailer_len = (dp->num_of_ports + 7) / 8;

    return 0;

  fail:
    for (i = 0; i < frs_cfg->num_of_ports; i++) {
        if (dp->port[i]) {
            if (dp->port[i]->ext_phy.bus_id) {
                kfree(dp->port[i]->ext_phy.bus_id);
            }
            if (dp->port[i]->sfp.phy.bus_id) {
                kfree(dp->port[i]->sfp.phy.bus_id);
            }
            if (dp->port[i]->sfp.eeprom_name) {
                kfree(dp->port[i]->sfp.eeprom_name);
            }
            kfree(dp->port[i]);
            dp->port[i] = NULL;
        }
    }

    return -ENOMEM;
}

/**
 * Init FRS IP. Run SW reset and init default settings.
 * @param dp FRS device privates.
 */
static int flx_frs_init_registers(struct flx_frs_dev_priv *dp)
{
    unsigned int timeout = 100;
    unsigned int column;
    int data;

    // SW Reset
    data = flx_frs_write_switch_reg(dp, FRS_REG_GEN, FRS_GEN_RESET);
    if (data < 0) {
        dev_err(dp->this_dev, "FRS SW reset failed: write I/O error\n");
        return data;
    }

    // Wait reset to complete
    do {
        if (timeout-- == 0) {
            dev_err(dp->this_dev, "FRS SW reset failed: timeout\n");
            return -EBUSY;
        }
        msleep(1);

        data = flx_frs_read_switch_reg(dp, FRS_REG_GEN);
        if (data < 0) {
            dev_err(dp->this_dev, "FRS SW reset failed: read I/O error\n");
            return data;
        }
    } while ((data & FRS_GEN_RESET) != 0);

    dev_printk(KERN_DEBUG, dp->this_dev, "FRS SW reset done\n");

    if (flx_frs_dev_has_cpu_port(dp)) {
        // Init FRS timestamper
        flx_frs_write_switch_reg(dp, FRS_REG_TS_CTRL_RX, 0xf);
#ifndef USE_FRS_TIMETRAILER
        flx_frs_write_switch_reg(dp, FRS_REG_TS_CTRL_TX, 0xf);
        dp->tx_stamper_index = 0;
#endif
        dp->rx_stamper_index = 0;

        // Enable FRS interrupts.
        flx_frs_write_switch_reg(dp, FRS_REG_INTMASK,
                                 FRS_INT_RX_TSTAMP | FRS_INT_TX_TSTAMP);
    }
    else {
        // Disable FRS interrupts.
        flx_frs_write_switch_reg(dp, FRS_REG_INTMASK, 0);
    }

    // Read SMAC configuration.
    data = flx_frs_read_switch_reg(dp, FRS_REG_SMAC_TABLE);
    if (data < 0)
        return data;
    for (column = 0; column < FRS_SMAC_TABLE_COLS; column++) {
        if (dp->features.flags & FLX_FRS_FEAT_SMAC_HASH) {
            switch (FRS_SMAC_TABLE_TO_ROW_SEL(column, data)) {
            case FRS_SMAC_TABLE_XOR_VLAN_DISABLE:
                dp->smac.cfg.row_sel[column] = FLX_FRS_SMAC_ROW_SEL_NO_VLAN;
                break;
            case FRS_SMAC_TABLE_XOR_VLAN_ENABLE:
                dp->smac.cfg.row_sel[column] = FLX_FRS_SMAC_ROW_SEL_VLAN;
                break;
            default:
                dev_warn(dp->this_dev, "Unknown FRS SMAC table config 0x%x, "
                         "disabling SMAC support\n",
                         data);
                // Disable SMAC support.
                dp->features.smac_rows = 0;
                dp->features.flags &= ~FLX_FRS_FEAT_SMAC_HASH;
                column = FRS_SMAC_TABLE_COLS;
            }
        }
        else {
            dp->smac.cfg.row_sel[column] = FLX_FRS_SMAC_ROW_SEL_ADDR;
        }
    }

    return 0;
}

/**
 * Setup FRS register access.
 * FRS switch registers and FRS port registers can be accessed either
 * through MDIO or memory mapped I/O.
 */
static int __devinit flx_frs_reg_access_init_device(
        struct flx_frs_dev_priv *dp,
        struct platform_device *pdev,
        struct flx_frs_cfg *frs_cfg)
{
    int ret = 0;
    bool indirect = false;

    // Setup FRS register access.

#ifdef CONFIG_FLX_BUS
    // Check for indirect register access.
#ifdef CONFIG_OF
    if (dp->regs.flx_bus) {
        indirect = true;
        ret = flx_frs_indirect_init_device(dp, pdev, frs_cfg);
    }
#else
    if (frs_cfg->flx_bus_name) {
        // TODO: Currently only possible via device tree
        dev_err(&pdev->dev,
                "Currently indirect register access requires device tree\n");
        return -EINVAL;
    }
#endif
#endif

    if (!indirect) {
        ret = flx_frs_mmio_init_device(dp, pdev, frs_cfg);
    }

    return ret;
}

/**
 * Cleanup FRS register access.
 */
static int flx_frs_reg_access_cleanup_device(struct flx_frs_dev_priv *dp)
{
    int ret = 0;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

#ifdef CONFIG_FLX_BUS
    if (dp->regs.flx_bus) {
        flx_frs_indirect_cleanup_device(dp);
    }
    else {
        flx_frs_mmio_cleanup_device(dp);
    }
#else
    flx_frs_mmio_cleanup_device(dp);
#endif

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return ret;
}

/**
 * Get bitmask of independent FRS ports (no management trailer offset).
 * Parses ifacemodes module parameter.
 */
static unsigned int flx_frs_ifacemodes(struct flx_frs_dev_priv *dp)
{
    unsigned int dp_ifacemodes = 0;
    unsigned int i;
    char tmp_ifacemodes[FLX_FRS_MAX_DEVICES * 8];
    char *next = tmp_ifacemodes;
    char *start = NULL;
    int ret;

    if (!ifacemodes || strlen(ifacemodes) >= sizeof(tmp_ifacemodes))
        return 0;

    // strsep modifies the string, use local copy.
    strcpy(tmp_ifacemodes, ifacemodes);

    for (i = 0; i <= dp->dev_num; i++) {
        start = strsep(&next, ",");
        if (!start)
            return 0;
    }

    ret = kstrtouint(start, 0, &dp_ifacemodes);
    if (ret < 0)
        return 0;

    return dp_ifacemodes;
}

/**
 * Sets interface mode to switched or independent according to
 * module parameter.
 */
static void flx_frs_ifacemode_fixup(struct flx_frs_dev_priv *dp)
{
    struct flx_frs_drv_priv *drv = &flx_frs_drv_priv;
    struct flx_frs_dev_priv *dp_tmp = NULL;
    struct flx_frs_port_priv *port = NULL;
    unsigned int dp_ifacemodes = flx_frs_ifacemodes(dp);
    unsigned int i;
    unsigned int j;

    dev_printk(KERN_DEBUG, dp->this_dev,
               "Ifacemodes: 0x%x\n", dp_ifacemodes);

    if (dp_ifacemodes == 0)
        return;

    for (i = 0; i < ARRAY_SIZE(drv->dev_priv); i++) {

        dp_tmp = drv->dev_priv[i];
        if (!dp_tmp)
            continue;

        for (j = 0; j < ARRAY_SIZE(dp_tmp->port); j++) {

            port = dp_tmp->port[j];
            if (!port)
                continue;

            // Don't allow CPU port to be set independent.
            if ((1u << port->port_num) == dp_tmp->cpu_port_mask) {
                continue;
            }

            // If port number matches ifacemodes param, set iface independent.
            // In case of switch port, don't do anything. It is the default.
            if (dp_ifacemodes & (1u << port->port_num)) {
                flx_frs_set_ifacemode(port, FLX_FRS_IFACE_INDEPENDENT);
            }

        }

    }

    return;
}

/**
 * Sets interface mode. Modifies port masks of all ports
 * according to given port mode.
 * @param port Port privates
 * @param mode Port mode
 */
void flx_frs_set_ifacemode(struct flx_frs_port_priv *port,
                           enum flx_frs_iface_mode mode)
{
    struct flx_frs_drv_priv *drv = &flx_frs_drv_priv;
    struct flx_frs_dev_priv *dp_tmp = NULL;
    struct flx_frs_port_priv *port_tmp = NULL;
    unsigned int i;
    unsigned int j;
    uint16_t portmask_data;

    dev_printk(KERN_DEBUG, port->dp->this_dev,
               "  Ifacemode for port %u (mask 0x%x) to %s\n",
               port->port_num, port->port_mask,
               mode == FLX_FRS_IFACE_INDEPENDENT ? "INDEPENDENT" : "SWITCH");

    if (mode == FLX_FRS_IFACE_INDEPENDENT)
        port->flags |= FLX_FRS_PORT_INDEPENDENT;
    else
        port->flags &= ~FLX_FRS_PORT_INDEPENDENT;

    for (i = 0; i < ARRAY_SIZE(drv->dev_priv); i++) {

        dp_tmp = drv->dev_priv[i];
        if (!dp_tmp)
            continue;

        // Loop through all ports and set port forward masks accordingly
        for (j = 0; j < ARRAY_SIZE(dp_tmp->port); j++) {

            port_tmp = dp_tmp->port[j];
            if (!port_tmp)
                continue;

            // Don't touch CPU port forward mask
            if (port_tmp->flags & FLX_FRS_PORT_CPU) {
                dev_dbg(port->dp->this_dev,
                        "    Don't set CPU port %u PORT_FWD_MASK",
                        port_tmp->port_num);
                continue;
            }

            mutex_lock(&port_tmp->port_reg_lock);

            // In case of this port, set to only forward to CPU port
            if (port_tmp == port) {
                portmask_data = ~dp_tmp->cpu_port_mask;
                dev_printk(KERN_DEBUG, port->dp->this_dev,
                           "    Set port %u PORT_FWD_MASK to 0x%x",
                           port_tmp->port_num,
                           portmask_data);
                flx_frs_write_port_reg(port_tmp, PORT_REG_FWD_PORT_MASK,
                                       portmask_data);
                mutex_unlock(&port_tmp->port_reg_lock);
                continue;
            }

            // For other ports, remove this independent port
            // from their forward masks
            portmask_data = flx_frs_read_port_reg(port_tmp,
                                                  PORT_REG_FWD_PORT_MASK);

            switch (mode) {
            case FLX_FRS_IFACE_SWITCH:
                if (!(portmask_data | port->port_mask)) {
                    // No write needed.
                    mutex_unlock(&port_tmp->port_reg_lock);
                    continue;
                }
                portmask_data &= ~port->port_mask;
            case FLX_FRS_IFACE_INDEPENDENT:
                if (portmask_data & port->port_mask) {
                    // No write needed.
                    mutex_unlock(&port_tmp->port_reg_lock);
                    continue;
                }
                portmask_data |= port->port_mask;
            }
            dev_printk(KERN_DEBUG, port->dp->this_dev,
                       "    Set port %u PORT_FWD_MASK to 0x%x",
                       port_tmp->port_num,
                       portmask_data);
            flx_frs_write_port_reg(port_tmp, PORT_REG_FWD_PORT_MASK,
                                   portmask_data);
            mutex_unlock(&port_tmp->port_reg_lock);

        }
    }

    return;
}

/**
 * Sets trailer length and offset of linked devices upon initialization.
 * General register values need to be written after all devices
 * are initialized. Will not write to registers.
 * @param dp FRS device privates
 */
static void flx_frs_trailer_fixup(struct flx_frs_dev_priv *dp)
{
    struct flx_frs_drv_priv *drv = &flx_frs_drv_priv;
    struct flx_frs_dev_priv *dp_tmp = NULL;
    unsigned int i;
    unsigned int j;

    dp->trailer_offset = 0;

    // Fix trailer length and offset of linked devices.
    for (i = 0; i < ARRAY_SIZE(drv->dev_priv); i++) {
        struct flx_frs_dev_priv *dp_cpu = NULL;
        struct flx_frs_dev_priv *dp_linked = NULL;
        struct flx_frs_port_priv *port = NULL;
        uint16_t linked_port_mask = 0;

        dp_tmp = drv->dev_priv[i];
        if (!dp_tmp || dp_tmp == dp)
            continue;

        // Linked FRS devices use the same MAC.
        if (strcmp(dp->mac_name, dp_tmp->mac_name))
            continue;

        // Linked FRS device found.
        dp->trailer_len = 2;
        dp_tmp->trailer_len = 2;
        if (flx_frs_dev_has_cpu_port(dp)) {
            dp_cpu = dp;
            dp_linked = dp_tmp;
        }
        else {
            dp_cpu = dp_tmp;
            dp_linked = dp;
        }

        dp_linked->trailer_offset = 8;
        dp_linked->dev_num_with_cpu = dp_cpu->dev_num;
        dp_cpu->dev_num_with_cpu = FLX_FRS_MAX_DEVICES;

        // Detect port mask for sending through linked FRS port.
        // FRS with CPU port must be told to send only through linked port.
        for (j = 0; j < ARRAY_SIZE(dp_cpu->port); j++) {
            port = dp_cpu->port[j];
            if (!port)
                continue;

            if (port->flags & FLX_FRS_PORT_IE) {
                // This port mask needs to be ORed for linked FRS ports.
                linked_port_mask |=
                    1u << (port->port_num + dp_cpu->trailer_offset);
                /*
                 * When sending to link port of FRS with CPU port
                 * send to all ports of linked FRS
                 * (with that trailer part zero).
                 * That happens with the default port mask,
                 * if not changed here.
                 */
            }
        }

        // Adjust port masks.
        for (j = 0; j < ARRAY_SIZE(dp_linked->port); j++) {
            struct flx_frs_port_priv *port = dp_linked->port[j];
            if (!port)
                continue;

            port->port_mask <<= dp_linked->trailer_offset;
            if (port->flags & FLX_FRS_PORT_IE) {
                /*
                 * When sending to linked FRS link port
                 * send to the other FRS (with CPU) ports only,
                 * not to linked FRS at all, and not back to CPU.
                 * Unfortunately there is no way to use trailer part zero
                 * to prevent sending to linked FRS, thus such frames
                 * cannot be managed by IPOs.
                 */
                port->port_mask = ~(dp_cpu->cpu_port_mask | linked_port_mask);
                /*
                 * It is also very important to avoid loops
                 * that would happen if port mask would contain
                 * the link port of both FRS devices.
                 * Such frame would oscillate between FRS devices forever.
                 * If not taken care here, must be taken care of when
                 * passing frames to FRS.
                 */
            }
            else {
                port->port_mask |= linked_port_mask;
            }
        }
    }

    // Log all used trailer lengths and offsets.
    for (i = 0; i < ARRAY_SIZE(drv->dev_priv); i++) {
        unsigned int j;

        dp_tmp = drv->dev_priv[i];
        if (!dp_tmp)
            continue;

        dev_printk(KERN_DEBUG, dp_tmp->this_dev,
                   "Trailer length %i offset %i %s\n",
                   dp_tmp->trailer_len, dp_tmp->trailer_offset,
                   flx_frs_dev_has_cpu_port(dp_tmp) ?
                   "with CPU port" : "linked device");

        for (j = 0; j < ARRAY_SIZE(dp_tmp->port); j++) {
            struct flx_frs_port_priv *port = dp_tmp->port[j];
            if (!port)
                continue;

            // Enable MACsec for Deterministic Switch.
            if (dp->features.macsec_ports) {
                port->port_mask |= flx_frs_get_macsec_trailer(dp_tmp);
            }

            dev_printk(KERN_DEBUG, dp_tmp->this_dev,
                       "  Port %u %s send trailer 0x%x %s\n",
                       port->port_num, port->if_name, port->port_mask,
                       port->port_mask & flx_frs_get_macsec_trailer(dp_tmp) ?
                       "(MACsec)" : "");
            }
    }

    return;
}

/**
 * Function to initialise FRS platform devices.
 * @param pdev Platform device
 * @return 0 on success or negative error code.
 */
static int __devinit flx_frs_device_init(struct platform_device *pdev)
{
    int ret;
    struct flx_frs_drv_priv *drv = &flx_frs_drv_priv;
    struct flx_frs_dev_priv *dp = NULL;
    struct flx_frs_cfg frs_cfg_dev_tree = { .mac_name = NULL };
    struct flx_frs_cfg *frs_cfg = NULL;
    uint32_t pdev_id = 0;

    dev_info(&pdev->dev, "Init device\n");

    // use pdev->id if provided, if only one, pdev->id == -1
    if (pdev->id >= 0) {
        pdev_id = pdev->id;
    } else {
        // With device tree pdev->id may always be -1.
        while (pdev_id < ARRAY_SIZE(drv->dev_priv)) {
            if (!drv->dev_priv[pdev_id])
                break;
            pdev_id++;
        }
    }
    if (pdev_id >= ARRAY_SIZE(drv->dev_priv)) {
        dev_err(&pdev->dev, "Too many FRS devices\n");
        return -ENODEV;
    }
    /// Allocate device private
    dp = kmalloc(sizeof(*dp), GFP_KERNEL);
    if (!dp) {
        dev_err(&pdev->dev, "kmalloc failed\n");
        goto err_alloc;
    }

    *dp = (struct flx_frs_dev_priv) {
        .this_dev = &pdev->dev,
        .dev_num = pdev_id,
    };
    init_dev_privates(dp, pdev_id);

    frs_cfg = flx_frs_device_config(dp, pdev, &frs_cfg_dev_tree);
    if (!frs_cfg) {
        dev_err(dp->this_dev, "Failed to configure device\n");
        goto err_device_config;
    }

    if (dp->features.smac_rows > 0) {
        unsigned int bits = dp->features.smac_rows * FRS_SMAC_TABLE_COLS;
        size_t alloc_size = BITS_TO_LONGS(bits) * sizeof(*dp->smac.used);
        dp->smac.used = kzalloc(alloc_size, GFP_KERNEL);
        if (!dp->smac.used) {
            dev_err(dp->this_dev, "Failed to allocate SMAC usage bitmap\n");
            ret = -ENOMEM;
            goto err_smac;
        }
    }

    ret = flx_frs_init_ports(dp, frs_cfg);
    if (ret) {
        goto err_init_ports;
    }

    ret = flx_frs_reg_access_init_device(dp, pdev, frs_cfg);
    if (ret) {
        goto err_reg_access;
    }

    ret = flx_frs_irq_init(dp);
    if (ret) {
        goto err_irq;
    }

    ret = flx_frs_init_registers(dp);
    if (ret) {
        goto err_registers;
    }

    ret = flx_frs_switchdev_init_device(dp);
    if (ret)
        goto err_switchdev;

    ret = flx_frs_netdev_init(dp, frs_cfg);
    if (ret) {
        goto err_netdev;
    }

    ret = flx_frs_netdevif_init(dp);
    if (ret) {
        goto err_netdevif;
    }

    flx_frs_proc_init_device(dp);

    flx_frs_trailer_fixup(dp);

    flx_frs_ifacemode_fixup(dp);

    return 0;

err_netdevif:
    flx_frs_netdev_cleanup(dp);

err_netdev:
    flx_frs_switchdev_cleanup_device(dp);

err_switchdev:
err_registers:
    flx_frs_irq_cleanup(dp);

err_irq:
    flx_frs_reg_access_cleanup_device(dp);

err_reg_access:
err_init_ports:
    if (dp->smac.used) {
        kfree(dp->smac.used);
        dp->smac.used = NULL;
    }

err_smac:
err_device_config:
    flx_frs_drv_priv.dev_priv[pdev_id] = NULL;
    kfree(dp);

err_alloc:
    return -EFAULT;
}

/**
 * Function to clean device data
 */
static void __devexit flx_frs_device_cleanup(struct flx_frs_dev_priv *dp)
{
    unsigned int i;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    flx_frs_irq_cleanup(dp);

    flx_frs_proc_cleanup_device(dp);

    flx_frs_netdevif_cleanup(dp);

    flx_frs_netdev_cleanup(dp);

    flx_frs_switchdev_cleanup_device(dp);

    if (dp->smac.used) {
        kfree(dp->smac.used);
        dp->smac.used = NULL;
    }

    flx_frs_reg_access_cleanup_device(dp);

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct flx_frs_port_priv *pp = dp->port[i];

        if (!pp)
            continue;

        if (pp->ext_phy.bus_id) {
            kfree(pp->ext_phy.bus_id);
            pp->ext_phy.bus_id = NULL;
        }

        if (pp->sfp.phy.bus_id) {
            kfree(pp->sfp.phy.bus_id);
            pp->sfp.phy.bus_id = NULL;
        }

        if (pp->sfp.eeprom_name) {
            kfree(pp->sfp.eeprom_name);
            pp->sfp.eeprom_name = NULL;
        }

        pp->dp = NULL;
        kfree(pp);
    }

    dp->drv = NULL;
    flx_frs_drv_priv.dev_priv[dp->dev_num] = NULL;

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    dp->this_dev = NULL;
    kfree(dp);

    return;
}

/**
 * Helper function to read 32-bit port counter register.
 * @param pp Port privates whose counter register to read
 * @param low_reg_num Register number of the lowest 16 bits
 * of the 32-bit counter register
 * @return 32-bit counter value
 */
uint32_t flx_frs_read_port_counter(struct flx_frs_port_priv *pp,
                                   int low_reg_num)
{
    uint32_t value = 0;
    int data;

    data = flx_frs_read_port_reg(pp, low_reg_num);
    if (data < 0)
        goto error;

    value = data;

    data = flx_frs_read_port_reg(pp, low_reg_num + 1);
    if (data < 0)
        goto error;

    value |= (uint32_t) data << 16;

    return value;

  error:
    return 0xffffffffu;
}

