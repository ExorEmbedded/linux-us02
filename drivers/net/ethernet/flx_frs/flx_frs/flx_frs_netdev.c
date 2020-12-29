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
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/version.h>
#include <linux/etherdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#ifdef CONFIG_OF
#include <linux/of_mdio.h>
#endif

/* schedule() */
#include <linux/sched.h>

#include "flx_frs_main.h"
#include "flx_frs_types.h"
#include "flx_frs_if.h"
#include "flx_frs_netdev.h"
#include "flx_frs_netdevif.h"
#include "flx_frs_ioctl.h"
#include "flx_frs_ethtool.h"
#include "flx_frs_adapter.h"
#include "flx_frs_sfp.h"
#include "flx_frs_aux_netdev.h"
#include "flx_frs_switchdev.h"
#include "flx_frs_hw.h"

// IPO entry numbering sanity check.
#if (FRS_DRIVER_MAX_USER_ENTRIES < 0) || (FRS_DRIVER_MAX_USER_ENTRIES > 12)
# error(FRS_DRIVER_MAX_USER_ENTRIES must be between 0 and 12, inclusive)
#endif

/// Default NETIF message level
#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV|NETIF_MSG_PROBE|NETIF_MSG_LINK)

//#define DEBUG_FRS_MODIFY_XMIT

// Module parameters
static int debug = -1;                  ///< -1 means defaults above
module_param(debug, int, S_IRUGO);
MODULE_PARM_DESC(debug, "NETIF message level.");

static int ipo = 0;
module_param(ipo, int, S_IRUGO);
MODULE_PARM_DESC(ipo,
                 "Driver IPO handling:"
                 " 0=auto (default), 1=allow all multicast, 2=none");

#define IPO_AUTO                0       ///< automatic rules enabled
#define IPO_ALL_MULTICAST       1       ///< always allow all multicast
#define IPO_NONE                2       ///< no IPO rules automatically

/// Port statistics capture interval in jiffies
#define FLX_FRS_STATS_CAPTURE_INTERVAL (1*HZ)

/// Packets should be transmitted in 60 seconds
#define TX_TIMEOUT (60*HZ)

// Netdev API functions
static int flx_frs_open(struct net_device *dev);
static int flx_frs_close(struct net_device *dev);
static int flx_frs_enable_interface(struct net_device *dev);
static void flx_frs_disable_interface(struct net_device *dev);
static struct net_device_stats *flx_frs_get_stats(struct net_device *dev);
static netdev_tx_t flx_frs_start_xmit(struct sk_buff *skb,
                                      struct net_device *dev);
static void flx_frs_netdev_tx_timeout(struct net_device *netdev);
static void flx_frs_set_rx_mode(struct net_device *netdev);
static int flx_frs_set_mac_address(struct net_device *netdev, void *p);

static const uint8_t null_mac_addr[] = {
    0, 0, 0, 0, 0, 0
};
static const uint8_t multicast_mac_addr[] = {
    0x01, 0, 0, 0, 0, 0
};
static const uint8_t broadcast_mac_addr[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/// FRS netdevice operations
static const struct net_device_ops flx_frs_netdev_ops = {
    .ndo_open = &flx_frs_open,
    .ndo_start_xmit = &flx_frs_start_xmit,
    .ndo_stop = &flx_frs_close,
    .ndo_get_stats = &flx_frs_get_stats,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
    .ndo_set_multicast_list = &flx_frs_set_rx_mode,
#endif
    .ndo_do_ioctl = &flx_frs_netdev_ioctl,
    .ndo_validate_addr = NULL,
    .ndo_set_rx_mode = &flx_frs_set_rx_mode,
    .ndo_set_mac_address = &flx_frs_set_mac_address,
    .ndo_tx_timeout = &flx_frs_netdev_tx_timeout,
    .ndo_change_mtu = NULL,
#ifdef FLX_FRS_SWITCHDEV
    .ndo_get_phys_port_name = &flx_frs_switchdev_get_phys_port_name,
    .ndo_bridge_setlink = &switchdev_port_bridge_setlink,
    .ndo_bridge_getlink = &switchdev_port_bridge_getlink,
    .ndo_bridge_dellink = &switchdev_port_bridge_dellink,
    .ndo_fdb_add = &switchdev_port_fdb_add,
    .ndo_fdb_del = &switchdev_port_fdb_del,
    .ndo_fdb_dump = &switchdev_port_fdb_dump,
#endif
};

// phydev_name appeared in Linux 4.5.
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)

static inline const char *phydev_name(const struct phy_device *phydev)
{
    return dev_name(&phydev->dev);
}

static inline struct device *phydev_dev(struct phy_device *phydev)
{
    return &phydev->dev;
}

#else

static inline struct device *phydev_dev(struct phy_device *phydev)
{
    return &phydev->mdio.dev;
}

#endif

#ifdef CONFIG_FLX_BUS
/**
 * Netdevice set RX mode for indirect register access.
 * Kicks a work to handle it from sleepable context.
 */
static void flx_frs_set_rx_mode_indirect(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_drv_priv *drv = pp->dp->drv;

    queue_work(drv->wq, &pp->set_rx_mode);
}

/**
 * Netdevice set RX mode work handler.
 */
static void flx_frs_set_rx_mode_work(struct work_struct *work)
{
    struct flx_frs_port_priv *pp =
        container_of(work, struct flx_frs_port_priv, set_rx_mode);

    flx_frs_set_rx_mode(pp->netdev);
}

/**
 * FRS netdevice operations for indirect register access.
 * Some callbacks need to access FRS registers,
 * but are invoked from atomic context.
 */
static const struct net_device_ops flx_frs_netdev_ops_indirect = {
    .ndo_open = &flx_frs_open,
    .ndo_start_xmit = &flx_frs_start_xmit,
    .ndo_stop = &flx_frs_close,
    .ndo_get_stats = &flx_frs_get_stats,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
    .ndo_set_multicast_list = &flx_frs_set_rx_mode_indirect,
#endif
    .ndo_do_ioctl = &flx_frs_netdev_ioctl,
    .ndo_validate_addr = NULL,
    .ndo_set_rx_mode = &flx_frs_set_rx_mode_indirect,
    .ndo_set_mac_address = &flx_frs_set_mac_address,
    .ndo_tx_timeout = &flx_frs_netdev_tx_timeout,
    .ndo_change_mtu = NULL,
#ifdef FLX_FRS_SWITCHDEV
    .ndo_get_phys_port_name = &flx_frs_switchdev_get_phys_port_name,
    .ndo_bridge_setlink = &switchdev_port_bridge_setlink,
    .ndo_bridge_getlink = &switchdev_port_bridge_getlink,
    .ndo_bridge_dellink = &switchdev_port_bridge_dellink,
    .ndo_fdb_add = &switchdev_port_fdb_add,
    .ndo_fdb_del = &switchdev_port_fdb_del,
    .ndo_fdb_dump = &switchdev_port_fdb_dump,
#endif
};
#endif

/**
 * New netdevice initialization function.
 */
static void flx_frs_netdev_setup(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);

    ether_setup(netdev);

    netdev->tx_queue_len = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
    SET_ETHTOOL_OPS(netdev, &flx_frs_ethtool_ops);
#else
    netdev->ethtool_ops = &flx_frs_ethtool_ops;
#endif

    netdev->priv_destructor = &free_netdev;

    *np = (struct flx_frs_netdev_priv) {
        .msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE),
        .stp_state = BR_STATE_FORWARDING,
    };

    flx_frs_switchdev_setup_netdev(netdev);

    return;
}

/**
 * Add netdevice
 * @param real_netdev_name Name of the used Ethernet MAC interface
 * @param netdev_name Created netdevice
 * @param pp Port private data
 * @return New netdevice.
 */
static struct net_device *flx_frs_add_netdev(const char *real_netdev_name,
                                             const char *netdev_name,
                                             struct flx_frs_port_priv *pp)
{
    struct net_device *real_netdev = NULL;
    struct flx_frs_netdev_priv *np = NULL;
    struct net_device *netdev = NULL;
    int ret = 0;

    real_netdev = dev_get_by_name(&init_net, real_netdev_name);

    if (!real_netdev) {
        dev_err(pp->dp->this_dev, "Netdevice %s not found\n",
                real_netdev_name);
        return NULL;
    }
    // Get memory for the data structures
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0)
    netdev = alloc_netdev_mq(sizeof(struct flx_frs_netdev_priv), netdev_name,
                             &flx_frs_netdev_setup, 1);
#else
    netdev = alloc_netdev_mq(sizeof(struct flx_frs_netdev_priv),
                             netdev_name, NET_NAME_PREDICTABLE,
                             &flx_frs_netdev_setup, 1);
#endif
    if (!netdev) {
        printk(KERN_WARNING DRV_NAME ": cannot allocate net_device\n");
        dev_put(real_netdev);
        return NULL;
    }

    np = netdev_priv(netdev);
    np->port_priv = pp;

    mutex_init(&np->link_mode_lock);
    pp->netdev = netdev;

    SET_NETDEV_DEV(netdev, pp->dp->this_dev);

    netdev->addr_len = ETH_ALEN;
    memcpy(netdev->dev_addr, real_netdev->dev_addr, real_netdev->addr_len);

    netdev->base_addr = real_netdev->base_addr;
    netdev->irq = real_netdev->irq;

#ifdef CONFIG_FLX_BUS
    if (pp->regs.flx_bus)
        netdev->netdev_ops = &flx_frs_netdev_ops_indirect;
#endif
    if (!netdev->netdev_ops)
        netdev->netdev_ops = &flx_frs_netdev_ops;
    netdev->watchdog_timeo = TX_TIMEOUT;

    netdev->flags |= (IFF_MULTICAST);   // We can handle multicast packets
    dev_put(real_netdev);

    ret = register_netdev(netdev);
    if (ret) {
        printk(KERN_ERR DRV_NAME ": register_netdev failed for %s\n",
               netdev_name);
        goto free_netdev;
    }

    // Make sure ETHTOOL reports something sane also before interface is UP.
    ret = flx_frs_init_adapter(pp);
    if (ret)
        goto unregister_netdev;

    flx_frs_set_port_mode(netdev, LM_DOWN);

    dev_info(pp->dp->this_dev, "Flexibilis Redundant Switch (FRS) port %s\n",
             netdev->name);

    return netdev;

unregister_netdev:
    // Calls free_netdev.
    unregister_netdev(netdev);
    pp->netdev = NULL;
    return NULL;

free_netdev:
    pp->netdev = NULL;
    free_netdev(netdev);

    return NULL;
}

/**
 * Remove netdevice.
 * @param pp Port private data
 */
static void flx_frs_remove_netdev(struct flx_frs_port_priv *pp)
{
    struct net_device *netdev = pp->netdev;

    netdev_dbg(netdev, "%s()\n", __func__);

    // Unregister, calls destructor which is free_netdev (frees np also)
    unregister_netdev(netdev);
    pp->netdev = NULL;

    return;
}

/**
 * Create management port netdevice.
 * @param pp Port private data.
 * @param name netdevice name
 * @return New netdevice.
 */
static struct net_device *flx_frs_create_mgmt_netdev(
        struct flx_frs_port_priv *pp,
        const char *name)
{
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);
    struct net_device *netdev = NULL;
    int data = 0;

    netdev = flx_frs_add_netdev(dp->mac_name, name, pp);
    if (!netdev)
        return NULL;

    netdev_dbg(netdev, "Enable management\n");

    // Enable management trailer.
    data = flx_frs_read_port_reg(pp, PORT_REG_STATE);
    if (data < 0)
        return NULL;

    data |= PORT_STATE_MANAGEMENT;
    flx_frs_write_port_reg(pp, PORT_REG_STATE, data);

    return netdev;
}

/**
 * Create (external) port netdevice.
 * @param pp Port private data.
 * @param name netdevice name
 * @return New netdevice.
 */
static struct net_device *flx_frs_create_port_netdev(
        struct flx_frs_port_priv *pp,
        const char *name)
{
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);
    struct net_device *netdev = NULL;

    netdev = flx_frs_add_netdev(dp->mac_name, name, pp);

    return netdev;
}

/**
 * Adjust FRS port link mode taking net device running status into account.
 * Link mode mutex must be held when calling this.
 * @param netdev Netdevice associated with an FRS port.
 * @param link_mode New link mode for port, ignored if link mode is forced.
 */
int flx_frs_update_port_mode(struct net_device *netdev,
                             enum link_mode link_mode)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    int ret = 0;

    if (!netif_running(netdev)) {
        link_mode = LM_DOWN;
    }
    else if (np->force_link_mode != LM_DOWN) {
        struct flx_frs_port_priv *pp = np->port_priv;
        if ((pp->ext_phy.phydev && !pp->ext_phy.phydev->link) ||
            (pp->sfp.phy.phydev && !pp->sfp.phy.phydev->link))
            link_mode = LM_DOWN;
        else
            link_mode = np->force_link_mode;
    }

    if (link_mode != np->link_mode)
        ret = flx_frs_set_port_mode(netdev, link_mode);

    return ret;
}

/**
 * PHY device callback function to adjust FRS port link mode.
 * This is called from PHY polling thread.
 * @param netdev Netdevice associated with an FRS port.
 */
static void flx_frs_phy_adjust_link(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct phy_device *phydev;

    // Ignore information from PHY when using external/automatic signals.
    if (pp->flags & FLX_FRS_PORT_SPEED_EXT)
        return;

    mutex_lock(&np->link_mode_lock);

    phydev = pp->ext_phy.phydev;
    if (phydev) {
        enum link_mode link_mode = flx_frs_get_phy_link_mode(phydev);

        netdev_dbg(netdev, "PHY link %s autoneg %s speed %u %s"
                   " link mode %i forced %i"
                   " supported 0x%x adv 0x%x lpa 0x%x state %i\n",
                   phydev->link ? "UP" : "DOWN",
                   phydev->autoneg ? "ON" : "OFF",
                   phydev->speed,
                   phydev->duplex == DUPLEX_FULL ? "full-duplex" :
                   (phydev->duplex == DUPLEX_HALF ? "half-duplex" : "unknown"),
                   link_mode, np->force_link_mode,
                   phydev->supported, phydev->advertising,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
                   phydev->lp_advertising,
#else
                   0,
#endif
                   phydev->state);

        flx_frs_update_port_mode(netdev, link_mode);
    }

    mutex_unlock(&np->link_mode_lock);

    return;
}

/**
 * SFP PHY device callback function to adjust FRS port link mode.
 * This is called from PHY polling thread.
 * @param netdev Netdevice associated with an FRS port.
 */
static void flx_frs_sfp_phy_adjust_link(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct phy_device *phydev;

    // Ignore information from PHY when using external/automatic signals.
    if (pp->flags & FLX_FRS_PORT_SPEED_EXT)
        return;

    mutex_lock(&np->link_mode_lock);

    phydev = pp->sfp.phy.phydev;
    if (phydev) {
        enum link_mode link_mode = flx_frs_get_phy_link_mode(phydev);

        netdev_dbg(netdev, "SFP PHY link %s autoneg %s speed %u %s"
                   " link mode %i forced %i"
                   " supported 0x%x adv 0x%x lpa 0x%x state %i\n",
                   phydev->link ? "UP" : "DOWN",
                   phydev->autoneg ? "ON" : "OFF",
                   phydev->speed,
                   phydev->duplex == DUPLEX_FULL ? "full-duplex" :
                   (phydev->duplex == DUPLEX_HALF ? "half-duplex" : "unknown"),
                   link_mode, np->force_link_mode,
                   phydev->supported, phydev->advertising,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
                   phydev->lp_advertising,
#else
                   0,
#endif
                   phydev->state);

        /*
         * Use notifications from primary PHY to avoid periods of
         * conflicting information.
         */
        if (!pp->ext_phy.phydev) {
            flx_frs_update_port_mode(netdev, link_mode);
        }
        else {
            // Revert conflicting netif_carrier_on from PHY framework.
            if (!pp->ext_phy.phydev->link)
                netif_carrier_off(netdev);
        }
    }

    mutex_unlock(&np->link_mode_lock);

    return;
}

/**
 * Helper function to try to connect FRS port netdevice to PHY device.
 * @param netdev FRS port net device.
 * @param phy PHY context of PHY to connect to.
 * @param adjust_link Callback function for PHY to inform link changes.
 * @return true if successfully connected, false otherwise.
 */
static bool flx_frs_phy_connect(struct net_device *netdev,
                                struct flx_frs_phy *phy,
                                void (*adjust_link)(struct net_device *netdev))
{
    struct phy_device *orig_phydev = netdev->phydev;
    struct device_node *phy_node = NULL;

#ifdef CONFIG_OF
    phy_node = phy->node;
#endif

    if (!phy_node && !phy->bus_id) {
        netdev_dbg(netdev, "No PHY configured\n");
        return NULL;
    }

    // We may be attaching more than one PHY device to netdev.
    netdev->phydev = NULL;

#ifdef CONFIG_OF
    if (phy_node) {
        phy->phydev = of_phy_connect(netdev, phy_node, adjust_link, 0,
                                     phy->interface);
        if (!phy->phydev)
            goto fail;
    }
#endif
    if (!phy->phydev && phy->bus_id) {
        struct device *dev = NULL;

        // Avoid an error message each time by checking explicitly first
        // if phy_connect can find the device.
        dev = bus_find_device_by_name(&mdio_bus_type, NULL, phy->bus_id);

        if (!dev)
            goto fail;

        put_device(dev);
        phy->phydev = phy_connect(netdev, phy->bus_id, adjust_link,
                                  // Flags disappeared in Linux 3.9.
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
                                  0,
#endif
                                  phy->interface);
        if (IS_ERR(phy->phydev)) {
            phy->phydev = NULL;
            goto fail;
        }
    }

    // Save original supported features.
    phy->orig_supported = phy->phydev->supported;

    netdev_info(netdev, "Attached PHY driver [%s] (mii_bus:phy_addr=%s)\n",
                phy->phydev->drv->name, phydev_name(phy->phydev));

out:
    if (orig_phydev)
        netdev->phydev = orig_phydev;

    return phy->phydev != NULL;

fail:
    netdev_dbg(netdev, "Failed to attach PHY\n");
    goto out;
}

/**
 * Disconnect PHY device from FRS port.
 * @param netdev FRS port net device.
 * @param phy PHY context of PHY to disconnect.
 * @param alt_phydev Alternate PHY device to leave net device pointed to.
 */
static void flx_frs_phy_disconnect(struct net_device *netdev,
                                   struct flx_frs_phy *phy,
                                   struct phy_device *alt_phydev)
{
    int ret = 0;

    // PHY device may already be going away, don't access its pointers here.
    netdev_info(netdev, "Detach PHY (mii_bus:phy_addr=%s)\n",
                phydev_name(phy->phydev));

    netif_carrier_off(netdev);

    // We may be attached to more than one PHY device.
    netdev->phydev = phy->phydev;
    phy_stop(phy->phydev);

    if (phy->orig_supported) {
        // Restore original supported features.
        phy->phydev->supported = phy->orig_supported;
        phy->phydev->advertising = phy->orig_supported;
    }

    /*
     * In addition to phy_disconnect, release PHY from its driver
     * (temporarily), that allows SFP I2C driver to safely remove PHY and
     * thus handle SFP module changes. Reattach PHY device to a driver
     * immediately so that correct driver will still be used with it
     * if PHY device is not going to be removed entirely from system.
     */

    get_device(phydev_dev(phy->phydev));
    phy_disconnect(phy->phydev);
    device_release_driver(phydev_dev(phy->phydev));
    ret = device_attach(phydev_dev(phy->phydev));
    put_device(phydev_dev(phy->phydev));
    phy->phydev = NULL;

    netdev->phydev = alt_phydev;

    WARN_ON(ret < 0);

    return;
}

/**
 * Helper function to try to connect FRS port netdevice to SFP PHY device.
 * @param pp FRS port privates.
 * @return true if successfully connected to SFP PHY, false otherwise.
 */
static inline bool flx_frs_phy_connect_sfp(struct flx_frs_port_priv *pp)
{
    // Connect to SFP PHY, if configured.
    return flx_frs_phy_connect(pp->netdev, &pp->sfp.phy,
                               &flx_frs_sfp_phy_adjust_link);
}

/**
 * Try to connect FRS port netdevice to PHY and SFP PHY device(s).
 * @param pp FRS port privates.
 * @return true if successfully connected to PHY(s), false otherwise.
 */
static bool flx_frs_phy_connect_all(struct flx_frs_port_priv *pp)
{
    // Connect to external PHY, if configured.
    flx_frs_phy_connect(pp->netdev, &pp->ext_phy,
                        &flx_frs_phy_adjust_link);

    // Connect to SFP PHY, if configured.
    flx_frs_phy_connect_sfp(pp);

    return pp->ext_phy.phydev || pp->sfp.phy.phydev;
}

/**
 * Detect if PHY is present.
 * When e.g. SFP is removed, PHY register accesses fail and cause the PHY
 * to be put in PHY_HALTED state by the PHY framework.
 * @param phy PHY context of PHY to check.
 * @return true if PHY is still present.
 */
static inline bool flx_frs_is_phy_present(struct flx_frs_phy *phy)
{
    struct phy_device *phydev = phy->phydev;
    bool present = false;

    if (phydev) {
        mutex_lock(&phydev->lock);

        present = phydev->state != PHY_HALTED;

        mutex_unlock(&phydev->lock);
    }

    return present;
}

/**
 * Link polling helper function to acquire locks.
 * @param pp FRS port privates.
 * @param locked Placed for already locked flag.
 */
static void flx_frs_poll_lock(struct flx_frs_port_priv *pp, bool *locked)
{
    if (!*locked) {
        struct net_device *netdev = pp->netdev;
        struct flx_frs_netdev_priv *np = netdev_priv(netdev);

        /*
         * Prevent races with PHY callbacks.
         * Acquire mutexes in correct (the same) order.
         * Note that PHY polling calls adjust_link with
         * PHY lock held.
         */

        if (pp->ext_phy.phydev)
            mutex_lock(&pp->ext_phy.phydev->lock);
        mutex_lock(&np->link_mode_lock);

        *locked = true;
    }

    return;
}

/**
 * Link polling helper function to release locks.
 * @param pp FRS port privates.
 * @param locked Placed for locked flag.
 */
static void flx_frs_poll_unlock(struct flx_frs_port_priv *pp, bool *locked)
{
    if (*locked) {
        struct net_device *netdev = pp->netdev;
        struct flx_frs_netdev_priv *np = netdev_priv(netdev);

        mutex_unlock(&np->link_mode_lock);
        if (pp->ext_phy.phydev)
            mutex_unlock(&pp->ext_phy.phydev->lock);
        *locked = false;
    }

    return;
}

/**
 * Work function to check link state.
 * If PHY is present, actual link state from PHY framework is used,
 * otherwise this function checks link state from port adapter.
 * Detect SFP module changes, too, and connect to or disconnect from
 * SFP PHY as needed.
 * Reinitialize also port adapter when required.
 * @param work Port work structure.
 */
static void flx_frs_poll_link(struct work_struct *work)
{
    struct flx_frs_port_priv *pp =
        container_of(work, struct flx_frs_port_priv, check_link.work);
    struct flx_frs_phy *sfp_phy = &pp->sfp.phy;
    struct flx_frs_drv_priv *drv = pp->dp->drv;
    struct net_device *netdev = pp->netdev;
    enum link_mode link_mode;
    bool restart_ext_phy = false;

    // Delay locking until needeed to reduce latencies on PHY polling.
    // Be careful with locking here.
    bool locked = false;

    // Detect SFP module changes.
    if (pp->medium_type == FLX_FRS_MEDIUM_SFP) {
        // Detect SFP module from SFP EEPROM.
        if (pp->sfp.eeprom) {
            enum flx_frs_sfp_type sfp = flx_frs_detect_sfp(pp);
            if (sfp != pp->sfp.type) {
                flx_frs_poll_lock(pp, &locked);

                flx_frs_set_sfp(pp, sfp);

                // Trigger complete SFP PHY reinitialization (see below).
                if (sfp_phy->phydev) {
                    flx_frs_phy_disconnect(pp->netdev, sfp_phy,
                                           pp->ext_phy.phydev);
                }

                // Adjust adapter to new SFP module.
                flx_frs_init_adapter(pp);

                restart_ext_phy = true;

                /*
                 * Do dot reconnect to PHY at the same iteration,
                 * let it be removed completely from system.
                 */
                goto unlock_and_out;
            }
        }

        // Handle connection to SFP PHY.
        if (pp->flags & FLX_FRS_HAS_SFP_PHY) {
            /*
             * This must be checked before acquiring link mode mutex
             * to avoid deadlock.
             */
            bool sfp_phy_present = flx_frs_is_phy_present(sfp_phy);

            flx_frs_poll_lock(pp, &locked);

            if (sfp_phy->phydev) {
                if (!sfp_phy_present) {
                    // SFP PHY disappeared, e.g. SFP module removed.
                    flx_frs_phy_disconnect(pp->netdev, sfp_phy,
                                           pp->ext_phy.phydev);
                    flx_frs_init_adapter(pp);

                    restart_ext_phy = true;
                }
            }
            else {
                if (flx_frs_phy_connect_sfp(pp)) {
                    // SFP PHY appeared, e.g. SFP module plugged in.
                    flx_frs_init_adapter(pp);

                    // Must release mutex before phy_start.
                    flx_frs_poll_unlock(pp, &locked);

                    phy_start(sfp_phy->phydev);

                    restart_ext_phy = true;
                    goto out;
                }
            }
        }
    }

    // Take link status from adapter when there is no PHY,
    // or from external/automatic speed selection signals.
    if ((pp->flags & FLX_FRS_PORT_SPEED_EXT) ||
        (!pp->ext_phy.phydev && !sfp_phy->phydev &&
         pp->adapter.ops.check_link)) {

        flx_frs_poll_lock(pp, &locked);

        // Adapter interface might have changed already.
        // In any case external/automatic signals override everything else.
        if (pp->flags & FLX_FRS_PORT_SPEED_EXT) {
            link_mode = flx_frs_get_ext_link_mode(pp);
            flx_frs_update_port_mode(netdev, link_mode);
        }
        else if (pp->adapter.ops.check_link) {
            link_mode = pp->adapter.ops.check_link(pp);
            flx_frs_update_port_mode(netdev, link_mode);
        }
    }

unlock_and_out:
    flx_frs_poll_unlock(pp, &locked);

out:
    /*
     * Inform PHY driver about interface changes
     * by restarting auto-negotiation.
     */
    if (pp->ext_phy.phydev && restart_ext_phy) {
        phy_start_aneg(pp->ext_phy.phydev);
    }

    // Reschedule check.
    queue_delayed_work(drv->wq, &pp->check_link, FLX_FRS_LINK_CHECK_INTERVAL);

    return;
}

/**
 * Init all netdevices.
 * @param dp Device private data
 * @return 0 on success, or negative error code.
 */
int flx_frs_netdev_init(struct flx_frs_dev_priv *dp,
                        struct flx_frs_cfg *frs_cfg)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct flx_frs_port_priv *pp = dp->port[i];
        struct net_device *netdev = NULL;

        if (!pp)
            continue;

        if (pp->flags & (FLX_FRS_PORT_CPU | FLX_FRS_PORT_IE)) {
            netdev = flx_frs_create_mgmt_netdev(pp, pp->if_name);
        }
        else {
            netdev = flx_frs_create_port_netdev(pp, pp->if_name);
        }

        if (!netdev)
            goto err_1;
    }

    return 0;

  err_1:
    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct flx_frs_port_priv *pp = dp->port[i];

        if (!pp)
            continue;

        if (!pp->netdev)
            continue;

        flx_frs_remove_netdev(pp);
    }

    return 0;
}

/**
 * Cleanup netdevices.
 * @param dp Device data
 */
void flx_frs_netdev_cleanup(struct flx_frs_dev_priv *dp)
{
    unsigned int i;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    flx_frs_aux_remove_all(dp);

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct flx_frs_port_priv *pp = dp->port[i];

        if (!pp)
            continue;

        if (!pp->netdev)
            continue;

        flx_frs_remove_netdev(pp);
    }

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return;
}

/**
 * Work function to capture port statistics.
 * @param work Port work structure.
 */
static void flx_frs_capture_stats(struct work_struct *work)
{
    struct flx_frs_port_priv *pp =
        container_of(work, struct flx_frs_port_priv, capture_stats.work);
    struct flx_frs_drv_priv *drv = pp->dp->drv;

    mutex_lock(&pp->stats_lock);

    flx_frs_update_port_stats(pp);

    mutex_unlock(&pp->stats_lock);

    // Reschedule capture.
    queue_delayed_work(drv->wq, &pp->capture_stats,
                       FLX_FRS_STATS_CAPTURE_INTERVAL);

    return;
}

/**
 * Opens the network device interface (ifconfig XX up).
 * @param dev Netdevice.
 * @return 0 on success or negative error code.
 */
static int flx_frs_open(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);
    struct flx_frs_drv_priv *drv = pp->dp->drv;
    int ret = 0;

    netdev_dbg(netdev, "%s()\n", __func__);

    flx_frs_phy_connect_all(pp);

    switch (pp->medium_type) {
    case FLX_FRS_MEDIUM_PHY:
        np->force_link_mode = LM_DOWN;  // allow autoneg
        np->link_mode = LM_DOWN;
        flx_frs_enable_interface(netdev);
        break;
    case FLX_FRS_MEDIUM_SFP:
        np->force_link_mode = LM_DOWN;  // allow autoneg
        np->link_mode = LM_DOWN;
        flx_frs_init_sfp(pp);
        if (pp->sfp.eeprom) {
            // Detect SFP so that adapter can be initialized correctly.
            flx_frs_set_sfp(pp, flx_frs_detect_sfp(pp));
        }
        flx_frs_enable_interface(netdev);
        break;
    case FLX_FRS_MEDIUM_NOPHY:
        // Try to determine link mode from EMAC (CPU port) or from adapter.
        if (pp->flags & FLX_FRS_PORT_CPU) {
            struct net_device *real_netdev;

            real_netdev = dev_get_by_name(&init_net, dp->mac_name);
            if (!real_netdev) {
                dev_err(dp->this_dev, "Netdevice %s not found\n",
                        dp->mac_name);
                return -ENXIO;
            }

            if (real_netdev->phydev) {
                np->link_mode = flx_frs_get_phy_link_mode(real_netdev->phydev);
            }
            dev_put(real_netdev);
        }

        flx_frs_enable_interface(netdev);
        break;
    case FLX_FRS_MEDIUM_NONE:
    default:
        netdev_dbg(netdev, "Port not in use\n");
        return -ENODEV;
    }

#ifdef CONFIG_FLX_BUS
    INIT_WORK(&pp->set_rx_mode, &flx_frs_set_rx_mode_work);
#endif

    // Capture statistics counters periodically.
    if (dp->features.flags & FLX_FRS_FEAT_STATS) {
        INIT_DELAYED_WORK(&pp->capture_stats, &flx_frs_capture_stats);
        queue_delayed_work(drv->wq, &pp->capture_stats,
                           FLX_FRS_STATS_CAPTURE_INTERVAL);
    }

    // Start interface
    netif_start_queue(netdev);

    if (pp->flags & FLX_FRS_PORT_CPU) {
        struct net_device *real_netdev =
            dev_get_by_name(&init_net, pp->dp->mac_name);
        if (real_netdev) {
            dev_set_promiscuity(real_netdev, 1);
            dev_put(real_netdev);
        }
    }

    netdev_info(netdev, "Interface open\n");

    netdev_dbg(netdev,
               "Supported PHY 0x%x SFP 0x%x adapter 0x%x\n",
               pp->ext_phy.phydev ? pp->ext_phy.phydev->supported : 0,
               pp->sfp.supported,
               pp->adapter.supported);

    return ret;
}

/**
 * Close the network interface.
 * @param dev Netdevice.
 * @return 0 on success or negative error code.
 */
static int flx_frs_close(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = pp->dp;

    netdev_dbg(netdev, "%s()\n", __func__);

    if (pp->flags & FLX_FRS_PORT_CPU) {
        struct net_device *real_netdev =
            dev_get_by_name(&init_net, pp->dp->mac_name);
        if (real_netdev) {
            dev_set_promiscuity(real_netdev, -1);
            dev_put(real_netdev);
        }
    }

    // We can not take any more packets to be sent
    if (!netif_queue_stopped(netdev))
        netif_stop_queue(netdev);

#ifdef CONFIG_FLX_BUS
    cancel_work_sync(&pp->set_rx_mode);
#endif

    // Stop capturing statistics counters.
    if (dp->features.flags & FLX_FRS_FEAT_STATS) {
        cancel_delayed_work_sync(&pp->capture_stats);
    }

    flx_frs_disable_interface(netdev);

    // Ensure IPO rules are updated.
    flx_frs_set_rx_mode(netdev);

    if (pp->medium_type == FLX_FRS_MEDIUM_SFP) {
        flx_frs_cleanup_sfp(pp);
    }

    netdev_dbg(netdev, "Interface closed\n");

    return 0;
}

/**
 * Returns statistics of the network interface.
 * @param dev Netdevice.
 * @return statistics.
 */
static struct net_device_stats *flx_frs_get_stats(struct net_device *netdev)
{
    static struct net_device_stats stats;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);

    // Start from local statistics
    stats = np->stats;

    return &stats;
}

/**
 * Sends a packet, given in a buffer.
 * @param skb Buffer to send.
 * @param dev netdevice.
 * @return 0 on success or negative error code.
 */
static netdev_tx_t flx_frs_start_xmit(struct sk_buff *skb,
                                      struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);
    struct net_device *real_netdev = NULL;
    int skb_len_dif = 0;
    int ret = 0;
    unsigned int trailer = 0;

    // Get netdevice (MAC) we use for sending frames to FRS
    real_netdev = dev_get_by_name(&init_net, dp->mac_name);
    if (!real_netdev) {
        dev_err(dp->this_dev, "Netdevice %s not found\n", dp->mac_name);
        dev_kfree_skb_any(skb);
        return NETDEV_TX_OK;
    }

    trailer = pp->port_mask;

    /*
     * It is not possible to send frames to both linked FRS link ports
     * in such a way that the other FRS would see its part of
     * management trailer being zero. That would allow use of IPO
     * rules to define from which ports of the other FRS to send the frame out.
     * Sending anything through linked ports is disabled here,
     * for consistency and to prevent frames leaking to undesired ports
     * and to also prevent frame looping forever between linked FRS devices.
     */
    if (pp->flags & FLX_FRS_PORT_IE) {
        netdev_printk(KERN_DEBUG, netdev, "Not sending to avoid loop\n");
        dev_kfree_skb_any(skb);
        goto free_real_netdev;
    }

    // Add Management trailer to frame.
    // Ethernet has minimum packet length, pad also if necessary.
    if (skb->len < ETH_ZLEN) {
        // Verify that there is space
        skb_len_dif = ETH_ZLEN - skb->len;
        ret = skb_pad(skb, skb_len_dif + dp->trailer_len);
        if (ret) {
            dev_kfree_skb_any(skb);
            goto free_real_netdev;
        }
        memset(skb_put(skb, skb_len_dif), 0, skb_len_dif);
    } else {
        // Verify that there is space for trailer in tail
        ret = skb_pad(skb, dp->trailer_len);
        if (ret) {
            dev_kfree_skb_any(skb);
            goto free_real_netdev;
        }
    }
    flx_frs_set_skb_trailer(skb, trailer, dp->trailer_len);

#ifdef DEBUG_FRS_MODIFY_XMIT
    netdev_printk(KERN_DEBUG, netdev, "Sending with trailer 0x%x\n", trailer);
    print_hex_dump_bytes("tx_frame: ", DUMP_PREFIX_OFFSET,
                         &skb->data[0], skb->len);
#endif

    np->stats.tx_packets++;
    np->stats.tx_bytes += skb->len;

    // Transmit a buffer
    flx_frs_xmit(real_netdev, skb);

  free_real_netdev:
    dev_put(real_netdev);

    return NETDEV_TX_OK;
}

/**
 * Netdevice timeout, currently unimplemented.
 */
static void flx_frs_netdev_tx_timeout(struct net_device *netdev)
{
    netdev_dbg(netdev, "Netdev timeout\n");
}

/**
 * Deal with automatically handled IPO rules.
 * @param netdev FRS CPU port netdevice
 * @param real_netdev FRS CPU port EMAC netdevice
 */
static void flx_frs_update_ipo_rules(struct net_device *netdev,
                                     struct net_device *real_netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_port_priv *port_tmp = NULL;
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);
    struct netdev_hw_addr *ha = NULL;
    uint8_t *a = NULL;
    unsigned int entry = 0;
    unsigned int i = 0;
    bool enable_all_multicast = false;
    uint8_t dev_addr[IFHWADDRLEN];

    netdev_dbg(netdev, "%s()\n", __func__);

    if (ipo == IPO_ALL_MULTICAST) {
        enable_all_multicast = true;
    } else if (netdev_mc_count(netdev) > FRS_DRIVER_MAX_MULT_ENTRIES) {
        netdev_dbg(netdev,
                   "More multicast entries than can be handled, enable all\n");
        enable_all_multicast = true;
    } else {
        // Set IPO for every external port for a specific MAC
        netdev_for_each_mc_addr(ha, netdev) {
            netdev_dbg(netdev, "MULTICAST: %pM\n", ha->addr);
            a = ha->addr;
            for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
                port_tmp = dp->port[i];
                if (!port_tmp)
                    continue;
                if (port_tmp->flags & FLX_FRS_PORT_CPU)
                    continue;
                flx_frs_write_port_ipo(port_tmp,
                                       FRS_DRIVER_FIRST_FILT_IPO_ENTRY +
                                       entry,
                                       PORT_ETH_ADDR_ENABLE |
                                       PORT_ETH_ADDR_DEST |
                                       PORT_ETH_ADDR_PRESERVE_PRIORITY,
                                       0xffff, 0, 0,
                                       a, 48);
            }
            entry++;
        }
    }

    for (; entry < FRS_DRIVER_MAX_MULT_ENTRIES; entry++) {
        // Clear the rest IPO entries.
        for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
            port_tmp = dp->port[i];
            if (!port_tmp)
                continue;
            if (port_tmp->flags & FLX_FRS_PORT_CPU)
                continue;
            // In case all multicasts are to be allowed,
            // use the last multicast filter IPO entry for that.
            if (enable_all_multicast &&
                entry == FRS_DRIVER_MAX_MULT_ENTRIES - 1) {
                flx_frs_write_port_ipo(port_tmp,
                                       FRS_DRIVER_FIRST_FILT_IPO_ENTRY +
                                       entry,
                                       PORT_ETH_ADDR_ENABLE |
                                       PORT_ETH_ADDR_DEST |
                                       PORT_ETH_ADDR_PRESERVE_PRIORITY,
                                       0xffff, 0, 0,
                                       multicast_mac_addr, 1);
            }
            else {
                flx_frs_write_port_ipo(port_tmp,
                                       FRS_DRIVER_FIRST_FILT_IPO_ENTRY + entry,
                                       0, 0, 0, 0,
                                       null_mac_addr, 48);
            }
        }
    }

    // Set unicast, broadcast, and block all entries.
    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        port_tmp = dp->port[i];
        if (!port_tmp)
            continue;
        if (port_tmp->flags & FLX_FRS_PORT_CPU)
            continue;
        flx_frs_write_port_ipo(port_tmp,
                               FRS_DRIVER_BROADCAST_ENTRY,
                               PORT_ETH_ADDR_ENABLE |
                               PORT_ETH_ADDR_DEST |
                               PORT_ETH_ADDR_PRESERVE_PRIORITY,
                               0xffff, 0, 0,
                               broadcast_mac_addr, 48);
        // In case a MAC address was changed to something else
        if (port_tmp->flags & FLX_FRS_PORT_INDEPENDENT) {
            memcpy(dev_addr, port_tmp->netdev->dev_addr,
                   port_tmp->netdev->addr_len);
        } else {
            memcpy(dev_addr, real_netdev->dev_addr, real_netdev->addr_len);
        }
        flx_frs_write_port_ipo(port_tmp,
                               FRS_DRIVER_UNICAST_ENTRY,
                               PORT_ETH_ADDR_ENABLE |
                               PORT_ETH_ADDR_DEST |
                               PORT_ETH_ADDR_PRESERVE_PRIORITY,
                               dp->cpu_port_mask, 0, 0,
                               dev_addr, 48);
        flx_frs_write_port_ipo(port_tmp,
                               FRS_DRIVER_BLOCK_ALL_ENTRY,
                               PORT_ETH_ADDR_ENABLE |
                               PORT_ETH_ADDR_DEST |
                               PORT_ETH_ADDR_PRESERVE_PRIORITY,
                               0xffffu & ~dp->cpu_port_mask, 0, 0,
                               null_mac_addr, 0);
    }

    netdev_dbg(netdev, "%s() done\n", __func__);

    return;
}

/**
 * Netdevice set RX mode.
 * Syncronizes CPU port multicast addresses to EMAC
 * and writes IPO rules for multicast handling.
 */
static void flx_frs_set_rx_mode(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);
#define COPY_FLAGS (IFF_LOOPBACK | IFF_BROADCAST | IFF_PROMISC | IFF_ALLMULTI)
    unsigned int flags = netdev->flags & COPY_FLAGS;
    struct net_device *real_netdev = NULL;
    const uint8_t ptp_multicast_mac[IFHWADDRLEN] =
        { 0x01, 0x1B, 0x19, 0x00, 0x00, 0x00 };
    struct netdev_hw_addr *ha = NULL;
    int ret = 0;

    // Check if PTP enabled (L2)
    pp->flags &= ~FLX_FRS_MSG_PTP; // first clear
    netdev_for_each_mc_addr(ha, netdev) {
        if (!memcmp(ha->addr, ptp_multicast_mac, ETH_ALEN)) {
            pp->flags |= FLX_FRS_MSG_PTP;
            netdev_dbg(netdev, "PTP enabled\n");
            break;
        }
    }

    if (!(pp->flags & FLX_FRS_PORT_CPU)) {
        netdev_dbg(netdev,
                   "Multicast filter can be configured only to CPU interface\n");
        return;
    }

    real_netdev = dev_get_by_name(&init_net, dp->mac_name);
    if (!real_netdev) {
        dev_err(dp->this_dev, "Netdevice %s not found\n", dp->mac_name);
        return;
    }

    netdev_dbg(netdev, "%s(): flags 0x%x (mc count: %i)\n",
               __func__, flags, netdev_mc_count(netdev));

    if (ipo == IPO_NONE) {
        netdev_dbg(netdev, "Automatic IPO rules disabled\n");
    }
    else {
        flx_frs_update_ipo_rules(netdev, real_netdev);
    }

    // Set mode to underlying MAC

    // Update flags
    real_netdev->flags &= ~COPY_FLAGS;
    real_netdev->flags |= flags | IFF_PROMISC;

    // Update address lists in MAC
    ret = dev_uc_sync(real_netdev, netdev);
    if (ret) {
        netdev_dbg(netdev, "%s(): dev_uc_sync failed (%i)\n",
                   __func__, ret);
    }
    dev_mc_sync(real_netdev, netdev);
    if (ret) {
        netdev_dbg(netdev, "%s(): dev_mc_sync failed (%i)\n",
                   __func__, ret);
    }
#if 0
    netdev_dbg(netdev, "%s(): from flags 0x%x (mc count: %i)\n",
               __func__, netdev->flags, netdev_mc_count(netdev));
    netdev_dbg(netdev, "%s(): Set flags 0x%x (mc count: %i)\n",
               __func__, real_netdev->flags,
               netdev_mc_count(real_netdev));
    netdev_for_each_mc_addr(ha, real_netdev)
        netdev_dbg(netdev, "MULT3: %pM\n", ha->addr);
#endif
    dev_put(real_netdev);

    netdev_dbg(netdev, "%s() done\n", __func__);
    return;
}

/**
 * Change the MAC address, the address can be changed also on-the-fly.
 */
static int flx_frs_set_mac_address(struct net_device *netdev, void *p)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);
    struct sockaddr *addr = p;
    struct net_device *real_netdev = NULL;
    int ret = 0;

    real_netdev = dev_get_by_name(&init_net, dp->mac_name);
    if (!real_netdev) {
        dev_err(dp->this_dev, "Netdevice %s not found\n", dp->mac_name);
        return -ENXIO;
    }

    netdev_printk(KERN_DEBUG, netdev, "Set MAC address %pM\n", addr->sa_data);

    if (!is_valid_ether_addr(addr->sa_data)) {
        ret = -EADDRNOTAVAIL;
        goto ret;
    }

    if (pp->flags & FLX_FRS_PORT_INDEPENDENT) {
        // Modify unicast IPO rule to get frames in by new address
        mutex_lock(&pp->port_reg_lock);
        flx_frs_write_port_ipo(pp,
                               FRS_DRIVER_UNICAST_ENTRY,
                               PORT_ETH_ADDR_ENABLE | PORT_ETH_ADDR_DEST,
                               dp->cpu_port_mask,
                               0, 0, addr->sa_data, 48);
        mutex_unlock(&pp->port_reg_lock);

        goto success;
    }

    // switch MAC address to underlying MAC
    ret = real_netdev->netdev_ops->ndo_set_mac_address(real_netdev, p);
    if (ret) {
        dev_printk(KERN_DEBUG, dp->this_dev,
                   "Real netdev ndo_set_mac_address returned error %i\n", ret);
        goto ret;
    } else {
        dev_dbg(dp->this_dev,
                "Real netdev ndo_set_mac_address returned success\n");
    }

success:
    memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

ret:
    dev_put(real_netdev);

    return ret;
}

/**
 * Function to receive frame.
 * @see struct flx_frs_port_priv
 * @param pp port private data.
 * @param rx_frame
 */
void flx_frs_rx_frame(struct flx_frs_port_priv *pp,
                      struct sk_buff *rx_frame)
{
    struct net_device *netdev = pp->netdev;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);

    if (!netif_carrier_ok(netdev)) {
        netdev_dbg(netdev, "%s(): No carrier\n", __func__);
        // Not connected
        dev_kfree_skb(rx_frame);
        np->stats.rx_errors++;
        return;
    }

    rx_frame->dev = netdev;
#ifdef FLX_FRS_SWITCHDEV
    flx_frs_switchdev_set_skb_offload_fwd_mark(rx_frame, netdev);
#endif

    /* If a MAC address has been changed for a port,
     * unicast frames are received with an IPO rule
     * set by flx_frs_set_mac_address().
     * When the address doess not match the address
     * of the real device, pkt_type in frames may
     * get changed to PACKET_OTHERHOST.
     * Higher level would drop such frames, hence
     * change pkt_type in this case to PACKET_HOST.
     */
    if (rx_frame->pkt_type == PACKET_OTHERHOST) {
        rx_frame->pkt_type = PACKET_HOST;
    }
    // update statistics
    np->stats.rx_bytes += rx_frame->len;
    np->stats.rx_packets++;

    //netdev_dbg(netdev, "%s(): RX %i\n", __func__, rx_frame->len);

    // give the packet to the stack
    netif_rx(rx_frame);
    // the buffer was given away

    return;
}

/**
 * Enable interface.
 * @param dev Netdevice.
 */
static int flx_frs_enable_interface(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_drv_priv *drv = pp->dp->drv;
    int ret;

    netif_info(np, ifup, netdev, "Enabling interface\n");

    INIT_DELAYED_WORK(&pp->check_link, &flx_frs_poll_link);

    ret = flx_frs_init_adapter(pp);
    if (ret)
        return ret;

    if (pp->medium_type == FLX_FRS_MEDIUM_NOPHY) {
        if (np->link_mode == LM_DOWN)
            np->link_mode = flx_frs_best_adapter_link_mode(pp);
        if (np->link_mode == LM_DOWN)
            np->link_mode = LM_1000FULL;
    }
    if (pp->ext_phy.phydev) {
        phy_start(pp->ext_phy.phydev);
        np->link_mode = flx_frs_get_phy_link_mode(pp->ext_phy.phydev);
    }
    if (pp->sfp.phy.phydev) {
        phy_start(pp->sfp.phy.phydev);
        if (!pp->ext_phy.phydev)
            np->link_mode = flx_frs_get_phy_link_mode(pp->sfp.phy.phydev);
    }

    flx_frs_set_port_mode(netdev, np->link_mode);

    // Poll link state.
    queue_delayed_work(drv->wq, &pp->check_link, FLX_FRS_LINK_CHECK_INTERVAL);

    flx_frs_switchdev_enable(netdev);

    return 0;
}

/**
 * Disable interface.
 * @param dev Netdevice.
 */
static void flx_frs_disable_interface(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;

    netif_info(np, ifdown, netdev, "Disabling interface\n");

    flx_frs_switchdev_disable(netdev);

    // Stop polling link state.
    cancel_delayed_work_sync(&pp->check_link);

    /*
     * Disconnect PHY(s).
     * Note: adjust_link may still be running, cannot acquire (just)
     * link mode lock from here. We don't need to.
     */
    if (pp->sfp.phy.phydev) {
        flx_frs_phy_disconnect(pp->netdev, &pp->sfp.phy, pp->ext_phy.phydev);
    }
    if (pp->ext_phy.phydev) {
        flx_frs_phy_disconnect(pp->netdev, &pp->ext_phy, NULL);
    }

    flx_frs_set_port_mode(netdev, LM_DOWN);

    netdev_dbg(netdev, "%s() done\n", __func__);

    return;
}

/**
 * Determine whether a netdevice refers to FRS port.
 * @param netdev Net device to check
 * @return True if netdev is an FRS port
 */
bool flx_frs_is_port(struct net_device *netdev)
{
#ifdef CONFIG_FLX_BUS
    if (netdev->netdev_ops == &flx_frs_netdev_ops_indirect)
        return true;
#endif
    if (netdev->netdev_ops == &flx_frs_netdev_ops)
        return true;

    return false;
}

