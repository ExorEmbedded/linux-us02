/** @file
 */

/*

   FRS Linux driver

   Copyright (C) 2015 Flexibilis Oy

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
#include <net/rtnetlink.h>

#include "flx_frs_main.h"
#include "flx_frs_types.h"
#include "flx_frs_iflib.h"
#include "flx_frs_netdevif.h"
#include "flx_frs_netdev.h"
#include "flx_frs_aux_netdev.h"
#include "flx_frs_if.h"

/// Uncomment to have netlink support for adding/removing auxiliary netdevices.
#define CONFIG_FLX_FRS_NETLINK

/// Uncomment to debug frame sending
//#define DEBUG_FRS_MODIFY_XMIT

/// Sysfs directory name for symlinks to ports
#define FLX_FRS_AUX_PORTS_ADDR "ports"

/// Packets should be transmitted in 60 seconds
#define TX_TIMEOUT (60*HZ)

#ifdef CONFIG_FLX_FRS_NETLINK
static struct rtnl_link_ops flx_frs_aux_netdev_rtnl_ops __read_mostly;
#endif

static int flx_frs_aux_remove(struct net_device *netdev);

/**
 * Opens the network device interface (ifconfig XX up).
 * @param dev Netdevice.
 * @return 0 on success or negative error code.
 */
static int flx_frs_aux_open(struct net_device *netdev)
{
    int ret = 0;

    netif_carrier_on(netdev);

    // Start interface
    netif_start_queue(netdev);

    netdev_dbg(netdev, "Interface open\n");

    return ret;
}

/**
 * Close the network interface.
 * @param dev Netdevice.
 * @return 0 on success or negative error code.
 */
static int flx_frs_aux_close(struct net_device *netdev)
{
    // We can not take any more packets to be sent
    if (!netif_queue_stopped(netdev))
        netif_stop_queue(netdev);

    netif_carrier_off(netdev);

    netdev_dbg(netdev, "Interface closed\n");

    return 0;
}

/**
 * Returns statistics of the network interface.
 * @param dev Netdevice.
 * @return statistics.
 */
static struct net_device_stats *flx_frs_aux_get_stats(
        struct net_device *netdev)
{
    struct flx_frs_aux_netdev_priv *anp = netdev_priv(netdev);

    return &anp->stats;
}

/**
 * Sends a packet, given in a buffer.
 * @param skb Buffer to send.
 * @param dev netdevice.
 * @return 0 on success or negative error code.
 */
static netdev_tx_t flx_frs_aux_start_xmit(struct sk_buff *skb,
                                          struct net_device *netdev)
{
    struct flx_frs_aux_netdev_priv *anp = netdev_priv(netdev);
    struct flx_frs_dev_priv *dp = anp->dp;
    struct net_device *real_netdev = NULL;
    int skb_len_dif = 0;
    uint16_t port_mask = anp->port_mask;
    int ret = 0;

    // Do not send to ports which are down or don't have link
    spin_lock(&dp->link_mask_lock);
    port_mask &= dp->link_mask;
    spin_unlock(&dp->link_mask_lock);

    // Do not fallback to CPU port behavior if no ports are defined and up.
    if (port_mask == 0) {
        netdev_dbg(netdev, "Refusing to use management trailer zero\n");
        dev_kfree_skb_any(skb);
        return NETDEV_TX_OK;
    }

    /* Enable MACsec bit in trailer for Deterministic Switch,
     * link masking probably removed it from port_mask
     */
    if (dp->features.macsec_ports) {
        port_mask |= flx_frs_get_macsec_trailer(dp);
    }

    // Get netdevice (MAC) we use for sending frames to FRS
    real_netdev = dev_get_by_name(&init_net, dp->mac_name);
    if (!real_netdev) {
        dev_err(dp->this_dev, "Netdevice %s not found\n", dp->mac_name);
        dev_kfree_skb_any(skb);
        return NETDEV_TX_OK;
    }

    // Add Management trailer to frame
    if (skb->len < ETH_ZLEN) {  // Ethernet has minimum packet length
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
    flx_frs_set_skb_trailer(skb, port_mask, dp->trailer_len);

#ifdef DEBUG_FRS_MODIFY_XMIT
    netdev_printk(KERN_DEBUG, netdev, "Sending from aux with trailer 0x%x\n",
                  port_mask);
    print_hex_dump_bytes("tx_frame: ", DUMP_PREFIX_OFFSET,
                         &skb->data[0], skb->len);
#endif

    anp->stats.tx_packets++;
    anp->stats.tx_bytes += skb->len;

    // Transmit a buffer
    flx_frs_xmit(real_netdev, skb);

free_real_netdev:
    dev_put(real_netdev);

    return NETDEV_TX_OK;
}

/**
 * Add FRS port to auxiliary FRS netdevice.
 * @param netdev Auxiliary FRS netdevice.
 * @param ifindex FRS port interface index to add, will be checked.
 * @return error code
 */
static int flx_frs_aux_add_port(struct net_device *netdev,
                                unsigned int ifindex)
{
    struct flx_frs_aux_netdev_priv *anp = netdev_priv(netdev);
    struct net_device *port_netdev = NULL;
    struct flx_frs_netdev_priv *np = NULL;
    struct flx_frs_port_priv *pp = NULL;
    struct flx_frs_aux_port *app = NULL;
    int ret = -EINVAL;

    // We know netdev is an FRS auxiliary netdevice.

    netdev_dbg(netdev, "%s(): Add interface index %u\n", __func__, ifindex);

    port_netdev = dev_get_by_index(&init_net, ifindex);
    if (!port_netdev)
        return -ENODEV;
    if (!flx_frs_is_port(port_netdev))
        goto out;

    np = netdev_priv(port_netdev);
    pp = np->port_priv;

    // All ports must use the same underlying MAC towards CPU.
    if (anp->dp != pp->dp &&
        strcmp(anp->dp->mac_name, pp->dp->mac_name) != 0) {
        netdev_warn(netdev, "Port %s uses a different MAC, cannot add\n",
                    port_netdev->name);
        goto out;
    }

    // Do not allow to add CPU port.
    if (pp->flags & FLX_FRS_PORT_CPU) {
        netdev_warn(netdev, "Refusing to add CPU port %s\n",
                    port_netdev->name);
        goto out;
    }

    // Do not allow to add linked ports.
    if (pp->flags & FLX_FRS_PORT_IE) {
        netdev_warn(netdev, "Refusing to add linked port %s\n",
                    port_netdev->name);
        goto out;
    }

    list_for_each_entry(app, &anp->port_list, list) {
        if (app->ifindex == port_netdev->ifindex) {
            netdev_warn(netdev, "Ports %s is already a member\n",
                        port_netdev->name);
            goto out;
        }
    }

    app = kzalloc(sizeof(*app), GFP_KERNEL);
    if (!app) {
        netdev_err(netdev, "kmalloc failed\n");
        ret = -ENOMEM;
        goto out;
    }

    *app = (struct flx_frs_aux_port){
        .ifindex = ifindex,
        .port_mask = pp->port_mask,
    };
    INIT_LIST_HEAD(&app->list);
    list_add(&app->list, &anp->port_list);

    anp->port_mask |= app->port_mask;

    ret = sysfs_create_link(anp->ports, &port_netdev->dev.kobj,
                            port_netdev->name);
    if (ret) {
        netdev_err(netdev, "Failed to create sysfs symlink to %s\n",
                   port_netdev->name);
    }

    netdev_dbg(netdev, "Added port %s, port mask now 0x%x\n",
               port_netdev->name, anp->port_mask);

    ret = 0;

out:
    dev_put(port_netdev);

    return ret;
}

/**
 * Remove FRS port from auxiliary FRS netdevice.
 * @param netdev FRS netdevice.
 * @param ifindex Interface index of FRS port to remove.
 * @return error code
 */
static int flx_frs_aux_remove_port(struct net_device *netdev,
                                   unsigned int ifindex)
{
    struct flx_frs_aux_netdev_priv *anp = netdev_priv(netdev);
    struct net_device *port_netdev = NULL;
    struct flx_frs_netdev_priv *np = NULL;
    struct flx_frs_port_priv *pp = NULL;
    struct flx_frs_aux_port *app = NULL;
    struct flx_frs_aux_port *tmp = NULL;
    uint16_t port_mask = 0;
    bool removed = false;
    int ret = -EINVAL;

    // We know netdev is an FRS auxiliary netdevice.

    netdev_dbg(netdev, "%s(): Remove interface index %u\n", __func__, ifindex);

    port_netdev = dev_get_by_index(&init_net, ifindex);
    if (!port_netdev)
        return -ENODEV;
    if (!flx_frs_is_port(port_netdev))
        goto out;

    np = netdev_priv(port_netdev);
    pp = np->port_priv;

    // Delete port from auxiliary netdevice.
    // Must recalculate aux netdev port mask.

    list_for_each_entry_safe(app, tmp, &anp->port_list, list) {
        if (app->ifindex == port_netdev->ifindex) {
            list_del(&app->list);
            kfree(app);
            removed = true;
            continue;
        }
        port_mask |= app->port_mask;
    }

    if (!removed) {
        netdev_err(netdev, "Port %s is not a member\n",
                   port_netdev->name);
        goto out;
    }

    anp->port_mask = port_mask;

    sysfs_remove_link(anp->ports, port_netdev->name);

    netdev_dbg(netdev, "Removed port %s, port mask now 0x%x\n",
               port_netdev->name, anp->port_mask);

    ret = 0;

out:
    dev_put(port_netdev);

    return ret;
}

/**
 * Handle auxiliary FRS netdevice ioctl.
 * @param dev Netdevice.
 * @param rq Request
 * @param cmd Command
 */
static int flx_frs_aux_netdev_ioctl(struct net_device *netdev,
                                    struct ifreq *rq, int cmd)
{
    enum frs_ioctl_cmd frs_cmd = *frs_ioctl_cmd(rq);
    int ret = -EOPNOTSUPP;

    switch (cmd) {
    case SIOCDEVFRSCMD:
        switch (frs_cmd) {
        case FRS_AUX_DEV_DEL:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            ret = flx_frs_aux_remove(netdev);
            break;
        case FRS_AUX_PORT_ADD:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            ret = flx_frs_aux_add_port(netdev, frs_ioctl_data(rq)->ifindex);
            break;
        case FRS_AUX_PORT_DEL:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            ret = flx_frs_aux_remove_port(netdev, frs_ioctl_data(rq)->ifindex);
            break;
        default:
            netdev_warn(netdev, "Invalid FRS IOCTL command 0x%x\n",
                        frs_cmd);
            ret = -EINVAL;
        }
        break;
    default:
        netdev_dbg(netdev, "Unknown IOCTL command 0x%x\n", cmd);
    }

    return ret;
}

/**
 * Netdevice timeout, currently unimplemented.
 */
static void flx_frs_aux_netdev_tx_timeout(struct net_device *netdev)
{
    netdev_dbg(netdev, "%s()\n", __func__);
}

/**
 * Netdevice set RX mode.
 * Syncronizes CPU port multicast addresses to EMAC
 * and writes IPO rules for multicast handling.
 */
static void flx_frs_aux_set_rx_mode(struct net_device *netdev)
{
    netdev_dbg(netdev, "%s()\n", __func__);

    // Nothing to do.
    return;
}

/**
 * Change the MAC address, the address can be changed also on-the-fly.
 */
static int flx_frs_aux_set_mac_address(struct net_device *netdev, void *p)
{
    struct sockaddr *addr = p;

    netdev_printk(KERN_DEBUG, netdev, "Set MAC address %pM\n", addr->sa_data);

    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;

    memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

    return 0;
}

/// FRS netdevice operations
static const struct net_device_ops flx_frs_aux_netdev_ops = {
    .ndo_open = &flx_frs_aux_open,
    .ndo_start_xmit = &flx_frs_aux_start_xmit,
    .ndo_stop = &flx_frs_aux_close,
    .ndo_get_stats = &flx_frs_aux_get_stats,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
    .ndo_set_multicast_list = &flx_frs_aux_set_rx_mode,
#endif
    .ndo_do_ioctl = &flx_frs_aux_netdev_ioctl,
    .ndo_validate_addr = NULL,
    .ndo_set_rx_mode = &flx_frs_aux_set_rx_mode,
    .ndo_set_mac_address = &flx_frs_aux_set_mac_address,
    .ndo_tx_timeout = &flx_frs_aux_netdev_tx_timeout,
    .ndo_change_mtu = NULL,
};

/// Device type for auxiliary FRS netdevices
static const struct device_type flx_frs_aux_type = {
    .name = "flx_frs_aux",
};

/**
 * Auxiliary FRS netdevice early initialization function.
 */
static void flx_frs_aux_netdev_setup(struct net_device *netdev)
{
    ether_setup(netdev);

    netdev->priv_flags |= IFF_DONT_BRIDGE;
    netdev->tx_queue_len = 0;

    netdev->netdev_ops = &flx_frs_aux_netdev_ops;
    netdev->priv_destructor = free_netdev;

    SET_NETDEV_DEVTYPE(netdev, &flx_frs_aux_type);

    return;
}

/**
 * Auxiliary FRS netdevice common initialization function.
 * Netdevice is also registered.
 * @param dp FRS device privates.
 * @param netdev Unregistered auxiliary netdevice to initialize.
 */
static int flx_frs_aux_netdev_init(struct flx_frs_dev_priv *dp,
                                   struct net_device *netdev)
{
    struct net_device *real_netdev = NULL;
    struct flx_frs_aux_netdev_priv *anp = netdev_priv(netdev);
    int ret = -EINVAL;

    // Add to FRS with CPU if linked FRS.
    if (!flx_frs_dev_has_cpu_port(dp)) {
        struct flx_frs_drv_priv *drv = get_drv_priv();
        struct flx_frs_dev_priv *dp_cpu = NULL;

        if (dp->dev_num_with_cpu < FLX_FRS_MAX_DEVICES)
            dp_cpu = drv->dev_priv[dp->dev_num_with_cpu];

        if (!dp_cpu || !flx_frs_dev_has_cpu_port(dp_cpu)) {
            dev_warn(dp->this_dev, "No linked FRS with CPU port\n");
            return -ENODEV;
        }

        dp = dp_cpu;
    }

    *anp = (struct flx_frs_aux_netdev_priv) {
        .dp = dp,
        .netdev = netdev,
    };

    INIT_LIST_HEAD(&anp->port_list);

    real_netdev = dev_get_by_name(&init_net, dp->mac_name);

    if (!real_netdev) {
        dev_err(dp->this_dev, "Netdevice %s not found\n", dp->mac_name);
        return -ENODEV;
    }

    SET_NETDEV_DEV(netdev, dp->this_dev);

    netdev->addr_len = ETH_ALEN;
    memcpy(netdev->dev_addr, real_netdev->dev_addr, real_netdev->addr_len);

    netdev->base_addr = real_netdev->base_addr;
    netdev->irq = real_netdev->irq;

    netdev->netdev_ops = &flx_frs_aux_netdev_ops;
    netdev->watchdog_timeo = TX_TIMEOUT;
    netdev->flags |= (IFF_MULTICAST);   // We can handle multicast packets

    dev_put(real_netdev);

#ifdef CONFIG_FLX_FRS_NETLINK
    if (!netdev->rtnl_link_ops) {
        // Allow use of netlink API.
        netdev->rtnl_link_ops = &flx_frs_aux_netdev_rtnl_ops;
    }
#endif

    ret = register_netdevice(netdev);
    if (ret) {
        dev_err(dp->this_dev, "register_netdevice failed for %s\n",
                netdev->name);
        goto err_register_netdev;
    }

    list_add(&anp->list, &dp->aux_netdev_list);
    anp->ports = kobject_create_and_add(FLX_FRS_AUX_PORTS_ADDR,
                                        &netdev->dev.kobj);
    if (!anp->ports) {
        netdev_warn(netdev, "Failed to create sysfs directory for ports\n");
        // Sysfs support is not required.
    }

    netif_carrier_off(netdev);

    netdev_info(netdev, "Registered new FRS aux netdevice\n");

    ret = 0;

err_register_netdev:
    return ret;
}

/**
 * Create auxiliary netdevice.
 * @param dp device private data.
 * @param name netdevice name
 * @return New netdevice.
 */
struct net_device *flx_frs_aux_add(struct flx_frs_dev_priv *dp,
                                   const char *name)
{
    struct net_device *netdev;
    struct flx_frs_aux_netdev_priv *anp = NULL;
    int ret = 0;

    pr_debug(DRV_NAME ": %s()\n", __func__);

    // Get memory for the data structures
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0)
    netdev = alloc_netdev_mq(sizeof(*anp), name,
                             &flx_frs_aux_netdev_setup, 1);
#else
    netdev = alloc_netdev_mq(sizeof(*anp), name,
                             NET_NAME_PREDICTABLE,
                             &flx_frs_aux_netdev_setup, 1);
#endif
    if (!netdev) {
        dev_warn(dp->this_dev, "cannot allocate net_device\n");
        return NULL;
    }

    anp = netdev_priv(netdev);

    ret = flx_frs_aux_netdev_init(dp, netdev);
    if (ret)
        goto free_netdev;

    return netdev;

free_netdev:
    free_netdev(netdev);

    return NULL;
}

/**
 * Remove all ports from auxiliary netdevice.
 * @param anp Auxiliary netdevice privates.
 */
static void flx_frs_aux_remove_ports(struct flx_frs_aux_netdev_priv *anp)
{
    struct flx_frs_aux_port *ap = NULL;
    struct flx_frs_aux_port *tmp = NULL;

    netdev_dbg(anp->netdev, "%s()\n", __func__);

    anp->port_mask = 0;

    list_for_each_entry_safe(ap, tmp, &anp->port_list, list) {
        list_del(&ap->list);
        kfree(ap);
    }

    list_del(&anp->list);

    return;
}

/**
 * Remove auxiliary netdevice.
 * Called with rtnl_lock held.
 * @param anp Auxiliary netdevice private data
 */
static int flx_frs_aux_remove(struct net_device *netdev)
{
    struct flx_frs_aux_netdev_priv *anp = netdev_priv(netdev);

    if (netdev->netdev_ops != &flx_frs_aux_netdev_ops) {
        pr_err(DRV_NAME ": Netdevice %s is not an FRS aux netdevice\n",
               netdev->name);
        return -EINVAL;
    }

    netdev_info(netdev, "Removing auxiliary FRS netdevice\n");

    flx_frs_aux_remove_ports(anp);

    kobject_put(anp->ports);
    anp->ports = NULL;

    // Unregister, calls destructor which is free_netdev (frees np also)
    unregister_netdevice(netdev);

    return 0;
}

/**
 * Remove all auxiliary netdevices associated with an FRS device.
 */
void flx_frs_aux_remove_all(struct flx_frs_dev_priv *dp)
{
    struct flx_frs_aux_netdev_priv *anp = NULL;
    struct flx_frs_aux_netdev_priv *tmp = NULL;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    rtnl_lock();

    list_for_each_entry_safe(anp, tmp, &dp->aux_netdev_list, list) {
        flx_frs_aux_remove(anp->netdev);
    }

    rtnl_unlock();

    return;
}

#ifdef CONFIG_FLX_FRS_NETLINK

// Netlink support for auxiliary FRS netdevices

/**
 * Custom netlink parameters for auxiliary netdevices.
 */
enum {
    IFLA_FLX_FRS_AUX_UNSPEC,    ///< zero value not used
    //IFLA_FLX_FRS_AUX_DUMMY,     ///< for testing

    IFLA_FLX_FRS_AUX_MAX,       ///< number of custom parameters
};

/**
 * Netlink policy defines custom parameter types for basic validity checks.
 */
static const struct nla_policy flx_frs_aux_policy[IFLA_FLX_FRS_AUX_MAX] = {
    [IFLA_FLX_FRS_AUX_UNSPEC] = { .type = NLA_UNSPEC },
    //[IFLA_FLX_FRS_AUX_DUMMY] = { .type = NLA_U32 },
};

/**
 * Validate custom netlink parameters.
 * @param tb Table of parameters, non-existing point to NULL.
 * @param data Parameter data.
 */
static int flx_frs_aux_netdev_validate(struct nlattr *tb[],
                                       struct nlattr *data[],
                                       struct netlink_ext_ack *extack
                                      )
{
    unsigned int i;

    pr_debug(DRV_NAME ": %s()\n", __func__);

    for (i = IFLA_FLX_FRS_AUX_UNSPEC; i < IFLA_FLX_FRS_AUX_MAX; i++) {
        pr_debug(DRV_NAME ": %s(): %i len %u %p\n",
                 __func__,
                 i,
                 tb[i] ? nla_len(tb[i]) : 0,
                 tb[i] ? nla_data(tb[i]) : NULL);
    }

    return 0;
}

/**
 * Netlink callback handler to create auxiliary netdevices.
 * @param src_net
 * @param netdev New, prepared, unregistered auxiliary netdevice.
 * @param tb Netlink parameters.
 * @param data Netlink parameter data.
 */
static int flx_frs_aux_netdev_newlink(struct net *src_net,
                                      struct net_device *netdev,
                                      struct nlattr *tb[],
                                      struct nlattr *data[],
                                      struct netlink_ext_ack *extack
                                     )
{
    struct net_device *frs_netdev = NULL;
    struct flx_frs_netdev_priv *np = NULL;
    struct flx_frs_dev_priv *dp = NULL;
    int ret = -ENODEV;

    pr_debug(DRV_NAME ": %s(): %s\n", __func__, netdev->name);

    if (!tb[IFLA_LINK]) {
        pr_err(DRV_NAME ": %s: Missing link parameter\n", netdev->name);
        return -EINVAL;
    }

    frs_netdev = __dev_get_by_index(src_net, nla_get_u32(tb[IFLA_LINK]));
    if (!frs_netdev) {
        netdev_err(netdev, "Link parameter %u not found\n",
                   nla_get_u32(tb[IFLA_LINK]));
        return -ENODEV;
    }

    if (!flx_frs_is_port(frs_netdev)) {
        netdev_notice(netdev,
                      "Link parameter %u does not specify an FRS port\n",
                      nla_get_u32(tb[IFLA_LINK]));
        return -EINVAL;
    }

    np = netdev_priv(frs_netdev);
    if (!np->port_priv)
        return -ENODEV;

    dp = np->port_priv->dp;
    if (!dp)
        return -ENODEV;

    ret = flx_frs_aux_netdev_init(dp, netdev);
    if (ret)
        return ret;

    return 0;
}

/**
 * Netlink callback handler to change auxiliary netdevices.
 * @param netdev Existing, registered auxiliary netdevice.
 * @param tb Netlink parameters.
 * @param data Netlink parameter data.
 */
static int flx_frs_aux_netdev_changelink(struct net_device *netdev,
                                         struct nlattr *tb[],
                                         struct nlattr *data[],
                                         struct netlink_ext_ack *extack
                                        )
{
    //struct flx_frs_aux_netdev_priv *anp = netdev_priv(netdev);

    netdev_dbg(netdev, "%s()\n", __func__);

    if (!data)
        return 0;

    return 0;
}

/**
 * Netlink callback handler to delete auxiliary netdevices.
 * @param netdev Existing, registered auxiliary netdevice.
 * @param head List head to pass to unregister_netdevice_queue.
 */
static void flx_frs_aux_netdev_dellink(struct net_device *netdev,
                                       struct list_head *head)
{
    struct flx_frs_aux_netdev_priv *anp = netdev_priv(netdev);

    netdev_info(netdev, "Removing auxiliary FRS netdevice\n");

    flx_frs_aux_remove_ports(anp);

    unregister_netdevice_queue(netdev, head);

    return;
}

/**
 * Netlink callback handler to get size required for custom netlink parameters.
 * @param netlink Auxiliary netdevice, not necessarily registered yet.
 */
static size_t flx_frs_aux_netdev_get_size(const struct net_device *netdev)
{
    pr_debug(DRV_NAME ": %s() %s\n", __func__, netdev->name);

    return nla_total_size(0);
}

/**
 * Netlink callback handler to fill sk_buff with custom netlink parameter
 * information.
 * @param skb sk_buff to fill.
 * @param netdev Auxiliary netdevice whose parameters to fill in skb.
 * Net device is not necessarily registered yet.
 */
static int flx_frs_aux_netdev_fill_info(struct sk_buff *skb,
                                        const struct net_device *netdev)
{
    //struct flx_frs_aux_netdev_priv *anp = netdev_priv(netdev);

    pr_debug(DRV_NAME ": %s() %s\n", __func__, netdev->name);
#if 0
    if (nla_put_u32(skb, IFLA_FLX_FRS_AUX_DUMMY, 0))
        return -EMSGSIZE;
#endif

    return 0;
}

/**
 * Auxiliary netdevice netlink operations.
 */
static struct rtnl_link_ops flx_frs_aux_netdev_rtnl_ops __read_mostly = {
    .kind = "flx_frs_aux",
    .priv_size = sizeof(struct flx_frs_aux_netdev_priv),
    .setup = &flx_frs_aux_netdev_setup,
    .maxtype = IFLA_FLX_FRS_AUX_MAX - 1,
    .policy = flx_frs_aux_policy,
    .validate = &flx_frs_aux_netdev_validate,
    .newlink = &flx_frs_aux_netdev_newlink,
    .changelink = &flx_frs_aux_netdev_changelink,
    .dellink = &flx_frs_aux_netdev_dellink,
    .get_size = &flx_frs_aux_netdev_get_size,
    .fill_info = &flx_frs_aux_netdev_fill_info,
};

/**
 * Initialize netlink support for auxiliary netdevices.
 * To be called once when ready to handle netlink requests.
 */
int flx_frs_aux_init(struct flx_frs_drv_priv *drv)
{
    int ret;

    pr_debug(DRV_NAME ":%s()\n", __func__);

    ret = rtnl_link_register(&flx_frs_aux_netdev_rtnl_ops);

    return ret;
}

/**
 * Cleanup netlink support for auxiliary netdevices.
 * To be called once before not anymore ready to handle netlink requests.
 */
void flx_frs_aux_cleanup(struct flx_frs_drv_priv *drv)
{
    pr_debug(DRV_NAME ":%s()\n", __func__);

    rtnl_link_unregister(&flx_frs_aux_netdev_rtnl_ops);

    return;
}

#else

// Netlink support disabled

int flx_frs_aux_init(struct flx_frs_drv_priv *drv)
{
    return 0;
}

void flx_frs_aux_cleanup(struct flx_frs_drv_priv *drv)
{
    return;
}

#endif

