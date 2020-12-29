/** @file
 */

/*

   FRS Linux driver

   Copyright (C) 2016 Flexibilis Oy

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

/*
 * NOTE: None of these will be available without CONFIG_NET_SWITCHDEV.
 * Do not put any function here which is needed without it.
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
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/if_bridge.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/version.h>
#include <net/switchdev.h>
#include <net/netlink.h>

#include "flx_frs_main.h"
#include "flx_frs_types.h"
#include "flx_frs_if.h"
#include "flx_frs_hw.h"
#include "flx_frs_netdev.h"
#include "flx_frs_switchdev.h"

// CONFIG_NET_SWITCHDEV is not enough to enable driver switchdev support.
#ifdef FLX_FRS_SWITCHDEV

#define FLX_FRS_PHYS_PORT_BASE_NAME "fes"

/**
 * Uncomment if this patch has been applied to Linux < 4.4.4:
 * switchdev: Require RTNL mutex to be held when sending FDB notifications
 * It is highly recommended.
 */
#define FLX_FRS_SWITCHDEV_RTNL_FIXED

/// FDB (FES MAC address table) change notification interval in jiffies
#define FLX_FRS_SWITCHDEV_NOTIFY_FDB_INTERVAL (2*HZ)

/**
 * Convert switchdev transaction phase to string.
 */
static const char *flx_frs_switchdev_trans_str(struct switchdev_trans *trans)
{
    if (switchdev_trans_ph_prepare(trans))
        return "prepare";
    if (switchdev_trans_ph_commit(trans))
        return "commit";
    return "";
}

/**
 * Notify upper layers of switchdev events.
 * Must call with rtnl lock held.
 * RTNL lock usage changed in Linux 4.4.4.
 * This wrapper releases rtnl lock temporarily if necessary.
 */
static inline int flx_frs_switchdev_call_notifiers(
        unsigned long int val,
        struct net_device *dev,
        struct switchdev_notifier_info *info)
{
    int ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,4) && !defined(FLX_FRS_SWITCHDEV_RTNL_FIXED)
#warning "RTNL locking patch is highly recommended with Linux < 4.4.4"
#warning "Look for and apply this patch:"
#warning "switchdev: Require RTNL mutex to be held when sending FDB notifications"
#warning "and uncomment #define FLX_FRS_SWITCHDEV_RTNL_FIXED (or update kernel)."
    // Deadlock is still possible.
    rtnl_unlock();
    ret = call_switchdev_notifiers(val, dev, info);
    rtnl_lock();
#else
    ret = call_switchdev_notifiers(val, dev, info);
#endif

    return ret;
}

/**
 * Get switch ID.
 * @param dp FES device privates.
 * @param phys_id Place for switch ID.
 */
static int flx_frs_switchdev_id(struct flx_frs_dev_priv *dp,
                                struct netdev_phys_item_id *phys_id)
{
    int ret;

    // "<driver name><device number>"
    ret = snprintf(&phys_id->id[0], sizeof(phys_id->id), "%s%u",
                   DRV_NAME, dp->dev_num);
    if (ret < 0 || ret >= sizeof(phys_id->id))
        return -ENOBUFS;

    phys_id->id_len = ret;

    return 0;
}

/**
 * Get FES physical port name.
 * @param netdev FES port netdevice.
 * @param name Place for physical name.
 * @param length Number of bytes room in name.
 */
int flx_frs_switchdev_get_phys_port_name(struct net_device *netdev,
                                         char *name, size_t length)
{
    int ret = -ENOBUFS;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);

    // "<base name><device number>p<port number>"
    ret = snprintf(name, length, "%s%up%u",
                   FLX_FRS_PHYS_PORT_BASE_NAME, dp->dev_num, pp->port_num);
    if (ret < 0 || ret >= length)
        return -ENOBUFS;

    return 0;
}

/**
 * Switchdev function to get attributes.
 * @param netdev FES port netdevice.
 * @param attr Attribute to get.
 */
static int flx_frs_switchdev_port_attr_get(struct net_device *netdev,
                                           struct switchdev_attr *attr)
{
    int ret = -EOPNOTSUPP;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);

    netdev_dbg(netdev, "%s() id 0x%x\n", __func__, attr->id);

    switch (attr->id) {
    case SWITCHDEV_ATTR_ID_PORT_PARENT_ID:
        ret = flx_frs_switchdev_id(dp, &attr->u.ppid);
        break;
    case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
        mutex_lock(&np->link_mode_lock);
        attr->u.stp_state = np->stp_state;
        mutex_unlock(&np->link_mode_lock);
        ret = 0;
        break;
    case SWITCHDEV_ATTR_ID_PORT_BRIDGE_FLAGS:
        attr->u.brport_flags = np->brport_flags;
        ret = 0;
        break;
    default:
        return -EOPNOTSUPP;
    }

    return ret;
}

/**
 * Switchdev function to set attributes.
 * Called with rtnl lock held.
 * @param netdev FES port netdevice.
 * @param attr Attribute to set.
 */
static int flx_frs_switchdev_port_attr_set(struct net_device *netdev,
                                           const struct switchdev_attr *attr,
                                           struct switchdev_trans *trans)

{
    int ret = -EOPNOTSUPP;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;

    switch (attr->id) {
    case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
        netdev_dbg(netdev, "%s() set STP state to %u ph %s\n",
                   __func__, attr->u.stp_state,
                   flx_frs_switchdev_trans_str(trans));
        if (!switchdev_trans_ph_prepare(trans))
            ret = flx_frs_set_port_stp_state(netdev, attr->u.stp_state);
        else
            ret = 0;
        break;
    case SWITCHDEV_ATTR_ID_PORT_BRIDGE_FLAGS:
        netdev_dbg(netdev, "%s() set flags to 0x%lx %s\n",
                   __func__, attr->u.brport_flags,
                   flx_frs_switchdev_trans_str(trans));
        // Do not allow learning and flooding unicast on internal ports.
        if ((pp->flags & (FLX_FRS_PORT_CPU | FLX_FRS_PORT_IE)) &&
            attr->u.brport_flags & (BR_LEARNING | BR_LEARNING_SYNC | BR_FLOOD))
            return -EINVAL;
        if (!switchdev_trans_ph_prepare(trans))
            np->brport_flags = attr->u.brport_flags;
        ret = 0;
        break;
    default:
        netdev_dbg(netdev, "%s() id 0x%x not supported ph %s\n",
                   __func__, attr->id, flx_frs_switchdev_trans_str(trans));
        return -EOPNOTSUPP;
    }

    return ret;
}

/**
 * Add static FDB entry.
 * This writes an SMAC table entry, updating an existing entry
 * or using an unused entry, if possible.
 * @param netdev FES port netdevice.
 * @param fdb FDB information.
 * @param trans Switchdev transaction information.
 */
static int flx_frs_switchdev_port_fdb_add(
        struct net_device *netdev,
        const struct switchdev_obj_port_fdb *fdb,
        struct switchdev_trans *trans)
{
    int ret = -EIO;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = pp->dp;
    struct frs_smac_table_entry old_entry = { .column = 0 };
    struct frs_smac_table_entry new_entry = {
        .column = FRS_SMAC_TABLE_COLS,
        .config = FRS_SMAC_CONFIG_ENABLED,
        .fwd_mask = 1u << pp->port_num,
        .vlan = fdb->vid & VLAN_VID_MASK,
    };
    uint16_t row = 0;
    uint16_t col = FRS_SMAC_TABLE_COLS;

    if (dp->features.smac_rows == 0)
        return -EOPNOTSUPP;

    // Keep VLAN ID always zero if VLAN match is not enabled.
    if (fdb->vid)
        new_entry.config |= FRS_SMAC_CONFIG_VLAN;
    ether_addr_copy(new_entry.mac_address, fdb->addr);

    netdev_dbg(netdev, "%s() ADD %pM VID %hu %s\n",
               __func__, fdb->addr, fdb->vid,
               flx_frs_switchdev_trans_str(trans));

    mutex_lock(&dp->smac_table_lock);

    ret = flx_frs_get_smac_pos(dp, &new_entry, &old_entry, &row, &col);
    if (ret)
        goto out;

    new_entry.fwd_mask |= old_entry.fwd_mask;

    if (!switchdev_trans_ph_prepare(trans)) {
        ret = flx_frs_write_smac_entry(dp, row, col, &new_entry);
        if (ret == 0) {
            flx_frs_update_smac_usage(dp, row, col,
                                      new_entry.config, old_entry.config);
        }
    }

out:
    mutex_unlock(&dp->smac_table_lock);

    return ret;
}

/**
 * Add VLAN entries.
 * This updates FES VLAN configuration.
 * @param netdev FES port netdevice.
 * @param vlan VLAN information.
 * @param trans Switchdev transaction information.
 */
static int flx_frs_switchdev_port_vlan_add(
        struct net_device *netdev,
        const struct switchdev_obj_port_vlan *vlan,
        struct switchdev_trans *trans)
{
    int ret = -EINVAL;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = pp->dp;
    bool port_vlan_change = false;
    uint16_t port_vlan = 0;
    uint16_t port_vlan0_map = 0;
    uint16_t port_mask = 0;
    uint16_t vid = 0;

    if (!(dp->features.flags & FLX_FRS_FEAT_VLAN))
        return -EOPNOTSUPP;

    netdev_dbg(netdev, "%s() ADD %u .. %u flags 0x%x ph %s\n",
               __func__, vlan->vid_begin, vlan->vid_end, vlan->flags,
               flx_frs_switchdev_trans_str(trans));

    // VLAN membership configuration. Cannot fail.
    if (!switchdev_trans_ph_prepare(trans)) {
        for (vid = vlan->vid_begin; vid <= vlan->vid_end; vid++) {
            ret = flx_frs_read_switch_reg(dp, FRS_VLAN_CFG(vid));
            if (ret < 0)
                goto error;
            port_mask = ret;

            port_mask |= 1u << pp->port_num;

            ret = flx_frs_write_switch_reg(dp, FRS_VLAN_CFG(vid),
                                           port_mask);
            if (ret)
                goto error;
        }
    }

    // VLAN flags are BRIDGE_VLAN_INFO_xxx values.

    // Untagging tagged frames at egress configuration.
    if (vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED) {
        ret = flx_frs_read_port_reg(pp, PORT_REG_VLAN);
        if (ret < 0)
            goto error;
        port_vlan = ret;

        /*
         * FES removes VLAN tag from either only the default VLAN or
         * from all VLANs at egress.
         * VLAN ID 4095 is actually not allowed.
         */
        if (vlan->vid_begin <= 1 && vlan->vid_end >= VLAN_VID_MASK - 1) {
            port_vlan0_map = 0;
            port_vlan &= ~PORT_VLAN_TAGGED;
            port_vlan |= VLAN_VID_MASK;
        }
        else if (vlan->vid_begin == vlan->vid_end) {
            port_vlan0_map = vlan->vid_begin & VLAN_VID_MASK;
            port_vlan |= PORT_VLAN_TAGGED;
            port_vlan &= ~VLAN_VID_MASK;
            port_vlan |= port_vlan0_map;
        }
        else {
            // FES does not support removing tag from arbitrary VLANs.
            goto error;
        }

        port_vlan_change = true;
    }

    // Tagging untagged frames at ingress configuration.
    if (vlan->flags & BRIDGE_VLAN_INFO_PVID) {
        // There can be only one.
        if (vlan->vid_begin != vlan->vid_end) {
            ret = -EINVAL;
            goto error;
        }

        ret = flx_frs_read_port_reg(pp, PORT_REG_VLAN);
        if (ret < 0)
            goto error;
        port_vlan = ret;

        port_vlan0_map = vlan->vid_begin & VLAN_VID_MASK;

        port_vlan |= PORT_VLAN_TAGGED;
        port_vlan &= ~VLAN_VID_MASK;
        port_vlan |= port_vlan0_map;

        /*
         * PVID defines also VLAN ID for frames with VLAN ID 0.
         * Treat unallowed value VLAN_VID_MASK 4095 specially:
         * use FES defaults, i.e. map VLAN ID 0 to 0 (not 4095).
         */
        if (port_vlan0_map == VLAN_VID_MASK)
            port_vlan0_map = 0;

        port_vlan_change = true;
    }

    if (!switchdev_trans_ph_prepare(trans) && port_vlan_change) {
        ret = flx_frs_write_port_reg(pp, PORT_REG_VLAN, port_vlan);
        if (ret)
            goto error;

        ret = flx_frs_write_port_reg(pp, PORT_REG_VLAN0_MAP, port_vlan0_map);
        if (ret)
            goto error;
    }

    ret = 0;

error:
    return ret;
}

/**
 * Switchdev object add callback function.
 * Called with rtnl lock held.
 * @param netdev FES port netdevice.
 * @param obj Object to add.
 * @param trans Switchdev transaction information.
 */
static int flx_frs_switchdev_port_obj_add(struct net_device *netdev,
                                          const struct switchdev_obj *obj,
                                          struct switchdev_trans *trans)
{
    int ret = 0;

    switch (obj->id) {
    case SWITCHDEV_OBJ_ID_PORT_VLAN:
        ret = flx_frs_switchdev_port_vlan_add(netdev,
                                              SWITCHDEV_OBJ_PORT_VLAN(obj),
                                              trans);
        break;
    case SWITCHDEV_OBJ_ID_PORT_FDB:
        ret = flx_frs_switchdev_port_fdb_add(netdev,
                                             SWITCHDEV_OBJ_PORT_FDB(obj),
                                             trans);
        break;
    default:
        ret = -EOPNOTSUPP;
        break;
    }

    return ret;
}

/**
 * Delete static FDB entry.
 * This updates an SMAC table entry.
 * @param netdev FES port netdevice.
 * @param fdb FDB information.
 */
static int flx_frs_switchdev_port_fdb_del(
        struct net_device *netdev,
        const struct switchdev_obj_port_fdb *fdb)
{
    int ret = 0;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = pp->dp;
    uint16_t row = 0;
    struct frs_smac_table_entry old_entry = { .config = 0 };
    struct frs_smac_table_entry new_entry = {
        .config = 0,
        .vlan = fdb->vid,
    };
    unsigned int col = 0;

    if (dp->features.smac_rows == 0)
        return -EOPNOTSUPP;

    netdev_dbg(netdev, "%s() DEL %pM VID %hu\n",
               __func__, fdb->addr, fdb->vid);

    ether_addr_copy(new_entry.mac_address, fdb->addr);

    mutex_lock(&dp->smac_table_lock);

    // Find column.
    for (col = 0; col < FRS_SMAC_TABLE_COLS; col++) {
        new_entry.column = col;
        row = flx_frs_get_smac_row(dp, &new_entry);
        if (!flx_frs_is_smac_used(dp, row, col))
            continue;

        ret = flx_frs_read_smac_entry(dp, row, col, &old_entry);
        if (ret)
            goto out;

        if (!(old_entry.config & FRS_SMAC_CONFIG_ENABLED))
            continue;

        if (!flx_frs_match_smac_entries(&new_entry, &old_entry))
            continue;

        if (!(old_entry.fwd_mask & (1u << pp->port_num)))
            continue;

        // Remove for this port. If no ports are left, remove the whole entry.
        new_entry.fwd_mask &= ~(1u << pp->port_num);
        if (new_entry.fwd_mask == 0)
            new_entry.config = 0;

        // Zero the whole entry when removing it.
        if (new_entry.config == 0) {
            eth_zero_addr(new_entry.mac_address);
            new_entry.fwd_mask = 0;
            new_entry.vlan = 0;
        }

        ret = flx_frs_write_smac_entry(dp, row, col, &new_entry);
        if (ret)
            goto out;

        flx_frs_update_smac_usage(dp, row, col,
                                  new_entry.config, old_entry.config);

        break;
    }

    // It is okay if not found.

out:
    mutex_unlock(&dp->smac_table_lock);

    return ret;
}

/**
 * Add VLAN entries.
 * This updates FES VLAN configuration.
 * @param netdev FES port netdevice.
 * @param vlan VLAN information.
 * @param trans Switchdev transaction information.
 */
static int flx_frs_switchdev_port_vlan_del(
        struct net_device *netdev,
        const struct switchdev_obj_port_vlan *vlan)
{
    int ret = -EINVAL;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = pp->dp;
    uint16_t port_mask = 0;
    uint16_t vid = 0;

    if (!(dp->features.flags & FLX_FRS_FEAT_VLAN))
        return -EOPNOTSUPP;

    netdev_dbg(netdev, "%s() DEL %u .. %u flags 0x%x\n",
               __func__, vlan->vid_begin, vlan->vid_end, vlan->flags);

    // VLAN membership configuration.
    for (vid = vlan->vid_begin; vid <= vlan->vid_end; vid++) {
        ret = flx_frs_read_switch_reg(dp, FRS_VLAN_CFG(vid));
        if (ret < 0)
            goto error;
        port_mask = ret;

        port_mask &= ~(1u << pp->port_num);

        ret = flx_frs_write_switch_reg(dp, FRS_VLAN_CFG(vid), port_mask);
        if (ret)
            goto error;
    }

    // Port VLAN configuration.
    if (vlan->vid_begin == 0 && vlan->vid_end == VLAN_VID_MASK) {
        // This unalllowed combination is used internally to reset config.
        uint16_t port_vlan = 0;

        // Retain PCP bits.
        ret = flx_frs_read_port_reg(pp, PORT_REG_VLAN);
        if (ret < 0)
            goto error;
        port_vlan = ret;

        port_vlan |= PORT_VLAN_TAGGED;
        port_vlan &= ~VLAN_VID_MASK;
        port_vlan |= 1;

        ret = flx_frs_write_port_reg(pp, PORT_REG_VLAN, port_vlan);
        if (ret)
            goto error;

        ret = flx_frs_write_port_reg(pp, PORT_REG_VLAN0_MAP, 1);
        if (ret)
            goto error;
    }

    ret = 0;

error:
    if (ret)
        netdev_dbg(netdev, "%s() returning %i\n", __func__, ret);

    return ret;
}

/**
 * Switchdev object delete callback function.
 * Called with rtnl lock held.
 * @param netdev FES port netdevice.
 * @param obj Object to delete.
 */
static int flx_frs_switchdev_port_obj_del(struct net_device *netdev,
                                          const struct switchdev_obj *obj)
{
    int ret = 0;

    switch (obj->id) {
    case SWITCHDEV_OBJ_ID_PORT_VLAN:
        ret = flx_frs_switchdev_port_vlan_del(netdev,
                                              SWITCHDEV_OBJ_PORT_VLAN(obj));
        break;
    case SWITCHDEV_OBJ_ID_PORT_FDB:
        ret = flx_frs_switchdev_port_fdb_del(netdev,
                                             SWITCHDEV_OBJ_PORT_FDB(obj));
        break;
    default:
        ret = -EOPNOTSUPP;
        break;
    }

    return ret;
}

/**
 * Dump static FDB information.
 * This gets information from SMAC table.
 * @param netdev FES port netdevice.
 * @param fdb FDB information.
 * @param cb Callback function to use for dumping.
 */
static int flx_frs_switchdev_port_fdb_dump(struct net_device *netdev,
                                           struct switchdev_obj_port_fdb *fdb,
                                           switchdev_obj_dump_cb_t *cb)
{
    int ret = 0;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = pp->dp;
    struct frs_smac_table_entry entry = { .config = 0 };
    unsigned int row = 0;
    unsigned int col = 0;
    struct flx_frs_switchdev_fdb_entry *fdb_entry = NULL;
    int bkt;

    if (dp->features.smac_rows == 0)
        return -EOPNOTSUPP;

    netdev_dbg(netdev, "%s() DUMP\n", __func__);

    // Dump static FDB entries.
    mutex_lock(&dp->smac_table_lock);

    for (row = 0; row < dp->features.smac_rows; row++) {
        for (col = 0; col < FRS_SMAC_TABLE_COLS; col++) {
            if (!flx_frs_is_smac_used(dp, row, col))
                continue;

            ret = flx_frs_read_smac_entry(dp, row, col, &entry);
            if (ret)
                break;

            if (!(entry.config & FRS_SMAC_CONFIG_ENABLED))
                continue;

            if (!(entry.fwd_mask & (1u << pp->port_num)))
                continue;

            ether_addr_copy(fdb->addr, entry.mac_address);
            fdb->ndm_state = NUD_REACHABLE;
            if (entry.config & FRS_SMAC_CONFIG_VLAN)
                fdb->vid = entry.vlan;
            else
                fdb->vid = 0;

            ret = cb(&fdb->obj);
            if (ret)
                break;
        }
    }

    mutex_unlock(&dp->smac_table_lock);

    if (ret)
        return ret;

    // Dump learned FDB entries.
    hash_for_each(dp->switchdev.fdb, bkt, fdb_entry, hlist) {
        if (fdb_entry->key.port_num != pp->port_num)
            continue;

        ether_addr_copy(fdb->addr, fdb_entry->key.mac_address);
        fdb->ndm_state = NUD_REACHABLE;
        fdb->vid = 0;

        ret = cb(&fdb->obj);
        if (ret)
            break;
    }

    return ret;
}

/**
 * Dump port VLAN information.
 * @param netdev FES port netdevice.
 * @param vlan VLAN information.
 * @param cb Callback function to use for dumping.
 */
static int flx_frs_switchdev_port_vlan_dump(
        struct net_device *netdev,
        struct switchdev_obj_port_vlan *vlan,
        switchdev_obj_dump_cb_t *cb)
{
    int ret = -EIO;
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = pp->dp;
    uint16_t port_vlan = 0;
    uint16_t port_mask = 0;
    uint16_t vid = 0;

    if (!(dp->features.flags & FLX_FRS_FEAT_VLAN))
        return -EOPNOTSUPP;

    netdev_dbg(netdev, "%s() DUMP\n", __func__);

    ret = flx_frs_read_port_reg(pp, PORT_REG_VLAN);
    if (ret < 0)
        goto out;
    port_vlan = ret;

    // VLAN ID 4095 is not allowed here?
    for (vid = 1; vid < VLAN_N_VID; vid++) {
        ret = flx_frs_read_switch_reg(dp, FRS_VLAN_CFG(vid));
        if (ret < 0)
            goto out;
        port_mask = ret;

        if (!(port_mask & (1u << pp->port_num)))
            continue;

        vlan->vid_begin = vid;
        vlan->vid_end = vid;
        vlan->flags = 0;

        if (vid == (port_vlan & VLAN_VID_MASK))
            vlan->flags |= BRIDGE_VLAN_INFO_UNTAGGED | BRIDGE_VLAN_INFO_PVID;
        if (!(port_vlan & PORT_VLAN_TAGGED))
            vlan->flags |= BRIDGE_VLAN_INFO_UNTAGGED;

        ret = cb(&vlan->obj);
        if (ret) {
            netdev_dbg(netdev, "%s() cb failed at VLAN %u error %i\n",
                       __func__, vlan->vid_begin, ret);
            ret = 0;
            goto out;
        }
    }

    ret = 0;

out:
    return ret;
}

/**
 * Switchdev object dump callback function.
 * Called with rtnl lock held.
 * @param netdev FES port netdevice.
 * @param obj Object to dump.
 * @param cb Callback function to use for dumping.
 */
static int flx_frs_switchdev_port_obj_dump(struct net_device *netdev,
                                           struct switchdev_obj *obj,
                                           switchdev_obj_dump_cb_t *cb)
{
    int ret = 0;

    switch (obj->id) {
    case SWITCHDEV_OBJ_ID_PORT_VLAN:
        ret = flx_frs_switchdev_port_vlan_dump(netdev,
                                               SWITCHDEV_OBJ_PORT_VLAN(obj),
                                               cb);
        break;
    case SWITCHDEV_OBJ_ID_PORT_FDB:
        ret = flx_frs_switchdev_port_fdb_dump(netdev,
                                              SWITCHDEV_OBJ_PORT_FDB(obj),
                                              cb);
        break;
    default:
        ret = -EOPNOTSUPP;
        break;
    }

    return ret;
}

/// Switchdev operations
static const struct switchdev_ops flx_frs_switchdev_ops = {
    .switchdev_port_attr_get = &flx_frs_switchdev_port_attr_get,
    .switchdev_port_attr_set = &flx_frs_switchdev_port_attr_set,
    .switchdev_port_obj_add = &flx_frs_switchdev_port_obj_add,
    .switchdev_port_obj_del = &flx_frs_switchdev_port_obj_del,
    .switchdev_port_obj_dump = &flx_frs_switchdev_port_obj_dump,
};

/**
 * Initialize switchdev support for FES port netdevice.
 * This is called very early, netdevice is not yet operational.
 * @param netdev FES port netdevice.
 */
int flx_frs_switchdev_setup_netdev(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);

    netdev->switchdev_ops = &flx_frs_switchdev_ops;

    netdev->features |= NETIF_F_NETNS_LOCAL; // | NETIF_F_HW_VLAN_CTAG_FILTER

    np->brport_flags = BR_LEARNING | BR_LEARNING_SYNC | BR_FLOOD;

    return 0;
}

/**
 * Helper function to calculate hash for an FDB entry.
 * @param dmac FES MAC address table entry.
 * @return Hash key for FDB hash table.
 */
static inline uint32_t flx_frs_switchdev_calc_fdb_hkey(
        const struct flx_frs_dmac_entry *dmac)
{
    return crc32(~0, dmac, sizeof(*dmac));
}

/**
 * Find static FDB entry from hash table.
 * @param dp FES device privates.
 * @param match MAC address and port number of FDB entry to find.
 * @return Existing FDB entry or NULL.
 */
static struct flx_frs_switchdev_fdb_entry *flx_frs_switchdev_find_fdb(
        struct flx_frs_dev_priv *dp,
        const struct flx_frs_dmac_entry *match)
{
    struct flx_frs_switchdev_fdb_entry *fdb = NULL;
    uint32_t hkey = flx_frs_switchdev_calc_fdb_hkey(match);

    hash_for_each_possible(dp->switchdev.fdb, fdb, hlist, hkey) {
        if (memcmp(&fdb->key, match, sizeof(fdb->key)) == 0)
            return fdb;
    }

    return NULL;
}

/**
 * Check FES MAC address table entry.
 * Notifies upper layers when new entry is learned,
 * and marks existing entries as still valid.
 * @param dp FES device privates.
 * @param dmac FES dynamic MAC address table entry.
 * @param arg Notifier FDB info to use.
 */
static void flx_frs_switchdev_check_fdb_entry(struct flx_frs_dev_priv *dp,
                                              struct flx_frs_dmac_entry *dmac,
                                              void *arg)
{
    struct switchdev_notifier_fdb_info *info = arg;
    struct flx_frs_port_priv *pp = NULL;
    struct flx_frs_netdev_priv *np = NULL;
    struct flx_frs_switchdev_fdb_entry *fdb = NULL;

    dev_dbg(dp->this_dev, "%s() port %u address %pM\n",
            __func__, dmac->port_num, dmac->mac_address);

    // Ignore entries of unknown ports.
    if (dmac->port_num >= FLX_FRS_MAX_PORTS)
        return;

    pp = dp->port[dmac->port_num];
    if (!pp) {
        dev_warn(dp->this_dev, "%s() port %u unknown\n",
                 __func__, dmac->port_num);
        return;
    }

    if (!pp->netdev) {
        // This can happen when driver is unloaded.
        dev_warn(dp->this_dev, "%s() port %u netdev missing\n",
                 __func__, dmac->port_num);
        return;
    }

    if (!(pp->flags & FLX_FRS_HAS_MASTER))
        return;

    // Do not learn MAC addresses from CPU port, crashes Linux 4.4.
    if (pp->flags & FLX_FRS_PORT_CPU)
        return;
    // Do not learn MAC addresses from internal linked ports.
    if (pp->flags & FLX_FRS_PORT_IE)
        return;

    np = netdev_priv(pp->netdev);
    if (!(np->brport_flags & BR_LEARNING))
        return;

    info->addr = dmac->mac_address;
    info->vid = 0;

    fdb = flx_frs_switchdev_find_fdb(dp, dmac);

    netdev_dbg(pp->netdev, "%s() FDB %s %pM VID %hu\n",
               __func__, fdb ? "REFRESH" : "ADD", info->addr, info->vid);

    if (fdb) {
        fdb->last_seen = dp->switchdev.fdb_counter;
    }
    else {
        fdb = kmalloc(sizeof(*fdb), GFP_KERNEL);
        if (!fdb) {
            dev_err(dp->this_dev, "Failed to allocate FDB entry\n");
            return;
        }

        *fdb = (struct flx_frs_switchdev_fdb_entry){
            .hkey = flx_frs_switchdev_calc_fdb_hkey(dmac),
            .last_seen = dp->switchdev.fdb_counter,
            .key = *dmac,
        };

        hash_add(dp->switchdev.fdb, &fdb->hlist, fdb->hkey);
    }

    if (!(np->brport_flags & BR_LEARNING_SYNC))
        return;

    flx_frs_switchdev_call_notifiers(SWITCHDEV_FDB_ADD_TO_BRIDGE, pp->netdev,
                                     &info->info);

    return;
}

/**
 * FDB notifier work.
 * @param work Work within FES device privates.
 */
static void flx_frs_switchdev_notify_fdb_work(struct work_struct *work)
{
    int ret = -EIO;
    struct flx_frs_dev_priv *dp =
        container_of(work, struct flx_frs_dev_priv, switchdev.notify_fdb.work);
    struct flx_frs_drv_priv *drv = dp->drv;
    struct flx_frs_port_priv *pp = NULL;
    struct flx_frs_netdev_priv *np = NULL;
    struct switchdev_notifier_fdb_info info = {
        .vid = 0,
    };
    struct flx_frs_switchdev_fdb_entry *fdb_entry = NULL;
    struct hlist_node *tmp = NULL;
    int bkt;

    // Locking order: 1. RTNL 2. FES.
    rtnl_lock();

    dp->switchdev.fdb_counter++;

    ret = flx_frs_get_mac_table(dp, &flx_frs_switchdev_check_fdb_entry, &info);
    if (ret < 0) {
        dev_dbg(dp->this_dev, "%s() Failed to read MAC address table\n",
                __func__);
    }

    // Inform upper layers of aged entries and drop them.
    hash_for_each_safe(dp->switchdev.fdb, bkt, tmp, fdb_entry, hlist) {
        if (fdb_entry->last_seen == dp->switchdev.fdb_counter)
            continue;

        // This entry has been aged by FES.
        pp = dp->port[fdb_entry->key.port_num];

        // No notifications for nonexistent ports.
        if (pp && pp->netdev) {
            np = netdev_priv(pp->netdev);
            if (np->brport_flags & BR_LEARNING_SYNC) {
                info.addr = fdb_entry->key.mac_address;
                netdev_dbg(pp->netdev, "%s() FDB DEL %pM VID %hu\n",
                           __func__, info.addr, info.vid);
                flx_frs_switchdev_call_notifiers(SWITCHDEV_FDB_DEL_TO_BRIDGE,
                                                 pp->netdev,
                                                 &info.info);
            }
        }

        hash_del(&fdb_entry->hlist);
        kfree(fdb_entry);
    }

    rtnl_unlock();

    queue_delayed_work(drv->wq, &dp->switchdev.notify_fdb,
                       FLX_FRS_SWITCHDEV_NOTIFY_FDB_INTERVAL);

    return;
}

/**
 * Setup FES port for switchdev support when enabling it.
 * This is called when port netdevice is brought up.
 * @param netdev FES port netdevice.
 */
int flx_frs_switchdev_enable(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = pp->dp;
    struct flx_frs_drv_priv *drv = dp->drv;

    netdev_dbg(netdev, "%s()\n", __func__);

    if (dp->switchdev.enabled_port_count++ == 0) {
        if (dp->features.flags & FLX_FRS_FEAT_MAC_TABLE) {
            dev_dbg(dp->this_dev, "%s() Start polling MAC address table\n",
                    __func__);
            queue_delayed_work(drv->wq, &dp->switchdev.notify_fdb,
                               FLX_FRS_SWITCHDEV_NOTIFY_FDB_INTERVAL);
        }
    }

    return 0;
}

/**
 * Cleanup FES port for switchdev support when disabling it.
 * This is called when port netdevice is brought down.
 * @param netdev FES port netdevice.
 */
void flx_frs_switchdev_disable(struct net_device *netdev)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = pp->dp;

    netdev_dbg(netdev, "%s()\n", __func__);

    // Retain learned FDB entries as long as FES remembers them.

    if (--dp->switchdev.enabled_port_count == 0) {
        if (dp->features.flags & FLX_FRS_FEAT_MAC_TABLE) {
            dev_dbg(dp->this_dev, "%s() Stop polling MAC address table\n",
                    __func__);
            cancel_delayed_work_sync(&dp->switchdev.notify_fdb);
        }
    }

    return;
}

/**
 * Initialize port to defaults when adding it to bridge.
 * - Remove port from all VLANs.
 * - Map VLAN ID 0 to 1.
 * - Set default port VLAN ID to 1.
 * @param netdev FES port netdevice.
 * @param master Master netdevice (bridge we are joining).
 */
static int flx_frs_switchdev_join_bridge(struct net_device *netdev,
                                         struct net_device *master)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    int ret = 0;

    // Remove port from all VLANs.
    struct switchdev_obj_port_vlan vlan = {
        .vid_begin = 0,
        .vid_end = VLAN_VID_MASK,
        .flags = 0,
    };

    // Log this.
    netdev_info(netdev, "Joining %s, resetting port config\n",
                netdev_name(master));

    ret = flx_frs_switchdev_port_vlan_del(netdev, &vlan);
    if (ret)
        return ret;

    //switchdev_port_fwd_mark_set(netdev, master, true);

    // Do not allow learning and flooding unicast on internal ports.
    if (pp->flags & (FLX_FRS_PORT_CPU | FLX_FRS_PORT_IE))
        np->brport_flags = 0;
    else
        np->brport_flags = BR_LEARNING | BR_LEARNING_SYNC | BR_FLOOD;

    return ret;
}

/**
 * Initialize port to defaults when removing it from bridge.
 * This uses FES defaults, for compatibility without switchdev support.
 * - Add port to all VLANs.
 * - Map VLAN ID 0 to 0.
 * - Set default port VLAN ID to 4095.
 * @param netdev FES port netdevice.
 * @param master Master netdevice (bridge we are leaving).
 */
static int flx_frs_switchdev_leave_bridge(struct net_device *netdev,
                                          struct net_device *master)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    int ret = 0;

    // Add port to all VLANs.
    struct switchdev_obj_port_vlan vlan = {
        .vid_begin = 0,
        .vid_end = VLAN_VID_MASK,
        .flags = 0,
    };

    // Log this.
    netdev_info(netdev, "Leaving %s, restoring default port config\n",
                netdev_name(master));

    ret = flx_frs_switchdev_port_vlan_add(netdev, &vlan, NULL);
    if (ret)
        return ret;

    // Set default port VLAN to 4095 and map VLAN ID 0 to 0 (not 4095).
    vlan.vid_begin = 4095;
    vlan.vid_end = 4095;
    vlan.flags = BRIDGE_VLAN_INFO_PVID | BRIDGE_VLAN_INFO_UNTAGGED;

    ret = flx_frs_switchdev_port_vlan_add(netdev, &vlan, NULL);
    if (ret)
        return ret;

    // Restore also port STP state.
    ret = flx_frs_set_port_stp_state(netdev, BR_STATE_FORWARDING);
    if (ret)
        return ret;

    //switchdev_port_fwd_mark_set(netdev, master, false);

    np->brport_flags = 0;

    return ret;
}

/**
 * Handle netdevice notifications to detect port topology changes.
 * @param nb Notifier block.
 * @paramm event Type of notification event.
 * @param arg Notifier block type specific argument.
 * @return One of NOTIFY_xxx values.
 */
static int flx_frs_switchdev_netdev_event(struct notifier_block *nb,
                                          unsigned long int event, void *arg)
{
    struct net_device *netdev = netdev_notifier_info_to_dev(arg);
    struct netdev_notifier_changeupper_info *info = NULL;
    struct flx_frs_netdev_priv *np = NULL;
    struct flx_frs_port_priv *pp = np->port_priv;
    int ret = NOTIFY_DONE;

    if (!flx_frs_is_port(netdev))
        return NOTIFY_DONE;

    switch (event) {
    case NETDEV_CHANGEUPPER:
        info = arg;
        if (!info->master)
            goto out;
        np = netdev_priv(netdev);
        pp = np->port_priv;
        if (info->linking) {
            pp->flags |= FLX_FRS_HAS_MASTER;
            ret = flx_frs_switchdev_join_bridge(netdev, info->upper_dev);
        }
        else {
            pp->flags &= ~FLX_FRS_HAS_MASTER;
            ret = flx_frs_switchdev_leave_bridge(netdev, info->upper_dev);
        }
        if (ret)
            netdev_warn(netdev,
                        "Failed to reflect master change (error %i)\n",
                        ret);
        break;
    }

out:
    return NOTIFY_DONE;
}

/**
 * Initialize FES device for switchdev support.
 * @param dp FES device privates.
 */
int flx_frs_switchdev_init_device(struct flx_frs_dev_priv *dp)
{
    dev_dbg(dp->this_dev, "%s()\n", __func__);

    hash_init(dp->switchdev.fdb);
    INIT_DELAYED_WORK(&dp->switchdev.notify_fdb,
                      &flx_frs_switchdev_notify_fdb_work);

    return 0;
}

/**
 * Cleanup FES device switchdev support.
 * @param dp FES device privates.
 */
void flx_frs_switchdev_cleanup_device(struct flx_frs_dev_priv *dp)
{
    struct flx_frs_switchdev_fdb_entry *fdb_entry = NULL;
    struct hlist_node *tmp = NULL;
    int bkt;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    if (dp->features.flags & FLX_FRS_FEAT_MAC_TABLE) {
        dev_dbg(dp->this_dev, "%s() Stop polling MAC address table\n",
                __func__);
        cancel_delayed_work_sync(&dp->switchdev.notify_fdb);

        hash_for_each_safe(dp->switchdev.fdb, bkt, tmp, fdb_entry, hlist) {
            hash_del(&fdb_entry->hlist);
            kfree(fdb_entry);
        }
    }

    return;
}

/// Notifier block for receiving netdevice notifications
static struct notifier_block flx_frs_switchdev_netdev_nb __read_mostly = {
    .notifier_call = &flx_frs_switchdev_netdev_event,
};

/**
 * Initialize driver switchdev support.
 */
int flx_frs_switchdev_init_driver(void)
{
    int ret = -EOPNOTSUPP;

    ret = register_netdevice_notifier(&flx_frs_switchdev_netdev_nb);

    return ret;
}

/**
 * Cleanup driver switchdev support.
 */
void flx_frs_switchdev_cleanup_driver(void)
{
    unregister_netdevice_notifier(&flx_frs_switchdev_netdev_nb);

    return;
}

#endif // FLX_FRS_SWITCHDEV
