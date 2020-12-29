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

#include "flx_frs_main.h"
#include "flx_frs_types.h"
#include "flx_frs_if.h"
#include "flx_frs_iflib.h"
#include "flx_frs_aux_netdev.h"
#include "flx_frs_hw.h"
#include "flx_frs_ioctl.h"

/**
 * Iterator context for copying MAC table entries to user space.
 */
struct flx_frs_mac_table_copy_ctx {
    unsigned int count;         ///< number of entries copied
    unsigned int size;          ///< number of entries that can be copied
    struct frs_mac_table_entry __user *entries; ///< entries in user space
    int ret;                    ///< result
};

/**
 * Helper function to handle FRS specific ioctl command FRS_MAC_TABLE_READ.
 * @param dp Device privates.
 * @param rq Request.
 * @return Zero or negative error code
 */
static int flx_frs_handle_features_ioctl(struct flx_frs_dev_priv *dp,
                                         struct ifreq *rq,
                                         size_t size)
{
    void __user *features = frs_ioctl_data(rq)->features;
    unsigned long int not_copied;

    if (!access_ok(VERIFY_WRITE, features, sizeof(*features)))
        return -EFAULT;

    dev_dbg(dp->this_dev, "%s() features size %zu/%zu\n",
            __func__, size, sizeof(dp->features));

    not_copied = copy_to_user(features, &dp->features, size);
    if (not_copied)
        return -EFAULT;

    return 0;
}

/**
 * MAC table iterator function to copy entry to user space buffer.
 * @param dp Device privates
 * @param dmac FES dynamic MAC address table entry to copy
 * @param arg Iterator context
 */
static void flx_frs_copy_mac_table_entry(struct flx_frs_dev_priv *dp,
                                         struct flx_frs_dmac_entry *dmac,
                                         void *arg)
{
    struct flx_frs_mac_table_copy_ctx *ctx = arg;
    struct frs_mac_table_entry entry = { .ifindex = 0 };
    unsigned long int not_copied;

    // Stop on first error.
    if (ctx->ret)
        return;

    if (ctx->count >= ctx->size)
        return;

    if (dmac->port_num < FLX_FRS_MAX_PORTS) {
        struct flx_frs_port_priv *pp = dp->port[dmac->port_num];
        if (pp && pp->netdev && pp->netdev->ifindex > 0)
            entry.ifindex = pp->netdev->ifindex;
    }
    memcpy(entry.mac_address, dmac->mac_address, ETH_ALEN);

    not_copied = copy_to_user(&ctx->entries[ctx->count], &entry,
                              sizeof(entry));
    if (not_copied)
        ctx->ret = -EFAULT;

    ctx->count++;

    return;
}

/**
 * Helper function to handle FRS specific ioctl command FRS_MAC_TABLE_READ.
 * @param dp Device privates.
 * @param rq Request.
 * @return Zero or negative error code
 */
static int flx_frs_handle_read_mac_table_ioctl(struct flx_frs_dev_priv *dp,
                                               struct ifreq *rq)
{
    struct frs_mac_table *mac_table = &frs_ioctl_data(rq)->mac_table;
    int ret = -EINVAL;

    if (mac_table->count > 0 && mac_table->entries) {
        struct flx_frs_mac_table_copy_ctx ctx = {
            .size = mac_table->count,
            .entries = (struct frs_mac_table_entry __user *)mac_table->entries,
        };

        if (!access_ok(VERIFY_WRITE, mac_table->entries,
                       mac_table->count * sizeof(*mac_table->entries)))
            return -EFAULT;

        ret = flx_frs_get_mac_table(dp, &flx_frs_copy_mac_table_entry, &ctx);
        if (ret < 0)
            return ret;
        if (ctx.ret != 0)
            return ctx.ret;

        mac_table->count = ctx.count;
    }
    else {
        // Asking for number of entries only.
        ret = flx_frs_get_mac_table(dp, NULL, NULL);
        if (ret < 0)
            return ret;

        mac_table->count = ret;
    }

    return 0;
}

/**
 * Helper function to handle FRS specific ioctl command
 * FRS_SMAC_TABLE_CONFIG_READ.
 * @param dp Device privates.
 * @param rq Request.
 * @return Zero or negative error code.
 */
static int flx_frs_handle_read_smac_config_ioctl(struct flx_frs_dev_priv *dp,
                                                 struct ifreq *rq)
{
    struct frs_smac_table_config __user *user_cfg =
        (struct frs_smac_table_config __user *)frs_ioctl_data(rq)->smac_cfg;
    unsigned long int not_copied = 0;
    int ret = -EINVAL;

    if (!access_ok(VERIFY_WRITE, user_cfg, sizeof(*user_cfg)))
        return -EFAULT;

    ret = 0;

    mutex_lock(&dp->smac_table_lock);

    not_copied = copy_to_user(user_cfg, &dp->smac.cfg, sizeof(*user_cfg));
    if (not_copied)
        ret = -EFAULT;
    else
        ret = 0;

    mutex_unlock(&dp->smac_table_lock);

    return ret;
}

/**
 * Helper function to handle FRS specific ioctl command
 * FRS_SMAC_TABLE_CONFIG_WRITE.
 * @param dp Device privates.
 * @param rq Request.
 * @return Zero or negative error code.
 */
static int flx_frs_handle_write_smac_config_ioctl(struct flx_frs_dev_priv *dp,
                                                  struct ifreq *rq)
{
    struct frs_smac_table_config smac_cfg = { .reserved = 0 };
    struct frs_smac_table_config __user *user_cfg =
        (struct frs_smac_table_config __user *)frs_ioctl_data(rq)->smac_cfg;
    unsigned long int not_copied = 0;
    int ret = -EINVAL;

    if (!access_ok(VERIFY_READ, user_cfg, sizeof(*user_cfg)))
        return -EFAULT;

    ret = 0;

    // Copy to local memory first, for validation.
    not_copied = copy_from_user(&smac_cfg, user_cfg, sizeof(smac_cfg));
    if (not_copied)
        return -EFAULT;

    mutex_lock(&dp->smac_table_lock);

    ret = flx_frs_update_smac_config(dp, &smac_cfg);

    mutex_unlock(&dp->smac_table_lock);

    return ret;
}

/**
 * Helper function to handle FRS specific ioctl command FRS_SMAC_TABLE_READ.
 * @param dp Device privates.
 * @param rq Request.
 * @return Zero or negative error code.
 */
static int flx_frs_handle_read_smac_table_ioctl(struct flx_frs_dev_priv *dp,
                                                struct ifreq *rq)
{
    struct frs_smac_table *smac_table = &frs_ioctl_data(rq)->smac_table;
    bool enabled_only = frs_ioctl_data(rq)->cmd == FRS_SMAC_TABLE_READ_ENABLED;
    struct frs_smac_table_entry __user *user_entries =
        (struct frs_smac_table_entry __user *)smac_table->entries;
    struct frs_smac_table_entry entry = { .column = 0 };
    unsigned int count = 0;
    unsigned int i = 0;
    uint16_t row = smac_table->row;
    uint16_t column = 0;
    unsigned long int not_copied = 0;
    int ret = -EINVAL;

    if (smac_table->count == 0 || !smac_table->entries)
        return -EINVAL;

    if (row >= dp->features.smac_rows)
        return -EINVAL;

    if (!access_ok(VERIFY_WRITE, user_entries,
                   smac_table->count * sizeof(*user_entries)))
        return -EFAULT;

    ret = 0;

    mutex_lock(&dp->smac_table_lock);

    for (i = 0; i < smac_table->count; i++) {
        if (!enabled_only) {
            if (__get_user(column, &user_entries[count].column)) {
                ret = -EFAULT;
                break;
            }
        }

        if (!enabled_only || flx_frs_is_smac_used(dp, row, column)) {
            ret = flx_frs_read_smac_entry(dp, row, column, &entry);
            if (ret)
                break;

            not_copied = copy_to_user(&user_entries[count],
                                      &entry, sizeof(entry));
            if (not_copied) {
                ret = -EFAULT;
                break;
            }

            count++;
        }

        // Switch to next row after last column.
        if (++column >= FRS_SMAC_TABLE_COLS) {
            column = 0;
            row++;
        }
    }

    mutex_unlock(&dp->smac_table_lock);

    // If we got successfully something, return it successfully.
    smac_table->count = count;

    if (count > 0 && ret)
        ret = 0;

    return ret;
}

/**
 * Helper function to handle FRS specific ioctl command FRS_SMAC_TABLE_WRITE.
 * @param dp Device privates.
 * @param rq Request.
 * @return Zero or negative error code.
 */
static int flx_frs_handle_write_smac_table_ioctl(struct flx_frs_dev_priv *dp,
                                                 struct ifreq *rq)
{
    struct frs_smac_table *smac_table = &frs_ioctl_data(rq)->smac_table;
    struct frs_smac_table_entry __user *user_entries =
        (struct frs_smac_table_entry __user *)smac_table->entries;
    struct frs_smac_table_entry old_entry = { .column = 0 };
    struct frs_smac_table_entry new_entry = { .column = 0 };
    unsigned int count = 0;
    uint16_t row = 0;
    unsigned long int not_copied = 0;
    int ret = -EINVAL;

    if (smac_table->count == 0 || !smac_table->entries)
        return -EINVAL;

    if (row >= dp->features.smac_rows)
        return -EINVAL;

    if (!access_ok(VERIFY_READ, user_entries,
                   smac_table->count * sizeof(*user_entries)))
        return -EFAULT;

    ret = 0;

    mutex_lock(&dp->smac_table_lock);

    for (count = 0; count < smac_table->count; count++) {
        not_copied = copy_from_user(&new_entry, &user_entries[count],
                                    sizeof(new_entry));
        if (not_copied) {
            ret = -EFAULT;
            break;
        }

        row = flx_frs_get_smac_row(dp, &new_entry);
        ret = flx_frs_read_smac_entry(dp, row, new_entry.column, &old_entry);
        if (ret)
            break;
        ret = flx_frs_write_smac_entry(dp, row, new_entry.column, &new_entry);
        if (ret)
            break;
        flx_frs_update_smac_usage(dp, row, new_entry.column,
                                  new_entry.config, old_entry.config);

        dev_dbg(dp->this_dev, "%s() row %u col %u %s #novlan %u #vlan %u\n",
                __func__, row, new_entry.column,
                test_bit(FLX_FRS_SMAC_USED_BIT(row, new_entry.column),
                         dp->smac.used) ? "USED" : "UNUSED",
                dp->smac.col_count[new_entry.column].no_vlan,
                dp->smac.col_count[new_entry.column].vlan);

        count++;
    }

    mutex_unlock(&dp->smac_table_lock);

    // If we wrote successfully something, report it.
    smac_table->count = count;

    if (count > 0 && ret)
        ret = 0;

    return ret;
}

/**
 * Helper function to handle FRS specific ioctl command FRS_SMAC_TABLE_ADD.
 * @param dp Device privates.
 * @param rq Request.
 * @return Zero or negative error code.
 */
static int flx_frs_handle_add_smac_table_ioctl(struct flx_frs_dev_priv *dp,
                                               struct ifreq *rq)
{
    struct frs_smac_table *smac_table = &frs_ioctl_data(rq)->smac_table;
    struct frs_smac_table_entry __user *user_entries =
        (struct frs_smac_table_entry __user *)smac_table->entries;
    struct frs_smac_table_entry old_entry = { .column = 0 };
    struct frs_smac_table_entry new_entry = { .column = 0 };
    unsigned int count = 0;
    uint16_t row = 0;
    uint16_t col = 0;
    unsigned long int not_copied = 0;
    int ret = -EINVAL;

    if (smac_table->count == 0 || !smac_table->entries)
        return -EINVAL;

    if (row >= dp->features.smac_rows)
        return -EINVAL;

    if (!access_ok(VERIFY_WRITE, user_entries,
                   smac_table->count * sizeof(*user_entries)))
        return -EFAULT;

    ret = 0;

    mutex_lock(&dp->smac_table_lock);

    for (count = 0; count < smac_table->count; count++) {
        not_copied = copy_from_user(&new_entry, &user_entries[count],
                                    sizeof(new_entry));
        if (not_copied) {
            ret = -EFAULT;
            break;
        }

        ret = flx_frs_get_smac_pos(dp, &new_entry, &old_entry, &row, &col);
        if (ret)
            break;

        ret = flx_frs_write_smac_entry(dp, row, col, &new_entry);
        if (ret)
            break;
        flx_frs_update_smac_usage(dp, row, col,
                                  new_entry.config, old_entry.config);

        // Update column for user space.
        ret = put_user(col, &user_entries[count].column);
        if (ret)
            break;
    }

    mutex_unlock(&dp->smac_table_lock);

    // If we wrote successfully something, report it.
    smac_table->count = count;

    if (count > 0 && ret)
        ret = 0;

    return ret;
}

/**
 * Helper function to handle FRS specific ioctl command FRS_SMAC_TABLE_DEL.
 * @param dp Device privates.
 * @param rq Request.
 * @return Zero or negative error code.
 */
static int flx_frs_handle_del_smac_table_ioctl(struct flx_frs_dev_priv *dp,
                                               struct ifreq *rq)
{
    struct frs_smac_table *smac_table = &frs_ioctl_data(rq)->smac_table;
    struct frs_smac_table_entry __user *user_entries =
        (struct frs_smac_table_entry __user *)smac_table->entries;
    struct frs_smac_table_entry old_entry = { .column = 0 };
    struct frs_smac_table_entry new_entry = { .column = 0 };
    unsigned int count = 0;
    uint16_t row = 0;
    uint16_t col = 0;
    unsigned long int not_copied = 0;
    int ret = -EINVAL;

    if (smac_table->count == 0 || !smac_table->entries)
        return -EINVAL;

    if (row >= dp->features.smac_rows)
        return -EINVAL;

    if (!access_ok(VERIFY_READ, user_entries,
                   smac_table->count * sizeof(*user_entries)))
        return -EFAULT;

    ret = 0;

    mutex_lock(&dp->smac_table_lock);

    for (count = 0; count < smac_table->count; count++) {
        not_copied = copy_from_user(&new_entry, &user_entries[count],
                                    sizeof(new_entry));
        if (not_copied) {
            ret = -EFAULT;
            break;
        }

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

            // Zero the whole entry when removing it.
            eth_zero_addr(new_entry.mac_address);
            new_entry.config = 0;
            new_entry.fwd_mask = 0;
            new_entry.vlan = 0;

            ret = flx_frs_write_smac_entry(dp, row, col, &new_entry);
            if (ret)
                goto out;

            flx_frs_update_smac_usage(dp, row, col,
                                      new_entry.config, old_entry.config);

            break;
        }

        // It is okay if not found.
    }

out:
    mutex_unlock(&dp->smac_table_lock);

    // If we wrote successfully something, report it.
    smac_table->count = count;

    if (count > 0 && ret)
        ret = 0;

    return ret;
}

/**
 * Helper function to handle FRS specific ioctl command FRS_SMAC_TABLE_cLEAR.
 * @param dp Device privates.
 * @param rq Request.
 * @return Zero or negative error code.
 */
static int flx_frs_handle_clear_smac_table_ioctl(struct flx_frs_dev_priv *dp,
                                                 struct ifreq *rq)
{
    struct frs_smac_table *smac_table = &frs_ioctl_data(rq)->smac_table;
    struct frs_smac_table_entry old_entry = { .column = 0 };
    struct frs_smac_table_entry new_entry = { .column = 0 };
    uint16_t row = smac_table->row;
    unsigned int count = 0;
    unsigned int bits = dp->features.smac_rows * FRS_SMAC_TABLE_COLS;
    unsigned int max_count = bits;
    bool clear_all = false;
    int ret = -EINVAL;

    if (row >= dp->features.smac_rows)
        return -EINVAL;

    max_count -= row * FRS_SMAC_TABLE_COLS;

    if (smac_table->count == 0)
        return -EINVAL;

    // Do not clear more than possible.
    if (smac_table->count > max_count)
        smac_table->count = max_count;

    // Avoid reading old entries when we know the end result.
    if (smac_table->count >= bits)
        clear_all = true;

    ret = 0;

    mutex_lock(&dp->smac_table_lock);

    for (count = 0; count < smac_table->count; count++) {
        if (!clear_all) {
            ret = flx_frs_read_smac_entry(dp, row,
                                          new_entry.column, &old_entry);
            if (ret)
                break;
        }
        ret = flx_frs_write_smac_entry(dp, row,
                                       new_entry.column, &new_entry);
        if (ret)
            break;

        if (!clear_all) {
            flx_frs_update_smac_usage(dp, row, new_entry.column,
                                      new_entry.config, old_entry.config);
        }

        // Switch to next row after last column.
        if (++new_entry.column >= FRS_SMAC_TABLE_COLS) {
            new_entry.column = 0;
            row++;
        }
    }

    if (clear_all) {
        size_t bitmap_size = BITS_TO_LONGS(bits) * sizeof(*dp->smac.used);

        memset(dp->smac.used, 0, bitmap_size);
        memset(dp->smac.col_count, 0, sizeof(dp->smac.col_count));
    }

    mutex_unlock(&dp->smac_table_lock);

    // If we wrote successfully something, return it successfully.
    smac_table->count = count;

    if (count > 0 && ret)
        ret = 0;

    return ret;
}

/**
 * Helper function to handle FRS specific ioctl commands for auxiliary
 * net devices.
 * @param netdev FRS netdevice.
 * @param rq Request.
 * @param cmd FRS specific ioctl command.
 * @return error code
 */
static int flx_frs_handle_aux_dev_ioctl(struct net_device *netdev,
                                        struct flx_frs_dev_priv *dp,
                                        struct ifreq *rq,
                                        enum frs_ioctl_cmd cmd)
{
    struct frs_dev_info info = { .name = "" };
    struct frs_dev_info __user *req_info = frs_ioctl_data(rq)->dev_info;
    struct net_device *aux_netdev = NULL;
    unsigned long int not_copied = 0;
    size_t len;
    int ret = -EINVAL;

    if (!access_ok(VERIFY_READ, req_info->name,
                   sizeof(req_info->name)))
        return -EFAULT;

    not_copied = copy_from_user(&info, req_info, sizeof(info));
    if (not_copied)
        return -EFAULT;

    len = strnlen(info.name, sizeof(info.name));
    if (len >= sizeof(info.name))
        return -EINVAL;

    if (cmd == FRS_AUX_DEV_ADD) {
        aux_netdev = flx_frs_aux_add(dp, info.name);
        if (aux_netdev)
            ret = 0;
    }
    else {
        ret = -EINVAL;
    }

    return ret;
}

/**
 * Perform netdev ioctl.
 * @param dev Netdevice.
 * @param rq Request
 * @param cmd Command
 * @return Error code.
 */
int flx_frs_netdev_ioctl(struct net_device *netdev,
                         struct ifreq *rq, int cmd)
{
    struct flx_frs_netdev_priv *np = netdev_priv(netdev);
    struct flx_frs_port_priv *pp = np->port_priv;
    struct flx_frs_dev_priv *dp = flx_frs_port_to_dev(pp);
    enum frs_ioctl_cmd frs_cmd = *frs_ioctl_cmd(rq);
    struct mii_ioctl_data *data = NULL;
    int ret = -EOPNOTSUPP;

    // Note: rq will not be udpated to userspace if non-zero value is returned.

    switch (cmd) {
    case SIOCDEVFRSCMD:
        // Called with rtnl_lock held.
        switch (frs_cmd) {
        case FRS_FEATURES:
            ret = flx_frs_handle_features_ioctl(dp, rq, sizeof(dp->features));
            break;
        case FRS_FEATURES_COMPAT:
            {
                // flags .. macsec_ports
                size_t size = 2*sizeof(uint32_t) + 6*sizeof(uint16_t);
                ret = flx_frs_handle_features_ioctl(dp, rq, size);
            }
            break;
        case FRS_PORT_NUM:     // Read FRS port number
            frs_ioctl_data(rq)->port_num = pp->port_num;
            ret = 0;
            break;
        case FRS_PORT_READ:    // Read FRS port register
            data = frs_mdio(rq);
            if (data->reg_num * 2 >= FLX_FRS_PORT_IOSIZE)
                return -EINVAL;
            // Require elevated rights to access MACsec registers.
            if (data->reg_num >= PORT_REG_MACSEC_BASE &&
                data->reg_num < PORT_REG_MACSEC_BASE + 0x1000 &&
                !capable(CAP_NET_ADMIN)) {
                return -EPERM;
            }
            ret = flx_frs_read_port_reg(pp, data->reg_num);
            if (ret >= 0) {
                data->val_out = (uint16_t)ret;
                ret = 0;
            }
            break;
        case FRS_PORT_WRITE:   // Write FRS port register
            if (!capable(CAP_NET_ADMIN)) {
                return -EPERM;
            }
            data = frs_mdio(rq);
            if (data->reg_num * 2 >= FLX_FRS_PORT_IOSIZE)
                return -EINVAL;
            ret = flx_frs_write_port_reg(pp, data->reg_num, data->val_in);
            break;
        case FRS_SWITCH_READ:  // Read FRS switch register
            data = frs_mdio(rq);
            if (data->reg_num * 2 >= FLX_FRS_SWITCH_IOSIZE)
                return -EINVAL;
            // Synchronize accesses to general register
            if (data->reg_num == FRS_REG_GEN)
                mutex_lock(&dp->common_reg_lock);
            ret = flx_frs_read_switch_reg(dp, data->reg_num);
            if (ret >= 0) {
                data->val_out = (uint16_t)ret;
                ret = 0;
            }
            if (data->reg_num == FRS_REG_GEN)
                mutex_unlock(&dp->common_reg_lock);
            break;
        case FRS_SWITCH_WRITE: // Write FRS switch register
            if (!capable(CAP_NET_ADMIN)) {
                return -EPERM;
            }
            data = frs_mdio(rq);
            if (data->reg_num * 2 >= FLX_FRS_SWITCH_IOSIZE)
                return -EINVAL;
            // Synchronize accesses to general register
            if (data->reg_num == FRS_REG_GEN)
                mutex_lock(&dp->common_reg_lock);
            ret = flx_frs_write_switch_reg(dp, data->reg_num, data->val_in);
            if (data->reg_num == FRS_REG_GEN)
                mutex_unlock(&dp->common_reg_lock);
            break;
        case FRS_AUX_DEV_ADD:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            ret = flx_frs_handle_aux_dev_ioctl(netdev, dp, rq, frs_cmd);
            break;
        case FRS_PORT_SET_FWD_STATE:
            {
                // Convert to STP state.
                unsigned int stp_state = BR_STATE_DISABLED;
                switch (frs_ioctl_data(rq)->port_fwd_state) {
                case FRS_PORT_FWD_STATE_FORWARDING:
                // For compatibility.
                case FRS_PORT_FWD_STATE_AUTO:
                    stp_state = BR_STATE_FORWARDING;
                    break;
                case FRS_PORT_FWD_STATE_LEARNING:
                    stp_state = BR_STATE_LEARNING;
                    break;
                case FRS_PORT_FWD_STATE_DISABLED:
                    stp_state = BR_STATE_DISABLED;
                    break;
                default:
                    netdev_dbg(netdev, "Unknown FRS port forward state %d\n",
                               frs_ioctl_data(rq)->port_fwd_state);
                    return -EINVAL;
                }
                ret = flx_frs_set_port_stp_state(netdev, stp_state);
            }
            break;
        case FRS_MAC_TABLE_READ: // Read FRS MAC table addresses
            if (!(dp->features.flags & FLX_FRS_FEAT_MAC_TABLE))
                return -EOPNOTSUPP;
            ret = flx_frs_handle_read_mac_table_ioctl(dp, rq);
            break;
        case FRS_MAC_TABLE_CLEAR:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            if (!(dp->features.flags & FLX_FRS_FEAT_MAC_TABLE))
                return -EOPNOTSUPP;
            ret = flx_frs_clear_mac_table(pp, frs_ioctl_data(rq)->port_mask);
            break;
        case FRS_SMAC_TABLE_CONFIG_READ:
            if (dp->features.smac_rows == 0)
                return -EOPNOTSUPP;
            ret = flx_frs_handle_read_smac_config_ioctl(dp, rq);
            break;
        case FRS_SMAC_TABLE_CONFIG_WRITE:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            if (dp->features.smac_rows == 0)
                return -EOPNOTSUPP;
            ret = flx_frs_handle_write_smac_config_ioctl(dp, rq);
            break;
        case FRS_SMAC_TABLE_READ:
        case FRS_SMAC_TABLE_READ_ENABLED:
            if (dp->features.smac_rows == 0)
                return -EOPNOTSUPP;
            ret = flx_frs_handle_read_smac_table_ioctl(dp, rq);
            break;
        case FRS_SMAC_TABLE_WRITE:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            if (dp->features.smac_rows == 0)
                return -EOPNOTSUPP;
            ret = flx_frs_handle_write_smac_table_ioctl(dp, rq);
            break;
        case FRS_SMAC_TABLE_ADD:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            if (dp->features.smac_rows == 0)
                return -EOPNOTSUPP;
            ret = flx_frs_handle_add_smac_table_ioctl(dp, rq);
            break;
        case FRS_SMAC_TABLE_DEL:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            if (dp->features.smac_rows == 0)
                return -EOPNOTSUPP;
            ret = flx_frs_handle_del_smac_table_ioctl(dp, rq);
            break;
        case FRS_SMAC_TABLE_CLEAR:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            if (dp->features.smac_rows == 0)
                return -EOPNOTSUPP;
            ret = flx_frs_handle_clear_smac_table_ioctl(dp, rq);
            break;
        case FRS_SET_RX_DELAY:   // Write FRS port RX delay
            if (!capable(CAP_NET_ADMIN)) {
                return -EPERM;
            }
            ret = flx_frs_write_port_reg(pp, PORT_REG_PTP_RX_DELAY_NS,
                                         frs_ioctl_data(rq)->delay);
            if (!ret) {
                ret = flx_frs_write_port_reg(pp, PORT_REG_PTP_DELAY_NSL,
                                             frs_ioctl_data(rq)->delay);
            }
            if (!ret)
                pp->rx_delay = frs_ioctl_data(rq)->delay;
            break;
        case FRS_SET_TX_DELAY:   // Write FRS port RX delay
            if (!capable(CAP_NET_ADMIN)) {
                return -EPERM;
            }
            ret = flx_frs_write_port_reg(pp, PORT_REG_PTP_TX_DELAY_NS,
                                         frs_ioctl_data(rq)->delay);
            if (!ret)
                pp->tx_delay = frs_ioctl_data(rq)->delay;
            break;
        case FRS_SET_P2P_DELAY:   // Write FRS port P2P delay
            if (!capable(CAP_NET_ADMIN)) {
                return -EPERM;
            }
            // Add RX delay compensation to add up total delay
            frs_ioctl_data(rq)->delay += pp->rx_delay;
            ret = 0;
            if ((pp->p2p_delay & 0xffff0000) !=
                (frs_ioctl_data(rq)->delay & 0xffff0000)) {
                // If high part changes - we need to update both.
                // Reason is that new value is taken into use only
                // when PORT_REG_PTP_DELAY_NSL is updated.
                // Same is done when peer delay is set to zero.
                ret = flx_frs_write_port_reg(pp, PORT_REG_PTP_DELAY_NSH,
                                             frs_ioctl_data(rq)->delay >> 16);
            }
            if (!ret) {
                ret = flx_frs_write_port_reg(pp, PORT_REG_PTP_DELAY_NSL,
                                             frs_ioctl_data(rq)->delay);
            }
            if (!ret)
                pp->p2p_delay = frs_ioctl_data(rq)->delay;
            break;
        case FRS_POLICER_READ:
            {
                struct flx_frs_policer *policer =
                    &frs_ioctl_data(rq)->policer;
                ret = flx_frs_read_policer(pp, policer->policer_num,
                                           &policer->limit,
                                           &policer->rate_status);
            }
            break;
        case FRS_POLICER_WRITE:
            if (!capable(CAP_NET_ADMIN)) {
                return -EPERM;
            }
            else {
                const struct flx_frs_policer *policer =
                    &frs_ioctl_data(rq)->policer;
                ret = flx_frs_write_policer(pp, policer->policer_num,
                                            policer->limit,
                                            policer->rate_status);
            }
            break;
        default:
            netdev_warn(netdev, "Invalid FRS IOCTL command 0x%x\n", frs_cmd);
            return -EINVAL;
        }
        break;
    default:
        netdev_dbg(netdev, "Unknown IOCTL command 0x%x\n", cmd);
    }

    return ret;
}

