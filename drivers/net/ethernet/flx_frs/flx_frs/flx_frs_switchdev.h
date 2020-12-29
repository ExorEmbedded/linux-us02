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

#ifndef FLX_FRS_SWITCHDEV_H
#define FLX_FRS_SWITCHDEV_H

#include <linux/version.h>

// Early versions of switchdev are not supported
#if defined(CONFIG_NET_SWITCHDEV) && LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,0)

/// Switchdev support is enabled in driver
#define FLX_FRS_SWITCHDEV

#include <linux/hashtable.h>
#include <net/switchdev.h>

#include "flx_frs_hw_type.h"

struct flx_frs_dev_priv;

/**
 * Learned FDB entry for FES MAC table addresses.
 */
struct flx_frs_switchdev_fdb_entry {
    struct hlist_node hlist;            ///< hash list
    uint32_t hkey;                      ///< hash key
    unsigned int last_seen;             ///< last seen counter value
    struct flx_frs_dmac_entry key;      ///< dynamic MAC address table entry
};

/**
 * FES context for switchdev support.
 */
struct flx_frs_switchdev {
    unsigned int enabled_port_count;    ///< number of enabled port netdevices
    unsigned int fdb_counter;           ///< current FDB check counter
    DECLARE_HASHTABLE(fdb, 16);         ///< learned FDB entries,
                                        ///< synchronized via RTNL lock
    struct delayed_work notify_fdb;     ///< notify FDB changes
};

int flx_frs_switchdev_init_driver(void);
int flx_frs_switchdev_init_device(struct flx_frs_dev_priv *dp);
int flx_frs_switchdev_setup_netdev(struct net_device *netdev);
int flx_frs_switchdev_enable(struct net_device *netdev);
void flx_frs_switchdev_disable(struct net_device *netdev);

int flx_frs_switchdev_get_phys_port_name(struct net_device *netdev,
                                         char *name, size_t len);

void flx_frs_switchdev_cleanup_device(struct flx_frs_dev_priv *dp);
void flx_frs_switchdev_cleanup_driver(void);

# define flx_frs_switchdev_set_skb_offload_fwd_mark(skb, netdev) \
        (skb)->offload_fwd_mark = 1

#else // CONFIG_NET_SWITCHDEV

struct flx_frs_dev_priv;

/**
 * Dummy FES context for switchdev support.
 */
struct flx_frs_switchdev {
};

static inline int flx_frs_switchdev_init_driver(void)
{ return 0; }

static inline int flx_frs_switchdev_init_device(struct flx_frs_dev_priv *dp)
{ return 0; }

static inline int flx_frs_switchdev_setup_netdev(struct net_device *netdev)
{ return 0; }

static inline int flx_frs_switchdev_enable(struct net_device *netdev)
{ return 0; }

static inline void flx_frs_switchdev_disable(struct net_device *netdev)
{ }

static inline void flx_frs_switchdev_cleanup_device(
        struct flx_frs_dev_priv *dp)
{ }

static inline void flx_frs_switchdev_cleanup_driver(void)
{ }

#endif // CONFIG_NET_SWITCHDEV

#endif
