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

#ifndef FLX_FRS_NETDEV_H
#define FLX_FRS_NETDEV_H

#include "flx_frs_types.h"
#include "flx_frs_hw_type.h"
#include "flx_frs_iflib.h"

/// Link check interval in jiffies
#define FLX_FRS_LINK_CHECK_INTERVAL (1*HZ)

// Forward declarations
struct sk_buff;

int flx_frs_netdev_init(struct flx_frs_dev_priv *dp,
                        struct flx_frs_cfg *frs_cfg);
void flx_frs_netdev_cleanup(struct flx_frs_dev_priv *dp);
void flx_frs_rx_frame(struct flx_frs_port_priv *pp,
                      struct sk_buff *rx_frame);
int flx_frs_update_port_mode(struct net_device *netdev,
                             enum link_mode link_mode);
bool flx_frs_is_port(struct net_device *netdev);

#endif
