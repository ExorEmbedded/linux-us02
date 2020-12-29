/** @file
 */

/*

   FRS Linux driver

   Copyright (C) 2007 Flexibilis Oy

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

#ifndef FLX_FRS_ETHTOOL_H
#define FLX_FRS_ETHTOOL_H

#include <linux/ethtool.h>

#include "flx_frs_types.h"

enum link_mode flx_frs_get_phy_link_mode(struct phy_device *phy);

void flx_frs_update_port_stats(struct flx_frs_port_priv *pp);

extern const struct ethtool_ops flx_frs_ethtool_ops;

#endif
