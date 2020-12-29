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

#ifndef FLX_FRS_AUX_NETDEV_H
#define FLX_FRS_AUX_NETDEV_H

#include "flx_frs_types.h"

struct net_device *flx_frs_aux_add(struct flx_frs_dev_priv *dp,
                                   const char *name);
void flx_frs_aux_remove_all(struct flx_frs_dev_priv *dp);

int flx_frs_aux_init(struct flx_frs_drv_priv *drv);
void flx_frs_aux_cleanup(struct flx_frs_drv_priv *drv);

#endif
