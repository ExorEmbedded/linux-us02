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

#ifndef FLX_FRS_MAIN_H
#define FLX_FRS_MAIN_H

#include "flx_frs_types.h"

#define DRV_NAME        "flx_frs"
#define DRV_VERSION     "2.0"

struct flx_frs_drv_priv *get_drv_priv(void);
struct flx_frs_dev_priv *get_dev_priv(int minor);

/**
 * Sets interface mode. Modifies port masks of all ports
 * according to given port mode.
 * @param port Port privates
 * @param mode Port mode
 */
void flx_frs_set_ifacemode(struct flx_frs_port_priv *port,
                           enum flx_frs_iface_mode mode);

#endif
