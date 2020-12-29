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

#ifndef FLX_FRS_SFP_H
#define FLX_FRS_SFP_H

#include "flx_frs_types.h"

// Common SFP registers

#define FLX_FRS_SFP_REG_ID                      0x00
#define FLX_FRS_SFP_REG_EXT_ID                  0x01
#define FLX_FRS_SFP_REG_CONN                    0x02
#define FLX_FRS_SFP_REG_ETH_10G_INFINIBAND      0x03
#define FLX_FRS_SFP_REG_ESCON_SONET1            0x04
#define FLX_FRS_SFP_REG_ESCON_SONET2            0x05
#define FLX_FRS_SFP_REG_ETH                     0x06
#define FLX_FRS_SFP_REG_FIBER1                  0x07
#define FLX_FRS_SFP_REG_FIBER2                  0x08
#define FLX_FRS_SFP_REG_FIBER_MEDIA             0x09
#define FLX_FRS_SFP_REG_FIBER_SPEED             0x0a
#define FLX_FRS_SFP_REG_BR                      0x0a
#define FLX_FRS_SFP_REG_MAX_BR                  0x42

int flx_frs_init_sfp(struct flx_frs_port_priv *pp);
const char *flx_frs_sfp_type_str(enum flx_frs_sfp_type sfp);
bool flx_frs_set_sfp(struct flx_frs_port_priv *pp, enum flx_frs_sfp_type sfp);
enum flx_frs_sfp_type flx_frs_detect_sfp(struct flx_frs_port_priv *pp);
void flx_frs_cleanup_sfp(struct flx_frs_port_priv *pp);

#endif
