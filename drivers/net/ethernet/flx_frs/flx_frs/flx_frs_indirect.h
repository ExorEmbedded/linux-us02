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

#ifndef FLX_FRS_INDIRECT_H
#define FLX_FRS_INDIRECT_H

int __devinit flx_frs_indirect_init_device(struct flx_frs_dev_priv *dp,
                                           struct platform_device *pdev,
                                           struct flx_frs_cfg *frs_cfg);
void flx_frs_indirect_cleanup_device(struct flx_frs_dev_priv *dp);

#endif
