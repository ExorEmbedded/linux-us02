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

#ifndef FLX_FRS_HW_H
#define FLX_FRS_HW_H

#include "flx_frs_types.h"
#include "flx_frs_hw_type.h"
#include "flx_frs_iflib.h"

/// SMAC usage tracking bitmap bit number of SMAC entry
#define FLX_FRS_SMAC_USED_BIT(row,col) ((row)*FRS_SMAC_TABLE_COLS + (col))

int flx_frs_init_crc40(struct flx_frs_drv_priv *drv);
void flx_frs_cleanup_crc40(struct flx_frs_drv_priv *drv);
uint16_t flx_frs_get_smac_row(struct flx_frs_dev_priv *dp,
                              const struct frs_smac_table_entry *entry);
int flx_frs_update_smac_config(struct flx_frs_dev_priv *dp,
                               const struct frs_smac_table_config *smac_cfg);

void flx_frs_update_smac_usage(struct flx_frs_dev_priv *dp,
                               uint16_t row, uint16_t col,
                               uint16_t new_config,
                               uint16_t old_config);

static inline bool flx_frs_is_smac_used(struct flx_frs_dev_priv *dp,
                                        uint16_t row, uint16_t col)
{
    return test_bit(FLX_FRS_SMAC_USED_BIT(row, col), dp->smac.used);
}

enum link_mode flx_frs_get_ext_link_mode(struct flx_frs_port_priv *pp);

int flx_frs_set_port_mode(struct net_device *netdev,
                          enum link_mode link_mode);

int flx_frs_set_port_stp_state(struct net_device *netdev, uint8_t stp_state);

int flx_frs_get_mac_table(struct flx_frs_dev_priv *dp,
                          void (*new_entry)(struct flx_frs_dev_priv *dp,
                                            struct flx_frs_dmac_entry *dmac,
                                            void *arg),
                          void *arg);
int flx_frs_clear_mac_table(struct flx_frs_port_priv *pp, uint16_t port_mask);

int flx_frs_read_smac_entry(struct flx_frs_dev_priv *dp,
                            uint16_t row, uint16_t col,
                            struct frs_smac_table_entry *entry);
int flx_frs_write_smac_entry(struct flx_frs_dev_priv *dp,
                             uint16_t row, uint16_t col,
                             const struct frs_smac_table_entry *entry);
bool flx_frs_match_smac_entries(const struct frs_smac_table_entry *entry,
                                const struct frs_smac_table_entry *ref);
int flx_frs_get_smac_pos(struct flx_frs_dev_priv *dp,
                         struct frs_smac_table_entry *entry,
                         struct frs_smac_table_entry *cur_entry,
                         uint16_t *row,
                         uint16_t *col);

int flx_frs_read_policer(struct flx_frs_port_priv *pp,
                         unsigned int policer,
                         uint16_t *limit, uint16_t *rate_status);
int flx_frs_write_policer(struct flx_frs_port_priv *pp,
                          unsigned int policer,
                          uint16_t limit, uint16_t rate);

int flx_frs_write_port_ipo(struct flx_frs_port_priv *pp,
                           uint16_t entry,
                           uint16_t flags,
                           uint16_t allow_mask,
                           uint16_t mirror_mask,
                           uint8_t priority,
                           const uint8_t addr[IFHWADDRLEN],
                           uint8_t compare_length);
#if 0
int flx_frs_read_port_ipo(struct flx_frs_port_priv *pp,
                          uint16_t entry,
                          uint16_t *flags,
                          uint16_t *allow_mask,
                          uint16_t *mirror_mask,
                          uint8_t *priority,
                          uint8_t addr[IFHWADDRLEN],
                          uint8_t *compare_length);
#endif

#endif
