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

#ifndef FLX_FRS_IFLIB_H
#define FLX_FRS_IFLIB_H

#include <linux/mii.h>

#include "flx_frs_if.h"

/// FRS ioctl request number
#define SIOCDEVFRSCMD (SIOCDEVPRIVATE+15)

/**
 * FRS netdevice ioctl commands.
 * To use the ioctl interface:
 * - create socket, e.g. socket(AF_PACKET, SOCK_DGRAM, htons(ETH_P_ALL))
 * - create struct ifreq
 * - setup member ifrn_name to FRS netdevice name
 * - setup struct frs_ioctl_data within struct ifreq using provided helper,
 *   according to used FRS ioctl command
 * - call ioctl passing it the socket, SIOCDEVFRSCMD as request,
 *   and pointer to the struct ifreq
 * - examine return value, negative value indicates error
 * - some FRS commands return data in struct frs_ioctl_data
 */
enum frs_ioctl_cmd {
    FRS_PORT_READ,              ///< read FRS port register, uses mdio_data
    FRS_PORT_WRITE,             ///< write FRS port register, uses mdio_data
    FRS_SWITCH_READ,            ///< read FRS switch register, uses mdio_data
    FRS_SWITCH_WRITE,           ///< write FRS switch register, uses mdio_data
    FRS_PORT_NUM,               ///< get FRS port number, uses port_num
    FRS_MAC_TABLE_READ,         ///< read FRS MAC table, uses mac_table
    FRS_PORT_SET_FWD_STATE,     ///< set a port state,
                                ///< preserved over link mode changes,
                                ///< uses port_fwd_state
    FRS_AUX_DEV_ADD,            ///< create auxiliary netdevice, uses dev_info
    FRS_AUX_DEV_DEL,            ///< remove auxiliary netdevice
    FRS_AUX_PORT_ADD,           ///< add FRS port to additional netdevice,
                                ///< uses ifindex
    FRS_AUX_PORT_DEL,           ///< remove FRS port from additional netdevice,
                                ///< uses ifindex
    FRS_MAC_TABLE_CLEAR,        ///< clear MAC table entries of defined ports,
                                ///< uses port_mask
    FRS_SET_RX_DELAY,           ///< set RX (input) delay for PTP messages
    FRS_SET_TX_DELAY,           ///< set TX (output) delay for PTP messages
    FRS_SET_P2P_DELAY,          ///< set calculated P2P delay to be added to
                                ///< PTP sync message correction field calc
    FRS_FEATURES_COMPAT,        ///< get switch features (compatibility only)
    FRS_POLICER_READ,           ///< read policer
    FRS_POLICER_WRITE,          ///< write policer
    FRS_SMAC_TABLE_READ,        ///< read SMAC address table entries
    FRS_SMAC_TABLE_READ_ENABLED,///< read enabled SMAC address table entries
    FRS_SMAC_TABLE_WRITE,       ///< write SMAC address table entries,
                                ///< consider using FRS_SMAC_TABLE_ADD or
                                ///< FRS_SMAC_TABLE_DEL instead
    FRS_SMAC_TABLE_CLEAR,       ///< clear SMAC address table entries
    FRS_SMAC_TABLE_CONFIG_READ, ///< read SMAC table configuration
    FRS_SMAC_TABLE_CONFIG_WRITE, ///< write SMAC table configuration
    FRS_FEATURES,               ///< get switch features
    FRS_SMAC_TABLE_ADD,         ///< add/update SMAC address table entries
    FRS_SMAC_TABLE_DEL,         ///< remove SMAC address table entries
};

/**
 * FRS MAC address table entry.
 */
struct frs_mac_table_entry {
    unsigned int ifindex;               ///< FRS port network interface index
    uint8_t mac_address[ETH_ALEN];      ///< MAC address
};

/**
 * FRS MAC address table exchange information.
 */
struct frs_mac_table {
    /**
     * Number of entries writable in below location on input and
     * number of entries written on output. Set to zero to query
     * number of available entries.
     */
    unsigned int count;

    /**
     * Place to store MAC address table entries, or NULL to query number of
     * available entries.
     */
    struct frs_mac_table_entry *entries;
};

/**
 * SMAC table row selection algorithms.
 */
enum frs_smac_row_sel {
    FLX_FRS_SMAC_ROW_SEL_ADDR,          ///< use bits from MAC address,
                                        ///< only for FES without SMAC hash
    FLX_FRS_SMAC_ROW_SEL_NO_VLAN,       ///< XOR with VLAN ID disabled
    FLX_FRS_SMAC_ROW_SEL_VLAN,          ///< XOR with VLAN ID enabled
};

/**
 * SMAC table configuration.
 */
struct frs_smac_table_config {
    /// row selection algorithm for each column
    enum frs_smac_row_sel row_sel[FRS_SMAC_TABLE_COLS];
    uint32_t reserved;                  ///< for future
};

/**
 * FRS static MAC address table entry.
 * Use of fields depends on operation:
 * - FRS_SMAC_TABLE_READ: column is used, other fields are only written to
 * - FRS_SMAC_TABLE_READ_ENABLED: fields are only written to
 * - FRS_SMAC_TABLE_WRITE: all fields are used
 * - FRS_SMAC_TABLE_CLEAR: none of the fields are used even if present
 * - FRS_SMAC_TABLE_ADD: all fields except column are used
 * - FRS_SMAC_TABLE_DEL: config, mac_address and vlan fields are used
 */
struct frs_smac_table_entry {
    uint8_t mac_address[ETH_ALEN];      ///< MAC address
    uint16_t column;                    ///< static MAC address table column
    uint16_t config;                    ///< static MAC config register
                                        ///< (SMAC_TABLE0)
    uint16_t fwd_mask;                  ///< forward port mask
    uint16_t policed_mask;              ///< policed port mask
    uint16_t policer;                   ///< policer number
    uint16_t vlan;                      ///< VLAN number
};

/**
 * FRS static MAC address table exchange information.
 */
struct frs_smac_table {
    /**
     * Number of entries readable from or writable to entries field (below)
     * on input, and number of entries read or written on output.
     */
    unsigned int count;

    /**
     * First row number. Used when reading and clearing entries.
     * Not used when writing entries.
     */
    unsigned int row;

    /**
     * Place to store static MAC address table entries when reading,
     * or where to get static MAC table entries when writing.
     * Not used when clearing entries. Note that when clearing,
     * count specifies number of entries, not rows, and clearing starts
     * from first column on given row. Use FRS_SMAC_TABLE_DEL
     * for selective clearing.
     */
    struct frs_smac_table_entry *entries;
};

/**
 * FRS netdevice name
 */
struct frs_dev_info {
    char name[IFNAMSIZ];                ///< netdevice name to add or remove
};

/**
 * Allowed values for port forward state.
 */
enum frs_port_fwd_state_val {
    FRS_PORT_FWD_STATE_DISABLED,        ///< not forwarding, not learning
    FRS_PORT_FWD_STATE_LEARNING,        ///< learns MAC addresses
    FRS_PORT_FWD_STATE_FORWARDING,      ///< learns MAC addresses and forwards
    FRS_PORT_FWD_STATE_AUTO,            ///< identical to
                                        ///< FRS_PORT_FWD_STATE_FORWARDING
};

// FES feature flags
#define FLX_FRS_FEAT_AUTO       (1u << 0)       ///< automatically detected
#define FLX_FRS_FEAT_DEV_ID     (1u << 1)       ///< switch has ID registers
#define FLX_FRS_FEAT_GIGABIT    (1u << 2)       ///< gigabit speed
#define FLX_FRS_FEAT_STATS      (1u << 3)       ///< statistics counters
#define FLX_FRS_FEAT_VLAN       (1u << 4)       ///< VLAN support
#define FLX_FRS_FEAT_MAC_TABLE  (1u << 5)       ///< MAC address table
#define FLX_FRS_FEAT_SHAPER     (1u << 6)       ///< traffic shaper
#define FLX_FRS_FEAT_SMAC_HASH  (1u << 7)       ///< hash based SMAC, FES 4.0.1

/**
 * FES features.
 */
struct flx_frs_features {
    uint32_t flags;             ///< feature flags
    uint32_t clock_freq;        ///< clock frequency in Hz
    uint16_t smac_rows;         ///< number of static MAC address table rows
    uint16_t policers;          ///< number of policers
    uint16_t prio_queues;       ///< number of priority queues
    uint16_t hsr_ports;         ///< bitmask of HSR-capable ports
    uint16_t prp_ports;         ///< bitmask of PRP-capable ports
    uint16_t macsec_ports;      ///< bitmask of MACsec-capable ports
    uint16_t sched_ports;       ///< bitmask of scheduled ports
    uint16_t reserved[21];      ///< for future, keep total size the same
};

/**
 * FRS policer settings and status.
 * Policer rate = basic_rate x clk / 16^scale
 */
struct flx_frs_policer {
    unsigned int policer_num;   ///< policer number
    uint16_t rate_status;       ///< rate and status (POLICER1)
    uint16_t limit;             ///< max. number of tokens in bucket (POLICER0)
};

/**
 * FRS netdevice ioctl definitions.
 * This struct replaces member ifr_ifru in struct ifreq for FRS use.
 * Its size is limited, so pointers to other structs are used when needed.
 * Access contents through frs_ioctl_data helper function, defined below.
 */
struct frs_ioctl_data {
    enum frs_ioctl_cmd cmd;                     ///< command
    union {
        struct mii_ioctl_data mdio_data;        ///< read/write command data
        unsigned int port_num;                  ///< FRS port number
        struct frs_mac_table mac_table;         ///< FRS MAC table data
        struct frs_smac_table smac_table;       ///< FRS SMAC table data
        struct frs_smac_table_config *smac_cfg; ///< SMAC table configuration
        enum frs_port_fwd_state_val port_fwd_state;     ///< port forward state
        struct frs_dev_info *dev_info;          ///< FRS device information
        unsigned int ifindex;                   ///< netdevice interface index
        unsigned int port_mask;                 ///< bitmask of ports, or
                                                ///< zero for single port
        unsigned int delay;                     ///< RX/TX/P2P delay
        struct flx_frs_features *features;      ///< switch features
        struct flx_frs_policer policer;         ///< read/write policer
    };
};

// Helper function for accessing struct frs_ioctl_data members

static inline struct frs_ioctl_data *frs_ioctl_data(struct ifreq *rq)
{
    return (struct frs_ioctl_data *) &rq->ifr_ifru;
}

// Legacy helper functions

static inline struct mii_ioctl_data *frs_mdio(struct ifreq *rq)
{
    return &((struct frs_ioctl_data *) &rq->ifr_ifru)->mdio_data;
}

static inline unsigned int *frs_port_num(struct ifreq *rq)
{
    return &((struct frs_ioctl_data *) &rq->ifr_ifru)->port_num;
}

static inline enum frs_ioctl_cmd *frs_ioctl_cmd(struct ifreq *rq)
{
    return &((struct frs_ioctl_data *) &rq->ifr_ifru)->cmd;
}

static inline struct frs_mac_table *frs_ioctl_mac_table(struct ifreq *rq)
{
    return &((struct frs_ioctl_data *) &rq->ifr_ifru)->mac_table;
}

#endif
