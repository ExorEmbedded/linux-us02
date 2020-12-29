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

#ifndef FLX_FRS_TYPES_H
#define FLX_FRS_TYPES_H

#include <linux/if.h>
#include <linux/netdevice.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/if_bridge.h>

#include "flx_frs_hw_type.h"
#include "flx_frs_iflib.h"
#include "flx_frs_switchdev.h"

/// Maximum number of FRS devices
#define FLX_FRS_MAX_DEVICES  8

/// Modes supported by FRS
#define FLX_FRS_ETHTOOL_SUPPORTED \
    (SUPPORTED_10baseT_Full | \
     SUPPORTED_100baseT_Full | \
     SUPPORTED_1000baseT_Full | \
     SUPPORTED_Autoneg | \
     SUPPORTED_TP | \
     SUPPORTED_MII | \
     SUPPORTED_FIBRE | \
     SUPPORTED_Autoneg)

// __devinit and friends disappeared in Linux 3.8.
#ifndef __devinit
#define __devinit
#define __devexit
#endif

struct flx_frs_drv_priv;
struct flx_frs_dev_priv;
struct flx_frs_port_priv;

/**
 * Interface mode.
 */
enum flx_frs_iface_mode {
    FLX_FRS_IFACE_SWITCH,
    FLX_FRS_IFACE_INDEPENDENT,
};

/**
 * Link mode.
 */
enum link_mode {
    LM_DOWN,
    LM_10FULL,
    LM_100FULL,
    LM_1000FULL,
};

/**
 * Recognized SFP types.
 */
enum flx_frs_sfp_type {
    FLX_FRS_SFP_NONE,           ///< None

    // Fiber
    FLX_FRS_SFP_1000BASEX,      ///< 1000Base-X or any other 1000 Mb/s fiber
    FLX_FRS_SFP_100BASEFX,      ///< 100Base-FX or any other 100 Mb/s fiber

    // Copper
    FLX_FRS_SFP_1000BASET,      ///< 1000Base-T
    FLX_FRS_SFP_100BASET,       ///< 100Base-T

    FLX_FRS_SFP_UNSUPPORTED,    ///< Unsupported or unrecognized SFP module
};

/**
 * FRS statistics directly available from FRS counters.
 * References to RMON refer to RMON statistics group etherStatsXxxx.
 */
enum {
    // Available from FRS registers
    FRS_CNT_RX_GOOD_OCTETS,
    FRS_CNT_RX_BAD_OCTETS,
    FRS_CNT_RX_UNICAST,         ///< RMON UnicastPkts
    FRS_CNT_RX_BROADCAST,       ///< RMON BroadcastPkts
    FRS_CNT_RX_MULTICAST,       ///< RMON MulticastPkts
    FRS_CNT_RX_UNDERSIZE,       ///< RMON UndersizePkts
    FRS_CNT_RX_FRAGMENT,        ///< RMON Fragments
    FRS_CNT_RX_OVERSIZE,        ///< RMON OverSizePkts
    FRS_CNT_RX_JABBER,          ///< RMON Jabbers
    FRS_CNT_RX_ERR,
    FRS_CNT_RX_CRC,             ///< RMON CRCAlignErrors
    FRS_CNT_RX_64,              ///< RMON Pkts64Octets
    FRS_CNT_RX_65_127,          ///< RMON Pkts65to127Octets
    FRS_CNT_RX_128_255,         ///< RMON Pkts128to255Octets
    FRS_CNT_RX_256_511,         ///< RMON Pkts256to511ctets
    FRS_CNT_RX_512_1023,        ///< RMON Pkts512to1023ctets
    FRS_CNT_RX_1024_1536,       ///< RMON Pkts1024to1536Octets
    FRS_CNT_RX_HSRPRP,
    FRS_CNT_RX_WRONGLAN,
    FRS_CNT_RX_DUPLICATE,
    FRS_CNT_RX_MEM_FULL_DROP,
    FRS_CNT_RX_POLICED,
    FRS_CNT_RX_MACSEC_UNTAGGED,
    FRS_CNT_RX_MACSEC_NOTSUPP,
    FRS_CNT_RX_MACSEC_UNKNOWN_SCI,
    FRS_CNT_RX_MACSEC_NOTVALID,
    FRS_CNT_RX_MACSEC_LATE,
    FRS_CNT_TX_OCTETS,
    FRS_CNT_TX_UNICAST,
    FRS_CNT_TX_BROADCAST,
    FRS_CNT_TX_MULTICAST,
    FRS_CNT_TX_HSRPRP,
    FRS_CNT_TX_PRIQ_DROP,
    FRS_CNT_TX_EARLY_DROP,

    /// Number of statistics available from FRS counters
    FRS_CNT_REG_COUNT,
};

/**
 * FRS netdevice privates structure.
 */
struct flx_frs_netdev_priv {
    struct flx_frs_port_priv *port_priv;        ///< FRS port privates
    struct net_device_stats stats;      ///< Statistics
    enum link_mode link_mode;           ///< Current link mode
    enum link_mode force_link_mode;     ///< Forced link mode
    struct mutex link_mode_lock;        ///< Synchronize changes
                                        ///< Cannot use spinlock
    uint32_t msg_enable;                ///< NETIF message level
    unsigned int stp_state;             ///< port STP state, BR_STATE_xxx
#ifdef FLX_FRS_SWITCHDEV
    unsigned long int brport_flags;     ///< bridge port flags
#endif
};

/**
 * PHY device context.
 */
struct flx_frs_phy {
#ifdef CONFIG_OF
    struct device_node *node;           ///< PHY device tree node
#endif
    const char *bus_id;                 ///< id string of the PHY device
    phy_interface_t interface;          ///< PHY interface mode to use
    struct phy_device *phydev;          ///< PHY device itself
    uint32_t orig_supported;            ///< supported PHY ETHTOOL features
};

/**
 * SFP module information.
 */
struct flx_frs_sfp {
#ifdef CONFIG_OF
    struct device_node *eeprom_node;    ///< SFP EEPROM node
#endif
    const char *eeprom_name;            ///< SFP EEPROM I2C device name
    struct i2c_client *eeprom;          ///< SFP EEPROM I2C client
    enum flx_frs_sfp_type type;         ///< SFP module type
    uint32_t supported;                 ///< supported ETHTOOL features/speeds,
                                        ///< set also when SFP not in use
    struct flx_frs_phy phy;             ///< SFP PHY context
};

/**
 * Register access context.
 */
struct flx_frs_reg_access {
#ifdef CONFIG_FLX_BUS
    struct flx_bus *flx_bus;            ///< indirect register access bus
#endif
    union {
        void __iomem *ioaddr;           ///< memory mapped I/O address
        uint32_t addr;                  ///< indirect access bus address
    };
};

/**
 * FRS port adapter operations.
 */
struct flx_frs_port_adapter_ops {
    /**
     * Function to setup adapter and PHY for detected SFP type.
     */
    void (*setup) (struct flx_frs_port_priv *port);

    /**
     * Function to check link status from adapter, or NULL.
     * This is used when there is no PHY and adapter provides link status.
     */
    enum link_mode (*check_link) (struct flx_frs_port_priv *port);

    /**
     * Function to update link status to adapter, or NULL.
     * This is used when adapter needs to be informed
     * about link status changes.
     */
    void (*update_link) (struct flx_frs_port_priv *port);
};

/**
 * FRS port adapter structure.
 */
struct flx_frs_port_adapter {
    struct flx_frs_reg_access regs;     ///< register access
    uint32_t supported;                 ///< supported ETHTOOL features/speeds,
                                        ///< set also when adapter is not recognized
    uint8_t port;                       ///< current ETHTOOL port
    struct flx_frs_port_adapter_ops ops;        ///< port adapter operations
};

/**
 * FRS port private information structure.
 */
struct flx_frs_port_priv {
    struct flx_frs_dev_priv *dp;        ///< back reference
    char if_name[IFNAMSIZ];             ///< net_device name
    uint8_t port_num;                   ///< port number
    uint16_t port_mask;                 ///< management trailer for sending

#ifdef CONFIG_FLX_BUS
    struct work_struct set_rx_mode;     ///< for indirect register access
#endif
    struct delayed_work check_link;     ///< check link state periodically
    struct delayed_work capture_stats;  ///< capture statistics counters

    struct flx_frs_reg_access regs;     ///< register access
    enum flx_frs_medium_type medium_type; ///< medium type
    uint32_t flags;                     ///< port flags
    struct net_device *netdev;          ///< net_device for port

    struct flx_frs_port_adapter adapter;        ///< adapter handling state
    struct flx_frs_phy ext_phy;         ///< external PHY device context
    struct flx_frs_sfp sfp;             ///< SFP module information

    struct mutex stats_lock;            ///< synchronize statistics access
    uint64_t stats[FRS_CNT_REG_COUNT];  ///< statistics

    struct mutex port_reg_lock;         ///< synchronize port register access

    uint16_t rx_delay;                  ///< RX delay for PTP messages
    uint16_t tx_delay;                  ///< TX delay for PTP messages
    uint32_t p2p_delay;                 ///< P2P delay
};

/**
 * FRS operations for supporting different FRS register access methods.
 */
struct flx_frs_ops {
    /// Function to read FRS switch configuration register
    int (*read_switch_reg) (struct flx_frs_dev_priv *dp, int regnum);

    /// Function to write FRS switch configuration register
    int (*write_switch_reg) (struct flx_frs_dev_priv *dp, int regnum,
                             uint16_t value);

    /// Function to read FRS switch configuration register
    int (*read_port_reg) (struct flx_frs_port_priv *pp, int regnum);

    /// Function to write FRS switch configuration register
    int (*write_port_reg) (struct flx_frs_port_priv *pp, int regnum,
                           uint16_t value);

    /// Function to read FRS switch configuration register
    int (*read_adapter_reg) (struct flx_frs_port_priv *pp, int regnum);

    /// Function to write FRS switch configuration register
    int (*write_adapter_reg) (struct flx_frs_port_priv *pp, int regnum,
                              uint16_t value);
};

/**
 * Device statistics. Other that netdev stats.
 */
struct flx_frs_stats {
    uint32_t rx_stamp;          ///< PTP timestamper RX event
    uint32_t tx_stamp;          ///< PTP timestamper TX event
    uint32_t rx_error;          ///< RX error event
    uint32_t congested;         ///< Congested event
};

/**
 * Port information structure for auxiliary netdevice.
 */
struct flx_frs_aux_port {
    struct list_head list;              ///< linked list
    unsigned int ifindex;               ///< FRS port network interface index
    uint16_t port_mask;                 ///< management trailer for sending
};

/**
 * Auxiliary netdevice privates structure for sending using specific
 * management trailers.
 */
struct flx_frs_aux_netdev_priv {
    struct list_head list;              ///< linked list
    struct flx_frs_dev_priv *dp;        ///< back reference
    struct net_device *netdev;          ///< auxiliary netdevice
    uint16_t port_mask;                 ///< management trailer for sending
    struct net_device_stats stats;      ///< statistics

    struct list_head port_list;         ///< list of ports
    struct kobject *ports;              ///< sysfs directory for port symlinks
};

/**
 * FRS device private information structure.
 */
struct flx_frs_dev_priv {
    struct flx_frs_drv_priv *drv;       ///< back reference
    struct device *this_dev;            ///< pointer to (platform) device
    unsigned int dev_num;               ///< number of this device
    unsigned int irq;                   ///< IRQ number

    spinlock_t link_mask_lock;          ///< synchronize access to link_mask
    uint16_t link_mask;                 ///< bitmask of ports with link up

    unsigned int trailer_len;           ///< management trailer length
    unsigned int trailer_offset;        ///< management trailer offset

#ifdef CONFIG_FLX_BUS
    struct work_struct irq_work;        ///< for indirect register access
    atomic_t irq_disable;               ///< refcount for disabling interrupt
#endif
    struct flx_frs_reg_access regs;     ///< register access

    uint16_t cpu_port_mask;             ///< port mask of CPU port, 1 bit only
    unsigned int dev_num_with_cpu;      ///< FRS device number with CPU port
    char mac_name[IFNAMSIZ];            ///< MAC netdev name connected to FRS

    uint32_t rx_stamper_index;          ///< PTP frame timestamper RX index
#ifndef USE_FRS_TIMETRAILER
    uint32_t tx_stamper_index;          ///< PTP frame timestamper TX index
#endif

    unsigned int num_of_ports;          ///< number of FRS ports
    /// Port privates (some may be NULL)
    struct flx_frs_port_priv *port[FLX_FRS_MAX_PORTS];

    struct flx_frs_ops ops;             ///< register access operations

    struct flx_frs_switchdev switchdev; ///< switchdev support
    struct flx_frs_stats stats;         ///< IP statistics
    struct mutex common_reg_lock;       ///< synchronize switch register access
    struct mutex smac_table_lock;       ///< synchronize SMAC table access
    struct {
        struct frs_smac_table_config cfg; ///< table configuration
        unsigned long int *used;        ///< bitmap of used entries
        struct {
            unsigned int no_vlan;       ///< number of rows used without VLAN
            unsigned int vlan;          ///< number of rows used with VLAN
        } col_count[FRS_SMAC_TABLE_COLS]; ///< row usage counts
    } smac;                             ///< SMAC configuration information

    struct flx_frs_features features;   ///< switch features
    struct list_head aux_netdev_list;   ///< list of auxiliary netdevices
};

/**
 * FRS driver private information structure.
 */
struct flx_frs_drv_priv {
    struct workqueue_struct *wq;        ///< medium and indirect reg. access

    unsigned int dev_count;             ///< FRS device count
    /// Privates of found devices
    struct flx_frs_dev_priv *dev_priv[FLX_FRS_MAX_DEVICES];

    uint64_t *crc40_table;              ///< small CRC40 table (4-bit index)
};

/**
 * Get FRS port's FRS device.
 * @param pp Port privates whose FRS device to get
 * @return FRS device privates
 */
static inline struct flx_frs_dev_priv *flx_frs_port_to_dev(
        struct flx_frs_port_priv *pp)
{
    return pp->dp;
}

/**
 * Checks if an FRS device has a CPU port.
 * @param dp FRS device privates
 * @return True if device has a CPU port, false if it doesn't
 */
static inline bool flx_frs_dev_has_cpu_port(struct flx_frs_dev_priv *dp)
{
    return dp->cpu_port_mask != 0;
}

static inline int flx_frs_read_port_reg(struct flx_frs_port_priv *pp,
                                        int regnum)
{
    return flx_frs_port_to_dev(pp)->ops.read_port_reg(pp, regnum);
}

/**
 * Get management trailer with MACsec bit set for given switch.
 * @param dp FRS device privates.
 * @return Management trailer which has only the MACsec bit set.
 */
static inline uint16_t flx_frs_get_macsec_trailer(struct flx_frs_dev_priv *dp)
{
    // Most significant bit.
    return 1u << (dp->trailer_len*8 - 1);
}

/**
 * Helper to read 32-bit port counter values.
 * @param pp FRS port privates.
 * @param low_reg_num Register number of the low word of 32-bit counter.
 * @return 32-bit counter value or 0xffffffff on error.
 */
uint32_t flx_frs_read_port_counter(struct flx_frs_port_priv *pp,
                                   int low_reg_num);

static inline int flx_frs_write_port_reg(struct flx_frs_port_priv *pp,
                                         int regnum, uint16_t value)
{
    return flx_frs_port_to_dev(pp)->ops.write_port_reg(pp, regnum, value);
}

static inline int flx_frs_read_adapter_reg(struct flx_frs_port_priv *pp,
                                           int regnum)
{
    return flx_frs_port_to_dev(pp)->ops.read_adapter_reg(pp, regnum);
}

static inline int flx_frs_write_adapter_reg(struct flx_frs_port_priv *pp,
                                            int regnum, uint16_t value)
{
    return flx_frs_port_to_dev(pp)->ops.write_adapter_reg(pp, regnum,
                                                          value);
}

static inline int flx_frs_read_switch_reg(struct flx_frs_dev_priv *dp,
                                          int regnum)
{
    return dp->ops.read_switch_reg(dp, regnum);
}

static inline int flx_frs_write_switch_reg(struct flx_frs_dev_priv *dp,
                                           int regnum, uint16_t value)
{
    return dp->ops.write_switch_reg(dp, regnum, value);
}

#endif
