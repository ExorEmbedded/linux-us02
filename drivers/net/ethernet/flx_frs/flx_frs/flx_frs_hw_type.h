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

#ifndef FLX_FRS_HW_TYPE_H
#define FLX_FRS_HW_TYPE_H

#include <linux/phy.h>

/// Maximum number of ports in one FRS
#define FLX_FRS_MAX_PORTS 16

/// Size of FRS switch register address region
#define FLX_FRS_SWITCH_IOSIZE   0x8000
/// Size of FRS port register address region
#define FLX_FRS_PORT_IOSIZE     0x10000
/// Size of FRS port adapter register address region
#define FLX_FRS_ADAPTER_IOSIZE  0x100

// Port flags
#define FLX_FRS_PORT_ADDR_VALID         (1u << 0)       ///< Port address ok
#define FLX_FRS_ADAPTER_ADDR_VALID      (1u << 1)       ///< Adapter address ok
#define FLX_FRS_PORT_CPU                (1u << 2)       ///< CPU port
#define FLX_FRS_PORT_IE                 (1u << 3)       ///< Linked port
#define FLX_FRS_PORT_INDEPENDENT        (1u << 4)       ///< Independent port
#define FLX_FRS_PORT_SPEED_EXT          (1u << 5)       ///< Auto/ext speed sel
#define FLX_FRS_ADAPTER_SGMII_PHY_MODE  (1u << 6)       ///< SGMII in PHY mode
#define FLX_FRS_SFP_EEPROM              (1u << 7)       ///< Use SFP EEPROM
#define FLX_FRS_MSG_PTP                 (1u << 8)       ///< PTP enabled from
                                                        ///< Multicast list
#define FLX_FRS_HAS_PHY                 (1u << 9)       ///< PHY configured
#define FLX_FRS_HAS_SFP_PHY             (1u << 10)      ///< SFP PHY configured
#define FLX_FRS_HAS_SEPARATE_SFP        (1u << 11)      ///< Separate SFP port
#define FLX_FRS_HAS_MASTER              (1u << 12)      ///< Attached to master

// For sanity checks

#define FLX_FRS_MAX_PRIO_QUEUES         8       ///< limit for priority queues
#define FLX_FRS_MAX_POLICERS            4096    ///< limit for policers

/**
 * Medium types.
 */
enum flx_frs_medium_type {
    FLX_FRS_MEDIUM_NONE = 0,    ///< Indicates non used port
    FLX_FRS_MEDIUM_SFP = 1,     ///< SFP module
    FLX_FRS_MEDIUM_PHY = 2,     ///< Normal PHY
    FLX_FRS_MEDIUM_NOPHY = 5,   ///< External port, no PHY
};

/**
 * FRS port initialization data structure.
 */
struct flx_frs_port_cfg {
    const char *if_name;        ///< Interface name
    enum flx_frs_medium_type medium_type;       ///< medium type
    uint32_t baseaddr;          ///< Port register base address
    uint32_t adapter_baseaddr;  ///< Adapter base address
    uint32_t flags;             ///< Flags for driver internal use
#ifdef CONFIG_OF
    struct device_node *ext_phy_node;   ///< PHY device tree node
    struct device_node *sfp_phy_node;   ///< SFP PHY device tree node
    struct device_node *sfp_eeprom_node;        ///< SFP EEPROM I2C device node
#endif
    const char *ext_phy_id;     ///< System PHY ID of external PHY on port
    const char *sfp_eeprom_name;        ///< System SFP EEPROM device name
    phy_interface_t ext_phy_if; ///< External PHY interface
    const char *sfp_phy_id;     ///< System PHY ID of SFP PHY on port
    phy_interface_t sfp_phy_if; ///< SFP PHY interface
};

/**
 * FRS component initialisation data structure.
 * This information needs to be provided through platform_device
 * platform_data.
 */
struct flx_frs_cfg {
    const char *mac_name;       ///< MAC, connected to FRS, net_device name
    uint32_t baseaddr;          ///< Switch register base address
#ifdef CONFIG_FLX_BUS
#ifndef CONFIG_OF
    const char *flx_bus_name;   ///< Indirect register access bus name
#endif
#endif

    unsigned int num_of_ports;  ///< Number of FRS ports
    struct flx_frs_port_cfg port[FLX_FRS_MAX_PORTS];
};

/**
 * FRS dynamic MAC address table entry.
 */
struct flx_frs_dmac_entry {
    uint16_t port_num;                  ///< port number
    uint8_t mac_address[ETH_ALEN];      ///< MAC address
};

#endif
