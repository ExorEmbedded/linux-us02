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

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/version.h>
#include <linux/sched.h>

#include "flx_frs_main.h"
#include "flx_frs_types.h"
#include "flx_frs_if.h"
#include "flx_frs_iflib.h"
#include "flx_frs_netdev.h"
#include "flx_frs_netdevif.h"

// ETH_P_802_3_MIN appeared in Linux 3.10
#ifndef ETH_P_802_3_MIN
# define ETH_P_802_3_MIN 0x0600
#endif

/// Uncomment to enable FRS port resolve debug messages
//#define DEBUG_FRS_RESOLVE_PORT

#ifdef DEBUG_FRS_RESOLVE_PORT
# define dev_resolve_dbg(dev, ...) dev_printk(KERN_DEBUG, (dev), __VA_ARGS__)
#else
# define dev_resolve_dbg(dev, ...) do { } while (0)
#endif

// Local functions
static rx_handler_result_t flx_frs_handle_frame(struct sk_buff **pskb);
static struct flx_frs_port_priv *flx_frs_resolve_port(
        struct flx_frs_dev_priv *dp,
        struct sk_buff *rx_frame);
static int flx_frs_update_rx_timestamp(struct flx_frs_dev_priv *dp,
                                       struct sk_buff *rx_frame);
static int flx_frs_get_tx_timestamp(struct flx_frs_dev_priv *dp);
static inline int frs_rx_timestamp_available(struct flx_frs_dev_priv *dp);
static int frs_rx_timestamp(struct flx_frs_dev_priv *dp,
                            uint16_t rx_ts_ctrl,
                            uint8_t *hdr, uint64_t *sec, uint32_t *nsec);
#ifndef USE_FRS_TIMETRAILER
static int frs_tx_timestamp(struct flx_frs_dev_priv *dp,
                            uint8_t *hdr, uint64_t *sec, uint32_t *nsec);
#endif

// Local data
static const uint8_t ptp_peer_multicast_mac[IFHWADDRLEN] = {
    0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E
};
static const uint8_t ptp_multicast_mac[IFHWADDRLEN] = {
    0x01, 0x1B, 0x19, 0x00, 0x00, 0x00
};
static const uint8_t bpdu_multicast_mac[IFHWADDRLEN] = {
    0x01, 0x80, 0xC2, 0x00, 0x00, 0x00
};

/**
 * Gets an FRS device by a management trailer for received frame.
 * @param trailer Actual management trailer with management trailer offset
 * @return FRS device privates or NULL if port mask is zero or matching device
 * for trailer offset is not found
 */
static struct flx_frs_dev_priv *flx_frs_get_dev_priv_by_trailer(
        unsigned int trailer)
{
    struct flx_frs_drv_priv *drv = get_drv_priv();
    unsigned long int port_mask = trailer;
    unsigned int dev_trailer_offset;
    int i;

    if (trailer == 0)
        return NULL;

    dev_trailer_offset =
        (find_first_bit(&port_mask, FLX_FRS_MAX_PORTS) / 8) * 8;

    for (i = 0; i < ARRAY_SIZE(drv->dev_priv); i++) {
        struct flx_frs_dev_priv *dp = drv->dev_priv[i];
        if (!dp)
            continue;

        if (dp->trailer_offset == dev_trailer_offset)
            return dp;
    }

    return NULL;
}

/**
 * Initialize netdevif interface, to receive frames from the used Ethernet
 * interface.
 * @param dp private dev data
 * @return 0 on success ore negative Linux error code.
 */
int flx_frs_netdevif_init(struct flx_frs_dev_priv *dp)
{
    struct net_device *real_netdev = NULL;
    int ret = -ENODEV;

    dev_dbg(dp->this_dev, "Init netdevif\n");

    // Only register RX handlers for instances with CPU ports
    if (!flx_frs_dev_has_cpu_port(dp)) {
        dev_printk(KERN_DEBUG, dp->this_dev,
                   "No CPU port, not registering rx handler\n");
        return 0;
    }

    // Get netdev for MAC
    real_netdev = dev_get_by_name(&init_net, dp->mac_name);
    if (!real_netdev) {
        dev_err(dp->this_dev, "Netdevice %s not found\n", dp->mac_name);
        return -ENODEV;
    }
    // Register rx handler for MAC netdevice
    rtnl_lock();
    ret =
        netdev_rx_handler_register(real_netdev, flx_frs_handle_frame, dp);
    rtnl_unlock();

    dev_put(real_netdev);

    if (ret) {
        dev_err(dp->this_dev, "netdev_rx_handler_register failed for %s\n",
                dp->mac_name);
    }

    return ret;
}

/**
 * Close netdevif interface.
 * @param dp private dev data
 */
void flx_frs_netdevif_cleanup(struct flx_frs_dev_priv *dp)
{
    struct net_device *real_netdev = NULL;

    // Get netdev for MAC
    real_netdev = dev_get_by_name(&init_net, dp->mac_name);
    if (!real_netdev) {
        dev_err(dp->this_dev, "Netdevice %s not found\n", dp->mac_name);
        return;
    }

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    // Unregister rx handler for MAC netdevice
    // Handlers are registered for devices with a CPU port only.
    if (flx_frs_dev_has_cpu_port(dp)) {
        rtnl_lock();
        netdev_rx_handler_unregister(real_netdev);
        rtnl_unlock();
    }

    dev_put(real_netdev);

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return;
}

/**
 * Receive frames from FRS (via Ethernet MAC).
 * note: already called with rcu_read_lock
 *
 * @param skb received frame
 * @return rx_handler_result
 */
static rx_handler_result_t flx_frs_handle_frame(struct sk_buff **pskb)
{
    struct sk_buff *skb = *pskb;
    const struct net_device *dev = skb->dev;
    struct flx_frs_dev_priv *dp = NULL;
    struct flx_frs_port_priv *port = NULL;

    if (!dev) {
        // No input device
        pr_err(DRV_NAME ": No input device\n");
        goto drop;
    }
    //netdev_printk(KERN_DEBUG, dev, "input\n");

    // Device with CPU port
    dp = rcu_dereference(dev->rx_handler_data);

    if (unlikely(skb->pkt_type == PACKET_LOOPBACK))
        return RX_HANDLER_PASS;

    skb = skb_share_check(skb, GFP_ATOMIC);
    if (!skb)
        return RX_HANDLER_CONSUMED;

    // Use device with CPU port for time stamp.
    // Must call this in any case.
    if (flx_frs_update_rx_timestamp(dp, skb)) {
        // PTP event frame and timestamp not found -> DROP
        dev_err(dp->this_dev, "Timestamp not found\n");
        goto drop;
    }

    // Resolve port
    port = flx_frs_resolve_port(dp, skb);
    if (port) {
        dev_resolve_dbg(dp->this_dev,
                        "Received frame from %s device %u port %u\n",
                        netdev_name(port->netdev),
                        port->dp->dev_num, port->port_num);
        flx_frs_rx_frame(port, skb);
        return RX_HANDLER_CONSUMED;
    }

drop:
    kfree_skb(skb);

    return RX_HANDLER_CONSUMED;
}

/**
 * Transmit frame.
 * @param to Output device
 * @param skb frame to send.
 */
void flx_frs_xmit(struct net_device *to, struct sk_buff *skb)
{
    // Set new destination
    skb->dev = to;
    dev_queue_xmit(skb);

    return;
}

/**
 * Handle FRS interrupts.
 * This may be called from HW interrupt handler or from interrupt work.
 * @param irq IRQ number.
 * @param device Device data.
 * @return IRQ return value.
 */
static irqreturn_t flx_frs_handle_interrupt(struct flx_frs_dev_priv *dp)
{
    int intstat = flx_frs_read_switch_reg(dp, FRS_REG_INTSTAT);
    int ret = IRQ_NONE;

    intstat = flx_frs_read_switch_reg(dp, FRS_REG_INTSTAT);
    if (intstat < 0)
        return IRQ_NONE;

    flx_frs_write_switch_reg(dp, FRS_REG_INTSTAT, ~intstat);

    if (flx_frs_dev_has_cpu_port(dp)) {
        if (intstat & FRS_INT_RX_TSTAMP) {
            // PTP event frame sent to FRS
            dp->stats.rx_stamp++;
            ret = IRQ_HANDLED;
            flx_frs_get_tx_timestamp(dp);
        }
        if (intstat & FRS_INT_TX_TSTAMP) {
            // PTP event frame received from FRS
            dp->stats.tx_stamp++;
            ret = IRQ_HANDLED;
        }
    }
    if (intstat & FRS_INT_RX_ERROR) {
        // RX error happened
        dp->stats.rx_error++;
        ret = IRQ_HANDLED;
    }
    if (intstat & FRS_INT_CONGESTED) {
        // Congested situation
        dp->stats.congested++;
        ret = IRQ_HANDLED;
    }

    return ret;
}

#ifdef CONFIG_FLX_BUS
/**
 * FRS interrupt work handler.
 * This is used to deal with FRS interrupts
 * when FRS registers are accessed indirectly.
 */
static void flx_frs_interrupt_work(struct work_struct *work)
{
    struct flx_frs_dev_priv *dp =
        container_of(work, struct flx_frs_dev_priv, irq_work);

    // Disable interrupts from FRS until we have handled them.
    flx_frs_write_switch_reg(dp, FRS_REG_INTMASK, 0);

    // Let others use the interrupt line.
    atomic_dec(&dp->irq_disable);
    enable_irq(dp->irq);

    flx_frs_handle_interrupt(dp);

    // Reenable interrupts from FRS.
    flx_frs_write_switch_reg(dp, FRS_REG_INTMASK,
                             FRS_INT_RX_TSTAMP | FRS_INT_TX_TSTAMP);

    return;
}

/**
 * FRS interrupt handler for indirect register access.
 * Kicks a work to deal with the interrupt.
 * @param irq IRQ number.
 * @param device Device data.
 * @return IRQ return value.
 */
static irqreturn_t flx_frs_interrupt_indirect(int irq, void *device)
{
    struct flx_frs_dev_priv *dp = device;

    /*
     * Cannot access registers in interrupt context.
     * Disable interrupt and kick work to handle interrupt
     * and to reenable interrupt again.
     */
    disable_irq_nosync(dp->irq);
    atomic_inc(&dp->irq_disable);

    queue_work(dp->drv->wq, &dp->irq_work);

    return IRQ_HANDLED;
}
#endif

/**
 * FRS interrupt handler.
 * @param irq IRQ number.
 * @param device Device data.
 * @return IRQ return value.
 */
static irqreturn_t flx_frs_interrupt(int irq, void *device)
{
    struct flx_frs_dev_priv *dp = device;

    return flx_frs_handle_interrupt(dp);
}

/**
 * Initialize FRS interrupt handling.
 * @param dp FRS device privates.
 */
int __devinit flx_frs_irq_init(struct flx_frs_dev_priv *dp)
{
    int ret;
    bool indirect = false;

    // Allow operation without IRQ.
    if (!dp->irq)
        return 0;

    // We are interested in interrupts only if it has a CPU port.
    if (!flx_frs_dev_has_cpu_port(dp))
        return 0;

#ifdef CONFIG_FLX_BUS
    if (dp->regs.flx_bus) {
        INIT_WORK(&dp->irq_work, &flx_frs_interrupt_work);
        atomic_set(&dp->irq_disable, 0);
        ret = request_irq(dp->irq, &flx_frs_interrupt_indirect, IRQF_SHARED,
                          DRV_NAME, dp);

        indirect = true;
    }
#endif
    if (!indirect) {
        ret = request_irq(dp->irq, &flx_frs_interrupt, IRQF_SHARED,
                          DRV_NAME, dp);
    }
    if (ret) {
        dev_err(dp->this_dev, "unable to allocate IRQ %u, error 0x%x",
                dp->irq, ret);
        return ret;
    }

    dev_printk(KERN_DEBUG, dp->this_dev, "FRS IRQ %i allocated\n", dp->irq);

    return 0;
}

/**
 * Cleanup FRS interrupt handling.
 * @param dp FRS device privates.
 */
void flx_frs_irq_cleanup(struct flx_frs_dev_priv *dp)
{
    bool indirect = false;

    // Allow operation without IRQ.
    if (!dp->irq)
        return;

    if (!flx_frs_dev_has_cpu_port(dp))
        return;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

#ifdef CONFIG_FLX_BUS
    if (dp->regs.flx_bus) {

        // Disable interrupt from FRS. First ensure interrupt will not fire.
        disable_irq(dp->irq);
        atomic_inc(&dp->irq_disable);

        cancel_work_sync(&dp->irq_work);
        flx_frs_write_switch_reg(dp, FRS_REG_INTMASK, 0);
        flx_frs_write_switch_reg(dp, FRS_REG_INTSTAT, 0);

        free_irq(dp->irq, dp);

        // Ensure disable_irq() will be left balanced.
        while (atomic_dec_return(&dp->irq_disable) >= 0) {
            enable_irq(dp->irq);
        }

        indirect = true;
    }
#endif

    if (!indirect) {
        flx_frs_write_switch_reg(dp, FRS_REG_INTMASK, 0);
        free_irq(dp->irq, dp);
    }

    // Just in case.
    flx_frs_write_switch_reg(dp, FRS_REG_INTMASK, 0);
    flx_frs_write_switch_reg(dp, FRS_REG_INTSTAT, 0);

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return;
}

/**
 * Get management trailer from an sk_buff, where it appears big-endian.
 * @param skb sk_buff with an Ethernet frame to get management trailer from.
 * @param trailer_len Number of octets in management trailer.
 * @return Management trailer.
 */
static inline unsigned int flx_frs_get_skb_trailer(struct sk_buff *skb,
                                                   unsigned int trailer_len)
{
    const uint8_t *skb_trailer = skb_tail_pointer(skb) - trailer_len;

    if (trailer_len == 1) {
        return (unsigned int) skb_trailer[0];
    }
    else {
        return
            ((unsigned int) skb_trailer[0] << 8) |
            ((unsigned int) skb_trailer[1] << 0);
    }

    return 0;
}

/**
 * Function to resolve receive port. Resolves correct port by using
 * the rx_frame management trailer. May modify rx_frame, e.g. remove trailer.
 * @param dp FRS private data with CPU port from which frame was received.
 * @param rx_frame Received frame.
 * @return Port from which to pass the frame to networking stack,
 * or NULL if frame should be dropped
 */
static struct flx_frs_port_priv *flx_frs_resolve_port(
        struct flx_frs_dev_priv *dp,
        struct sk_buff *rx_frame)
{
    struct flx_frs_dev_priv *dp_recv = NULL;
    struct flx_frs_port_priv *port = NULL;
    unsigned int trailer = 0;
    unsigned long int port_mask = 0;
    unsigned char *data = skb_mac_header(rx_frame);
    struct ethhdr *eth = eth_hdr(rx_frame);
    unsigned int port_num = 0;

    if (!data) {
        dev_err(dp->this_dev, "No data\n");
        return NULL;
    }

#ifdef DEBUG_FRS_RESOLVE_PORT
    print_hex_dump_bytes("rx_frame: ", DUMP_PREFIX_OFFSET,
                         data, rx_frame->len);
    dev_resolve_dbg(dp->this_dev, "Trailer length %i skb length %i\n",
                    dp->trailer_len, rx_frame->len);
#endif

    trailer = flx_frs_get_skb_trailer(rx_frame, dp->trailer_len);

    // Remove trailer from the frame length.
    skb_trim(rx_frame, rx_frame->len - dp->trailer_len);
    // Newer kernels emit HW csum failure without this.
    if (rx_frame->ip_summed == CHECKSUM_COMPLETE)
        rx_frame->ip_summed = CHECKSUM_NONE;

    dev_resolve_dbg(dp->this_dev, "Trailer 0x%x length %i skb length %i"
                    " eth_hdr %li mac_header %li proto 0x%x\n",
                    trailer, dp->trailer_len, rx_frame->len,
                    (long int)((unsigned char *)eth - rx_frame->head),
                    (long int)(skb_mac_header(rx_frame) - rx_frame->head),
                    (int)ntohs(eth->h_proto));

    // Get the real device from the management trailer.
    dp_recv = flx_frs_get_dev_priv_by_trailer(trailer);
    if (!dp_recv) {
        dev_err(dp->this_dev, "No FRS device found by trailer 0x%x\n",
                trailer);
        return NULL;
    }

    // Determine port number from management trailer.
    port_mask = trailer >> dp_recv->trailer_offset;
    port_num = find_first_bit(&port_mask, FLX_FRS_MAX_PORTS);
    if (port_num >= ARRAY_SIZE(dp_recv->port)) {
        dev_err(dp->this_dev, "Invalid trailer 0x%x\n", trailer);
        return NULL;
    }

    port = dp_recv->port[port_num];
    if (!port) {
        // Refuse to handle frames from unknown ports.
        dev_err(dp->this_dev, "Device %u port %u is unknown\n",
                dp_recv->dev_num, port_num);
        return NULL;
    }

    // Check if the port is an independent interface.
    if (port->flags & FLX_FRS_PORT_INDEPENDENT) {
        dev_resolve_dbg(dp->this_dev,
                   "Interface is independent, using real netdevice");
        return port;
    }

    // We do not use Linux bridge for traffic.
#if 0
    // When e.g. attached to bridge, always use real port.
    if (port->flags & FLX_FRS_HAS_MASTER)
        return port;
#endif

    // Detect frames that must be passed from real port netdev.
    if (eth->h_proto == htons(ETH_P_HSR_SUPERVISION)) {
        // This is a HSR supervision frame, forward to specific interface
        dev_resolve_dbg(dp->this_dev, "port (HSR) trailer 0x%x\n",
                        trailer);
        return port;
    }
    else if (eth->h_proto == htons(ETH_P_1588)) {
        if (!memcmp(eth->h_dest, ptp_peer_multicast_mac, ETH_ALEN)) {
            // This is a PTP peer-to-peer message
            dev_resolve_dbg(dp->this_dev,
                            "port (PTP peer-to-peer) trailer 0x%x\n",
                            trailer);
            return port;
        }
        else if (!memcmp(eth->h_dest, ptp_multicast_mac, ETH_ALEN)) {
            // This is a PTP message
            dev_resolve_dbg(dp->this_dev, "port (PTP) trailer 0x%x\n",
                            trailer);
            return port;
        }
    }
    else if (ntohs(eth->h_proto) < ETH_P_802_3_MIN) {
        if (!memcmp(eth->h_dest, bpdu_multicast_mac, ETH_ALEN)) {
            // This is a BPDU
            dev_resolve_dbg(dp->this_dev,
                            "port (BPDU) trailer 0x%x\n",
                            trailer);
            return port;
        }
    }

    // Pass all other frames from CPU port netdev.
    dp_recv = dp;
    port_mask = dp_recv->cpu_port_mask;
    port_num = find_first_bit(&port_mask, FLX_FRS_MAX_PORTS);
    if (port_num >= ARRAY_SIZE(dp_recv->port)) {
        dev_err(dp->this_dev, "Invalid port number %i\n", port_num);
        return NULL;
    }
    port = dp_recv->port[port_num];

    if (!port) {
        dev_err(dp->this_dev, "Device %u port %i is unknown\n",
                dp_recv->dev_num, port_num);
        return NULL;
    }

    return port;
}

/**
 * Get PTP timestamp
 * @param dp FRS private data.
 * @param rx_frame
 * @return 0 if ok, or -1 if fails
 */
static int flx_frs_update_rx_timestamp(struct flx_frs_dev_priv *dp,
                                       struct sk_buff *rx_frame)
{
#ifndef USE_FRS_TIMETRAILER
    struct ethhdr *ehdr = eth_hdr(rx_frame);
    uint8_t *payload = (uint8_t *) (ehdr + 1);
    uint64_t sec;
    uint32_t nsec;

    if (!ehdr) {
        dev_err(dp->this_dev, "No data\n");
        return 0;
    }
    if (ehdr->h_proto != htons(ETH_P_1588)) {
        return 0;
    }
    //dev_printk(KERN_DEBUG, dp->this_dev, "1588 frame\n");

    // Get timestamp
    switch (payload[0] & 0x0f) {
        // Event messages, timestamped by FRS
    case 0:
    case 1:
    case 2:
    case 3:
        sec = nsec = 0;
        if (frs_tx_timestamp(dp, payload, &sec, &nsec) == 0) {
            // Get frame timestamp and store it to skb
            rx_frame->tstamp = ktime_set(sec, nsec);
        } else {
            // No timestamp available, must drop
            return -1;
        }
        break;
    default:
        break;
    }

#endif
    return 0;
}

/**
 * Get sent frame and timestamp from FRS.
 * @param dp FRS private data.
 * @return 0 if ok, or -1 if fails
 */
static int flx_frs_get_tx_timestamp(struct flx_frs_dev_priv *dp)
{
    struct flx_frs_port_priv *port = NULL;
    struct net_device *dev = NULL;
    uint64_t sec;
    uint32_t nsec;
#define PTP_FRAME_SIZE 64
    struct sk_buff *skb = NULL;
    struct ethhdr *ehdr = NULL;
    uint8_t *data = NULL;
    const uint8_t ptp_multicast_mac[IFHWADDRLEN] =
        { 0x01, 0x1B, 0x19, 0x00, 0x00, 0x00 };
    int ret = 0;
    int i = 0;
    uint16_t rx_ts_ctrl = 0;

    while ((ret = frs_rx_timestamp_available(dp)) != -1) {
        rx_ts_ctrl = ret;
        dev_dbg(dp->this_dev, "RX timestamp (0x%x)\n", rx_ts_ctrl);
        skb = dev_alloc_skb(2 * PTP_FRAME_SIZE);
        if (skb) {
            skb_reserve(skb, NET_IP_ALIGN);     // 16 byte align the IP header
            data = skb_put(skb, PTP_FRAME_SIZE);
        } else {
            data = NULL;
        }
        // Get PTP frame and TX timestamp
        sec = nsec = 0;
        ret = frs_rx_timestamp(dp, rx_ts_ctrl, data, &sec, &nsec);
        if (ret) {
            dev_err(dp->this_dev, "No RX timestamp available\n");
            goto free_skb;
        }
        if (!data) {
            dev_err(dp->this_dev, "No buffer for RX timestamp frame\n");
            continue;
        }
        /*
         * Pass always back to OS from first PTP enabled port, because
         * - there is no way to determine the original net device
         *   through which the frame was sent, and
         * - this is only done for PTP frames
         */
        for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
            port = dp->port[i];
            if (!port)
                continue;

            // Check netdevice status
            if (!netif_carrier_ok(port->netdev))
                continue;

            // Netdev up
            if (port->flags & FLX_FRS_MSG_PTP) {
                // This is ok for us
                dev = port->netdev;
                break;
            }
        }

        if (!dev) {
            dev_err(dp->this_dev,
                    "No active netdev for RX timestamp frame\n");
            goto free_skb;
        }

        ehdr = (struct ethhdr *) skb_push(skb, sizeof(struct ethhdr));
        memcpy(ehdr->h_dest, ptp_multicast_mac, ETH_ALEN);
        memcpy(ehdr->h_source, dev->dev_addr, ETH_ALEN);
        ehdr->h_proto = htons(ETH_P_1588);

        skb->protocol = eth_type_trans(skb, dev);
        skb->tstamp = ktime_set(sec, nsec);
        skb->ip_summed = CHECKSUM_UNNECESSARY;

        netif_rx(skb);          // give the packet to the stack
    }

    return 0;

  free_skb:
    if (skb) {
        dev_kfree_skb(skb);
    }

    return -1;
}

/**
 * Get FRS RX timestamper index.
 * @param dp FRS private data.
 * @return RX timestamper index on when available or -1
 */
static inline int frs_rx_timestamp_available(struct flx_frs_dev_priv *dp)
{
    uint16_t rx_ts_ctrl = flx_frs_read_switch_reg(dp, FRS_REG_TS_CTRL_RX);

    //dev_printk(KERN_DEBUG, dp->this_dev, "RX 0x%x 0x%x %i\n",
    //           rx_ts_ctrl, 1 << dp->rx_stamper_index, dp->rx_stamper_index);
    if (!(rx_ts_ctrl & (1 << dp->rx_stamper_index))) {  // Timestamp available
        //dev_printk(KERN_DEBUG, dp->this_dev, "RX 0x%x 0x%x %i\n",
        //           rx_ts_ctrl, 1 << dp->rx_stamper_index, dp->rx_stamper_index);
        return rx_ts_ctrl;
    }

    return -1;
}

/**
 * Read FRS RX timestamp.
 * @param dp FRS private data.
 * @param rx_ts_ctrl timestamper index
 * @param hdr PTP header of the received frame. Skip frame if NULL.
 * @param sec Timestamp seconds
 * @param nsec Timestamp nanoseconds
 * @return 0 on success or -1
 */
static int frs_rx_timestamp(struct flx_frs_dev_priv *dp,
                            uint16_t rx_ts_ctrl,
                            uint8_t *hdr, uint64_t *sec, uint32_t *nsec)
{
    uint16_t *tmp = NULL;
    int i = 0;

    //dev_printk(KERN_DEBUG, dp->this_dev, "RX 0x%x 0x%x %i\n",
    //           rx_ts_ctrl, 1 << dp->rx_stamper_index, dp->rx_stamper_index);
    if (!(rx_ts_ctrl & (1 << dp->rx_stamper_index))) {  // Timestamp available
        //dev_printk(KERN_DEBUG, dp->this_dev, "RX 0x%x 0x%x %i\n",
        //           rx_ts_ctrl, 1 << dp->rx_stamper_index, dp->rx_stamper_index);
        if (hdr) {
            // Copy frame if buffer available
            tmp = (uint16_t *) hdr;
            for (i = 0; i < (FRS_TS_HDR_LEN / 2); i++) {
                // LSB contains first byte.
                tmp[i] =
                    __cpu_to_le16(flx_frs_read_switch_reg
                                  (dp,
                                   FRS_TS_RX_HDR(dp->rx_stamper_index,
                                                 i)));
            }
            *sec =
                flx_frs_read_switch_reg(dp,
                                        FRS_TS_RX_S_HI(dp->
                                                       rx_stamper_index))
                << 16;
            *sec |=
                flx_frs_read_switch_reg(dp,
                                        FRS_TS_RX_S_LO(dp->
                                                       rx_stamper_index));
            *nsec =
                flx_frs_read_switch_reg(dp,
                                        FRS_TS_RX_NS_HI(dp->
                                                        rx_stamper_index))
                << 16;
            *nsec |=
                flx_frs_read_switch_reg(dp,
                                        FRS_TS_RX_NS_LO(dp->
                                                        rx_stamper_index));
            // Normalize value.
            while (*nsec >= 1000000000u) {
                (*nsec)--;
                (*sec)++;
            }
            //dev_printk(KERN_DEBUG, dp->this_dev, "RX: %llus %uns\n",
            //           *sec, *nsec);
        }
        // Enable read timestamp
        flx_frs_write_switch_reg(dp, FRS_REG_TS_CTRL_RX,
                                 (1 << dp->rx_stamper_index));

        // Go to next entry
        dp->rx_stamper_index =
            (dp->rx_stamper_index + 1) % FRS_TS_NUMBER_TIMESTAMPS;

        return 0;
    }

    return -1;
}

#ifndef USE_FRS_TIMETRAILER
/**
 * Read FRS TX timestamp.
 * @param dp FRS private data.
 * @param hdr PTP header of the received frame.
 * @param sec Timestamp seconds
 * @param nsec Timestamp nanoseconds
 * @return 0 on success or -1
 */
static int frs_tx_timestamp(struct flx_frs_dev_priv *dp,
                            uint8_t *hdr, uint64_t *sec, uint32_t *nsec)
{
    uint16_t tx_ts_ctrl = flx_frs_read_switch_reg(dp, FRS_REG_TS_CTRL_TX);
    uint16_t tx_ts_ctrl_enable = 0;
    uint16_t *tmp = NULL;
    uint16_t data;
    int i = 0;
    int ret = -1;
    int found = 0;

    //dev_printk(KERN_DEBUG, dp->this_dev, "TX 0x%x 0x%x %i\n",
    //           tx_ts_ctrl, 1 << dp->tx_stamper_index, dp->tx_stamper_index);
    while (!(tx_ts_ctrl & (1 << dp->tx_stamper_index))) {       // Timestamp available
        //dev_printk(KERN_DEBUG, dp->this_dev, "TX 0x%x 0x%x %i\n",
        //           tx_ts_ctrl, 1 << dp->tx_stamper_index, dp->tx_stamper_index);
        // Check if timestamp header is the desired one
        tmp = (uint16_t *) hdr;
        found = 1;
        for (i = 0; i < (FRS_TS_HDR_LEN / 2); i++) {
            data =
                flx_frs_read_switch_reg(dp,
                                        FRS_TS_TX_HDR(dp->tx_stamper_index,
                                                      i));
            // LSB contains first byte.
            if (__le16_to_cpu(tmp[i]) != data) {
                found = 0;
                break;
            }
        }
        if (found) {
            *sec =
                flx_frs_read_switch_reg(dp,
                                        FRS_TS_TX_S_HI(dp->
                                                       tx_stamper_index))
                << 16;
            *sec |=
                flx_frs_read_switch_reg(dp,
                                        FRS_TS_TX_S_LO(dp->
                                                       tx_stamper_index));
            *nsec =
                flx_frs_read_switch_reg(dp,
                                        FRS_TS_TX_NS_HI(dp->
                                                        tx_stamper_index))
                << 16;
            *nsec |=
                flx_frs_read_switch_reg(dp,
                                        FRS_TS_TX_NS_LO(dp->
                                                        tx_stamper_index));
            //dev_printk(KERN_DEBUG, dp->this_dev, "TX: %llus %uns\n",
            //           *sec, *nsec);
        }
        // Enable read timestamp
        tx_ts_ctrl_enable = (1 << dp->tx_stamper_index);
        tx_ts_ctrl |= tx_ts_ctrl_enable;        // To avoid loop
        flx_frs_write_switch_reg(dp, FRS_REG_TS_CTRL_TX,
                                 tx_ts_ctrl_enable);

        // Go to next entry
        dp->tx_stamper_index =
            (dp->tx_stamper_index + 1) % FRS_TS_NUMBER_TIMESTAMPS;
        if (found) {
            ret = 0;
            break;
        }
    }

    return ret;
}
#endif
