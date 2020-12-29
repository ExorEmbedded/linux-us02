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

#ifndef FLX_FRS_NETDEVIF_H
#define FLX_FRS_NETDEVIF_H

#include <linux/interrupt.h>

#include "flx_frs_types.h"

int flx_frs_netdevif_init(struct flx_frs_dev_priv *dp);
void flx_frs_netdevif_cleanup(struct flx_frs_dev_priv *dp);
void flx_frs_xmit(struct net_device *to, struct sk_buff *skb);
int __devinit flx_frs_irq_init(struct flx_frs_dev_priv *dp);
void flx_frs_irq_cleanup(struct flx_frs_dev_priv *dp);

/**
 * Add management trailer to an sk_buff, in big-endian form.
 * @param skb sk_buff to add management trailer to.
 * @param trailer Management trailer to add.
 * @parm trailer_len Number of octets in management trailer.
 */
static inline void flx_frs_set_skb_trailer(struct sk_buff *skb,
                                           unsigned int trailer,
                                           unsigned int trailer_len)
{
    uint8_t *skb_trailer = skb_put(skb, trailer_len);

    if (trailer_len == 1) {
        skb_trailer[0] = (uint8_t) ((trailer >> 0) & 0xffu);
    }
    else {
        skb_trailer[0] = (uint8_t) ((trailer >> 8) & 0xffu);
        skb_trailer[1] = (uint8_t) ((trailer >> 0) & 0xffu);
    }

    return;
}

#endif
