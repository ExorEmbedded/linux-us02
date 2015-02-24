/*
 * Data structures, function prototypes and registers layout
 *
 * Copyright (C) 2013 Exor International
 * Author: Giovanni Pavoni (Exor)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#ifndef ULTIEVCH_H
#define ULTIEVCH_H

#include "displayconfig.h"


/* -----------------------------------------------------------
   Function prototypes
   -----------------------------------------------------------*/
int ultievc_setvideomode(void); /* Configures the ultievc core according with the selected display and activates it */

/* -----------------------------------------------------------
   UltiEVC regster mapping
   -----------------------------------------------------------*/
#define HSY_FP      0x00
#define HSY_W       0x04
#define HSY_BP      0x08
#define HRES        0x0C

#define VSY_FP      0x10
#define VSY_W       0x14
#define VSY_BP      0x18
#define VRES        0x1C

#define SCTRL       0x20
#define CLKCTRL     0x24
#define DCTRL       0x28
#define PWRCTRL     0x2C

#define LBADDR      0x40
#define LSTRIPE     0x44
#define LCTRL       0x48

/*
 * Ultievc register fields
 */
#define SCTRL_VSEN  (1 << 0)
#define SCTRL_VSPOL (1 << 1)
#define SCTRL_HSEN  (1 << 2)
#define SCTRL_HSPOL (1 << 3)
#define SCTRL_ENEN  (1 << 4)
#define SCTRL_ENPOL (1 << 5)

#define DCTRL_16BOUT 0x21
#define DCTRL_24BOUT 0x22

#define PWRCTRL_VEN  (1 << 0)
#define PWRCTRL_VDEN (1 << 1)
#define PWRCTRL_VEEN (1 << 2)
#define PWRCTRL_BLEN (1 << 3)

#define LCTRL_ENABLE 0x120


#endif
