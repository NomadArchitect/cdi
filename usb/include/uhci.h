/******************************************************************************
 * Copyright (c) 2009 Max Reitz                                               *
 *                                                                            *
 * Permission is hereby granted,  free of charge,  to any  person obtaining a *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction,  including without limitation *
 * the rights to use,  copy, modify, merge, publish,  distribute, sublicense, *
 * and/or sell copies  of the  Software,  and to permit  persons to whom  the *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING  BUT NOT  LIMITED TO THE WARRANTIES OF MERCHANTABILITY, *
 * FITNESS FOR A PARTICULAR  PURPOSE AND  NONINFRINGEMENT.  IN NO EVENT SHALL *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY,  WHETHER IN AN ACTION OF CONTRACT,  TORT OR OTHERWISE,  ARISING *
 * FROM,  OUT OF  OR IN CONNECTION  WITH THE  SOFTWARE  OR THE  USE OR  OTHER *
 * DEALINGS IN THE SOFTWARE.                                                  *
 ******************************************************************************/

#ifndef _CDI__USB__UHCI_H
#define _CDI__USB__UHCI_H

#include <stdint.h>

#include "cdi/misc.h"
#include "cdi/pci.h"

#include "usb.h"


#define UHCI_USBCMD    0x00 //w
#define UHCI_USBSTS    0x02 //w
#define UHCI_USBINTR   0x04 //w
#define UHCI_FRNUM     0x06 //w
#define UHCI_FRBASEADD 0x08 //l
#define UHCI_SOFMOD    0x0C //b
#define UHCI_RPORTS    0x10
#define UHCI_PORTSC1   0x10 //w
#define UHCI_PORTSC2   0x12 //w

#define MAXP           0x0080
#define GRESET         0x0004
#define HCRESET        0x0002
#define USB_RUN        0x0001

#define RPORT_RESET    0x0200
#define RPORT_LOSPD    0x0100 //Low speed device attached
#define RPORT_ENABLE   0x0004
#define RPORT_CSC      0x0002 //Connect status change
#define RPORT_DEVICE   0x0001


struct uhci {
    struct hci gen_hci;
    uint16_t pbase;
    uintptr_t phys_frame_list;
    uint32_t* frame_list;
    int root_ports;
};

struct uhci_qh {
    volatile uint32_t next;
    volatile uint32_t transfer;
} __attribute__((packed));

struct uhci_td {
    volatile uint32_t next;

    unsigned trans_len : 11;
    unsigned rsvd0 : 6;
    unsigned bitstuff_err : 1;
    unsigned crc_time_err : 1;
    unsigned nak : 1;
    unsigned babble : 1;
    unsigned buf_err : 1;
    unsigned stalled_err : 1;
    unsigned active : 1;
    unsigned ioc : 1;
    unsigned isochronous : 1;
    unsigned low_speed : 1;
    unsigned errors : 2;
    unsigned spd : 1;
    unsigned rsvd1 : 2;

    unsigned pid : 8;
    unsigned device : 7;
    unsigned endpoint : 4;
    unsigned data_toggle : 1;
    unsigned rsvd2 : 1;
    unsigned maxlen : 11;

    uint32_t buffer;

    uint32_t user[4];
} __attribute__((packed));

struct transfer {
    void* virt;
    uintptr_t phys;
    volatile int error;
};

#endif
