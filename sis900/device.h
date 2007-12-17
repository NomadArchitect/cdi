/*  
 * Copyright (c) 2007 The LOST Project. All rights reserved.
 *
 * This code is derived from software contributed to the LOST Project
 * by Kevin Wolf.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *     This product includes software developed by the LOST Project
 *     and its contributors.
 * 4. Neither the name of the LOST Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SIS900_DEVICES_H_
#define _SIS900_DEVICES_H_

#include "cdi.h"
#include "cdi/net.h"
#include "cdi/pci.h"


#define REG_COMMAND 0x00
#define REG_CONFIG  0x04
#define REG_ISR     0x10
#define REG_IMR     0x14
#define REG_IER     0x18
#define REG_TX_PTR  0x20
#define REG_TX_CFG  0x24
#define REG_RX_PTR  0x30
#define REG_RX_CFG  0x34
#define REG_RX_FILT 0x48

#define CR_ENABLE_TX    0x01
#define CR_ENABLE_RX    0x04
#define CR_RESET_TX     0x10
#define CR_RESET_RX     0x20
#define CR_RESET        0x100

#define RXFCR_PHYS      (1 << 28)
#define RXFCR_BROADCAST (1 << 30)

struct sis900_device {
    struct cdi_net_device       dev;
    struct cdi_pci_device*      pci;

    uint16_t                    port_base;
};

void sis900_init_device(struct cdi_driver* driver, struct cdi_device* device);
void sis900_remove_device(struct cdi_driver* driver, 
    struct cdi_device* device);

void sis900_send_packet(struct cdi_device* device, void* data, size_t size);

#endif
