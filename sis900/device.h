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

#include <stdint.h>

#include "cdi.h"
#include "cdi/net.h"
#include "cdi/pci.h"


#define REG_COMMAND 0x00
#define REG_CONFIG  0x04
#define REG_EEPROM  0x08
#define REG_ISR     0x10
#define REG_IMR     0x14
#define REG_IER     0x18
#define REG_TX_PTR  0x20
#define REG_TX_CFG  0x24
#define REG_RX_PTR  0x30
#define REG_RX_CFG  0x34
#define REG_RX_FILT 0x48
#define REG_RX_FDAT 0x4C

#define CR_ENABLE_TX    0x01
#define CR_DISABLE_TX   0x02
#define CR_ENABLE_RX    0x04
#define CR_DISABLE_RX   0x08
#define CR_RESET_TX     0x10
#define CR_RESET_RX     0x20
#define CR_RESET        0x100
#define CR_RELOAD_MAC   0x400

#define ISR_ROK             0x01
#define ISR_RERR            0x04
#define ISR_RX_IDLE         0x10
#define ISR_RX_OVERFLOW     0x20
#define ISR_TOK             0x40
#define ISR_TERR            0x100
#define ISR_RX_RESET_COMP   0x01000000
#define ISR_TX_RESET_COMP   0x02000000

#define TXC_DRAIN_TSH   (48 << 0)
#define TXC_FILL_TSH    (48 << 8)
#define TXC_PADDING     (1 << 28)
#define TXC_LOOPBACK    (1 << 29)
#define TXC_HBI         (1 << 30)
#define TXC_CSI         (1 << 31)

#define RXC_DRAIN_TSH   (16 << 1)
#define RXC_ACCEPT_TP   (1 << 28)

#define RXFCR_PHYS      (1 << 28)
#define RXFCR_BROADCAST (1 << 30)
#define RXFCR_ENABLE    (1 << 31)

#define EEPROM_DATA_IN  0x01
#define EEPROM_DATA_OUT 0x02
#define EEPROM_CLOCK    0x04
#define EEPROM_CHIPSEL  0x08

#define EEPROM_OP_READ  0x180

#define EEPROM_OFS_MAC  0x08


#define TX_BUFFER_SIZE  2048
#define TX_BUFFER_NUM   8

#define RX_BUFFER_SIZE  1536
#define RX_BUFFER_NUM   8


#define DESC_STATUS_OWN (1 << 31)

struct sis900_tx_descriptor {
    uint32_t            link;
    volatile uint32_t   status;
    uint32_t            buffer;
} __attribute__((packed)) __attribute__((aligned (4)));

struct sis900_device {
    struct cdi_net_device       net;
    struct cdi_pci_device*      pci;

    void*                       phys;

    struct sis900_tx_descriptor tx_desc[TX_BUFFER_NUM];
    uint8_t                     tx_buffer[TX_BUFFER_NUM * TX_BUFFER_SIZE];
    int                         tx_cur_buffer;
    
    struct sis900_tx_descriptor rx_desc[RX_BUFFER_NUM];
    uint8_t                     rx_buffer[RX_BUFFER_NUM * RX_BUFFER_SIZE];
    int                         rx_cur_buffer;

    uint16_t                    port_base;
};

void sis900_init_device(struct cdi_device* device);
void sis900_remove_device(struct cdi_device* device);

void sis900_send_packet
    (struct cdi_net_device* device, void* data, size_t size);

uint16_t sis900_eeprom_read(struct sis900_device* device, uint16_t offset);

#endif
