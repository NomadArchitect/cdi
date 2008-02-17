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

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <syscall.h>

#include "cdi.h"
#include "cdi/misc.h"

#include "device.h"
#include "sis900_io.h"

#define DEBUG

#define PHYS(netcard, field) \
    ((uintptr_t) netcard->phys + offsetof(struct sis900_device, field))

static void sis900_handle_interrupt(struct cdi_device* device);


static void enable_intr(struct sis900_device* device)
{
    reg_outl(device, REG_IER, 1);
}

static void disable_intr(struct sis900_device* device)
{
    reg_outl(device, REG_IER, 0);
}

static void reset_nic(struct sis900_device* netcard)
{    
    // Interrupts deaktivieren
    disable_intr(netcard);

    // Soft-Reset ausfuehren
    reg_outl(netcard, REG_COMMAND, CR_RESET | CR_RESET_TX | CR_RESET_RX);
    while (reg_inl(netcard, REG_COMMAND) & CR_RESET);

    uint32_t isr = 0;
    uint32_t complete = (ISR_RX_RESET_COMP | ISR_TX_RESET_COMP);
    do {
        isr |= reg_inl(netcard, REG_ISR);
    } while ((isr & complete) != complete);

    // Receiver und Transmitter konfigurieren
    reg_outl(netcard, REG_RX_CFG, 
        RXC_DRAIN_TSH | RXC_ACCEPT_TP);

    reg_outl(netcard, REG_TX_CFG, 
        TXC_CSI | TXC_HBI | TXC_PADDING | TXC_DRAIN_TSH | TXC_FILL_TSH);

    // Wir wollen alles, was an unsere MAC geht und Broadcast
    // TODO Multicast-Hashtabelle setzen
    reg_outl(netcard, REG_RX_FILT, 
        RXFCR_ENABLE | RXFCR_PHYS | RXFCR_BROADCAST);
    
    // Deskriptoren initialisieren
    netcard->tx_desc.link = 0;
    netcard->tx_desc.status = 0;
    netcard->tx_desc.buffer = 0;
   
    int i;
    for (i = 0; i < RX_BUFFER_NUM; i++) {
        netcard->rx_desc[i].link = 
            PHYS(netcard, rx_desc[(i + 1) % RX_BUFFER_NUM]);
        netcard->rx_desc[i].status = RX_BUFFER_SIZE;
        netcard->rx_desc[i].buffer = 
            PHYS(netcard, rx_buffer[i * RX_BUFFER_SIZE]);

#ifdef DEBUG
        printf("sis900: [%d] Rx: Buffer @ phys %08x, Desc @ phys %08x\n",
            i,
            netcard->rx_desc[i].buffer, 
            PHYS(netcard, rx_desc[i]));
#endif
    }
    netcard->rx_cur_buffer = 0;

    reg_outl(netcard, REG_TX_PTR, PHYS(netcard, tx_desc));
    reg_outl(netcard, REG_RX_PTR, PHYS(netcard, rx_desc[0]));
    
    // Receiver aktivieren
    reg_outl(netcard, REG_COMMAND, CR_ENABLE_RX);

    // Interrups wieder aktivieren
    reg_outl(netcard, REG_IMR, 0xffffffff /*ISR_ROK | ISR_TOK | ISR_RERR | ISR_TERR*/);
    enable_intr(netcard);
}


static uint64_t get_mac_address(struct sis900_device* device)
{
    uint64_t mac = 0;
    int i;
    
    /* Das hier ist fuer aeltere Revisionen
    for (i = 0; i < 3; i++) {
        mac |= sis900_eeprom_read(device, EEPROM_OFS_MAC + i) << (i * 16); 
    }
    */

    // Revision 0x82 bis 0x90
    
    // CR_RELOAD_MAC laedt die MAC-Adresse in den Filter
    uint32_t old_rfcr = reg_inl(device, REG_RX_FILT);
    reg_outl(device, REG_COMMAND, old_rfcr | CR_RELOAD_MAC);
    reg_outl(device, REG_COMMAND, 0);

    reg_outl(device, REG_RX_FILT, old_rfcr & ~RXFCR_ENABLE);

    // Dreimal jeweils ein Word der MAC-Adresse, die jetzt im Register Receive
    // Filter Data vorliegt, adressieren und einlesen.
    for (i = 0; i < 3; i++) {
        reg_outl(device, REG_RX_FILT, i << 16);
        mac |= ((uint64_t) reg_inw(device, REG_RX_FDAT)) << (i * 16);
    }

    // Altes Filterregister wiederherstellen
    reg_outl(device, REG_RX_FILT, old_rfcr | RXFCR_ENABLE);

    return mac;
}

void sis900_init_device(struct cdi_device* device)
{
    struct sis900_device* netcard = (struct sis900_device*) device;
    netcard->net.send_packet = sis900_send_packet;

    cdi_net_device_init((struct cdi_net_device*) device);

    // PCI-bezogenes Zeug initialisieren
    cdi_register_irq(netcard->pci->irq, sis900_handle_interrupt, device);
    cdi_pci_alloc_ioports(netcard->pci);
    
    cdi_list_t reslist = netcard->pci->resources;
    struct cdi_pci_resource* res;
    int i;
    for (i = 0; (res = cdi_list_get(reslist, i)); i++) {
        if (res->type == CDI_PCI_IOPORTS) {
            netcard->port_base = res->start;
        }
    }

    // Karte initialisieren
    printf("sis900: IRQ %d, Ports an %x\n", 
        netcard->pci->irq, netcard->port_base);

    printf("sis900: Fuehre Reset der Karte durch\n");
    reset_nic(netcard);
    
    netcard->net.mac = get_mac_address(netcard);
    printf("sis900: MAC-Adresse: %012llx\n", netcard->net.mac);
   
    printf("sis900: CR = %08x\n", reg_inl(netcard, REG_COMMAND));

    reg_outl(netcard, REG_COMMAND, 0xC0);

/*
    uint32_t sendbuf[4];
    sendbuf[0] = 0x54520012;
    sendbuf[1] = 0x23455452;
    sendbuf[2] = 0x00123457;
    sendbuf[3] = 0x0006abcd;

    sis900_send_packet(device, sendbuf, 16);
*/    
}

void sis900_remove_device(struct cdi_device* device)
{
}

void sis900_send_packet(struct cdi_net_device* device, void* data, size_t size)
{
    struct sis900_device* netcard = (struct sis900_device*) device;

    // Transmitter stoppen
    reg_outl(netcard, REG_COMMAND, CR_DISABLE_TX);
    
    // Buffer befuellen
    if (size > TX_BUFFER_SIZE) {
        size = TX_BUFFER_SIZE;
    }
    memcpy(netcard->tx_buffer, data, size);
    
    // Padding (Eigentlich soll die Karte das auch selber koennen)
    if (size < 60) {
        memset(netcard->tx_buffer + size, 0, 60 - size);
        size = 60;
    }

    // TX-Deskriptor setzen und laden
    netcard->tx_desc.link = 0;
    netcard->tx_desc.status = size | DESC_STATUS_OWN;
    netcard->tx_desc.buffer = PHYS(netcard, tx_buffer);

    reg_outl(netcard, REG_TX_PTR, PHYS(netcard, tx_desc));

    // Transmitter wieder starten
    reg_outl(netcard, REG_COMMAND, CR_ENABLE_TX);

    // Warten, bis das Paket gesendet ist
    // FIXME: Nicht portabel
    qword timeout = get_tick_count() + 500000;
    while ((netcard->tx_desc.status & DESC_STATUS_OWN) 
        && (get_tick_count() < timeout));

    if (netcard->tx_desc.status & DESC_STATUS_OWN) {        
        printf("sis900: Fehler beim Senden eines Pakets (%d Bytes)\n", size);
        printf("sis900: Status: %08x, CR = %08x, ISR = %08x\n", 
            netcard->tx_desc.status, reg_inl(netcard, REG_COMMAND),
            reg_inl(netcard, REG_ISR));

/*
        reg_outl(netcard, REG_COMMAND, CR_RESET_TX);

        uint32_t isr;
        do {
            isr = reg_inl(netcard, REG_ISR);
        } while ((isr & ISR_TX_RESET_COMP) == 0);
        reset_nic(netcard);
        get_mac_address(netcard);

        printf("sis900: Reset durchgefuehrt, verwerfe Paket. CR = %08x\n", 
            reg_inl(netcard, REG_COMMAND));
*/
    } else {
        printf("sis900: Paket gesendet (%d Bytes)\n", size);
    }
}

static void sis900_handle_interrupt(struct cdi_device* device)
{
    struct sis900_device* netcard = (struct sis900_device*) device;

#ifdef DEBUG
    uint32_t isr = reg_inl(netcard, REG_ISR);
    printf("sis900: Interrupt, ISR = %08x\n", isr);

    printf("sis900: RxDesc = %08x, RxCfg = %08x, CR = %08x, cmdsts0 = %08x\n",
        reg_inl(netcard, REG_RX_PTR), reg_inl(netcard, REG_RX_CFG),
        reg_inl(netcard, REG_COMMAND), netcard->rx_desc[0].status);
    
    printf("sis900: rx_cur_buffer = %d, cmdsts_cur = %08x, desc_cur = %08x\n",
        netcard->rx_cur_buffer,
        netcard->rx_desc[netcard->rx_cur_buffer].status,
        PHYS(netcard, rx_desc[netcard->rx_cur_buffer]));

    if (isr & ISR_RX_OVERFLOW) {
        printf("RxOverflow. Status: ");
        int i;
        for (i = 0; i < RX_BUFFER_NUM; i++) {
            printf("[%d] %08x ", i, netcard->rx_desc[i].status);
        }
        //disable_intr(netcard);
        printf("\n");
    }
#endif

//    if (isr & ISR_ROK) {
        while (1) {
            uint32_t status = netcard->rx_desc[netcard->rx_cur_buffer].status;

            if ((status & DESC_STATUS_OWN) == 0) {
                break;
            }

            // 4 Bytes CRC von der Laenge abziehen
            size_t size = (status & 0xFFF) - 4;

#ifdef DEBUG
            printf("sis900: %d Bytes empfangen (status = %x)\n", size, status);
/*            
            int i;
            for (i = 0; i < (size < 49 ? size : 49); i++) {
                printf("%02hhx ", netcard->rx_buffer[
                    netcard->rx_cur_buffer * RX_BUFFER_SIZE + i]);
                if (i % 25 == 0) {
                    printf("\n");
                }
            }
            printf("\n\n");
*/            
#endif
            
            cdi_net_receive(
                (struct cdi_net_device*) netcard, 
                &netcard->rx_buffer[netcard->rx_cur_buffer * RX_BUFFER_SIZE],
                size);

            netcard->rx_desc[netcard->rx_cur_buffer].status = RX_BUFFER_SIZE;
            netcard->rx_cur_buffer++;
            netcard->rx_cur_buffer %= RX_BUFFER_NUM;
        }

        reg_outl(netcard, REG_COMMAND, CR_ENABLE_RX);
//    }
}
