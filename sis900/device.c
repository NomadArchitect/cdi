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

#include "cdi.h"
#include "cdi/misc.h"

#include "device.h"
#include "sis900_io.h"

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

    // Wir wollen alles, was an unsere MAC geht und Broadcast
    // TODO Multicast-Hashtabelle setzen
    reg_outl(netcard, REG_RX_FILT, 
        RXFCR_ENABLE | RXFCR_PHYS | RXFCR_BROADCAST);
    
    // Deskriptoren initialisieren
    netcard->tx_desc.link = 0;
    netcard->tx_desc.status = 0;
    netcard->tx_desc.buffer = 0;
    
    netcard->rx_desc.link = 0;
    netcard->rx_desc.status = RX_BUFFER_SIZE;
    netcard->rx_desc.buffer = cdi_get_phys_addr(netcard->rx_buffer);

    reg_outl(netcard, REG_TX_PTR, cdi_get_phys_addr(&netcard->tx_desc));
    reg_outl(netcard, REG_RX_PTR, cdi_get_phys_addr(&netcard->rx_desc));

    printf("sis900: Rx: Desc @ phys %08x, Buffer @ phys %08x\n",
        netcard->rx_desc.buffer, cdi_get_phys_addr(&netcard->rx_desc));
    
    // Receiver aktivieren
    reg_outl(netcard, REG_COMMAND, CR_ENABLE_RX);

    // Interrups wieder aktivieren
    reg_outl(netcard, REG_IMR,  ISR_ROK | ISR_TOK | ISR_RERR | ISR_TERR);
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

    reg_outl(device, REG_RX_FILT, old_rfcr & RXFCR_ENABLE);

    // Dreimal jeweils ein Word der MAC-Adresse, die jetzt im Register Receive
    // Filter Data vorliegt, adressieren und einlesen.
    for (i = 0; i < 3; i++) {
        reg_outl(device, REG_RX_FILT, i << 16);
        mac |= ((uint64_t) reg_inw(device, REG_RX_FDAT)) << ((2 - i) * 16);
    }

    // Altes Filterregister wiederherstellen
    reg_outl(device, REG_RX_FILT, old_rfcr | RXFCR_ENABLE);

    return mac;
}

void sis900_init_device(struct cdi_driver* driver, struct cdi_device* device)
{
    struct sis900_device* netcard = (struct sis900_device*) device;

    // PCI-bezogenes Zeug initialisieren
    cdi_register_irq(netcard->pci->irq, sis900_handle_interrupt, device);
    cdi_pci_alloc_ioports(netcard->pci);
    
    cdi_list_t* reslist = netcard->pci->resources;
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

    printf("sis900: MAC-Adresse: %012llx\n", get_mac_address(netcard));
   
    printf("sis900: CR = %08x\n", reg_inl(netcard, REG_COMMAND));

    reg_outl(netcard, REG_COMMAND, 0xC0);
}

void sis900_remove_device(struct cdi_driver* driver, struct cdi_device* device)
{
}

void sis900_send_packet(struct cdi_device* device, void* data, size_t size)
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
    netcard->tx_desc.buffer = cdi_get_phys_addr(netcard->tx_buffer);

    reg_outl(netcard, REG_TX_PTR, cdi_get_phys_addr(&netcard->tx_desc));

    // Transmitter wieder starten
    reg_outl(netcard, REG_COMMAND, CR_ENABLE_TX);

    // Warten, bis das Paket gesendet ist
    while (netcard->tx_desc.status & DESC_STATUS_OWN);
}

static void sis900_handle_interrupt(struct cdi_device* device)
{
    struct sis900_device* netcard = (struct sis900_device*) device;

    printf("sis900: Interrupt, ISR = %08x\n", reg_inl(netcard, REG_ISR));

    uint32_t isr = reg_inl(netcard, REG_ISR);

    if (isr & ISR_ROK) {
        while (1) {
            uint32_t status = netcard->rx_desc.status;

            if ((status & DESC_STATUS_OWN) == 0) {
                break;
            }

            // 4 Bytes CRC von der Laenge abziehen
            size_t size = (status & 0xFFF) - 4;

            printf("sis900: %d Bytes empfangen (status =%x)\n", size, status);
            int i;
            for (i = 0; i < (size < 49 ? size : 49); i++) {
                printf("%02hhx ", netcard->rx_buffer[i]);
                if (i % 25 == 0) {
                    printf("\n");
                }
            }
            printf("\n\n");
        
            netcard->rx_desc.link = 0;
            netcard->rx_desc.status = RX_BUFFER_SIZE;
            netcard->rx_desc.buffer = cdi_get_phys_addr(netcard->rx_buffer);
        }

        reg_outl(netcard, REG_COMMAND, CR_ENABLE_RX);
    }
}
