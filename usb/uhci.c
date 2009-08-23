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

#include <stdlib.h>
#include <string.h>

#include "cdi/io.h"
#include "cdi/lists.h"
#include "cdi/misc.h"
#include "cdi/pci.h"

#include "uhci.h"
#include "usb.h"
#include "mempool.h"

#define DEBUG

#ifdef DEBUG
#include <stdio.h>
#include <stdarg.h>
#define dprintf(fmt, args...) printf("[uhci] " fmt, ## args)
#define _dprintf(fmt, args...) printf(fmt, ## args)
#else
static int dprintf(const char* fmt, ...)
{
    return 0;
}

#define _dprintf(fmt, args...) dprintf(fmt, ## args)
#endif

static const char* driver_name = "uhci";

static struct cdi_driver cdi_driver;
static cdi_list_t active_transfers;

static void uhci_handler(struct cdi_device* dev);
static int uhci_do_packet(struct usb_device* usbdev, struct usb_packet* packet);
static cdi_list_t get_devices(struct hci* gen_hci);
static void activate_device(struct hci* gen_hci, struct usb_device* device);
static int get_current_frame(struct hci* gen_hci);
static void uhci_deinit(struct cdi_device* cdi_hci);
static void uhci_kill(struct cdi_driver* cdi_hcd);
static void uhci_reset_device(struct usb_device* device);


struct cdi_driver* init_uhcd()
{
    active_transfers = cdi_list_create();
    cdi_driver.type = CDI_UNKNOWN;
    cdi_driver.name = driver_name;
    cdi_driver.init_device = &uhci_init;
    cdi_driver.remove_device = &uhci_deinit;
    cdi_driver.destroy = &uhci_kill;
    cdi_driver_init(&cdi_driver);
    cdi_driver_register(&cdi_driver);
    return &cdi_driver;
}

static void uhci_kill(struct cdi_driver* cdi_hcd)
{
    //TODO: Da geht doch noch was
}

static void uhci_deinit(struct cdi_device* cdi_hci)
{
    cdi_outw(
        ((struct uhci*) ((struct cdi_hci*) cdi_hci)->hci)->pbase + UHCI_USBCMD,
        MAXP);                                                                                  //HC anhalten
    //TODO: Hier doch auch
}

void uhci_init(struct cdi_device* cdi_hci)
{
    struct hci* gen_hci = ((struct cdi_hci*) cdi_hci)->hci;
    struct uhci* uhci = (struct uhci*) gen_hci;
    struct cdi_pci_resource* res;
    int i, size = 0x14;
    size_t request_size;

    uhci->pbase = 0;
    for (i = 0; (res = cdi_list_get(gen_hci->pcidev->resources, i)) != NULL;
         i++)
    {
        if (res->type == CDI_PCI_IOPORTS) {
            size = res->length ? res->length : 0x14;
            if (cdi_ioports_alloc(res->start, size) == -1) {
                dprintf("I/O-Ports konnten nicht reserviert werden.\n");
                return;
            }
            uhci->pbase = res->start;
            break;
        }
    }
    if (!uhci->pbase) {
        dprintf("I/O-Basis nicht gefunden!\n");
        return;
    }

    request_size =
        // Maximale Paketlaenge (TODO Stimmt das?)
        1024 +
        // Transferdeskriptor
        sizeof(struct uhci_td);

    uhci->buffers = mempool_create(8192, request_size);
    if (uhci->buffers == NULL) {
        dprintf("Speicherpool konnte nicht erzeugt werden\n");
        return;
    }

    if (cdi_alloc_phys_mem(sizeof(*uhci->queue_heads) * 1024,
            (void**) &uhci->queue_heads,
            (void**) &uhci->phys_queue_heads) == -1)
    {
        dprintf("QH-Speicher konnte nicht allociert werden!\n");
        return;
    }
    if (cdi_alloc_phys_mem(4096, (void**) &uhci->frame_list,
            (void**) &uhci->phys_frame_list) == -1)
    {
        dprintf("Frame List konnte nicht allociert werden!\n");
        return;
    }

    cdi_register_irq(gen_hci->pcidev->irq, &uhci_handler, cdi_hci);
    uhci->root_ports = (size - 0x10) >> 1;
    if (uhci->root_ports > 7) { //Laut Linuxkernel ist das so "weird", dass da was nicht stimmen kann...
        uhci->root_ports = 2;
    }
    dprintf("UHC mit I/O 0x%04X (%i Ports) und IRQ %i\n", uhci->pbase,
        uhci->root_ports,
        gen_hci->pcidev->irq);
    for (i = 0; i < 1024; i++) {
        uhci->queue_heads[i].next = 1; //Invalid
        uhci->queue_heads[i].transfer = 1; //Invalid
    }
    for (i = 0; i < 1024; i++) {
        uhci->frame_list[i] = (uintptr_t) &uhci->phys_queue_heads[i] | 2;
    }
    dprintf("Resetten...\n");
    //HC zurücksetzen
    cdi_outw(uhci->pbase + UHCI_USBCMD, MAXP | HCRESET);
    for (i = 0; (cdi_inw(uhci->pbase + UHCI_USBCMD) & HCRESET) && (i < 50);
         i++)
    {
        cdi_sleep_ms(10);
    }
    //Reset auf dem USB treiben
    cdi_outw(uhci->pbase + UHCI_USBCMD, MAXP | GRESET);
    for (i = 0; (cdi_inw(uhci->pbase + UHCI_USBCMD) & GRESET) && (i < 50);
         i++)
    {
        cdi_sleep_ms(10);
    }
    //Alle Interrupts senden
    cdi_outw(uhci->pbase + UHCI_USBINTR, 0xF);
    //Framelistadresse eintragen
    cdi_outl(uhci->pbase + UHCI_FRBASEADD, uhci->phys_frame_list);
    cdi_outw(uhci->pbase + UHCI_FRNUM, 0);      //Frame zurücksetzen
    //Ports abschalten, damit selektiv eingeschaltet werden kann (Enumeration)
    for (i = 0; i < uhci->root_ports; i++) {
        cdi_outw(uhci->pbase + UHCI_RPORTS + i * 2, RPORT_CSC);
    }
    dprintf("  Fertig\n");
    gen_hci->find_devices = &get_devices;
    gen_hci->activate_device = &activate_device;
    gen_hci->do_packet = &uhci_do_packet;
    gen_hci->get_frame = &get_current_frame;

    enumerate_hci(gen_hci);
}

static cdi_list_t get_devices(struct hci* gen_hci)
{
    struct uhci* uhci = (struct uhci*) gen_hci;
    struct usb_device* dev;
    cdi_list_t dlist = cdi_list_create();
    int i;

    dprintf("Geräte finden: ");
    cdi_outw(uhci->pbase + UHCI_USBCMD, USB_RUN);
    for (i = 0; i < uhci->root_ports; i++) {
        if (!(cdi_inw(uhci->pbase + UHCI_RPORTS + 2 * i) & RPORT_DEVICE)) {
            _dprintf("-");
            continue;
        }
        _dprintf("!");
        dev = malloc(sizeof(struct usb_device));
        dev->hci = gen_hci;
        dev->id = 0;
        dev->port = i;
        dev->low_speed =
            (cdi_inw(uhci->pbase + UHCI_RPORTS + 2 * i) & RPORT_LOSPD) && 1;
        dev->reset = &uhci_reset_device;
        cdi_list_push(dlist, dev);
    }
    _dprintf("\n");
    return dlist;
}

static void uhci_reset_device(struct usb_device* device)
{
    struct uhci* uhci = (struct uhci*) device->hci;

    cdi_outw(uhci->pbase + UHCI_RPORTS + 2 * device->port,
        RPORT_ENABLE | RPORT_RESET | RPORT_CSC);
    cdi_sleep_ms(20);
}

static void activate_device(struct hci* gen_hci, struct usb_device* device)
{
    struct uhci* uhci = (struct uhci*) gen_hci;

    dprintf("Gerät an Port %i wird aktiviert.\n", device->port);
    cdi_outw(uhci->pbase + UHCI_RPORTS + 2 * device->port,
        RPORT_ENABLE | RPORT_CSC);
    cdi_sleep_ms(20);
}

static int get_current_frame(struct hci* gen_hci)
{
    return cdi_inw(((struct uhci*) gen_hci)->pbase + UHCI_FRNUM);
}

static inline int tsl(volatile int* variable)
{
    int rval;
    rval = *variable;
    *variable = 1;
    return rval;
}

static volatile int locked = 0;

static int uhci_do_packet(struct usb_device* usbdev, struct usb_packet* packet)
{
    struct uhci* uhci = (struct uhci*) usbdev->hci;
    struct uhci_td td;
    struct transfer addr;
    int timeout, frame;

    void* buf;
    uintptr_t pbuf;

    memset(&td, 0, sizeof(td));
    td.next = 1; //Invalid
    td.active = 1;
    td.ioc = 1;
    td.data_toggle = usbdev->data_toggle;
    td.low_speed = usbdev->low_speed;
    td.errors = 1;
    td.pid = packet->type;
    td.device = usbdev->id;
    td.endpoint = packet->endpoint->endpoint_address & 0x07;
    td.maxlen = packet->length ? packet->length - 1 : 0x7FF;
    while (tsl(&locked)) {
#ifndef CDI_STANDALONE
        __asm__ __volatile__ ("hlt");
#endif
    }
    frame = (cdi_inw(uhci->pbase + UHCI_FRNUM) + 5) & 0x3FF;
    while (!(uhci->queue_heads[frame].transfer & 1)) {
        frame++;
        frame &= 0x3FF;
    }

   if (mempool_get(uhci->buffers, &buf, &pbuf) < 0) {
       // FIXME Oder sowas
       return USB_NAK;
   }

    addr.virt = buf;
    addr.phys = pbuf;
    addr.error = 0xFFFF;

    buf = (void*)((uintptr_t) buf + sizeof(td));
    pbuf += sizeof(td);

    if (!packet->length) {
        td.buffer = 0;
    } else {
        td.buffer = pbuf;
        if (packet->type != PACKET_IN) {
            memcpy(buf, packet->data, packet->length);
        }
    }

    memcpy(addr.virt, &td, sizeof(td));


    uhci->queue_heads[frame].transfer = addr.phys;
    cdi_list_push(active_transfers, &addr);

    locked = 0;
    for (timeout = 0;
         !(uhci->queue_heads[frame].transfer & 1) && (timeout < 1000);
         timeout++)
    {
        cdi_sleep_ms(1);
    }
    while ((timeout < 1000) && (addr.error == 0xFFFF)) {
#ifndef CDI_STANDALONE
        __asm__ __volatile__ ("hlt");
#endif
    }
    if (packet->type == PACKET_IN) {
        memcpy(packet->data, buf, packet->length);
    }
    if (addr.error == 0xFFFF) {
        addr.error = USB_TIMEOUT;
    }

    mempool_put(uhci->buffers, addr.virt);

    return addr.error;
}

static void uhci_handler(struct cdi_device* cdi_hci)
{
    struct uhci* uhci = (struct uhci*) ((struct cdi_hci*) cdi_hci)->hci;
    int status = cdi_inw(uhci->pbase + UHCI_USBSTS), i;
    struct transfer* addr;
    struct uhci_td* td;

    if (!status) { //Also, von hier kommt der IRQ nicht.
        return;
    }
    if (status & ~0x0001) {
        dprintf("Unerwarteter IRQ 0x%04X von %s\n", status, cdi_hci->name);
    }
    if (status & 0x10) {
        printf("[uhci] SCHWERWIEGENDER FEHLER - HC WIRD ANGEHALTEN\n");
        cdi_outw(uhci->pbase + UHCI_USBCMD, MAXP | HCRESET); //FU!
    } else {
        for (i = 0; (addr = cdi_list_get(active_transfers, i)) != NULL; i++) {
            td = addr->virt;
            if (td->active) {
                continue;
            }
            addr->error = USB_NO_ERROR;
            cdi_list_remove(active_transfers, i--);
            if (td->stalled_err) {
                dprintf("ENDPOINT STALLED\n");
                addr->error |= USB_STALLED;
            }
            if (td->buf_err) {
                dprintf("Pufferüberlauf oder Underrun\n");
                addr->error |= USB_BUFFER_ERROR;
            }
            if (td->babble) {
                dprintf("Da war ein Gerät wohl sehr gesprächig: Babble.\n");
                addr->error |= USB_BABBLE;
            }
            if (td->nak) {
                dprintf("NAK empfangen\n");
                addr->error |= USB_NAK;
            }
            if (td->crc_time_err) {
                dprintf("CRC-Fehler oder Timeout\n");
                addr->error |= USB_CRC | USB_TIMEOUT;
            }
            if (td->bitstuff_err) {
                dprintf("Bitstufffehler\n");
                addr->error |= USB_BITSTUFF;
            }
            //TODO: free(td) - dazu wäre eine CDI-Funktion nützlich
        }
    }
    cdi_outw(uhci->pbase + UHCI_USBSTS, status);
}
