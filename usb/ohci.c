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

#include <string.h>

#include "cdi/io.h"
#include "cdi/lists.h"
#include "cdi/misc.h"
#include "cdi/pci.h"

#include "mempool.h"
#include "ohci.h"

#define DEBUG

#ifdef DEBUG
#include <stdio.h>
#include <stdarg.h>
#define dprintf(fmt, args...) printf("[ohci] " fmt, ## args)
#define _dprintf(fmt, args...) printf(fmt, ## args)
#else
static int dprintf(const char* fmt, ...)
{
    return 0;
}

#define _dprintf(fmt, args...) dprintf(fmt, ## args)
#endif

static struct cdi_driver cdi_driver;
static const char* driver_name = "ohcd";

static void ohci_kill(struct cdi_driver* cdi_hcd);
static void ohci_deinit(struct cdi_device* cdi_hci);
void ohci_init(struct cdi_device* cdi_hci);
static void ohci_handler(struct cdi_device* dev);
static int ohci_do_packets(struct usb_packet* packet, int num_packets);
static cdi_list_t get_devices(struct hci* gen_hci);
static void activate_device(struct usb_device* device);
static void ohci_reset_device(struct usb_device* device);
static void ohci_establish_pipe(struct usb_pipe* pipe);

struct cdi_driver* init_ohcd()
{
    cdi_driver.type = CDI_UNKNOWN;
    cdi_driver.name = driver_name;
    cdi_driver.init_device = &ohci_init;
    cdi_driver.remove_device = &ohci_deinit;
    cdi_driver.destroy = &ohci_kill;
    cdi_driver_init(&cdi_driver);
    cdi_driver_register(&cdi_driver);
    return &cdi_driver;
}

static void ohci_kill(struct cdi_driver* cdi_hcd)
{
    //TODO: Da geht doch noch was
}

static void ohci_deinit(struct cdi_device* cdi_hci)
{
    cdi_outw(
        ((struct uhci*) ((struct cdi_hci*) cdi_hci)->hci)->pbase + UHCI_USBCMD,
        MAXP);                                                                                  //HC anhalten
    //TODO: Hier doch auch
}

void ohci_init(struct cdi_device* cdi_hci)
{
    struct hci* gen_hci = ((struct cdi_hci*) cdi_hci)->hci;
    struct ohci* ohci = (struct ohci*) gen_hci;
    struct cdi_pci_resource* res;
    int i;
    uintptr_t phys_hcca;

    ohci->memory = NULL;
    for (i = 0; (res = cdi_list_get(gen_hci->pcidev->resources, i)) != NULL;
         i++)
    {
        if (res->type == CDI_PCI_MEMORY) {
            ohci->memory = cdi_alloc_phys_addr(res->length, res->start);
            if (ohci->memory == NULL) {
                dprintf("Speicher konnten nicht allokiert werden.\n");
                return;
            }
            break;
        }
    }
    if (ohci->memory == NULL) {
        dprintf("Speicher nicht gefunden!\n");
        return;
    }
    dprintf("Speicher @ 0x%08X\n", (uintptr_t) ohci->memory);
    if ((ohci->memory->hc_revision & 0xFF) != 0x10) {
        dprintf(
            "Dieses OHCI könnte inkompatibel mit dem Treiber sein (HCI %i.%i, HCD 1.0)\n",
            (ohci->memory->hc_revision & 0xF0) >> 4,
            ohci->memory->hc_revision & 0xF);
    }
    if (cdi_alloc_phys_mem(sizeof(*ohci->hcca), (void**) &ohci->hcca,
            (void**) &phys_hcca) == -1)
    {
        dprintf("Konnte den HCCA nicht allokieren.\n");
        return;
    }
    memset(ohci->hcca, 0, sizeof(*ohci->hcca));

    dprintf("IRQ: %i\n", gen_hci->pcidev->irq);
    cdi_register_irq(gen_hci->pcidev->irq, &ohci_handler, cdi_hci);

    ohci->ed_pool =
        mempool_create(64 * sizeof(struct ohci_ed), sizeof(struct ohci_ed));
    if (ohci->ed_pool == NULL) {
        dprintf("ED-Speicherpool konnte nicht erzeugt werden!\n");
        return;
    }

    ohci->transfer_pool = mempool_create(56 * (1024 + sizeof(struct ohci_td)),
        1024 + sizeof(struct ohci_td));
    if (ohci->transfer_pool == NULL) {
        dprintf("Transfer-Speicherpool konnte nicht erzeugt werden!\n");
        return;
    }

    ohci->memory->hc_interrupt_disable = OHC_INT_MIE;

    if (ohci->memory->hc_control & OHC_CTRL_IR) {
        dprintf("Übernahme der Kontrolle vom SMM... ");
        //Änderung des "Besitzers" beantragen
        ohci->memory->hc_command_status |= OHC_CMST_OCR;
        for (i = 0; (ohci->memory->hc_control & OHC_CTRL_IR) && (i < 1000);
             i++)
        {
            cdi_sleep_ms(1);
        }
        if (i < 100) {
            _dprintf("Er hat mitgespielt.\n");
        } else {
            _dprintf("Idiot.\n");
            ohci->memory->hc_control &= ~OHC_CTRL_IR;
        }
    } else if ((ohci->memory->hc_control & OHC_CTRL_CBSR) != OHC_USB_RESET) {
        dprintf("BIOS aktiv\n");
        if ((ohci->memory->hc_control & OHC_CTRL_CBSR) !=
            OHC_USB_OPERATIONAL)
        {
            dprintf("RESUME aktivieren...\n");
            ohci->memory->hc_control =
                (ohci->memory->hc_control & ~OHC_CTRL_CBSR) | OHC_USB_RESUME;
            cdi_sleep_ms(10);
        }
    } else { //Nix, weder BIOS noch SMM
        dprintf("RESET abwarten\n");
        cdi_sleep_ms(10); //Reset abwarten
    }

    dprintf("Resetten...\n");
    ohci->memory->hc_command_status |= OHC_CMST_RESET;
    //Normalerweise sollte man den Wert von hc_fm_interval von vor dem Reset
    //wiederherstellen, aber ich finde das Schwachsinn. Die Werte sind so in
    //Ordnung, wie sie jetzt sind.

    cdi_sleep_ms(3); //10 µs für den Reset und 2 ms für Resume
    if ((ohci->memory->hc_control & OHC_CTRL_CBSR) == OHC_USB_SUSPEND) { //Boah...
        dprintf("Das Ding wurde nicht fertig...\n");
        ohci->memory->hc_control =
            (ohci->memory->hc_control & ~OHC_CTRL_CBSR) | OHC_USB_RESUME;
        cdi_sleep_ms(10);
    }

    ohci->memory->hc_interrupt_disable = OHC_INT_MIE;

    ohci->memory->hc_hcca = phys_hcca;
    ohci->memory->hc_interrupt_status = 0xFFFFFFFF;
    ohci->memory->hc_interrupt_enable =
        OHC_INT_SO   |  //Scheduling overrun
        OHC_INT_WDH  |  //Write back done head
        OHC_INT_RD   |  //Resume detected
        OHC_INT_UE   |  //Schwerer Fehler
        OHC_INT_RHSC |  //Root hub status change
        OHC_INT_MIE;
    //Eigentlich sollte man alle Transfers aktivieren. Aber wir wollen ja noch
    //gar keine periodischen Transfers...
    //Periodisches Zeug deaktivieren
    ohci->memory->hc_control &= ~(OHC_CTRL_PLE | OHC_CTRL_IE);
    //Control und Bulk aktivieren
    ohci->memory->hc_control |= OHC_CTRL_CLE | OHC_CTRL_BLE;
    //Merkwürdigerweise soll das jetzt nicht auf 10 %, sondern auf 90 % gesetzt
    //werden. Aber das klingt total bescheuert, weil nicht erklärt wird, wann
    //man es denn auf 10 % setzen soll. Deshalb machen wir das gleich.
    ohci->memory->hc_periodic_start = 0x4B0;

    dprintf("HC wird aktiviert.\n");

    //DO IT, MAN!
    ohci->memory->hc_control =
        (ohci->memory->hc_control & ~OHC_CTRL_CBSR) | OHC_USB_OPERATIONAL;

    ohci->memory->hc_rh_status |= OHC_RHS_LPSC; //Strom andrehen
    ohci->root_ports = ohci->memory->hc_rh_descriptor_a & OHC_RHA_NDP;
    cdi_sleep_ms((ohci->memory->hc_rh_descriptor_a & OHC_RHA_POTPGT) >> 23);

    dprintf("%i Rootports gefunden.\n", ohci->root_ports);

    for (i = 0; i < ohci->root_ports; i++) {
        ohci->memory->hc_rh_port_status[i] |= OHC_RP_CCS; //Deaktivieren

    }
    ohci->ed_list = cdi_list_create();

    gen_hci->find_devices = &get_devices;
    gen_hci->activate_device = &activate_device;
    gen_hci->do_packets = &ohci_do_packets;
    gen_hci->add_pipe = &ohci_establish_pipe;

    enumerate_hci(gen_hci);
}

static inline int tsl(volatile int* variable)
{
    int rval;
    rval = *variable;
    *variable = 1;
    return rval;
}

static int ohci_do_packets(struct usb_packet* packets, int num_packets)
{
    struct ohci_ed_desc* edsc;
    struct ohci_td_desc* tdsc, * otdsc;
    int i, j, cond, toggle, done_packets;
    struct ohci* ohci = (struct ohci*) packets[0].pipe->device->hci;
    struct ohci_td* vtd[num_packets];
    uintptr_t ptd[num_packets];
    static volatile int locked = 0;

    for (i = 0; (edsc = cdi_list_get(ohci->ed_list, i)) != NULL; i++) {
        if ((edsc->function == packets[0].pipe->device->id) &&
            (edsc->endpoint == packets[0].pipe->endpoint->endpoint_address))
        {
            break;
        }
    }
    if (edsc == NULL) {
        return USB_TIMEOUT; //Genau das würde passieren
    }
    toggle = packets[0].pipe->data_toggle;
    for (i = 0; i < num_packets; i++) {
        if (mempool_get(ohci->transfer_pool, (void**) &vtd[i],
                &ptd[i]) == -1)
        {
            return USB_TRIVIAL_ERROR;
        }
        if ((packets[i].data != NULL) && (packets[i].type != PACKET_IN)) {
            memcpy((void*) vtd[i] + sizeof(struct ohci_td), packets[i].data,
                packets[i].length);
        }
        vtd[i]->rounding = 1; //Warum nicht
        switch (packets[i].type) {
            case PACKET_SETUP:
                vtd[i]->direction = OHC_TD_DIR_SETUP;
                break;
            case PACKET_IN:
                vtd[i]->direction = OHC_TD_DIR_IN;
                break;
            case PACKET_OUT:
                vtd[i]->direction = OHC_TD_DIR_OUT;
                break;
            default: //Hm, passt nicht ganz
                for (j = 0; j <= i; j++) {
                    mempool_put(ohci->transfer_pool, vtd[j]);
                }
                return USB_TRIVIAL_ERROR;
        }
        vtd[i]->di = 0;
        if (packets[i].use_toggle == TOGGLE_0) {
            toggle = 0;
        } else if (packets[i].use_toggle == TOGGLE_1) {
            toggle = 1;
        }
        vtd[i]->toggle = 0x2 | toggle;
        toggle ^= 1;
        vtd[i]->error = 0;
        vtd[i]->condition = 15;
        if ((packets[i].data == NULL) || !packets[i].length) {
            vtd[i]->current_buffer_pointer = 0;
            vtd[i]->buffer_end = 0;
        } else {
            vtd[i]->current_buffer_pointer = ptd[i] + sizeof(struct ohci_td);
            vtd[i]->buffer_end = ptd[i] + sizeof(struct ohci_td) +
                                 packets[i].length - 1;
        }
        vtd[i]->next_td = 0;
        tdsc = malloc(sizeof(*tdsc));
        if (tdsc == NULL) {
            for (j = 0; j <= i; j++) {
                mempool_put(ohci->transfer_pool, vtd[j]);
            }
            return USB_TRIVIAL_ERROR;
        }
        tdsc->virt = vtd[i];
        tdsc->phys = ptd[i];
        tdsc->endpoint = edsc;

        while (tsl(&locked)) {
#ifndef CDI_STANDALONE
            __asm__ __volatile__ ("hlt");
#endif
        }
        if (cdi_list_size(edsc->transfers)) {
            otdsc = cdi_list_get(edsc->transfers, 0);
            otdsc->virt->next_td = ptd[i];
        }
        if (!edsc->virt->td_queue_head) {
            edsc->virt->td_queue_head = ptd[i];
        }
        locked = 0;

        cdi_list_push(edsc->transfers, tdsc);
    }
    packets[0].pipe->data_toggle = toggle;
    if (edsc->type == USB_CONTROL) {
        ohci->memory->hc_command_status |= OHC_CMST_CLF;
    } else if (edsc->type == USB_BULK) {
        ohci->memory->hc_command_status |= OHC_CMST_BLF;
    }
    done_packets = 0;
    cond = USB_NO_ERROR;
    while (done_packets < num_packets) {
        cdi_sleep_ms(1);
        for (i = 0; i < num_packets; i++) {
            if (vtd[i]->condition != 15) {
                done_packets++;
                switch (vtd[i]->condition) {
                    case 0x00:
                        packets[i].condition = USB_NO_ERROR;
                        break;
                    case 0x02:
                        packets[i].condition = USB_BITSTUFF;
                        break;
                    case 0x04:
                        packets[i].condition = USB_STALLED;
                        for (j = 0; j < num_packets; j++) {
                            vtd[j]->condition = 0x04; //Allet, was nicht fertig ist, ist STALLed
                        }
                        break;
                    case 0x05:
                        packets[i].condition = USB_TIMEOUT;
                        break;
                    case 0x08:
                        packets[i].condition = USB_BABBLE;
                        break;
                    case 0x0C:
                    case 0x0D:
                        packets[i].condition = USB_BUFFER_ERROR;
                        break;
                    default:
                        packets[i].condition = USB_CRC;
                        break;
                }
                cond |= packets[i].condition;
                vtd[i]->condition = 15; //Nicht nochmal überprüfen
            }
        }
    }
    for (i = 0; i < num_packets; i++) {
        if ((packets[i].data != NULL) && (packets[i].type == PACKET_IN)) {
            memcpy(packets[i].data, (void*) vtd[i] + sizeof(struct ohci_td),
                packets[i].length);
        }
        if (vtd[i]->next_td == 0) {
            edsc->virt->td_queue_head = 0;
        }
        mempool_put(ohci->transfer_pool, vtd[i]);
    }
    if (cond & USB_STALLED) { //Transferliste für diesen Endpoint leeren
        edsc->virt->td_queue_head = 0;
        while ((tdsc = cdi_list_pop(edsc->transfers)) != NULL) {
            free(tdsc);
        }
        edsc->virt->td_queue_tail = 0; //Weitermachen
    }
    return cond;
}

static cdi_list_t get_devices(struct hci* gen_hci)
{
    struct ohci* ohci = (struct ohci*) gen_hci;
    struct usb_device* dev;
    cdi_list_t dlist = cdi_list_create();
    int i;

    dprintf("Geräte finden: ");
    for (i = 0; i < ohci->root_ports; i++) {
        if (!(ohci->memory->hc_rh_port_status[i] & OHC_RP_CCS)) {
            _dprintf("-");
            continue;
        }
        _dprintf("!");
        dev = malloc(sizeof(struct usb_device));
        dev->hci = gen_hci;
        dev->id = 0;
        dev->port = i;
        dev->low_speed = !!(ohci->memory->hc_rh_port_status[i] & OHC_RP_LSDA);
        dev->reset = &ohci_reset_device;
        cdi_list_push(dlist, dev);
    }
    _dprintf("\n");
    return dlist;
}

static void activate_device(struct usb_device* device)
{
    struct ohci* ohci = (struct ohci*) device->hci;

    dprintf("Gerät an Port %i wird aktiviert.\n", device->port);
    ohci->memory->hc_rh_port_status[device->port] |= OHC_RP_PES;
    cdi_sleep_ms(20);
}

static void ohci_reset_device(struct usb_device* device)
{
    struct ohci* ohci = (struct ohci*) device->hci;

    ohci->memory->hc_rh_port_status[device->port] |= OHC_RP_PRS;
    cdi_sleep_ms(20);
}

static void ohci_establish_pipe(struct usb_pipe* pipe)
{
    struct ohci* ohci = (struct ohci*) pipe->device->hci;
    struct ohci_ed* ved;
    uintptr_t ped;
    struct ohci_ed_desc* dsc;
    //TODO Mehr als Control und Bulk
    int iscontrol = 0;

    if (mempool_get(ohci->ed_pool, (void**) &ved, &ped) == -1) {
        return;
    }
    ved->function = pipe->device->id;
    ved->endpoint = pipe->endpoint->endpoint_address;
    if (ved->endpoint == 0) { //EP0
        ved->direction = OHC_ED_DIR_TD;
        iscontrol = 1;
    } else if (pipe->endpoint->endpoint_address & 0x80) {
        ved->direction = OHC_ED_DIR_IN;
    } else {
        ved->direction = OHC_ED_DIR_OUT;
    }
    ved->low_speed = pipe->device->low_speed;
    ved->skip = 0;
    ved->format = 0;
    ved->mps = pipe->endpoint->max_packet_size;
    ved->td_queue_tail = 0;
    ved->td_queue_head = 0;
    dsc = cdi_list_get(ohci->ed_list, 0);
    if (dsc == NULL) {
        ved->next_ed = 0;
    } else {
        ved->next_ed = dsc->phys;
    }
    dsc = malloc(sizeof(*dsc));
    if (dsc == NULL) {
        mempool_put(ohci->ed_pool, ved);
        return;
    }
    dsc->virt = ved;
    dsc->phys = ped;
    dsc->function = pipe->device->id;
    dsc->endpoint = pipe->endpoint->endpoint_address;
    dsc->type = iscontrol ? USB_CONTROL : USB_BULK;
    dsc->transfers = cdi_list_create();
    cdi_list_push(ohci->ed_list, dsc);

    if (iscontrol) {
        ohci->memory->hc_control_head_ed = ped;
    } else {
        ohci->memory->hc_bulk_head_ed = ped;
    }
}

static void ohci_handler(struct cdi_device* dev)
{
    struct ohci* ohci = (struct ohci*) ((struct cdi_hci*) dev)->hci;
    struct ohci_ed_desc* edsc;
    struct ohci_td_desc* tdsc;
    uint32_t status, done = 0;
    int i, j;
    uintptr_t phys;

    status = ohci->memory->hc_interrupt_status;

    if (status & OHC_INT_RHSC) {
        done |= OHC_INT_RHSC;
    }
    if (status & OHC_INT_UE) {
        dprintf("Schwerer HC-Fehler.\n");
        ohci->memory->hc_command_status |= OHC_CMST_RESET;
        done |= OHC_INT_UE;
    }
    if (status & OHC_INT_SF) { //Öhm... Na ja.
        done |= OHC_INT_SF;
    }
    if (status & OHC_INT_WDH) { //Paket gesendet
        done |= OHC_INT_WDH;
        phys = ohci->hcca->done_head;
        for (i = 0; (edsc = cdi_list_get(ohci->ed_list, i)) != NULL; i++) {
            for (j = 0; (tdsc = cdi_list_get(edsc->transfers, j)) != NULL;
                 j++)
            {
                if (tdsc->phys == phys) { //Das ist der fertige Transfer
                    cdi_list_remove(edsc->transfers, j);
                    free(tdsc);
                    break;
                }
            }
            if (tdsc != NULL) { //Gefunden
                break;
            }
        }
    }

    if (status & ~done) {
        dprintf("Nicht behandelter Interrupt: 0x%08X\n", status & ~done);
    }

    ohci->memory->hc_interrupt_status = status;
}
