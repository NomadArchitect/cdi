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
static int ohci_do_packet(struct usb_packet* packet);
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
    dprintf("Speicher @ 0x%08x\n", (uintptr_t) ohci->memory);
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

    ohci->transfer_pool = mempool_create(32 * (1024 + sizeof(struct ohci_td)),
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
    gen_hci->do_packet = &ohci_do_packet;
    gen_hci->add_pipe = &ohci_establish_pipe;

    enumerate_hci(gen_hci);
}

static int ohci_do_packet(struct usb_packet* packet)
{
    struct ohci_ed_desc* edsc;
    struct ohci_td_desc* tdsc, * otdsc;
    int i, cond;
    struct ohci* ohci = (struct ohci*) packet->pipe->device->hci;
    struct ohci_td* vtd;
    uintptr_t ptd;

    for (i = 0; (edsc = cdi_list_get(ohci->ed_list, i)) != NULL; i++) {
        if ((edsc->function == packet->pipe->device->id) &&
            (edsc->endpoint == packet->pipe->endpoint->endpoint_address))
        {
            break;
        }
    }
    if (edsc == NULL) {
        return USB_TIMEOUT; //Genau das würde passieren
    }
    if (mempool_get(ohci->transfer_pool, (void**) &vtd, &ptd) == -1) {
        return USB_TRIVIAL_ERROR;
    }
    if ((packet->data != NULL) && (packet->type != PACKET_IN)) {
        memcpy((void*) vtd + sizeof(struct ohci_td), packet->data,
            packet->length);
    }
    vtd->rounding = 1; //Warum nicht
    switch (packet->type) {
        case PACKET_SETUP:
            vtd->direction = OHC_TD_DIR_SETUP;
            break;
        case PACKET_IN:
            vtd->direction = OHC_TD_DIR_IN;
            break;
        case PACKET_OUT:
            vtd->direction = OHC_TD_DIR_OUT;
            break;
        default:
            mempool_put(ohci->transfer_pool, vtd);
            return USB_TRIVIAL_ERROR; //Hm, passt nicht ganz
    }
    vtd->di = 0;
    vtd->toggle = 0x2 | packet->pipe->data_toggle;
    vtd->error = 0;
    vtd->condition = 15;
    if ((packet->data == NULL) || !packet->length) {
        vtd->current_buffer_pointer = 0;
        vtd->buffer_end = 0;
    } else {
        vtd->current_buffer_pointer = ptd + sizeof(struct ohci_td);
        vtd->buffer_end = ptd + sizeof(struct ohci_td) + packet->length - 1;
    }
    vtd->next_td = 0;
    tdsc = malloc(sizeof(*tdsc));
    if (tdsc == NULL) {
        mempool_put(ohci->transfer_pool, vtd);
        return USB_TRIVIAL_ERROR;
    }
    tdsc->virt = vtd;
    tdsc->phys = ptd;
    tdsc->endpoint = edsc;
    if (cdi_list_size(edsc->transfers)) {
        otdsc = cdi_list_get(edsc->transfers, cdi_list_size(
                edsc->transfers) - 1);
        otdsc->virt->next_td = ptd;
    }
    cdi_list_insert(edsc->transfers, cdi_list_size(edsc->transfers), tdsc);
    if (!edsc->virt->td_queue_head) {
        edsc->virt->td_queue_head = ptd;
    }
    if (edsc->type == USB_CONTROL) {
        ohci->memory->hc_command_status |= OHC_CMST_CLF;
    } else if (edsc->type == USB_BULK) {
        ohci->memory->hc_command_status |= OHC_CMST_BLF;
    }
    while (vtd->condition == 15) {
        cdi_sleep_ms(1);
    }
    cond = vtd->condition;
    if ((packet->data != NULL) && (packet->type == PACKET_IN)) {
        memcpy(packet->data, (void*) vtd + sizeof(struct ohci_td),
            packet->length);
    }
    if (vtd->next_td == 0) {
        edsc->virt->td_queue_head = 0;
    }
    mempool_put(ohci->transfer_pool, vtd);
    switch (cond) {
        case 0x00:
            return USB_NO_ERROR;
        case 0x01:
            return USB_CRC;
        case 0x02:
            return USB_BITSTUFF;
        case 0x03: //Datatoggle-Fehler
            return USB_CRC;
        case 0x04:
            return USB_STALLED;
        case 0x05:
            return USB_TIMEOUT;
        case 0x06: //PID-Fehler
        case 0x07: //Ebenso
            return USB_CRC;
        case 0x08:
            return USB_BABBLE;
        case 0x09: //Data underrun
            return USB_CRC;
        case 0x0C: //Buffer overrun
        case 0x0D: //Buffer underrun
            return USB_BUFFER_ERROR;
        default: //Öhm...
            return USB_CRC;
    }
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

    dprintf(
        "%s-Endpoint %i (Gerät %i) hinzugefügt (%s)\n",
        iscontrol ? "Control" : "Bulk", ved->endpoint, ved->function,
        (ved->direction ==
         OHC_ED_DIR_TD) ? "IN/OUT/SETUP" : ((ved->direction ==
                                             OHC_ED_DIR_IN) ? "IN" : "OUT"));

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
