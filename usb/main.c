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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cdi/io.h"
#include "cdi/misc.h"
#include "cdi/pci.h"

#include "usb.h"

#define DEBUG

#ifdef DEBUG
#include <stdarg.h>
#define dprintf(fmt, args...) printf("[usb] " fmt, ## args)
#define _dprintf(fmt, args...) printf(fmt, ## args)
#else
static int dprintf(const char* fmt, ...)
{
    return 0;
}

#define _dprintf(fmt, args...) dprintf(fmt, ## args)
#endif

static struct usb_device* devices[128];
static cdi_list_t ehci, ohci, uhci;
static void(*ccdriver_take_device[0x100]) (struct usb_device*);
static int usb_dev_ids = 1;

static void usb_init(void);
static void enumerate_hub(struct usb_device* usbdev);
static void* do_control(struct usb_device* device, int direction, void* buffer,
    int length, int rtype, int recipient, int request, int value,
    int index);

#ifdef CDI_STANDALONE
int main(void)
#else
int init_usb(void)
#endif
{
    cdi_init();

    memset(devices, 0, 128 * sizeof(struct usb_device*));

    usb_init();

    cdi_run_drivers();

    return 0;
}

static const int next_data_type[9] = {
    USB_TOD_SETUP_DATA_IN | USB_TOD_SETUP_DATA_OUT | USB_TOD_SETUP_ACK_IN,
    USB_TOD_SETUP_ACK_OUT,
    USB_TOD_SETUP_ACK_IN,
    USB_TOD_SETUP | USB_TOD_COMMAND,
    USB_TOD_SETUP | USB_TOD_COMMAND,
    USB_TOD_DATA_IN | USB_TOD_DATA_OUT | USB_TOD_STATUS,
    USB_TOD_STATUS | USB_TOD_DATA_IN,
    USB_TOD_STATUS | USB_TOD_DATA_OUT,
    USB_TOD_SETUP | USB_TOD_COMMAND
};

/*static const char *tod_name[9] = {
    "SETUP",
    "SETUP DS IN",
    "SETUP DS OUT",
    "SETUP ACK IN",
    "SETUP ACK OUT",
    "COMMAND",
    "DATA IN",
    "DATA OUT",
    "STATUS"
   };*/

/**
  * Verarbeitet ein USB-Paket.
  *
  * @param device Das USB-Gerät
  */

int usb_do_packet(struct usb_device* device, struct usb_packet* packet)
{
    int error = USB_NAK;
    int tod_short, tod = packet->type_of_data;
    int i = 0, mps;

    if (!(device->expects & tod)) {
        dprintf("0x%04X erwartet, 0x%04X bekommen...?\n", device->expects, tod);
        // FIXME Das ist keine elegante Loesung des Problems
        for (;;) {
        }
    }
    tod >>= 1;
    for (tod_short = 0; tod; tod_short++) {
        tod >>= 1;
    }
    //dprintf("TOD: %s\n", tod_name[tod_short]);
    device->expects = next_data_type[tod_short];
    packet->type &= 0xFF;

    struct usb_packet send_packet = {
        .type      = packet->type,
        .endpoint  = packet->endpoint,
        .data      = packet->data
    };
    mps = packet->endpoint->max_packet_size;

    for (i = 0; (i < packet->length) || (!i && !packet->length); i += mps) {
        send_packet.length =
            (packet->length - i > mps) ? mps : (packet->length - i);
        error = USB_NAK;
        while (error == USB_NAK) {
            error = device->hci->do_packet(device, &send_packet);
            if (error == USB_NAK) {
                cdi_sleep_ms(5);
            }
        }
        if (error != USB_NO_ERROR) {
            i -= mps;
        } else {
            device->data_toggle ^= 1;
            send_packet.data += mps;
        }
        if (error == USB_STALLED) {
            printf("[usb] ENDPOINT %i DES GERÄTS %i STALLED!\n",
                packet->endpoint->endpoint_address,
                device->id);
            do_control(device, HOST_TO_DEV | NO_DATA, NULL, 0, STD_REQUEST,
                REC_ENDPOINT, CLEAR_FEATURE, 0,
                packet->endpoint->endpoint_address);
        }
    }
    return error;
}

static void* do_control(struct usb_device* device, int direction, void* buffer,
    int length, int rtype, int recipient, int request, int value,
    int index)
{
    int rval, no_data;
    struct setup_packet _setup;
    struct setup_packet* setup = &_setup;

    no_data = direction & NO_DATA;
    direction &= 0x80;

    if ((direction != HOST_TO_DEV) || !length || (buffer == NULL)) {
        buffer = malloc(length);
    }

    setup->request_type = direction | (rtype & 0x60) | (recipient & 0x1F);
    setup->request = request;
    setup->value = value;
    setup->index = index;
    setup->length = length;

    struct usb_packet setup_packet = {
        .type         = PACKET_SETUP,
        .endpoint     = device->ep0,
        .data         = setup,
        .length       = sizeof(*setup),
        .type_of_data = USB_TOD_SETUP,
    };

    if (usb_do_packet(device, &setup_packet)) {
        return NULL;
    }

    if (no_data) {
        rval = USB_NO_ERROR;
        buffer = (void*) 0xFFFFFFFF;    //Kein Fehler, aber auch keine Daten
    } else {
        struct usb_packet data_packet = {
            .type         = (direction == DEV_TO_HOST ? PACKET_IN : PACKET_OUT),
            .endpoint     = device->ep0,
            .data         = buffer,
            .length       = length,
            .type_of_data =
                (direction ==
                 DEV_TO_HOST ? USB_TOD_SETUP_DATA_IN : USB_TOD_SETUP_DATA_OUT),
        };

        rval = usb_do_packet(device, &data_packet);
    }

    if (rval == USB_NO_ERROR) {
        struct usb_packet ack_packet = {
            .endpoint   = device->ep0,
            .data       = NULL,
            .length     = 0,
        };
        device->data_toggle = 1;

        if (no_data || (direction == HOST_TO_DEV)) {
            ack_packet.type = PACKET_IN;
            ack_packet.type_of_data = USB_TOD_SETUP_ACK_IN;
        } else {
            ack_packet.type = PACKET_OUT;
            ack_packet.type_of_data = USB_TOD_SETUP_ACK_OUT;
        }

        rval = usb_do_packet(device, &ack_packet);
    }
    return (rval == USB_NO_ERROR) ? buffer : NULL;
}

static void usb_init(void)
{
    struct cdi_pci_device* dev;
    struct hci* hci;
    struct cdi_hci* cdi_hci;
    struct cdi_driver* uhcd;
    char* dev_name;
    int i;

    init_msc_driver();

    memset(ccdriver_take_device, 0, sizeof(void(*) (
                                               struct usb_device*)) * 0x100);
    ccdriver_take_device[8] = &register_msd;
    ccdriver_take_device[9] = &enumerate_hub;

    ehci = cdi_list_create();
    ohci = cdi_list_create();
    uhci = cdi_list_create();

    cdi_list_t pci_devices = cdi_list_create();
    cdi_pci_get_all_devices(pci_devices);
    for (i = 0; (dev = cdi_list_get(pci_devices, i)) != NULL; i++) {
        if ((dev->class_id != 0x0C) || (dev->subclass_id != 0x03)) {
            cdi_pci_device_destroy(dev);
        } else {
            switch (dev->interface_id) {
                case 0x00:
                    hci = malloc(HCI_STRUCT_SIZE);
                    hci->pcidev = dev;
                    hci->type = HCI_UHCI;
                    cdi_list_push(uhci, hci);
                    break;
                case 0x10:
                    hci = malloc(HCI_STRUCT_SIZE);
                    hci->pcidev = dev;
                    hci->type = HCI_OHCI;
                    cdi_list_push(ohci, hci);
                    break;
                case 0x20:
                    hci = malloc(HCI_STRUCT_SIZE);
                    hci->pcidev = dev;
                    hci->type = HCI_EHCI;
                    cdi_list_push(ehci, hci);
            }
        }
    }

    cdi_list_destroy(pci_devices);

    dprintf("%i EHCIs, %i OHCIs und %i UHCIs gefunden.\n", cdi_list_size(
            ehci), cdi_list_size(ohci), cdi_list_size(uhci));

    uhcd = init_uhcd();

    for (i = 0; (hci = cdi_list_pop(uhci)) != NULL; i++) {
        cdi_hci = malloc(sizeof(struct cdi_hci));
        cdi_hci->cdi_device.type = CDI_UNKNOWN;
        dev_name = malloc(10);
        sprintf(dev_name, "uhci%i", i);
        cdi_hci->cdi_device.name = dev_name;
        hci->find_devices = NULL;
        cdi_hci->hci = hci;
        //cdi_list_push(uhcd->devices, cdi_hci);
        //^ Das ist zwar sehr schön, geht so aber leider nicht
        dprintf("%s registriert.\n", dev_name);
        cdi_hci->cdi_device.driver = uhcd;
        uhci_init(&cdi_hci->cdi_device);
    }

    cdi_list_destroy(ehci);
    cdi_list_destroy(ohci);
    cdi_list_destroy(uhci);
}

static void enum_device(struct usb_device* usbdev)
{
    struct device_desc* dev_desc;
    struct config_desc* conf_desc;
    struct interface_desc* if_desc, * best_if;
    char* name;
    void* position;
    int i, id;

    //Gerät und EP0 initialisieren
    usbdev->locked = 0;
    usbdev->expects = USB_TOD_SETUP | USB_TOD_COMMAND;
    usbdev->data_toggle = 0;
    usbdev->ep0 = malloc(sizeof(*usbdev->ep0));
    usbdev->ep0->length = sizeof(*usbdev->ep0);
    usbdev->ep0->descriptor_type = DESC_ENDPOINT;
    usbdev->ep0->endpoint_address = 0;
    usbdev->ep0->attributes = 0;
    usbdev->ep0->max_packet_size = 8;
    usbdev->ep0->interval = 0;

    //Resetten
    usbdev->reset(usbdev);

    //Erste acht Bytes des Device-Descriptors abrufen und die maximale Paketgröße von EP0 feststellen
    dev_desc =
        do_control(usbdev, DEV_TO_HOST, NULL, 8, STD_REQUEST, REC_DEVICE,
            GET_DESCRIPTOR,
            DESC_DEVICE << 8,
            0);
    if (dev_desc == NULL) {
        return;
    }
    usbdev->ep0->max_packet_size = dev_desc->max_packet_size0;

    //Nochmals resetten
    usbdev->reset(usbdev);

    //USB-Adresse zuweisen
    do_control(usbdev, HOST_TO_DEV | NO_DATA, NULL, 0, STD_REQUEST, REC_DEVICE,
        SET_ADDRESS, (id = usb_dev_ids++),
        0);
    usbdev->id = id;

    //Den ganzen Device-Descriptor einlesen
    dev_desc =
        do_control(usbdev, DEV_TO_HOST, NULL, sizeof(*dev_desc), STD_REQUEST,
            REC_DEVICE, GET_DESCRIPTOR, DESC_DEVICE << 8,
            0);
    usbdev->device = dev_desc;
    dprintf("0x%04X:0x%04X (%i) -> %i\n", dev_desc->vendor_id,
        dev_desc->device_id, dev_desc->class_id,
        id);

    if (dev_desc->iManufacturer) {
        name =
            do_control(usbdev, DEV_TO_HOST, NULL, 64, STD_REQUEST, REC_DEVICE,
                GET_DESCRIPTOR, (DESC_STRING << 8) | dev_desc->iManufacturer,
                0);
        if (name == NULL) {
            return;
        }
        dprintf(" -> Hersteller: ");
        for (i = 2; i < name[0]; i += 2) {
            _dprintf("%c", name[i]);
        }
        _dprintf("\n");
    }
    if (dev_desc->iProduct) {
        name =
            do_control(usbdev, DEV_TO_HOST, NULL, 64, STD_REQUEST, REC_DEVICE,
                GET_DESCRIPTOR, (DESC_STRING << 8) | dev_desc->iProduct,
                0);
        if (name == NULL) {
            return;
        }
        dprintf(" -> Produkt: ");
        for (i = 2; i < name[0]; i += 2) {
            _dprintf("%c", name[i]);
        }
        _dprintf("\n");
    }
    devices[id] = usbdev;
    //TODO: Man kann doch nicht immer den ersten nehmen...
    conf_desc =
        do_control(usbdev, DEV_TO_HOST, NULL, sizeof(struct config_desc),
            STD_REQUEST,
            REC_DEVICE, GET_DESCRIPTOR, DESC_CONFIGURATION << 8,
            0);
    if (conf_desc == NULL) {
        return;
    }
    conf_desc =
        do_control(usbdev, DEV_TO_HOST, NULL, conf_desc->total_length,
            STD_REQUEST,
            REC_DEVICE, GET_DESCRIPTOR, DESC_CONFIGURATION << 8,
            0);
    if (conf_desc == NULL) {
        return;
    }
    usbdev->config = conf_desc;
    do_control(usbdev, HOST_TO_DEV, NULL, 0, STD_REQUEST, REC_DEVICE,
        SET_CONFIGURATION, conf_desc->config_value,
        0);
    if (conf_desc->iConfiguration) {
        name =
            do_control(usbdev, DEV_TO_HOST, NULL, 64, STD_REQUEST, REC_DEVICE,
                GET_DESCRIPTOR, (DESC_STRING << 8) | conf_desc->iConfiguration,
                0);
        if (name == NULL) {
            return;
        }
        dprintf("Verwende Konfiguration ");
        for (i = 2; i < name[0]; i += 2) {
            _dprintf("%c", name[i]);
        }
        _dprintf(".\n");
    }
    position = conf_desc;
    position += sizeof(struct config_desc);
    best_if = position; //Standard-IF
    for (i = 0; i < conf_desc->num_interfaces; i++) {
        if_desc = position;
        //TODO: Mehr Klassencodes natürlich!
        if (if_desc->interface_class == 8) {
            best_if = if_desc;
            break;
        }
        position += sizeof(struct interface_desc) + if_desc->num_endpoints *
                    sizeof(struct endpoint_desc);
    }
    do_control(usbdev, HOST_TO_DEV, NULL, 0, STD_REQUEST, REC_DEVICE,
        SET_INTERFACE, best_if->interface_number,
        0);
    usbdev->interface = best_if;
    if (best_if->iInterface) {
        name =
            do_control(usbdev, DEV_TO_HOST, NULL, 64, STD_REQUEST, REC_DEVICE,
                GET_DESCRIPTOR, (DESC_STRING << 8) | best_if->iInterface,
                0);
        if (name == NULL) {
            return;
        }
        dprintf("Verwende Interface ");
        for (i = 2; i < name[0]; i += 2) {
            _dprintf("%c", name[i]);
        }
        _dprintf(".\n");
    }
    dprintf("Konfiguration: %i:%i (%i)\n", conf_desc->config_value,
        best_if->interface_number,
        best_if->interface_class);
    if (ccdriver_take_device[best_if->interface_class] != NULL) {
        ccdriver_take_device[best_if->interface_class](usbdev);
    }
}

static void reset_hub_device(struct usb_device* usbdev)
{
    do_control(usbdev->hub, HOST_TO_DEV, NULL, 0, CLS_REQUEST, REC_OTHER,
        SET_FEATURE, PORTF_RESET,
        usbdev->port + 1);
    cdi_sleep_ms(20);
}

static void enumerate_hub(struct usb_device* usbdev)
{
    struct hub_desc* hub_desc;
    struct usb_device* down;
    uint16_t* port_status;
    int i;

    //Mehr als 8 Ports sollte es nicht geben, die Größe des Deskriptors wird
    //daher wohl nicht größer als sizeof()+16 werden
    hub_desc =
        do_control(usbdev, DEV_TO_HOST, NULL, sizeof(*hub_desc) + 16,
            CLS_REQUEST,
            REC_DEVICE, GET_DESCRIPTOR, DESC_DEVICE << 8,
            0);
    for (i = 0; i < hub_desc->nbr_ports; i++) {
        port_status =
            do_control(usbdev, DEV_TO_HOST, NULL, 4, CLS_REQUEST, REC_OTHER,
                GET_STATUS, 0,
                i + 1);
        if (!(port_status[0] & PORT_DEVICE)) {
            continue;
        }
        //Strom an
        do_control(usbdev, HOST_TO_DEV, NULL, 0, CLS_REQUEST, REC_OTHER,
            SET_FEATURE, PORTF_POWER,
            i + 1);
        cdi_sleep_ms(hub_desc->pwron2pwrgood * 2);
        down = malloc(sizeof(struct usb_device));
        down->hci = usbdev->hci;
        down->hub = usbdev;
        down->id = 0;
        down->port = i;
        down->low_speed = (port_status[0] & PORT_LOWSPEED) && 1;
        down->reset = &reset_hub_device;
        enum_device(down);
    }
}

void enumerate_hci(struct hci* hci)
{
    struct usb_device* usbdev;
    cdi_list_t usb_devices;

    if (hci->find_devices != NULL) {
        usb_devices = hci->find_devices(hci);
        while ((usbdev = cdi_list_pop(usb_devices)) != NULL) {
            hci->activate_device(hci, usbdev);
            cdi_sleep_ms(50);
            enum_device(usbdev);
        }
    }
}
