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

#ifndef _CDI__USB__USB_H
#define _CDI__USB__USB_H

#include <stdint.h>

#include "cdi/misc.h"
#include "cdi/pci.h"

enum usb_packet_type {
    PACKET_SETUP = 0x2D,
    PACKET_IN = 0x69,
    PACKET_OUT = 0xE1
};

#define NO_DATA                        0x01
#define DEV_TO_HOST                    0x80
#define HOST_TO_DEV                    0x00
#define STD_REQUEST                    0x00
#define CLS_REQUEST                    0x20
#define VEN_REQUEST                    0x40
#define REC_DEVICE                     0x00
#define REC_INTERFACE                  0x01
#define REC_ENDPOINT                   0x02
#define REC_OTHER                      0x03

#define GET_STATUS                     0
#define CLEAR_FEATURE                  1
#define SET_FEATURE                    3
#define SET_ADDRESS                    5
#define GET_DESCRIPTOR                 6
#define SET_DESCRIPTOR                 7
#define GET_CONFIGURATION              8
#define SET_CONFIGURATION              9
#define GET_INTERFACE                  10
#define SET_INTERFACE                  11
#define SYNC_FRAME                     12

#define PORT_DEVICE                    0x0001
#define PORT_ENABLED                   0x0002
#define PORT_SUSPENDED                 0x0004
#define PORT_OVERCURRENT               0x0008
#define PORT_RESET                     0x0010
#define PORT_POWER                     0x0100
#define PORT_LOWSPEED                  0x0200
#define PORT_HIGHSPEED                 0x0400
#define PORT_TEST                      0x0800
#define PORT_INDICATOR                 0x1000

#define PORTF_CONNECTION               0
#define PORTF_ENABLE                   1
#define PORTF_SUSPEND                  2
#define PORTF_OVERCURRENT              3
#define PORTF_RESET                    4
#define PORTF_POWER                    8
#define PORTF_LOWSPEED                 9
#define PORTF_HIGHSPEED                10
#define PORTF_C_CONNECTION             16
#define PORTF_C_ENABLE                 17
#define PORTF_C_SUSPEND                18
#define PORTF_C_OVERCURRENT            19
#define PORTF_C_RESET                  20
#define PORTF_TEST                     21
#define PORTF_INDICATOR                22

#define DESC_DEVICE                    1
#define DESC_CONFIGURATION             2
#define DESC_STRING                    3
#define DESC_INTERFACE                 4
#define DESC_ENDPOINT                  5
#define DESC_DEVICE_QUALIFIER          6
#define DESC_OTHER_SPEED_CONFIGURATION 7
#define DESC_INTERFACE_POWER           8

#define USB_NO_ERROR                   0x00
#define USB_STALLED                    0x01
#define USB_BUFFER_ERROR               0x02
#define USB_BABBLE                     0x04
#define USB_NAK                        0x08
#define USB_CRC                        0x10
#define USB_TIMEOUT                    0x20
#define USB_BITSTUFF                   0x40
#define USB_TRIVIAL_ERROR              0x80

#define USB_TOD_SETUP                  0x0001
#define USB_TOD_SETUP_DATA_IN          0x0002
#define USB_TOD_SETUP_DATA_OUT         0x0004
#define USB_TOD_SETUP_ACK_IN           0x0008
#define USB_TOD_SETUP_ACK_OUT          0x0010
#define USB_TOD_COMMAND                0x0020
#define USB_TOD_DATA_IN                0x0040
#define USB_TOD_DATA_OUT               0x0080
#define USB_TOD_STATUS                 0x0100

#define USB_TODS_SETUP                 0
#define USB_TODS_SETUP_DATA_IN         1
#define USB_TODS_SETUP_DATA_OUT        2
#define USB_TODS_SETUP_ACK_IN          3
#define USB_TODS_SETUP_ACK_OUT         4
#define USB_TODS_COMMAND               5
#define USB_TODS_DATA_IN               6
#define USB_TODS_DATA_OUT              7
#define USB_TODS_STATUS                8


typedef enum {
    HCI_UHCI,
    HCI_OHCI,
    HCI_EHCI
} hci_type_t;

struct cdi_hci {
    struct cdi_device cdi_device;
    struct hci* hci;
};

struct usb_device;

struct hci {
    struct cdi_pci_device* pcidev;
    hci_type_t type;
    cdi_list_t (* find_devices)(struct hci*);
    void (* activate_device)(struct hci*, struct usb_device*);
    int (* do_packet)(struct hci*, int frame, int type, int device,
        int endpoint, int low_speed, uintptr_t phys_data, int length,
        int datatoggle);
    int (* get_frame)(struct hci*);
};

struct setup_packet {
    uint8_t request_type;
    uint8_t request;
    uint16_t value;
    uint16_t index;
    uint16_t length;
} __attribute__((packed));

struct device_desc {
    uint8_t length;
    uint8_t descriptor_type;
    uint16_t bcdUSB;
    uint8_t class_id;
    uint8_t subclass_id;
    uint8_t protocol_id;
    uint8_t max_packet_size0;
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t num_configurations;
} __attribute__((packed));

struct config_desc {
    uint8_t length;
    uint8_t descriptor_type;
    uint16_t total_length;
    uint8_t num_interfaces;
    uint8_t config_value;
    uint8_t iConfiguration;
    uint8_t attributes;
    uint8_t max_power;
} __attribute__((packed));

struct interface_desc {
    uint8_t length;
    uint8_t descriptor_type;
    uint8_t interface_number;
    uint8_t alternate_setting;
    uint8_t num_endpoints;
    uint8_t interface_class;
    uint8_t interface_subclass;
    uint8_t interface_protocol;
    uint8_t iInterface;
} __attribute__((packed));

struct endpoint_desc {
    uint8_t length;
    uint8_t descriptor_type;
    uint8_t endpoint_address;
    uint8_t attributes;
    uint16_t max_packet_size;
    uint8_t interval;
} __attribute__((packed));

struct hub_desc {
    uint8_t length;
    uint8_t descriptor_type;
    uint8_t nbr_ports;
    uint16_t hub_characteristics;
    uint8_t pwron2pwrgood;
    uint8_t hub_contr_current;
} __attribute__((packed));

typedef enum {
    USBDC_COMPOSITE = 0x00,
    USBDC_AUDIO = 0x01,
    USBDC_COMM_CDC = 0x02,
    USBDC_HID = 0x03,
    USBDC_PID = 0x05,
    USBDC_IMAGE = 0x06,
    USBDC_PRINTER = 0x07,
    USBDC_MSC = 0x08,
    USBDC_HUB = 0x09,
    USBDC_CDC_DATA = 0x0A,
    USBDC_CHIP = 0x0B,
    USBDC_SECURITY = 0x0D,
    USBDC_VIDEO = 0x0E,
    USBDC_HEALTH = 0x0F,
    USBDC_DIAGNOSIS = 0xDC,
    USBDC_WIRELESS = 0xE0,
    USBDC_MISC = 0xEF,
    USBDC_SOFT_SPEC = 0xFE,
    USBDC_VENDOR_SPEC = 0xFF
} usb_class_t;

struct class_data {
    usb_class_t type;
};

struct usb_device {
    struct hci* hci;
    int id;
    int port;
    int low_speed;
    struct device_desc* device;
    struct config_desc* config;
    struct interface_desc* interface;
    struct class_data* classd;
    int stalled;
    int locked;
    int expects;
};

struct msclass_data {
    struct class_data gen_class;
    struct endpoint_desc* bulk_ep_in;
    struct endpoint_desc* bulk_ep_out;
};

/** Beschreibt ein USB-Paket */
struct usb_packet {
    /// Der Typ des Pakets (PACKET_IN, PACKET_OUT, PACKET_SETUP)
    enum usb_packet_type type;
    /// Der gewuenschte Endpoint (0 bis 15)
    int endpoint;
    /// Die physische Adresse des zu verwendenden Datenpuffers
    uintptr_t phys_data;
    /// Die Laenge des Puffers
    int length;
    /// Gibt an, ob DATA0 (0) oder DATA1 (1) verwendet werden soll
    int datatoggle;
    /// Typ der Daten (zur Sicherheit, damit es kein doofes STALL gibt)
    int type_of_data;
};


#include "uhci.h"


#define HCI_STRUCT_SIZE sizeof(struct uhci)


int usb_do_packet(struct usb_device* device, struct usb_packet* packet);
void enumerate_hci(struct hci*);
struct cdi_driver* init_uhcd(void);
void init_msc_driver(void);
void register_msd(struct usb_device* usbdev);
void uhci_init(struct cdi_device* cdi_hci);

#endif
