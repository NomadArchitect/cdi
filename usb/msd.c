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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cdi/misc.h"
#include "cdi/storage.h"

#include "msd.h"
#include "usb.h"

//DEBUG definieren, um einige Debugmeldungen anzuzeigen
#define DEBUG
//FULLDEBUG definieren, um ALLE Debugmeldungen anzuzeigen
#define FULLDEBUG

#if defined FULLDEBUG && !defined DEBUG
#define DEBUG
#endif

#ifdef DEBUG
#define dprintf(fmt, args...) printf("[usb-msc] " fmt, ## args)
#define _dprintf(fmt, args...) printf(fmt, ## args)
#else
static int dprintf(const char* fmt, ...)
{
    return 0;
}

#define _dprintf(fmt, args...) dprintf(fmt, ## args)
#endif

#ifdef FULLDEBUG
#define fdprintf(fmt, args...) printf("[usb-msc] " fmt, ## args)
#else
static int fdprintf(const char* fmt, ...)
{
    return 0;
}

#endif

static uint32_t cbw_tag = 1;

static const char* driver_name = "usb-msc";

static struct cdi_storage_driver cdi_driver;

static void deinit_msd(struct cdi_device* cdi_msd);
static void get_partitions(struct cdi_storage_device* cdistrg);
static void init_msd(struct cdi_device* cdi_msd);
static void kill_driver(struct cdi_driver* cdi_msc_driver);
static int msd_cdi_read(struct cdi_storage_device* strgdev, uint64_t start,
    uint64_t count,
    void* buffer);
static int msd_cdi_write(struct cdi_storage_device* strgdev, uint64_t start,
    uint64_t count,
    void* buffer);
static int msd_get_capacity(struct usb_device* usbdev, uint32_t* block_size,
    uint32_t* block_count);

//TODO: CDI-Funktion wäre hier sicher nützlich...
#define CPU_IS_LITTLE_ENDIAN

#define change_endianess_32(_) ((((_) & \
                                  0xFF000000) >> \
                                 24) | \
                                (((_) & \
                                  0xFF0000) >> \
                                 8) | \
                                (((_) & 0xFF00) << 8) | (((_) & 0xFF) << 24))
#ifdef CPU_IS_LITTLE_ENDIAN
#define CPU2BE(_)               change_endianess_32(_)
#define CPU2LE(_)               (_)
#define BE2CPU(_)               change_endianess_32(_)
#define LE2CPU(_)               (_)
#else
#define CPU2BE(_)               (_)
#define CPU2LE(_)               change_endianess_32(_)
#define BE2CPU(_)               (_)
#define LE2CPU(_)               change_endianess_32(_)
#endif

//Das muss definiert werden, um vor einem Befehl zu warten, bis das MSD fertig
//ist. Theoretisch ist das äußerst sinnvoll, praktisch hängt sich der Treiber
//bisweilen auf - das sollte allerdings mit dem Timeout gegessen sein
/**
  #define WAIT_FOR_MSD_READY
  */

//TODO: Storage bietet eine SCSI-Klasse. Die könnte man durchaus benutzen. xD

void init_msc_driver()
{
    struct cdi_driver* stddrv = (struct cdi_driver*) &cdi_driver;
    stddrv->type = CDI_STORAGE;
    stddrv->name = driver_name;
    stddrv->init_device = &init_msd;
    stddrv->remove_device = &deinit_msd;
    stddrv->destroy = &kill_driver;
    cdi_driver.read_blocks = &msd_cdi_read;
    cdi_driver.write_blocks = &msd_cdi_write;
    cdi_storage_driver_init(&cdi_driver);
    cdi_storage_driver_register(&cdi_driver);
}

void register_msd(struct usb_device* usbdev)
{
    struct cdi_storage_device* strgdev;
    struct cdi_msd* cdimsd;
    struct msclass_data* msc;
    struct endpoint_desc* ep_desc;
    void* address;
    int i;
    uint32_t bs, bc;
    uint64_t size;
    static int msdnum = 0;

    if (usbdev->interface->interface_protocol != 0x50) {
        dprintf("Es werden nur bulk-only-Interfaces unterstützt.\n");
        return;
    }
    cdimsd = malloc(sizeof(struct cdi_msd));
    if (cdimsd == NULL) {
        return;
    }
    cdimsd->usb_device = usbdev;
    cdimsd->offset = 0;
    strgdev = (struct cdi_storage_device*) cdimsd;
    strgdev->dev.type = CDI_STORAGE;
    strgdev->dev.name = malloc(10);
    if (strgdev->dev.name == NULL) {
        free(strgdev);
        return;
    }
    sprintf((char*) strgdev->dev.name, "msd%i", msdnum++);
    msc = malloc(sizeof(struct msclass_data));
    if (msc == NULL) {
        free((void*) strgdev->dev.name);
        free(strgdev);
        return;
    }
    usbdev->classd = (struct class_data*) msc;
    msc->bulk_ep_in = NULL;
    msc->bulk_ep_out = NULL;
    address = (void*) usbdev->interface + sizeof(struct interface_desc);
    for (i = 0; i < usbdev->interface->num_endpoints; i++) {
        ep_desc = address;
        if ((ep_desc->endpoint_address & 0x80) &&
            (ep_desc->attributes == 0x02) && (msc->bulk_ep_in == NULL))
        {
            //BULK-IN
            msc->bulk_ep_in = ep_desc;
        } else if (!(ep_desc->endpoint_address & 0x80) &&
                   (ep_desc->attributes == 0x02) &&
                   (msc->bulk_ep_out == NULL))
        {
            //BULK-OUT
            msc->bulk_ep_out = ep_desc;
        }
        address += sizeof(struct endpoint_desc);
    }
    if ((msc->bulk_ep_in == NULL) || (msc->bulk_ep_out == NULL)) {
        dprintf("Nicht genügend Endpoints gefunden.\n");
        return;
    }
    if (!msd_get_capacity(usbdev, &bs, &bc)) {
        strgdev->block_size = 0;
        strgdev->block_count = 0;
        dprintf("Konnte Größe für %s nicht ermitteln.\n", strgdev->dev.name);
    } else {
        strgdev->block_size = bs;
        strgdev->block_count = bc;
        size = bs;
        size *= bc;
        dprintf("%s: %i * %i B (ca. %lld MB).\n", strgdev->dev.name, bc, bs,
            size >> 20);
        dprintf("EP %i: %i; EP %i: %i\n", msc->bulk_ep_in->endpoint_address,
            msc->bulk_ep_in->max_packet_size,
            msc->bulk_ep_out->endpoint_address,
            msc->bulk_ep_out->max_packet_size);
    }
    cdi_storage_device_init(strgdev);
    cdi_list_push(cdi_driver.drv.devices, strgdev);
    get_partitions(strgdev);
}

static void init_msd(struct cdi_device* cdi_dev)
{
    //Hier wird nichts gemacht, da die Partitionen schon in register_msd()
    //erkannt werden MÜSSEN. Somit läuft die ganze Initialisierung dort ab.
}

static void kill_driver(struct cdi_driver* cdi_msc_driver)
{
    //TODO: Man könnte doch noch irgendwie die Geräte entfernen
}

static void deinit_msd(struct cdi_device* cdi_msd)
{
    //TODO: Und gerade hier...
}

static int write_cmd(struct usb_device* usbdev, void* src)
{
    struct msclass_data* msc = (struct msclass_data*) usbdev->classd;
    struct usb_packet cmd_packet = {
        .type         = PACKET_OUT,
        .endpoint     = msc->bulk_ep_out,
        .data         = src,
        .length       = 0x1F,
        .type_of_data = USB_TOD_COMMAND,
    };

    return usb_do_packet(usbdev, &cmd_packet);
}

/**
  * Liest den Status von einem MSD
  *
  * @param usbdev Das bewusste Gerät
  * @param expected_tag Das erwartete Tag
  *
  * @return Wenn der Status OK ist USB_NO_ERROR, sonst entsprechender Fehler
  */

static int read_status(struct usb_device* usbdev, uint32_t expected_tag)
{
    struct msclass_data* msc = (struct msclass_data*) usbdev->classd;
    struct command_status_wrapper _csw;
    struct command_status_wrapper* csw = &_csw;
    int error;

    struct usb_packet status_packet = {
        .type         = PACKET_IN,
        .endpoint     = msc->bulk_ep_in,
        .data         = csw,
        .length       = 0x0D,
        .type_of_data = USB_TOD_STATUS,
    };

    error = usb_do_packet(usbdev, &status_packet);
    if (error != USB_NO_ERROR) {
        return error;
    }
    if ((csw->csw_signature != CSW_SIGNATURE) ||
        (csw->csw_tag != expected_tag) || csw->csw_status)
    {
        dprintf("0x%08X %i==%i 0x%08X\n", csw->csw_signature, csw->csw_tag,
            expected_tag,
            csw->csw_status);
        return USB_STATUS_ERROR;
    }

    return USB_NO_ERROR;
}

static int msd_get_capacity(struct usb_device* usbdev, uint32_t* block_size,
    uint32_t* block_count)
{
    struct msclass_data* msc;
    struct command_block_wrapper _cbw;
    struct command_block_wrapper* cbw = &_cbw;
    struct msd_capacity _cap;
    struct msd_capacity* cap = &_cap;
    uint32_t expected_tag;

    if ((usbdev == NULL) || (block_size == NULL) || (block_count == NULL)) {
        return 0;
    }

    msc = (struct msclass_data*) usbdev->classd;
    memset(cbw, 0, 0x1F);
    cbw->cbw_signature = CBW_SIGNATURE;
    cbw->cbw_tag = (expected_tag = cbw_tag++);
    cbw->cbw_data_transfer_length = sizeof(struct msd_capacity);
    cbw->cbw_flags = 0x80;      //IN
    cbw->cbw_lun = 0;           //Was weiß ich, wie viele LUNs datt Dingens hat?
    cbw->cbw_cb_length = 12;
    cbw->cbw_cb[0] = MSC_CMD_CAPACITY;
    cbw->cbw_cb[1] = 0;         //LUN: 0

    if (write_cmd(usbdev, cbw) != USB_NO_ERROR) {
        return 0;
    }

    struct usb_packet in_packet = {
        .type         = PACKET_IN,
        .endpoint     = msc->bulk_ep_in,
        .data         = cap,
        .length       = sizeof(*cap),
        .type_of_data = USB_TOD_DATA_IN,
    };

    if (usb_do_packet(usbdev, &in_packet) != USB_NO_ERROR) {
        return 0;
    }

    if (read_status(usbdev, expected_tag) != USB_NO_ERROR) {
        return 0;
    }

    *block_size = BE2CPU(cap->block_length);
    *block_count = BE2CPU(cap->last_lba) + 1;

    return 1;
}

#ifdef WAIT_FOR_MSD_READY
static int msd_ready(struct usb_device* usbdev)
{
    struct msclass_data* msc;
    struct command_block_wrapper _cbw;
    struct command_block_wrapper* cbw = &_cbw;
    uint32_t expected_tag;

    msc = (struct msclass_data*) usbdev->classd;
    memset(cbw, 0, 0x1F);
    cbw->cbw_signature = CBW_SIGNATURE;
    cbw->cbw_tag = (expected_tag = cbw_tag++);
    cbw->cbw_data_transfer_length = 0;
    cbw->cbw_flags = 0x80; //IN
    cbw->cbw_lun = 0; //Was weiß ich, wie viele LUNs datt Dingens hat?
    cbw->cbw_cb_length = 12; //Alles null, also "Test unit ready"

    if (write_cmd(usbdev, cbw) != USB_NO_ERROR) {
        return 0;
    }

    if (read_status(usbdev, expected_tag) != USB_NO_ERROR) {
        return 0;
    }

    return 1;
}

#endif

static inline int tsl(volatile int* variable)
{
    int rval;
    rval = *variable;
    *variable = 1;
    return rval;
}

static uint32_t msd_read(struct usb_device* usbdev, uint32_t lba,
    uint16_t sectors, void* buffer,
    size_t length);

static int msd_cdi_read(struct cdi_storage_device* strgdev, uint64_t start,
    uint64_t count,
    void* buffer)
{
    int bs = strgdev->block_size, error;
#ifdef WAIT_FOR_MSD_READY
    int i;
#endif
    struct usb_device* usbdev = ((struct cdi_msd*) strgdev)->usb_device;

    fdprintf("read(%i, %i)\n", (int) start, (int) count);
    start += ((struct cdi_msd*) strgdev)->offset;

    if (!count) {
        dprintf("Leere Leseanfrage.\n");
        return 0;
    }
    while (tsl(&usbdev->locked)) {
#ifndef CDI_STANDALONE
        __asm__ __volatile__ ("hlt");
#endif
    }
#ifdef WAIT_FOR_MSD_READY
    for (i = 0; !msd_ready(usbdev) && (i < 10); i++) {
        cdi_sleep_ms(20);
    }
#endif
    error = msd_read(usbdev, start, count, buffer, count * bs);
    if (error != USB_NO_ERROR) {
        dprintf("Lesefehler 0x%X bei Block %lld.\n", error, start);
        usbdev->locked = 0;
        return -1;
    }
    usbdev->locked = 0;
    return 0;
}

static uint32_t msd_read(struct usb_device* usbdev, uint32_t lba,
    uint16_t sectors, void* buffer,
    size_t length)
{
    struct msclass_data* msc;
    struct command_block_wrapper _cbw;
    struct command_block_wrapper* cbw = &_cbw;
    uint32_t expected_tag;
    int error;

    if ((buffer == NULL) || !length) {
        return USB_TRIVIAL_ERROR;
    }
    msc = (struct msclass_data*) usbdev->classd;
    memset(cbw, 0, 0x1F);
    cbw->cbw_signature = CBW_SIGNATURE;
    cbw->cbw_tag = (expected_tag = cbw_tag++);
    cbw->cbw_data_transfer_length = length;
    cbw->cbw_flags = 0x80; //IN
    cbw->cbw_lun = 0; //Was weiß ich, wie viele LUNs datt Dingens hat?
    cbw->cbw_cb_length = 12;
    cbw->cbw_cb[0] = MSC_CMD_READ10;
    cbw->cbw_cb[1] = 0; //LUN: 0
    cbw->cbw_cb[2] = (lba & 0xFF000000) >> 24;
    cbw->cbw_cb[3] = (lba & 0x00FF0000) >> 16;
    cbw->cbw_cb[4] = (lba & 0x0000FF00) >> 8;
    cbw->cbw_cb[5] = lba & 0x000000FF;
    cbw->cbw_cb[7] = (sectors & 0x0000FF00) >> 8;
    cbw->cbw_cb[8] = sectors & 0x000000FF;

    error = write_cmd(usbdev, cbw);
    if (error != USB_NO_ERROR) {
        return error;
    }

    struct usb_packet in_packet = {
        .type         = PACKET_IN,
        .endpoint     = msc->bulk_ep_in,
        .data         = buffer,
        .length       = length,
        .type_of_data = USB_TOD_DATA_IN,
    };

    error = usb_do_packet(usbdev, &in_packet);
    if (error != USB_NO_ERROR) {
        return error;
    }

    error = read_status(usbdev, expected_tag);
    if (error != USB_NO_ERROR) {
        return error;
    }

    return USB_NO_ERROR;
}

static uint32_t msd_write(struct usb_device* usbdev, uint32_t lba,
    uint16_t sectors, void* buffer,
    size_t length);

static int msd_cdi_write(struct cdi_storage_device* strgdev, uint64_t start,
    uint64_t count,
    void* buffer)
{
    int bs = strgdev->block_size, error;
#ifdef WAIT_FOR_MSD_READY
    int i;
#endif
    struct usb_device* usbdev = ((struct cdi_msd*) strgdev)->usb_device;

    fdprintf("write(%i, %i)\n", (int) start, (int) count);
    start += ((struct cdi_msd*) strgdev)->offset;

    if (!count) {
        dprintf("Leere Schreibanfrage.\n");
        return 0;
    }
    while (tsl(&usbdev->locked)) {
#ifndef CDI_STANDALONE
        __asm__ __volatile__ ("hlt");
#endif
    }
#ifdef WAIT_FOR_MSD_READY
    for (i = 0; !msd_ready(usbdev) && (i < 10); i++) {
        cdi_sleep_ms(20);
    }
#endif
    error = msd_write(usbdev, start, count, buffer, count * bs);
    if (error != USB_NO_ERROR) {
        dprintf("Schreibfehler 0x%X bei Block %i.\n", error, start);
        usbdev->locked = 0;
        return -1;
    }
    usbdev->locked = 0;
    return 0;
}

static uint32_t msd_write(struct usb_device* usbdev, uint32_t lba,
    uint16_t sectors, void* buffer,
    size_t length)
{
    struct msclass_data* msc;
    struct command_block_wrapper _cbw;
    struct command_block_wrapper* cbw = &_cbw;
    uint32_t expected_tag;
    int error;

    if ((buffer == NULL) || !length) {
        return USB_TRIVIAL_ERROR;
    }
    msc = (struct msclass_data*) usbdev->classd;
    memset(cbw, 0, 0x1F);
    cbw->cbw_signature = CBW_SIGNATURE;
    cbw->cbw_tag = (expected_tag = cbw_tag++);
    cbw->cbw_data_transfer_length = length;
    cbw->cbw_flags = 0x00; //OUT
    cbw->cbw_lun = 0; //Was weiß ich, wie viele LUNs datt Dingens hat?
    cbw->cbw_cb_length = 12;
    cbw->cbw_cb[0] = MSC_CMD_WRITE10;
    cbw->cbw_cb[1] = 0; //LUN: 0
    cbw->cbw_cb[2] = (lba & 0xFF000000) >> 24;
    cbw->cbw_cb[3] = (lba & 0x00FF0000) >> 16;
    cbw->cbw_cb[4] = (lba & 0x0000FF00) >> 8;
    cbw->cbw_cb[5] = lba & 0x000000FF;
    cbw->cbw_cb[7] = (sectors & 0x0000FF00) >> 8;
    cbw->cbw_cb[8] = sectors & 0x000000FF;

    error = write_cmd(usbdev, cbw);
    if (error != USB_NO_ERROR) {
        return error;
    }

    struct usb_packet out_packet = {
        .type         = PACKET_OUT,
        .endpoint     = msc->bulk_ep_out,
        .data         = buffer,
        .length       = length,
        .type_of_data = USB_TOD_DATA_OUT,
    };

    error = usb_do_packet(usbdev, &out_packet);
    if (error != USB_NO_ERROR) {
        return error;
    }

    error = read_status(usbdev, expected_tag);
    if (error != USB_NO_ERROR) {
        return error;
    }

    return USB_NO_ERROR;
}

static void get_partitions(struct cdi_storage_device* cdistrg)
{
    void* mbr;
    struct part_table_entry* partition;
    struct cdi_msd* cdimsd, * base_cdimsd = (struct cdi_msd*) cdistrg;
    int i, j = 0;

    mbr = malloc(512);
    if (msd_cdi_read(cdistrg, 0, 1, mbr) != USB_NO_ERROR) {
        dprintf("MBR konnte nicht eingelesen werden.\n");
        return;
    }
    if (((uint16_t*) mbr)[255] != 0xAA55) {
        return;
    }
    partition = mbr + 0x1BE;
    dprintf("Partitionen auf %s:", cdistrg->dev.name);
    for (i = 0; i < 4; i++) {
        if (partition[i].type && partition[i].size &&
            (partition[i].start + partition[i].size <= cdistrg->block_count))
        {
            cdimsd = malloc(sizeof(struct cdi_msd));
            if (cdimsd == NULL) {
                break;
            }
            cdimsd->offset = partition[i].start;
            cdimsd->usb_device = base_cdimsd->usb_device;
            cdimsd->cdi_device.dev.type = CDI_STORAGE;
            cdimsd->cdi_device.dev.name = malloc(14);
            if (cdimsd->cdi_device.dev.name == NULL) {
                free(cdimsd);
                break;
            }
            sprintf((char*) cdimsd->cdi_device.dev.name, "%sp%i",
                cdistrg->dev.name,
                j++);
            cdimsd->cdi_device.block_size = base_cdimsd->cdi_device.block_size;
            cdimsd->cdi_device.block_count = partition[i].size;
            _dprintf(" %s", cdimsd->cdi_device.dev.name);
            cdi_storage_device_init((struct cdi_storage_device*) cdimsd);
            cdi_list_push(cdi_driver.drv.devices, cdimsd);
        }
    }
    _dprintf("\n");
}
