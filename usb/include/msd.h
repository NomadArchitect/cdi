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

#ifndef _CDI__USB__MSD_H
#define _CDI__USB__MSD_H

#include <stdint.h>

#include "cdi/storage.h"

#include "usb.h"


//"USBC"
#define CBW_SIGNATURE 0x43425355
struct command_block_wrapper {
    uint32_t cbw_signature;
    uint32_t cbw_tag;
    uint32_t cbw_data_transfer_length;
    uint8_t cbw_flags;
    uint8_t cbw_lun;
    uint8_t cbw_cb_length;
    uint8_t cbw_cb[16];
} __attribute__((packed));

//"USBS"
#define CSW_SIGNATURE 0x53425355
struct command_status_wrapper {
    uint32_t csw_signature;
    uint32_t csw_tag;
    uint32_t csw_data_residue;
    uint8_t csw_status;
} __attribute__((packed));

struct msc_sense {
    unsigned error : 7;
    unsigned valid : 1;
    uint8_t rsvd0;
    unsigned sense_key : 4;
    unsigned rsvd1 : 4;
    uint32_t information;
    uint8_t additional_length;
    uint32_t rsvd2;
    uint8_t additional_code;
    uint8_t additional_code_qualifier;
    uint32_t rsvd;
} __attribute__((packed));

struct msd_capacity {
    uint32_t last_lba;
    uint32_t block_length;
} __attribute__((packed));

struct cdi_msd {
    struct cdi_storage_device cdi_device;
    struct usb_device* usb_device;
    uint32_t offset;            //Für Partitionen
};

struct part_table_entry {
    uint8_t active;
    uint8_t begin_chs[3];
    uint8_t type;
    uint8_t end_chs[3];
    uint32_t start;
    uint32_t size;
} __attribute__((packed));


#define MSC_CMD_REZERO   0x01
#define MSC_CMD_SENSE    0x03
#define MSC_CMD_CAPACITY 0x25
#define MSC_CMD_READ10   0x28
#define MSC_CMD_WRITE10  0x2A
#define MSC_CMD_SEEK     0x2B
#define MSC_CMD_READ12   0xA8
#define MSC_CMD_WRITE12  0xAA

#endif