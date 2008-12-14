/*  
 * Copyright (c) 2007 The LOST Project. All rights reserved.
 *
 * This code is derived from software contributed to the LOST Project
 * by Antoine Kaufmann.
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

#ifndef _ATA_DEVICE_H_
#define _ATA_DEVICE_H_

#include <stdint.h>

#include "cdi.h"
#include "cdi/storage.h"
#include "cdi/io.h"
#include "cdi/lists.h"

#define ATA_PRIMARY_CMD_BASE    0x1F0
#define ATA_PRIMARY_CTL_BASE    0x3F6
#define ATA_PRIMARY_IRQ         14

#define ATA_SECONDARY_CMD_BASE  0x170
#define ATA_SECONDARY_CTL_BASE  0x376
#define ATA_SECONDARY_IRQ       15

#define ATA_DELAY(c) do { ata_reg_inb(c, REG_ALT_STATUS); \
    ata_reg_inb(c, REG_ALT_STATUS);\
    ata_reg_inb(c, REG_ALT_STATUS); } while (0)

// Hinweis: Worst Case nach ATA-Spec waeren 30 Sekunden
#define ATA_IDENTIFY_TIMEOUT    5000

// So lange wird gewartet, bis ein Geraet bereit ist, wenn ein Befehl
// ausgefuehrt werden soll
#define ATA_READY_TIMEOUT       500

// Timeout beim Warten auf einen IRQ
#define ATA_IRQ_TIMEOUT         500

// Normale Sektorgroesse
#define ATA_SECTOR_SIZE         512

// Diese Register werden ueber die port_cmd_base angesprochen
#define REG_DATA                0x0
#define REG_ERROR               0x1
#define REG_SEC_CNT             0x2
#define REG_LBA_LOW             0x3
#define REG_LBA_MID             0x4
#define REG_LBA_HIG             0x5
#define REG_DEVICE              0x6
#define REG_STATUS              0x7
#define REG_COMMAND             0x7

// Diese Register werden ueber port_ctl_base angesprochen. Das wird den
// Funktionen fuer den Registerzugriff mit den 0x10 mitgeteilt.
#define REG_CONTROL             0x10
#define REG_ALT_STATUS          0x10


// Device Register
#define DEVICE_DEV(x)           (x << 4)

// Status Register
#define STATUS_BSY              (1 << 7)
#define STATUS_DRDY             (1 << 6)
#define STATUS_DF               (1 << 5)
#define STATUS_DRQ              (1 << 3)
#define STATUS_ERR              (1 << 0)
#define STATUS_MASK             (STATUS_BSY | STATUS_DRDY | STATUS_DF | \
                                STATUS_DRQ | STATUS_ERR)

// Befehle
#define COMMAND_IDENTIFY        0xEC

// Control Register
#define CONTROL_HOB             (1 << 7)
#define CONTROL_SRST            (1 << 2)
#define CONTROL_NIEN            (1 << 1)



struct ata_partition {
    struct cdi_storage_device   storage;
    // Muss auf NULL gesetzt werden
    void*                       null;
    struct ata_device*          dev;

    // Startsektor
    uint32_t                    start;
};

struct ata_device {
    struct cdi_storage_device   storage;
    // In der Partitionstruktur ist dieses Feld null, um Partitionen erkennen
    // zu koennen.
    struct ata_controller*      controller;
    
    // Liste mit den Partitionen
    cdi_list_t                  partition_list;

    uint8_t                     id;

    // 1 wenn es sich um ein ATAPI-Gerat handelt, 0 sonst
    uint8_t                     atapi;

    // 1 Wenn das Geraet lba48 unterstuetzt
    uint8_t                     lba48;

    // 1 Wenn das Geraet lba28 unterstuetzt
    uint8_t                     lba28;
    
    // Funktionen fuer den Zugriff auf dieses Geraet
    int (*read_sectors) (struct ata_device* dev, uint64_t start, size_t count,
        void* dest);
    int (*write_sectors) (struct ata_device* dev, uint64_t start,
        size_t count, void* dest);
};

struct ata_controller {
    struct cdi_storage_driver*  driver;

    uint8_t                     id;
    uint16_t                    port_cmd_base;
    uint16_t                    port_ctl_base;
    uint16_t                    irq;
    uint16_t                    irq_cnt;

    // Wird auf 1 gesetzt wenn IRQs benutzt werden sollen, also das NIEN-Bit im
    // Control register nicht aktiviert ist.
    int                         irq_use;
    // HACKKK ;-)
    struct ata_device           irq_dev;
};

struct ata_request {
    struct ata_device* dev;
    
    enum {
        NON_DATA,
        PIO
    } protocol;

    // Flags fuer die Uebertragung
    struct {
        enum {
            READ,
            WRITE
        } direction;
        
        // 1 fuer Polling; 0 fuer Interrupts
        uint8_t poll;
        
        // 1 fuer ATAPI; 0 fuer ata
        uint8_t ata;
        
        // 1 Wenn das LBA-Bit im Geraeteregister
        uint8_t lba;
    } flags;

    union {
        // Registersatz fuer ATA-Operationen
        struct {
            enum {
                IDENTIFY_DEVICE = 0xEC,
                IDENTIFY_PACKET_DEVICE = 0xA1,
                READ_SECTORS = 0x20,
                WRITE_SECTORS = 0x30
            } command;
            uint8_t count;
            uint64_t lba;
        } ata;
        
        // Parameter fuer ATAPI-Operationen
        struct {
        } atapi;
    } registers;

    // Anzahl der Blocks die uebertragen werden sollen
    uint16_t block_count;
    
    // Groesse eines Blocks
    uint16_t block_size;

    // Anzahl der schon verarbeitetn Blocks
    uint16_t blocks_done;

    // Puffer in den die Daten geschrieben werden sollen/aus dem sie gelesen
    // werden sollen.
    void* buffer;
    
    // Moegliche Fehler
    enum {
        NO_ERROR = 0,
        DEVICE_READY_TIMEOUT,
        IRQ_TIMEOUT

    } error;
};


void ata_init_controller(struct ata_controller* controller);
void ata_remove_controller(struct ata_controller* controller);
void ata_init_device(struct cdi_device* device);
void ata_remove_device(struct cdi_device* device);
int ata_read_blocks(struct cdi_storage_device* device, uint64_t block,
    uint64_t count, void* buffer);
int ata_write_blocks(struct cdi_storage_device* device, uint64_t block,
    uint64_t count, void* buffer);


// Einen ATA-Request absenden und ausfuehren
int ata_request(struct ata_request* request);

// ATA-Funktionen
int ata_drv_identify(struct ata_device* dev);
int ata_drv_read_sectors(struct ata_device* dev, uint64_t start, size_t count,
    void* buffer);
int ata_drv_write_sectors(struct ata_device* dev, uint64_t start, size_t count,
    void* buffer);

// ATAPI-Funktionen
int atapi_drv_identify(struct ata_device* dev);

// Auf einen IRQ warten
int ata_wait_irq(struct ata_controller* controller, uint32_t timeout);



/**
 * Basis fuer ein bestimmtes Register ausfindig machen
 */
static inline uint16_t ata_reg_base(struct ata_controller* controller,
    uint8_t reg)
{
    // Fuer alle Register die ueber die ctl_base angesprochen werden muessen
    // setzen wir Bit 4.
    if ((reg & 0x10) == 0) {
        return controller->port_cmd_base;
    } else {
        return controller->port_ctl_base;
    }
}

/**
 * Byte aus Kontrollerregister lesen
 */
static inline uint8_t ata_reg_inb(struct ata_controller* controller,
    uint8_t reg)
{
    uint16_t base = ata_reg_base(controller, reg);
    return cdi_inb(base + (reg & 0xF));
}

/**
 * Byte in Kontrollerregister schreiben
 */
static inline void ata_reg_outb(struct ata_controller* controller, 
    uint8_t reg, uint8_t value)
{
    uint16_t base = ata_reg_base(controller, reg);
    cdi_outb(base + (reg & 0xF), value);
}

/**
 * Word aus Kontrollerregister lesen
 */
static inline uint16_t ata_reg_inw(struct ata_controller* controller,
    uint8_t reg)
{
    uint16_t base = ata_reg_base(controller, reg);
    return cdi_inw(base + (reg & 0xF));
}

/**
 * Byte in Kontrollerregister schreiben
 */
static inline void ata_reg_outw(struct ata_controller* controller,
    uint8_t reg, uint16_t value)
{
    uint16_t base = ata_reg_base(controller, reg);
    cdi_outw(base + (reg & 0xF), value);
}

/**
 * Geraet auwaehlen
 */
static inline void ata_drv_select(struct ata_device* dev)
{
    ata_reg_outb(dev->controller, REG_DEVICE, DEVICE_DEV(dev->id));
    ATA_DELAY(dev->controller);
}

/**
 * Mehrere Words von einem Port einlesen
 *
 * @param port   Portnummer
 * @param buffer Puffer in dem die Words abgelegt werden sollen
 * @param count  Anzahl der Words
 */
static inline void ata_insw(uint16_t port, void* buffer, uint32_t count)
{
    asm volatile("rep insw" : "+D"(buffer), "+c"(count) : "d"(port) : "memory");
}

#endif
