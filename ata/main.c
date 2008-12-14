/*  
 * Copyright (c) 2007 The tyndur Project. All rights reserved.
 *
 * This code is derived from software contributed to the tyndur Project
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
 *     This product includes software developed by the tyndur Project
 *     and its contributors.
 * 4. Neither the name of the tyndur Project nor the names of its
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
#include <stdlib.h>
#include <stdio.h>

#include "cdi/storage.h"
#include "cdi/pci.h"
#include "cdi/lists.h"

#include "device.h"

struct ata_driver {
    struct cdi_storage_driver storage;
};

static struct ata_driver driver;
static const char* driver_name = "ata";
static cdi_list_t controller_list = NULL;

static void ata_driver_init(void);
static void ata_driver_destroy(struct cdi_driver* driver);

#ifdef CDI_STANDALONE
int main(void)
#else
int init_ata(void)
#endif
{
    cdi_init();
    ata_driver_init();
    cdi_storage_driver_register((struct cdi_storage_driver*) &driver);

#ifdef CDI_STANDALONE
    cdi_run_drivers();
#endif    

    return 0;
}

/**
 * Initialisiert die Datenstrukturen fuer den sis900-Treiber
 */
static void ata_driver_init()
{
    struct ata_controller* controller;

    // Konstruktor der Vaterklasse
    cdi_storage_driver_init((struct cdi_storage_driver*) &driver);
    
    // Namen setzen
    driver.storage.drv.name = driver_name;

    // Funktionspointer initialisieren
    driver.storage.drv.destroy          = ata_driver_destroy;
    driver.storage.drv.init_device      = ata_init_device;
    driver.storage.drv.remove_device    = ata_remove_device;
    driver.storage.read_blocks          = ata_read_blocks;
    driver.storage.write_blocks         = ata_write_blocks;
    
    // Liste mit Controllern initialisieren
    controller_list = cdi_list_create();
    
    
    // Primaeren Controller vorbereiten
    controller = malloc(sizeof(*controller));
    controller->port_cmd_base = ATA_PRIMARY_CMD_BASE;
    controller->port_ctl_base = ATA_PRIMARY_CTL_BASE;
    controller->irq = ATA_PRIMARY_IRQ;
    controller->id = 0;
    controller->driver = (struct cdi_storage_driver*) &driver;
    ata_init_controller(controller);
    cdi_list_push(controller_list, controller);
    
    // Sekundaeren Controller vorbereiten
    controller = malloc(sizeof(*controller));
    controller->port_cmd_base = ATA_SECONDARY_CMD_BASE;
    controller->port_ctl_base = ATA_SECONDARY_CTL_BASE;
    controller->irq = ATA_SECONDARY_IRQ;
    controller->id = 1;
    controller->driver = (struct cdi_storage_driver*) &driver;
    ata_init_controller(controller);
    cdi_list_push(controller_list, controller);
}

/**
 * Deinitialisiert die Datenstrukturen fuer den ata-Treiber
 */
static void ata_driver_destroy(struct cdi_driver* driver)
{
    cdi_storage_driver_destroy((struct cdi_storage_driver*) driver);

    // TODO Alle Karten deinitialisieren
}
