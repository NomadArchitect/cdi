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
#include <stdlib.h>
#include <stdio.h>

#include "cdi/net.h"
#include "cdi/pci.h"

#include "device.h"

struct sis900_driver {
    struct cdi_net_driver net;
};

static struct sis900_driver sis900_driver;
static const char* driver_name = "sis900";

static void sis900_driver_init(struct sis900_driver* driver);
static void sis900_driver_destroy(struct cdi_driver* driver);

#ifdef CDI_STANDALONE
int main()
#else
int init_sis900
#endif
{
    cdi_init();

    sis900_driver_init(&sis900_driver);
    cdi_driver_register((struct cdi_driver*) &sis900_driver);

#ifdef CDI_STANDALONE
    cdi_run_drivers();
#endif    

    return 0;
}

/**
 * Initialisiert die Datenstrukturen fuer den sis900-Treiber
 */
static void sis900_driver_init(struct sis900_driver* driver)
{
    // Konstruktor der Vaterklasse
    cdi_net_driver_init((struct cdi_net_driver*) driver);
    
    // Namen setzen
    driver->net.drv.name = driver_name;

    // Funktionspointer initialisieren
    driver->net.drv.destroy         = sis900_driver_destroy;
    driver->net.drv.init_device     = sis900_init_device;
    driver->net.drv.remove_device   = sis900_remove_device;

    // Passende PCI-Geraete suchen
    cdi_list_t pci_devices = cdi_list_create();
    cdi_pci_get_all_devices(pci_devices);

    struct cdi_pci_device* dev;
    int i;
    for (i = 0; (dev = cdi_list_get(pci_devices, i)); i++) {
        if ((dev->vendor_id == 0x1039) && (dev->device_id == 0x0900)) {
            struct sis900_device* device = malloc(sizeof(*device));
            device->pci = dev;
            cdi_list_push(driver->net.drv.devices, device);
        } else {
            cdi_pci_device_destroy(dev);
        }
    }

    cdi_list_destroy(pci_devices);
}

/**
 * Deinitialisiert die Datenstrukturen fuer den sis900-Treiber
 */
static void sis900_driver_destroy(struct cdi_driver* driver)
{
    cdi_net_driver_destroy((struct cdi_net_driver*) driver);

    // TODO Alle Karten deinitialisieren
}
