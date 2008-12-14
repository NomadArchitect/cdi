/*  
 * Copyright (c) 2007 The tyndur Project. All rights reserved.
 *
 * This code is derived from software contributed to the tyndur Project
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

#include "cdi/net.h"
#include "cdi/pci.h"
#include "cdi/misc.h"

#include "device.h"


struct module_options {
    uint32_t ip;
};

// FIXME: LOST-spezifisch
uint32_t string_to_ip(char* ip);
static void process_parameter(struct module_options* options, char* param);

struct sis900_driver {
    struct cdi_net_driver net;
};

static struct sis900_driver driver;
static const char* driver_name = "sis900";

static void sis900_driver_init(int argc, char* argv[]);
static void sis900_driver_destroy(struct cdi_driver* driver);

#ifdef CDI_STANDALONE
int main(int argc, char* argv[])
#else
int init_sis900(int argc, char* argv[])
#endif
{
    cdi_init();

    sis900_driver_init(argc, argv);
    cdi_driver_register((struct cdi_driver*) &driver);

#ifdef CDI_STANDALONE
    cdi_run_drivers();
#endif    

    return 0;
}

/**
 * Initialisiert die Datenstrukturen fuer den sis900-Treiber
 */
static void sis900_driver_init(int argc, char* argv[])
{
    struct module_options options = {
        // TODO Auf 0 setzen und am Ende prüfen und ggf. einfach was
        // freies suchen
        .ip = 0x0b01a8c0
    };

    // Debugausgaben auf VT2
    stdout = fopen("vterm:/vterm1/out", "w");

    // Konstruktor der Vaterklasse
    cdi_net_driver_init((struct cdi_net_driver*) &driver);
    
    // Namen setzen
    driver.net.drv.name = driver_name;

    // Funktionspointer initialisieren
    driver.net.drv.destroy         = sis900_driver_destroy;
    driver.net.drv.init_device     = sis900_init_device;
    driver.net.drv.remove_device   = sis900_remove_device;
    
    // Parameter verarbeiten
    int i;
    for (i = 1; i < argc; i++) {
        process_parameter(&options, argv[i]);
    }

    // Passende PCI-Geraete suchen
    cdi_list_t pci_devices = cdi_list_create();
    cdi_pci_get_all_devices(pci_devices);

    struct cdi_pci_device* dev;
    for (i = 0; (dev = cdi_list_get(pci_devices, i)); i++) {
        if ((dev->vendor_id == 0x1039) && (dev->device_id == 0x0900)) {
            void* phys_device;
            struct sis900_device* device;
            
            cdi_alloc_phys_mem(sizeof(*device), 
                (void**) &device, &phys_device);
            memset(device, 0, sizeof(*device));

            device->phys = phys_device;
            device->pci = dev;
            device->net.ip = options.ip;
            cdi_list_push(driver.net.drv.devices, device);
        } else {
            cdi_pci_device_destroy(dev);
        }
    }

    printf("sis900: %d Karten gefunden.\n", 
        cdi_list_size(driver.net.drv.devices));

    cdi_list_destroy(pci_devices);
}

static void process_parameter(struct module_options* options, char* param)
{
    printf("sis900-Parameter: %s\n", param);

    if (strncmp(param, "ip=", 3) == 0) {
        uint32_t ip = string_to_ip(&param[3]);
        printf("IP-Adresse: %08x\n", ip);
        options->ip = ip;
    } else {
        printf("Unbekannter Parameter %s\n", param);
    }
}

/**
 * Deinitialisiert die Datenstrukturen fuer den sis900-Treiber
 */
static void sis900_driver_destroy(struct cdi_driver* driver)
{
    cdi_net_driver_destroy((struct cdi_net_driver*) driver);

    // TODO Alle Karten deinitialisieren
}
