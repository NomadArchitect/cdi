/*
 * Copyright (c) 2007 The tyndur Project. All rights reserved.
 *
 * This code is derived from software contributed to the tyndur Project
 * by Matthew Iselin.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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

#include <stdio.h>
#include <string.h>

#include "cdi/net.h"
#include "cdi/pci.h"
#include "cdi/misc.h"

#include "ne2k.h"

#define DRIVER_NAME "ne2k"

static struct cdi_net_driver ne2k_driver;

static int ne2k_driver_init(void)
{
    // Konstruktor der Vaterklasse
    cdi_net_driver_init(&ne2k_driver);

    // Passende PCI-Geraete suchen
    cdi_list_t pci_devices = cdi_list_create();
    cdi_pci_get_all_devices(pci_devices);

    struct cdi_pci_device* dev;
    int i;
    for (i = 0; (dev = cdi_list_get(pci_devices, i)); i++) {
        if ((dev->vendor_id == 0x10ec) && (dev->device_id == 0x8029)) {
            void* phys_device;
            struct ne2k_device* device;

            cdi_alloc_phys_mem(sizeof(*device),
                (void**) &device, &phys_device);
            memset(device, 0, sizeof(*device));

            device->phys = phys_device;
            device->net.dev.bus_data = (struct cdi_bus_data*) dev;
            cdi_list_push(ne2k_driver.drv.devices, device);
        } else {
            cdi_pci_device_destroy(dev);
        }
    }

    cdi_list_destroy(pci_devices);

    return 0;
}

/**
 * Deinitialisiert die Datenstrukturen fuer den ne2k-Treiber
 */
static int ne2k_driver_destroy(void)
{
    cdi_net_driver_destroy(&ne2k_driver);

    // TODO Alle Karten deinitialisieren

    return 0;
}


static struct cdi_net_driver ne2k_driver = {
    .drv = {
        .type           = CDI_NETWORK,
        .name           = DRIVER_NAME,
        .init           = ne2k_driver_init,
        .destroy        = ne2k_driver_destroy,
        .init_device    = ne2k_init_device,
        .remove_device  = ne2k_remove_device,
    },
};

CDI_DRIVER(DRIVER_NAME, ne2k_driver)
