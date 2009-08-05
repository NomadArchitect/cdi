/*
 * Copyright (c) 2008 The tyndur Project. All rights reserved.
 *
 * This code is derived from software contributed to the tyndur Project
 * by Alexander Siol.
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "cdi/fs.h"
#include "cdi/misc.h"

#include "serial_cdi.h"

struct serial_driver {
    struct cdi_fs_driver fs;
};

static struct serial_driver serial_driver;
static const char* driver_name = "serial";

static int serial_driver_init(struct serial_driver* driver);
static void serial_driver_destroy(struct cdi_driver* driver);

#ifdef CDI_STANDALONE
int main(void)
#else
int init_serial(void)
#endif
{
    cdi_init();

    if (serial_driver_init(&serial_driver) != 0) {
        return -1;
    }
    cdi_fs_driver_register((struct cdi_fs_driver*) &serial_driver);

#ifdef CDI_STANDALONE
    cdi_run_drivers();
#endif

    return 0;
}

/**
 * Initialisiert die Datenstrukturen fuer den serial-Treiber
 */
static int serial_driver_init(struct serial_driver* driver)
{
    // Konstruktor der Vaterklasse
    cdi_fs_driver_init((struct cdi_fs_driver*) driver);

    // Namen setzen
    driver->fs.drv.name = driver_name;
    driver->fs.fs_init = serial_fs_init;
    driver->fs.fs_destroy = serial_fs_destroy;

    driver->fs.drv.destroy = serial_driver_destroy;
    return 0;
}

/**
 * Deinitialisiert die Datenstrukturen fuer den serial-Treiber
 */
static void serial_driver_destroy(struct cdi_driver* driver)
{
    cdi_fs_driver_destroy((struct cdi_fs_driver*) driver);
}
