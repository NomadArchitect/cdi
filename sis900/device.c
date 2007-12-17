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

#include "cdi.h"

#include "device.h"
#include "io.h"

static void enable_intr(struct sis900_device* device)
{
    reg_outl(device, REG_IER, 1);
}

static void disable_intr(struct sis900_device* device)
{
    reg_outl(device, REG_IER, 0);
}

static void reset_nic(struct sis900_device* device)
{
    // Interrupts deaktivieren
    disable_intr(device);

    // Soft-Reset ausfuehren
    reg_outl(device, REG_COMMAND, CR_RESET | CR_RESET_TX | CR_RESET_RX);
    while (reg_inl(device, REG_COMMAND) & CR_RESET);

    // Wir wollen alles, was an unsere MAC geht und Broadcast
    // TODO Multicast-Hashtabelle setzen
    reg_outl(device, REG_RX_FILT, RXFCR_PHYS | RXFCR_BROADCAST);
    
    // Transmitter und Receiver aktivieren
    reg_outl(device, REG_COMMAND, CR_ENABLE_TX | CR_ENABLE_RX);

    // Interrups wieder aktivieren
    enable_intr(device);
}


void sis900_init_device(struct cdi_driver* driver, struct cdi_device* device)
{
    struct sis900_device* netcard = (struct sis900_device*) device;

    reset_nic(netcard);
}

void sis900_remove_device(struct cdi_driver* driver, struct cdi_device* device)
{
}

void sis900_send_packet(struct cdi_device* device, void* data, size_t size)
{
}
