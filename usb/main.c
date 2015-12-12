/******************************************************************************
 * Copyright (c) 2015 Max Reitz                                               *
 *                                                                            *
 * Permission is hereby granted,  free of charge,  to any person  obtaining a *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction,  including without limitation *
 * the rights to use, copy,  modify, merge, publish,  distribute, sublicense, *
 * and/or  sell copies of the  Software,  and to permit  persons to whom  the *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED,  INCLUDING BUT NOT LIMITED TO THE WARRANTIES  OF MERCHANTABILITY, *
 * FITNESS  FOR A PARTICULAR  PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY,  WHETHER IN AN ACTION OF CONTRACT,  TORT OR OTHERWISE,  ARISING *
 * FROM,  OUT OF  OR IN CONNECTION  WITH  THE SOFTWARE  OR THE  USE OR  OTHER *
 * DEALINGS IN THE SOFTWARE.                                                  *
 ******************************************************************************/

#include <cdi.h>
#include <cdi/usb.h>

#include "usb.h"


#define DRIVER_NAME "usb"


static struct cdi_usb_driver usbd;

static int usbd_init(void)
{
    cdi_driver_init(&usbd.drv);
    return 0;
}

static int usbd_destroy(void)
{
    cdi_driver_destroy(&usbd.drv);
    return 0;
}

static struct cdi_usb_driver usbd = {
    .drv = {
        .name           = DRIVER_NAME,
        .type           = CDI_USB,
        .bus            = CDI_USB_HCD,
        .init           = usbd_init,
        .destroy        = usbd_destroy,
        .init_device    = usb_init_hc,
    },

    .get_endpoint_descriptor    = usb_get_endpoint_descriptor,
    .control_transfer           = usb_control_transfer,
    .bulk_transfer              = usb_bulk_transfer,
};

CDI_DRIVER(DRIVER_NAME, usbd)
