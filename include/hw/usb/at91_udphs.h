/*
 * Atmel/Microchip AT91 USB Device Port High Speed controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_USB_AT91_UDPHS_H
#define HW_USB_AT91_UDPHS_H

#include "hw/core/sysbus.h"
#include "hw/usb/usb.h"
#include "qom/object.h"

#define TYPE_AT91_UDPHS "at91-udphs"
OBJECT_DECLARE_SIMPLE_TYPE(AT91UDPHSState, AT91_UDPHS)

/* Internal cable endpoint used by the SAM9G45 loopback test connection. */
#define TYPE_AT91_UDPHS_USB "at91-udphs-usb"

#endif /* HW_USB_AT91_UDPHS_H */
