/*
 * ChipIdea USB device controller test model
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_USB_CI_UDC_H
#define HW_USB_CI_UDC_H

#include "hw/usb/usb.h"
#include "qom/object.h"

#define TYPE_CI_UDC "ci-udc"
OBJECT_DECLARE_SIMPLE_TYPE(CIUdcState, CI_UDC)

#define TYPE_CI_UDC_LINK "ci-udc-link"
OBJECT_DECLARE_SIMPLE_TYPE(CIUdcLinkState, CI_UDC_LINK)

/* create the host-side USB device of @udc on @bus (attached on pull-up) */
void ci_udc_connect_link(CIUdcState *udc, USBBus *bus);

#endif
