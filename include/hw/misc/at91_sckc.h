/*
 * Atmel AT91 slow-clock controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_AT91_SCKC_H
#define HW_MISC_AT91_SCKC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_AT91_SCKC "at91-sckc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91SckcState, AT91_SCKC)

#endif /* HW_MISC_AT91_SCKC_H */
