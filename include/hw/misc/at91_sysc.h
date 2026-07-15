/*
 * Atmel/Microchip AT91 system-controller reset path: Reset Controller (RSTC),
 * Shutdown Controller (SHDWC) and Watchdog Timer (WDT).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_AT91_SYSC_H
#define HW_MISC_AT91_SYSC_H

#include "qom/object.h"

#define TYPE_AT91_RSTC "at91-rstc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91RstcState, AT91_RSTC)

#define TYPE_AT91_SHDWC "at91-shdwc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91ShdwcState, AT91_SHDWC)

#define TYPE_AT91_WDT "at91-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(AT91WdtState, AT91_WDT)

#endif /* HW_MISC_AT91_SYSC_H */
