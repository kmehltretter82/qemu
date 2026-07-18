/*
 * Atmel/Microchip AT91 RTC + RTT + GPBR (backup-area timekeeping).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_RTC_AT91_RTC_H
#define HW_RTC_AT91_RTC_H

#include "qom/object.h"

#define TYPE_AT91_RTC  "at91-rtc"    /* RM9200 BCD RTC (0xFFFFFDB0) */
#define TYPE_AT91_RTT  "at91-rtt"    /* Real-Time Timer  (0xFFFFFD20) */
#define TYPE_AT91_GPBR "at91-gpbr"   /* General Purpose Backup Registers */

OBJECT_DECLARE_SIMPLE_TYPE(AT91RtcState, AT91_RTC)
OBJECT_DECLARE_SIMPLE_TYPE(AT91RttState, AT91_RTT)
OBJECT_DECLARE_SIMPLE_TYPE(AT91GpbrState, AT91_GPBR)

#endif /* HW_RTC_AT91_RTC_H */
