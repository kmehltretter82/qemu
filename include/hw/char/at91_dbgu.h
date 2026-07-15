/*
 * Atmel/Microchip AT91 Debug Unit (DBGU) - console UART.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_CHAR_AT91_DBGU_H
#define HW_CHAR_AT91_DBGU_H

#include "qom/object.h"

#define TYPE_AT91_DBGU "at91-dbgu"
OBJECT_DECLARE_SIMPLE_TYPE(AT91DbguState, AT91_DBGU)

#endif /* HW_CHAR_AT91_DBGU_H */
