/*
 * Atmel AT91 true random number generator.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_AT91_TRNG_H
#define HW_MISC_AT91_TRNG_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_AT91_TRNG "at91-trng"
OBJECT_DECLARE_SIMPLE_TYPE(AT91TrngState, AT91_TRNG)

#endif /* HW_MISC_AT91_TRNG_H */
