/*
 * Atmel/Microchip AT91 Power Management Controller (PMC).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_AT91_PMC_H
#define HW_MISC_AT91_PMC_H

#include "qom/object.h"

#define TYPE_AT91_PMC "at91-pmc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91PmcState, AT91_PMC)

#endif /* HW_MISC_AT91_PMC_H */
