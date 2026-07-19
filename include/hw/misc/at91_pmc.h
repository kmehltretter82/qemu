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

/*
 * Value for the "mckr-reset" property describing the modelled clock tree
 * (CSS=PLLA, PRES=/2, MDIV=/3 -> MCK = 792/2/3 = 132 MHz) in the
 * at91sam9x5_master_layout encoding, which places PRES at bit 4 rather than
 * bit 2.  Boards with a sam9x5-or-later PMC must select this; the device
 * defaults to the rm9200/sam9g45 encoding.
 */
#define AT91_PMC_MCKR_RESET_SAM9X5   0x00000312

#endif /* HW_MISC_AT91_PMC_H */
