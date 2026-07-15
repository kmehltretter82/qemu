/*
 * Atmel/Microchip AT91 Periodic Interval Timer (PIT) - system tick.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_TIMER_AT91_PIT_H
#define HW_TIMER_AT91_PIT_H

#include "qom/object.h"

#define TYPE_AT91_PIT "at91-pit"
OBJECT_DECLARE_SIMPLE_TYPE(AT91PitState, AT91_PIT)

#endif /* HW_TIMER_AT91_PIT_H */
