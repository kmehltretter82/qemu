/*
 * Atmel/Microchip AT91 Parallel I/O Controller (PIO / GPIO).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_GPIO_AT91_PIO_H
#define HW_GPIO_AT91_PIO_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_AT91_PIO "at91-pio"
OBJECT_DECLARE_SIMPLE_TYPE(AT91PioState, AT91_PIO)

/* Drive a static input level onto a pin that survives device reset (used by
 * the board to model always-present signals such as SD card-detect). */
void pio_set_reset_input(DeviceState *dev, int pin, bool level);

#endif /* HW_GPIO_AT91_PIO_H */
