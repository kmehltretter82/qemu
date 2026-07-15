/*
 * Atmel/Microchip AT91 Two-Wire Interface (TWI / I2C) controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_I2C_AT91_TWI_H
#define HW_I2C_AT91_TWI_H

#include "qom/object.h"

#define TYPE_AT91_TWI "at91-twi"
OBJECT_DECLARE_SIMPLE_TYPE(AT91TwiState, AT91_TWI)

#endif /* HW_I2C_AT91_TWI_H */
