/*
 * Atmel/Microchip AT91 Pulse Width Modulation (PWM) controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_AT91_PWM_H
#define HW_MISC_AT91_PWM_H

#include "qom/object.h"

#define TYPE_AT91_PWM "at91-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(AT91PwmState, AT91_PWM)

#endif /* HW_MISC_AT91_PWM_H */
