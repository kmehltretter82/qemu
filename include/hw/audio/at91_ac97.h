/*
 * Atmel AT91 AC97 controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_AUDIO_AT91_AC97_H
#define HW_AUDIO_AT91_AC97_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_AT91_AC97 "at91-ac97"
OBJECT_DECLARE_SIMPLE_TYPE(AT91AC97State, AT91_AC97)

#endif /* HW_AUDIO_AT91_AC97_H */
