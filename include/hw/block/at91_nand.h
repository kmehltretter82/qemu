/*
 * Atmel/Microchip AT91 EBI NAND flash interface (CS3 window glue).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_BLOCK_AT91_NAND_H
#define HW_BLOCK_AT91_NAND_H

#include "qom/object.h"

#define TYPE_AT91_NAND "at91-nand"
OBJECT_DECLARE_SIMPLE_TYPE(AT91NandState, AT91_NAND)

#endif /* HW_BLOCK_AT91_NAND_H */
