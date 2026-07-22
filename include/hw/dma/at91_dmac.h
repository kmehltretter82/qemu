/*
 * Atmel/Microchip AT91 DMA Controller (DMAC).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_DMA_AT91_DMAC_H
#define HW_DMA_AT91_DMAC_H

#include "qom/object.h"

#define TYPE_AT91_DMAC "at91-dmac"
OBJECT_DECLARE_SIMPLE_TYPE(AT91DmacState, AT91_DMAC)

#define AT91_DMAC_REQUEST_GPIO "peripheral-request"
#define AT91_DMAC_REQUEST_MASK "peripheral-request-mask"
#define AT91_DMAC_MAX_REQUESTS 64

#endif /* HW_DMA_AT91_DMAC_H */
