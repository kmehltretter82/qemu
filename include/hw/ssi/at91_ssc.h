/*
 * Atmel AT91 synchronous serial controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_AT91_SSC_H
#define HW_SSI_AT91_SSC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_AT91_SSC "at91-ssc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91SscState, AT91_SSC)

#define AT91_SSC_TX_DMA_REQUEST "tx-dma-request"
#define AT91_SSC_RX_DMA_REQUEST "rx-dma-request"

#endif /* HW_SSI_AT91_SSC_H */
