/*
 * Atmel/Microchip AT91 High Speed MultiMedia Card Interface (HSMCI).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_SD_AT91_HSMCI_H
#define HW_SD_AT91_HSMCI_H

#include "qom/object.h"

#define TYPE_AT91_HSMCI     "at91-hsmci"
#define TYPE_AT91_HSMCI_BUS "at91-hsmci-bus"
OBJECT_DECLARE_SIMPLE_TYPE(AT91HsmciState, AT91_HSMCI)

#define AT91_HSMCI_DMA_REQUEST "dma-request"
#define AT91_HSMCI_SDIO_IRQ    "sdio-irq"

#endif /* HW_SD_AT91_HSMCI_H */
