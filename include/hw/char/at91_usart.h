/*
 * Atmel/Microchip AT91 USART / Debug Unit (DBGU) UART.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_CHAR_AT91_USART_H
#define HW_CHAR_AT91_USART_H

#include "qom/object.h"

#define TYPE_AT91_USART "at91-usart"
OBJECT_DECLARE_SIMPLE_TYPE(AT91UsartState, AT91_USART)

/*
 * DMAC hardware request lines.  SAM9x5-family boards drive the DBGU and the
 * USARTs from the DMA controllers (at91sam9x5.dtsi gives every port a dmas
 * entry); the SAM9G45 uses the embedded PDC instead and leaves these
 * unconnected.
 */
#define AT91_USART_TX_DMA_REQUEST "tx-dma-request"
#define AT91_USART_RX_DMA_REQUEST "rx-dma-request"

#endif /* HW_CHAR_AT91_USART_H */
