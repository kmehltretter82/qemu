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

#endif /* HW_CHAR_AT91_USART_H */
