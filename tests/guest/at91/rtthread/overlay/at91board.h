/*
 * Board selection for the AT91 RT-Thread test overlay.
 *
 * The same overlay builds for the SAM9M10-G45-EK and, with
 * -DRT_BOARD_SAM9X5, for the SAM9G35-EK: the peripheral IP is the same but
 * the SAM9x5 family lays the system controller out differently and moves
 * everything else into the 0xF0000000/0xF8000000 windows.  Only the
 * constants the overlay reaches for directly live here; anything the BSP
 * already exposes through AT91C_BASE_* comes from the platform header.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef AT91BOARD_H__
#define AT91BOARD_H__

#ifdef RT_BOARD_SAM9X5

#define AT91BOARD_NAME          "sam9g35ek"
#define AT91BOARD_SDRAM_BASE    0x20000000U
#define AT91BOARD_WDT_MR        0xfffffe44U
#define AT91BOARD_AIC_BASE      0xfffff000U
#define AT91BOARD_PMC_PCER      0xfffffc10U
#define AT91BOARD_PIT_BASE      0xfffffe30U
#define AT91BOARD_RTT_BASE      0xfffffe20U
#define AT91BOARD_GPBR_BASE     0xfffffe60U
#define AT91BOARD_RTC_BASE      0xfffffeb0U

#else

#define AT91BOARD_NAME          "sam9m10g45ek"
#define AT91BOARD_SDRAM_BASE    0x70000000U
#define AT91BOARD_WDT_MR        0xfffffd44U
#define AT91BOARD_AIC_BASE      0xfffff000U
#define AT91BOARD_PMC_PCER      0xfffffc10U
#define AT91BOARD_PIT_BASE      0xfffffd30U
#define AT91BOARD_RTT_BASE      0xfffffd20U
#define AT91BOARD_GPBR_BASE     0xfffffd60U
#define AT91BOARD_RTC_BASE      0xfffffdb0U

#endif /* RT_BOARD_SAM9X5 */

#endif /* AT91BOARD_H__ */
