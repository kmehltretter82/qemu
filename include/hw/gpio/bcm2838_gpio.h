/*
 * Raspberry Pi (BCM2838) GPIO Controller
 * This implementation is based on bcm2835_gpio (hw/gpio/bcm2835_gpio.c)
 *
 * Copyright (c) 2022 Auriga LLC
 *
 * Authors:
 *  Lotosh, Aleksey <aleksey.lotosh@auriga.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef BCM2838_GPIO_H
#define BCM2838_GPIO_H

#include "hw/sd/sd.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_BCM2838_GPIO "bcm2838-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(BCM2838GpioState, BCM2838_GPIO)

#define BCM2838_GPIO_REGS_SIZE 0x1000
#define BCM2838_GPIO_NUM       58
#define GPIO_PUP_PDN_CNTRL_NUM 4
#define BCM2838_GPIO_NUM_IRQS   3

struct BCM2838GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* SDBus selector */
    SDBus sdbus;
    SDBus *sdbus_sdhci;
    SDBus *sdbus_sdhost;

    uint8_t fsel[BCM2838_GPIO_NUM];
    uint32_t lev0, lev1;
    uint8_t sd_fsel;
    qemu_irq out[BCM2838_GPIO_NUM];
    /* Levels last driven on the individual GPIO output lines. */
    uint64_t out_level;
    uint32_t pup_cntrl_reg[GPIO_PUP_PDN_CNTRL_NUM];

    /* Event detection: two banks (0: GPIO 0-31, 1: GPIO 32-57) */
    uint32_t ren[2];   /* rising edge enable  */
    uint32_t fen[2];   /* falling edge enable */
    uint32_t hen[2];   /* high level enable   */
    uint32_t len[2];   /* low level enable    */
    uint32_t aren[2];  /* async rising enable  */
    uint32_t afen[2];  /* async falling enable */
    uint32_t eds[2];   /* event detect status */
    qemu_irq irq[BCM2838_GPIO_NUM_IRQS];
};

#endif
