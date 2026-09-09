/*
 * Raspberry Pi (BCM2835) GPIO Controller
 *
 * Copyright (c) 2017 Antfield SAS
 *
 * Authors:
 *  Clement Deschamps <clement.deschamps@antfield.fr>
 *  Luc Michel <luc.michel@antfield.fr>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef BCM2835_GPIO_H
#define BCM2835_GPIO_H

#include "hw/sd/sd.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

struct BCM2835GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* SDBus selector */
    SDBus sdbus;
    SDBus *sdbus_sdhci;
    SDBus *sdbus_sdhost;

    uint8_t fsel[54];
    uint32_t lev0, lev1;
    uint8_t sd_fsel;
    qemu_irq out[54];
    /* Levels last driven on the individual GPIO output lines. */
    uint64_t out_level;

    /* Event detection: bank 0 (GPIO 0-31), bank 1 (GPIO 32-53) */
    uint32_t ren[2], fen[2], hen[2], len[2], aren[2], afen[2], eds[2];
    qemu_irq irq[3];
};

#define TYPE_BCM2835_GPIO "bcm2835_gpio"
OBJECT_DECLARE_SIMPLE_TYPE(BCM2835GpioState, BCM2835_GPIO)

#endif
