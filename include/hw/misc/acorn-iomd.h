/*
 * Acorn IOMD (I/O, Memory and DMA controller) of the RiscPC.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ACORN_IOMD_H
#define HW_MISC_ACORN_IOMD_H

#include "hw/core/sysbus.h"
#include "hw/core/ptimer.h"
#include "qom/object.h"

#define TYPE_ACORN_IOMD "acorn-iomd"
OBJECT_DECLARE_SIMPLE_TYPE(AcornIOMDState, ACORN_IOMD)

#define ACORN_IOMD_NUM_TIMERS 2
/* backing store covers 0x000..0x1ff */
#define ACORN_IOMD_NUM_REGS   (0x200 >> 2)

typedef struct AcornIOMDTimer {
    struct AcornIOMDState *parent;
    int index;
    ptimer_state *timer;
    uint32_t reload;
    uint32_t captured;
} AcornIOMDTimer;

struct AcornIOMDState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq fiq;

    uint32_t irqa_latch;
    uint32_t irqa_mask;
    uint32_t irqb_level;
    uint32_t irqb_mask;
    uint32_t fiq_level;
    uint32_t fiq_mask;
    uint32_t dma_level;
    uint32_t dma_mask;

    uint32_t regs[ACORN_IOMD_NUM_REGS];

    AcornIOMDTimer timer[ACORN_IOMD_NUM_TIMERS];
};

/* GPIO input line numbers */
#define ACORN_IOMD_IRQ_SERIAL   10  /* bank B bit 2: SuperIO 16550 */
#define ACORN_IOMD_IRQ_HARDDISK 9   /* bank B bit 1: onboard IDE */

#endif
