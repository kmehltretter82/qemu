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

/*
 * Video DMA, driven by the guest framebuffer driver and consumed by the
 * VIDC20 model.  The IOMD holds the addresses; VIDC20 holds the timing.
 * Offsets per Linux arch/arm/include/asm/hardware/iomd.h.
 */
#define ACORN_IOMD_VIDCUR       0x1d0
#define ACORN_IOMD_VIDEND       0x1d4
#define ACORN_IOMD_VIDSTART     0x1d8
#define ACORN_IOMD_VIDINIT      0x1dc
#define ACORN_IOMD_VIDCR        0x1e0

#define ACORN_IOMD_VIDCR_ENABLE (1 << 5)    /* Linux DMA_CR_E */

/*
 * Physical address the display should currently be fetching from, or
 * false when video DMA is disabled.  VIDINIT is the live pointer (it is
 * what acornfb rewrites to pan); VIDSTART is the buffer base.
 */
bool acorn_iomd_video_dma(AcornIOMDState *s, hwaddr *base);

#endif
