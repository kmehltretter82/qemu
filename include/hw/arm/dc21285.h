/*
 * Intel/DEC 21285 "Footbridge" core logic for StrongARM SA-110.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_DC21285_H
#define HW_ARM_DC21285_H

#include "hw/core/sysbus.h"
#include "hw/core/ptimer.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_DC21285 "dc21285"
OBJECT_DECLARE_SIMPLE_TYPE(DC21285State, DC21285)

#define DC21285_NUM_TIMERS 4
/* Backing store for the CSR block up to the timers (0x000..0x2ff). */
#define DC21285_NUM_REGS   (0x300 >> 2)

typedef struct DC21285Timer {
    struct DC21285State *parent;
    int index;
    ptimer_state *timer;
    uint32_t load;
    uint32_t control;
} DC21285Timer;

struct DC21285State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq fiq;

    uint32_t fclk;

    uint32_t int_raw;
    uint32_t irq_enable;
    uint32_t fiq_enable;

    uint32_t regs[DC21285_NUM_REGS];

    DC21285Timer timer[DC21285_NUM_TIMERS];

    CharFrontend chr;
    uint8_t uart_rx_fifo[16];
    int32_t uart_rx_pos;
    int32_t uart_rx_len;
    uint32_t h_ubrlcr;
    uint32_t m_ubrlcr;
    uint32_t l_ubrlcr;
    uint32_t uartcon;
};

/* Interrupt input line numbers (raw status bit positions). */
#define DC21285_IRQ_IN0     8
#define DC21285_IRQ_IN1     9
#define DC21285_IRQ_IN2     10
#define DC21285_IRQ_IN3     11
#define DC21285_IRQ_PCI     18

#endif
