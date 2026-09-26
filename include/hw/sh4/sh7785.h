/*
 * Renesas SH7785 (SH-4A) SoC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SH4_SH7785_H
#define HW_SH4_SH7785_H

#include "hw/core/irq.h"
#include "target/sh4/cpu-qom.h"

typedef struct SH7785State SH7785State;

SH7785State *sh7785_init(SuperHCPU *cpu, MemoryRegion *sysmem,
                         uint32_t pclk_hz);

/* Set a catch-all register (e.g. a mode-pin dependent reset value). */
void sh7785_set_reg(SH7785State *s, uint32_t addr, uint32_t val);

/* External interrupt pins IRQ0..7 (level, active while 1). */
qemu_irq sh7785_irq_pin(SH7785State *s, int n);

#endif
