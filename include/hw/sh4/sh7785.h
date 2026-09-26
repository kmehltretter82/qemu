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

/* After cpu_reset(): apply the mode-pin selected 32-bit boot state. */
void sh7785_reset_32bit_boot(SH7785State *s);

/* Set a catch-all register (e.g. a mode-pin dependent reset value). */
void sh7785_set_reg(SH7785State *s, uint32_t addr, uint32_t val);

/*
 * MMSELR.AREASEL (11.4.1) selects whether areas 2 to 5 are DDR2, PCI or
 * local bus. The board maps its memory accordingly in the hook, which is
 * called at once and on every accepted write. preset sets the value as
 * boot firmware would.
 */
void sh7785_set_mmselr_hook(SH7785State *s,
                            void (*hook)(void *opaque, int areasel),
                            void *opaque);
void sh7785_preset_mmselr(SH7785State *s, int areasel);

/* External interrupt pins IRQ0..7 (level, active while 1). */
qemu_irq sh7785_irq_pin(SH7785State *s, int n);

#endif
