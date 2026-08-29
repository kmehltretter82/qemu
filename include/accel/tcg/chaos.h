/*
 * qemu-exact chaos scheduling. See accel/tcg/chaos.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef ACCEL_TCG_CHAOS_H
#define ACCEL_TCG_CHAOS_H

extern bool tcg_chaos_enabled;
extern uint64_t tcg_chaos_seed;
extern unsigned tcg_chaos_permille;
extern unsigned tcg_chaos_max_us;
extern unsigned tcg_chaos_irq_blocks;

void tcg_chaos_maybe_stall(CPUState *cpu);
bool tcg_chaos_defer_irq(CPUState *cpu);

#endif
