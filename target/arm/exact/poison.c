/*
 * qemu-exact: poison architecturally UNKNOWN reset state.
 *
 * The Linux arm64 boot protocol (Documentation/arch/arm64/booting.rst) only
 * defines x0..x3 for the primary CPU and x0 for PSCI-started secondaries;
 * everything else, including most EL1 system registers, is UNKNOWN at entry.
 * Stock QEMU zeroes all of it, which hides code that relies on zero. Real
 * silicon and the Arm FVP (scramble_unknowns_at_reset) do not.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "cpu.h"
#include "internals.h"

bool arm_exact_poison_regs;
uint64_t arm_exact_poison_seed;

static uint64_t xs64(uint64_t *s)
{
    uint64_t x = *s;

    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *s = x;
    return x;
}

/* SCTLR_EL1 RES1 bits (ARMv8 without extensions): 29, 28, 23, 22, 20, 11 */
#define SCTLR_EL1_RES1 0x30d00800ull

void arm_exact_poison_cpu(ARMCPU *cpu, bool primary)
{
    CPUARMState *env = &cpu->env;
    uint64_t s;
    int i;

    if (!arm_exact_poison_regs || !env->aarch64) {
        return;
    }
    s = arm_exact_poison_seed ^ (0x9e3779b97f4a7c15ull * (CPU(cpu)->cpu_index + 1));
    if (!s) {
        s = 0x2545f4914f6cdd1dull;
    }

    /*
     * Primary: the boot stub loads x0..x4 (x4 is the stub's scratch);
     * PSCI secondary: x0 = context_id was just set by the caller.
     */
    for (i = primary ? 5 : 1; i < 31; i++) {
        env->xregs[i] = xs64(&s);
    }
    env->xregs[31] = xs64(&s) & ~0xfull;
    env->sp_el[0] = xs64(&s) & ~0xfull;
    env->sp_el[1] = xs64(&s) & ~0xfull;
    env->pstate = (env->pstate & ~0xf0000000u) | (xs64(&s) & 0xf0000000u);
    for (i = 0; i < 32; i++) {
        env->vfp.zregs[i].d[0] = xs64(&s);
        env->vfp.zregs[i].d[1] = xs64(&s);
    }
    vfp_set_fpsr(env, xs64(&s) & 0xf80000ffu);

    /* EL1 system registers that are UNKNOWN at reset / at kernel entry */
    env->cp15.ttbr0_el[1] = xs64(&s);
    env->cp15.ttbr1_el[1] = xs64(&s);
    env->cp15.tcr_el[1] = xs64(&s);
    env->cp15.mair_el[1] = xs64(&s);
    env->cp15.vbar_el[1] = xs64(&s) & ~0x7ffull;
    env->cp15.tpidr_el[0] = xs64(&s);
    env->cp15.tpidr_el[1] = xs64(&s);
    env->cp15.tpidrro_el[0] = xs64(&s);
    env->cp15.contextidr_el[1] = xs64(&s);
    env->cp15.esr_el[1] = xs64(&s) & 0xffffffffull;
    env->cp15.far_el[1] = xs64(&s);
    env->elr_el[1] = xs64(&s);
    env->banked_spsr[aarch64_banked_spsr_index(1)] = xs64(&s) & 0xf0000000u;
    env->cp15.cpacr_el1 = xs64(&s) & 0x3ff0000ull;
    /*
     * SCTLR_EL1 is only UNKNOWN when the kernel is entered at EL2 (it then
     * initialises it before dropping to EL1); at EL1 entry M/C must be 0.
     */
    if (arm_current_el(env) == 2) {
        env->cp15.sctlr_el[1] = (xs64(&s) & 0xffffffffull) | SCTLR_EL1_RES1;
    }
}
