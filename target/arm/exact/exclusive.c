/*
 * qemu-exact: an adversarial local exclusive monitor.
 *
 * QEMU's LDXR/STXR pair is far kinder than the architecture requires. Its
 * monitor survives exception entry, survives a context switch, and fails only
 * when the value at the reservation address has changed - so an STXR whose
 * loop the guest never expected to spin succeeds first time, every time, and
 * code that cannot cope with repeated failure runs perfectly here and hangs on
 * hardware.
 *
 * The architecture is explicit that the local monitor may transition from
 * Exclusive to Open state for reasons software cannot see: an interrupt, an
 * eviction, a store by another agent anywhere in the same reservation granule,
 * a speculative access. Software must therefore treat any single failed STXR
 * as normal. Failing a store-exclusive that QEMU would have passed is thus
 * always legal, which is what makes this model sound: every report is a guest
 * that relied on a guarantee it never had.
 *
 * What is *not* legal is failing forever - see arm_exact_stxr_fail() and
 * DDI0487 B2.12.5 - which is why the failures are capped per address.
 *
 * Two behaviours, both enabled by x-exact-exclusive:
 *
 *  - Seeded spurious STXR failure at x-exact-exclusive-rate (per million).
 *    Bounded: see arm_exact_stxr_fail() for why an unbounded model is useless.
 *  - The monitor is cleared on exception entry, which real cores do (it is
 *    why AArch64 needs no CLREX on the exception return path). A kernel that
 *    holds a reservation across a taken exception loses it here.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "internals.h"
#include "exact.h"
#ifndef CONFIG_USER_ONLY
#include "qemu/notify.h"
#include "system/system.h"
#endif

bool arm_exact_exclusive_enabled;
unsigned arm_exact_exclusive_rate = 10000;      /* per million STXRs = 1% */
unsigned arm_exact_exclusive_maxrun = 2;
uint64_t arm_exact_exclusive_seed = 1;

typedef struct ExMon {
    uint64_t rng;
    uint64_t last_addr;
    uint32_t run;                               /* consecutive forced fails */
    bool seeded;
} ExMon;

#define EX_MAX_CPUS 64
static ExMon ex_mon[EX_MAX_CPUS];

static uint64_t ex_stat_stxr, ex_stat_forced, ex_stat_capped;
static uint64_t ex_stat_exc_clear, ex_stat_held;
static uint64_t ex_stat_llsc_pairs, ex_stat_llsc_unsafe;

static uint64_t ex_rand(ExMon *m)
{
    uint64_t x = m->rng;

    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    m->rng = x;
    return x * 0x2545f4914f6cdd1dULL;
}

void arm_exact_exclusive_dump(void)
{
    if (!arm_exact_exclusive_enabled) {
        return;
    }
    qemu_log_mask(LOG_EXACT,
                  "exact-exclusive: %" PRIu64 " store-exclusives, %" PRIu64
                  " failed spuriously, %" PRIu64 " passes forced by the"
                  " per-address cap, %" PRIu64 " reservations dropped on"
                  " exception entry (%" PRIu64 " exceptions taken with no"
                  " reservation held)\n",
                  ex_stat_stxr, ex_stat_forced, ex_stat_capped,
                  ex_stat_exc_clear, ex_stat_held);
}

void arm_exact_llsc_dump(void)
{
    if (!ex_stat_llsc_pairs) {
        return;
    }
    qemu_log_mask(LOG_EXACT,
                  "exact-llsc: %" PRIu64 " LDXR/STXR pairs translated, %"
                  PRIu64 " with no forward-progress guarantee\n",
                  ex_stat_llsc_pairs, ex_stat_llsc_unsafe);
}

#ifndef CONFIG_USER_ONLY
static void ex_exit_notify(Notifier *n, void *opaque)
{
    arm_exact_exclusive_dump();
    arm_exact_llsc_dump();
}

static Notifier ex_exit_notifier = { .notify = ex_exit_notify };
#endif

void arm_exact_exclusive_init(void)
{
#ifndef CONFIG_USER_ONLY
    static bool registered;

    if (!registered) {
        registered = true;
        qemu_add_exit_notifier(&ex_exit_notifier);
    }
#endif
}

/*
 * Called from gen_store_exclusive() before the address and value checks, so a
 * forced failure looks to the guest exactly like losing the reservation.
 */
bool arm_exact_stxr_fail(CPUARMState *env, uint64_t addr)
{
    CPUState *cs = env_cpu(env);
    ExMon *m;

    if (cs->cpu_index >= EX_MAX_CPUS) {
        return false;
    }
    m = &ex_mon[cs->cpu_index];
    if (!m->seeded) {
        m->seeded = true;
        m->rng = arm_exact_exclusive_seed + cs->cpu_index * 0x9e3779b97f4a7c15ULL;
        m->rng |= 1;
    }
    ex_stat_stxr++;

    if (ex_rand(m) % 1000000u >= arm_exact_exclusive_rate) {
        m->last_addr = addr;
        m->run = 0;
        return false;
    }

    /*
     * The cap is what keeps the model *legal*, not merely usable. Failing any
     * one STXR is always allowed, but DDI0487 B2.12.5 ("Load-Exclusive and
     * Store-Exclusive instruction usage restrictions") guarantees that a
     * conforming LDXR/STXR loop - no explicit memory effects, no system
     * register writes, no cache or TLB maintenance, no exception-generating
     * instructions, no ISB or indirect branch between the pair - does make
     * forward progress. A CPU that failed every attempt at one address forever
     * would be violating that guarantee, so a guest that livelocked under it
     * would not have a bug worth reporting. Capping the consecutive forced
     * failures per address keeps every report sound.
     *
     * x-exact-exclusive-maxrun=0 removes the cap and with it the guarantee.
     * It is a stress knob for looking at latency under sustained contention
     * (the concern behind Linux commit 03110a5cb216), not a model whose hangs
     * are findings. Anything it turns up has to be re-argued from the loop's
     * own constraints before it counts.
     */
    if (arm_exact_exclusive_maxrun && addr == m->last_addr &&
        m->run >= arm_exact_exclusive_maxrun) {
        ex_stat_capped++;
        m->run = 0;
        m->last_addr = addr;
        return false;
    }

    m->run = (addr == m->last_addr) ? m->run + 1 : 1;
    m->last_addr = addr;
    ex_stat_forced++;
    return true;
}

/*
 * Exception entry. Clearing the monitor is architecturally permitted at any
 * time, and every core we know of does it here; QEMU does not, which is why a
 * reservation held across an interrupt works under emulation only.
 */
void arm_exact_exclusive_exception(CPUARMState *env)
{
    if (!arm_exact_exclusive_enabled) {
        return;
    }
    if (env->exclusive_addr == -1) {
        ex_stat_held++;
        return;
    }
    env->exclusive_addr = -1;
    ex_stat_exc_clear++;
}

/*
 * DDI0487 B2.12.5: an LDXR/STXR loop is guaranteed to make forward progress
 * only if there is nothing between the pair but the arithmetic on the loaded
 * value. Anything below breaks that guarantee, so the loop can spin forever on
 * hardware while running perfectly here - QEMU's store-exclusive never fails
 * unless the value changed, so the loop body is executed once and the question
 * never comes up.
 *
 * This is a property of the *code*, not of any execution, so it is decided at
 * translation time and costs nothing at run time. Classification is by the
 * top-level A64 encoding (DDI0487 C4.1), which is exact: no decode tables and
 * nothing to keep in step with new instructions, because the groups below are
 * architecturally fixed.
 */
const char *arm_exact_llsc_forbidden_a64(uint32_t insn)
{
    /* op0 = insn[28:25] = x1x0: Loads and Stores (PRFM included). */
    if ((insn & 0x0a000000) == 0x08000000) {
        return "a memory access";
    }
    /* MSR (register): system register write. */
    if ((insn & 0xfff00000) == 0xd5100000) {
        return "a system register write";
    }
    /* SYS: cache maintenance, TLB maintenance, address translation. */
    if ((insn & 0xfff80000) == 0xd5080000) {
        return "a cache, TLB or address-translation operation";
    }
    /* ISB (DMB and DSB are permitted between the pair; ISB is not). */
    if ((insn & 0xfffff0ff) == 0xd50330df) {
        return "an ISB";
    }
    /* Exception generation: SVC, HVC, SMC, BRK, HLT. */
    if ((insn & 0xff000000) == 0xd4000000) {
        return "an exception-generating instruction";
    }
    /* Unconditional branch (register): BR, BLR, RET, ERET. */
    if ((insn & 0xfe000000) == 0xd6000000) {
        return "an indirect branch";
    }
    /* BL. */
    if ((insn & 0xfc000000) == 0x94000000) {
        return "a branch with link";
    }
    return NULL;
}

/*
 * One call per LDXR/STXR pair *translated*, not executed, so the cost is a
 * hash lookup once per code site. Counting the safe pairs too is the point:
 * "0 unsafe" only means something next to "N pairs looked at".
 */
void arm_exact_llsc_pair(uint64_t ldex_pc, uint64_t stex_pc, uint64_t bad_pc,
                         const char *what)
{
    static GHashTable *seen;
    gpointer key = (gpointer)(uintptr_t)ldex_pc;

    if (!seen) {
        seen = g_hash_table_new(NULL, NULL);
    }
    if (g_hash_table_contains(seen, key)) {
        return;
    }
    g_hash_table_add(seen, key);
    ex_stat_llsc_pairs++;
    if (!what) {
        return;
    }

    ex_stat_llsc_unsafe++;
    if (stex_pc) {
        qemu_log_mask(LOG_EXACT,
                      "exact-llsc: no forward-progress guarantee: ldxr at 0x%"
                      PRIx64 " and stxr at 0x%" PRIx64 " have %s at 0x%" PRIx64
                      " between them (DDI0487 B2.12.5)\n",
                      ldex_pc, stex_pc, what, bad_pc);
    } else {
        qemu_log_mask(LOG_EXACT,
                      "exact-llsc: no forward-progress guarantee: ldxr at 0x%"
                      PRIx64 " is followed by %s at 0x%" PRIx64
                      " (DDI0487 B2.12.5)\n", ldex_pc, what, bad_pc);
    }
}

/*
 * The A32 half. Deliberately conservative: every encoding below is one whose
 * meaning is unambiguous from these bits alone, and anything doubtful is left
 * out. A check that misses a hazard costs a report; a check that invents one
 * costs the reader's trust in every other report the run produced.
 *
 * Thumb-2 is not covered at all (the caller declines to open a pair in Thumb
 * state): the 16/32-bit instruction boundary is not recoverable from the
 * encoding word the translator hands us here.
 */
const char *arm_exact_llsc_forbidden_a32(uint32_t insn)
{
    /* ISB lives in the unconditional space and has a fixed encoding. */
    if ((insn & 0xfffffff0) == 0xf57ff060) {
        return "an ISB";
    }
    if ((insn & 0x0f000000) == 0x0f000000) {    /* SVC */
        return "an exception-generating instruction";
    }
    /*
     * LDR/STR/LDRB/STRB, and PLD, which is in the same space and is a software
     * prefetch - also forbidden. The exclusion is the media instructions
     * (REV, UBFX, SXTB, ...), which share these bits and differ only by having
     * both bit 25 and bit 4 set; without it every byte-swap between the pair
     * would be reported as a memory access.
     */
    if ((insn & 0x0c000000) == 0x04000000 &&
        (insn & 0x02000010) != 0x02000010) {
        return "a memory access";
    }
    if ((insn & 0x0e000000) == 0x08000000) {    /* LDM/STM */
        return "a memory access";
    }
    if ((insn & 0x0e000000) == 0x0c000000) {    /* LDC/STC, VLDR/VSTR */
        return "a memory access";
    }
    /*
     * Extra load/store: LDRH/STRH/LDRD/STRD/LDRSB/LDRSH. The same bits also
     * cover the multiplies, which are harmless here; bits[6:5] separate them
     * (multiply has 00, every load/store form has something else).
     */
    if ((insn & 0x0e000090) == 0x00000090 && (insn & 0x60) != 0) {
        return "a memory access";
    }
    if ((insn & 0x0f000010) == 0x0e000010) {    /* MCR/MRC, i.e. CP15 */
        return "a coprocessor register transfer";
    }
    if ((insn & 0x0f000000) == 0x0b000000) {    /* BL */
        return "a branch with link";
    }
    if ((insn & 0x0ff000f0) == 0x01200030) {    /* BLX (register) */
        return "a branch with link";
    }
    if ((insn & 0xfe000000) == 0xfa000000) {    /* BLX (immediate) */
        return "a branch with link";
    }
    if ((insn & 0x0ff000f0) == 0x01200010) {    /* BX */
        return "an indirect branch";
    }
    return NULL;
}
