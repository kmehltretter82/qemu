/*
 * qemu-exact chaos scheduling: make the guest's own races show up.
 *
 * Under MTTCG a vCPU runs long stretches uninterrupted, so a window between two
 * guest instructions that a real machine would sometimes lose to an interrupt,
 * a cache miss or another core's traffic is, here, almost never lost. Kernel
 * races that need one thread to be delayed at exactly the wrong point therefore
 * stay invisible. This stalls vCPUs at seeded random points and delays interrupt
 * delivery, which widens those windows without changing anything architectural:
 * a vCPU that makes no progress for a while, and an interrupt that arrives a few
 * blocks later, are both entirely legal.
 *
 * Deterministic for a given seed and vCPU count, so a failure can be re-run.
 * Pair it with a KCSAN or lockdep kernel: this makes races happen, those report
 * them.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "hw/core/cpu.h"
#include "qemu/log.h"
#include "qemu/notify.h"
#include "system/system.h"
#include "accel/tcg/chaos.h"

bool tcg_chaos_enabled;
uint64_t tcg_chaos_seed = 1;
/*
 * Stalls per million opportunities. The decision is taken between translation
 * blocks, which happens on the order of a million times a second per vCPU, so
 * a per-mille rate would mean thousands of sleeps a second and the guest would
 * crawl instead of being perturbed. 100 per million is roughly one stall every
 * 10ms of guest execution.
 */
unsigned tcg_chaos_rate = 100;
unsigned tcg_chaos_max_us = 200;        /* longest stall */
unsigned tcg_chaos_irq_blocks = 8;      /* longest interrupt delay, in TBs */

/* Per-vCPU state; no locking, each vCPU touches only its own slot. */
typedef struct ChaosCpu {
    uint64_t rng;
    unsigned irq_hold;      /* interrupts still to be held back */
    bool seeded;
} ChaosCpu;

#define CHAOS_MAX_CPUS 64
static ChaosCpu chaos_cpu[CHAOS_MAX_CPUS];

/*
 * How often it actually fired. Without this, "the result did not change" and
 * "chaos never ran" look identical, which is the way to fool yourself.
 */
static uint64_t chaos_stalls, chaos_stall_us, chaos_irq_defers;

static void chaos_report(Notifier *n, void *unused)
{
    if (!tcg_chaos_enabled) {
        return;
    }
    qemu_log_mask(LOG_EXACT, "exact-chaos: %" PRIu64 " stalls totalling %"
                  PRIu64 " us, %" PRIu64 " interrupts held back (rate %u per "
                  "million, seed %" PRIu64 ")\n",
                  chaos_stalls, chaos_stall_us, chaos_irq_defers,
                  tcg_chaos_rate, tcg_chaos_seed);
}

static Notifier chaos_exit_notifier = { .notify = chaos_report };
static bool chaos_notifier_added;

static uint64_t chaos_next(ChaosCpu *c)
{
    /* xorshift64*: tiny, deterministic, good enough to pick moments */
    c->rng ^= c->rng >> 12;
    c->rng ^= c->rng << 25;
    c->rng ^= c->rng >> 27;
    return c->rng * 0x2545f4914f6cdd1dULL;
}

static ChaosCpu *chaos_for(CPUState *cpu)
{
    ChaosCpu *c;

    if (cpu->cpu_index < 0 || cpu->cpu_index >= CHAOS_MAX_CPUS) {
        return NULL;
    }
    c = &chaos_cpu[cpu->cpu_index];
    if (!c->seeded) {
        c->rng = tcg_chaos_seed ^ (0x9e3779b97f4a7c15ULL * (cpu->cpu_index + 1));
        c->rng |= 1;
        c->seeded = true;
    }
    return c;
}

/*
 * Called between translation blocks, with no lock held. Sleeping here is the
 * same thing the host scheduler does to us anyway.
 */
void tcg_chaos_maybe_stall(CPUState *cpu)
{
    ChaosCpu *c = chaos_for(cpu);
    uint64_t r;
    unsigned us;

    if (!c) {
        return;
    }
    if (!chaos_notifier_added) {
        chaos_notifier_added = true;
        qemu_add_exit_notifier(&chaos_exit_notifier);
    }
    r = chaos_next(c);
    if ((r % 1000000) >= tcg_chaos_rate) {
        return;
    }
    us = 1 + ((r >> 16) % tcg_chaos_max_us);
    qatomic_inc(&chaos_stalls);
    qatomic_add(&chaos_stall_us, us);
    g_usleep(us);
}

/*
 * Called when an interrupt could be taken. Returning true holds it back for
 * this block; the request stays pending, so it is delivered a little later.
 */
bool tcg_chaos_defer_irq(CPUState *cpu)
{
    ChaosCpu *c = chaos_for(cpu);

    if (!c) {
        return false;
    }
    if (c->irq_hold) {
        c->irq_hold--;
        qatomic_inc(&chaos_irq_defers);
        return true;
    }
    if ((chaos_next(c) % 1000000) < tcg_chaos_rate * 100) {
        c->irq_hold = 1 + (chaos_next(c) % tcg_chaos_irq_blocks);
        return true;
    }
    return false;
}
