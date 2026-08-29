/*
 * qemu-exact: non-coherent instruction cache.
 *
 * TCG keeps translated code coherent with guest stores: a write to a page
 * holding a translation block throws the block away, so the next execution
 * sees the new instructions. Real cores do nothing of the kind. Data written
 * through the D-side sits in the data cache, the I-side keeps whatever it
 * fetched earlier, and the kernel must clean the data to the point of
 * unification and invalidate the instruction cache before executing it.
 *
 * The model is per physical 64 byte line, keyed by ram_addr so that writing
 * through one alias and executing through another is the same line, and it
 * is byte granular within the line. That matters: the kernel keeps data in
 * code lines (the ftrace call-ops literal sits twelve bytes before the
 * instruction it belongs to) and writes it with no instruction cache
 * maintenance, correctly, because only the D-side ever reads it. A model that
 * marks whole lines reports every one of those.
 *
 * Two masks per line, one bit per byte:
 *
 *   dirty    stored since the last clean to the PoU. An instruction fetch of
 *            such a byte reads stale memory. Cleared by dc cvau/cvac/civac.
 *   changed  stored since the instruction side last fetched it. The I-cache
 *            still holds the old bytes. Cleared by ic ivau, and for the
 *            fetched range by a fetch.
 *
 * A fetch of bytes that are dirty is reported as "not cleaned"; a fetch of
 * bytes that are changed while the line is present in the I-cache is
 * reported as "not invalidated". An ic ivau on a line whose bytes are still
 * dirty achieves nothing (the refill re-reads the same stale data), and the
 * dirty mask correctly survives it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/notify.h"
#include "system/system.h"
#include "system/memory.h"
#include "system/ram_addr.h"
#include "system/ramlist.h"
#include "system/ramblock.h"
#include "qemu/bitmap.h"
#include "system/physmem.h"
#include "cpu.h"
#include "internals.h"
#include "cpregs.h"
#include "exact.h"

bool arm_exact_icache_enabled;
bool arm_exact_icache_full;

/*
 * One bit per line of RAM: written since it was last cleaned to the PoU.
 * This is what catches code that is executed from memory that never held
 * code before, such as a kernel image an EFI stub has just relocated: the
 * per line records below only exist for lines the instruction side has
 * fetched, so a store into fresh memory would otherwise be invisible.
 *
 * In normal mode only stores to pages holding translations take the slow
 * path, so the bitmap covers just those. With x-exact-icache-full=on every
 * page of RAM is protected at machine init and every store is seen, at a
 * cost of a few times the runtime; that is the mode for calibrations of
 * this class.
 */
static unsigned long *ic_dirty;
static uint64_t ic_dirty_lines;

#define IC_LINE 64

typedef struct IcLine {
    uint64_t dirty;             /* bytes not yet cleaned to the PoU */
    uint64_t changed;           /* bytes changed since the I-side fetched them */
    uint64_t writer_pc, writer_lr;
    uint64_t clean_pc, inval_pc;
    uint8_t old[IC_LINE];       /* what the bytes were before modification */
    uint16_t writer_cpu;
    bool present;               /* the instruction side holds this line */
} IcLine;

/*
 * B2.2.5 "Concurrent modification and execution of instructions": for these
 * instructions the architecture guarantees that a PE executes either the old
 * or the new one while a modification is in progress, without any cache
 * maintenance having happened yet. It is what lets ftrace flip NOP and BL
 * under running code, and the kernel relies on it: __aarch64_insn_write()
 * calls __set_fixmap() and raw_spin_unlock_irqrestore() after writing an
 * instruction and before the maintenance, and those functions' own ftrace
 * entries are among the instructions it patches.
 */
static bool cmodx_safe(uint32_t insn)
{
    if ((insn & 0xfc000000) == 0x14000000) {   /* B   */
        return true;
    }
    if ((insn & 0xfc000000) == 0x94000000) {   /* BL  */
        return true;
    }
    if (insn == 0xd503201f) {                  /* NOP */
        return true;
    }
    if ((insn & 0xfffff0ff) == 0xd50330df) {   /* ISB */
        return true;
    }
    switch (insn & 0xffe0001f) {
    case 0xd4200000:                           /* BRK */
    case 0xd4000001:                           /* SVC */
    case 0xd4000002:                           /* HVC */
    case 0xd4000003:                           /* SMC */
        return true;
    }
    return false;
}

/* a ring of recent events, so a report can show the line's whole history */
enum { EV_FETCH, EV_STORE, EV_CLEAN, EV_INVAL, EV_INVAL_ALL };
typedef struct IcEvent {
    uint64_t line;
    uint64_t pc, lr;
    uint64_t mask;
    uint16_t cpu;
    uint8_t kind;
} IcEvent;
#define IC_EVENTS 16384
static IcEvent ic_ev[IC_EVENTS];
static unsigned ic_ev_next;

static QemuMutex ic_lock;
static GHashTable *ic_lines;        /* line number -> IcLine */
static GHashTable *ic_sites;
static uint64_t ic_stat_fetch, ic_stat_store, ic_stat_clean, ic_stat_inval;
static uint64_t ic_stat_reports, ic_stat_maint_miss, ic_stat_data_only;
static uint64_t ic_stat_cmodx, ic_stat_fresh;

static void ic_exit_notify(Notifier *n, void *opaque)
{
    arm_exact_icache_dump();
}

static Notifier ic_exit_notifier = { .notify = ic_exit_notify };

static int ic_ram_block_cb(RAMBlock *rb, void *opaque)
{
    ram_addr_t end = qemu_ram_get_offset(rb) + qemu_ram_get_used_length(rb);

    if (end > ic_dirty_lines * IC_LINE) {
        ic_dirty_lines = DIV_ROUND_UP(end, IC_LINE);
    }
    if (arm_exact_icache_full) {
        /*
         * Mark the whole block clean for the code client, which arms
         * TLB_NOTDIRTY on every existing TLB entry and on every future fill
         * for these pages. Nothing ever marks them dirty again, because the
         * unprotect path only runs for pages that had translations, so all
         * stores keep taking the slow path from here on.
         */
        physical_memory_test_and_clear_dirty(qemu_ram_get_offset(rb),
                                             qemu_ram_get_used_length(rb),
                                             DIRTY_MEMORY_CODE, NULL);
    }
    return 0;
}

static void ic_machine_done(Notifier *n, void *opaque)
{
    qemu_ram_foreach_block(ic_ram_block_cb, NULL);
    ic_dirty = bitmap_new(ic_dirty_lines);
    if (arm_exact_icache_full) {
        qemu_log_mask(LOG_EXACT, "exact-icache: full store tracking, %" PRIu64
                      " lines of RAM watched\n", ic_dirty_lines);
    }
}

static Notifier ic_machine_done_notifier = { .notify = ic_machine_done };

void arm_exact_icache_init(void)
{
    if (ic_lines) {
        return;
    }
    qemu_add_exit_notifier(&ic_exit_notifier);
    qemu_add_machine_init_done_notifier(&ic_machine_done_notifier);
    qemu_mutex_init(&ic_lock);
    ic_lines = g_hash_table_new_full(NULL, NULL, NULL, g_free);
    ic_sites = g_hash_table_new(NULL, NULL);
}

/* caller holds ic_lock */
static IcLine *ic_line(ram_addr_t ra, bool create)
{
    gpointer key = GUINT_TO_POINTER(ra / IC_LINE);
    IcLine *l = g_hash_table_lookup(ic_lines, key);

    if (!l && create) {
        l = g_new0(IcLine, 1);
        g_hash_table_insert(ic_lines, key, l);
    }
    return l;
}

static void ic_event(uint64_t line, int kind, CPUState *cs, uint64_t mask)
{
    CPUARMState *env = cpu_env(cs);
    IcEvent *e = &ic_ev[ic_ev_next++ % IC_EVENTS];

    e->line = line;
    e->kind = kind;
    e->cpu = cs->cpu_index;
    e->pc = env->pc;
    e->lr = env->xregs[30];
    e->mask = mask;
}

static bool ic_site_seen(uint64_t a, uint64_t b)
{
    uint64_t k = (a * 1099511628211ULL) ^ b;
    uintptr_t n = (uintptr_t)g_hash_table_lookup(ic_sites,
                                                 (gpointer)(uintptr_t)k);

    g_hash_table_insert(ic_sites, (gpointer)(uintptr_t)k, (gpointer)(n + 1));
    return n != 0;
}

/* bit mask for [off, off+len) within a line, both already clipped */
static inline uint64_t ic_mask(unsigned off, unsigned len)
{
    uint64_t m = len >= 64 ? ~0ULL : ((1ULL << len) - 1);

    return m << off;
}

/* walk [ram_addr, ram_addr+size) line by line */
#define IC_FOREACH_LINE(ram_addr, size, a, off, len)                        \
    for (ram_addr_t a = (ram_addr) & ~(ram_addr_t)(IC_LINE - 1),            \
         _end = (ram_addr) + (size),                                        \
         off = (ram_addr) - a,                                              \
         len = MIN(_end - (ram_addr), IC_LINE - off);                       \
         a < _end;                                                          \
         a += IC_LINE, off = 0, len = MIN(_end - a, (ram_addr_t)IC_LINE))

static void ic_dump_history(uint64_t line)
{
    unsigned i, n = MIN(ic_ev_next, IC_EVENTS), shown = 0;
    static const char *const names[] = {
        "fetch", "store", "clean", "inval", "inval-all",
    };

    for (i = 0; i < n; i++) {
        IcEvent *e = &ic_ev[(ic_ev_next - n + i) % IC_EVENTS];

        if (e->line != line && e->kind != EV_INVAL_ALL) {
            continue;
        }
        if (shown++ >= 24) {
            qemu_log_mask(LOG_EXACT, "exact-icache:     ...\n");
            break;
        }
        qemu_log_mask(LOG_EXACT,
                      "exact-icache:     %-9s cpu=%u pc=0x%" PRIx64
                      " lr=0x%" PRIx64 " bytes=0x%016" PRIx64 "\n",
                      names[e->kind], e->cpu, e->pc, e->lr, e->mask);
    }
}

/*
 * A translation block is being created from these bytes: the instruction
 * side is fetching them now.
 */
void arm_exact_icache_fetch(CPUState *cs, uint64_t ram_addr, unsigned size)
{
    CPUARMState *env = cpu_env(cs);

    if (!arm_exact_icache_enabled || !ic_lines) {
        return;
    }
    qemu_mutex_lock(&ic_lock);
    IC_FOREACH_LINE(ram_addr, size, a, off, len) {
        IcLine *l = ic_line(a, false);
        uint64_t m = ic_mask(off, len);
        const char *why = NULL;
        uint64_t ln = a / IC_LINE;

        ic_stat_fetch++;
        ic_event(ln, EV_FETCH, cs, m);

        if (!l) {
            l = ic_line(a, true);
            if (ic_dirty && ln < ic_dirty_lines && test_bit(ln, ic_dirty)) {
                /*
                 * Never fetched before, but written since it was last
                 * cleaned: code executed from memory the data side filled
                 * and nobody cleaned to the point of unification.
                 */
                ic_stat_fresh++;
                ic_stat_reports++;
                if (!ic_site_seen(0, (uint64_t)env->pc)) {
                    const uint8_t *cur = qemu_map_ram_ptr(NULL, a);
                    uint32_t w0 = ldl_le_p(cur + (off & ~3u));

                    qemu_log_mask(LOG_EXACT,
                        "exact-icache: VIOLATION executing freshly written "
                        "code that was never cleaned to the point of "
                        "unification\n"
                        "exact-icache:   fetch cpu=%d pc=0x%" PRIx64
                        " of line ram 0x%" PRIx64 " bytes 0x%016" PRIx64
                        ", first instruction 0x%08x\n"
                        "exact-icache:   the line had never been executed "
                        "before, so the writer was not recorded; the last "
                        "store to it was after its last dc cvau\n",
                        cs->cpu_index, (uint64_t)env->pc, (uint64_t)a, m, w0);
                }
            }
            l->present = true;
            continue;
        }

        uint64_t stale = m & (l->dirty | (l->present ? l->changed : 0));
        uint32_t oldw = 0, neww = 0;
        unsigned w, bad = IC_LINE;

        if (stale) {
            const uint8_t *cur = qemu_map_ram_ptr(NULL, a);

            /*
             * Something modified is being executed. Look at each affected
             * instruction word: if both what the I-side may still hold and
             * what memory holds now are CMODX-safe, the architecture says
             * executing either is fine.
             */
            for (w = 0; w < IC_LINE; w += 4) {
                uint64_t wm = 0xfULL << w;
                unsigned b;

                if (!(stale & wm)) {
                    continue;
                }
                oldw = neww = 0;
                for (b = 0; b < 4; b++) {
                    uint8_t o = (stale & (1ULL << (w + b))) ? l->old[w + b]
                                                            : cur[w + b];
                    oldw |= (uint32_t)o << (8 * b);
                    neww |= (uint32_t)cur[w + b] << (8 * b);
                }
                if (!(cmodx_safe(oldw) && cmodx_safe(neww))) {
                    bad = w;
                    break;
                }
            }
            if (bad == IC_LINE) {
                ic_stat_cmodx++;
            } else if (l->dirty & (0xfULL << bad)) {
                why = "not cleaned to the point of unification: the fetch "
                      "reads stale memory (no dc cvau after the store)";
            } else {
                why = "not invalidated: the instruction cache still holds "
                      "the old bytes (no ic ivau after the store)";
            }
        } else if ((l->dirty | l->changed) && !((l->dirty | l->changed) & m)) {
            ic_stat_data_only++;        /* data in a code line, not executed */
        }
        if (why) {
            ic_stat_reports++;
            if (!ic_site_seen(l->writer_lr, (uint64_t)env->pc)) {
                qemu_log_mask(LOG_EXACT,
                    "exact-icache: VIOLATION executing bytes %s\n"
                    "exact-icache:   instruction at line offset %u was "
                    "0x%08x and is now 0x%08x (neither concurrent "
                    "modification safe)\n"
                    "exact-icache:   fetch cpu=%d pc=0x%" PRIx64
                    " of line ram 0x%" PRIx64 " bytes 0x%016" PRIx64 "\n"
                    "exact-icache:   written by cpu=%u pc=0x%" PRIx64
                    " (called from 0x%" PRIx64 "), dirty=0x%016" PRIx64
                    " changed=0x%016" PRIx64 " present=%d\n"
                    "exact-icache:   last clean pc=0x%" PRIx64
                    ", last invalidate pc=0x%" PRIx64 "; history:\n",
                    why, bad, oldw, neww,
                    cs->cpu_index, (uint64_t)env->pc, (uint64_t)a, m,
                    (unsigned)l->writer_cpu, l->writer_pc, l->writer_lr,
                    l->dirty, l->changed, l->present,
                    l->clean_pc, l->inval_pc);
                ic_dump_history(a / IC_LINE);
            }
        }
        l->present = true;
        l->changed &= ~m;           /* the I-side now holds these bytes */
    }
    qemu_mutex_unlock(&ic_lock);
}

/* A store landed on a line. */
void arm_exact_icache_store(CPUState *cs, uint64_t ram_addr, unsigned size)
{
    CPUARMState *env = cpu_env(cs);

    if (!arm_exact_icache_enabled || !ic_lines) {
        return;
    }
    qemu_mutex_lock(&ic_lock);
    IC_FOREACH_LINE(ram_addr, size, a, off, len) {
        IcLine *l = ic_line(a, false);
        uint64_t m = ic_mask(off, len);
        uint64_t ln = a / IC_LINE;

        if (ic_dirty && ln < ic_dirty_lines) {
            set_bit(ln, ic_dirty);      /* dirty in the D-cache until cleaned */
        }
        if (!l) {
            continue;                   /* never fetched: the bit is enough */
        }
        ic_stat_store++;
        ic_event(a / IC_LINE, EV_STORE, cs, m);
        {
            /*
             * This hook runs before the store lands, so the bytes are still
             * the old ones. Keep the value a byte had before its *first*
             * modification: that is what the instruction side may still be
             * holding, however many stores follow.
             */
            const uint8_t *cur = qemu_map_ram_ptr(NULL, a);
            uint64_t fresh = m & ~(l->dirty | l->changed);
            unsigned b;

            for (b = 0; b < IC_LINE; b++) {
                if (fresh & (1ULL << b)) {
                    l->old[b] = cur[b];
                }
            }
        }
        l->dirty |= m;
        l->changed |= m;
        l->writer_pc = env->pc;
        l->writer_lr = env->xregs[30];
        l->writer_cpu = cs->cpu_index;
    }
    qemu_mutex_unlock(&ic_lock);
}

/* cache maintenance, by line or over everything */
void arm_exact_icache_maint(CPUState *cs, uint64_t ram_addr, bool all,
                            bool clean, bool invalidate)
{
    CPUARMState *env = cpu_env(cs);
    GHashTableIter it;
    gpointer k, v;
    IcLine *l;

    if (!arm_exact_icache_enabled || !ic_lines) {
        return;
    }
    qemu_mutex_lock(&ic_lock);
    if (all) {
        if (clean && ic_dirty) {
            bitmap_zero(ic_dirty, ic_dirty_lines);
        }
        g_hash_table_iter_init(&it, ic_lines);
        while (g_hash_table_iter_next(&it, &k, &v)) {
            l = v;
            if (clean) {
                l->dirty = 0;
            }
            if (invalidate) {
                l->present = false;
                l->changed = 0;
            }
        }
        ic_event(0, EV_INVAL_ALL, cs, 0);
        ic_stat_inval += invalidate;
        ic_stat_clean += clean;
        qemu_mutex_unlock(&ic_lock);
        return;
    }
    if (clean && ic_dirty && ram_addr / IC_LINE < ic_dirty_lines) {
        clear_bit(ram_addr / IC_LINE, ic_dirty);
    }
    l = ic_line(ram_addr, false);
    if (!l) {
        ic_stat_maint_miss++;
        qemu_mutex_unlock(&ic_lock);
        return;
    }
    if (clean) {
        ic_event(ram_addr / IC_LINE, EV_CLEAN, cs, l->dirty);
        l->dirty = 0;
        l->clean_pc = env->pc;
        ic_stat_clean++;
    }
    if (invalidate) {
        ic_event(ram_addr / IC_LINE, EV_INVAL, cs, l->changed);
        l->present = false;
        l->changed = 0;
        l->inval_pc = env->pc;
        ic_stat_inval++;
    }
    qemu_mutex_unlock(&ic_lock);
}

void arm_exact_icache_dump(void)
{
    if (!ic_lines) {
        return;
    }
    qemu_log_mask(LOG_EXACT,
                  "exact-icache: %u lines tracked, %" PRIu64 " line fetches, %"
                  PRIu64 " stores to fetched lines, %" PRIu64 " cleans, %"
                  PRIu64 " invalidates, %" PRIu64 " violations, %" PRIu64
                  " fetches of lines whose only changes were data, %" PRIu64
                  " fetches of concurrently modified CMODX-safe instructions, %"
                  PRIu64 " fetches of never-cleaned fresh code, %" PRIu64
                  " maintenance ops on untracked lines\n",
                  g_hash_table_size(ic_lines), ic_stat_fetch, ic_stat_store,
                  ic_stat_clean, ic_stat_inval, ic_stat_reports,
                  ic_stat_data_only, ic_stat_cmodx, ic_stat_fresh,
                  ic_stat_maint_miss);
}
