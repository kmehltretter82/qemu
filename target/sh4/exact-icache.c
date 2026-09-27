/*
 * SH-4 instruction cache model for finding guest cache maintenance bugs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The SH-4 instruction cache is not coherent with data stores: code that
 * is written after its line was fetched keeps executing in its old form
 * until software invalidates the line (ICBI on SH-4A, CCR.ICI, or a write
 * to the IC address array, see the SH7750 and SH7785 hardware manuals,
 * "Instruction Cache"). TCG is coherent, so a missing invalidation is
 * invisible. This model remembers which lines the instruction side may
 * hold and reports a line that was written after the fetch and executed
 * again without an invalidation in between.
 *
 * Enabled with -global superh-cpu.x-exact-icache=on, reports with -d exact.
 *
 * Lines are tracked by RAM address, so all virtual aliases of a physical
 * line share one record. That is the conservative choice for invalidation
 * (an invalidation through any alias counts), which can hide a bug where
 * software invalidates the wrong colour, but never invents one. For the
 * same reason an index based invalidation drops every tracked line whose
 * address bits [11:5] match, since bit 12 of the index is a virtual bit.
 *
 * Stores are seen through the code page write protection of the softmmu
 * TLB. The model keeps every RAM page protected (tcg_exact_store_watch),
 * so every store takes the slow path.
 *
 * The data side is not modelled: a store is assumed to reach memory at
 * once, as if the operand cache were write-through.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/notify.h"
#include "cpu.h"
#include "exec/translation-block.h"
#include "exec/tb-flush.h"
#include "accel/tcg/cpu-ops.h"
#include "accel/tcg/cpu-loop.h"
#include "accel/tcg/probe.h"
#include "accel/tcg/cpu-mmu-index.h"
#include "tcg/insn-start-words.h"
#include "system/memory.h"
#include "system/physmem.h"
#include "system/ramblock.h"
#include "system/ramlist.h"
#include "system/runstate.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "hw/core/boards.h"

#define IC_LINE     32
#define IC_INDEXES  128         /* address bits [11:5] */

bool sh4_exact_icache;

typedef struct IcLine {
    uint64_t line;              /* RAM address / IC_LINE */
    uint32_t changed;           /* bytes written since the fetch */
    uint32_t fetch_pc;          /* first fetch since it became present */
    uint32_t writer_pc, writer_pr, writer_addr;
    uint8_t old[IC_LINE];       /* the bytes as fetched */
    bool present;               /* the instruction side may hold it */
} IcLine;

static GHashTable *ic_lines;            /* line -> IcLine */
static GHashTable *ic_present[IC_INDEXES]; /* line -> IcLine, present ones */
static GHashTable *ic_sites;            /* reported (fetch pc, writer pc) */
static bool ic_enabled;                 /* CCR.ICE */

static struct {
    uint64_t fetches, cached_fetches, stores, changes;
    uint64_t icbi, ici, index_inval, assoc_inval, stale, stale_sites;
} ic_stat;

#define IC_MAX_REPORTS 200

static unsigned ic_index(uint64_t line)
{
    return line % IC_INDEXES;
}

static IcLine *ic_line(uint64_t line, bool create)
{
    IcLine *l = g_hash_table_lookup(ic_lines, &line);

    if (!l && create) {
        l = g_new0(IcLine, 1);
        l->line = line;
        g_hash_table_insert(ic_lines, &l->line, l);
    }
    return l;
}

/*
 * The line is gone from the instruction side. QEMU may still hold
 * translations of it, which would run again without another fetch; drop
 * them so that the next execution is seen as a fetch.
 */
static void ic_drop(IcLine *l, CPUState *cs)
{
    tb_page_addr_t start = l->line * IC_LINE;

    l->present = false;
    l->changed = 0;
    g_hash_table_remove(ic_present[ic_index(l->line)], &l->line);
    if (cs) {
        tb_invalidate_phys_range(cs, start, start + IC_LINE - 1);
    }
}

static bool ic_ram_addr(hwaddr phys, ram_addr_t *ra)
{
    MemoryRegion *mr;
    hwaddr xlat, len = 1;

    RCU_READ_LOCK_GUARD();
    mr = address_space_translate(&address_space_memory, phys, &xlat, &len,
                                 false, MEMTXATTRS_UNSPECIFIED);
    if (!memory_region_is_ram(mr)) {
        return false;
    }
    *ra = memory_region_get_ram_addr(mr) + xlat;
    return true;
}

static void ic_report(CPUState *cs, IcLine *l, vaddr pc, ram_addr_t ra,
                      uint32_t mask)
{
    CPUSH4State *env = cpu_env(cs);
    uint64_t key = ((uint64_t)l->writer_pc << 32) | l->fetch_pc;
    unsigned first = ctz32(mask);
    uint8_t *now = qemu_map_ram_ptr(NULL, l->line * IC_LINE);
    uint16_t oldw, neww;

    ic_stat.stale++;
    if (g_hash_table_contains(ic_sites, &key)) {
        return;
    }
    ic_stat.stale_sites++;
    g_hash_table_add(ic_sites, g_memdup2(&key, sizeof(key)));
    if (ic_stat.stale_sites > IC_MAX_REPORTS) {
        return;
    }

    first &= ~1;
    oldw = lduw_le_p(l->old + first);
    neww = lduw_le_p(now + first);
    qemu_log_mask(LOG_EXACT,
                  "exact-icache: stale line ram 0x%" PRIx64
                  " executed at pc 0x%08" VADDR_PRIx " (pr 0x%08x)\n"
                  "  fetched at pc 0x%08x, then written at pc 0x%08x"
                  " (pr 0x%08x) through 0x%08x, not invalidated\n"
                  "  changed bytes 0x%08x, first insn at +%u: old 0x%04x"
                  " new 0x%04x\n",
                  (uint64_t)(l->line * IC_LINE), pc, env->pr,
                  l->fetch_pc, l->writer_pc, l->writer_pr, l->writer_addr,
                  mask, first, oldw, neww);
}

static bool ic_cacheable(vaddr pc)
{
    /* P2 is uncached in both address modes as Linux sets up the PMB */
    return ic_enabled && !(pc >= 0xa0000000 && pc < 0xc0000000);
}

void sh4_exact_icache_fetch(CPUState *cs, vaddr pc, uint64_t ra,
                            unsigned size)
{
    uint64_t end = ra + size;
    bool cached = ic_cacheable(pc);

    if (!sh4_exact_icache) {
        return;
    }
    ic_stat.fetches++;
    ic_stat.cached_fetches += cached;

    for (uint64_t a = ra; a < end; ) {
        uint64_t line = a / IC_LINE;
        uint64_t next = MIN((line + 1) * IC_LINE, end);
        IcLine *l = ic_line(line, cached);

        if (l && l->present && cached) {
            uint32_t lo = a % IC_LINE, n = next - a;
            uint32_t mask = (n == 32 ? ~0u : ((1u << n) - 1)) << lo;

            if (l->changed & mask) {
                ic_report(cs, l, pc + (a - ra), a, l->changed & mask);
                l->changed = 0;
                memcpy(l->old, qemu_map_ram_ptr(NULL, line * IC_LINE),
                       IC_LINE);
            }
        } else if (l && cached && !l->present) {
            l->present = true;
            l->changed = 0;
            l->fetch_pc = pc + (a - ra);
            memcpy(l->old, qemu_map_ram_ptr(NULL, line * IC_LINE), IC_LINE);
            g_hash_table_insert(ic_present[ic_index(line)], &l->line, l);
        }
        a = next;
    }
}

void sh4_exact_store(CPUState *cs, vaddr addr, uint64_t ra, unsigned size,
                     uintptr_t retaddr)
{
    uint64_t end = ra + size;

    if (!sh4_exact_icache) {
        return;
    }
    ic_stat.stores++;

    for (uint64_t a = ra; a < end; ) {
        uint64_t line = a / IC_LINE;
        uint64_t next = MIN((line + 1) * IC_LINE, end);
        IcLine *l = g_hash_table_lookup(ic_present[ic_index(line)], &line);

        if (l) {
            uint32_t lo = a % IC_LINE, n = next - a;
            uint32_t mask = ((1u << n) - 1) << lo;

            if (!l->changed) {
                CPUSH4State *env = cpu_env(cs);
                uint64_t data[INSN_START_WORDS];

                ic_stat.changes++;
                l->writer_pc = env->pc;
                if (retaddr && cpu_unwind_state_data(cs, retaddr, data)) {
                    l->writer_pc = data[0];
                }
                l->writer_pr = env->pr;
                l->writer_addr = addr + (a - ra);
            }
            l->changed |= mask;
        }
        a = next;
    }
}

/* ICBI @Rn: invalidate the line holding the virtual address */
void sh4_exact_icbi(CPUSH4State *env, uint32_t vaddr, uintptr_t retaddr)
{
    CPUState *cs = env_cpu(env);
    void *host;
    IcLine *l;
    uint64_t line;

    if (!sh4_exact_icache) {
        return;
    }
    ic_stat.icbi++;
    host = probe_access(env, vaddr, 1, MMU_DATA_LOAD,
                        cpu_mmu_index(cs, false), retaddr);
    if (!host) {
        return;
    }
    line = qemu_ram_addr_from_host(host);
    if (line == RAM_ADDR_INVALID) {
        return;
    }
    line /= IC_LINE;
    l = g_hash_table_lookup(ic_present[ic_index(line)], &line);
    if (l) {
        ic_drop(l, cs);
    }
}

/* A CCR write from the SoC: ICE and ICI */
void sh4_exact_ccr_write(uint32_t ccr)
{
    if (!sh4_exact_icache) {
        return;
    }
    ic_enabled = ccr & (1 << 8);
    if (ccr & (1 << 11)) {
        ic_stat.ici++;
        for (int i = 0; i < IC_INDEXES; i++) {
            GHashTableIter it;
            gpointer k, v;

            g_hash_table_iter_init(&it, ic_present[i]);
            while (g_hash_table_iter_next(&it, &k, &v)) {
                IcLine *l = v;

                l->present = false;
                l->changed = 0;
                g_hash_table_iter_remove(&it);
            }
        }
        if (current_cpu) {
            queue_tb_flush(current_cpu);
        }
    }
}

/*
 * A write to the IC address array, H'F000 0000 - H'F0FF FFFF. @off is the
 * offset into it. Bit 3 selects an associative write, which compares the
 * tag; otherwise the entry the index and way select is written.
 * Either way only a write with V = 0 invalidates something.
 */
void sh4_exact_ic_array_write(uint32_t off, uint32_t val)
{
    CPUState *cs = current_cpu;

    if (!sh4_exact_icache || (val & 1)) {
        return;
    }
    if (off & 8) {
        ram_addr_t ra;
        IcLine *l;
        uint64_t line;

        ic_stat.assoc_inval++;
        if (!ic_ram_addr((val & 0x1ffffc00) | (off & 0x3e0), &ra)) {
            return;
        }
        line = ra / IC_LINE;
        l = g_hash_table_lookup(ic_present[ic_index(line)], &line);
        if (l) {
            ic_drop(l, cs);
        }
    } else {
        GHashTable *set = ic_present[(off >> 5) % IC_INDEXES];
        GHashTableIter it;
        gpointer k, v;
        GSList *drop = NULL;

        ic_stat.index_inval++;
        g_hash_table_iter_init(&it, set);
        while (g_hash_table_iter_next(&it, &k, &v)) {
            drop = g_slist_prepend(drop, v);
        }
        for (GSList *e = drop; e; e = e->next) {
            ic_drop(e->data, cs);
        }
        g_slist_free(drop);
    }
}

static void ic_exit_notify(Notifier *n, void *data)
{
    qemu_log_mask(LOG_EXACT,
                  "exact-icache: %" PRIu64 " fetches (%" PRIu64 " cached),"
                  " %" PRIu64 " stores watched, %" PRIu64
                  " to fetched lines; invalidations: %" PRIu64 " icbi, %"
                  PRIu64 " ici, %" PRIu64 " index, %" PRIu64
                  " associative; %u lines tracked, %" PRIu64
                  " stale executions at %" PRIu64 " sites\n",
                  ic_stat.fetches, ic_stat.cached_fetches, ic_stat.stores,
                  ic_stat.changes, ic_stat.icbi, ic_stat.ici,
                  ic_stat.index_inval, ic_stat.assoc_inval,
                  g_hash_table_size(ic_lines), ic_stat.stale,
                  ic_stat.stale_sites);
}

static Notifier ic_exit_notifier = { .notify = ic_exit_notify };

static int ic_ram_block_cb(RAMBlock *rb, void *opaque)
{
    /*
     * Clear the code client's dirty bits: every TLB entry for these pages
     * gets TLB_NOTDIRTY, and tlb_unprotect_code() leaves them that way.
     */
    physical_memory_test_and_clear_dirty(qemu_ram_get_offset(rb),
                                         qemu_ram_get_used_length(rb),
                                         DIRTY_MEMORY_CODE, NULL);
    return 0;
}

static void ic_machine_done(Notifier *n, void *opaque)
{
    qemu_ram_foreach_block(ic_ram_block_cb, NULL);
    qemu_log_mask(LOG_EXACT, "exact-icache: on, every store to RAM watched\n");
}

static Notifier ic_machine_done_notifier = { .notify = ic_machine_done };

void sh4_exact_icache_init(void)
{
    if (ic_lines) {
        return;
    }
    sh4_exact_icache = true;
    tcg_exact_store_watch = true;
    ic_lines = g_hash_table_new_full(g_int64_hash, g_int64_equal, NULL,
                                     g_free);
    for (int i = 0; i < IC_INDEXES; i++) {
        ic_present[i] = g_hash_table_new(g_int64_hash, g_int64_equal);
    }
    ic_sites = g_hash_table_new_full(g_int64_hash, g_int64_equal, g_free,
                                     NULL);
    qemu_add_exit_notifier(&ic_exit_notifier);
    qemu_add_machine_init_done_notifier(&ic_machine_done_notifier);
}
