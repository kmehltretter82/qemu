/*
 * qemu-exact: non-coherent data cache versus device DMA.
 *
 * The virt machine tells the guest that all DMA is coherent and QEMU makes
 * it so: a device reading RAM sees the CPU's stores, a CPU load sees a
 * device's writes, always. Most real SoCs do neither. The CPU's data cache
 * holds dirty lines the device never sees until they are cleaned to the
 * point of coherency, and keeps stale lines the device has since written
 * until they are invalidated. Linux's DMA API exists to issue exactly those
 * operations, and a driver that gets one wrong, or shares a cache line
 * between CPU data and a device buffer, works on virt and corrupts data on
 * hardware.
 *
 * The model is per line, on pages that have ever been touched by a device
 * or by a clean-to-PoC operation ("DMA pages"), and it never evicts: a dirty
 * line stays dirty until cleaned. States:
 *
 *   DIRTY_DEFAULT  never cleaned since the page was first seen: the CPU may
 *                  have written it at any time before we started watching
 *   CLEAN          cleaned to the PoC, no CPU store since
 *   DIRTY          a cacheable CPU store since the last clean (writer kept)
 *   DMA_WRITTEN    a device wrote it, and the CPU has not invalidated since
 *
 * Reports:
 *   device reads DIRTY or DIRTY_DEFAULT   data the device cannot see
 *   device writes DIRTY                   the CPU's dirty line will later
 *                                         clobber the device's data
 *                                         (cache line sharing)
 *   CPU cacheable load of DMA_WRITTEN     stale read, no invalidate
 *   CPU cacheable store to DMA_WRITTEN    write-allocate pulls stale bytes in
 *   DC IVAC on DIRTY                      the invalidate discards CPU data
 *
 * Only cacheable Normal memory accesses count; the kernel maps coherent
 * allocations Normal-NC on non-coherent platforms and those bypass the cache.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/notify.h"
#include "system/system.h"
#include "system/memory.h"
#include "system/physmem.h"
#include "exec/target_page.h"
#include "cpu.h"
#include "internals.h"
#include "exact.h"
#include "exec/cputlb.h"
#include "hw/core/cpu.h"

bool arm_exact_dcache_enabled;

#define DC_LINE 64
#define DC_LINES_PER_PAGE_MAX 1024      /* 64K pages */

enum { DC_DIRTY_DEFAULT = 0, DC_CLEAN, DC_DIRTY, DC_DMA_WRITTEN };

typedef struct DcLine {
    uint64_t writer_pc, writer_lr;
    uint16_t writer_cpu;
    uint8_t state;
    bool reported;
    bool ever_stored;   /* the CPU has stored into this line at some point */
} DcLine;

typedef struct DcPage {
    DcLine line[DC_LINES_PER_PAGE_MAX];
} DcPage;

static QemuMutex dc_lock;
static GHashTable *dc_pages;        /* page number -> DcPage */
static GHashTable *dc_sites;
static uint64_t dc_stat_dma_rd, dc_stat_dma_wr, dc_stat_cpu_ld, dc_stat_cpu_st;
static uint64_t dc_stat_clean, dc_stat_inval, dc_stat_joins, dc_stat_reports;
static uint64_t dc_stat_nc_skipped;
/* State of the line just before each device access, for diagnosing misses. */
static uint64_t dc_stat_dev_wr_state[4], dc_stat_dev_rd_state[4];
static uint64_t dc_stat_dev_wr_stored;

static bool dc_site_seen(int cls, uint64_t a, uint64_t b)
{
    uint64_t k = (a * 1099511628211ULL) ^ (b * 31) ^ ((uint64_t)cls << 58);
    uintptr_t n = (uintptr_t)g_hash_table_lookup(dc_sites,
                                                 (gpointer)(uintptr_t)k);

    g_hash_table_insert(dc_sites, (gpointer)(uintptr_t)k, (gpointer)(n + 1));
    return n != 0;
}

/* caller holds dc_lock */
static DcPage *dc_page(ram_addr_t ra, bool create)
{
    gpointer key = GUINT_TO_POINTER(ra >> TARGET_PAGE_BITS);
    DcPage *p = g_hash_table_lookup(dc_pages, key);

    if (!p && create) {
        p = g_new0(DcPage, 1);      /* every line starts DIRTY_DEFAULT */
        g_hash_table_insert(dc_pages, key, p);
        dc_stat_joins++;
        /*
         * CPU accesses to this page must now take the slow path. New TLB
         * fills check the page set; entries other vCPUs already hold are
         * dropped, which is rare (once per page, ever) and safe.
         */
        arm_exact_dcache_page_joined(ra & TARGET_PAGE_MASK);
    }
    return p;
}

static inline DcLine *dc_line(DcPage *p, ram_addr_t ra)
{
    return &p->line[(ra & ~TARGET_PAGE_MASK) / DC_LINE];
}

/* Normal memory that the cache can hold: not Device, not Normal-NC */
static bool dc_cacheable(uint8_t attrs)
{
    return (attrs & 0xf0) != 0 && (attrs & 0x0f) != 0x4 &&
           (attrs & 0xf0) != 0x40;
}

bool arm_exact_dcache_track_page(CPUState *cs, CPUTLBEntryFull *full,
                                 uint64_t ram_addr)
{
    if (!arm_exact_dcache_enabled || !dc_pages) {
        return false;
    }
    if (!arm_exact_dcache_page_tracked(ram_addr)) {
        return false;
    }
    if (!dc_cacheable(full->extra.arm.pte_attrs)) {
        dc_stat_nc_skipped++;
        return false;
    }
    return true;
}

void arm_exact_dcache_track_access(CPUState *cs, CPUTLBEntryFull *full,
                                   uint64_t ram_addr, unsigned size,
                                   bool is_store)
{
    arm_exact_dcache_cpu(cs, ram_addr, size, is_store);
}

static void dc_flush_work(CPUState *cs, run_on_cpu_data data)
{
    tlb_flush(cs);
}

/*
 * A page joined the set, so TLB entries other vCPUs already hold for it
 * lack the tracking flag. tlb_flush() is synchronous and must run on the
 * owning vCPU: calling it for another CPU from here races with that CPU's
 * fast path (it reads an entry the memset is turning into -1 and loads
 * through an addend of -1), so queue it as work instead. The window until
 * that runs can only lose accesses, never invent reports.
 */
void arm_exact_dcache_page_joined(uint64_t ram_addr)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        if (qemu_cpu_is_self(cs)) {
            tlb_flush(cs);
        } else {
            async_run_on_cpu(cs, dc_flush_work, RUN_ON_CPU_NULL);
        }
    }
}

bool arm_exact_dcache_page_tracked(uint64_t ram_addr)
{
    bool r;

    if (!dc_pages) {
        return false;
    }
    qemu_mutex_lock(&dc_lock);
    r = g_hash_table_contains(dc_pages,
                              GUINT_TO_POINTER(ram_addr >> TARGET_PAGE_BITS));
    qemu_mutex_unlock(&dc_lock);
    return r;
}

/* --------------------------------------------------------------- device */

static const char *dc_state_name(int s)
{
    switch (s) {
    case DC_CLEAN:        return "clean";
    case DC_DIRTY:        return "dirty (CPU stored, not cleaned)";
    case DC_DMA_WRITTEN:  return "device-written, not invalidated";
    default:              return "never cleaned since first seen";
    }
}

/*
 * A device read or wrote [ram_addr, ram_addr + len). as_name identifies the
 * requester, which for PCI devices is that device's bus master address space.
 */
void arm_exact_dcache_dma(uint64_t ram_addr, uint64_t len, bool is_write,
                          const char *as_name)
{
    ram_addr_t a, end = ram_addr + len;

    if (!arm_exact_dcache_enabled || !dc_pages) {
        return;
    }
    qemu_mutex_lock(&dc_lock);
    for (a = ram_addr & ~(ram_addr_t)(DC_LINE - 1); a < end; a += DC_LINE) {
        DcPage *p = dc_page(a, true);
        DcLine *l = dc_line(p, a);

        if (is_write) {
            dc_stat_dma_wr++;
            dc_stat_dev_wr_state[l->state & 3]++;
            dc_stat_dev_wr_stored += l->ever_stored;
            if (l->state == DC_DIRTY_DEFAULT && !l->reported) {
                /*
                 * The page joined the watched set at this very access, so we
                 * never saw what the CPU did to the line before. Adversarially
                 * the line may still be dirty in the cache; say so, but keep it
                 * distinct from the case where we watched the store happen.
                 */
                l->reported = true;
                dc_stat_reports++;
                if (!dc_site_seen(5, a, 0)) {
                    qemu_log_mask(LOG_EXACT,
                        "exact-dcache: VIOLATION device (%s) writes a line "
                        "that has never been cleaned while we watched it: if "
                        "the CPU holds it dirty, the write-back will overwrite "
                        "what the device wrote\n"
                        "exact-dcache:   line ram 0x%" PRIx64 " (no CPU store "
                        "seen: the page joined the watched set here)\n",
                        as_name ? as_name : "?", (uint64_t)a);
                }
            }
            if (l->state == DC_DIRTY && !l->reported) {
                l->reported = true;
                dc_stat_reports++;
                if (!dc_site_seen(1, l->writer_lr, 0)) {
                    qemu_log_mask(LOG_EXACT,
                        "exact-dcache: VIOLATION device (%s) writes a line "
                        "holding CPU data that was never cleaned: when the "
                        "CPU's dirty line is written back it will overwrite "
                        "what the device wrote (cache line sharing)\n"
                        "exact-dcache:   line ram 0x%" PRIx64 ", CPU store by "
                        "cpu=%u pc=0x%" PRIx64 " (called from 0x%" PRIx64
                        ")\n",
                        as_name ? as_name : "?", (uint64_t)a,
                        (unsigned)l->writer_cpu, l->writer_pc, l->writer_lr);
                }
            }
            l->state = DC_DMA_WRITTEN;
            l->reported = false;
        } else {
            dc_stat_dma_rd++;
            dc_stat_dev_rd_state[l->state & 3]++;
            if ((l->state == DC_DIRTY || l->state == DC_DIRTY_DEFAULT) &&
                !l->reported) {
                l->reported = true;
                dc_stat_reports++;
                if (!dc_site_seen(2, l->writer_lr, l->state)) {
                    qemu_log_mask(LOG_EXACT,
                        "exact-dcache: VIOLATION device (%s) reads a line "
                        "that is %s: the device sees memory, not the CPU's "
                        "cache\n"
                        "exact-dcache:   line ram 0x%" PRIx64 "%s%s cpu=%u "
                        "pc=0x%" PRIx64 " (called from 0x%" PRIx64 ")\n",
                        as_name ? as_name : "?", dc_state_name(l->state),
                        (uint64_t)a,
                        l->state == DC_DIRTY ? ", last CPU store by" : "",
                        l->state == DC_DIRTY ? "" : " (no store seen)",
                        (unsigned)l->writer_cpu, l->writer_pc, l->writer_lr);
                }
            }
        }
    }
    qemu_mutex_unlock(&dc_lock);
}

/* ------------------------------------------------------------------ CPU */

/* a cacheable CPU access on a DMA page, from the slow path */
void arm_exact_dcache_cpu(CPUState *cs, uint64_t ram_addr, unsigned size,
                          bool is_store)
{
    CPUARMState *env = cpu_env(cs);
    ram_addr_t a, end = ram_addr + size;

    if (!arm_exact_dcache_enabled || !dc_pages) {
        return;
    }
    qemu_mutex_lock(&dc_lock);
    for (a = ram_addr & ~(ram_addr_t)(DC_LINE - 1); a < end; a += DC_LINE) {
        DcPage *p = dc_page(a, false);
        DcLine *l;

        if (!p) {
            continue;
        }
        l = dc_line(p, a);
        if (is_store) {
            dc_stat_cpu_st++;
            if (l->state == DC_DMA_WRITTEN && !l->reported) {
                l->reported = true;
                dc_stat_reports++;
                if (!dc_site_seen(3, env->pc, 0)) {
                    qemu_log_mask(LOG_EXACT,
                        "exact-dcache: VIOLATION CPU store into a line the "
                        "device wrote and nobody invalidated: the write "
                        "allocate brings the stale line into the cache, so "
                        "the device's other bytes in it are lost\n"
                        "exact-dcache:   line ram 0x%" PRIx64 " cpu=%d pc=0x%"
                        PRIx64 " (called from 0x%" PRIx64 ")\n",
                        (uint64_t)a, cs->cpu_index, (uint64_t)env->pc,
                        (uint64_t)env->xregs[30]);
                }
            }
            l->state = DC_DIRTY;
            l->ever_stored = true;
            l->writer_pc = env->pc;
            l->writer_lr = env->xregs[30];
            l->writer_cpu = cs->cpu_index;
            l->reported = false;
        } else {
            dc_stat_cpu_ld++;
            if (l->state == DC_DMA_WRITTEN && !l->reported) {
                l->reported = true;
                dc_stat_reports++;
                if (!dc_site_seen(4, env->pc, 0)) {
                    qemu_log_mask(LOG_EXACT,
                        "exact-dcache: VIOLATION CPU load from a line the "
                        "device wrote and nobody invalidated: the cache may "
                        "still hold the old contents\n"
                        "exact-dcache:   line ram 0x%" PRIx64 " cpu=%d pc=0x%"
                        PRIx64 " (called from 0x%" PRIx64 ")\n",
                        (uint64_t)a, cs->cpu_index, (uint64_t)env->pc,
                        (uint64_t)env->xregs[30]);
                }
            }
        }
    }
    qemu_mutex_unlock(&dc_lock);
}

/* ---------------------------------------------------------- maintenance */

/*
 * kind: 'c' clean to PoC (CVAC, CVAP, CVADP, CSW, CISW-as-clean),
 *       'i' invalidate (IVAC), 'b' clean and invalidate (CIVAC, CISW).
 * A clean is what makes a page join the set when no device has touched it
 * yet: the kernel cleans a buffer before the first DMA, and that is the
 * moment we start caring about its lines.
 */
void arm_exact_dcache_maint(CPUState *cs, uint64_t ram_addr, bool all,
                            char kind)
{
    CPUARMState *env = cpu_env(cs);
    GHashTableIter it;
    gpointer k, v;
    DcPage *p;
    DcLine *l;
    unsigned i;

    if (!arm_exact_dcache_enabled || !dc_pages) {
        return;
    }
    qemu_mutex_lock(&dc_lock);
    if (all) {
        g_hash_table_iter_init(&it, dc_pages);
        while (g_hash_table_iter_next(&it, &k, &v)) {
            p = v;
            for (i = 0; i < TARGET_PAGE_SIZE / DC_LINE; i++) {
                if (kind != 'i' || p->line[i].state == DC_DMA_WRITTEN) {
                    p->line[i].state = DC_CLEAN;
                }
            }
        }
        qemu_mutex_unlock(&dc_lock);
        return;
    }
    p = dc_page(ram_addr, kind != 'i');
    if (!p) {
        qemu_mutex_unlock(&dc_lock);
        return;
    }
    l = dc_line(p, ram_addr);
    if (kind == 'i') {
        dc_stat_inval++;
        if (l->state == DC_DIRTY) {
            dc_stat_reports++;
            if (!dc_site_seen(5, env->pc, l->writer_lr)) {
                qemu_log_mask(LOG_EXACT,
                    "exact-dcache: VIOLATION DC IVAC on a line with CPU data "
                    "that was never cleaned: the invalidate discards the "
                    "store\n"
                    "exact-dcache:   line ram 0x%" PRIx64 " invalidated by "
                    "cpu=%d pc=0x%" PRIx64 "; the store was by cpu=%u pc=0x%"
                    PRIx64 " (called from 0x%" PRIx64 ")\n",
                    (uint64_t)ram_addr, cs->cpu_index, (uint64_t)env->pc,
                    (unsigned)l->writer_cpu, l->writer_pc, l->writer_lr);
            }
        }
        l->state = DC_CLEAN;
    } else {
        dc_stat_clean++;
        l->state = DC_CLEAN;
    }
    l->reported = false;
    qemu_mutex_unlock(&dc_lock);
}

/* ------------------------------------------------------------- plumbing */

static void dc_exit_notify(Notifier *n, void *opaque)
{
    arm_exact_dcache_dump();
}

static Notifier dc_exit_notifier = { .notify = dc_exit_notify };

void arm_exact_dcache_init(void)
{
    if (dc_pages) {
        return;
    }
    qemu_mutex_init(&dc_lock);
    dc_pages = g_hash_table_new_full(NULL, NULL, NULL, g_free);
    dc_sites = g_hash_table_new(NULL, NULL);
    qemu_add_exit_notifier(&dc_exit_notifier);
    physmem_dma_observer = arm_exact_dcache_dma;
}

void arm_exact_dcache_dump(void)
{
    if (!dc_pages) {
        return;
    }
    qemu_log_mask(LOG_EXACT,
                  "exact-dcache: %u DMA pages, device reads %" PRIu64
                  " writes %" PRIu64 " lines; CPU loads %" PRIu64 " stores %"
                  PRIu64 " on them; cleans %" PRIu64 " invalidates %" PRIu64
                  "; %" PRIu64 " non-cacheable accesses ignored; %" PRIu64
                  " violations\n",
                  g_hash_table_size(dc_pages), dc_stat_dma_rd, dc_stat_dma_wr,
                  dc_stat_cpu_ld, dc_stat_cpu_st, dc_stat_clean, dc_stat_inval,
                  dc_stat_nc_skipped, dc_stat_reports);
    qemu_log_mask(LOG_EXACT,
                  "exact-dcache: line state at device write: %" PRIu64
                  " never-cleaned, %" PRIu64 " clean, %" PRIu64 " dirty, %"
                  PRIu64 " device-written; at device read: %" PRIu64
                  " never-cleaned, %" PRIu64 " clean, %" PRIu64 " dirty, %"
                  PRIu64 " device-written\n",
                  dc_stat_dev_wr_state[DC_DIRTY_DEFAULT],
                  dc_stat_dev_wr_state[DC_CLEAN],
                  dc_stat_dev_wr_state[DC_DIRTY],
                  dc_stat_dev_wr_state[DC_DMA_WRITTEN],
                  dc_stat_dev_rd_state[DC_DIRTY_DEFAULT],
                  dc_stat_dev_rd_state[DC_CLEAN],
                  dc_stat_dev_rd_state[DC_DIRTY],
                  dc_stat_dev_rd_state[DC_DMA_WRITTEN]);
    qemu_log_mask(LOG_EXACT,
                  "exact-dcache: %" PRIu64 " device writes landed in a line the"
                  " CPU had stored into at some point\n", dc_stat_dev_wr_stored);
}

void arm_exact_dcache_nc_skipped(void)
{
    dc_stat_nc_skipped++;
}
