/*
 * qemu-exact: architectural TLB shadow and break-before-make detector.
 *
 * Stock QEMU's softmmu TLB is untagged, so it must be flushed whenever the
 * ASID or VMID changes, and it re-walks the guest page tables far more often
 * than real hardware would. A real ARM64 TLB is tagged by (regime, VMID,
 * ASID) and is only allowed to drop an entry when the guest executes the
 * matching TLBI. The kernel may therefore never rely on an entry disappearing
 * on its own, and any modification of a live translation without the
 * architecturally required invalidation is a bug that only shows up on real
 * silicon.
 *
 * This models that rule as an observer. On every successful page table walk
 * we record the leaf descriptor (its physical address and value) under the
 * architectural key. TLBI operations remove entries with exactly the scope
 * the architecture gives them. If a later walk of the same key finds the
 * descriptor materially changed while our entry was still live, the guest
 * changed a mapping that hardware would still have been caching: that is a
 * missing TLBI, or a break-before-make violation.
 *
 * Changes that hardware is allowed to observe without an invalidation
 * (access flag, dirty state, permissions) are classified separately and not
 * reported.
 *
 * Limitations of this first version, deliberately chosen so that it cannot
 * produce false positives:
 *  - one global table, and every TLBI is treated as broadcast, so a missing
 *    inner-shareable qualifier is not detected;
 *  - TLBI operations that are not decoded drop everything, which can only
 *    lose detections, never invent them;
 *  - only the EL1&0 and EL2&0 stage 1 regimes are tracked.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/notify.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "accel/tcg/system-page-protection.h"
#include "system/ram_addr.h"
#include "exec/target_page.h"
#include "cpu.h"
#include "internals.h"
#include "cpregs.h"
#include "mmuidx-internal.h"
#include "exact.h"

bool arm_exact_tlb_enabled;

#define EX_ASID_GLOBAL 0x10000u
/*
 * Stage 2 entries live in the same tables as stage 1 ones, keyed by a regime
 * number that cannot collide with an exception level. Their "address" is an
 * IPA and they carry no ASID, only a VMID.
 */
#define EX_REGIME_S2   8
#define EX_REGIME_S2_S 9
#define EX_MAX_ENTRIES (1u << 20)
#define EX_MAX_REPORTS 32
#define EX_MAX_CPUS 256

/*
 * Descriptor bits that hardware may act on without an invalidation, so a
 * change to them alone is not a missing-TLBI bug:
 *   10    AF        set by the hardware access flag update
 *   51    DBM       dirty bit modifier
 *   7:6   AP[2:1]   permissions (a permission *relaxation* needs no TLBI,
 *                   and a restriction is caught by its own use, so treat
 *                   permission changes as benign here and report them only
 *                   under the "perm" class)
 *   53,54 PXN/UXN
 *   58:55 software use
 *   62:59 PBHA / ignored
 */
#define EX_BENIGN_MASK ((1ULL << 10) | (1ULL << 51) | (3ULL << 6) |     \
                        (1ULL << 53) | (1ULL << 54) |                   \
                        (0xfULL << 55) | (0xfULL << 59))

typedef struct ExKey {
    uint64_t va;
    uint32_t asid;      /* EX_ASID_GLOBAL for a global (nG == 0) mapping */
    uint16_t vmid;
    uint8_t regime;     /* regime_el() of the stage 1 regime */
    uint8_t space;
    uint8_t level;      /* so a block and a page over the same VA never alias */
} ExKey;


typedef struct ExEntry {
    ExKey key;
    uint64_t desc_pa;
    uint64_t desc_val;
    uint64_t oa;
    uint32_t fill_cpu;
    uint64_t fill_tlbi_seq;     /* value of ex_tlbi_seq when this was filled */
    uint16_t contig_countdown;  /* re-walks left before re-checking the block */
    bool is_table;              /* an intermediate (walk cache) entry */
    uint8_t level;
    uint8_t lg_page_size;
} ExEntry;


typedef struct ExDesc {
    void *host;                 /* host pointer to the eight byte descriptor */
    uint64_t last_val;
    uint64_t broke_seq;         /* ex_tlbi_seq + 1 when cleared, 0 if live */
    uint64_t broke_val;         /* what it held before it was cleared */
    uint64_t broke_pc;
    int broke_cpu;
    ExKey key;                  /* what this descriptor last translated */
    uint64_t desc_pa;           /* where it lives, to tell hierarchies apart */
    bool have_key;
} ExDesc;

typedef struct ExPending {
    ram_addr_t ram_addr;
    unsigned size;
    uint64_t pc;
    bool valid;
} ExPending;

static GHashTable *ex_desc;         /* ram_addr_t -> ExDesc */
static GHashTable *ex_pt_pages;     /* set of page numbers holding descriptors */
static ExPending ex_pending[EX_MAX_CPUS];
static uint64_t ex_stat_pt_pages, ex_stat_desc_stores, ex_stat_bbm;

static QemuMutex ex_lock;
/*
 * One shadow per vCPU, because a TLB is per-PE: an operation without the
 * inner-shareable qualifier only affects the PE that issued it, so a kernel
 * that invalidates locally where it needed a broadcast leaves the other PEs
 * holding a stale entry. That is only visible if we track them separately.
 */
static GHashTable *ex_tab_cpu[EX_MAX_CPUS];     /* ExKey -> ExEntry */
static unsigned ex_ncpus;
static uint64_t ex_stat_fills, ex_stat_hits, ex_stat_benign, ex_stat_violations;
static uint64_t ex_tlbi_seq, ex_stat_removed, ex_stat_contig_checked;
static uint64_t ex_stat_broadcast, ex_stat_local, ex_stat_table_fills;
static bool ex_capped;
static GHashTable *ex_sites;        /* distinct report sites, with counts */

static const char *ex_regime_name(uint8_t r)
{
    switch (r) {
    case EX_REGIME_S2:   return "stage2";
    case EX_REGIME_S2_S: return "stage2-secure";
    case 1:              return "EL1&0";
    case 2:              return "EL2&0";
    case 3:              return "EL3";
    default:             return "?";
    }
}

/*
 * A violation repeats once per descriptor, which for a contiguous block means
 * sixteen times per operation and drowns everything else. Report each distinct
 * site once and count the rest, so a second call site cannot hide behind the
 * volume of the first.
 */
static bool ex_site_seen(int cls, uint64_t a, uint64_t b)
{
    uint64_t key = (a * 31) ^ (b * 131) ^ ((uint64_t)cls << 60);
    gpointer k = (gpointer)(uintptr_t)key;
    uintptr_t n;

    if (!ex_sites) {
        ex_sites = g_hash_table_new(NULL, NULL);
    }
    n = (uintptr_t)g_hash_table_lookup(ex_sites, k);
    g_hash_table_insert(ex_sites, k, (gpointer)(n + 1));
    return n != 0;
}


static guint ex_hash(gconstpointer p)
{
    const ExKey *k = p;
    uint64_t h = k->va ^ ((uint64_t)k->asid << 13) ^
                 ((uint64_t)k->vmid << 29) ^
                 ((uint64_t)k->regime << 45) ^ ((uint64_t)k->space << 50) ^
                 ((uint64_t)k->level << 55);

    h ^= h >> 33;
    h *= 0xff51afd7ed558ccdULL;
    h ^= h >> 33;
    return (guint)h;
}

static gboolean ex_equal(gconstpointer a, gconstpointer b)
{
    const ExKey *x = a, *y = b;

    return x->va == y->va && x->asid == y->asid && x->vmid == y->vmid &&
           x->regime == y->regime && x->space == y->space &&
           x->level == y->level;
}

static void ex_exit_notify(Notifier *n, void *opaque)
{
    arm_exact_tlb_dump();
}

static Notifier ex_exit_notifier = { .notify = ex_exit_notify };

void arm_exact_tlb_init(void)
{
    if (ex_ncpus) {
        return;
    }
    qemu_mutex_init(&ex_lock);
    qemu_add_exit_notifier(&ex_exit_notifier);
    ex_desc = g_hash_table_new_full(NULL, NULL, NULL, g_free);
    ex_pt_pages = g_hash_table_new(NULL, NULL);
    ex_ncpus = 1;               /* tables are created on first use */
}

/* caller holds ex_lock */
static GHashTable *ex_tab_of(unsigned cpu)
{
    if (cpu >= EX_MAX_CPUS) {
        return NULL;
    }
    if (!ex_tab_cpu[cpu]) {
        ex_tab_cpu[cpu] = g_hash_table_new_full(ex_hash, ex_equal,
                                                g_free, g_free);
        if (cpu + 1 > ex_ncpus) {
            ex_ncpus = cpu + 1;
        }
    }
    return ex_tab_cpu[cpu];
}

/* ---------------------------------------------------------------- lookup */

static uint32_t ex_asid_of(CPUARMState *env, ARMMMUIdx mmu_idx, uint64_t desc)
{
    uint64_t tcr, ttbr;
    unsigned bits;

    if (regime_is_stage2(mmu_idx)) {
        return EX_ASID_GLOBAL;      /* stage 2 has no ASID, only a VMID */
    }
    if (!(desc & (1ULL << 11))) {
        return EX_ASID_GLOBAL;          /* nG == 0 */
    }
    if (!regime_has_2_ranges(mmu_idx)) {
        return EX_ASID_GLOBAL;
    }
    tcr = regime_tcr(env, mmu_idx);
    /* TCR_ELx.A1 (bit 22) selects which TTBR holds the ASID */
    ttbr = extract64(tcr, 22, 1) ? env->cp15.ttbr1_el[regime_el(mmu_idx)]
                                 : env->cp15.ttbr0_el[regime_el(mmu_idx)];
    bits = FIELD_EX64(GET_IDREG(&env_archcpu(env)->isar, ID_AA64MMFR0),
                      ID_AA64MMFR0, ASIDBITS) == 2 ? 16 : 8;
    return extract64(ttbr, 48, bits);
}

static uint16_t ex_vmid_of(CPUARMState *env, ARMMMUIdx mmu_idx)
{
    if (mmu_idx == ARMMMUIdx_Stage2_S) {
        return extract64(env->cp15.vsttbr_el2, 48, 16);
    }
    if (!regime_is_stage2(mmu_idx) &&
        (regime_el(mmu_idx) != 1 || !arm_feature(env, ARM_FEATURE_EL2))) {
        return 0;
    }
    return extract64(env->cp15.vttbr_el2, 48, 16);
}

static bool ex_tracked(ARMMMUIdx mmu_idx)
{
    if (regime_is_stage2(mmu_idx)) {
        return true;
    }
    switch (regime_el(mmu_idx)) {
    case 1:
    case 2:
        return true;
    default:
        return false;
    }
}

/* the regime number an entry is filed under */
static uint8_t ex_regime_of(ARMMMUIdx mmu_idx)
{
    if (regime_is_stage2(mmu_idx)) {
        return mmu_idx == ARMMMUIdx_Stage2_S ? EX_REGIME_S2_S : EX_REGIME_S2;
    }
    return regime_el(mmu_idx);
}

/* --------------------------------------------------------------- report */

static void ex_report(CPUARMState *env, const char *cls, const ExEntry *old,
                      uint64_t new_desc, uint64_t new_oa)
{
    CPUState *cs = env_cpu(env);
    uint64_t diff = old->desc_val ^ new_desc;

    if (ex_site_seen(0, (uint64_t)env->pc, old->key.va)) {
        return;
    }
    qemu_log_mask(LOG_EXACT,
             "exact-tlb: VIOLATION %s cpu=%d pc=0x%" PRIx64 " el=%d regime=%s "
             "asid=%s0x%x vmid=0x%x va=0x%" PRIx64 " level=%d\n"
             "exact-tlb:   descriptor at PA 0x%" PRIx64
             " changed 0x%016" PRIx64 " -> 0x%016" PRIx64 " (diff 0x%" PRIx64 ")\n"
             "exact-tlb:   output address 0x%" PRIx64 " -> 0x%" PRIx64
             ", filled by cpu=%u, %" PRIu64 " TLBI ops executed since (none "
             "of them covered this entry)\n",
             cls, cs->cpu_index, (uint64_t)env->pc, arm_current_el(env),
             ex_regime_name(old->key.regime),
             old->key.asid == EX_ASID_GLOBAL ? "global " : "",
             (unsigned)(old->key.asid == EX_ASID_GLOBAL ? 0 : old->key.asid),
             (unsigned)old->key.vmid, old->key.va, (int)old->level,
             old->desc_pa, old->desc_val, new_desc, diff,
             old->oa, new_oa, (unsigned)old->fill_cpu,
             ex_tlbi_seq - old->fill_tlbi_seq);
}

/* --------------------------------------------------- page table stores */

/*
 * Watching the stores themselves, rather than only comparing descriptors at
 * walk time, is what makes a break-before-make violation visible. A kernel
 * that clears a descriptor and writes it again without an invalidation in
 * between leaves a window in which hardware may still be using the old
 * translation, and the final value may differ only in permission bits, which
 * is indistinguishable at walk time from a legal in-place permission change.
 *
 * Pages holding descriptors we have walked are marked with QEMU's existing
 * code-page protection, so every store to them takes the slow path. That
 * callback runs *before* the store, so the new value is read back lazily, at
 * the next event on the same vCPU.
 */

/*
 * caller holds ex_lock: could any PE still be caching a translation that came
 * from this very descriptor? Matching the address alone is not enough. Early
 * boot maps the same addresses through several page table hierarchies in turn,
 * so an entry for the address may well have been produced by a different
 * descriptor, and clearing this one then owes nothing.
 */
static bool ex_key_cached(const ExKey *key, uint64_t desc_pa)
{
    unsigned i;

    for (i = 0; i < ex_ncpus; i++) {
        ExEntry *e;

        if (!ex_tab_cpu[i]) {
            continue;
        }
        e = g_hash_table_lookup(ex_tab_cpu[i], key);
        if (e && e->desc_pa == desc_pa) {
            return true;
        }
    }
    return false;
}

/* caller holds ex_lock; resolves this vCPU's outstanding store */
static void ex_resolve_locked(CPUState *cs)
{
    ExPending *p = &ex_pending[cs->cpu_index];
    ram_addr_t a, end;

    if (!p->valid) {
        return;
    }
    p->valid = false;
    end = p->ram_addr + p->size;
    for (a = p->ram_addr & ~7ULL; a < end; a += 8) {
        ExDesc *d = g_hash_table_lookup(ex_desc, GUINT_TO_POINTER(a));
        uint64_t newv, old;

        if (!d) {
            continue;           /* not a descriptor we have ever walked */
        }
        newv = ldq_le_p(d->host);
        old = d->last_val;
        if (newv == old) {
            continue;
        }
        ex_stat_desc_stores++;
        if ((old & 1) && !(newv & 1)) {
            /*
             * The break half. It only creates an obligation if the
             * translation could still be cached: a page table page that was
             * torn down (and invalidated) earlier and is now being reused is
             * cleared and refilled with no invalidation in between, entirely
             * legally, and its entries are long gone from the shadow.
             */
            if (d->have_key && ex_key_cached(&d->key, d->desc_pa)) {
                d->broke_seq = ex_tlbi_seq + 1;
                d->broke_val = old;
                d->broke_pc = p->pc;
                d->broke_cpu = cs->cpu_index;
            }
        } else if (!(old & 1) && (newv & 1)) {
            /*
             * Clearing an entry and writing it again with no invalidation is
             * a legal idiom for a permission change: the architecture does
             * not require break-before-make for those, and Linux uses it in
             * ptep_modify_prot_start()/commit() with the flush deferred to
             * the mmu_gather. It is only a violation if the mapping really
             * changed, or if it is part of a contiguous block, where
             * hardware may hold one entry covering the whole block.
             */
            if (d->broke_seq && d->broke_seq == ex_tlbi_seq + 1 &&
                (((d->broke_val ^ newv) & ~EX_BENIGN_MASK) ||
                 ((d->broke_val | newv) & (1ULL << 52)))) {
                ex_stat_bbm++;
                ex_stat_violations++;
                if (!ex_site_seen(1, d->broke_pc, p->pc)) {
                    qemu_log_mask(LOG_EXACT,
                        "exact-tlb: VIOLATION break-before-make without the "
                        "invalidation: descriptor at ram 0x%" PRIx64
                        " was cleared by cpu=%d pc=0x%" PRIx64 " and made "
                        "valid again by cpu=%d pc=0x%" PRIx64
                        " with no TLBI in between\n"
                        "exact-tlb:   0x%016" PRIx64 " -> 0 -> 0x%016" PRIx64
                        "%s\n",
                        (uint64_t)a, d->broke_cpu, d->broke_pc,
                        cs->cpu_index, p->pc, d->broke_val, newv,
                        ((d->broke_val | newv) & (1ULL << 52))
                        ? " (contiguous block)" : "");
                }
            }
            d->broke_seq = 0;
        }
        d->last_val = newv;
    }
}

/* remember a descriptor we have walked, and make stores to its page trap */
static ram_addr_t ex_desc_note(void *host, uint64_t val, const ExKey *key,
                               uint64_t desc_pa)
{
    MemoryRegion *mr;
    ram_addr_t offset, ra;
    ExDesc *d;

    if (!host) {
        return 0;               /* descriptor is not in RAM */
    }
    mr = memory_region_from_host(host, &offset);
    if (!mr || !memory_region_is_ram(mr)) {
        return 0;
    }
    ra = memory_region_get_ram_addr(mr) + offset;

    d = g_hash_table_lookup(ex_desc, GUINT_TO_POINTER(ra));
    if (d) {
        d->last_val = val;
        d->key = *key;
        d->desc_pa = desc_pa;
        d->have_key = true;
        return ra;
    }
    d = g_new0(ExDesc, 1);
    d->host = host;
    d->last_val = val;
    d->key = *key;
    d->desc_pa = desc_pa;
    d->have_key = true;
    g_hash_table_insert(ex_desc, GUINT_TO_POINTER(ra), d);

    if (!g_hash_table_contains(ex_pt_pages,
                               GUINT_TO_POINTER(ra & TARGET_PAGE_MASK))) {
        g_hash_table_add(ex_pt_pages, GUINT_TO_POINTER(ra & TARGET_PAGE_MASK));
        ex_stat_pt_pages++;
        /*
         * Reuse the protection QEMU already has for pages holding translated
         * code: it arms TLB_NOTDIRTY on every vCPU's existing entries, and
         * for a page with no code the invalidation path is a no-op, so the
         * page stays armed for every subsequent store.
         */
        tlb_protect_code(ra & TARGET_PAGE_MASK);
    }
    return ra;
}

void arm_exact_ptwatch_write(CPUState *cs, uint64_t ram_addr, unsigned size,
                             uintptr_t retaddr)
{
    CPUARMState *env = cpu_env(cs);

    /*
     * The same callback serves the instruction cache model: pages holding
     * translated code already take this slow path, which is how QEMU keeps
     * translations coherent, and that is exactly the coherence real hardware
     * does not provide.
     */
    arm_exact_icache_store(cs, ram_addr, size);

    if (!arm_exact_tlb_enabled || !ex_desc) {
        return;
    }
    qemu_mutex_lock(&ex_lock);
    if (g_hash_table_contains(ex_pt_pages,
                              GUINT_TO_POINTER(ram_addr & TARGET_PAGE_MASK))) {
        ex_resolve_locked(cs);
        ex_pending[cs->cpu_index] = (ExPending){
            .ram_addr = ram_addr, .size = size, .pc = env->pc, .valid = true,
        };
    }
    qemu_mutex_unlock(&ex_lock);
}

/* ------------------------------------------------------------ contiguous */

/*
 * A leaf with the contiguous hint (bit 52) tells the CPU it may cache one TLB
 * entry covering the whole aligned block. The architecture then requires every
 * descriptor of that block to be consistent: same attributes, consecutive
 * output addresses, and the hint set throughout. Linux maintains this by
 * unfolding a block (clear every entry, invalidate, then rewrite) before
 * changing any single entry, so a block that is fully valid but internally
 * inconsistent is a bug of the contpte/hugetlb kind.
 *
 * Blocks containing an invalid entry are skipped: that is the legitimate
 * transient state in the middle of contpte_convert(), where entries are
 * cleared one at a time before the invalidation.
 */
#define EX_CONTIG_MAX     128       /* the largest block: 16K granule, level 3 */
#define EX_CONTIG_SAMPLE  64        /* re-walks between block re-checks */

/*
 * How many descriptors a contiguous block covers (DDI0487 D8.6). It depends on
 * the translation granule, which the walk does not hand us directly - but the
 * leaf's page size does: at level 3 it *is* the granule, and a level-2 block is
 * granule + (granule - 3) address bits wide, so lg = 2g - 3.
 *
 *   granule   level 3   level 2
 *      4K       16        16
 *     16K      128        32
 *     64K       32        32
 */
static unsigned ex_contig_entries(int level, int lg_page_size)
{
    int g = level == 3 ? lg_page_size : (lg_page_size + 3) / 2;

    switch (g) {
    case 12: return 16;
    case 14: return level == 3 ? 128 : 32;
    case 16: return 32;
    default: return 0;          /* not a granule we know: do not guess */
    }
}

static void ex_check_contig(CPUARMState *env, const ExKey *key, uint64_t desc_pa,
                            uint64_t desc_val, int level, int lg_page_size)
{
    uint64_t oa_mask = MAKE_64BIT_MASK(lg_page_size, 48 - lg_page_size);
    uint64_t attr_mask = ~(oa_mask | (1ULL << 10) | (1ULL << 51) | (3ULL << 6));
    unsigned entries = ex_contig_entries(level, lg_page_size);
    uint64_t base_pa, d[EX_CONTIG_MAX], base_oa;
    unsigned i;

    if (!entries) {
        return;
    }
    base_pa = desc_pa & ~(uint64_t)((entries * 8) - 1);

    ex_stat_contig_checked++;

    for (i = 0; i < entries; i++) {
        MemTxResult res;

        d[i] = address_space_ldq(&address_space_memory, base_pa + i * 8,
                                 MEMTXATTRS_UNSPECIFIED, &res);
        if (res != MEMTX_OK) {
            return;
        }
        if (!(d[i] & 1)) {
            return;             /* mid-unfold, not a bug */
        }
    }

    base_oa = d[0] & oa_mask;
    for (i = 0; i < entries; i++) {
        const char *why = NULL;

        if (!(d[i] & (1ULL << 52))) {
            why = "contiguous hint missing on a member";
        } else if ((d[i] & attr_mask) != (d[0] & attr_mask)) {
            why = "attributes differ inside the block";
        } else if ((d[i] & oa_mask) !=
                   base_oa + ((uint64_t)i << lg_page_size)) {
            why = "output addresses not consecutive";
        }
        if (why) {
            if (!ex_site_seen(2, (uint64_t)env->pc, key->va)) {
                qemu_log_mask(LOG_EXACT,
                    "exact-tlb: VIOLATION contiguous block inconsistent (%s) "
                    "cpu=%d pc=0x%" PRIx64 " va=0x%" PRIx64 " regime=%s\n"
                    "exact-tlb:   block at PA 0x%" PRIx64 ", entry %u of %u is "
                    "0x%016" PRIx64 ", entry 0 is 0x%016" PRIx64 "\n",
                    why, env_cpu(env)->cpu_index, (uint64_t)env->pc,
                    key->va, ex_regime_name(key->regime), base_pa, i,
                    entries, d[i], d[0]);
            }
            ex_stat_violations++;
            return;
        }
    }
}

/* ----------------------------------------------------------------- leaf */

/*
 * An intermediate descriptor: hardware may keep it in a walk cache, which is
 * only removed by an invalidation that is not of the last-level form. A
 * kernel that unmaps a table and frees the page with only a leaf invalidation
 * leaves hardware walking through memory it has given back to the allocator.
 */
void arm_exact_tlb_table(CPUARMState *env, ARMMMUIdx mmu_idx,
                         ARMSecuritySpace space, uint64_t va, uint64_t desc_pa,
                         uint64_t desc_val, int level, int lg_cover, void *host)
{
    ExKey key;
    ExEntry *e;
    GHashTable *tab;
    ram_addr_t ra_desc = 0;

    if (!arm_exact_tlb_enabled || !ex_ncpus || !ex_tracked(mmu_idx)) {
        return;
    }

    memset(&key, 0, sizeof(key));
    key.va = va & ~((1ULL << lg_cover) - 1);
    /*
     * Intermediate entries are not tagged by the leaf's nG bit: they belong
     * to whichever TTBR was walked. Kernel space (TTBR1) is global, user
     * space carries the current ASID.
     */
    key.asid = (regime_has_2_ranges(mmu_idx) && !((int64_t)va < 0))
               ? ex_asid_of(env, mmu_idx, 1ULL << 11) : EX_ASID_GLOBAL;
    key.vmid = ex_vmid_of(env, mmu_idx);
    key.regime = ex_regime_of(mmu_idx);
    key.space = space;
    key.level = level;

    qemu_mutex_lock(&ex_lock);
    ex_resolve_locked(env_cpu(env));
    ra_desc = ex_desc_note(host, desc_val, &key, desc_pa);
    tab = ex_tab_of(env_cpu(env)->cpu_index);
    if (!tab) {
        qemu_mutex_unlock(&ex_lock);
        return;
    }
    e = g_hash_table_lookup(tab, &key);
    if (e) {
        ex_stat_hits++;
        if (e->desc_pa == desc_pa && e->desc_val != desc_val) {
            /*
             * Only the next level table address matters here; the upper
             * attribute bits are gathered on the way down and a change to
             * them is a permission change, not a stale walk cache.
             */
            if ((e->desc_val ^ desc_val) & MAKE_64BIT_MASK(12, 36)) {
                ex_stat_violations++;
                ExDesc *sd = g_hash_table_lookup(ex_desc,
                                                 GUINT_TO_POINTER(ra_desc));
                if (!ex_site_seen(3, (uint64_t)env->pc, key.va)) {
                    qemu_log_mask(LOG_EXACT,
                        "exact-tlb: VIOLATION cached table descriptor changed "
                        "without a non-last-level invalidation cpu=%d "
                        "pc=0x%" PRIx64 " regime=%s va=0x%" PRIx64
                        " level=%d\n"
                        "exact-tlb:   descriptor at PA 0x%" PRIx64
                        " changed 0x%016" PRIx64 " -> 0x%016" PRIx64
                        ", next level table 0x%" PRIx64 " -> 0x%" PRIx64
                        ", last store to it from pc=0x%" PRIx64 "\n",
                        env_cpu(env)->cpu_index, (uint64_t)env->pc,
                        ex_regime_name(key.regime), key.va, level, desc_pa,
                        e->desc_val, desc_val,
                        (uint64_t)(e->desc_val & MAKE_64BIT_MASK(12, 36)),
                        (uint64_t)(desc_val & MAKE_64BIT_MASK(12, 36)),
                        sd ? sd->broke_pc : 0);
                }
            } else {
                ex_stat_benign++;
            }
        }
        e->desc_pa = desc_pa;
        e->desc_val = desc_val;
        e->fill_tlbi_seq = ex_tlbi_seq;
    } else if (g_hash_table_size(tab) < EX_MAX_ENTRIES) {
        ExKey *nk = g_memdup2(&key, sizeof(key));

        e = g_new0(ExEntry, 1);
        e->key = key;
        e->desc_pa = desc_pa;
        e->desc_val = desc_val;
        e->level = level;
        e->lg_page_size = lg_cover;
        e->is_table = true;
        e->fill_cpu = env_cpu(env)->cpu_index;
        e->fill_tlbi_seq = ex_tlbi_seq;
        g_hash_table_insert(tab, nk, e);
        ex_stat_table_fills++;
    }
    qemu_mutex_unlock(&ex_lock);
}

void arm_exact_tlb_leaf(CPUARMState *env, ARMMMUIdx mmu_idx,
                        ARMSecuritySpace space, uint64_t va, uint64_t desc_pa,
                        uint64_t desc_val, uint64_t oa, int level,
                        int lg_page_size, void *host)
{
    ExKey key;
    ExEntry *e;
    GHashTable *ex_tab;

    if (!arm_exact_tlb_enabled || !ex_ncpus || !ex_tracked(mmu_idx)) {
        return;
    }

    memset(&key, 0, sizeof(key));
    key.va = va & ~((1ULL << lg_page_size) - 1);
    key.asid = ex_asid_of(env, mmu_idx, desc_val);
    key.vmid = ex_vmid_of(env, mmu_idx);
    key.regime = ex_regime_of(mmu_idx);
    key.space = space;
    key.level = level;

    qemu_mutex_lock(&ex_lock);
    ex_resolve_locked(env_cpu(env));
    ex_desc_note(host, desc_val, &key, desc_pa);
    ex_tab = ex_tab_of(env_cpu(env)->cpu_index);
    if (!ex_tab) {
        qemu_mutex_unlock(&ex_lock);
        return;
    }
    e = g_hash_table_lookup(ex_tab, &key);
    if (e) {
        ex_stat_hits++;
        if (e->desc_pa == desc_pa && e->desc_val != desc_val) {
            uint64_t diff = e->desc_val ^ desc_val;

            if (diff & ~EX_BENIGN_MASK) {
                ex_stat_violations++;
                ex_report(env, (desc_val & 1) && (e->desc_val & 1)
                          ? "live translation changed without invalidation"
                          : "translation reused without invalidation",
                          e, desc_val, oa);
            } else {
                ex_stat_benign++;
            }
            e->contig_countdown = 0;    /* changed: check it now */
        }
        /*
         * Re-walks are sampled rather than checked every time: the block is
         * 16 extra guest loads, and a corruption introduced through a member
         * we never walk would otherwise stay invisible.
         */
        if ((desc_val & (1ULL << 52)) &&
            ex_contig_entries(level, lg_page_size) &&
            key.space == ARMSS_NonSecure && e->contig_countdown-- == 0) {
            e->contig_countdown = EX_CONTIG_SAMPLE;
            ex_check_contig(env, &key, desc_pa, desc_val, level, lg_page_size);
        }
        e->desc_pa = desc_pa;
        e->desc_val = desc_val;
        e->oa = oa;
        e->level = level;
        e->lg_page_size = lg_page_size;
        e->fill_tlbi_seq = ex_tlbi_seq;
    } else if (g_hash_table_size(ex_tab) < EX_MAX_ENTRIES) {
        ExKey *nk = g_memdup2(&key, sizeof(key));

        e = g_new0(ExEntry, 1);
        e->key = key;
        e->desc_pa = desc_pa;
        e->desc_val = desc_val;
        e->oa = oa;
        e->level = level;
        e->lg_page_size = lg_page_size;
        e->fill_cpu = env_cpu(env)->cpu_index;
        e->fill_tlbi_seq = ex_tlbi_seq;
        g_hash_table_insert(ex_tab, nk, e);
        ex_stat_fills++;
        if ((desc_val & (1ULL << 52)) &&
            ex_contig_entries(level, lg_page_size) &&
            key.space == ARMSS_NonSecure) {
            ex_check_contig(env, &key, desc_pa, desc_val, level, lg_page_size);
        }
    } else if (!ex_capped) {
        ex_capped = true;
        qemu_log_mask(LOG_EXACT,
                      "exact-tlb: entry cap (%u) reached, stopped tracking new "
                      "translations; detections may be missed\n",
                      EX_MAX_ENTRIES);
    }
    qemu_mutex_unlock(&ex_lock);
}

/* ----------------------------------------------------------------- TLBI */

typedef struct ExInval {
    bool all;               /* drop everything we track */
    bool last_level_only;   /* TLBI VALE1/VAALE1: leaves only, walk caches stay */
    int ttl_level;          /* TTL hint: only this level, or -1 for all */
    bool match_asid;
    bool match_globals;     /* a VA-matched op also hits global entries */
    bool match_va;
    bool match_range;
    bool regime_pair;       /* EL1&0 stage 1 and stage 2 together */
    uint32_t asid;
    uint64_t va;
    uint64_t base, length;  /* for the TLBI R* range operations */
    uint8_t regime;
    uint16_t vmid;
    bool match_vmid;
} ExInval;

/* how invalidations were scoped, to show how much detection we give away */
static uint64_t ex_tlbi_by_kind[6];
enum { EX_K_ALL, EX_K_REGIME, EX_K_ASID, EX_K_VA, EX_K_RANGE, EX_K_UNDECODED };

static gboolean ex_inval_cb(gpointer key, gpointer val, gpointer opaque)
{
    const ExInval *inv = opaque;
    const ExEntry *e = val;

    if (inv->all) {
        return TRUE;
    }
    /*
     * The last-level forms (VALE1, VAALE1 and their range variants) leave
     * cached intermediate descriptors alone, which is exactly how a kernel
     * that frees a page table with only a leaf invalidation goes wrong.
     */
    if (inv->last_level_only && e->is_table) {
        return FALSE;
    }
    /* a TTL hint permits hardware to invalidate only entries of that level */
    if (inv->ttl_level >= 0 && e->key.level != inv->ttl_level) {
        return FALSE;
    }
    if (inv->regime_pair) {
        /* ALLE1 and VMALLS12E1 reach EL1&0 stage 1 and stage 2 alike */
        if (e->key.regime != 1 && e->key.regime != EX_REGIME_S2 &&
            e->key.regime != EX_REGIME_S2_S) {
            return FALSE;
        }
    } else if (e->key.regime != inv->regime) {
        return FALSE;
    }
    if (inv->match_vmid && e->key.vmid != inv->vmid) {
        return FALSE;
    }
    if (inv->match_asid && e->key.asid != inv->asid &&
        !(inv->match_globals && e->key.asid == EX_ASID_GLOBAL)) {
        /*
         * TLBI ASIDE1 invalidates only entries of that ASID and leaves
         * global entries alone; TLBI VAE1/VALE1 also hit global entries
         * for the given VA.
         */
        return FALSE;
    }
    if (inv->match_va) {
        uint64_t mask = ~((1ULL << e->lg_page_size) - 1);

        if ((inv->va & mask) != e->key.va) {
            return FALSE;
        }
    }
    if (inv->match_range) {
        uint64_t esize = 1ULL << e->lg_page_size;

        /* intersect [base, base+length) with the entry's page range */
        if (e->key.va + esize <= inv->base ||
            e->key.va >= inv->base + inv->length) {
            return FALSE;
        }
    }
    return TRUE;
}

/*
 * TTL, Xt[47:44] of a by-address operation: [45:44] names a level and
 * [47:46] the granule it applies to. The architecture says the hint is
 * ignored unless that granule is the one in use, which matters because an
 * operand that was not masked to Xt[43:0] leaves address bits in this field.
 * Only the 4K granule is recognised here; with any other the hint is dropped,
 * which can only cost detections.
 */
/*
 * The TLBI R* operand: BaseADDR[36:0], TTL[38:37], NUM[43:39], SCALE[45:44],
 * TG[47:46]. Used by the EL1 range forms and, with the base being an IPA
 * rather than a VA, by the stage 2 range forms. Returns false for a reserved
 * granule encoding, where the caller should fall back to dropping everything.
 */
static bool ex_decode_range(CPUARMState *env, uint64_t value, bool stage2,
                            uint64_t *base, uint64_t *length, int *ttl)
{
    unsigned tg = extract64(value, 46, 2);
    unsigned shift = tg == 1 ? 12 : tg == 2 ? 14 : tg == 3 ? 16 : 0;
    unsigned num = extract64(value, 39, 5);
    unsigned scale = extract64(value, 44, 2);
    unsigned rttl = extract64(value, 37, 2);
    uint64_t tcr = stage2 ? env->cp15.vtcr_el2 : regime_tcr(env, arm_mmu_idx(env));
    bool ds = extract64(tcr, 59, 1);
    int64_t b;

    if (!shift) {
        return false;
    }
    b = (!stage2 && extract64(value, 36, 1)) ? sextract64(value, 0, 37)
                                             : (int64_t)extract64(value, 0, 37);
    *base = (uint64_t)b << (ds ? 16 : shift);
    *length = (uint64_t)(num + 1) << (5 * scale + 1 + shift);
    *ttl = rttl ? (int)rttl : -1;
    return true;
}

static int ex_ttl_level(uint64_t value)
{
    unsigned level = extract64(value, 44, 2);
    unsigned gran = extract64(value, 46, 2);

    return (level && gran == 1) ? (int)level : -1;
}

/*
 * Called for every AArch64 TLBI (opc0 == 1, crn == 8 or 9 for the NXS
 * aliases). Decoding here rather than in each of the forty writefns keeps
 * the change to upstream code to a single line.
 */
void arm_exact_tlb_tlbi(CPUARMState *env, const struct ARMCPRegInfo *ri,
                        uint64_t value)
{
    ExInval inv = { .ttl_level = -1 };
    unsigned el = regime_el(arm_mmu_idx(env));
    unsigned self = env_cpu(env)->cpu_index;
    bool broadcast;
    unsigned i;

    if (!arm_exact_tlb_enabled || !ex_ncpus) {
        return;
    }

    switch (ri->opc1) {
    case 0:                             /* EL1 operations */
        inv.regime = el == 2 ? 2 : 1;   /* E2H&TGE redirects EL1 ops to EL2&0 */
        /*
         * Only the EL1&0 regime is VMID tagged. When a VHE host at EL2 issues
         * these for its own mappings they are not, and scoping them by
         * whatever VMID the hypervisor happens to have loaded makes the
         * invalidation miss every host entry: they are filed under VMID 0,
         * while VTTBR_EL2 holds a guest's VMID whenever one is resident.
         */
        inv.match_vmid = inv.regime == 1;
        inv.vmid = extract64(env->cp15.vttbr_el2, 48, 16);
        switch (ri->crm) {
        case 2:                         /* range, inner shareable */
        case 5:                         /* range, local */
        case 6:                         /* range, outer shareable */
            /*
             * TLBI R<op>: BaseADDR[36:0], TTL[38:37], NUM[43:39],
             * SCALE[45:44], TG[47:46], ASID[63:48]. The covered length is
             * (NUM + 1) << (5 * SCALE + 1 + page_shift). With TCR.DS the
             * base is always shifted by 16 so it can address 52 VA bits.
             */
            {
                if (!ex_decode_range(env, value, false, &inv.base,
                                     &inv.length, &inv.ttl_level)) {
                    inv.all = true;     /* reserved granule encoding */
                    break;
                }
                inv.match_range = true;
                switch (ri->opc2) {
                case 3:                 /* RVALE1: last level only */
                case 7:                 /* RVAALE1 */
                    inv.last_level_only = true;
                    break;
                default:
                    break;
                }
                switch (ri->opc2) {
                case 1:                 /* RVAE1  */
                case 3:                 /* RVALE1 */
                    inv.match_asid = true;
                    inv.match_globals = true;
                    inv.asid = extract64(value, 48, 16);
                    break;
                case 5:                 /* RVAAE1  */
                case 7:                 /* RVAALE1 */
                    break;              /* all ASIDs, range only */
                default:
                    inv.match_range = false;
                    inv.all = true;
                    break;
                }
            }
            break;
        case 3:                         /* inner shareable */
        case 7:                         /* local */
        case 1:                         /* outer shareable */
            switch (ri->opc2) {
            case 0:                     /* VMALLE1  */
                break;                  /* whole regime, matched above */
            case 1:                     /* VAE1  */
            case 5:                     /* VALE1 */
                inv.match_asid = true;
                inv.match_globals = true;
                inv.asid = extract64(value, 48, 16);
                inv.match_va = true;
                inv.va = sextract64(value << 12, 0, 56);
                inv.last_level_only = ri->opc2 == 5;
                inv.ttl_level = ex_ttl_level(value);
                break;
            case 2:                     /* ASIDE1 */
                inv.match_asid = true;
                inv.asid = extract64(value, 48, 16);
                break;
            case 3:                     /* VAAE1  */
            case 7:                     /* VAALE1 */
                inv.match_va = true;
                inv.va = sextract64(value << 12, 0, 56);
                inv.last_level_only = ri->opc2 == 7;
                inv.ttl_level = ex_ttl_level(value);
                break;
            default:
                inv.all = true;
                break;
            }
            break;
        default:
            inv.all = true;             /* anything we do not decode */
            break;
        }
        break;
    case 4:                             /* EL2 operations */
        switch (ri->crm) {
        case 0:                         /* IPAS2*, inner shareable */
        case 4:                         /* IPAS2*, local */
            /*
             * TLBI IPAS2E1 invalidates stage 2 entries for one IPA of the
             * current VMID and leaves the combined stage 1 and 2 entries
             * alone: that is what the following VMALLE1IS is for. A kernel
             * that issues only the first is exactly the bug this catches.
             *
             * opc2 1 and 5 take a bare IPA, 2 and 6 are the range forms and
             * take the R* operand layout. They share this crm, so decoding
             * one as the other silently misses every invalidation KVM makes
             * through __kvm_tlb_flush_vmid_range().
             */
            inv.regime = EX_REGIME_S2;
            inv.match_vmid = true;
            inv.vmid = extract64(env->cp15.vttbr_el2, 48, 16);
            inv.last_level_only = ri->opc2 == 5 || ri->opc2 == 6;
            if (ri->opc2 == 2 || ri->opc2 == 6) {
                if (!ex_decode_range(env, value, true, &inv.base,
                                     &inv.length, &inv.ttl_level)) {
                    inv.all = true;
                    break;
                }
                inv.match_range = true;
            } else if (ri->opc2 == 1 || ri->opc2 == 5) {
                inv.match_va = true;
                inv.va = extract64(value, 0, 36) << 12;         /* IPA */
            } else {
                inv.all = true;
            }
            break;
        case 3:                         /* inner shareable */
        case 7:                         /* local */
        case 1:                         /* outer shareable */
            switch (ri->opc2) {
            case 4:                     /* ALLE1: every VMID, both stages */
            case 6:                     /* VMALLS12E1: this VMID, both stages */
                inv.regime_pair = true; /* EL1&0 stage 1 plus stage 2 */
                if (ri->opc2 == 6) {
                    inv.match_vmid = true;
                    inv.vmid = extract64(env->cp15.vttbr_el2, 48, 16);
                }
                break;
            case 0:                     /* ALLE2  */
            case 1:                     /* VAE2   */
            case 5:                     /* VALE2  */
                inv.regime = 2;
                if (ri->opc2 != 0) {
                    inv.match_va = true;
                    inv.va = sextract64(value << 12, 0, 56);
                    inv.last_level_only = ri->opc2 == 5;
                    inv.ttl_level = ex_ttl_level(value);
                }
                break;
            default:
                inv.all = true;
                break;
            }
            break;
        default:
            inv.all = true;
            break;
        }
        break;
    case 6:                             /* EL3 */
    default:
        inv.all = true;
        break;
    }

    /*
     * An ASID-matched invalidation must also drop global entries, and our
     * ASID match already lets them through; a VA-only operation keeps
     * entries of other pages. Anything we could not decode is a full drop.
     */
    /*
     * crm tells us the shareability: 3 and 2 are inner shareable, 1 and 6
     * outer shareable, 7 and 5 are local to this PE. HCR_EL2.FB promotes
     * EL1 operations to broadcast.
     */
    switch (ri->crm) {
    case 3: case 2: case 1: case 6:
        broadcast = true;
        break;
    default:
        broadcast = ri->opc1 == 0 && (arm_hcr_el2_eff(env) & HCR_FB);
        break;
    }

    ex_tlbi_by_kind[inv.all ? EX_K_ALL :
                    inv.match_range ? EX_K_RANGE :
                    inv.match_va ? EX_K_VA :
                    inv.match_asid ? EX_K_ASID : EX_K_REGIME]++;

    qemu_mutex_lock(&ex_lock);
    ex_resolve_locked(env_cpu(env));
    ex_tlbi_seq++;
    if (broadcast) {
        for (i = 0; i < ex_ncpus; i++) {
            if (ex_tab_cpu[i]) {
                ex_stat_removed += g_hash_table_foreach_remove(ex_tab_cpu[i],
                                                        ex_inval_cb, &inv);
            }
        }
        ex_stat_broadcast++;
    } else if (ex_tab_cpu[self]) {
        ex_stat_removed += g_hash_table_foreach_remove(ex_tab_cpu[self],
                                                      ex_inval_cb, &inv);
        ex_stat_local++;
    }
    qemu_mutex_unlock(&ex_lock);
}

void arm_exact_tlb_dump(void)
{
    unsigned i, live = 0;

    if (!ex_ncpus) {
        return;
    }
    for (i = 0; i < ex_ncpus; i++) {
        if (ex_tab_cpu[i]) {
            live += g_hash_table_size(ex_tab_cpu[i]);
        }
    }
    qemu_log_mask(LOG_EXACT,
                  "exact-tlb: %u live entries, %" PRIu64 " fills, %" PRIu64
                  " re-walks, %" PRIu64 " benign changes, %" PRIu64
                  " violations, %" PRIu64 " TLBI ops removing %" PRIu64
                  " entries\n"
                  "exact-tlb: TLBI scope: all=%" PRIu64 " regime=%" PRIu64
                  " asid=%" PRIu64 " va=%" PRIu64 " range=%" PRIu64 "\n"
                  "exact-tlb: contiguous blocks checked: %" PRIu64
                  ", TLBI broadcast=%" PRIu64 " local=%" PRIu64
                  ", walk cache entries filled=%" PRIu64 "\n"
                  "exact-tlb: page table pages watched: %" PRIu64
                  ", descriptor stores seen: %" PRIu64
                  ", break-before-make violations: %" PRIu64 "\n",
                  live, ex_stat_fills, ex_stat_hits,
                  ex_stat_benign, ex_stat_violations, ex_tlbi_seq,
                  ex_stat_removed,
                  ex_tlbi_by_kind[EX_K_ALL], ex_tlbi_by_kind[EX_K_REGIME],
                  ex_tlbi_by_kind[EX_K_ASID], ex_tlbi_by_kind[EX_K_VA],
                  ex_tlbi_by_kind[EX_K_RANGE], ex_stat_contig_checked,
                  ex_stat_broadcast, ex_stat_local, ex_stat_table_fills,
                  ex_stat_pt_pages, ex_stat_desc_stores, ex_stat_bbm);
}
