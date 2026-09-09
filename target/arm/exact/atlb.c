/*
 * qemu-exact: architectural TLB shadow and break-before-make detector.
 *
 * Stock QEMU's softmmu TLB is untagged, so it must be flushed whenever the
 * ASID or VMID changes, and it re-walks the guest page tables far more often
 * than real hardware would. A real ARM64 TLB is tagged by (regime, VMID,
 * ASID). Hardware may evict entries, but software cannot rely on eviction.
 * Retaining entries until a matching TLBI models one allowed behavior.
 *
 * This models that rule as an observer. On every successful page table walk
 * we record the leaf descriptor (its physical address and value) under the
 * architectural key. TLBI operations remove entries with exactly the scope
 * the architecture gives them. If a later walk of the same key finds the
 * descriptor materially changed while our entry was still live, the guest
 * changed a mapping that hardware could still have been caching. Reports
 * identify candidate missing-TLBI or BBM errors; their architectural
 * preconditions still need checking against the guest's configuration.
 *
 * Changes that hardware is allowed to observe without an invalidation
 * (access flag, dirty state, permissions) are classified separately and not
 * reported.  This also applies while a hinted contiguous set is being
 * repainted: Arm rule R_JQQTC permits a complete OA/attributes/permissions
 * choice consistent with one valid member, not a mixture of members' fields.
 *
 * Limits (a clean run is not proof of architectural correctness):
 *  - per-PE shadows observe successful walks, not every hardware TLB lookup;
 *  - contiguous snapshots are observations, not proof of update completion;
 *  - no coalesced translations, stale permissions or MTE faults are injected;
 *  - TLBI completion is synchronous here; DSB completion is not modeled;
 *  - TLBI operations that are not decoded drop everything, which can only
 *    lose detections, never invent them;
 *  - the contiguous contract is limited to conventional AArch64 stage-1
 *    descriptors; other layouts are counted as unavailable.
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
#include "tlb-rules.h"

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
 *   7:6   AP[2:1]   permissions (allow updates with deferred TLBI; this
 *                   observer does not verify eventual permission revocation)
 *   53,54 PXN/UXN
 *   58:55 software use
 *   62:59 PBHA / ignored
 */
#define EX_BENIGN_MASK ((1ULL << 10) | (1ULL << 51) | (3ULL << 6) |     \
                        (1ULL << 53) | (1ULL << 54) |                   \
                        (0xfULL << 55) | (0xfULL << 59))

/*
 * The same idea for short descriptors (ARMv7 without LPAE), where the bits sit
 * somewhere else entirely - applying the long-descriptor mask to them reports
 * every ordinary permission change as a missing TLBI. A boot of a non-LPAE
 * kernel produced 11 such reports, all of them AP[1] (bit 11 of a section) and
 * XN (bit 4), before this existed.
 *
 *   level 1, section:    4 XN, 11:10 AP[1:0], 15 AP[2], 9 impl-def
 *   level 2, small page: 0 XN,  5:4  AP[1:0],  9 AP[2]
 *   level 2, large page: 15 XN, 5:4  AP[1:0],  9 AP[2]
 *
 * Large and small pages share the AP positions, so one mask covers level 2 if
 * it includes both XN bits.
 */
#define EX_BENIGN_MASK_V6_L1 ((1ULL << 4) | (3ULL << 10) | (1ULL << 15) | \
                              (1ULL << 9))
#define EX_BENIGN_MASK_V6_L2 ((1ULL << 0) | (3ULL << 4) | (1ULL << 9) |   \
                              (1ULL << 15))

static uint64_t ex_benign_mask(CPUARMState *env, ARMMMUIdx mmu_idx, int level)
{
    if (regime_using_lpae_format(env, mmu_idx)) {
        return EX_BENIGN_MASK;
    }
    return level == 1 ? EX_BENIGN_MASK_V6_L1 : EX_BENIGN_MASK_V6_L2;
}

static ExLeafPolicy ex_leaf_policy(CPUARMState *env, ARMMMUIdx mmu_idx,
                                   uint64_t va, int level)
{
    ExLeafPolicy p = { .benign_mask = ex_benign_mask(env, mmu_idx, level) };
    unsigned el = regime_el(mmu_idx);

    if (!regime_is_stage2(mmu_idx) && arm_el_is_aa64(env, el)) {
        ARMVAParameters param = aa64_va_parameters(env, va, mmu_idx, true,
                                                   false);

        p.s1_attrs = true;
        p.mair = env->cp15.mair_el[el];
        p.mair2 = env->cp15.mair2_el[el];
        p.aie = param.aie;
        p.mte = cpu_isar_feature(aa64_mte, env_archcpu(env));
        /* GP is a guarded-page permission, not a cacheability change. */
        if (cpu_isar_feature(aa64_bti, env_archcpu(env))) {
            p.benign_mask |= EX_DESC_GP;
        }
    }
    return p;
}

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
    void *host;                 /* host pointer to the descriptor */
    uint8_t size;               /* 8 for a long descriptor, 4 for a short one */
    uint64_t last_val;
    uint64_t broke_seq;         /* ex_tlbi_seq + 1 when cleared, 0 if live */
    uint64_t broke_val;         /* what it held before it was cleared */
    uint64_t broke_pc;
    int broke_cpu;
    ExKey key;                  /* what this descriptor last translated */
    uint64_t desc_pa;           /* where it lives, to tell hierarchies apart */
    bool have_key;
    ExLeafPolicy policy;
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
static uint64_t ex_stat_bbm_accepted;

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
static uint64_t ex_stat_contig[EX_CONTIG_STATES], ex_stat_contig_partial;
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

/*
 * Eight bytes for a long descriptor, four for a short one. The descriptor
 * watch reads back what a store changed, and reading a pair of short
 * descriptors as one long one reports a break-before-make violation against an
 * entry that never moved.
 */
static unsigned ex_desc_size(CPUARMState *env, ARMMMUIdx mmu_idx)
{
    return regime_using_lpae_format(env, mmu_idx) ? 8 : 4;
}

/*
 * A 32-bit guest keeps its PC in r15, not in env->pc, so every arm32 TLB
 * report attributed the offending access to pc=0x0. This is the third model
 * to have had it (icache.c, then dcache.c), which is why it was found by
 * auditing all of them for env->pc rather than waiting for the next report.
 */
static uint64_t ex_pc(CPUARMState *env)
{
    return is_a64(env) ? env->pc : env->regs[15];
}

static uint32_t ex_asid_of(CPUARMState *env, ARMMMUIdx mmu_idx, uint64_t desc,
                           int level)
{
    uint64_t tcr, ttbr;
    unsigned bits;

    if (regime_is_stage2(mmu_idx)) {
        return EX_ASID_GLOBAL;      /* stage 2 has no ASID, only a VMID */
    }
    if (!regime_using_lpae_format(env, mmu_idx)) {
        /*
         * Short descriptors (ARMv7 without LPAE). Two differences from the
         * long format, both of which read as "everything is global" if you
         * ignore them: the ASID is in CONTEXTIDR rather than the top of TTBR,
         * and nG is bit 17 of a section descriptor but bit 11 of a page
         * descriptor. A level-1 table descriptor has no nG at all.
         */
        uint32_t ng = level == 1 ? (1u << 17) : (1u << 11);

        if (!(desc & ng)) {
            return EX_ASID_GLOBAL;
        }
        return env->cp15.contextidr_el[1] & 0xff;
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
    /*
     * Track all 16 bits even when the guest has been told it only has 8.
     *
     * A CPU with 16-bit ASIDs may report ASIDBits == 0, and the architecture
     * then requires *software* to pass zero in ASID[15:8] - the hardware is
     * entitled to use whatever it is given. That mismatch is the whole subject
     * of c0900d15d31c ("arm64: Ensure bits ASID[15:8] are masked out when the
     * kernel uses 8-bit ASIDs"), where Linux leaves its ASID generation number
     * in those bits and two threads of one process end up with different
     * 16-bit ASIDs after a rollover.
     *
     * Tracking 16 bits is therefore the adversarial-but-legal reading, and it
     * costs nothing against a correct kernel: one that zeroes ASID[15:8] gives
     * the same key either way. Forcing the *CPU* to 8-bit ASIDs, as the FVP
     * profile did, removes the mismatch and makes the bug unreproducible.
     */
    bits = 16;
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

    if (ex_site_seen(0, ex_pc(env), old->key.va)) {
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
             cls, cs->cpu_index, ex_pc(env), arm_current_el(env),
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
 * between matters when the replacement changes something requiring BBM.
 * A clear/remake that changes only permissions is not itself a violation.
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
    /*
     * Step by four, not eight: a short descriptor (ARMv7 without LPAE) is four
     * bytes, and the kernel writes two adjacent ones with a single 64-bit
     * store. Reading that back as one eight-byte descriptor produced values
     * like 0x79945e7f79944e7f - two entries glued together - and reported a
     * break-before-make violation against a descriptor that had not changed,
     * tagged "(contiguous block)" because bit 52 landed inside the neighbour.
     * A long-descriptor table has nothing at the odd four-byte offsets, so
     * the extra lookups simply miss.
     *
     * Start from the eight-byte boundary even though the step is four: a store
     * that begins in the middle of a long descriptor - a memset or memcpy over
     * a page-table page, say - still overwrites the descriptor that starts
     * before it, and rounding down only to four would step straight past it.
     * Rounding down to eight and stepping by four covers both formats.
     */
    for (a = p->ram_addr & ~7ULL; a < end; a += 4) {
        ExDesc *d = g_hash_table_lookup(ex_desc, GUINT_TO_POINTER(a));
        uint64_t newv, old;

        if (!d) {
            continue;           /* not a descriptor we have ever walked */
        }
        newv = d->size == 4 ? ldl_le_p(d->host) : ldq_le_p(d->host);
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
             * the mmu_gather. R_JQQTC makes the same transition permissible
             * for a hinted contiguous set: while members differ, a lookup may
             * use one member's complete tuple. A difference outside the
             * modeled permission/attribute exceptions is checked separately.
             */
            bool pending_break = d->broke_seq &&
                                 d->broke_seq == ex_tlbi_seq + 1;

            if (pending_break &&
                ex_leaf_change_benign(d->policy, d->broke_val, newv)) {
                ex_stat_bbm_accepted++;
            } else if (pending_break) {
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
                        "\n",
                        (uint64_t)a, d->broke_cpu, d->broke_pc,
                        cs->cpu_index, p->pc, d->broke_val, newv);
                }
            }
            d->broke_seq = 0;
        }
        d->last_val = newv;
    }
}

/* remember a descriptor we have walked, and make stores to its page trap */
static ram_addr_t ex_desc_note(void *host, uint64_t val, const ExKey *key,
                               uint64_t desc_pa, unsigned size,
                               ExLeafPolicy policy)
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
        d->size = size;
        d->key = *key;
        d->desc_pa = desc_pa;
        d->have_key = true;
        d->policy = policy;
        return ra;
    }
    d = g_new0(ExDesc, 1);
    d->host = host;
    d->size = size;
    d->last_val = val;
    d->key = *key;
    d->desc_pa = desc_pa;
    d->have_key = true;
    d->policy = policy;
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
            .ram_addr = ram_addr, .size = size, .pc = ex_pc(env), .valid = true,
        };
    }
    qemu_mutex_unlock(&ex_lock);
}

/* ------------------------------------------------------------ contiguous */

/*
 * R_JQQTC bounds the choices when CONT is consistent, even while other
 * fields differ. I_PGVGZ excludes invalid descriptors, not the entire set.
 * Read all members and classify the observed set. A multi-load snapshot
 * cannot establish that a guest update has completed, nor prove missing
 * BBM by itself. Keep observations out of the VIOLATION count.
 */
#define EX_CONTIG_SAMPLE  64        /* re-walks between block re-checks */

static void ex_check_contig(CPUARMState *env, ARMMMUIdx mmu_idx,
                            const ExKey *key, uint64_t desc_pa,
                            int level, int lg_page_size)
{
    ExContigGeometry g = ex_contig_geometry(level, lg_page_size);
    ExContigSummary s;
    ARMVAParameters param;
    uint64_t base_pa, d[EX_CONTIG_MAX];
    unsigned i;

    ex_stat_contig_checked++;
    if (!g.entries || regime_is_stage2(mmu_idx) ||
        !arm_el_is_aa64(env, regime_el(mmu_idx)) ||
        key->space != ARMSS_NonSecure) {
        ex_stat_contig[EX_CONTIG_UNAVAILABLE]++;
        return;
    }
    param = aa64_va_parameters(env, key->va, mmu_idx, true, false);
    if (param.ds || param.ps > 5 || param.aie || param.pie ||
        (regime_sctlr(env, mmu_idx) & SCTLR_EE)) {
        ex_stat_contig[EX_CONTIG_UNAVAILABLE]++;
        return;
    }
    base_pa = desc_pa & ~(uint64_t)((g.entries * 8) - 1);

    for (i = 0; i < g.entries; i++) {
        MemTxResult res;

        d[i] = address_space_ldq(&address_space_memory, base_pa + i * 8,
                                 MEMTXATTRS_UNSPECIFIED, &res);
        if (res != MEMTX_OK) {
            ex_stat_contig[EX_CONTIG_UNAVAILABLE]++;
            return;
        }
    }
    s = ex_contig_analyze(d, g);
    ex_stat_contig[s.state]++;
    if (s.valid && s.invalid) {
        ex_stat_contig_partial++;
    }
    if ((s.state == EX_CONTIG_MIXED_HINT ||
         s.state == EX_CONTIG_OA_UNRESOLVED) &&
        !ex_site_seen(2, ex_pc(env), key->va)) {
        qemu_log_mask(LOG_EXACT,
            "exact-tlb: OBSERVATION contiguous snapshot rule=%s "
            "cpu=%d pc=0x%" PRIx64 " va=0x%" PRIx64 " regime=%s "
            "block_pa=0x%" PRIx64 " valid=%u invalid=%u; "
            "update completion and cached outcomes not observed\n",
            s.state == EX_CONTIG_MIXED_HINT ? "R_NGLXZ(mixed-hint)" :
                                            "R_JQQTC(oa-unresolved)",
            env_cpu(env)->cpu_index, ex_pc(env), key->va,
            ex_regime_name(key->regime), base_pa, s.valid, s.invalid);
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
               ? ex_asid_of(env, mmu_idx, 1ULL << 11, 3) : EX_ASID_GLOBAL;
    key.vmid = ex_vmid_of(env, mmu_idx);
    key.regime = ex_regime_of(mmu_idx);
    key.space = space;
    key.level = level;

    qemu_mutex_lock(&ex_lock);
    ex_resolve_locked(env_cpu(env));
    ra_desc = ex_desc_note(host, desc_val, &key, desc_pa,
                          ex_desc_size(env, mmu_idx), (ExLeafPolicy) {
                              .benign_mask = ex_benign_mask(env, mmu_idx,
                                                           level),
                          });
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
                if (!ex_site_seen(3, ex_pc(env), key.va)) {
                    qemu_log_mask(LOG_EXACT,
                        "exact-tlb: VIOLATION cached table descriptor changed "
                        "without a non-last-level invalidation cpu=%d "
                        "pc=0x%" PRIx64 " regime=%s va=0x%" PRIx64
                        " level=%d\n"
                        "exact-tlb:   descriptor at PA 0x%" PRIx64
                        " changed 0x%016" PRIx64 " -> 0x%016" PRIx64
                        ", next level table 0x%" PRIx64 " -> 0x%" PRIx64
                        ", last store to it from pc=0x%" PRIx64 "\n",
                        env_cpu(env)->cpu_index, ex_pc(env),
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
    ExLeafPolicy policy;

    if (!arm_exact_tlb_enabled || !ex_ncpus || !ex_tracked(mmu_idx)) {
        return;
    }

    memset(&key, 0, sizeof(key));
    key.va = va & ~((1ULL << lg_page_size) - 1);
    key.asid = ex_asid_of(env, mmu_idx, desc_val, level);
    key.vmid = ex_vmid_of(env, mmu_idx);
    key.regime = ex_regime_of(mmu_idx);
    key.space = space;
    key.level = level;
    policy = ex_leaf_policy(env, mmu_idx, va, level);

    qemu_mutex_lock(&ex_lock);
    ex_resolve_locked(env_cpu(env));
    ex_desc_note(host, desc_val, &key, desc_pa,
                 ex_desc_size(env, mmu_idx), policy);
    ex_tab = ex_tab_of(env_cpu(env)->cpu_index);
    if (!ex_tab) {
        qemu_mutex_unlock(&ex_lock);
        return;
    }
    e = g_hash_table_lookup(ex_tab, &key);
    if (e) {
        ex_stat_hits++;
        if (e->desc_pa == desc_pa && e->desc_val != desc_val) {
            if (!ex_leaf_change_benign(policy, e->desc_val, desc_val)) {
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
         * up to 128 extra guest loads, and a change through a member
         * we never walk would otherwise stay invisible.
         */
        if ((desc_val & EX_DESC_CONT) && e->contig_countdown-- == 0) {
            e->contig_countdown = EX_CONTIG_SAMPLE;
            ex_check_contig(env, mmu_idx, &key, desc_pa, level, lg_page_size);
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
        if (desc_val & EX_DESC_CONT) {
            ex_check_contig(env, mmu_idx, &key, desc_pa, level, lg_page_size);
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
static void ex_tlbi_apply(CPUARMState *env, ExInval *inv, bool broadcast);

void arm_exact_tlb_tlbi(CPUARMState *env, const struct ARMCPRegInfo *ri,
                        uint64_t value)
{
    ExInval inv = { .ttl_level = -1 };
    unsigned el = regime_el(arm_mmu_idx(env));
    bool broadcast;

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

    ex_tlbi_apply(env, &inv, broadcast);
}

/* Remove what @inv describes, from this PE or from all of them. */
static void ex_tlbi_apply(CPUARMState *env, ExInval *inv, bool broadcast)
{
    unsigned self = env_cpu(env)->cpu_index;
    unsigned i;

    ex_tlbi_by_kind[inv->all ? EX_K_ALL :
                    inv->match_range ? EX_K_RANGE :
                    inv->match_va ? EX_K_VA :
                    inv->match_asid ? EX_K_ASID : EX_K_REGIME]++;

    qemu_mutex_lock(&ex_lock);
    ex_resolve_locked(env_cpu(env));
    ex_tlbi_seq++;
    if (broadcast) {
        for (i = 0; i < ex_ncpus; i++) {
            if (ex_tab_cpu[i]) {
                ex_stat_removed += g_hash_table_foreach_remove(ex_tab_cpu[i],
                                                        ex_inval_cb, inv);
            }
        }
        ex_stat_broadcast++;
    } else if (ex_tab_cpu[self]) {
        ex_stat_removed += g_hash_table_foreach_remove(ex_tab_cpu[self],
                                                      ex_inval_cb, inv);
        ex_stat_local++;
    }
    qemu_mutex_unlock(&ex_lock);
}

/*
 * The AArch32 CP15 TLB maintenance operations (MCR p15, 0, Rt, c8, ...).
 * They are a different encoding of the same intent, and they are what a 32-bit
 * guest - or an AArch64 CPU running a 32-bit kernel at EL1 - issues. Leaving
 * them undecoded does not make the model conservative, it makes it wrong: every
 * invalidation the guest performs would go unseen and the next legitimate reuse
 * of a descriptor would be reported as a violation.
 *
 *   crm 3   inner shareable (broadcast)     crm 5  instruction TLB, local
 *   crm 7   unified, local                  crm 6  data TLB, local
 *   opc2 0  ALL      1 MVA (VA + ASID)      2 ASID    3 MVAA (VA, any ASID)
 *        5  MVAL (last level only)          7 MVAAL (last level, any ASID)
 *
 * The operand carries the VA in bits 31:12 and, for the ASID-matching forms,
 * an 8-bit ASID in bits 7:0.
 */
void arm_exact_tlb_tlbi32(CPUARMState *env, const struct ARMCPRegInfo *ri,
                          uint64_t value)
{
    ExInval inv = { .ttl_level = -1 };
    bool broadcast;

    if (!arm_exact_tlb_enabled || !ex_ncpus) {
        return;
    }

    /* Hyp-mode operations (opc1 == 4) act on the EL2 regime. */
    inv.regime = ri->opc1 == 4 ? 2 : 1;

    switch (ri->opc2) {
    case 0:                             /* TLBIALL: this regime, every ASID */
        inv.all = true;
        break;
    case 2:                             /* TLBIASID */
        inv.match_asid = true;
        inv.asid = extract64(value, 0, 8);
        break;
    case 1:                             /* TLBIMVA  */
    case 5:                             /* TLBIMVAL */
        inv.match_va = true;
        inv.match_asid = true;
        inv.asid = extract64(value, 0, 8);
        inv.va = value & ~(uint64_t)0xfff;
        inv.last_level_only = ri->opc2 == 5;
        break;
    case 3:                             /* TLBIMVAA  */
    case 7:                             /* TLBIMVAAL */
        inv.match_va = true;
        inv.match_globals = true;       /* any ASID, including global entries */
        inv.va = value & ~(uint64_t)0xfff;
        inv.last_level_only = ri->opc2 == 7;
        break;
    default:
        /* Anything we do not recognise must not leave stale entries behind. */
        inv.all = true;
        break;
    }

    /* A VA-matched operation also removes global (nG == 0) entries. */
    if (inv.match_va) {
        inv.match_globals = true;
    }

    broadcast = ri->crm == 3 || ri->crm == 1 ||
                (ri->opc1 == 0 && (arm_hcr_el2_eff(env) & HCR_FB));

    ex_tlbi_apply(env, &inv, broadcast);
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
                  ", break-before-make violations: %" PRIu64
                  ", benign break-remakes: %" PRIu64 "\n",
                  live, ex_stat_fills, ex_stat_hits,
                  ex_stat_benign, ex_stat_violations, ex_tlbi_seq,
                  ex_stat_removed,
                  ex_tlbi_by_kind[EX_K_ALL], ex_tlbi_by_kind[EX_K_REGIME],
                  ex_tlbi_by_kind[EX_K_ASID], ex_tlbi_by_kind[EX_K_VA],
                  ex_tlbi_by_kind[EX_K_RANGE], ex_stat_contig_checked,
                  ex_stat_broadcast, ex_stat_local, ex_stat_table_fills,
                  ex_stat_pt_pages, ex_stat_desc_stores, ex_stat_bbm,
                  ex_stat_bbm_accepted);
    qemu_log_mask(LOG_EXACT,
        "exact-tlb: contiguous contracts: uniform=%" PRIu64
        " bounded=%" PRIu64 " partial=%" PRIu64 " empty=%" PRIu64
        " unhinted=%" PRIu64 " mixed-hint=%" PRIu64
        " oa-unresolved=%" PRIu64 " unavailable=%" PRIu64 "\n",
        ex_stat_contig[EX_CONTIG_UNIFORM], ex_stat_contig[EX_CONTIG_BOUNDED],
        ex_stat_contig_partial, ex_stat_contig[EX_CONTIG_EMPTY],
        ex_stat_contig[EX_CONTIG_UNHINTED],
        ex_stat_contig[EX_CONTIG_MIXED_HINT],
        ex_stat_contig[EX_CONTIG_OA_UNRESOLVED],
        ex_stat_contig[EX_CONTIG_UNAVAILABLE]);
}
