/*
 *  SH4 emulation
 *
 *  Copyright (c) 2005 Samuel Tardieu
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"

#include "cpu.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/log.h"
#include "accel/tcg/cpu-loop.h"
#include "qemu/plugin.h"

#if !defined(CONFIG_USER_ONLY)
#include "hw/sh4/sh_intc.h"
#include "system/runstate.h"
#endif

#define MMU_OK                   0
#define MMU_ITLB_MISS            (-1)
#define MMU_ITLB_MULTIPLE        (-2)
#define MMU_ITLB_VIOLATION       (-3)
#define MMU_DTLB_MISS_READ       (-4)
#define MMU_DTLB_MISS_WRITE      (-5)
#define MMU_DTLB_INITIAL_WRITE   (-6)
#define MMU_DTLB_VIOLATION_READ  (-7)
#define MMU_DTLB_VIOLATION_WRITE (-8)
#define MMU_DTLB_MULTIPLE        (-9)
#define MMU_DTLB_MISS            (-10)
#define MMU_IADDR_ERROR          (-11)
#define MMU_DADDR_ERROR_READ     (-12)
#define MMU_DADDR_ERROR_WRITE    (-13)
#define MMU_PMB_MISS             (-14)

#if defined(CONFIG_USER_ONLY)

int cpu_sh4_is_cached(CPUSH4State *env, uint32_t addr)
{
    /* For user mode, only U0 area is cacheable. */
    return !(addr & 0x80000000);
}

#else /* !CONFIG_USER_ONLY */

void superh_cpu_do_interrupt(CPUState *cs)
{
    CPUSH4State *env = cpu_env(cs);
    int do_irq = cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
    int do_exp, irq_vector = cs->exception_index;
    uint64_t last_pc = env->pc;

    /* prioritize exceptions over interrupts */

    do_exp = cs->exception_index != -1;
    do_irq = do_irq && (cs->exception_index == -1);

    if (env->sr & (1u << SR_BL)) {
        if (do_exp && cs->exception_index != 0x1e0) {
            /* In theory a masked exception generates a reset exception,
               which in turn jumps to the reset vector. However this only
               works when using a bootloader. When using a kernel and an
               initrd, they need to be reloaded and the program counter
               should be loaded with the kernel entry point.
               qemu_system_reset_request takes care of that.  */
            qemu_log_mask(LOG_GUEST_ERROR, "sh4: exception 0x%03x at pc "
                          "0x%08x with SR.BL set, resetting (tea 0x%08x)\n",
                          cs->exception_index, env->pc, env->tea);
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            return;
        }
        if (do_irq && !env->in_sleep) {
            return; /* masked */
        }
    }
    env->in_sleep = 0;

    if (do_irq) {
        irq_vector = env->intc_get_vector ?
            env->intc_get_vector(env->intc_handle, (env->sr >> 4) & 0xf) :
            sh_intc_get_pending_vector(env->intc_handle,
                                       (env->sr >> 4) & 0xf);
        if (irq_vector == -1) {
            return; /* masked */
        }
    }

    if (qemu_loglevel_mask(CPU_LOG_INT)) {
        const char *expname;
        switch (cs->exception_index) {
        case 0x0e0:
            expname = "addr_error";
            break;
        case 0x040:
            expname = "tlb_miss";
            break;
        case 0x0a0:
            expname = "tlb_violation";
            break;
        case 0x180:
            expname = "illegal_instruction";
            break;
        case 0x1a0:
            expname = "slot_illegal_instruction";
            break;
        case 0x800:
            expname = "fpu_disable";
            break;
        case 0x820:
            expname = "slot_fpu";
            break;
        case 0x100:
            expname = "data_write";
            break;
        case 0x060:
            expname = "dtlb_miss_write";
            break;
        case 0x0c0:
            expname = "dtlb_violation_write";
            break;
        case 0x120:
            expname = "fpu_exception";
            break;
        case 0x080:
            expname = "initial_page_write";
            break;
        case 0x160:
            expname = "trapa";
            break;
        default:
            expname = do_irq ? "interrupt" : "???";
            break;
        }
        qemu_log("exception 0x%03x [%s] raised\n",
                  irq_vector, expname);
        log_cpu_state(cs, 0);
    }

    env->ssr = cpu_read_sr(env);
    env->spc = env->pc;
    env->sgr = env->gregs[15];
    env->sr |= (1u << SR_BL) | (1u << SR_MD) | (1u << SR_RB);
    env->lock_addr = -1;

    if (env->flags & TB_FLAG_DELAY_SLOT_MASK) {
        /* Branch instruction should be executed again before delay slot. */
        env->spc -= 2;
        /* Clear flags for exception/interrupt routine. */
        env->flags &= ~TB_FLAG_DELAY_SLOT_MASK;
    }

    if (do_exp) {
        env->expevt = cs->exception_index;
        switch (cs->exception_index) {
        case 0x000:
        case 0x020:
        case 0x140:
            env->sr &= ~(1u << SR_FD);
            env->sr |= 0xf << 4; /* IMASK */
            env->pc = 0xa0000000;
            break;
        case 0x040:
        case 0x060:
            env->pc = env->vbr + 0x400;
            break;
        case 0x160:
            env->spc += 2; /* special case for TRAPA */
            /* fall through */
        default:
            env->pc = env->vbr + 0x100;
            break;
        }
        qemu_plugin_vcpu_exception_cb(cs, last_pc);
        return;
    }

    if (do_irq) {
        env->intevt = irq_vector;
        env->pc = env->vbr + 0x600;
        qemu_plugin_vcpu_interrupt_cb(cs, last_pc);
        return;
    }
}

static void update_itlb_use(CPUSH4State * env, int itlbnb)
{
    uint32_t or_mask = 0, and_mask = 0xff;

    switch (itlbnb) {
    case 0:
        and_mask = 0x1f;
        break;
    case 1:
        and_mask = 0xe7;
        or_mask = 0x80;
        break;
    case 2:
        and_mask = 0xfb;
        or_mask = 0x50;
        break;
    case 3:
        or_mask = 0x2c;
        break;
    }

    env->mmucr &= (and_mask << 24) | 0x00ffffff;
    env->mmucr |= (or_mask << 24);
}

static int itlb_replacement(CPUSH4State * env)
{
    if ((env->mmucr & 0xe0000000) == 0xe0000000) {
        return 0;
    }
    if ((env->mmucr & 0x98000000) == 0x18000000) {
        return 1;
    }
    if ((env->mmucr & 0x54000000) == 0x04000000) {
        return 2;
    }
    if ((env->mmucr & 0x2c000000) == 0x00000000) {
        return 3;
    }
    cpu_abort(env_cpu(env), "Unhandled itlb_replacement");
}

/* MMU indexes used by this target: 0 (privileged) and MMU_USER_IDX. */
#define SH4_MMUIDX_ALL ((1 << 0) | (1 << MMU_USER_IDX))

/*
 * First virtual address covered by a TLB entry; VPN bits below the page
 * size are ignored by the hardware compare.
 */
static vaddr tlb_entry_start(const tlb_t *entry)
{
    return (entry->vpn << 10) & ~(vaddr)(entry->size - 1);
}

/*
 * Drop every softmmu mapping that a (valid) SH TLB entry may have created.
 * Entries can be up to 1 MiB, so flushing only the first page would leave
 * stale translations for the rest of the entry.
 */
static void flush_tlb_entry(CPUSH4State *env, const tlb_t *entry)
{
    vaddr start = tlb_entry_start(entry) & TARGET_PAGE_MASK;
    vaddr len = MAX(entry->size, TARGET_PAGE_SIZE);

    tlb_flush_range_by_mmuidx(env_cpu(env), start, len, SH4_MMUIDX_ALL,
                              TARGET_LONG_BITS);
}

/* Does a valid entry cover the given virtual address (usual compare rules)? */
static bool tlb_entry_covers(const tlb_t *entry, vaddr address, int use_asid,
                             uint8_t asid)
{
    vaddr start;

    if (!entry->v) {
        return false;
    }
    if (!entry->sh && use_asid && entry->asid != asid) {
        return false;
    }
    start = tlb_entry_start(entry);
    return address >= start && address <= start + entry->size - 1;
}

/*
 * SH-4A TLB extended mode (MMUCR.ME = 1): page size from ESZ[3:0] and
 * access rights from EPR[5:0] (SH-4A Extended Functions Software Manual
 * R01US0060EJ0200, 7.4). SH-4 has no ME bit.
 */
static bool tlb_extended_mode(CPUSH4State *env)
{
    return (env->features & SH_FEATURE_SH4A) && (env->mmucr & MMUCR_ME);
}

/* SH-4A 32-bit address extended mode (PASCR.SE, software manual 7.8) */
static bool addr32_mode(CPUSH4State *env)
{
    return (env->features & SH_FEATURE_SH4A) && (env->pascr & PASCR_SE);
}

/* PPN from PTEL or a data array: bits 28:10, or 31:10 in 32-bit mode */
static uint32_t ptel_ppn(CPUSH4State *env, uint32_t value)
{
    return (value & (addr32_mode(env) ? 0xfffffc00 : 0x1ffffc00)) >> 10;
}

static uint32_t pmb_size(const pmb_t *e)
{
    static const uint32_t sizes[4] = { 16 * MiB, 64 * MiB, 128 * MiB,
                                       512 * MiB };
    return sizes[e->sz];
}

static void pmb_flush_entry(CPUSH4State *env, const pmb_t *e)
{
    uint32_t size = pmb_size(e);

    tlb_flush_range_by_mmuidx(env_cpu(env),
                              ((uint32_t)e->vpn << 24) & ~(size - 1),
                              size, SH4_MMUIDX_ALL, TARGET_LONG_BITS);
}

/*
 * Translate a P1/P2 address through the PMB (7.8.4). Multiple hits are
 * "not guaranteed"; the first match is used and the case is logged.
 */
static int pmb_translate(CPUSH4State *env, vaddr address, hwaddr *physical,
                         uint32_t *page_size)
{
    const pmb_t *match = NULL;
    int i;

    for (i = 0; i < PMB_SIZE; i++) {
        const pmb_t *e = &env->pmb[i];
        uint32_t size = pmb_size(e);

        if (e->v &&
            ((address ^ ((uint32_t)e->vpn << 24)) & ~(vaddr)(size - 1)) == 0) {
            if (match) {
                qemu_log_mask(LOG_GUEST_ERROR, "sh4: multiple PMB hits for "
                              "0x%" VADDR_PRIx " (not guaranteed)\n", address);
                break;
            }
            match = e;
        }
    }
    if (!match) {
        return MMU_PMB_MISS;
    }
    *page_size = pmb_size(match);
    *physical = (((hwaddr)match->ppn << 24) & ~(hwaddr)(*page_size - 1)) |
                (address & (*page_size - 1));
    return MMU_OK;
}

/* ESZ page size codes (7.4.1); 0 for codes the manual does not define. */
static uint32_t esz_to_size(uint8_t esz)
{
    switch (esz) {
    case 0x0:
        return 1 * KiB;
    case 0x1:
        return 4 * KiB;
    case 0x2:
        return 8 * KiB;
    case 0x4:
        return 64 * KiB;
    case 0x5:
        return 256 * KiB;
    case 0x7:
        return 1 * MiB;
    case 0x8:
        return 4 * MiB;
    case 0xc:
        return 64 * MiB;
    default:
        return 0;
    }
}

/* Set esz and size of an extended-mode entry. */
static void tlb_entry_set_esz(CPUSH4State *env, tlb_t *entry, uint8_t esz)
{
    entry->esz = esz;
    entry->size = esz_to_size(esz);
    if (!entry->size) {
        qemu_log_mask(LOG_GUEST_ERROR, "sh4: TLB entry with undefined ESZ "
                      "0x%x (operation not guaranteed), using 4 KiB\n", esz);
        entry->size = 4 * KiB;
    }
}

/* Set sz and size of a compatible-mode entry: 1K, 4K, 64K or 1M. */
static void tlb_entry_set_sz(tlb_t *entry, uint8_t sz)
{
    static const uint32_t sizes[4] = { 1 * KiB, 4 * KiB, 64 * KiB, 1 * MiB };

    entry->sz = sz & 3;
    entry->size = sizes[entry->sz];
}

/* EPR bit numbers (7.4.1) */
#define EPR_PRIV_READ   5
#define EPR_PRIV_WRITE  4
#define EPR_PRIV_EXEC   3
#define EPR_USER_READ   2
#define EPR_USER_WRITE  1
#define EPR_USER_EXEC   0

/*
 * Does the entry allow this access? In compatible mode PR[1] grants user
 * access and PR[0] grants writes; in extended mode each of privileged and
 * user read, write and execute has its own EPR bit.
 */
static bool tlb_entry_allows(CPUSH4State *env, const tlb_t *entry,
                             MMUAccessType access_type, bool user)
{
    if (tlb_extended_mode(env)) {
        int bit;

        switch (access_type) {
        case MMU_INST_FETCH:
            bit = user ? EPR_USER_EXEC : EPR_PRIV_EXEC;
            break;
        case MMU_DATA_STORE:
            bit = user ? EPR_USER_WRITE : EPR_PRIV_WRITE;
            break;
        default:
            bit = user ? EPR_USER_READ : EPR_PRIV_READ;
            break;
        }
        return entry->epr & (1 << bit);
    }
    if (user && !(entry->pr & 2)) {
        return false;
    }
    return access_type != MMU_DATA_STORE || (entry->pr & 1);
}

/* Find the corresponding entry in the right TLB
   Return entry, MMU_DTLB_MISS or MMU_DTLB_MULTIPLE
*/
static int find_tlb_entry(CPUSH4State *env, vaddr address,
                          tlb_t * entries, uint8_t nbtlb, int use_asid)
{
    int match = MMU_DTLB_MISS;
    uint8_t asid;
    int i;

    asid = env->pteh & 0xff;

    for (i = 0; i < nbtlb; i++) {
        if (tlb_entry_covers(&entries[i], address, use_asid, asid)) {
            if (match != MMU_DTLB_MISS)
                return MMU_DTLB_MULTIPLE; /* Multiple match */
            match = i;
        }
    }
    return match;
}

static void increment_urc(CPUSH4State * env)
{
    uint8_t urb, urc;

    /* Increment URC */
    urb = ((env->mmucr) >> 18) & 0x3f;
    urc = ((env->mmucr) >> 10) & 0x3f;
    urc++;
    if ((urb > 0 && urc > urb) || urc > (UTLB_SIZE - 1))
        urc = 0;
    env->mmucr = (env->mmucr & 0xffff03ff) | (urc << 10);
}

/* Copy and utlb entry into itlb
   Return entry
*/
static int copy_utlb_entry_itlb(CPUSH4State *env, int utlb)
{
    int itlb;

    tlb_t * ientry;
    itlb = itlb_replacement(env);
    ientry = &env->itlb[itlb];
    if (ientry->v) {
        flush_tlb_entry(env, ientry);
    }
    *ientry = env->utlb[utlb];
    update_itlb_use(env, itlb);
    return itlb;
}

/* Find itlb entry
   Return entry, MMU_ITLB_MISS, MMU_ITLB_MULTIPLE or MMU_DTLB_MULTIPLE
*/
static int find_itlb_entry(CPUSH4State *env, vaddr address,
                           int use_asid)
{
    int e;

    e = find_tlb_entry(env, address, env->itlb, ITLB_SIZE, use_asid);
    if (e == MMU_DTLB_MULTIPLE) {
        e = MMU_ITLB_MULTIPLE;
    } else if (e == MMU_DTLB_MISS) {
        e = MMU_ITLB_MISS;
    } else if (e >= 0) {
        update_itlb_use(env, e);
    }
    return e;
}

/* Find utlb entry
   Return entry, MMU_DTLB_MISS, MMU_DTLB_MULTIPLE */
static int find_utlb_entry(CPUSH4State *env, vaddr address, int use_asid)
{
    /* per utlb access */
    increment_urc(env);

    /* Return entry */
    return find_tlb_entry(env, address, env->utlb, UTLB_SIZE, use_asid);
}

/* Match address against MMU
   Return MMU_OK, MMU_DTLB_MISS_READ, MMU_DTLB_MISS_WRITE,
   MMU_DTLB_INITIAL_WRITE, MMU_DTLB_VIOLATION_READ,
   MMU_DTLB_VIOLATION_WRITE, MMU_ITLB_MISS,
   MMU_ITLB_MULTIPLE, MMU_ITLB_VIOLATION,
   MMU_IADDR_ERROR, MMU_DADDR_ERROR_READ, MMU_DADDR_ERROR_WRITE.
*/
static int get_mmu_address(CPUSH4State *env, hwaddr *physical,
                           int *prot, uint32_t *page_size, vaddr address,
                           MMUAccessType access_type)
{
    int use_asid, n;
    tlb_t *matching = NULL;
    bool user = !(env->sr & (1u << SR_MD));

    use_asid = !(env->mmucr & MMUCR_SV) || user;

    if (access_type == MMU_INST_FETCH) {
        n = find_itlb_entry(env, address, use_asid);
        if (n >= 0) {
            matching = &env->itlb[n];
            if (!tlb_entry_allows(env, matching, MMU_INST_FETCH, user)) {
                n = MMU_ITLB_VIOLATION;
            } else {
                *prot = PAGE_EXEC;
            }
        } else {
            n = find_utlb_entry(env, address, use_asid);
            if (n >= 0) {
                n = copy_utlb_entry_itlb(env, n);
                matching = &env->itlb[n];
                if (!tlb_entry_allows(env, matching, MMU_INST_FETCH, user)) {
                    n = MMU_ITLB_VIOLATION;
                } else {
                    *prot = PAGE_EXEC;
                    if (tlb_entry_allows(env, matching, MMU_DATA_LOAD, user)) {
                        *prot |= PAGE_READ;
                    }
                    if (tlb_entry_allows(env, matching, MMU_DATA_STORE, user)
                        && matching->d) {
                        *prot |= PAGE_WRITE;
                    }
                }
            } else if (n == MMU_DTLB_MULTIPLE) {
                n = MMU_ITLB_MULTIPLE;
            } else if (n == MMU_DTLB_MISS) {
                n = MMU_ITLB_MISS;
            }
        }
    } else {
        n = find_utlb_entry(env, address, use_asid);
        if (n >= 0) {
            matching = &env->utlb[n];
            if (!tlb_entry_allows(env, matching, access_type, user)) {
                n = (access_type == MMU_DATA_STORE)
                    ? MMU_DTLB_VIOLATION_WRITE : MMU_DTLB_VIOLATION_READ;
            } else if ((access_type == MMU_DATA_STORE) && !matching->d) {
                n = MMU_DTLB_INITIAL_WRITE;
            } else {
                *prot = 0;
                if (tlb_entry_allows(env, matching, MMU_DATA_LOAD, user)) {
                    *prot |= PAGE_READ;
                }
                if (tlb_entry_allows(env, matching, MMU_DATA_STORE, user)
                    && matching->d) {
                    *prot |= PAGE_WRITE;
                }
            }
        } else if (n == MMU_DTLB_MISS) {
            n = (access_type == MMU_DATA_STORE)
                ? MMU_DTLB_MISS_WRITE : MMU_DTLB_MISS_READ;
        }
    }
    if (n >= 0) {
        n = MMU_OK;
        *physical = ((matching->ppn << 10) & ~(matching->size - 1))
                    | (address & (matching->size - 1));
        *page_size = matching->size;
    }
    return n;
}

static int get_physical_address(CPUSH4State *env, hwaddr* physical,
                                int *prot, uint32_t *page_size, vaddr address,
                                MMUAccessType access_type)
{
    *page_size = TARGET_PAGE_SIZE;

    /* P1, P2 and P4 areas do not use translation */
    if ((address >= 0x80000000 && address < 0xc0000000) || address >= 0xe0000000) {
        if (!(env->sr & (1u << SR_MD))
                && (address < 0xe0000000 || address >= 0xe4000000)) {
            /* Unauthorized access in user mode (only store queues are available) */
            qemu_log_mask(LOG_GUEST_ERROR, "Unauthorized access\n");
            if (access_type == MMU_DATA_LOAD) {
                return MMU_DADDR_ERROR_READ;
            } else if (access_type == MMU_DATA_STORE) {
                return MMU_DADDR_ERROR_WRITE;
            } else {
                return MMU_IADDR_ERROR;
            }
        }
        if (address >= 0x80000000 && address < 0xc0000000) {
            if (addr32_mode(env)) {
                /* P1 and P2 go through the PMB in 32-bit mode */
                int ret = pmb_translate(env, address, physical, page_size);

                if (ret != MMU_OK) {
                    return ret;
                }
                *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
                return MMU_OK;
            }
            /* Mask upper 3 bits for P1 and P2 areas */
            *physical = address & 0x1fffffff;
        } else {
            *physical = address;
        }
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        return MMU_OK;
    }

    /* If MMU is disabled, return the corresponding physical page */
    if (!(env->mmucr & MMUCR_AT)) {
        /* U0/P0/P3 are the 32-bit physical address in 32-bit mode */
        *physical = addr32_mode(env) ? address : address & 0x1FFFFFFF;
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        return MMU_OK;
    }

    /* We need to resort to the MMU */
    return get_mmu_address(env, physical, prot, page_size, address,
                           access_type);
}

/*
 * exact-dcache: how the operand cache treats a data access to @address,
 * from the TLB and PMB as they are now and without touching URC. 0 is
 * uncached, 1 write-through, 2 copy-back. A TLB miss counts as uncached.
 */
int sh4_data_cache_mode(CPUSH4State *env, vaddr address, uint32_t ccr)
{
    const int oce = 1 << 0, wt = 1 << 1, cb = 1 << 2;
    int i, use_asid;

    if (!(ccr & oce) || address >= 0xe0000000) {
        return 0;
    }
    if (address >= 0x80000000 && address < 0xc0000000) {
        if (addr32_mode(env)) {
            for (i = 0; i < PMB_SIZE; i++) {
                const pmb_t *e = &env->pmb[i];

                if (e->v && ((address ^ ((uint32_t)e->vpn << 24)) &
                             ~(vaddr)(pmb_size(e) - 1)) == 0) {
                    return !e->c ? 0 : e->wt ? 1 : 2;
                }
            }
            return 0;
        }
        if (address >= 0xa0000000) {
            return 0;                   /* P2 */
        }
        return ccr & cb ? 2 : 1;        /* P1 */
    }
    if (!(env->mmucr & MMUCR_AT)) {
        return ccr & wt ? 1 : 2;
    }
    use_asid = !(env->mmucr & MMUCR_SV) || !(env->sr & (1u << SR_MD));
    i = find_tlb_entry(env, address, env->utlb, UTLB_SIZE, use_asid);
    if (i < 0) {
        return 0;
    }
    return !env->utlb[i].c ? 0 : env->utlb[i].wt ? 1 : 2;
}

hwaddr superh_cpu_get_phys_addr_debug(CPUState *cs, vaddr addr)
{
    hwaddr physical;
    int prot;
    uint32_t page_size;

    if (get_physical_address(cpu_env(cs), &physical, &prot, &page_size, addr,
                             MMU_DATA_LOAD) == MMU_OK) {
        return physical;
    }

    return -1;
}

/*
 * exact-tlb (-d exact): LDTLB operands that are legal but almost certainly
 * not what the software meant. In TLB extended mode PTEL.SZ1/SZ0/PR1/PR0
 * are ignored (7.2.2 note), so a page size written there instead of in
 * PTEA.ESZ silently gives a different page. Each pattern is reported once.
 */
static void exact_ldtlb_check(CPUSH4State *env, const tlb_t *e)
{
    static GHashTable *seen;
    static const uint32_t compat[4] = { 1 * KiB, 4 * KiB, 64 * KiB, 1 * MiB };
    uint32_t sz = cpu_ptel_sz(env->ptel);
    uint32_t vaddr = e->vpn << 10, paddr = e->ppn << 10;
    const char *what = NULL;
    uint64_t key;

    if (!e->v) {
        return;
    }
    if (tlb_extended_mode(env) && (env->ptel & 0xf0) &&
        compat[sz] != e->size) {
        what = "PTEL.SZ/PR set in TLB extended mode, where they are ignored;"
               " the page size comes from PTEA.ESZ";
    } else if (tlb_extended_mode(env) && !esz_to_size((env->ptea >> 4) & 0xf)) {
        what = "undefined PTEA.ESZ code";
    } else if ((vaddr | paddr) & (e->size - 1) & ~0xfffu) {
        /*
         * Bits below the page size are ignored ("with a 4-Kbyte page, PPN
         * bits [28:12] are valid"), so a larger page maps an aligned
         * block other than the one meant. Linux keeps software flags in
         * PPN bits 10 and 11, hence the 4 KiB floor.
         */
        what = "VPN or PPN not aligned to the page size";
    } else if (e->size == 1 * KiB) {
        what = "1 KiB page";
    }
    if (!what) {
        return;
    }
    if (!seen) {
        seen = g_hash_table_new_full(g_int64_hash, g_int64_equal, g_free,
                                     NULL);
    }
    key = ((uint64_t)(uintptr_t)what << 16) ^ ((env->ptel & 0x1ff) << 8) ^
          ((env->ptea >> 4) & 0xff) ^ ((uint64_t)e->size << 32);
    if (g_hash_table_contains(seen, &key)) {
        return;
    }
    g_hash_table_add(seen, g_memdup2(&key, sizeof(key)));
    qemu_log_mask(LOG_EXACT, "exact-tlb: %s\n"
                  "  ldtlb at pc 0x%08x (pr 0x%08x): PTEH 0x%08x PTEL 0x%08x"
                  " PTEA 0x%08x -> %u KiB page va 0x%08x pa 0x%08x\n",
                  what, env->pc, env->pr, env->pteh, env->ptel, env->ptea,
                  (unsigned)(e->size / KiB), vaddr, paddr);
}

void cpu_load_tlb(CPUSH4State * env)
{
    int n = cpu_mmucr_urc(env->mmucr);
    tlb_t * entry = &env->utlb[n];

    if (entry->v) {
        /* Overwriting valid entry in utlb. */
        flush_tlb_entry(env, entry);
    }

    /* Take values into cpu status from registers. */
    entry->asid = (uint8_t)cpu_pteh_asid(env->pteh);
    entry->vpn  = cpu_pteh_vpn(env->pteh);
    entry->v    = (uint8_t)cpu_ptel_v(env->ptel);
    entry->ppn  = ptel_ppn(env, env->ptel);
    if (tlb_extended_mode(env)) {
        /* 7.5.3: PTEA supplies EPR[13:8] and ESZ[7:4]; PTEL SZ/PR unused */
        entry->sz = 0;
        entry->pr = 0;
        entry->epr = (env->ptea >> 8) & 0x3f;
        tlb_entry_set_esz(env, entry, (env->ptea >> 4) & 0xf);
    } else {
        tlb_entry_set_sz(entry, cpu_ptel_sz(env->ptel));
        entry->pr = (uint8_t)cpu_ptel_pr(env->ptel);
    }
    entry->sh   = (uint8_t)cpu_ptel_sh(env->ptel);
    entry->c    = (uint8_t)cpu_ptel_c(env->ptel);
    entry->d    = (uint8_t)cpu_ptel_d(env->ptel);
    entry->wt   = (uint8_t)cpu_ptel_wt(env->ptel);
    if (!(env->features & SH_FEATURE_SH4A)) {
        entry->sa   = (uint8_t)cpu_ptea_sa(env->ptea);
        entry->tc   = (uint8_t)cpu_ptea_tc(env->ptea);
    }
    if (qemu_loglevel_mask(LOG_EXACT)) {
        exact_ldtlb_check(env, entry);
    }
}

 void cpu_sh4_invalidate_tlb(CPUSH4State *s)
{
    int i;

    /* UTLB */
    for (i = 0; i < UTLB_SIZE; i++) {
        tlb_t * entry = &s->utlb[i];
        entry->v = 0;
    }
    /* ITLB */
    for (i = 0; i < ITLB_SIZE; i++) {
        tlb_t * entry = &s->itlb[i];
        entry->v = 0;
    }

    tlb_flush(env_cpu(s));
}

uint32_t cpu_sh4_read_mmaped_itlb_addr(CPUSH4State *s,
                                       hwaddr addr)
{
    int index = (addr & 0x00000300) >> 8;
    tlb_t * entry = &s->itlb[index];

    return (entry->vpn  << 10) |
           (entry->v    <<  8) |
           (entry->asid);
}

void cpu_sh4_write_mmaped_itlb_addr(CPUSH4State *s, hwaddr addr,
                                    uint32_t mem_value)
{
    uint32_t vpn = (mem_value & 0xfffffc00) >> 10;
    uint8_t v = (uint8_t)((mem_value & 0x00000100) >> 8);
    uint8_t asid = (uint8_t)(mem_value & 0x000000ff);

    int index = (addr & 0x00000300) >> 8;
    tlb_t * entry = &s->itlb[index];
    if (entry->v) {
        /* Overwriting valid entry in itlb. */
        flush_tlb_entry(s, entry);
    }
    entry->asid = asid;
    entry->vpn = vpn;
    entry->v = v;
}

uint32_t cpu_sh4_read_mmaped_itlb_data(CPUSH4State *s,
                                       hwaddr addr)
{
    int array = (addr & 0x00800000) >> 23;
    int index = (addr & 0x00000300) >> 8;
    tlb_t * entry = &s->itlb[index];

    if (array == 0) {
        /* ITLB Data Array 1; PR and SZ are reserved in extended mode */
        uint32_t ret = (entry->ppn << 10) |
                       (entry->v   <<  8) |
                       (entry->c   <<  3) |
                       (entry->sh  <<  1);

        if (!tlb_extended_mode(s)) {
            ret |= (entry->pr  <<  5) |
                   ((entry->sz & 1) <<  6) |
                   ((entry->sz & 2) <<  4);
        }
        return ret;
    } else if (s->features & SH_FEATURE_SH4A) {
        /* SH-4A ITLB Data Array 2 (7.7.3): EPR[5,3,2,0] and ESZ */
        if (!tlb_extended_mode(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "sh4: ITLB data array 2 read in "
                          "TLB compatible mode\n");
            return 0;
        }
        return (entry->epr & 0x2d) << 8 | entry->esz << 4;
    } else {
        /* ITLB Data Array 2 */
        return (entry->tc << 1) |
               (entry->sa);
    }
}

void cpu_sh4_write_mmaped_itlb_data(CPUSH4State *s, hwaddr addr,
                                    uint32_t mem_value)
{
    int array = (addr & 0x00800000) >> 23;
    int index = (addr & 0x00000300) >> 8;
    tlb_t * entry = &s->itlb[index];

    if (array == 0) {
        /* ITLB Data Array 1 */
        if (entry->v) {
            /* Overwriting valid entry in itlb. */
            flush_tlb_entry(s, entry);
        }
        entry->ppn = ptel_ppn(s, mem_value);
        entry->v   = (mem_value & 0x00000100) >> 8;
        if (!tlb_extended_mode(s)) {
            tlb_entry_set_sz(entry, (mem_value & 0x00000080) >> 6 |
                                    (mem_value & 0x00000010) >> 4);
            entry->pr  = (mem_value & 0x00000040) >> 5;
        }
        entry->c   = (mem_value & 0x00000008) >> 3;
        entry->sh  = (mem_value & 0x00000002) >> 1;
    } else if (s->features & SH_FEATURE_SH4A) {
        /* SH-4A ITLB Data Array 2 (7.7.3): EPR[5,3,2,0] and ESZ */
        if (!tlb_extended_mode(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "sh4: ITLB data array 2 write in "
                          "TLB compatible mode\n");
            return;
        }
        if (entry->v) {
            flush_tlb_entry(s, entry);
        }
        entry->epr = (mem_value >> 8) & 0x2d;
        tlb_entry_set_esz(s, entry, (mem_value >> 4) & 0xf);
    } else {
        /* ITLB Data Array 2 */
        entry->tc  = (mem_value & 0x00000008) >> 3;
        entry->sa  = (mem_value & 0x00000007);
    }
}

uint32_t cpu_sh4_read_mmaped_utlb_addr(CPUSH4State *s,
                                       hwaddr addr)
{
    int index = (addr & 0x00003f00) >> 8;
    tlb_t * entry = &s->utlb[index];

    increment_urc(s); /* per utlb access */

    return (entry->vpn  << 10) |
           (entry->d    <<  9) |
           (entry->v    <<  8) |
           (entry->asid);
}

void cpu_sh4_write_mmaped_utlb_addr(CPUSH4State *s, hwaddr addr,
                                    uint32_t mem_value)
{
    int associate = addr & 0x0000080;
    uint32_t vpn = (mem_value & 0xfffffc00) >> 10;
    uint8_t d = (uint8_t)((mem_value & 0x00000200) >> 9);
    uint8_t v = (uint8_t)((mem_value & 0x00000100) >> 8);
    uint8_t asid = (uint8_t)(mem_value & 0x000000ff);
    int use_asid = !(s->mmucr & MMUCR_SV) || !(s->sr & (1u << SR_MD));

    if (associate) {
        /* The compare uses PTEH.ASID, not the ASID in the data field. */
        uint8_t pteh_asid = s->pteh & PTEH_ASID_MASK;
        int i;
        tlb_t * utlb_match_entry = NULL;

        /* search UTLB, with the usual address comparison rules */
        for (i = 0; i < UTLB_SIZE; i++) {
            tlb_t * entry = &s->utlb[i];
            if (!entry->v)
                continue;

            if (tlb_entry_covers(entry, vpn << 10, use_asid, pteh_asid)) {
                if (utlb_match_entry) {
                    CPUState *cs = env_cpu(s);

                    /*
                     * SH-4 raises a data TLB multiple hit exception;
                     * SH-4A does not (SH-4A software manual, list of
                     * changes for 7.7.4). Which entries SH-4A then
                     * writes is not documented; the first match is.
                     */
                    if (!(s->features & SH_FEATURE_SH4A)) {
                        cs->exception_index = 0x140;
                        s->tea = addr;
                    }
                    break;
                }
                flush_tlb_entry(s, entry);
                entry->v = v;
                entry->d = d;
                utlb_match_entry = entry;
            }
            increment_urc(s); /* per utlb access */
        }

        /* search ITLB */
        for (i = 0; i < ITLB_SIZE; i++) {
            tlb_t * entry = &s->itlb[i];
            if (tlb_entry_covers(entry, vpn << 10, use_asid, pteh_asid)) {
                flush_tlb_entry(s, entry);
                if (utlb_match_entry)
                    *entry = *utlb_match_entry;
                else
                    entry->v = v;
                break;
            }
        }
    } else {
        int index = (addr & 0x00003f00) >> 8;
        tlb_t * entry = &s->utlb[index];
        if (entry->v) {
            /* Overwriting valid entry in utlb. */
            flush_tlb_entry(s, entry);
        }
        entry->asid = asid;
        entry->vpn = vpn;
        entry->d = d;
        entry->v = v;
        increment_urc(s);
    }
}

uint32_t cpu_sh4_read_mmaped_utlb_data(CPUSH4State *s,
                                       hwaddr addr)
{
    int array = (addr & 0x00800000) >> 23;
    int index = (addr & 0x00003f00) >> 8;
    tlb_t * entry = &s->utlb[index];

    increment_urc(s); /* per utlb access */

    if (array == 0) {
        /* UTLB Data Array 1; PR and SZ are reserved in extended mode */
        uint32_t ret = (entry->ppn << 10) |
                       (entry->v   <<  8) |
                       (entry->c   <<  3) |
                       (entry->d   <<  2) |
                       (entry->sh  <<  1) |
                       (entry->wt);

        if (!tlb_extended_mode(s)) {
            ret |= (entry->pr  <<  5) |
                   ((entry->sz & 1) <<  6) |
                   ((entry->sz & 2) <<  4);
        }
        return ret;
    } else if (s->features & SH_FEATURE_SH4A) {
        /* SH-4A UTLB Data Array 2 (7.7.6): EPR and ESZ */
        if (!tlb_extended_mode(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "sh4: UTLB data array 2 read in "
                          "TLB compatible mode\n");
            return 0;
        }
        return entry->epr << 8 | entry->esz << 4;
    } else {
        /* UTLB Data Array 2 */
        return (entry->tc << 1) |
               (entry->sa);
    }
}

void cpu_sh4_write_mmaped_utlb_data(CPUSH4State *s, hwaddr addr,
                                    uint32_t mem_value)
{
    int array = (addr & 0x00800000) >> 23;
    int index = (addr & 0x00003f00) >> 8;
    tlb_t * entry = &s->utlb[index];

    increment_urc(s); /* per utlb access */

    if (array == 0) {
        /* UTLB Data Array 1 */
        if (entry->v) {
            /* Overwriting valid entry in utlb. */
            flush_tlb_entry(s, entry);
        }
        entry->ppn = ptel_ppn(s, mem_value);
        entry->v   = (mem_value & 0x00000100) >> 8;
        if (!tlb_extended_mode(s)) {
            tlb_entry_set_sz(entry, (mem_value & 0x00000080) >> 6 |
                                    (mem_value & 0x00000010) >> 4);
            entry->pr  = (mem_value & 0x00000060) >> 5;
        }
        entry->c   = (mem_value & 0x00000008) >> 3;
        entry->d   = (mem_value & 0x00000004) >> 2;
        entry->sh  = (mem_value & 0x00000002) >> 1;
        entry->wt  = (mem_value & 0x00000001);
    } else if (s->features & SH_FEATURE_SH4A) {
        /* SH-4A UTLB Data Array 2 (7.7.6): EPR and ESZ */
        if (!tlb_extended_mode(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "sh4: UTLB data array 2 write in "
                          "TLB compatible mode\n");
            return;
        }
        if (entry->v) {
            flush_tlb_entry(s, entry);
        }
        entry->epr = (mem_value >> 8) & 0x3f;
        tlb_entry_set_esz(s, entry, (mem_value >> 4) & 0xf);
    } else {
        /* UTLB Data Array 2 */
        entry->tc = (mem_value & 0x00000008) >> 3;
        entry->sa = (mem_value & 0x00000007);
    }
}

int cpu_sh4_is_cached(CPUSH4State *env, uint32_t addr)
{
    int n;
    int use_asid = !(env->mmucr & MMUCR_SV) || !(env->sr & (1u << SR_MD));

    /* check area */
    if (env->sr & (1u << SR_MD)) {
        /* For privileged mode, P2 and P4 area is not cacheable. */
        if ((0xA0000000 <= addr && addr < 0xC0000000) || 0xE0000000 <= addr)
            return 0;
    } else {
        /* For user mode, only U0 area is cacheable. */
        if (0x80000000 <= addr)
            return 0;
    }

    /*
     * TODO : Evaluate CCR and check if the cache is on or off.
     *        Now CCR is not in CPUSH4State, but in SH7750State.
     *        When you move the ccr into CPUSH4State, the code will be
     *        as follows.
     */
#if 0
    /* check if operand cache is enabled or not. */
    if (!(env->ccr & 1))
        return 0;
#endif

    /* if MMU is off, no check for TLB. */
    if (env->mmucr & MMUCR_AT)
        return 1;

    /* check TLB */
    n = find_tlb_entry(env, addr, env->itlb, ITLB_SIZE, use_asid);
    if (n >= 0)
        return env->itlb[n].c;

    n = find_tlb_entry(env, addr, env->utlb, UTLB_SIZE, use_asid);
    if (n >= 0)
        return env->utlb[n].c;

    return 0;
}

bool superh_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    if (interrupt_request & CPU_INTERRUPT_HARD) {
        /* Delay slots are indivisible, ignore interrupts */
        if (cpu_env(cs)->flags & TB_FLAG_DELAY_SLOT_MASK) {
            return false;
        } else {
            superh_cpu_do_interrupt(cs);
            return true;
        }
    }
    return false;
}

bool superh_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                         MMUAccessType access_type, int mmu_idx,
                         bool probe, uintptr_t retaddr)
{
    CPUSH4State *env = cpu_env(cs);
    int ret;

    hwaddr physical;
    int prot;
    uint32_t page_size;

    ret = get_physical_address(env, &physical, &prot, &page_size, address,
                               access_type);

    if (ret == MMU_OK) {
        /*
         * A 1 KiB page is smaller than TARGET_PAGE_SIZE. Passing its size
         * makes the softmmu check every access again, so neighbouring
         * 1 KiB pages still miss or fault. The access itself uses the
         * TARGET_PAGE_SIZE frame, which is only right if the virtual and
         * physical address agree above the 1 KiB offset.
         */
        if ((address ^ physical) & ~(hwaddr)(page_size - 1) &
            ~TARGET_PAGE_MASK) {
            qemu_log_mask(LOG_UNIMP, "sh4: 1 KiB page at 0x%" VADDR_PRIx
                          " maps to 0x%" HWADDR_PRIx ", which differs in bits"
                          " [11:10]; accesses will use the wrong address\n",
                          address, physical);
        }
        address &= TARGET_PAGE_MASK;
        physical &= TARGET_PAGE_MASK;
        tlb_set_page(cs, address, physical, prot, mmu_idx, page_size);
        return true;
    }
    if (probe) {
        return false;
    }

    if (ret != MMU_DTLB_MULTIPLE && ret != MMU_ITLB_MULTIPLE) {
        env->pteh = (env->pteh & PTEH_ASID_MASK) | (address & PTEH_VPN_MASK);
    }

    env->tea = address;
    switch (ret) {
    case MMU_ITLB_MISS:
    case MMU_DTLB_MISS_READ:
        cs->exception_index = 0x040;
        break;
    case MMU_DTLB_MULTIPLE:
    case MMU_ITLB_MULTIPLE:
        cs->exception_index = 0x140;
        break;
    case MMU_PMB_MISS:
        /* 7.8.4: a P1/P2 access without a PMB entry resets the CPU */
        qemu_log_mask(LOG_GUEST_ERROR, "sh4: PMB miss at 0x%" VADDR_PRIx
                      ", TLB reset\n", address);
        cs->exception_index = 0x140;
        break;
    case MMU_ITLB_VIOLATION:
        cs->exception_index = 0x0a0;
        break;
    case MMU_DTLB_MISS_WRITE:
        cs->exception_index = 0x060;
        break;
    case MMU_DTLB_INITIAL_WRITE:
        cs->exception_index = 0x080;
        break;
    case MMU_DTLB_VIOLATION_READ:
        cs->exception_index = 0x0a0;
        break;
    case MMU_DTLB_VIOLATION_WRITE:
        cs->exception_index = 0x0c0;
        break;
    case MMU_IADDR_ERROR:
    case MMU_DADDR_ERROR_READ:
        cs->exception_index = 0x0e0;
        break;
    case MMU_DADDR_ERROR_WRITE:
        cs->exception_index = 0x100;
        break;
    default:
        cpu_abort(cs, "Unhandled MMU fault");
    }
    cpu_loop_exit_restore(cs, retaddr);
}
/*
 * Memory-mapped PMB (7.8.5): address array H'F610 0000 (VPN[31:24], V),
 * data array H'F710 0000 (PPN[31:24], UB, V, SZ in bits 7 and 4, C, WT).
 * The entry is selected by address bits 11:8.
 */
uint32_t cpu_sh4_read_mmaped_pmb_addr(CPUSH4State *s, hwaddr addr)
{
    pmb_t *e = &s->pmb[(addr >> 8) & 0xf];

    return (uint32_t)e->vpn << 24 | e->v << 8;
}

void cpu_sh4_write_mmaped_pmb_addr(CPUSH4State *s, hwaddr addr,
                                   uint32_t mem_value)
{
    pmb_t *e = &s->pmb[(addr >> 8) & 0xf];

    if (e->v) {
        pmb_flush_entry(s, e);
    }
    e->vpn = mem_value >> 24;
    e->v = (mem_value >> 8) & 1;
}

uint32_t cpu_sh4_read_mmaped_pmb_data(CPUSH4State *s, hwaddr addr)
{
    pmb_t *e = &s->pmb[(addr >> 8) & 0xf];

    return (uint32_t)e->ppn << 24 | e->ub << 9 | e->v << 8 | (e->sz & 2) << 6 |
           (e->sz & 1) << 4 | e->c << 3 | e->wt;
}

void cpu_sh4_write_mmaped_pmb_data(CPUSH4State *s, hwaddr addr,
                                   uint32_t mem_value)
{
    pmb_t *e = &s->pmb[(addr >> 8) & 0xf];

    if (e->v) {
        pmb_flush_entry(s, e);
    }
    e->ppn = mem_value >> 24;
    e->ub = (mem_value >> 9) & 1;
    e->v = (mem_value >> 8) & 1;
    e->sz = ((mem_value >> 6) & 2) | ((mem_value >> 4) & 1);
    e->c = (mem_value >> 3) & 1;
    e->wt = mem_value & 1;
}

/* PASCR (7.8.6): SE bit 31, UB bits 7:0. Changing SE remaps everything. */
void cpu_sh4_write_pascr(CPUSH4State *s, uint32_t value)
{
    value &= PASCR_SE | 0xff;
    if ((s->pascr ^ value) & PASCR_SE) {
        tlb_flush(env_cpu(s));
    }
    s->pascr = value;
}

#endif /* !CONFIG_USER_ONLY */


