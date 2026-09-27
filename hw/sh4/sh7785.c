/*
 * Renesas SH7785 (SH-4A, SH-X2 core)
 *
 * Source: SH7785 Hardware Manual REJ09B0261-0100 and SH-4A Extended
 * Functions Software Manual R01US0060EJ0200. Section numbers in comments
 * refer to the hardware manual unless noted.
 *
 * Modelled: CCN/MMU registers including TLB extended mode, 32-bit address
 * extended mode with the PMB, memory-mapped TLB and PMB arrays,
 * INTC/INTC2, SCIF0-5, TMU0-5, on-chip IL/OL/U memory.
 * Every other register listed in section 31 is a plain register with its
 * power-on reset value, and every access to it is traced, so a guest that
 * relies on real behaviour there shows up in the log.
 *
 * The SH7786 (SH-X3, two cores; hardware manual REJ09B0501-0100) shares the
 * CCN, MMU and array code and is set up by sh7786_init(): each core sees its
 * own CCN registers (CPIDR tells them apart), and C0/C1STBCR with
 * C0/C1RESETVEC (section 19/20) start and stop the cores. It has no
 * consolidated register list, so its catch-all registers just hold what
 * was written, starting at 0.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/or-irq.h"
#include "hw/sh4/sh.h"
#include "hw/sh4/sh7785.h"
#include "hw/intc/sh7785_intc.h"
#include "hw/intc/sh7786_intc.h"
#include "hw/pci-host/sh7785_pcic.h"
#include "hw/timer/tmu012.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "target/sh4/cpu.h"
#include "exec/cputlb.h"
#include "trace.h"
#include "hw/core/cpu.h"
#include "system/cpus.h"

typedef struct {
    uint32_t addr;
    uint8_t size;
    uint32_t reset;
    uint8_t undefined;
    const char *name;
} SH7785Reg;

#include "sh7785_regs.h"

#define SH_MAX_CPUS 2

struct SH7785State {
    SuperHCPU *cpu;             /* CPU0 */
    SuperHCPU *cpus[SH_MAX_CPUS];
    int ncpus;
    bool sh7786;
    MemoryRegion ccn, ccn_a7;
    MemoryRegion mmct;
    MemoryRegion regfile, regfile_a7;
    MemoryRegion il_ram, ol_ram, u_ram;
    MemoryRegion cpuctl, cpuctl_a7;
    SH7785IntcState *intc;
    SH7786IntcState *intc86;
    SH7785PCICState *pcic;
    uint32_t ccr[SH_MAX_CPUS], qacr[SH_MAX_CPUS][2];
    uint32_t ramcr[SH_MAX_CPUS], irmcr[SH_MAX_CPUS];
    uint32_t stbcr[SH_MAX_CPUS], resetvec[SH_MAX_CPUS];
    bool firmware;              /* something at H'A000 0000 parks cores */
    GHashTable *regs86;         /* SH7786 catch-all: address -> value */
    MemoryRegion mmselr_mr;
    uint32_t mmselr;
    void (*mmselr_hook)(void *opaque, int areasel);
    void *mmselr_opaque;
    uint32_t regvals[ARRAY_SIZE(sh7785_regs)];
};

/*
 * CCN / MMU control registers, H'FF00 0000 (sections 7 and 8 of the SH-4A
 * software manual)
 */
#define CCN_PTEH    0x00
#define CCN_PTEL    0x04
#define CCN_TTB     0x08
#define CCN_TEA     0x0c
#define CCN_MMUCR   0x10
#define CCN_CCR     0x1c
#define CCN_TRA     0x20
#define CCN_EXPEVT  0x24
#define CCN_INTEVT  0x28
#define CCN_PVR     0x30
#define CCN_PTEA    0x34
#define CCN_QACR0   0x38
#define CCN_QACR1   0x3c
#define CCN_CVR     0x40
#define CCN_PRR     0x44
#define CCN_CPIDR   0x48    /* SH7786: number of the core that reads it */
#define CCN_PASCR   0x70
#define CCN_RAMCR   0x74
#define CCN_IRMCR   0x78
#define CCN_PTEAEX  0x7c    /* SH-X3 16-bit ASID (appendix A of the SH7786) */

/* MMUCR bits that can be written: LRUI, URB, URC, SQMD, SV, ME, TI, AT */
#define MMUCR_WMASK 0xfcfcff85
/* PTEA: EPR[13:8], ESZ[7:4] (7.2.6 of the software manual) */
#define PTEA_WMASK  0x00003ff0
/* CCR: ICI (bit 11) and OCI (bit 3) always read as 0 */
#define CCR_ICI     (1 << 11)
#define CCR_OCI     (1 << 3)

/* The core that makes the access: CCN and the arrays are per core */
static int sh7785_cur(SH7785State *s)
{
    for (int i = 0; i < s->ncpus; i++) {
        if (current_cpu == CPU(s->cpus[i])) {
            return i;
        }
    }
    return 0;
}

static uint64_t sh7785_ccn_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785State *s = opaque;
    int n = sh7785_cur(s);
    CPUSH4State *env = &s->cpus[n]->env;
    SuperHCPUClass *scc = SUPERH_CPU_GET_CLASS(s->cpus[n]);

    switch (addr) {
    case CCN_PTEH:
        return env->pteh;
    case CCN_PTEL:
        return env->ptel;
    case CCN_TTB:
        return env->ttb;
    case CCN_TEA:
        return env->tea;
    case CCN_MMUCR:
        return env->mmucr;
    case CCN_CCR:
        return s->ccr[n];
    case CCN_TRA:
        return env->tra;
    case CCN_EXPEVT:
        return env->expevt;
    case CCN_INTEVT:
        return env->intevt;
    case CCN_PVR:
        return scc->pvr;
    case CCN_PTEA:
        return env->ptea;
    case CCN_QACR0:
    case CCN_QACR1:
        return s->qacr[n][(addr - CCN_QACR0) / 4];
    case CCN_CVR:
        return scc->cvr;
    case CCN_PRR:
        return scc->prr;
    case CCN_PASCR:
        return env->pascr;
    case CCN_RAMCR:
        return s->ramcr[n];
    case CCN_IRMCR:
        return s->irmcr[n];
    case CCN_CPIDR:
        if (s->sh7786) {
            return n;
        }
        break;
    case CCN_PTEAEX:
        if (env->features & SH_FEATURE_PTEAEX) {
            return env->pteaex;
        }
        break;
    }
    qemu_log_mask(LOG_UNIMP, "sh7785: read of unknown CCN register "
                  "0x%" HWADDR_PRIx "\n", addr);
    return 0;
}

static void sh7785_ccn_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    SH7785State *s = opaque;
    int n = sh7785_cur(s);
    CPUSH4State *env = &s->cpus[n]->env;
    CPUState *cs = CPU(s->cpus[n]);
    bool aex = (env->features & SH_FEATURE_PTEAEX) &&
               (env->mmucr & MMUCR_AEX);
    uint32_t old;

    switch (addr) {
    case CCN_PTEH:
        /* The softmmu TLB is not tagged with the ASID */
        if (!aex && (env->pteh & 0xff) != (val & 0xff)) {
            tlb_flush(cs);
        }
        env->pteh = val;
        return;
    case CCN_PTEL:
        /* PPN[31:29] and UB (bit 9) exist only in 32-bit mode (7.8.6) */
        env->ptel = val & (env->pascr & PASCR_SE ? 0xffffffff : 0x1ffffdff);
        return;
    case CCN_TTB:
        env->ttb = val;
        return;
    case CCN_TEA:
        env->tea = val;
        return;
    case CCN_MMUCR:
        old = env->mmucr;
        env->mmucr = val & (MMUCR_WMASK |
            (env->features & SH_FEATURE_PTEAEX ? MMUCR_AEX : 0)) & ~MMUCR_TI;
        if (val & MMUCR_TI) {
            cpu_sh4_invalidate_tlb(env);
        } else if ((old ^ env->mmucr) &
                   (MMUCR_AT | MMUCR_SV | MMUCR_ME | MMUCR_AEX)) {
            /* Softmmu entries depend on AT, SV, ME and the ASID mode */
            tlb_flush(cs);
        }
        return;
    case CCN_CCR:
        s->ccr[n] = val & ~(CCR_ICI | CCR_OCI);
        sh4_exact_ccr_write(val);
        return;
    case CCN_TRA:
        env->tra = val & 0x000003fc;
        return;
    case CCN_EXPEVT:
        env->expevt = val & 0x00000fff;
        return;
    case CCN_INTEVT:
        env->intevt = val & 0x00003fff;
        return;
    case CCN_PTEA:
        env->ptea = val & PTEA_WMASK;
        return;
    case CCN_QACR0:
    case CCN_QACR1:
        s->qacr[n][(addr - CCN_QACR0) / 4] = val & 0x1c;
        return;
    case CCN_PASCR:
        cpu_sh4_write_pascr(env, val);
        return;
    case CCN_RAMCR:
        s->ramcr[n] = val;
        return;
    case CCN_IRMCR:
        s->irmcr[n] = val & 0x1f;
        return;
    case CCN_PTEAEX:
        if (!(env->features & SH_FEATURE_PTEAEX)) {
            break;
        }
        if (aex && (env->pteaex & 0xffff) != (val & 0xffff)) {
            tlb_flush(cs);
        }
        env->pteaex = val & 0xffff;
        return;
    case CCN_CPIDR:
        if (s->sh7786) {
            return;
        }
        break;
    case CCN_PVR:
    case CCN_CVR:
    case CCN_PRR:
        return;
    }
    qemu_log_mask(LOG_UNIMP, "sh7785: write of unknown CCN register "
                  "0x%" HWADDR_PRIx "\n", addr);
}

static const MemoryRegionOps sh7785_ccn_ops = {
    .read = sh7785_ccn_read,
    .write = sh7785_ccn_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * Memory-mapped cache, TLB and PMB arrays, H'F000 0000 - H'F7FF FFFF
 * (software manual 7.7, 7.8.5, 8.6)
 */
static uint64_t sh7785_mmct_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785State *s = opaque;
    CPUSH4State *env = &s->cpus[sh7785_cur(s)]->env;

    switch (addr >> 24) {
    case 0x0: /* IC address array */
    case 0x1: /* IC data array */
    case 0x4: /* OC address array */
    case 0x5: /* OC data array */
        return 0;       /* caches are not modelled */
    case 0x2:
        if ((addr & 0x00800000) && (env->features & SH_FEATURE_PTEAEX)) {
            return cpu_sh4_read_mmaped_tlb_addr2(env, addr, false);
        }
        return cpu_sh4_read_mmaped_itlb_addr(env, addr);
    case 0x3:
        return cpu_sh4_read_mmaped_itlb_data(env, addr);
    case 0x6:
        if ((addr & 0x00f00000) == 0) {
            return cpu_sh4_read_mmaped_utlb_addr(env, addr);
        }
        if ((addr & 0x00f00000) == 0x00100000) {
            return cpu_sh4_read_mmaped_pmb_addr(env, addr);
        }
        if ((addr & 0x00800000) && (env->features & SH_FEATURE_PTEAEX)) {
            return cpu_sh4_read_mmaped_tlb_addr2(env, addr, true);
        }
        break;
    case 0x7:
        if ((addr & 0x00700000) == 0) {
            return cpu_sh4_read_mmaped_utlb_data(env, addr);
        }
        if ((addr & 0x00f00000) == 0x00100000) {
            return cpu_sh4_read_mmaped_pmb_data(env, addr);
        }
        break;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "sh7785: read of reserved array address"
                  " 0x%08" HWADDR_PRIx "\n", addr + 0xf0000000);
    return 0;
}

static void sh7785_mmct_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    SH7785State *s = opaque;
    CPUSH4State *env = &s->cpus[sh7785_cur(s)]->env;

    switch (addr >> 24) {
    case 0x0:
        sh4_exact_ic_array_write(addr, val);
        return;
    case 0x4:
        sh4_exact_oc_array_write(addr, val);
        return;
    case 0x1:
    case 0x5:
        return;
    case 0x2:
        if ((addr & 0x00800000) && (env->features & SH_FEATURE_PTEAEX)) {
            cpu_sh4_write_mmaped_tlb_addr2(env, addr, val, false);
            return;
        }
        cpu_sh4_write_mmaped_itlb_addr(env, addr, val);
        return;
    case 0x3:
        cpu_sh4_write_mmaped_itlb_data(env, addr, val);
        return;
    case 0x6:
        if ((addr & 0x00f00000) == 0) {
            cpu_sh4_write_mmaped_utlb_addr(env, addr, val);
            return;
        }
        if ((addr & 0x00f00000) == 0x00100000) {
            cpu_sh4_write_mmaped_pmb_addr(env, addr, val);
            return;
        }
        if ((addr & 0x00800000) && (env->features & SH_FEATURE_PTEAEX)) {
            cpu_sh4_write_mmaped_tlb_addr2(env, addr, val, true);
            return;
        }
        break;
    case 0x7:
        if ((addr & 0x00700000) == 0) {
            cpu_sh4_write_mmaped_utlb_data(env, addr, val);
            return;
        }
        if ((addr & 0x00f00000) == 0x00100000) {
            cpu_sh4_write_mmaped_pmb_data(env, addr, val);
            return;
        }
        break;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "sh7785: write of reserved array "
                  "address 0x%08" HWADDR_PRIx "\n", addr + 0xf0000000);
}

static const MemoryRegionOps sh7785_mmct_ops = {
    .read = sh7785_mmct_read,
    .write = sh7785_mmct_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * Catch-all for the rest of H'FC00 0000 - H'FFFF FFFF: registers from
 * section 31 hold their value, everything else reads as 0.
 */
static int sh7785_reg_find(uint32_t addr)
{
    int lo = 0, hi = ARRAY_SIZE(sh7785_regs) - 1;

    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        const SH7785Reg *r = &sh7785_regs[mid];

        if (addr < r->addr) {
            hi = mid - 1;
        } else if (addr >= r->addr + r->size) {
            lo = mid + 1;
        } else {
            return mid;
        }
    }
    return -1;
}

/* SH7786: no register list to go by, so any register holds its value */
static uint64_t sh7786_regfile_read(SH7785State *s, uint32_t a, unsigned size)
{
    gpointer v = g_hash_table_lookup(s->regs86, GUINT_TO_POINTER(a & ~3));
    uint32_t val = GPOINTER_TO_UINT(v);

    if (!g_hash_table_contains(s->regs86, GUINT_TO_POINTER(a & ~3))) {
        qemu_log_mask(LOG_UNIMP, "sh7786: read of unmodelled register "
                      "0x%08x\n", a);
    }
    return extract32(val, (a & 3) * 8, size * 8);
}

static void sh7786_regfile_write(SH7785State *s, uint32_t a, uint64_t val,
                                 unsigned size)
{
    gpointer key = GUINT_TO_POINTER(a & ~3);
    uint32_t old = GPOINTER_TO_UINT(g_hash_table_lookup(s->regs86, key));

    qemu_log_mask(LOG_UNIMP, "sh7786: write 0x%" PRIx64 " to unmodelled "
                  "register 0x%08x\n", val, a);
    g_hash_table_insert(s->regs86, key, GUINT_TO_POINTER(
        deposit32(old, (a & 3) * 8, size * 8, val)));
}

static uint64_t sh7785_regfile_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785State *s = opaque;
    uint32_t a = 0xfc000000 + addr;
    int i;
    uint64_t val;

    if (s->sh7786) {
        return sh7786_regfile_read(s, a, size);
    }
    i = sh7785_reg_find(a);

    if (i < 0) {
        qemu_log_mask(LOG_UNIMP, "sh7785: read of unknown register "
                      "0x%08x\n", a);
        return 0;
    }
    val = s->regvals[i] >> ((a - sh7785_regs[i].addr) * 8);
    trace_sh7785_reg_read(sh7785_regs[i].name, a, size, val);
    return val;
}

static void sh7785_regfile_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    SH7785State *s = opaque;
    uint32_t a = 0xfc000000 + addr;
    int i;
    unsigned shift;
    uint64_t mask;

    if (s->sh7786) {
        sh7786_regfile_write(s, a, val, size);
        return;
    }
    i = sh7785_reg_find(a);

    if (i < 0) {
        qemu_log_mask(LOG_UNIMP, "sh7785: write of unknown register "
                      "0x%08x = 0x%" PRIx64 "\n", a, val);
        return;
    }
    trace_sh7785_reg_write(sh7785_regs[i].name, a, size, val);
    shift = (a - sh7785_regs[i].addr) * 8;
    mask = MAKE_64BIT_MASK(shift, size * 8);
    s->regvals[i] = (s->regvals[i] & ~mask) | ((val << shift) & mask);
}

static const MemoryRegionOps sh7785_regfile_ops = {
    .read = sh7785_regfile_read,
    .write = sh7785_regfile_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/*
 * State after a power-on or manual reset with the mode pins selecting
 * 32-bit boot (7.9.1 of the SH-4A software manual, PPN from 7.9.1 of the
 * SH7785 manual): SE set, P1 and P2 mapped to physical 0 in 512 MiB pages.
 */
void sh7785_reset_32bit_boot(SH7785State *s)
{
    CPUSH4State *env = &s->cpu->env;

    env->pascr = PASCR_SE;
    env->pmb[0] = (pmb_t) { .vpn = 0x80, .ppn = 0x00, .v = 1, .sz = 3,
                            .c = 1, .ub = 0, .wt = 1 };
    env->pmb[1] = (pmb_t) { .vpn = 0xa0, .ppn = 0x00, .v = 1, .sz = 3,
                            .c = 0, .ub = 0, .wt = 0 };
    tlb_flush(CPU(s->cpu));
}

void sh7785_set_reg(SH7785State *s, uint32_t addr, uint32_t val)
{
    int i;

    if (s->sh7786) {
        g_hash_table_insert(s->regs86, GUINT_TO_POINTER(addr & ~3),
                            GUINT_TO_POINTER(val));
        return;
    }
    i = sh7785_reg_find(addr);

    assert(i >= 0);
    s->regvals[i] = val;
}

/* MMSELR, H'FC40 0020 (11.4.1): writes need H'A5A5 in bits 31:16 */
static uint64_t sh7785_mmselr_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785State *s = opaque;

    return s->mmselr;
}

static void sh7785_mmselr_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    SH7785State *s = opaque;

    if ((val >> 16) != 0xa5a5) {
        qemu_log_mask(LOG_GUEST_ERROR, "sh7785: MMSELR write without the "
                      "H'A5A5 code ignored (0x%" PRIx64 ")\n", val);
        return;
    }
    s->mmselr = val & 7;
    if (s->mmselr > 6) {
        qemu_log_mask(LOG_GUEST_ERROR, "sh7785: MMSELR.AREASEL 7 is "
                      "reserved\n");
    }
    if (s->mmselr_hook) {
        s->mmselr_hook(s->mmselr_opaque, s->mmselr);
    }
}

static const MemoryRegionOps sh7785_mmselr_ops = {
    .read = sh7785_mmselr_read,
    .write = sh7785_mmselr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

void sh7785_set_mmselr_hook(SH7785State *s,
                            void (*hook)(void *opaque, int areasel),
                            void *opaque)
{
    s->mmselr_hook = hook;
    s->mmselr_opaque = opaque;
    hook(opaque, s->mmselr);
}

void sh7785_preset_mmselr(SH7785State *s, int areasel)
{
    s->mmselr = areasel;
    if (s->mmselr_hook) {
        s->mmselr_hook(s->mmselr_opaque, areasel);
    }
}

static void sh7785_alias(MemoryRegion *sysmem, MemoryRegion *alias,
                         const char *name, MemoryRegion *mr, hwaddr p4,
                         int prio)
{
    memory_region_add_subregion_overlap(sysmem, p4, mr, prio);
    memory_region_init_alias(alias, NULL, name, mr, 0,
                             memory_region_size(mr));
    memory_region_add_subregion_overlap(sysmem, A7ADDR(p4), alias, prio);
}

static void sh7785_map_sysbus(MemoryRegion *sysmem, SysBusDevice *sb, int n,
                              hwaddr p4)
{
    MemoryRegion *mr = sysbus_mmio_get_region(sb, n);
    MemoryRegion *alias = g_new(MemoryRegion, 1);

    memory_region_add_subregion(sysmem, p4, mr);
    memory_region_init_alias(alias, NULL, "a7-alias", mr, 0,
                             memory_region_size(mr));
    memory_region_add_subregion(sysmem, A7ADDR(p4), alias);
}

/*
 * PCIC (section 13): registers at H'FE04 0000, PCI memory space 0 at
 * H'FD00 0000, I/O at H'FE20 0000, memory space 2 at physical H'C000 0000.
 * Memory space 1 (area 4) is mapped by the board as MMSELR selects.
 */
static void sh7785_pcic_init(SH7785State *s, MemoryRegion *sysmem)
{
    DeviceState *dev = qdev_new(TYPE_SH7785_PCIC);
    SysBusDevice *sb = SYS_BUS_DEVICE(dev);
    DeviceState *intc = DEVICE(s->intc);

    sysbus_realize_and_unref(sb, &error_fatal);
    s->pcic = SH7785_PCIC(dev);
    sh7785_alias(sysmem, g_new(MemoryRegion, 1), "sh7785-pcic-a7",
                 sysbus_mmio_get_region(sb, SH7785_PCIC_MMIO_REGS),
                 0xfe040000, 1);
    sh7785_alias(sysmem, g_new(MemoryRegion, 1), "sh7785-pcic.mem0-a7",
                 sysbus_mmio_get_region(sb, SH7785_PCIC_MMIO_MEM0),
                 0xfd000000, 1);
    sh7785_alias(sysmem, g_new(MemoryRegion, 1), "sh7785-pcic.io-a7",
                 sysbus_mmio_get_region(sb, SH7785_PCIC_MMIO_IO),
                 0xfe200000, 1);
    memory_region_add_subregion(sysmem, 0xc0000000,
                        sysbus_mmio_get_region(sb, SH7785_PCIC_MMIO_MEM2));
    for (int i = 0; i < 4; i++) {
        sysbus_connect_irq(sb, i, qdev_get_gpio_in_named(intc, "onchip",
                                                 SH7785_IRQ_PCIINTA + i));
    }
}

/* SCIF0-5 at H'FFEA 0000 + n * H'1 0000 (section 21) */
static void sh7785_scif_init(SH7785State *s, MemoryRegion *sysmem, int n)
{
    static const int single[4] = {
        SH7785_IRQ_SCIF2, SH7785_IRQ_SCIF3, SH7785_IRQ_SCIF4, SH7785_IRQ_SCIF5
    };
    DeviceState *dev = qdev_new(TYPE_SH_SERIAL);
    DeviceState *intc = DEVICE(s->intc);
    SysBusDevice *sb = SYS_BUS_DEVICE(dev);

    dev->id = g_strdup_printf("scif%d", n);
    qdev_prop_set_chr(dev, "chardev", serial_hd(n));
    qdev_prop_set_uint8(dev, "features",
                        SH_SERIAL_FEAT_SCIF | SH_SERIAL_FEAT_FIFODATA);
    sysbus_realize_and_unref(sb, &error_fatal);
    sh7785_map_sysbus(sysmem, sb, 0, 0xffea0000 + n * 0x10000);

    if (n < 2) {
        int base = n == 0 ? SH7785_IRQ_ERI0 : SH7785_IRQ_ERI1;

        qdev_connect_gpio_out_named(dev, "eri", 0,
            qdev_get_gpio_in_named(intc, "onchip", base));
        qdev_connect_gpio_out_named(dev, "rxi", 0,
            qdev_get_gpio_in_named(intc, "onchip", base + 1));
        qdev_connect_gpio_out_named(dev, "bri", 0,
            qdev_get_gpio_in_named(intc, "onchip", base + 2));
        qdev_connect_gpio_out_named(dev, "txi", 0,
            qdev_get_gpio_in_named(intc, "onchip", base + 3));
    } else {
        /* SCIF2-5 have one INTEVT code for all four sources (table 10.1) */
        DeviceState *or = qdev_new(TYPE_OR_IRQ);

        qdev_prop_set_uint16(or, "num-lines", 4);
        qdev_realize_and_unref(or, NULL, &error_fatal);
        qdev_connect_gpio_out(or, 0,
            qdev_get_gpio_in_named(intc, "onchip", single[n - 2]));
        qdev_connect_gpio_out_named(dev, "eri", 0, qdev_get_gpio_in(or, 0));
        qdev_connect_gpio_out_named(dev, "rxi", 0, qdev_get_gpio_in(or, 1));
        qdev_connect_gpio_out_named(dev, "bri", 0, qdev_get_gpio_in(or, 2));
        qdev_connect_gpio_out_named(dev, "txi", 0, qdev_get_gpio_in(or, 3));
    }
}

SH7785State *sh7785_init(SuperHCPU *cpu, MemoryRegion *sysmem,
                         uint32_t pclk_hz)
{
    SH7785State *s = g_new0(SH7785State, 1);
    DeviceState *intc;
    SysBusDevice *sb;
    int i;

    s->cpu = cpu;
    s->cpus[0] = cpu;
    s->ncpus = 1;
    for (i = 0; i < ARRAY_SIZE(sh7785_regs); i++) {
        s->regvals[i] = sh7785_regs[i].reset;
    }

    /* Catch-all register file below everything else */
    memory_region_init_io(&s->regfile, NULL, &sh7785_regfile_ops, s,
                          "sh7785-regs", 64 * MiB);
    sh7785_alias(sysmem, &s->regfile_a7, "sh7785-regs-a7", &s->regfile,
                 0xfc000000, -1);

    memory_region_init_io(&s->ccn, NULL, &sh7785_ccn_ops, s, "sh7785-ccn",
                          0x100);
    sh7785_alias(sysmem, &s->ccn_a7, "sh7785-ccn-a7", &s->ccn, 0xff000000, 0);

    memory_region_init_io(&s->mmselr_mr, NULL, &sh7785_mmselr_ops, s,
                          "sh7785-mmselr", 4);
    memory_region_add_subregion(sysmem, 0xfc400020, &s->mmselr_mr);

    memory_region_init_io(&s->mmct, NULL, &sh7785_mmct_ops, s,
                          "sh7785-cache-tlb", 128 * MiB);
    memory_region_add_subregion(sysmem, 0xf0000000, &s->mmct);

    /* On-chip memory (9.1): OL 16 KiB, IL 8 KiB, U 128 KiB */
    memory_region_init_ram(&s->ol_ram, NULL, "sh7785-ol-ram", 16 * KiB,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0xe500e000, &s->ol_ram);
    memory_region_init_ram(&s->il_ram, NULL, "sh7785-il-ram", 8 * KiB,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0xe5200000, &s->il_ram);
    memory_region_init_ram(&s->u_ram, NULL, "sh7785-u-ram", 128 * KiB,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0xe55f0000, &s->u_ram);

    /* INTC (section 10) */
    intc = qdev_new(TYPE_SH7785_INTC);
    object_property_set_link(OBJECT(intc), "cpu", OBJECT(cpu), &error_abort);
    sb = SYS_BUS_DEVICE(intc);
    sysbus_realize_and_unref(sb, &error_fatal);
    s->intc = SH7785_INTC(intc);
    sh7785_map_sysbus(sysmem, sb, 0, 0xffd00000);
    sh7785_map_sysbus(sysmem, sb, 1, 0xffd30000);
    sh7785_map_sysbus(sysmem, sb, 2, 0xffd40000);
    cpu->env.intc_handle = s->intc;
    cpu->env.intc_get_vector = sh7785_intc_get_vector;

    /* TMU0-2 and TMU3-5 (section 18), clocked by Pck */
    tmu012_init(sysmem, 0xffd80000, TMU012_FEAT_3CHAN, pclk_hz,
                qdev_get_gpio_in_named(intc, "onchip", SH7785_IRQ_TUNI0),
                qdev_get_gpio_in_named(intc, "onchip", SH7785_IRQ_TUNI1),
                qdev_get_gpio_in_named(intc, "onchip", SH7785_IRQ_TUNI2),
                qdev_get_gpio_in_named(intc, "onchip", SH7785_IRQ_TICPI2));
    tmu012_init(sysmem, 0xffdc0000, TMU012_FEAT_3CHAN, pclk_hz,
                qdev_get_gpio_in_named(intc, "onchip", SH7785_IRQ_TUNI3),
                qdev_get_gpio_in_named(intc, "onchip", SH7785_IRQ_TUNI4),
                qdev_get_gpio_in_named(intc, "onchip", SH7785_IRQ_TUNI5),
                NULL);

    for (i = 0; i < 6; i++) {
        sh7785_scif_init(s, sysmem, i);
    }
    sh7785_pcic_init(s, sysmem);
    return s;
}

/*
 * SH7786 core control, H'FE40 0000 + n * H'1000 (19.3.11, 20.3.6/7):
 * CnSTBCR at +4 (LTSLP, SLEEP, RESET, MSTP) and CnRESETVEC at +8. A core
 * in module stop does not run. Clearing MSTP with RESET set power-on
 * resets the core, which then starts at CnRESETVEC.
 */
#define CNSTBCR     0x004
#define CNRESETVEC  0x008
#define STBCR_MSTP  (1u << 0)
#define STBCR_RESET (1u << 1)
#define STBCR_WMASK 0x80000007

static void sh7786_core_start(CPUState *cs, run_on_cpu_data data)
{
    CPUSH4State *env = cpu_env(cs);

    cpu_reset(cs);
    env->pc = data.host_int;
    env->mstp = false;
    cs->halted = 0;
}

static void sh7786_core_stop(CPUState *cs, run_on_cpu_data data)
{
    cpu_env(cs)->mstp = true;
    cs->halted = 1;
    cpu_exit(cs);
}

static uint64_t sh7786_cpuctl_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785State *s = opaque;
    int n = addr >> 12;

    if (n >= s->ncpus) {
        return 0;
    }
    switch (addr & 0xfff) {
    case CNSTBCR:
        return s->stbcr[n];
    case CNRESETVEC:
        return s->resetvec[n];
    }
    qemu_log_mask(LOG_UNIMP, "sh7786: read of unmodelled CPU control "
                  "register 0x%08" HWADDR_PRIx "\n", 0xfe400000 + addr);
    return 0;
}

static void sh7786_cpuctl_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    SH7785State *s = opaque;
    int n = addr >> 12;
    CPUState *cs;
    uint32_t old;

    if (n >= s->ncpus) {
        return;
    }
    cs = CPU(s->cpus[n]);
    switch (addr & 0xfff) {
    case CNSTBCR:
        old = s->stbcr[n];
        s->stbcr[n] = val & STBCR_WMASK;
        if (!(old & STBCR_MSTP) && (val & STBCR_MSTP)) {
            async_run_on_cpu(cs, sh7786_core_stop, RUN_ON_CPU_NULL);
        } else if ((old & STBCR_MSTP) && !(val & STBCR_MSTP)) {
            if (!(val & STBCR_RESET)) {
                qemu_log_mask(LOG_UNIMP, "sh7786: core %d leaves module stop"
                              " without reset, not modelled\n", n);
            } else if (s->resetvec[n] == 0xa0000000 && !s->firmware) {
                /*
                 * Nothing at the boot address to catch the core: firmware
                 * would park it, so it stays stopped until reset again
                 * with a vector software has set.
                 */
                s->stbcr[n] |= STBCR_MSTP;
            } else {
                async_run_on_cpu(cs, sh7786_core_start,
                                 RUN_ON_CPU_HOST_INT(s->resetvec[n]));
            }
        }
        return;
    case CNRESETVEC:
        s->resetvec[n] = val;
        return;
    }
    qemu_log_mask(LOG_UNIMP, "sh7786: write 0x%" PRIx64 " to unmodelled CPU "
                  "control register 0x%08" HWADDR_PRIx "\n", val,
                  0xfe400000 + addr);
}

static const MemoryRegionOps sh7786_cpuctl_ops = {
    .read = sh7786_cpuctl_read,
    .write = sh7786_cpuctl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* Power-on state of the cores: CPU0 runs, the others are in module stop */
void sh7786_reset_cores(SH7785State *s)
{
    for (int n = 0; n < s->ncpus; n++) {
        CPUState *cs = CPU(s->cpus[n]);

        s->resetvec[n] = 0xa0000000;
        s->stbcr[n] = n ? STBCR_RESET | STBCR_MSTP : 0;
        cpu_env(cs)->mstp = n != 0;
        if (n) {
            cs->halted = 1;
        }
    }
}

static qemu_irq sh7786_irq(SH7785State *s, int n)
{
    return qdev_get_gpio_in_named(DEVICE(s->intc86), "onchip", n);
}

/* Several sources on one INTC input */
static DeviceState *sh7786_or(qemu_irq out, int lines)
{
    DeviceState *or = qdev_new(TYPE_OR_IRQ);

    qdev_prop_set_uint16(or, "num-lines", lines);
    qdev_realize_and_unref(or, NULL, &error_fatal);
    qdev_connect_gpio_out(or, 0, out);
    return or;
}

/* SCIF0-5 at H'FFEA 0000 + n * H'1 0000; only SCIF0 has four event codes */
static void sh7786_scif_init(SH7785State *s, MemoryRegion *sysmem, int n)
{
    static const int single[6] = {
        -1, SH7786_IRQ_SCIF1, SH7786_IRQ_SCIF2, SH7786_IRQ_SCIF3,
        SH7786_IRQ_SCIF4, SH7786_IRQ_SCIF5,
    };
    DeviceState *dev = qdev_new(TYPE_SH_SERIAL);
    SysBusDevice *sb = SYS_BUS_DEVICE(dev);
    static const char *const names[4] = { "eri", "rxi", "bri", "txi" };

    dev->id = g_strdup_printf("scif%d", n);
    qdev_prop_set_chr(dev, "chardev", serial_hd(n));
    qdev_prop_set_uint8(dev, "features",
                        SH_SERIAL_FEAT_SCIF | SH_SERIAL_FEAT_FIFODATA);
    sysbus_realize_and_unref(sb, &error_fatal);
    sh7785_map_sysbus(sysmem, sb, 0, 0xffea0000 + n * 0x10000);

    if (n == 0) {
        /* ERI H'700, RXI H'720, BRI H'740, TXI H'760 */
        for (int i = 0; i < 4; i++) {
            qdev_connect_gpio_out_named(dev, names[i], 0,
                                        sh7786_irq(s, SH7786_IRQ_SCIF0_0 + i));
        }
    } else {
        DeviceState *or = sh7786_or(sh7786_irq(s, single[n]), 4);

        for (int i = 0; i < 4; i++) {
            qdev_connect_gpio_out_named(dev, names[i], 0,
                                        qdev_get_gpio_in(or, i));
        }
    }
}

SH7785State *sh7786_init(SuperHCPU **cpus, int ncpus, MemoryRegion *sysmem,
                         uint32_t pclk_hz, bool firmware)
{
    SH7785State *s = g_new0(SH7785State, 1);
    DeviceState *intc;
    SysBusDevice *sb;
    DeviceState *or;

    assert(ncpus >= 1 && ncpus <= SH_MAX_CPUS);
    s->sh7786 = true;
    s->cpu = cpus[0];
    s->ncpus = ncpus;
    s->firmware = firmware;
    for (int i = 0; i < ncpus; i++) {
        s->cpus[i] = cpus[i];
        if (ncpus > 1) {
            cpus[i]->env.features |= SH_FEATURE_SMP;
        }
    }
    s->regs86 = g_hash_table_new(NULL, NULL);

    memory_region_init_io(&s->regfile, NULL, &sh7785_regfile_ops, s,
                          "sh7786-regs", 64 * MiB);
    sh7785_alias(sysmem, &s->regfile_a7, "sh7786-regs-a7", &s->regfile,
                 0xfc000000, -1);
    memory_region_init_io(&s->ccn, NULL, &sh7785_ccn_ops, s, "sh7786-ccn",
                          0x100);
    sh7785_alias(sysmem, &s->ccn_a7, "sh7786-ccn-a7", &s->ccn, 0xff000000, 0);
    memory_region_init_io(&s->mmct, NULL, &sh7785_mmct_ops, s,
                          "sh7786-cache-tlb", 128 * MiB);
    memory_region_add_subregion(sysmem, 0xf0000000, &s->mmct);
    memory_region_init_io(&s->cpuctl, NULL, &sh7786_cpuctl_ops, s,
                          "sh7786-cpuctl", 0x1000 * SH_MAX_CPUS);
    sh7785_alias(sysmem, &s->cpuctl_a7, "sh7786-cpuctl-a7", &s->cpuctl,
                 0xfe400000, 0);

    /* INTC (section 10) at H'FE41 0000, USERIMASK at H'FE41 1000 */
    intc = qdev_new(TYPE_SH7786_INTC);
    for (int i = 0; i < ncpus; i++) {
        char name[8];

        snprintf(name, sizeof(name), "cpu%d", i);
        object_property_set_link(OBJECT(intc), name, OBJECT(cpus[i]),
                                 &error_abort);
    }
    sb = SYS_BUS_DEVICE(intc);
    sysbus_realize_and_unref(sb, &error_fatal);
    s->intc86 = SH7786_INTC(intc);
    sh7785_map_sysbus(sysmem, sb, 0, 0xfe410000);
    sh7785_map_sysbus(sysmem, sb, 1, 0xfe411000);
    for (int i = 0; i < ncpus; i++) {
        cpus[i]->env.intc_handle = &s->intc86->handle[i];
        cpus[i]->env.intc_get_vector = sh7786_intc_get_vector;
    }

    /*
     * TMU0-3, three channels each (section 17): TMU0 and TMU1 have an
     * event code per channel, TMU2 and TMU3 one for all three.
     */
    tmu012_init(sysmem, 0xffd80000, TMU012_FEAT_3CHAN, pclk_hz,
                sh7786_irq(s, SH7786_IRQ_TMU0_0),
                sh7786_irq(s, SH7786_IRQ_TMU0_1),
                sh7786_irq(s, SH7786_IRQ_TMU0_2),
                sh7786_irq(s, SH7786_IRQ_TMU0_3));
    tmu012_init(sysmem, 0xffda0000, TMU012_FEAT_3CHAN, pclk_hz,
                sh7786_irq(s, SH7786_IRQ_TMU1_0),
                sh7786_irq(s, SH7786_IRQ_TMU1_1),
                sh7786_irq(s, SH7786_IRQ_TMU1_2), NULL);
    or = sh7786_or(sh7786_irq(s, SH7786_IRQ_TMU2), 3);
    tmu012_init(sysmem, 0xffdc0000, TMU012_FEAT_3CHAN, pclk_hz,
                qdev_get_gpio_in(or, 0), qdev_get_gpio_in(or, 1),
                qdev_get_gpio_in(or, 2), NULL);
    or = sh7786_or(sh7786_irq(s, SH7786_IRQ_TMU3), 3);
    tmu012_init(sysmem, 0xffde0000, TMU012_FEAT_3CHAN, pclk_hz,
                qdev_get_gpio_in(or, 0), qdev_get_gpio_in(or, 1),
                qdev_get_gpio_in(or, 2), NULL);

    for (int i = 0; i < 6; i++) {
        sh7786_scif_init(s, sysmem, i);
    }
    return s;
}

SH7785PCICState *sh7785_pcic(SH7785State *s)
{
    return s->pcic;
}

qemu_irq sh7785_irq_pin(SH7785State *s, int n)
{
    return qdev_get_gpio_in_named(DEVICE(s->intc), "irq", n);
}
