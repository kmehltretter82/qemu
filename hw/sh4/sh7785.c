/*
 * Renesas SH7785 (SH-4A, SH-X2 core)
 *
 * Source: SH7785 Hardware Manual REJ09B0261-0100 and SH-4A Extended
 * Functions Software Manual R01US0060EJ0200. Section numbers in comments
 * refer to the hardware manual unless noted.
 *
 * Modelled: CCN/MMU registers including TLB extended mode, memory-mapped
 * TLB arrays, INTC/INTC2, SCIF0-5, TMU0-5, on-chip IL/OL/U memory.
 * Every other register listed in section 31 is a plain register with its
 * power-on reset value, and every access to it is traced, so a guest that
 * relies on real behaviour there shows up in the log.
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
#include "hw/timer/tmu012.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "target/sh4/cpu.h"
#include "exec/cputlb.h"
#include "trace.h"

typedef struct {
    uint32_t addr;
    uint8_t size;
    uint32_t reset;
    uint8_t undefined;
    const char *name;
} SH7785Reg;

#include "sh7785_regs.h"

struct SH7785State {
    SuperHCPU *cpu;
    MemoryRegion ccn, ccn_a7;
    MemoryRegion mmct;
    MemoryRegion regfile, regfile_a7;
    MemoryRegion il_ram, ol_ram, u_ram;
    SH7785IntcState *intc;
    uint32_t ccr, qacr[2], pascr, ramcr, irmcr;
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
#define CCN_PASCR   0x70
#define CCN_RAMCR   0x74
#define CCN_IRMCR   0x78

/* MMUCR bits that can be written: LRUI, URB, URC, SQMD, SV, ME, TI, AT */
#define MMUCR_WMASK 0xfcfcff85
/* PTEA: EPR[13:8], ESZ[7:4] (7.2.6 of the software manual) */
#define PTEA_WMASK  0x00003ff0
/* CCR: ICI (bit 11) and OCI (bit 3) always read as 0 */
#define CCR_ICI     (1 << 11)
#define CCR_OCI     (1 << 3)

static uint64_t sh7785_ccn_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785State *s = opaque;
    CPUSH4State *env = &s->cpu->env;
    SuperHCPUClass *scc = SUPERH_CPU_GET_CLASS(s->cpu);

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
        return s->ccr;
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
        return s->qacr[(addr - CCN_QACR0) / 4];
    case CCN_CVR:
        return scc->cvr;
    case CCN_PRR:
        return scc->prr;
    case CCN_PASCR:
        return s->pascr;
    case CCN_RAMCR:
        return s->ramcr;
    case CCN_IRMCR:
        return s->irmcr;
    }
    qemu_log_mask(LOG_UNIMP, "sh7785: read of unknown CCN register "
                  "0x%" HWADDR_PRIx "\n", addr);
    return 0;
}

static void sh7785_ccn_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    SH7785State *s = opaque;
    CPUSH4State *env = &s->cpu->env;
    uint32_t old;

    switch (addr) {
    case CCN_PTEH:
        /* The softmmu TLB is not tagged with the ASID */
        if ((env->pteh & 0xff) != (val & 0xff)) {
            tlb_flush(CPU(s->cpu));
        }
        env->pteh = val;
        return;
    case CCN_PTEL:
        env->ptel = val & 0x1ffffdff;
        return;
    case CCN_TTB:
        env->ttb = val;
        return;
    case CCN_TEA:
        env->tea = val;
        return;
    case CCN_MMUCR:
        old = env->mmucr;
        env->mmucr = val & MMUCR_WMASK & ~MMUCR_TI;
        if (val & MMUCR_TI) {
            cpu_sh4_invalidate_tlb(env);
        } else if ((old ^ env->mmucr) & (MMUCR_AT | MMUCR_SV | MMUCR_ME)) {
            /* Softmmu entries depend on AT, SV and ME */
            tlb_flush(CPU(s->cpu));
        }
        return;
    case CCN_CCR:
        s->ccr = val & ~(CCR_ICI | CCR_OCI);
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
        s->qacr[(addr - CCN_QACR0) / 4] = val & 0x1c;
        return;
    case CCN_PASCR:
        if (val & (1u << 31)) {
            qemu_log_mask(LOG_UNIMP, "sh7785: 32-bit address extended mode "
                          "(PASCR.SE) is not modelled yet\n");
        }
        s->pascr = val;
        return;
    case CCN_RAMCR:
        s->ramcr = val;
        return;
    case CCN_IRMCR:
        s->irmcr = val & 0x1f;
        return;
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
    CPUSH4State *env = &s->cpu->env;

    switch (addr >> 24) {
    case 0x0: /* IC address array */
    case 0x1: /* IC data array */
    case 0x4: /* OC address array */
    case 0x5: /* OC data array */
        return 0;       /* caches are not modelled */
    case 0x2:
        return cpu_sh4_read_mmaped_itlb_addr(env, addr);
    case 0x3:
        return cpu_sh4_read_mmaped_itlb_data(env, addr);
    case 0x6:
        if ((addr & 0x00f00000) == 0) {
            return cpu_sh4_read_mmaped_utlb_addr(env, addr);
        }
        break;
    case 0x7:
        if ((addr & 0x00700000) == 0) {
            return cpu_sh4_read_mmaped_utlb_data(env, addr);
        }
        break;
    }
    qemu_log_mask(LOG_UNIMP, "sh7785: read of array 0x%08" HWADDR_PRIx
                  " (PMB or reserved)\n", addr + 0xf0000000);
    return 0;
}

static void sh7785_mmct_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    SH7785State *s = opaque;
    CPUSH4State *env = &s->cpu->env;

    switch (addr >> 24) {
    case 0x0:
    case 0x1:
    case 0x4:
    case 0x5:
        return;
    case 0x2:
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
        break;
    case 0x7:
        if ((addr & 0x00700000) == 0) {
            cpu_sh4_write_mmaped_utlb_data(env, addr, val);
            return;
        }
        break;
    }
    qemu_log_mask(LOG_UNIMP, "sh7785: write of array 0x%08" HWADDR_PRIx
                  " (PMB or reserved)\n", addr + 0xf0000000);
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

static uint64_t sh7785_regfile_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785State *s = opaque;
    uint32_t a = 0xfc000000 + addr;
    int i = sh7785_reg_find(a);
    uint64_t val;

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
    int i = sh7785_reg_find(a);
    unsigned shift;
    uint64_t mask;

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

void sh7785_set_reg(SH7785State *s, uint32_t addr, uint32_t val)
{
    int i = sh7785_reg_find(addr);

    assert(i >= 0);
    s->regvals[i] = val;
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
    return s;
}

qemu_irq sh7785_irq_pin(SH7785State *s, int n)
{
    return qdev_get_gpio_in_named(DEVICE(s->intc), "irq", n);
}
