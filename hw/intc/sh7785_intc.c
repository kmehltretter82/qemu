/*
 * SH7785 interrupt controller (INTC + INTC2)
 *
 * Source: SH7785 Hardware Manual REJ09B0261-0100, section 10.
 *
 * On-chip sources have a 5-bit priority in INT2PRI0..9 (H'00/H'01 mask the
 * request) and a mask bit in INT2MSKR. The CPU sees the priority with the
 * lowest bit dropped (10.4.5). IRQ0..7 have a 4-bit priority in INTPRI and a
 * mask bit in INTMSK0. Among requests above SR.IMASK the highest 4-bit level
 * wins; at the same level an external request (IRQ) beats an on-chip one,
 * then the higher 5-bit value, then the default order of table 10.13.
 *
 * Not modelled yet: IRL mode (ICR0.IRLM), NMI, edge-sensed IRQ (ICR1),
 * USERIMASK, INT2B0..7 detail registers. Accesses to them are logged.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/intc/sh7785_intc.h"
#include "hw/core/cpu.h"
#include "target/sh4/cpu.h"
#include "migration/vmstate.h"

typedef struct {
    uint16_t intevt;
    uint8_t pri_reg;    /* INT2PRIn */
    uint8_t pri_shift;  /* 24, 16, 8 or 0 */
    uint8_t mask_bit;   /* INT2MSKR / INT2A0 bit */
} OnchipSource;

#define SRC(ev, reg, field, bit) { ev, reg, 24 - 8 * (field), bit }

/* Table 10.13 order; priority fields from table 10.5, mask bits 10.3.3 */
static const OnchipSource onchip[SH7785_NR_ONCHIP] = {
    [SH7785_IRQ_WDT]     = SRC(0x560, 3, 2, 8),
    [SH7785_IRQ_TUNI0]   = SRC(0x580, 0, 0, 0),
    [SH7785_IRQ_TUNI1]   = SRC(0x5a0, 0, 1, 0),
    [SH7785_IRQ_TUNI2]   = SRC(0x5c0, 0, 2, 0),
    [SH7785_IRQ_TICPI2]  = SRC(0x5e0, 0, 3, 0),
    [SH7785_IRQ_HUDI]    = SRC(0x600, 4, 0, 9),
    [SH7785_IRQ_DMINT0]  = SRC(0x620, 4, 1, 10),
    [SH7785_IRQ_DMINT1]  = SRC(0x640, 4, 1, 10),
    [SH7785_IRQ_DMINT2]  = SRC(0x660, 4, 1, 10),
    [SH7785_IRQ_DMINT3]  = SRC(0x680, 4, 1, 10),
    [SH7785_IRQ_DMINT4]  = SRC(0x6a0, 4, 1, 10),
    [SH7785_IRQ_DMINT5]  = SRC(0x6c0, 4, 1, 10),
    [SH7785_IRQ_DMAE0]   = SRC(0x6e0, 4, 1, 10),
    [SH7785_IRQ_ERI0]    = SRC(0x700, 2, 0, 2),
    [SH7785_IRQ_RXI0]    = SRC(0x720, 2, 0, 2),
    [SH7785_IRQ_BRI0]    = SRC(0x740, 2, 0, 2),
    [SH7785_IRQ_TXI0]    = SRC(0x760, 2, 0, 2),
    [SH7785_IRQ_ERI1]    = SRC(0x780, 2, 1, 3),
    [SH7785_IRQ_RXI1]    = SRC(0x7a0, 2, 1, 3),
    [SH7785_IRQ_BRI1]    = SRC(0x7c0, 2, 1, 3),
    [SH7785_IRQ_TXI1]    = SRC(0x7e0, 2, 1, 3),
    [SH7785_IRQ_DMINT6]  = SRC(0x880, 4, 2, 11),
    [SH7785_IRQ_DMINT7]  = SRC(0x8a0, 4, 2, 11),
    [SH7785_IRQ_DMINT8]  = SRC(0x8c0, 4, 2, 11),
    [SH7785_IRQ_DMINT9]  = SRC(0x8e0, 4, 2, 11),
    [SH7785_IRQ_DMINT10] = SRC(0x900, 4, 2, 11),
    [SH7785_IRQ_DMINT11] = SRC(0x920, 4, 2, 11),
    [SH7785_IRQ_DMAE1]   = SRC(0x940, 4, 2, 11),
    [SH7785_IRQ_SPII]    = SRC(0x960, 7, 1, 21),
    [SH7785_IRQ_SCIF2]   = SRC(0x980, 2, 2, 4),
    [SH7785_IRQ_SCIF3]   = SRC(0x9a0, 2, 3, 5),
    [SH7785_IRQ_SCIF4]   = SRC(0x9c0, 3, 0, 6),
    [SH7785_IRQ_SCIF5]   = SRC(0x9e0, 3, 1, 7),
    [SH7785_IRQ_PCISERR] = SRC(0xa00, 5, 2, 14),
    [SH7785_IRQ_PCIINTA] = SRC(0xa20, 5, 3, 15),
    [SH7785_IRQ_PCIINTB] = SRC(0xa40, 6, 0, 16),
    [SH7785_IRQ_PCIINTC] = SRC(0xa60, 6, 1, 17),
    [SH7785_IRQ_PCIINTD] = SRC(0xa80, 6, 2, 18),
    [SH7785_IRQ_PCIERR]  = SRC(0xaa0, 6, 3, 19),
    [SH7785_IRQ_PCIPWD3] = SRC(0xac0, 6, 3, 19),
    [SH7785_IRQ_PCIPWD2] = SRC(0xac0, 6, 3, 19),
    [SH7785_IRQ_PCIPWD1] = SRC(0xac0, 6, 3, 19),
    [SH7785_IRQ_PCIPWD0] = SRC(0xae0, 6, 3, 19),
    [SH7785_IRQ_SIOF]    = SRC(0xce0, 7, 0, 20),
    [SH7785_IRQ_MMCIF_FSTAT] = SRC(0xd00, 7, 2, 22),
    [SH7785_IRQ_MMCIF_TRAN]  = SRC(0xd20, 7, 2, 22),
    [SH7785_IRQ_MMCIF_ERR]   = SRC(0xd40, 7, 2, 22),
    [SH7785_IRQ_MMCIF_FRDY]  = SRC(0xd60, 7, 2, 22),
    [SH7785_IRQ_DU]      = SRC(0xd80, 9, 0, 27),
    [SH7785_IRQ_GACLI]   = SRC(0xda0, 9, 1, 28),
    [SH7785_IRQ_GAMCI]   = SRC(0xdc0, 9, 1, 28),
    [SH7785_IRQ_GAERI]   = SRC(0xde0, 9, 1, 28),
    [SH7785_IRQ_TUNI3]   = SRC(0xe00, 1, 0, 1),
    [SH7785_IRQ_TUNI4]   = SRC(0xe20, 1, 1, 1),
    [SH7785_IRQ_TUNI5]   = SRC(0xe40, 1, 2, 1),
    [SH7785_IRQ_SSI0]    = SRC(0xe80, 8, 2, 25),
    [SH7785_IRQ_SSI1]    = SRC(0xea0, 8, 3, 26),
    [SH7785_IRQ_HAC0]    = SRC(0xec0, 5, 0, 12),
    [SH7785_IRQ_HAC1]    = SRC(0xee0, 5, 1, 13),
    [SH7785_IRQ_FLSTE]   = SRC(0xf00, 8, 0, 23),
    [SH7785_IRQ_FLTEND]  = SRC(0xf20, 8, 0, 23),
    [SH7785_IRQ_FLTRQ0]  = SRC(0xf40, 8, 0, 23),
    [SH7785_IRQ_FLTRQ1]  = SRC(0xf60, 8, 0, 23),
    [SH7785_IRQ_GPIOI0]  = SRC(0xf80, 8, 1, 24),
    [SH7785_IRQ_GPIOI1]  = SRC(0xfa0, 8, 1, 24),
    [SH7785_IRQ_GPIOI2]  = SRC(0xfc0, 8, 1, 24),
    [SH7785_IRQ_GPIOI3]  = SRC(0xfe0, 8, 1, 24),
};

/* IRQ0..7 INTEVT codes, table 10.13 */
static const uint16_t irq_intevt[SH7785_NR_IRQ_PINS] = {
    0x240, 0x280, 0x2c0, 0x300, 0x340, 0x380, 0x3c0, 0x200,
};

/* Register offsets */
#define ICR0        0x00
#define INTPRI      0x10
#define ICR1        0x1c
#define INTREQ      0x24
#define INTMSK0     0x44
#define INTMSK1     0x48
#define INTMSKCLR0  0x64
#define INTMSKCLR1  0x68
#define NMIFCR      0xc0

#define INT2PRI0    0x00
#define INT2PRI9    0x24
#define INT2A0      0x30
#define INT2A1      0x34
#define INT2MSKR    0x38
#define INT2MSKCR   0x3c
#define INT2B0      0x40
#define INT2B7      0x5c
#define INTMSK2     0x80
#define INTMSKCLR2  0x84
#define INT2GPIC    0x90

/* INT2A0/INT2MSKR bits that exist (bits 29..31 are reserved) */
#define INT2_VALID  0x1fffffff

static unsigned onchip_prio5(SH7785IntcState *s, int i)
{
    return (s->int2pri[onchip[i].pri_reg] >> onchip[i].pri_shift) & 0x1f;
}

static bool onchip_enabled(SH7785IntcState *s, int i)
{
    return !(s->int2mskr & (1u << onchip[i].mask_bit));
}

static unsigned irq_prio(SH7785IntcState *s, int n)
{
    return (s->intpri >> (28 - 4 * n)) & 0xf;
}

static bool irq_enabled(SH7785IntcState *s, int n)
{
    return !(s->intmsk0 & (1u << (31 - n)));
}

/*
 * Best pending request: returns the INTEVT code and its 4-bit level, or -1.
 */
static int sh7785_intc_best(SH7785IntcState *s, int *level)
{
    int best = -1, best_level = 0, best_prio5 = -1;
    bool best_ext = false;
    int i;

    for (i = 0; i < SH7785_NR_IRQ_PINS; i++) {
        int l = irq_prio(s, i);

        if (!s->irq_level[i] || !irq_enabled(s, i) || !l) {
            continue;
        }
        if (l > best_level) {
            best = irq_intevt[i];
            best_level = l;
            best_ext = true;
        }
    }
    for (i = 0; i < SH7785_NR_ONCHIP; i++) {
        int p5, l;

        if (!s->onchip_level[i] || !onchip_enabled(s, i)) {
            continue;
        }
        p5 = onchip_prio5(s, i);
        l = p5 >> 1;
        if (!l) {
            continue;
        }
        /* external wins at equal level; then 5-bit value; then order */
        if (l > best_level ||
            (l == best_level && !best_ext && p5 > best_prio5)) {
            best = onchip[i].intevt;
            best_level = l;
            best_prio5 = p5;
            best_ext = false;
        }
    }
    *level = best_level;
    return best;
}

static void sh7785_intc_update(SH7785IntcState *s)
{
    int level;

    if (!s->cpu) {
        return;
    }
    if (sh7785_intc_best(s, &level) >= 0) {
        cpu_interrupt(s->cpu, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(s->cpu, CPU_INTERRUPT_HARD);
    }
}

int sh7785_intc_get_vector(void *opaque, int imask)
{
    SH7785IntcState *s = opaque;
    int level;
    int ev = sh7785_intc_best(s, &level);

    if (ev < 0 || level <= imask) {
        return -1;
    }
    return ev;
}

static void sh7785_intc_onchip_set(void *opaque, int n, int level)
{
    SH7785IntcState *s = opaque;

    s->onchip_level[n] = level;
    sh7785_intc_update(s);
}

static void sh7785_intc_irq_set(void *opaque, int n, int level)
{
    SH7785IntcState *s = opaque;

    /* Level-sensed low: the board drives 1 while the request is active */
    s->irq_level[n] = level;
    if (level) {
        s->intreq |= 1u << (31 - n);
    } else {
        s->intreq &= ~(1u << (31 - n));
    }
    sh7785_intc_update(s);
}

static uint32_t int2a(SH7785IntcState *s, bool masked_too)
{
    uint32_t v = 0;
    int i;

    for (i = 0; i < SH7785_NR_ONCHIP; i++) {
        if (s->onchip_level[i] && (masked_too || onchip_enabled(s, i))) {
            v |= 1u << onchip[i].mask_bit;
        }
    }
    return v;
}

static uint64_t sh7785_intc_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785IntcState *s = opaque;

    switch (addr) {
    case ICR0:
        return s->icr0;
    case ICR1:
        return s->icr1;
    case INTPRI:
        return s->intpri;
    case INTREQ:
        return s->intreq;
    case INTMSK0:
        return s->intmsk0;
    case INTMSK1:
        return s->intmsk1 | 0x3f000000;
    case INTMSKCLR0:
    case INTMSKCLR1:
    case NMIFCR:
        return 0;
    }
    qemu_log_mask(LOG_UNIMP, "sh7785-intc: read of unknown INTC register "
                  "0x%" HWADDR_PRIx "\n", addr);
    return 0;
}

static void sh7785_intc_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    SH7785IntcState *s = opaque;

    switch (addr) {
    case ICR0:
        /* IRLM0/IRLM1 = 1 select IRQ mode for IRQ3-0 / IRQ7-4 (10.3.1) */
        if ((val & 0x00c00000) != 0x00c00000) {
            qemu_log_mask(LOG_UNIMP, "sh7785-intc: IRL mode (ICR0.IRLMn = 0)"
                          " not modelled\n");
        }
        s->icr0 = val;
        return;
    case ICR1:
        if (val & 0xffff0000) {
            qemu_log_mask(LOG_UNIMP, "sh7785-intc: edge-sensed IRQ not "
                          "modelled (ICR1 0x%" PRIx64 ")\n", val);
        }
        s->icr1 = val;
        return;
    case INTPRI:
        s->intpri = val;
        break;
    case INTREQ:
        /* Only edge-sensed requests are cleared by writing 0 */
        return;
    case INTMSK0:
        s->intmsk0 |= val & 0xff000000;
        break;
    case INTMSKCLR0:
        s->intmsk0 &= ~(val & 0xff000000);
        break;
    case INTMSK1:
        s->intmsk1 |= val & 0xc0000000;
        break;
    case INTMSKCLR1:
        s->intmsk1 &= ~(val & 0xc0000000);
        break;
    case NMIFCR:
        return;
    default:
        qemu_log_mask(LOG_UNIMP, "sh7785-intc: write of unknown INTC register "
                      "0x%" HWADDR_PRIx "\n", addr);
        return;
    }
    sh7785_intc_update(s);
}

static uint64_t sh7785_intc2_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785IntcState *s = opaque;

    if (addr <= INT2PRI9 && !(addr & 3)) {
        return s->int2pri[addr / 4];
    }
    switch (addr) {
    case INT2A0:
        return int2a(s, true);
    case INT2A1:
        return int2a(s, false);
    case INT2MSKR:
        return s->int2mskr;
    case INT2MSKCR:
        return 0;
    case INTMSK2:
        return s->intmsk2;
    case INTMSKCLR2:
        return 0;
    case INT2GPIC:
        return s->int2gpic;
    }
    if (addr >= INT2B0 && addr <= INT2B7) {
        qemu_log_mask(LOG_UNIMP, "sh7785-intc: INT2B%d not modelled\n",
                      (int)(addr - INT2B0) / 4);
        return 0;
    }
    qemu_log_mask(LOG_UNIMP, "sh7785-intc: read of unknown INTC2 register "
                  "0x%" HWADDR_PRIx "\n", addr);
    return 0;
}

static void sh7785_intc2_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    SH7785IntcState *s = opaque;

    if (addr <= INT2PRI9 && !(addr & 3)) {
        s->int2pri[addr / 4] = val & 0x1f1f1f1f;
        sh7785_intc_update(s);
        return;
    }
    switch (addr) {
    case INT2MSKR:
        s->int2mskr |= val & INT2_VALID;
        break;
    case INT2MSKCR:
        s->int2mskr &= ~(val & INT2_VALID);
        break;
    case INTMSK2:
        s->intmsk2 |= val;
        break;
    case INTMSKCLR2:
        s->intmsk2 &= ~val;
        break;
    case INT2GPIC:
        s->int2gpic = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sh7785-intc: write of unknown INTC2 "
                      "register 0x%" HWADDR_PRIx "\n", addr);
        return;
    }
    sh7785_intc_update(s);
}

static uint64_t sh7785_uimask_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785IntcState *s = opaque;

    return addr == 0 ? s->userimask : 0;
}

static void sh7785_uimask_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    SH7785IntcState *s = opaque;

    /* Writes need H'A5 in bits 31:24 (10.3.2) */
    if (addr != 0 || (val >> 24) != 0xa5) {
        return;
    }
    s->userimask = val & 0xf0;
    if (s->userimask) {
        qemu_log_mask(LOG_UNIMP, "sh7785-intc: USERIMASK not modelled\n");
    }
}

static const MemoryRegionOps sh7785_intc_ops = {
    .read = sh7785_intc_read,
    .write = sh7785_intc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps sh7785_intc2_ops = {
    .read = sh7785_intc2_read,
    .write = sh7785_intc2_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps sh7785_uimask_ops = {
    .read = sh7785_uimask_read,
    .write = sh7785_uimask_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void sh7785_intc_reset_hold(Object *obj, ResetType type)
{
    SH7785IntcState *s = SH7785_INTC(obj);

    /* Initial values from the register descriptions in 10.3 */
    s->icr0 = 0;
    s->icr1 = 0;
    s->intpri = 0;
    s->intreq = 0;
    s->intmsk0 = 0xff000000;
    s->intmsk1 = 0xc0000000;    /* bits 29:24 always read as 1 */
    s->intmsk2 = 0;
    s->userimask = 0;
    memset(s->int2pri, 0, sizeof(s->int2pri));
    s->int2mskr = 0xffffffff;  /* reserved bits 31:29 read as 1 */
    s->int2gpic = 0;
    sh7785_intc_update(s);
}

static void sh7785_intc_init(Object *obj)
{
    SH7785IntcState *s = SH7785_INTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem_intc, obj, &sh7785_intc_ops, s,
                          "sh7785-intc", 0x100);
    memory_region_init_io(&s->iomem_uimask, obj, &sh7785_uimask_ops, s,
                          "sh7785-intc-uimask", 0x4);
    memory_region_init_io(&s->iomem_intc2, obj, &sh7785_intc2_ops, s,
                          "sh7785-intc2", 0x100);
    sysbus_init_mmio(sbd, &s->iomem_intc);
    sysbus_init_mmio(sbd, &s->iomem_uimask);
    sysbus_init_mmio(sbd, &s->iomem_intc2);
    qdev_init_gpio_in_named(DEVICE(obj), sh7785_intc_onchip_set, "onchip",
                            SH7785_NR_ONCHIP);
    qdev_init_gpio_in_named(DEVICE(obj), sh7785_intc_irq_set, "irq",
                            SH7785_NR_IRQ_PINS);
}

static const Property sh7785_intc_properties[] = {
    DEFINE_PROP_LINK("cpu", SH7785IntcState, cpu, TYPE_CPU, CPUState *),
};

static void sh7785_intc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = sh7785_intc_reset_hold;
    device_class_set_props(dc, sh7785_intc_properties);
    dc->user_creatable = false;
}

static const TypeInfo sh7785_intc_info = {
    .name = TYPE_SH7785_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SH7785IntcState),
    .instance_init = sh7785_intc_init,
    .class_init = sh7785_intc_class_init,
};

static void sh7785_intc_register_types(void)
{
    type_register_static(&sh7785_intc_info);
}

type_init(sh7785_intc_register_types)
