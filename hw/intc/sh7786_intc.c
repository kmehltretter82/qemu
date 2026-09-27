/*
 * SH7786 interrupt controller
 *
 * Source: SH7786 Hardware Manual REJ09B0501-0100, section 10.
 *
 * Two CPUs share one INTC. On-chip sources have a 5-bit priority in
 * INT2PRI0..24 (H'00/H'01 mask the request; the CPU sees the priority with
 * the lowest bit dropped) and, per CPU, a mask bit in CnINT2MSKR0..3.
 * IRQ0..7 have a 4-bit priority in INTPRI and a per-CPU mask in
 * CnINTMSK0. Each CPU also has eight inter-CPU sources: CnINTICI fields
 * request them (write 1 to set, CnINTICICLR to clear) and CnICIPRI fields
 * give their 4-bit level (same set/clear pair), with event codes H'F00 to
 * H'FE0.
 *
 * Distribution (10.3.7, 10.5.3): with the source's bit in INTDISTCR0 or
 * INT2DISTCRn clear (fixed distribution) a request goes to every CPU that
 * has it unmasked. With the bit set (automatic distribution) the first CPU
 * to acknowledge it gets INTACK = 1 and the source is then masked for all
 * CPUs until software writes its event code to INTACKCLR; a CPU that
 * acknowledges it later reads INTACK = 0.
 *
 * Among requests above SR.IMASK the highest 4-bit level wins; at the same
 * level an IRQ beats an inter-CPU request, which beats an on-chip one;
 * on-chip requests then compare their 5-bit value, then event code order.
 * TODO(manual): the default order among equal levels is taken from the
 * SH7785 (table 10.13 there); the SH7786 manual was not checked for it.
 *
 * Not modelled yet: IRL mode (ICR0.IRLM), NMI, edge-sensed IRQ (ICR1),
 * USERIMASK, the INT2A/INT2B source registers and the ACKMASK registers
 * are read as their plain meaning without extra detail. Accesses to
 * unknown registers are logged.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/intc/sh7786_intc.h"
#include "hw/core/cpu.h"
#include "target/sh4/cpu.h"

typedef struct {
    uint16_t intevt;
    uint8_t pri_reg;    /* INT2PRIn */
    uint8_t pri_shift;  /* 24, 16, 8 or 0 */
    uint8_t msk_reg;    /* CnINT2MSKRn, INT2DISTCRn */
    uint8_t msk_bit;
} OnchipSource;

#define SRC(ev, reg, field, mreg, bit) { ev, reg, 24 - 8 * (field), mreg, bit }

/* INT2PRI fields: table 10.5; mask and distribution bits: 10.3.8/10.3.7 */
static const OnchipSource onchip[SH7786_NR_ONCHIP] = {
    [SH7786_IRQ_WDT]      = SRC(0x3e0, 0, 3, 0, 0),
    [SH7786_IRQ_TMU0_0]   = SRC(0x400, 1, 0, 1, 31),
    [SH7786_IRQ_TMU0_1]   = SRC(0x420, 1, 1, 1, 30),
    [SH7786_IRQ_TMU0_2]   = SRC(0x440, 1, 2, 1, 29),
    [SH7786_IRQ_TMU0_3]   = SRC(0x460, 1, 3, 1, 28),
    [SH7786_IRQ_TMU1_0]   = SRC(0x480, 2, 0, 1, 27),
    [SH7786_IRQ_TMU1_1]   = SRC(0x4a0, 2, 1, 1, 26),
    [SH7786_IRQ_TMU1_2]   = SRC(0x4c0, 2, 2, 1, 25),
    [SH7786_IRQ_DMAC0_0]  = SRC(0x500, 3, 0, 1, 23),
    [SH7786_IRQ_DMAC0_1]  = SRC(0x520, 3, 1, 1, 22),
    [SH7786_IRQ_DMAC0_2]  = SRC(0x540, 3, 2, 1, 21),
    [SH7786_IRQ_DMAC0_3]  = SRC(0x560, 3, 3, 1, 20),
    [SH7786_IRQ_DMAC0_4]  = SRC(0x580, 4, 0, 1, 19),
    [SH7786_IRQ_DMAC0_5]  = SRC(0x5a0, 4, 1, 1, 18),
    [SH7786_IRQ_DMAC0_6]  = SRC(0x5c0, 4, 2, 1, 17),
    [SH7786_IRQ_HUDI1]    = SRC(0x5e0, 4, 3, 1, 16),
    [SH7786_IRQ_HUDI0]    = SRC(0x600, 5, 0, 1, 15),
    [SH7786_IRQ_DMAC1_0]  = SRC(0x620, 5, 1, 1, 14),
    [SH7786_IRQ_DMAC1_1]  = SRC(0x640, 5, 2, 1, 13),
    [SH7786_IRQ_DMAC1_2]  = SRC(0x660, 5, 3, 1, 12),
    [SH7786_IRQ_DMAC1_3]  = SRC(0x680, 6, 0, 1, 11),
    [SH7786_IRQ_HPB_0]    = SRC(0x6a0, 6, 1, 1, 10),
    [SH7786_IRQ_HPB_1]    = SRC(0x6c0, 6, 2, 1, 9),
    [SH7786_IRQ_HPB_2]    = SRC(0x6e0, 6, 3, 1, 8),
    [SH7786_IRQ_SCIF0_0]  = SRC(0x700, 7, 0, 1, 7),
    [SH7786_IRQ_SCIF0_1]  = SRC(0x720, 7, 1, 1, 6),
    [SH7786_IRQ_SCIF0_2]  = SRC(0x740, 7, 2, 1, 5),
    [SH7786_IRQ_SCIF0_3]  = SRC(0x760, 7, 3, 1, 4),
    [SH7786_IRQ_SCIF1]    = SRC(0x780, 8, 0, 1, 3),
    [SH7786_IRQ_TMU2]     = SRC(0x7a0, 8, 1, 1, 2),
    [SH7786_IRQ_TMU3]     = SRC(0x7c0, 8, 2, 1, 1),
    [SH7786_IRQ_SCIF2]    = SRC(0x840, 9, 2, 2, 29),
    [SH7786_IRQ_SCIF3]    = SRC(0x860, 9, 3, 2, 28),
    [SH7786_IRQ_SCIF4]    = SRC(0x880, 10, 0, 2, 27),
    [SH7786_IRQ_SCIF5]    = SRC(0x8a0, 10, 1, 2, 26),
    [SH7786_IRQ_ETH_0]    = SRC(0x8c0, 10, 2, 2, 25),
    [SH7786_IRQ_ETH_1]    = SRC(0x8e0, 10, 3, 2, 24),
    [SH7786_IRQ_PCIEC0_0] = SRC(0xae0, 14, 3, 2, 8),
    [SH7786_IRQ_PCIEC0_1] = SRC(0xb00, 15, 0, 2, 7),
    [SH7786_IRQ_PCIEC0_2] = SRC(0xb20, 15, 1, 2, 6),
    [SH7786_IRQ_PCIEC1_0] = SRC(0xb40, 15, 2, 2, 5),
    [SH7786_IRQ_PCIEC1_1] = SRC(0xb60, 15, 3, 2, 4),
    [SH7786_IRQ_PCIEC1_2] = SRC(0xb80, 16, 0, 2, 3),
    [SH7786_IRQ_USB]      = SRC(0xba0, 16, 1, 2, 2),
    [SH7786_IRQ_I2C0]     = SRC(0xcc0, 18, 2, 3, 25),
    [SH7786_IRQ_I2C1]     = SRC(0xce0, 18, 3, 3, 24),
    [SH7786_IRQ_DU]       = SRC(0xd00, 19, 0, 3, 23),
    [SH7786_IRQ_SSI0]     = SRC(0xd20, 19, 1, 3, 22),
    [SH7786_IRQ_SSI1]     = SRC(0xd40, 19, 2, 3, 21),
    [SH7786_IRQ_SSI2]     = SRC(0xd60, 19, 3, 3, 20),
    [SH7786_IRQ_SSI3]     = SRC(0xd80, 20, 0, 3, 19),
    [SH7786_IRQ_PCIEC2_0] = SRC(0xda0, 20, 1, 3, 18),
    [SH7786_IRQ_PCIEC2_1] = SRC(0xdc0, 20, 2, 3, 17),
    [SH7786_IRQ_PCIEC2_2] = SRC(0xde0, 20, 3, 3, 16),
    [SH7786_IRQ_HAC0]     = SRC(0xe00, 21, 0, 3, 15),
    [SH7786_IRQ_HAC1]     = SRC(0xe20, 21, 1, 3, 14),
    [SH7786_IRQ_FLCTL]    = SRC(0xe40, 21, 2, 3, 13),
    [SH7786_IRQ_HSPI]     = SRC(0xe80, 22, 0, 3, 11),
    [SH7786_IRQ_GPIO0]    = SRC(0xea0, 22, 1, 3, 10),
    [SH7786_IRQ_GPIO1]    = SRC(0xec0, 22, 2, 3, 9),
    [SH7786_IRQ_THERMAL]  = SRC(0xee0, 22, 3, 3, 8),
};

/* IRQ0..7 event codes (table 10.4) */
static const uint16_t irq_intevt[SH7786_NR_IRQ_PINS] = {
    0x200, 0x240, 0x280, 0x2c0, 0x300, 0x340, 0x380, 0x3c0,
};

/* Register offsets from H'FE41 0000 (10.3) */
#define ICR0        0x000
#define INTPRI      0x010
#define ICR1        0x01c
#define INTREQ      0x024
#define CNINTMSK0   0x030   /* + 4n */
#define CNINTMSK1   0x040
#define CNINTMSKCLR0 0x050
#define CNINTMSKCLR1 0x060
#define INTMSK2     0x068
#define INTMSKCLR2  0x06c
#define CNINTICI    0x070
#define CNINTICICLR 0x080
#define CNICIPRI    0x090
#define CNICIPRICLR 0x0a0
#define INTDISTCR0  0x0b0
#define INTDISTCR1  0x0b4
#define INTACK      0x0b8
#define INTACKCLR   0x0bc
#define NMIFCR      0x0c0
#define NMISET      0x0c4
#define INT2PRI0    0x800
#define INT2PRI24   0x860
#define INT2DISTCR0 0x900
#define INT2DISTCR3 0x90c
#define CNINT2A0    0xa00   /* + 0x100 n, + 4 m */
#define CNINT2A1    0xa10
#define CNINT2MSKR  0xa20
#define CNINT2MSKCR 0xa30

static unsigned onchip_prio5(SH7786IntcState *s, int i)
{
    return (s->int2pri[onchip[i].pri_reg] >> onchip[i].pri_shift) & 0x1f;
}

static bool onchip_auto(SH7786IntcState *s, int i)
{
    return s->int2distcr[onchip[i].msk_reg] & (1u << onchip[i].msk_bit);
}

static bool onchip_unmasked(SH7786IntcState *s, int c, int i)
{
    return !(s->int2msk[c][onchip[i].msk_reg] & (1u << onchip[i].msk_bit));
}

static bool onchip_pending(SH7786IntcState *s, int c, int i)
{
    return s->onchip_level[i] && onchip_unmasked(s, c, i) &&
           !s->onchip_acked[i];
}

static unsigned irq_prio(SH7786IntcState *s, int n)
{
    return (s->intpri >> (28 - 4 * n)) & 0xf;
}

/* INTDISTCR0: bits 31 to 24 are DIST7 to DIST0 (10.3.7) */
static bool irq_auto(SH7786IntcState *s, int n)
{
    return s->intdistcr[0] & (1u << (24 + n));
}

static bool irq_pending(SH7786IntcState *s, int c, int n)
{
    return s->irq_level[n] && !(s->intmsk0[c] & (1u << (31 - n))) &&
           !s->irq_acked[n];
}

enum { KIND_IRQ, KIND_ICI, KIND_ONCHIP };

/*
 * Best pending request for CPU c: the event code, its 4-bit level and
 * what it is (kind, index), or -1.
 */
static int sh7786_intc_best(SH7786IntcState *s, int c, int *level,
                            int *kind, int *index)
{
    int best = -1, best_level = 0, best_prio5 = -1, best_kind = KIND_ONCHIP;
    int i;

    for (i = 0; i < SH7786_NR_IRQ_PINS; i++) {
        int l = irq_prio(s, i);

        if (l && irq_pending(s, c, i) && l > best_level) {
            best = irq_intevt[i];
            best_level = l;
            best_kind = KIND_IRQ;
            *index = i;
        }
    }
    for (i = 0; i < SH7786_NR_ICI; i++) {
        int l = (s->icipri[c] >> (4 * i)) & 0xf;

        if (!((s->intici[c] >> (4 * i)) & 0xf) || !l) {
            continue;
        }
        if (l > best_level || (l == best_level && best_kind == KIND_ONCHIP)) {
            best = 0xf00 + 0x20 * i;
            best_level = l;
            best_kind = KIND_ICI;
            *index = i;
        }
    }
    for (i = 0; i < SH7786_NR_ONCHIP; i++) {
        int p5, l;

        if (!onchip_pending(s, c, i)) {
            continue;
        }
        p5 = onchip_prio5(s, i);
        l = p5 >> 1;
        if (!l) {
            continue;
        }
        if (l > best_level ||
            (l == best_level && best_kind == KIND_ONCHIP && p5 > best_prio5)) {
            best = onchip[i].intevt;
            best_level = l;
            best_prio5 = p5;
            best_kind = KIND_ONCHIP;
            *index = i;
        }
    }
    *level = best_level;
    *kind = best_kind;
    return best;
}

static void sh7786_intc_update(SH7786IntcState *s)
{
    for (int c = 0; c < SH7786_NR_CPUS; c++) {
        int level, kind, index;

        if (!s->cpu[c]) {
            continue;
        }
        if (sh7786_intc_best(s, c, &level, &kind, &index) >= 0) {
            cpu_interrupt(s->cpu[c], CPU_INTERRUPT_HARD);
        } else {
            cpu_reset_interrupt(s->cpu[c], CPU_INTERRUPT_HARD);
        }
    }
}

int sh7786_intc_get_vector(void *opaque, int imask)
{
    SH7786IntcCpu *h = opaque;
    SH7786IntcState *s = h->intc;
    int level, kind, index = 0;
    int ev = sh7786_intc_best(s, h->n, &level, &kind, &index);

    if (ev < 0 || level <= imask) {
        return -1;
    }
    /* Acknowledge: automatically distributed sources become exclusive */
    s->intack[h->n] = 1;
    if (kind == KIND_ONCHIP && onchip_auto(s, index)) {
        s->onchip_acked[index] = true;
        sh7786_intc_update(s);
    } else if (kind == KIND_IRQ && irq_auto(s, index)) {
        s->irq_acked[index] = true;
        sh7786_intc_update(s);
    }
    return ev;
}

static void sh7786_intc_onchip_set(void *opaque, int n, int level)
{
    SH7786IntcState *s = opaque;

    s->onchip_level[n] = level;
    sh7786_intc_update(s);
}

static void sh7786_intc_irq_set(void *opaque, int n, int level)
{
    SH7786IntcState *s = opaque;

    s->irq_level[n] = level;
    sh7786_intc_update(s);
}

static uint32_t int2a(SH7786IntcState *s, int c, int reg, bool masked_too)
{
    uint32_t v = 0;

    for (int i = 0; i < SH7786_NR_ONCHIP; i++) {
        if (onchip[i].msk_reg == reg && s->onchip_level[i] &&
            (masked_too || onchip_unmasked(s, c, i))) {
            v |= 1u << onchip[i].msk_bit;
        }
    }
    return v;
}

static void intackclr(SH7786IntcState *s, uint32_t ev)
{
    for (int i = 0; i < SH7786_NR_ONCHIP; i++) {
        if (onchip[i].intevt == ev) {
            s->onchip_acked[i] = false;
        }
    }
    for (int i = 0; i < SH7786_NR_IRQ_PINS; i++) {
        if (irq_intevt[i] == ev) {
            s->irq_acked[i] = false;
        }
    }
}

static int this_cpu(SH7786IntcState *s)
{
    for (int c = 0; c < SH7786_NR_CPUS; c++) {
        if (current_cpu && current_cpu == s->cpu[c]) {
            return c;
        }
    }
    return 0;
}

static uint64_t sh7786_intc_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7786IntcState *s = opaque;
    int c = (addr >> 2) & 3;

    switch (addr) {
    case ICR0:
        return s->icr0;
    case ICR1:
        return s->icr1;
    case INTPRI:
        return s->intpri;
    case INTREQ: {
        uint32_t v = 0;

        for (int i = 0; i < SH7786_NR_IRQ_PINS; i++) {
            v |= s->irq_level[i] ? 1u << (31 - i) : 0;
        }
        return v;
    }
    case CNINTMSK0 ... CNINTMSK0 + 4:
        return s->intmsk0[c];
    case CNINTMSK1 ... CNINTMSK1 + 4:
        return s->intmsk1[c];
    case INTMSK2:
        return s->intmsk2;
    case CNINTICI ... CNINTICI + 4:
        return s->intici[c];
    case CNICIPRI ... CNICIPRI + 4:
        return s->icipri[c];
    case INTDISTCR0:
        return s->intdistcr[0];
    case INTDISTCR1:
        return s->intdistcr[1];
    case INTACK: {
        /* one address, each CPU reads its own value (10.3.8) */
        int me = this_cpu(s);
        uint32_t v = s->intack[me];

        s->intack[me] = 0;
        return v;
    }
    case CNINTMSKCLR0 ... CNINTMSKCLR0 + 4:
    case CNINTMSKCLR1 ... CNINTMSKCLR1 + 4:
    case INTMSKCLR2:
    case CNINTICICLR ... CNINTICICLR + 4:
    case CNICIPRICLR ... CNICIPRICLR + 4:
    case INTACKCLR:
    case NMIFCR:
    case NMISET:
        return 0;
    case INT2PRI0 ... INT2PRI24:
        return s->int2pri[(addr - INT2PRI0) / 4];
    case INT2DISTCR0 ... INT2DISTCR3:
        return s->int2distcr[(addr - INT2DISTCR0) / 4];
    }
    if (addr >= CNINT2A0 && addr < CNINT2A0 + 0x100 * SH7786_NR_CPUS) {
        int cpu = (addr - CNINT2A0) >> 8, off = addr & 0xff;
        int reg = (off >> 2) & 3;

        switch (off & ~0xf) {
        case CNINT2A0 & 0xff:
            return int2a(s, cpu, reg, true);
        case CNINT2A1 & 0xff:
            return int2a(s, cpu, reg, false);
        case CNINT2MSKR & 0xff:
            return s->int2msk[cpu][reg];
        case CNINT2MSKCR & 0xff:
            return 0;
        }
    }
    qemu_log_mask(LOG_UNIMP, "sh7786-intc: read of unknown register "
                  "H'FE41%04" HWADDR_PRIx "\n", addr);
    return 0;
}

static void sh7786_intc_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    SH7786IntcState *s = opaque;
    int c = (addr >> 2) & 3;

    switch (addr) {
    case ICR0:
        /* IRLM0/IRLM1 = 1 select IRQ mode for IRQ3-0 / IRQ7-4 */
        if ((val & 0x00c00000) != 0x00c00000) {
            qemu_log_mask(LOG_UNIMP, "sh7786-intc: IRL mode (ICR0.IRLMn = 0)"
                          " not modelled\n");
        }
        s->icr0 = val;
        return;
    case ICR1:
        if (val) {
            qemu_log_mask(LOG_UNIMP, "sh7786-intc: edge-sensed IRQ not "
                          "modelled (ICR1 0x%" PRIx64 ")\n", val);
        }
        s->icr1 = val;
        return;
    case INTPRI:
        s->intpri = val;
        break;
    case INTREQ:
        return;
    case CNINTMSK0 ... CNINTMSK0 + 4:
        s->intmsk0[c] |= val & 0xff000000;
        break;
    case CNINTMSKCLR0 ... CNINTMSKCLR0 + 4:
        s->intmsk0[c] &= ~(val & 0xff000000);
        break;
    case CNINTMSK1 ... CNINTMSK1 + 4:
        s->intmsk1[c] |= val;
        break;
    case CNINTMSKCLR1 ... CNINTMSKCLR1 + 4:
        s->intmsk1[c] &= ~val;
        break;
    case INTMSK2:
        s->intmsk2 |= val;
        break;
    case INTMSKCLR2:
        s->intmsk2 &= ~val;
        break;
    case CNINTICI ... CNINTICI + 4:
        s->intici[c] |= val;
        break;
    case CNINTICICLR ... CNINTICICLR + 4:
        s->intici[c] &= ~val;
        break;
    case CNICIPRI ... CNICIPRI + 4:
        s->icipri[c] |= val;
        break;
    case CNICIPRICLR ... CNICIPRICLR + 4:
        s->icipri[c] &= ~val;
        break;
    case INTDISTCR0:
        s->intdistcr[0] = val & 0xff000000;
        break;
    case INTDISTCR1:
        s->intdistcr[1] = val & 0xc0000000;
        break;
    case INTACKCLR:
        intackclr(s, val & 0x3fff);
        break;
    case NMIFCR:
    case NMISET:
        return;
    case INT2PRI0 ... INT2PRI24:
        s->int2pri[(addr - INT2PRI0) / 4] = val & 0x1f1f1f1f;
        break;
    case INT2DISTCR0 ... INT2DISTCR3:
        s->int2distcr[(addr - INT2DISTCR0) / 4] = val;
        break;
    default:
        if (addr >= CNINT2A0 && addr < CNINT2A0 + 0x100 * SH7786_NR_CPUS) {
            int cpu = (addr - CNINT2A0) >> 8, off = addr & 0xff;
            int reg = (off >> 2) & 3;

            if ((off & ~0xf) == (CNINT2MSKR & 0xff)) {
                s->int2msk[cpu][reg] |= val;
                break;
            }
            if ((off & ~0xf) == (CNINT2MSKCR & 0xff)) {
                s->int2msk[cpu][reg] &= ~val;
                break;
            }
        }
        qemu_log_mask(LOG_UNIMP, "sh7786-intc: write 0x%" PRIx64 " to "
                      "unknown register H'FE41%04" HWADDR_PRIx "\n", val, addr);
        return;
    }
    sh7786_intc_update(s);
}

static uint64_t sh7786_uimask_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7786IntcState *s = opaque;

    return addr == 0 ? s->userimask : 0;
}

static void sh7786_uimask_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    SH7786IntcState *s = opaque;

    /* Writes need H'A5 in bits 31:24 */
    if (addr != 0 || (val >> 24) != 0xa5) {
        return;
    }
    s->userimask = val & 0xf0;
    if (s->userimask) {
        qemu_log_mask(LOG_UNIMP, "sh7786-intc: USERIMASK not modelled\n");
    }
}

static const MemoryRegionOps sh7786_intc_ops = {
    .read = sh7786_intc_read,
    .write = sh7786_intc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps sh7786_uimask_ops = {
    .read = sh7786_uimask_read,
    .write = sh7786_uimask_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void sh7786_intc_reset_hold(Object *obj, ResetType type)
{
    SH7786IntcState *s = SH7786_INTC(obj);

    s->icr0 = s->icr1 = s->intpri = s->intmsk2 = s->userimask = 0;
    for (int c = 0; c < SH7786_NR_CPUS; c++) {
        s->intmsk0[c] = 0xff000000;
        s->intmsk1[c] = 0;
        s->intici[c] = 0;
        s->icipri[c] = 0;
        s->intack[c] = 0;
        for (int r = 0; r < 4; r++) {
            s->int2msk[c][r] = 0xffffffff;
        }
    }
    memset(s->intdistcr, 0, sizeof(s->intdistcr));
    memset(s->int2distcr, 0, sizeof(s->int2distcr));
    memset(s->int2pri, 0, sizeof(s->int2pri));
    memset(s->onchip_acked, 0, sizeof(s->onchip_acked));
    memset(s->irq_acked, 0, sizeof(s->irq_acked));
    sh7786_intc_update(s);
}

static void sh7786_intc_init(Object *obj)
{
    SH7786IntcState *s = SH7786_INTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &sh7786_intc_ops, s,
                          "sh7786-intc", 0x1000);
    memory_region_init_io(&s->iomem_uimask, obj, &sh7786_uimask_ops, s,
                          "sh7786-intc-uimask", 0x4);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->iomem_uimask);
    qdev_init_gpio_in_named(DEVICE(obj), sh7786_intc_onchip_set, "onchip",
                            SH7786_NR_ONCHIP);
    qdev_init_gpio_in_named(DEVICE(obj), sh7786_intc_irq_set, "irq",
                            SH7786_NR_IRQ_PINS);
    for (int c = 0; c < SH7786_NR_CPUS; c++) {
        s->handle[c] = (SH7786IntcCpu) { s, c };
    }
}

static const Property sh7786_intc_properties[] = {
    DEFINE_PROP_LINK("cpu0", SH7786IntcState, cpu[0], TYPE_CPU, CPUState *),
    DEFINE_PROP_LINK("cpu1", SH7786IntcState, cpu[1], TYPE_CPU, CPUState *),
};

static void sh7786_intc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = sh7786_intc_reset_hold;
    device_class_set_props(dc, sh7786_intc_properties);
    dc->user_creatable = false;
}

static const TypeInfo sh7786_intc_info = {
    .name = TYPE_SH7786_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SH7786IntcState),
    .instance_init = sh7786_intc_init,
    .class_init = sh7786_intc_class_init,
};

static void sh7786_intc_register_types(void)
{
    type_register_static(&sh7786_intc_info);
}

type_init(sh7786_intc_register_types)
