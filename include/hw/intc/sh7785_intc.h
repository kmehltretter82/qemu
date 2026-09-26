/*
 * SH7785 interrupt controller (INTC + INTC2)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_SH7785_INTC_H
#define HW_INTC_SH7785_INTC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SH7785_INTC "sh7785-intc"
OBJECT_DECLARE_SIMPLE_TYPE(SH7785IntcState, SH7785_INTC)

/*
 * On-chip interrupt sources, one GPIO input each (named "onchip"), in the
 * default priority order of table 10.13 of the SH7785 hardware manual.
 */
enum {
    SH7785_IRQ_WDT,
    SH7785_IRQ_TUNI0, SH7785_IRQ_TUNI1, SH7785_IRQ_TUNI2, SH7785_IRQ_TICPI2,
    SH7785_IRQ_HUDI,
    SH7785_IRQ_DMINT0, SH7785_IRQ_DMINT1, SH7785_IRQ_DMINT2,
    SH7785_IRQ_DMINT3, SH7785_IRQ_DMINT4, SH7785_IRQ_DMINT5,
    SH7785_IRQ_DMAE0,
    SH7785_IRQ_ERI0, SH7785_IRQ_RXI0, SH7785_IRQ_BRI0, SH7785_IRQ_TXI0,
    SH7785_IRQ_ERI1, SH7785_IRQ_RXI1, SH7785_IRQ_BRI1, SH7785_IRQ_TXI1,
    SH7785_IRQ_DMINT6, SH7785_IRQ_DMINT7, SH7785_IRQ_DMINT8,
    SH7785_IRQ_DMINT9, SH7785_IRQ_DMINT10, SH7785_IRQ_DMINT11,
    SH7785_IRQ_DMAE1,
    SH7785_IRQ_SPII,
    SH7785_IRQ_SCIF2, SH7785_IRQ_SCIF3, SH7785_IRQ_SCIF4, SH7785_IRQ_SCIF5,
    SH7785_IRQ_PCISERR, SH7785_IRQ_PCIINTA, SH7785_IRQ_PCIINTB,
    SH7785_IRQ_PCIINTC, SH7785_IRQ_PCIINTD,
    SH7785_IRQ_PCIERR, SH7785_IRQ_PCIPWD3, SH7785_IRQ_PCIPWD2,
    SH7785_IRQ_PCIPWD1, SH7785_IRQ_PCIPWD0,
    SH7785_IRQ_SIOF,
    SH7785_IRQ_MMCIF_FSTAT, SH7785_IRQ_MMCIF_TRAN, SH7785_IRQ_MMCIF_ERR,
    SH7785_IRQ_MMCIF_FRDY,
    SH7785_IRQ_DU,
    SH7785_IRQ_GACLI, SH7785_IRQ_GAMCI, SH7785_IRQ_GAERI,
    SH7785_IRQ_TUNI3, SH7785_IRQ_TUNI4, SH7785_IRQ_TUNI5,
    SH7785_IRQ_SSI0, SH7785_IRQ_SSI1,
    SH7785_IRQ_HAC0, SH7785_IRQ_HAC1,
    SH7785_IRQ_FLSTE, SH7785_IRQ_FLTEND, SH7785_IRQ_FLTRQ0, SH7785_IRQ_FLTRQ1,
    SH7785_IRQ_GPIOI0, SH7785_IRQ_GPIOI1, SH7785_IRQ_GPIOI2, SH7785_IRQ_GPIOI3,
    SH7785_NR_ONCHIP
};

#define SH7785_NR_IRQ_PINS 8

struct SH7785IntcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem_intc;    /* 0xffd00000 */
    MemoryRegion iomem_uimask;  /* 0xffd30000 */
    MemoryRegion iomem_intc2;   /* 0xffd40000 */

    CPUState *cpu;

    /* INTC */
    uint32_t icr0, icr1, intpri, intreq, intmsk0, intmsk1, intmsk2;
    uint32_t userimask;
    /* INTC2 */
    uint32_t int2pri[10];
    uint32_t int2mskr;
    uint32_t int2gpic;

    bool onchip_level[SH7785_NR_ONCHIP];
    bool irq_level[SH7785_NR_IRQ_PINS];
};

/* Called by the CPU: INTEVT code of the interrupt to take, or -1. */
int sh7785_intc_get_vector(void *opaque, int imask);

#endif
