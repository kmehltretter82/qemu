/*
 * SH7786 interrupt controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_SH7786_INTC_H
#define HW_INTC_SH7786_INTC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SH7786_INTC "sh7786-intc"
OBJECT_DECLARE_SIMPLE_TYPE(SH7786IntcState, SH7786_INTC)

#define SH7786_NR_CPUS      2
#define SH7786_NR_IRQ_PINS  8
#define SH7786_NR_ICI       8

/*
 * On-chip interrupt sources, one GPIO input each (named "onchip"), in
 * event code order (table 10.5 of the SH7786 hardware manual).
 */
enum {
    SH7786_IRQ_WDT,
    SH7786_IRQ_TMU0_0, SH7786_IRQ_TMU0_1, SH7786_IRQ_TMU0_2,
    SH7786_IRQ_TMU0_3,
    SH7786_IRQ_TMU1_0, SH7786_IRQ_TMU1_1, SH7786_IRQ_TMU1_2,
    SH7786_IRQ_DMAC0_0, SH7786_IRQ_DMAC0_1, SH7786_IRQ_DMAC0_2,
    SH7786_IRQ_DMAC0_3, SH7786_IRQ_DMAC0_4, SH7786_IRQ_DMAC0_5,
    SH7786_IRQ_DMAC0_6,
    SH7786_IRQ_HUDI1, SH7786_IRQ_HUDI0,
    SH7786_IRQ_DMAC1_0, SH7786_IRQ_DMAC1_1, SH7786_IRQ_DMAC1_2,
    SH7786_IRQ_DMAC1_3,
    SH7786_IRQ_HPB_0, SH7786_IRQ_HPB_1, SH7786_IRQ_HPB_2,
    SH7786_IRQ_SCIF0_0, SH7786_IRQ_SCIF0_1, SH7786_IRQ_SCIF0_2,
    SH7786_IRQ_SCIF0_3,
    SH7786_IRQ_SCIF1, SH7786_IRQ_TMU2, SH7786_IRQ_TMU3,
    SH7786_IRQ_SCIF2, SH7786_IRQ_SCIF3, SH7786_IRQ_SCIF4, SH7786_IRQ_SCIF5,
    SH7786_IRQ_ETH_0, SH7786_IRQ_ETH_1,
    SH7786_IRQ_PCIEC0_0, SH7786_IRQ_PCIEC0_1, SH7786_IRQ_PCIEC0_2,
    SH7786_IRQ_PCIEC1_0, SH7786_IRQ_PCIEC1_1, SH7786_IRQ_PCIEC1_2,
    SH7786_IRQ_USB,
    SH7786_IRQ_I2C0, SH7786_IRQ_I2C1,
    SH7786_IRQ_DU,
    SH7786_IRQ_SSI0, SH7786_IRQ_SSI1, SH7786_IRQ_SSI2, SH7786_IRQ_SSI3,
    SH7786_IRQ_PCIEC2_0, SH7786_IRQ_PCIEC2_1, SH7786_IRQ_PCIEC2_2,
    SH7786_IRQ_HAC0, SH7786_IRQ_HAC1,
    SH7786_IRQ_FLCTL,
    SH7786_IRQ_HSPI, SH7786_IRQ_GPIO0, SH7786_IRQ_GPIO1, SH7786_IRQ_THERMAL,
    SH7786_NR_ONCHIP
};

/* What env->intc_handle of CPU n points to */
typedef struct SH7786IntcCpu {
    SH7786IntcState *intc;
    int n;
} SH7786IntcCpu;

struct SH7786IntcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;         /* H'FE41 0000, 4 KiB */
    MemoryRegion iomem_uimask;  /* H'FE41 1000 */

    CPUState *cpu[SH7786_NR_CPUS];
    SH7786IntcCpu handle[SH7786_NR_CPUS];

    uint32_t icr0, icr1, intpri, intmsk2, userimask;
    uint32_t intmsk0[SH7786_NR_CPUS], intmsk1[SH7786_NR_CPUS];
    uint32_t intici[SH7786_NR_CPUS], icipri[SH7786_NR_CPUS];
    uint32_t intdistcr[2], int2distcr[4];
    uint32_t int2pri[25];
    uint32_t int2msk[SH7786_NR_CPUS][4];
    uint32_t intack[SH7786_NR_CPUS];

    /* automatic distribution: acknowledged, masked until INTACKCLR */
    bool onchip_acked[SH7786_NR_ONCHIP];
    bool irq_acked[SH7786_NR_IRQ_PINS];

    bool onchip_level[SH7786_NR_ONCHIP];
    bool irq_level[SH7786_NR_IRQ_PINS];
};

/* Called by the CPU: INTEVT code of the interrupt to take, or -1. */
int sh7786_intc_get_vector(void *opaque, int imask);

#endif
