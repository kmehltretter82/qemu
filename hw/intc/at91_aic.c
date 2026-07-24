/*
 * Atmel/Microchip AT91 Advanced Interrupt Controller (AIC).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/intc/at91_aic.h"
#include "trace.h"

#define AIC_SMR(n)  (0x000 + (n) * 4)   /* Source Mode 0..31   */
#define AIC_SVR(n)  (0x080 + (n) * 4)   /* Source Vector 0..31 */
#define AIC_IVR     0x100               /* Interrupt Vector        */
#define AIC_FVR     0x104               /* FIQ Vector              */
#define AIC_ISR     0x108               /* Interrupt Status        */
#define AIC_IPR     0x10C               /* Interrupt Pending       */
#define AIC_IMR     0x110               /* Interrupt Mask          */
#define AIC_CISR    0x114               /* Core Interrupt Status   */
#define AIC_IECR    0x120               /* Interrupt Enable Cmd    */
#define AIC_IDCR    0x124               /* Interrupt Disable Cmd   */
#define AIC_ICCR    0x128               /* Interrupt Clear Cmd     */
#define AIC_ISCR    0x12C               /* Interrupt Set Cmd       */
#define AIC_EOICR   0x130               /* End Of Interrupt Cmd    */
#define AIC_SPU     0x134               /* Spurious Vector         */
#define AIC_DCR     0x138               /* Debug Control           */
#define AIC_FFER    0x140               /* Fast Forcing Enable     */
#define AIC_FFDR    0x144               /* Fast Forcing Disable    */
#define AIC_FFSR    0x148               /* Fast Forcing Status     */

#define AIC_SMR_PRIOR(smr)   ((smr) & 0x7)
#define AIC_SMR_SRCTYPE(smr) (((smr) >> 5) & 0x3)
/* SRCTYPE low bit selects edge (1) vs level (0) sensitivity. */
#define AIC_SRCTYPE_IS_EDGE(smr) (AIC_SMR_SRCTYPE(smr) & 0x1)

#define AIC_NUM_IRQ   32
#define AIC_STACK_SZ  8


struct AT91AicState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;       /* nIRQ -> CPU */
    qemu_irq fiq;       /* nFIQ -> CPU */

    uint32_t smr[AIC_NUM_IRQ];
    uint32_t svr[AIC_NUM_IRQ];
    uint32_t imr;       /* mask (IECR/IDCR)     */
    uint32_t raw;       /* raw input line levels */
    uint32_t edge;      /* latched edge pendings */
    uint32_t ffsr;      /* fast-forcing-to-FIQ mask */
    uint32_t isr;       /* current interrupt number */
    uint32_t spu;       /* spurious vector */
    uint32_t dcr;       /* debug control */

    int prio_stack[AIC_STACK_SZ];
    int sp;

    bool irq_asserted;
    bool fiq_asserted;
};

/* Current pending state per source: level sources follow the input line,
 * edge sources use the latched bit. */
static uint32_t aic_pending(AT91AicState *s)
{
    uint32_t edgemask = 0;
    int i;

    for (i = 0; i < AIC_NUM_IRQ; i++) {
        if (AIC_SRCTYPE_IS_EDGE(s->smr[i])) {
            edgemask |= 1u << i;
        }
    }
    return (s->raw & ~edgemask) | (s->edge & edgemask);
}

/* Highest-priority enabled IRQ source (excludes source 0 / fast-forced ones,
 * which are FIQ).  Returns -1 if none. */
static int aic_best_irq(AT91AicState *s, uint32_t pending, int *best_prio)
{
    uint32_t eligible = pending & s->imr & ~s->ffsr;
    int best = -1;
    int bestp = -1;
    int i;

    /* Source 0 is always FIQ, never a normal IRQ. */
    for (i = 1; i < AIC_NUM_IRQ; i++) {
        if (!(eligible & (1u << i))) {
            continue;
        }
        int p = AIC_SMR_PRIOR(s->smr[i]);
        if (p > bestp) {          /* higher priority, or first found */
            bestp = p;
            best = i;
        }
    }
    *best_prio = bestp;
    return best;
}

static void aic_update(AT91AicState *s)
{
    uint32_t pending = aic_pending(s);
    int best_prio;
    int best = aic_best_irq(s, pending, &best_prio);
    int cur_prio = (s->sp > 0) ? s->prio_stack[s->sp - 1] : -1;
    bool irq, fiq;

    /* nIRQ: a higher-priority source than the one currently in service. */
    irq = (best >= 0) && (best_prio > cur_prio);

    /* nFIQ: source 0, or any fast-forced source, pending and enabled. */
    fiq = (pending & s->imr & (s->ffsr | 0x1)) != 0;

    s->irq_asserted = irq;
    s->fiq_asserted = fiq;
    qemu_set_irq(s->irq, irq);
    qemu_set_irq(s->fiq, fiq);
}

static void aic_set_irq(void *opaque, int n, int level)
{
    AT91AicState *s = AT91_AIC(opaque);
    uint32_t bit = 1u << n;
    bool old = s->raw & bit;

    trace_at91_aic_irq(n, level);
    if (level) {
        s->raw |= bit;
        if (AIC_SRCTYPE_IS_EDGE(s->smr[n]) && !old) {
            s->edge |= bit;         /* latch rising edge */
        }
    } else {
        s->raw &= ~bit;
    }
    aic_update(s);
}

static uint64_t aic_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91AicState *s = AT91_AIC(opaque);
    uint32_t pending, r = 0;
    int best, best_prio, cur_prio;

    if (offset <= AIC_SMR(31)) {
        return s->smr[offset / 4];
    }
    if (offset >= AIC_SVR(0) && offset <= AIC_SVR(31)) {
        return s->svr[(offset - AIC_SVR(0)) / 4];
    }

    switch (offset) {
    case AIC_IVR:
        /* Entry point of interrupt handling: return the vector of the current
         * highest-priority IRQ, push its priority, ack edge sources. */
        pending = aic_pending(s);
        best = aic_best_irq(s, pending, &best_prio);
        cur_prio = (s->sp > 0) ? s->prio_stack[s->sp - 1] : -1;
        if (best < 0 || best_prio <= cur_prio) {
            /* Spurious: no eligible interrupt (the source de-asserted between
             * nIRQ being raised and this read).  The driver still writes EOICR
             * once per IVR read, so we must push here too - pushing the current
             * level keeps the priority stack balanced without changing it. */
            s->isr = 0;
            if (s->sp < AIC_STACK_SZ) {
                s->prio_stack[s->sp++] = cur_prio;
            }
            trace_at91_aic_ivr(-1, cur_prio, s->sp);
            return s->spu;
        }
        s->isr = best;
        if (s->sp < AIC_STACK_SZ) {
            s->prio_stack[s->sp++] = best_prio;
        }
        trace_at91_aic_ivr(best, best_prio, s->sp);
        if (AIC_SRCTYPE_IS_EDGE(s->smr[best])) {
            s->edge &= ~(1u << best);
        }
        r = s->svr[best];
        aic_update(s);
        break;
    case AIC_FVR:
        /* An edge-programmed fast interrupt stays pending until the
         * FVR is read (the FIQ handler's entry point acknowledges it). */
        r = s->svr[0];
        if (AIC_SRCTYPE_IS_EDGE(s->smr[0])) {
            s->edge &= ~1u;
            aic_update(s);
        }
        break;
    case AIC_ISR:
        r = s->isr;
        break;
    case AIC_IPR:
        r = aic_pending(s);
        break;
    case AIC_IMR:
        r = s->imr;
        break;
    case AIC_CISR:
        r = (s->fiq_asserted ? 1 : 0) | (s->irq_asserted ? 2 : 0);
        break;
    case AIC_SPU:
        r = s->spu;
        break;
    case AIC_DCR:
        r = s->dcr;
        break;
    case AIC_FFSR:
        r = s->ffsr;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-aic: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        break;
    }
    return r;
}

static void aic_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91AicState *s = AT91_AIC(opaque);
    uint32_t val = value;

    if (offset <= AIC_SMR(31)) {
        s->smr[offset / 4] = val & 0x67;   /* PRIOR + SRCTYPE */
        aic_update(s);
        return;
    }
    if (offset >= AIC_SVR(0) && offset <= AIC_SVR(31)) {
        s->svr[(offset - AIC_SVR(0)) / 4] = val;
        return;
    }

    switch (offset) {
    case AIC_IECR:
        s->imr |= val;
        aic_update(s);
        break;
    case AIC_IDCR:
        s->imr &= ~val;
        aic_update(s);
        break;
    case AIC_ICCR:
        s->edge &= ~val;           /* clear edge-latched pendings */
        aic_update(s);
        break;
    case AIC_ISCR:
        s->edge |= val;            /* software-set edge pendings */
        aic_update(s);
        break;
    case AIC_EOICR:
        if (s->sp > 0) {
            s->sp--;               /* exit point: pop priority stack */
        }
        trace_at91_aic_eoi(s->sp);
        aic_update(s);
        break;
    case AIC_SPU:
        s->spu = val;
        break;
    case AIC_DCR:
        s->dcr = val;
        break;
    case AIC_FFER:
        s->ffsr |= val & ~0x1u;    /* source 0 cannot be fast-forced */
        aic_update(s);
        break;
    case AIC_FFDR:
        s->ffsr &= ~val;
        aic_update(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-aic: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps aic_ops = {
    .read = aic_read,
    .write = aic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void aic_reset(DeviceState *dev)
{
    AT91AicState *s = AT91_AIC(dev);

    memset(s->smr, 0, sizeof(s->smr));
    memset(s->svr, 0, sizeof(s->svr));
    s->imr = 0;
    s->raw = 0;
    s->edge = 0;
    s->ffsr = 0;
    s->isr = 0;
    s->spu = 0;
    s->dcr = 0;
    s->sp = 0;
    s->irq_asserted = false;
    s->fiq_asserted = false;
    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->fiq, 0);
}

static void aic_init(Object *obj)
{
    AT91AicState *s = AT91_AIC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &aic_ops, s, "at91-aic", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(DEVICE(obj), aic_set_irq, AIC_NUM_IRQ);
}

static const VMStateDescription vmstate_at91_aic = {
    .name = "at91-aic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(smr, AT91AicState, AIC_NUM_IRQ),
        VMSTATE_UINT32_ARRAY(svr, AT91AicState, AIC_NUM_IRQ),
        VMSTATE_UINT32(imr, AT91AicState),
        VMSTATE_UINT32(raw, AT91AicState),
        VMSTATE_UINT32(edge, AT91AicState),
        VMSTATE_UINT32(ffsr, AT91AicState),
        VMSTATE_UINT32(isr, AT91AicState),
        VMSTATE_UINT32(spu, AT91AicState),
        VMSTATE_UINT32(dcr, AT91AicState),
        VMSTATE_INT32_ARRAY(prio_stack, AT91AicState, AIC_STACK_SZ),
        VMSTATE_INT32(sp, AT91AicState),
        VMSTATE_BOOL(irq_asserted, AT91AicState),
        VMSTATE_BOOL(fiq_asserted, AT91AicState),
        VMSTATE_END_OF_LIST()
    }
};

static void aic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, aic_reset);
    dc->vmsd = &vmstate_at91_aic;
}

static const TypeInfo aic_type = {
    .name = TYPE_AT91_AIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91AicState),
    .instance_init = aic_init,
    .class_init = aic_class_init,
};

static void at91_aic_register_types(void)
{
    type_register_static(&aic_type);
}

type_init(at91_aic_register_types)
