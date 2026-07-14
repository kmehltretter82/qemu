/*
 * Atmel/Microchip SAM9M10-G45-EK board + AT91SAM9G45 SoC (ARM926EJ-S).
 *
 * Bring-up target: boot a mainline arm multi_v5 zImage to an initramfs shell
 * over the DBGU console.  Register-level references are from the Atmel-6438
 * datasheet; see qemu/.localdir/qemu-at91sam9g45-plan.md.
 *
 * Modelled so far:
 *   - machine scaffold: arm926 + 128 MB DDR2 at 0x70000000
 *   - DBGU  (0xFFFFEE00) console UART, with RX + IRQ
 *   - AIC   (0xFFFFF000) Advanced Interrupt Controller
 *   - PIT   (0xFFFFFD30) Periodic Interval Timer (system tick)
 *   - PMC   (0xFFFFFC00) Power Management Controller (permissive clock stub)
 *
 * The system interrupt (AIC source 1) is the wired-OR of DBGU + PIT (+ other
 * system peripherals on real HW); it is modelled with an OR gate.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/or-irq.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "system/runstate.h"
#include "system/blockdev.h"
#include "chardev/char-fe.h"
#include "hw/sd/sd.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/misc/unimp.h"
#include "qom/object.h"
#include "target/arm/cpu-qom.h"

/* ---- SoC memory map (datasheet Fig 5-1, section 6.1) -------------------- */
#define SAM9G45_SDRAM_BASE   0x70000000   /* DDRSDRC0 chip select (main RAM) */
#define SAM9G45_PERIPH_BASE  0xFFF78000   /* start of APB peripheral window  */
#define SAM9G45_PERIPH_SIZE  0x00088000   /* ...through 0xFFFFFFFF            */
#define SAM9G45_DBGU_BASE    0xFFFFEE00   /* Debug Unit (console UART)        */
#define SAM9G45_AIC_BASE     0xFFFFF000   /* Advanced Interrupt Controller    */
#define SAM9G45_DMAC_BASE    0xFFFFEC00   /* DMA Controller                    */
#define SAM9G45_PMC_BASE     0xFFFFFC00   /* Power Management Controller       */
#define SAM9G45_RSTC_BASE    0xFFFFFD00   /* Reset Controller                  */
#define SAM9G45_SHDWC_BASE   0xFFFFFD10   /* Shutdown Controller               */
#define SAM9G45_PIT_BASE     0xFFFFFD30   /* Periodic Interval Timer           */
#define SAM9G45_WDT_BASE     0xFFFFFD40   /* Watchdog Timer                    */
#define SAM9G45_HSMCI0_BASE  0xFFF80000   /* High Speed MMC Interface 0        */
#define SAM9G45_HSMCI1_BASE  0xFFFD0000   /* High Speed MMC Interface 1        */
#define SAM9G45_OHCI_BASE    0x00700000   /* USB Host OHCI (full/low speed)    */
#define SAM9G45_EHCI_BASE    0x00800000   /* USB Host EHCI (high speed)        */

#define SAM9G45_DEFAULT_RAM  (128 * MiB)  /* SAM9M10-G45-EK: 128 MB DDR2      */

/* Peripheral interrupt IDs (AIC source numbers, datasheet Table 7-1). */
#define SAM9G45_IRQ_HSMCI0   11
#define SAM9G45_IRQ_HSMCI1   29
#define SAM9G45_IRQ_UHPHS    22   /* USB host (OHCI + EHCI share this) */
#define SAM9G45_IRQ_DMAC     21

/*
 * Master clock.  With no boot loader, our PMC reports MCK sourced from the
 * 12 MHz main crystal (see AT91PmcState), so model the PIT at the same rate to
 * keep guest timekeeping self-consistent.
 */
#define SAM9G45_MCK_HZ       12000000

/* AIC source 1 is the shared "system interrupt" (DBGU, PIT, RTT, ...). */
#define SAM9G45_IRQ_SYS      1

/* ======================================================================== */
/*  DBGU - Debug Unit (datasheet section 27)                                 */
/* ======================================================================== */

#define DBGU_CR     0x00   /* Control Register            (write-only) */
#define DBGU_MR     0x04   /* Mode Register                            */
#define DBGU_IER    0x08   /* Interrupt Enable Register   (write-only) */
#define DBGU_IDR    0x0C   /* Interrupt Disable Register  (write-only) */
#define DBGU_IMR    0x10   /* Interrupt Mask Register     (read-only)  */
#define DBGU_SR     0x14   /* Status Register             (read-only)  */
#define DBGU_RHR    0x18   /* Receive Holding Register    (read-only)  */
#define DBGU_THR    0x1C   /* Transmit Holding Register   (write-only) */
#define DBGU_BRGR   0x20   /* Baud Rate Generator Register             */
#define DBGU_CIDR   0x40   /* Chip ID Register            (read-only)  */
#define DBGU_EXID   0x44   /* Chip ID Extension Register  (read-only)  */

/* Status/CSR bits (shared with USART) */
#define DBGU_RXRDY    (1u << 0)
#define DBGU_TXRDY    (1u << 1)
#define DBGU_TXEMPTY  (1u << 9)

/* Control Register bits */
#define DBGU_CR_RSTRX   (1u << 2)
#define DBGU_CR_RSTTX   (1u << 3)
#define DBGU_CR_RXEN    (1u << 4)
#define DBGU_CR_RXDIS   (1u << 5)
#define DBGU_CR_TXEN    (1u << 6)
#define DBGU_CR_TXDIS   (1u << 7)
#define DBGU_CR_RSTSTA  (1u << 8)

/* Chip identification for AT91SAM9G45 (datasheet section 6.3) */
#define SAM9G45_CIDR  0x819B05A2
#define SAM9G45_EXID  0x00000004

#define TYPE_AT91_DBGU "at91-dbgu"
OBJECT_DECLARE_SIMPLE_TYPE(AT91DbguState, AT91_DBGU)

struct AT91DbguState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend chr;
    qemu_irq irq;

    uint32_t imr;       /* interrupt mask                    */
    uint32_t mr;        /* mode register (stored, unused)    */
    uint32_t brgr;      /* baud divisor (stored, unused)     */
    uint8_t  rx_fifo;   /* single receive holding byte       */
    bool     rx_pending;
    bool     rx_enabled;
    bool     tx_enabled;
};

static uint32_t dbgu_status(AT91DbguState *s)
{
    /* TX is instantaneous in the model, so it is always ready/empty. */
    uint32_t sr = DBGU_TXRDY | DBGU_TXEMPTY;
    if (s->rx_pending) {
        sr |= DBGU_RXRDY;
    }
    return sr;
}

static void dbgu_update_irq(AT91DbguState *s)
{
    qemu_set_irq(s->irq, (dbgu_status(s) & s->imr) ? 1 : 0);
}

static uint64_t dbgu_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91DbguState *s = AT91_DBGU(opaque);
    uint32_t r = 0;

    switch (offset) {
    case DBGU_MR:
        r = s->mr;
        break;
    case DBGU_IMR:
        r = s->imr;
        break;
    case DBGU_SR:
        r = dbgu_status(s);
        break;
    case DBGU_RHR:
        r = s->rx_fifo;
        s->rx_pending = false;
        dbgu_update_irq(s);
        qemu_chr_fe_accept_input(&s->chr);
        break;
    case DBGU_BRGR:
        r = s->brgr;
        break;
    case DBGU_CIDR:
        r = SAM9G45_CIDR;
        break;
    case DBGU_EXID:
        r = SAM9G45_EXID;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-dbgu: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        break;
    }
    return r;
}

static void dbgu_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AT91DbguState *s = AT91_DBGU(opaque);
    unsigned char ch;

    switch (offset) {
    case DBGU_CR:
        if (value & DBGU_CR_RXEN) {
            s->rx_enabled = true;
        }
        if (value & DBGU_CR_RXDIS) {
            s->rx_enabled = false;
        }
        if (value & DBGU_CR_TXEN) {
            s->tx_enabled = true;
        }
        if (value & DBGU_CR_TXDIS) {
            s->tx_enabled = false;
        }
        if (value & DBGU_CR_RSTRX) {
            s->rx_pending = false;
        }
        /* RSTTX/RSTSTA have no observable state in this model. */
        dbgu_update_irq(s);
        break;
    case DBGU_MR:
        s->mr = value;
        break;
    case DBGU_IER:
        s->imr |= value;
        dbgu_update_irq(s);
        break;
    case DBGU_IDR:
        s->imr &= ~value;
        dbgu_update_irq(s);
        break;
    case DBGU_THR:
        ch = value & 0xff;
        /* Instant, blocking-free drain: fine for a console. */
        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        dbgu_update_irq(s);
        break;
    case DBGU_BRGR:
        s->brgr = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-dbgu: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n",
                      offset, (uint32_t)value);
        break;
    }
}

static const MemoryRegionOps dbgu_ops = {
    .read = dbgu_read,
    .write = dbgu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static int dbgu_can_receive(void *opaque)
{
    AT91DbguState *s = AT91_DBGU(opaque);
    return s->rx_pending ? 0 : 1;
}

static void dbgu_receive(void *opaque, const uint8_t *buf, int size)
{
    AT91DbguState *s = AT91_DBGU(opaque);

    if (size <= 0) {
        return;
    }
    s->rx_fifo = buf[0];
    s->rx_pending = true;
    dbgu_update_irq(s);
}

static void dbgu_reset(DeviceState *dev)
{
    AT91DbguState *s = AT91_DBGU(dev);

    s->imr = 0;
    s->mr = 0;
    s->brgr = 0;
    s->rx_pending = false;
    s->rx_enabled = false;
    s->tx_enabled = false;
    dbgu_update_irq(s);
}

static void dbgu_realize(DeviceState *dev, Error **errp)
{
    AT91DbguState *s = AT91_DBGU(dev);

    qemu_chr_fe_set_handlers(&s->chr, dbgu_can_receive, dbgu_receive,
                             NULL, NULL, s, NULL, true);
}

static void dbgu_init(Object *obj)
{
    AT91DbguState *s = AT91_DBGU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &dbgu_ops, s, "at91-dbgu", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property dbgu_properties[] = {
    DEFINE_PROP_CHR("chardev", AT91DbguState, chr),
};

static void dbgu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = dbgu_realize;
    device_class_set_legacy_reset(dc, dbgu_reset);
    device_class_set_props(dc, dbgu_properties);
}

static const TypeInfo dbgu_type = {
    .name = TYPE_AT91_DBGU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91DbguState),
    .instance_init = dbgu_init,
    .class_init = dbgu_class_init,
};

/* ======================================================================== */
/*  AIC - Advanced Interrupt Controller (datasheet section 26)               */
/* ======================================================================== */

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

#define TYPE_AT91_AIC "at91-aic"
OBJECT_DECLARE_SIMPLE_TYPE(AT91AicState, AT91_AIC)

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
            /* Spurious: no eligible interrupt. */
            s->isr = 0;
            return s->spu;
        }
        s->isr = best;
        if (s->sp < AIC_STACK_SZ) {
            s->prio_stack[s->sp++] = best_prio;
        }
        if (AIC_SRCTYPE_IS_EDGE(s->smr[best])) {
            s->edge &= ~(1u << best);
        }
        r = s->svr[best];
        aic_update(s);
        break;
    case AIC_FVR:
        r = s->svr[0];
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

static void aic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, aic_reset);
}

static const TypeInfo aic_type = {
    .name = TYPE_AT91_AIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91AicState),
    .instance_init = aic_init,
    .class_init = aic_class_init,
};

/* ======================================================================== */
/*  PIT - Periodic Interval Timer (datasheet section 14)                     */
/* ======================================================================== */

#define PIT_MR      0x00   /* Mode Register                */
#define PIT_SR      0x04   /* Status Register              */
#define PIT_PIVR    0x08   /* Periodic Interval Value Reg  */
#define PIT_PIIR    0x0C   /* Periodic Interval Image Reg  */

#define PIT_MR_PIV(mr)   ((mr) & 0xFFFFF)
#define PIT_MR_PITEN     (1u << 24)
#define PIT_MR_PITIEN    (1u << 25)
#define PIT_SR_PITS      (1u << 0)

#define TYPE_AT91_PIT "at91-pit"
OBJECT_DECLARE_SIMPLE_TYPE(AT91PitState, AT91_PIT)

struct AT91PitState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t mck_freq;     /* master clock (Hz); PIT counts at mck/16 */

    uint32_t mr;
    uint32_t picnt;        /* completed intervals since last PIVR read */
    bool pits;
    int64_t last_fire;     /* ns of last interval boundary */
};

static uint32_t pit_clk(AT91PitState *s)
{
    return s->mck_freq / 16;
}

/* nanoseconds per PIT interval, given the programmed PIV */
static int64_t pit_period_ns(AT91PitState *s)
{
    uint32_t piv = PIT_MR_PIV(s->mr);
    return (int64_t)(piv + 1) * NANOSECONDS_PER_SECOND / pit_clk(s);
}

static void pit_update_irq(AT91PitState *s)
{
    qemu_set_irq(s->irq, (s->pits && (s->mr & PIT_MR_PITIEN)) ? 1 : 0);
}

/* Current sub-interval counter value (CPIV). */
static uint32_t pit_cpiv(AT91PitState *s)
{
    int64_t now, elapsed;
    uint32_t piv, cpiv;

    if (!(s->mr & PIT_MR_PITEN) || PIT_MR_PIV(s->mr) == 0) {
        return 0;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed = now - s->last_fire;
    piv = PIT_MR_PIV(s->mr);
    cpiv = muldiv64(elapsed, pit_clk(s), NANOSECONDS_PER_SECOND);
    if (cpiv > piv) {
        cpiv = piv;
    }
    return cpiv;
}

static void pit_tick(void *opaque)
{
    AT91PitState *s = opaque;

    s->last_fire = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->pits = true;
    if (s->picnt < 0xFFF) {
        s->picnt++;
    }
    pit_update_irq(s);
    if ((s->mr & PIT_MR_PITEN) && PIT_MR_PIV(s->mr) != 0) {
        timer_mod(s->timer, s->last_fire + pit_period_ns(s));
    }
}

static void pit_rearm(AT91PitState *s)
{
    if ((s->mr & PIT_MR_PITEN) && PIT_MR_PIV(s->mr) != 0) {
        s->last_fire = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(s->timer, s->last_fire + pit_period_ns(s));
    } else {
        timer_del(s->timer);
    }
}

static uint64_t pit_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91PitState *s = AT91_PIT(opaque);
    uint32_t r = 0;

    switch (offset) {
    case PIT_MR:
        r = s->mr;
        break;
    case PIT_SR:
        r = s->pits ? PIT_SR_PITS : 0;
        break;
    case PIT_PIVR:
        /* Reading PIVR returns CPIV|PICNT and clears PITS + PICNT (ack). */
        r = pit_cpiv(s) | (s->picnt << 20);
        s->picnt = 0;
        s->pits = false;
        pit_update_irq(s);
        break;
    case PIT_PIIR:
        r = pit_cpiv(s) | (s->picnt << 20);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-pit: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
        break;
    }
    return r;
}

static void pit_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91PitState *s = AT91_PIT(opaque);

    switch (offset) {
    case PIT_MR:
        s->mr = value & 0x03FFFFFF;
        pit_rearm(s);
        pit_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-pit: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n",
                      offset, (uint32_t)value);
        break;
    }
}

static const MemoryRegionOps pit_ops = {
    .read = pit_read,
    .write = pit_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void pit_reset(DeviceState *dev)
{
    AT91PitState *s = AT91_PIT(dev);

    timer_del(s->timer);
    s->mr = 0x000FFFFF;    /* reset value: PIV=0xFFFFF, disabled */
    s->picnt = 0;
    s->pits = false;
    s->last_fire = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    pit_update_irq(s);
}

static void pit_realize(DeviceState *dev, Error **errp)
{
    AT91PitState *s = AT91_PIT(dev);

    if (s->mck_freq == 0) {
        error_setg(errp, "at91-pit: mck-frequency must be set");
        return;
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pit_tick, s);
}

static void pit_dev_init(Object *obj)
{
    AT91PitState *s = AT91_PIT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pit_ops, s, "at91-pit", 0x10);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property pit_properties[] = {
    DEFINE_PROP_UINT32("mck-frequency", AT91PitState, mck_freq,
                       SAM9G45_MCK_HZ),
};

static void pit_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pit_realize;
    device_class_set_legacy_reset(dc, pit_reset);
    device_class_set_props(dc, pit_properties);
}

static const TypeInfo pit_type = {
    .name = TYPE_AT91_PIT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91PitState),
    .instance_init = pit_dev_init,
    .class_init = pit_class_init,
};

/* ======================================================================== */
/*  PMC - Power Management Controller (datasheet section 25)                 */
/*                                                                           */
/*  Permissive clock stub: registers are read/write storage, all lock/ready  */
/*  status bits read as set (the at91 clk driver spin-waits on them), and the */
/*  master clock is reported as sourced from the 12 MHz main crystal.         */
/* ======================================================================== */

#define PMC_SCER    0x00   /* System Clock Enable      */
#define PMC_SCDR    0x04   /* System Clock Disable     */
#define PMC_SCSR    0x08   /* System Clock Status      */
#define PMC_PCER    0x10   /* Peripheral Clock Enable  */
#define PMC_PCDR    0x14   /* Peripheral Clock Disable */
#define PMC_PCSR    0x18   /* Peripheral Clock Status  */
#define CKGR_UCKR   0x1C   /* UTMI Clock Config        */
#define CKGR_MOR    0x20   /* Main Oscillator          */
#define CKGR_MCFR   0x24   /* Main Clock Frequency     */
#define CKGR_PLLAR  0x28   /* PLLA Register            */
#define PMC_MCKR    0x30   /* Master Clock Register    */
#define PMC_USB     0x38   /* USB Clock Register       */
#define PMC_PCK0    0x40   /* Programmable Clock 0     */
#define PMC_PCK1    0x44   /* Programmable Clock 1     */
#define PMC_IER     0x60   /* Interrupt Enable         */
#define PMC_IDR     0x64   /* Interrupt Disable        */
#define PMC_SR      0x68   /* Status Register          */
#define PMC_IMR     0x6C   /* Interrupt Mask           */
#define PMC_PLLICPR 0x80   /* PLL Charge Pump Current  */

/* PMC_SR status bits we always report ready (datasheet 25.12.16). */
#define PMC_SR_READY \
    ((1u << 0)  /* MOSCS   */ | (1u << 1)  /* LOCKA    */ | \
     (1u << 3)  /* MCKRDY  */ | (1u << 6)  /* LOCKU    */ | \
     (1u << 7)  /* OSCSELS */ | (1u << 8)  /* PCKRDY0  */ | \
     (1u << 9)  /* PCKRDY1 */ | (1u << 16) /* MOSCSELS */ | \
     (1u << 17) /* MOSCRCS */)

/* CKGR_MCFR: MAINF measured in 16 slow-clock periods for a 12 MHz main clock. */
#define PMC_MAINF_12MHZ  ((12000000u * 16u) / 32768u)   /* ~5859 */
#define PMC_MCFR_VALUE   ((1u << 16) /* MAINRDY */ | PMC_MAINF_12MHZ)

/* Reset MCKR: CSS=1 (main clock), PRES=0, MDIV=0 -> MCK = 12 MHz main. */
#define PMC_MCKR_RESET   0x00000001

#define TYPE_AT91_PMC "at91-pmc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91PmcState, AT91_PMC)

struct AT91PmcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t scsr;
    uint32_t pcsr;
    uint32_t uckr;
    uint32_t mor;
    uint32_t pllar;
    uint32_t mckr;
    uint32_t usb;
    uint32_t pck[2];
    uint32_t imr;
    uint32_t pllicpr;
};

static uint64_t pmc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91PmcState *s = AT91_PMC(opaque);

    switch (offset) {
    case PMC_SCSR:   return s->scsr;
    case PMC_PCSR:   return s->pcsr;
    case CKGR_UCKR:  return s->uckr;
    case CKGR_MOR:   return s->mor;
    case CKGR_MCFR:  return PMC_MCFR_VALUE;
    case CKGR_PLLAR: return s->pllar;
    case PMC_MCKR:   return s->mckr;
    case PMC_USB:    return s->usb;
    case PMC_PCK0:   return s->pck[0];
    case PMC_PCK1:   return s->pck[1];
    case PMC_SR:     return PMC_SR_READY;
    case PMC_IMR:    return s->imr;
    case PMC_PLLICPR: return s->pllicpr;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-pmc: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void pmc_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91PmcState *s = AT91_PMC(opaque);
    uint32_t val = value;

    switch (offset) {
    case PMC_SCER:   s->scsr |= val;   break;
    case PMC_SCDR:   s->scsr &= ~val;  break;
    case PMC_PCER:   s->pcsr |= val;   break;
    case PMC_PCDR:   s->pcsr &= ~val;  break;
    case CKGR_UCKR:  s->uckr = val;    break;
    case CKGR_MOR:   s->mor = val;     break;
    case CKGR_PLLAR: s->pllar = val;   break;
    case PMC_MCKR:   s->mckr = val;    break;
    case PMC_USB:    s->usb = val;     break;
    case PMC_PCK0:   s->pck[0] = val;  break;
    case PMC_PCK1:   s->pck[1] = val;  break;
    case PMC_IER:    s->imr |= val;    break;
    case PMC_IDR:    s->imr &= ~val;   break;
    case PMC_PLLICPR: s->pllicpr = val; break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-pmc: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps pmc_ops = {
    .read = pmc_read,
    .write = pmc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void pmc_reset(DeviceState *dev)
{
    AT91PmcState *s = AT91_PMC(dev);

    s->scsr = 0x01;
    s->pcsr = 0;
    s->uckr = 0x10200800;
    s->mor = 0;
    s->pllar = 0x00003F00;
    s->mckr = PMC_MCKR_RESET;
    s->usb = 0;
    s->pck[0] = 0;
    s->pck[1] = 0;
    s->imr = 0;
    s->pllicpr = 0;
}

static void pmc_dev_init(Object *obj)
{
    AT91PmcState *s = AT91_PMC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pmc_ops, s, "at91-pmc", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void pmc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, pmc_reset);
}

static const TypeInfo pmc_type = {
    .name = TYPE_AT91_PMC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91PmcState),
    .instance_init = pmc_dev_init,
    .class_init = pmc_class_init,
};

/* ======================================================================== */
/*  HSMCI - High Speed MultiMedia Card Interface (datasheet section 35)       */
/*                                                                           */
/*  PIO-mode SD/MMC host: commands go to QEMU's SD card model over an SDBus, */
/*  data is transferred a word at a time through RDR/TDR.  No DMA/PDC.        */
/* ======================================================================== */

#define HSMCI_CR     0x00
#define HSMCI_MR     0x04
#define HSMCI_DTOR   0x08
#define HSMCI_SDCR   0x0C
#define HSMCI_ARGR   0x10
#define HSMCI_CMDR   0x14
#define HSMCI_BLKR   0x18
#define HSMCI_CSTOR  0x1C
#define HSMCI_RSPR   0x20   /* 0x20..0x2C */
#define HSMCI_RDR    0x30
#define HSMCI_TDR    0x34
#define HSMCI_SR     0x40
#define HSMCI_IER    0x44
#define HSMCI_IDR    0x48
#define HSMCI_IMR    0x4C
#define HSMCI_DMA    0x50
#define HSMCI_CFG    0x54
#define HSMCI_WPMR   0xE4
#define HSMCI_WPSR   0xE8
#define HSMCI_VERSION 0xFC
#define HSMCI_FIFO   0x200  /* 0x200..0x5FC */

/*
 * IP version reported in HSMCI_VERSION.  The atmel-mci driver keys its
 * capabilities off this: 0x3xx selects the HSMCI feature set (has_dma_conf_reg,
 * no PDC).  With no external DMA channel in the DT the driver then falls back
 * to PIO, which is what this model implements.
 */
#define HSMCI_IP_VERSION  0x300

/* Control Register bits */
#define HSMCI_CR_MCIEN   (1u << 0)
#define HSMCI_CR_MCIDIS  (1u << 1)
#define HSMCI_CR_SWRST   (1u << 7)

/* Command Register fields */
#define HSMCI_CMDR_CMDNB(c)    ((c) & 0x3F)
#define HSMCI_CMDR_RSPTYP(c)   (((c) >> 6) & 0x3)
#define HSMCI_CMDR_TRCMD(c)    (((c) >> 16) & 0x3)
#define HSMCI_CMDR_TRDIR_READ  (1u << 18)
#define HSMCI_RSPTYP_NONE   0
#define HSMCI_RSPTYP_48     1
#define HSMCI_RSPTYP_136    2
#define HSMCI_TRCMD_START   1

/* Block Register fields */
#define HSMCI_BLKR_BCNT(b)     ((b) & 0xFFFF)
#define HSMCI_BLKR_BLKLEN(b)   (((b) >> 16) & 0xFFFF)

/* Status/Interrupt bits */
#define HSMCI_SR_CMDRDY   (1u << 0)
#define HSMCI_SR_RXRDY    (1u << 1)
#define HSMCI_SR_TXRDY    (1u << 2)
#define HSMCI_SR_BLKE     (1u << 3)
#define HSMCI_SR_NOTBUSY  (1u << 5)
#define HSMCI_SR_RESET    0x0000C0E5

#define TYPE_AT91_HSMCI     "at91-hsmci"
#define TYPE_AT91_HSMCI_BUS "at91-hsmci-bus"
OBJECT_DECLARE_SIMPLE_TYPE(AT91HsmciState, AT91_HSMCI)

static void hsmci_reset(DeviceState *dev);

struct AT91HsmciState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;

    uint32_t mr, dtor, sdcr, argr, blkr, cstor, dma, cfg, wpmr;
    uint32_t sr;
    uint32_t imr;
    uint32_t rsp[4];
    int rsp_ptr;

    uint32_t blklen;        /* current transfer block length */
    int32_t  data_len;      /* bytes left in the current data transfer */
    bool     reading;       /* transfer direction */
};

static void hsmci_update_irq(AT91HsmciState *s)
{
    qemu_set_irq(s->irq, (s->sr & s->imr) ? 1 : 0);
}

static void hsmci_do_command(AT91HsmciState *s, uint32_t cmdr)
{
    SDRequest req = {
        .cmd = HSMCI_CMDR_CMDNB(cmdr),
        .arg = s->argr,
    };
    uint8_t resp[16];
    size_t rlen;

    rlen = sdbus_do_command(&s->sdbus, &req, resp, sizeof(resp));

    s->rsp_ptr = 0;
    if (HSMCI_CMDR_RSPTYP(cmdr) == HSMCI_RSPTYP_136 && rlen == 16) {
        s->rsp[0] = ldl_be_p(&resp[0]);
        s->rsp[1] = ldl_be_p(&resp[4]);
        s->rsp[2] = ldl_be_p(&resp[8]);
        s->rsp[3] = ldl_be_p(&resp[12]);
    } else if (rlen >= 4) {
        s->rsp[0] = ldl_be_p(&resp[0]);
        s->rsp[1] = s->rsp[2] = s->rsp[3] = 0;
    } else {
        s->rsp[0] = s->rsp[1] = s->rsp[2] = s->rsp[3] = 0;
    }

    s->sr |= HSMCI_SR_CMDRDY;

    /* Set up a data transfer if the command starts one. */
    if (HSMCI_CMDR_TRCMD(cmdr) == HSMCI_TRCMD_START) {
        uint32_t blklen = HSMCI_BLKR_BLKLEN(s->blkr);
        uint32_t bcnt = HSMCI_BLKR_BCNT(s->blkr);

        if (blklen == 0) {
            blklen = s->mr >> 16;      /* MR.BLKLEN fallback */
        }
        if (bcnt == 0) {
            bcnt = 1;
        }
        s->blklen = blklen;
        s->data_len = (int32_t)(blklen * bcnt);
        s->reading = (cmdr & HSMCI_CMDR_TRDIR_READ) != 0;
        s->sr &= ~(HSMCI_SR_NOTBUSY | HSMCI_SR_BLKE);
        if (s->reading) {
            s->sr |= HSMCI_SR_RXRDY;
        } else {
            s->sr |= HSMCI_SR_TXRDY;
        }
    }
    hsmci_update_irq(s);
}

static void hsmci_xfer_done(AT91HsmciState *s)
{
    s->sr &= ~(HSMCI_SR_RXRDY | HSMCI_SR_TXRDY);
    s->sr |= HSMCI_SR_TXRDY | HSMCI_SR_BLKE | HSMCI_SR_NOTBUSY;
    s->data_len = 0;
}

static uint32_t hsmci_read_data(AT91HsmciState *s)
{
    uint32_t v = 0;
    int i;

    if (s->data_len <= 0 || !s->reading) {
        return 0;
    }
    for (i = 0; i < 4; i++) {
        v |= (uint32_t)sdbus_read_byte(&s->sdbus) << (8 * i);
    }
    s->data_len -= 4;
    if (s->data_len <= 0) {
        hsmci_xfer_done(s);
    }
    hsmci_update_irq(s);
    return v;
}

static void hsmci_write_data(AT91HsmciState *s, uint32_t value)
{
    int i;

    if (s->data_len <= 0 || s->reading) {
        return;
    }
    for (i = 0; i < 4; i++) {
        sdbus_write_byte(&s->sdbus, (value >> (8 * i)) & 0xff);
    }
    s->data_len -= 4;
    if (s->data_len <= 0) {
        hsmci_xfer_done(s);
    }
    hsmci_update_irq(s);
}

static uint64_t hsmci_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91HsmciState *s = AT91_HSMCI(opaque);

    if (offset >= HSMCI_RSPR && offset <= HSMCI_RSPR + 0xC) {
        if (offset == HSMCI_RSPR) {
            uint32_t v = s->rsp[s->rsp_ptr & 3];
            s->rsp_ptr++;
            return v;
        }
        return s->rsp[(offset - HSMCI_RSPR) / 4];
    }
    if (offset >= HSMCI_FIFO) {
        return hsmci_read_data(s);
    }

    switch (offset) {
    case HSMCI_MR:   return s->mr;
    case HSMCI_DTOR: return s->dtor;
    case HSMCI_SDCR: return s->sdcr;
    case HSMCI_ARGR: return s->argr;
    case HSMCI_BLKR: return s->blkr;
    case HSMCI_CSTOR: return s->cstor;
    case HSMCI_RDR:  return hsmci_read_data(s);
    case HSMCI_SR:   return s->sr;
    case HSMCI_IMR:  return s->imr;
    case HSMCI_DMA:  return s->dma;
    case HSMCI_CFG:  return s->cfg;
    case HSMCI_WPMR: return s->wpmr;
    case HSMCI_WPSR: return 0;
    case HSMCI_VERSION: return HSMCI_IP_VERSION;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-hsmci: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void hsmci_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    AT91HsmciState *s = AT91_HSMCI(opaque);
    uint32_t val = value;

    if (offset >= HSMCI_FIFO) {
        hsmci_write_data(s, val);
        return;
    }

    switch (offset) {
    case HSMCI_CR:
        if (val & HSMCI_CR_SWRST) {
            /* Software reset of the controller only - must NOT reset the SD
             * card on the bus (that would clear its app-command state). */
            uint32_t saved_mr = s->mr, saved_sdcr = s->sdcr;
            hsmci_reset(DEVICE(s));
            s->mr = saved_mr;
            s->sdcr = saved_sdcr;
        }
        break;
    case HSMCI_MR:   s->mr = val;   break;
    case HSMCI_DTOR: s->dtor = val; break;
    case HSMCI_SDCR: s->sdcr = val; break;
    case HSMCI_ARGR: s->argr = val; break;
    case HSMCI_CMDR: hsmci_do_command(s, val); break;
    case HSMCI_BLKR: s->blkr = val; break;
    case HSMCI_CSTOR: s->cstor = val; break;
    case HSMCI_TDR:  hsmci_write_data(s, val); break;
    case HSMCI_IER:  s->imr |= val; hsmci_update_irq(s); break;
    case HSMCI_IDR:  s->imr &= ~val; hsmci_update_irq(s); break;
    case HSMCI_DMA:  s->dma = val; break;
    case HSMCI_CFG:  s->cfg = val; break;
    case HSMCI_WPMR: s->wpmr = val; break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-hsmci: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps hsmci_ops = {
    .read = hsmci_read,
    .write = hsmci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void hsmci_reset(DeviceState *dev)
{
    AT91HsmciState *s = AT91_HSMCI(dev);

    s->mr = s->dtor = s->sdcr = s->argr = s->blkr = 0;
    s->cstor = s->dma = s->cfg = s->wpmr = 0;
    s->sr = HSMCI_SR_RESET;
    s->imr = 0;
    s->rsp[0] = s->rsp[1] = s->rsp[2] = s->rsp[3] = 0;
    s->rsp_ptr = 0;
    s->data_len = 0;
    s->blklen = 0;
    s->reading = false;
}

static void hsmci_dev_init(Object *obj)
{
    AT91HsmciState *s = AT91_HSMCI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hsmci_ops, s, "at91-hsmci", 0x600);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_AT91_HSMCI_BUS, DEVICE(obj),
              "sd-bus");
}

static void hsmci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, hsmci_reset);
    dc->user_creatable = false;   /* needs board IRQ + card wiring */
}

static const TypeInfo hsmci_types[] = {
    {
        .name = TYPE_AT91_HSMCI,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AT91HsmciState),
        .instance_init = hsmci_dev_init,
        .class_init = hsmci_class_init,
    },
    {
        .name = TYPE_AT91_HSMCI_BUS,
        .parent = TYPE_SD_BUS,
        .instance_size = sizeof(SDBus),
    },
};

/* ======================================================================== */
/*  DMAC - DMA Controller (datasheet section 40)                             */
/*                                                                           */
/*  Functional model of the 8-channel scatter-gather DMA.  Transfers run     */
/*  synchronously over the system address space when a channel is enabled,   */
/*  so both memory-to-memory (dmatest) and peripheral transfers (the FIFO is */
/*  just an MMIO address) work.  Picture-in-Picture / chunk striding are not */
/*  modelled.                                                                */
/* ======================================================================== */

#define DMAC_GCFG    0x00
#define DMAC_EN      0x04
#define DMAC_EBCIER  0x18
#define DMAC_EBCIDR  0x1C
#define DMAC_EBCIMR  0x20
#define DMAC_EBCISR  0x24
#define DMAC_CHER    0x28
#define DMAC_CHDR    0x2C
#define DMAC_CHSR    0x30
#define DMAC_CH_BASE 0x3C
#define DMAC_CH_SIZE 0x28

/* per-channel register offsets within a channel block */
#define DMAC_SADDR   0x00
#define DMAC_DADDR   0x04
#define DMAC_DSCR    0x08
#define DMAC_CTRLA   0x0C
#define DMAC_CTRLB   0x10
#define DMAC_CFG     0x14

#define DMAC_N_CHANNELS  8

/* CTRLA fields */
#define DMAC_CTRLA_BTSIZE(a)     ((a) & 0xFFFF)
#define DMAC_CTRLA_SRC_WIDTH(a)  (((a) >> 24) & 0x3)
#define DMAC_CTRLA_DST_WIDTH(a)  (((a) >> 28) & 0x3)
/* CTRLB address modes: 0 = increment, 1 = decrement, 2 = fixed */
#define DMAC_CTRLB_SRC_MODE(b)   (((b) >> 24) & 0x3)
#define DMAC_CTRLB_DST_MODE(b)   (((b) >> 28) & 0x3)

/* EBCISR/EBCIMR bits */
#define DMAC_BTC(x)   (1u << (x))          /* buffer transfer completed */
#define DMAC_CBTC(x)  (1u << (8 + (x)))    /* chained buffer completed  */

#define TYPE_AT91_DMAC "at91-dmac"
OBJECT_DECLARE_SIMPLE_TYPE(AT91DmacState, AT91_DMAC)

struct AT91DmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t gcfg, en, ebcimr, ebcisr, chsr;
    struct {
        uint32_t saddr, daddr, dscr, ctrla, ctrlb, cfg;
    } ch[DMAC_N_CHANNELS];
};

static void dmac_update_irq(AT91DmacState *s)
{
    qemu_set_irq(s->irq, (s->ebcisr & s->ebcimr) ? 1 : 0);
}

static unsigned dmac_width_bytes(unsigned w)
{
    return 1u << (w & 0x3);   /* 0->1, 1->2, 2->4 */
}

/* Transfer one buffer described by (saddr, daddr, ctrla, ctrlb). */
static void dmac_run_buffer(uint32_t saddr, uint32_t daddr,
                            uint32_t ctrla, uint32_t ctrlb)
{
    uint32_t btsize = DMAC_CTRLA_BTSIZE(ctrla);
    unsigned sw = dmac_width_bytes(DMAC_CTRLA_SRC_WIDTH(ctrla));
    unsigned dw = dmac_width_bytes(DMAC_CTRLA_DST_WIDTH(ctrla));
    int smode = DMAC_CTRLB_SRC_MODE(ctrlb);
    int dmode = DMAC_CTRLB_DST_MODE(ctrlb);
    uint32_t sa = saddr, da = daddr;
    uint32_t i;

    for (i = 0; i < btsize; i++) {
        uint8_t buf[4] = { 0 };
        unsigned n = sw < dw ? dw : sw;   /* move the wider of the two */

        if (n > sizeof(buf)) {
            n = sizeof(buf);
        }
        address_space_read(&address_space_memory, sa,
                           MEMTXATTRS_UNSPECIFIED, buf, sw);
        address_space_write(&address_space_memory, da,
                            MEMTXATTRS_UNSPECIFIED, buf, dw);
        sa += (smode == 0) ? sw : (smode == 1) ? -sw : 0;
        da += (dmode == 0) ? dw : (dmode == 1) ? -dw : 0;
    }
}

static void dmac_run_channel(AT91DmacState *s, int n)
{
    uint32_t dscr = s->ch[n].dscr;

    if (dscr == 0) {
        /* single-buffer transfer straight from the channel registers */
        dmac_run_buffer(s->ch[n].saddr, s->ch[n].daddr,
                        s->ch[n].ctrla, s->ch[n].ctrlb);
        s->ebcisr |= DMAC_BTC(n);
    } else {
        /* linked-list of hardware descriptors: saddr,daddr,ctrla,ctrlb,next */
        int guard = 0;
        while (dscr != 0 && guard++ < 1024) {
            uint32_t d[5];
            address_space_read(&address_space_memory, dscr,
                               MEMTXATTRS_UNSPECIFIED, d, sizeof(d));
            dmac_run_buffer(d[0], d[1], d[2], d[3]);
            s->ebcisr |= DMAC_BTC(n);
            dscr = d[4];
        }
        s->ebcisr |= DMAC_CBTC(n);
    }
    s->chsr &= ~(1u << n);   /* channel finished */
    dmac_update_irq(s);
}

static uint64_t dmac_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91DmacState *s = AT91_DMAC(opaque);
    uint32_t r = 0;

    if (offset >= DMAC_CH_BASE) {
        unsigned n = (offset - DMAC_CH_BASE) / DMAC_CH_SIZE;
        unsigned reg = (offset - DMAC_CH_BASE) % DMAC_CH_SIZE;
        if (n >= DMAC_N_CHANNELS) {
            return 0;
        }
        switch (reg) {
        case DMAC_SADDR: return s->ch[n].saddr;
        case DMAC_DADDR: return s->ch[n].daddr;
        case DMAC_DSCR:  return s->ch[n].dscr;
        case DMAC_CTRLA: return s->ch[n].ctrla;
        case DMAC_CTRLB: return s->ch[n].ctrlb;
        case DMAC_CFG:   return s->ch[n].cfg;
        default:         return 0;
        }
    }

    switch (offset) {
    case DMAC_GCFG:   r = s->gcfg; break;
    case DMAC_EN:     r = s->en; break;
    case DMAC_EBCIMR: r = s->ebcimr; break;
    case DMAC_EBCISR:                   /* read clears the status */
        r = s->ebcisr;
        s->ebcisr = 0;
        dmac_update_irq(s);
        break;
    case DMAC_CHSR:   r = s->chsr; break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-dmac: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        break;
    }
    return r;
}

static void dmac_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AT91DmacState *s = AT91_DMAC(opaque);
    uint32_t val = value;

    if (offset >= DMAC_CH_BASE) {
        unsigned n = (offset - DMAC_CH_BASE) / DMAC_CH_SIZE;
        unsigned reg = (offset - DMAC_CH_BASE) % DMAC_CH_SIZE;
        if (n >= DMAC_N_CHANNELS) {
            return;
        }
        switch (reg) {
        case DMAC_SADDR: s->ch[n].saddr = val; break;
        case DMAC_DADDR: s->ch[n].daddr = val; break;
        case DMAC_DSCR:  s->ch[n].dscr = val; break;
        case DMAC_CTRLA: s->ch[n].ctrla = val; break;
        case DMAC_CTRLB: s->ch[n].ctrlb = val; break;
        case DMAC_CFG:   s->ch[n].cfg = val; break;
        default: break;
        }
        return;
    }

    switch (offset) {
    case DMAC_GCFG:   s->gcfg = val; break;
    case DMAC_EN:     s->en = val & 0x1; break;
    case DMAC_EBCIER: s->ebcimr |= val; dmac_update_irq(s); break;
    case DMAC_EBCIDR: s->ebcimr &= ~val; dmac_update_irq(s); break;
    case DMAC_CHER: {
        int n;
        for (n = 0; n < DMAC_N_CHANNELS; n++) {
            if (val & (1u << n)) {
                s->chsr |= 1u << n;
                dmac_run_channel(s, n);   /* synchronous transfer */
            }
        }
        break;
    }
    case DMAC_CHDR:
        s->chsr &= ~(val & 0xFF);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-dmac: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps dmac_ops = {
    .read = dmac_read,
    .write = dmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void dmac_reset(DeviceState *dev)
{
    AT91DmacState *s = AT91_DMAC(dev);

    s->gcfg = 0x10;
    s->en = 0;
    s->ebcimr = 0;
    s->ebcisr = 0;
    s->chsr = 0;
    memset(s->ch, 0, sizeof(s->ch));
}

static void dmac_dev_init(Object *obj)
{
    AT91DmacState *s = AT91_DMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &dmac_ops, s, "at91-dmac", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void dmac_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), dmac_reset);
}

static const TypeInfo dmac_type = {
    .name = TYPE_AT91_DMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91DmacState),
    .instance_init = dmac_dev_init,
    .class_init = dmac_class_init,
};

/* ======================================================================== */
/*  RSTC / SHDWC / WDT (datasheet sections 11, 16, 15)                        */
/*                                                                           */
/*  Reset and shutdown are wired to the QEMU run-state so that guest reboot  */
/*  and poweroff behave correctly.  The watchdog is a benign stub: it        */
/*  accepts its (write-once) mode register at the datasheet reset value so   */
/*  the kernel driver probes cleanly, but never actually fires.              */
/* ======================================================================== */

/* KEY password (bits [31:24]) shared by RSTC_CR, SHDW_CR and WDT_CR. */
#define AT91_WPKEY  0xA5
static inline bool at91_key_ok(uint32_t val)
{
    return (val >> 24) == AT91_WPKEY;
}

/* --- RSTC (0xFFFFFD00) --- */
#define RSTC_CR   0x00
#define RSTC_SR   0x04
#define RSTC_MR   0x08
#define RSTC_CR_PROCRST  (1u << 0)
#define RSTC_CR_PERRST   (1u << 2)
#define RSTC_CR_EXTRST   (1u << 3)
#define RSTC_SR_RESET    0x00000001   /* general reset, NRST high, no cmd busy */

#define TYPE_AT91_RSTC "at91-rstc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91RstcState, AT91_RSTC)
struct AT91RstcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mr;
};

static uint64_t rstc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91RstcState *s = AT91_RSTC(opaque);

    switch (offset) {
    case RSTC_SR: return RSTC_SR_RESET;
    case RSTC_MR: return s->mr;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-rstc: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void rstc_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AT91RstcState *s = AT91_RSTC(opaque);
    uint32_t val = value;

    switch (offset) {
    case RSTC_CR:
        if (at91_key_ok(val) &&
            (val & (RSTC_CR_PROCRST | RSTC_CR_PERRST | RSTC_CR_EXTRST))) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    case RSTC_MR:
        if (at91_key_ok(val)) {
            s->mr = val & 0x00000f11;   /* URSTEN, URSTIEN, ERSTL */
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-rstc: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps rstc_ops = {
    .read = rstc_read,
    .write = rstc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void rstc_reset(DeviceState *dev)
{
    AT91RstcState *s = AT91_RSTC(dev);
    s->mr = 0x00000001;
}

static void rstc_dev_init(Object *obj)
{
    AT91RstcState *s = AT91_RSTC(obj);
    memory_region_init_io(&s->iomem, obj, &rstc_ops, s, "at91-rstc", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void rstc_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), rstc_reset);
}

static const TypeInfo rstc_type = {
    .name = TYPE_AT91_RSTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91RstcState),
    .instance_init = rstc_dev_init,
    .class_init = rstc_class_init,
};

/* --- SHDWC (0xFFFFFD10) --- */
#define SHDW_CR   0x00
#define SHDW_MR   0x04
#define SHDW_SR   0x08
#define SHDW_CR_SHDW  (1u << 0)

#define TYPE_AT91_SHDWC "at91-shdwc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91ShdwcState, AT91_SHDWC)
struct AT91ShdwcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mr;
};

static uint64_t shdwc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91ShdwcState *s = AT91_SHDWC(opaque);

    switch (offset) {
    case SHDW_MR: return s->mr;
    case SHDW_SR: return 0;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-shdwc: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void shdwc_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    AT91ShdwcState *s = AT91_SHDWC(opaque);
    uint32_t val = value;

    switch (offset) {
    case SHDW_CR:
        if (at91_key_ok(val) && (val & SHDW_CR_SHDW)) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
        break;
    case SHDW_MR:
        s->mr = val & 0x00070007;   /* WKMODE0, CPTWK0, RTTWKEN, RTCWKEN */
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-shdwc: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps shdwc_ops = {
    .read = shdwc_read,
    .write = shdwc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void shdwc_reset(DeviceState *dev)
{
    AT91ShdwcState *s = AT91_SHDWC(dev);
    s->mr = 0;
}

static void shdwc_dev_init(Object *obj)
{
    AT91ShdwcState *s = AT91_SHDWC(obj);
    memory_region_init_io(&s->iomem, obj, &shdwc_ops, s, "at91-shdwc", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void shdwc_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), shdwc_reset);
}

static const TypeInfo shdwc_type = {
    .name = TYPE_AT91_SHDWC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91ShdwcState),
    .instance_init = shdwc_dev_init,
    .class_init = shdwc_class_init,
};

/* --- WDT (0xFFFFFD40) - benign stub, never fires --- */
#define WDT_CR   0x00
#define WDT_MR   0x04
#define WDT_SR   0x08
#define WDT_MR_RESET  0x3FFF2FFF

#define TYPE_AT91_WDT "at91-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(AT91WdtState, AT91_WDT)
struct AT91WdtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mr;
    bool mr_written;      /* WDT_MR is write-once */
};

static uint64_t wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91WdtState *s = AT91_WDT(opaque);

    switch (offset) {
    case WDT_MR: return s->mr;
    case WDT_SR: return 0;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-wdt: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void wdt_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91WdtState *s = AT91_WDT(opaque);
    uint32_t val = value;

    switch (offset) {
    case WDT_CR:
        /* WDRSTT (with key) reloads the counter; we never fire, so no-op. */
        break;
    case WDT_MR:
        if (!s->mr_written) {       /* write-once */
            s->mr = val;
            s->mr_written = true;
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-wdt: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps wdt_ops = {
    .read = wdt_read,
    .write = wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void wdt_reset(DeviceState *dev)
{
    AT91WdtState *s = AT91_WDT(dev);
    s->mr = WDT_MR_RESET;
    s->mr_written = false;
}

static void wdt_dev_init(Object *obj)
{
    AT91WdtState *s = AT91_WDT(obj);
    memory_region_init_io(&s->iomem, obj, &wdt_ops, s, "at91-wdt", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void wdt_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), wdt_reset);
}

static const TypeInfo wdt_type = {
    .name = TYPE_AT91_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91WdtState),
    .instance_init = wdt_dev_init,
    .class_init = wdt_class_init,
};

static void at91_register_types(void)
{
    type_register_static(&dbgu_type);
    type_register_static(&aic_type);
    type_register_static(&pit_type);
    type_register_static(&pmc_type);
    type_register_static(&rstc_type);
    type_register_static(&shdwc_type);
    type_register_static(&wdt_type);
    type_register_static(&dmac_type);
    type_register_static_array(hsmci_types, ARRAY_SIZE(hsmci_types));
}

type_init(at91_register_types)

/* ======================================================================== */
/*  SAM9M10-G45-EK board                                                     */
/* ======================================================================== */

static struct arm_boot_info sam9m10g45ek_binfo;

/* Create an HSMCI controller, wire its interrupt to the AIC, and attach an
 * SD card from the matching -sd drive (if any). */
static void sam9_create_hsmci(hwaddr base, DeviceState *aic, int irqno,
                              int sd_unit)
{
    DeviceState *mci = qdev_new(TYPE_AT91_HSMCI);
    DriveInfo *di = drive_get(IF_SD, 0, sd_unit);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(mci), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mci), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(mci), 0, qdev_get_gpio_in(aic, irqno));

    if (di) {
        DeviceState *card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
        qdev_realize_and_unref(card, qdev_get_child_bus(mci, "sd-bus"),
                               &error_fatal);
    }
}

static void sam9m10g45ek_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    ARMCPU *cpu;
    DeviceState *dbgu, *aic, *pit, *pmc, *orgate;

    cpuobj = object_new(machine->cpu_type);
    /* Older ARMv5 cores have no EL3; guard is harmless if absent. */
    if (object_property_find(cpuobj, "has_el3")) {
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
    }
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    /* DDR2 SDRAM at the DDRSDRC0 chip select. */
    memory_region_add_subregion(sysmem, SAM9G45_SDRAM_BASE, machine->ram);

    /* Log (rather than abort on) any access to a not-yet-modelled
     * peripheral, so a boot shows how far it gets. */
    create_unimplemented_device("at91-periph", SAM9G45_PERIPH_BASE,
                                SAM9G45_PERIPH_SIZE);

    /* AIC: outputs drive the CPU's nIRQ/nFIQ. */
    aic = qdev_new(TYPE_AT91_AIC);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(aic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(aic), 0, SAM9G45_AIC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(aic), 0,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(aic), 1,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));

    /* System interrupt (AIC source 1) is the wired-OR of DBGU + PIT. */
    orgate = qdev_new(TYPE_OR_IRQ);
    qdev_prop_set_uint16(orgate, "num-lines", 2);
    qdev_realize_and_unref(orgate, NULL, &error_fatal);
    qdev_connect_gpio_out(orgate, 0,
                          qdev_get_gpio_in(aic, SAM9G45_IRQ_SYS));

    /* PMC clock controller. */
    pmc = qdev_new(TYPE_AT91_PMC);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pmc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pmc), 0, SAM9G45_PMC_BASE);

    /* DMA controller. */
    {
        DeviceState *dmac = qdev_new(TYPE_AT91_DMAC);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dmac), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dmac), 0, SAM9G45_DMAC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(dmac), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_DMAC));
    }

    /* Reset / shutdown / watchdog controllers. */
    sysbus_create_simple(TYPE_AT91_RSTC, SAM9G45_RSTC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_SHDWC, SAM9G45_SHDWC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_WDT, SAM9G45_WDT_BASE, NULL);

    /* High Speed MMC interfaces (SD via -sd / -drive if=sd). */
    sam9_create_hsmci(SAM9G45_HSMCI0_BASE, aic, SAM9G45_IRQ_HSMCI0, 0);
    sam9_create_hsmci(SAM9G45_HSMCI1_BASE, aic, SAM9G45_IRQ_HSMCI1, 1);

    /* USB host: OHCI (FS/LS) + EHCI (HS) share AIC source 22 (UHPHS). */
    {
        DeviceState *ohci, *ehci, *usb_or;

        usb_or = qdev_new(TYPE_OR_IRQ);
        qdev_prop_set_uint16(usb_or, "num-lines", 2);
        qdev_realize_and_unref(usb_or, NULL, &error_fatal);
        qdev_connect_gpio_out(usb_or, 0,
                              qdev_get_gpio_in(aic, SAM9G45_IRQ_UHPHS));

        ohci = qdev_new(TYPE_SYSBUS_OHCI);
        qdev_prop_set_uint32(ohci, "num-ports", 2);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ohci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ohci), 0, SAM9G45_OHCI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(ohci), 0,
                           qdev_get_gpio_in(usb_or, 0));

        ehci = qdev_new(TYPE_PLATFORM_EHCI);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ehci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ehci), 0, SAM9G45_EHCI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(ehci), 0,
                           qdev_get_gpio_in(usb_or, 1));
    }

    /* DBGU console -> OR gate input 0. */
    dbgu = qdev_new(TYPE_AT91_DBGU);
    qdev_prop_set_chr(dbgu, "chardev", serial_hd(0));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dbgu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dbgu), 0, SAM9G45_DBGU_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dbgu), 0,
                       qdev_get_gpio_in(orgate, 0));

    /* PIT system tick -> OR gate input 1. */
    pit = qdev_new(TYPE_AT91_PIT);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pit), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pit), 0, SAM9G45_PIT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(pit), 0,
                       qdev_get_gpio_in(orgate, 1));

    sam9m10g45ek_binfo.loader_start = SAM9G45_SDRAM_BASE;
    sam9m10g45ek_binfo.ram_size = machine->ram_size;
    arm_load_kernel(cpu, machine, &sam9m10g45ek_binfo);
}

static void sam9m10g45ek_machine_init(MachineClass *mc)
{
    mc->desc = "Atmel SAM9M10-G45-EK (AT91SAM9G45, ARM926EJ-S)";
    mc->init = sam9m10g45ek_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_id = "sam9g45.ram";
    mc->default_ram_size = SAM9G45_DEFAULT_RAM;
    /* Bring-up: don't hard-abort on stray MMIO to unmodelled peripherals. */
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("sam9m10g45ek", sam9m10g45ek_machine_init)
