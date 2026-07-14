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
#include "chardev/char-fe.h"
#include "hw/misc/unimp.h"
#include "qom/object.h"
#include "target/arm/cpu-qom.h"

/* ---- SoC memory map (datasheet Fig 5-1, section 6.1) -------------------- */
#define SAM9G45_SDRAM_BASE   0x70000000   /* DDRSDRC0 chip select (main RAM) */
#define SAM9G45_PERIPH_BASE  0xFFF78000   /* start of APB peripheral window  */
#define SAM9G45_PERIPH_SIZE  0x00088000   /* ...through 0xFFFFFFFF            */
#define SAM9G45_DBGU_BASE    0xFFFFEE00   /* Debug Unit (console UART)        */
#define SAM9G45_AIC_BASE     0xFFFFF000   /* Advanced Interrupt Controller    */
#define SAM9G45_PMC_BASE     0xFFFFFC00   /* Power Management Controller       */
#define SAM9G45_RSTC_BASE    0xFFFFFD00   /* Reset Controller                  */
#define SAM9G45_SHDWC_BASE   0xFFFFFD10   /* Shutdown Controller               */
#define SAM9G45_PIT_BASE     0xFFFFFD30   /* Periodic Interval Timer           */
#define SAM9G45_WDT_BASE     0xFFFFFD40   /* Watchdog Timer                    */

#define SAM9G45_DEFAULT_RAM  (128 * MiB)  /* SAM9M10-G45-EK: 128 MB DDR2      */

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
}

type_init(at91_register_types)

/* ======================================================================== */
/*  SAM9M10-G45-EK board                                                     */
/* ======================================================================== */

static struct arm_boot_info sam9m10g45ek_binfo;

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

    /* Reset / shutdown / watchdog controllers. */
    sysbus_create_simple(TYPE_AT91_RSTC, SAM9G45_RSTC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_SHDWC, SAM9G45_SHDWC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_WDT, SAM9G45_WDT_BASE, NULL);

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
