/*
 * Atmel/Microchip AT91SAM9G45 Touch Screen ADC Controller (TSADCC).
 *
 * SAR ADC with an integrated 4-wire resistive touchscreen sequencer
 * (datasheet Atmel-6438 section 39).  ADC-mode conversions return fixed
 * synthetic per-channel values; touchscreen contact and position come from
 * the QEMU absolute-pointer input layer, so pointer clicks/drags on a
 * display (or QMP input-send-event) appear to the guest as pen contacts.
 *
 * In Touchscreen Only mode the sequencer stores the panel measurements so
 * that the positions are the ratios y = CDR1/CDR0 and x = CDR3/CDR2 (the
 * denominators hold the full-scale panel span, the numerators the sensed
 * pen voltage), which is how both the datasheet and the Linux at91_adc
 * driver compute them.
 *
 * Unmodelled: pressure measurement (MR.PRES; XPDR/Z1DR/Z2DR read 0),
 * interleaved and manual modes, the external TSADTRG trigger (no source),
 * and the PDC (ENDRX/RXBUFF read as the level bits of permanently-empty
 * PDC counters, which matches the documented SR reset value 0x000C0000).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "ui/input.h"
#include "hw/adc/at91_tsadcc.h"
#include "trace.h"

#define TSADCC_CR      0x00   /* Control Register (WO)         */
#define TSADCC_MR      0x04   /* Mode Register                 */
#define TSADCC_TRGR    0x08   /* Trigger Register              */
#define TSADCC_TSR     0x0C   /* Touchscreen Register          */
#define TSADCC_CHER    0x10   /* Channel Enable Register (WO)  */
#define TSADCC_CHDR    0x14   /* Channel Disable Register (WO) */
#define TSADCC_CHSR    0x18   /* Channel Status Register (RO)  */
#define TSADCC_SR      0x1C   /* Status Register (RO)          */
#define TSADCC_LCDR    0x20   /* Last Converted Data (RO)      */
#define TSADCC_IER     0x24   /* Interrupt Enable (WO)         */
#define TSADCC_IDR     0x28   /* Interrupt Disable (WO)        */
#define TSADCC_IMR     0x2C   /* Interrupt Mask (RO)           */
#define TSADCC_CDR(n)  (0x30 + (n) * 4)   /* Channel Data 0..7 (RO) */
#define TSADCC_XPDR    0x50   /* X Position Data (RO)          */
#define TSADCC_Z1DR    0x54   /* Z1 Data (RO)                  */
#define TSADCC_Z2DR    0x58   /* Z2 Data (RO)                  */

#define CR_SWRST           (1u << 0)
#define CR_START           (1u << 1)

#define MR_TSAMOD(mr)      ((mr) & 3)
#define   TSAMOD_ADC           0
#define   TSAMOD_TS_ONLY       1
#define   TSAMOD_INTERLEAVED   2
#define   TSAMOD_MANUAL        3
#define MR_LOWRES          (1u << 4)
#define MR_PENDET          (1u << 6)
#define MR_PRESCAL(mr)     (((mr) >> 8) & 0xff)
#define MR_VALID_MASK      0xff7ffffbu    /* bits 2 and 23 reserved */

#define TRGR_TRGMOD(t)     ((t) & 7)
#define   TRGMOD_NONE          0
#define   TRGMOD_EXT_RISE      1
#define   TRGMOD_EXT_FALL      2
#define   TRGMOD_EXT_ANY       3
#define   TRGMOD_PENDET        4
#define   TRGMOD_PERIODIC      5
#define   TRGMOD_CONTINUOUS    6
#define TRGR_TRGPER(t)     (((t) >> 16) & 0xffff)
#define TRGR_VALID_MASK    0xffff0007u

#define TSR_VALID_MASK     0x0f00000fu    /* TSSHTIM[27:24], TSFREQ[3:0] */

#define SR_EOC(n)          (1u << (n))
#define SR_OVRE(n)         (1u << ((n) + 8))
#define SR_DRDY            (1u << 16)
#define SR_GOVRE           (1u << 17)
#define SR_ENDRX           (1u << 18)
#define SR_RXBUFF          (1u << 19)
#define SR_PENCNT          (1u << 20)
#define SR_NOCNT           (1u << 21)
/* OVRE0-7, GOVRE, PENCNT and NOCNT are cleared by reading SR. */
#define SR_READ_CLEARS     (0xff00u | SR_GOVRE | SR_PENCNT | SR_NOCNT)
/* Implemented interrupt sources (bits 22-23, 27 and 31 are reserved). */
#define SR_VALID_MASK      0x773fffffu

#define TSADCC_CHAN_MAX    0x3ffu    /* 10-bit conversions */

struct AT91TsadccState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *trigger_timer;
    QemuInputHandlerState *input;
    uint32_t mck_freq;     /* master clock (Hz); ADCCLK is divided from it */

    uint32_t mr;
    uint32_t trgr;
    uint32_t tsr;
    uint32_t chsr;
    uint32_t sr;
    uint32_t imr;
    uint32_t lcdr;
    uint32_t last_conv;    /* channel whose EOC an LCDR read also clears */
    uint32_t cdr[8];

    /* pen state, fed by the QEMU input layer */
    bool pen_down;
    uint32_t ts_x;         /* 10-bit panel positions */
    uint32_t ts_y;
};

static void tsadcc_update_irq(AT91TsadccState *s)
{
    qemu_set_irq(s->irq,
                 ((s->sr | SR_ENDRX | SR_RXBUFF) & s->imr) ? 1 : 0);
}

/* Synthetic conversion result for analog channel n. */
static uint32_t tsadcc_channel_value(AT91TsadccState *s, int n)
{
    uint32_t val = 0x200 + 0x40 * n;

    if ((s->mr & MR_LOWRES) && MR_TSAMOD(s->mr) == TSAMOD_ADC) {
        val >>= 2;    /* 8-bit resolution */
    }
    return val;
}

static void tsadcc_store(AT91TsadccState *s, int n, uint32_t val)
{
    if (s->sr & SR_EOC(n)) {
        s->sr |= SR_OVRE(n);
    }
    if (s->sr & SR_DRDY) {
        s->sr |= SR_GOVRE;
    }
    s->cdr[n] = val & TSADCC_CHAN_MAX;
    s->lcdr = val & TSADCC_CHAN_MAX;
    s->last_conv = n;
    s->sr |= SR_EOC(n) | SR_DRDY;
}

/* One conversion sequence (one trigger), per the current operating mode. */
static void tsadcc_convert(AT91TsadccState *s)
{
    uint32_t mode = MR_TSAMOD(s->mr);
    int n;

    if (mode == TSAMOD_TS_ONLY) {
        s->chsr |= 0xf;    /* CH0-3 auto-activate in touchscreen mode */
        tsadcc_store(s, 0, TSADCC_CHAN_MAX);
        tsadcc_store(s, 1, (s->ts_y * TSADCC_CHAN_MAX) >> 10);
        tsadcc_store(s, 2, TSADCC_CHAN_MAX);
        tsadcc_store(s, 3, (s->ts_x * TSADCC_CHAN_MAX) >> 10);
    }
    /* The remaining (in ADC mode: all) enabled channels. */
    for (n = (mode == TSAMOD_TS_ONLY) ? 4 : 0; n < 8; n++) {
        if (s->chsr & (1u << n)) {
            tsadcc_store(s, n, tsadcc_channel_value(s, n));
        }
    }
    trace_at91_tsadcc_convert(mode, s->chsr, s->pen_down, s->ts_x, s->ts_y);
    tsadcc_update_irq(s);
}

/* ADCCLK = MCK / ((PRESCAL + 1) * 2) (MR). */
static uint64_t tsadcc_adcclk(AT91TsadccState *s)
{
    return MAX(s->mck_freq / ((MR_PRESCAL(s->mr) + 1) * 2), 1u);
}

static int64_t tsadcc_trigger_period_ns(AT91TsadccState *s)
{
    uint64_t cycles;

    if (TRGR_TRGMOD(s->trgr) == TRGMOD_CONTINUOUS) {
        /* Back-to-back sequences: startup + conversions, in ADCCLK cycles. */
        uint32_t startup = ((s->mr >> 16) & 0x7f) + 1;
        uint32_t shtim = (s->mr >> 24) & 0xf;

        cycles = startup * 8 + ctpop32(s->chsr & 0xff) * (shtim + 10);
    } else {
        /* Periodic trigger: period = (TRGPER + 1) / ADCCLK. */
        cycles = TRGR_TRGPER(s->trgr) + 1;
    }
    /*
     * Functional floor so a tiny TRGPER/PRESCAL cannot turn the trigger
     * into a timer storm (Linux programs ~10 ms for touch sampling).
     */
    return MAX(muldiv64(cycles, NANOSECONDS_PER_SECOND, tsadcc_adcclk(s)),
               100 * 1000);
}

static void tsadcc_trigger_rearm(AT91TsadccState *s)
{
    switch (TRGR_TRGMOD(s->trgr)) {
    case TRGMOD_PERIODIC:
    case TRGMOD_CONTINUOUS:
        timer_mod(s->trigger_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  tsadcc_trigger_period_ns(s));
        break;
    case TRGMOD_EXT_RISE:
    case TRGMOD_EXT_FALL:
    case TRGMOD_EXT_ANY:
        qemu_log_mask(LOG_UNIMP,
                      "at91-tsadcc: external TSADTRG trigger not modelled\n");
        /* fall through */
    default:
        timer_del(s->trigger_timer);
        break;
    }
}

static void tsadcc_trigger_fire(void *opaque)
{
    AT91TsadccState *s = opaque;

    tsadcc_convert(s);
    tsadcc_trigger_rearm(s);
}

/* Pen contact changed (or pen detection was just enabled with contact). */
static void tsadcc_set_pen(AT91TsadccState *s, bool down)
{
    if (down == s->pen_down) {
        return;
    }
    s->pen_down = down;
    trace_at91_tsadcc_pen(down);
    if (!(s->mr & MR_PENDET)) {
        return;
    }
    s->sr |= down ? SR_PENCNT : SR_NOCNT;
    if (down && TRGR_TRGMOD(s->trgr) == TRGMOD_PENDET) {
        tsadcc_convert(s);    /* pen-detect hardware trigger */
    }
    tsadcc_update_irq(s);
}

static void tsadcc_input_event(DeviceState *dev, QemuConsole *src,
                               QemuInputEvent *evt)
{
    AT91TsadccState *s = AT91_TSADCC(dev);
    uint32_t pos;

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        pos = qemu_input_scale_axis(evt->abs.value, INPUT_EVENT_ABS_MIN,
                                    INPUT_EVENT_ABS_MAX, 0, TSADCC_CHAN_MAX);
        if (evt->abs.axis == INPUT_AXIS_X) {
            s->ts_x = pos;
        } else if (evt->abs.axis == INPUT_AXIS_Y) {
            s->ts_y = pos;
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button == INPUT_BUTTON_LEFT) {
            tsadcc_set_pen(s, evt->btn.down);
        }
        break;
    default:
        break;
    }
}

static const QemuInputHandler tsadcc_input_handler = {
    .name = "AT91 TSADCC Touchscreen",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = tsadcc_input_event,
};

static void tsadcc_soft_reset(AT91TsadccState *s)
{
    timer_del(s->trigger_timer);
    s->mr = 0;
    s->trgr = 0;
    s->tsr = 0;
    s->chsr = 0;
    s->sr = 0;      /* ENDRX/RXBUFF (SR reset 0x000C0000) are OR'd at read */
    s->imr = 0;
    s->lcdr = 0;
    s->last_conv = 0;
    memset(s->cdr, 0, sizeof(s->cdr));
    tsadcc_update_irq(s);
}

static uint64_t tsadcc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91TsadccState *s = AT91_TSADCC(opaque);
    uint32_t r = 0;
    int n;

    switch (offset) {
    case TSADCC_MR:
        r = s->mr;
        break;
    case TSADCC_TRGR:
        r = s->trgr;
        break;
    case TSADCC_TSR:
        r = s->tsr;
        break;
    case TSADCC_CHSR:
        r = s->chsr;
        break;
    case TSADCC_SR:
        r = s->sr | SR_ENDRX | SR_RXBUFF;
        s->sr &= ~SR_READ_CLEARS;
        tsadcc_update_irq(s);
        break;
    case TSADCC_LCDR:
        r = s->lcdr;
        s->sr &= ~(SR_DRDY | SR_EOC(s->last_conv));
        tsadcc_update_irq(s);
        break;
    case TSADCC_IMR:
        r = s->imr;
        break;
    case TSADCC_CDR(0) ... TSADCC_CDR(7):
        n = (offset - TSADCC_CDR(0)) >> 2;
        r = s->cdr[n];
        s->sr &= ~SR_EOC(n);
        tsadcc_update_irq(s);
        break;
    case TSADCC_XPDR:
    case TSADCC_Z1DR:
    case TSADCC_Z2DR:
        /* pressure measurement (MR.PRES) is not modelled */
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-tsadcc: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
        break;
    }
    return r;
}

static void tsadcc_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
    AT91TsadccState *s = AT91_TSADCC(opaque);
    bool pendet_was;

    switch (offset) {
    case TSADCC_CR:
        if (value & CR_SWRST) {
            tsadcc_soft_reset(s);
        }
        if (value & CR_START) {
            tsadcc_convert(s);
        }
        break;
    case TSADCC_MR:
        pendet_was = s->mr & MR_PENDET;
        s->mr = value & MR_VALID_MASK;
        if (MR_TSAMOD(s->mr) == TSAMOD_INTERLEAVED ||
            MR_TSAMOD(s->mr) == TSAMOD_MANUAL) {
            qemu_log_mask(LOG_UNIMP, "at91-tsadcc: TSAMOD %u (interleaved/"
                          "manual) not modelled\n", MR_TSAMOD(s->mr));
        }
        if (!(s->mr & MR_PENDET)) {
            /* PENCNT/NOCNT read as 0 while pen detection is off. */
            s->sr &= ~(SR_PENCNT | SR_NOCNT);
        } else if (!pendet_was && s->pen_down) {
            /* Contact already present when detection was enabled. */
            s->sr |= SR_PENCNT;
        }
        if (MR_TSAMOD(s->mr) == TSAMOD_TS_ONLY) {
            s->chsr |= 0xf;
        }
        tsadcc_update_irq(s);
        break;
    case TSADCC_TRGR:
        s->trgr = value & TRGR_VALID_MASK;
        tsadcc_trigger_rearm(s);
        break;
    case TSADCC_TSR:
        s->tsr = value & TSR_VALID_MASK;
        break;
    case TSADCC_CHER:
        s->chsr |= value & 0xff;
        break;
    case TSADCC_CHDR:
        s->chsr &= ~(value & 0xff);
        break;
    case TSADCC_IER:
        s->imr |= value & SR_VALID_MASK;
        tsadcc_update_irq(s);
        break;
    case TSADCC_IDR:
        s->imr &= ~value;
        tsadcc_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-tsadcc: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n",
                      offset, (uint32_t)value);
        break;
    }
}

static const MemoryRegionOps tsadcc_ops = {
    .read = tsadcc_read,
    .write = tsadcc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void tsadcc_reset(DeviceState *dev)
{
    /* The pen state deliberately survives reset (it is the physical panel). */
    tsadcc_soft_reset(AT91_TSADCC(dev));
}

static void tsadcc_realize(DeviceState *dev, Error **errp)
{
    AT91TsadccState *s = AT91_TSADCC(dev);

    if (s->mck_freq == 0) {
        error_setg(errp, "at91-tsadcc: mck-frequency must be set");
        return;
    }
    s->trigger_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    tsadcc_trigger_fire, s);
    s->input = qemu_input_handler_register(dev, &tsadcc_input_handler);
    qemu_input_handler_activate(s->input);
}

static void tsadcc_dev_init(Object *obj)
{
    AT91TsadccState *s = AT91_TSADCC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &tsadcc_ops, s, "at91-tsadcc",
                          0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    /* Until the pointer moves, a pen contact lands mid-panel. */
    s->ts_x = 0x200;
    s->ts_y = 0x200;
}

/* Nominal fallback; the board sets the real MCK via the mck-frequency prop. */
#define AT91_TSADCC_DEFAULT_MCK_HZ  132000000

static const Property tsadcc_properties[] = {
    DEFINE_PROP_UINT32("mck-frequency", AT91TsadccState, mck_freq,
                       AT91_TSADCC_DEFAULT_MCK_HZ),
};

static const VMStateDescription vmstate_at91_tsadcc = {
    .name = "at91-tsadcc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91TsadccState),
        VMSTATE_UINT32(trgr, AT91TsadccState),
        VMSTATE_UINT32(tsr, AT91TsadccState),
        VMSTATE_UINT32(chsr, AT91TsadccState),
        VMSTATE_UINT32(sr, AT91TsadccState),
        VMSTATE_UINT32(imr, AT91TsadccState),
        VMSTATE_UINT32(lcdr, AT91TsadccState),
        VMSTATE_UINT32(last_conv, AT91TsadccState),
        VMSTATE_UINT32_ARRAY(cdr, AT91TsadccState, 8),
        VMSTATE_BOOL(pen_down, AT91TsadccState),
        VMSTATE_UINT32(ts_x, AT91TsadccState),
        VMSTATE_UINT32(ts_y, AT91TsadccState),
        VMSTATE_TIMER_PTR(trigger_timer, AT91TsadccState),
        VMSTATE_END_OF_LIST()
    }
};

static void tsadcc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tsadcc_realize;
    device_class_set_legacy_reset(dc, tsadcc_reset);
    device_class_set_props(dc, tsadcc_properties);
    dc->vmsd = &vmstate_at91_tsadcc;
}

static const TypeInfo tsadcc_type = {
    .name = TYPE_AT91_TSADCC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91TsadccState),
    .instance_init = tsadcc_dev_init,
    .class_init = tsadcc_class_init,
};

static void at91_tsadcc_register_types(void)
{
    type_register_static(&tsadcc_type);
}

type_init(at91_tsadcc_register_types)
