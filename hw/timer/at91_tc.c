/*
 * Atmel/Microchip AT91 Timer Counter (TC) block - 3 x 16-bit channels.
 *
 * Modelled for the waveform-mode uses the Linux tcb drivers make of it
 * (tcb_clksrc / timer-atmel-tcb):
 *   - a channel free-running at an MCK-derived rate (WAVSEL=UP), read
 *     through CV: clocksource low word;
 *   - the next channel clocked from XC1 = TIOA0 (BMR TC1XC1S), counting
 *     one tick per 65536 ticks of channel 0 (channel 0 toggles TIOA0 via
 *     ACPA at RA=0 / ACPC at RC=0x8000): clocksource high word.  This
 *     chaining is modelled as a rate divider rather than as edge events;
 *   - a channel in WAVSEL=UP_RC with an RC compare interrupt (CPCS) as
 *     the tick clockevent, periodic (auto-restart) or one-shot
 *     (CPCSTOP/CPCDIS), usually on TIMER_CLOCK5 = the 32.768 kHz slow
 *     clock.
 *
 * Capture mode (WAVE=0), RA/RB loading, external triggers and the
 * quadrature decoder are not modelled (logged as unimplemented).  COVFS
 * overflow status is not maintained for free-running channels - the
 * Linux drivers never read it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/timer/at91_tc.h"
#include "trace.h"

#define TC_NCH        3
#define TC_CH_SPAN    0x40

/* Per-channel registers (offset within the channel) */
#define TC_CCR        0x00   /* Channel Control (write-only)     */
#define TC_CMR        0x04   /* Channel Mode                     */
#define TC_SMMR       0x08   /* Stepper Motor Mode               */
#define TC_CV         0x10   /* Counter Value (read-only)        */
#define TC_RA         0x14
#define TC_RB         0x18
#define TC_RC         0x1C
#define TC_SR         0x20   /* Status (read-only, clear-on-read) */
#define TC_IER        0x24
#define TC_IDR        0x28
#define TC_IMR        0x2C

/* Block registers */
#define TC_BCR        0xC0
#define TC_BMR        0xC4

#define TC_CCR_CLKEN  (1u << 0)
#define TC_CCR_CLKDIS (1u << 1)
#define TC_CCR_SWTRG  (1u << 2)

#define TC_CMR_TCCLKS(cmr)  ((cmr) & 7)
#define TC_CMR_CPCSTOP      (1u << 6)
#define TC_CMR_CPCDIS       (1u << 7)
#define TC_CMR_WAVSEL(cmr)  (((cmr) >> 13) & 3)
#define TC_CMR_WAVE         (1u << 15)

#define TC_WAVSEL_UP        0
#define TC_WAVSEL_UP_RC     2

#define TC_SR_COVFS   (1u << 0)
#define TC_SR_CPCS    (1u << 4)
#define TC_SR_CLKSTA  (1u << 16)
#define TC_SR_EVENTS  0xFF

#define TC_BCR_SYNC   (1u << 0)

#define TC_BMR_TC1XC1S(bmr)  (((bmr) >> 2) & 3)
#define TC_XC1S_TIOA0        2

typedef struct AT91TcChan {
    uint32_t cmr;
    uint32_t ra, rb, rc;
    uint32_t imr;
    uint32_t sr;           /* event bits only (COVFS..ETRGS) */
    bool clken;
    int64_t epoch;         /* vtime when the counter was last (re)started */
    uint32_t cv_frozen;    /* CV held while the clock is disabled */
    QEMUTimer *timer;
    struct AT91TcState *parent;   /* back-link for the timer callback */
    int idx;
} AT91TcChan;

struct AT91TcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t mck_freq;
    uint32_t slck_freq;
    uint32_t bmr;

    AT91TcChan ch[TC_NCH];
};

/* Input clock in Hz for a channel; 0 if unmodelled (external XC input
 * other than the TIOA0 chain). */
static uint64_t tc_rate(AT91TcState *s, int n)
{
    static const uint32_t mck_div[4] = { 2, 8, 32, 128 };
    uint32_t tcclks = TC_CMR_TCCLKS(s->ch[n].cmr);

    if (tcclks < 4) {
        return s->mck_freq / mck_div[tcclks];
    }
    if (tcclks == 4) {
        return s->slck_freq;
    }
    /* XC inputs: model only XC1 = TIOA0 (the tcb_clksrc 16-bit chain):
     * channel 0 produces one TIOA0 period per 65536 of its own ticks. */
    if (n == 1 && tcclks == 6 && TC_BMR_TC1XC1S(s->bmr) == TC_XC1S_TIOA0) {
        return tc_rate(s, 0) >> 16;
    }
    return 0;
}

static uint64_t tc_ticks(AT91TcState *s, int n, int64_t now)
{
    uint64_t rate = tc_rate(s, n);

    if (!rate) {
        return 0;
    }
    /*
     * The chained channel is the exact high word of channel 0.  Do not
     * approximate it by multiplying elapsed time by (rate >> 16): integer
     * truncation would make the high word lag the low word and the composed
     * 32-bit clocksource would jump backwards whenever channel 0 wraps.
     */
    if (n == 1 && TC_CMR_TCCLKS(s->ch[n].cmr) == 6) {
        uint64_t low_rate = tc_rate(s, 0);

        return muldiv64(now - s->ch[0].epoch, low_rate,
                        NANOSECONDS_PER_SECOND) >> 16;
    }
    return muldiv64(now - s->ch[n].epoch, rate, NANOSECONDS_PER_SECOND);
}

static uint32_t tc_cv(AT91TcState *s, int n, int64_t now)
{
    AT91TcChan *c = &s->ch[n];
    uint64_t ticks;

    if (!c->clken) {
        return c->cv_frozen;
    }
    ticks = tc_ticks(s, n, now);
    if (TC_CMR_WAVSEL(c->cmr) == TC_WAVSEL_UP_RC && c->rc) {
        return ticks % c->rc;
    }
    return ticks & 0xFFFF;
}

static void tc_update_irq(AT91TcState *s)
{
    int n;
    bool level = false;

    for (n = 0; n < TC_NCH; n++) {
        if (s->ch[n].sr & s->ch[n].imr) {
            level = true;
        }
    }
    qemu_set_irq(s->irq, level);
}

/* Arm the RC-compare timer for a WAVSEL=UP_RC channel. */
static void tc_rearm(AT91TcState *s, int n)
{
    AT91TcChan *c = &s->ch[n];
    uint64_t rate = tc_rate(s, n);
    int64_t period;

    if (!c->clken || !c->rc || !rate ||
        TC_CMR_WAVSEL(c->cmr) != TC_WAVSEL_UP_RC) {
        timer_del(c->timer);
        return;
    }
    period = muldiv64(c->rc, NANOSECONDS_PER_SECOND, rate);
    period = MAX(period, 1000);   /* keep pathological RC values sane */
    timer_mod(c->timer, c->epoch + period);
}

static void tc_compare_fire(void *opaque)
{
    AT91TcChan *c = opaque;
    AT91TcState *s = c->parent;
    int n = c->idx;

    c->sr |= TC_SR_CPCS;
    trace_at91_tc_fire(n);
    if (c->cmr & (TC_CMR_CPCDIS | TC_CMR_CPCSTOP)) {
        /* one-shot: the RC compare stops/disables the clock */
        c->cv_frozen = 0;
        c->clken = false;
        timer_del(c->timer);
    } else {
        /* periodic: counter restarted at the compare */
        c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        tc_rearm(s, n);
    }
    tc_update_irq(s);
}

static void tc_trigger(AT91TcState *s, int n)
{
    s->ch[n].epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    tc_rearm(s, n);
}

static uint64_t tc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91TcState *s = AT91_TC(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t r = 0;
    int n = offset / TC_CH_SPAN;

    if (n < TC_NCH) {
        AT91TcChan *c = &s->ch[n];

        switch (offset % TC_CH_SPAN) {
        case TC_CMR:  r = c->cmr; break;
        case TC_CV:   r = tc_cv(s, n, now); break;
        case TC_RA:   r = c->ra; break;
        case TC_RB:   r = c->rb; break;
        case TC_RC:   r = c->rc; break;
        case TC_SR:
            r = c->sr | (c->clken ? TC_SR_CLKSTA : 0);
            c->sr &= ~TC_SR_EVENTS;
            tc_update_irq(s);
            break;
        case TC_IMR:  r = c->imr; break;
        default:
            qemu_log_mask(LOG_UNIMP, "at91-tc: read from unimplemented "
                          "ch%d offset 0x%02x\n", n,
                          (unsigned)(offset % TC_CH_SPAN));
            break;
        }
    } else if (offset == TC_BMR) {
        r = s->bmr;
    } else {
        qemu_log_mask(LOG_UNIMP, "at91-tc: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
    }
    return r;
}

static void tc_write(void *opaque, hwaddr offset, uint64_t value,
                     unsigned size)
{
    AT91TcState *s = AT91_TC(opaque);
    int n = offset / TC_CH_SPAN;

    if (n < TC_NCH) {
        AT91TcChan *c = &s->ch[n];

        switch (offset % TC_CH_SPAN) {
        case TC_CCR:
            trace_at91_tc_ccr(n, (uint32_t)value);
            if (value & TC_CCR_CLKDIS) {
                c->cv_frozen = tc_cv(s, n,
                                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
                c->clken = false;
                timer_del(c->timer);
            } else if (value & TC_CCR_CLKEN) {
                if (!c->clken) {
                    c->clken = true;
                    tc_trigger(s, n);
                }
            }
            if (value & TC_CCR_SWTRG) {
                tc_trigger(s, n);
            }
            break;
        case TC_CMR:
            c->cmr = value;
            trace_at91_tc_cmr(n, (uint32_t)value);
            if (!(value & TC_CMR_WAVE)) {
                qemu_log_mask(LOG_UNIMP,
                              "at91-tc: ch%d capture mode not modelled\n", n);
            }
            tc_rearm(s, n);
            break;
        case TC_RA:   c->ra = value & 0xFFFF; break;
        case TC_RB:   c->rb = value & 0xFFFF; break;
        case TC_RC:
            c->rc = value & 0xFFFF;
            tc_rearm(s, n);
            break;
        case TC_IER:
            c->imr |= value & TC_SR_EVENTS;
            tc_update_irq(s);
            break;
        case TC_IDR:
            c->imr &= ~(value & TC_SR_EVENTS);
            tc_update_irq(s);
            break;
        case TC_SMMR:
            qemu_log_mask(LOG_UNIMP, "at91-tc: ch%d SMMR not modelled\n", n);
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "at91-tc: write to unimplemented "
                          "ch%d offset 0x%02x = 0x%08x\n", n,
                          (unsigned)(offset % TC_CH_SPAN), (uint32_t)value);
            break;
        }
    } else if (offset == TC_BCR) {
        if (value & TC_BCR_SYNC) {
            for (n = 0; n < TC_NCH; n++) {
                if (s->ch[n].clken) {
                    tc_trigger(s, n);
                }
            }
        }
    } else if (offset == TC_BMR) {
        s->bmr = value;
    } else {
        qemu_log_mask(LOG_UNIMP, "at91-tc: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n",
                      offset, (uint32_t)value);
    }
}

static const MemoryRegionOps tc_ops = {
    .read = tc_read,
    .write = tc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void tc_reset(DeviceState *dev)
{
    AT91TcState *s = AT91_TC(dev);
    int n;

    for (n = 0; n < TC_NCH; n++) {
        AT91TcChan *c = &s->ch[n];

        if (c->timer) {
            timer_del(c->timer);
        }
        c->cmr = c->ra = c->rb = c->rc = 0;
        c->imr = c->sr = 0;
        c->clken = false;
        c->cv_frozen = 0;
        c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
    s->bmr = 0;
    tc_update_irq(s);
}

static void tc_realize(DeviceState *dev, Error **errp)
{
    AT91TcState *s = AT91_TC(dev);
    int n;

    if (s->mck_freq == 0) {
        error_setg(errp, "at91-tc: mck-frequency must be set");
        return;
    }
    for (n = 0; n < TC_NCH; n++) {
        s->ch[n].parent = s;
        s->ch[n].idx = n;
        s->ch[n].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc_compare_fire,
                                      &s->ch[n]);
    }
}

static void tc_dev_init(Object *obj)
{
    AT91TcState *s = AT91_TC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &tc_ops, s, "at91-tc", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property tc_properties[] = {
    DEFINE_PROP_UINT32("mck-frequency", AT91TcState, mck_freq, 100000000),
    DEFINE_PROP_UINT32("slck-frequency", AT91TcState, slck_freq, 32768),
};

static const VMStateDescription vmstate_at91_tc_chan = {
    .name = "at91-tc-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cmr, AT91TcChan),
        VMSTATE_UINT32(ra, AT91TcChan),
        VMSTATE_UINT32(rb, AT91TcChan),
        VMSTATE_UINT32(rc, AT91TcChan),
        VMSTATE_UINT32(imr, AT91TcChan),
        VMSTATE_UINT32(sr, AT91TcChan),
        VMSTATE_BOOL(clken, AT91TcChan),
        VMSTATE_INT64(epoch, AT91TcChan),
        VMSTATE_UINT32(cv_frozen, AT91TcChan),
        VMSTATE_TIMER_PTR(timer, AT91TcChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_at91_tc = {
    .name = "at91-tc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(bmr, AT91TcState),
        VMSTATE_STRUCT_ARRAY(ch, AT91TcState, TC_NCH, 1,
                             vmstate_at91_tc_chan, AT91TcChan),
        VMSTATE_END_OF_LIST()
    }
};

static void tc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tc_realize;
    device_class_set_legacy_reset(dc, tc_reset);
    device_class_set_props(dc, tc_properties);
    dc->vmsd = &vmstate_at91_tc;
}

static const TypeInfo tc_type = {
    .name = TYPE_AT91_TC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91TcState),
    .instance_init = tc_dev_init,
    .class_init = tc_class_init,
};

static void at91_tc_register_types(void)
{
    type_register_static(&tc_type);
}

type_init(at91_tc_register_types)
