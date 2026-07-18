/*
 * Atmel/Microchip AT91 backup-area timekeeping: RTC + RTT + GPBR.
 *
 * Three small devices that all live in the 0xFFFFFDxx system-controller area
 * and share AIC source 1 (the system wired-OR):
 *
 *  - TYPE_AT91_RTC  (0xFFFFFDB0, "atmel,at91rm9200-rtc"): a BCD time/calendar
 *    RTC.  Seeded from the host clock, it advances one second at a time off a
 *    virtual-clock timer that also raises SECEV/ACKUPD/ALARM on AIC 1 - the
 *    Linux driver's settime path blocks on the ACKUPD interrupt, so the IRQ is
 *    mandatory.
 *  - TYPE_AT91_RTT  (0xFFFFFD20, "atmel,at91sam9260-rtt"): a 32-bit up-counter
 *    (VR) ticking at slck/RTPRES, with alarm (AR/ALMS) and increment (RTTINC)
 *    events.  The rtc-at91sam9 driver reads real time as GPBR_offset + VR.
 *  - TYPE_AT91_GPBR (0xFFFFFD60, "atmel,at91sam9260-gpbr"): 4 battery-backup
 *    scratch registers; the RTT-rtc stores its epoch base in one of them.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/bcd.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "system/rtc.h"
#include "hw/rtc/at91_rtc.h"
#include "trace.h"

/* ===================================================================== */
/*  RM9200 RTC (0xFFFFFDB0)                                               */
/* ===================================================================== */

#define RTC_CR      0x00
#define  CR_UPDTIM   (1u << 0)
#define  CR_UPDCAL   (1u << 1)
#define RTC_MR      0x04
#define RTC_TIMR    0x08
#define RTC_CALR    0x0c
#define RTC_TIMALR  0x10
#define RTC_CALALR  0x14
#define RTC_SR      0x18
#define  SR_ACKUPD   (1u << 0)
#define  SR_ALARM    (1u << 1)
#define  SR_SECEV    (1u << 2)
#define  SR_TIMEV    (1u << 3)
#define  SR_CALEV    (1u << 4)
#define RTC_SCCR    0x1c
#define RTC_IER     0x20
#define RTC_IDR     0x24
#define RTC_IMR     0x28
#define RTC_VER     0x2c

/* TIMALR/CALALR field-enable bits */
#define ALR_SECEN   (1u << 7)
#define ALR_MINEN   (1u << 15)
#define ALR_HOUREN  (1u << 23)
#define ALR_MTHEN   (1u << 23)
#define ALR_DATEEN  (1u << 31)

struct AT91RtcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer timer;

    int64_t offset;     /* epoch seconds = offset + virtual_clock_seconds */
    int64_t frozen;     /* epoch seconds latched while stopped            */
    uint32_t mr;
    uint32_t sr;
    uint32_t imr;
    uint32_t timalr;
    uint32_t calalr;
    uint32_t timr_latch;
    uint32_t calr_latch;
    bool stopped;       /* CR.UPDTIM|UPDCAL held: counter frozen */
};

/* Current RTC time in epoch seconds (frozen value while an update is held). */
static int64_t rtc_now(AT91RtcState *s)
{
    if (s->stopped) {
        return s->frozen;
    }
    return s->offset +
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / NANOSECONDS_PER_SECOND;
}

static void rtc_update_irq(AT91RtcState *s)
{
    qemu_set_irq(s->irq, !!(s->sr & s->imr));
}

/*
 * Run the 1 Hz tick only while a second-event or alarm interrupt is wanted,
 * so an idle guest is not woken (and the shared IRQ not toggled) needlessly.
 */
static void rtc_arm(AT91RtcState *s)
{
    if (s->imr & (SR_SECEV | SR_ALARM)) {
        timer_mod(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                             NANOSECONDS_PER_SECOND);
    } else {
        timer_del(&s->timer);
    }
}

/* Encode the current time into the TIMR/CALR BCD layout. */
static uint32_t rtc_timr(const struct tm *tm)
{
    return (to_bcd(tm->tm_sec) & 0x7f)
         | ((to_bcd(tm->tm_min) & 0x7f) << 8)
         | ((to_bcd(tm->tm_hour) & 0x3f) << 16);
}

static uint32_t rtc_calr(const struct tm *tm)
{
    int year = tm->tm_year + 1900;

    return (to_bcd(year / 100) & 0x7f)
         | ((to_bcd(year % 100) & 0xff) << 8)
         | ((to_bcd(tm->tm_mon + 1) & 0x1f) << 16)
         | ((to_bcd(tm->tm_wday + 1) & 0x7) << 21)
         | ((to_bcd(tm->tm_mday) & 0x3f) << 24);
}

/* Decode the latched TIMR/CALR BCD back into epoch seconds. */
static int64_t rtc_decode(uint32_t timr, uint32_t calr)
{
    struct tm tm = { 0 };

    tm.tm_sec  = from_bcd(timr & 0x7f);
    tm.tm_min  = from_bcd((timr >> 8) & 0x7f);
    tm.tm_hour = from_bcd((timr >> 16) & 0x3f);
    tm.tm_year = from_bcd(calr & 0x7f) * 100
               + from_bcd((calr >> 8) & 0xff) - 1900;
    tm.tm_mon  = from_bcd((calr >> 16) & 0x1f) - 1;
    tm.tm_mday = from_bcd((calr >> 24) & 0x3f);
    return mktimegm(&tm);
}

/* Does the (enabled) alarm match the current time? */
static bool rtc_alarm_match(AT91RtcState *s, const struct tm *tm)
{
    if (!(s->timalr & (ALR_SECEN | ALR_MINEN | ALR_HOUREN)) &&
        !(s->calalr & (ALR_MTHEN | ALR_DATEEN))) {
        return false;
    }
    if ((s->timalr & ALR_SECEN) && from_bcd(s->timalr & 0x7f) != tm->tm_sec) {
        return false;
    }
    if ((s->timalr & ALR_MINEN) &&
        from_bcd((s->timalr >> 8) & 0x7f) != tm->tm_min) {
        return false;
    }
    if ((s->timalr & ALR_HOUREN) &&
        from_bcd((s->timalr >> 16) & 0x3f) != tm->tm_hour) {
        return false;
    }
    if ((s->calalr & ALR_MTHEN) &&
        from_bcd((s->calalr >> 16) & 0x1f) != tm->tm_mon + 1) {
        return false;
    }
    if ((s->calalr & ALR_DATEEN) &&
        from_bcd((s->calalr >> 24) & 0x3f) != tm->tm_mday) {
        return false;
    }
    return true;
}

static void rtc_tick(void *opaque)
{
    AT91RtcState *s = opaque;
    int64_t now = rtc_now(s);
    struct tm tm;
    bool alarm;

    s->sr |= SR_SECEV;
    gmtime_r(&now, &tm);
    alarm = rtc_alarm_match(s, &tm);
    if (alarm) {
        s->sr |= SR_ALARM;
    }
    trace_at91_rtc_tick(now, alarm);
    rtc_update_irq(s);
    rtc_arm(s);
}

static void rtc_reset(DeviceState *dev)
{
    AT91RtcState *s = AT91_RTC(dev);
    struct tm now;

    qemu_get_timedate(&now, 0);
    s->offset = mktimegm(&now) -
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / NANOSECONDS_PER_SECOND;
    s->mr = 0;
    s->sr = 0;
    s->imr = 0;
    s->timalr = s->calalr = 0;
    s->timr_latch = s->calr_latch = 0;
    s->frozen = 0;
    s->stopped = false;
    timer_del(&s->timer);
    qemu_set_irq(s->irq, 0);
}

static uint64_t rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91RtcState *s = opaque;
    int64_t now = rtc_now(s);
    struct tm tm;

    gmtime_r(&now, &tm);
    switch (offset) {
    case RTC_CR:
        return s->stopped ? (CR_UPDTIM | CR_UPDCAL) : 0;
    case RTC_MR:
        return s->mr;
    case RTC_TIMR:
        return rtc_timr(&tm);
    case RTC_CALR:
        return rtc_calr(&tm);
    case RTC_TIMALR:
        return s->timalr;
    case RTC_CALALR:
        return s->calalr;
    case RTC_SR:
        return s->sr;
    case RTC_IMR:
        return s->imr;
    case RTC_VER:
        return 0;   /* all entries valid */
    default:
        return 0;
    }
}

static void rtc_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91RtcState *s = opaque;
    uint32_t val = value;

    switch (offset) {
    case RTC_CR:
        if (val & (CR_UPDTIM | CR_UPDCAL)) {
            /* Request to update: freeze the counter and acknowledge. */
            if (!s->stopped) {
                s->frozen = rtc_now(s);
                s->stopped = true;
            }
            s->sr |= SR_ACKUPD;
            rtc_update_irq(s);
        } else if (s->stopped) {
            /* Update finished: apply the latched time and resume. */
            s->offset = rtc_decode(s->timr_latch, s->calr_latch) -
                        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) /
                        NANOSECONDS_PER_SECOND;
            s->stopped = false;
            trace_at91_rtc_set(rtc_now(s));
        }
        break;
    case RTC_MR:
        s->mr = val;
        break;
    case RTC_TIMR:
        s->timr_latch = val;
        break;
    case RTC_CALR:
        s->calr_latch = val;
        break;
    case RTC_TIMALR:
        s->timalr = val;
        break;
    case RTC_CALALR:
        s->calalr = val;
        break;
    case RTC_SCCR:
        s->sr &= ~val;
        rtc_update_irq(s);
        break;
    case RTC_IER:
        s->imr |= val;
        rtc_update_irq(s);
        rtc_arm(s);
        break;
    case RTC_IDR:
        s->imr &= ~val;
        rtc_update_irq(s);
        rtc_arm(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps rtc_ops = {
    .read = rtc_read,
    .write = rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static int rtc_post_load(void *opaque, int version_id)
{
    AT91RtcState *s = opaque;

    rtc_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_at91_rtc = {
    .name = "at91-rtc",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rtc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, AT91RtcState),
        VMSTATE_INT64(offset, AT91RtcState),
        VMSTATE_INT64(frozen, AT91RtcState),
        VMSTATE_UINT32(mr, AT91RtcState),
        VMSTATE_UINT32(sr, AT91RtcState),
        VMSTATE_UINT32(imr, AT91RtcState),
        VMSTATE_UINT32(timalr, AT91RtcState),
        VMSTATE_UINT32(calalr, AT91RtcState),
        VMSTATE_UINT32(timr_latch, AT91RtcState),
        VMSTATE_UINT32(calr_latch, AT91RtcState),
        VMSTATE_BOOL(stopped, AT91RtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void rtc_instance_init(Object *obj)
{
    AT91RtcState *s = AT91_RTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &rtc_ops, s, "at91-rtc", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, rtc_tick, s);
}

static void rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, rtc_reset);
    dc->vmsd = &vmstate_at91_rtc;
}

static const TypeInfo rtc_type = {
    .name = TYPE_AT91_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91RtcState),
    .instance_init = rtc_instance_init,
    .class_init = rtc_class_init,
};

/* ===================================================================== */
/*  RTT - Real-Time Timer (0xFFFFFD20)                                    */
/* ===================================================================== */

#define RTT_MR      0x00
#define  MR_RTPRES   0xffff
#define  MR_ALMIEN   (1u << 16)
#define  MR_RTTINCIEN (1u << 17)
#define  MR_RTTRST   (1u << 18)
#define RTT_AR      0x04
#define RTT_VR      0x08
#define RTT_SR      0x0c
#define  RTT_ALMS    (1u << 0)
#define  RTT_RTTINC  (1u << 1)

#define SLCK_HZ     32768

struct AT91RttState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer timer;

    int64_t base_ns;    /* virtual-clock time at which VR == 0 */
    uint32_t mr;
    uint32_t ar;
    uint32_t sr;
};

static uint32_t rtt_prescaler(AT91RttState *s)
{
    uint32_t p = s->mr & MR_RTPRES;
    return p ? p : 0x10000;   /* RTPRES == 0 selects 2^16 */
}

static int64_t rtt_period_ns(AT91RttState *s)
{
    return (int64_t)rtt_prescaler(s) * NANOSECONDS_PER_SECOND / SLCK_HZ;
}

static uint32_t rtt_value(AT91RttState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    return (uint32_t)((now - s->base_ns) / rtt_period_ns(s));
}

static void rtt_update_irq(AT91RttState *s)
{
    int level = ((s->sr & RTT_ALMS) && (s->mr & MR_ALMIEN)) ||
                ((s->sr & RTT_RTTINC) && (s->mr & MR_RTTINCIEN));
    qemu_set_irq(s->irq, level);
}

static void rtt_arm(AT91RttState *s)
{
    /*
     * Only tick while an increment or alarm interrupt is enabled.  VR itself
     * is computed on read, so an idle guest needs no periodic wakeup.
     */
    if (s->mr & (MR_ALMIEN | MR_RTTINCIEN)) {
        timer_mod(&s->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + rtt_period_ns(s));
    } else {
        timer_del(&s->timer);
    }
}

static void rtt_tick(void *opaque)
{
    AT91RttState *s = opaque;
    uint32_t value = rtt_value(s);
    bool alarm = value >= s->ar;

    s->sr |= RTT_RTTINC;
    if (alarm) {
        s->sr |= RTT_ALMS;
    }
    trace_at91_rtt_tick(value, alarm);
    rtt_update_irq(s);
    rtt_arm(s);
}

static void rtt_reset(DeviceState *dev)
{
    AT91RttState *s = AT91_RTT(dev);

    s->mr = 0x00008000;   /* reset value: RTPRES = 0x8000 (1 Hz) */
    s->ar = 0xffffffff;
    s->sr = 0;
    s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    rtt_arm(s);
    qemu_set_irq(s->irq, 0);
}

static uint64_t rtt_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91RttState *s = opaque;
    uint32_t r;

    switch (offset) {
    case RTT_MR:
        return s->mr;
    case RTT_AR:
        return s->ar;
    case RTT_VR:
        return rtt_value(s);
    case RTT_SR:
        r = s->sr;
        s->sr = 0;              /* status is clear-on-read */
        rtt_update_irq(s);
        return r;
    default:
        return 0;
    }
}

static void rtt_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91RttState *s = opaque;
    uint32_t val = value;

    switch (offset) {
    case RTT_MR:
        s->mr = val & ~MR_RTTRST;
        if (val & MR_RTTRST) {
            s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->sr = 0;
        }
        rtt_update_irq(s);
        rtt_arm(s);
        break;
    case RTT_AR:
        s->ar = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps rtt_ops = {
    .read = rtt_read,
    .write = rtt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static int rtt_post_load(void *opaque, int version_id)
{
    AT91RttState *s = opaque;

    rtt_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_at91_rtt = {
    .name = "at91-rtt",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rtt_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, AT91RttState),
        VMSTATE_INT64(base_ns, AT91RttState),
        VMSTATE_UINT32(mr, AT91RttState),
        VMSTATE_UINT32(ar, AT91RttState),
        VMSTATE_UINT32(sr, AT91RttState),
        VMSTATE_END_OF_LIST()
    }
};

static void rtt_instance_init(Object *obj)
{
    AT91RttState *s = AT91_RTT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &rtt_ops, s, "at91-rtt", 0x10);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, rtt_tick, s);
}

static void rtt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, rtt_reset);
    dc->vmsd = &vmstate_at91_rtt;
}

static const TypeInfo rtt_type = {
    .name = TYPE_AT91_RTT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91RttState),
    .instance_init = rtt_instance_init,
    .class_init = rtt_class_init,
};

/* ===================================================================== */
/*  GPBR - General Purpose Backup Registers (0xFFFFFD60)                  */
/* ===================================================================== */

#define GPBR_NUM_REGS 4

struct AT91GpbrState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[GPBR_NUM_REGS];
};

static uint64_t gpbr_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91GpbrState *s = opaque;
    unsigned idx = offset / 4;

    return idx < GPBR_NUM_REGS ? s->regs[idx] : 0;
}

static void gpbr_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AT91GpbrState *s = opaque;
    unsigned idx = offset / 4;

    if (idx < GPBR_NUM_REGS) {
        s->regs[idx] = value;
        trace_at91_gpbr_write(idx, value);
    }
}

static const MemoryRegionOps gpbr_ops = {
    .read = gpbr_read,
    .write = gpbr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const VMStateDescription vmstate_at91_gpbr = {
    .name = "at91-gpbr",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AT91GpbrState, GPBR_NUM_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void gpbr_instance_init(Object *obj)
{
    AT91GpbrState *s = AT91_GPBR(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &gpbr_ops, s, "at91-gpbr",
                          GPBR_NUM_REGS * 4);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void gpbr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_at91_gpbr;
}

static const TypeInfo gpbr_type = {
    .name = TYPE_AT91_GPBR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91GpbrState),
    .instance_init = gpbr_instance_init,
    .class_init = gpbr_class_init,
};

static void at91_rtc_register_types(void)
{
    type_register_static(&rtc_type);
    type_register_static(&rtt_type);
    type_register_static(&gpbr_type);
}

type_init(at91_rtc_register_types)
