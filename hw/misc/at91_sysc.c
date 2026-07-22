/*
 * Atmel/Microchip AT91 system-controller reset path: RSTC / SHDWC / WDT.
 *
 * RSTC_CR (key 0xa5 + PROCRST) requests a system reset; EXTRST only pulses the
 * external NRST pin.  SHDW_CR (key + SHDW) requests a shutdown.  The WDT is
 * a 12-bit, 256 Hz windowed watchdog with interrupt and reset actions.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "system/runstate.h"
#include "system/watchdog.h"
#include "hw/misc/at91_sysc.h"
#include "trace.h"

/*  RSTC / SHDWC / WDT (datasheet sections 11, 16, 15)                        */
/*                                                                           */
/*  Reset, shutdown and watchdog reset are wired to the QEMU run-state so    */
/*  that guest reboot, poweroff and watchdog expiry behave correctly.        */
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
#define RSTC_SR_NRSTL    (1u << 16)
#define RSTC_SR_RESET    RSTC_SR_NRSTL  /* external NRST is deasserted */

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
        if (at91_key_ok(val) && (val & RSTC_CR_PROCRST)) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        /* PERRST and EXTRST complete synchronously in this model. */
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

static const VMStateDescription vmstate_at91_rstc = {
    .name = "at91-rstc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91RstcState),
        VMSTATE_END_OF_LIST()
    }
};

static void rstc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, rstc_reset);
    dc->vmsd = &vmstate_at91_rstc;
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

struct AT91ShdwcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mr;
};

static uint64_t shdwc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91ShdwcState *s = AT91_SHDWC(opaque);

    switch (offset) {
    case SHDW_CR: return 0;
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

static const VMStateDescription vmstate_at91_shdwc = {
    .name = "at91-shdwc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91ShdwcState),
        VMSTATE_END_OF_LIST()
    }
};

static void shdwc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, shdwc_reset);
    dc->vmsd = &vmstate_at91_shdwc;
}

static const TypeInfo shdwc_type = {
    .name = TYPE_AT91_SHDWC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91ShdwcState),
    .instance_init = shdwc_dev_init,
    .class_init = shdwc_class_init,
};

/* --- WDT (0xFFFFFD40) --- */
#define WDT_CR   0x00
#define WDT_MR   0x04
#define WDT_SR   0x08

#define WDT_CR_WDRSTT      (1u << 0)

#define WDT_MR_WDV         0x00000fff
#define WDT_MR_WDFIEN      (1u << 12)
#define WDT_MR_WDRSTEN     (1u << 13)
#define WDT_MR_WDRPROC     (1u << 14)
#define WDT_MR_WDDIS       (1u << 15)
#define WDT_MR_WDD         0x0fff0000
#define WDT_MR_VALID       0x3fffffff
#define WDT_MR_RESET       0x3fff2fff

#define WDT_SR_WDUNF       (1u << 0)
#define WDT_SR_WDERR       (1u << 1)

#define WDT_TICK_NS        (NANOSECONDS_PER_SECOND / 256)

struct AT91WdtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t mr;
    uint32_t sr;
    int64_t remaining_ns;
    bool mr_written;       /* WDT_MR is write-once */
    bool counter_expired;
};

static void wdt_update_irq(AT91WdtState *s)
{
    qemu_set_irq(s->irq, (s->mr & WDT_MR_WDFIEN) && s->sr);
}

static void wdt_reload(AT91WdtState *s)
{
    uint32_t ticks = (s->mr & WDT_MR_WDV) + 1;

    s->counter_expired = false;
    if (s->mr & WDT_MR_WDDIS) {
        timer_del(s->timer);
        return;
    }

    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (int64_t)ticks * WDT_TICK_NS);
    trace_at91_wdt_reload(ticks);
}

static void wdt_fault(AT91WdtState *s, uint32_t status)
{
    bool reset = s->mr & WDT_MR_WDRSTEN;

    s->sr |= status;
    if (status & WDT_SR_WDUNF) {
        s->counter_expired = true;
    }
    wdt_update_irq(s);
    trace_at91_wdt_fault(status, !!(s->mr & WDT_MR_WDFIEN), reset);

    if (reset) {
        timer_del(s->timer);
        if (s->mr & WDT_MR_WDRPROC) {
            qemu_log_mask(LOG_UNIMP, "at91-wdt: processor-only reset "
                          "treated as a system watchdog reset\n");
        }
        watchdog_perform_action();
    }
}

static void wdt_expired(void *opaque)
{
    AT91WdtState *s = opaque;

    wdt_fault(s, WDT_SR_WDUNF);
}

static uint32_t wdt_counter(AT91WdtState *s)
{
    int64_t remaining;
    uint32_t counter;

    if (s->counter_expired) {
        return 0;
    }
    if ((s->mr & WDT_MR_WDDIS) || !timer_pending(s->timer)) {
        return s->mr & WDT_MR_WDV;
    }

    remaining = timer_expire_time_ns(s->timer) -
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (remaining <= 0) {
        return 0;
    }
    counter = (remaining - 1) / WDT_TICK_NS;
    return MIN(counter, s->mr & WDT_MR_WDV);
}

static uint64_t wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91WdtState *s = AT91_WDT(opaque);
    uint32_t value;

    switch (offset) {
    case WDT_CR: return 0;
    case WDT_MR: return s->mr;
    case WDT_SR:
        value = s->sr;
        s->sr = 0;
        wdt_update_irq(s);
        return value;
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
        if (at91_key_ok(val) && (val & WDT_CR_WDRSTT)) {
            uint32_t wdd = (s->mr & WDT_MR_WDD) >> 16;

            if (wdt_counter(s) > wdd) {
                wdt_fault(s, WDT_SR_WDERR);
            } else {
                wdt_reload(s);
            }
        }
        break;
    case WDT_MR:
        if (!s->mr_written) {
            s->mr = val & WDT_MR_VALID;
            s->mr_written = true;
            wdt_update_irq(s);
            wdt_reload(s);
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

    timer_del(s->timer);
    s->mr = WDT_MR_RESET;
    s->sr = 0;
    s->mr_written = false;
    s->counter_expired = false;
    wdt_update_irq(s);
    wdt_reload(s);
}

static void wdt_dev_init(Object *obj)
{
    AT91WdtState *s = AT91_WDT(obj);

    memory_region_init_io(&s->iomem, obj, &wdt_ops, s, "at91-wdt", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wdt_expired, s);
}

static int wdt_post_load(void *opaque, int version_id)
{
    AT91WdtState *s = opaque;

    s->mr &= WDT_MR_VALID;
    if (version_id < 2) {
        s->sr = 0;
        s->counter_expired = false;
        wdt_reload(s);
    } else if ((s->mr & WDT_MR_WDDIS) || s->counter_expired ||
               s->remaining_ns < 0) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  s->remaining_ns);
    }
    trace_at91_wdt_post_load(version_id, s->mr, timer_pending(s->timer),
                             timer_expire_time_ns(s->timer));
    wdt_update_irq(s);
    return 0;
}

static int wdt_pre_save(void *opaque)
{
    AT91WdtState *s = opaque;

    if (timer_pending(s->timer)) {
        s->remaining_ns = MAX((int64_t)timer_expire_time_ns(s->timer) -
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), 0);
    } else {
        s->remaining_ns = -1;
    }
    return 0;
}

static const VMStateDescription vmstate_at91_wdt = {
    .name = "at91-wdt",
    .version_id = 2,
    .minimum_version_id = 1,
    .pre_save = wdt_pre_save,
    .post_load = wdt_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91WdtState),
        VMSTATE_BOOL(mr_written, AT91WdtState),
        VMSTATE_UINT32_V(sr, AT91WdtState, 2),
        VMSTATE_INT64_V(remaining_ns, AT91WdtState, 2),
        VMSTATE_BOOL_V(counter_expired, AT91WdtState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, wdt_reset);
    dc->vmsd = &vmstate_at91_wdt;
    set_bit(DEVICE_CATEGORY_WATCHDOG, dc->categories);
}

static const TypeInfo wdt_type = {
    .name = TYPE_AT91_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91WdtState),
    .instance_init = wdt_dev_init,
    .class_init = wdt_class_init,
};

static void at91_sysc_register_types(void)
{
    type_register_static(&rstc_type);
    type_register_static(&shdwc_type);
    type_register_static(&wdt_type);
}

type_init(at91_sysc_register_types)
