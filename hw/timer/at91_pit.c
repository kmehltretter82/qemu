/*
 * Atmel/Microchip AT91 Periodic Interval Timer (PIT) - system tick.
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
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/timer/at91_pit.h"

#define PIT_MR      0x00   /* Mode Register                */
#define PIT_SR      0x04   /* Status Register              */
#define PIT_PIVR    0x08   /* Periodic Interval Value Reg  */
#define PIT_PIIR    0x0C   /* Periodic Interval Image Reg  */

#define PIT_MR_PIV(mr)   ((mr) & 0xFFFFF)
#define PIT_MR_PITEN     (1u << 24)
#define PIT_MR_PITIEN    (1u << 25)
#define PIT_SR_PITS      (1u << 0)

struct AT91PitState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    Clock *mck;
    uint32_t mck_freq;     /* master clock (Hz); PIT counts at mck/16 */
    uint32_t clock_update_cpiv;
    uint32_t migration_cpiv;

    uint32_t mr;
    uint32_t picnt;        /* completed intervals since last PIVR read */
    bool pits;
    bool irq_level;        /* last level driven onto the AIC line */
    int64_t last_fire;     /* ns of last interval boundary */
};

static uint32_t pit_clk(AT91PitState *s)
{
    return s->mck_freq / 16;
}

static int64_t pit_cpiv_elapsed_ns(AT91PitState *s, uint32_t cpiv)
{
    return DIV_ROUND_UP((uint64_t)cpiv * NANOSECONDS_PER_SECOND,
                        pit_clk(s));
}

/* nanoseconds per PIT interval, given the programmed PIV */
static int64_t pit_period_ns(AT91PitState *s)
{
    uint32_t piv = PIT_MR_PIV(s->mr);

    return (int64_t)(piv + 1) * NANOSECONDS_PER_SECOND / pit_clk(s);
}

static void pit_update_irq(AT91PitState *s)
{
    bool level = s->pits && (s->mr & PIT_MR_PITIEN);

    /* Only drive an edge onto the shared AIC line when the level actually
     * changes.  Otherwise, once the guest switches its tick to the TCB and
     * masks PITIEN (but leaves the PIT running as clocksource), every interval
     * boundary would spam a redundant de-assert onto AIC source 1, which the
     * PIT shares with the DBGU/RTC/RTT/scheduler tick. */
    if (level != s->irq_level) {
        s->irq_level = level;
        qemu_set_irq(s->irq, level);
    }
}

/* Current sub-interval counter value (CPIV). */
static uint32_t pit_cpiv(AT91PitState *s)
{
    int64_t now, elapsed;
    uint32_t piv, cpiv;

    if (!(s->mr & PIT_MR_PITEN) || PIT_MR_PIV(s->mr) == 0 ||
        pit_clk(s) == 0) {
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
    if ((s->mr & PIT_MR_PITEN) && PIT_MR_PIV(s->mr) != 0 && pit_clk(s)) {
        timer_mod(s->timer, s->last_fire + pit_period_ns(s));
    }
}

static void pit_rearm(AT91PitState *s)
{
    if ((s->mr & PIT_MR_PITEN) && PIT_MR_PIV(s->mr) != 0 && pit_clk(s)) {
        s->last_fire = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(s->timer, s->last_fire + pit_period_ns(s));
    } else {
        timer_del(s->timer);
    }
}

static void pit_clock_update(void *opaque, ClockEvent event)
{
    AT91PitState *s = AT91_PIT(opaque);
    int64_t now;

    if (!s->timer) {
        return;
    }

    switch (event) {
    case ClockPreUpdate:
        s->clock_update_cpiv = pit_cpiv(s);
        break;
    case ClockUpdate:
        s->mck_freq = clock_get_hz(s->mck);
        if (!(s->mr & PIT_MR_PITEN) || PIT_MR_PIV(s->mr) == 0 ||
            pit_clk(s) == 0) {
            timer_del(s->timer);
            break;
        }

        /* Keep CPIV continuous while changing the duration of each tick. */
        now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->last_fire = now - pit_cpiv_elapsed_ns(s, s->clock_update_cpiv);
        timer_mod(s->timer, s->last_fire + pit_period_ns(s));
        break;
    default:
        break;
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

    if (clock_has_source(s->mck)) {
        s->mck_freq = clock_get_hz(s->mck);
    } else if (s->mck_freq != 0) {
        clock_set_hz(s->mck, s->mck_freq);
    } else {
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
    s->mck = qdev_init_clock_in(DEVICE(obj), "mck", pit_clock_update, s,
                                ClockPreUpdate | ClockUpdate);
}

/* Nominal fallback; the board sets the real MCK via the mck-frequency prop. */
#define AT91_PIT_DEFAULT_MCK_HZ  100000000

static const Property pit_properties[] = {
    DEFINE_PROP_UINT32("mck-frequency", AT91PitState, mck_freq,
                       AT91_PIT_DEFAULT_MCK_HZ),
};

static int pit_pre_save(void *opaque)
{
    AT91PitState *s = opaque;

    s->migration_cpiv = pit_cpiv(s);
    return 0;
}

static int pit_post_load(void *opaque, int version_id)
{
    AT91PitState *s = opaque;

    s->mck_freq = clock_get_hz(s->mck);
    if ((s->mr & PIT_MR_PITEN) && PIT_MR_PIV(s->mr) != 0 && pit_clk(s)) {
        if (version_id >= 2) {
            uint64_t elapsed = pit_cpiv_elapsed_ns(s, s->migration_cpiv);

            /* Rebase the saved CPIV onto the destination virtual clock. */
            s->last_fire = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - elapsed;
            timer_mod(s->timer, s->last_fire + pit_period_ns(s));
        } else if (timer_pending(s->timer)) {
            /* Best-effort compatibility for streams without a CPIV image. */
            s->last_fire = (int64_t)timer_expire_time_ns(s->timer) -
                           pit_period_ns(s);
        }
    }
    pit_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_at91_pit = {
    .name = "at91-pit",
    .version_id = 2,
    .minimum_version_id = 1,
    .pre_save = pit_pre_save,
    .post_load = pit_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91PitState),
        VMSTATE_UINT32(picnt, AT91PitState),
        VMSTATE_BOOL(pits, AT91PitState),
        VMSTATE_BOOL(irq_level, AT91PitState),
        VMSTATE_INT64(last_fire, AT91PitState),
        VMSTATE_TIMER_PTR(timer, AT91PitState),
        VMSTATE_CLOCK_V(mck, AT91PitState, 2),
        VMSTATE_UINT32_V(migration_cpiv, AT91PitState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void pit_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pit_realize;
    device_class_set_legacy_reset(dc, pit_reset);
    device_class_set_props(dc, pit_properties);
    dc->vmsd = &vmstate_at91_pit;
}

static const TypeInfo pit_type = {
    .name = TYPE_AT91_PIT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91PitState),
    .instance_init = pit_dev_init,
    .class_init = pit_class_init,
};

static void at91_pit_register_types(void)
{
    type_register_static(&pit_type);
}

type_init(at91_pit_register_types)
