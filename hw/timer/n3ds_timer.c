/*
 * Nintendo 3DS ARM9 timers (0x10003000): four 16-bit up-counters in the
 * DS layout, each at 4*n: +0 VAL (reload value on write, current count
 * on read), +2 CNT (bits 0-1 prescaler 1/64/256/1024, bit 2 count-up
 * from the previous timer's overflow, bit 6 IRQ enable, bit 7 start).
 * Input clock: 67.03 MHz. IRQ outputs 0-3 -> ARM9 IRQ 8-11.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/ptimer.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/misc/n3ds.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(N3DSTimerState, N3DS_TIMER)

#define N3DS_NTIMERS 4
#define N3DS_TIMER_CLK 67027964

#define CNT_PRESCALE_MASK 0x3
#define CNT_CASCADE       0x4
#define CNT_IRQEN         0x40
#define CNT_START         0x80

typedef struct N3DSTimer {
    struct N3DSTimerState *parent;
    ptimer_state *ptimer;
    qemu_irq irq;
    uint16_t reload;
    uint16_t cnt;
    uint16_t count;   /* cascade mode counter */
    int index;
} N3DSTimer;

struct N3DSTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    N3DSTimer timer[N3DS_NTIMERS];
};

static const unsigned prescale[4] = { 1, 64, 256, 1024 };

static void n3ds_timer_overflow(N3DSTimer *t);

static void n3ds_timer_cascade_tick(N3DSTimer *t)
{
    if (!(t->cnt & CNT_START) || !(t->cnt & CNT_CASCADE)) {
        return;
    }
    if (++t->count == 0) {
        t->count = t->reload;
        n3ds_timer_overflow(t);
    }
}

static void n3ds_timer_overflow(N3DSTimer *t)
{
    N3DSTimerState *s = t->parent;

    if (t->cnt & CNT_IRQEN) {
        qemu_irq_pulse(t->irq);
    }
    if (t->index + 1 < N3DS_NTIMERS) {
        n3ds_timer_cascade_tick(&s->timer[t->index + 1]);
    }
}

static void n3ds_timer_tick(void *opaque)
{
    n3ds_timer_overflow(opaque);
}

static void n3ds_timer_update(N3DSTimer *t)
{
    ptimer_transaction_begin(t->ptimer);
    if ((t->cnt & CNT_START) && !(t->cnt & CNT_CASCADE)) {
        ptimer_set_freq(t->ptimer, N3DS_TIMER_CLK / prescale[t->cnt & 3]);
        ptimer_set_limit(t->ptimer, 0x10000 - t->reload, 1);
        ptimer_run(t->ptimer, 0);
    } else {
        ptimer_stop(t->ptimer);
    }
    ptimer_transaction_commit(t->ptimer);
}

static uint16_t n3ds_timer_get_count(N3DSTimer *t)
{
    if (!(t->cnt & CNT_START)) {
        return t->reload;
    }
    if (t->cnt & CNT_CASCADE) {
        return t->count;
    }
    return 0x10000 - ptimer_get_count(t->ptimer);
}

static uint64_t n3ds_timer_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSTimerState *s = opaque;
    unsigned idx = (off >> 2) & 3;
    N3DSTimer *t = &s->timer[idx];
    uint32_t v;

    if (off >= 0x10) {
        return 0;
    }
    v = n3ds_timer_get_count(t) | ((uint32_t)t->cnt << 16);
    return (v >> (8 * (off & 3))) & (size == 4 ? 0xFFFFFFFF :
                                     size == 2 ? 0xFFFF : 0xFF);
}

static void n3ds_timer_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSTimerState *s = opaque;
    unsigned idx = (off >> 2) & 3;
    N3DSTimer *t = &s->timer[idx];
    unsigned shift = 8 * (off & 3);
    uint32_t mask = (size == 4 ? 0xFFFFFFFF : size == 2 ? 0xFFFF : 0xFF) << shift;
    uint32_t val = (uint32_t)v << shift;
    uint16_t old_cnt = t->cnt;

    if (off >= 0x10) {
        return;
    }
    if (mask & 0xFFFF) {
        t->reload = (t->reload & ~(mask & 0xFFFF)) | (val & mask & 0xFFFF);
    }
    if (mask & 0xFFFF0000) {
        t->cnt = (t->cnt & ~(mask >> 16)) | ((val & mask) >> 16);
        t->cnt &= CNT_PRESCALE_MASK | CNT_CASCADE | CNT_IRQEN | CNT_START;
        if ((t->cnt & CNT_START) && !(old_cnt & CNT_START)) {
            t->count = t->reload;
        }
    }
    n3ds_timer_update(t);
}

static const MemoryRegionOps n3ds_timer_ops = {
    .read = n3ds_timer_read,
    .write = n3ds_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void n3ds_timer_reset(DeviceState *dev)
{
    N3DSTimerState *s = N3DS_TIMER(dev);
    int i;

    for (i = 0; i < N3DS_NTIMERS; i++) {
        N3DSTimer *t = &s->timer[i];

        t->reload = 0;
        t->cnt = 0;
        t->count = 0;
        n3ds_timer_update(t);
    }
}

static void n3ds_timer_init(Object *obj)
{
    N3DSTimerState *s = N3DS_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &n3ds_timer_ops, s, "n3ds.timer9",
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (i = 0; i < N3DS_NTIMERS; i++) {
        N3DSTimer *t = &s->timer[i];

        t->parent = s;
        t->index = i;
        t->ptimer = ptimer_init(n3ds_timer_tick, t, PTIMER_POLICY_LEGACY);
        sysbus_init_irq(sbd, &t->irq);
    }
}

static void n3ds_timer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n3ds_timer_reset);
}

static const TypeInfo n3ds_timer_info = {
    .name          = TYPE_N3DS_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSTimerState),
    .instance_init = n3ds_timer_init,
    .class_init    = n3ds_timer_class_init,
};

static void n3ds_timer_register_types(void)
{
    type_register_static(&n3ds_timer_info);
}

type_init(n3ds_timer_register_types)
