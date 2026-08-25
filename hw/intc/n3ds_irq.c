/*
 * Nintendo 3DS ARM9 interrupt controller (0x10001000):
 *   0x00 IE  enable mask
 *   0x04 IF  pending flags, write 1 to clear
 * 32 edge-triggered inputs, one IRQ output to the ARM946.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/n3ds.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(N3DSIrqState, N3DS_IRQ)

struct N3DSIrqState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t ie;
    uint32_t iflags;
    uint32_t level;
};

static void n3ds_irq_update(N3DSIrqState *s)
{
    qemu_set_irq(s->irq, (s->ie & s->iflags) != 0);
}

static void n3ds_irq_set(void *opaque, int n, int level)
{
    N3DSIrqState *s = opaque;
    uint32_t bit = 1u << n;

    if (level && !(s->level & bit)) {
        s->iflags |= bit;
    }
    if (level) {
        s->level |= bit;
    } else {
        s->level &= ~bit;
    }
    n3ds_irq_update(s);
}

static uint64_t n3ds_irq_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSIrqState *s = opaque;

    switch (off) {
    case 0x0:
        return s->ie;
    case 0x4:
        return s->iflags;
    default:
        return 0;
    }
}

static void n3ds_irq_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSIrqState *s = opaque;

    switch (off) {
    case 0x0:
        s->ie = v;
        break;
    case 0x4:
        s->iflags &= ~v;
        break;
    default:
        break;
    }
    n3ds_irq_update(s);
}

static const MemoryRegionOps n3ds_irq_ops = {
    .read = n3ds_irq_read,
    .write = n3ds_irq_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void n3ds_irq_reset(DeviceState *dev)
{
    N3DSIrqState *s = N3DS_IRQ(dev);

    /* as left by Luma3DS on the console: timers 0/1 and SDIO1 enabled and pending */
    s->ie = 0x00010300;
    s->iflags = 0x00010300;
    s->level = 0;
    n3ds_irq_update(s);
}

static void n3ds_irq_init(Object *obj)
{
    N3DSIrqState *s = N3DS_IRQ(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &n3ds_irq_ops, s, "n3ds.irq9", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in(DEVICE(obj), n3ds_irq_set, 32);
}

static void n3ds_irq_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n3ds_irq_reset);
}

static const TypeInfo n3ds_irq_info = {
    .name          = TYPE_N3DS_IRQ,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSIrqState),
    .instance_init = n3ds_irq_init,
    .class_init    = n3ds_irq_class_init,
};

static void n3ds_irq_register_types(void)
{
    type_register_static(&n3ds_irq_info);
}

type_init(n3ds_irq_register_types)
