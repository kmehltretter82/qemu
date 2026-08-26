/*
 * Nintendo 3DS GPIO register block (0x10147000): register storage for
 * the six small GPIO/interrupt groups the kernel drives (data, direction
 * and interrupt enable bits). Inputs read back as written; no external
 * lines are modelled yet.
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

#define TYPE_N3DS_GPIO "n3ds-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(N3DSGpioState, N3DS_GPIO)

struct N3DSGpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint8_t regs[0x100];
};

static uint64_t n3ds_gpio_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSGpioState *s = opaque;
    uint64_t v = 0;
    unsigned i;

    if (off + size > sizeof(s->regs)) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)s->regs[off + i] << (8 * i);
    }
    return v;
}

static void n3ds_gpio_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    N3DSGpioState *s = opaque;
    unsigned i;

    if (off + size > sizeof(s->regs)) {
        return;
    }
    for (i = 0; i < size; i++) {
        s->regs[off + i] = val >> (8 * i);
    }
}

static const MemoryRegionOps n3ds_gpio_ops = {
    .read = n3ds_gpio_read,
    .write = n3ds_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void n3ds_gpio_reset(DeviceState *dev)
{
    N3DSGpioState *s = N3DS_GPIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    /*
     * What the console reads here after the Luma3DS chainloader
     * (n3ds-probes io_gpio_a / io_gpio_b). The WiFi enables at 0x14 and
     * 0x28 are zero at this point - the module is still powered down.
     */
    s->regs[0x00] = 0x03;
    s->regs[0x10] = 0x02;
    s->regs[0x20] = 0xba;
    s->regs[0x21] = 0x0f;
    s->regs[0x22] = 0x40;
}

static void n3ds_gpio_init(Object *obj)
{
    N3DSGpioState *s = N3DS_GPIO(obj);

    memory_region_init_io(&s->iomem, obj, &n3ds_gpio_ops, s, "n3ds.gpio", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void n3ds_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, n3ds_gpio_reset);
}

static const TypeInfo n3ds_gpio_info = {
    .name          = TYPE_N3DS_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSGpioState),
    .instance_init = n3ds_gpio_init,
    .class_init    = n3ds_gpio_class_init,
};

static void n3ds_gpio_register_types(void)
{
    type_register_static(&n3ds_gpio_info);
}

type_init(n3ds_gpio_register_types)
