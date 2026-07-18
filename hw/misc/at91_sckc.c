/*
 * Atmel AT91 slow-clock controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/at91_sckc.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "trace.h"

#define SCKC_CR          0x00
#define SCKC_CR_RCEN     BIT(0)
#define SCKC_CR_OSC32EN  BIT(1)
#define SCKC_CR_OSC32BYP BIT(2)
#define SCKC_CR_OSCSEL   BIT(3)
#define SCKC_CR_MASK     (SCKC_CR_RCEN | SCKC_CR_OSC32EN | \
                          SCKC_CR_OSC32BYP | SCKC_CR_OSCSEL)

struct AT91SckcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t cr;
};

static uint64_t at91_sckc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91SckcState *s = opaque;

    trace_at91_sckc_read(s->cr);
    return s->cr;
}

static void at91_sckc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    AT91SckcState *s = opaque;

    s->cr = value & SCKC_CR_MASK;
    trace_at91_sckc_write(s->cr);
}

static const MemoryRegionOps at91_sckc_ops = {
    .read = at91_sckc_read,
    .write = at91_sckc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void at91_sckc_reset(DeviceState *dev)
{
    AT91SckcState *s = AT91_SCKC(dev);

    /* The internal 32 kHz RC oscillator starts enabled. */
    s->cr = SCKC_CR_RCEN;
}

static const VMStateDescription vmstate_at91_sckc = {
    .name = "at91-sckc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr, AT91SckcState),
        VMSTATE_END_OF_LIST()
    },
};

static void at91_sckc_init(Object *obj)
{
    AT91SckcState *s = AT91_SCKC(obj);

    memory_region_init_io(&s->iomem, obj, &at91_sckc_ops, s,
                          TYPE_AT91_SCKC, 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void at91_sckc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Atmel AT91 Slow Clock Controller";
    dc->vmsd = &vmstate_at91_sckc;
    device_class_set_legacy_reset(dc, at91_sckc_reset);
}

static const TypeInfo at91_sckc_info = {
    .name = TYPE_AT91_SCKC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91SckcState),
    .instance_init = at91_sckc_init,
    .class_init = at91_sckc_class_init,
};

static void at91_sckc_register_types(void)
{
    type_register_static(&at91_sckc_info);
}

type_init(at91_sckc_register_types)
