/*
 * Atmel AT91 true random number generator.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/misc/at91_trng.h"
#include "migration/vmstate.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

#define TRNG_CR         0x00
#define TRNG_MR         0x04
#define TRNG_ISR        0x1c
#define TRNG_ODATA      0x50

#define TRNG_KEY        0x524e4700
#define TRNG_CR_ENABLE  BIT(0)
#define TRNG_ISR_DATRDY BIT(0)

#define TRNG_IOMEM_SIZE 0x100

struct AT91TrngState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t mr;
    uint32_t data;
    bool enabled;
};

static uint64_t at91_trng_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91TrngState *s = opaque;
    uint32_t value = 0;

    switch (offset) {
    case TRNG_MR:
        value = s->mr;
        break;
    case TRNG_ISR:
        value = s->enabled ? TRNG_ISR_DATRDY : 0;
        break;
    case TRNG_ODATA:
        if (s->enabled) {
            qemu_guest_getrandom_nofail(&s->data, sizeof(s->data));
            value = s->data;
            trace_at91_trng_data(value);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "at91-trng: unimplemented read at 0x%02" HWADDR_PRIx
                      "\n", offset);
        break;
    }
    return value;
}

static void at91_trng_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    AT91TrngState *s = opaque;
    uint32_t val = value;

    switch (offset) {
    case TRNG_CR:
        if ((val & 0xffffff00) == TRNG_KEY) {
            s->enabled = val & TRNG_CR_ENABLE;
            qemu_set_irq(s->irq, 0);
        }
        break;
    case TRNG_MR:
        s->mr = val & BIT(0);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "at91-trng: unimplemented write at 0x%02" HWADDR_PRIx
                      " value 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps at91_trng_ops = {
    .read = at91_trng_read,
    .write = at91_trng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void at91_trng_reset(DeviceState *dev)
{
    AT91TrngState *s = AT91_TRNG(dev);

    s->mr = 0;
    s->data = 0;
    s->enabled = false;
    qemu_set_irq(s->irq, 0);
}

static int at91_trng_post_load(void *opaque, int version_id)
{
    AT91TrngState *s = opaque;

    qemu_set_irq(s->irq, 0);
    return 0;
}

static const VMStateDescription vmstate_at91_trng = {
    .name = "at91-trng",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = at91_trng_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91TrngState),
        VMSTATE_UINT32(data, AT91TrngState),
        VMSTATE_BOOL(enabled, AT91TrngState),
        VMSTATE_END_OF_LIST()
    },
};

static void at91_trng_init(Object *obj)
{
    AT91TrngState *s = AT91_TRNG(obj);

    memory_region_init_io(&s->iomem, obj, &at91_trng_ops, s,
                          TYPE_AT91_TRNG, TRNG_IOMEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void at91_trng_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Atmel AT91 True Random Number Generator";
    dc->vmsd = &vmstate_at91_trng;
    device_class_set_legacy_reset(dc, at91_trng_reset);
}

static const TypeInfo at91_trng_info = {
    .name = TYPE_AT91_TRNG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91TrngState),
    .instance_init = at91_trng_init,
    .class_init = at91_trng_class_init,
};

static void at91_trng_register_types(void)
{
    type_register_static(&at91_trng_info);
}

type_init(at91_trng_register_types)
