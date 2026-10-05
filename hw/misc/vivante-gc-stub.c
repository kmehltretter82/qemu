/*
 * Register stub for a Vivante GC GPU core, enough for the Linux etnaviv
 * driver to identify the core and finish its reset sequence. It executes
 * no commands and raises no interrupts.
 *
 *  - CHIP_MODEL and CHIP_REV report the configured model and revision
 *  - CHIP_FEATURE reports a 3D pipe, all other feature words read 0
 *  - IDLE_STATE reports every module idle
 *  - CLOCK_CONTROL reads back the written value with 2D and 3D idle set
 *  - all other registers read 0 and ignore writes
 *
 * Local test infrastructure for dma-buf importer tests, not intended for
 * submission to upstream QEMU.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_VIVANTE_GC_STUB "vivante-gc-stub"
OBJECT_DECLARE_SIMPLE_TYPE(VivanteGCStubState, VIVANTE_GC_STUB)

#define REG_CLOCK_CONTROL       0x00
#define   CLOCK_CONTROL_IDLE    0x00030000
#define REG_IDLE_STATE          0x04
#define REG_CHIP_FEATURE        0x1c
#define   CHIP_FEATURE_PIPE_3D  0x00000004
#define REG_CHIP_MODEL          0x20
#define REG_CHIP_REV            0x24

struct VivanteGCStubState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t clock_control;
    uint32_t model;
    uint32_t revision;
};

static uint64_t vivante_gc_stub_read(void *opaque, hwaddr addr, unsigned size)
{
    VivanteGCStubState *s = opaque;

    switch (addr) {
    case REG_CLOCK_CONTROL:
        return s->clock_control | CLOCK_CONTROL_IDLE;
    case REG_IDLE_STATE:
        return 0x7fffffff;
    case REG_CHIP_FEATURE:
        return CHIP_FEATURE_PIPE_3D;
    case REG_CHIP_MODEL:
        return s->model;
    case REG_CHIP_REV:
        return s->revision;
    default:
        return 0;
    }
}

static void vivante_gc_stub_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    VivanteGCStubState *s = opaque;

    if (addr == REG_CLOCK_CONTROL) {
        s->clock_control = val;
    }
}

static const MemoryRegionOps vivante_gc_stub_ops = {
    .read = vivante_gc_stub_read,
    .write = vivante_gc_stub_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void vivante_gc_stub_init(Object *obj)
{
    VivanteGCStubState *s = VIVANTE_GC_STUB(obj);

    memory_region_init_io(&s->iomem, obj, &vivante_gc_stub_ops, s,
                          TYPE_VIVANTE_GC_STUB, 0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property vivante_gc_stub_props[] = {
    DEFINE_PROP_UINT32("model", VivanteGCStubState, model, 0x2000),
    DEFINE_PROP_UINT32("revision", VivanteGCStubState, revision, 0x5108),
};

static void vivante_gc_stub_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_props(dc, vivante_gc_stub_props);
    dc->desc = "Vivante GC GPU register stub (test only)";
}

static const TypeInfo vivante_gc_stub_info = {
    .name = TYPE_VIVANTE_GC_STUB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(VivanteGCStubState),
    .instance_init = vivante_gc_stub_init,
    .class_init = vivante_gc_stub_class_init,
};

static void vivante_gc_stub_register_types(void)
{
    type_register_static(&vivante_gc_stub_info);
}

type_init(vivante_gc_stub_register_types)
