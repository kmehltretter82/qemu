/*
 * Atmel AT91 DDRAMC, static-memory, matrix and ECC register banks.
 *
 * The board's SDRAM remains a normal QEMU RAM region.  These devices model
 * the configuration plane used by boot firmware and Linux: register writes
 * are retained and read back, and migrate with the guest.  SMC timing and
 * matrix arbitration do not delay QEMU memory transactions.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/misc/at91_memc.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "trace.h"

#define MEMC_IOMEM_SIZE  0x200
#define MEMC_NUM_REGS    (MEMC_IOMEM_SIZE / sizeof(uint32_t))

#define DDRAMC_MDR       0x20
#define DDRAMC_MDR_DDR2  6

typedef struct AT91MemcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[MEMC_NUM_REGS];
    const char *name;
    bool is_ddramc;
} AT91MemcState;

static uint64_t at91_memc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91MemcState *s = opaque;
    uint32_t value = s->regs[offset >> 2];

    trace_at91_memc_read(s->name, offset, value);
    return value;
}

static void at91_memc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    AT91MemcState *s = opaque;
    uint32_t val = value;

    s->regs[offset >> 2] = val;
    trace_at91_memc_write(s->name, offset, val);
}

static const MemoryRegionOps at91_memc_ops = {
    .read = at91_memc_read,
    .write = at91_memc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void at91_memc_reset(DeviceState *dev)
{
    AT91MemcState *s = (AT91MemcState *)dev;

    memset(s->regs, 0, sizeof(s->regs));
    if (s->is_ddramc) {
        /* The EK has DDR2 attached; report that to reset/power-management. */
        s->regs[DDRAMC_MDR >> 2] = DDRAMC_MDR_DDR2;
    }
}

static const VMStateDescription vmstate_at91_memc = {
    .name = "at91-memc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AT91MemcState, MEMC_NUM_REGS),
        VMSTATE_END_OF_LIST()
    },
};

static void at91_memc_init_common(Object *obj, const char *name,
                                  bool is_ddramc)
{
    AT91MemcState *s = (AT91MemcState *)obj;

    s->name = name;
    s->is_ddramc = is_ddramc;
    memory_region_init_io(&s->iomem, obj, &at91_memc_ops, s, name,
                          MEMC_IOMEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void at91_ddramc_init(Object *obj)
{
    at91_memc_init_common(obj, TYPE_AT91_DDRAMC, true);
}

static void at91_smc_init(Object *obj)
{
    at91_memc_init_common(obj, TYPE_AT91_SMC, false);
}

static void at91_matrix_init(Object *obj)
{
    at91_memc_init_common(obj, TYPE_AT91_MATRIX, false);
}

static void at91_ecc_init(Object *obj)
{
    at91_memc_init_common(obj, TYPE_AT91_ECC, false);
}

static void at91_memc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_at91_memc;
    device_class_set_legacy_reset(dc, at91_memc_reset);
}

static const TypeInfo at91_memc_types[] = {
    {
        .name = TYPE_AT91_DDRAMC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AT91MemcState),
        .instance_init = at91_ddramc_init,
        .class_init = at91_memc_class_init,
    }, {
        .name = TYPE_AT91_SMC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AT91MemcState),
        .instance_init = at91_smc_init,
        .class_init = at91_memc_class_init,
    }, {
        .name = TYPE_AT91_MATRIX,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AT91MemcState),
        .instance_init = at91_matrix_init,
        .class_init = at91_memc_class_init,
    }, {
        .name = TYPE_AT91_ECC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AT91MemcState),
        .instance_init = at91_ecc_init,
        .class_init = at91_memc_class_init,
    },
};

DEFINE_TYPES(at91_memc_types)
