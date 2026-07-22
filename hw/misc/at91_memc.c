/*
 * Atmel AT91 DDRAMC, static-memory, matrix and ECC register banks.
 *
 * The board's SDRAM remains a normal QEMU RAM region.  These devices model
 * the configuration plane used by boot firmware and Linux: register writes
 * are retained and read back, and migrate with the guest.  On SAM9G45 the
 * MATRIX also partitions the shared 64 KiB SRAM backing between ordinary
 * AHB SRAM and the ARM926 ITCM/DTCM ports.  SMC timing and matrix arbitration
 * do not delay QEMU memory transactions.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/at91_memc.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "target/arm/cpu-qom.h"
#include "trace.h"

#define MEMC_IOMEM_SIZE  0x200
#define MEMC_NUM_REGS    (MEMC_IOMEM_SIZE / sizeof(uint32_t))

#define DDRAMC_MDR       0x20
#define DDRAMC_MDR_DDR2  6

#define MATRIX_TCMR               0x110
#define MATRIX_TCMR_ITCM_SIZE     0x00000007
#define MATRIX_TCMR_DTCM_SIZE     0x00000070
#define MATRIX_TCMR_TCM_NWS       (1U << 11)
#define MATRIX_TCMR_MASK          (MATRIX_TCMR_ITCM_SIZE | \
                                   MATRIX_TCMR_DTCM_SIZE | \
                                   MATRIX_TCMR_TCM_NWS)
#define MATRIX_TCM_SIZE_32K       6
#define MATRIX_TCM_SIZE_64K       7
#define MATRIX_SRAM_SIZE          (64 * KiB)
#define MATRIX_TCM_HALF_SIZE      (32 * KiB)

typedef struct AT91MemcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[MEMC_NUM_REGS];
    const char *name;
    bool is_ddramc;

    ARMCPU *cpu;
    MemoryRegion *sram;
    MemoryRegion *memory;
    uint64_t itcm_base;
    uint64_t dtcm_base;
    uint64_t sram_base;
    MemoryRegion sram_alias;
    MemoryRegion itcm_alias;
    MemoryRegion dtcm_alias;
    bool tcm_wired;
} AT91MemcState;

static bool at91_matrix_tcm_valid(uint32_t value)
{
    uint32_t itcm_size = value & MATRIX_TCMR_ITCM_SIZE;
    uint32_t dtcm_size = (value & MATRIX_TCMR_DTCM_SIZE) >> 4;

    return (!itcm_size && !dtcm_size) ||
           (!itcm_size && dtcm_size == MATRIX_TCM_SIZE_64K) ||
           (itcm_size == MATRIX_TCM_SIZE_32K &&
            dtcm_size == MATRIX_TCM_SIZE_32K);
}

static void at91_matrix_apply_tcm(AT91MemcState *s)
{
    uint32_t value = s->regs[MATRIX_TCMR >> 2];
    uint8_t itcm_size = value & MATRIX_TCMR_ITCM_SIZE;
    uint8_t dtcm_size = (value & MATRIX_TCMR_DTCM_SIZE) >> 4;

    if (!s->tcm_wired) {
        return;
    }

    memory_region_transaction_begin();
    memory_region_set_enabled(&s->sram_alias, false);
    memory_region_set_enabled(&s->itcm_alias, false);
    memory_region_set_enabled(&s->dtcm_alias, false);

    if (!itcm_size && !dtcm_size) {
        memory_region_set_enabled(&s->sram_alias, true);
    } else if (!itcm_size) {
        memory_region_set_alias_offset(&s->dtcm_alias, 0);
        memory_region_set_size(&s->dtcm_alias, MATRIX_SRAM_SIZE);
        memory_region_set_enabled(&s->dtcm_alias, true);
    } else {
        memory_region_set_enabled(&s->itcm_alias, true);
        memory_region_set_alias_offset(&s->dtcm_alias,
                                       MATRIX_TCM_HALF_SIZE);
        memory_region_set_size(&s->dtcm_alias, MATRIX_TCM_HALF_SIZE);
        memory_region_set_enabled(&s->dtcm_alias, true);
    }
    memory_region_transaction_commit();

    arm_cpu_tcm_configure(s->cpu, itcm_size, dtcm_size,
                          s->itcm_base, s->dtcm_base);
}

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

    if (s->tcm_wired && offset == MATRIX_TCMR) {
        val &= MATRIX_TCMR_MASK;
        if (!at91_matrix_tcm_valid(val)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "at91-matrix: invalid TCM partition 0x%08x\n",
                          val);
            trace_at91_memc_write(s->name, offset,
                                  s->regs[offset >> 2]);
            return;
        }
    }

    s->regs[offset >> 2] = val;
    trace_at91_memc_write(s->name, offset, val);
    if (s->tcm_wired && offset == MATRIX_TCMR) {
        at91_matrix_apply_tcm(s);
    }
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
    at91_matrix_apply_tcm(s);
}

static int at91_memc_post_load(void *opaque, int version_id)
{
    AT91MemcState *s = opaque;

    if (s->tcm_wired &&
        !at91_matrix_tcm_valid(s->regs[MATRIX_TCMR >> 2])) {
        return -EINVAL;
    }
    at91_matrix_apply_tcm(s);
    return 0;
}

static const VMStateDescription vmstate_at91_memc = {
    .name = "at91-memc",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = at91_memc_post_load,
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

static void at91_matrix_realize(DeviceState *dev, Error **errp)
{
    AT91MemcState *s = (AT91MemcState *)dev;
    bool no_wiring = !s->cpu && !s->sram && !s->memory &&
                     s->itcm_base == UINT64_MAX &&
                     s->dtcm_base == UINT64_MAX &&
                     s->sram_base == UINT64_MAX;

    /* Other AT91 SoCs may use this as a retained register bank only. */
    if (no_wiring) {
        return;
    }
    if (!s->cpu || !s->sram || !s->memory ||
        s->itcm_base == UINT64_MAX || s->dtcm_base == UINT64_MAX ||
        s->sram_base == UINT64_MAX) {
        error_setg(errp, "at91-matrix: incomplete TCM/SRAM wiring");
        return;
    }
    if (!arm_cpu_has_tcm(s->cpu)) {
        error_setg(errp, "at91-matrix: linked CPU has no ARM926 TCM support");
        return;
    }
    if (memory_region_size(s->sram) < MATRIX_SRAM_SIZE) {
        error_setg(errp, "at91-matrix: SRAM backing is smaller than 64 KiB");
        return;
    }
    if (s->itcm_base > UINT32_MAX - (MATRIX_TCM_HALF_SIZE - 1) ||
        s->dtcm_base > UINT32_MAX - (MATRIX_SRAM_SIZE - 1) ||
        s->sram_base > UINT32_MAX - (MATRIX_SRAM_SIZE - 1) ||
        !QEMU_IS_ALIGNED(s->itcm_base, MATRIX_TCM_HALF_SIZE) ||
        !QEMU_IS_ALIGNED(s->dtcm_base, MATRIX_SRAM_SIZE) ||
        !QEMU_IS_ALIGNED(s->sram_base, MATRIX_SRAM_SIZE)) {
        error_setg(errp, "at91-matrix: invalid TCM/SRAM AHB address");
        return;
    }

    memory_region_init_alias(&s->sram_alias, OBJECT(s), "at91.sram-c",
                             s->sram, 0, MATRIX_SRAM_SIZE);
    memory_region_init_alias(&s->itcm_alias, OBJECT(s), "at91.itcm-ahb",
                             s->sram, 0, MATRIX_TCM_HALF_SIZE);
    memory_region_init_alias(&s->dtcm_alias, OBJECT(s), "at91.dtcm-ahb",
                             s->sram, 0, MATRIX_SRAM_SIZE);
    memory_region_set_enabled(&s->itcm_alias, false);
    memory_region_set_enabled(&s->dtcm_alias, false);
    memory_region_add_subregion(s->memory, s->sram_base, &s->sram_alias);
    memory_region_add_subregion(s->memory, s->itcm_base, &s->itcm_alias);
    memory_region_add_subregion(s->memory, s->dtcm_base, &s->dtcm_alias);
    s->tcm_wired = true;
    at91_matrix_apply_tcm(s);
}

static void at91_memc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_at91_memc;
    device_class_set_legacy_reset(dc, at91_memc_reset);
}

static const Property at91_matrix_properties[] = {
    DEFINE_PROP_LINK("cpu", AT91MemcState, cpu, TYPE_ARM_CPU, ARMCPU *),
    DEFINE_PROP_LINK("sram", AT91MemcState, sram,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_LINK("memory", AT91MemcState, memory,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_UINT64("itcm-ahb-base", AT91MemcState, itcm_base,
                       UINT64_MAX),
    DEFINE_PROP_UINT64("dtcm-ahb-base", AT91MemcState, dtcm_base,
                       UINT64_MAX),
    DEFINE_PROP_UINT64("sram-ahb-base", AT91MemcState, sram_base,
                       UINT64_MAX),
};

static void at91_matrix_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    at91_memc_class_init(klass, data);
    dc->realize = at91_matrix_realize;
    device_class_set_props(dc, at91_matrix_properties);
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
        .class_init = at91_matrix_class_init,
    }, {
        .name = TYPE_AT91_ECC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AT91MemcState),
        .instance_init = at91_ecc_init,
        .class_init = at91_memc_class_init,
    },
};

DEFINE_TYPES(at91_memc_types)
