/*
 * Minimal model of the Analog Devices generic AXI-ADC core (10.0.a), as
 * used by Linux drivers/iio/adc/adi-axi-adc.c. Register file only: the
 * sample data comes from the AXI-DMAC model's generator.
 *
 *  - VERSION reports 10.0.a
 *  - DRP_STATUS always reports the MMCM as locked
 *  - CHAN_STATUS always reads 0 (no PN errors), so interface calibration
 *    passes at every tap
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ADI_AXI_ADC "adi-axi-adc"
OBJECT_DECLARE_SIMPLE_TYPE(AdiAxiAdcState, ADI_AXI_ADC)

#define ADC_REGS                0x4000  /* 64 KiB window, 32-bit regs */
#define REG_VERSION             0x0000
#define REG_DRP_STATUS          0x0074
#define   DRP_LOCKED            BIT(17)
#define REG_SYNC_STATUS         0x0068
#define REG_CHAN_STATUS(c)      (0x0404 + (c) * 0x40)

#define ADC_VERSION             ((10 << 16) | (0 << 8) | 'a')

struct AdiAxiAdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[ADC_REGS];
};

static bool adi_axi_adc_is_chan_status(hwaddr addr)
{
    return addr >= 0x400 && addr < 0x800 && (addr & 0x3f) == 0x04;
}

static uint64_t adi_axi_adc_read(void *opaque, hwaddr addr, unsigned size)
{
    AdiAxiAdcState *s = opaque;

    if (addr == REG_VERSION) {
        return ADC_VERSION;
    }
    if (addr == REG_DRP_STATUS) {
        return s->regs[addr / 4] | DRP_LOCKED;
    }
    if (addr == REG_SYNC_STATUS) {
        return 1;
    }
    if (adi_axi_adc_is_chan_status(addr)) {
        return 0;
    }
    return s->regs[addr / 4];
}

static void adi_axi_adc_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    AdiAxiAdcState *s = opaque;

    s->regs[addr / 4] = val;
}

static const MemoryRegionOps adi_axi_adc_ops = {
    .read = adi_axi_adc_read,
    .write = adi_axi_adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void adi_axi_adc_reset(DeviceState *dev)
{
    AdiAxiAdcState *s = ADI_AXI_ADC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void adi_axi_adc_init(Object *obj)
{
    AdiAxiAdcState *s = ADI_AXI_ADC(obj);

    memory_region_init_io(&s->iomem, obj, &adi_axi_adc_ops, s,
                          TYPE_ADI_AXI_ADC, ADC_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void adi_axi_adc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, adi_axi_adc_reset);
    dc->desc = "ADI AXI-ADC test model";
    dc->user_creatable = false;
}

static const TypeInfo adi_axi_adc_info = {
    .name          = TYPE_ADI_AXI_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AdiAxiAdcState),
    .instance_init = adi_axi_adc_init,
    .class_init    = adi_axi_adc_class_init,
};

static void adi_axi_adc_register_types(void)
{
    type_register_static(&adi_axi_adc_info);
}

type_init(adi_axi_adc_register_types)
