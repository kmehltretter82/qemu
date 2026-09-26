/*
 * Minimal SPI model of the AD9467 ADC control interface (AN-877 register
 * map), enough for Linux drivers/iio/adc/ad9467.c to probe and calibrate.
 *
 * Each chip-select assertion carries a 16-bit instruction (bit 15 = read,
 * bits 12:0 = address) followed by data bytes. Register 0x01 reads the
 * chip ID (0x50); writes to 0xff (transfer) self-clear.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define TYPE_AD9467 "ad9467"
OBJECT_DECLARE_SIMPLE_TYPE(AD9467State, AD9467)

#define AD9467_CHIP_ID          0x50
#define REG_CHIP_ID             0x01
#define REG_TRANSFER            0xff

struct AD9467State {
    SSIPeripheral parent_obj;

    uint8_t regs[256];
    unsigned int pos;
    uint16_t instr;
    uint16_t addr;
    bool read;
};

static int ad9467_set_cs(SSIPeripheral *dev, bool select)
{
    AD9467State *s = AD9467(dev);

    /* a new instruction starts with every chip select assertion */
    if (select) {
        s->pos = 0;
    }
    return 0;
}

static uint32_t ad9467_transfer(SSIPeripheral *dev, uint32_t val)
{
    AD9467State *s = AD9467(dev);
    uint32_t ret = 0;

    switch (s->pos) {
    case 0:
        s->instr = (val & 0xff) << 8;
        break;
    case 1:
        s->instr |= val & 0xff;
        s->read = s->instr & 0x8000;
        s->addr = s->instr & 0xff;
        break;
    default:
        if (s->read) {
            ret = s->regs[s->addr];
        } else if (s->addr == REG_TRANSFER) {
            /* self-clearing: the update takes effect immediately */
        } else if (s->addr != REG_CHIP_ID) {
            s->regs[s->addr] = val;
        }
        s->addr = (s->addr + 1) & 0xff;
        break;
    }
    s->pos++;
    return ret;
}

static void ad9467_realize(SSIPeripheral *dev, Error **errp)
{
    AD9467State *s = AD9467(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[REG_CHIP_ID] = AD9467_CHIP_ID;
}

static void ad9467_class_init(ObjectClass *klass, const void *data)
{
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->realize = ad9467_realize;
    k->transfer = ad9467_transfer;
    k->set_cs = ad9467_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->desc = "AD9467 ADC SPI control interface test model";
}

static const TypeInfo ad9467_info = {
    .name          = TYPE_AD9467,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(AD9467State),
    .class_init    = ad9467_class_init,
};

static void ad9467_register_types(void)
{
    type_register_static(&ad9467_info);
}

type_init(ad9467_register_types)
