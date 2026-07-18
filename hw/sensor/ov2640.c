/*
 * OmniVision OV2640 camera sensor, I2C (SCCB) control interface.
 *
 * Models only the two-bank register file the Linux ov2640 driver talks to:
 * bank selection via register 0xff, identification registers (PID/VER and
 * the manufacturer ID) served from fixed values, and retention of every
 * other register write.  Pixel data does not flow through this device; a
 * camera interface model (e.g. the AT91 ISI) synthesizes frame content.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_OV2640 "ov2640"
OBJECT_DECLARE_SIMPLE_TYPE(OV2640State, OV2640)

#define OV2640_BANK_SELECT   0xff
#define OV2640_BANK_DSP      0
#define OV2640_BANK_SENSOR   1

/* Sensor-bank identification registers. */
#define OV2640_REG_PID       0x0a
#define OV2640_REG_VER       0x0b
#define OV2640_REG_MIDH      0x1c
#define OV2640_REG_MIDL      0x1d

#define OV2640_PID           0x26
#define OV2640_VER           0x42
#define OV2640_MIDH          0x7f
#define OV2640_MIDL          0xa2

struct OV2640State {
    I2CSlave parent_obj;

    uint8_t regs[2][256];
    uint8_t bank;
    uint8_t reg_ptr;
    bool have_reg;
};

static int ov2640_event(I2CSlave *i2c, enum i2c_event event)
{
    OV2640State *s = OV2640(i2c);

    if (event == I2C_START_SEND) {
        s->have_reg = false;
    }
    return 0;
}

static uint8_t ov2640_rx(I2CSlave *i2c)
{
    OV2640State *s = OV2640(i2c);

    if (s->bank == OV2640_BANK_SENSOR) {
        switch (s->reg_ptr) {
        case OV2640_REG_PID:
            return OV2640_PID;
        case OV2640_REG_VER:
            return OV2640_VER;
        case OV2640_REG_MIDH:
            return OV2640_MIDH;
        case OV2640_REG_MIDL:
            return OV2640_MIDL;
        default:
            break;
        }
    }
    return s->regs[s->bank][s->reg_ptr];
}

static int ov2640_tx(I2CSlave *i2c, uint8_t data)
{
    OV2640State *s = OV2640(i2c);

    if (!s->have_reg) {
        s->reg_ptr = data;
        s->have_reg = true;
    } else if (s->reg_ptr == OV2640_BANK_SELECT) {
        s->bank = data & 1;
    } else {
        s->regs[s->bank][s->reg_ptr] = data;
    }
    return 0;
}

static void ov2640_reset(DeviceState *dev)
{
    OV2640State *s = OV2640(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->bank = OV2640_BANK_DSP;
    s->reg_ptr = 0;
    s->have_reg = false;
}

static const VMStateDescription vmstate_ov2640 = {
    .name = "ov2640",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, OV2640State),
        VMSTATE_UINT8_2DARRAY(regs, OV2640State, 2, 256),
        VMSTATE_UINT8(bank, OV2640State),
        VMSTATE_UINT8(reg_ptr, OV2640State),
        VMSTATE_BOOL(have_reg, OV2640State),
        VMSTATE_END_OF_LIST()
    },
};

static void ov2640_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = ov2640_event;
    k->recv = ov2640_rx;
    k->send = ov2640_tx;
    dc->desc = "OmniVision OV2640 camera sensor (control interface)";
    dc->vmsd = &vmstate_ov2640;
    device_class_set_legacy_reset(dc, ov2640_reset);
}

static const TypeInfo ov2640_types[] = {
    {
        .name = TYPE_OV2640,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(OV2640State),
        .class_init = ov2640_class_init,
    },
};

DEFINE_TYPES(ov2640_types)
