/*
 * Nintendo 3DS I2C bus controller (three instances: 0x10161000,
 * 0x10144000, 0x10148000).
 *
 * Byte-at-a-time controller: DATA holds the byte, CNT carries the
 * start/stop/direction flags and the busy/ack status.
 *   CNT bit 0: send STOP after this byte
 *   CNT bit 1: send START before this byte (DATA = address | R/W)
 *   CNT bit 2: no data byte (stop/ack handling only)
 *   CNT bit 4: on write, ACK to send after a received byte;
 *              on read, ACK status of the last transfer
 *   CNT bit 5: direction, 1 = read
 *   CNT bit 6: interrupt enable
 *   CNT bit 7: start transfer / busy
 *
 * Documentation: 3dbrew I2C_Registers, GBATEK "3DS I2C Bus".
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/n3ds.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(N3DSI2CState, N3DS_I2C)

#define REG_DATA    0x0
#define REG_CNT     0x1
#define REG_CNTEX   0x2
#define REG_SCL     0x4

#define CNT_STOP    0x01
#define CNT_START   0x02
#define CNT_NODATA  0x04
#define CNT_ACK     0x10
#define CNT_READ    0x20
#define CNT_IRQEN   0x40
#define CNT_BUSY    0x80

struct N3DSI2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;

    uint8_t data;
    uint8_t cnt;
    uint8_t cnt_reset;
    uint16_t cntex;
    uint16_t scl;
    bool in_xfer;
};

static void n3ds_i2c_cnt_write(N3DSI2CState *s, uint8_t v)
{
    bool nack = false;

    if (!(v & CNT_BUSY)) {
        s->cnt = v & (CNT_IRQEN | CNT_READ | CNT_ACK);
        return;
    }

    if (v & CNT_START) {
        nack = i2c_start_transfer(s->bus, s->data >> 1, s->data & 1) != 0;
        s->in_xfer = !nack;
    } else if (v & CNT_NODATA) {
        nack = false;
    } else if (!s->in_xfer) {
        nack = true;
    } else if (v & CNT_READ) {
        s->data = i2c_recv(s->bus);
        if (!(v & CNT_ACK)) {
            i2c_nack(s->bus);
        }
    } else {
        nack = i2c_send(s->bus, s->data) != 0;
    }

    if ((v & CNT_STOP) && s->in_xfer) {
        i2c_end_transfer(s->bus);
        s->in_xfer = false;
    }

    s->cnt = (v & (CNT_IRQEN | CNT_READ)) | (nack ? 0 : CNT_ACK);
    if (v & CNT_IRQEN) {
        qemu_irq_pulse(s->irq);
    }
}

static uint64_t n3ds_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSI2CState *s = opaque;

    switch (off) {
    case REG_DATA:
        return s->data;
    case REG_CNT:
        return s->cnt;
    case REG_CNTEX:
        return s->cntex & 0xFF;
    case REG_CNTEX + 1:
        return s->cntex >> 8;
    case REG_SCL:
        return s->scl & 0xFF;
    case REG_SCL + 1:
        return s->scl >> 8;
    default:
        return 0;
    }
}

static void n3ds_i2c_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSI2CState *s = opaque;

    switch (off) {
    case REG_DATA:
        s->data = v;
        break;
    case REG_CNT:
        n3ds_i2c_cnt_write(s, v);
        break;
    case REG_CNTEX:
        s->cntex = (s->cntex & 0xFF00) | (v & 0xFF);
        break;
    case REG_CNTEX + 1:
        s->cntex = (s->cntex & 0x00FF) | ((v & 0xFF) << 8);
        break;
    case REG_SCL:
        s->scl = (s->scl & 0xFF00) | (v & 0xFF);
        break;
    case REG_SCL + 1:
        s->scl = (s->scl & 0x00FF) | ((v & 0xFF) << 8);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps n3ds_i2c_ops = {
    .read = n3ds_i2c_read,
    .write = n3ds_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
};

static void n3ds_i2c_reset(DeviceState *dev)
{
    N3DSI2CState *s = N3DS_I2C(dev);

    s->data = 0;
    s->cnt = s->cnt_reset;
    /* CNTEX and SCL read this on all three buses (n3ds-probes io_i2c) */
    s->cntex = 0x0003;
    s->scl = 0x0500;
    s->in_xfer = false;
}

static const Property n3ds_i2c_extra_properties[] = {
    /* CNT keeps whatever the last transfer left; the MCU bus reads 0x61 */
    DEFINE_PROP_UINT8("cnt-reset", N3DSI2CState, cnt_reset, 0),
};

static void n3ds_i2c_init(Object *obj)
{
    N3DSI2CState *s = N3DS_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &n3ds_i2c_ops, s, "n3ds.i2c", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static void n3ds_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n3ds_i2c_reset);
    device_class_set_props(dc, n3ds_i2c_extra_properties);
}

static const TypeInfo n3ds_i2c_info = {
    .name          = TYPE_N3DS_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSI2CState),
    .instance_init = n3ds_i2c_init,
    .class_init    = n3ds_i2c_class_init,
};

static void n3ds_i2c_register_types(void)
{
    type_register_static(&n3ds_i2c_info);
}

type_init(n3ds_i2c_register_types)
