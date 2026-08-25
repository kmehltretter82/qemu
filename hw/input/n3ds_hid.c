/*
 * Nintendo 3DS HID pad register (0x10146000): 12 active-low button
 * bits, driven from the QEMU keyboard.
 *
 *   A=X  B=Z  X=S  Y=A  L=Q  R=W  Start=Enter  Select=Backspace
 *   D-pad = cursor keys
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "ui/input.h"
#include "hw/misc/n3ds.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(N3DSHidState, N3DS_HID)

#define N3DS_HID_NBUTTONS 12

struct N3DSHidState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QemuInputHandlerState *hs;
    uint16_t pad;   /* active low */
};

static const int n3ds_hid_keys[N3DS_HID_NBUTTONS] = {
    [0]  = Q_KEY_CODE_X,          /* A */
    [1]  = Q_KEY_CODE_Z,          /* B */
    [2]  = Q_KEY_CODE_BACKSPACE,  /* Select */
    [3]  = Q_KEY_CODE_RET,        /* Start */
    [4]  = Q_KEY_CODE_RIGHT,
    [5]  = Q_KEY_CODE_LEFT,
    [6]  = Q_KEY_CODE_UP,
    [7]  = Q_KEY_CODE_DOWN,
    [8]  = Q_KEY_CODE_W,          /* R */
    [9]  = Q_KEY_CODE_Q,          /* L */
    [10] = Q_KEY_CODE_S,          /* X */
    [11] = Q_KEY_CODE_A,          /* Y */
};

static void n3ds_hid_event(DeviceState *dev, QemuConsole *src,
                           QemuInputEvent *evt)
{
    N3DSHidState *s = N3DS_HID(dev);
    int qcode = qemu_input_linux_to_qcode(evt->key.key);
    int i;

    for (i = 0; i < N3DS_HID_NBUTTONS; i++) {
        if (n3ds_hid_keys[i] == qcode) {
            if (evt->key.down) {
                s->pad &= ~(1 << i);
            } else {
                s->pad |= 1 << i;
            }
        }
    }
}

static const QemuInputHandler n3ds_hid_handler = {
    .name  = "Nintendo 3DS pad",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = n3ds_hid_event,
};

static uint64_t n3ds_hid_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSHidState *s = opaque;

    switch (off) {
    case 0x0:
        return s->pad & 0x0FFF;
    default:
        return 0;
    }
}

static void n3ds_hid_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
}

static const MemoryRegionOps n3ds_hid_ops = {
    .read = n3ds_hid_read,
    .write = n3ds_hid_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void n3ds_hid_reset(DeviceState *dev)
{
    N3DSHidState *s = N3DS_HID(dev);
    s->pad = 0x0FFF;
}

static void n3ds_hid_realize(DeviceState *dev, Error **errp)
{
    N3DSHidState *s = N3DS_HID(dev);

    s->hs = qemu_input_handler_register(dev, &n3ds_hid_handler);
}

static void n3ds_hid_init(Object *obj)
{
    N3DSHidState *s = N3DS_HID(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &n3ds_hid_ops, s, "n3ds.hid", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    s->pad = 0x0FFF;
}

static void n3ds_hid_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = n3ds_hid_realize;
    device_class_set_legacy_reset(dc, n3ds_hid_reset);
}

static const TypeInfo n3ds_hid_info = {
    .name          = TYPE_N3DS_HID,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSHidState),
    .instance_init = n3ds_hid_init,
    .class_init    = n3ds_hid_class_init,
};

static void n3ds_hid_register_types(void)
{
    type_register_static(&n3ds_hid_info);
}

type_init(n3ds_hid_register_types)
