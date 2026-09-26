/*
 * Minimal Samsung SUR40 (045e:0775) model for testing the Linux sur40
 * driver's video path on a real, DMA capable host controller.
 *
 *   ep 0x86 bulk in: touch packets, 16-byte header with zero blobs
 *   ep 0x82 bulk in: 20-byte "SUBF" image header, then a 960x540 image
 *
 * Every image byte of frame n is (n & 0xff).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "desc.h"
#include "qom/object.h"

#define SUR40_IMG_SIZE      (960 * 540)
#define SUR40_TOUCH_EP      6
#define SUR40_VIDEO_EP      2

struct USBSur40State {
    USBDevice dev;
    bool in_image;
    uint32_t remaining;
    uint32_t frame;
};

#define TYPE_USB_SUR40 "usb-sur40"
OBJECT_DECLARE_SIMPLE_TYPE(USBSur40State, USB_SUR40)

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings desc_strings = {
    [STR_MANUFACTURER]     = "QEMU",
    [STR_PRODUCT]          = "SUR40 test model",
    [STR_SERIALNUMBER]     = "1",
};

/* sur40 requires endpoint[4] == 0x86 and reads video from 0x82 */
static const USBDescIface desc_iface_sur40 = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 5,
    .bInterfaceClass               = 0xff,
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | 0x01,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        }, {
            .bEndpointAddress      = USB_DIR_OUT | 0x02,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        }, {
            .bEndpointAddress      = USB_DIR_IN | SUR40_VIDEO_EP,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        }, {
            .bEndpointAddress      = USB_DIR_OUT | 0x03,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        }, {
            .bEndpointAddress      = USB_DIR_IN | SUR40_TOUCH_EP,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        },
    },
};

static const USBDescDevice desc_device_sur40 = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE | USB_CFG_ATT_SELFPOWER,
            .bMaxPower             = 50,
            .nif = 1,
            .ifs = &desc_iface_sur40,
        },
    },
};

static const USBDesc desc_sur40 = {
    .id = {
        .idVendor          = 0x045e,
        .idProduct         = 0x0775,
        .bcdDevice         = 0x0100,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIALNUMBER,
    },
    .high = &desc_device_sur40,
    .str  = desc_strings,
};

static void usb_sur40_handle_reset(USBDevice *dev)
{
    USBSur40State *s = USB_SUR40(dev);

    s->in_image = false;
    s->remaining = 0;
}

static void usb_sur40_handle_control(USBDevice *dev, USBPacket *p,
               int request, int value, int index, int length, uint8_t *data)
{
    int ret;

    ret = usb_desc_handle_control(dev, p, request, value, index, length, data);
    if (ret >= 0) {
        return;
    }

    switch (request >> 8) {
    case USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE:
        /* version, sensor and accel caps reads: content is ignored */
        memset(data, 0, length);
        p->actual_length = length;
        break;
    case USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE:
        /* register pokes */
        break;
    default:
        p->status = USB_RET_STALL;
        break;
    }
}

static void usb_sur40_video_in(USBSur40State *s, USBPacket *p)
{
    if (!s->in_image) {
        uint32_t hdr[5] = {
            cpu_to_le32(0x46425553),            /* "SUBF" */
            cpu_to_le32(s->frame),
            cpu_to_le32(SUR40_IMG_SIZE),
            cpu_to_le32(s->frame * 16),
            cpu_to_le32(2),
        };

        usb_packet_copy(p, hdr, MIN(sizeof(hdr), p->iov.size));
        s->in_image = true;
        s->remaining = SUR40_IMG_SIZE;
        return;
    }

    {
        size_t len = MIN(p->iov.size, s->remaining);
        g_autofree uint8_t *buf = g_malloc(len);

        memset(buf, s->frame & 0xff, len);
        usb_packet_copy(p, buf, len);
        s->remaining -= len;
        if (!s->remaining) {
            s->in_image = false;
            s->frame++;
        }
    }
}

static void usb_sur40_handle_data(USBDevice *dev, USBPacket *p)
{
    USBSur40State *s = USB_SUR40(dev);
    uint8_t touch[16] = { 0x01, 0x00 };         /* type 1, 0 blobs */

    if (p->pid != USB_TOKEN_IN) {
        p->status = USB_RET_STALL;
        return;
    }

    switch (p->ep->nr) {
    case SUR40_TOUCH_EP:
        usb_packet_copy(p, touch, MIN(sizeof(touch), p->iov.size));
        break;
    case SUR40_VIDEO_EP:
        usb_sur40_video_in(s, p);
        break;
    default:
        p->status = USB_RET_STALL;
        break;
    }
}

static void usb_sur40_realize(USBDevice *dev, Error **errp)
{
    usb_desc_create_serial(dev);
    usb_desc_init(dev);
}

static const VMStateDescription vmstate_usb_sur40 = {
    .name = "usb-sur40",
    .unmigratable = 1,
};

static void usb_sur40_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->product_desc   = "SUR40 test model";
    uc->usb_desc       = &desc_sur40;
    uc->realize        = usb_sur40_realize;
    uc->handle_attach  = usb_desc_attach;
    uc->handle_reset   = usb_sur40_handle_reset;
    uc->handle_control = usb_sur40_handle_control;
    uc->handle_data    = usb_sur40_handle_data;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
    dc->desc = "SUR40 test model";
    dc->vmsd = &vmstate_usb_sur40;
}

static const TypeInfo sur40_info = {
    .name          = TYPE_USB_SUR40,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(USBSur40State),
    .class_init    = usb_sur40_class_init,
};

static void usb_sur40_register_types(void)
{
    type_register_static(&sur40_info);
}

type_init(usb_sur40_register_types)
