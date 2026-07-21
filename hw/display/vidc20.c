/*
 * Acorn VIDC20 video controller (RiscPC).
 *
 * VIDC20 has no address decode worth the name: the guest writes a single
 * 32-bit word to one location and the register is selected by the top
 * bits of the *data*.  Bits 31..28 pick the register group, and for the
 * timing groups bits 27..24 pick the register within it (VIDC20
 * datasheet S4.1, "VIDC20 register allocation").
 *
 * Registers are write-only.  Reads float, like the rest of the I/O bus.
 *
 * The frame store lives elsewhere: VIDC20 supplies timing and pixel
 * format, while the IOMD supplies the DMA address (VIDSTART/VIDINIT) and
 * the enable bit.  See acorn_iomd_video_dma().
 *
 * Written from docs/vidc20-datasheet.pdf, cross-checked against two
 * independent implementations - RPCEmu's src/vidc20.c (written against
 * RISC OS) and NetBSD's sys/arch/arm/iomd/vidc20config.c - rather than
 * from any one guest driver's usage.  Where the three agree, that is
 * taken as hardware behaviour.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/vidc20.h"
#include "hw/display/framebuffer.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "system/address-spaces.h"

/* Register groups, from bits 31..28 of the written word. */
#define VIDC20_PALETTE      0x0
#define VIDC20_PALADDR      0x1
#define VIDC20_BORDER       0x4
#define VIDC20_CURSPAL1     0x5
#define VIDC20_CURSPAL2     0x6
#define VIDC20_CURSPAL3     0x7
#define VIDC20_HORIZ        0x8
#define VIDC20_VERT         0x9
#define VIDC20_STEREO       0xa
#define VIDC20_SOUND        0xb
#define VIDC20_ECTL         0xc
#define VIDC20_FSYNTH       0xd
#define VIDC20_CONTROL      0xe
#define VIDC20_DCTL         0xf

/* Timing registers, from bits 31..24. */
#define VIDC20_HDSR         0x83
#define VIDC20_HDER         0x84
#define VIDC20_VDSR         0x93
#define VIDC20_VDER         0x94

/*
 * Field widths.  RPCEmu masks HDSR/HDER with 0x3ffe (14 bits, and the
 * horizontal registers count in units of two pixels, so bit 0 is not
 * part of the value) and VDSR/VDER with 0x1fff (13 bits).
 */
#define VIDC20_H_MASK       0x3ffe
#define VIDC20_V_MASK       0x1fff

/* Control register: bits 7..5 select pixel depth. */
#define VIDC20_CTRL_BPP(v)  (((v) >> 5) & 7)

static const int vidc20_bpp[8] = { 1, 2, 4, 8, 16, 0, 32, 0 };

static void vidc20_update_palette(VIDC20State *s, unsigned idx)
{
    uint32_t p = s->palette[idx];

    /* 28-bit entry: R 7..0, G 15..8, B 23..16, external 27..24. */
    s->pal_argb[idx] = rgb_to_pixel32(p & 0xff, (p >> 8) & 0xff,
                                      (p >> 16) & 0xff);
}

static void vidc20_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
    VIDC20State *s = opaque;
    uint32_t val = value;

    switch (val >> 28) {
    case VIDC20_PALETTE:
        s->palette[s->palindex] = val & 0x0fffffff;
        vidc20_update_palette(s, s->palindex);
        /* The pointer post-increments and wraps 255 -> 0 (datasheet S4.1.2). */
        s->palindex = (s->palindex + 1) & 0xff;
        s->invalidate = true;
        break;

    case VIDC20_PALADDR:
        /* Reserved bits set means the write is not a palette address. */
        if (val & 0x0fffff00) {
            return;
        }
        s->palindex = val & 0xff;
        break;

    case VIDC20_BORDER:
        s->border = val & 0x0fffffff;
        s->invalidate = true;
        break;

    case VIDC20_CURSPAL1:
    case VIDC20_CURSPAL2:
    case VIDC20_CURSPAL3:
        s->cursor_palette[(val >> 28) - VIDC20_CURSPAL1] = val & 0x0fffffff;
        break;

    case VIDC20_HORIZ:
    case VIDC20_VERT:
        switch (val >> 24) {
        case VIDC20_HDSR:
            s->hdsr = val & VIDC20_H_MASK;
            s->invalidate = true;
            break;
        case VIDC20_HDER:
            s->hder = val & VIDC20_H_MASK;
            s->invalidate = true;
            break;
        case VIDC20_VDSR:
            s->vdsr = val & VIDC20_V_MASK;
            s->invalidate = true;
            break;
        case VIDC20_VDER:
            s->vder = val & VIDC20_V_MASK;
            s->invalidate = true;
            break;
        default:
            /*
             * Cycle, sync width, border and interlace registers only
             * affect the analogue timing of a real monitor, and the
             * cursor registers are not modelled yet.
             */
            break;
        }
        break;

    case VIDC20_CONTROL:
        if (val & 0x0ff00000) {
            return;     /* reserved bits set */
        }
        if (VIDC20_CTRL_BPP(val) != VIDC20_CTRL_BPP(s->control)) {
            s->invalidate = true;
        }
        s->control = val & 0x000fffff;
        break;

    case VIDC20_DCTL:
        s->dctl = val & 0x000fffff;
        break;

    case VIDC20_ECTL:
        s->ectl = val & 0x0fffffff;
        break;

    case VIDC20_FSYNTH:
    case VIDC20_STEREO:
    case VIDC20_SOUND:
        /* Pixel clock and sound: no effect on what we display. */
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "vidc20: write to unknown register 0x%x (0x%08x)\n",
                      val >> 28, val);
        break;
    }
}

static uint64_t vidc20_read(void *opaque, hwaddr offset, unsigned size)
{
    /* VIDC20 is write-only; the bus floats high. */
    return (uint64_t)-1;
}

static const MemoryRegionOps vidc20_ops = {
    .read = vidc20_read,
    .write = vidc20_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* Row converters. src is the guest frame store, dest a 32bpp surface. */
#define DRAW_PALETTE(name, bits)                                        \
static void name(void *opaque, uint8_t *d, const uint8_t *src,          \
                 int width, int deststep)                               \
{                                                                       \
    VIDC20State *s = opaque;                                            \
    unsigned per_byte = 8 / (bits);                                     \
    unsigned mask = (1u << (bits)) - 1;                                 \
    int x;                                                              \
                                                                        \
    for (x = 0; x < width; x++) {                                       \
        unsigned b = src[x / per_byte];                                 \
        unsigned shift = (x % per_byte) * (bits);                       \
        *(uint32_t *)d = s->pal_argb[(b >> shift) & mask];              \
        d += deststep;                                                  \
    }                                                                   \
}

DRAW_PALETTE(vidc20_draw_1bpp, 1)
DRAW_PALETTE(vidc20_draw_2bpp, 2)
DRAW_PALETTE(vidc20_draw_4bpp, 4)
DRAW_PALETTE(vidc20_draw_8bpp, 8)

static void vidc20_draw_16bpp(void *opaque, uint8_t *d, const uint8_t *src,
                              int width, int deststep)
{
    int x;

    /* 5:5:5, red in bits 4..0 (acornfb.c:473-482 and the datasheet). */
    for (x = 0; x < width; x++) {
        uint16_t p = lduw_le_p(src + x * 2);
        unsigned r = (p >> 0) & 0x1f, g = (p >> 5) & 0x1f, b = (p >> 10) & 0x1f;

        *(uint32_t *)d = rgb_to_pixel32(r << 3 | r >> 2, g << 3 | g >> 2,
                                        b << 3 | b >> 2);
        d += deststep;
    }
}

static void vidc20_draw_32bpp(void *opaque, uint8_t *d, const uint8_t *src,
                              int width, int deststep)
{
    int x;

    for (x = 0; x < width; x++) {
        uint32_t p = ldl_le_p(src + x * 4);

        *(uint32_t *)d = rgb_to_pixel32(p & 0xff, (p >> 8) & 0xff,
                                        (p >> 16) & 0xff);
        d += deststep;
    }
}

static drawfn vidc20_drawfn(int bpp)
{
    switch (bpp) {
    case 1:  return vidc20_draw_1bpp;
    case 2:  return vidc20_draw_2bpp;
    case 4:  return vidc20_draw_4bpp;
    case 8:  return vidc20_draw_8bpp;
    case 16: return vidc20_draw_16bpp;
    case 32: return vidc20_draw_32bpp;
    default: return NULL;
    }
}

static bool vidc20_update_display(void *opaque)
{
    VIDC20State *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    int xres = s->hder - s->hdsr;
    int yres = s->vder - s->vdsr;
    int bpp = vidc20_bpp[VIDC20_CTRL_BPP(s->control)];
    int first = 0, last = 0, src_width;
    hwaddr base;
    drawfn fn;

    /*
     * Blank unless the IOMD is actually feeding us: RPCEmu makes the same
     * check on IOMD VIDCR bit 5 and on vdsr > vder.
     */
    if (xres <= 0 || yres <= 0 || bpp == 0 ||
        !acorn_iomd_video_dma(s->iomd, &base)) {
        return true;
    }

    fn = vidc20_drawfn(bpp);
    if (!fn) {
        return true;
    }

    if (xres != s->last_width || yres != s->last_height ||
        bpp != s->last_bpp) {
        s->last_width = xres;
        s->last_height = yres;
        s->last_bpp = bpp;
        s->invalidate = true;
        qemu_console_resize(s->con, xres, yres);
        surface = qemu_console_surface(s->con);
    }

    src_width = (xres * bpp) / 8;

    if (s->invalidate || base != s->last_base) {
        s->last_base = base;
        framebuffer_update_memory_section(&s->fbsection, s->fbmem, base,
                                          yres, src_width);
    }

    framebuffer_update_display(surface, &s->fbsection, xres, yres,
                               src_width, surface_stride(surface), 4,
                               s->invalidate, fn, s, &first, &last);
    if (first >= 0) {
        qemu_console_update(s->con, 0, first, xres, last - first + 1);
    }
    s->invalidate = false;
    return true;
}

static void vidc20_invalidate_display(void *opaque)
{
    VIDC20State *s = opaque;

    s->invalidate = true;
}

static const GraphicHwOps vidc20_gfx_ops = {
    .invalidate = vidc20_invalidate_display,
    .gfx_update = vidc20_update_display,
};

static void vidc20_reset_hold(Object *obj, ResetType type)
{
    VIDC20State *s = VIDC20(obj);

    memset(s->palette, 0, sizeof(s->palette));
    memset(s->pal_argb, 0, sizeof(s->pal_argb));
    memset(s->cursor_palette, 0, sizeof(s->cursor_palette));
    s->palindex = 0;
    s->border = 0;
    s->control = 0;
    s->dctl = 0;
    s->ectl = 0;
    s->hdsr = s->hder = s->vdsr = s->vder = 0;
    s->last_width = s->last_height = s->last_bpp = -1;
    s->last_base = 0;
    s->invalidate = true;
}

static void vidc20_realize(DeviceState *dev, Error **errp)
{
    VIDC20State *s = VIDC20(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (!s->fbmem) {
        error_setg(errp, "'framebuffer-memory' property was not set");
        return;
    }
    if (!s->iomd) {
        error_setg(errp, "'iomd' link was not set");
        return;
    }

    /*
     * The guest only ever writes one word, but VIDC20 decodes a whole
     * megabyte of the I/O space at 0x03400000.
     */
    memory_region_init_io(&s->iomem, OBJECT(s), &vidc20_ops, s,
                          "vidc20", 0x100000);
    sysbus_init_mmio(sbd, &s->iomem);
    s->con = qemu_graphic_console_create(dev, 0, &vidc20_gfx_ops, s);
}

static const VMStateDescription vmstate_vidc20 = {
    .name = "vidc20",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(palette, VIDC20State, 256),
        VMSTATE_UINT32_ARRAY(cursor_palette, VIDC20State, 3),
        VMSTATE_UINT32(palindex, VIDC20State),
        VMSTATE_UINT32(border, VIDC20State),
        VMSTATE_UINT32(control, VIDC20State),
        VMSTATE_UINT32(dctl, VIDC20State),
        VMSTATE_UINT32(ectl, VIDC20State),
        VMSTATE_UINT32(hdsr, VIDC20State),
        VMSTATE_UINT32(hder, VIDC20State),
        VMSTATE_UINT32(vdsr, VIDC20State),
        VMSTATE_UINT32(vder, VIDC20State),
        VMSTATE_END_OF_LIST(),
    },
};

static const Property vidc20_properties[] = {
    DEFINE_PROP_LINK("framebuffer-memory", VIDC20State, fbmem,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_LINK("iomd", VIDC20State, iomd,
                     TYPE_ACORN_IOMD, AcornIOMDState *),
};

static void vidc20_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Acorn VIDC20";
    dc->realize = vidc20_realize;
    dc->vmsd = &vmstate_vidc20;
    rc->phases.hold = vidc20_reset_hold;
    device_class_set_props(dc, vidc20_properties);
}

static const TypeInfo vidc20_info = {
    .name          = TYPE_VIDC20,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(VIDC20State),
    .class_init    = vidc20_class_init,
};

static void vidc20_register_types(void)
{
    type_register_static(&vidc20_info);
}

type_init(vidc20_register_types)
