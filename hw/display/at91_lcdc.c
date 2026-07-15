/*
 * Atmel/Microchip AT91 LCD Controller (atmel_lcdfb).
 *
 * Framebuffer scan-out: the base DMA address (DMABADDR1) points at a frame in
 * guest RAM; geometry comes from LCDFRMCFG and depth from LCDCON2 PIXELSIZE,
 * converted (32/24/16bpp) into the console's xRGB surface.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "ui/console.h"
#include "hw/display/at91_lcdc.h"
#include "trace.h"

#define LCDC_DMABADDR1  0x00
#define LCDC_DMAFRMCFG  0x18
#define LCDC_DMACON     0x1C
#define LCDC_LCDCON1    0x0800
#define LCDC_LCDCON2    0x0804
#define LCDC_LCDFRMCFG  0x0810
#define LCDC_PWRCON     0x083C
#define LCDC_IER        0x0848
#define LCDC_IDR        0x084C
#define LCDC_IMR        0x0850
#define LCDC_ISR        0x0854
#define LCDC_ICR        0x0858

#define LCDC_PWRCON_PWR (1u << 0)   /* LCD module power on */

#define LCDC_WIDTH   480
#define LCDC_HEIGHT  272

struct AT91LcdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QemuConsole *con;
    qemu_irq irq;

    uint32_t dmabaddr1;
    uint32_t dmafrmcfg;
    uint32_t dmacon;
    uint32_t lcdcon1, lcdcon2;
    uint32_t lcdfrmcfg;
    uint32_t pwrcon;
    uint32_t imr;
    bool invalidate;
};

/* Derive geometry (LCDFRMCFG) and pixel depth (LCDCON2 PIXELSIZE) from the
 * programmed registers.  Falls back to the board panel if unprogrammed. */
static void lcdc_get_mode(AT91LcdcState *s, int *width, int *height, int *bpp)
{
    static const int pixelsize[8] = { 1, 2, 4, 8, 16, 24, 32, 32 };
    int w = ((s->lcdfrmcfg >> 21) & 0x7ff) + 1;   /* HOZVAL  + 1 */
    int h = (s->lcdfrmcfg & 0x7ff) + 1;           /* LINEVAL + 1 */

    if (w <= 1 || h <= 1) {
        w = LCDC_WIDTH;
        h = LCDC_HEIGHT;
    }
    *width = w;
    *height = h;
    *bpp = pixelsize[(s->lcdcon2 >> 5) & 0x7];
}

static bool lcdc_enabled(AT91LcdcState *s)
{
    return (s->pwrcon & LCDC_PWRCON_PWR) && s->dmabaddr1 != 0;
}

/* Convert one scanline of guest pixels (any supported LCDC depth) into the
 * console's xRGB surface.  The straight 32bpp path assumes guest xRGB == host
 * surface format (true on the LE hosts this board runs on). */
static void lcdc_convert_line(uint32_t *dst, const uint8_t *src, int w, int bpp)
{
    int x;

    switch (bpp) {
    case 32:
        memcpy(dst, src, (size_t)w * 4);
        break;
    case 24:
        for (x = 0; x < w; x++) {
            uint8_t b = src[3 * x], g = src[3 * x + 1], r = src[3 * x + 2];
            dst[x] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        }
        break;
    case 16:                                     /* RGB565 */
        for (x = 0; x < w; x++) {
            uint16_t v = ((const uint16_t *)src)[x];
            uint8_t r = (v >> 11) & 0x1f, g = (v >> 5) & 0x3f, b = v & 0x1f;
            dst[x] = ((uint32_t)((r << 3) | (r >> 2)) << 16) |
                     ((uint32_t)((g << 2) | (g >> 4)) << 8) |
                     ((b << 3) | (b >> 2));
        }
        break;
    default:                                     /* 1/2/4/8bpp: not modelled */
        memset(dst, 0, (size_t)w * 4);
        break;
    }
}

static bool lcdc_gfx_update(void *opaque)
{
    AT91LcdcState *s = AT91_LCDC(opaque);
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t src[2048 * 4];
    uint32_t *dst;
    int w, h, bpp, y, stride;

    if (!lcdc_enabled(s)) {
        return true;
    }
    lcdc_get_mode(s, &w, &h, &bpp);
    if (w > 2048) {
        w = 2048;
    }
    stride = w * bpp / 8;

    if (surface_width(surface) != w || surface_height(surface) != h) {
        trace_at91_lcdc_mode(w, h, bpp);
        qemu_console_resize(s->con, w, h);
        surface = qemu_console_surface(s->con);
        s->invalidate = true;
    }

    dst = surface_data(surface);
    for (y = 0; y < h; y++) {
        hwaddr line = s->dmabaddr1 + (hwaddr)y * stride;
        address_space_read(&address_space_memory, line, MEMTXATTRS_UNSPECIFIED,
                           src, stride);
        lcdc_convert_line(dst + (size_t)y * w, src, w, bpp);
    }
    qemu_console_update(s->con, 0, 0, w, h);
    s->invalidate = false;
    return true;
}

static void lcdc_invalidate(void *opaque)
{
    AT91LcdcState *s = AT91_LCDC(opaque);
    s->invalidate = true;
}

static const GraphicHwOps lcdc_gfx_ops = {
    .invalidate = lcdc_invalidate,
    .gfx_update = lcdc_gfx_update,
};

static uint64_t lcdc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91LcdcState *s = AT91_LCDC(opaque);

    switch (offset) {
    case LCDC_DMABADDR1: return s->dmabaddr1;
    case LCDC_DMAFRMCFG: return s->dmafrmcfg;
    case LCDC_DMACON:    return s->dmacon;
    case LCDC_LCDCON1:   return s->lcdcon1;
    case LCDC_LCDCON2:   return s->lcdcon2;
    case LCDC_LCDFRMCFG: return s->lcdfrmcfg;
    case LCDC_PWRCON:    return s->pwrcon;
    case LCDC_IMR:       return s->imr;
    case LCDC_ISR:       return 0;
    default:
        return 0;
    }
}

static void lcdc_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AT91LcdcState *s = AT91_LCDC(opaque);
    uint32_t val = value;

    switch (offset) {
    case LCDC_DMABADDR1: s->dmabaddr1 = val; s->invalidate = true; break;
    case LCDC_DMAFRMCFG: s->dmafrmcfg = val; break;
    case LCDC_DMACON:    s->dmacon = val; break;
    case LCDC_LCDCON1:   s->lcdcon1 = val; break;
    case LCDC_LCDCON2:   s->lcdcon2 = val; s->invalidate = true; break;
    case LCDC_LCDFRMCFG: s->lcdfrmcfg = val; s->invalidate = true; break;
    case LCDC_PWRCON:    s->pwrcon = val; break;
    case LCDC_IER:       s->imr |= val; break;
    case LCDC_IDR:       s->imr &= ~val; break;
    case LCDC_ICR:       break;
    default:
        break;
    }
}

static const MemoryRegionOps lcdc_ops = {
    .read = lcdc_read,
    .write = lcdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void lcdc_reset(DeviceState *dev)
{
    AT91LcdcState *s = AT91_LCDC(dev);

    s->dmabaddr1 = s->dmafrmcfg = s->dmacon = 0;
    s->lcdcon1 = s->lcdcon2 = s->lcdfrmcfg = s->pwrcon = s->imr = 0;
    s->invalidate = true;
}

static void lcdc_realize(DeviceState *dev, Error **errp)
{
    AT91LcdcState *s = AT91_LCDC(dev);

    s->con = qemu_graphic_console_create(dev, 0, &lcdc_gfx_ops, s);
    qemu_console_resize(s->con, LCDC_WIDTH, LCDC_HEIGHT);
}

static void lcdc_dev_init(Object *obj)
{
    AT91LcdcState *s = AT91_LCDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &lcdc_ops, s, "at91-lcdc", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_at91_lcdc = {
    .name = "at91-lcdc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dmabaddr1, AT91LcdcState),
        VMSTATE_UINT32(dmafrmcfg, AT91LcdcState),
        VMSTATE_UINT32(dmacon, AT91LcdcState),
        VMSTATE_UINT32(lcdcon1, AT91LcdcState),
        VMSTATE_UINT32(lcdcon2, AT91LcdcState),
        VMSTATE_UINT32(lcdfrmcfg, AT91LcdcState),
        VMSTATE_UINT32(pwrcon, AT91LcdcState),
        VMSTATE_UINT32(imr, AT91LcdcState),
        VMSTATE_END_OF_LIST()
    }
};

static void lcdc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = lcdc_realize;
    device_class_set_legacy_reset(dc, lcdc_reset);
    dc->vmsd = &vmstate_at91_lcdc;
}

static const TypeInfo lcdc_type = {
    .name = TYPE_AT91_LCDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91LcdcState),
    .instance_init = lcdc_dev_init,
    .class_init = lcdc_class_init,
};

static void at91_lcdc_register_types(void)
{
    type_register_static(&lcdc_type);
}

type_init(at91_lcdc_register_types)
