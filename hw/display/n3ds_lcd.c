/*
 * Nintendo 3DS display path: LCD registers (0x10202000), the GPU
 * "external" register block (0x10400000) with the two PDC framebuffer
 * controllers and the PSC memory-fill engines, and a QEMU console that
 * shows both screens (top 400x240 above bottom 320x240).
 *
 * The 3DS framebuffers are column-major: each column of 240 pixels is
 * contiguous, starting with the bottom row.
 *
 * Documentation: 3dbrew GPU/External_Registers, LCD_Registers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "ui/surface.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "hw/misc/n3ds.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(N3DSLcdState, N3DS_LCD)

#define GPU_EXT_SIZE    0x2000
#define PSC0_BASE       0x10
#define PSC1_BASE       0x20
#define PDC0_BASE       0x400
#define PDC1_BASE       0x500
#define PDC_SIZE        0x100

#define PDC_FB_A0       0x68
#define PDC_FB_A1       0x6C
#define PDC_FORMAT      0x70
#define PDC_SELECT      0x78
#define PDC_STRIDE      0x90
#define PDC_FB_B0       0x94
#define PDC_FB_B1       0x98

#define TOP_WIDTH       400
#define BOT_WIDTH       320
#define SCREEN_HEIGHT   240
#define CON_WIDTH       400
#define CON_HEIGHT      480

struct N3DSLcdState {
    SysBusDevice parent_obj;

    MemoryRegion lcd_iomem;
    MemoryRegion gpu_iomem;
    QemuConsole *con;

    uint32_t lcd_regs[0x1000 / 4];
    uint32_t gpu_regs[GPU_EXT_SIZE / 4];
    uint8_t linebuf[TOP_WIDTH * 4 * SCREEN_HEIGHT];
};

/* ---- LCD registers: plain storage ---- */

static uint64_t lcd_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSLcdState *s = opaque;
    return s->lcd_regs[off >> 2];
}

static void lcd_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSLcdState *s = opaque;
    s->lcd_regs[off >> 2] = v;
}

static const MemoryRegionOps lcd_ops = {
    .read = lcd_read,
    .write = lcd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ---- GPU external registers ---- */

static void psc_fill(N3DSLcdState *s, unsigned base)
{
    uint32_t *r = &s->gpu_regs[base >> 2];
    hwaddr start = (hwaddr)r[0] << 3;
    hwaddr end = (hwaddr)r[1] << 3;
    uint32_t value = r[2];
    unsigned width = (r[3] >> 8) & 3;  /* 0: 16-bit, 1: 24-bit, 2: 32-bit */
    unsigned bytes = width == 0 ? 2 : width == 1 ? 3 : 4;
    uint8_t buf[4096];
    unsigned i;

    for (i = 0; i < sizeof(buf); i += bytes) {
        memcpy(&buf[i], &value, bytes);
    }
    while (start < end) {
        unsigned chunk = MIN(end - start, sizeof(buf) - sizeof(buf) % bytes);
        address_space_write(&address_space_memory, start,
                            MEMTXATTRS_UNSPECIFIED, buf, chunk);
        start += chunk;
    }
    r[3] = (r[3] & ~1u) | 2;  /* done */
}

static uint64_t gpu_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSLcdState *s = opaque;
    return s->gpu_regs[off >> 2];
}

static void gpu_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSLcdState *s = opaque;

    s->gpu_regs[off >> 2] = v;
    if ((off == PSC0_BASE + 0xC || off == PSC1_BASE + 0xC) && (v & 1)) {
        psc_fill(s, off - 0xC);
    }
}

static const MemoryRegionOps gpu_ops = {
    .read = gpu_read,
    .write = gpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ---- display ---- */

static inline uint32_t pixel_from_fb(const uint8_t *p, unsigned fmt)
{
    uint16_t v16;

    switch (fmt) {
    case 0: /* RGBA8: bytes A, B, G, R */
        return (p[3] << 16) | (p[2] << 8) | p[1];
    case 1: /* RGB8: bytes B, G, R */
        return (p[2] << 16) | (p[1] << 8) | p[0];
    case 2: /* RGB565 */
        v16 = p[0] | (p[1] << 8);
        return ((v16 >> 11) << 19) | (((v16 >> 5) & 0x3F) << 10) |
               ((v16 & 0x1F) << 3);
    case 3: /* RGB5A1 */
        v16 = p[0] | (p[1] << 8);
        return ((v16 >> 11) << 19) | (((v16 >> 6) & 0x1F) << 11) |
               (((v16 >> 1) & 0x1F) << 3);
    default: /* RGBA4 */
        v16 = p[0] | (p[1] << 8);
        return ((v16 >> 12) << 20) | (((v16 >> 8) & 0xF) << 12) |
               (((v16 >> 4) & 0xF) << 4);
    }
}

static void draw_screen(N3DSLcdState *s, DisplaySurface *surface,
                        unsigned pdc, unsigned width, int xoff, int yoff)
{
    uint32_t *r = &s->gpu_regs[pdc >> 2];
    unsigned fmt = r[PDC_FORMAT >> 2] & 7;
    unsigned bpp = fmt == 0 ? 4 : fmt == 1 ? 3 : 2;
    unsigned stride = r[PDC_STRIDE >> 2];
    hwaddr fb = (r[PDC_SELECT >> 2] & 1) ? r[PDC_FB_A1 >> 2] : r[PDC_FB_A0 >> 2];
    uint8_t *dst = surface_data(surface);
    int dst_stride = surface_stride(surface);
    unsigned x, y;

    if (stride < bpp * SCREEN_HEIGHT || stride > TOP_WIDTH * 4 || fb == 0) {
        for (y = 0; y < SCREEN_HEIGHT; y++) {
            memset(dst + (yoff + y) * dst_stride + xoff * 4, 0, width * 4);
        }
        return;
    }

    for (x = 0; x < width; x++) {
        const uint8_t *col = s->linebuf + x * stride;
        uint32_t *line;

        address_space_read(&address_space_memory, fb + x * stride,
                           MEMTXATTRS_UNSPECIFIED, s->linebuf + x * stride,
                           stride);
        for (y = 0; y < SCREEN_HEIGHT; y++) {
            line = (uint32_t *)(dst + (yoff + y) * dst_stride) + xoff + x;
            *line = pixel_from_fb(col + (SCREEN_HEIGHT - 1 - y) * bpp, fmt);
        }
    }
}

static bool n3ds_lcd_gfx_update(void *opaque)
{
    N3DSLcdState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);

    if (!surface || surface_width(surface) != CON_WIDTH ||
        surface_height(surface) != CON_HEIGHT ||
        surface_bits_per_pixel(surface) != 32) {
        return true;
    }
    draw_screen(s, surface, PDC0_BASE, TOP_WIDTH, 0, 0);
    draw_screen(s, surface, PDC1_BASE, BOT_WIDTH, (TOP_WIDTH - BOT_WIDTH) / 2,
                SCREEN_HEIGHT);
    qemu_console_update(s->con, 0, 0, CON_WIDTH, CON_HEIGHT);
    return true;
}

static void n3ds_lcd_invalidate(void *opaque)
{
}

static const GraphicHwOps n3ds_lcd_gfx_ops = {
    .invalidate = n3ds_lcd_invalidate,
    .gfx_update = n3ds_lcd_gfx_update,
};

static void n3ds_lcd_reset(DeviceState *dev)
{
    N3DSLcdState *s = N3DS_LCD(dev);

    memset(s->lcd_regs, 0, sizeof(s->lcd_regs));
    memset(s->gpu_regs, 0, sizeof(s->gpu_regs));
    /* Framebuffer setup as left behind by the Luma3DS chainloader. */
    s->gpu_regs[(PDC0_BASE + PDC_FB_A0) >> 2] = 0x18300000;
    s->gpu_regs[(PDC0_BASE + PDC_FB_A1) >> 2] = 0x18400000;
    s->gpu_regs[(PDC0_BASE + PDC_FB_B0) >> 2] = 0x18300000;
    s->gpu_regs[(PDC0_BASE + PDC_FB_B1) >> 2] = 0x18400000;
    s->gpu_regs[(PDC0_BASE + PDC_FORMAT) >> 2] = 0x80341;
    s->gpu_regs[(PDC0_BASE + PDC_STRIDE) >> 2] = 0x2D0;
    s->gpu_regs[(PDC1_BASE + PDC_FB_A0) >> 2] = 0x18346500;
    s->gpu_regs[(PDC1_BASE + PDC_FB_A1) >> 2] = 0x18446500;
    s->gpu_regs[(PDC1_BASE + PDC_FORMAT) >> 2] = 0x80301;
    s->gpu_regs[(PDC1_BASE + PDC_STRIDE) >> 2] = 0x2D0;
}

static void n3ds_lcd_realize(DeviceState *dev, Error **errp)
{
    N3DSLcdState *s = N3DS_LCD(dev);

    s->con = qemu_graphic_console_create(dev, 0, &n3ds_lcd_gfx_ops, s);
    qemu_console_resize(s->con, CON_WIDTH, CON_HEIGHT);
}

static void n3ds_lcd_init(Object *obj)
{
    N3DSLcdState *s = N3DS_LCD(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->lcd_iomem, obj, &lcd_ops, s, "n3ds.lcd", 0x1000);
    memory_region_init_io(&s->gpu_iomem, obj, &gpu_ops, s, "n3ds.gpu-ext",
                          GPU_EXT_SIZE);
    sysbus_init_mmio(sbd, &s->lcd_iomem);
    sysbus_init_mmio(sbd, &s->gpu_iomem);
}

static void n3ds_lcd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = n3ds_lcd_realize;
    device_class_set_legacy_reset(dc, n3ds_lcd_reset);
}

static const TypeInfo n3ds_lcd_info = {
    .name          = TYPE_N3DS_LCD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSLcdState),
    .instance_init = n3ds_lcd_init,
    .class_init    = n3ds_lcd_class_init,
};

static void n3ds_lcd_register_types(void)
{
    type_register_static(&n3ds_lcd_info);
}

type_init(n3ds_lcd_register_types)
