/*
 * Atmel/Microchip AT91 LCD Controller (atmel_lcdfb).
 *
 * Framebuffer scan-out: the base DMA address (DMABADDR1) points at a frame in
 * guest RAM; geometry comes from LCDFRMCFG and depth from LCDCON2 PIXELSIZE,
 * converted (32/24/16bpp or 1/2/4/8bpp through the RGB565 LUT) into the
 * console's xRGB surface.  A virtual-time frame timer generates DMA EOF.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
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
#define LCDC_TIM1       0x0808
#define LCDC_TIM2       0x080C
#define LCDC_LCDFRMCFG  0x0810
#define LCDC_PWRCON     0x083C
#define LCDC_IER        0x0848
#define LCDC_IDR        0x084C
#define LCDC_IMR        0x0850
#define LCDC_ISR        0x0854
#define LCDC_ICR        0x0858
#define LCDC_ITR        0x0860
#define LCDC_IRR        0x0864
#define LCDC_LUT_BASE   0x0C00
#define LCDC_LUT_COUNT  256

#define LCDC_DMACON_DMAEN (1u << 0)
#define LCDC_LCDCON1_BYPASS (1u << 0)
#define LCDC_LCDCON1_CLKVAL(v) (((v) >> 12) & 0x1ff)
#define LCDC_LCDCON2_LITTLE (1u << 31)
#define LCDC_PWRCON_PWR (1u << 0)   /* LCD module power on */
#define LCDC_IRQ_EOF    (1u << 2)
#define LCDC_IRQ_MASK   0x77

#define LCDC_WIDTH   480
#define LCDC_HEIGHT  272

struct AT91LcdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QemuConsole *con;
    qemu_irq irq;
    Clock *mck;
    QEMUTimer *frame_timer;

    uint32_t dmabaddr1;
    uint32_t dmafrmcfg;
    uint32_t dmacon;
    uint32_t lcdcon1, lcdcon2, tim1, tim2;
    uint32_t lcdfrmcfg;
    uint32_t pwrcon;
    uint32_t imr;
    uint32_t isr;
    uint16_t lut[LCDC_LUT_COUNT];
    int64_t migration_remaining_ns;
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
    return (s->pwrcon & LCDC_PWRCON_PWR) &&
           (s->dmacon & LCDC_DMACON_DMAEN) && s->dmabaddr1 != 0;
}

static uint64_t lcdc_frame_period_ns(AT91LcdcState *s)
{
    uint64_t mck = clock_get_hz(s->mck);
    uint64_t pixel_hz;
    uint64_t htotal, vtotal;
    uint64_t period;
    int w, h, bpp;

    lcdc_get_mode(s, &w, &h, &bpp);
    htotal = w + ((s->tim1 >> 24) & 0xf) +
             (s->tim2 & 0xff) + 1 +
             ((s->tim2 >> 8) & 0x3f) + 1 +
             ((s->tim2 >> 21) & 0x7ff) + 1;
    vtotal = h + (s->tim1 & 0xff) + ((s->tim1 >> 8) & 0xff) +
             ((s->tim1 >> 16) & 0x3f) + 1;
    pixel_hz = (s->lcdcon1 & LCDC_LCDCON1_BYPASS) ? mck :
               mck / (LCDC_LCDCON1_CLKVAL(s->lcdcon1) + 1);
    if (pixel_hz == 0) {
        return NANOSECONDS_PER_SECOND / 60;
    }
    period = muldiv64(htotal * vtotal, NANOSECONDS_PER_SECOND, pixel_hz);
    return MAX(period, UINT64_C(1));
}

static void lcdc_update_irq(AT91LcdcState *s)
{
    qemu_set_irq(s->irq, (s->isr & s->imr) != 0);
}

static void lcdc_rearm_frame(AT91LcdcState *s)
{
    if (s->frame_timer && lcdc_enabled(s)) {
        timer_mod(s->frame_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  lcdc_frame_period_ns(s));
    } else if (s->frame_timer) {
        timer_del(s->frame_timer);
    }
}

static void lcdc_frame_done(void *opaque)
{
    AT91LcdcState *s = opaque;

    s->isr |= LCDC_IRQ_EOF;
    trace_at91_lcdc_eof(s->isr, s->imr);
    lcdc_update_irq(s);
    lcdc_rearm_frame(s);
}

static void lcdc_clock_update(void *opaque, ClockEvent event)
{
    AT91LcdcState *s = opaque;

    if (event == ClockUpdate) {
        lcdc_rearm_frame(s);
    }
}

static uint32_t lcdc_rgb565(uint16_t value)
{
    uint8_t r = (value >> 11) & 0x1f;
    uint8_t g = (value >> 5) & 0x3f;
    uint8_t b = value & 0x1f;

    return ((uint32_t)((r << 3) | (r >> 2)) << 16) |
           ((uint32_t)((g << 2) | (g >> 4)) << 8) |
           ((b << 3) | (b >> 2));
}

static unsigned lcdc_indexed_pixel(AT91LcdcState *s, const uint8_t *src,
                                   int x, int bpp)
{
    unsigned mask = (1u << bpp) - 1;

    if (s->lcdcon2 & LCDC_LCDCON2_LITTLE) {
        unsigned bit = x * bpp;

        return (src[bit / 8] >> (bit % 8)) & mask;
    } else {
        unsigned pixels_per_word = 32 / bpp;
        unsigned shift = 32 - bpp - (x % pixels_per_word) * bpp;
        uint32_t word = ldl_le_p(src + (x / pixels_per_word) * 4);

        return (word >> shift) & mask;
    }
}

/* Convert one scanline of guest pixels (any supported LCDC depth) into the
 * console's xRGB surface.  The straight 32bpp path assumes guest xRGB == host
 * surface format (true on the LE hosts this board runs on). */
static void lcdc_convert_line(AT91LcdcState *s, uint32_t *dst,
                              const uint8_t *src, int w, int bpp)
{
    int x;

    switch (bpp) {
    case 32:
        for (x = 0; x < w; x++) {
            dst[x] = ldl_le_p(src + 4 * x);
        }
        break;
    case 24:
        for (x = 0; x < w; x++) {
            uint8_t b = src[3 * x], g = src[3 * x + 1], r = src[3 * x + 2];
            dst[x] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        }
        break;
    case 16:                                     /* RGB565 */
        for (x = 0; x < w; x++) {
            dst[x] = lcdc_rgb565(lduw_le_p(src + 2 * x));
        }
        break;
    default:                                     /* 1/2/4/8bpp through LUT */
        for (x = 0; x < w; x++) {
            unsigned index = lcdc_indexed_pixel(s, src, x, bpp);

            dst[x] = lcdc_rgb565(s->lut[index]);
        }
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
    stride = DIV_ROUND_UP(w * bpp, 32) * 4;

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
        lcdc_convert_line(s, dst + (size_t)y * w, src, w, bpp);
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

    if (offset >= LCDC_LUT_BASE &&
        offset < LCDC_LUT_BASE + LCDC_LUT_COUNT * 4) {
        return s->lut[(offset - LCDC_LUT_BASE) / 4];
    }

    switch (offset) {
    case LCDC_DMABADDR1: return s->dmabaddr1;
    case LCDC_DMAFRMCFG: return s->dmafrmcfg;
    case LCDC_DMACON:    return s->dmacon;
    case LCDC_LCDCON1:   return s->lcdcon1;
    case LCDC_LCDCON2:   return s->lcdcon2;
    case LCDC_TIM1:      return s->tim1;
    case LCDC_TIM2:      return s->tim2;
    case LCDC_LCDFRMCFG: return s->lcdfrmcfg;
    case LCDC_PWRCON:    return s->pwrcon;
    case LCDC_IMR:       return s->imr;
    case LCDC_ISR:
    case LCDC_IRR:       return s->isr;
    default:
        return 0;
    }
}

static void lcdc_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AT91LcdcState *s = AT91_LCDC(opaque);
    uint32_t val = value;

    if (offset >= LCDC_LUT_BASE &&
        offset < LCDC_LUT_BASE + LCDC_LUT_COUNT * 4) {
        s->lut[(offset - LCDC_LUT_BASE) / 4] = val;
        s->invalidate = true;
        return;
    }

    switch (offset) {
    case LCDC_DMABADDR1:
        s->dmabaddr1 = val;
        s->invalidate = true;
        lcdc_rearm_frame(s);
        break;
    case LCDC_DMAFRMCFG: s->dmafrmcfg = val; break;
    case LCDC_DMACON:
        s->dmacon = val;
        lcdc_rearm_frame(s);
        break;
    case LCDC_LCDCON1:
        s->lcdcon1 = val;
        lcdc_rearm_frame(s);
        break;
    case LCDC_LCDCON2:
        s->lcdcon2 = val;
        s->invalidate = true;
        lcdc_rearm_frame(s);
        break;
    case LCDC_TIM1:
        s->tim1 = val;
        lcdc_rearm_frame(s);
        break;
    case LCDC_TIM2:
        s->tim2 = val;
        lcdc_rearm_frame(s);
        break;
    case LCDC_LCDFRMCFG:
        s->lcdfrmcfg = val;
        s->invalidate = true;
        lcdc_rearm_frame(s);
        break;
    case LCDC_PWRCON:
        s->pwrcon = val;
        lcdc_rearm_frame(s);
        break;
    case LCDC_IER:
        s->imr |= val & LCDC_IRQ_MASK;
        lcdc_update_irq(s);
        break;
    case LCDC_IDR:
        s->imr &= ~(val & LCDC_IRQ_MASK);
        lcdc_update_irq(s);
        break;
    case LCDC_ICR:
        s->isr &= ~(val & LCDC_IRQ_MASK);
        lcdc_update_irq(s);
        break;
    case LCDC_ITR:
        s->isr |= val & LCDC_IRQ_MASK;
        lcdc_update_irq(s);
        break;
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
    s->lcdcon1 = s->lcdcon2 = s->tim1 = s->tim2 = 0;
    s->lcdfrmcfg = s->pwrcon = s->imr = s->isr = 0;
    memset(s->lut, 0, sizeof(s->lut));
    s->migration_remaining_ns = -1;
    s->invalidate = true;
    if (s->frame_timer) {
        timer_del(s->frame_timer);
    }
    lcdc_update_irq(s);
}

static void lcdc_realize(DeviceState *dev, Error **errp)
{
    AT91LcdcState *s = AT91_LCDC(dev);

    if (!clock_has_source(s->mck)) {
        error_setg(errp, "at91-lcdc: mck input must be connected");
        return;
    }
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lcdc_frame_done, s);
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
    s->mck = qdev_init_clock_in(DEVICE(obj), "mck", lcdc_clock_update, s,
                                ClockUpdate);
}

static int lcdc_pre_save(void *opaque)
{
    AT91LcdcState *s = opaque;

    if (s->frame_timer && timer_pending(s->frame_timer)) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        s->migration_remaining_ns =
            MAX(timer_expire_time_ns(s->frame_timer) - now, INT64_C(0));
    } else {
        s->migration_remaining_ns = -1;
    }
    return 0;
}

static int lcdc_post_load(void *opaque, int version_id)
{
    AT91LcdcState *s = opaque;

    s->invalidate = true;
    lcdc_update_irq(s);
    if (lcdc_enabled(s) && s->migration_remaining_ns >= 0) {
        timer_mod(s->frame_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  s->migration_remaining_ns);
    } else {
        lcdc_rearm_frame(s);
    }
    s->migration_remaining_ns = -1;
    return 0;
}

static const VMStateDescription vmstate_at91_lcdc = {
    .name = "at91-lcdc",
    .version_id = 2,
    .minimum_version_id = 1,
    .pre_save = lcdc_pre_save,
    .post_load = lcdc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dmabaddr1, AT91LcdcState),
        VMSTATE_UINT32(dmafrmcfg, AT91LcdcState),
        VMSTATE_UINT32(dmacon, AT91LcdcState),
        VMSTATE_UINT32(lcdcon1, AT91LcdcState),
        VMSTATE_UINT32(lcdcon2, AT91LcdcState),
        VMSTATE_UINT32(lcdfrmcfg, AT91LcdcState),
        VMSTATE_UINT32(pwrcon, AT91LcdcState),
        VMSTATE_UINT32(imr, AT91LcdcState),
        VMSTATE_UINT32_V(tim1, AT91LcdcState, 2),
        VMSTATE_UINT32_V(tim2, AT91LcdcState, 2),
        VMSTATE_UINT32_V(isr, AT91LcdcState, 2),
        VMSTATE_UINT16_ARRAY_V(lut, AT91LcdcState, LCDC_LUT_COUNT, 2),
        VMSTATE_INT64_V(migration_remaining_ns, AT91LcdcState, 2),
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
