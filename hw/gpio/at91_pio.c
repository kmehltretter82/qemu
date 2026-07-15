/*
 * Atmel/Microchip AT91 Parallel I/O Controller (PIO / GPIO).
 *
 * One instance per bank (PIOA..PIOE).  Exposes 32 gpio-in + 32 gpio-out so the
 * board can wire real signals (SD card-detect, LEDs, buttons).  Models
 * direction, output data, pin data status (with pull-ups) and any-edge
 * input-change interrupts.  Peripheral-mux/filter/drive registers are stored
 * but otherwise inert.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/gpio/at91_pio.h"

#define PIO_PER   0x00
#define PIO_PDR   0x04
#define PIO_PSR   0x08
#define PIO_OER   0x10
#define PIO_ODR   0x14
#define PIO_OSR   0x18
#define PIO_IFER  0x20
#define PIO_IFDR  0x24
#define PIO_IFSR  0x28
#define PIO_SODR  0x30
#define PIO_CODR  0x34
#define PIO_ODSR  0x38
#define PIO_PDSR  0x3C
#define PIO_IER   0x40
#define PIO_IDR   0x44
#define PIO_IMR   0x48
#define PIO_ISR   0x4C
#define PIO_MDER  0x50
#define PIO_MDDR  0x54
#define PIO_MDSR  0x58
#define PIO_PUDR  0x60
#define PIO_PUER  0x64
#define PIO_PUSR  0x68
#define PIO_ASR   0x70
#define PIO_BSR   0x74
#define PIO_ABSR  0x78
#define PIO_OWER  0xA0
#define PIO_OWDR  0xA4
#define PIO_OWSR  0xA8

struct AT91PioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;              /* to AIC */
    qemu_irq out[32];          /* output pins */

    uint32_t psr;              /* 1 = PIO controls the pin (vs peripheral) */
    uint32_t osr;              /* 1 = output */
    uint32_t odsr;             /* output data */
    uint32_t ifsr;             /* input glitch filter */
    uint32_t mdsr;             /* multi-driver (open-drain) */
    uint32_t pull_up;          /* 1 = pull-up enabled (PUSR is inverted) */
    uint32_t absr;             /* peripheral A/B select */
    uint32_t owsr;             /* ODSR direct-write mask */
    uint32_t imr;              /* interrupt mask */
    uint32_t isr;              /* interrupt status (latched, read-clears) */

    uint32_t pin_in;           /* externally-driven input levels */
    uint32_t pin_driven;       /* which inputs have an external driver */
    uint32_t last_pdsr;

    /* Board-configured input levels re-applied on every reset (e.g. a static
     * SD card-detect line), so they survive reset regardless of ordering. */
    uint32_t reset_driven;
    uint32_t reset_level;
};

/* Set an input pin's post-reset level from the board (before machine reset). */
void pio_set_reset_input(DeviceState *dev, int pin, bool level)
{
    AT91PioState *s = AT91_PIO(dev);

    s->reset_driven |= 1u << pin;
    if (level) {
        s->reset_level |= 1u << pin;
    } else {
        s->reset_level &= ~(1u << pin);
    }
}

/* Actual pin levels: output pins show ODSR; input pins show their driver, or
 * the pull-up when floating. */
static uint32_t pio_pdsr(AT91PioState *s)
{
    uint32_t in_level = (s->pin_driven & s->pin_in) |
                        (~s->pin_driven & s->pull_up);
    return (s->osr & s->odsr) | (~s->osr & in_level);
}

static void pio_update(AT91PioState *s)
{
    uint32_t pdsr = pio_pdsr(s);
    uint32_t changed = pdsr ^ s->last_pdsr;
    int n;

    /* Any edge on any pin latches into ISR; masked pins assert the IRQ. */
    s->isr |= changed;
    s->last_pdsr = pdsr;

    for (n = 0; n < 32; n++) {
        qemu_set_irq(s->out[n], (s->osr & s->odsr) >> n & 1);
    }
    qemu_set_irq(s->irq, (s->isr & s->imr) ? 1 : 0);
}

static void pio_set_input(void *opaque, int n, int level)
{
    AT91PioState *s = AT91_PIO(opaque);

    s->pin_driven |= 1u << n;
    if (level) {
        s->pin_in |= 1u << n;
    } else {
        s->pin_in &= ~(1u << n);
    }
    pio_update(s);
}

static uint64_t pio_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91PioState *s = AT91_PIO(opaque);
    uint32_t r = 0;

    switch (offset) {
    case PIO_PSR:  r = s->psr; break;
    case PIO_OSR:  r = s->osr; break;
    case PIO_IFSR: r = s->ifsr; break;
    case PIO_ODSR: r = s->odsr; break;
    case PIO_PDSR: r = pio_pdsr(s); break;
    case PIO_IMR:  r = s->imr; break;
    case PIO_ISR:                       /* read clears */
        r = s->isr;
        s->isr = 0;
        qemu_set_irq(s->irq, 0);
        break;
    case PIO_MDSR: r = s->mdsr; break;
    case PIO_PUSR: r = ~s->pull_up; break;   /* 0 = enabled */
    case PIO_ABSR: r = s->absr; break;
    case PIO_OWSR: r = s->owsr; break;
    default:
        break;
    }
    return r;
}

static void pio_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91PioState *s = AT91_PIO(opaque);
    uint32_t val = value;

    switch (offset) {
    case PIO_PER:  s->psr |= val; break;
    case PIO_PDR:  s->psr &= ~val; break;
    case PIO_OER:  s->osr |= val; pio_update(s); break;
    case PIO_ODR:  s->osr &= ~val; pio_update(s); break;
    case PIO_IFER: s->ifsr |= val; break;
    case PIO_IFDR: s->ifsr &= ~val; break;
    case PIO_SODR: s->odsr |= val; pio_update(s); break;
    case PIO_CODR: s->odsr &= ~val; pio_update(s); break;
    case PIO_ODSR:
        s->odsr = (s->odsr & ~s->owsr) | (val & s->owsr);
        pio_update(s);
        break;
    case PIO_IER:  s->imr |= val; pio_update(s); break;
    case PIO_IDR:  s->imr &= ~val; pio_update(s); break;
    case PIO_MDER: s->mdsr |= val; break;
    case PIO_MDDR: s->mdsr &= ~val; break;
    case PIO_PUER: s->pull_up |= val; pio_update(s); break;
    case PIO_PUDR: s->pull_up &= ~val; pio_update(s); break;
    case PIO_ASR:  s->absr &= ~val; break;
    case PIO_BSR:  s->absr |= val; break;
    case PIO_OWER: s->owsr |= val; break;
    case PIO_OWDR: s->owsr &= ~val; break;
    default:
        break;
    }
}

static const MemoryRegionOps pio_ops = {
    .read = pio_read,
    .write = pio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void pio_reset(DeviceState *dev)
{
    AT91PioState *s = AT91_PIO(dev);

    s->psr = 0xFFFFFFFF;      /* pins default to PIO control */
    s->osr = 0;               /* inputs */
    s->odsr = 0;
    s->ifsr = 0;
    s->mdsr = 0;
    s->pull_up = 0xFFFFFFFF;   /* pull-ups enabled at reset */
    s->absr = 0;
    s->owsr = 0;
    s->imr = 0;
    s->isr = 0;
    s->pin_driven = s->reset_driven;   /* board-static inputs (e.g. card-detect) */
    s->pin_in = s->reset_level;
    s->last_pdsr = pio_pdsr(s);
}

static void pio_dev_init(Object *obj)
{
    AT91PioState *s = AT91_PIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pio_ops, s, "at91-pio", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in(DEVICE(obj), pio_set_input, 32);
    qdev_init_gpio_out(DEVICE(obj), s->out, 32);
}

static const VMStateDescription vmstate_at91_pio = {
    .name = "at91-pio",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(psr, AT91PioState),
        VMSTATE_UINT32(osr, AT91PioState),
        VMSTATE_UINT32(odsr, AT91PioState),
        VMSTATE_UINT32(ifsr, AT91PioState),
        VMSTATE_UINT32(mdsr, AT91PioState),
        VMSTATE_UINT32(pull_up, AT91PioState),
        VMSTATE_UINT32(absr, AT91PioState),
        VMSTATE_UINT32(owsr, AT91PioState),
        VMSTATE_UINT32(imr, AT91PioState),
        VMSTATE_UINT32(isr, AT91PioState),
        VMSTATE_UINT32(pin_in, AT91PioState),
        VMSTATE_UINT32(pin_driven, AT91PioState),
        VMSTATE_UINT32(last_pdsr, AT91PioState),
        VMSTATE_UINT32(reset_driven, AT91PioState),
        VMSTATE_UINT32(reset_level, AT91PioState),
        VMSTATE_END_OF_LIST()
    }
};

static void pio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, pio_reset);
    dc->vmsd = &vmstate_at91_pio;
}

static const TypeInfo pio_type = {
    .name = TYPE_AT91_PIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91PioState),
    .instance_init = pio_dev_init,
    .class_init = pio_class_init,
};

static void at91_pio_register_types(void)
{
    type_register_static(&pio_type);
}

type_init(at91_pio_register_types)
