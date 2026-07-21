/*
 * Acorn IOMD (I/O, Memory and DMA controller) of the RiscPC.
 *
 * Models the interrupt controller (banks A/B, DMA requests, FIQ),
 * the two 2 MHz IOC-style 16-bit down-counting timers, the KART
 * PS/2 keyboard port, the quadrature mouse counters and the
 * memory/timing control registers (storage only). DMA channels
 * other than video are not implemented.
 *
 * Register layout per Linux's arch/arm/include/asm/hardware/iomd.h
 * and arch/arm/mach-rpc/{irq,time}.c usage:
 *  - every bank has STAT (+0, raw), REQ (+4, raw & mask; writing
 *    clears latched bits in bank A) and MASK (+8)
 *  - bank A (0x010): latched sources - timers on bits 5/6, POR bit 4
 *  - bank B (0x020), DMA (0x1f0), FIQ (0x030): level sources
 *  - timers: write LTCHL/LTCHH to set the reload value, write GO to
 *    (re)start, write LATCH to capture the count for CNTL/CNTH reads
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/ptimer.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/misc/acorn-iomd.h"
#include "hw/input/ps2.h"
#include "ui/console.h"
#include "ui/input.h"

#define IOMD_CONTROL    0x000
#define IOMD_KARTTX     0x004
#define IOMD_KARTRX     0x004   /* same address, read side */
#define IOMD_KCTRL      0x008

#define IOMD_IRQSTATA   0x010
#define IOMD_IRQREQA    0x014   /* write: IRQCLRA */
#define IOMD_IRQMASKA   0x018
#define IOMD_IRQSTATB   0x020
#define IOMD_IRQREQB    0x024
#define IOMD_IRQMASKB   0x028
#define IOMD_FIQSTAT    0x030
#define IOMD_FIQREQ     0x034
#define IOMD_FIQMASK    0x038

#define IOMD_T0CNTL     0x040
#define IOMD_T0CNTH     0x044
#define IOMD_T0GO       0x048
#define IOMD_T0LATCH    0x04c
#define IOMD_T1CNTL     0x050
#define IOMD_T1CNTH     0x054
#define IOMD_T1GO       0x058
#define IOMD_T1LATCH    0x05c

#define IOMD_ID0        0x094
#define IOMD_ID1        0x098
#define IOMD_VERSION    0x09c

#define IOMD_MOUSEX     0x0a0
#define IOMD_MOUSEY     0x0a4

#define IOMD_DMASTAT    0x1f0
#define IOMD_DMAREQ     0x1f4
#define IOMD_DMAMASK    0x1f8

#define IOMD_TIMER_FREQ 2000000

/* Bank A latched source bits */
#define IRQA_POR        (1 << 4)
#define IRQA_TIMER0     (1 << 5)
#define IRQA_TIMER1     (1 << 6)

/*
 * KCTRL bits used by Linux rpckbd.c: it spins on TXEMPTY before writing
 * KARTTX, and drains KARTRX while RXFULL is set.  Bit 3 is written at
 * open() to enable the port.
 */
#define KCTRL_ENABLE    (1 << 3)
#define KCTRL_RXFULL    (1 << 5)
#define KCTRL_TXEMPTY   (1 << 7)

/*
 * Video DMA state for the VIDC20 model.  The video registers (0x1c0..0x1e0)
 * have no side effects, so they land in the generic backing store; this is
 * just a typed view of them.  VIDINIT is the live pointer - acornfb
 * rewrites it to pan (acornfb.c:442) - and VIDSTART the buffer base.
 */
bool acorn_iomd_video_dma(AcornIOMDState *s, hwaddr *base)
{
    uint32_t cr = s->regs[ACORN_IOMD_VIDCR >> 2];
    uint32_t init = s->regs[ACORN_IOMD_VIDINIT >> 2];
    uint32_t start = s->regs[ACORN_IOMD_VIDSTART >> 2];

    if (!(cr & ACORN_IOMD_VIDCR_ENABLE)) {
        return false;
    }

    /*
     * Bit 30 is the wrap flag acornfb sets via video_set_dma() when the
     * pan offset reaches the end of the buffer; it is not part of the
     * address.
     */
    init &= ~(1u << 30);

    *base = init ? init : start;
    return *base != 0;
}

static void iomd_apply_boot_video(AcornIOMDState *s)
{
    if (!s->boot_video) {
        return;
    }
    s->regs[ACORN_IOMD_VIDSTART >> 2] = s->boot_vidstart;
    s->regs[ACORN_IOMD_VIDINIT >> 2] = s->boot_vidstart;
    s->regs[ACORN_IOMD_VIDEND >> 2] = s->boot_vidend;
    s->regs[ACORN_IOMD_VIDCR >> 2] = ACORN_IOMD_VIDCR_ENABLE;
}

void acorn_iomd_set_video_dma(AcornIOMDState *s, hwaddr start, hwaddr end)
{
    s->boot_video = true;
    s->boot_vidstart = start;
    s->boot_vidend = end;
    iomd_apply_boot_video(s);
}

static void iomd_update(AcornIOMDState *s)
{
    int level = (s->irqa_latch & s->irqa_mask) ||
                (s->irqb_level & s->irqb_mask) ||
                (s->dma_level & s->dma_mask);

    qemu_set_irq(s->irq, level);
    qemu_set_irq(s->fiq, (s->fiq_level & s->fiq_mask) != 0);
}

/*
 * Input lines: 0..7 bank A (latched on rising edge), 8..15 bank B,
 * 16..21 DMA requests, 24..31 FIQ bank.
 */
static void iomd_set_irq(void *opaque, int line, int level)
{
    AcornIOMDState *s = opaque;

    if (line < 8) {
        if (level) {
            s->irqa_latch |= 1 << line;
        }
    } else if (line < 16) {
        if (level) {
            s->irqb_level |= 1 << (line - 8);
        } else {
            s->irqb_level &= ~(1 << (line - 8));
        }
    } else if (line < 22) {
        if (level) {
            s->dma_level |= 1 << (line - 16);
        } else {
            s->dma_level &= ~(1 << (line - 16));
        }
    } else if (line >= 24 && line < 32) {
        if (level) {
            s->fiq_level |= 1 << (line - 24);
        } else {
            s->fiq_level &= ~(1 << (line - 24));
        }
    }
    iomd_update(s);
}

static void iomd_timer_tick(void *opaque)
{
    AcornIOMDTimer *t = opaque;
    AcornIOMDState *s = t->parent;

    s->irqa_latch |= t->index ? IRQA_TIMER1 : IRQA_TIMER0;
    iomd_update(s);
}

static uint64_t iomd_read(void *opaque, hwaddr offset, unsigned size)
{
    AcornIOMDState *s = opaque;

    switch (offset) {
    case IOMD_KARTRX:
        return ps2_read_data(PS2_DEVICE(&s->kbd));
    case IOMD_KCTRL:
        /*
         * Transmit is instantaneous here, so TXEMPTY is always set.
         * RXFULL tracks the ps2 queue: rpckbd_rx() loops on it.
         */
        return s->kctrl | KCTRL_TXEMPTY |
               (ps2_queue_empty(PS2_DEVICE(&s->kbd)) ? 0 : KCTRL_RXFULL);

    case IOMD_IRQSTATA:
        return s->irqa_latch;
    case IOMD_IRQREQA:
        return s->irqa_latch & s->irqa_mask;
    case IOMD_IRQMASKA:
        return s->irqa_mask;
    case IOMD_IRQSTATB:
        return s->irqb_level;
    case IOMD_IRQREQB:
        return s->irqb_level & s->irqb_mask;
    case IOMD_IRQMASKB:
        return s->irqb_mask;
    case IOMD_FIQSTAT:
        return s->fiq_level;
    case IOMD_FIQREQ:
        return s->fiq_level & s->fiq_mask;
    case IOMD_FIQMASK:
        return s->fiq_mask;
    case IOMD_DMASTAT:
        return s->dma_level;
    case IOMD_DMAREQ:
        return s->dma_level & s->dma_mask;
    case IOMD_DMAMASK:
        return s->dma_mask;

    case IOMD_T0CNTL:
        return s->timer[0].captured & 0xff;
    case IOMD_T0CNTH:
        return (s->timer[0].captured >> 8) & 0xff;
    case IOMD_T1CNTL:
        return s->timer[1].captured & 0xff;
    case IOMD_T1CNTH:
        return (s->timer[1].captured >> 8) & 0xff;

    case IOMD_ID0:
        return 0xe7;    /* IOMD id 0xd4e7 */
    case IOMD_ID1:
        return 0xd4;
    case IOMD_VERSION:
        return 0;

    case IOMD_MOUSEX:
        return (uint16_t)s->mouse_x;
    case IOMD_MOUSEY:
        return (uint16_t)s->mouse_y;

    default:
        if (offset < ACORN_IOMD_NUM_REGS * 4) {
            return s->regs[offset >> 2];
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x"HWADDR_FMT_plx"\n",
                      __func__, offset);
        return 0;
    }
}

static void iomd_timer_write(AcornIOMDState *s, int n, hwaddr reg,
                             uint32_t value)
{
    AcornIOMDTimer *t = &s->timer[n];

    switch (reg) {
    case IOMD_T0CNTL:   /* write: latch low byte */
        t->reload = (t->reload & 0xff00) | (value & 0xff);
        break;
    case IOMD_T0CNTH:   /* write: latch high byte */
        t->reload = (t->reload & 0x00ff) | ((value & 0xff) << 8);
        break;
    case IOMD_T0GO:
        ptimer_transaction_begin(t->timer);
        ptimer_stop(t->timer);
        ptimer_set_limit(t->timer, t->reload ? t->reload : 0x10000, 1);
        ptimer_run(t->timer, 0);
        ptimer_transaction_commit(t->timer);
        break;
    case IOMD_T0LATCH:
        t->captured = ptimer_get_count(t->timer);
        break;
    }
}

static void iomd_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AcornIOMDState *s = opaque;

    switch (offset) {
    case IOMD_KARTTX:
        ps2_write_keyboard(&s->kbd, value & 0xff);
        break;
    case IOMD_KCTRL:
        s->kctrl = value & KCTRL_ENABLE;
        break;

    case IOMD_IRQREQA:  /* IRQCLRA */
        s->irqa_latch &= ~value;
        break;
    case IOMD_IRQMASKA:
        s->irqa_mask = value & 0xff;
        break;
    case IOMD_IRQMASKB:
        s->irqb_mask = value & 0xff;
        break;
    case IOMD_FIQMASK:
        s->fiq_mask = value & 0xff;
        break;
    case IOMD_DMAMASK:
        s->dma_mask = value & 0xff;
        break;

    case IOMD_T0CNTL:
    case IOMD_T0CNTH:
    case IOMD_T0GO:
    case IOMD_T0LATCH:
        iomd_timer_write(s, 0, offset, value);
        return;
    case IOMD_T1CNTL:
    case IOMD_T1CNTH:
    case IOMD_T1GO:
    case IOMD_T1LATCH:
        iomd_timer_write(s, 1, offset - 0x10, value);
        return;

    default:
        if (offset < ACORN_IOMD_NUM_REGS * 4) {
            s->regs[offset >> 2] = value;
            return;
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x"HWADDR_FMT_plx"\n",
                      __func__, offset);
        return;
    }
    iomd_update(s);
}

static const MemoryRegionOps iomd_ops = {
    .read = iomd_read,
    .write = iomd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/*
 * Mouse buttons live in their own decode, not in the IOMD register file.
 * Linux rpcmouse reads IOMEM(0xe0310000) and XORs with 0x70, so the bits
 * are active low: left 0x40, middle 0x20, right 0x10.
 */
static uint64_t iomd_mouse_read(void *opaque, hwaddr offset, unsigned size)
{
    AcornIOMDState *s = opaque;

    return 0x70 & ~s->mouse_buttons;
}

static void iomd_mouse_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
}

static const MemoryRegionOps iomd_mouse_ops = {
    .read = iomd_mouse_read,
    .write = iomd_mouse_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/*
 * The quadrature counters free-run; the guest samples them on vsync and
 * differences the result itself, so there is no interrupt here and
 * wrapping is harmless as long as it wraps at 16 bits like the hardware.
 */
static void iomd_mouse_event(DeviceState *dev, QemuConsole *src,
                             QemuInputEvent *evt)
{
    AcornIOMDState *s = ACORN_IOMD(dev);
    static const uint8_t btn_bit[INPUT_BUTTON__MAX] = {
        [INPUT_BUTTON_LEFT]   = 0x40,
        [INPUT_BUTTON_MIDDLE] = 0x20,
        [INPUT_BUTTON_RIGHT]  = 0x10,
    };

    switch (evt->type) {
    case INPUT_EVENT_KIND_REL:
        if (evt->rel.axis == INPUT_AXIS_X) {
            s->mouse_x += evt->rel.value;
        } else {
            /* rpcmouse negates Y again on the way out */
            s->mouse_y -= evt->rel.value;
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (btn_bit[evt->btn.button]) {
            if (evt->btn.down) {
                s->mouse_buttons |= btn_bit[evt->btn.button];
            } else {
                s->mouse_buttons &= ~btn_bit[evt->btn.button];
            }
        }
        break;
    default:
        break;
    }
}

static const QemuInputHandler iomd_mouse_handler = {
    .name  = "Acorn Quadrature Mouse",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_REL,
    .event = iomd_mouse_event,
};

static void iomd_reset_hold(Object *obj, ResetType type)
{
    AcornIOMDState *s = ACORN_IOMD(obj);
    int i;

    s->irqa_latch = IRQA_POR;   /* power-on reset flag */
    s->irqa_mask = 0;
    s->irqb_level = 0;
    s->irqb_mask = 0;
    s->fiq_level = 0;
    s->fiq_mask = 0;
    s->dma_level = 0;
    s->dma_mask = 0;
    s->kctrl = 0;
    s->mouse_x = 0;
    s->mouse_y = 0;
    s->mouse_buttons = 0;
    memset(s->regs, 0, sizeof(s->regs));
    iomd_apply_boot_video(s);

    for (i = 0; i < ACORN_IOMD_NUM_TIMERS; i++) {
        AcornIOMDTimer *t = &s->timer[i];

        t->reload = 0;
        t->captured = 0;
        ptimer_transaction_begin(t->timer);
        ptimer_stop(t->timer);
        ptimer_transaction_commit(t->timer);
    }
    iomd_update(s);
}

static void iomd_init(Object *obj)
{
    AcornIOMDState *s = ACORN_IOMD(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &iomd_ops, s, "acorn-iomd", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    memory_region_init_io(&s->mouse_iomem, obj, &iomd_mouse_ops, s,
                          "acorn-iomd-mouse", 4);
    sysbus_init_mmio(sbd, &s->mouse_iomem);
    object_initialize_child(obj, "kbd", &s->kbd, TYPE_PS2_KBD_DEVICE);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(DEVICE(obj), iomd_set_irq, 32);

    for (i = 0; i < ACORN_IOMD_NUM_TIMERS; i++) {
        s->timer[i].parent = s;
        s->timer[i].index = i;
        s->timer[i].timer = ptimer_init(iomd_timer_tick, &s->timer[i],
                                        PTIMER_POLICY_LEGACY);
        ptimer_transaction_begin(s->timer[i].timer);
        ptimer_set_freq(s->timer[i].timer, IOMD_TIMER_FREQ);
        ptimer_transaction_commit(s->timer[i].timer);
    }
}

static void iomd_realize(DeviceState *dev, Error **errp)
{
    AcornIOMDState *s = ACORN_IOMD(dev);

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->kbd), errp)) {
        return;
    }
    /* The KART receive interrupt is bank B bit 7 (Linux IRQ_KEYBOARDRX). */
    qdev_connect_gpio_out(DEVICE(&s->kbd), PS2_DEVICE_IRQ,
                          qdev_get_gpio_in(dev, ACORN_IOMD_IRQ_KBDRX));

    s->mouse_handler = qemu_input_handler_register(dev,
                                                  &iomd_mouse_handler);
}

static const VMStateDescription vmstate_iomd_timer = {
    .name = "acorn-iomd-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(reload, AcornIOMDTimer),
        VMSTATE_UINT32(captured, AcornIOMDTimer),
        VMSTATE_PTIMER(timer, AcornIOMDTimer),
        VMSTATE_END_OF_LIST(),
    },
};

static const VMStateDescription vmstate_iomd = {
    .name = "acorn-iomd",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(irqa_latch, AcornIOMDState),
        VMSTATE_UINT32(irqa_mask, AcornIOMDState),
        VMSTATE_UINT32(irqb_level, AcornIOMDState),
        VMSTATE_UINT32(irqb_mask, AcornIOMDState),
        VMSTATE_UINT32(fiq_level, AcornIOMDState),
        VMSTATE_UINT32(fiq_mask, AcornIOMDState),
        VMSTATE_UINT32(dma_level, AcornIOMDState),
        VMSTATE_UINT32(dma_mask, AcornIOMDState),
        VMSTATE_UINT32_ARRAY(regs, AcornIOMDState, ACORN_IOMD_NUM_REGS),
        VMSTATE_STRUCT_ARRAY(timer, AcornIOMDState, ACORN_IOMD_NUM_TIMERS, 1,
                             vmstate_iomd_timer, AcornIOMDTimer),
        VMSTATE_UINT32(kctrl, AcornIOMDState),
        VMSTATE_INT16(mouse_x, AcornIOMDState),
        VMSTATE_INT16(mouse_y, AcornIOMDState),
        VMSTATE_UINT8(mouse_buttons, AcornIOMDState),
        VMSTATE_END_OF_LIST(),
    },
};

static void iomd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Acorn IOMD";
    dc->realize = iomd_realize;
    dc->vmsd = &vmstate_iomd;
    rc->phases.hold = iomd_reset_hold;
}

static const TypeInfo iomd_info = {
    .name          = TYPE_ACORN_IOMD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AcornIOMDState),
    .instance_init = iomd_init,
    .class_init    = iomd_class_init,
};

static void iomd_register_types(void)
{
    type_register_static(&iomd_info);
}

type_init(iomd_register_types)
