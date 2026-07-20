/*
 * Intel/DEC 21285 "Footbridge" core logic for StrongARM SA-110.
 *
 * System controller of the Rebel NetWinder and Intel EBSA-285.
 * This models the ARM-side CSR block at phys 0x42000000: interrupt
 * controller, four 24-bit down-counting timers and the internal UART.
 * The PCI host bridge function is not modelled yet; the PCI-header and
 * SDRAM/X-Bus control registers are readable/writable storage so that
 * firmware and kernel init code can run unmodified.
 *
 * Register layout per the 21285 datasheet; Linux's
 * arch/arm/include/asm/hardware/dec21285.h uses the same offsets.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/ptimer.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "migration/vmstate.h"
#include "hw/arm/dc21285.h"

/* CSR offsets (word registers) */
#define CSR_VENDOR_DEVICE   0x000
#define CSR_PCICMD          0x004
#define CSR_CLASSREV        0x008
#define CSR_SA110_CNTL      0x13c

#define CSR_UARTDR          0x160
#define CSR_RXSTAT          0x164
#define CSR_H_UBRLCR        0x168
#define CSR_M_UBRLCR        0x16c
#define CSR_L_UBRLCR        0x170
#define CSR_UARTCON         0x174
#define CSR_UARTFLG         0x178

#define CSR_IRQ_STATUS      0x180
#define CSR_IRQ_RAWSTATUS   0x184
#define CSR_IRQ_ENABLE      0x188
#define CSR_IRQ_DISABLE     0x18c
#define CSR_IRQ_SOFT        0x190
#define CSR_FIQ_STATUS      0x280
#define CSR_FIQ_RAWSTATUS   0x284
#define CSR_FIQ_ENABLE      0x288
#define CSR_FIQ_DISABLE     0x28c
#define CSR_FIQ_SOFT        0x290

#define CSR_TIMER_BASE(n)   (0x300 + (n) * 0x20)  /* n = 0..3 */
#define TIMER_LOAD          0x00
#define TIMER_VALUE         0x04
#define TIMER_CNTL          0x08
#define TIMER_CLR           0x0c

#define TIMER_CNTL_ENABLE       (1 << 7)
#define TIMER_CNTL_AUTORELOAD   (1 << 6)
#define TIMER_CNTL_MODEMASK     (3 << 2)
#define TIMER_CNTL_DIV16        (1 << 2)
#define TIMER_CNTL_DIV256       (2 << 2)

#define TIMER_MAX           0xffffff  /* 24-bit counters */

/* Interrupt source bit numbers (shared raw status for IRQ and FIQ) */
#define INT_SOFTIRQ         1
#define INT_UART_RX         2
#define INT_UART_TX         3
#define INT_TIMER(n)        (4 + (n))  /* timer1..4 -> bits 4..7 */

#define UARTFLG_TXBUSY      (1 << 3)
#define UARTFLG_RXFE        (1 << 4)
#define UARTFLG_TXFF        (1 << 5)

#define SA110_CNTL_INITCMPLETE  (1 << 0)
#define SA110_CNTL_ROMWIDTH_32  (2 << 14)
#define SA110_CNTL_PCICFN       (1u << 31)

#define UART_RX_FIFO_LEN    16

static void dc21285_update(DC21285State *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->irq_enable) != 0);
    qemu_set_irq(s->fiq, (s->int_raw & s->fiq_enable) != 0);
}

static void dc21285_set_raw(DC21285State *s, int bit, int level)
{
    if (level) {
        s->int_raw |= 1u << bit;
    } else {
        s->int_raw &= ~(1u << bit);
    }
    dc21285_update(s);
}

static void dc21285_set_irq(void *opaque, int irq, int level)
{
    dc21285_set_raw(opaque, irq, level);
}

/* Timers */

static void dc21285_timer_tick(void *opaque)
{
    DC21285Timer *t = opaque;
    DC21285State *s = t->parent;

    dc21285_set_raw(s, INT_TIMER(t->index), 1);
}

/* Must be called inside a ptimer transaction. */
static void dc21285_timer_reprogram(DC21285Timer *t)
{
    uint32_t freq = t->parent->fclk;
    uint32_t limit;

    ptimer_stop(t->timer);

    switch (t->control & TIMER_CNTL_MODEMASK) {
    case TIMER_CNTL_DIV16:
        freq >>= 4;
        break;
    case TIMER_CNTL_DIV256:
        freq >>= 8;
        break;
    default:
        break;
    }
    ptimer_set_freq(t->timer, freq);

    if (t->control & TIMER_CNTL_AUTORELOAD) {
        limit = t->load & TIMER_MAX;
    } else {
        /* Free running: wraps through the full 24-bit range. */
        limit = TIMER_MAX;
    }
    ptimer_set_limit(t->timer, limit ? limit : TIMER_MAX, 0);

    if (t->control & TIMER_CNTL_ENABLE) {
        /* Enabling loads the counter from the load register. */
        ptimer_set_count(t->timer, (t->load & TIMER_MAX) ?
                         (t->load & TIMER_MAX) : TIMER_MAX);
        ptimer_run(t->timer, 0);
    }
}

static uint64_t dc21285_timer_read(DC21285State *s, int n, hwaddr offset)
{
    DC21285Timer *t = &s->timer[n];

    switch (offset) {
    case TIMER_LOAD:
        return t->load;
    case TIMER_VALUE:
        return ptimer_get_count(t->timer);
    case TIMER_CNTL:
        return t->control;
    case TIMER_CLR:
        return 0;
    default:
        g_assert_not_reached();
    }
}

static void dc21285_timer_write(DC21285State *s, int n, hwaddr offset,
                                uint32_t value)
{
    DC21285Timer *t = &s->timer[n];

    switch (offset) {
    case TIMER_LOAD:
        t->load = value & TIMER_MAX;
        break;
    case TIMER_VALUE:
        /* read only */
        break;
    case TIMER_CNTL:
        t->control = value & 0xff;
        ptimer_transaction_begin(t->timer);
        dc21285_timer_reprogram(t);
        ptimer_transaction_commit(t->timer);
        break;
    case TIMER_CLR:
        dc21285_set_raw(s, INT_TIMER(n), 0);
        break;
    default:
        g_assert_not_reached();
    }
}

/* UART */

static void dc21285_uart_update_rx_irq(DC21285State *s)
{
    dc21285_set_raw(s, INT_UART_RX, s->uart_rx_len != 0);
}

static int dc21285_uart_can_receive(void *opaque)
{
    DC21285State *s = opaque;

    return UART_RX_FIFO_LEN - s->uart_rx_len;
}

static void dc21285_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    DC21285State *s = opaque;
    int i;

    for (i = 0; i < size && s->uart_rx_len < UART_RX_FIFO_LEN; i++) {
        s->uart_rx_fifo[(s->uart_rx_pos + s->uart_rx_len) % UART_RX_FIFO_LEN] =
            buf[i];
        s->uart_rx_len++;
    }
    dc21285_uart_update_rx_irq(s);
}

static void dc21285_uart_event(void *opaque, QEMUChrEvent event)
{
}

static uint32_t dc21285_uart_rx_pop(DC21285State *s)
{
    uint32_t c = 0;

    if (s->uart_rx_len) {
        c = s->uart_rx_fifo[s->uart_rx_pos];
        s->uart_rx_pos = (s->uart_rx_pos + 1) % UART_RX_FIFO_LEN;
        s->uart_rx_len--;
        qemu_chr_fe_accept_input(&s->chr);
    }
    dc21285_uart_update_rx_irq(s);
    return c;
}

/* MMIO */

static uint64_t dc21285_read(void *opaque, hwaddr offset, unsigned size)
{
    DC21285State *s = opaque;

    if (offset >= CSR_TIMER_BASE(0) && offset < CSR_TIMER_BASE(4)) {
        return dc21285_timer_read(s, (offset - CSR_TIMER_BASE(0)) >> 5,
                                  offset & 0x1c);
    }

    switch (offset) {
    case CSR_UARTDR:
        return dc21285_uart_rx_pop(s);
    case CSR_RXSTAT:
        return 0; /* no framing/parity/overrun errors */
    case CSR_H_UBRLCR:
        return s->h_ubrlcr;
    case CSR_M_UBRLCR:
        return s->m_ubrlcr;
    case CSR_L_UBRLCR:
        return s->l_ubrlcr;
    case CSR_UARTCON:
        return s->uartcon;
    case CSR_UARTFLG:
        /* TX always idle and never full; RXFE tracks the FIFO. */
        return s->uart_rx_len ? 0 : UARTFLG_RXFE;

    case CSR_IRQ_STATUS:
        return s->int_raw & s->irq_enable;
    case CSR_IRQ_RAWSTATUS:
        return s->int_raw;
    case CSR_IRQ_ENABLE:
        return s->irq_enable;
    case CSR_FIQ_STATUS:
        return s->int_raw & s->fiq_enable;
    case CSR_FIQ_RAWSTATUS:
        return s->int_raw;
    case CSR_FIQ_ENABLE:
        return s->fiq_enable;
    case CSR_IRQ_SOFT:
    case CSR_FIQ_SOFT:
        return (s->int_raw >> INT_SOFTIRQ) & 1;

    default:
        if (offset < DC21285_NUM_REGS * 4) {
            return s->regs[offset >> 2];
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x"HWADDR_FMT_plx"\n",
                      __func__, offset);
        return 0;
    }
}

static void dc21285_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    DC21285State *s = opaque;
    uint8_t ch;

    if (offset >= CSR_TIMER_BASE(0) && offset < CSR_TIMER_BASE(4)) {
        dc21285_timer_write(s, (offset - CSR_TIMER_BASE(0)) >> 5,
                            offset & 0x1c, value);
        return;
    }

    switch (offset) {
    case CSR_UARTDR:
        ch = value;
        /* XXX this blocks entire thread. Rewrite to use
         * qemu_chr_fe_write and background I/O callbacks */
        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        break;
    case CSR_H_UBRLCR:
        s->h_ubrlcr = value;
        break;
    case CSR_M_UBRLCR:
        s->m_ubrlcr = value;
        break;
    case CSR_L_UBRLCR:
        s->l_ubrlcr = value;
        break;
    case CSR_UARTCON:
        s->uartcon = value;
        break;

    case CSR_IRQ_ENABLE:
        s->irq_enable |= value;
        dc21285_update(s);
        break;
    case CSR_IRQ_DISABLE:
        s->irq_enable &= ~value;
        dc21285_update(s);
        break;
    case CSR_IRQ_SOFT:
        dc21285_set_raw(s, INT_SOFTIRQ, value & 1);
        break;
    case CSR_FIQ_ENABLE:
        s->fiq_enable |= value;
        dc21285_update(s);
        break;
    case CSR_FIQ_DISABLE:
        s->fiq_enable &= ~value;
        dc21285_update(s);
        break;
    case CSR_FIQ_SOFT:
        dc21285_set_raw(s, INT_SOFTIRQ, value & 1);
        break;

    case CSR_PCICMD:
        /*
         * Low half is the PCI command register (storage); the status
         * half is write-one-to-clear. Linux clears the abort flags
         * this way and treats config cycles as failed while any are
         * set.
         */
        s->regs[CSR_PCICMD >> 2] =
            (s->regs[CSR_PCICMD >> 2] & 0xffff0000 & ~value) |
            (value & 0xffff);
        break;

    default:
        if (offset < DC21285_NUM_REGS * 4) {
            s->regs[offset >> 2] = value;
            return;
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x"HWADDR_FMT_plx"\n",
                      __func__, offset);
        break;
    }
}

static const MemoryRegionOps dc21285_ops = {
    .read = dc21285_read,
    .write = dc21285_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void dc21285_reset_hold(Object *obj, ResetType type)
{
    DC21285State *s = DC21285(obj);
    int i;

    s->int_raw = 0;
    s->irq_enable = 0;
    s->fiq_enable = 0;
    s->uart_rx_pos = 0;
    s->uart_rx_len = 0;
    s->h_ubrlcr = 0;
    s->m_ubrlcr = 0;
    s->l_ubrlcr = 0;
    s->uartcon = 0;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[CSR_VENDOR_DEVICE >> 2] = 0x10651011; /* DEC 21285 */
    s->regs[CSR_CLASSREV >> 2] = 0x06000005;      /* host bridge, rev 5 */
    s->regs[CSR_SA110_CNTL >> 2] = SA110_CNTL_PCICFN |
        SA110_CNTL_ROMWIDTH_32 | SA110_CNTL_INITCMPLETE;

    for (i = 0; i < DC21285_NUM_TIMERS; i++) {
        DC21285Timer *t = &s->timer[i];

        t->load = 0;
        t->control = 0;
        ptimer_transaction_begin(t->timer);
        ptimer_stop(t->timer);
        ptimer_transaction_commit(t->timer);
    }

    /* The TX path is always ready: raw TX interrupt is permanently set. */
    dc21285_set_raw(s, INT_UART_TX, 1);
}

static void dc21285_init(Object *obj)
{
    DC21285State *s = DC21285(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &dc21285_ops, s, "dc21285-csr",
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(dev, dc21285_set_irq, 32);

    for (i = 0; i < DC21285_NUM_TIMERS; i++) {
        s->timer[i].parent = s;
        s->timer[i].index = i;
        s->timer[i].timer = ptimer_init(dc21285_timer_tick, &s->timer[i],
                                        PTIMER_POLICY_LEGACY);
    }
}

static void dc21285_realize(DeviceState *dev, Error **errp)
{
    DC21285State *s = DC21285(dev);

    qemu_chr_fe_set_handlers(&s->chr, dc21285_uart_can_receive,
                             dc21285_uart_receive, dc21285_uart_event,
                             NULL, s, NULL, true);
}

static const VMStateDescription vmstate_dc21285_timer = {
    .name = "dc21285-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(load, DC21285Timer),
        VMSTATE_UINT32(control, DC21285Timer),
        VMSTATE_PTIMER(timer, DC21285Timer),
        VMSTATE_END_OF_LIST(),
    },
};

static const VMStateDescription vmstate_dc21285 = {
    .name = "dc21285",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(int_raw, DC21285State),
        VMSTATE_UINT32(irq_enable, DC21285State),
        VMSTATE_UINT32(fiq_enable, DC21285State),
        VMSTATE_UINT32_ARRAY(regs, DC21285State, DC21285_NUM_REGS),
        VMSTATE_STRUCT_ARRAY(timer, DC21285State, DC21285_NUM_TIMERS, 1,
                             vmstate_dc21285_timer, DC21285Timer),
        VMSTATE_UINT8_ARRAY(uart_rx_fifo, DC21285State, 16),
        VMSTATE_INT32(uart_rx_pos, DC21285State),
        VMSTATE_INT32(uart_rx_len, DC21285State),
        VMSTATE_UINT32(h_ubrlcr, DC21285State),
        VMSTATE_UINT32(m_ubrlcr, DC21285State),
        VMSTATE_UINT32(l_ubrlcr, DC21285State),
        VMSTATE_UINT32(uartcon, DC21285State),
        VMSTATE_END_OF_LIST(),
    },
};

static const Property dc21285_properties[] = {
    DEFINE_PROP_CHR("chardev", DC21285State, chr),
    DEFINE_PROP_UINT32("fclk", DC21285State, fclk, 50000000),
};

static void dc21285_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "DC21285 Footbridge core logic";
    dc->realize = dc21285_realize;
    dc->vmsd = &vmstate_dc21285;
    rc->phases.hold = dc21285_reset_hold;
    device_class_set_props(dc, dc21285_properties);
}

static const TypeInfo dc21285_info = {
    .name          = TYPE_DC21285,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DC21285State),
    .instance_init = dc21285_init,
    .class_init    = dc21285_class_init,
};

static void dc21285_register_types(void)
{
    type_register_static(&dc21285_info);
}

type_init(dc21285_register_types)
