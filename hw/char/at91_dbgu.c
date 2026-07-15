/*
 * Atmel/Microchip AT91 Debug Unit (DBGU) - console UART.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "chardev/char-fe.h"
#include "hw/char/at91_dbgu.h"
#include "trace.h"

#define DBGU_CR     0x00   /* Control Register            (write-only) */
#define DBGU_MR     0x04   /* Mode Register                            */
#define DBGU_IER    0x08   /* Interrupt Enable Register   (write-only) */
#define DBGU_IDR    0x0C   /* Interrupt Disable Register  (write-only) */
#define DBGU_IMR    0x10   /* Interrupt Mask Register     (read-only)  */
#define DBGU_SR     0x14   /* Status Register             (read-only)  */
#define DBGU_RHR    0x18   /* Receive Holding Register    (read-only)  */
#define DBGU_THR    0x1C   /* Transmit Holding Register   (write-only) */
#define DBGU_BRGR   0x20   /* Baud Rate Generator Register             */
#define DBGU_CIDR   0x40   /* Chip ID Register            (read-only)  */
#define DBGU_EXID   0x44   /* Chip ID Extension Register  (read-only)  */

/* Status/CSR bits (shared with USART) */
#define DBGU_RXRDY    (1u << 0)
#define DBGU_TXRDY    (1u << 1)
#define DBGU_TXEMPTY  (1u << 9)

/* Control Register bits */
#define DBGU_CR_RSTRX   (1u << 2)
#define DBGU_CR_RSTTX   (1u << 3)
#define DBGU_CR_RXEN    (1u << 4)
#define DBGU_CR_RXDIS   (1u << 5)
#define DBGU_CR_TXEN    (1u << 6)
#define DBGU_CR_TXDIS   (1u << 7)
#define DBGU_CR_RSTSTA  (1u << 8)

/* Chip identification for AT91SAM9G45 (datasheet section 6.3) */
#define SAM9G45_CIDR  0x819B05A2
#define SAM9G45_EXID  0x00000004

struct AT91DbguState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend chr;
    qemu_irq irq;

    uint32_t imr;       /* interrupt mask                    */
    uint32_t mr;        /* mode register (stored, unused)    */
    uint32_t brgr;      /* baud divisor (stored, unused)     */
    uint8_t  rx_fifo;   /* single receive holding byte       */
    bool     rx_pending;
    bool     rx_enabled;
    bool     tx_enabled;
};

static uint32_t dbgu_status(AT91DbguState *s)
{
    /* TX is instantaneous in the model, so it is always ready/empty. */
    uint32_t sr = DBGU_TXRDY | DBGU_TXEMPTY;
    if (s->rx_pending) {
        sr |= DBGU_RXRDY;
    }
    return sr;
}

static void dbgu_update_irq(AT91DbguState *s)
{
    qemu_set_irq(s->irq, (dbgu_status(s) & s->imr) ? 1 : 0);
}

static uint64_t dbgu_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91DbguState *s = AT91_DBGU(opaque);
    uint32_t r = 0;

    switch (offset) {
    case DBGU_MR:
        r = s->mr;
        break;
    case DBGU_IMR:
        r = s->imr;
        break;
    case DBGU_SR:
        r = dbgu_status(s);
        break;
    case DBGU_RHR:
        r = s->rx_fifo;
        s->rx_pending = false;
        dbgu_update_irq(s);
        qemu_chr_fe_accept_input(&s->chr);
        break;
    case DBGU_BRGR:
        r = s->brgr;
        break;
    case DBGU_CIDR:
        r = SAM9G45_CIDR;
        break;
    case DBGU_EXID:
        r = SAM9G45_EXID;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-dbgu: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        break;
    }
    return r;
}

static void dbgu_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AT91DbguState *s = AT91_DBGU(opaque);
    unsigned char ch;

    switch (offset) {
    case DBGU_CR:
        if (value & DBGU_CR_RXEN) {
            s->rx_enabled = true;
        }
        if (value & DBGU_CR_RXDIS) {
            s->rx_enabled = false;
        }
        if (value & DBGU_CR_TXEN) {
            s->tx_enabled = true;
        }
        if (value & DBGU_CR_TXDIS) {
            s->tx_enabled = false;
        }
        if (value & DBGU_CR_RSTRX) {
            s->rx_pending = false;
        }
        /* RSTTX/RSTSTA have no observable state in this model. */
        dbgu_update_irq(s);
        break;
    case DBGU_MR:
        s->mr = value;
        break;
    case DBGU_IER:
        s->imr |= value;
        dbgu_update_irq(s);
        break;
    case DBGU_IDR:
        s->imr &= ~value;
        dbgu_update_irq(s);
        break;
    case DBGU_THR:
        ch = value & 0xff;
        trace_at91_dbgu_tx(ch);
        /* Instant, blocking-free drain: fine for a console. */
        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        dbgu_update_irq(s);
        break;
    case DBGU_BRGR:
        s->brgr = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-dbgu: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n",
                      offset, (uint32_t)value);
        break;
    }
}

static const MemoryRegionOps dbgu_ops = {
    .read = dbgu_read,
    .write = dbgu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static int dbgu_can_receive(void *opaque)
{
    AT91DbguState *s = AT91_DBGU(opaque);
    return s->rx_pending ? 0 : 1;
}

static void dbgu_receive(void *opaque, const uint8_t *buf, int size)
{
    AT91DbguState *s = AT91_DBGU(opaque);

    if (size <= 0) {
        return;
    }
    trace_at91_dbgu_rx(buf[0]);
    s->rx_fifo = buf[0];
    s->rx_pending = true;
    dbgu_update_irq(s);
}

static void dbgu_reset(DeviceState *dev)
{
    AT91DbguState *s = AT91_DBGU(dev);

    s->imr = 0;
    s->mr = 0;
    s->brgr = 0;
    s->rx_pending = false;
    s->rx_enabled = false;
    s->tx_enabled = false;
    dbgu_update_irq(s);
}

static void dbgu_realize(DeviceState *dev, Error **errp)
{
    AT91DbguState *s = AT91_DBGU(dev);

    qemu_chr_fe_set_handlers(&s->chr, dbgu_can_receive, dbgu_receive,
                             NULL, NULL, s, NULL, true);
}

static void dbgu_init(Object *obj)
{
    AT91DbguState *s = AT91_DBGU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &dbgu_ops, s, "at91-dbgu", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property dbgu_properties[] = {
    DEFINE_PROP_CHR("chardev", AT91DbguState, chr),
};

static const VMStateDescription vmstate_at91_dbgu = {
    .name = "at91-dbgu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(imr, AT91DbguState),
        VMSTATE_UINT32(mr, AT91DbguState),
        VMSTATE_UINT32(brgr, AT91DbguState),
        VMSTATE_UINT8(rx_fifo, AT91DbguState),
        VMSTATE_BOOL(rx_pending, AT91DbguState),
        VMSTATE_BOOL(rx_enabled, AT91DbguState),
        VMSTATE_BOOL(tx_enabled, AT91DbguState),
        VMSTATE_END_OF_LIST()
    }
};

static void dbgu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = dbgu_realize;
    device_class_set_legacy_reset(dc, dbgu_reset);
    device_class_set_props(dc, dbgu_properties);
    dc->vmsd = &vmstate_at91_dbgu;
}

static const TypeInfo dbgu_type = {
    .name = TYPE_AT91_DBGU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91DbguState),
    .instance_init = dbgu_init,
    .class_init = dbgu_class_init,
};

static void at91_dbgu_register_types(void)
{
    type_register_static(&dbgu_type);
}

type_init(at91_dbgu_register_types)
