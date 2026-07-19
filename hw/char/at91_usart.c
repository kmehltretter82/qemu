/*
 * Atmel/Microchip AT91 USART / Debug Unit (DBGU) UART.
 *
 * One model serves both the DBGU (a cut-down USART) and the USART0-3 blocks:
 * they share the CR/MR/IER/IDR/IMR/CSR/RHR/THR/BRGR core the Linux
 * atmel_serial driver programs.  A non-zero "chip-id" property makes the
 * device expose the Debug Unit's Chip ID registers (CIDR/EXID at 0x40/0x44);
 * the plain USARTs leave it 0.
 *
 * The peripheral DMA controller (PDC) built into the USART is modelled too,
 * because the board's USART DT nodes use it ("atmel,use-dma-tx/rx"): TX is
 * copied straight to the chardev, RX is written byte-by-byte into the PDC
 * buffer with an idle-timeout so partial buffers get flushed.  The ENDTX/
 * TXBUFE/ENDRX/RXBUFF status bits track the PDC counter state (level), which
 * is what the driver relies on to kick the first transfer.  TX/RX are
 * otherwise instantaneous; advanced USART features (manchester, hardware
 * handshake, ISO7816) are not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "chardev/char-fe.h"
#include "hw/char/at91_usart.h"
#include "trace.h"

#define US_CR      0x00   /* Control Register            (write-only) */
#define US_MR      0x04   /* Mode Register                            */
#define US_IER     0x08   /* Interrupt Enable Register   (write-only) */
#define US_IDR     0x0C   /* Interrupt Disable Register  (write-only) */
#define US_IMR     0x10   /* Interrupt Mask Register     (read-only)  */
#define US_CSR     0x14   /* Channel Status Register     (read-only)  */
#define US_RHR     0x18   /* Receive Holding Register    (read-only)  */
#define US_THR     0x1C   /* Transmit Holding Register   (write-only) */
#define US_BRGR    0x20   /* Baud Rate Generator Register             */
#define US_RTOR    0x24   /* Receiver Time-out Register               */
#define US_CIDR    0x40   /* (DBGU) Chip ID Register     (read-only)  */
#define US_EXID    0x44   /* (DBGU) Chip ID Extension    (read-only)  */
#define US_NAME    0xF0   /* IP Name Register            (read-only)  */
#define US_VERSION 0xFC   /* IP Version Register         (read-only)  */

/* US_NAME values ("USAR"/"DBGU" big-endian ASCII) and matching versions.
 * The atmel_serial driver identifies the IP flavor from these registers
 * (atmel_get_ip_name(); Atmel vendor kernels fail the probe outright on an
 * unknown name), distinguishing full USARTs from the cut-down DBGU. */
#define US_NAME_USART    0x55534152
#define US_NAME_DBGU     0x44424755
#define US_VERSION_USART 0x10302
#define US_VERSION_DBGU  0x10202
/* PDC (peripheral DMA) registers */
#define US_RPR     0x100
#define US_RCR     0x104
#define US_TPR     0x108
#define US_TCR     0x10C
#define US_RNPR    0x110
#define US_RNCR    0x114
#define US_TNPR    0x118
#define US_TNCR    0x11C
#define US_PTCR    0x120
#define US_PTSR    0x124

/* Status/CSR bits */
#define US_RXRDY    (1u << 0)
#define US_TXRDY    (1u << 1)
#define US_ENDRX    (1u << 3)
#define US_ENDTX    (1u << 4)
#define US_TIMEOUT  (1u << 8)
#define US_TXEMPTY  (1u << 9)
#define US_TXBUFE   (1u << 11)
#define US_RXBUFF   (1u << 12)

/* Control Register bits */
#define US_CR_RSTRX   (1u << 2)
#define US_CR_RSTTX   (1u << 3)
#define US_CR_RXEN    (1u << 4)
#define US_CR_RXDIS   (1u << 5)
#define US_CR_TXEN    (1u << 6)
#define US_CR_TXDIS   (1u << 7)
#define US_CR_RSTSTA  (1u << 8)
#define US_CR_STTTO   (1u << 11)  /* start receiver time-out */
#define US_CR_RETTO   (1u << 15)  /* rearm receiver time-out */

/* PTCR bits */
#define PTCR_RXTEN    (1u << 0)
#define PTCR_RXTDIS   (1u << 1)
#define PTCR_TXTEN    (1u << 8)
#define PTCR_TXTDIS   (1u << 9)

/* Idle-timeout for flushing a partial PDC RX buffer (virtual time). */
#define USART_RX_TIMEOUT_NS  (2 * SCALE_MS)

struct AT91UsartState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend chr;
    qemu_irq irq;
    QEMUTimer *rx_timeout;

    uint32_t chip_id;   /* DBGU CIDR (0 = plain USART) */
    uint32_t chip_exid; /* DBGU EXID                  */

    uint32_t imr;       /* interrupt mask                    */
    uint32_t mr;        /* mode register (stored, unused)    */
    uint32_t brgr;      /* baud divisor (stored, unused)     */
    uint32_t rtor;      /* receiver time-out (stored)        */
    bool     timeout;   /* receiver time-out status (event)  */
    uint8_t  rx_fifo;   /* PIO single receive holding byte   */
    bool     rx_pending;
    bool     rx_enabled;
    bool     tx_enabled;

    /* PDC state */
    uint32_t rpr, rcr, tpr, tcr, rnpr, rncr, tnpr, tncr;
    bool     rxten, txten;
};

/*
 * ENDTX/TXBUFE/ENDRX/RXBUFF are level indicators of the PDC counter state:
 * ENDTX/ENDRX assert while the current counter is 0, TXBUFE/RXBUFF while both
 * the current and next counters are 0.  The driver enables the ENDTX interrupt
 * with TCR already 0 and relies on it firing to load the first buffer.
 */
static uint32_t usart_status(AT91UsartState *s)
{
    uint32_t sr = US_TXRDY | US_TXEMPTY;   /* TX is instantaneous */

    if (s->tcr == 0) {
        sr |= US_ENDTX;
    }
    if (s->tcr == 0 && s->tncr == 0) {
        sr |= US_TXBUFE;
    }
    if (s->rcr == 0) {
        sr |= US_ENDRX;
    }
    if (s->rcr == 0 && s->rncr == 0) {
        sr |= US_RXBUFF;
    }
    if (s->timeout) {
        sr |= US_TIMEOUT;
    }
    if (s->rx_pending) {
        sr |= US_RXRDY;
    }
    return sr;
}

static void usart_update_irq(AT91UsartState *s)
{
    qemu_set_irq(s->irq, (usart_status(s) & s->imr) ? 1 : 0);
}

/* Push the current (and any queued next) PDC TX buffer straight to the chardev. */
static void usart_pdc_tx(AT91UsartState *s)
{
    while (s->txten && s->tcr > 0) {
        g_autofree uint8_t *buf = g_malloc(s->tcr);

        address_space_read(&address_space_memory, s->tpr,
                           MEMTXATTRS_UNSPECIFIED, buf, s->tcr);
        /* Non-blocking (see US_THR): never stall the vCPU on the chardev. */
        qemu_chr_fe_write(&s->chr, buf, s->tcr);
        s->tpr += s->tcr;
        s->tcr = 0;
        if (s->tncr > 0) {          /* chain to the queued next buffer */
            s->tpr = s->tnpr;
            s->tcr = s->tncr;
            s->tnpr = s->tncr = 0;
        }
    }
    usart_update_irq(s);
}

static void usart_rx_timeout(void *opaque)
{
    AT91UsartState *s = opaque;

    s->timeout = true;
    usart_update_irq(s);
}

static uint64_t usart_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91UsartState *s = AT91_USART(opaque);
    uint32_t r = 0;

    switch (offset) {
    case US_MR:    return s->mr;
    case US_IMR:   return s->imr;
    case US_CSR:   return usart_status(s);
    case US_RHR:
        r = s->rx_fifo;
        s->rx_pending = false;
        usart_update_irq(s);
        qemu_chr_fe_accept_input(&s->chr);
        return r;
    case US_BRGR:  return s->brgr;
    case US_RTOR:  return s->rtor;
    case US_CIDR:  return s->chip_id;             /* 0 for a plain USART */
    case US_EXID:  return s->chip_id ? s->chip_exid : 0;
    case US_NAME:  return s->chip_id ? US_NAME_DBGU : US_NAME_USART;
    case US_VERSION:
        return s->chip_id ? US_VERSION_DBGU : US_VERSION_USART;
    case US_RPR:   return s->rpr;
    case US_RCR:   return s->rcr;
    case US_TPR:   return s->tpr;
    case US_TCR:   return s->tcr;
    case US_RNPR:  return s->rnpr;
    case US_RNCR:  return s->rncr;
    case US_TNPR:  return s->tnpr;
    case US_TNCR:  return s->tncr;
    case US_PTSR:  return (s->rxten ? PTCR_RXTEN : 0) |
                          (s->txten ? PTCR_TXTEN : 0);
    default:
        qemu_log_mask(LOG_UNIMP, "at91-usart: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void usart_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    AT91UsartState *s = AT91_USART(opaque);
    unsigned char ch;

    switch (offset) {
    case US_CR:
        if (value & US_CR_RXEN) {
            s->rx_enabled = true;
        }
        if (value & US_CR_RXDIS) {
            s->rx_enabled = false;
        }
        if (value & US_CR_TXEN) {
            s->tx_enabled = true;
        }
        if (value & US_CR_TXDIS) {
            s->tx_enabled = false;
        }
        if (value & US_CR_RSTRX) {
            s->rx_pending = false;
        }
        if (value & (US_CR_STTTO | US_CR_RETTO)) {
            /* (Re)start the receiver time-out: clear a pending time-out. */
            s->timeout = false;
            timer_del(s->rx_timeout);
        }
        usart_update_irq(s);
        break;
    case US_MR:    s->mr = value; break;
    case US_IER:   s->imr |= value;  usart_update_irq(s); break;
    case US_IDR:   s->imr &= ~value; usart_update_irq(s); break;
    case US_THR:
        ch = value & 0xff;
        trace_at91_usart_tx(ch);
        /* Non-blocking: TX is reported as always-ready (US_TXRDY/US_TXEMPTY),
         * so never block the vCPU on the chardev.  A blocking write_all() here
         * busy-spins the vCPU thread whenever the backend backpressures (e.g. a
         * pipe consumer that is not draining instantly, as with syz-manager),
         * hanging the guest.  Drop bytes the backend cannot take immediately. */
        qemu_chr_fe_write(&s->chr, &ch, 1);
        usart_update_irq(s);
        break;
    case US_BRGR:  s->brgr = value; break;
    case US_RTOR:  s->rtor = value; break;
    case US_RPR:   s->rpr = value; break;
    case US_RCR:   s->rcr = value; usart_update_irq(s); break;
    case US_TPR:   s->tpr = value; break;
    case US_TCR:   s->tcr = value; usart_pdc_tx(s); break;
    case US_RNPR:  s->rnpr = value; break;
    case US_RNCR:  s->rncr = value; usart_update_irq(s); break;
    case US_TNPR:  s->tnpr = value; break;
    case US_TNCR:  s->tncr = value; usart_update_irq(s); break;
    case US_PTCR:
        if (value & PTCR_RXTEN) {
            s->rxten = true;
        }
        if (value & PTCR_RXTDIS) {
            s->rxten = false;
        }
        if (value & PTCR_TXTEN) {
            s->txten = true;
        }
        if (value & PTCR_TXTDIS) {
            s->txten = false;
        }
        usart_pdc_tx(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-usart: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n",
                      offset, (uint32_t)value);
        break;
    }
}

static const MemoryRegionOps usart_ops = {
    .read = usart_read,
    .write = usart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static int usart_can_receive(void *opaque)
{
    AT91UsartState *s = AT91_USART(opaque);

    if (s->rxten) {
        return s->rcr > 0;      /* PDC: room while the buffer has space */
    }
    return s->rx_pending ? 0 : 1;
}

static void usart_receive(void *opaque, const uint8_t *buf, int size)
{
    AT91UsartState *s = AT91_USART(opaque);
    int i;

    if (size <= 0) {
        return;
    }

    if (s->rxten) {
        /* PDC receive: DMA the bytes into the receive buffer in guest memory. */
        for (i = 0; i < size && s->rcr > 0; i++) {
            trace_at91_usart_rx(buf[i]);
            address_space_write(&address_space_memory, s->rpr,
                                MEMTXATTRS_UNSPECIFIED, &buf[i], 1);
            s->rpr++;
            s->rcr--;
            if (s->rcr == 0 && s->rncr > 0) {   /* chain to next buffer */
                s->rpr = s->rnpr;
                s->rcr = s->rncr;
                s->rnpr = s->rncr = 0;
            }
        }
        /* Flush a partial buffer after a short idle, like the RX time-out. */
        timer_mod(s->rx_timeout,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + USART_RX_TIMEOUT_NS);
        usart_update_irq(s);
        return;
    }

    trace_at91_usart_rx(buf[0]);
    s->rx_fifo = buf[0];
    s->rx_pending = true;
    usart_update_irq(s);
}

static void usart_reset(DeviceState *dev)
{
    AT91UsartState *s = AT91_USART(dev);

    s->imr = s->mr = s->brgr = s->rtor = 0;
    s->timeout = false;
    s->rx_pending = s->rx_enabled = s->tx_enabled = false;
    s->rpr = s->rcr = s->tpr = s->tcr = 0;
    s->rnpr = s->rncr = s->tnpr = s->tncr = 0;
    s->rxten = s->txten = false;
    if (s->rx_timeout) {
        timer_del(s->rx_timeout);
    }
    usart_update_irq(s);
}

static void usart_realize(DeviceState *dev, Error **errp)
{
    AT91UsartState *s = AT91_USART(dev);

    s->rx_timeout = timer_new_ns(QEMU_CLOCK_VIRTUAL, usart_rx_timeout, s);
    qemu_chr_fe_set_handlers(&s->chr, usart_can_receive, usart_receive,
                             NULL, NULL, s, NULL, true);
}

static void usart_init(Object *obj)
{
    AT91UsartState *s = AT91_USART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &usart_ops, s, "at91-usart", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property usart_properties[] = {
    DEFINE_PROP_CHR("chardev", AT91UsartState, chr),
    DEFINE_PROP_UINT32("chip-id", AT91UsartState, chip_id, 0),
    DEFINE_PROP_UINT32("chip-exid", AT91UsartState, chip_exid, 0),
};

static const VMStateDescription vmstate_at91_usart = {
    .name = "at91-usart",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(imr, AT91UsartState),
        VMSTATE_UINT32(mr, AT91UsartState),
        VMSTATE_UINT32(brgr, AT91UsartState),
        VMSTATE_UINT32(rtor, AT91UsartState),
        VMSTATE_BOOL(timeout, AT91UsartState),
        VMSTATE_UINT8(rx_fifo, AT91UsartState),
        VMSTATE_BOOL(rx_pending, AT91UsartState),
        VMSTATE_BOOL(rx_enabled, AT91UsartState),
        VMSTATE_BOOL(tx_enabled, AT91UsartState),
        VMSTATE_UINT32(rpr, AT91UsartState),
        VMSTATE_UINT32(rcr, AT91UsartState),
        VMSTATE_UINT32(tpr, AT91UsartState),
        VMSTATE_UINT32(tcr, AT91UsartState),
        VMSTATE_UINT32(rnpr, AT91UsartState),
        VMSTATE_UINT32(rncr, AT91UsartState),
        VMSTATE_UINT32(tnpr, AT91UsartState),
        VMSTATE_UINT32(tncr, AT91UsartState),
        VMSTATE_BOOL(rxten, AT91UsartState),
        VMSTATE_BOOL(txten, AT91UsartState),
        VMSTATE_TIMER_PTR(rx_timeout, AT91UsartState),
        VMSTATE_END_OF_LIST()
    }
};

static void usart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = usart_realize;
    device_class_set_legacy_reset(dc, usart_reset);
    device_class_set_props(dc, usart_properties);
    dc->vmsd = &vmstate_at91_usart;
}

static const TypeInfo usart_type = {
    .name = TYPE_AT91_USART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91UsartState),
    .instance_init = usart_init,
    .class_init = usart_class_init,
};

static void at91_usart_register_types(void)
{
    type_register_static(&usart_type);
}

type_init(at91_usart_register_types)
