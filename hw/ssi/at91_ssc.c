/*
 * Atmel AT91 synchronous serial controller.
 *
 * This models the SSC register interface, PIO data path and embedded PDC.
 * The receiver loopback bit provides a self-contained data path for board and
 * migration tests.  External DMA also works through the RHR/THR MMIO ports.
 * Serial clock edges and external codec pins are not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "hw/core/irq.h"
#include "hw/ssi/at91_ssc.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"
#include "trace.h"

#define SSC_CR          0x000
#define SSC_CMR         0x004
#define SSC_RCMR        0x010
#define SSC_RFMR        0x014
#define SSC_TCMR        0x018
#define SSC_TFMR        0x01c
#define SSC_RHR         0x020
#define SSC_THR         0x024
#define SSC_RSHR        0x030
#define SSC_TSHR        0x034
#define SSC_RC0R        0x038
#define SSC_RC1R        0x03c
#define SSC_SR          0x040
#define SSC_IER         0x044
#define SSC_IDR         0x048
#define SSC_IMR         0x04c
#define SSC_VERSION     0x0fc

#define PDC_RPR         0x100
#define PDC_RCR         0x104
#define PDC_TPR         0x108
#define PDC_TCR         0x10c
#define PDC_RNPR        0x110
#define PDC_RNCR        0x114
#define PDC_TNPR        0x118
#define PDC_TNCR        0x11c
#define PDC_PTCR        0x120
#define PDC_PTSR        0x124

#define CR_RXEN         BIT(0)
#define CR_RXDIS        BIT(1)
#define CR_TXEN         BIT(8)
#define CR_TXDIS        BIT(9)
#define CR_SWRST        BIT(15)

#define RFMR_LOOP       BIT(5)

#define SR_TXRDY        BIT(0)
#define SR_TXEMPTY      BIT(1)
#define SR_ENDTX        BIT(2)
#define SR_TXBUFE       BIT(3)
#define SR_RXRDY        BIT(4)
#define SR_OVRUN        BIT(5)
#define SR_ENDRX        BIT(6)
#define SR_RXBUFF       BIT(7)
#define SR_TXEN         BIT(16)
#define SR_RXEN         BIT(17)

#define PDC_RXTEN       BIT(0)
#define PDC_RXTDIS      BIT(1)
#define PDC_TXTEN       BIT(8)
#define PDC_TXTDIS      BIT(9)

#define SSC_IOMEM_SIZE  0x4000
#define SSC_VERSION_VAL 0x00000100

struct AT91SscState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t cmr;
    uint32_t rcmr;
    uint32_t rfmr;
    uint32_t tcmr;
    uint32_t tfmr;
    uint32_t rhr;
    uint32_t rshr;
    uint32_t tshr;
    uint32_t rc0r;
    uint32_t rc1r;
    uint32_t imr;

    uint32_t rpr;
    uint32_t rcr;
    uint32_t tpr;
    uint32_t tcr;
    uint32_t rnpr;
    uint32_t rncr;
    uint32_t tnpr;
    uint32_t tncr;

    bool rx_enabled;
    bool tx_enabled;
    bool rxten;
    bool txten;
    bool rx_pending;
    bool overrun;
    bool endrx_event;
    bool endtx_event;

    qemu_irq tx_request;
    qemu_irq rx_request;
    QEMUBH *tx_request_bh;
    bool tx_request_rearm;
};

static unsigned at91_ssc_word_bytes(uint32_t fmr)
{
    unsigned bits = (fmr & 0x1f) + 1;

    return bits <= 8 ? 1 : bits <= 16 ? 2 : 4;
}

static uint32_t at91_ssc_status(AT91SscState *s)
{
    uint32_t status = SR_TXRDY | SR_TXEMPTY;

    if (!s->tcr || s->endtx_event) {
        status |= SR_ENDTX;
    }
    if (!s->tcr && !s->tncr) {
        status |= SR_TXBUFE;
    }
    if (!s->rcr || s->endrx_event) {
        status |= SR_ENDRX;
    }
    if (!s->rcr && !s->rncr) {
        status |= SR_RXBUFF;
    }
    if (s->rx_pending) {
        status |= SR_RXRDY;
    }
    if (s->overrun) {
        status |= SR_OVRUN;
    }
    if (s->tx_enabled) {
        status |= SR_TXEN;
    }
    if (s->rx_enabled) {
        status |= SR_RXEN;
    }
    return status;
}

static void at91_ssc_update_irq(AT91SscState *s)
{
    qemu_set_irq(s->irq, (at91_ssc_status(s) & s->imr) != 0);
}

/*
 * Hardware DMA request lines (Table 40-1 ids 5..8).  TX request follows
 * TXRDY (always set here - transmission is synchronous) with a per-write
 * rearm edge, and additionally waits for RHR to be drained so a loopback
 * word cannot be overrun by an early TX grant; RX request follows RXRDY
 * with natural edges.  Same contract as the SPI request lines.
 */
static void at91_ssc_update_dma_requests(AT91SscState *s)
{
    qemu_set_irq(s->tx_request,
                 s->tx_enabled && !s->tx_request_rearm && !s->rx_pending);
    qemu_set_irq(s->rx_request, s->rx_enabled && s->rx_pending);
}

static void at91_ssc_tx_request_bh(void *opaque)
{
    AT91SscState *s = opaque;

    s->tx_request_rearm = false;
    at91_ssc_update_dma_requests(s);
}

static void at91_ssc_rearm_tx_request(AT91SscState *s)
{
    if (!s->tx_enabled) {
        return;
    }
    s->tx_request_rearm = true;
    at91_ssc_update_dma_requests(s);
    qemu_bh_schedule(s->tx_request_bh);
}

/*
 * The PDC loads the next pointer/counter into the current registers as
 * soon as the current counter is zero on an enabled channel - including
 * when the next counter is programmed AFTER the current buffer emptied.
 */
static void at91_ssc_promote_rx(AT91SscState *s)
{
    if (s->rxten && !s->rcr && s->rncr) {
        s->rpr = s->rnpr;
        s->rcr = s->rncr;
        s->rnpr = 0;
        s->rncr = 0;
    }
}

static void at91_ssc_promote_tx(AT91SscState *s)
{
    if (s->txten && !s->tcr && s->tncr) {
        s->tpr = s->tnpr;
        s->tcr = s->tncr;
        s->tnpr = 0;
        s->tncr = 0;
    }
}

static void at91_ssc_receive_word(AT91SscState *s, uint32_t value)
{
    unsigned bytes = at91_ssc_word_bytes(s->rfmr);
    uint8_t buffer[4];

    if (!s->rx_enabled) {
        return;
    }

    at91_ssc_promote_rx(s);
    if (s->rxten && s->rcr) {
        stn_le_p(buffer, bytes, value);
        address_space_write(&address_space_memory, s->rpr,
                            MEMTXATTRS_UNSPECIFIED, buffer, bytes);
        s->rpr += bytes;
        s->rcr--;
        if (!s->rcr) {
            s->endrx_event = true;
            at91_ssc_promote_rx(s);
        }
    } else {
        if (s->rx_pending) {
            s->overrun = true;
        }
        s->rhr = value;
        s->rx_pending = true;
    }

    trace_at91_ssc_receive(value, bytes);
    at91_ssc_update_dma_requests(s);
    at91_ssc_update_irq(s);
}

static void at91_ssc_pdc_tx(AT91SscState *s)
{
    unsigned bytes = at91_ssc_word_bytes(s->tfmr);

    at91_ssc_promote_tx(s);
    while (s->txten && s->tx_enabled && s->tcr) {
        uint8_t buffer[4];
        uint32_t value;

        address_space_read(&address_space_memory, s->tpr,
                           MEMTXATTRS_UNSPECIFIED, buffer, bytes);
        value = ldn_le_p(buffer, bytes);
        s->tpr += bytes;
        s->tcr--;
        trace_at91_ssc_transmit(value, bytes);
        if ((s->rfmr & RFMR_LOOP) && s->rx_enabled) {
            at91_ssc_receive_word(s, value);
        }

        if (!s->tcr) {
            s->endtx_event = true;
            at91_ssc_promote_tx(s);
        }
    }
    at91_ssc_update_irq(s);
}

static void at91_ssc_reset(DeviceState *dev)
{
    AT91SscState *s = AT91_SSC(dev);

    s->cmr = s->rcmr = s->rfmr = s->tcmr = s->tfmr = 0;
    s->rhr = s->rshr = s->tshr = s->rc0r = s->rc1r = 0;
    s->imr = 0;
    s->rpr = s->rcr = s->tpr = s->tcr = 0;
    s->rnpr = s->rncr = s->tnpr = s->tncr = 0;
    s->rx_enabled = s->tx_enabled = false;
    s->tx_request_rearm = false;
    if (s->tx_request_bh) {
        qemu_bh_cancel(s->tx_request_bh);
    }
    at91_ssc_update_dma_requests(s);
    s->rxten = s->txten = false;
    s->rx_pending = s->overrun = false;
    s->endrx_event = s->endtx_event = false;
    at91_ssc_update_irq(s);
}

static uint64_t at91_ssc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91SscState *s = opaque;
    uint32_t value = 0;

    switch (offset) {
    case SSC_CMR:
        value = s->cmr;
        break;
    case SSC_RCMR:
        value = s->rcmr;
        break;
    case SSC_RFMR:
        value = s->rfmr;
        break;
    case SSC_TCMR:
        value = s->tcmr;
        break;
    case SSC_TFMR:
        value = s->tfmr;
        break;
    case SSC_RHR:
        value = s->rhr;
        s->rx_pending = false;
        s->overrun = false;
        at91_ssc_update_dma_requests(s);
        at91_ssc_update_irq(s);
        break;
    case SSC_RSHR:
        value = s->rshr;
        break;
    case SSC_TSHR:
        value = s->tshr;
        break;
    case SSC_RC0R:
        value = s->rc0r;
        break;
    case SSC_RC1R:
        value = s->rc1r;
        break;
    case SSC_SR:
        value = at91_ssc_status(s);
        s->endrx_event = false;
        s->endtx_event = false;
        s->overrun = false;
        at91_ssc_update_irq(s);
        break;
    case SSC_IMR:
        value = s->imr;
        break;
    case SSC_VERSION:
        value = SSC_VERSION_VAL;
        break;
    case PDC_RPR:
        value = s->rpr;
        break;
    case PDC_RCR:
        value = s->rcr;
        break;
    case PDC_TPR:
        value = s->tpr;
        break;
    case PDC_TCR:
        value = s->tcr;
        break;
    case PDC_RNPR:
        value = s->rnpr;
        break;
    case PDC_RNCR:
        value = s->rncr;
        break;
    case PDC_TNPR:
        value = s->tnpr;
        break;
    case PDC_TNCR:
        value = s->tncr;
        break;
    case PDC_PTSR:
        value = (s->rxten ? PDC_RXTEN : 0) |
                (s->txten ? PDC_TXTEN : 0);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "at91-ssc: unimplemented read at 0x%03" HWADDR_PRIx
                      "\n", offset);
        break;
    }
    return value;
}

static void at91_ssc_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    AT91SscState *s = opaque;
    uint32_t val = value;

    switch (offset) {
    case SSC_CR:
        if (val & CR_SWRST) {
            at91_ssc_reset(DEVICE(s));
            break;
        }
        if (val & CR_RXEN) {
            s->rx_enabled = true;
        }
        if (val & CR_RXDIS) {
            s->rx_enabled = false;
        }
        if (val & CR_TXEN) {
            s->tx_enabled = true;
        }
        if (val & CR_TXDIS) {
            s->tx_enabled = false;
            s->tx_request_rearm = false;
            qemu_bh_cancel(s->tx_request_bh);
        }
        at91_ssc_update_dma_requests(s);
        at91_ssc_pdc_tx(s);
        break;
    case SSC_CMR:
        s->cmr = val & 0xfff;
        break;
    case SSC_RCMR:
        s->rcmr = val;
        break;
    case SSC_RFMR:
        s->rfmr = val;
        break;
    case SSC_TCMR:
        s->tcmr = val;
        break;
    case SSC_TFMR:
        s->tfmr = val;
        break;
    case SSC_THR:
        trace_at91_ssc_transmit(val, size);
        if (s->tx_enabled) {
            if (s->rfmr & RFMR_LOOP) {
                at91_ssc_receive_word(s, val);
            }
            at91_ssc_rearm_tx_request(s);
        }
        break;
    case SSC_RSHR:
        s->rshr = val & 0xffff;
        break;
    case SSC_TSHR:
        s->tshr = val & 0xffff;
        break;
    case SSC_RC0R:
        s->rc0r = val & 0xffff;
        break;
    case SSC_RC1R:
        s->rc1r = val & 0xffff;
        break;
    case SSC_IER:
        s->imr |= val;
        at91_ssc_update_irq(s);
        break;
    case SSC_IDR:
        s->imr &= ~val;
        at91_ssc_update_irq(s);
        break;
    case PDC_RPR:
        s->rpr = val;
        break;
    case PDC_RCR:
        s->rcr = val;
        at91_ssc_update_irq(s);
        break;
    case PDC_TPR:
        s->tpr = val;
        break;
    case PDC_TCR:
        s->tcr = val;
        at91_ssc_pdc_tx(s);
        break;
    case PDC_RNPR:
        s->rnpr = val;
        break;
    case PDC_RNCR:
        s->rncr = val;
        at91_ssc_promote_rx(s);
        at91_ssc_update_irq(s);
        break;
    case PDC_TNPR:
        s->tnpr = val;
        break;
    case PDC_TNCR:
        s->tncr = val;
        at91_ssc_pdc_tx(s);
        break;
    case PDC_PTCR:
        if (val & PDC_RXTEN) {
            s->rxten = true;
            at91_ssc_promote_rx(s);
        }
        if (val & PDC_RXTDIS) {
            s->rxten = false;
        }
        if (val & PDC_TXTEN) {
            s->txten = true;
        }
        if (val & PDC_TXTDIS) {
            s->txten = false;
        }
        at91_ssc_pdc_tx(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "at91-ssc: unimplemented write at 0x%03" HWADDR_PRIx
                      " value 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps at91_ssc_ops = {
    .read = at91_ssc_read,
    .write = at91_ssc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static int at91_ssc_post_load(void *opaque, int version_id)
{
    AT91SscState *s = opaque;

    at91_ssc_update_dma_requests(s);
    if (s->tx_request_rearm) {
        qemu_bh_schedule(s->tx_request_bh);
    }
    at91_ssc_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_at91_ssc = {
    .name = "at91-ssc",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = at91_ssc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cmr, AT91SscState),
        VMSTATE_UINT32(rcmr, AT91SscState),
        VMSTATE_UINT32(rfmr, AT91SscState),
        VMSTATE_UINT32(tcmr, AT91SscState),
        VMSTATE_UINT32(tfmr, AT91SscState),
        VMSTATE_UINT32(rhr, AT91SscState),
        VMSTATE_UINT32(rshr, AT91SscState),
        VMSTATE_UINT32(tshr, AT91SscState),
        VMSTATE_UINT32(rc0r, AT91SscState),
        VMSTATE_UINT32(rc1r, AT91SscState),
        VMSTATE_UINT32(imr, AT91SscState),
        VMSTATE_UINT32(rpr, AT91SscState),
        VMSTATE_UINT32(rcr, AT91SscState),
        VMSTATE_UINT32(tpr, AT91SscState),
        VMSTATE_UINT32(tcr, AT91SscState),
        VMSTATE_UINT32(rnpr, AT91SscState),
        VMSTATE_UINT32(rncr, AT91SscState),
        VMSTATE_UINT32(tnpr, AT91SscState),
        VMSTATE_UINT32(tncr, AT91SscState),
        VMSTATE_BOOL(rx_enabled, AT91SscState),
        VMSTATE_BOOL(tx_enabled, AT91SscState),
        VMSTATE_BOOL(rxten, AT91SscState),
        VMSTATE_BOOL(txten, AT91SscState),
        VMSTATE_BOOL(rx_pending, AT91SscState),
        VMSTATE_BOOL(overrun, AT91SscState),
        VMSTATE_BOOL(endrx_event, AT91SscState),
        VMSTATE_BOOL(endtx_event, AT91SscState),
        VMSTATE_BOOL_V(tx_request_rearm, AT91SscState, 2),
        VMSTATE_END_OF_LIST()
    },
};

static void at91_ssc_init(Object *obj)
{
    AT91SscState *s = AT91_SSC(obj);

    memory_region_init_io(&s->iomem, obj, &at91_ssc_ops, s,
                          TYPE_AT91_SSC, SSC_IOMEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->tx_request,
                             AT91_SSC_TX_DMA_REQUEST, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->rx_request,
                             AT91_SSC_RX_DMA_REQUEST, 1);
    s->tx_request_bh = qemu_bh_new(at91_ssc_tx_request_bh, s);
}

static void at91_ssc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Atmel AT91 Synchronous Serial Controller";
    dc->vmsd = &vmstate_at91_ssc;
    device_class_set_legacy_reset(dc, at91_ssc_reset);
}

static const TypeInfo at91_ssc_info = {
    .name = TYPE_AT91_SSC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91SscState),
    .instance_init = at91_ssc_init,
    .class_init = at91_ssc_class_init,
};

static void at91_ssc_register_types(void)
{
    type_register_static(&at91_ssc_info);
}

type_init(at91_ssc_register_types)
