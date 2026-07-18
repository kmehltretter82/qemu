/*
 * Atmel/Microchip AT91 USB Device Port High Speed controller.
 *
 * This models the register/FIFO contract used by Linux's atmel_usba_udc
 * driver, including the controller's internal endpoint DMA.  A small internal
 * USBDevice object can be plugged into a QEMU host-controller bus to represent
 * a physical cable; the SAM9M10-G45-EK board exposes that as an opt-in
 * loopback connection.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/usb/at91_udphs.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "trace.h"

#define UDPHS_FIFO_IOMEM_SIZE    (512 * KiB)
#define UDPHS_CTRL_IOMEM_SIZE    0x400
#define UDPHS_NUM_EPS            7
#define UDPHS_FIFO_SIZE          1024
#define UDPHS_CTRL_BUFFER_SIZE   4096

/* Global registers. */
#define UDPHS_CTRL               0x000
#define UDPHS_FNUM               0x004
#define UDPHS_INT_ENB            0x010
#define UDPHS_INT_STA            0x014
#define UDPHS_INT_CLR            0x018
#define UDPHS_EPT_RST            0x01c
#define UDPHS_TST                0x0e0

#define CTRL_DEV_ADDR_MASK       0x7f
#define CTRL_FADDR_EN            BIT(7)
#define CTRL_EN_USBA             BIT(8)
#define CTRL_DETACH              BIT(9)
#define CTRL_REMOTE_WAKE_UP      BIT(10)
#define CTRL_PULLD_DIS           BIT(11)

#define INT_HIGH_SPEED           BIT(0)
#define INT_END_OF_RESET         BIT(4)
#define INT_EPT(ep)              BIT(8 + (ep))
#define INT_DMA(ep)              BIT(24 + (ep))

/* Endpoint register block. */
#define UDPHS_EPT_BASE(ep)       (0x100 + (ep) * 0x20)
#define EPT_CFG                  0x00
#define EPT_CTL_ENB              0x04
#define EPT_CTL_DIS              0x08
#define EPT_CTL                  0x0c
#define EPT_SET_STA              0x14
#define EPT_CLR_STA              0x18
#define EPT_STA                  0x1c

#define EPT_CFG_SIZE_MASK        0x7
#define EPT_CFG_DIR_IN           BIT(3)
#define EPT_CFG_TYPE_SHIFT       4
#define EPT_CFG_TYPE_MASK        (0x3 << EPT_CFG_TYPE_SHIFT)
#define EPT_CFG_MAPPED           BIT(31)

#define EPT_ENABLE               BIT(0)
#define EPT_AUTO_VALID           BIT(1)
#define EPT_INTDIS_DMA           BIT(3)

#define EPT_FORCE_STALL          BIT(5)
#define EPT_TOGGLE_CLR           BIT(6)
#define EPT_ERR_OVERFLOW         BIT(8)
#define EPT_RX_BK_RDY            BIT(9)
#define EPT_TX_COMPLETE          BIT(10)
#define EPT_TX_PK_RDY            BIT(11)
#define EPT_RX_SETUP             BIT(12)
#define EPT_STALL_SENT           BIT(13)
#define EPT_NAK_IN               BIT(14)
#define EPT_NAK_OUT              BIT(15)
#define EPT_BUSY_BANKS_SHIFT     18
#define EPT_BUSY_BANKS_MASK      (0x3 << EPT_BUSY_BANKS_SHIFT)
#define EPT_BYTE_COUNT_SHIFT     20
#define EPT_BYTE_COUNT_MASK      (0x7ff << EPT_BYTE_COUNT_SHIFT)
#define EPT_SHORT_PACKET         BIT(31)

#define EPT_DIRECT_IRQ_MASK      (EPT_ERR_OVERFLOW | EPT_RX_BK_RDY | \
                                  EPT_TX_COMPLETE | EPT_RX_SETUP | \
                                  EPT_STALL_SENT | EPT_NAK_IN | \
                                  EPT_NAK_OUT | EPT_SHORT_PACKET)

/* Internal DMA register block. */
#define UDPHS_DMA_BASE(ep)       (0x300 + (ep) * 0x10)
#define DMA_NXT_DSC              0x00
#define DMA_ADDRESS              0x04
#define DMA_CONTROL              0x08
#define DMA_STATUS               0x0c

#define DMA_CH_EN                BIT(0)
#define DMA_LINK                 BIT(1)
#define DMA_END_TR_EN            BIT(2)
#define DMA_END_BUF_EN           BIT(3)
#define DMA_END_TR_IE            BIT(4)
#define DMA_END_BUF_IE           BIT(5)
#define DMA_DESC_LOAD_IE         BIT(6)
#define DMA_CH_ACTIVE            BIT(1)
#define DMA_END_TR_ST            BIT(4)
#define DMA_END_BUF_ST           BIT(5)
#define DMA_DESC_LOAD_ST         BIT(6)
#define DMA_EVENT_MASK           (DMA_END_TR_ST | DMA_END_BUF_ST | \
                                  DMA_DESC_LOAD_ST)
#define DMA_BUF_LEN_SHIFT        16

typedef struct AT91UDPHSEp {
    uint32_t cfg;
    uint32_t ctl;
    uint32_t sta;
    uint32_t dma_next;
    uint32_t dma_addr;
    uint32_t dma_control;
    uint32_t dma_events;
    uint32_t dma_remaining;
    uint16_t fifo_len;
    uint8_t fifo[UDPHS_FIFO_SIZE];
} AT91UDPHSEp;

struct AT91UDPHSState {
    SysBusDevice parent_obj;

    MemoryRegion fifo_iomem;
    MemoryRegion ctrl_iomem;
    qemu_irq irq;

    uint32_t ctrl;
    uint32_t int_enb;
    uint32_t int_sta;
    uint32_t tst;
    AT91UDPHSEp ep[UDPHS_NUM_EPS];

    USBDevice *usb;
};

typedef enum AT91UDPHSControlPhase {
    CTRL_PHASE_IDLE,
    CTRL_PHASE_IN_DATA,
    CTRL_PHASE_IN_STATUS,
    CTRL_PHASE_IN_STATUS_WAIT,
    CTRL_PHASE_OUT_DATA,
    CTRL_PHASE_OUT_STATUS,
} AT91UDPHSControlPhase;

typedef struct AT91UDPHSUSBState {
    USBDevice parent_obj;

    AT91UDPHSState *controller;
    uint8_t setup[8];
    uint8_t data[UDPHS_CTRL_BUFFER_SIZE];
    uint16_t expected;
    uint16_t length;
    uint16_t offset;
    uint8_t phase;
} AT91UDPHSUSBState;

#define AT91_UDPHS_USB(obj) \
    OBJECT_CHECK(AT91UDPHSUSBState, (obj), TYPE_AT91_UDPHS_USB)

static unsigned udphs_ep_maxpacket(const AT91UDPHSEp *ep)
{
    return 8U << (ep->cfg & EPT_CFG_SIZE_MASK);
}

static bool udphs_ep_pending(const AT91UDPHSEp *ep)
{
    if (ep->sta & ep->ctl & EPT_DIRECT_IRQ_MASK) {
        return true;
    }

    /* TX_PK_RDY is an interrupt-on-space condition: Linux enables this bit
     * when it wants notification that hardware consumed the current bank. */
    return (ep->ctl & EPT_TX_PK_RDY) && !(ep->sta & EPT_TX_PK_RDY);
}

static bool udphs_dma_pending(const AT91UDPHSEp *ep)
{
    return ep->dma_events & ep->dma_control & DMA_EVENT_MASK;
}

static uint32_t udphs_int_status(AT91UDPHSState *s)
{
    uint32_t status = s->int_sta;
    int i;

    for (i = 0; i < UDPHS_NUM_EPS; i++) {
        if (udphs_ep_pending(&s->ep[i])) {
            status |= INT_EPT(i);
        }
        if (i && udphs_dma_pending(&s->ep[i])) {
            status |= INT_DMA(i);
        }
    }
    return status;
}

static void udphs_update_irq(AT91UDPHSState *s)
{
    qemu_set_irq(s->irq, (udphs_int_status(s) & s->int_enb) != 0);
}

static void udphs_update_usb_ep(AT91UDPHSState *s, unsigned epnum)
{
    AT91UDPHSEp *ep;
    int pid;
    unsigned type;

    if (!s->usb || epnum == 0 || epnum >= UDPHS_NUM_EPS) {
        return;
    }

    ep = &s->ep[epnum];
    usb_ep_set_type(s->usb, USB_TOKEN_IN, epnum,
                    USB_ENDPOINT_XFER_INVALID);
    usb_ep_set_type(s->usb, USB_TOKEN_OUT, epnum,
                    USB_ENDPOINT_XFER_INVALID);

    if (!(ep->cfg & EPT_CFG_MAPPED)) {
        return;
    }

    pid = (ep->cfg & EPT_CFG_DIR_IN) ? USB_TOKEN_IN : USB_TOKEN_OUT;
    type = (ep->cfg & EPT_CFG_TYPE_MASK) >> EPT_CFG_TYPE_SHIFT;
    usb_ep_set_type(s->usb, pid, epnum, type);
    usb_ep_set_ifnum(s->usb, pid, epnum, 0);
    usb_ep_set_max_packet_size(s->usb, pid, epnum,
                               udphs_ep_maxpacket(ep));
}

static void udphs_reset_ep(AT91UDPHSState *s, unsigned epnum)
{
    AT91UDPHSEp *ep = &s->ep[epnum];

    memset(ep, 0, sizeof(*ep));
    udphs_update_usb_ep(s, epnum);
}

static void udphs_update_attach(AT91UDPHSState *s)
{
    bool connect = (s->ctrl & CTRL_EN_USBA) &&
                   !(s->ctrl & CTRL_DETACH);
    Error *err = NULL;

    if (!s->usb || !s->usb->port) {
        return;
    }
    if (connect && !s->usb->attached) {
        usb_device_attach(s->usb, &err);
        if (err) {
            error_report_err(err);
        }
    } else if (!connect && s->usb->attached) {
        usb_device_detach(s->usb);
    }
}

static void udphs_set_rx_packet(AT91UDPHSState *s, unsigned epnum,
                                const uint8_t *buf, size_t len,
                                bool setup, bool short_packet)
{
    AT91UDPHSEp *ep = &s->ep[epnum];

    len = MIN(len, sizeof(ep->fifo));
    if (len) {
        memcpy(ep->fifo, buf, len);
    }
    ep->fifo_len = len;
    ep->sta &= ~(EPT_RX_BK_RDY | EPT_RX_SETUP | EPT_BUSY_BANKS_MASK |
                 EPT_BYTE_COUNT_MASK | EPT_SHORT_PACKET |
                 EPT_TX_COMPLETE);
    ep->sta |= setup ? EPT_RX_SETUP : EPT_RX_BK_RDY;
    ep->sta |= (uint32_t)len << EPT_BYTE_COUNT_SHIFT;
    if (!setup) {
        ep->sta |= 1U << EPT_BUSY_BANKS_SHIFT;
    }
    if (short_packet) {
        ep->sta |= EPT_SHORT_PACKET;
    }
    udphs_update_irq(s);
}

static size_t udphs_take_tx_packet(AT91UDPHSState *s, unsigned epnum,
                                   uint8_t *buf, size_t maxlen)
{
    AT91UDPHSEp *ep = &s->ep[epnum];
    size_t len;

    if (!(ep->sta & EPT_TX_PK_RDY)) {
        return SIZE_MAX;
    }

    len = MIN((size_t)ep->fifo_len, maxlen);
    if (len) {
        memcpy(buf, ep->fifo, len);
    }
    ep->fifo_len = 0;
    ep->sta &= ~EPT_TX_PK_RDY;
    udphs_update_irq(s);
    return len;
}

static void udphs_dma_finish(AT91UDPHSState *s, unsigned epnum,
                             uint32_t events)
{
    AT91UDPHSEp *ep = &s->ep[epnum];

    ep->dma_control &= ~DMA_CH_EN;
    ep->dma_events |= events;
    udphs_update_irq(s);
}

static uint32_t udphs_dma_status(const AT91UDPHSEp *ep)
{
    uint32_t status = ep->dma_events |
                      ((ep->dma_remaining & 0xffff) << DMA_BUF_LEN_SHIFT);

    if (ep->dma_control & DMA_CH_EN) {
        status |= DMA_CH_ACTIVE;
    }
    return status;
}

static uint64_t udphs_fifo_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91UDPHSState *s = opaque;
    unsigned epnum = offset >> 16;
    unsigned local = offset & 0xffff;
    uint64_t value = 0;
    unsigned i;

    if (epnum >= UDPHS_NUM_EPS || local + size > UDPHS_FIFO_SIZE) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        value |= (uint64_t)s->ep[epnum].fifo[local + i] << (i * 8);
    }
    return value;
}

static void udphs_fifo_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    AT91UDPHSState *s = opaque;
    unsigned epnum = offset >> 16;
    unsigned local = offset & 0xffff;
    AT91UDPHSEp *ep;
    unsigned i;

    if (epnum >= UDPHS_NUM_EPS || local + size > UDPHS_FIFO_SIZE) {
        return;
    }
    ep = &s->ep[epnum];
    for (i = 0; i < size; i++) {
        ep->fifo[local + i] = value >> (i * 8);
    }
    ep->fifo_len = MAX(ep->fifo_len, local + size);
}

static const MemoryRegionOps udphs_fifo_ops = {
    .read = udphs_fifo_read,
    .write = udphs_fifo_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static bool udphs_decode_ep(hwaddr offset, unsigned *epnum,
                            hwaddr *reg)
{
    if (offset < UDPHS_EPT_BASE(0) || offset >= UDPHS_EPT_BASE(UDPHS_NUM_EPS)) {
        return false;
    }
    *epnum = (offset - UDPHS_EPT_BASE(0)) / 0x20;
    *reg = (offset - UDPHS_EPT_BASE(0)) % 0x20;
    return true;
}

static bool udphs_decode_dma(hwaddr offset, unsigned *epnum,
                             hwaddr *reg)
{
    if (offset < UDPHS_DMA_BASE(1) ||
        offset >= UDPHS_DMA_BASE(UDPHS_NUM_EPS)) {
        return false;
    }
    *epnum = (offset - UDPHS_DMA_BASE(0)) / 0x10;
    *reg = (offset - UDPHS_DMA_BASE(0)) % 0x10;
    return *epnum < UDPHS_NUM_EPS;
}

static uint64_t udphs_ctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91UDPHSState *s = opaque;
    AT91UDPHSEp *ep;
    unsigned epnum;
    hwaddr reg;
    uint32_t value;

    switch (offset) {
    case UDPHS_CTRL:
        return s->ctrl;
    case UDPHS_FNUM: {
        uint64_t us = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000;
        return ((us / 125) & 0x7) | (((us / 1000) & 0x7ff) << 3);
    }
    case UDPHS_INT_ENB:
        return s->int_enb;
    case UDPHS_INT_STA:
        return udphs_int_status(s);
    case UDPHS_EPT_RST:
        return 0;
    case UDPHS_TST:
        return s->tst;
    default:
        break;
    }

    if (udphs_decode_ep(offset, &epnum, &reg)) {
        ep = &s->ep[epnum];
        switch (reg) {
        case EPT_CFG:
            return ep->cfg;
        case EPT_CTL:
            return ep->ctl;
        case EPT_STA:
            return ep->sta;
        default:
            return 0;
        }
    }

    if (udphs_decode_dma(offset, &epnum, &reg)) {
        ep = &s->ep[epnum];
        switch (reg) {
        case DMA_NXT_DSC:
            return ep->dma_next;
        case DMA_ADDRESS:
            return ep->dma_addr;
        case DMA_CONTROL:
            return ep->dma_control;
        case DMA_STATUS:
            value = udphs_dma_status(ep);
            ep->dma_events &= ~DMA_EVENT_MASK;
            udphs_update_irq(s);
            return value;
        default:
            return 0;
        }
    }

    qemu_log_mask(LOG_UNIMP,
                  "at91-udphs: unimplemented read at 0x%03" HWADDR_PRIx
                  "\n", offset);
    return 0;
}

static void udphs_ep_clear_status(AT91UDPHSEp *ep, uint32_t value)
{
    uint32_t clear = value & ~EPT_TOGGLE_CLR;

    ep->sta &= ~clear;
    if (value & (EPT_RX_BK_RDY | EPT_RX_SETUP)) {
        ep->sta &= ~(EPT_RX_BK_RDY | EPT_RX_SETUP | EPT_BUSY_BANKS_MASK |
                     EPT_BYTE_COUNT_MASK | EPT_SHORT_PACKET);
        ep->fifo_len = 0;
    }
}

static void udphs_ctrl_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    AT91UDPHSState *s = opaque;
    AT91UDPHSEp *ep;
    unsigned epnum;
    hwaddr reg;
    uint32_t val = value;
    int i;

    switch (offset) {
    case UDPHS_CTRL:
        s->ctrl = val;
        udphs_update_attach(s);
        return;
    case UDPHS_INT_ENB:
        s->int_enb = val;
        udphs_update_irq(s);
        return;
    case UDPHS_INT_CLR:
        s->int_sta &= ~val;
        udphs_update_irq(s);
        return;
    case UDPHS_EPT_RST:
        for (i = 0; i < UDPHS_NUM_EPS; i++) {
            if (val & BIT(i)) {
                udphs_reset_ep(s, i);
            }
        }
        udphs_update_irq(s);
        return;
    case UDPHS_TST:
        s->tst = val;
        return;
    default:
        break;
    }

    if (udphs_decode_ep(offset, &epnum, &reg)) {
        ep = &s->ep[epnum];
        switch (reg) {
        case EPT_CFG:
            ep->cfg = val | EPT_CFG_MAPPED;
            udphs_update_usb_ep(s, epnum);
            break;
        case EPT_CTL_ENB:
            ep->ctl |= val;
            break;
        case EPT_CTL_DIS:
            ep->ctl &= ~val;
            break;
        case EPT_SET_STA:
            ep->sta |= val & ~EPT_TOGGLE_CLR;
            break;
        case EPT_CLR_STA:
            udphs_ep_clear_status(ep, val);
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "at91-udphs: unimplemented endpoint write ep%u "
                          "reg 0x%02" HWADDR_PRIx " value 0x%08x\n",
                          epnum, reg, val);
            break;
        }
        trace_at91_udphs_ep_write(epnum, reg, val, ep->sta, ep->ctl);
        udphs_update_irq(s);
        return;
    }

    if (udphs_decode_dma(offset, &epnum, &reg)) {
        ep = &s->ep[epnum];
        switch (reg) {
        case DMA_NXT_DSC:
            ep->dma_next = val;
            break;
        case DMA_ADDRESS:
            ep->dma_addr = val;
            break;
        case DMA_CONTROL:
            ep->dma_control = val;
            ep->dma_events = 0;
            if (val & DMA_CH_EN) {
                ep->dma_remaining = val >> DMA_BUF_LEN_SHIFT;
                if (!ep->dma_remaining) {
                    ep->dma_remaining = 0x10000;
                }
            } else {
                ep->dma_remaining = 0;
            }
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "at91-udphs: unimplemented DMA write ep%u "
                          "reg 0x%02" HWADDR_PRIx " value 0x%08x\n",
                          epnum, reg, val);
            break;
        }
        udphs_update_irq(s);
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  "at91-udphs: unimplemented write at 0x%03" HWADDR_PRIx
                  " value 0x%08x\n", offset, val);
}

static const MemoryRegionOps udphs_ctrl_ops = {
    .read = udphs_ctrl_read,
    .write = udphs_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static bool udphs_usb_ready(AT91UDPHSUSBState *u)
{
    AT91UDPHSState *s = u->controller;

    return s && (s->ctrl & CTRL_EN_USBA) &&
           !(s->ctrl & CTRL_DETACH) &&
           (s->ep[0].ctl & EPT_ENABLE);
}

static void udphs_control_start(AT91UDPHSUSBState *u, int request,
                                int value, int index, int length,
                                const uint8_t *data)
{
    AT91UDPHSState *s = u->controller;
    AT91UDPHSEp *ep = &s->ep[0];

    u->setup[0] = request >> 8;
    u->setup[1] = request;
    u->setup[2] = value;
    u->setup[3] = value >> 8;
    u->setup[4] = index;
    u->setup[5] = index >> 8;
    u->setup[6] = length;
    u->setup[7] = length >> 8;
    u->expected = length;
    u->length = 0;
    u->offset = 0;
    if (!(u->setup[0] & USB_DIR_IN) && length) {
        memcpy(u->data, data, length);
    }

    /* A new SETUP token clears an endpoint-zero protocol stall and aborts
     * any previous control transfer status. */
    ep->sta &= ~(EPT_FORCE_STALL | EPT_TX_COMPLETE | EPT_TX_PK_RDY |
                 EPT_RX_BK_RDY | EPT_SHORT_PACKET);
    ep->fifo_len = 0;
    udphs_set_rx_packet(s, 0, u->setup, sizeof(u->setup), true, true);

    if (u->setup[0] & USB_DIR_IN) {
        u->phase = CTRL_PHASE_IN_DATA;
    } else if (length) {
        u->phase = CTRL_PHASE_OUT_DATA;
    } else {
        u->phase = CTRL_PHASE_OUT_STATUS;
    }
    trace_at91_udphs_setup(u->setup[0], u->setup[1], value, index, length);
}

static bool udphs_same_control(const AT91UDPHSUSBState *u, int request,
                               int value, int index, int length)
{
    return u->setup[0] == (uint8_t)(request >> 8) &&
           u->setup[1] == (uint8_t)request &&
           u->setup[2] == (uint8_t)value &&
           u->setup[3] == (uint8_t)(value >> 8) &&
           u->setup[4] == (uint8_t)index &&
           u->setup[5] == (uint8_t)(index >> 8) &&
           u->expected == length;
}

static void udphs_usb_handle_control(USBDevice *dev, USBPacket *p,
                                     int request, int value, int index,
                                     int length, uint8_t *data)
{
    AT91UDPHSUSBState *u = AT91_UDPHS_USB(dev);
    AT91UDPHSState *s = u->controller;
    AT91UDPHSEp *ep;
    size_t packet_len;
    size_t maxpacket;

    if (!udphs_usb_ready(u)) {
        p->status = USB_RET_NAK;
        return;
    }
    ep = &s->ep[0];
    maxpacket = udphs_ep_maxpacket(ep);

    static const char *const phase_name[] = {
        [CTRL_PHASE_IDLE] = "idle",
        [CTRL_PHASE_IN_DATA] = "in-data",
        [CTRL_PHASE_IN_STATUS] = "in-status",
        [CTRL_PHASE_IN_STATUS_WAIT] = "in-status-wait",
        [CTRL_PHASE_OUT_DATA] = "out-data",
        [CTRL_PHASE_OUT_STATUS] = "out-status",
    };
    trace_at91_udphs_control(phase_name[u->phase], ep->sta, ep->ctl,
                             u->length, u->offset);

    if (u->phase == CTRL_PHASE_IDLE ||
        !udphs_same_control(u, request, value, index, length)) {
        udphs_control_start(u, request, value, index, length, data);
        if (u->setup[0] & USB_DIR_IN) {
            p->status = USB_RET_NAK;
        } else {
            p->status = USB_RET_NAK;
        }
        return;
    }

    if (ep->sta & EPT_FORCE_STALL) {
        u->phase = CTRL_PHASE_IDLE;
        p->status = USB_RET_STALL;
        return;
    }

    switch (u->phase) {
    case CTRL_PHASE_IN_DATA:
        packet_len = udphs_take_tx_packet(s, 0, u->data + u->length,
                                          sizeof(u->data) - u->length);
        if (packet_len == SIZE_MAX) {
            break;
        }
        u->length += packet_len;
        if (packet_len < maxpacket || u->length >= u->expected) {
            ep->sta |= EPT_TX_COMPLETE;
            u->phase = CTRL_PHASE_IN_STATUS;
            udphs_update_irq(s);
        }
        break;
    case CTRL_PHASE_IN_STATUS:
        if (!(ep->sta & EPT_TX_COMPLETE) &&
            (ep->ctl & EPT_RX_BK_RDY) &&
            !(ep->sta & EPT_RX_BK_RDY)) {
            udphs_set_rx_packet(s, 0, NULL, 0, false, true);
            u->phase = CTRL_PHASE_IN_STATUS_WAIT;
        }
        break;
    case CTRL_PHASE_IN_STATUS_WAIT:
        if (!(ep->sta & EPT_RX_BK_RDY)) {
            p->actual_length = MIN((unsigned)u->length, (unsigned)length);
            memcpy(data, u->data, p->actual_length);
            u->phase = CTRL_PHASE_IDLE;
            return;
        }
        break;
    case CTRL_PHASE_OUT_DATA:
        if ((ep->ctl & EPT_RX_BK_RDY) &&
            !(ep->sta & EPT_RX_BK_RDY)) {
            packet_len = MIN((size_t)(u->expected - u->offset), maxpacket);
            udphs_set_rx_packet(s, 0, u->data + u->offset, packet_len,
                                false, packet_len < maxpacket);
            u->offset += packet_len;
            if (u->offset == u->expected) {
                u->phase = CTRL_PHASE_OUT_STATUS;
            }
        }
        break;
    case CTRL_PHASE_OUT_STATUS:
        packet_len = udphs_take_tx_packet(s, 0, NULL, 0);
        if (packet_len != SIZE_MAX) {
            /* QEMU's generic EP0 core finishes an OUT request as soon as
             * handle_control() returns from the status-IN token; unlike a
             * SETUP token it cannot retain a NAK here.  The guest has already
             * committed the status packet, so complete the host token now and
             * let its TX_COMPLETE IRQ retire the guest request afterward. */
            ep->sta |= EPT_TX_COMPLETE;
            udphs_update_irq(s);
            if (u->setup[0] == USB_RECIP_DEVICE &&
                u->setup[1] == USB_REQ_SET_ADDRESS) {
                dev->addr = u->setup[2] | (u->setup[3] << 8);
            }
            u->phase = CTRL_PHASE_IDLE;
            p->actual_length = 0;
            return;
        }
        break;
    default:
        g_assert_not_reached();
    }

    p->status = USB_RET_NAK;
}

static void udphs_usb_dma_in(AT91UDPHSUSBState *u, USBPacket *p,
                             unsigned epnum)
{
    AT91UDPHSState *s = u->controller;
    AT91UDPHSEp *ep = &s->ep[epnum];
    size_t len = MIN((size_t)ep->dma_remaining, usb_packet_size(p));
    g_autofree uint8_t *buf = g_malloc(len ? len : 1);

    if (len) {
        address_space_read(&address_space_memory, ep->dma_addr,
                           MEMTXATTRS_UNSPECIFIED, buf, len);
        usb_packet_copy(p, buf, len);
        ep->dma_addr += len;
        ep->dma_remaining -= len;
    }
    trace_at91_udphs_data(epnum, "in", len, true);
    if (!ep->dma_remaining) {
        udphs_dma_finish(s, epnum, DMA_END_BUF_ST);
    } else if (usb_packet_size(p) % udphs_ep_maxpacket(ep)) {
        /* The device streams full max-packet-size packets while DMA data
         * remains; a host transfer ending in a partial packet cannot absorb
         * the next one.  Real hardware overflows the host buffer here. */
        p->status = USB_RET_BABBLE;
    }
}

static void udphs_usb_dma_out(AT91UDPHSUSBState *u, USBPacket *p,
                              unsigned epnum)
{
    AT91UDPHSState *s = u->controller;
    AT91UDPHSEp *ep = &s->ep[epnum];
    size_t offered = usb_packet_size(p);
    size_t len = MIN((size_t)ep->dma_remaining, offered);
    unsigned maxpacket = udphs_ep_maxpacket(ep);
    uint32_t events = 0;
    g_autofree uint8_t *buf = g_malloc(len ? len : 1);

    if (len) {
        usb_packet_copy(p, buf, len);
        address_space_write(&address_space_memory, ep->dma_addr,
                            MEMTXATTRS_UNSPECIFIED, buf, len);
        ep->dma_addr += len;
        ep->dma_remaining -= len;
    }

    if (!ep->dma_remaining) {
        events |= DMA_END_BUF_ST;
    }
    if ((ep->dma_control & DMA_END_TR_EN) &&
        (offered == 0 || offered % maxpacket)) {
        events |= DMA_END_TR_ST;
    }
    trace_at91_udphs_data(epnum, "out", len, true);
    if (events) {
        udphs_dma_finish(s, epnum, events);
    }
}

static void udphs_usb_handle_data(USBDevice *dev, USBPacket *p)
{
    AT91UDPHSUSBState *u = AT91_UDPHS_USB(dev);
    AT91UDPHSState *s = u->controller;
    unsigned epnum = p->ep->nr;
    AT91UDPHSEp *ep;
    bool in;
    size_t len;
    g_autofree uint8_t *buf = NULL;

    if (!s || epnum == 0 || epnum >= UDPHS_NUM_EPS ||
        !(s->ctrl & CTRL_EN_USBA) || (s->ctrl & CTRL_DETACH)) {
        p->status = USB_RET_NAK;
        return;
    }
    ep = &s->ep[epnum];
    in = p->pid == USB_TOKEN_IN;

    if (!(ep->ctl & EPT_ENABLE) ||
        in != !!(ep->cfg & EPT_CFG_DIR_IN)) {
        p->status = USB_RET_NAK;
        return;
    }
    if (ep->sta & EPT_FORCE_STALL) {
        p->status = USB_RET_STALL;
        return;
    }

    if (ep->dma_control & DMA_CH_EN) {
        if (in) {
            udphs_usb_dma_in(u, p, epnum);
        } else {
            udphs_usb_dma_out(u, p, epnum);
        }
        return;
    }

    if (in) {
        buf = g_malloc(MIN(usb_packet_size(p),
                           (size_t)UDPHS_FIFO_SIZE) ?: 1);
        len = udphs_take_tx_packet(s, epnum, buf,
                                   MIN(usb_packet_size(p),
                                       (size_t)UDPHS_FIFO_SIZE));
        if (len == SIZE_MAX) {
            p->status = USB_RET_NAK;
            return;
        }
        usb_packet_copy(p, buf, len);
        trace_at91_udphs_data(epnum, "in", len, false);
    } else {
        if (!(ep->ctl & EPT_RX_BK_RDY) || (ep->sta & EPT_RX_BK_RDY)) {
            p->status = USB_RET_NAK;
            return;
        }
        len = MIN(usb_packet_size(p), (size_t)UDPHS_FIFO_SIZE);
        buf = g_malloc(len ? len : 1);
        if (len) {
            usb_packet_copy(p, buf, len);
        }
        udphs_set_rx_packet(s, epnum, buf, len, false,
                            len < udphs_ep_maxpacket(ep));
        trace_at91_udphs_data(epnum, "out", len, false);
    }
}

static void udphs_usb_handle_reset(USBDevice *dev)
{
    AT91UDPHSUSBState *u = AT91_UDPHS_USB(dev);
    AT91UDPHSState *s = u->controller;
    int i;

    u->phase = CTRL_PHASE_IDLE;
    if (!s) {
        return;
    }
    s->ctrl &= ~(CTRL_DEV_ADDR_MASK | CTRL_FADDR_EN);
    for (i = 0; i < UDPHS_NUM_EPS; i++) {
        udphs_reset_ep(s, i);
    }
    s->int_sta |= INT_END_OF_RESET;
    if (dev->speed == USB_SPEED_HIGH) {
        s->int_sta |= INT_HIGH_SPEED;
    } else {
        s->int_sta &= ~INT_HIGH_SPEED;
    }
    trace_at91_udphs_reset(dev->speed == USB_SPEED_HIGH ? "high" : "full");
    udphs_update_irq(s);
}

static void udphs_usb_realize(USBDevice *dev, Error **errp)
{
    AT91UDPHSUSBState *u = AT91_UDPHS_USB(dev);

    if (!u->controller) {
        error_setg(errp, "at91-udphs-usb requires a controller link");
        return;
    }
    if (u->controller->usb) {
        error_setg(errp, "at91-udphs already has a USB cable endpoint");
        return;
    }

    dev->speedmask = USB_SPEED_MASK_FULL | USB_SPEED_MASK_HIGH;
    dev->auto_attach = 0;
    u->controller->usb = dev;
}

static void udphs_usb_unrealize(USBDevice *dev)
{
    AT91UDPHSUSBState *u = AT91_UDPHS_USB(dev);

    if (u->controller && u->controller->usb == dev) {
        u->controller->usb = NULL;
    }
}

static int udphs_usb_post_load(void *opaque, int version_id)
{
    AT91UDPHSUSBState *u = opaque;
    int i;

    if (u->controller) {
        u->controller->usb = &u->parent_obj;
        for (i = 1; i < UDPHS_NUM_EPS; i++) {
            udphs_update_usb_ep(u->controller, i);
        }
    }
    return 0;
}

static const VMStateDescription vmstate_udphs_usb = {
    .name = "at91-udphs-usb",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = udphs_usb_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_USB_DEVICE(parent_obj, AT91UDPHSUSBState),
        VMSTATE_UINT8_ARRAY(setup, AT91UDPHSUSBState, 8),
        VMSTATE_UINT8_ARRAY(data, AT91UDPHSUSBState,
                            UDPHS_CTRL_BUFFER_SIZE),
        VMSTATE_UINT16(expected, AT91UDPHSUSBState),
        VMSTATE_UINT16(length, AT91UDPHSUSBState),
        VMSTATE_UINT16(offset, AT91UDPHSUSBState),
        VMSTATE_UINT8(phase, AT91UDPHSUSBState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property udphs_usb_properties[] = {
    DEFINE_PROP_LINK("controller", AT91UDPHSUSBState, controller,
                     TYPE_AT91_UDPHS, AT91UDPHSState *),
};

static void udphs_usb_class_init(ObjectClass *klass, const void *data)
{
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    uc->realize = udphs_usb_realize;
    uc->unrealize = udphs_usb_unrealize;
    uc->handle_reset = udphs_usb_handle_reset;
    uc->handle_control = udphs_usb_handle_control;
    uc->handle_data = udphs_usb_handle_data;
    uc->product_desc = "AT91 UDPHS guest gadget";
    dc->vmsd = &vmstate_udphs_usb;
    dc->user_creatable = false;
    device_class_set_props(dc, udphs_usb_properties);
}

static void udphs_reset(DeviceState *dev)
{
    AT91UDPHSState *s = AT91_UDPHS(dev);
    int i;

    if (s->usb && s->usb->attached) {
        usb_device_detach(s->usb);
    }
    s->ctrl = CTRL_DETACH;
    s->int_enb = 0;
    s->int_sta = 0;
    s->tst = 0;
    for (i = 0; i < UDPHS_NUM_EPS; i++) {
        udphs_reset_ep(s, i);
    }
    udphs_update_irq(s);
}

static int udphs_post_load(void *opaque, int version_id)
{
    AT91UDPHSState *s = opaque;
    int i;

    for (i = 1; i < UDPHS_NUM_EPS; i++) {
        udphs_update_usb_ep(s, i);
    }
    udphs_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_udphs_ep = {
    .name = "at91-udphs/endpoint",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, AT91UDPHSEp),
        VMSTATE_UINT32(ctl, AT91UDPHSEp),
        VMSTATE_UINT32(sta, AT91UDPHSEp),
        VMSTATE_UINT32(dma_next, AT91UDPHSEp),
        VMSTATE_UINT32(dma_addr, AT91UDPHSEp),
        VMSTATE_UINT32(dma_control, AT91UDPHSEp),
        VMSTATE_UINT32(dma_events, AT91UDPHSEp),
        VMSTATE_UINT32(dma_remaining, AT91UDPHSEp),
        VMSTATE_UINT16(fifo_len, AT91UDPHSEp),
        VMSTATE_UINT8_ARRAY(fifo, AT91UDPHSEp, UDPHS_FIFO_SIZE),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_at91_udphs = {
    .name = "at91-udphs",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = udphs_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, AT91UDPHSState),
        VMSTATE_UINT32(int_enb, AT91UDPHSState),
        VMSTATE_UINT32(int_sta, AT91UDPHSState),
        VMSTATE_UINT32(tst, AT91UDPHSState),
        VMSTATE_STRUCT_ARRAY(ep, AT91UDPHSState, UDPHS_NUM_EPS, 1,
                             vmstate_udphs_ep, AT91UDPHSEp),
        VMSTATE_END_OF_LIST()
    },
};

static void udphs_init(Object *obj)
{
    AT91UDPHSState *s = AT91_UDPHS(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->fifo_iomem, obj, &udphs_fifo_ops, s,
                          "at91-udphs-fifo", UDPHS_FIFO_IOMEM_SIZE);
    memory_region_init_io(&s->ctrl_iomem, obj, &udphs_ctrl_ops, s,
                          "at91-udphs-ctrl", UDPHS_CTRL_IOMEM_SIZE);
    /* Match the DT resource order: FIFO aperture first, registers second. */
    sysbus_init_mmio(sbd, &s->fifo_iomem);
    sysbus_init_mmio(sbd, &s->ctrl_iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void udphs_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Atmel AT91 USB Device Port High Speed controller";
    dc->vmsd = &vmstate_at91_udphs;
    device_class_set_legacy_reset(dc, udphs_reset);
}

static const TypeInfo udphs_types[] = {
    {
        .name = TYPE_AT91_UDPHS,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AT91UDPHSState),
        .instance_init = udphs_init,
        .class_init = udphs_class_init,
    }, {
        .name = TYPE_AT91_UDPHS_USB,
        .parent = TYPE_USB_DEVICE,
        .instance_size = sizeof(AT91UDPHSUSBState),
        .class_init = udphs_usb_class_init,
    },
};

static void udphs_register_types(void)
{
    type_register_static_array(udphs_types, ARRAY_SIZE(udphs_types));
}

type_init(udphs_register_types)
