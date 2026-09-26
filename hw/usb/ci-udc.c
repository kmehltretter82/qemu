/*
 * ChipIdea (ci_hdrc) USB 2.0 device controller test model.
 *
 * Emulates the device-mode register interface and the dQH/dTD DMA engine
 * as used by Linux drivers/usb/chipidea/udc.c. The USB side of the
 * gadget is a second QEMU device, "ci-udc-link", which sits on a host
 * controller bus of the same machine, so gadget and host run in one
 * guest.
 *
 * Not modelled: isochronous OUT, LPM, suspend/resume, OTG role switching,
 * ULPI. High speed only.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/usb/usb.h"
#include "hw/usb/ci-udc.h"
#include "system/dma.h"
#include "qom/object.h"

/* ID and capability registers */
#define REG_ID              0x000
#define REG_HWGENERAL       0x004
#define REG_HWHOST          0x008
#define REG_HWDEVICE        0x00c
#define REG_HWTXBUF         0x010
#define REG_HWRXBUF         0x014
#define REG_CAPLENGTH       0x100
#define REG_HCSPARAMS       0x104
#define REG_HCCPARAMS       0x108
#define REG_DCIVERSION      0x120
#define REG_DCCPARAMS       0x124
/* operational registers (non-LPM layout) */
#define REG_USBCMD          0x140
#define REG_USBSTS          0x144
#define REG_USBINTR         0x148
#define REG_FRINDEX         0x14c
#define REG_DEVICEADDR      0x154
#define REG_ENDPTLISTADDR   0x158
#define REG_PORTSC          0x184
#define REG_OTGSC           0x1a4
#define REG_USBMODE         0x1a8
#define REG_ENDPTSETUPSTAT  0x1ac
#define REG_ENDPTPRIME      0x1b0
#define REG_ENDPTFLUSH      0x1b4
#define REG_ENDPTSTAT       0x1b8
#define REG_ENDPTCOMPLETE   0x1bc
#define REG_ENDPTCTRL0      0x1c0

#define USBCMD_RS           BIT(0)
#define USBCMD_RST          BIT(1)
#define USBSTS_UI           BIT(0)
#define USBSTS_UEI          BIT(1)
#define USBSTS_PCI          BIT(2)
#define USBSTS_URI          BIT(6)
#define USBSTS_SLI          BIT(8)
#define PORTSC_CCS          BIT(0)
#define PORTSC_CSC          BIT(1)
#define PORTSC_PE           BIT(2)
#define PORTSC_PEC          BIT(3)
#define PORTSC_OCC          BIT(5)
#define PORTSC_HSP          BIT(9)
#define PORTSC_PP           BIT(12)
#define PORTSC_W1C          (PORTSC_CSC | PORTSC_PEC | PORTSC_OCC)
#define OTGSC_ID            BIT(8)
#define OTGSC_AVV           BIT(9)
#define OTGSC_ASV           BIT(10)
#define OTGSC_BSV           BIT(11)
#define OTGSC_W1C           (0x7f << 16)
#define USBMODE_CM          0x3
#define USBMODE_CM_DC       0x2
#define ENDPTCTRL_RXS       BIT(0)
#define ENDPTCTRL_TXS       BIT(16)

/* dQH / dTD layout, see Linux drivers/usb/chipidea/udc.h */
#define QH_SIZE             64
#define QH_CAP              0x00
#define QH_CURR             0x04
#define QH_NEXT             0x08
#define QH_TOKEN            0x0c
#define QH_SETUP            0x28
#define QH_MAX_PKT(cap)     (((cap) >> 16) & 0x7ff)
#define TD_NEXT             0x00
#define TD_TOKEN            0x04
#define TD_PAGE0            0x08
#define TD_TERMINATE        BIT(0)
#define TD_ADDR_MASK        0xffffffe0
#define TD_ACTIVE           BIT(7)
#define TD_HALTED           BIT(6)
#define TD_IOC              BIT(15)
#define TD_BYTES(t)         (((t) >> 16) & 0x7fff)

#define NUM_EP              4           /* per direction, incl. ep0 */
#define EP_BIT(n, in)       ((in) ? 16 + (n) : (n))

/* set QEMU_CI_UDC_DEBUG=1 in the environment to trace the model */
#define DPRINTF(fmt, ...) do {                                          \
        static int dbg = -1;                                            \
        if (dbg < 0) {                                                  \
            dbg = getenv("QEMU_CI_UDC_DEBUG") != NULL;                  \
        }                                                               \
        if (dbg) {                                                      \
            fprintf(stderr, "ci-udc: " fmt "\n", ## __VA_ARGS__);       \
        }                                                               \
    } while (0)

enum {
    CTRL_IDLE,
    CTRL_DATA,
    CTRL_STATUS,
};

struct CIUdcLinkState {
    USBDevice parent_obj;
    CIUdcState *udc;
};

struct CIUdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    CIUdcLinkState *link;

    uint32_t usbcmd;
    uint32_t usbsts;
    uint32_t usbintr;
    uint32_t deviceaddr;
    uint32_t listaddr;
    uint32_t portsc;
    uint32_t otgsc;
    uint32_t usbmode;
    uint32_t setupstat;
    uint32_t stat;
    uint32_t complete;
    uint32_t endptctrl[NUM_EP];

    /* per endpoint bit: current dTD and bytes done in it */
    uint32_t cur_td[32];
    uint32_t td_done[32];

    /* host packets waiting for dTDs, per endpoint bit (ep0 unused) */
    USBPacket *pending[32];

    /* control transfer in progress */
    USBPacket *ctrl_p;
    int ctrl_stage;
    bool ctrl_in;
    uint16_t ctrl_len;
    uint32_t ctrl_done;
    uint8_t *ctrl_data;
    uint8_t ctrl_request;
    uint16_t ctrl_value;

    /* SET_ADDRESS takes effect after the host has seen the status stage */
    QEMUTimer *addr_timer;
    uint8_t new_addr;
};

static uint32_t ci_ld32(dma_addr_t addr)
{
    uint32_t v = 0;

    dma_memory_read(&address_space_memory, addr, &v, 4, MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

static void ci_st32(dma_addr_t addr, uint32_t v)
{
    v = cpu_to_le32(v);
    dma_memory_write(&address_space_memory, addr, &v, 4, MEMTXATTRS_UNSPECIFIED);
}

static dma_addr_t ci_qh(CIUdcState *s, int bit)
{
    int n = bit & 15, in = bit >= 16;

    return (s->listaddr & ~0x7ffu) + (2 * n + in) * QH_SIZE;
}

static void ci_update_irq(CIUdcState *s)
{
    qemu_set_irq(s->irq, !!(s->usbsts & s->usbintr));
}

static void ci_set_sts(CIUdcState *s, uint32_t bits)
{
    s->usbsts |= bits;
    ci_update_irq(s);
}

/* guest address of byte @pos of the dTD at @td */
static dma_addr_t ci_td_addr(dma_addr_t td, uint32_t pos)
{
    uint32_t page0 = ci_ld32(td + TD_PAGE0);
    uint32_t off = (page0 & 0xfff) + pos;
    uint32_t page = ci_ld32(td + TD_PAGE0 + 4 * (off >> 12));

    return (page & ~0xfffu) + (off & 0xfff);
}

/* make the next dTD of endpoint @bit current, or stop the endpoint */
static void ci_load_next(CIUdcState *s, int bit, uint32_t next)
{
    dma_addr_t qh = ci_qh(s, bit);

    if (next & TD_TERMINATE) {
        s->stat &= ~BIT(bit);
        s->cur_td[bit] = 0;
        ci_st32(qh + QH_NEXT, TD_TERMINATE);
        return;
    }
    s->cur_td[bit] = next & TD_ADDR_MASK;
    s->td_done[bit] = 0;
    ci_st32(qh + QH_CURR, s->cur_td[bit]);
    ci_st32(qh + QH_NEXT, ci_ld32(s->cur_td[bit] + TD_NEXT));
    ci_st32(qh + QH_TOKEN, ci_ld32(s->cur_td[bit] + TD_TOKEN));
}

/* retire the current dTD of @bit with @done bytes transferred */
static void ci_retire_td(CIUdcState *s, int bit, uint32_t done)
{
    dma_addr_t td = s->cur_td[bit];
    uint32_t token = ci_ld32(td + TD_TOKEN);
    uint32_t left = TD_BYTES(token) - done;

    token &= ~(TD_ACTIVE | (0x7fffu << 16));
    token |= left << 16;
    ci_st32(td + TD_TOKEN, token);
    ci_st32(ci_qh(s, bit) + QH_TOKEN, token);
    if (token & TD_IOC) {
        s->complete |= BIT(bit);
        ci_set_sts(s, USBSTS_UI);
    }
    /* re-read the link: the driver may have appended dTDs meanwhile */
    ci_load_next(s, bit, ci_ld32(td + TD_NEXT));
}

/*
 * Move data between @buf and the dTDs of endpoint @bit. Returns the
 * number of bytes moved. *short_end is set when a dTD ended a transfer
 * (IN: short or zero-length dTD, OUT: the host data ended in a dTD).
 */
static uint32_t ci_xfer(CIUdcState *s, int bit, uint8_t *buf, uint32_t len,
                        bool in, bool *short_end)
{
    uint32_t moved = 0, mps;

    *short_end = false;
    mps = QH_MAX_PKT(ci_ld32(ci_qh(s, bit) + QH_CAP));
    if (!mps) {
        mps = 512;
    }

    while ((s->stat & BIT(bit)) && s->cur_td[bit]) {
        dma_addr_t td = s->cur_td[bit];
        uint32_t token = ci_ld32(td + TD_TOKEN);
        uint32_t total = TD_BYTES(token);
        uint32_t chunk, i;

        if (!(token & TD_ACTIVE)) {
            ci_load_next(s, bit, ci_ld32(td + TD_NEXT));
            continue;
        }
        if (total == 0) {
            /* zero-length dTD: ZLP (IN) or status stage */
            ci_retire_td(s, bit, 0);
            *short_end = true;
            break;
        }
        chunk = MIN(total - s->td_done[bit], len - moved);
        for (i = 0; i < chunk; ) {
            uint32_t pos = s->td_done[bit] + i;
            uint32_t n = MIN(chunk - i, 0x1000 - ((ci_ld32(td + TD_PAGE0) + pos) & 0xfff));
            dma_addr_t a = ci_td_addr(td, pos);

            if (in) {
                dma_memory_read(&address_space_memory, a, buf + moved + i, n,
                                MEMTXATTRS_UNSPECIFIED);
            } else {
                dma_memory_write(&address_space_memory, a, buf + moved + i, n,
                                 MEMTXATTRS_UNSPECIFIED);
            }
            i += n;
        }
        s->td_done[bit] += chunk;
        moved += chunk;

        if (s->td_done[bit] == total) {
            ci_retire_td(s, bit, total);
            if (in && total % mps) {
                *short_end = true;
                break;
            }
        } else if (!in && moved == len && len % mps) {
            /* short OUT packet ends the dTD early */
            ci_retire_td(s, bit, s->td_done[bit]);
            *short_end = true;
            break;
        }
        if (moved == len) {
            break;
        }
    }
    return moved;
}

/* ---- control endpoint ---- */

static void ci_ctrl_finish(CIUdcState *s, int status)
{
    USBPacket *p = s->ctrl_p;

    if (!p) {
        return;
    }
    s->ctrl_p = NULL;
    s->ctrl_stage = CTRL_IDLE;
    DPRINTF("ctrl done status %d actual %d", status, p->actual_length);
    p->status = status;
    usb_generic_async_ctrl_complete(USB_DEVICE(s->link), p);
    if (status == USB_RET_SUCCESS && s->ctrl_request == USB_REQ_SET_ADDRESS) {
        /*
         * Changing the address now would make the host controller drop
         * the status stage it is still writing back (its queue is for
         * the old address). Hosts wait a few ms before using the new one.
         */
        s->new_addr = s->ctrl_value;
        timer_mod(s->addr_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SCALE_MS);
    }
}

static void ci_set_addr(void *opaque)
{
    CIUdcState *s = opaque;

    DPRINTF("address %d", s->new_addr);
    USB_DEVICE(s->link)->addr = s->new_addr;
}

static void ci_ctrl_run(CIUdcState *s)
{
    bool short_end;
    uint32_t n;

    if (!s->ctrl_p) {
        return;
    }
    if (s->ctrl_stage == CTRL_DATA) {
        if (s->ctrl_in) {
            n = ci_xfer(s, EP_BIT(0, 1), s->ctrl_data + s->ctrl_done,
                        s->ctrl_len - s->ctrl_done, true, &short_end);
        } else {
            n = ci_xfer(s, EP_BIT(0, 0), s->ctrl_data + s->ctrl_done,
                        s->ctrl_len - s->ctrl_done, false, &short_end);
        }
        s->ctrl_done += n;
        if (s->ctrl_done == s->ctrl_len || short_end) {
            s->ctrl_p->actual_length = s->ctrl_in ? s->ctrl_done : 0;
            s->ctrl_stage = CTRL_STATUS;
        }
    }
    if (s->ctrl_stage == CTRL_STATUS) {
        /* status is in the opposite direction of the data stage */
        int bit = EP_BIT(0, !(s->ctrl_in && s->ctrl_len));

        if ((s->stat & BIT(bit)) && s->cur_td[bit]) {
            ci_xfer(s, bit, NULL, 0, bit >= 16, &short_end);
            ci_ctrl_finish(s, USB_RET_SUCCESS);
        }
    }
}

/* ---- data endpoints ---- */

/*
 * Serve the pending host packet of @bit. Returns true when it completed.
 * From handle_data (@async false) the caller reports the status itself.
 */
static bool ci_ep_run(CIUdcState *s, int bit, bool async)
{
    USBPacket *p = s->pending[bit];
    bool in = bit >= 16, short_end;
    g_autofree uint8_t *buf = NULL;
    uint32_t want, n;

    if (!p || !(s->stat & BIT(bit))) {
        return false;
    }
    want = p->iov.size - p->actual_length;
    buf = g_malloc(want ? want : 1);
    if (!in) {
        iov_to_buf(p->iov.iov, p->iov.niov, p->actual_length, buf, want);
    }
    n = ci_xfer(s, bit, buf, want, in, &short_end);
    DPRINTF("ep bit %d moved %u of %u short %d", bit, n, want, short_end);
    if (in) {
        usb_packet_copy(p, buf, n);
    } else {
        p->actual_length += n;
    }
    if (p->actual_length == p->iov.size || short_end) {
        s->pending[bit] = NULL;
        p->status = USB_RET_SUCCESS;
        if (async) {
            usb_packet_complete(USB_DEVICE(s->link), p);
        }
        return true;
    }
    return false;
}

static void ci_run_all(CIUdcState *s)
{
    int bit;

    ci_ctrl_run(s);
    for (bit = 1; bit < 32; bit++) {
        if (bit != 16) {
            ci_ep_run(s, bit, true);
        }
    }
}

/* ---- port ---- */

static void ci_connect(CIUdcState *s, bool on)
{
    USBDevice *udev = USB_DEVICE(s->link);

    if (!s->link) {
        return;
    }
    DPRINTF("connect %d (attached %d)", on, udev->attached);
    if (on && !udev->attached) {
        s->portsc |= PORTSC_CCS | PORTSC_CSC;
        usb_device_attach(udev, &error_abort);
    } else if (!on && udev->attached) {
        s->portsc &= ~(PORTSC_CCS | PORTSC_PE | PORTSC_HSP);
        usb_device_detach(udev);
    }
}

static void ci_bus_reset(CIUdcState *s)
{
    int bit;

    s->setupstat = 0;
    s->complete = 0;
    s->stat = 0;
    s->deviceaddr = 0;
    for (bit = 0; bit < 32; bit++) {
        s->cur_td[bit] = 0;
        s->pending[bit] = NULL;
    }
    s->ctrl_p = NULL;
    s->ctrl_stage = CTRL_IDLE;
    DPRINTF("bus reset");
    /* reset received, then the port runs at high speed */
    s->portsc |= PORTSC_PE | PORTSC_HSP | PORTSC_PEC;
    ci_set_sts(s, USBSTS_URI | USBSTS_PCI);
}

/* ---- registers ---- */

static uint64_t ci_udc_read(void *opaque, hwaddr addr, unsigned size)
{
    CIUdcState *s = opaque;

    switch (addr) {
    case REG_ID:
        /* VERSION 2 (bits 28:25), REVISION 2, ID pattern 0x05 */
        return (2 << 25) | (2 << 21) | (0xfa << 8) | 0x05;
    case REG_HWGENERAL:
        return 0x35;
    case REG_HWHOST:
        return 0x10020001;
    case REG_HWDEVICE:
        return (NUM_EP << 1) | 1;
    case REG_HWTXBUF:
    case REG_HWRXBUF:
        return 0x80000000 | 0x10;
    case REG_CAPLENGTH:
        return 0x01000040;
    case REG_HCSPARAMS:
        return 0x00010011;
    case REG_HCCPARAMS:
        return 0x00000006;
    case REG_DCIVERSION:
        return 0x0001;
    case REG_DCCPARAMS:
        /* device and host capable (OTG), NUM_EP endpoints */
        return BIT(8) | BIT(7) | NUM_EP;
    case REG_USBCMD:
        return s->usbcmd;
    case REG_USBSTS:
        return s->usbsts;
    case REG_USBINTR:
        return s->usbintr;
    case REG_DEVICEADDR:
        return s->deviceaddr;
    case REG_ENDPTLISTADDR:
        return s->listaddr;
    case REG_PORTSC:
        return s->portsc | PORTSC_PP;
    case REG_OTGSC:
        return s->otgsc | OTGSC_ID | OTGSC_AVV | OTGSC_ASV | OTGSC_BSV;
    case REG_USBMODE:
        return s->usbmode;
    case REG_ENDPTSETUPSTAT:
        return s->setupstat;
    case REG_ENDPTPRIME:
    case REG_ENDPTFLUSH:
        return 0;
    case REG_ENDPTSTAT:
        return s->stat;
    case REG_ENDPTCOMPLETE:
        return s->complete;
    default:
        if (addr >= REG_ENDPTCTRL0 && addr < REG_ENDPTCTRL0 + 4 * NUM_EP) {
            uint32_t v = s->endptctrl[(addr - REG_ENDPTCTRL0) / 4];

            /* ep0 is always enabled, control type */
            return addr == REG_ENDPTCTRL0 ? v | 0x00800080 : v;
        }
        return 0;
    }
}

static void ci_udc_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    CIUdcState *s = opaque;
    int bit;

    switch (addr) {
    case REG_USBCMD:
        DPRINTF("usbcmd %08" PRIx64 " usbmode %08x", val, s->usbmode);
        if (val & USBCMD_RST) {
            ci_connect(s, false);
            s->usbcmd = 0;
            s->usbsts = 0;
            s->usbintr = 0;
            s->stat = 0;
            s->setupstat = 0;
            s->complete = 0;
            ci_update_irq(s);
            break;
        }
        s->usbcmd = val;
        ci_connect(s, (val & USBCMD_RS) &&
                   (s->usbmode & USBMODE_CM) == USBMODE_CM_DC);
        break;
    case REG_USBSTS:
        s->usbsts &= ~val;
        ci_update_irq(s);
        break;
    case REG_USBINTR:
        s->usbintr = val;
        ci_update_irq(s);
        break;
    case REG_DEVICEADDR:
        s->deviceaddr = val;
        break;
    case REG_ENDPTLISTADDR:
        s->listaddr = val;
        break;
    case REG_PORTSC:
        s->portsc &= ~(val & PORTSC_W1C);
        break;
    case REG_OTGSC:
        s->otgsc = (s->otgsc & ~(val & OTGSC_W1C)) | (val & ~OTGSC_W1C & 0xff00ff00);
        break;
    case REG_USBMODE:
        s->usbmode = val;
        break;
    case REG_ENDPTSETUPSTAT:
        s->setupstat &= ~val;
        break;
    case REG_ENDPTPRIME:
        DPRINTF("prime %08" PRIx64, val);
        for (bit = 0; bit < 32; bit++) {
            if ((val & BIT(bit)) && (bit & 15) < NUM_EP) {
                s->stat |= BIT(bit);
                ci_load_next(s, bit, ci_ld32(ci_qh(s, bit) + QH_NEXT));
            }
        }
        ci_run_all(s);
        break;
    case REG_ENDPTFLUSH:
        s->stat &= ~val;
        for (bit = 0; bit < 32; bit++) {
            if (val & BIT(bit)) {
                s->cur_td[bit] = 0;
            }
        }
        break;
    case REG_ENDPTCOMPLETE:
        s->complete &= ~val;
        break;
    default:
        if (addr >= REG_ENDPTCTRL0 && addr < REG_ENDPTCTRL0 + 4 * NUM_EP) {
            int n = (addr - REG_ENDPTCTRL0) / 4;

            s->endptctrl[n] = val;
            if (n && s->link) {
                /* EHCI dispatches iTDs only to endpoints marked isochronous. */
                usb_ep_set_type(USB_DEVICE(s->link), USB_TOKEN_OUT, n,
                                (val >> 2) & 3);
                usb_ep_set_type(USB_DEVICE(s->link), USB_TOKEN_IN, n,
                                (val >> 18) & 3);
            }
            if (n == 0 && s->ctrl_p && (val & (ENDPTCTRL_RXS | ENDPTCTRL_TXS))) {
                ci_ctrl_finish(s, USB_RET_STALL);
            }
            if (n) {
                if ((val & ENDPTCTRL_RXS) && s->pending[n]) {
                    USBPacket *p = s->pending[n];

                    s->pending[n] = NULL;
                    p->status = USB_RET_STALL;
                    usb_packet_complete(USB_DEVICE(s->link), p);
                }
                if ((val & ENDPTCTRL_TXS) && s->pending[16 + n]) {
                    USBPacket *p = s->pending[16 + n];

                    s->pending[16 + n] = NULL;
                    p->status = USB_RET_STALL;
                    usb_packet_complete(USB_DEVICE(s->link), p);
                }
            }
        }
        break;
    }
}

static const MemoryRegionOps ci_udc_ops = {
    .read = ci_udc_read,
    .write = ci_udc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ci_udc_reset(DeviceState *dev)
{
    CIUdcState *s = CI_UDC(dev);

    s->usbcmd = 0;
    s->usbsts = 0;
    s->usbintr = 0;
    s->deviceaddr = 0;
    s->listaddr = 0;
    s->portsc = 0;
    s->otgsc = 0;
    s->usbmode = 0;
    s->setupstat = 0;
    s->stat = 0;
    s->complete = 0;
    memset(s->endptctrl, 0, sizeof(s->endptctrl));
    memset(s->cur_td, 0, sizeof(s->cur_td));
    memset(s->pending, 0, sizeof(s->pending));
    s->ctrl_p = NULL;
    s->ctrl_stage = CTRL_IDLE;
}

static void ci_udc_init(Object *obj)
{
    CIUdcState *s = CI_UDC(obj);

    memory_region_init_io(&s->iomem, obj, &ci_udc_ops, s, TYPE_CI_UDC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->addr_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ci_set_addr, s);
}

static void ci_udc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ci_udc_reset);
    dc->desc = "ChipIdea USB device controller test model";
    dc->user_creatable = false;
}

/* ---- the gadget as seen by the host ---- */

static void ci_link_handle_reset(USBDevice *dev)
{
    CIUdcLinkState *l = CI_UDC_LINK(dev);

    if (l->udc) {
        ci_bus_reset(l->udc);
    }
}

static void ci_link_handle_control(USBDevice *dev, USBPacket *p, int request,
                                   int value, int index, int length,
                                   uint8_t *data)
{
    CIUdcState *s = CI_UDC_LINK(dev)->udc;
    uint8_t setup[8];

    if (!s) {
        p->status = USB_RET_STALL;
        return;
    }
    if (s->ctrl_p) {
        /* a new SETUP aborts the previous control transfer */
        ci_ctrl_finish(s, USB_RET_IOERROR);
    }

    setup[0] = request >> 8;
    setup[1] = request & 0xff;
    stw_le_p(setup + 2, value);
    stw_le_p(setup + 4, index);
    stw_le_p(setup + 6, length);
    dma_memory_write(&address_space_memory, ci_qh(s, 0) + QH_SETUP, setup, 8,
                     MEMTXATTRS_UNSPECIFIED);

    DPRINTF("setup %04x val %04x idx %04x len %d", request, value, index, length);
    s->ctrl_p = p;
    s->ctrl_in = request & (USB_DIR_IN << 8);
    s->ctrl_len = length;
    s->ctrl_done = 0;
    s->ctrl_data = data;
    s->ctrl_request = request & 0xff;
    s->ctrl_value = value;
    s->ctrl_stage = length ? CTRL_DATA : CTRL_STATUS;

    /* a SETUP clears ep0 stall and flushes ep0 */
    s->endptctrl[0] &= ~(ENDPTCTRL_RXS | ENDPTCTRL_TXS);
    s->stat &= ~(BIT(EP_BIT(0, 0)) | BIT(EP_BIT(0, 1)));
    s->setupstat |= 1;
    ci_set_sts(s, USBSTS_UI);
    p->status = USB_RET_ASYNC;
}

/* endpoint @n transfer type from ENDPTCTRL (0 ctrl, 1 iso, 2 bulk, 3 int) */
static int ci_ep_type(CIUdcState *s, int n, bool in)
{
    return (s->endptctrl[n] >> (in ? 18 : 2)) & 3;
}

/*
 * Isochronous IN: one primed dTD per host packet, answered synchronously
 * (a host controller cannot wait for iso data). No dTD: empty packet.
 */
static void ci_isoc_in(CIUdcState *s, int bit, USBPacket *p)
{
    dma_addr_t td = s->cur_td[bit];
    g_autofree uint8_t *buf = NULL;
    uint32_t token, n;

    p->status = USB_RET_SUCCESS;
    if (!(s->stat & BIT(bit)) || !td) {
        return;
    }
    token = ci_ld32(td + TD_TOKEN);
    if (!(token & TD_ACTIVE)) {
        ci_load_next(s, bit, ci_ld32(td + TD_NEXT));
        return;
    }
    n = MIN(TD_BYTES(token), p->iov.size);
    buf = g_malloc(n ? n : 1);
    for (uint32_t i = 0; i < n; ) {
        uint32_t c = MIN(n - i, 0x1000 - ((ci_ld32(td + TD_PAGE0) + i) & 0xfff));

        dma_memory_read(&address_space_memory, ci_td_addr(td, i), buf + i, c,
                        MEMTXATTRS_UNSPECIFIED);
        i += c;
    }
    usb_packet_copy(p, buf, n);
    ci_retire_td(s, bit, n);
}

static void ci_link_handle_data(USBDevice *dev, USBPacket *p)
{
    CIUdcState *s = CI_UDC_LINK(dev)->udc;
    int bit = EP_BIT(p->ep->nr, p->pid == USB_TOKEN_IN);

    if (!s || p->ep->nr >= NUM_EP || p->ep->nr == 0) {
        p->status = USB_RET_STALL;
        return;
    }
    if (s->endptctrl[p->ep->nr] & (bit >= 16 ? ENDPTCTRL_TXS : ENDPTCTRL_RXS)) {
        p->status = USB_RET_STALL;
        return;
    }
    if (ci_ep_type(s, p->ep->nr, bit >= 16) == 1) {
        if (bit >= 16) {
            ci_isoc_in(s, bit, p);
        } else {
            p->status = USB_RET_STALL;  /* iso OUT not modelled */
        }
        return;
    }
    if (s->pending[bit]) {
        p->status = USB_RET_NAK;
        return;
    }
    DPRINTF("data ep %d %s size %zu stat %08x", p->ep->nr,
            bit >= 16 ? "IN" : "OUT", p->iov.size, s->stat);
    s->pending[bit] = p;
    if (!ci_ep_run(s, bit, false)) {
        p->status = USB_RET_ASYNC;
    }
}

static void ci_link_cancel_packet(USBDevice *dev, USBPacket *p)
{
    CIUdcState *s = CI_UDC_LINK(dev)->udc;
    int bit;

    if (!s) {
        return;
    }
    if (s->ctrl_p == p) {
        s->ctrl_p = NULL;
        s->ctrl_stage = CTRL_IDLE;
    }
    for (bit = 0; bit < 32; bit++) {
        if (s->pending[bit] == p) {
            s->pending[bit] = NULL;
        }
    }
}

static void ci_link_realize(USBDevice *dev, Error **errp)
{
    dev->speed = USB_SPEED_HIGH;
    dev->speedmask = USB_SPEED_MASK_HIGH;
    dev->auto_attach = 0;
}

static void ci_link_class_init(ObjectClass *klass, const void *data)
{
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    uc->realize = ci_link_realize;
    uc->product_desc = "ChipIdea gadget link";
    uc->handle_reset = ci_link_handle_reset;
    uc->handle_control = ci_link_handle_control;
    uc->handle_data = ci_link_handle_data;
    uc->cancel_packet = ci_link_cancel_packet;
    dc->user_creatable = false;
}

void ci_udc_connect_link(CIUdcState *udc, USBBus *bus)
{
    DeviceState *dev = qdev_new(TYPE_CI_UDC_LINK);
    USBDevice *udev = USB_DEVICE(dev);

    udev->auto_attach = 0;
    udc->link = CI_UDC_LINK(dev);
    udc->link->udc = udc;
    qdev_realize_and_unref(dev, BUS(bus), &error_fatal);
}

static const TypeInfo ci_udc_types[] = {
    {
        .name          = TYPE_CI_UDC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(CIUdcState),
        .instance_init = ci_udc_init,
        .class_init    = ci_udc_class_init,
    },
    {
        .name          = TYPE_CI_UDC_LINK,
        .parent        = TYPE_USB_DEVICE,
        .instance_size = sizeof(CIUdcLinkState),
        .class_init    = ci_link_class_init,
    },
};

DEFINE_TYPES(ci_udc_types)
