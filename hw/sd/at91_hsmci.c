/*
 * Atmel/Microchip AT91 High Speed MultiMedia Card Interface (HSMCI).
 *
 * PIO-mode SD/MMC host: commands go to QEMU's SD card model over an SDBus,
 * data is transferred a word at a time through RDR/TDR; peripheral DMA is
 * driven externally by the DMAC over the RDR/TDR FIFO.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/sd/sd.h"
#include "hw/sd/at91_hsmci.h"
#include "trace.h"

#define HSMCI_CR     0x00
#define HSMCI_MR     0x04
#define HSMCI_DTOR   0x08
#define HSMCI_SDCR   0x0C
#define HSMCI_ARGR   0x10
#define HSMCI_CMDR   0x14
#define HSMCI_BLKR   0x18
#define HSMCI_CSTOR  0x1C
#define HSMCI_RSPR   0x20   /* 0x20..0x2C */
#define HSMCI_RDR    0x30
#define HSMCI_TDR    0x34
#define HSMCI_SR     0x40
#define HSMCI_IER    0x44
#define HSMCI_IDR    0x48
#define HSMCI_IMR    0x4C
#define HSMCI_DMA    0x50
#define HSMCI_CFG    0x54
#define HSMCI_WPMR   0xE4
#define HSMCI_WPSR   0xE8
#define HSMCI_VERSION 0xFC
#define HSMCI_FIFO   0x200  /* 0x200..0x5FC */

/*
 * IP version reported in HSMCI_VERSION.  The atmel-mci driver keys its
 * capabilities off this: 0x3xx selects the HSMCI feature set (has_dma_conf_reg,
 * no PDC).  With no external DMA channel in the DT the driver then falls back
 * to PIO, which is what this model implements.
 */
#define HSMCI_IP_VERSION  0x300

/* Control Register bits */
#define HSMCI_CR_MCIEN   (1u << 0)
#define HSMCI_CR_MCIDIS  (1u << 1)
#define HSMCI_CR_SWRST   (1u << 7)

/* Command Register fields */
#define HSMCI_CMDR_CMDNB(c)    ((c) & 0x3F)
#define HSMCI_CMDR_RSPTYP(c)   (((c) >> 6) & 0x3)
#define HSMCI_CMDR_TRCMD(c)    (((c) >> 16) & 0x3)
#define HSMCI_CMDR_TRDIR_READ  (1u << 18)
#define HSMCI_RSPTYP_NONE   0
#define HSMCI_RSPTYP_48     1
#define HSMCI_RSPTYP_136    2
#define HSMCI_TRCMD_START   1

/* Block Register fields */
#define HSMCI_BLKR_BCNT(b)     ((b) & 0xFFFF)
#define HSMCI_BLKR_BLKLEN(b)   (((b) >> 16) & 0xFFFF)

/* Status/Interrupt bits */
#define HSMCI_SR_CMDRDY   (1u << 0)
#define HSMCI_SR_RXRDY    (1u << 1)
#define HSMCI_SR_TXRDY    (1u << 2)
#define HSMCI_SR_BLKE     (1u << 3)
#define HSMCI_SR_NOTBUSY  (1u << 5)
#define HSMCI_SR_RTOE     (1u << 20)  /* response time-out (no card responded) */
#define HSMCI_SR_RESET    0x0000C0E5

static void hsmci_reset(DeviceState *dev);

struct AT91HsmciState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;
    qemu_irq dma_request;

    uint32_t mr, dtor, sdcr, argr, blkr, cstor, dma, cfg, wpmr;
    uint32_t sr;
    uint32_t imr;
    uint32_t rsp[4];
    int rsp_ptr;

    uint32_t blklen;        /* current transfer block length */
    int32_t  data_len;      /* bytes left in the current data transfer */
    bool     reading;       /* transfer direction */
};

static void hsmci_update_irq(AT91HsmciState *s)
{
    qemu_set_irq(s->irq, (s->sr & s->imr) ? 1 : 0);
}

static void hsmci_update_dma_request(AT91HsmciState *s)
{
    qemu_set_irq(s->dma_request,
                 s->data_len > 0 &&
                 (s->sr & (HSMCI_SR_RXRDY | HSMCI_SR_TXRDY)));
}

static void hsmci_do_command(AT91HsmciState *s, uint32_t cmdr)
{
    SDRequest req = {
        .cmd = HSMCI_CMDR_CMDNB(cmdr),
        .arg = s->argr,
    };
    uint8_t resp[16];
    size_t rlen;

    /* A fresh command clears the previous response-timeout status. */
    s->sr &= ~HSMCI_SR_RTOE;

    rlen = sdbus_do_command(&s->sdbus, &req, resp, sizeof(resp));
    trace_at91_hsmci_command(req.cmd, req.arg, (unsigned)rlen);

    /* A command that expects a response but gets none (e.g. an empty card
     * slot) is a response time-out - the driver treats RTOE as card-absent,
     * instead of spinning in __mmc_poll_for_busy. */
    if (HSMCI_CMDR_RSPTYP(cmdr) != HSMCI_RSPTYP_NONE && rlen == 0) {
        s->sr |= HSMCI_SR_RTOE;
        trace_at91_hsmci_rtoe(req.cmd);
    }

    s->rsp_ptr = 0;
    if (HSMCI_CMDR_RSPTYP(cmdr) == HSMCI_RSPTYP_136 && rlen == 16) {
        s->rsp[0] = ldl_be_p(&resp[0]);
        s->rsp[1] = ldl_be_p(&resp[4]);
        s->rsp[2] = ldl_be_p(&resp[8]);
        s->rsp[3] = ldl_be_p(&resp[12]);
    } else if (rlen >= 4) {
        s->rsp[0] = ldl_be_p(&resp[0]);
        s->rsp[1] = s->rsp[2] = s->rsp[3] = 0;
    } else {
        s->rsp[0] = s->rsp[1] = s->rsp[2] = s->rsp[3] = 0;
    }

    s->sr |= HSMCI_SR_CMDRDY;

    /* Set up a data transfer if the command starts one. */
    if (HSMCI_CMDR_TRCMD(cmdr) == HSMCI_TRCMD_START) {
        uint32_t blklen = HSMCI_BLKR_BLKLEN(s->blkr);
        uint32_t bcnt = HSMCI_BLKR_BCNT(s->blkr);

        if (blklen == 0) {
            blklen = s->mr >> 16;      /* MR.BLKLEN fallback */
        }
        if (bcnt == 0) {
            bcnt = 1;
        }
        s->blklen = blklen;
        s->data_len = (int32_t)(blklen * bcnt);
        s->reading = (cmdr & HSMCI_CMDR_TRDIR_READ) != 0;
        s->sr &= ~(HSMCI_SR_NOTBUSY | HSMCI_SR_BLKE);
        if (s->reading) {
            s->sr |= HSMCI_SR_RXRDY;
        } else {
            s->sr |= HSMCI_SR_TXRDY;
        }
        hsmci_update_dma_request(s);
    }
    hsmci_update_irq(s);
}

static void hsmci_xfer_done(AT91HsmciState *s)
{
    s->sr &= ~(HSMCI_SR_RXRDY | HSMCI_SR_TXRDY);
    s->sr |= HSMCI_SR_TXRDY | HSMCI_SR_BLKE | HSMCI_SR_NOTBUSY;
    s->data_len = 0;
    hsmci_update_dma_request(s);
}

static uint32_t hsmci_read_data(AT91HsmciState *s)
{
    uint32_t v = 0;
    int i;

    if (s->data_len <= 0 || !s->reading) {
        return 0;
    }
    for (i = 0; i < 4; i++) {
        v |= (uint32_t)sdbus_read_byte(&s->sdbus) << (8 * i);
    }
    s->data_len -= 4;
    if (s->data_len <= 0) {
        hsmci_xfer_done(s);
    }
    hsmci_update_irq(s);
    return v;
}

static void hsmci_write_data(AT91HsmciState *s, uint32_t value)
{
    int i;

    if (s->data_len <= 0 || s->reading) {
        return;
    }
    for (i = 0; i < 4; i++) {
        sdbus_write_byte(&s->sdbus, (value >> (8 * i)) & 0xff);
    }
    s->data_len -= 4;
    if (s->data_len <= 0) {
        hsmci_xfer_done(s);
    }
    hsmci_update_irq(s);
}

static uint64_t hsmci_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91HsmciState *s = AT91_HSMCI(opaque);

    if (offset >= HSMCI_RSPR && offset <= HSMCI_RSPR + 0xC) {
        if (offset == HSMCI_RSPR) {
            uint32_t v = s->rsp[s->rsp_ptr & 3];
            s->rsp_ptr++;
            return v;
        }
        return s->rsp[(offset - HSMCI_RSPR) / 4];
    }
    if (offset >= HSMCI_FIFO) {
        return hsmci_read_data(s);
    }

    switch (offset) {
    case HSMCI_MR:   return s->mr;
    case HSMCI_DTOR: return s->dtor;
    case HSMCI_SDCR: return s->sdcr;
    case HSMCI_ARGR: return s->argr;
    case HSMCI_BLKR: return s->blkr;
    case HSMCI_CSTOR: return s->cstor;
    case HSMCI_RDR:  return hsmci_read_data(s);
    case HSMCI_SR:   return s->sr;
    case HSMCI_IMR:  return s->imr;
    case HSMCI_DMA:  return s->dma;
    case HSMCI_CFG:  return s->cfg;
    case HSMCI_WPMR: return s->wpmr;
    case HSMCI_WPSR: return 0;
    case HSMCI_VERSION: return HSMCI_IP_VERSION;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-hsmci: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void hsmci_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    AT91HsmciState *s = AT91_HSMCI(opaque);
    uint32_t val = value;

    if (offset >= HSMCI_FIFO) {
        hsmci_write_data(s, val);
        return;
    }

    switch (offset) {
    case HSMCI_CR:
        if (val & HSMCI_CR_SWRST) {
            /* Software reset of the controller only - must NOT reset the SD
             * card on the bus (that would clear its app-command state). */
            uint32_t saved_mr = s->mr, saved_sdcr = s->sdcr;
            hsmci_reset(DEVICE(s));
            s->mr = saved_mr;
            s->sdcr = saved_sdcr;
        }
        break;
    case HSMCI_MR:   s->mr = val;   break;
    case HSMCI_DTOR: s->dtor = val; break;
    case HSMCI_SDCR: s->sdcr = val; break;
    case HSMCI_ARGR: s->argr = val; break;
    case HSMCI_CMDR: hsmci_do_command(s, val); break;
    case HSMCI_BLKR: s->blkr = val; break;
    case HSMCI_CSTOR: s->cstor = val; break;
    case HSMCI_TDR:  hsmci_write_data(s, val); break;
    case HSMCI_IER:  s->imr |= val; hsmci_update_irq(s); break;
    case HSMCI_IDR:  s->imr &= ~val; hsmci_update_irq(s); break;
    case HSMCI_DMA:  s->dma = val; break;
    case HSMCI_CFG:  s->cfg = val; break;
    case HSMCI_WPMR: s->wpmr = val; break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-hsmci: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps hsmci_ops = {
    .read = hsmci_read,
    .write = hsmci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void hsmci_reset(DeviceState *dev)
{
    AT91HsmciState *s = AT91_HSMCI(dev);

    s->mr = s->dtor = s->sdcr = s->argr = s->blkr = 0;
    s->cstor = s->dma = s->cfg = s->wpmr = 0;
    s->sr = HSMCI_SR_RESET;
    s->imr = 0;
    s->rsp[0] = s->rsp[1] = s->rsp[2] = s->rsp[3] = 0;
    s->rsp_ptr = 0;
    s->data_len = 0;
    s->blklen = 0;
    s->reading = false;
    hsmci_update_irq(s);
    hsmci_update_dma_request(s);
}

static void hsmci_dev_init(Object *obj)
{
    AT91HsmciState *s = AT91_HSMCI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hsmci_ops, s, "at91-hsmci", 0x600);
    /* The DMAC legitimately reads/writes the RDR/TDR FIFO here; it does not
     * recurse, so exempt the region from the DMA re-entrancy guard. */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->dma_request,
                             AT91_HSMCI_DMA_REQUEST, 1);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_AT91_HSMCI_BUS, DEVICE(obj),
              "sd-bus");
}

static int hsmci_post_load(void *opaque, int version_id)
{
    AT91HsmciState *s = opaque;

    hsmci_update_irq(s);
    hsmci_update_dma_request(s);
    return 0;
}

static const VMStateDescription vmstate_at91_hsmci = {
    .name = "at91-hsmci",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = hsmci_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91HsmciState),
        VMSTATE_UINT32(dtor, AT91HsmciState),
        VMSTATE_UINT32(sdcr, AT91HsmciState),
        VMSTATE_UINT32(argr, AT91HsmciState),
        VMSTATE_UINT32(blkr, AT91HsmciState),
        VMSTATE_UINT32(cstor, AT91HsmciState),
        VMSTATE_UINT32(dma, AT91HsmciState),
        VMSTATE_UINT32(cfg, AT91HsmciState),
        VMSTATE_UINT32(wpmr, AT91HsmciState),
        VMSTATE_UINT32(sr, AT91HsmciState),
        VMSTATE_UINT32(imr, AT91HsmciState),
        VMSTATE_UINT32_ARRAY(rsp, AT91HsmciState, 4),
        VMSTATE_INT32(rsp_ptr, AT91HsmciState),
        VMSTATE_UINT32(blklen, AT91HsmciState),
        VMSTATE_INT32(data_len, AT91HsmciState),
        VMSTATE_BOOL(reading, AT91HsmciState),
        VMSTATE_END_OF_LIST()
    }
};

static void hsmci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, hsmci_reset);
    dc->user_creatable = false;   /* needs board IRQ + card wiring */
    dc->vmsd = &vmstate_at91_hsmci;
}

static const TypeInfo hsmci_types[] = {
    {
        .name = TYPE_AT91_HSMCI,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AT91HsmciState),
        .instance_init = hsmci_dev_init,
        .class_init = hsmci_class_init,
    },
    {
        .name = TYPE_AT91_HSMCI_BUS,
        .parent = TYPE_SD_BUS,
        .instance_size = sizeof(SDBus),
    },
};

static void at91_hsmci_register_types(void)
{
    type_register_static_array(hsmci_types, ARRAY_SIZE(hsmci_types));
}

type_init(at91_hsmci_register_types)
