/*
 * Atmel/Microchip AT91 DMA Controller (DMAC).
 *
 * Functional model of the 8-channel scatter-gather DMA.  Transfers (and their
 * completion interrupt) run from a bottom half, so both memory-to-memory
 * (dmatest) and peripheral transfers work without reentrant completion
 * corrupting driver state machines.  Flow-control / chunk striding / PiP are
 * not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/host-utils.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "hw/dma/at91_dmac.h"
#include "trace.h"

#define DMAC_GCFG    0x00
#define DMAC_EN      0x04
#define DMAC_EBCIER  0x18
#define DMAC_EBCIDR  0x1C
#define DMAC_EBCIMR  0x20
#define DMAC_EBCISR  0x24
#define DMAC_CHER    0x28
#define DMAC_CHDR    0x2C
#define DMAC_CHSR    0x30
#define DMAC_CH_BASE 0x3C
#define DMAC_CH_SIZE 0x28

/* per-channel register offsets within a channel block */
#define DMAC_SADDR   0x00
#define DMAC_DADDR   0x04
#define DMAC_DSCR    0x08
#define DMAC_CTRLA   0x0C
#define DMAC_CTRLB   0x10
#define DMAC_CFG     0x14

#define DMAC_N_CHANNELS  8

/* CTRLA fields */
#define DMAC_CTRLA_BTSIZE(a)     ((a) & 0xFFFF)
#define DMAC_CTRLA_SRC_WIDTH(a)  (((a) >> 24) & 0x3)
#define DMAC_CTRLA_DST_WIDTH(a)  (((a) >> 28) & 0x3)
/* CTRLB address modes: 0 = increment, 1 = decrement, 2 = fixed */
#define DMAC_CTRLB_SRC_MODE(b)   (((b) >> 24) & 0x3)
#define DMAC_CTRLB_DST_MODE(b)   (((b) >> 28) & 0x3)

/* EBCISR/EBCIMR bits */
#define DMAC_BTC(x)   (1u << (x))          /* buffer transfer completed */
#define DMAC_CBTC(x)  (1u << (8 + (x)))    /* chained buffer completed  */

typedef struct AT91DmacChan {
    uint32_t saddr, daddr, dscr, ctrla, ctrlb, cfg;
} AT91DmacChan;

struct AT91DmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUBH *bh;                /* runs transfers asynchronously */
    uint32_t pending;          /* channels awaiting transfer */

    uint32_t gcfg, en, ebcimr, ebcisr, chsr;
    AT91DmacChan ch[DMAC_N_CHANNELS];
};

static void dmac_update_irq(AT91DmacState *s)
{
    qemu_set_irq(s->irq, (s->ebcisr & s->ebcimr) ? 1 : 0);
}

static unsigned dmac_width_bytes(unsigned w)
{
    return 1u << (w & 0x3);   /* 0->1, 1->2, 2->4 */
}

/* Transfer one buffer described by (saddr, daddr, ctrla, ctrlb). */
static void dmac_run_buffer(uint32_t saddr, uint32_t daddr,
                            uint32_t ctrla, uint32_t ctrlb)
{
    uint32_t btsize = DMAC_CTRLA_BTSIZE(ctrla);
    unsigned sw = dmac_width_bytes(DMAC_CTRLA_SRC_WIDTH(ctrla));
    unsigned dw = dmac_width_bytes(DMAC_CTRLA_DST_WIDTH(ctrla));
    int smode = DMAC_CTRLB_SRC_MODE(ctrlb);
    int dmode = DMAC_CTRLB_DST_MODE(ctrlb);

    uint32_t sa = saddr, da = daddr;
    uint32_t i;

    for (i = 0; i < btsize; i++) {
        uint8_t buf[4] = { 0 };
        unsigned n = sw < dw ? dw : sw;   /* move the wider of the two */

        if (n > sizeof(buf)) {
            n = sizeof(buf);
        }
        address_space_read(&address_space_memory, sa,
                           MEMTXATTRS_UNSPECIFIED, buf, sw);
        address_space_write(&address_space_memory, da,
                            MEMTXATTRS_UNSPECIFIED, buf, dw);
        sa += (smode == 0) ? sw : (smode == 1) ? -sw : 0;
        da += (dmode == 0) ? dw : (dmode == 1) ? -dw : 0;
    }
}

static void dmac_run_channel(AT91DmacState *s, int n)
{
    uint32_t dscr = s->ch[n].dscr;

    trace_at91_dmac_run(n, s->ch[n].saddr, s->ch[n].daddr,
                        s->ch[n].ctrla & 0xffff);
    if (dscr == 0) {
        /* single-buffer transfer straight from the channel registers */
        dmac_run_buffer(s->ch[n].saddr, s->ch[n].daddr,
                        s->ch[n].ctrla, s->ch[n].ctrlb);
        s->ebcisr |= DMAC_BTC(n);
    } else {
        /* linked-list of hardware descriptors: saddr,daddr,ctrla,ctrlb,next */
        int guard = 0;
        while (dscr != 0 && guard++ < 1024) {
            uint32_t d[5];
            address_space_read(&address_space_memory, dscr,
                               MEMTXATTRS_UNSPECIFIED, d, sizeof(d));
            dmac_run_buffer(d[0], d[1], d[2], d[3]);
            s->ebcisr |= DMAC_BTC(n);
            dscr = d[4];
        }
        s->ebcisr |= DMAC_CBTC(n);
    }
    s->chsr &= ~(1u << n);   /* channel finished */
    trace_at91_dmac_complete(n);
    dmac_update_irq(s);
}

/* Transfers run from a bottom half so completion (and its interrupt) happen
 * after the enabling MMIO write returns - real DMA completes asynchronously,
 * and a reentrant completion corrupts driver state machines (e.g. atmel-mci). */
static void dmac_bh(void *opaque)
{
    AT91DmacState *s = opaque;

    while (s->pending) {
        int n = ctz32(s->pending);
        s->pending &= ~(1u << n);
        dmac_run_channel(s, n);
    }
}

static uint64_t dmac_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91DmacState *s = AT91_DMAC(opaque);
    uint32_t r = 0;

    if (offset >= DMAC_CH_BASE) {
        unsigned n = (offset - DMAC_CH_BASE) / DMAC_CH_SIZE;
        unsigned reg = (offset - DMAC_CH_BASE) % DMAC_CH_SIZE;
        if (n >= DMAC_N_CHANNELS) {
            return 0;
        }
        switch (reg) {
        case DMAC_SADDR: return s->ch[n].saddr;
        case DMAC_DADDR: return s->ch[n].daddr;
        case DMAC_DSCR:  return s->ch[n].dscr;
        case DMAC_CTRLA: return s->ch[n].ctrla;
        case DMAC_CTRLB: return s->ch[n].ctrlb;
        case DMAC_CFG:   return s->ch[n].cfg;
        default:         return 0;
        }
    }

    switch (offset) {
    case DMAC_GCFG:   r = s->gcfg; break;
    case DMAC_EN:     r = s->en; break;
    case DMAC_EBCIMR: r = s->ebcimr; break;
    case DMAC_EBCISR:                   /* read clears the status */
        r = s->ebcisr;
        s->ebcisr = 0;
        dmac_update_irq(s);
        break;
    case DMAC_CHSR:   r = s->chsr; break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-dmac: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        break;
    }
    return r;
}

static void dmac_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AT91DmacState *s = AT91_DMAC(opaque);
    uint32_t val = value;

    if (offset >= DMAC_CH_BASE) {
        unsigned n = (offset - DMAC_CH_BASE) / DMAC_CH_SIZE;
        unsigned reg = (offset - DMAC_CH_BASE) % DMAC_CH_SIZE;
        if (n >= DMAC_N_CHANNELS) {
            return;
        }
        switch (reg) {
        case DMAC_SADDR: s->ch[n].saddr = val; break;
        case DMAC_DADDR: s->ch[n].daddr = val; break;
        case DMAC_DSCR:  s->ch[n].dscr = val; break;
        case DMAC_CTRLA: s->ch[n].ctrla = val; break;
        case DMAC_CTRLB: s->ch[n].ctrlb = val; break;
        case DMAC_CFG:   s->ch[n].cfg = val; break;
        default: break;
        }
        return;
    }

    switch (offset) {
    case DMAC_GCFG:   s->gcfg = val; break;
    case DMAC_EN:     s->en = val & 0x1; break;
    case DMAC_EBCIER: s->ebcimr |= val; dmac_update_irq(s); break;
    case DMAC_EBCIDR: s->ebcimr &= ~val; dmac_update_irq(s); break;
    case DMAC_CHER: {
        int n;
        for (n = 0; n < DMAC_N_CHANNELS; n++) {
            if (val & (1u << n)) {
                s->chsr |= 1u << n;
                s->pending |= 1u << n;
            }
        }
        qemu_bh_schedule(s->bh);   /* transfer asynchronously */
        break;
    }
    case DMAC_CHDR:
        s->chsr &= ~(val & 0xFF);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-dmac: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps dmac_ops = {
    .read = dmac_read,
    .write = dmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void dmac_reset(DeviceState *dev)
{
    AT91DmacState *s = AT91_DMAC(dev);

    s->gcfg = 0x10;
    s->en = 0;
    s->ebcimr = 0;
    s->ebcisr = 0;
    s->chsr = 0;
    s->pending = 0;
    memset(s->ch, 0, sizeof(s->ch));
}

static void dmac_dev_init(Object *obj)
{
    AT91DmacState *s = AT91_DMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &dmac_ops, s, "at91-dmac", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void dmac_realize(DeviceState *dev, Error **errp)
{
    AT91DmacState *s = AT91_DMAC(dev);

    s->bh = qemu_bh_new(dmac_bh, s);
}

static int dmac_post_load(void *opaque, int version_id)
{
    AT91DmacState *s = opaque;

    /* The bottom half is not migrated; if a transfer was still pending,
     * run it after load. */
    if (s->pending) {
        qemu_bh_schedule(s->bh);
    }
    return 0;
}

static const VMStateDescription vmstate_at91_dmac_chan = {
    .name = "at91-dmac-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(saddr, AT91DmacChan),
        VMSTATE_UINT32(daddr, AT91DmacChan),
        VMSTATE_UINT32(dscr, AT91DmacChan),
        VMSTATE_UINT32(ctrla, AT91DmacChan),
        VMSTATE_UINT32(ctrlb, AT91DmacChan),
        VMSTATE_UINT32(cfg, AT91DmacChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_at91_dmac = {
    .name = "at91-dmac",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = dmac_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(gcfg, AT91DmacState),
        VMSTATE_UINT32(en, AT91DmacState),
        VMSTATE_UINT32(ebcimr, AT91DmacState),
        VMSTATE_UINT32(ebcisr, AT91DmacState),
        VMSTATE_UINT32(chsr, AT91DmacState),
        VMSTATE_UINT32(pending, AT91DmacState),
        VMSTATE_STRUCT_ARRAY(ch, AT91DmacState, DMAC_N_CHANNELS, 1,
                             vmstate_at91_dmac_chan, AT91DmacChan),
        VMSTATE_END_OF_LIST()
    }
};

static void dmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = dmac_realize;
    device_class_set_legacy_reset(dc, dmac_reset);
    dc->vmsd = &vmstate_at91_dmac;
}

static const TypeInfo dmac_type = {
    .name = TYPE_AT91_DMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91DmacState),
    .instance_init = dmac_dev_init,
    .class_init = dmac_class_init,
};

static void at91_dmac_register_types(void)
{
    type_register_static(&dmac_type);
}

type_init(at91_dmac_register_types)
