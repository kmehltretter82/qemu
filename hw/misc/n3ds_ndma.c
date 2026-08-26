/*
 * Nintendo 3DS ARM9 NDMA (0x10002000): 8 channels of word DMA between
 * memory and the ARM9 peripherals (3dbrew "NDMA Registers", same IP as the
 * DSi NDMA). Transfers run synchronously when started or when a register
 * is touched: immediate mode moves the whole transfer, triggered modes move
 * one logical block per trigger, with the AES/SHA FIFO conditions checked
 * through the engine control registers so in/out channel pairs interlock.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/n3ds.h"
#include "trace.h"

#define TYPE_N3DS_NDMA "n3ds-ndma"
OBJECT_DECLARE_SIMPLE_TYPE(N3DSNdmaState, N3DS_NDMA)

#define NDMA_CHANNELS   8
#define REG_GLOBAL      0x00
#define CH_SAD          0x00
#define CH_DAD          0x04
#define CH_TCNT         0x08
#define CH_WCNT         0x0C
#define CH_BCNT         0x10
#define CH_FDATA        0x14
#define CH_CNT          0x18
#define CH_STRIDE       0x1C

#define CNT_DST_MODE(c)   (((c) >> 10) & 3)
#define CNT_DST_RELOAD    (1u << 12)
#define CNT_SRC_MODE(c)   (((c) >> 13) & 3)
#define CNT_SRC_RELOAD    (1u << 15)
#define CNT_BLOCK(c)      (1u << (((c) >> 16) & 0xF))
#define CNT_STARTUP(c)    (((c) >> 24) & 0xF)
#define CNT_IMMEDIATE     (1u << 28)
#define CNT_REPEAT        (1u << 29)
#define CNT_IRQ_EN        (1u << 30)
#define CNT_BUSY          (1u << 31)
/*
 * Implemented CNT bits, measured on a New 2DS XL (n3ds-probes ndma_regs:
 * writing 0x7fffffff reads back 0x7f0ffc1f): device-to-device startup
 * (4-0), address modes and reloads (15-10), block size (19-16), startup
 * mode, immediate/repeat/IRQ (30-24).
 */
#define CNT_IMPL_MASK     0x7F0FFC1Fu
#define BCNT_MASK         0x0003FFFFu

#define STARTUP_AES_IN    8
#define STARTUP_AES_OUT   9
#define STARTUP_SHA_IN    10
#define STARTUP_SHA_OUT   11

#define AES_CNT_ADDR      0x10009000u

typedef struct {
    /*
     * Writes land in the latches; the internal working registers are loaded
     * from them when the channel starts, and reads return the working ones
     * (measured: writing 0xffffffff to SAD/TCNT/WCNT while idle does not
     * change what they read back, and after a transfer SAD/DAD show the
     * advanced addresses and WCNT the words left).
     */
    uint32_t sad, dad, tcnt, wcnt, bcnt, fdata, cnt;
    uint32_t cur_sad, cur_dad, remaining, cur_wcnt;
} NdmaChannel;

struct N3DSNdmaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion *dma_mr;
    AddressSpace dma_as;
    qemu_irq irq[NDMA_CHANNELS];

    uint32_t global;
    NdmaChannel ch[NDMA_CHANNELS];
    bool running;
};

static uint32_t ndma_ld(N3DSNdmaState *s, uint32_t addr)
{
    return address_space_ldl_le(&s->dma_as, addr, MEMTXATTRS_UNSPECIFIED, NULL);
}

static void ndma_st(N3DSNdmaState *s, uint32_t addr, uint32_t v)
{
    address_space_stl_le(&s->dma_as, addr, v, MEMTXATTRS_UNSPECIFIED, NULL);
}

/* can this triggered channel move a block now? */
static bool ndma_ready(N3DSNdmaState *s, NdmaChannel *c, uint32_t words)
{
    uint32_t aes;

    switch (CNT_STARTUP(c->cnt)) {
    case STARTUP_AES_IN:
        aes = ndma_ld(s, AES_CNT_ADDR);
        return (aes & CNT_BUSY) && (aes & 0x1F) + words <= 16;
    case STARTUP_AES_OUT:
        aes = ndma_ld(s, AES_CNT_ADDR);
        return ((aes >> 5) & 0x1F) >= words;
    default:
        return true;    /* SHA, timers, cards: treat as always ready */
    }
}

static void ndma_move(N3DSNdmaState *s, NdmaChannel *c, uint32_t words)
{
    uint32_t i;

    for (i = 0; i < words; i++) {
        uint32_t v;

        switch (CNT_SRC_MODE(c->cnt)) {
        case 3:
            v = c->fdata;
            break;
        default:
            v = ndma_ld(s, c->cur_sad);
            if (CNT_SRC_MODE(c->cnt) == 0) {
                c->cur_sad += 4;
            } else if (CNT_SRC_MODE(c->cnt) == 1) {
                c->cur_sad -= 4;
            }
            break;
        }
        ndma_st(s, c->cur_dad, v);
        if (CNT_DST_MODE(c->cnt) == 0) {
            c->cur_dad += 4;
        } else if (CNT_DST_MODE(c->cnt) == 1) {
            c->cur_dad -= 4;
        }
    }
}

static void ndma_finish(N3DSNdmaState *s, int n)
{
    NdmaChannel *c = &s->ch[n];

    c->cnt &= ~CNT_BUSY;
    if (c->cnt & CNT_IRQ_EN) {
        qemu_irq_pulse(s->irq[n]);
    }
}

/* run every busy channel as far as its trigger conditions allow */
static void ndma_run(N3DSNdmaState *s)
{
    bool progress;
    int n, guard = 0;

    if (s->running) {
        return;
    }
    s->running = true;
    do {
        progress = false;
        for (n = 0; n < NDMA_CHANNELS; n++) {
            NdmaChannel *c = &s->ch[n];
            uint32_t words;

            if (!(c->cnt & CNT_BUSY)) {
                continue;
            }
            if (c->cnt & CNT_IMMEDIATE) {
                /*
                 * Immediate mode always moves the write count, whatever the
                 * total count says (measured: with tcnt 6 / wcnt 2 exactly
                 * two words move). boot9strap's register image relies on it.
                 */
                ndma_move(s, c, c->cur_wcnt);
                c->cur_wcnt = 0;
                ndma_finish(s, n);
                progress = true;
                continue;
            }
            words = MIN(c->cur_wcnt ? c->cur_wcnt : 1,
                        c->remaining ? c->remaining : c->cur_wcnt);
            if (!ndma_ready(s, c, words)) {
                continue;
            }
            ndma_move(s, c, words);
            progress = true;
            if (c->remaining) {
                c->remaining -= MIN(words, c->remaining);
                if (c->remaining == 0) {
                    if (c->cnt & CNT_REPEAT) {
                        c->remaining = c->tcnt;
                        c->cur_wcnt = c->wcnt;
                        if (c->cnt & CNT_SRC_RELOAD) c->cur_sad = c->sad;
                        if (c->cnt & CNT_DST_RELOAD) c->cur_dad = c->dad;
                    } else {
                        ndma_finish(s, n);
                    }
                }
            }
        }
    } while (progress && ++guard < 100000);
    s->running = false;
}

static uint64_t n3ds_ndma_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSNdmaState *s = opaque;
    uint32_t v = 0;

    off &= 0xFF;
    if (off < 4) {
        v = s->global;
    } else if (off < 4 + NDMA_CHANNELS * CH_STRIDE) {
        NdmaChannel *c = &s->ch[(off - 4) / CH_STRIDE];
        switch ((off - 4) % CH_STRIDE) {
        case CH_SAD:   v = c->cur_sad; break;
        case CH_DAD:   v = c->cur_dad; break;
        case CH_TCNT:  v = c->remaining; break;
        case CH_WCNT:  v = c->cur_wcnt; break;
        case CH_BCNT:  v = c->bcnt; break;
        case CH_FDATA: v = c->fdata; break;
        case CH_CNT:   ndma_run(s); v = c->cnt; break;
        }
    }
    return (v >> (8 * (off & 3))) & (size == 4 ? 0xFFFFFFFF : size == 2 ? 0xFFFF : 0xFF);
}

static void n3ds_ndma_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    N3DSNdmaState *s = opaque;
    uint32_t v = val;

    off &= 0xFF;
    if (off < 4) {
        s->global = v;
        return;
    }
    if (off < 4 + NDMA_CHANNELS * CH_STRIDE) {
        int n = (off - 4) / CH_STRIDE;
        NdmaChannel *c = &s->ch[n];
        switch ((off - 4) % CH_STRIDE) {
        case CH_SAD:   c->sad = v & ~3u; break;
        case CH_DAD:   c->dad = v & ~3u; break;
        case CH_TCNT:  c->tcnt = v & 0x0FFFFFFF; break;
        case CH_WCNT:  c->wcnt = v & 0x00FFFFFF; break;
        case CH_BCNT:  c->bcnt = v & BCNT_MASK; break;
        case CH_FDATA: c->fdata = v; break;
        case CH_CNT:
            c->cnt = v & CNT_IMPL_MASK;
            if (v & CNT_BUSY) {
                trace_n3ds_ndma_start(n, v, c->sad, c->dad, c->tcnt, c->wcnt);
                c->cnt |= CNT_BUSY;
                c->cur_sad = c->sad;
                c->cur_dad = c->dad;
                c->remaining = c->tcnt;
                c->cur_wcnt = c->wcnt;
                ndma_run(s);
            }
            break;
        }
        return;
    }
    /* 0xE4..0xFF is past the 8 channels: reserved, ignore quietly */
}

static const MemoryRegionOps n3ds_ndma_ops = {
    .read = n3ds_ndma_read,
    .write = n3ds_ndma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void n3ds_ndma_reset(DeviceState *dev)
{
    N3DSNdmaState *s = N3DS_NDMA(dev);

    /* global control as the console reports it (n3ds-probes ndma_regs) */
    s->global = 0x80060001;
    memset(s->ch, 0, sizeof(s->ch));
    s->running = false;
}

static void n3ds_ndma_realize(DeviceState *dev, Error **errp)
{
    N3DSNdmaState *s = N3DS_NDMA(dev);

    if (!s->dma_mr) {
        error_setg(errp, "n3ds-ndma: 'memory' link not set");
        return;
    }
    address_space_init(&s->dma_as, s->dma_mr, "n3ds-ndma");
}

static void n3ds_ndma_init(Object *obj)
{
    N3DSNdmaState *s = N3DS_NDMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &n3ds_ndma_ops, s, "n3ds-ndma", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (i = 0; i < NDMA_CHANNELS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static const Property n3ds_ndma_properties[] = {
    DEFINE_PROP_LINK("memory", N3DSNdmaState, dma_mr, TYPE_MEMORY_REGION,
                     MemoryRegion *),
};

static void n3ds_ndma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = n3ds_ndma_realize;
    device_class_set_legacy_reset(dc, n3ds_ndma_reset);
    device_class_set_props(dc, n3ds_ndma_properties);
}

static const TypeInfo n3ds_ndma_info = {
    .name = TYPE_N3DS_NDMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSNdmaState),
    .instance_init = n3ds_ndma_init,
    .class_init = n3ds_ndma_class_init,
};

static void n3ds_ndma_register_types(void)
{
    type_register_static(&n3ds_ndma_info);
}

type_init(n3ds_ndma_register_types)
