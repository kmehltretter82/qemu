/*
 * Atmel/Microchip AT91 DMA Controller (DMAC).
 *
 * Functional model of the 8-channel scatter-gather DMA.  Transfers (and their
 * completion interrupt) run from a bottom half, so both memory-to-memory
 * (dmatest) and peripheral transfers work without reentrant completion
 * corrupting driver state machines.  Software single/chunk handshakes and
 * peripheral-controlled LAST and picture-in-picture address generation are
 * supported; hardware-request beat pacing, arbitration and replay are not
 * modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/host-utils.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "hw/dma/at91_dmac.h"
#include "trace.h"

#define DMAC_GCFG    0x00
#define DMAC_EN      0x04
#define DMAC_SREQ    0x08
#define DMAC_CREQ    0x0C
#define DMAC_LAST    0x10
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
#define DMAC_SPIP    0x18
#define DMAC_DPIP    0x1C

#define DMAC_N_CHANNELS  8
#define DMAC_CHANNEL_MASK 0xff
#define DMAC_SUSPEND_MASK (DMAC_CHANNEL_MASK << 8)
#define DMAC_EMPTY_MASK   (DMAC_CHANNEL_MASK << 16)
#define DMAC_STALL_MASK   (DMAC_CHANNEL_MASK << 24)

/* CTRLA fields */
#define DMAC_CTRLA_BTSIZE(a)     ((a) & 0xFFFF)
#define DMAC_CTRLA_SCSIZE(a)     (((a) >> 16) & 0x7)
#define DMAC_CTRLA_DCSIZE(a)     (((a) >> 20) & 0x7)
#define DMAC_CTRLA_SRC_WIDTH(a)  (((a) >> 24) & 0x3)
#define DMAC_CTRLA_DST_WIDTH(a)  (((a) >> 28) & 0x3)
/* CTRLB address modes: 0 = increment, 1 = decrement, 2 = fixed */
#define DMAC_CTRLB_SRC_MODE(b)   (((b) >> 24) & 0x3)
#define DMAC_CTRLB_DST_MODE(b)   (((b) >> 28) & 0x3)
#define DMAC_CTRLB_FC(b)         (((b) >> 21) & 0x7)
#define DMAC_CTRLB_IEN            (1u << 30) /* BTC enable, active low */
#define DMAC_CTRLB_SRC_PIP        (1u << 8)
#define DMAC_CTRLB_DST_PIP        (1u << 12)
#define DMAC_CTRLB_SRC_DSCR_DIS   (1u << 16)
#define DMAC_CTRLB_DST_DSCR_DIS   (1u << 20)
#define DMAC_CTRLA_DONE           (1u << 31)
#define DMAC_CFG_RESET            (1u << 24) /* reset AHB protection value */
#define DMAC_CFG_SRC_H2SEL        (1u << 9)
#define DMAC_CFG_DST_H2SEL        (1u << 13)
#define DMAC_CFG_SOD              (1u << 16)
#define DMAC_PIP_MASK             0x03ffffffu
#define DMAC_PIP_HOLE(p)          ((p) & 0xffffu)
#define DMAC_PIP_BOUNDARY(p)      (((p) >> 16) & 0x3ffu)

#define DMAC_FC_MEM2PER          1
#define DMAC_FC_PER2MEM          2
#define DMAC_FC_PER2MEM_PER      4
#define DMAC_FC_MEM2PER_PER      5

/* EBCISR/EBCIMR bits */
#define DMAC_BTC(x)   (1u << (x))          /* buffer transfer completed */
#define DMAC_CBTC(x)  (1u << (8 + (x)))    /* chained buffer completed  */
#define DMAC_ERR(x)   (1u << (16 + (x)))   /* AHB access error          */

#define DMAC_SW_SRC_REQ(x) (1u << (2 * (x)))
#define DMAC_SW_DST_REQ(x) (1u << (1 + 2 * (x)))

typedef struct AT91DmacChan {
    uint32_t saddr, daddr, dscr, ctrla, ctrlb, cfg, spip, dpip;
    uint32_t active_dscr; /* descriptor currently loaded, for writeback */
    uint16_t src_pip_count;
    uint16_t dst_pip_count;
    uint8_t fifo[8];
    uint8_t fifo_fill;
    bool cyclic;   /* peripheral-paced cyclic list (UART/audio RX): not run */
} AT91DmacChan;

/*
 * Offset of the "next descriptor" (DSCR) field within a hardware LLI, which
 * is {saddr, daddr, ctrla, ctrlb, next} = 5 words.
 */
#define DMAC_LLI_NEXT_OFF   16

/*
 * A cyclic (looping) descriptor list is used for peripheral-paced receive
 * (e.g. the USART/DBGU RX ring): its last LLI's next pointer loops back to an
 * earlier LLI instead of terminating at 0.
 */
static bool dmac_chain_is_cyclic(uint32_t head)
{
    uint32_t cur = head;
    int g = 0;

    while (cur != 0 && g++ < 1024) {
        uint32_t nxt = address_space_ldl_le(&address_space_memory,
                                            (cur & ~3u) +
                                            DMAC_LLI_NEXT_OFF,
                                            MEMTXATTRS_UNSPECIFIED, NULL);
        if ((nxt & ~3u) == (head & ~3u)) {
            return true;
        }
        cur = nxt;
    }
    return false;
}

struct AT91DmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUBH *bh;                /* runs transfers asynchronously */
    uint32_t pending;          /* channels awaiting transfer */
    uint64_t requests;         /* levels of connected peripheral requests */
    uint64_t request_mask;     /* requests for which flow control is modelled */

    uint32_t gcfg, en, sreq, creq, last, ebcimr, ebcisr, chsr;
    AT91DmacChan ch[DMAC_N_CHANNELS];
};

static uint32_t dmac_channel_ctrlb(AT91DmacState *s, int n)
{
    uint32_t ctrlb = s->ch[n].ctrlb;

    if (!s->ch[n].active_dscr && s->ch[n].dscr) {
        ctrlb = address_space_ldl_le(&address_space_memory,
                                     (s->ch[n].dscr & ~3u) + 12,
                                     MEMTXATTRS_UNSPECIFIED, NULL);
    }
    return ctrlb;
}

static bool dmac_source_is_peripheral(uint32_t fc)
{
    return fc == 2 || fc == 3 || fc == 4 || fc == 6 || fc == 7;
}

static bool dmac_destination_is_peripheral(uint32_t fc)
{
    return fc == 1 || fc == 3 || fc == 5 || fc == 6 || fc == 7;
}

static bool dmac_source_controls_flow(uint32_t fc)
{
    return fc == 4 || fc == 6;
}

static bool dmac_destination_controls_flow(uint32_t fc)
{
    return fc == 5 || fc == 7;
}

static int dmac_channel_request_id(AT91DmacState *s, int n, bool source)
{
    uint32_t cfg = s->ch[n].cfg;

    if (source) {
        return (cfg & 0xf) | (((cfg >> 10) & 0x3) << 4);
    }
    return ((cfg >> 4) & 0xf) | (((cfg >> 14) & 0x3) << 4);
}

static bool dmac_channel_side_ready(AT91DmacState *s, int n, bool source,
                                    uint32_t fc)
{
    uint32_t h2sel = source ? DMAC_CFG_SRC_H2SEL : DMAC_CFG_DST_H2SEL;
    uint32_t sw_bit = source ? DMAC_SW_SRC_REQ(n) : DMAC_SW_DST_REQ(n);
    bool peripheral = source ? dmac_source_is_peripheral(fc) :
                               dmac_destination_is_peripheral(fc);
    int request;

    if (!peripheral) {
        return true;
    }
    if (!(s->ch[n].cfg & h2sel)) {
        return (s->sreq | s->creq) & sw_bit;
    }

    request = dmac_channel_request_id(s, n, source);
    return !(s->request_mask & (UINT64_C(1) << request)) ||
           (s->requests & (UINT64_C(1) << request));
}

static bool dmac_channel_ready(AT91DmacState *s, int n)
{
    uint32_t fc = DMAC_CTRLB_FC(dmac_channel_ctrlb(s, n));
    uint32_t channel_bit = 1u << n;

    return (s->en & 1) && !(s->chsr & (channel_bit << 8)) &&
           dmac_channel_side_ready(s, n, true, fc) &&
           dmac_channel_side_ready(s, n, false, fc);
}

static bool dmac_channel_uses_hardware_request(AT91DmacState *s, int n,
                                               int request)
{
    uint32_t fc = DMAC_CTRLB_FC(dmac_channel_ctrlb(s, n));

    return (dmac_source_is_peripheral(fc) &&
            (s->ch[n].cfg & DMAC_CFG_SRC_H2SEL) &&
            dmac_channel_request_id(s, n, true) == request) ||
           (dmac_destination_is_peripheral(fc) &&
            (s->ch[n].cfg & DMAC_CFG_DST_H2SEL) &&
            dmac_channel_request_id(s, n, false) == request);
}

static void dmac_schedule_channel(AT91DmacState *s, int n)
{
    if (!s->ch[n].cyclic && dmac_channel_ready(s, n)) {
        s->pending |= 1u << n;
        qemu_bh_schedule(s->bh);
    }
}

static void dmac_request(void *opaque, int request, int level)
{
    AT91DmacState *s = opaque;
    uint64_t bit = UINT64_C(1) << request;
    int n;

    if (level) {
        s->requests |= bit;
    } else {
        s->requests &= ~bit;
        return;
    }

    for (n = 0; n < DMAC_N_CHANNELS; n++) {
        if ((s->chsr & (1u << n)) && !s->ch[n].cyclic &&
            dmac_channel_uses_hardware_request(s, n, request)) {
            dmac_schedule_channel(s, n);
        }
    }
}

static void dmac_update_irq(AT91DmacState *s)
{
    qemu_set_irq(s->irq, (s->ebcisr & s->ebcimr) ? 1 : 0);
}

static unsigned dmac_width_bytes(unsigned w)
{
    /* Both encodings 2 and 3 mean WORD on this IP revision. */
    return w >= 2 ? 4 : 1u << w;
}

static unsigned dmac_chunk_beats(unsigned encoding)
{
    return encoding ? 1u << (encoding + 1) : 1;
}

static void dmac_advance_address(uint32_t *address, int mode, unsigned width,
                                 bool pip_enabled, uint32_t pip,
                                 uint16_t *pip_count)
{
    uint32_t boundary = DMAC_PIP_BOUNDARY(pip);

    /*
     * At a PiP boundary the HOLE increment replaces the ordinary address-mode
     * step.  This is why Linux programs (inter-chunk gap / width) + 1: a value
     * of one is the same advance as an ordinary incrementing transfer.
     */
    if (pip_enabled && boundary && ++(*pip_count) >= boundary) {
        *address += DMAC_PIP_HOLE(pip) * width;
        *pip_count = 0;
        return;
    }

    if (mode == 0) {
        *address += width;
    } else if (mode == 1) {
        *address -= width;
    }
}

/*
 * Run at most one source and/or destination transaction.  Hardware-requested
 * paths pass unlimited limits and retain the old whole-buffer abstraction;
 * software requests pass one beat or the programmed chunk size.  Keeping the
 * tiny conversion FIFO in channel state makes short transactions and migration
 * correct even when source and destination widths differ.
 */
static MemTxResult dmac_run_transaction(AT91DmacChan *c,
                                        uint32_t source_limit,
                                        uint32_t destination_limit)
{
    uint32_t btsize = DMAC_CTRLA_BTSIZE(c->ctrla);
    unsigned sw = dmac_width_bytes(DMAC_CTRLA_SRC_WIDTH(c->ctrla));
    unsigned dw = dmac_width_bytes(DMAC_CTRLA_DST_WIDTH(c->ctrla));
    int smode = DMAC_CTRLB_SRC_MODE(c->ctrlb);
    int dmode = DMAC_CTRLB_DST_MODE(c->ctrlb);
    uint32_t source_done = 0;
    uint32_t destination_done = 0;
    MemTxResult result;

    while (true) {
        while (c->fifo_fill >= dw &&
               destination_done < destination_limit) {
            result = address_space_write(&address_space_memory, c->daddr,
                                         MEMTXATTRS_UNSPECIFIED,
                                         c->fifo, dw);
            if (result != MEMTX_OK) {
                return result;
            }
            dmac_advance_address(&c->daddr, dmode, dw,
                                 c->ctrlb & DMAC_CTRLB_DST_PIP,
                                 c->dpip, &c->dst_pip_count);
            destination_done++;
            c->fifo_fill -= dw;
            if (c->fifo_fill) {
                memmove(c->fifo, c->fifo + dw, c->fifo_fill);
            }
        }

        if (destination_done >= destination_limit || btsize == 0 ||
            source_done >= source_limit) {
            break;
        }

        g_assert(c->fifo_fill + sw <= sizeof(c->fifo));
        result = address_space_read(&address_space_memory, c->saddr,
                                    MEMTXATTRS_UNSPECIFIED,
                                    c->fifo + c->fifo_fill, sw);
        if (result != MEMTX_OK) {
            return result;
        }
        c->fifo_fill += sw;
        dmac_advance_address(&c->saddr, smode, sw,
                             c->ctrlb & DMAC_CTRLB_SRC_PIP,
                             c->spip, &c->src_pip_count);
        source_done++;
        btsize--;
        c->ctrla = (c->ctrla & ~0xffffu) | btsize;
    }

    /* Invalid partial-width buffers cannot form another destination beat. */
    if (btsize == 0 && c->fifo_fill < dw) {
        c->fifo_fill = 0;
    }
    return MEMTX_OK;
}

static bool dmac_buffer_complete(AT91DmacChan *c)
{
    return DMAC_CTRLA_BTSIZE(c->ctrla) == 0 && c->fifo_fill == 0;
}

static void dmac_clear_software_channel(AT91DmacState *s, int n)
{
    uint32_t mask = 3u << (2 * n);

    s->sreq &= ~mask;
    s->creq &= ~mask;
    s->last &= ~mask;
}

static void dmac_finish_channel(AT91DmacState *s, int n, bool chained)
{
    uint32_t channel_bit = 1u << n;

    if (chained) {
        s->ebcisr |= DMAC_CBTC(n);
    }
    s->chsr &= ~(channel_bit | (channel_bit << 8) | (channel_bit << 24));
    s->chsr |= channel_bit << 16;
    s->pending &= ~channel_bit;
    s->ch[n].active_dscr = 0;
    s->ch[n].src_pip_count = 0;
    s->ch[n].dst_pip_count = 0;
    s->ch[n].fifo_fill = 0;
    s->ch[n].cyclic = false;
    dmac_clear_software_channel(s, n);
    trace_at91_dmac_complete(n);
    dmac_update_irq(s);
}

static void dmac_fail_channel(AT91DmacState *s, int n)
{
    s->ebcisr |= DMAC_ERR(n);
    dmac_finish_channel(s, n, false);
}

static MemTxResult dmac_load_descriptor(AT91DmacChan *c)
{
    uint32_t descriptor = c->dscr & ~3u;
    uint32_t previous_ctrlb = c->ctrlb;
    uint32_t d[5];
    MemTxResult result;
    int i;

    for (i = 0; i < ARRAY_SIZE(d); i++) {
        d[i] = address_space_ldl_le(&address_space_memory,
                                    descriptor + i * sizeof(uint32_t),
                                    MEMTXATTRS_UNSPECIFIED, &result);
        if (result != MEMTX_OK) {
            return result;
        }
    }

    if (!(previous_ctrlb & DMAC_CTRLB_SRC_DSCR_DIS)) {
        c->saddr = d[0];
    }
    if (!(previous_ctrlb & DMAC_CTRLB_DST_DSCR_DIS)) {
        c->daddr = d[1];
    }
    c->ctrla = d[2];
    c->ctrlb = d[3];
    c->dscr = d[4];
    c->active_dscr = descriptor;
    c->src_pip_count = 0;
    c->dst_pip_count = 0;
    c->fifo_fill = 0;
    return MEMTX_OK;
}

static MemTxResult dmac_writeback_descriptor(AT91DmacChan *c)
{
    MemTxResult result;

    address_space_stl_le(&address_space_memory, c->active_dscr + 8,
                         c->ctrla | DMAC_CTRLA_DONE,
                         MEMTXATTRS_UNSPECIFIED, &result);
    return result;
}

static void dmac_run_channel(AT91DmacState *s, int n)
{
    AT91DmacChan *c = &s->ch[n];
    uint32_t channel_bit = 1u << n;
    int guard = 0;

    while (s->chsr & channel_bit) {
        uint32_t source_bit = DMAC_SW_SRC_REQ(n);
        uint32_t destination_bit = DMAC_SW_DST_REQ(n);
        uint32_t source_limit = UINT32_MAX;
        uint32_t destination_limit = UINT32_MAX;
        uint32_t fc;
        bool source_software, destination_software;
        bool source_single = false, source_chunk = false;
        bool destination_single = false, destination_chunk = false;
        bool source_last, destination_last;
        bool descriptor;
        MemTxResult result;

        if (!c->active_dscr && c->dscr) {
            if (++guard > 1024) {
                dmac_fail_channel(s, n);
                return;
            }
            result = dmac_load_descriptor(c);
            if (result != MEMTX_OK) {
                dmac_fail_channel(s, n);
                return;
            }
            if ((c->cfg & DMAC_CFG_SOD) &&
                (c->ctrla & DMAC_CTRLA_DONE)) {
                dmac_finish_channel(s, n, false);
                return;
            }
        }

        descriptor = c->active_dscr != 0;
        fc = DMAC_CTRLB_FC(c->ctrlb);
        source_software = dmac_source_is_peripheral(fc) &&
                          !(c->cfg & DMAC_CFG_SRC_H2SEL);
        destination_software = dmac_destination_is_peripheral(fc) &&
                               !(c->cfg & DMAC_CFG_DST_H2SEL);

        if (source_software) {
            source_single = s->sreq & source_bit;
            source_chunk = !source_single && (s->creq & source_bit);
            if (!source_single && !source_chunk) {
                return;
            }
            source_limit = source_single ? 1 :
                           dmac_chunk_beats(DMAC_CTRLA_SCSIZE(c->ctrla));
        }
        if (destination_software) {
            destination_single = s->sreq & destination_bit;
            destination_chunk = !destination_single &&
                                (s->creq & destination_bit);
            if (!destination_single && !destination_chunk) {
                return;
            }
            destination_limit = destination_single ? 1 :
                                dmac_chunk_beats(
                                    DMAC_CTRLA_DCSIZE(c->ctrla));
        }

        source_last = source_software && (s->last & source_bit) &&
                      dmac_source_controls_flow(fc);
        destination_last = destination_software &&
                           (s->last & destination_bit) &&
                           dmac_destination_controls_flow(fc);

        trace_at91_dmac_run(n, c->saddr, c->daddr,
                            DMAC_CTRLA_BTSIZE(c->ctrla));
        s->chsr &= ~(channel_bit << 16);
        result = dmac_run_transaction(c, source_limit, destination_limit);
        if (result != MEMTX_OK) {
            dmac_fail_channel(s, n);
            return;
        }

        if (source_single) {
            s->sreq &= ~source_bit;
        } else if (source_chunk) {
            s->creq &= ~source_bit;
        }
        if (destination_single) {
            s->sreq &= ~destination_bit;
        } else if (destination_chunk) {
            s->creq &= ~destination_bit;
        }
        if (source_software) {
            s->last &= ~source_bit;
        }
        if (destination_software) {
            s->last &= ~destination_bit;
        }

        /*
         * A peripheral flow controller ends the buffer with LAST rather than
         * with the programmed BTSIZE.  Any bytes already in the FIFO still
         * have to reach the destination before completion.
         */
        if (source_last || destination_last) {
            c->ctrla &= ~0xffffu;
        }

        if (!dmac_buffer_complete(c)) {
            if (c->fifo_fill) {
                s->chsr &= ~(channel_bit << 16);
            } else {
                s->chsr |= channel_bit << 16;
            }
            dmac_schedule_channel(s, n);
            dmac_update_irq(s);
            return;
        }

        s->chsr |= channel_bit << 16;
        if (!(c->ctrlb & DMAC_CTRLB_IEN)) {
            s->ebcisr |= DMAC_BTC(n);
        }

        if (!descriptor) {
            dmac_finish_channel(s, n, false);
            return;
        }

        result = dmac_writeback_descriptor(c);
        if (result != MEMTX_OK) {
            dmac_fail_channel(s, n);
            return;
        }
        c->active_dscr = 0;
        c->fifo_fill = 0;

        if (!c->dscr) {
            dmac_finish_channel(s, n, true);
            return;
        }

        /*
         * A single software request completes one transaction.  Continue a
         * chain only when another request is already pending (or when its
         * next descriptor is hardware/memory paced).
         */
        if (!dmac_channel_ready(s, n)) {
            return;
        }
    }
}

/*
 * Transfers run from a bottom half so completion (and its interrupt) happen
 * after the enabling MMIO write returns - real DMA completes asynchronously,
 * and a reentrant completion corrupts driver state machines (for example,
 * atmel-mci).
 */
static void dmac_bh(void *opaque)
{
    AT91DmacState *s = opaque;

    while (s->pending) {
        int n = ctz32(s->pending);
        s->pending &= ~(1u << n);
        if (!(s->chsr & (1u << n)) || !dmac_channel_ready(s, n)) {
            continue;
        }
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
        case DMAC_DSCR:
            /*
             * For a cyclic RX channel that we deliberately leave idle (no DMA
             * flow-control modelled), report the head descriptor's "next"
             * pointer: real hardware advances DSCR to the next LLI as soon as
             * it loads the current one.  Together with CTRLA==0 this makes the
             * at_hdmac residue calculation take the first-descriptor path and
             * compute residue == total_len, i.e. "nothing received" - so the
             * USART driver does not push a flood of phantom RX bytes.
             */
            if (s->ch[n].cyclic && s->ch[n].dscr != 0) {
                return address_space_ldl_le(&address_space_memory,
                                            (s->ch[n].dscr & ~3u) +
                                            DMAC_LLI_NEXT_OFF,
                                            MEMTXATTRS_UNSPECIFIED, NULL);
            }
            return s->ch[n].dscr;
        case DMAC_CTRLA:
            return s->ch[n].cyclic ? 0 : s->ch[n].ctrla;
        case DMAC_CTRLB:
            return s->ch[n].ctrlb;
        case DMAC_CFG:
            return s->ch[n].cfg;
        case DMAC_SPIP:
            return s->ch[n].spip;
        case DMAC_DPIP:
            return s->ch[n].dpip;
        default:
            return 0;
        }
    }

    switch (offset) {
    case DMAC_GCFG:
        r = s->gcfg;
        break;
    case DMAC_EN:
        r = s->en;
        break;
    case DMAC_SREQ:
        r = s->sreq;
        break;
    case DMAC_CREQ:
        r = s->creq;
        break;
    case DMAC_LAST:
        r = s->last;
        break;
    case DMAC_EBCIMR:
        r = s->ebcimr;
        break;
    case DMAC_EBCISR:                   /* read clears the status */
        r = s->ebcisr;
        s->ebcisr = 0;
        dmac_update_irq(s);
        break;
    case DMAC_CHSR:
        r = s->chsr;
        break;
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
        case DMAC_SADDR:
            s->ch[n].saddr = val;
            break;
        case DMAC_DADDR:
            s->ch[n].daddr = val;
            break;
        case DMAC_DSCR:
            s->ch[n].dscr = val;
            s->ch[n].active_dscr = 0;
            s->ch[n].src_pip_count = 0;
            s->ch[n].dst_pip_count = 0;
            s->ch[n].fifo_fill = 0;
            break;
        case DMAC_CTRLA:
            s->ch[n].ctrla = val;
            break;
        case DMAC_CTRLB:
            s->ch[n].ctrlb = val;
            break;
        case DMAC_CFG:
            s->ch[n].cfg = val;
            break;
        case DMAC_SPIP:
            s->ch[n].spip = val & DMAC_PIP_MASK;
            break;
        case DMAC_DPIP:
            s->ch[n].dpip = val & DMAC_PIP_MASK;
            break;
        default:
            break;
        }
        return;
    }

    switch (offset) {
    case DMAC_GCFG:
        s->gcfg = val & 0x10;
        break;
    case DMAC_EN:
        s->en = val & 0x1;
        if (!s->en) {
            int n;

            /* Global disable is an abrupt request to stop every channel. */
            s->chsr = DMAC_EMPTY_MASK;
            s->pending = 0;
            s->sreq = 0;
            s->creq = 0;
            s->last = 0;
            for (n = 0; n < DMAC_N_CHANNELS; n++) {
                s->ch[n].active_dscr = 0;
                s->ch[n].src_pip_count = 0;
                s->ch[n].dst_pip_count = 0;
                s->ch[n].fifo_fill = 0;
                s->ch[n].cyclic = false;
            }
        }
        break;
    case DMAC_SREQ: {
        int n;

        s->sreq |= val & 0xffff;
        for (n = 0; n < DMAC_N_CHANNELS; n++) {
            if (val & (3u << (2 * n))) {
                dmac_schedule_channel(s, n);
            }
        }
        break;
    }
    case DMAC_CREQ: {
        int n;

        s->creq |= val & 0xffff;
        for (n = 0; n < DMAC_N_CHANNELS; n++) {
            if (val & (3u << (2 * n))) {
                dmac_schedule_channel(s, n);
            }
        }
        break;
    }
    case DMAC_LAST:
        s->last |= val & 0xffff;
        break;
    case DMAC_EBCIER:
        s->ebcimr |= val & 0x00ffffff;
        dmac_update_irq(s);
        break;
    case DMAC_EBCIDR:
        s->ebcimr &= ~(val & 0x00ffffff);
        dmac_update_irq(s);
        break;
    case DMAC_CHER: {
        int n;

        if (!s->en) {
            break;
        }
        for (n = 0; n < DMAC_N_CHANNELS; n++) {
            uint32_t channel_bit = 1u << n;

            if (val & channel_bit) {
                bool was_enabled = s->chsr & channel_bit;

                s->chsr |= channel_bit | (channel_bit << 16);
                if (!was_enabled) {
                    s->ch[n].active_dscr = 0;
                    s->ch[n].src_pip_count = 0;
                    s->ch[n].dst_pip_count = 0;
                    s->ch[n].fifo_fill = 0;
                }
                /*
                 * A cyclic list is peripheral-paced (UART/audio RX), driven by
                 * the device's DMA request which we do not model.  Running it
                 * would fabricate phantom data; instead flag it and leave it
                 * "running" - its residue reads report an idle head so the
                 * driver sees "nothing received".
                 */
                if (s->ch[n].dscr != 0 &&
                    dmac_chain_is_cyclic(s->ch[n].dscr)) {
                    s->ch[n].cyclic = true;
                } else {
                    s->ch[n].cyclic = false;
                }
            }
            if ((val & (channel_bit << 8)) &&
                (s->chsr & channel_bit)) {
                s->chsr |= channel_bit << 8;
                s->pending &= ~channel_bit;
            }
            if (val & (channel_bit << 24)) {
                s->chsr &= ~(channel_bit << 24);
            }
        }
        for (n = 0; n < DMAC_N_CHANNELS; n++) {
            uint32_t channel_bit = 1u << n;

            if ((val & (channel_bit | (channel_bit << 24))) &&
                (s->chsr & channel_bit)) {
                dmac_schedule_channel(s, n);
            }
        }
        break;
    }
    case DMAC_CHDR: {
        int n;

        for (n = 0; n < DMAC_N_CHANNELS; n++) {
            uint32_t channel_bit = 1u << n;

            if (val & channel_bit) {
                s->chsr &= ~(channel_bit | (channel_bit << 8) |
                             (channel_bit << 24));
                s->chsr |= channel_bit << 16;
                s->pending &= ~channel_bit;
                s->ch[n].active_dscr = 0;
                s->ch[n].src_pip_count = 0;
                s->ch[n].dst_pip_count = 0;
                s->ch[n].fifo_fill = 0;
                s->ch[n].cyclic = false;
                dmac_clear_software_channel(s, n);
            } else if (val & (channel_bit << 8)) {
                s->chsr &= ~(channel_bit << 8);
                if (s->chsr & channel_bit) {
                    dmac_schedule_channel(s, n);
                }
            }
        }
        break;
    }
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
    s->sreq = 0;
    s->creq = 0;
    s->last = 0;
    s->ebcimr = 0;
    s->ebcisr = 0;
    s->chsr = DMAC_EMPTY_MASK;
    s->pending = 0;
    s->requests = 0;
    memset(s->ch, 0, sizeof(s->ch));
    for (int n = 0; n < DMAC_N_CHANNELS; n++) {
        s->ch[n].cfg = DMAC_CFG_RESET;
    }
}

static void dmac_dev_init(Object *obj)
{
    AT91DmacState *s = AT91_DMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &dmac_ops, s, "at91-dmac", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), dmac_request,
                            AT91_DMAC_REQUEST_GPIO,
                            AT91_DMAC_MAX_REQUESTS);
}

static void dmac_realize(DeviceState *dev, Error **errp)
{
    AT91DmacState *s = AT91_DMAC(dev);

    s->bh = qemu_bh_new(dmac_bh, s);
}

static int dmac_post_load(void *opaque, int version_id)
{
    AT91DmacState *s = opaque;
    int n;

    for (n = 0; n < DMAC_N_CHANNELS; n++) {
        if (s->ch[n].fifo_fill > sizeof(s->ch[n].fifo) ||
            s->ch[n].src_pip_count > 0x3ff ||
            s->ch[n].dst_pip_count > 0x3ff) {
            return -EINVAL;
        }
    }

    /* Output IRQ levels are wiring, not migration state. */
    dmac_update_irq(s);

    /*
     * The bottom half is not migrated; if a transfer was still pending, run it
     * after load.
     */
    if (s->pending) {
        qemu_bh_schedule(s->bh);
    }
    return 0;
}

static const VMStateDescription vmstate_at91_dmac_chan = {
    .name = "at91-dmac-chan",
    .version_id = 4,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(saddr, AT91DmacChan),
        VMSTATE_UINT32(daddr, AT91DmacChan),
        VMSTATE_UINT32(dscr, AT91DmacChan),
        VMSTATE_UINT32(ctrla, AT91DmacChan),
        VMSTATE_UINT32(ctrlb, AT91DmacChan),
        VMSTATE_UINT32(cfg, AT91DmacChan),
        VMSTATE_UINT32_V(spip, AT91DmacChan, 4),
        VMSTATE_UINT32_V(dpip, AT91DmacChan, 4),
        VMSTATE_UINT16_V(src_pip_count, AT91DmacChan, 4),
        VMSTATE_UINT16_V(dst_pip_count, AT91DmacChan, 4),
        VMSTATE_BOOL_V(cyclic, AT91DmacChan, 2),
        VMSTATE_UINT32_V(active_dscr, AT91DmacChan, 3),
        VMSTATE_UINT8_ARRAY_V(fifo, AT91DmacChan, 8, 3),
        VMSTATE_UINT8_V(fifo_fill, AT91DmacChan, 3),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_at91_dmac = {
    .name = "at91-dmac",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = dmac_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(gcfg, AT91DmacState),
        VMSTATE_UINT32(en, AT91DmacState),
        VMSTATE_UINT32_V(sreq, AT91DmacState, 2),
        VMSTATE_UINT32_V(creq, AT91DmacState, 2),
        VMSTATE_UINT32_V(last, AT91DmacState, 2),
        VMSTATE_UINT32(ebcimr, AT91DmacState),
        VMSTATE_UINT32(ebcisr, AT91DmacState),
        VMSTATE_UINT32(chsr, AT91DmacState),
        VMSTATE_UINT32(pending, AT91DmacState),
        VMSTATE_STRUCT_ARRAY(ch, AT91DmacState, DMAC_N_CHANNELS, 1,
                             vmstate_at91_dmac_chan, AT91DmacChan),
        VMSTATE_END_OF_LIST()
    }
};

static const Property dmac_properties[] = {
    DEFINE_PROP_UINT64(AT91_DMAC_REQUEST_MASK, AT91DmacState, request_mask, 0),
};

static void dmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = dmac_realize;
    device_class_set_legacy_reset(dc, dmac_reset);
    device_class_set_props(dc, dmac_properties);
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
