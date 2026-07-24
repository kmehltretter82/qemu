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
#include "qemu/main-loop.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/clock.h"
#include "hw/core/qdev-clock.h"
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
#define HSMCI_DMA_DMAEN (1u << 8)
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
#define HSMCI_CMDR_SPCMD(c)    (((c) >> 8) & 0x7)
#define HSMCI_CMDR_MAXLAT(c)   (((c) >> 12) & 0x1)
#define HSMCI_CMDR_TRCMD(c)    (((c) >> 16) & 0x3)
#define HSMCI_CMDR_TRDIR_READ  (1u << 18)
#define HSMCI_CMDR_TRTYP(c)    (((c) >> 19) & 0x7)
#define HSMCI_RSPTYP_NONE   0
#define HSMCI_RSPTYP_48     1
#define HSMCI_RSPTYP_136    2
#define HSMCI_RSPTYP_R1B    3
#define HSMCI_SPCMD_INIT     1
#define HSMCI_TRCMD_START   1
#define HSMCI_TRCMD_STOP    2
#define HSMCI_TRTYP_SINGLE  0
#define HSMCI_TRTYP_MULTI   1
#define HSMCI_TRTYP_STREAM  2
#define HSMCI_TRTYP_SDIO_BYTE  4
#define HSMCI_TRTYP_SDIO_BLOCK 5

/* Block Register fields */
#define HSMCI_BLKR_BCNT(b)     ((b) & 0xFFFF)
#define HSMCI_BLKR_BLKLEN(b)   (((b) >> 16) & 0xFFFF)

/* Status/Interrupt bits */
#define HSMCI_SR_CMDRDY   (1u << 0)
#define HSMCI_SR_RXRDY    (1u << 1)
#define HSMCI_SR_TXRDY    (1u << 2)
#define HSMCI_SR_BLKE     (1u << 3)
#define HSMCI_SR_DTIP     (1u << 4)
#define HSMCI_SR_NOTBUSY  (1u << 5)
#define HSMCI_SR_SDIOIRQA (1u << 8)
#define HSMCI_SR_CSRCV    (1u << 13)
#define HSMCI_SR_RINDE    (1u << 16)
#define HSMCI_SR_RDIRE    (1u << 17)
#define HSMCI_SR_RCRCE    (1u << 18)
#define HSMCI_SR_RENDE    (1u << 19)
#define HSMCI_SR_RTOE     (1u << 20)  /* response time-out (no card responded) */
#define HSMCI_SR_DCRCE    (1u << 21)
#define HSMCI_SR_DTOE     (1u << 22)
#define HSMCI_SR_CSTOE    (1u << 23)
#define HSMCI_SR_BLKOVRE  (1u << 24)
#define HSMCI_SR_DMADONE  (1u << 25)
#define HSMCI_SR_XFRDONE  (1u << 27)
#define HSMCI_SR_ACKRCV   (1u << 28)
#define HSMCI_SR_ACKRCVE  (1u << 29)
#define HSMCI_SR_OVRE     (1u << 30)
#define HSMCI_SR_UNRE     (1u << 31)
#define HSMCI_SR_RESET    0x0000C0E5
#define HSMCI_SR_COMMAND_ERRORS \
    (HSMCI_SR_RINDE | HSMCI_SR_RDIRE | HSMCI_SR_RCRCE | \
     HSMCI_SR_RENDE | HSMCI_SR_RTOE)
#define HSMCI_SR_READ_CLEAR \
    (HSMCI_SR_BLKE | HSMCI_SR_SDIOIRQA | HSMCI_SR_CSRCV | \
     HSMCI_SR_DCRCE | HSMCI_SR_DTOE | HSMCI_SR_CSTOE | \
     HSMCI_SR_BLKOVRE | HSMCI_SR_DMADONE | HSMCI_SR_ACKRCV | \
     HSMCI_SR_ACKRCVE)

#define HSMCI_WPMR_WPEN       (1u << 0)
#define HSMCI_WPMR_KEY(v)     (((v) >> 8) & 0xffffff)
#define HSMCI_WPMR_VALID_KEY  0x4d4349
#define HSMCI_WPSR_WPVS_MASK  0x3
#define HSMCI_WPSR_WPVS_WRITE 0x1
#define HSMCI_WPSR_WPVS_RESET 0x2

/* A command frame is 48 bits.  Responses need eight trailing card clocks. */
#define HSMCI_COMMAND_CYCLES       48
#define HSMCI_INIT_CYCLES          74
#define HSMCI_RESPONSE_48_CYCLES   (48 + 8)
#define HSMCI_RESPONSE_136_CYCLES  (136 + 8)
#define HSMCI_DATA_END_CYCLES      17

typedef enum HsmciCommandPhase {
    HSMCI_CMD_IDLE,
    HSMCI_CMD_TRANSMIT,
    HSMCI_CMD_RESPONSE,
    HSMCI_CMD_TIMEOUT,
} HsmciCommandPhase;

static void hsmci_reset(DeviceState *dev);

struct AT91HsmciState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;
    qemu_irq dma_request;
    Clock *mck;
    QEMUTimer *command_timer;
    QEMUTimer *transfer_timer;
    QEMUTimer *timeout_timer;
    QEMUBH *dma_request_bh;

    uint32_t mr, dtor, sdcr, argr, blkr, cstor, dma, cfg, wpmr;
    uint32_t wpsr;
    uint32_t sr;
    uint32_t imr;
    uint32_t rsp[4];
    int rsp_ptr;

    uint32_t cmdr;
    uint8_t command_phase;
    bool enabled;
    bool command_pending;
    bool transfer_pending;
    bool infinite;
    bool sdio_irq_level;
    bool dma_request_rearm;
    uint32_t blklen;        /* current transfer block length */
    uint32_t block_remaining;
    int32_t  data_len;      /* bytes left in the current data transfer */
    uint64_t data_remaining;
    bool     reading;       /* transfer direction */

    uint64_t command_cycles;
    uint64_t transfer_cycles;
    uint32_t command_mck_hz;
    uint32_t command_divider;
    uint32_t transfer_mck_hz;
    uint32_t transfer_divider;
    int64_t command_remaining_ns;
    int64_t transfer_remaining_ns;

    bool timeout_pending;
    uint64_t timeout_cycles;
    uint32_t timeout_mck_hz;
    int64_t timeout_remaining_ns;
};

static void hsmci_update_irq(AT91HsmciState *s)
{
    qemu_set_irq(s->irq, (s->sr & s->imr) ? 1 : 0);
}

static bool hsmci_has_data(AT91HsmciState *s)
{
    return s->infinite || s->data_remaining > 0;
}

static void hsmci_update_dma_request(AT91HsmciState *s)
{
    /*
     * DMAEN gates the whole hardware-handshake interface; the driver
     * programs it before a DMA transfer and clears it for CPU (PIO)
     * transfers, so a PIO transfer never toggles the request line.
     * CHKSIZE and OFFSET are retained configuration only: card data is
     * synchronously available here, so the chunk-availability threshold
     * has no observable pacing effect.
     */
    qemu_set_irq(s->dma_request,
                 s->enabled && (s->dma & HSMCI_DMA_DMAEN) &&
                 !s->dma_request_rearm &&
                 hsmci_has_data(s) &&
                 (s->sr & (HSMCI_SR_RXRDY | HSMCI_SR_TXRDY)));
}

static void hsmci_dma_request_bh(void *opaque)
{
    AT91HsmciState *s = opaque;

    s->dma_request_rearm = false;
    hsmci_update_dma_request(s);
}

static void hsmci_rearm_dma_request(AT91HsmciState *s)
{
    if (!s->enabled || !(s->dma & HSMCI_DMA_DMAEN) ||
        !hsmci_has_data(s) ||
        !(s->sr & (HSMCI_SR_RXRDY | HSMCI_SR_TXRDY))) {
        return;
    }

    /*
     * Each FIFO word completes one peripheral handshake.  Drop the request
     * before scheduling the next one so the DMAC observes a distinct edge.
     */
    s->dma_request_rearm = true;
    hsmci_update_dma_request(s);
    qemu_bh_schedule(s->dma_request_bh);
}

static void hsmci_cancel_dma_rearm(AT91HsmciState *s)
{
    s->dma_request_rearm = false;
    if (s->dma_request_bh) {
        qemu_bh_cancel(s->dma_request_bh);
    }
}

static uint32_t hsmci_card_divider(AT91HsmciState *s)
{
    return 2 * ((s->mr & 0xff) + 1);
}

static uint64_t hsmci_cycles_to_ns(AT91HsmciState *s, uint64_t cycles)
{
    uint32_t mck_hz = clock_get_hz(s->mck);

    if (mck_hz == 0) {
        return 0;
    }
    return MAX(muldiv64_round_up(cycles * hsmci_card_divider(s),
                                 NANOSECONDS_PER_SECOND, mck_hz),
               UINT64_C(1));
}

static uint64_t hsmci_mck_cycles_to_ns(AT91HsmciState *s, uint64_t cycles)
{
    uint32_t mck_hz = clock_get_hz(s->mck);

    if (mck_hz == 0) {
        return 0;
    }
    return MAX(muldiv64_round_up(cycles, NANOSECONDS_PER_SECOND, mck_hz),
               UINT64_C(1));
}

/* DTOR: the data timeout is DTOCYC scaled by the DTOMUL multiplier. */
static uint64_t hsmci_dtor_cycles(AT91HsmciState *s)
{
    static const uint64_t multiplier[8] = {
        1, 16, 128, 256, 1024, 4096, 65536, 1048576
    };

    return (uint64_t)(s->dtor & 0xf) * multiplier[(s->dtor >> 4) & 7];
}

static uint64_t hsmci_pause_timer(QEMUTimer *timer, uint64_t cycles,
                                  uint32_t mck_hz, uint32_t divider)
{
    int64_t remaining;
    uint64_t mck_ticks;

    if (!timer_pending(timer)) {
        return cycles;
    }
    remaining = MAX(timer_expire_time_ns(timer) -
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), INT64_C(0));
    timer_del(timer);
    if (remaining == 0 || mck_hz == 0 || divider == 0) {
        return 1;
    }
    mck_ticks = muldiv64_round_up(remaining, mck_hz,
                                  NANOSECONDS_PER_SECOND);
    return MAX(DIV_ROUND_UP(mck_ticks, divider), UINT64_C(1));
}

static void hsmci_pause_timers(AT91HsmciState *s)
{
    if (s->command_pending) {
        s->command_cycles = hsmci_pause_timer(s->command_timer,
                                              s->command_cycles,
                                              s->command_mck_hz,
                                              s->command_divider);
    }
    if (s->transfer_pending) {
        s->transfer_cycles = hsmci_pause_timer(s->transfer_timer,
                                               s->transfer_cycles,
                                               s->transfer_mck_hz,
                                               s->transfer_divider);
    }
    if (s->timeout_pending) {
        s->timeout_cycles = hsmci_pause_timer(s->timeout_timer,
                                              s->timeout_cycles,
                                              s->timeout_mck_hz, 1);
    }
}

static void hsmci_schedule_command(AT91HsmciState *s, uint64_t cycles)
{
    uint64_t delay = hsmci_cycles_to_ns(s, cycles);

    s->command_cycles = cycles;
    s->command_mck_hz = clock_get_hz(s->mck);
    s->command_divider = hsmci_card_divider(s);
    if (s->enabled && delay) {
        timer_mod(s->command_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay);
    } else {
        timer_del(s->command_timer);
    }
}

static void hsmci_schedule_transfer(AT91HsmciState *s, uint64_t cycles)
{
    uint64_t delay = hsmci_cycles_to_ns(s, cycles);

    s->transfer_cycles = cycles;
    s->transfer_mck_hz = clock_get_hz(s->mck);
    s->transfer_divider = hsmci_card_divider(s);
    if (s->enabled && delay) {
        timer_mod(s->transfer_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay);
    } else {
        timer_del(s->transfer_timer);
    }
}

static void hsmci_schedule_data_timeout(AT91HsmciState *s, uint64_t cycles)
{
    uint64_t delay = hsmci_mck_cycles_to_ns(s, cycles);

    s->timeout_cycles = cycles;
    s->timeout_mck_hz = clock_get_hz(s->mck);
    if (s->enabled && delay) {
        timer_mod(s->timeout_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay);
    } else {
        timer_del(s->timeout_timer);
    }
}

static void hsmci_cancel_data_timeout(AT91HsmciState *s)
{
    s->timeout_pending = false;
    s->timeout_cycles = 0;
    timer_del(s->timeout_timer);
}

/*
 * DTOR counts Master Clock cycles that the controller waits between two
 * data accesses; the counter restarts at every access.  DTOCYC == 0 is
 * left disabled: real drivers (Linux atmel-mci, U-Boot gen_atmel_mci)
 * always program DTOR before a data command, and an immediate timeout
 * would fire on any CPU-paced transfer.
 */
static void hsmci_arm_data_timeout(AT91HsmciState *s)
{
    uint64_t cycles = hsmci_dtor_cycles(s);

    if (!cycles || !hsmci_has_data(s)) {
        hsmci_cancel_data_timeout(s);
        return;
    }
    s->timeout_pending = true;
    hsmci_schedule_data_timeout(s, cycles);
}

static void hsmci_resume_timers(AT91HsmciState *s)
{
    if (s->command_pending) {
        hsmci_schedule_command(s, s->command_cycles);
    }
    if (s->transfer_pending) {
        hsmci_schedule_transfer(s, s->transfer_cycles);
    }
    if (s->timeout_pending) {
        hsmci_schedule_data_timeout(s, s->timeout_cycles);
    }
}

static void hsmci_clock_update(void *opaque, ClockEvent event)
{
    AT91HsmciState *s = opaque;

    hsmci_pause_timers(s);
    hsmci_resume_timers(s);
}

static void hsmci_sync_data_len(AT91HsmciState *s)
{
    s->data_len = s->infinite ? INT32_MAX :
                  (int32_t)MIN(s->data_remaining, (uint64_t)INT32_MAX);
}

static void hsmci_finish_transfer(AT91HsmciState *s)
{
    hsmci_cancel_dma_rearm(s);
    hsmci_cancel_data_timeout(s);
    timer_del(s->transfer_timer);
    s->transfer_pending = false;
    s->transfer_cycles = 0;
    s->infinite = false;
    s->data_remaining = 0;
    s->data_len = 0;
    s->block_remaining = 0;
    s->sr &= ~(HSMCI_SR_RXRDY | HSMCI_SR_DTIP);
    s->sr |= HSMCI_SR_TXRDY | HSMCI_SR_NOTBUSY | HSMCI_SR_XFRDONE;
    hsmci_update_dma_request(s);
    hsmci_update_irq(s);
}

static void hsmci_transfer_timer_cb(void *opaque)
{
    AT91HsmciState *s = opaque;

    hsmci_finish_transfer(s);
}

static void hsmci_data_timeout_cb(void *opaque)
{
    AT91HsmciState *s = opaque;

    s->timeout_pending = false;
    s->sr |= HSMCI_SR_DTOE;
    hsmci_finish_transfer(s);
}

static void hsmci_schedule_transfer_end(AT91HsmciState *s)
{
    hsmci_cancel_dma_rearm(s);
    hsmci_cancel_data_timeout(s);
    s->transfer_pending = true;
    s->sr &= ~(HSMCI_SR_RXRDY | HSMCI_SR_TXRDY);
    hsmci_update_dma_request(s);
    hsmci_update_irq(s);
    hsmci_schedule_transfer(s, HSMCI_DATA_END_CYCLES);
}

static void hsmci_advance_block(AT91HsmciState *s, uint32_t bytes)
{
    while (bytes && s->block_remaining) {
        uint32_t step = MIN(bytes, s->block_remaining);

        s->block_remaining -= step;
        bytes -= step;
        if (s->block_remaining == 0) {
            if (!s->reading) {
                s->sr |= HSMCI_SR_BLKE;
            }
            s->block_remaining = MAX(s->blklen, 1u);
        }
    }
}

static void hsmci_abort_transfer(AT91HsmciState *s)
{
    if (hsmci_has_data(s) || s->transfer_pending ||
        (s->sr & HSMCI_SR_DTIP)) {
        hsmci_finish_transfer(s);
    }
}

static void hsmci_start_transfer(AT91HsmciState *s)
{
    uint32_t trtyp = HSMCI_CMDR_TRTYP(s->cmdr);
    uint32_t blklen = HSMCI_BLKR_BLKLEN(s->blkr);
    uint32_t bcnt = HSMCI_BLKR_BCNT(s->blkr);
    uint64_t length = 0;

    hsmci_abort_transfer(s);
    if (blklen == 0) {
        blklen = (s->mr >> 16) & 0xffff;
    }

    s->infinite = false;
    switch (trtyp) {
    case HSMCI_TRTYP_SINGLE:
        length = blklen;
        break;
    case HSMCI_TRTYP_MULTI:
    case HSMCI_TRTYP_STREAM:
        if (bcnt == 0) {
            s->infinite = true;
        } else {
            length = (uint64_t)blklen * bcnt;
        }
        break;
    case HSMCI_TRTYP_SDIO_BYTE:
        length = bcnt ? bcnt : 512;
        blklen = (uint32_t)length;
        break;
    case HSMCI_TRTYP_SDIO_BLOCK:
        if (bcnt == 0) {
            s->infinite = true;
        } else {
            length = (uint64_t)blklen * bcnt;
        }
        break;
    default:
        break;
    }

    if (!s->infinite && length == 0) {
        s->sr |= HSMCI_SR_NOTBUSY | HSMCI_SR_XFRDONE;
        return;
    }

    s->blklen = MAX(blklen, 1u);
    s->block_remaining = s->blklen;
    s->data_remaining = length;
    s->reading = (s->cmdr & HSMCI_CMDR_TRDIR_READ) != 0;
    hsmci_sync_data_len(s);
    s->sr &= ~(HSMCI_SR_RXRDY | HSMCI_SR_TXRDY | HSMCI_SR_BLKE |
               HSMCI_SR_NOTBUSY | HSMCI_SR_XFRDONE |
               HSMCI_SR_OVRE | HSMCI_SR_UNRE);
    s->sr |= HSMCI_SR_DTIP;
    s->sr |= s->reading ? HSMCI_SR_RXRDY : HSMCI_SR_TXRDY;
    hsmci_arm_data_timeout(s);
    hsmci_update_dma_request(s);
    hsmci_update_irq(s);
}

static void hsmci_store_response(AT91HsmciState *s, const uint8_t *resp,
                                 size_t rlen)
{
    s->rsp_ptr = 0;
    if (HSMCI_CMDR_RSPTYP(s->cmdr) == HSMCI_RSPTYP_136 && rlen == 16) {
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
}

static void hsmci_command_complete(AT91HsmciState *s, bool timed_out)
{
    s->command_pending = false;
    s->command_cycles = 0;
    s->command_phase = HSMCI_CMD_IDLE;
    s->sr |= HSMCI_SR_CMDRDY;
    if (timed_out) {
        s->sr |= HSMCI_SR_RTOE;
        trace_at91_hsmci_rtoe(HSMCI_CMDR_CMDNB(s->cmdr));
    }

    if (HSMCI_CMDR_TRCMD(s->cmdr) == HSMCI_TRCMD_STOP) {
        hsmci_abort_transfer(s);
    } else if (HSMCI_CMDR_TRCMD(s->cmdr) == HSMCI_TRCMD_START &&
               !timed_out) {
        hsmci_start_transfer(s);
    } else if (!hsmci_has_data(s) && !s->transfer_pending) {
        s->sr |= HSMCI_SR_XFRDONE;
    }
    hsmci_update_dma_request(s);
    hsmci_update_irq(s);
}

static void hsmci_command_timer_cb(void *opaque)
{
    AT91HsmciState *s = opaque;
    SDRequest req = {
        .cmd = HSMCI_CMDR_CMDNB(s->cmdr),
        .arg = s->argr,
    };
    uint8_t resp[16] = { 0 };
    size_t rlen;

    s->command_cycles = 0;
    switch (s->command_phase) {
    case HSMCI_CMD_TRANSMIT:
        if (HSMCI_CMDR_SPCMD(s->cmdr) == HSMCI_SPCMD_INIT) {
            hsmci_command_complete(s, false);
            return;
        }
        rlen = sdbus_do_command(&s->sdbus, &req, resp, sizeof(resp));
        trace_at91_hsmci_command(req.cmd, req.arg, (unsigned)rlen);
        hsmci_store_response(s, resp, rlen);
        if (HSMCI_CMDR_RSPTYP(s->cmdr) == HSMCI_RSPTYP_NONE) {
            hsmci_command_complete(s, false);
        } else if (rlen == 0) {
            s->command_phase = HSMCI_CMD_TIMEOUT;
            hsmci_schedule_command(s,
                HSMCI_CMDR_MAXLAT(s->cmdr) ? 64 : 5);
        } else {
            s->command_phase = HSMCI_CMD_RESPONSE;
            hsmci_schedule_command(s,
                HSMCI_CMDR_RSPTYP(s->cmdr) == HSMCI_RSPTYP_136 ?
                HSMCI_RESPONSE_136_CYCLES : HSMCI_RESPONSE_48_CYCLES);
        }
        return;
    case HSMCI_CMD_RESPONSE:
        hsmci_command_complete(s, false);
        return;
    case HSMCI_CMD_TIMEOUT:
        hsmci_command_complete(s, true);
        return;
    default:
        return;
    }
}

static void hsmci_do_command(AT91HsmciState *s, uint32_t cmdr)
{
    if (!s->enabled) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "at91-hsmci: command while interface disabled\n");
        return;
    }
    if (!(s->sr & HSMCI_SR_CMDRDY)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "at91-hsmci: command while CMDRDY is clear\n");
        return;
    }

    s->cmdr = cmdr;
    s->command_pending = true;
    s->command_phase = HSMCI_CMD_TRANSMIT;
    s->sr &= ~(HSMCI_SR_CMDRDY | HSMCI_SR_XFRDONE |
               HSMCI_SR_COMMAND_ERRORS);
    if (HSMCI_CMDR_TRCMD(cmdr) == HSMCI_TRCMD_START) {
        s->sr &= ~(HSMCI_SR_OVRE | HSMCI_SR_UNRE);
    }
    hsmci_update_irq(s);
    hsmci_schedule_command(s,
        HSMCI_CMDR_SPCMD(cmdr) == HSMCI_SPCMD_INIT ?
        HSMCI_INIT_CYCLES : HSMCI_COMMAND_CYCLES);
}

static uint32_t hsmci_read_data(AT91HsmciState *s)
{
    uint32_t count, value = 0;
    int i;

    if (!hsmci_has_data(s) || !s->reading ||
        !(s->sr & HSMCI_SR_RXRDY)) {
        return 0;
    }
    count = s->infinite ? 4 : MIN(s->data_remaining, UINT64_C(4));
    for (i = 0; i < count; i++) {
        value |= (uint32_t)sdbus_read_byte(&s->sdbus) << (8 * i);
    }
    if (!s->infinite) {
        s->data_remaining -= count;
    }
    hsmci_advance_block(s, count);
    hsmci_sync_data_len(s);
    if (!s->infinite && s->data_remaining == 0) {
        hsmci_schedule_transfer_end(s);
    } else {
        hsmci_arm_data_timeout(s);
        hsmci_rearm_dma_request(s);
    }
    hsmci_update_irq(s);
    return value;
}

static void hsmci_write_data(AT91HsmciState *s, uint32_t value)
{
    uint32_t count;
    int i;

    if (!hsmci_has_data(s) || s->reading ||
        !(s->sr & HSMCI_SR_TXRDY)) {
        return;
    }
    count = s->infinite ? 4 : MIN(s->data_remaining, UINT64_C(4));
    for (i = 0; i < count; i++) {
        sdbus_write_byte(&s->sdbus, (value >> (8 * i)) & 0xff);
    }
    if (!s->infinite) {
        s->data_remaining -= count;
    }
    hsmci_advance_block(s, count);
    hsmci_sync_data_len(s);
    if (!s->infinite && s->data_remaining == 0) {
        hsmci_schedule_transfer_end(s);
    } else {
        hsmci_arm_data_timeout(s);
        hsmci_rearm_dma_request(s);
    }
    hsmci_update_irq(s);
}

static uint64_t hsmci_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91HsmciState *s = AT91_HSMCI(opaque);
    uint32_t value;

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
    case HSMCI_SR:
        value = s->sr;
        s->sr &= ~HSMCI_SR_READ_CLEAR;
        if ((s->cfg & (1u << 4)) != 0) {
            s->sr &= ~(HSMCI_SR_OVRE | HSMCI_SR_UNRE);
        }
        if (s->sdio_irq_level) {
            s->sr |= HSMCI_SR_SDIOIRQA;
        }
        hsmci_update_irq(s);
        return value;
    case HSMCI_IMR:  return s->imr;
    case HSMCI_DMA:  return s->dma;
    case HSMCI_CFG:  return s->cfg;
    case HSMCI_WPMR: return s->wpmr;
    case HSMCI_WPSR: return s->wpsr;
    case HSMCI_VERSION: return HSMCI_IP_VERSION;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-hsmci: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static bool hsmci_write_protected(hwaddr offset)
{
    switch (offset) {
    case HSMCI_MR:
    case HSMCI_DTOR:
    case HSMCI_SDCR:
    case HSMCI_CSTOR:
    case HSMCI_DMA:
    case HSMCI_CFG:
        return true;
    default:
        return false;
    }
}

static void hsmci_write_wpmr(AT91HsmciState *s, uint32_t value)
{
    if (HSMCI_WPMR_KEY(value) != HSMCI_WPMR_VALID_KEY) {
        return;
    }
    s->wpmr = value & HSMCI_WPMR_WPEN;
    s->wpsr = 0;
}

static void hsmci_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    AT91HsmciState *s = AT91_HSMCI(opaque);
    uint32_t val = value;
    uint32_t old_wpvs;
    bool was_protected;

    if (offset >= HSMCI_FIFO) {
        hsmci_write_data(s, val);
        return;
    }

    if ((s->wpmr & HSMCI_WPMR_WPEN) && hsmci_write_protected(offset)) {
        old_wpvs = s->wpsr & HSMCI_WPSR_WPVS_MASK;
        s->wpsr = ((offset & 0xffff) << 8) |
                   (old_wpvs | HSMCI_WPSR_WPVS_WRITE);
        return;
    }

    switch (offset) {
    case HSMCI_CR:
        if (val & HSMCI_CR_SWRST) {
            /* SWRST resets only the host, never the SD card on its bus. */
            old_wpvs = s->wpsr & HSMCI_WPSR_WPVS_MASK;
            was_protected = (s->wpmr & HSMCI_WPMR_WPEN) != 0;
            hsmci_reset(DEVICE(s));
            if (was_protected) {
                s->wpsr = (HSMCI_CR << 8) |
                           (old_wpvs | HSMCI_WPSR_WPVS_RESET);
            }
        } else if (val & HSMCI_CR_MCIDIS) {
            hsmci_pause_timers(s);
            s->enabled = false;
            hsmci_cancel_dma_rearm(s);
            hsmci_update_dma_request(s);
        } else if (val & HSMCI_CR_MCIEN) {
            s->enabled = true;
            hsmci_resume_timers(s);
            hsmci_update_dma_request(s);
        }
        break;
    case HSMCI_MR:
        hsmci_pause_timers(s);
        s->mr = val;
        hsmci_resume_timers(s);
        break;
    case HSMCI_DTOR: s->dtor = val; break;
    case HSMCI_SDCR: s->sdcr = val; break;
    case HSMCI_ARGR: s->argr = val; break;
    case HSMCI_CMDR: hsmci_do_command(s, val); break;
    case HSMCI_BLKR: s->blkr = val; break;
    case HSMCI_CSTOR: s->cstor = val; break;
    case HSMCI_TDR:  hsmci_write_data(s, val); break;
    case HSMCI_IER:  s->imr |= val; hsmci_update_irq(s); break;
    case HSMCI_IDR:  s->imr &= ~val; hsmci_update_irq(s); break;
    case HSMCI_DMA:
        s->dma = val;
        hsmci_update_dma_request(s);
        break;
    case HSMCI_CFG:  s->cfg = val; break;
    case HSMCI_WPMR:
        hsmci_write_wpmr(s, val);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-hsmci: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static void hsmci_sdio_irq(void *opaque, int n, int level)
{
    AT91HsmciState *s = opaque;

    s->sdio_irq_level = level;
    if (level) {
        s->sr |= HSMCI_SR_SDIOIRQA;
    }
    hsmci_update_irq(s);
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

    if (s->command_timer) {
        timer_del(s->command_timer);
    }
    if (s->transfer_timer) {
        timer_del(s->transfer_timer);
    }
    if (s->timeout_timer) {
        timer_del(s->timeout_timer);
    }
    hsmci_cancel_dma_rearm(s);
    s->mr = s->dtor = s->sdcr = s->argr = s->blkr = 0;
    s->cstor = s->dma = s->cfg = s->wpmr = 0;
    s->wpsr = 0;
    s->sr = HSMCI_SR_RESET |
            (s->sdio_irq_level ? HSMCI_SR_SDIOIRQA : 0);
    s->imr = 0;
    s->rsp[0] = s->rsp[1] = s->rsp[2] = s->rsp[3] = 0;
    s->rsp_ptr = 0;
    s->cmdr = 0;
    s->command_phase = HSMCI_CMD_IDLE;
    s->enabled = false;
    s->command_pending = false;
    s->transfer_pending = false;
    s->infinite = false;
    s->data_len = 0;
    s->data_remaining = 0;
    s->blklen = 0;
    s->block_remaining = 0;
    s->reading = false;
    s->dma_request_rearm = false;
    s->command_cycles = 0;
    s->transfer_cycles = 0;
    s->command_mck_hz = 0;
    s->command_divider = 0;
    s->transfer_mck_hz = 0;
    s->transfer_divider = 0;
    s->command_remaining_ns = -1;
    s->transfer_remaining_ns = -1;
    s->timeout_pending = false;
    s->timeout_cycles = 0;
    s->timeout_mck_hz = 0;
    s->timeout_remaining_ns = -1;
    hsmci_update_irq(s);
    hsmci_update_dma_request(s);
}

static void hsmci_realize(DeviceState *dev, Error **errp)
{
    AT91HsmciState *s = AT91_HSMCI(dev);

    if (!clock_has_source(s->mck)) {
        error_setg(errp, "at91-hsmci: mck input must be connected");
        return;
    }
    s->command_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    hsmci_command_timer_cb, s);
    s->transfer_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                     hsmci_transfer_timer_cb, s);
    s->timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    hsmci_data_timeout_cb, s);
    s->dma_request_bh = qemu_bh_new(hsmci_dma_request_bh, s);
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
    qdev_init_gpio_in_named(DEVICE(obj), hsmci_sdio_irq,
                            AT91_HSMCI_SDIO_IRQ, 1);
    s->mck = qdev_init_clock_in(DEVICE(obj), "mck", hsmci_clock_update, s,
                                ClockUpdate);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_AT91_HSMCI_BUS, DEVICE(obj),
              "sd-bus");
}

static void hsmci_finalize(Object *obj)
{
    AT91HsmciState *s = AT91_HSMCI(obj);

    timer_free(s->command_timer);
    timer_free(s->transfer_timer);
    timer_free(s->timeout_timer);
    /*
     * An instance that was never realized - QMP device-list-properties
     * creates one - has no bottom half yet, and qemu_bh_delete() is not
     * NULL-tolerant the way timer_free() is.
     */
    if (s->dma_request_bh) {
        qemu_bh_delete(s->dma_request_bh);
    }
}

static int hsmci_pre_save(void *opaque)
{
    AT91HsmciState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->command_remaining_ns = timer_pending(s->command_timer) ?
        MAX(timer_expire_time_ns(s->command_timer) - now, INT64_C(0)) : -1;
    s->transfer_remaining_ns = timer_pending(s->transfer_timer) ?
        MAX(timer_expire_time_ns(s->transfer_timer) - now, INT64_C(0)) : -1;
    s->timeout_remaining_ns = timer_pending(s->timeout_timer) ?
        MAX(timer_expire_time_ns(s->timeout_timer) - now, INT64_C(0)) : -1;
    return 0;
}

static int hsmci_post_load(void *opaque, int version_id)
{
    AT91HsmciState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    timer_del(s->command_timer);
    timer_del(s->transfer_timer);
    timer_del(s->timeout_timer);
    qemu_bh_cancel(s->dma_request_bh);
    if (version_id < 2) {
        s->data_remaining = MAX(s->data_len, 0);
        s->enabled = true;
        s->command_pending = false;
        s->transfer_pending = false;
        s->infinite = false;
        s->command_remaining_ns = -1;
        s->transfer_remaining_ns = -1;
    }
    if (version_id < 4) {
        s->timeout_pending = false;
        s->timeout_cycles = 0;
        s->timeout_remaining_ns = -1;
    }
    if (s->command_pending) {
        if (s->enabled && s->command_remaining_ns >= 0 &&
            clock_get_hz(s->mck)) {
            s->command_mck_hz = clock_get_hz(s->mck);
            s->command_divider = hsmci_card_divider(s);
            timer_mod(s->command_timer, now + s->command_remaining_ns);
        } else {
            hsmci_schedule_command(s, s->command_cycles);
        }
    }
    if (s->transfer_pending) {
        if (s->enabled && s->transfer_remaining_ns >= 0 &&
            clock_get_hz(s->mck)) {
            s->transfer_mck_hz = clock_get_hz(s->mck);
            s->transfer_divider = hsmci_card_divider(s);
            timer_mod(s->transfer_timer, now + s->transfer_remaining_ns);
        } else {
            hsmci_schedule_transfer(s, s->transfer_cycles);
        }
    }
    if (s->timeout_pending) {
        if (s->enabled && s->timeout_remaining_ns >= 0 &&
            clock_get_hz(s->mck)) {
            s->timeout_mck_hz = clock_get_hz(s->mck);
            timer_mod(s->timeout_timer, now + s->timeout_remaining_ns);
        } else {
            hsmci_schedule_data_timeout(s, s->timeout_cycles);
        }
    }
    if (s->sdio_irq_level) {
        s->sr |= HSMCI_SR_SDIOIRQA;
    }
    s->command_remaining_ns = -1;
    s->transfer_remaining_ns = -1;
    s->timeout_remaining_ns = -1;

    hsmci_update_irq(s);
    hsmci_update_dma_request(s);
    if (s->dma_request_rearm) {
        qemu_bh_schedule(s->dma_request_bh);
    }
    return 0;
}

static const VMStateDescription vmstate_at91_hsmci = {
    .name = "at91-hsmci",
    .version_id = 4,
    .minimum_version_id = 1,
    .pre_save = hsmci_pre_save,
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
        VMSTATE_UINT32_V(wpsr, AT91HsmciState, 2),
        VMSTATE_UINT32_V(cmdr, AT91HsmciState, 2),
        VMSTATE_UINT8_V(command_phase, AT91HsmciState, 2),
        VMSTATE_BOOL_V(enabled, AT91HsmciState, 2),
        VMSTATE_BOOL_V(command_pending, AT91HsmciState, 2),
        VMSTATE_BOOL_V(transfer_pending, AT91HsmciState, 2),
        VMSTATE_BOOL_V(infinite, AT91HsmciState, 2),
        VMSTATE_BOOL_V(sdio_irq_level, AT91HsmciState, 2),
        VMSTATE_UINT32_V(block_remaining, AT91HsmciState, 2),
        VMSTATE_UINT64_V(data_remaining, AT91HsmciState, 2),
        VMSTATE_UINT64_V(command_cycles, AT91HsmciState, 2),
        VMSTATE_UINT64_V(transfer_cycles, AT91HsmciState, 2),
        VMSTATE_INT64_V(command_remaining_ns, AT91HsmciState, 2),
        VMSTATE_INT64_V(transfer_remaining_ns, AT91HsmciState, 2),
        VMSTATE_BOOL_V(dma_request_rearm, AT91HsmciState, 3),
        VMSTATE_BOOL_V(timeout_pending, AT91HsmciState, 4),
        VMSTATE_UINT64_V(timeout_cycles, AT91HsmciState, 4),
        VMSTATE_INT64_V(timeout_remaining_ns, AT91HsmciState, 4),
        VMSTATE_END_OF_LIST()
    }
};

static void hsmci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hsmci_realize;
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
        .instance_finalize = hsmci_finalize,
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
