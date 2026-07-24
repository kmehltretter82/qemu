/*
 * Atmel/Microchip AT91 Serial Peripheral Interface (SPI) controller.
 *
 * Master-mode model of the SAM9 SPI, as driven by the Linux spi-atmel driver
 * ("atmel,at91rm9200-spi").  Data flows through QEMU's SSI core to whatever
 * peripheral is attached to the bus (e.g. an m25p80 SPI-NOR flash).
 *
 * The driver selects its transfer method from the version register (0xFC):
 * >= 0x212 => external-DMA-capable, but with no "dmas" in the DT it falls back
 * to the PIO path ("Atmel SPI Controller using PIO only").  We report 0x212 so
 * the guest uses PIO (write TDR, poll SR.RDRF, read RDR) - no PDC/DMA needed.
 *
 * Chip select: in fixed-peripheral mode the driver drives NPCS via MR.PCS
 * (cs_activate => PCS = ~(1 << cs); cs_deactivate => PCS = 0xf).  We track those
 * transitions and drive per-NPCS gpio outputs, so a peripheral that resets its
 * command state on CS deassert (m25p80) is framed correctly.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/at91_spi.h"
#include "trace.h"

/* Register offsets */
#define SPI_CR      0x00   /* Control            */
#define SPI_MR      0x04   /* Mode               */
#define SPI_RDR     0x08   /* Receive Data       */
#define SPI_TDR     0x0c   /* Transmit Data      */
#define SPI_SR      0x10   /* Status             */
#define SPI_IER     0x14   /* Interrupt Enable   */
#define SPI_IDR     0x18   /* Interrupt Disable  */
#define SPI_IMR     0x1c   /* Interrupt Mask     */
#define SPI_CSR0    0x30   /* Chip Select 0..3   */
#define SPI_CSR3    0x3c
#define SPI_VERSION 0xfc

/* CR bits */
#define CR_SPIEN    (1u << 0)
#define CR_SPIDIS   (1u << 1)
#define CR_SWRST    (1u << 7)
#define CR_LASTXFER (1u << 24)

/* MR fields */
#define MR_MSTR     (1u << 0)
#define MR_PS       (1u << 1)    /* variable peripheral select     */
#define MR_PCSDEC   (1u << 2)    /* 1-of-16 chip-select decode     */
#define MR_PCS(m)   (((m) >> 16) & 0xf)

/* SR/IER/IDR/IMR bits */
#define SR_RDRF     (1u << 0)    /* Receive Data Register Full     */
#define SR_TDRE     (1u << 1)    /* Transmit Data Register Empty   */
#define SR_MODF     (1u << 2)
#define SR_OVRES    (1u << 3)    /* Overrun error                  */
#define SR_TXEMPTY  (1u << 9)    /* transmitter empty              */
#define SR_SPIENS   (1u << 16)   /* SPI enable status              */

#define SPI_NUM_CS  4
#define SPI_VERSION_VAL 0x00000212   /* forces the driver's PIO path */

struct AT91SpiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq cs_lines[SPI_NUM_CS];
    qemu_irq tx_request;
    qemu_irq rx_request;
    QEMUBH *tx_request_bh;
    SSIBus *bus;

    uint32_t mr;
    uint32_t sr;
    uint32_t imr;
    uint32_t rdr;
    uint32_t csr[SPI_NUM_CS];
    bool enabled;
    bool tx_request_rearm;
    int active_cs;    /* currently-asserted NPCS, or -1 for none */
};

static void spi_update_irq(AT91SpiState *s)
{
    qemu_set_irq(s->irq, !!(s->sr & s->imr));
}

/*
 * Hardware DMA request lines (Table 40-1 ids 1..4).  TX request follows
 * TDRE and RX request follows RDRF.  A TDR write completes the transfer
 * synchronously here, so TDRE never visibly drops; the request is lowered
 * and re-raised through a bottom half so the DMA controller observes one
 * distinct handshake edge per data register access, exactly like the HSMCI.
 * The TX request additionally waits for RDR to be drained: real hardware
 * overlaps the next TDR load with the shifter and the RX consumer drains
 * RDR inside that transfer window, but this model transfers instantly, so
 * an early TX grant would overrun RDR.  Holding TX until RDRF clears
 * serializes the full-duplex pipeline without changing CPU-driven flows.
 */
static void spi_update_dma_requests(AT91SpiState *s)
{
    qemu_set_irq(s->tx_request,
                 s->enabled && !s->tx_request_rearm &&
                 (s->sr & SR_TDRE) && !(s->sr & SR_RDRF));
    qemu_set_irq(s->rx_request, s->enabled && (s->sr & SR_RDRF));
}

static void spi_tx_request_bh(void *opaque)
{
    AT91SpiState *s = opaque;

    s->tx_request_rearm = false;
    spi_update_dma_requests(s);
}

static void spi_rearm_tx_request(AT91SpiState *s)
{
    if (!s->enabled) {
        return;
    }
    s->tx_request_rearm = true;
    spi_update_dma_requests(s);
    qemu_bh_schedule(s->tx_request_bh);
}

/* Decode the chip select currently selected by a PCS field (fixed or variable
 * peripheral mode); returns the NPCS index, or -1 if none/decoded-out. */
static int spi_decode_cs(AT91SpiState *s, uint32_t pcs)
{
    int i;

    if (s->mr & MR_PCSDEC) {
        /* External 1-of-16 decoder: not used by the board, treat as none. */
        return -1;
    }
    for (i = 0; i < SPI_NUM_CS; i++) {
        if (!(pcs & (1u << i))) {
            return i;
        }
    }
    return -1;   /* PCS == 0xf: all deselected */
}

/* Drive the NPCS gpio lines so only @cs (if >= 0) is asserted (low). */
static void spi_set_cs(AT91SpiState *s, int cs)
{
    int i;

    if (cs == s->active_cs) {
        return;
    }
    for (i = 0; i < SPI_NUM_CS; i++) {
        qemu_set_irq(s->cs_lines[i], i == cs ? 0 : 1);
    }
    s->active_cs = cs;
}

static void spi_do_transfer(AT91SpiState *s, uint32_t tdr)
{
    uint32_t pcs = (s->mr & MR_PS) ? ((tdr >> 16) & 0xf) : MR_PCS(s->mr);
    int cs = spi_decode_cs(s, pcs);
    uint32_t rx;

    spi_set_cs(s, cs);

    rx = ssi_transfer(s->bus, tdr & 0xffff);
    trace_at91_spi_xfer(cs, tdr & 0xffff, rx & 0xffff);

    if (s->sr & SR_RDRF) {
        s->sr |= SR_OVRES;   /* previous RDR not read before this transfer */
    }
    s->rdr = rx & 0xffff;
    s->sr |= SR_RDRF | SR_TDRE | SR_TXEMPTY;
    spi_update_dma_requests(s);
    spi_update_irq(s);
}

static void spi_reset(DeviceState *dev)
{
    AT91SpiState *s = AT91_SPI(dev);
    int i;

    s->mr = 0;
    s->imr = 0;
    s->rdr = 0;
    s->enabled = false;
    s->sr = SR_TDRE | SR_TXEMPTY;
    s->tx_request_rearm = false;
    if (s->tx_request_bh) {
        qemu_bh_cancel(s->tx_request_bh);
    }
    spi_update_dma_requests(s);
    for (i = 0; i < SPI_NUM_CS; i++) {
        s->csr[i] = 0;
    }
    /* Deassert every NPCS (drive high) so an attached peripheral starts in a
     * known deselected state. */
    s->active_cs = -2;   /* force spi_set_cs to drive all lines */
    spi_set_cs(s, -1);
}

static uint64_t spi_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91SpiState *s = opaque;
    uint32_t r;

    switch (offset) {
    case SPI_MR:
        return s->mr;
    case SPI_RDR:
        r = s->rdr | (MR_PCS(s->mr) << 16);
        s->sr &= ~SR_RDRF;
        spi_update_dma_requests(s);
        spi_update_irq(s);
        return r;
    case SPI_SR:
        r = s->sr | (s->enabled ? SR_SPIENS : 0);
        s->sr &= ~SR_OVRES;   /* OVRES is cleared by reading SR */
        spi_update_irq(s);
        return r;
    case SPI_IMR:
        return s->imr;
    case SPI_CSR0 ... SPI_CSR3:
        return s->csr[(offset - SPI_CSR0) / 4];
    case SPI_VERSION:
        return SPI_VERSION_VAL;
    default:
        return 0;
    }
}

static void spi_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91SpiState *s = opaque;
    uint32_t val = value;

    switch (offset) {
    case SPI_CR:
        if (val & CR_SWRST) {
            spi_reset(DEVICE(s));
            return;
        }
        if (val & CR_SPIEN) {
            s->enabled = true;
            spi_update_dma_requests(s);
        }
        if (val & CR_SPIDIS) {
            s->enabled = false;
            spi_set_cs(s, -1);   /* disabling releases NPCS */
            s->tx_request_rearm = false;
            qemu_bh_cancel(s->tx_request_bh);
            spi_update_dma_requests(s);
        }
        if (val & CR_LASTXFER) {
            /* End of chained transfer: release the current chip select. */
            spi_set_cs(s, -1);
        }
        break;
    case SPI_MR:
        s->mr = val;
        /* Fixed-peripheral mode selects NPCS through MR.PCS directly. */
        if (!(val & MR_PS)) {
            spi_set_cs(s, spi_decode_cs(s, MR_PCS(val)));
        }
        break;
    case SPI_TDR:
        if (s->enabled) {
            spi_do_transfer(s, val);
            spi_rearm_tx_request(s);
        }
        break;
    case SPI_IER:
        s->imr |= val;
        spi_update_irq(s);
        break;
    case SPI_IDR:
        s->imr &= ~val;
        spi_update_irq(s);
        break;
    case SPI_CSR0 ... SPI_CSR3:
        s->csr[(offset - SPI_CSR0) / 4] = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps spi_ops = {
    .read = spi_read,
    .write = spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* The spi-atmel DMA path byte-accesses TDR/RDR (8-bit transfer
     * widths), like the TWI driver does with its data registers. */
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static int spi_post_load(void *opaque, int version_id)
{
    AT91SpiState *s = opaque;

    spi_update_dma_requests(s);
    if (s->tx_request_rearm) {
        qemu_bh_schedule(s->tx_request_bh);
    }
    return 0;
}

static const VMStateDescription vmstate_at91_spi = {
    .name = "at91-spi",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = spi_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91SpiState),
        VMSTATE_UINT32(sr, AT91SpiState),
        VMSTATE_UINT32(imr, AT91SpiState),
        VMSTATE_UINT32(rdr, AT91SpiState),
        VMSTATE_UINT32_ARRAY(csr, AT91SpiState, SPI_NUM_CS),
        VMSTATE_BOOL(enabled, AT91SpiState),
        VMSTATE_INT32(active_cs, AT91SpiState),
        VMSTATE_BOOL_V(tx_request_rearm, AT91SpiState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void spi_init(Object *obj)
{
    AT91SpiState *s = AT91_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &spi_ops, s, "at91-spi", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->cs_lines, "cs", SPI_NUM_CS);
    qdev_init_gpio_out_named(DEVICE(obj), &s->tx_request,
                             AT91_SPI_TX_DMA_REQUEST, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->rx_request,
                             AT91_SPI_RX_DMA_REQUEST, 1);
    s->tx_request_bh = qemu_bh_new(spi_tx_request_bh, s);
    s->bus = ssi_create_bus(DEVICE(obj), "spi");
}

static void spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, spi_reset);
    dc->vmsd = &vmstate_at91_spi;
}

static const TypeInfo spi_type = {
    .name = TYPE_AT91_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91SpiState),
    .instance_init = spi_init,
    .class_init = spi_class_init,
};

static void at91_spi_register_types(void)
{
    type_register_static(&spi_type);
}

type_init(at91_spi_register_types)
