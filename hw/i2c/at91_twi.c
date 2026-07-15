/*
 * Atmel/Microchip AT91 Two-Wire Interface (TWI / I2C) controller.
 *
 * Master-mode model of the SAM9 TWI, as driven by the Linux i2c-at91 driver
 * ("atmel,at91sam9g10-i2c"): writing THR auto-issues START + slave address and
 * streams bytes; a read is kicked off by the START bit in CR with MREAD set in
 * MMR; the internal-address register (IADR/IADRSZ) provides the EEPROM-style
 * "write sub-address then repeated-START read" sequence.  Data flows through
 * QEMU's I2C core to whatever slave is attached (e.g. an at24c EEPROM).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/at91_twi.h"
#include "trace.h"

/* Register offsets */
#define TWI_CR    0x00   /* Control              */
#define TWI_MMR   0x04   /* Master Mode          */
#define TWI_SMR   0x08   /* Slave Mode           */
#define TWI_IADR  0x0c   /* Internal Address     */
#define TWI_CWGR  0x10   /* Clock Waveform Gen   */
#define TWI_SR    0x20   /* Status               */
#define TWI_IER   0x24   /* Interrupt Enable     */
#define TWI_IDR   0x28   /* Interrupt Disable    */
#define TWI_IMR   0x2c   /* Interrupt Mask       */
#define TWI_RHR   0x30   /* Receive Holding      */
#define TWI_THR   0x34   /* Transmit Holding     */

/* CR bits */
#define CR_START  (1u << 0)
#define CR_STOP   (1u << 1)
#define CR_MSEN   (1u << 2)
#define CR_MSDIS  (1u << 3)
#define CR_SVEN   (1u << 4)
#define CR_SVDIS  (1u << 5)
#define CR_SWRST  (1u << 7)

/* MMR fields */
#define MMR_IADRSZ(m)  (((m) >> 8) & 0x3)
#define MMR_MREAD      (1u << 12)
#define MMR_DADR(m)    (((m) >> 16) & 0x7f)

/* SR bits */
#define SR_TXCOMP  (1u << 0)   /* transmission complete / idle */
#define SR_RXRDY   (1u << 1)   /* RHR holds a received byte    */
#define SR_TXRDY   (1u << 2)   /* THR is empty                 */
#define SR_NACK    (1u << 8)   /* slave did not acknowledge    */

struct AT91TwiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;

    uint32_t mmr;
    uint32_t smr;
    uint32_t iadr;
    uint32_t cwgr;
    uint32_t sr;
    uint32_t imr;
    uint8_t  rhr;

    bool xfer;      /* a master transfer is in progress */
    bool reading;   /* current transfer is a read       */
    bool stopping;  /* STOP requested: end after next RHR read */
};

static void twi_update_irq(AT91TwiState *s)
{
    qemu_set_irq(s->irq, !!(s->sr & s->imr));
}

/* Send the internal (sub-)address bytes, MSB first. */
static void twi_send_iadr(AT91TwiState *s)
{
    int sz = MMR_IADRSZ(s->mmr);
    int i;

    for (i = sz - 1; i >= 0; i--) {
        i2c_send(s->bus, (s->iadr >> (8 * i)) & 0xff);
    }
}

/*
 * Open a transfer to the MMR-selected device.  With a non-zero internal
 * address size the sub-address is written first (in send mode), then for a
 * read the bus is turned around with a repeated START.  Sets NACK if the
 * slave does not respond.
 */
static void twi_start(AT91TwiState *s, bool reading)
{
    uint8_t dadr = MMR_DADR(s->mmr);
    int nack;

    if (MMR_IADRSZ(s->mmr) > 0) {
        nack = i2c_start_send(s->bus, dadr);
        if (!nack) {
            twi_send_iadr(s);
        }
        if (reading && !nack) {
            nack = i2c_start_recv(s->bus, dadr);
        }
    } else {
        nack = reading ? i2c_start_recv(s->bus, dadr)
                       : i2c_start_send(s->bus, dadr);
    }

    if (nack) {
        /* The addressed slave did not respond: abort cleanly so no dangling
         * transfer corrupts the next one (e.g. after an i2cdetect scan). */
        s->sr |= SR_NACK;
        i2c_end_transfer(s->bus);
        s->xfer = s->reading = s->stopping = false;
        s->sr |= SR_TXCOMP;
    } else {
        s->xfer = true;
        s->reading = reading;
        s->sr &= ~SR_TXCOMP;
    }
}

static void twi_reset(DeviceState *dev)
{
    AT91TwiState *s = AT91_TWI(dev);

    if (s->xfer) {
        i2c_end_transfer(s->bus);
    }
    s->mmr = s->smr = s->iadr = s->cwgr = s->imr = 0;
    s->rhr = 0;
    s->xfer = s->reading = s->stopping = false;
    s->sr = SR_TXCOMP | SR_TXRDY;   /* idle, THR empty */
}

static uint64_t twi_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91TwiState *s = opaque;
    uint32_t r;

    switch (offset) {
    case TWI_MMR:  return s->mmr;
    case TWI_SMR:  return s->smr;
    case TWI_IADR: return s->iadr;
    case TWI_CWGR: return s->cwgr;
    case TWI_IMR:  return s->imr;
    case TWI_SR:
        r = s->sr;
        s->sr &= ~SR_NACK;          /* NACK is cleared by reading SR */
        return r;
    case TWI_RHR:
        r = s->rhr;
        s->sr &= ~SR_RXRDY;
        if (s->reading && s->xfer) {
            if (s->stopping) {
                i2c_end_transfer(s->bus);
                s->xfer = s->reading = s->stopping = false;
                s->sr |= SR_TXCOMP;
            } else {
                s->rhr = i2c_recv(s->bus);
                s->sr |= SR_RXRDY;
            }
        }
        twi_update_irq(s);
        return r;
    default:
        return 0;
    }
}

static void twi_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91TwiState *s = opaque;
    uint32_t val = value;

    switch (offset) {
    case TWI_CR:
        if (val & CR_SWRST) {
            twi_reset(DEVICE(s));
            break;
        }
        if (val & CR_START) {
            trace_at91_twi_start(MMR_DADR(s->mmr), !!(s->mmr & MMR_MREAD));
            twi_start(s, !!(s->mmr & MMR_MREAD));
            if (s->reading) {
                /* Prefetch the first byte so RXRDY reflects data available. */
                if (!(s->sr & SR_NACK)) {
                    s->rhr = i2c_recv(s->bus);
                    s->sr |= SR_RXRDY;
                }
                if (val & CR_STOP) {
                    s->stopping = true;
                }
            }
        }
        if (val & CR_STOP) {
            if (s->xfer && !s->reading) {
                i2c_end_transfer(s->bus);
                s->xfer = false;
                s->sr |= SR_TXCOMP;
            } else if (s->reading) {
                s->stopping = true;
            }
        }
        twi_update_irq(s);
        break;
    case TWI_MMR:  s->mmr = val; break;
    case TWI_SMR:  s->smr = val; break;
    case TWI_IADR: s->iadr = val; break;
    case TWI_CWGR: s->cwgr = val; break;
    case TWI_THR:
        if (!s->xfer) {
            trace_at91_twi_start(MMR_DADR(s->mmr), false);
            twi_start(s, false);
        }
        if (!(s->sr & SR_NACK)) {
            if (i2c_send(s->bus, val & 0xff)) {
                s->sr |= SR_NACK;
            }
        }
        s->sr |= SR_TXRDY;          /* byte consumed, THR empty again */
        twi_update_irq(s);
        break;
    case TWI_IER:  s->imr |= val;  twi_update_irq(s); break;
    case TWI_IDR:  s->imr &= ~val; twi_update_irq(s); break;
    default:
        break;
    }
}

static const MemoryRegionOps twi_ops = {
    .read = twi_read,
    .write = twi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* The i2c-at91 driver accesses RHR/THR with byte ops (readb/writeb) and
     * the other registers with word ops, so allow 1..4-byte access. */
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_at91_twi = {
    .name = "at91-twi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mmr, AT91TwiState),
        VMSTATE_UINT32(smr, AT91TwiState),
        VMSTATE_UINT32(iadr, AT91TwiState),
        VMSTATE_UINT32(cwgr, AT91TwiState),
        VMSTATE_UINT32(sr, AT91TwiState),
        VMSTATE_UINT32(imr, AT91TwiState),
        VMSTATE_UINT8(rhr, AT91TwiState),
        VMSTATE_BOOL(xfer, AT91TwiState),
        VMSTATE_BOOL(reading, AT91TwiState),
        VMSTATE_BOOL(stopping, AT91TwiState),
        VMSTATE_END_OF_LIST()
    }
};

static void twi_init(Object *obj)
{
    AT91TwiState *s = AT91_TWI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &twi_ops, s, "at91-twi", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), NULL);
}

static void twi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, twi_reset);
    dc->vmsd = &vmstate_at91_twi;
}

static const TypeInfo twi_type = {
    .name = TYPE_AT91_TWI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91TwiState),
    .instance_init = twi_init,
    .class_init = twi_class_init,
};

static void at91_twi_register_types(void)
{
    type_register_static(&twi_type);
}

type_init(at91_twi_register_types)
