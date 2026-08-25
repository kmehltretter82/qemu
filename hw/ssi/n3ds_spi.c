/*
 * Nintendo 3DS SPI bus controller ("new" register interface at +0x800,
 * legacy interface at +0x000 kept as plain storage). Three instances:
 * 0x10142000 (bus 1: touchscreen/codec), 0x10143000 (bus 2),
 * 0x10160000 (bus 3: NVRAM).
 *
 *   0x800 CNT     bit 15 start/busy, bit 13 direction (1 = write),
 *                 bits 7:6 chip select, bits 2:0 baud rate
 *   0x804 DONE    write 0 to finish a transfer
 *   0x808 BLKLEN  transfer length in bytes
 *   0x80C FIFO    32-bit data window
 *   0x810 STATUS  bit 0: FIFO busy
 *
 * Transfers complete synchronously over a QEMU SSI bus; without a
 * peripheral attached, reads return zeros. The interrupt is raised when
 * the busy bit clears (the linux-3ds driver sleeps on it).
 *
 * Documentation: 3dbrew SPI_Registers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/n3ds.h"
#include "qom/object.h"

#define TYPE_N3DS_SPI "n3ds-spi"
OBJECT_DECLARE_SIMPLE_TYPE(N3DSSpiState, N3DS_SPI)

#define REG_CNT     0x800
#define REG_DONE    0x804
#define REG_BLKLEN  0x808
#define REG_FIFO    0x80C
#define REG_STATUS  0x810

#define CNT_START   (1u << 15)
#define CNT_WRITE   (1u << 13)

#define FIFO_MAX    0x40000

struct N3DSSpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *ssi;
    qemu_irq irq;

    uint32_t legacy[0x200];
    uint32_t cnt;
    uint32_t blklen;
    uint32_t pos;          /* bytes transferred so far */
    uint32_t fifo_word;
    unsigned fifo_bytes;
};

static uint64_t n3ds_spi_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSSpiState *s = opaque;
    uint32_t v = 0;
    unsigned i;

    switch (off) {
    case REG_CNT:
        return s->cnt;
    case REG_DONE:
        return 0;
    case REG_BLKLEN:
        return s->blklen;
    case REG_FIFO:
        if (!(s->cnt & CNT_START) || (s->cnt & CNT_WRITE)) {
            return 0;
        }
        for (i = 0; i < 4 && s->pos < s->blklen; i++, s->pos++) {
            v |= (uint32_t)ssi_transfer(s->ssi, 0) << (8 * i);
        }
        if (s->pos >= s->blklen) {
            s->cnt &= ~CNT_START;
            qemu_irq_pulse(s->irq);
        }
        return v;
    case REG_STATUS:
        return 0;   /* FIFO never busy: transfers are synchronous */
    default:
        if (off < 0x800) {
            return s->legacy[off >> 2];
        }
        return 0;
    }
}

static void n3ds_spi_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    N3DSSpiState *s = opaque;
    unsigned i;

    switch (off) {
    case REG_CNT:
        s->cnt = val & 0xFFFF;
        if (val & CNT_START) {
            s->pos = 0;
            if (s->blklen == 0) {
                s->cnt &= ~CNT_START;
                qemu_irq_pulse(s->irq);
            }
        }
        break;
    case REG_DONE:
        s->cnt &= ~CNT_START;
        s->pos = 0;
        break;
    case REG_BLKLEN:
        s->blklen = MIN(val, FIFO_MAX);
        break;
    case REG_FIFO:
        if (!(s->cnt & CNT_START) || !(s->cnt & CNT_WRITE)) {
            break;
        }
        for (i = 0; i < 4 && s->pos < s->blklen; i++, s->pos++) {
            ssi_transfer(s->ssi, (val >> (8 * i)) & 0xFF);
        }
        if (s->pos >= s->blklen) {
            s->cnt &= ~CNT_START;
            qemu_irq_pulse(s->irq);
        }
        break;
    case REG_STATUS:
        break;
    default:
        if (off < 0x800) {
            s->legacy[off >> 2] = val;
        }
        break;
    }
}

static const MemoryRegionOps n3ds_spi_ops = {
    .read = n3ds_spi_read,
    .write = n3ds_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void n3ds_spi_reset(DeviceState *dev)
{
    N3DSSpiState *s = N3DS_SPI(dev);

    memset(s->legacy, 0, sizeof(s->legacy));
    s->cnt = 0;
    s->blklen = 0;
    s->pos = 0;
}

static void n3ds_spi_init(Object *obj)
{
    N3DSSpiState *s = N3DS_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &n3ds_spi_ops, s, "n3ds.spi", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->ssi = ssi_create_bus(DEVICE(obj), "ssi");
}

static void n3ds_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n3ds_spi_reset);
}

static const TypeInfo n3ds_spi_info = {
    .name          = TYPE_N3DS_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSSpiState),
    .instance_init = n3ds_spi_init,
    .class_init    = n3ds_spi_class_init,
};

static void n3ds_spi_register_types(void)
{
    type_register_static(&n3ds_spi_info);
}

type_init(n3ds_spi_register_types)
