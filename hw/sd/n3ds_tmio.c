/*
 * Nintendo 3DS SD/MMC host controller (Toshiba TMIO-style, the same
 * register set as the DSi's), with two card ports selected through
 * SD_PORTSEL: port 0 = SD card slot, port 1 = eMMC ("NAND").
 *
 * Documentation: GBATEK "DSi SD/MMC Protocol" / 3DS "EMMC Registers",
 * 3dbrew, and the register usage of the sdmmc.c driver shared by
 * GodMode9 / Luma3DS / firm_linux_loader.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/sd/sd.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/n3ds.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(N3DSTmioState, N3DS_TMIO)

#define REG_CMD         0x000
#define REG_PORTSEL     0x002
#define REG_ARG0        0x004
#define REG_ARG1        0x006
#define REG_STOP        0x008
#define REG_BLKCOUNT    0x00A
#define REG_RESP0       0x00C   /* .. 0x01A */
#define REG_STATUS0     0x01C
#define REG_STATUS1     0x01E
#define REG_IRMASK0     0x020
#define REG_IRMASK1     0x022
#define REG_CLKCTL      0x024
#define REG_BLKLEN      0x026
#define REG_OPT         0x028
#define REG_FIFO16      0x030
#define REG_DATACTL     0x0D8
#define REG_RESET       0x0E0
#define REG_PROTECTED   0x0F6
#define REG_DATACTL32   0x100
#define REG_BLKLEN32    0x104
#define REG_BLKCOUNT32  0x108
#define REG_FIFO32      0x10C
#define REG_CLKWAIT     0x138
#define REG_RESET_SDIO  0x1E0

#define STAT0_CMDRESPEND    0x0001
#define STAT0_DATAEND       0x0004
#define STAT0_CARD_REMOVE   0x0008
#define STAT0_CARD_INSERT   0x0010
#define STAT0_SIGSTATE      0x0020
#define STAT0_WRPROTECT     0x0080
#define STAT0_LIVE_MASK     (STAT0_SIGSTATE | STAT0_WRPROTECT)

#define STAT1_CMD_IDX_ERR   0x0001
#define STAT1_CRCFAIL       0x0002
#define STAT1_STOPBIT_ERR   0x0004
#define STAT1_DATATIMEOUT   0x0008
#define STAT1_RXOVERFLOW    0x0010
#define STAT1_TXUNDERRUN    0x0020
#define STAT1_CMDTIMEOUT    0x0040
#define STAT1_RXRDY         0x0100
#define STAT1_TXRQ          0x0200
#define STAT1_ILL_FUNC      0x2000
#define STAT1_CMD_BUSY      0x4000
#define STAT1_ILL_ACCESS    0x8000

#define DATACTL32_32BIT     0x0002
#define DATACTL32_RXRDY     0x0100
#define DATACTL32_TXBUSY    0x0200
#define DATACTL32_LIVE_MASK (DATACTL32_RXRDY | DATACTL32_TXBUSY)

#define CMD_INDEX(c)    ((c) & 0x3F)
#define CMD_RESP(c)     (((c) >> 8) & 7)
#define CMD_DATA        0x0800
#define CMD_READ        0x1000
#define CMD_MULTI       0x2000

#define STOP_MANUAL     0x0001
#define STOP_AUTO       0x0100

#define N3DS_TMIO_NPORTS 2
#define FIFO_MAX 2048

enum {
    XFER_NONE,
    XFER_READ,
    XFER_WRITE,
};

struct N3DSTmioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus[N3DS_TMIO_NPORTS];
    qemu_irq irq;
    qemu_irq irq_async;

    uint16_t regs[0x100];       /* 16-bit registers, index = offset / 2 */

    int xfer;
    bool xfer_multi;
    uint32_t blocks_left;
    uint32_t blk_len;
    uint32_t fifo_pos;
    uint8_t fifo[FIFO_MAX];
};

static inline uint16_t R16(N3DSTmioState *s, unsigned off)
{
    return s->regs[off >> 1];
}

static inline void W16(N3DSTmioState *s, unsigned off, uint16_t v)
{
    s->regs[off >> 1] = v;
}

static SDBus *tmio_bus(N3DSTmioState *s)
{
    unsigned port = R16(s, REG_PORTSEL) & 3;

    if (port >= N3DS_TMIO_NPORTS) {
        return NULL;
    }
    return &s->sdbus[port];
}

static void tmio_update_irq(N3DSTmioState *s)
{
    uint16_t st0 = R16(s, REG_STATUS0) & ~R16(s, REG_IRMASK0);
    uint16_t st1 = R16(s, REG_STATUS1) & ~R16(s, REG_IRMASK1);

    qemu_set_irq(s->irq, (st0 | st1) != 0);
}

static void tmio_set_status(N3DSTmioState *s, uint16_t st0, uint16_t st1)
{
    W16(s, REG_STATUS0, R16(s, REG_STATUS0) | st0);
    W16(s, REG_STATUS1, R16(s, REG_STATUS1) | st1);
}

static void tmio_clear_status(N3DSTmioState *s, uint16_t st0, uint16_t st1)
{
    W16(s, REG_STATUS0, R16(s, REG_STATUS0) & ~st0);
    W16(s, REG_STATUS1, R16(s, REG_STATUS1) & ~st1);
}

static void tmio_xfer_end(N3DSTmioState *s)
{
    SDBus *bus = tmio_bus(s);

    if (s->xfer_multi && (R16(s, REG_STOP) & STOP_AUTO) && bus) {
        SDRequest req = { .cmd = 12, .arg = 0 };
        uint8_t resp[16];

        sdbus_do_command(bus, &req, resp, sizeof(resp));
    }
    s->xfer = XFER_NONE;
    s->fifo_pos = 0;
    s->blocks_left = 0;
    tmio_clear_status(s, 0, STAT1_RXRDY | STAT1_TXRQ);
    tmio_set_status(s, STAT0_DATAEND, 0);
}

static void tmio_load_block(N3DSTmioState *s)
{
    SDBus *bus = tmio_bus(s);

    if (!bus) {
        tmio_xfer_end(s);
        return;
    }
    sdbus_read_data(bus, s->fifo, s->blk_len);
    s->fifo_pos = 0;
    tmio_set_status(s, 0, STAT1_RXRDY);
}

static void tmio_store_block(N3DSTmioState *s)
{
    SDBus *bus = tmio_bus(s);

    if (bus) {
        sdbus_write_data(bus, s->fifo, s->blk_len);
    }
    s->fifo_pos = 0;
    if (--s->blocks_left == 0) {
        tmio_xfer_end(s);
    }
}

static void tmio_set_response(N3DSTmioState *s, const uint8_t *resp,
                              size_t rlen)
{
    uint8_t b[16] = { 0 };
    unsigned i;

    if (rlen == 16) {
        /* R2: bits 8..127 of the response, little-endian, CRC dropped */
        for (i = 0; i < 15; i++) {
            b[i] = resp[14 - i];
        }
    } else if (rlen >= 4) {
        b[0] = resp[3];
        b[1] = resp[2];
        b[2] = resp[1];
        b[3] = resp[0];
    }
    for (i = 0; i < 8; i++) {
        W16(s, REG_RESP0 + 2 * i, b[2 * i] | (b[2 * i + 1] << 8));
    }
}

static void tmio_send_command(N3DSTmioState *s, uint16_t cmd)
{
    SDBus *bus = tmio_bus(s);
    SDRequest req;
    uint8_t resp[16];
    size_t rlen;
    unsigned resp_type = CMD_RESP(cmd);

    /* Abort any transfer still pending from a previous command. */
    s->xfer = XFER_NONE;
    s->fifo_pos = 0;
    s->blocks_left = 0;
    tmio_clear_status(s, 0, STAT1_RXRDY | STAT1_TXRQ);

    W16(s, REG_CMD, cmd);
    tmio_set_status(s, 0, STAT1_CMD_BUSY);

    if (!bus || !sdbus_get_inserted(bus)) {
        tmio_clear_status(s, 0, STAT1_CMD_BUSY);
        tmio_set_status(s, 0, STAT1_CMDTIMEOUT);
        tmio_update_irq(s);
        return;
    }

    req.cmd = CMD_INDEX(cmd);
    req.arg = R16(s, REG_ARG0) | ((uint32_t)R16(s, REG_ARG1) << 16);
    req.crc = 0;
    rlen = sdbus_do_command(bus, &req, resp, sizeof(resp));

    if (resp_type != 0 && rlen == 0) {
        tmio_clear_status(s, 0, STAT1_CMD_BUSY);
        tmio_set_status(s, 0, STAT1_CMDTIMEOUT);
        tmio_update_irq(s);
        return;
    }
    tmio_set_response(s, resp, rlen);
    tmio_set_status(s, STAT0_CMDRESPEND, 0);

    if (cmd & CMD_DATA) {
        s->xfer_multi = (cmd & CMD_MULTI) != 0;
        s->blocks_left = s->xfer_multi ? R16(s, REG_BLKCOUNT) : 1;
        if (s->blocks_left == 0) {
            s->blocks_left = 1;
        }
        s->blk_len = R16(s, REG_BLKLEN);
        if (s->blk_len == 0 || s->blk_len > FIFO_MAX) {
            s->blk_len = 512;
        }
        if (cmd & CMD_READ) {
            s->xfer = XFER_READ;
            tmio_load_block(s);
        } else {
            s->xfer = XFER_WRITE;
            s->fifo_pos = 0;
            tmio_set_status(s, 0, STAT1_TXRQ);
        }
    }
    tmio_clear_status(s, 0, STAT1_CMD_BUSY);
    tmio_update_irq(s);
}

static uint32_t tmio_fifo_read(N3DSTmioState *s, unsigned width)
{
    uint32_t v = 0;
    unsigned i;

    if (s->xfer != XFER_READ) {
        return 0;
    }
    for (i = 0; i < width && s->fifo_pos < s->blk_len; i++) {
        v |= (uint32_t)s->fifo[s->fifo_pos++] << (8 * i);
    }
    if (s->fifo_pos >= s->blk_len) {
        tmio_clear_status(s, 0, STAT1_RXRDY);
        if (--s->blocks_left == 0) {
            tmio_xfer_end(s);
        } else {
            tmio_load_block(s);
        }
        tmio_update_irq(s);
    }
    return v;
}

static void tmio_fifo_write(N3DSTmioState *s, uint32_t v, unsigned width)
{
    unsigned i;

    if (s->xfer != XFER_WRITE) {
        return;
    }
    for (i = 0; i < width && s->fifo_pos < s->blk_len; i++) {
        s->fifo[s->fifo_pos++] = v >> (8 * i);
    }
    if (s->fifo_pos >= s->blk_len) {
        tmio_store_block(s);
        tmio_update_irq(s);
    }
}

static uint16_t tmio_status0(N3DSTmioState *s)
{
    SDBus *bus = tmio_bus(s);
    uint16_t v = R16(s, REG_STATUS0) & ~STAT0_LIVE_MASK;

    if (bus && sdbus_get_inserted(bus)) {
        v |= STAT0_SIGSTATE;
        if (!sdbus_get_readonly(bus)) {
            v |= STAT0_WRPROTECT;
        }
    }
    return v;
}

static uint16_t tmio_datactl32(N3DSTmioState *s)
{
    uint16_t v = R16(s, REG_DATACTL32) & ~DATACTL32_LIVE_MASK;

    if (s->xfer == XFER_READ) {
        v |= DATACTL32_RXRDY;
    }
    if (s->xfer != XFER_WRITE) {
        v |= DATACTL32_TXBUSY;
    }
    return v;
}

static uint64_t tmio_read16(N3DSTmioState *s, hwaddr off)
{
    switch (off) {
    case REG_STATUS0:
        return tmio_status0(s);
    case REG_DATACTL32:
        return tmio_datactl32(s);
    case REG_FIFO16:
        return tmio_fifo_read(s, 2);
    default:
        return R16(s, off);
    }
}

static uint64_t tmio_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSTmioState *s = opaque;

    if (off >= 0x200) {
        qemu_log_mask(LOG_GUEST_ERROR, "n3ds-tmio: read @0x%" HWADDR_PRIx "\n",
                      off);
        return 0;
    }
    if (size == 4) {
        if (off == REG_FIFO32) {
            return tmio_fifo_read(s, 4);
        }
        return tmio_read16(s, off) | (tmio_read16(s, off + 2) << 16);
    }
    if (size == 2) {
        return tmio_read16(s, off);
    }
    /* byte access */
    return (tmio_read16(s, off & ~1) >> (8 * (off & 1))) & 0xFF;
}

static void tmio_write16(N3DSTmioState *s, hwaddr off, uint16_t v)
{
    switch (off) {
    case REG_CMD:
        tmio_send_command(s, v);
        return;
    case REG_STATUS0:
        /* write-0-to-clear */
        W16(s, REG_STATUS0, R16(s, REG_STATUS0) & v);
        tmio_update_irq(s);
        return;
    case REG_STATUS1:
        W16(s, REG_STATUS1, R16(s, REG_STATUS1) & v);
        tmio_update_irq(s);
        return;
    case REG_IRMASK0:
    case REG_IRMASK1:
        W16(s, off, v);
        tmio_update_irq(s);
        return;
    case REG_STOP:
        W16(s, REG_STOP, v);
        if ((v & STOP_MANUAL) && s->xfer != XFER_NONE) {
            SDBus *bus = tmio_bus(s);

            if (bus) {
                SDRequest req = { .cmd = 12, .arg = 0 };
                uint8_t resp[16];
                sdbus_do_command(bus, &req, resp, sizeof(resp));
            }
            s->xfer_multi = false;
            tmio_xfer_end(s);
            tmio_update_irq(s);
        }
        return;
    case REG_RESET:
        W16(s, REG_RESET, v);
        if (!(v & 1)) {
            s->xfer = XFER_NONE;
            s->fifo_pos = 0;
            s->blocks_left = 0;
            W16(s, REG_STATUS0, 0);
            W16(s, REG_STATUS1, 0);
            tmio_update_irq(s);
        }
        return;
    case REG_FIFO16:
        tmio_fifo_write(s, v, 2);
        return;
    case REG_DATACTL32:
        W16(s, REG_DATACTL32, v & ~DATACTL32_LIVE_MASK);
        return;
    case REG_RESP0 ... REG_RESP0 + 14:
        return;
    default:
        W16(s, off, v);
        return;
    }
}

static void tmio_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSTmioState *s = opaque;

    if (off >= 0x200) {
        qemu_log_mask(LOG_GUEST_ERROR, "n3ds-tmio: write @0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, v);
        return;
    }
    if (size == 4) {
        if (off == REG_FIFO32) {
            tmio_fifo_write(s, v, 4);
            return;
        }
        tmio_write16(s, off, v & 0xFFFF);
        tmio_write16(s, off + 2, v >> 16);
        return;
    }
    if (size == 2) {
        tmio_write16(s, off, v);
        return;
    }
    {
        uint16_t old = R16(s, off & ~1);
        unsigned shift = 8 * (off & 1);
        tmio_write16(s, off & ~1, (old & ~(0xFF << shift)) | ((v & 0xFF) << shift));
    }
}

static const MemoryRegionOps tmio_ops = {
    .read = tmio_read,
    .write = tmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void n3ds_tmio_reset(DeviceState *dev)
{
    N3DSTmioState *s = N3DS_TMIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->xfer = XFER_NONE;
    s->fifo_pos = 0;
    s->blocks_left = 0;
    s->blk_len = 512;
    W16(s, REG_BLKLEN, 512);
    W16(s, REG_BLKLEN32, 512);
    W16(s, REG_IRMASK0, 0x031D);
    W16(s, REG_IRMASK1, 0x837F);
    W16(s, REG_PROTECTED, 0);
    qemu_set_irq(s->irq, 0);
}

static void n3ds_tmio_init(Object *obj)
{
    N3DSTmioState *s = N3DS_TMIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &tmio_ops, s, "n3ds.tmio", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->irq_async);
    qbus_init(&s->sdbus[0], sizeof(s->sdbus[0]), TYPE_SD_BUS, DEVICE(obj),
              "sd-bus");
    qbus_init(&s->sdbus[1], sizeof(s->sdbus[1]), TYPE_SD_BUS, DEVICE(obj),
              "nand-bus");
}

static void n3ds_tmio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n3ds_tmio_reset);
}

static const TypeInfo n3ds_tmio_info = {
    .name          = TYPE_N3DS_TMIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSTmioState),
    .instance_init = n3ds_tmio_init,
    .class_init    = n3ds_tmio_class_init,
};

static void n3ds_tmio_register_types(void)
{
    type_register_static(&n3ds_tmio_info);
}

type_init(n3ds_tmio_register_types)
