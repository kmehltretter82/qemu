/*
 * SDIO function model for the Nintendo 3DS WiFi module.
 *
 * QEMU's SD code models memory cards only - there is no CMD5/CMD52/CMD53
 * anywhere - so a guest can never enumerate the Atheros part sitting on the
 * controller at 0x10122000. This provides just enough of an SDIO card for
 * the SDIO stack to find it: the operation-condition handshake, direct
 * register access, the CCCR, and a CIS carrying the vendor and device ids a
 * driver matches on.
 *
 * It deliberately stops at enumeration: no AR6K firmware download (BMI), no
 * HTC/WMI messaging, no data path. The point is to let driver and kernel
 * work proceed without the console.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "qemu/log.h"
#include "hw/core/qdev-properties.h"
#include "hw/sd/n3ds_sdio.h"
#include "trace.h"

/* CMD52 / CMD53 argument fields */
#define IO_RW_WRITE       (1u << 31)
#define IO_RW_FUNC(a)     (((a) >> 28) & 7)
#define IO_RW_RAW         (1u << 27)
#define IO_RW_ADDR(a)     (((a) >> 9) & 0x1FFFF)
#define IO_RW_DATA(a)     ((a) & 0xFF)
#define IO_RW_EXT_BLOCK   (1u << 27)
#define IO_RW_EXT_INCR    (1u << 26)
#define IO_RW_EXT_COUNT(a) ((a) & 0x1FF)

/* R5 response: flags in bits 15-8, data in bits 7-0 */
#define R5_STATE_CMD      (1u << 12)
#define R5_ERR_FUNC       (1u << 9)
#define R5_ERR_RANGE      (1u << 8)

/* CCCR (function 0) */
#define CCCR_REV          0x00
#define CCCR_SD_REV       0x01
#define CCCR_IO_ENABLE    0x02
#define CCCR_IO_READY     0x03
#define CCCR_INT_ENABLE   0x04
#define CCCR_INT_PENDING  0x05
#define CCCR_IO_ABORT     0x06
#define CCCR_BUS_IFACE    0x07
#define CCCR_CARD_CAPS    0x08
#define CCCR_CIS_PTR      0x09   /* .. 0x0B */
#define CCCR_FN0_BLKSZ    0x10   /* .. 0x11 */
#define CCCR_POWER        0x12
#define CCCR_SPEED        0x13

#define FN0_CIS_ADDR      0x001000
#define FN1_CIS_ADDR      0x001100
#define FBR_BASE(fn)      (0x100 * (fn))

static uint8_t sdio_cis_fn0(N3DSSdioState *s, uint32_t off)
{
    /*
     * CISTPL_MANFID carries the ids a driver matches on, then a minimal
     * CISTPL_FUNCE for function 0 and the end marker.
     */
    switch (off) {
    case 0x00: return 0x20;                     /* CISTPL_MANFID */
    case 0x01: return 0x04;                     /* link (4 bytes) */
    case 0x02: return s->vendor_id & 0xFF;
    case 0x03: return s->vendor_id >> 8;
    case 0x04: return s->device_id & 0xFF;
    case 0x05: return s->device_id >> 8;
    case 0x06: return 0x22;                     /* CISTPL_FUNCE */
    case 0x07: return 0x04;                     /* link */
    case 0x08: return 0x00;                     /* type 0: common */
    case 0x09: return 0x00;                     /* fn0 max block size lo */
    case 0x0A: return 0x02;                     /* .. 512 */
    case 0x0B: return 0x32;                     /* max transfer rate code */
    case 0x0C: return 0xFF;                     /* CISTPL_END */
    default:   return 0x00;
    }
}

static uint8_t sdio_cis_fn1(N3DSSdioState *s, uint32_t off)
{
    /*
     * Function CISTPL_FUNCE, "type 1" layout. With SDIO revision 1.00 in
     * the CCCR the parser wants 28 bytes; only the max block size at
     * offset 0x0C-0x0D is read back.
     */
    if (off == 0x00) return 0x21;               /* CISTPL_FUNCID */
    if (off == 0x01) return 0x02;
    if (off == 0x02) return 0x0C;               /* function code */
    if (off == 0x03) return 0x00;
    if (off == 0x04) return 0x22;               /* CISTPL_FUNCE */
    if (off == 0x05) return 0x1C;               /* link: 28 bytes */
    if (off == 0x06) return 0x01;               /* type 1: function */
    if (off == 0x06 + 0x0C) return 0x00;        /* max block size lo */
    if (off == 0x06 + 0x0D) return 0x02;        /* .. 512 */
    if (off == 0x06 + 0x1C) return 0xFF;        /* CISTPL_END */
    return 0x00;
}

static uint8_t sdio_read_reg(N3DSSdioState *s, unsigned fn, uint32_t addr)
{
    if (fn == 0) {
        if (addr >= FN0_CIS_ADDR && addr < FN0_CIS_ADDR + 0x40) {
            return sdio_cis_fn0(s, addr - FN0_CIS_ADDR);
        }
        if (addr >= FN1_CIS_ADDR && addr < FN1_CIS_ADDR + 0x40) {
            return sdio_cis_fn1(s, addr - FN1_CIS_ADDR);
        }
        switch (addr) {
        case CCCR_REV:      return 0x02;   /* SDIO 1.00, CCCR 2.00 */
        case CCCR_SD_REV:   return 0x01;
        case CCCR_IO_ENABLE: return s->io_enable;
        case CCCR_IO_READY: return s->io_enable;   /* ready as soon as enabled */
        case CCCR_INT_ENABLE: return s->int_enable;
        case CCCR_INT_PENDING: return 0x00;
        case CCCR_BUS_IFACE: return s->bus_iface;
        case CCCR_CARD_CAPS: return 0x17;  /* multi-block, 4-bit support */
        case CCCR_CIS_PTR + 0: return FN0_CIS_ADDR & 0xFF;
        case CCCR_CIS_PTR + 1: return (FN0_CIS_ADDR >> 8) & 0xFF;
        case CCCR_CIS_PTR + 2: return (FN0_CIS_ADDR >> 16) & 0xFF;
        case CCCR_FN0_BLKSZ + 0: return s->fn0_blksz & 0xFF;
        case CCCR_FN0_BLKSZ + 1: return s->fn0_blksz >> 8;
        case CCCR_SPEED:    return 0x00;
        default: break;
        }
        /* function 1 basic registers */
        if (addr >= FBR_BASE(1) && addr < FBR_BASE(2)) {
            switch (addr - FBR_BASE(1)) {
            case 0x00: return 0x0C;               /* interface code */
            case 0x09: return FN1_CIS_ADDR & 0xFF;
            case 0x0A: return (FN1_CIS_ADDR >> 8) & 0xFF;
            case 0x0B: return (FN1_CIS_ADDR >> 16) & 0xFF;
            case 0x10: return s->fn1_blksz & 0xFF;
            case 0x11: return s->fn1_blksz >> 8;
            default: return 0x00;
            }
        }
        return 0x00;
    }
    /* No function-1 register space is modelled yet. */
    return 0x00;
}

static void sdio_write_reg(N3DSSdioState *s, unsigned fn, uint32_t addr,
                           uint8_t val)
{
    if (fn != 0) {
        return;
    }
    switch (addr) {
    case CCCR_IO_ENABLE:  s->io_enable = val; break;
    case CCCR_INT_ENABLE: s->int_enable = val; break;
    case CCCR_BUS_IFACE:  s->bus_iface = val; break;
    case CCCR_IO_ABORT:   break;
    case CCCR_FN0_BLKSZ + 0: s->fn0_blksz = (s->fn0_blksz & 0xFF00) | val; break;
    case CCCR_FN0_BLKSZ + 1: s->fn0_blksz = (s->fn0_blksz & 0xFF) | (val << 8); break;
    default:
        if (addr == FBR_BASE(1) + 0x10) {
            s->fn1_blksz = (s->fn1_blksz & 0xFF00) | val;
        } else if (addr == FBR_BASE(1) + 0x11) {
            s->fn1_blksz = (s->fn1_blksz & 0xFF) | (val << 8);
        }
        break;
    }
}

/*
 * Run one command. Returns the response length in bytes (4 for the short
 * responses this card uses, 0 when the card does not answer).
 */
int n3ds_sdio_command(N3DSSdioState *s, uint32_t cmd, uint32_t arg,
                      uint8_t *resp, size_t respsz)
{
    uint32_t r = 0;

    if (respsz < 4) {
        return 0;
    }
    switch (cmd) {
    case 0:                                   /* GO_IDLE_STATE */
        s->state = SDIO_ST_IDLE;
        return 0;                             /* no response */
    case 5:                                   /* IO_SEND_OP_COND */
        /*
         * First call carries no OCR and reports the card busy; once the
         * host echoes the voltage window back the card comes ready.
         */
        r = (1u << 28) |                      /* one I/O function */
            0x00300000;                       /* 3.2-3.4 V */
        if (arg & 0x00FFFF00) {
            r |= 1u << 31;                    /* ready */
            s->state = SDIO_ST_READY;
        }
        break;
    case 3:                                   /* SEND_RELATIVE_ADDR */
        s->rca = 0x0001;
        s->state = SDIO_ST_STANDBY;
        r = ((uint32_t)s->rca << 16) | 0x0500;
        break;
    case 7:                                   /* SELECT/DESELECT_CARD */
        s->state = (arg >> 16) == s->rca ? SDIO_ST_SELECTED : SDIO_ST_STANDBY;
        r = 0x0700;
        break;
    case 52: {                                /* IO_RW_DIRECT */
        unsigned fn = IO_RW_FUNC(arg);
        uint32_t addr = IO_RW_ADDR(arg);
        uint8_t data = IO_RW_DATA(arg);

        if (fn > 1) {
            r = R5_ERR_FUNC;
            break;
        }
        if (arg & IO_RW_WRITE) {
            sdio_write_reg(s, fn, addr, data);
            /* read-after-write returns the new value, otherwise the old */
            r = R5_STATE_CMD |
                ((arg & IO_RW_RAW) ? sdio_read_reg(s, fn, addr) : data);
        } else {
            r = R5_STATE_CMD | sdio_read_reg(s, fn, addr);
        }
        trace_n3ds_sdio_cmd52(!!(arg & IO_RW_WRITE), fn, addr, r & 0xFF);
        break;
    }
    case 53:                                  /* IO_RW_EXTENDED */
        /*
         * Bulk access is what the firmware download and the data path use;
         * neither is modelled, so report out-of-range rather than pretend.
         */
        qemu_log_mask(LOG_UNIMP,
                      "n3ds-sdio: CMD53 (extended I/O) not implemented\n");
        r = R5_STATE_CMD | R5_ERR_RANGE;
        break;
    default:
        return 0;                             /* unknown command: no answer */
    }

    resp[0] = r >> 24;
    resp[1] = r >> 16;
    resp[2] = r >> 8;
    resp[3] = r;
    trace_n3ds_sdio_cmd(cmd, arg, r);
    return 4;
}

static void n3ds_sdio_reset(DeviceState *dev)
{
    N3DSSdioState *s = N3DS_SDIO(dev);

    s->state = SDIO_ST_IDLE;
    s->rca = 0;
    s->io_enable = 0;
    s->int_enable = 0;
    s->bus_iface = 0;
    s->fn0_blksz = 0;
    s->fn1_blksz = 0;
}

static const Property n3ds_sdio_properties[] = {
    /*
     * Defaults describe an Atheros AR6004-class part, which is what the
     * New3DS carries (an AR6014) and what ath6kl binds.
     */
    DEFINE_PROP_UINT16("vendor-id", N3DSSdioState, vendor_id, 0x0271),
    DEFINE_PROP_UINT16("device-id", N3DSSdioState, device_id, 0x0400),
};

static void n3ds_sdio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Nintendo 3DS WiFi SDIO function";
    device_class_set_legacy_reset(dc, n3ds_sdio_reset);
    device_class_set_props(dc, n3ds_sdio_properties);
}

static const TypeInfo n3ds_sdio_info = {
    .name = TYPE_N3DS_SDIO,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(N3DSSdioState),
    .class_init = n3ds_sdio_class_init,
};

static void n3ds_sdio_register_types(void)
{
    type_register_static(&n3ds_sdio_info);
}

type_init(n3ds_sdio_register_types)
