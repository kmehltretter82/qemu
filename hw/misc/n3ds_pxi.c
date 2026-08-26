/*
 * Nintendo 3DS PXI: the ARM9 <-> ARM11 mailbox and FIFO pair.
 *
 * Each side has the same register block (ARM9 at 0x10008000, ARM11 at
 * 0x10163000), the DS/DSi IPC layout:
 *   0x00 SYNC  bits 0-7 byte from the other side, bits 8-15 byte to the
 *              other side (W), bit 29 (ARM9 side) / bits 29-30 (ARM11
 *              side) raise the other side's sync interrupt, bit 31
 *              sync interrupt enable
 *   0x04 CNT   bit 0 send FIFO empty, 1 send FIFO full, 2 send-empty
 *              IRQ enable, 3 flush send FIFO, 8 receive FIFO empty,
 *              9 receive FIFO full, 10 receive-not-empty IRQ enable,
 *              14 error (W1C), 15 FIFO enable
 *   0x08 SEND  (W)
 *   0x0C RECV  (R)
 *
 * IRQ outputs: 0-2 = ARM9 sync / send-empty / receive-not-empty,
 *              3-5 = ARM11 sync / send-empty / receive-not-empty.
 *
 * Documentation: 3dbrew PXI_Registers, GBATEK DS IPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/cpu.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/fifo32.h"
#include "hw/misc/n3ds.h"
#include "trace.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(N3DSPxiState, N3DS_PXI)

#define PXI_FIFO_DEPTH 16

#define REG_SYNC 0x0
#define REG_CNT  0x4
#define REG_SEND 0x8
#define REG_RECV 0xC

#define CNT_SEND_EMPTY   0x0001
#define CNT_SEND_FULL    0x0002
#define CNT_SEND_IRQEN   0x0004
#define CNT_SEND_FLUSH   0x0008
#define CNT_RECV_EMPTY   0x0100
#define CNT_RECV_FULL    0x0200
#define CNT_RECV_IRQEN   0x0400
#define CNT_ERROR        0x4000
#define CNT_ENABLE       0x8000
#define CNT_RW_MASK      (CNT_SEND_IRQEN | CNT_RECV_IRQEN | CNT_ENABLE)

#define SYNC_IRQ_A       (1u << 29)
#define SYNC_IRQ_B       (1u << 30)
#define SYNC_IRQEN       (1u << 31)

typedef struct N3DSPxiSide {
    MemoryRegion iomem;
    Fifo32 send;        /* this side -> other side */
    uint8_t sync_out;   /* byte this side sent */
    uint16_t cnt;
    bool sync_irqen;
    qemu_irq irq_sync;
    qemu_irq irq_send;
    qemu_irq irq_recv;
    bool send_lvl;
    bool recv_lvl;
    uint32_t last_recv;
} N3DSPxiSide;

struct N3DSPxiState {
    SysBusDevice parent_obj;
    N3DSPxiSide side[2];   /* 0 = ARM9, 1 = ARM11 */    bool luma_state;
};

static void pxi_update_irqs(N3DSPxiState *s)
{
    int i;

    for (i = 0; i < 2; i++) {
        N3DSPxiSide *me = &s->side[i];
        N3DSPxiSide *other = &s->side[1 - i];

        bool send_lvl = (me->cnt & CNT_SEND_IRQEN) && fifo32_is_empty(&me->send);
        bool recv_lvl = (me->cnt & CNT_RECV_IRQEN) && !fifo32_is_empty(&other->send);

        trace_n3ds_pxi_irq(i, send_lvl, recv_lvl, fifo32_num_used(&me->send),
                           fifo32_num_used(&other->send));
        /*
         * The PXI interrupts are edge events on the console (3dbrew: 0x50-0x53
         * "edge-triggered, 1-N"; GodMode9 relies on it): one pulse when the
         * send FIFO becomes empty / the receive FIFO becomes non-empty.
         */
        if (send_lvl && !me->send_lvl) {
            qemu_irq_pulse(me->irq_send);
        }
        if (recv_lvl && !me->recv_lvl) {
            qemu_irq_pulse(me->irq_recv);
        }
        me->send_lvl = send_lvl;
        me->recv_lvl = recv_lvl;
    }
}

static uint64_t pxi_read(N3DSPxiState *s, int idx, hwaddr off, unsigned size)
{
    N3DSPxiSide *me = &s->side[idx];
    N3DSPxiSide *other = &s->side[1 - idx];
    uint32_t v;

    switch (off) {
    case REG_SYNC:
        /*
         * Only the byte received from the other side is readable; the send
         * field is write-only and reads as zero. Measured on a New 2DS XL
         * three ways - byte, halfword and 32-bit stores of a marker into
         * bits 15-8 all leave SYNC unchanged (n3ds-probes pxi11_sync /
         * pxi11_sync32).
         *
         * boot9 does a read-modify-write of this register at 0xffff172c and
         * therefore writes its own send byte back as zero. That is harmless:
         * by then boot11 has already consumed the value it was waiting for.
         */
        v = other->sync_out | (me->sync_irqen ? SYNC_IRQEN : 0);
        break;
    case REG_CNT:
        v = me->cnt & (CNT_RW_MASK | CNT_ERROR);
        if (fifo32_is_empty(&me->send)) {
            v |= CNT_SEND_EMPTY;
        }
        if (fifo32_is_full(&me->send)) {
            v |= CNT_SEND_FULL;
        }
        if (fifo32_is_empty(&other->send)) {
            v |= CNT_RECV_EMPTY;
        }
        if (fifo32_is_full(&other->send)) {
            v |= CNT_RECV_FULL;
        }
        break;
    case REG_RECV:
        if (!(me->cnt & CNT_ENABLE)) {
            v = me->last_recv;
        } else if (fifo32_is_empty(&other->send)) {
            me->cnt |= CNT_ERROR;
            v = me->last_recv;
        } else {
            v = fifo32_pop(&other->send);
            me->last_recv = v;
            pxi_update_irqs(s);
        }
        break;
    default:
        v = 0;
        break;
    }
    return (v >> (8 * (off & 3))) & (size == 4 ? 0xFFFFFFFF :
                                     size == 2 ? 0xFFFF : 0xFF);
}

static void pxi_write(N3DSPxiState *s, int idx, hwaddr off, uint64_t v,
                      unsigned size)
{
    N3DSPxiSide *me = &s->side[idx];
    uint8_t old_sync = me->sync_out;
    N3DSPxiSide *other = &s->side[1 - idx];
    unsigned shift = 8 * (off & 3);
    uint32_t mask = (size == 4 ? 0xFFFFFFFF : size == 2 ? 0xFFFF : 0xFF) << shift;
    uint32_t val = (uint32_t)v << shift;

    switch (off & ~3) {
    case REG_SYNC:
        if (mask & 0xFF00) {
            me->sync_out = (val >> 8) & 0xFF;
            trace_n3ds_pxi_sync(idx, me->sync_out,
                                current_cpu ? current_cpu->cc->get_pc(current_cpu) : 0);
        }
        if (mask & SYNC_IRQEN) {
            me->sync_irqen = (val & SYNC_IRQEN) != 0;
        }
        if ((val & (SYNC_IRQ_A | SYNC_IRQ_B)) && other->sync_irqen) {
            qemu_irq_pulse(other->irq_sync);
        }
        /*
         * The boot ROMs hand-shake through the sync bytes with tight
         * poll loops on both CPUs; with round-robin TCG the writer would
         * otherwise run on and overwrite the byte before the other core
         * ever polls it. Give the other vCPUs a turn right away.
         */
        /*
         * Only the boot ROMs need this: they hand-shake through the sync
         * bytes with tight poll loops on both CPUs, and under round-robin
         * TCG the writer would run on before the other core ever polls.
         * Firmware booted the Luma way uses interrupts and must not pay
         * the round-trip cost.
         */
        if (current_cpu && !s->luma_state && old_sync != me->sync_out) {
            cpu_exit(current_cpu);
        }
        break;
    case REG_CNT:
        if (mask & 0xFF) {
            if (val & CNT_SEND_FLUSH) {
                fifo32_reset(&me->send);
            }
            me->cnt = (me->cnt & ~(CNT_SEND_IRQEN)) | (val & CNT_SEND_IRQEN);
        }
        if (mask & 0xFF00) {
            if (val & CNT_ERROR) {
                me->cnt &= ~CNT_ERROR;
            }
            me->cnt = (me->cnt & ~(CNT_RECV_IRQEN | CNT_ENABLE)) |
                      (val & (CNT_RECV_IRQEN | CNT_ENABLE));
        }
        pxi_update_irqs(s);
        break;
    case REG_SEND:
        if (!(me->cnt & CNT_ENABLE)) {
            break;
        }
        if (fifo32_is_full(&me->send)) {
            me->cnt |= CNT_ERROR;
        } else {
            fifo32_push(&me->send, val);
            trace_n3ds_pxi_send(idx, (uint32_t)val);
            pxi_update_irqs(s);
        }
        break;
    default:
        break;
    }
}

static uint64_t pxi9_read(void *opaque, hwaddr off, unsigned size)
{
    return pxi_read(opaque, 0, off, size);
}

static void pxi9_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    pxi_write(opaque, 0, off, v, size);
}

static uint64_t pxi11_read(void *opaque, hwaddr off, unsigned size)
{
    return pxi_read(opaque, 1, off, size);
}

static void pxi11_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    pxi_write(opaque, 1, off, v, size);
}

static const MemoryRegionOps pxi9_ops = {
    .read = pxi9_read,
    .write = pxi9_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static const MemoryRegionOps pxi11_ops = {
    .read = pxi11_read,
    .write = pxi11_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void n3ds_pxi_reset(DeviceState *dev)
{
    N3DSPxiState *s = N3DS_PXI(dev);
    int i;

    for (i = 0; i < 2; i++) {
        fifo32_reset(&s->side[i].send);
        s->side[i].sync_out = 0;
        s->side[i].cnt = 0;
        s->side[i].sync_irqen = false;
        s->side[i].last_recv = 0;
        s->side[i].send_lvl = false;
        s->side[i].recv_lvl = false;
    }
    /*
     * Console after Luma3DS: both sides read SYNC 0x8000000e - each has sent
     * 0x0e and has the sync IRQ enabled - and both read CNT 0x8101
     * (n3ds-probes io_pxi9, io_pxi11).
     */
    if (s->luma_state) {
        s->side[0].sync_out = 0x0e;
        s->side[1].sync_out = 0x0e;
        s->side[0].sync_irqen = true;
        s->side[1].sync_irqen = true;
        s->side[0].cnt = CNT_ENABLE;
        s->side[1].cnt = CNT_ENABLE;
    }
    pxi_update_irqs(s);
}

static void n3ds_pxi_init(Object *obj)
{
    N3DSPxiState *s = N3DS_PXI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->side[0].iomem, obj, &pxi9_ops, s, "n3ds.pxi9",
                          0x1000);
    memory_region_init_io(&s->side[1].iomem, obj, &pxi11_ops, s, "n3ds.pxi11",
                          0x1000);
    sysbus_init_mmio(sbd, &s->side[0].iomem);
    sysbus_init_mmio(sbd, &s->side[1].iomem);
    for (i = 0; i < 2; i++) {
        fifo32_create(&s->side[i].send, PXI_FIFO_DEPTH);
        sysbus_init_irq(sbd, &s->side[i].irq_sync);
        sysbus_init_irq(sbd, &s->side[i].irq_send);
        sysbus_init_irq(sbd, &s->side[i].irq_recv);
    }
}

static const Property n3ds_pxi_properties[] = {
    DEFINE_PROP_BOOL("luma-state", N3DSPxiState, luma_state, true),
};

static void n3ds_pxi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, n3ds_pxi_properties);

    device_class_set_legacy_reset(dc, n3ds_pxi_reset);
}

static const TypeInfo n3ds_pxi_info = {
    .name          = TYPE_N3DS_PXI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSPxiState),
    .instance_init = n3ds_pxi_init,
    .class_init    = n3ds_pxi_class_init,
};

static void n3ds_pxi_register_types(void)
{
    type_register_static(&n3ds_pxi_info);
}

type_init(n3ds_pxi_register_types)
