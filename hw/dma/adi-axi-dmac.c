/*
 * Minimal model of the Analog Devices AXI-DMAC (register map as used by
 * Linux drivers/dma/dma-axi-dmac.c), for testing the IIO DMABUF path.
 *
 * One channel, streaming (FIFO) source to memory mapped destination, no
 * hardware scatter-gather, no 2D and no hardware cyclic mode. The source
 * is a built-in sample generator: a free running 16-bit little-endian
 * counter, so the guest can check data continuity across transfers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_ADI_AXI_DMAC "adi-axi-dmac"
OBJECT_DECLARE_SIMPLE_TYPE(AdiAxiDmacState, ADI_AXI_DMAC)

#define REG_VERSION             0x000
#define REG_ID                  0x004
#define REG_SCRATCH             0x008
#define REG_INTERFACE_DESC      0x010
#define REG_IRQ_MASK            0x080
#define REG_IRQ_PENDING         0x084
#define REG_IRQ_SOURCE          0x088
#define REG_CTRL                0x400
#define REG_TRANSFER_ID         0x404
#define REG_START_TRANSFER      0x408
#define REG_FLAGS               0x40c
#define REG_DEST_ADDRESS        0x410
#define REG_SRC_ADDRESS         0x414
#define REG_X_LENGTH            0x418
#define REG_Y_LENGTH            0x41c
#define REG_DEST_STRIDE         0x420
#define REG_SRC_STRIDE          0x424
#define REG_TRANSFER_DONE       0x428
#define REG_ACTIVE_TRANSFER_ID  0x42c
#define REG_STATUS              0x430
#define REG_DEST_ADDRESS_HIGH   0x490
#define REG_SRC_ADDRESS_HIGH    0x494

#define CTRL_ENABLE             BIT(0)
#define IRQ_SOT                 BIT(0)
#define IRQ_EOT                 BIT(1)

/* 4.3.a: new enough for the driver to read INTERFACE_DESC */
#define DMAC_VERSION            ((4 << 16) | (3 << 8) | 'a')
/* src: FIFO (2), 64-bit; dest: AXI MM (0), 64-bit (widths are log2 bytes) */
#define DMAC_INTERFACE_DESC     ((2 << 12) | (3 << 8) | (0 << 4) | 3)
#define DMAC_ALIGN_MASK         7
#define DMAC_LENGTH_MASK        0x00ffffff
#define DMAC_NUM_IDS            4

/* Time for one transfer, independent of length: the guest only cares
 * that completion is asynchronous. */
#define DMAC_XFER_NS            (200 * SCALE_US)

typedef struct AdiAxiDmacXfer {
    bool valid;
    uint32_t id;
    uint64_t dest;
    uint32_t len;
} AdiAxiDmacXfer;

struct AdiAxiDmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t scratch;
    uint32_t irq_mask;
    uint32_t irq_pending;
    uint32_t ctrl;
    uint32_t flags;
    uint32_t dest_addr;
    uint32_t x_length;
    uint32_t transfer_done;
    uint32_t next_id;
    uint16_t sample;

    AdiAxiDmacXfer queued;
    AdiAxiDmacXfer active;
};

static void adi_axi_dmac_update_irq(AdiAxiDmacState *s)
{
    qemu_set_irq(s->irq, !!(s->irq_pending & ~s->irq_mask));
}

static void adi_axi_dmac_start_next(AdiAxiDmacState *s)
{
    if (s->active.valid || !s->queued.valid || !(s->ctrl & CTRL_ENABLE)) {
        return;
    }
    s->active = s->queued;
    s->queued.valid = false;
    s->irq_pending |= IRQ_SOT;
    adi_axi_dmac_update_irq(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DMAC_XFER_NS);
}

static void adi_axi_dmac_complete(void *opaque)
{
    AdiAxiDmacState *s = opaque;
    g_autofree uint8_t *buf = NULL;
    uint32_t i;

    if (!s->active.valid) {
        return;
    }

    buf = g_malloc(s->active.len);
    for (i = 0; i + 1 < s->active.len; i += 2) {
        stw_le_p(buf + i, s->sample++);
    }
    if (dma_memory_write(&address_space_memory, s->active.dest, buf,
                         s->active.len, MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "adi-axi-dmac: bad destination 0x%" PRIx64 "\n",
                      s->active.dest);
    }

    s->transfer_done |= BIT(s->active.id);
    s->active.valid = false;
    s->irq_pending |= IRQ_EOT;
    adi_axi_dmac_update_irq(s);
    adi_axi_dmac_start_next(s);
}

static void adi_axi_dmac_disable(AdiAxiDmacState *s)
{
    timer_del(s->timer);
    s->queued.valid = false;
    s->active.valid = false;
    s->next_id = 0;
    s->transfer_done = 0;
}

static uint64_t adi_axi_dmac_read(void *opaque, hwaddr addr, unsigned size)
{
    AdiAxiDmacState *s = opaque;

    switch (addr) {
    case REG_VERSION:
        return DMAC_VERSION;
    case REG_SCRATCH:
        return s->scratch;
    case REG_INTERFACE_DESC:
        return DMAC_INTERFACE_DESC;
    case REG_IRQ_MASK:
        return s->irq_mask;
    case REG_IRQ_PENDING:
    case REG_IRQ_SOURCE:
        return s->irq_pending;
    case REG_CTRL:
        return s->ctrl;
    case REG_TRANSFER_ID:
        return s->next_id;
    case REG_START_TRANSFER:
        return s->queued.valid;
    case REG_FLAGS:
        /* no hardware cyclic mode: flag bits read back as zero */
        return 0;
    case REG_DEST_ADDRESS:
        return s->dest_addr;
    case REG_X_LENGTH:
        return s->x_length;
    case REG_TRANSFER_DONE:
        return s->transfer_done;
    case REG_ACTIVE_TRANSFER_ID:
        return s->active.valid ? s->active.id : s->next_id;
    case REG_STATUS:
        return s->active.valid;
    default:
        /* SRC_ADDRESS, SG_ADDRESS, Y_LENGTH, *_HIGH etc. read as zero,
         * which is how the driver detects the missing features. */
        return 0;
    }
}

static void adi_axi_dmac_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    AdiAxiDmacState *s = opaque;

    switch (addr) {
    case REG_SCRATCH:
        s->scratch = val;
        break;
    case REG_IRQ_MASK:
        s->irq_mask = val & (IRQ_SOT | IRQ_EOT);
        adi_axi_dmac_update_irq(s);
        break;
    case REG_IRQ_PENDING:
        s->irq_pending &= ~val;
        adi_axi_dmac_update_irq(s);
        break;
    case REG_CTRL:
        s->ctrl = val;
        if (!(val & CTRL_ENABLE)) {
            adi_axi_dmac_disable(s);
        }
        break;
    case REG_FLAGS:
        s->flags = val;
        break;
    case REG_DEST_ADDRESS:
        s->dest_addr = val & ~DMAC_ALIGN_MASK;
        break;
    case REG_X_LENGTH:
        s->x_length = (val & DMAC_LENGTH_MASK) | DMAC_ALIGN_MASK;
        break;
    case REG_START_TRANSFER:
        if (!(val & 1) || !(s->ctrl & CTRL_ENABLE) || s->queued.valid) {
            break;
        }
        s->queued.valid = true;
        s->queued.id = s->next_id;
        s->queued.dest = s->dest_addr;
        s->queued.len = s->x_length + 1;
        s->transfer_done &= ~BIT(s->next_id);
        s->next_id = (s->next_id + 1) % DMAC_NUM_IDS;
        adi_axi_dmac_start_next(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps adi_axi_dmac_ops = {
    .read = adi_axi_dmac_read,
    .write = adi_axi_dmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void adi_axi_dmac_reset(DeviceState *dev)
{
    AdiAxiDmacState *s = ADI_AXI_DMAC(dev);

    adi_axi_dmac_disable(s);
    s->scratch = 0;
    s->irq_mask = IRQ_SOT | IRQ_EOT;
    s->irq_pending = 0;
    s->ctrl = 0;
    s->flags = 0;
    s->dest_addr = 0;
    s->x_length = DMAC_ALIGN_MASK;
    s->sample = 0;
}

static void adi_axi_dmac_init(Object *obj)
{
    AdiAxiDmacState *s = ADI_AXI_DMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &adi_axi_dmac_ops, s,
                          TYPE_ADI_AXI_DMAC, 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, adi_axi_dmac_complete, s);
}

static void adi_axi_dmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, adi_axi_dmac_reset);
    dc->desc = "ADI AXI-DMAC test model";
    dc->user_creatable = false;
}

static const TypeInfo adi_axi_dmac_info = {
    .name          = TYPE_ADI_AXI_DMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AdiAxiDmacState),
    .instance_init = adi_axi_dmac_init,
    .class_init    = adi_axi_dmac_class_init,
};

static void adi_axi_dmac_register_types(void)
{
    type_register_static(&adi_axi_dmac_info);
}

type_init(adi_axi_dmac_register_types)
