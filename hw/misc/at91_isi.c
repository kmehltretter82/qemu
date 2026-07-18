/*
 * Atmel/Microchip AT91 Image Sensor Interface (ISI_V2, as in the SAM9G45).
 *
 * Models the register/DMA contract used by Linux's atmel-isi capture driver:
 * clear-on-read status with INTEN/INTDIS/INTMASK, the SRST/DIS interrupt
 * handshakes, and the per-frame codec/preview DMA channels that fetch a
 * three-word frame-buffer descriptor (address, control, next) and raise
 * CXFR_DONE/PXFR_DONE per captured frame.
 *
 * There is no camera pixel bus in QEMU, so captured frames are synthesized:
 * each frame starts with a 32-bit little-endian frame counter followed by
 * bytes generated from a fixed position/counter formula that guest tests can
 * verify.  The companion I2C sensor (e.g. ov2640) only models the control
 * interface; frame content originates here.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/misc/at91_isi.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "trace.h"

#define ISI_IOMEM_SIZE     0x4000

#define ISI_CFG1           0x00
#define ISI_CFG2           0x04
#define ISI_PSIZE          0x08
#define ISI_PDECF          0x0c
#define ISI_Y2R_SET0       0x10
#define ISI_Y2R_SET1       0x14
#define ISI_R2Y_SET0       0x18
#define ISI_R2Y_SET1       0x1c
#define ISI_R2Y_SET2       0x20
#define ISI_CTRL           0x24
#define ISI_STATUS         0x28
#define ISI_INTEN          0x2c
#define ISI_INTDIS         0x30
#define ISI_INTMASK        0x34
#define ISI_DMA_CHER       0x38
#define ISI_DMA_CHDR       0x3c
#define ISI_DMA_CHSR       0x40
#define ISI_DMA_P_ADDR     0x44
#define ISI_DMA_P_CTRL     0x48
#define ISI_DMA_P_DSCR     0x4c
#define ISI_DMA_C_ADDR     0x50
#define ISI_DMA_C_CTRL     0x54
#define ISI_DMA_C_DSCR     0x58

#define ISI_CTRL_EN        BIT(0)
#define ISI_CTRL_DIS       BIT(1)
#define ISI_CTRL_SRST      BIT(2)
#define ISI_CTRL_CDC       BIT(8)

/* Status register events (clear-on-read); DIS/SRST completion share the
 * CTRL bit positions, CDC pending shares the CTRL_CDC position. */
#define ISI_SR_VSYNC       BIT(10)
#define ISI_SR_PXFR_DONE   BIT(16)
#define ISI_SR_CXFR_DONE   BIT(17)

#define ISI_CFG1_FRATE_SHIFT   8
#define ISI_CFG1_FRATE_MASK    (7u << ISI_CFG1_FRATE_SHIFT)

#define ISI_CFG2_IM_VSIZE_MASK 0x7ff
#define ISI_CFG2_IM_HSIZE_SHIFT 16
#define ISI_PSIZE_VSIZE_MASK   0x3ff
#define ISI_PSIZE_HSIZE_SHIFT  16

#define ISI_DMA_P_CH       BIT(0)
#define ISI_DMA_C_CH       BIT(1)

#define ISI_DMA_CTRL_WB    BIT(1)
#define ISI_DMA_CTRL_DONE  BIT(3)

/* Nominal sensor frame interval; CFG1.FRATE divides it further. */
#define ISI_FRAME_INTERVAL_NS  (NANOSECONDS_PER_SECOND / 30)

struct AT91ISIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *frame_timer;

    uint32_t cfg1;
    uint32_t cfg2;
    uint32_t psize;
    uint32_t pdecf;
    uint32_t y2r_set[2];
    uint32_t r2y_set[3];
    uint32_t status;
    uint32_t intmask;
    uint32_t chsr;
    uint32_t dma_p_addr;
    uint32_t dma_p_ctrl;
    uint32_t dma_p_dscr;
    uint32_t dma_c_addr;
    uint32_t dma_c_ctrl;
    uint32_t dma_c_dscr;
    uint32_t frame;
    bool enabled;
    bool cdc;
};

static void isi_update_irq(AT91ISIState *s)
{
    qemu_set_irq(s->irq, (s->status & s->intmask) != 0);
}

static int64_t isi_frame_interval(const AT91ISIState *s)
{
    unsigned div = ((s->cfg1 & ISI_CFG1_FRATE_MASK) >> ISI_CFG1_FRATE_SHIFT)
                   + 1;

    return ISI_FRAME_INTERVAL_NS * div;
}

static void isi_arm_timer(AT91ISIState *s)
{
    timer_mod(s->frame_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + isi_frame_interval(s));
}

/* Deterministic frame content: a leading little-endian frame counter,
 * then bytes from a fixed formula over (position, counter). */
static void isi_fill_frame(uint32_t addr, unsigned len, uint32_t counter)
{
    uint8_t buf[1024];
    unsigned pos = 0;

    while (pos < len) {
        unsigned chunk = MIN(len - pos, (unsigned)sizeof(buf));
        unsigned i;

        for (i = 0; i < chunk; i++) {
            unsigned j = pos + i;

            if (j < 4) {
                buf[i] = counter >> (8 * j);
            } else {
                buf[i] = j * 3 + counter * 11;
            }
        }
        address_space_write(&address_space_memory, addr + pos,
                            MEMTXATTRS_UNSPECIFIED, buf, chunk);
        pos += chunk;
    }
}

/* Capture one frame on a DMA channel: fetch the frame-buffer descriptor,
 * fill the frame, write back DONE if requested, chain to the next
 * descriptor and idle the channel until the driver re-enables it. */
static void isi_capture_frame(AT91ISIState *s, bool codec)
{
    uint32_t *addr = codec ? &s->dma_c_addr : &s->dma_p_addr;
    uint32_t *ctrl = codec ? &s->dma_c_ctrl : &s->dma_p_ctrl;
    uint32_t *dscr = codec ? &s->dma_c_dscr : &s->dma_p_dscr;
    uint32_t fbd[3];
    unsigned width, height, len;

    if (codec) {
        width = ((s->cfg2 >> ISI_CFG2_IM_HSIZE_SHIFT)
                 & ISI_CFG2_IM_VSIZE_MASK) + 1;
        height = (s->cfg2 & ISI_CFG2_IM_VSIZE_MASK) + 1;
    } else {
        width = ((s->psize >> ISI_PSIZE_HSIZE_SHIFT)
                 & ISI_PSIZE_VSIZE_MASK) + 1;
        height = (s->psize & ISI_PSIZE_VSIZE_MASK) + 1;
    }
    /* Codec path streams YCbCr 4:2:2, preview path RGB565: 2 B/pixel. */
    len = width * height * 2;

    address_space_read(&address_space_memory, *dscr, MEMTXATTRS_UNSPECIFIED,
                       fbd, sizeof(fbd));
    fbd[0] = le32_to_cpu(fbd[0]);
    fbd[1] = le32_to_cpu(fbd[1]);
    fbd[2] = le32_to_cpu(fbd[2]);

    isi_fill_frame(fbd[0], len, s->frame);
    trace_at91_isi_frame(codec ? "codec" : "preview", fbd[0], len, s->frame);

    if (fbd[1] & ISI_DMA_CTRL_WB) {
        uint32_t wb = cpu_to_le32(fbd[1] | ISI_DMA_CTRL_DONE);

        address_space_write(&address_space_memory, *dscr + 4,
                            MEMTXATTRS_UNSPECIFIED, &wb, sizeof(wb));
    }
    *addr = fbd[0] + len;
    *ctrl = fbd[1] | ISI_DMA_CTRL_DONE;
    *dscr = fbd[2];
    s->chsr &= ~(codec ? ISI_DMA_C_CH : ISI_DMA_P_CH);
    s->status |= codec ? ISI_SR_CXFR_DONE : ISI_SR_PXFR_DONE;
    s->frame++;
}

static void isi_frame_tick(void *opaque)
{
    AT91ISIState *s = opaque;

    if (!s->enabled) {
        return;
    }
    s->status |= ISI_SR_VSYNC;
    if (s->cdc && (s->chsr & ISI_DMA_C_CH)) {
        isi_capture_frame(s, true);
    }
    if (s->chsr & ISI_DMA_P_CH) {
        isi_capture_frame(s, false);
    }
    isi_update_irq(s);
    isi_arm_timer(s);
}

static void isi_soft_reset(AT91ISIState *s)
{
    timer_del(s->frame_timer);
    s->cfg1 = 0;
    s->cfg2 = 0;
    s->psize = 0;
    s->pdecf = 0x10;
    memset(s->y2r_set, 0, sizeof(s->y2r_set));
    memset(s->r2y_set, 0, sizeof(s->r2y_set));
    s->status = 0;
    s->chsr = 0;
    s->dma_p_addr = 0;
    s->dma_p_ctrl = 0;
    s->dma_p_dscr = 0;
    s->dma_c_addr = 0;
    s->dma_c_ctrl = 0;
    s->dma_c_dscr = 0;
    s->enabled = false;
    s->cdc = false;
}

static void isi_ctrl_write(AT91ISIState *s, uint32_t val)
{
    trace_at91_isi_ctrl(val);
    if (val & ISI_CTRL_SRST) {
        /* Reset completes immediately; the completion flag interrupts
         * through the mask the driver set up beforehand. */
        isi_soft_reset(s);
        s->status |= ISI_CTRL_SRST;
        isi_update_irq(s);
        return;
    }
    if (val & ISI_CTRL_DIS) {
        s->enabled = false;
        s->cdc = false;
        timer_del(s->frame_timer);
        s->status |= ISI_CTRL_DIS;
    }
    if (val & ISI_CTRL_EN) {
        s->enabled = true;
        isi_arm_timer(s);
    }
    if (val & ISI_CTRL_CDC) {
        s->cdc = true;
    }
    isi_update_irq(s);
}

static uint64_t isi_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91ISIState *s = opaque;
    uint32_t value;

    switch (offset) {
    case ISI_CFG1:       return s->cfg1;
    case ISI_CFG2:       return s->cfg2;
    case ISI_PSIZE:      return s->psize;
    case ISI_PDECF:      return s->pdecf;
    case ISI_Y2R_SET0:   return s->y2r_set[0];
    case ISI_Y2R_SET1:   return s->y2r_set[1];
    case ISI_R2Y_SET0:   return s->r2y_set[0];
    case ISI_R2Y_SET1:   return s->r2y_set[1];
    case ISI_R2Y_SET2:   return s->r2y_set[2];
    case ISI_STATUS:
        value = s->status;
        /* The codec-pending level bit reflects an armed codec capture. */
        if (s->enabled && s->cdc && (s->chsr & ISI_DMA_C_CH)) {
            value |= ISI_CTRL_CDC;
        }
        s->status = 0;
        isi_update_irq(s);
        return value;
    case ISI_INTMASK:    return s->intmask;
    case ISI_DMA_CHSR:   return s->chsr;
    case ISI_DMA_P_ADDR: return s->dma_p_addr;
    case ISI_DMA_P_CTRL: return s->dma_p_ctrl;
    case ISI_DMA_P_DSCR: return s->dma_p_dscr;
    case ISI_DMA_C_ADDR: return s->dma_c_addr;
    case ISI_DMA_C_CTRL: return s->dma_c_ctrl;
    case ISI_DMA_C_DSCR: return s->dma_c_dscr;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "at91-isi: unimplemented read at 0x%03" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
}

static void isi_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91ISIState *s = opaque;
    uint32_t val = value;

    switch (offset) {
    case ISI_CFG1:       s->cfg1 = val;       break;
    case ISI_CFG2:       s->cfg2 = val;       break;
    case ISI_PSIZE:      s->psize = val;      break;
    case ISI_PDECF:      s->pdecf = val;      break;
    case ISI_Y2R_SET0:   s->y2r_set[0] = val; break;
    case ISI_Y2R_SET1:   s->y2r_set[1] = val; break;
    case ISI_R2Y_SET0:   s->r2y_set[0] = val; break;
    case ISI_R2Y_SET1:   s->r2y_set[1] = val; break;
    case ISI_R2Y_SET2:   s->r2y_set[2] = val; break;
    case ISI_CTRL:
        isi_ctrl_write(s, val);
        break;
    case ISI_INTEN:
        s->intmask |= val;
        isi_update_irq(s);
        break;
    case ISI_INTDIS:
        s->intmask &= ~val;
        isi_update_irq(s);
        break;
    case ISI_DMA_CHER:
        s->chsr |= val & (ISI_DMA_P_CH | ISI_DMA_C_CH);
        break;
    case ISI_DMA_CHDR:
        s->chsr &= ~val;
        break;
    case ISI_DMA_P_ADDR: s->dma_p_addr = val; break;
    case ISI_DMA_P_CTRL: s->dma_p_ctrl = val; break;
    case ISI_DMA_P_DSCR: s->dma_p_dscr = val; break;
    case ISI_DMA_C_ADDR: s->dma_c_addr = val; break;
    case ISI_DMA_C_CTRL: s->dma_c_ctrl = val; break;
    case ISI_DMA_C_DSCR: s->dma_c_dscr = val; break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "at91-isi: unimplemented write at 0x%03" HWADDR_PRIx
                      " value 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps isi_ops = {
    .read = isi_read,
    .write = isi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void isi_reset(DeviceState *dev)
{
    AT91ISIState *s = AT91_ISI(dev);

    isi_soft_reset(s);
    s->intmask = 0;
    s->frame = 0;
    isi_update_irq(s);
}

static const VMStateDescription vmstate_at91_isi = {
    .name = "at91-isi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg1, AT91ISIState),
        VMSTATE_UINT32(cfg2, AT91ISIState),
        VMSTATE_UINT32(psize, AT91ISIState),
        VMSTATE_UINT32(pdecf, AT91ISIState),
        VMSTATE_UINT32_ARRAY(y2r_set, AT91ISIState, 2),
        VMSTATE_UINT32_ARRAY(r2y_set, AT91ISIState, 3),
        VMSTATE_UINT32(status, AT91ISIState),
        VMSTATE_UINT32(intmask, AT91ISIState),
        VMSTATE_UINT32(chsr, AT91ISIState),
        VMSTATE_UINT32(dma_p_addr, AT91ISIState),
        VMSTATE_UINT32(dma_p_ctrl, AT91ISIState),
        VMSTATE_UINT32(dma_p_dscr, AT91ISIState),
        VMSTATE_UINT32(dma_c_addr, AT91ISIState),
        VMSTATE_UINT32(dma_c_ctrl, AT91ISIState),
        VMSTATE_UINT32(dma_c_dscr, AT91ISIState),
        VMSTATE_UINT32(frame, AT91ISIState),
        VMSTATE_BOOL(enabled, AT91ISIState),
        VMSTATE_BOOL(cdc, AT91ISIState),
        VMSTATE_TIMER_PTR(frame_timer, AT91ISIState),
        VMSTATE_END_OF_LIST()
    },
};

static void isi_init(Object *obj)
{
    AT91ISIState *s = AT91_ISI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &isi_ops, s, "at91-isi",
                          ISI_IOMEM_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, isi_frame_tick, s);
}

static void isi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Atmel AT91 Image Sensor Interface";
    dc->vmsd = &vmstate_at91_isi;
    device_class_set_legacy_reset(dc, isi_reset);
}

static const TypeInfo isi_types[] = {
    {
        .name = TYPE_AT91_ISI,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AT91ISIState),
        .instance_init = isi_init,
        .class_init = isi_class_init,
    },
};

DEFINE_TYPES(isi_types)
