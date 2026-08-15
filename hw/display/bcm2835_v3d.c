/*
 * BCM2835 VideoCore IV 3D (V3D) GPU emulation -- Phase 0 skeleton.
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This models the register-file / job-control surface of the VideoCore IV 3D
 * core (V3D) as driven by the mainline Linux vc4 DRM driver
 * (drivers/gpu/drm/vc4).  It is deliberately *functional but empty*: it lets
 * the driver probe (IDENT0 identity check), enable/ack interrupts, and kick
 * the two control-list threads CT0 (binner) and CT1 (renderer), completing
 * each job immediately and raising the matching done interrupt.  No control
 * list is parsed and no rendering is performed yet -- that is the job of the
 * later phases.  Register offsets and semantics mirror
 * drivers/gpu/drm/vc4/vc4_regs.h.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "hw/display/bcm2835_v3d.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"

/* Register offsets (see drivers/gpu/drm/vc4/vc4_regs.h). */
#define V3D_IDENT0      0x000
#define V3D_IDENT1      0x004
#define V3D_IDENT2      0x008
#define V3D_SCRATCH     0x010
#define V3D_L2CACTL     0x020
#define V3D_SLCACTL     0x024
#define V3D_INTCTL      0x030
#define V3D_INTENA      0x034
#define V3D_INTDIS      0x038
#define V3D_CT0CS       0x100
#define V3D_CT1CS       0x104
#define V3D_CT0EA       0x108
#define V3D_CT1EA       0x10c
#define V3D_CT0CA       0x110
#define V3D_CT1CA       0x114
#define V3D_CT0LC       0x120
#define V3D_CT1LC       0x124
#define V3D_CT0PC       0x128
#define V3D_CT1PC       0x12c
#define V3D_PCS         0x130
#define V3D_BFC         0x134
#define V3D_RFC         0x138
#define V3D_BPOA        0x308
#define V3D_BPOS        0x30c
#define V3D_ERRSTAT     0xf20

/* Per-thread register strides: CTnCS = CT0CS + 4n, etc. */
#define V3D_CTNCS(n)    (V3D_CT0CS + 4 * (n))
#define V3D_CTNEA(n)    (V3D_CT0EA + 4 * (n))
#define V3D_CTNCA(n)    (V3D_CT0CA + 4 * (n))

/* Interrupt bits, identical layout in INTCTL/INTENA/INTDIS. */
#define V3D_INT_SPILLUSE    (1u << 3)
#define V3D_INT_OUTOMEM     (1u << 2)
#define V3D_INT_FLDONE      (1u << 1)   /* binner (CT0) flush done */
#define V3D_INT_FRDONE      (1u << 0)   /* frame render (CT1) done */
#define V3D_DRIVER_IRQS     (V3D_INT_OUTOMEM | V3D_INT_FLDONE | V3D_INT_FRDONE)

/* CT0CS/CT1CS control/status bits. */
#define V3D_CTRSTA          (1u << 15)  /* reset thread */
#define V3D_CTRUN           (1u << 5)   /* thread running */

/*
 * IDENT0 must read back exactly this or vc4_v3d_bind() aborts with -EINVAL:
 * bytes 'V','3','D' in bits 0/8/16 plus technology version 2 in bits 24-31.
 */
#define V3D_EXPECTED_IDENT0 ((2u << 24) | ('V' << 0) | ('3' << 8) | ('D' << 16))

/*
 * IDENT1/IDENT2 are never validated by the driver -- they are surfaced only
 * through debugfs and the GET_PARAM ioctl.  These values report a Pi-like
 * configuration (12 QPUs, 16 semaphores) for cosmetic faithfulness.
 *   IDENT1: VPM_SIZE[31:28]=8, NSEM[23:16]=16, TUPS[15:12]=2,
 *           QUPS[11:8]=4, NSLC[7:4]=3, REV[3:0]=1  => 12 QPUs, 6 TMUs.
 */
#define V3D_IDENT1_VALUE    0x80102431u
#define V3D_IDENT2_VALUE    0x00000000u

static void bcm2835_v3d_update_irq(BCM2835V3DState *s)
{
    qemu_set_irq(s->irq, !!(s->int_status & s->int_enable));
}

/* Bottom half: complete every pending control-list thread. */
static void bcm2835_v3d_job_bh(void *opaque)
{
    BCM2835V3DState *s = opaque;
    uint32_t pending = s->job_pending;

    s->job_pending = 0;

    for (unsigned n = 0; n < 2; n++) {
        if (!(pending & (1u << n))) {
            continue;
        }

        /*
         * Phase 0: we do not walk the control list.  Mark the whole list
         * consumed (CA advanced to EA so the driver's hangcheck is happy),
         * drop the running flag, bump the frame counter and latch the done
         * interrupt for this thread.
         */
        s->regs[V3D_CTNCA(n) / 4] = s->regs[V3D_CTNEA(n) / 4];
        s->regs[V3D_CTNCS(n) / 4] &= ~V3D_CTRUN;

        if (n == 0) {
            s->regs[V3D_BFC / 4]++;
            s->int_status |= V3D_INT_FLDONE;
        } else {
            s->regs[V3D_RFC / 4]++;
            s->int_status |= V3D_INT_FRDONE;
        }
    }

    bcm2835_v3d_update_irq(s);
}

/* A write to V3D_CTnEA starts thread n; schedule its (immediate) completion. */
static void bcm2835_v3d_kick(BCM2835V3DState *s, unsigned thread)
{
    s->regs[V3D_CTNCS(thread) / 4] |= V3D_CTRUN;
    s->job_pending |= 1u << thread;
    qemu_bh_schedule(s->job_bh);
}

/* A write of V3D_CTRSTA to CTnCS aborts and resets thread n. */
static void bcm2835_v3d_reset_thread(BCM2835V3DState *s, unsigned thread)
{
    s->job_pending &= ~(1u << thread);
    s->regs[V3D_CTNCS(thread) / 4] = 0;
    s->regs[V3D_CTNCA(thread) / 4] = 0;
}

static uint64_t bcm2835_v3d_read(void *opaque, hwaddr offset, unsigned size)
{
    BCM2835V3DState *s = opaque;

    switch (offset) {
    case V3D_IDENT0:
        return V3D_EXPECTED_IDENT0;
    case V3D_IDENT1:
        return V3D_IDENT1_VALUE;
    case V3D_IDENT2:
        return V3D_IDENT2_VALUE;
    case V3D_INTCTL:
        return s->int_status;
    case V3D_INTENA:
    case V3D_INTDIS:
        /* Both registers read back the currently-enabled interrupt mask. */
        return s->int_enable;
    default:
        return s->regs[offset / 4];
    }
}

static void bcm2835_v3d_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    BCM2835V3DState *s = opaque;
    uint32_t val = value;

    switch (offset) {
    case V3D_IDENT0:
    case V3D_IDENT1:
    case V3D_IDENT2:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to read-only IDENT register 0x%" HWADDR_PRIx
                      "\n", __func__, offset);
        break;
    case V3D_INTCTL:
        /* Write-1-to-clear the latched status. */
        s->int_status &= ~val;
        bcm2835_v3d_update_irq(s);
        break;
    case V3D_INTENA:
        /* Write-1-to-enable; writing 0 to a bit is ignored. */
        s->int_enable |= val & V3D_DRIVER_IRQS;
        bcm2835_v3d_update_irq(s);
        break;
    case V3D_INTDIS:
        /* Write-1-to-disable. */
        s->int_enable &= ~val;
        bcm2835_v3d_update_irq(s);
        break;
    case V3D_CT0CS:
    case V3D_CT1CS:
        if (val & V3D_CTRSTA) {
            bcm2835_v3d_reset_thread(s, (offset - V3D_CT0CS) / 4);
        }
        break;
    case V3D_CT0EA:
    case V3D_CT1EA:
        /* Writing the end address is what starts the thread. */
        s->regs[offset / 4] = val;
        bcm2835_v3d_kick(s, (offset - V3D_CT0EA) / 4);
        break;
    default:
        s->regs[offset / 4] = val;
        break;
    }
}

static const MemoryRegionOps bcm2835_v3d_ops = {
    .read = bcm2835_v3d_read,
    .write = bcm2835_v3d_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void bcm2835_v3d_reset(DeviceState *dev)
{
    BCM2835V3DState *s = BCM2835_V3D(dev);

    qemu_bh_cancel(s->job_bh);
    s->job_pending = 0;
    s->int_status = 0;
    s->int_enable = 0;
    memset(s->regs, 0, sizeof(s->regs));
    bcm2835_v3d_update_irq(s);
}

static void bcm2835_v3d_realize(DeviceState *dev, Error **errp)
{
    BCM2835V3DState *s = BCM2835_V3D(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bcm2835_v3d_ops, s,
                          TYPE_BCM2835_V3D, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    s->job_bh = qemu_bh_new_guarded(bcm2835_v3d_job_bh, s,
                                    &dev->mem_reentrancy_guard);
}

static void bcm2835_v3d_unrealize(DeviceState *dev)
{
    BCM2835V3DState *s = BCM2835_V3D(dev);

    if (s->job_bh) {
        qemu_bh_delete(s->job_bh);
        s->job_bh = NULL;
    }
}

static const VMStateDescription bcm2835_v3d_vmstate = {
    .name = "bcm2835_v3d",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(int_status, BCM2835V3DState),
        VMSTATE_UINT32(int_enable, BCM2835V3DState),
        VMSTATE_UINT32(job_pending, BCM2835V3DState),
        VMSTATE_UINT32_ARRAY(regs, BCM2835V3DState, BCM2835_V3D_NUM_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2835_v3d_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bcm2835_v3d_realize;
    dc->unrealize = bcm2835_v3d_unrealize;
    device_class_set_legacy_reset(dc, bcm2835_v3d_reset);
    dc->vmsd = &bcm2835_v3d_vmstate;
}

static const TypeInfo bcm2835_v3d_info = {
    .name          = TYPE_BCM2835_V3D,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835V3DState),
    .class_init    = bcm2835_v3d_class_init,
};

static void bcm2835_v3d_register_types(void)
{
    type_register_static(&bcm2835_v3d_info);
}

type_init(bcm2835_v3d_register_types);
