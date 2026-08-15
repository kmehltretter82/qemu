/*
 * BCM2835 VideoCore IV 3D (V3D) GPU emulation
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 0 "skeleton" model of the VideoCore IV 3D core as found on the
 * Raspberry Pi 0/1/2/3 (bcm2835/6/7).  It implements the register file the
 * mainline Linux vc4 DRM driver (drivers/gpu/drm/vc4) touches: the IDENT
 * identity registers, the interrupt block (INTCTL/INTENA/INTDIS) and the two
 * control-list threads CT0 (binner) and CT1 (renderer).  Kicking a thread
 * (a write to V3D_CTnEA) does not yet execute the control list; it simply
 * completes the job and raises the corresponding done interrupt.  That is
 * enough for the driver to bind, submit a job and observe it complete.
 */

#ifndef HW_DISPLAY_BCM2835_V3D_H
#define HW_DISPLAY_BCM2835_V3D_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_BCM2835_V3D "bcm2835-v3d"
OBJECT_DECLARE_SIMPLE_TYPE(BCM2835V3DState, BCM2835_V3D)

/* The V3D MMIO window is 4 KiB; back every word so debugfs dumps read clean. */
#define BCM2835_V3D_NUM_REGS (0x1000 / sizeof(uint32_t))

struct BCM2835V3DState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;

    /* V3D_INTCTL: latched interrupt status, write-1-to-clear. */
    uint32_t int_status;
    /* Enabled-interrupt mask; INTENA sets bits, INTDIS clears them, and a
     * read of either register returns this mask. */
    uint32_t int_enable;

    /* Generic backing store for every other register in the 4 KiB window. */
    uint32_t regs[BCM2835_V3D_NUM_REGS];

    /* Deferred job completion.  job_pending bit0 = CT0 (bin), bit1 = CT1
     * (render); the bottom half completes whichever threads are pending. */
    QEMUBH *job_bh;
    uint32_t job_pending;
};

#endif /* HW_DISPLAY_BCM2835_V3D_H */
