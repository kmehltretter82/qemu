/*
 * BCM2835 VideoCore IV 3D (V3D) GPU emulation
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Model of the VideoCore IV 3D core as found on the Raspberry Pi 0/1/2/3
 * (bcm2835/6/7).  It implements the register file the mainline Linux vc4 DRM
 * driver (drivers/gpu/drm/vc4) touches: the IDENT identity registers, the
 * interrupt block (INTCTL/INTENA/INTDIS) and the two control-list threads
 * CT0 (binner) and CT1 (renderer).  Kicking a thread (a write to V3D_CTnEA)
 * runs its control list to completion and raises the corresponding done
 * interrupt.
 *
 * The control lists are interpreted packet by packet: the model follows the
 * relocated GPU addresses the driver programmed (shader records, vertex and
 * index buffers, tile allocation and framebuffer addresses) and issues the
 * guest-memory DMA the hardware would.  Shading and rasterization are not
 * implemented yet, so no pixels are produced.
 */

#ifndef HW_DISPLAY_BCM2835_V3D_H
#define HW_DISPLAY_BCM2835_V3D_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_BCM2835_V3D "bcm2835-v3d"
OBJECT_DECLARE_SIMPLE_TYPE(BCM2835V3DState, BCM2835_V3D)

/* The V3D MMIO window is 4 KiB; back every word so debugfs dumps read clean. */
#define BCM2835_V3D_NUM_REGS (0x1000 / sizeof(uint32_t))

/* A GL shader record carries at most 8 vertex attributes. */
#define BCM2835_V3D_MAX_ATTRS 8

typedef struct AddressSpace AddressSpace;

/* One vertex attribute as programmed in the relocated GL shader record. */
typedef struct V3DVertexAttr {
    uint32_t addr;      /* absolute DMA address of element 0 */
    uint32_t size;      /* bytes per element (the record stores size - 1) */
    uint32_t stride;    /* bytes between consecutive elements */
} V3DVertexAttr;

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

    /* System RAM is the VC4's uncached DMA address space (there is no MMU). */
    AddressSpace *system_memory;

    /* Binning configuration, from VC4_PACKET_TILE_BINNING_MODE_CONFIG. */
    uint32_t tile_alloc_addr;
    uint32_t tile_alloc_size;
    uint32_t tile_state_addr;
    uint32_t bin_tiles_x;
    uint32_t bin_tiles_y;
    uint32_t bin_flags;
    bool bin_config_valid;

    /* The GL shader record most recently referenced by the bin list.  The
     * attributes are rebuilt on every job, so they are not migrated. */
    uint32_t shader_record_addr;
    bool shader_record_valid;
    bool shader_record_extended;
    uint32_t shader_flags;
    uint32_t fs_code_addr;
    uint32_t fs_uniforms_addr;
    uint32_t vs_code_addr;
    uint32_t vs_uniforms_addr;
    uint32_t cs_code_addr;
    uint32_t cs_uniforms_addr;
    uint32_t attr_count;
    V3DVertexAttr attrs[BCM2835_V3D_MAX_ATTRS];

    /* Render configuration, from VC4_PACKET_TILE_RENDERING_MODE_CONFIG. */
    uint32_t render_fb_addr;
    uint32_t render_width;
    uint32_t render_height;
    uint32_t render_bits;

    /* Cumulative interpreter statistics. */
    uint32_t bin_packets;
    uint32_t render_packets;
    uint32_t bin_prims;
    uint32_t render_stores;
    uint32_t vcd_reads;
    uint64_t vcd_bytes;
    uint32_t ib_reads;
    uint64_t ib_bytes;
    uint32_t cl_errors;

    /* Narrow B5 texture-fetch probe (not a general TMU implementation). */
    uint32_t tmu_fetches;
    uint32_t tmu_faults;
};

#endif /* HW_DISPLAY_BCM2835_V3D_H */
