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
#include "hw/display/vc4_vpm.h"
#include "qom/object.h"

#define TYPE_BCM2835_V3D "bcm2835-v3d"
OBJECT_DECLARE_SIMPLE_TYPE(BCM2835V3DState, BCM2835_V3D)

/* The V3D MMIO window is 4 KiB; back every word so debugfs dumps read clean. */
#define BCM2835_V3D_NUM_REGS (0x1000 / sizeof(uint32_t))

/* A GL shader record carries at most 8 vertex attributes. */
#define BCM2835_V3D_MAX_ATTRS 8

/* The VC4 renders into an on-chip tile buffer and stores it to memory one
 * tile at a time.  A non-multisampled tile is 64x64 pixels. */
#define BCM2835_V3D_TILE_W 64
#define BCM2835_V3D_TILE_H 64
#define BCM2835_V3D_TILE_PIXELS (BCM2835_V3D_TILE_W * BCM2835_V3D_TILE_H)

typedef struct AddressSpace AddressSpace;

/* How many shaded vertices one job may collect before the model stops
 * recording them.  A real binner spills to the tile allocation area. */
#define BCM2835_V3D_MAX_SHADED 256

/* Varyings the model interpolates per vertex: four vec4s worth. */
#define BCM2835_V3D_MAX_VARYINGS 16

/*
 * A vertex after the coordinate shader has run, read back out of the VPM.
 * Mesa's coordinate shader writes four clip coordinates and then the screen
 * position as 12.4 fixed point, Zs and 1/Wc (vc4_program.c:emit_coord_end),
 * which is rows 4, 5 and 6 of its VPM output.
 */
typedef struct V3DShadedVertex {
    float x;        /* screen space, pixels */
    float y;
    float z;
    float rcp_w;
    /* What the vertex shader wrote after the position, for the fragment
     * shader to interpolate. */
    float vary[BCM2835_V3D_MAX_VARYINGS];
} V3DShadedVertex;

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

    /* The tile buffer and the state the render list sets up around it.  The
     * buffer holds one tile of 32-bit colour; it is filled by a clear (and,
     * once there is a rasterizer, by fragments) and written to memory by the
     * tile store packets.  It never holds state between jobs, so like the
     * parsed attributes it is not migrated. */
    uint32_t tile_color[BCM2835_V3D_TILE_PIXELS];
    uint32_t clear_color[2];
    uint32_t clear_z;
    uint32_t clear_stencil;
    uint32_t tile_x;
    uint32_t tile_y;
    uint32_t tlb_stores;
    uint32_t tlb_loads;
    uint64_t tlb_pixels;

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

    /* QPU shader decode probe (no rendering yet; see v3d_probe_shader). */
    uint32_t shader_probes;
    uint32_t shader_incomplete;
    uint64_t shader_instructions;

    /* Vertex Pipe Memory: where the vertex fetch deposits attribute data and
     * where the coordinate and vertex shaders read it from.  Rebuilt for every
     * batch of vertices, so it is not migrated. */
    V3DVPM vpm;
    uint32_t vpm_rows;      /* rows the current batch filled */
    uint32_t vpm_batches;

    /* Vertices the coordinate shader has shaded, in screen space, collected
     * over the bin job so the render job can turn them into fragments. */
    V3DShadedVertex shaded[BCM2835_V3D_MAX_SHADED];
    uint32_t shaded_count;
    uint32_t shaded_dropped;
    uint32_t vary_count;    /* varyings per vertex the vertex shader wrote */

    /* Screen position the shaded coordinates are relative to, from
     * VC4_PACKET_VIEWPORT_OFFSET, in the same 12.4 fixed point. */
    int32_t viewport_x16;
    int32_t viewport_y16;

    /* Primitive mode of the last draw, and rasterizer statistics. */
    uint32_t prim_mode;
    uint32_t frag_shaded;
    uint32_t frag_culled;
    bool frag_reported;

    /* Face culling, from VC4_PACKET_CONFIGURATION_BITS. */
    bool cfg_valid;
    bool cfg_enable_front;
    bool cfg_enable_back;
    bool cfg_cw_front;

    /* Narrow B5 texture-fetch probe (not a general TMU implementation). */
    uint32_t tmu_fetches;
    uint32_t tmu_faults;
};

#endif /* HW_DISPLAY_BCM2835_V3D_H */
