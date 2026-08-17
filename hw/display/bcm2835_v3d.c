/*
 * BCM2835 VideoCore IV 3D (V3D) GPU emulation.
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This models the register-file / job-control surface of the VideoCore IV 3D
 * core (V3D) as driven by the mainline Linux vc4 DRM driver
 * (drivers/gpu/drm/vc4), plus an interpreter for the control lists it
 * submits.  The driver can probe (IDENT0 identity check), enable/ack
 * interrupts, and kick the two control-list threads CT0 (binner) and CT1
 * (renderer).  Kicking a thread walks its control list packet by packet,
 * following the relocated GPU addresses the driver programmed -- shader
 * records, vertex and index buffers, the tile allocation area, the
 * framebuffer -- and issuing the guest-memory DMA the hardware would.  The
 * job then completes and raises the matching done interrupt.
 *
 * Shading (the QPU ISA) and rasterization are not implemented, so no pixels
 * are produced yet: the binner does not emit primitives into the tile lists,
 * it only caps each tile list with a return so the render list can walk it.
 *
 * Register offsets and semantics mirror drivers/gpu/drm/vc4/vc4_regs.h;
 * packet opcodes, sizes and field layouts mirror
 * drivers/gpu/drm/vc4/vc4_packet.h and the (authoritative, since it rewrites
 * them) validator in drivers/gpu/drm/vc4/vc4_validate.c.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "hw/display/bcm2835_v3d.h"
#include "hw/display/vc4_qpu.h"
#include "hw/display/vc4_tiling.h"
#include "hw/display/vc4_vpm.h"
#include "hw/core/irq.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "migration/vmstate.h"
#include "trace.h"

#include <math.h>

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

/* The bcm2835 DT bus maps device DMA addresses 0x40000000..0x5fffffff to
 * guest RAM 0..0x1fffffff (dma-ranges on the simple-bus node). */
#define V3D_DMA_BUS_BASE   0x40000000u
#define V3D_DMA_BUS_LIMIT  0x60000000u

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

/* Control-list packet opcodes (drivers/gpu/drm/vc4/vc4_packet.h).  The
 * kernel's copy omits RETURN_FROM_SUB_LIST because the validator never
 * whitelists it: it is written into the tile lists by the binner, not by
 * userspace. */
enum {
    VC4_PACKET_HALT                         = 0,
    VC4_PACKET_NOP                          = 1,
    VC4_PACKET_FLUSH                        = 4,
    VC4_PACKET_FLUSH_ALL                    = 5,
    VC4_PACKET_START_TILE_BINNING           = 6,
    VC4_PACKET_INCREMENT_SEMAPHORE          = 7,
    VC4_PACKET_WAIT_ON_SEMAPHORE            = 8,
    VC4_PACKET_BRANCH                       = 16,
    VC4_PACKET_BRANCH_TO_SUB_LIST           = 17,
    VC4_PACKET_RETURN_FROM_SUB_LIST         = 18,
    VC4_PACKET_STORE_MS_TILE_BUFFER         = 24,
    VC4_PACKET_STORE_MS_TILE_BUFFER_AND_EOF = 25,
    VC4_PACKET_STORE_FULL_RES_TILE_BUFFER   = 26,
    VC4_PACKET_LOAD_FULL_RES_TILE_BUFFER    = 27,
    VC4_PACKET_STORE_TILE_BUFFER_GENERAL    = 28,
    VC4_PACKET_LOAD_TILE_BUFFER_GENERAL     = 29,
    VC4_PACKET_GL_INDEXED_PRIMITIVE         = 32,
    VC4_PACKET_GL_ARRAY_PRIMITIVE           = 33,
    VC4_PACKET_COMPRESSED_PRIMITIVE         = 48,
    VC4_PACKET_CLIPPED_COMPRESSED_PRIMITIVE = 49,
    VC4_PACKET_PRIMITIVE_LIST_FORMAT        = 56,
    VC4_PACKET_GL_SHADER_STATE              = 64,
    VC4_PACKET_NV_SHADER_STATE              = 65,
    VC4_PACKET_VG_SHADER_STATE              = 66,
    VC4_PACKET_CONFIGURATION_BITS           = 96,
    VC4_PACKET_FLAT_SHADE_FLAGS             = 97,
    VC4_PACKET_POINT_SIZE                   = 98,
    VC4_PACKET_LINE_WIDTH                   = 99,
    VC4_PACKET_RHT_X_BOUNDARY               = 100,
    VC4_PACKET_DEPTH_OFFSET                 = 101,
    VC4_PACKET_CLIP_WINDOW                  = 102,
    VC4_PACKET_VIEWPORT_OFFSET              = 103,
    VC4_PACKET_Z_CLIPPING                   = 104,
    VC4_PACKET_CLIPPER_XY_SCALING           = 105,
    VC4_PACKET_CLIPPER_Z_SCALING            = 106,
    VC4_PACKET_TILE_BINNING_MODE_CONFIG     = 112,
    VC4_PACKET_TILE_RENDERING_MODE_CONFIG   = 113,
    VC4_PACKET_CLEAR_COLORS                 = 114,
    VC4_PACKET_TILE_COORDINATES             = 115,
    VC4_PACKET_GEM_HANDLES                  = 254,
};

/* Low bits of the address word of the tile buffer load/store packets. */
#define VC4_LOADSTORE_TILE_BUFFER_EOF   (1u << 3)

/* Fields of the u16 in LOAD/STORE_TILE_BUFFER_GENERAL.  The memory layout
 * values are the VC4_TILING_* ones from vc4_tiling.h. */
#define VC4_LOADSTORE_BUFFER(bits)  ((bits) & 0x7)
#define VC4_LOADSTORE_TILING(bits)  (((bits) >> 4) & 0x3)
#define VC4_LOADSTORE_FORMAT(bits)  (((bits) >> 8) & 0x3)
#define VC4_LOADSTORE_BUFFER_NONE   0
#define VC4_LOADSTORE_BUFFER_COLOR  1
#define VC4_LOADSTORE_TILE_BUFFER_RGBA8888  0
#define VC4_STORE_TILE_BUFFER_DISABLE_COLOR_CLEAR (1u << 13)

/* Fields of the u16 in TILE_RENDERING_MODE_CONFIG.  Note the colour format
 * encoding differs from the load/store packets above. */
#define VC4_RENDER_CONFIG_FORMAT(bits)        (((bits) >> 2) & 0x3)
#define VC4_RENDER_CONFIG_MEMORY_FORMAT(bits) (((bits) >> 6) & 0x3)
#define VC4_RENDER_CONFIG_FORMAT_RGBA8888     1
/* Bits 4:3 of the TILE_BINNING_MODE_CONFIG flags select the size of the
 * initial per-tile list block: 32 << n bytes. */
#define VC4_BIN_CONFIG_INIT_BLOCK_SHIFT 3
#define VC4_BIN_CONFIG_INIT_BLOCK_MASK  0x3u

/* Opcode -> total packet length in bytes (including the opcode byte) and
 * name.  A zero length means "not a packet we know", which stops the walk. */
typedef struct V3DPacketInfo {
    uint8_t size;
    const char *name;
} V3DPacketInfo;

#define V3D_PACKET(op, len) [op] = { len, #op }

static const V3DPacketInfo v3d_packets[256] = {
    V3D_PACKET(VC4_PACKET_HALT, 1),
    V3D_PACKET(VC4_PACKET_NOP, 1),
    V3D_PACKET(VC4_PACKET_FLUSH, 1),
    V3D_PACKET(VC4_PACKET_FLUSH_ALL, 1),
    V3D_PACKET(VC4_PACKET_START_TILE_BINNING, 1),
    V3D_PACKET(VC4_PACKET_INCREMENT_SEMAPHORE, 1),
    V3D_PACKET(VC4_PACKET_WAIT_ON_SEMAPHORE, 1),
    V3D_PACKET(VC4_PACKET_BRANCH, 5),
    V3D_PACKET(VC4_PACKET_BRANCH_TO_SUB_LIST, 5),
    V3D_PACKET(VC4_PACKET_RETURN_FROM_SUB_LIST, 1),
    V3D_PACKET(VC4_PACKET_STORE_MS_TILE_BUFFER, 1),
    V3D_PACKET(VC4_PACKET_STORE_MS_TILE_BUFFER_AND_EOF, 1),
    V3D_PACKET(VC4_PACKET_STORE_FULL_RES_TILE_BUFFER, 5),
    V3D_PACKET(VC4_PACKET_LOAD_FULL_RES_TILE_BUFFER, 5),
    V3D_PACKET(VC4_PACKET_STORE_TILE_BUFFER_GENERAL, 7),
    V3D_PACKET(VC4_PACKET_LOAD_TILE_BUFFER_GENERAL, 7),
    V3D_PACKET(VC4_PACKET_GL_INDEXED_PRIMITIVE, 14),
    V3D_PACKET(VC4_PACKET_GL_ARRAY_PRIMITIVE, 10),
    V3D_PACKET(VC4_PACKET_COMPRESSED_PRIMITIVE, 1),
    V3D_PACKET(VC4_PACKET_CLIPPED_COMPRESSED_PRIMITIVE, 1),
    V3D_PACKET(VC4_PACKET_PRIMITIVE_LIST_FORMAT, 2),
    V3D_PACKET(VC4_PACKET_GL_SHADER_STATE, 5),
    V3D_PACKET(VC4_PACKET_NV_SHADER_STATE, 5),
    V3D_PACKET(VC4_PACKET_VG_SHADER_STATE, 5),
    V3D_PACKET(VC4_PACKET_CONFIGURATION_BITS, 4),
    V3D_PACKET(VC4_PACKET_FLAT_SHADE_FLAGS, 5),
    V3D_PACKET(VC4_PACKET_POINT_SIZE, 5),
    V3D_PACKET(VC4_PACKET_LINE_WIDTH, 5),
    V3D_PACKET(VC4_PACKET_RHT_X_BOUNDARY, 3),
    V3D_PACKET(VC4_PACKET_DEPTH_OFFSET, 5),
    V3D_PACKET(VC4_PACKET_CLIP_WINDOW, 9),
    V3D_PACKET(VC4_PACKET_VIEWPORT_OFFSET, 5),
    V3D_PACKET(VC4_PACKET_Z_CLIPPING, 9),
    V3D_PACKET(VC4_PACKET_CLIPPER_XY_SCALING, 9),
    V3D_PACKET(VC4_PACKET_CLIPPER_Z_SCALING, 9),
    V3D_PACKET(VC4_PACKET_TILE_BINNING_MODE_CONFIG, 16),
    V3D_PACKET(VC4_PACKET_TILE_RENDERING_MODE_CONFIG, 11),
    V3D_PACKET(VC4_PACKET_CLEAR_COLORS, 14),
    V3D_PACKET(VC4_PACKET_TILE_COORDINATES, 3),
    /* GEM_HANDLES is consumed by the kernel validator and never reaches the
     * executed list, but size it so a stray one is skipped rather than
     * aborting the walk. */
    V3D_PACKET(VC4_PACKET_GEM_HANDLES, 9),
};

#undef V3D_PACKET

/* Interpreter limits.  A malformed or hostile control list must not be able
 * to spin the bottom half or read unbounded amounts of guest memory. */
#define V3D_MAX_PACKETS         65536u  /* packets walked per job */
#define V3D_MAX_SUBLIST_DEPTH   4       /* nested BRANCH_TO_SUB_LIST */
#define V3D_MAX_VERTICES        4096u   /* vertices fetched per primitive */
#define V3D_JOB_DMA_BUDGET      (4u << 20) /* vertex+index bytes per job */

/* Walk state for one control-list thread. */
typedef struct V3DCLWalker {
    BCM2835V3DState *s;
    unsigned thread;        /* 0 = bin (CT0), 1 = render (CT1) */
    uint32_t ca;            /* thread start address */
    uint32_t ea;            /* thread end address */
    uint32_t pc;            /* address of the packet being executed */
    uint32_t next;          /* address of the next packet; dispatch may move it */
    uint32_t packets;
    uint32_t ret_stack[V3D_MAX_SUBLIST_DEPTH];
    unsigned ret_depth;
    uint64_t dma_left;      /* remaining vertex/index fetch budget */
    const char *stop;       /* non-NULL once the walk should end */
    bool failed;            /* stop reason is an error, not a clean end */
} V3DCLWalker;

static void bcm2835_v3d_update_irq(BCM2835V3DState *s)
{
    qemu_set_irq(s->irq, !!(s->int_status & s->int_enable));
}

static bool bcm2835_v3d_dma_read(BCM2835V3DState *s, hwaddr addr,
                                  void *buf, size_t len)
{
    MemTxResult result;
    hwaddr cpu_addr = addr;

    if (cpu_addr >= V3D_DMA_BUS_BASE && cpu_addr < V3D_DMA_BUS_LIMIT)
        cpu_addr -= V3D_DMA_BUS_BASE;

    result = dma_memory_read(s->system_memory, cpu_addr, buf, len,
                             MEMTXATTRS_UNSPECIFIED);
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bcm2835-v3d: guest DMA read at 0x%08" PRIx64
                      " (%zu bytes) failed: %d\n",
                      (uint64_t)addr, len, result);
        return false;
    }
    return true;
}

static bool bcm2835_v3d_dma_write(BCM2835V3DState *s, hwaddr addr,
                                  const void *buf, size_t len)
{
    MemTxResult result;
    hwaddr cpu_addr = addr;

    if (cpu_addr >= V3D_DMA_BUS_BASE && cpu_addr < V3D_DMA_BUS_LIMIT)
        cpu_addr -= V3D_DMA_BUS_BASE;

    result = dma_memory_write(s->system_memory, cpu_addr, buf, len,
                              MEMTXATTRS_UNSPECIFIED);
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bcm2835-v3d: guest DMA write at 0x%08" PRIx64
                      " (%zu bytes) failed: %d\n",
                      (uint64_t)addr, len, result);
        return false;
    }
    return true;
}

static bool v3d_read_u8(BCM2835V3DState *s, uint32_t addr, uint8_t *out)
{
    return bcm2835_v3d_dma_read(s, addr, out, sizeof(*out));
}

static bool v3d_read_u16(BCM2835V3DState *s, uint32_t addr, uint16_t *out)
{
    uint8_t buf[2];

    if (!bcm2835_v3d_dma_read(s, addr, buf, sizeof(buf)))
        return false;
    *out = lduw_le_p(buf);
    return true;
}

static bool v3d_read_u32(BCM2835V3DState *s, uint32_t addr, uint32_t *out)
{
    uint8_t buf[4];

    if (!bcm2835_v3d_dma_read(s, addr, buf, sizeof(buf)))
        return false;
    *out = ldl_le_p(buf);
    return true;
}

/* Stop the walk.  The first reason wins so the trace shows what went wrong
 * rather than a cascade of follow-on failures. */
static void v3d_walk_stop(V3DCLWalker *w, const char *why, bool failed)
{
    if (w->stop)
        return;
    w->stop = why;
    w->failed = failed;
    if (failed)
        w->s->cl_errors++;
}

/*
 * Parse the GL shader record the bin list pointed at.  The kernel validator
 * has already relocated every address in it to an absolute DMA address, so
 * this is the layout the hardware sees (see validate_gl_shader_rec()):
 *
 *   +0   u16  flags (bit 0 = FS single-threaded)
 *   +4   u32  FS code       +8   u32  FS uniforms
 *   +16  u32  VS code       +20  u32  VS uniforms
 *   +28  u32  CS code       +32  u32  CS uniforms
 *   +36 + i*8  attribute i: u32 address, u8 (size - 1), u8 stride
 *   extended records carry the high stride bits at +100 + i*4
 */
static void v3d_parse_shader_record(V3DCLWalker *w)
{
    BCM2835V3DState *s = w->s;
    uint8_t rec[100 + BCM2835_V3D_MAX_ATTRS * 4];
    uint32_t size;
    unsigned i;

    size = s->shader_record_extended ? 100 + s->attr_count * 4
                                     : 36 + s->attr_count * 8;
    assert(size <= sizeof(rec));

    s->shader_record_valid = false;
    if (!bcm2835_v3d_dma_read(s, s->shader_record_addr, rec, size)) {
        v3d_walk_stop(w, "shader record unreadable", true);
        return;
    }

    s->shader_flags = lduw_le_p(rec);
    s->fs_code_addr = ldl_le_p(rec + 4);
    s->fs_uniforms_addr = ldl_le_p(rec + 8);
    s->vs_code_addr = ldl_le_p(rec + 16);
    s->vs_uniforms_addr = ldl_le_p(rec + 20);
    s->cs_code_addr = ldl_le_p(rec + 28);
    s->cs_uniforms_addr = ldl_le_p(rec + 32);
    trace_bcm2835_v3d_shader_rec(s->shader_flags,
                                 s->fs_code_addr, s->fs_uniforms_addr,
                                 s->vs_code_addr, s->vs_uniforms_addr,
                                 s->cs_code_addr, s->cs_uniforms_addr);

    for (i = 0; i < s->attr_count; i++) {
        const uint8_t *a = rec + 36 + i * 8;

        s->attrs[i].addr = ldl_le_p(a);
        s->attrs[i].size = a[4] + 1;
        s->attrs[i].stride = a[5];
        if (s->shader_record_extended)
            s->attrs[i].stride |= ldl_le_p(rec + 100 + i * 4) & ~0xffu;
        trace_bcm2835_v3d_shader_attr(i, s->attrs[i].addr, s->attrs[i].size,
                                      s->attrs[i].stride);
    }
    s->shader_record_valid = true;
}

/*
 * The tile buffer and the tile load/store (TLB) path.
 *
 * The VC4 renders into an on-chip buffer one 64x64 tile at a time and the
 * render control list moves that buffer to and from memory.  A store also
 * clears the buffer for the next tile unless the packet disables it, which is
 * how a clear reaches the framebuffer: the driver sets the clear colour, does a
 * store in "none" mode to trigger the clear, then stores the cleared buffer for
 * every tile.
 *
 * Only 32-bit colour is handled.  The 565 formats and T tiling are declined
 * out loud rather than guessed at, since a wrong channel order or tile order
 * would be a silently incorrect image.
 */

/* The two packet families encode the colour format differently, so normalise
 * to one enum before acting on it. */
typedef enum V3DPixelFormat {
    V3D_FMT_RGBA8888,
    V3D_FMT_BGR565,
} V3DPixelFormat;

static V3DPixelFormat v3d_loadstore_format(uint32_t bits)
{
    return VC4_LOADSTORE_FORMAT(bits) == VC4_LOADSTORE_TILE_BUFFER_RGBA8888
         ? V3D_FMT_RGBA8888 : V3D_FMT_BGR565;
}

static V3DPixelFormat v3d_render_format(uint32_t bits)
{
    return VC4_RENDER_CONFIG_FORMAT(bits) == VC4_RENDER_CONFIG_FORMAT_RGBA8888
         ? V3D_FMT_RGBA8888 : V3D_FMT_BGR565;
}

static void v3d_tile_clear(BCM2835V3DState *s)
{
    unsigned i;

    /* The two clear colour words are the halves of the 64-bit tile buffer
     * word; for a 32-bit format the driver sets both to the same pixel. */
    for (i = 0; i < BCM2835_V3D_TILE_PIXELS; i++) {
        s->tile_color[i] = s->clear_color[0];
    }
}

/*
 * Move the colour tile buffer to or from memory.  The tile's position comes
 * from the last TILE_COORDINATES packet and its extent is clipped to the
 * framebuffer the render config declared.
 */
static void v3d_tlb_transfer(BCM2835V3DState *s, uint32_t addr, bool store,
                             V3DPixelFormat format, unsigned tiling)
{
    const unsigned cpp = 4;
    unsigned aligned_width;
    uint32_t px, py;
    uint64_t pixels = 0;

    if (format != V3D_FMT_RGBA8888) {
        qemu_log_mask(LOG_UNIMP,
                      "bcm2835-v3d: tile buffer format %u not implemented\n",
                      (unsigned)format);
        return;
    }
    if (!vc4_tiling_supported(tiling)) {
        qemu_log_mask(LOG_UNIMP,
                      "bcm2835-v3d: tile memory layout %u not implemented\n",
                      tiling);
        return;
    }
    if (!s->render_width || !s->render_height) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bcm2835-v3d: tile store with no render config\n");
        s->cl_errors++;
        return;
    }

    aligned_width = vc4_aligned_width(tiling, cpp, s->render_width);

    for (py = 0; py < BCM2835_V3D_TILE_H; py++) {
        uint32_t y = s->tile_y * BCM2835_V3D_TILE_H + py;

        if (y >= s->render_height) {
            break;
        }
        for (px = 0; px < BCM2835_V3D_TILE_W; px++) {
            uint32_t x = s->tile_x * BCM2835_V3D_TILE_W + px;
            uint32_t *pixel = &s->tile_color[py * BCM2835_V3D_TILE_W + px];
            uint8_t buf[4];
            uint32_t offset;

            if (x >= s->render_width) {
                break;
            }
            if (!vc4_tiling_offset(tiling, cpp, aligned_width, x, y,
                                   &offset)) {
                return;
            }

            if (store) {
                stl_le_p(buf, *pixel);
                if (!bcm2835_v3d_dma_write(s, addr + offset, buf, cpp)) {
                    return;
                }
            } else {
                if (!bcm2835_v3d_dma_read(s, addr + offset, buf, cpp)) {
                    return;
                }
                *pixel = ldl_le_p(buf);
            }
            pixels++;
        }
    }

    s->tlb_pixels += pixels;
    if (store) {
        s->tlb_stores++;
    } else {
        s->tlb_loads++;
    }
    trace_bcm2835_v3d_tlb_transfer(store ? "store" : "load", addr, s->tile_x,
                                   s->tile_y, pixels, tiling);
}

/*
 * Shader decode probe.
 *
 * The QPU interpreter is in place but the pipeline around it -- the VPM that
 * feeds the coordinate and vertex shaders, the TMU, the tile buffer -- is not,
 * so shaders cannot yet be run *as part of* the pipeline.  What the model does
 * instead, when the bin list points at a shader record, is decode and execute
 * each of the three programs on their own: the uniform stream is served from
 * guest memory (as the hardware streams it), writes to registers outside the
 * shader core are counted and dropped, and anything the interpreter does not
 * implement stops that program and is named in the trace.
 *
 * This produces no rendering.  It validates the decoder against the shaders
 * the guest actually submitted, and says exactly which feature the next phase
 * has to supply.
 */
#define V3D_SHADER_INST_LIMIT 4096
/* Cap the coordinate-shader runs one job can ask for (one per 16 vertices). */
#define V3D_MAX_SHADER_BATCHES 64

typedef struct V3DShaderProbe {
    BCM2835V3DState *s;
    uint32_t uniform_addr;
    unsigned uniform_reads;
    unsigned special_writes;
} V3DShaderProbe;

static bool v3d_shader_fetch(void *opaque, uint32_t addr, uint64_t *inst)
{
    V3DShaderProbe *p = opaque;
    uint8_t buf[8];

    if (!bcm2835_v3d_dma_read(p->s, addr, buf, sizeof(buf))) {
        return false;
    }
    *inst = ldq_le_p(buf);
    return true;
}

/* Special registers the environment serves.  The VPM ones are real; the rest
 * are counted and dropped. */
#define V3D_QPU_RADDR_VPM       48
#define V3D_QPU_WADDR_VPM       48
#define V3D_QPU_WADDR_VPM_SETUP 49
#define V3D_QPU_WADDR_VPM_ADDR  50

/*
 * The uniform FIFO streams 32-bit words from the address in the shader record,
 * one per read, broadcast to every lane.  A VPM read hands each lane one dword
 * of its own vertex.
 */
static bool v3d_shader_read_special(void *opaque, unsigned raddr, bool file_b,
                                    uint32_t lanes[VC4_QPU_LANES])
{
    V3DShaderProbe *p = opaque;
    uint32_t value;
    unsigned i;

    if (raddr == V3D_QPU_RADDR_VPM) {
        return vc4_vpm_read(&p->s->vpm, lanes);
    }
    if (raddr != VC4_QPU_RADDR_UNIF) {
        return false;
    }
    if (!v3d_read_u32(p->s, p->uniform_addr + p->uniform_reads * 4, &value)) {
        return false;
    }
    p->uniform_reads++;
    for (i = 0; i < VC4_QPU_LANES; i++) {
        lanes[i] = value;
    }
    return true;
}

static void v3d_shader_write_special(void *opaque, unsigned waddr, bool file_b,
                                     const uint32_t lanes[VC4_QPU_LANES],
                                     uint16_t mask)
{
    V3DShaderProbe *p = opaque;

    switch (waddr) {
    case V3D_QPU_WADDR_VPM:
        vc4_vpm_write(&p->s->vpm, lanes);
        break;

    case V3D_QPU_WADDR_VPM_SETUP:
        /* Regfile A configures reads, regfile B configures writes. */
        if (file_b) {
            vc4_vpm_setup_write(&p->s->vpm, lanes[0]);
        } else {
            vc4_vpm_setup_read(&p->s->vpm, lanes[0]);
        }
        break;

    case V3D_QPU_WADDR_VPM_ADDR:
        /* The VPM DMA trigger.  No shader the guest can submit reaches this:
         * the kernel's validator rejects it outright ("General VPM DMA
         * unsupported"), which is why the DMA half of the block is not
         * modelled at all. */
        qemu_log_mask(LOG_UNIMP, "bcm2835-v3d: VPM DMA is not modelled\n");
        break;

    default:
        /* TMU and tile-buffer writes land here.  Counting them is all this
         * phase can do with them. */
        break;
    }
    p->special_writes++;
}

static const V3DQPUOps v3d_shader_ops = {
    .fetch = v3d_shader_fetch,
    .read_special = v3d_shader_read_special,
    .write_special = v3d_shader_write_special,
    /* No signal_load: a shader that waits on a TMU or colour load stops, and
     * the trace names it. */
};

static void v3d_probe_shader(BCM2835V3DState *s, const char *which,
                             uint32_t code, uint32_t uniforms)
{
    V3DShaderProbe probe = {
        .s = s,
        .uniform_addr = uniforms,
    };
    V3DQPUState q;
    V3DQPUResult r;

    if (!code) {
        return;
    }

    vc4_qpu_init(&q, &v3d_shader_ops, &probe);
    r = vc4_qpu_run(&q, code, V3D_SHADER_INST_LIMIT);

    s->shader_probes++;
    s->shader_instructions += q.instructions;
    if (r != V3D_QPU_DONE) {
        s->shader_incomplete++;
    }
    trace_bcm2835_v3d_shader_probe(which, code, q.instructions,
                                   probe.uniform_reads, probe.special_writes,
                                   vc4_qpu_result_name(r),
                                   q.unsupported ? q.unsupported : "-");
}

/*
 * Fetch one vertex through the VCD: every attribute of the current shader
 * record contributes size bytes at addr + index * stride.  A stride of zero
 * means every vertex reads the same element (which is why the validator skips
 * its per-index bounds check in that case), so the address is used as is.
 *
 * The data is discarded -- there is no vertex shader to feed yet -- but the
 * reads are real, so they hit exactly the guest memory the hardware would
 * touch (and fault the same way if the driver programmed something bogus).
 */
static void v3d_vcd_fetch_vertex(V3DCLWalker *w, uint32_t index, unsigned lane)
{
    BCM2835V3DState *s = w->s;
    uint8_t buf[256];
    unsigned row = 0;
    unsigned i;

    for (i = 0; i < s->attr_count; i++) {
        const V3DVertexAttr *a = &s->attrs[i];
        unsigned off;

        if (w->dma_left < a->size) {
            v3d_walk_stop(w, "vertex fetch budget exhausted", false);
            return;
        }
        if (!bcm2835_v3d_dma_read(s, a->addr + index * a->stride, buf,
                                  a->size)) {
            v3d_walk_stop(w, "vertex fetch faulted", true);
            return;
        }
        w->dma_left -= a->size;
        s->vcd_reads++;
        s->vcd_bytes += a->size;

        /* Lay the vertex out for the shaders: row r, lane l is dword r of
         * vertex l, so one VPM read hands all 16 vertices in flight the same
         * dword of their own attribute data. */
        for (off = 0; off + 4 <= a->size; off += 4) {
            vc4_vpm_load_vertex(&s->vpm, row++, lane, ldl_le_p(buf + off));
        }
    }
    if (row > s->vpm_rows) {
        s->vpm_rows = row;
    }
}

/*
 * Read back what the coordinate shader produced.
 *
 * Mesa's coordinate shader writes, per vertex, the four clip coordinates and
 * then the screen position as two 12.4 fixed-point halves of one word, Zs, and
 * 1/Wc (vc4_program.c:emit_coord_end).  With its VPM write setup those land in
 * rows 0-3 and then 4, 5 and 6 -- so the screen position is row 4, packed X in
 * the low half by 16A and Y in the high half by 16B.
 */
#define V3D_CS_OUT_XY_ROW   4
#define V3D_CS_OUT_ZS_ROW   5
#define V3D_CS_OUT_RCPW_ROW 6

static float v3d_f32(uint32_t bits)
{
    float f;

    memcpy(&f, &bits, sizeof(f));
    return f;
}

static void v3d_collect_shaded(BCM2835V3DState *s, unsigned lanes)
{
    unsigned i;

    if (!s->shader_record_valid || s->vpm.declined) {
        return;
    }

    for (i = 0; i < lanes; i++) {
        uint32_t xy = s->vpm.mem[V3D_CS_OUT_XY_ROW][i];
        V3DShadedVertex v;

        if (s->shaded_count >= BCM2835_V3D_MAX_SHADED) {
            s->shaded_dropped++;
            return;
        }
        /* 12.4 fixed point, relative to the viewport centre the
         * VIEWPORT_OFFSET packet supplies. */
        int32_t x16 = (int16_t)(xy & 0xffff) + s->viewport_x16;
        int32_t y16 = (int16_t)(xy >> 16) + s->viewport_y16;
        uint32_t zbits = s->vpm.mem[V3D_CS_OUT_ZS_ROW][i];
        uint32_t wbits = s->vpm.mem[V3D_CS_OUT_RCPW_ROW][i];

        v.x = (float)x16 / 16.0f;
        v.y = (float)y16 / 16.0f;
        v.z = v3d_f32(zbits);
        v.rcp_w = v3d_f32(wbits);

        trace_bcm2835_v3d_shaded_vertex(s->shaded_count, x16, y16, zbits,
                                        wbits);
        s->shaded[s->shaded_count++] = v;
    }
}

/*
 * A batch of up to 16 vertices is now sitting in the VPM, which is when the
 * binner runs the coordinate shader over it.  (The vertex and fragment shaders
 * are decoded at shader-record time instead -- there is nowhere faithful to run
 * them until primitive setup and the rasterizer exist.)
 */
static void v3d_vcd_flush_batch(V3DCLWalker *w, unsigned lanes)
{
    BCM2835V3DState *s = w->s;

    if (!lanes) {
        return;
    }
    trace_bcm2835_v3d_vpm_load(s->vpm_rows, lanes);
    s->vpm_batches++;

    /* Bound the work a single job can ask for. */
    if (s->vpm_batches <= V3D_MAX_SHADER_BATCHES) {
        v3d_probe_shader(s, "cs", s->cs_code_addr, s->cs_uniforms_addr);
        v3d_collect_shaded(s, lanes);
    }

    vc4_vpm_reset(&s->vpm);
    s->vpm_rows = 0;
}

/* Clamp a primitive's vertex count to the per-job limit, tracing if it bites. */
static uint32_t v3d_clamp_count(V3DCLWalker *w, uint32_t count)
{
    if (count > V3D_MAX_VERTICES) {
        trace_bcm2835_v3d_clamp(w->pc, count, V3D_MAX_VERTICES);
        return V3D_MAX_VERTICES;
    }
    return count;
}

/* VC4_PACKET_GL_ARRAY_PRIMITIVE: vertices [base, base + count). */
static void v3d_fetch_array(V3DCLWalker *w, uint32_t base, uint32_t count)
{
    BCM2835V3DState *s = w->s;
    uint32_t fetched = 0;

    if (!s->shader_record_valid) {
        v3d_walk_stop(w, "primitive without shader state", true);
        return;
    }

    count = v3d_clamp_count(w, count);
    while (fetched < count && !w->stop) {
        v3d_vcd_fetch_vertex(w, base + fetched, fetched % VC4_VPM_LANES);
        fetched++;
        if (fetched % VC4_VPM_LANES == 0) {
            v3d_vcd_flush_batch(w, VC4_VPM_LANES);
        }
    }
    v3d_vcd_flush_batch(w, fetched % VC4_VPM_LANES);
    trace_bcm2835_v3d_vcd_fetch(fetched, s->attr_count);
}

/*
 * VC4_PACKET_GL_INDEXED_PRIMITIVE: read the index buffer the validator
 * relocated for us, then fetch the vertex each index selects -- the same
 * order the VCD walks it in.
 */
static void v3d_fetch_indexed(V3DCLWalker *w, uint32_t addr, uint32_t count,
                              unsigned index_size)
{
    BCM2835V3DState *s = w->s;
    uint8_t indices[512];
    uint32_t done = 0;

    if (!s->shader_record_valid) {
        v3d_walk_stop(w, "primitive without shader state", true);
        return;
    }

    count = v3d_clamp_count(w, count);
    while (done < count && !w->stop) {
        uint32_t chunk = MIN(count - done, sizeof(indices) / index_size);
        uint32_t bytes = chunk * index_size;
        uint32_t i;

        if (w->dma_left < bytes) {
            v3d_walk_stop(w, "index fetch budget exhausted", false);
            break;
        }
        if (!bcm2835_v3d_dma_read(s, addr + done * index_size, indices,
                                  bytes)) {
            v3d_walk_stop(w, "index fetch faulted", true);
            break;
        }
        w->dma_left -= bytes;
        s->ib_reads++;
        s->ib_bytes += bytes;

        for (i = 0; i < chunk && !w->stop; i++) {
            uint32_t index = index_size == 2 ? lduw_le_p(indices + i * 2)
                                             : indices[i];
            v3d_vcd_fetch_vertex(w, index, (done + i) % VC4_VPM_LANES);
            if ((done + i + 1) % VC4_VPM_LANES == 0) {
                v3d_vcd_flush_batch(w, VC4_VPM_LANES);
            }
        }
        done += chunk;
    }
    v3d_vcd_flush_batch(w, done % VC4_VPM_LANES);
    trace_bcm2835_v3d_vcd_fetch(done, s->attr_count);
}

/*
 * Cap every per-tile list with a RETURN_FROM_SUB_LIST.
 *
 * This is what the real binner does when it reaches VC4_PACKET_FLUSH: each
 * tile's list in the tile allocation area is terminated with a return, so the
 * render list's BRANCH_TO_SUB_LIST per tile comes back.  Without primitives
 * (no rasterizer yet) the lists are otherwise empty, but writing the return
 * byte is what lets the render control list be walked to its end.
 */
static void v3d_bin_cap_tile_lists(V3DCLWalker *w)
{
    BCM2835V3DState *s = w->s;
    const uint8_t ret = VC4_PACKET_RETURN_FROM_SUB_LIST;
    uint32_t block, tiles, i;

    if (!s->bin_config_valid)
        return;

    block = 32u << ((s->bin_flags >> VC4_BIN_CONFIG_INIT_BLOCK_SHIFT) &
                    VC4_BIN_CONFIG_INIT_BLOCK_MASK);
    tiles = s->bin_tiles_x * s->bin_tiles_y;
    if ((uint64_t)tiles * block > s->tile_alloc_size) {
        /* The hardware would run the binner out of memory here (INT_OUTOMEM);
         * cap only the lists that fit and leave the rest alone. */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bcm2835-v3d: tile alloc area 0x%x too small for %u "
                      "tiles of %u bytes\n", s->tile_alloc_size, tiles, block);
        s->cl_errors++;
        tiles = s->tile_alloc_size / block;
    }

    trace_bcm2835_v3d_tile_lists(tiles, s->tile_alloc_addr, block);
    for (i = 0; i < tiles; i++) {
        if (!bcm2835_v3d_dma_write(s, s->tile_alloc_addr + i * block, &ret,
                                   sizeof(ret))) {
            v3d_walk_stop(w, "tile list write faulted", true);
            return;
        }
    }
}

/*
 * Rasterization.
 *
 * A real binner sorts primitives into per-tile lists in the tile allocation
 * area, and the render list branches into a tile's list to draw it.  This model
 * keeps the shaded vertices in its own state instead and rasterizes them when
 * the render list branches to a tile -- the same point in the pipeline, without
 * having to reproduce the binner's internal list format.  Everything visible to
 * the guest (which pixels end up in the framebuffer) is the same.
 *
 * Coverage is the usual edge-function test at pixel centres, accepting either
 * winding: culling is configured by VC4_PACKET_CONFIGURATION_BITS, which the
 * model does not act on yet.  Fragments are shaded sixteen at a time by running
 * the fragment shader on the QPU and taking whatever it writes to the tile
 * buffer colour register.
 */
#define V3D_QPU_WADDR_TLB_Z         44
#define V3D_QPU_WADDR_TLB_COLOR_MS  45
#define V3D_QPU_WADDR_TLB_COLOR_ALL 46

typedef struct V3DFragmentRun {
    BCM2835V3DState *s;
    uint32_t uniform_addr;
    unsigned uniform_reads;
    uint32_t color[VC4_QPU_LANES];
    uint16_t color_mask;
} V3DFragmentRun;

static bool v3d_frag_fetch(void *opaque, uint32_t addr, uint64_t *inst)
{
    V3DFragmentRun *f = opaque;
    uint8_t buf[8];

    if (!bcm2835_v3d_dma_read(f->s, addr, buf, sizeof(buf))) {
        return false;
    }
    *inst = ldq_le_p(buf);
    return true;
}

static bool v3d_frag_read_special(void *opaque, unsigned raddr, bool file_b,
                                  uint32_t lanes[VC4_QPU_LANES])
{
    V3DFragmentRun *f = opaque;
    uint32_t value;
    unsigned i;

    if (raddr != VC4_QPU_RADDR_UNIF) {
        return false;   /* varyings and the rest are not modelled yet */
    }
    if (!v3d_read_u32(f->s, f->uniform_addr + f->uniform_reads * 4, &value)) {
        return false;
    }
    f->uniform_reads++;
    for (i = 0; i < VC4_QPU_LANES; i++) {
        lanes[i] = value;
    }
    return true;
}

static void v3d_frag_write_special(void *opaque, unsigned waddr, bool file_b,
                                   const uint32_t lanes[VC4_QPU_LANES],
                                   uint16_t mask)
{
    V3DFragmentRun *f = opaque;
    unsigned i;

    if (waddr != V3D_QPU_WADDR_TLB_COLOR_ALL &&
        waddr != V3D_QPU_WADDR_TLB_COLOR_MS) {
        return;         /* depth and the alpha mask are not modelled yet */
    }
    for (i = 0; i < VC4_QPU_LANES; i++) {
        if (mask & (1u << i)) {
            f->color[i] = lanes[i];
        }
    }
    f->color_mask |= mask;
}

static const V3DQPUOps v3d_frag_ops = {
    .fetch = v3d_frag_fetch,
    .read_special = v3d_frag_read_special,
    .write_special = v3d_frag_write_special,
};

/* Shade one group of up to 16 covered pixels and put them in the tile. */
static void v3d_shade_group(BCM2835V3DState *s, const uint16_t *px,
                            const uint16_t *py, unsigned count)
{
    V3DFragmentRun run = {
        .s = s,
        .uniform_addr = s->fs_uniforms_addr,
    };
    V3DQPUState q;
    V3DQPUResult r;
    unsigned i;

    if (!count) {
        return;
    }

    vc4_qpu_init(&q, &v3d_frag_ops, &run);
    r = vc4_qpu_run(&q, s->fs_code_addr, V3D_SHADER_INST_LIMIT);
    s->shader_probes++;
    s->shader_instructions += q.instructions;
    if (r != V3D_QPU_DONE) {
        s->shader_incomplete++;
        if (!s->frag_reported) {
            s->frag_reported = true;
            trace_bcm2835_v3d_shader_probe("fs (fragment)", s->fs_code_addr,
                                           q.instructions, run.uniform_reads,
                                           0, vc4_qpu_result_name(r),
                                           q.unsupported ? q.unsupported : "-");
        }
        return;
    }

    for (i = 0; i < count; i++) {
        if (run.color_mask & (1u << i)) {
            s->tile_color[py[i] * BCM2835_V3D_TILE_W + px[i]] = run.color[i];
            s->frag_shaded++;
        }
    }
}

/* Edge function: positive on one side of the line ab, negative on the other. */
static float v3d_edge(const V3DShadedVertex *a, const V3DShadedVertex *b,
                      float x, float y)
{
    return (b->x - a->x) * (y - a->y) - (b->y - a->y) * (x - a->x);
}

static void v3d_raster_triangle(BCM2835V3DState *s, const V3DShadedVertex *a,
                                const V3DShadedVertex *b,
                                const V3DShadedVertex *c)
{
    uint16_t gx[VC4_QPU_LANES], gy[VC4_QPU_LANES];
    unsigned group = 0;
    float minx, maxx, miny, maxy;
    int x0, x1, y0, y1, x, y;
    int tile_x0 = s->tile_x * BCM2835_V3D_TILE_W;
    int tile_y0 = s->tile_y * BCM2835_V3D_TILE_H;

    minx = MIN(a->x, MIN(b->x, c->x));
    maxx = MAX(a->x, MAX(b->x, c->x));
    miny = MIN(a->y, MIN(b->y, c->y));
    maxy = MAX(a->y, MAX(b->y, c->y));

    /* Clip the bounding box to this tile and to the framebuffer. */
    x0 = MAX((int)floorf(minx), tile_x0);
    x1 = MIN((int)ceilf(maxx), MIN(tile_x0 + BCM2835_V3D_TILE_W,
                                   (int)s->render_width));
    y0 = MAX((int)floorf(miny), tile_y0);
    y1 = MIN((int)ceilf(maxy), MIN(tile_y0 + BCM2835_V3D_TILE_H,
                                   (int)s->render_height));

    for (y = y0; y < y1; y++) {
        for (x = x0; x < x1; x++) {
            float cx = (float)x + 0.5f;
            float cy = (float)y + 0.5f;
            float w0 = v3d_edge(b, c, cx, cy);
            float w1 = v3d_edge(c, a, cx, cy);
            float w2 = v3d_edge(a, b, cx, cy);

            /* Accept either winding; culling is not modelled. */
            if (!((w0 >= 0.0f && w1 >= 0.0f && w2 >= 0.0f) ||
                  (w0 <= 0.0f && w1 <= 0.0f && w2 <= 0.0f))) {
                continue;
            }

            gx[group] = x - tile_x0;
            gy[group] = y - tile_y0;
            if (++group == VC4_QPU_LANES) {
                v3d_shade_group(s, gx, gy, group);
                group = 0;
            }
        }
    }
    v3d_shade_group(s, gx, gy, group);
}

/*
 * Draw everything the coordinate shader produced into the current tile.  The
 * primitive mode says how the vertex stream forms triangles.
 */
static void v3d_raster_tile(BCM2835V3DState *s)
{
    uint32_t i;
    uint32_t before = s->frag_shaded;

    if (!s->shaded_count || !s->fs_code_addr || !s->render_width) {
        return;
    }

    switch (s->prim_mode) {
    case 4:     /* triangles */
        for (i = 0; i + 2 < s->shaded_count; i += 3) {
            v3d_raster_triangle(s, &s->shaded[i], &s->shaded[i + 1],
                                &s->shaded[i + 2]);
        }
        break;
    case 5:     /* triangle strip */
        for (i = 0; i + 2 < s->shaded_count; i++) {
            v3d_raster_triangle(s, &s->shaded[i], &s->shaded[i + 1],
                                &s->shaded[i + 2]);
        }
        break;
    case 6:     /* triangle fan */
        for (i = 1; i + 1 < s->shaded_count; i++) {
            v3d_raster_triangle(s, &s->shaded[0], &s->shaded[i],
                                &s->shaded[i + 1]);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "bcm2835-v3d: primitive mode %u not rasterized\n",
                      s->prim_mode);
        return;
    }

    trace_bcm2835_v3d_raster(s->tile_x, s->tile_y, s->shaded_count,
                             s->frag_shaded - before);
}

/* Packets that mean the same thing on both threads: control flow. */
static bool v3d_dispatch_common(V3DCLWalker *w, uint8_t op)
{
    BCM2835V3DState *s = w->s;
    uint32_t target;

    switch (op) {
    case VC4_PACKET_HALT:
        v3d_walk_stop(w, "halt", false);
        return true;

    case VC4_PACKET_NOP:
        return true;

    case VC4_PACKET_BRANCH:
        if (!v3d_read_u32(s, w->pc + 1, &target)) {
            v3d_walk_stop(w, "branch target unreadable", true);
            return true;
        }
        trace_bcm2835_v3d_cl_branch(w->thread, w->pc, target, "branch");
        w->next = target;
        return true;

    case VC4_PACKET_BRANCH_TO_SUB_LIST:
        if (!v3d_read_u32(s, w->pc + 1, &target)) {
            v3d_walk_stop(w, "sub-list target unreadable", true);
            return true;
        }
        if (w->ret_depth == V3D_MAX_SUBLIST_DEPTH) {
            v3d_walk_stop(w, "sub-list nesting too deep", true);
            return true;
        }
        trace_bcm2835_v3d_cl_branch(w->thread, w->pc, target, "sub-list");
        /* This is where the hardware would run the tile's primitive list.
         * The model keeps the primitives in its own state instead. */
        if (w->thread == 1) {
            v3d_raster_tile(s);
        }
        w->ret_stack[w->ret_depth++] = w->next;
        w->next = target;
        return true;

    case VC4_PACKET_RETURN_FROM_SUB_LIST:
        if (w->ret_depth == 0) {
            v3d_walk_stop(w, "return outside a sub-list", true);
            return true;
        }
        w->next = w->ret_stack[--w->ret_depth];
        trace_bcm2835_v3d_cl_return(w->thread, w->pc, w->next);
        return true;

    default:
        return false;
    }
}

/* CT0, the bin control list: this is where the driver describes the draw. */
static void v3d_dispatch_bin(V3DCLWalker *w, uint8_t op)
{
    BCM2835V3DState *s = w->s;
    uint8_t payload[15];
    uint32_t shader_state;

    switch (op) {
    case VC4_PACKET_TILE_BINNING_MODE_CONFIG:
        /* In the executed list the validator has replaced the first three
         * words with absolute addresses; tiles_x/y and the flags byte are
         * copied through from userspace. */
        if (!bcm2835_v3d_dma_read(s, w->pc + 1, payload, sizeof(payload))) {
            v3d_walk_stop(w, "bin config unreadable", true);
            return;
        }
        s->tile_alloc_addr = ldl_le_p(payload + 0);
        s->tile_alloc_size = ldl_le_p(payload + 4);
        s->tile_state_addr = ldl_le_p(payload + 8);
        s->bin_tiles_x = payload[12];
        s->bin_tiles_y = payload[13];
        s->bin_flags = payload[14];
        s->bin_config_valid = s->bin_tiles_x && s->bin_tiles_y;
        trace_bcm2835_v3d_bin_config(s->tile_alloc_addr, s->tile_alloc_size,
                                     s->tile_state_addr, s->bin_tiles_x,
                                     s->bin_tiles_y, s->bin_flags);
        if (!s->bin_config_valid)
            v3d_walk_stop(w, "empty tile binning config", true);
        break;

    case VC4_PACKET_GL_SHADER_STATE:
        /* The low four bits of the (16-byte aligned) shader record address
         * carry the attribute count and the extended-record flag. */
        if (!v3d_read_u32(s, w->pc + 1, &shader_state)) {
            v3d_walk_stop(w, "shader state unreadable", true);
            return;
        }
        s->shader_record_addr = shader_state & ~0xfu;
        s->attr_count = (shader_state & 7) ? (shader_state & 7) : 8;
        s->shader_record_extended = !!(shader_state & 8);
        trace_bcm2835_v3d_shader_state(s->shader_record_addr, s->attr_count,
                                       s->shader_record_extended ?
                                       " (extended)" : "");
        v3d_parse_shader_record(w);
        if (s->shader_record_valid) {
            /* The coordinate shader runs per batch of vertices once the VPM
             * holds their data (see v3d_vcd_flush_batch).  These two have
             * nowhere faithful to run until primitive setup and the
             * rasterizer exist, so they are only decoded. */
            v3d_probe_shader(s, "vs", s->vs_code_addr, s->vs_uniforms_addr);
            v3d_probe_shader(s, "fs", s->fs_code_addr, s->fs_uniforms_addr);
        }
        break;

    case VC4_PACKET_GL_ARRAY_PRIMITIVE:
        if (!bcm2835_v3d_dma_read(s, w->pc + 1, payload, 9)) {
            v3d_walk_stop(w, "array primitive unreadable", true);
            return;
        }
        s->bin_prims++;
        s->prim_mode = payload[0] & 0xf;
        trace_bcm2835_v3d_array_prim(payload[0], ldl_le_p(payload + 1),
                                     ldl_le_p(payload + 5));
        v3d_fetch_array(w, ldl_le_p(payload + 5), ldl_le_p(payload + 1));
        break;

    case VC4_PACKET_GL_INDEXED_PRIMITIVE: {
        unsigned index_size;

        if (!bcm2835_v3d_dma_read(s, w->pc + 1, payload, 13)) {
            v3d_walk_stop(w, "indexed primitive unreadable", true);
            return;
        }
        /* Bit 4 of the mode byte selects u8 or u16 indices. */
        index_size = (payload[0] & (1u << 4)) ? 2 : 1;
        s->bin_prims++;
        s->prim_mode = payload[0] & 0xf;
        trace_bcm2835_v3d_indexed_prim(payload[0], ldl_le_p(payload + 1),
                                       ldl_le_p(payload + 5),
                                       ldl_le_p(payload + 9), index_size);
        v3d_fetch_indexed(w, ldl_le_p(payload + 5), ldl_le_p(payload + 1),
                          index_size);
        break;
    }

    case VC4_PACKET_VIEWPORT_OFFSET:
        /* Two signed 12.4 fixed-point values: the screen position the shaded
         * coordinates are relative to. */
        if (!bcm2835_v3d_dma_read(s, w->pc + 1, payload, 4)) {
            v3d_walk_stop(w, "viewport offset unreadable", true);
            return;
        }
        s->viewport_x16 = (int16_t)lduw_le_p(payload);
        s->viewport_y16 = (int16_t)lduw_le_p(payload + 2);
        trace_bcm2835_v3d_viewport(s->viewport_x16, s->viewport_y16);
        break;

    case VC4_PACKET_FLUSH:
    case VC4_PACKET_FLUSH_ALL:
        /* End of the bin list: terminate the per-tile lists the render list
         * is about to branch into. */
        v3d_bin_cap_tile_lists(w);
        break;

    default:
        /* Everything else is state the rasterizer will need (clip window,
         * viewport, configuration bits, ...) or is structural.  Trace it and
         * skip by size. */
        break;
    }
}

/* CT1, the render control list.  The kernel generates this one itself (see
 * vc4_render_cl.c), so the layouts here are exactly what it emits. */
static void v3d_dispatch_render(V3DCLWalker *w, uint8_t op)
{
    BCM2835V3DState *s = w->s;
    uint8_t payload[13];
    uint16_t bits;
    uint32_t addr;

    switch (op) {
    case VC4_PACKET_TILE_RENDERING_MODE_CONFIG:
        if (!bcm2835_v3d_dma_read(s, w->pc + 1, payload, 10)) {
            v3d_walk_stop(w, "render config unreadable", true);
            return;
        }
        s->render_fb_addr = ldl_le_p(payload + 0);
        s->render_width = lduw_le_p(payload + 4);
        s->render_height = lduw_le_p(payload + 6);
        s->render_bits = lduw_le_p(payload + 8);
        trace_bcm2835_v3d_render_config(s->render_fb_addr, s->render_width,
                                        s->render_height, s->render_bits);
        break;

    case VC4_PACKET_TILE_COORDINATES:
        if (!bcm2835_v3d_dma_read(s, w->pc + 1, payload, 2)) {
            v3d_walk_stop(w, "tile coordinates unreadable", true);
            return;
        }
        s->tile_x = payload[0];
        s->tile_y = payload[1];
        trace_bcm2835_v3d_tile_coords(payload[0], payload[1]);
        break;

    case VC4_PACKET_CLEAR_COLORS:
        if (!bcm2835_v3d_dma_read(s, w->pc + 1, payload, 13)) {
            v3d_walk_stop(w, "clear colors unreadable", true);
            return;
        }
        s->clear_color[0] = ldl_le_p(payload + 0);
        s->clear_color[1] = ldl_le_p(payload + 4);
        s->clear_z = ldl_le_p(payload + 8);
        s->clear_stencil = payload[12];
        trace_bcm2835_v3d_clear_colors(s->clear_color[0], s->clear_color[1],
                                       s->clear_z, s->clear_stencil);
        break;

    case VC4_PACKET_STORE_TILE_BUFFER_GENERAL:
    case VC4_PACKET_LOAD_TILE_BUFFER_GENERAL:
        /* u16 of mode/format bits, then the address with the EOF and
         * disable flags in its low four bits. */
        if (!v3d_read_u16(s, w->pc + 1, &bits) ||
            !v3d_read_u32(s, w->pc + 3, &addr)) {
            v3d_walk_stop(w, "tile load/store unreadable", true);
            return;
        }
        if (op == VC4_PACKET_LOAD_TILE_BUFFER_GENERAL) {
            trace_bcm2835_v3d_tile_load(addr & ~0xfu, bits);
            if (VC4_LOADSTORE_BUFFER(bits) == VC4_LOADSTORE_BUFFER_COLOR) {
                v3d_tlb_transfer(s, addr & ~0xfu, false,
                                 v3d_loadstore_format(bits),
                                 VC4_LOADSTORE_TILING(bits));
            }
            break;
        }

        s->render_stores++;
        trace_bcm2835_v3d_tile_store(addr & ~0xfu, bits,
                                     addr & VC4_LOADSTORE_TILE_BUFFER_EOF ?
                                     " EOF" : "");
        if (VC4_LOADSTORE_BUFFER(bits) == VC4_LOADSTORE_BUFFER_COLOR) {
            v3d_tlb_transfer(s, addr & ~0xfu, true, v3d_loadstore_format(bits),
                             VC4_LOADSTORE_TILING(bits));
        }
        /* A store is also what clears the tile buffer for the next tile.
         * The driver relies on this: it triggers the clear with a store in
         * "none" mode before the first real tile. */
        if (!(bits & VC4_STORE_TILE_BUFFER_DISABLE_COLOR_CLEAR)) {
            v3d_tile_clear(s);
        }
        if (addr & VC4_LOADSTORE_TILE_BUFFER_EOF)
            v3d_walk_stop(w, "end of frame", false);
        break;

    case VC4_PACKET_STORE_FULL_RES_TILE_BUFFER:
    case VC4_PACKET_LOAD_FULL_RES_TILE_BUFFER:
        if (!v3d_read_u32(s, w->pc + 1, &addr)) {
            v3d_walk_stop(w, "full-res tile load/store unreadable", true);
            return;
        }
        if (op == VC4_PACKET_LOAD_FULL_RES_TILE_BUFFER) {
            trace_bcm2835_v3d_tile_load(addr & ~0xfu, 0);
        } else {
            s->render_stores++;
            trace_bcm2835_v3d_tile_store(addr & ~0xfu, 0, "");
        }
        break;

    case VC4_PACKET_STORE_MS_TILE_BUFFER:
    case VC4_PACKET_STORE_MS_TILE_BUFFER_AND_EOF:
        /* Writes the tile buffer to the surface the render config named. */
        s->render_stores++;
        trace_bcm2835_v3d_tile_store(s->render_fb_addr, s->render_bits,
                                     op == VC4_PACKET_STORE_MS_TILE_BUFFER_AND_EOF ?
                                     " EOF" : "");
        v3d_tlb_transfer(s, s->render_fb_addr, true,
                         v3d_render_format(s->render_bits),
                         VC4_RENDER_CONFIG_MEMORY_FORMAT(s->render_bits));
        v3d_tile_clear(s);
        if (op == VC4_PACKET_STORE_MS_TILE_BUFFER_AND_EOF)
            v3d_walk_stop(w, "end of frame", false);
        break;

    case VC4_PACKET_WAIT_ON_SEMAPHORE:
        /* Jobs run to completion in order here, so the binner has always
         * finished by the time the render list waits for it. */
        break;

    default:
        break;
    }
}

/*
 * Walk one thread's control list from CTnCA to CTnEA, following branches and
 * sub-lists.  Every read is bounds-checked and goes through the DMA helper,
 * and the packet count is capped, so a malformed list ends the walk instead
 * of spinning the bottom half or reading unbounded guest memory.
 */
static void v3d_run_cl(BCM2835V3DState *s, unsigned thread)
{
    V3DCLWalker w = {
        .s = s,
        .thread = thread,
        .ca = s->regs[V3D_CTNCA(thread) / 4],
        .ea = s->regs[V3D_CTNEA(thread) / 4],
        .dma_left = V3D_JOB_DMA_BUDGET,
    };

    w.pc = w.ca;
    /* Shader state does not carry over between jobs: the bin list of every
     * job programs its own, and the render list (which the B5 probe runs
     * after) must see this job's record, not the previous one's. */
    if (thread == 0) {
        s->shader_record_valid = false;
        s->attr_count = 0;
        vc4_vpm_reset(&s->vpm);
        s->vpm_rows = 0;
        s->vpm_batches = 0;
        s->shaded_count = 0;
        s->shaded_dropped = 0;
        s->frag_reported = false;
    }
    trace_bcm2835_v3d_cl_begin(thread, thread ? "render" : "bin", w.ca, w.ea);

    while (!w.stop) {
        const V3DPacketInfo *info;
        uint8_t op;

        if (w.packets++ >= V3D_MAX_PACKETS) {
            v3d_walk_stop(&w, "packet limit reached", true);
            break;
        }
        /* CTnEA bounds the main list only; a sub-list lives in the tile
         * allocation area and ends with a return (or a halt). */
        if (w.ret_depth == 0 && w.pc >= w.ea) {
            v3d_walk_stop(&w, "end of list", false);
            break;
        }
        if (!v3d_read_u8(s, w.pc, &op)) {
            v3d_walk_stop(&w, "opcode unreadable", true);
            break;
        }

        info = &v3d_packets[op];
        if (!info->size) {
            v3d_walk_stop(&w, "unknown packet", true);
            break;
        }
        if (w.ret_depth == 0 && (uint64_t)w.pc + info->size > w.ea) {
            v3d_walk_stop(&w, "packet crosses end of list", true);
            break;
        }

        trace_bcm2835_v3d_cl_packet(thread, w.pc, op, info->name);
        if (thread == 0) {
            s->bin_packets++;
        } else {
            s->render_packets++;
        }

        w.next = w.pc + info->size;
        if (!v3d_dispatch_common(&w, op)) {
            if (thread == 0) {
                v3d_dispatch_bin(&w, op);
            } else {
                v3d_dispatch_render(&w, op);
            }
        }
        w.pc = w.next;
    }

    if (w.failed) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bcm2835-v3d: CT%u control list aborted at 0x%08x "
                      "after %u packets: %s\n",
                      thread, w.pc, w.packets, w.stop);
    }
    trace_bcm2835_v3d_cl_end(thread, w.pc, w.packets, w.stop);
}

static bool bcm2835_v3d_shader_has_tmu(BCM2835V3DState *s, uint32_t addr)
{
    /* QPU instructions are 64-bit little-endian words.  This is only a
     * recognizer for the canned B5 shader, not an instruction interpreter. */
    for (unsigned i = 0; i < 256; i++) {
        uint8_t bytes[8];
        uint64_t inst;
        uint32_t sig;
        uint32_t waddr_add;
        uint32_t waddr_mul;

        if (!bcm2835_v3d_dma_read(s, addr + i * sizeof(inst), bytes,
                                  sizeof(bytes)))
            return false;
        inst = ldq_le_p(bytes);
        sig = (inst >> 60) & 0xf;
        waddr_add = (inst >> 38) & 0x3f;
        waddr_mul = (inst >> 32) & 0x3f;
        if ((waddr_add >= 56 && waddr_add <= 63) ||
            (waddr_mul >= 56 && waddr_mul <= 63))
            return true;
        if (sig == 3) /* QPU_SIG_PROG_END */
            break;
    }
    return false;
}

/* Minimal B5-only TMU side effect.  The kernel validator has already
 * relocated the FS uniform p0 to tex->dma_addr + p0.  Reading that address
 * through QEMU's system-memory AddressSpace makes the attempted fetch visible
 * (as a successful read or a guest-DMA fault) without claiming to emulate
 * tiling, filtering, rasterization, or the general QPU ISA. */
static void bcm2835_v3d_probe_b5_tmu(BCM2835V3DState *s)
{
    uint32_t p0;
    uint8_t sample[4];
    bool ok;

    if (!s->shader_record_valid ||
        !bcm2835_v3d_shader_has_tmu(s, s->fs_code_addr) ||
        !v3d_read_u32(s, s->fs_uniforms_addr, &p0))
        return;

    s->tmu_fetches++;
    ok = bcm2835_v3d_dma_read(s, p0, sample, sizeof(sample));
    if (!ok)
        s->tmu_faults++;
    qemu_log_mask(LOG_GUEST_ERROR,
                  "bcm2835-v3d: B5 TMU probe fetch #%u at 0x%08x (%s)\n",
                  s->tmu_fetches, p0, ok ? "read" : "fault");
}

/* Bottom half: run every pending control-list thread to completion. */
static void bcm2835_v3d_job_bh(void *opaque)
{
    BCM2835V3DState *s = opaque;
    uint32_t pending = s->job_pending;

    s->job_pending = 0;

    for (unsigned n = 0; n < 2; n++) {
        if (!(pending & (1u << n))) {
            continue;
        }

        v3d_run_cl(s, n);
        if (n == 1) {
            bcm2835_v3d_probe_b5_tmu(s);
        }

        /*
         * The model completes the whole list in one go (CA advanced to EA so
         * the driver's hangcheck is happy), drops the running flag, bumps the
         * frame counter, and latches the done interrupt.
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
    s->tile_alloc_addr = 0;
    s->tile_alloc_size = 0;
    s->tile_state_addr = 0;
    s->bin_tiles_x = 0;
    s->bin_tiles_y = 0;
    s->bin_flags = 0;
    s->bin_config_valid = false;
    s->shader_record_addr = 0;
    s->shader_record_valid = false;
    s->shader_record_extended = false;
    s->shader_flags = 0;
    s->fs_code_addr = 0;
    s->fs_uniforms_addr = 0;
    s->vs_code_addr = 0;
    s->vs_uniforms_addr = 0;
    s->cs_code_addr = 0;
    s->cs_uniforms_addr = 0;
    s->attr_count = 0;
    memset(s->attrs, 0, sizeof(s->attrs));
    s->render_fb_addr = 0;
    s->render_width = 0;
    s->render_height = 0;
    s->render_bits = 0;
    s->bin_packets = 0;
    s->render_packets = 0;
    s->bin_prims = 0;
    s->render_stores = 0;
    s->vcd_reads = 0;
    s->vcd_bytes = 0;
    s->ib_reads = 0;
    s->ib_bytes = 0;
    s->cl_errors = 0;
    s->shader_probes = 0;
    s->shader_incomplete = 0;
    s->shader_instructions = 0;
    vc4_vpm_reset(&s->vpm);
    s->vpm_rows = 0;
    s->vpm_batches = 0;
    s->shaded_count = 0;
    s->shaded_dropped = 0;
    s->viewport_x16 = 0;
    s->viewport_y16 = 0;
    s->prim_mode = 0;
    s->frag_shaded = 0;
    s->frag_reported = false;
    memset(s->tile_color, 0, sizeof(s->tile_color));
    s->clear_color[0] = 0;
    s->clear_color[1] = 0;
    s->clear_z = 0;
    s->clear_stencil = 0;
    s->tile_x = 0;
    s->tile_y = 0;
    s->tlb_stores = 0;
    s->tlb_loads = 0;
    s->tlb_pixels = 0;
    s->tmu_fetches = 0;
    s->tmu_faults = 0;
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
    s->system_memory = &address_space_memory;

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

static int bcm2835_v3d_post_load(void *opaque, int version_id)
{
    BCM2835V3DState *s = opaque;

    /* A job that had been kicked but not yet completed when the snapshot was
     * taken has to be picked up again, or its thread never finishes and the
     * driver's hangcheck fires. */
    if (s->job_pending) {
        qemu_bh_schedule(s->job_bh);
    }
    return 0;
}

static const VMStateDescription bcm2835_v3d_vmstate = {
    .name = "bcm2835_v3d",
    .version_id = 5,
    .minimum_version_id = 1,
    .post_load = bcm2835_v3d_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(int_status, BCM2835V3DState),
        VMSTATE_UINT32(int_enable, BCM2835V3DState),
        VMSTATE_UINT32(job_pending, BCM2835V3DState),
        VMSTATE_UINT32(shader_record_addr, BCM2835V3DState),
        VMSTATE_BOOL(shader_record_valid, BCM2835V3DState),
        VMSTATE_UINT32(tmu_fetches, BCM2835V3DState),
        VMSTATE_UINT32(tmu_faults, BCM2835V3DState),
        VMSTATE_UINT32_ARRAY(regs, BCM2835V3DState, BCM2835_V3D_NUM_REGS),
        /* Control-list interpreter state.  The parsed vertex attributes are
         * rebuilt by the next walk, so they are deliberately not migrated. */
        VMSTATE_UINT32_V(tile_alloc_addr, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(tile_alloc_size, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(tile_state_addr, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(bin_tiles_x, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(bin_tiles_y, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(bin_flags, BCM2835V3DState, 3),
        VMSTATE_BOOL_V(bin_config_valid, BCM2835V3DState, 3),
        VMSTATE_BOOL_V(shader_record_extended, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(shader_flags, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(fs_code_addr, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(fs_uniforms_addr, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(vs_code_addr, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(vs_uniforms_addr, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(cs_code_addr, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(cs_uniforms_addr, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(attr_count, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(render_fb_addr, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(render_width, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(render_height, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(render_bits, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(bin_packets, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(render_packets, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(bin_prims, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(render_stores, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(vcd_reads, BCM2835V3DState, 3),
        VMSTATE_UINT64_V(vcd_bytes, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(ib_reads, BCM2835V3DState, 3),
        VMSTATE_UINT64_V(ib_bytes, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(cl_errors, BCM2835V3DState, 3),
        VMSTATE_UINT32_V(shader_probes, BCM2835V3DState, 4),
        VMSTATE_UINT32_V(shader_incomplete, BCM2835V3DState, 4),
        VMSTATE_UINT64_V(shader_instructions, BCM2835V3DState, 4),
        VMSTATE_UINT32_ARRAY_V(clear_color, BCM2835V3DState, 2, 5),
        VMSTATE_UINT32_V(clear_z, BCM2835V3DState, 5),
        VMSTATE_UINT32_V(clear_stencil, BCM2835V3DState, 5),
        VMSTATE_UINT32_V(tile_x, BCM2835V3DState, 5),
        VMSTATE_UINT32_V(tile_y, BCM2835V3DState, 5),
        VMSTATE_UINT32_V(tlb_stores, BCM2835V3DState, 5),
        VMSTATE_UINT32_V(tlb_loads, BCM2835V3DState, 5),
        VMSTATE_UINT64_V(tlb_pixels, BCM2835V3DState, 5),
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
