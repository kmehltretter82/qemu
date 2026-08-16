/*
 * VideoCore IV VPM (Vertex Pipe Memory).
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The VPM is the scratch memory between the vertex-fetch hardware, the QPUs
 * and the primitive setup block: 64 rows of 16 lanes of 32 bits.  A coordinate
 * or vertex shader reads its attributes out of it and writes its outputs back,
 * one row at a time across all 16 lanes -- so a row read hands each of the 16
 * vertices in flight one dword.
 *
 * Access goes through two setup registers, written by the QPU to
 * QPU_W_VPMVCD_SETUP: regfile A configures reads, regfile B configures writes
 * (the "LD for regfile a, ST for regfile b" in the kernel's enum).  The field
 * layout is the one Mesa's vc4 driver emits in
 * src/gallium/drivers/vc4/vc4_qpu_emit.c, which writes
 *
 *     read setup:   addr | 0x1a00 | (num << 20)
 *     write setup:         0x1a00
 *
 * i.e. ADDR in [7:0], SIZE in [9:8] (2 = 32-bit), LANED at 10, HORIZ at 11,
 * STRIDE from 12 up, and for reads NUM in [23:20].
 *
 * Only the configuration Mesa actually emits for the VS/CS is modelled --
 * 32-bit, horizontal, non-laned.  Vertical access, the 8- and 16-bit sizes and
 * laned mode are declined by name rather than approximated, because reading
 * the VPM the wrong way round produces plausible-looking garbage.
 *
 * The DMA half of the block (VCD: VDR/VDW) is deliberately absent.  It can only
 * be triggered by a write to QPU_W_VPM_ADDR, and the kernel's shader validator
 * rejects those outright ("General VPM DMA unsupported" in
 * vc4_validate_shaders.c), so no shader the guest can submit is able to reach
 * it.  Vertex data arrives in the VPM through vc4_vpm_load_vertex() instead,
 * which is what the fixed-function vertex fetch does.
 */

#ifndef HW_DISPLAY_VC4_VPM_H
#define HW_DISPLAY_VC4_VPM_H

#define VC4_VPM_ROWS    64
#define VC4_VPM_LANES   16

/* Values of the SIZE field. */
#define VC4_VPM_SIZE_8   0
#define VC4_VPM_SIZE_16  1
#define VC4_VPM_SIZE_32  2

typedef struct V3DVPMSetup {
    bool valid;
    unsigned addr;      /* starting row */
    unsigned size;      /* VC4_VPM_SIZE_* */
    unsigned stride;    /* rows to advance after each access */
    unsigned num;       /* reads only: rows the setup covers */
    bool horiz;
    bool laned;
} V3DVPMSetup;

typedef struct V3DVPM {
    uint32_t mem[VC4_VPM_ROWS][VC4_VPM_LANES];
    V3DVPMSetup rd;
    V3DVPMSetup wr;
    unsigned rd_row;
    unsigned wr_row;

    /* Set when an access was declined, naming what was asked for.  Sticky
     * until the next reset, so a caller can report the first cause. */
    const char *declined;
} V3DVPM;

void vc4_vpm_reset(V3DVPM *v);

/* Decode a value written to QPU_W_VPMVCD_SETUP.  Returns false (and sets
 * ->declined) for a configuration this model does not implement. */
bool vc4_vpm_setup_read(V3DVPM *v, uint32_t value);
bool vc4_vpm_setup_write(V3DVPM *v, uint32_t value);

/* One row across all 16 lanes, advancing by the configured stride. */
bool vc4_vpm_read(V3DVPM *v, uint32_t lanes[VC4_VPM_LANES]);
bool vc4_vpm_write(V3DVPM *v, const uint32_t lanes[VC4_VPM_LANES]);

/*
 * Place one dword of vertex data where a shader will find it: the fetch
 * hardware fills the VPM so that row r, lane l is dword r of the attribute
 * data for the l'th vertex of the batch.
 */
bool vc4_vpm_load_vertex(V3DVPM *v, unsigned row, unsigned lane,
                         uint32_t value);

#endif /* HW_DISPLAY_VC4_VPM_H */
