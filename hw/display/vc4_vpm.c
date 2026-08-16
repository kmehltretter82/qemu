/*
 * VideoCore IV VPM (Vertex Pipe Memory).
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/display/vc4_vpm.h for the field layout and for what is
 * deliberately not modelled.
 */

#include "qemu/osdep.h"
#include "hw/display/vc4_vpm.h"

/* Setup register fields. */
#define VC4_VPM_SETUP_ADDR(v)   ((v) & 0xff)
#define VC4_VPM_SETUP_SIZE(v)   (((v) >> 8) & 0x3)
#define VC4_VPM_SETUP_LANED(v)  (((v) >> 10) & 0x1)
#define VC4_VPM_SETUP_HORIZ(v)  (((v) >> 11) & 0x1)
#define VC4_VPM_SETUP_STRIDE(v) (((v) >> 12) & 0x3f)
#define VC4_VPM_SETUP_NUM(v)    (((v) >> 20) & 0xf)

void vc4_vpm_reset(V3DVPM *v)
{
    memset(v, 0, sizeof(*v));
}

static void vpm_decline(V3DVPM *v, const char *why)
{
    if (!v->declined) {
        v->declined = why;
    }
}

/*
 * Decode a setup word.  Only 32-bit horizontal non-laned access is
 * implemented, which is what Mesa emits for the coordinate and vertex shaders;
 * anything else is declined by name so a caller can report it rather than
 * silently reading the VPM the wrong way round.
 */
static bool vpm_decode(V3DVPM *v, uint32_t value, V3DVPMSetup *out, bool read)
{
    V3DVPMSetup s = {
        .addr = VC4_VPM_SETUP_ADDR(value),
        .size = VC4_VPM_SETUP_SIZE(value),
        .stride = VC4_VPM_SETUP_STRIDE(value),
        .num = read ? VC4_VPM_SETUP_NUM(value) : 0,
        .horiz = VC4_VPM_SETUP_HORIZ(value),
        .laned = VC4_VPM_SETUP_LANED(value),
    };

    if (s.size != VC4_VPM_SIZE_32) {
        vpm_decline(v, "VPM 8/16-bit access");
        return false;
    }
    if (!s.horiz) {
        vpm_decline(v, "vertical VPM access");
        return false;
    }
    if (s.laned) {
        vpm_decline(v, "laned VPM access");
        return false;
    }
    if (s.addr >= VC4_VPM_ROWS) {
        vpm_decline(v, "VPM setup address out of range");
        return false;
    }

    s.valid = true;
    *out = s;
    return true;
}

bool vc4_vpm_setup_read(V3DVPM *v, uint32_t value)
{
    if (!vpm_decode(v, value, &v->rd, true)) {
        return false;
    }
    v->rd_row = v->rd.addr;
    return true;
}

bool vc4_vpm_setup_write(V3DVPM *v, uint32_t value)
{
    if (!vpm_decode(v, value, &v->wr, false)) {
        return false;
    }
    v->wr_row = v->wr.addr;
    return true;
}

bool vc4_vpm_read(V3DVPM *v, uint32_t lanes[VC4_VPM_LANES])
{
    if (!v->rd.valid) {
        vpm_decline(v, "VPM read before read setup");
        return false;
    }
    if (v->rd_row >= VC4_VPM_ROWS) {
        vpm_decline(v, "VPM read past the end");
        return false;
    }

    memcpy(lanes, v->mem[v->rd_row], sizeof(v->mem[0]));
    v->rd_row += v->rd.stride;
    return true;
}

bool vc4_vpm_write(V3DVPM *v, const uint32_t lanes[VC4_VPM_LANES])
{
    if (!v->wr.valid) {
        vpm_decline(v, "VPM write before write setup");
        return false;
    }
    if (v->wr_row >= VC4_VPM_ROWS) {
        vpm_decline(v, "VPM write past the end");
        return false;
    }

    memcpy(v->mem[v->wr_row], lanes, sizeof(v->mem[0]));
    v->wr_row += v->wr.stride;
    return true;
}

bool vc4_vpm_load_vertex(V3DVPM *v, unsigned row, unsigned lane,
                         uint32_t value)
{
    if (row >= VC4_VPM_ROWS || lane >= VC4_VPM_LANES) {
        vpm_decline(v, "vertex data past the end of the VPM");
        return false;
    }
    v->mem[row][lane] = value;
    return true;
}
