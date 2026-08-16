/*
 * VideoCore IV surface tiling arithmetic.
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/display/vc4_tiling.h.  The microtile shapes and the padding
 * rules follow utile_width()/utile_height() and vc4_check_tex_size() in
 * drivers/gpu/drm/vc4/vc4_validate.c, which is the kernel's statement of the
 * same layout (it sizes buffers with it).
 */

#include "qemu/osdep.h"
#include "hw/display/vc4_tiling.h"

/* A microtile is 64 bytes however the pixels are shaped. */
#define VC4_MICROTILE_BYTES 64

unsigned vc4_utile_width(unsigned cpp)
{
    switch (cpp) {
    case 1:
    case 2:
        return 8;
    case 4:
        return 4;
    case 8:
        return 2;
    default:
        return 1;
    }
}

unsigned vc4_utile_height(unsigned cpp)
{
    return cpp == 1 ? 8 : 4;
}

unsigned vc4_aligned_width(unsigned tiling, unsigned cpp, unsigned width)
{
    unsigned utile_w = vc4_utile_width(cpp);

    /* T pads to whole supertiles, eight microtiles wide; the other two pad to
     * a single microtile. */
    if (tiling == VC4_TILING_T) {
        return ROUND_UP(width, utile_w * 8);
    }
    return ROUND_UP(width, utile_w);
}

bool vc4_tiling_supported(unsigned tiling)
{
    return tiling == VC4_TILING_LINEAR || tiling == VC4_TILING_LT;
}

bool vc4_tiling_offset(unsigned tiling, unsigned cpp, unsigned aligned_width,
                       unsigned x, unsigned y, uint32_t *offset)
{
    unsigned utile_w = vc4_utile_width(cpp);
    unsigned utile_h = vc4_utile_height(cpp);
    unsigned utiles_per_row;

    switch (tiling) {
    case VC4_TILING_LINEAR:
        *offset = y * aligned_width * cpp + x * cpp;
        return true;

    case VC4_TILING_LT:
        utiles_per_row = aligned_width / utile_w;
        *offset = ((y / utile_h) * utiles_per_row + (x / utile_w)) *
                      VC4_MICROTILE_BYTES +
                  ((y % utile_h) * utile_w + (x % utile_w)) * cpp;
        return true;

    default:
        /*
         * T arranges microtiles into 4 KB supertiles whose internal order
         * alternates direction between rows.  Guessing that order would
         * scramble the image in a way that is hard to spot, so decline it and
         * let the caller say so.
         */
        return false;
    }
}
