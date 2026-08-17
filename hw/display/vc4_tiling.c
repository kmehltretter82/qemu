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

bool vc4_size_is_lt(unsigned width, unsigned height, unsigned cpp)
{
    return width <= 4 * vc4_utile_width(cpp) ||
           height <= 4 * vc4_utile_height(cpp);
}

bool vc4_tiling_supported(unsigned tiling)
{
    return tiling == VC4_TILING_LINEAR || tiling == VC4_TILING_LT ||
           tiling == VC4_TILING_T;
}

/*
 * Where a microtile lives in a T image.
 *
 * T groups microtiles into 4 KB tiles of 8x8, each of which is four 1 KB
 * subtiles of 4x4.  Two things make it not a plain raster order: odd rows of
 * 4 KB tiles run right to left, and the four subtiles within a tile are
 * permuted, differently on odd and even rows.  Both come from Mesa's
 * t_utile_address() in src/gallium/drivers/vc4/vc4_tiling.c, which is the
 * layout the texture unit and the tile store actually use.
 */
static uint32_t t_subtile_offset(unsigned utile_x, unsigned utile_y,
                                 unsigned utile_stride)
{
    static const unsigned even_map[4] = { 0, 3, 1, 2 };
    static const unsigned odd_map[4] = { 2, 1, 3, 0 };
    unsigned tile_stride = utile_stride >> 3;
    unsigned tile_x = utile_x >> 3;
    unsigned tile_y = utile_y >> 3;
    bool odd_tile_y = tile_y & 1;
    unsigned stile_index = (((utile_y >> 2) & 1) << 1) | ((utile_x >> 2) & 1);

    if (odd_tile_y) {
        tile_x = tile_stride - tile_x - 1;
    }

    return 4096 * (tile_y * tile_stride + tile_x) +
           1024 * (odd_tile_y ? odd_map[stile_index] : even_map[stile_index]);
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

    case VC4_TILING_T: {
        unsigned utile_x = x / utile_w;
        unsigned utile_y = y / utile_h;

        utiles_per_row = aligned_width / utile_w;
        /* A T image is always a whole number of 4 KB tiles across. */
        if (utiles_per_row & 7) {
            return false;
        }
        *offset = t_subtile_offset(utile_x, utile_y, utiles_per_row) +
                  VC4_MICROTILE_BYTES * (((utile_y & 3) * 4) + (utile_x & 3)) +
                  ((y % utile_h) * utile_w + (x % utile_w)) * cpp;
        return true;
    }

    default:
        return false;
    }
}
