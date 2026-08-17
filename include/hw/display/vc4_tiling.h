/*
 * VideoCore IV surface tiling arithmetic.
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Where a pixel lives in memory, for each of the layouts the VC4 can read and
 * write.  Split out from the device model because it is pure arithmetic that
 * is easy to get subtly wrong and hard to notice: a wrong offset produces a
 * picture that is scrambled rather than absent.  Unit tested in
 * tests/unit/test-vc4-tiling.c.
 *
 * The layouts (drivers/gpu/drm/vc4/vc4_packet.h):
 *
 *   LINEAR  rows in order, each row padded out to a whole microtile in x.
 *   LT      64-byte microtiles in raster order; pixels raster order within.
 *   T       not implemented yet -- see vc4_tiling_supported().
 *
 * A microtile is always 64 bytes; its shape depends on the pixel size, per
 * utile_width()/utile_height() in drivers/gpu/drm/vc4/vc4_validate.c.
 */

#ifndef HW_DISPLAY_VC4_TILING_H
#define HW_DISPLAY_VC4_TILING_H

/* Values of the memory-format fields in the render config and the tile
 * load/store packets. */
#define VC4_TILING_LINEAR   0
#define VC4_TILING_T        1
#define VC4_TILING_LT       2

/* Width and height in pixels of a 64-byte microtile. */
unsigned vc4_utile_width(unsigned cpp);
unsigned vc4_utile_height(unsigned cpp);

/* Width a surface is padded to, in pixels, for the given layout. */
unsigned vc4_aligned_width(unsigned tiling, unsigned cpp, unsigned width);

/*
 * Which layout a surface of this size uses.  The texture unit decides per
 * miplevel with this rule, so miptrees are laid out to match
 * (vc4_tiling.c:vc4_size_is_lt in Mesa, size_is_lt in the kernel's
 * vc4_validate.c).
 */
bool vc4_size_is_lt(unsigned width, unsigned height, unsigned cpp);

/* Whether this model can place pixels in the given layout. */
bool vc4_tiling_supported(unsigned tiling);

/*
 * Byte offset of pixel (x, y) from the start of a surface whose padded width
 * is aligned_width pixels.  Returns false for a layout this model does not
 * implement, so a caller declines rather than writing to the wrong address.
 */
bool vc4_tiling_offset(unsigned tiling, unsigned cpp, unsigned aligned_width,
                       unsigned x, unsigned y, uint32_t *offset);

#endif /* HW_DISPLAY_VC4_TILING_H */
