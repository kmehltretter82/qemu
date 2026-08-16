/*
 * Unit tests for the VideoCore IV surface tiling arithmetic.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Two kinds of check.  The hand-computed cases pin the layout against offsets
 * worked out from the microtile geometry by hand.  The bijection check proves
 * the property that actually matters and that spot checks miss: every pixel of
 * a surface must map to a distinct offset, and the offsets must exactly fill
 * the surface.  An off-by-one in the microtile arithmetic breaks that even
 * when the corner pixels happen to look right.
 */

#include "qemu/osdep.h"
#include "hw/display/vc4_tiling.h"

static void test_utile_geometry(void)
{
    /* A microtile is always 64 bytes, however the pixels are shaped. */
    unsigned cpp;

    for (cpp = 1; cpp <= 8; cpp <<= 1) {
        g_assert_cmpuint(vc4_utile_width(cpp) * vc4_utile_height(cpp) * cpp,
                         ==, 64);
    }

    g_assert_cmpuint(vc4_utile_width(1), ==, 8);
    g_assert_cmpuint(vc4_utile_height(1), ==, 8);
    g_assert_cmpuint(vc4_utile_width(2), ==, 8);
    g_assert_cmpuint(vc4_utile_height(2), ==, 4);
    g_assert_cmpuint(vc4_utile_width(4), ==, 4);
    g_assert_cmpuint(vc4_utile_height(4), ==, 4);
    g_assert_cmpuint(vc4_utile_width(8), ==, 2);
    g_assert_cmpuint(vc4_utile_height(8), ==, 4);
}

static void test_aligned_width(void)
{
    /* 32-bit pixels: microtiles are 4 wide, supertiles 32. */
    g_assert_cmpuint(vc4_aligned_width(VC4_TILING_LINEAR, 4, 16), ==, 16);
    g_assert_cmpuint(vc4_aligned_width(VC4_TILING_LINEAR, 4, 17), ==, 20);
    g_assert_cmpuint(vc4_aligned_width(VC4_TILING_LT, 4, 17), ==, 20);
    g_assert_cmpuint(vc4_aligned_width(VC4_TILING_T, 4, 17), ==, 32);
    /* 16-bit pixels: microtiles are 8 wide. */
    g_assert_cmpuint(vc4_aligned_width(VC4_TILING_LINEAR, 2, 17), ==, 24);
}

static void test_linear_offsets(void)
{
    uint32_t off;

    /* 16 pixels wide, 4 bytes each: a row is 64 bytes. */
    g_assert_true(vc4_tiling_offset(VC4_TILING_LINEAR, 4, 16, 0, 0, &off));
    g_assert_cmpuint(off, ==, 0);
    g_assert_true(vc4_tiling_offset(VC4_TILING_LINEAR, 4, 16, 1, 0, &off));
    g_assert_cmpuint(off, ==, 4);
    g_assert_true(vc4_tiling_offset(VC4_TILING_LINEAR, 4, 16, 0, 1, &off));
    g_assert_cmpuint(off, ==, 64);
    g_assert_true(vc4_tiling_offset(VC4_TILING_LINEAR, 4, 16, 15, 15, &off));
    g_assert_cmpuint(off, ==, 15 * 64 + 15 * 4);
}

static void test_lt_offsets(void)
{
    uint32_t off;

    /* 32-bit pixels: microtiles are 4x4 and 64 bytes, so a 16-pixel-wide
     * surface is four microtiles per row. */

    /* Within the first microtile, pixels are in raster order. */
    g_assert_true(vc4_tiling_offset(VC4_TILING_LT, 4, 16, 0, 0, &off));
    g_assert_cmpuint(off, ==, 0);
    g_assert_true(vc4_tiling_offset(VC4_TILING_LT, 4, 16, 1, 0, &off));
    g_assert_cmpuint(off, ==, 4);
    g_assert_true(vc4_tiling_offset(VC4_TILING_LT, 4, 16, 0, 1, &off));
    g_assert_cmpuint(off, ==, 16);
    g_assert_true(vc4_tiling_offset(VC4_TILING_LT, 4, 16, 3, 3, &off));
    g_assert_cmpuint(off, ==, 60);

    /* The next microtile to the right starts one microtile in. */
    g_assert_true(vc4_tiling_offset(VC4_TILING_LT, 4, 16, 4, 0, &off));
    g_assert_cmpuint(off, ==, 64);

    /* The microtile below the first one starts after a whole row of them. */
    g_assert_true(vc4_tiling_offset(VC4_TILING_LT, 4, 16, 0, 4, &off));
    g_assert_cmpuint(off, ==, 4 * 64);
}

static void test_t_is_declined(void)
{
    uint32_t off = 0xdeadbeef;

    g_assert_false(vc4_tiling_supported(VC4_TILING_T));
    g_assert_false(vc4_tiling_offset(VC4_TILING_T, 4, 32, 0, 0, &off));
    /* Declining must not leave a plausible-looking offset behind. */
    g_assert_cmpuint(off, ==, 0xdeadbeef);
}

/*
 * Every pixel maps to its own offset, and together they exactly fill the
 * surface with no gaps.  This is the property a spot check cannot see.
 */
static void check_bijection(unsigned tiling, unsigned cpp, unsigned width,
                            unsigned height)
{
    unsigned aligned_width = vc4_aligned_width(tiling, cpp, width);
    unsigned aligned_height = tiling == VC4_TILING_LINEAR
                            ? height
                            : ROUND_UP(height, vc4_utile_height(cpp));
    uint32_t size = aligned_width * aligned_height * cpp;
    uint8_t *seen = g_malloc0(size);
    unsigned x, y, i;

    for (y = 0; y < aligned_height; y++) {
        for (x = 0; x < aligned_width; x++) {
            uint32_t off;

            g_assert_true(vc4_tiling_offset(tiling, cpp, aligned_width,
                                            x, y, &off));
            g_assert_cmpuint(off + cpp, <=, size);
            for (i = 0; i < cpp; i++) {
                g_assert_cmpuint(seen[off + i], ==, 0);
                seen[off + i] = 1;
            }
        }
    }
    for (i = 0; i < size; i++) {
        g_assert_cmpuint(seen[i], ==, 1);
    }
    g_free(seen);
}

static void test_linear_bijection(void)
{
    check_bijection(VC4_TILING_LINEAR, 4, 16, 16);
    check_bijection(VC4_TILING_LINEAR, 4, 17, 5);
    check_bijection(VC4_TILING_LINEAR, 2, 33, 9);
    check_bijection(VC4_TILING_LINEAR, 1, 7, 3);
}

static void test_lt_bijection(void)
{
    check_bijection(VC4_TILING_LT, 4, 16, 16);
    check_bijection(VC4_TILING_LT, 4, 17, 5);
    check_bijection(VC4_TILING_LT, 2, 33, 9);
    check_bijection(VC4_TILING_LT, 1, 7, 3);
    check_bijection(VC4_TILING_LT, 8, 6, 6);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/vc4-tiling/utile-geometry", test_utile_geometry);
    g_test_add_func("/vc4-tiling/aligned-width", test_aligned_width);
    g_test_add_func("/vc4-tiling/linear-offsets", test_linear_offsets);
    g_test_add_func("/vc4-tiling/lt-offsets", test_lt_offsets);
    g_test_add_func("/vc4-tiling/t-declined", test_t_is_declined);
    g_test_add_func("/vc4-tiling/linear-bijection", test_linear_bijection);
    g_test_add_func("/vc4-tiling/lt-bijection", test_lt_bijection);

    return g_test_run();
}
