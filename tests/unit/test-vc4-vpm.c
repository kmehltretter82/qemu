/*
 * Unit tests for the VideoCore IV VPM.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The setup words used here are the ones Mesa's vc4 driver actually emits
 * (vc4_qpu_emit.c): 0x1a00 for a write setup, and addr | 0x1a00 | (num << 20)
 * for a read setup.  Decoding those correctly is the whole job of the setup
 * register, so the tests are written in terms of them rather than in terms of
 * hand-assembled field values.
 */

#include "qemu/osdep.h"
#include "hw/display/vc4_vpm.h"

/* What Mesa emits for the VS/CS: 32-bit, horizontal, non-laned, stride 1. */
#define MESA_SETUP_BASE 0x00001a00u
#define MESA_READ_SETUP(addr, num) \
    (MESA_SETUP_BASE | (addr) | ((uint32_t)((num) & 0xf) << 20))

static void test_mesa_setup_decodes(void)
{
    V3DVPM v;

    vc4_vpm_reset(&v);

    g_assert_true(vc4_vpm_setup_write(&v, MESA_SETUP_BASE));
    g_assert_true(v.wr.valid);
    g_assert_cmpuint(v.wr.addr, ==, 0);
    g_assert_cmpuint(v.wr.size, ==, VC4_VPM_SIZE_32);
    g_assert_cmpuint(v.wr.stride, ==, 1);
    g_assert_true(v.wr.horiz);
    g_assert_false(v.wr.laned);

    g_assert_true(vc4_vpm_setup_read(&v, MESA_READ_SETUP(16, 12)));
    g_assert_true(v.rd.valid);
    g_assert_cmpuint(v.rd.addr, ==, 16);
    g_assert_cmpuint(v.rd.num, ==, 12);
    g_assert_cmpuint(v.rd.size, ==, VC4_VPM_SIZE_32);
    g_assert_cmpuint(v.rd.stride, ==, 1);
    g_assert_null(v.declined);
}

static void test_write_then_read_back(void)
{
    V3DVPM v;
    uint32_t out[VC4_VPM_LANES];
    unsigned lane, row;

    vc4_vpm_reset(&v);
    g_assert_true(vc4_vpm_setup_write(&v, MESA_SETUP_BASE));

    /* Three rows of 16 lanes, each lane distinguishable. */
    for (row = 0; row < 3; row++) {
        uint32_t in[VC4_VPM_LANES];

        for (lane = 0; lane < VC4_VPM_LANES; lane++) {
            in[lane] = 0x1000 * row + lane;
        }
        g_assert_true(vc4_vpm_write(&v, in));
    }
    g_assert_cmpuint(v.wr_row, ==, 3);

    g_assert_true(vc4_vpm_setup_read(&v, MESA_READ_SETUP(0, 3)));
    for (row = 0; row < 3; row++) {
        g_assert_true(vc4_vpm_read(&v, out));
        for (lane = 0; lane < VC4_VPM_LANES; lane++) {
            g_assert_cmphex(out[lane], ==, 0x1000 * row + lane);
        }
    }
    g_assert_null(v.declined);
}

/* A read setup starting part way up the VPM reads from there. */
static void test_read_offset(void)
{
    V3DVPM v;
    uint32_t out[VC4_VPM_LANES];

    vc4_vpm_reset(&v);
    g_assert_true(vc4_vpm_load_vertex(&v, 20, 5, 0xcafe));

    g_assert_true(vc4_vpm_setup_read(&v, MESA_READ_SETUP(20, 1)));
    g_assert_true(vc4_vpm_read(&v, out));
    g_assert_cmphex(out[5], ==, 0xcafe);
    g_assert_cmphex(out[4], ==, 0);
}

/*
 * The fetch hardware fills the VPM so that a row read hands each of the 16
 * vertices in flight one dword: row = which dword, lane = which vertex.
 */
static void test_vertex_load_layout(void)
{
    V3DVPM v;
    uint32_t out[VC4_VPM_LANES];
    unsigned vertex, dword;

    vc4_vpm_reset(&v);

    /* Three vertices of three dwords each, as a 12-byte attribute would be. */
    for (vertex = 0; vertex < 3; vertex++) {
        for (dword = 0; dword < 3; dword++) {
            g_assert_true(vc4_vpm_load_vertex(&v, dword, vertex,
                                              0x100 * vertex + dword));
        }
    }

    g_assert_true(vc4_vpm_setup_read(&v, MESA_READ_SETUP(0, 3)));
    for (dword = 0; dword < 3; dword++) {
        g_assert_true(vc4_vpm_read(&v, out));
        for (vertex = 0; vertex < 3; vertex++) {
            g_assert_cmphex(out[vertex], ==, 0x100 * vertex + dword);
        }
    }
}

static void test_stride(void)
{
    V3DVPM v;
    uint32_t in[VC4_VPM_LANES] = { 0 };
    uint32_t out[VC4_VPM_LANES];

    vc4_vpm_reset(&v);
    /* Stride 2: rows 0, 2, 4. */
    g_assert_true(vc4_vpm_setup_write(&v, 0x00002a00u));
    g_assert_cmpuint(v.wr.stride, ==, 2);

    in[0] = 0xaa;
    g_assert_true(vc4_vpm_write(&v, in));
    in[0] = 0xbb;
    g_assert_true(vc4_vpm_write(&v, in));
    g_assert_cmpuint(v.wr_row, ==, 4);

    g_assert_true(vc4_vpm_setup_read(&v, MESA_READ_SETUP(0, 3)));
    g_assert_true(vc4_vpm_read(&v, out));
    g_assert_cmphex(out[0], ==, 0xaa);
    g_assert_true(vc4_vpm_read(&v, out));
    g_assert_cmphex(out[0], ==, 0);      /* row 1 was skipped */
    g_assert_true(vc4_vpm_read(&v, out));
    g_assert_cmphex(out[0], ==, 0xbb);
}

static void test_unsupported_configurations_are_declined(void)
{
    V3DVPM v;

    /* Vertical access. */
    vc4_vpm_reset(&v);
    g_assert_false(vc4_vpm_setup_write(&v, MESA_SETUP_BASE & ~(1u << 11)));
    g_assert_cmpstr(v.declined, ==, "vertical VPM access");

    /* 16-bit access. */
    vc4_vpm_reset(&v);
    g_assert_false(vc4_vpm_setup_write(&v, (MESA_SETUP_BASE & ~0x300u) |
                                           (VC4_VPM_SIZE_16 << 8)));
    g_assert_cmpstr(v.declined, ==, "VPM 8/16-bit access");

    /* Laned access. */
    vc4_vpm_reset(&v);
    g_assert_false(vc4_vpm_setup_write(&v, MESA_SETUP_BASE | (1u << 10)));
    g_assert_cmpstr(v.declined, ==, "laned VPM access");

    /* A starting row off the end of the VPM. */
    vc4_vpm_reset(&v);
    g_assert_false(vc4_vpm_setup_read(&v, MESA_READ_SETUP(64, 1)));
    g_assert_cmpstr(v.declined, ==, "VPM setup address out of range");
}

static void test_access_without_setup_is_declined(void)
{
    V3DVPM v;
    uint32_t lanes[VC4_VPM_LANES] = { 0 };

    vc4_vpm_reset(&v);
    g_assert_false(vc4_vpm_read(&v, lanes));
    g_assert_cmpstr(v.declined, ==, "VPM read before read setup");

    vc4_vpm_reset(&v);
    g_assert_false(vc4_vpm_write(&v, lanes));
    g_assert_cmpstr(v.declined, ==, "VPM write before write setup");
}

/* Walking off the end must stop rather than wrap onto another shader's rows. */
static void test_running_off_the_end(void)
{
    V3DVPM v;
    uint32_t lanes[VC4_VPM_LANES] = { 0 };
    unsigned i;

    vc4_vpm_reset(&v);
    g_assert_true(vc4_vpm_setup_write(&v, MESA_SETUP_BASE | 60));
    for (i = 0; i < 4; i++) {
        g_assert_true(vc4_vpm_write(&v, lanes));
    }
    g_assert_false(vc4_vpm_write(&v, lanes));
    g_assert_cmpstr(v.declined, ==, "VPM write past the end");

    g_assert_false(vc4_vpm_load_vertex(&v, VC4_VPM_ROWS, 0, 0));
    g_assert_false(vc4_vpm_load_vertex(&v, 0, VC4_VPM_LANES, 0));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/vc4-vpm/mesa-setup-decodes", test_mesa_setup_decodes);
    g_test_add_func("/vc4-vpm/write-then-read-back", test_write_then_read_back);
    g_test_add_func("/vc4-vpm/read-offset", test_read_offset);
    g_test_add_func("/vc4-vpm/vertex-load-layout", test_vertex_load_layout);
    g_test_add_func("/vc4-vpm/stride", test_stride);
    g_test_add_func("/vc4-vpm/unsupported-declined",
                    test_unsupported_configurations_are_declined);
    g_test_add_func("/vc4-vpm/access-without-setup",
                    test_access_without_setup_is_declined);
    g_test_add_func("/vc4-vpm/running-off-the-end", test_running_off_the_end);

    return g_test_run();
}
