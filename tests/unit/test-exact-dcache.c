/*
 * Contract tests for byte ranges retained by the Exact D-cache observer.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "target/arm/exact/dcache-rules.h"

static void assert_range(ExByteRange r, unsigned lo, unsigned hi)
{
    g_assert_cmpuint(r.lo, ==, lo);
    g_assert_cmpuint(r.hi, ==, hi);
}

static void test_add(void)
{
    ExByteRange r = { 0 };

    ex_byte_range_add(&r, 16, 32);
    assert_range(r, 16, 32);
    ex_byte_range_add(&r, 32, 48);
    assert_range(r, 16, 48);
    ex_byte_range_add(&r, 8, 20);
    assert_range(r, 8, 48);

    /* Do not fill a gap between disjoint stores. */
    ex_byte_range_add(&r, 64, 80);
    assert_range(r, 8, 48);
}

static void test_remove(void)
{
    ExByteRange r = { 0, 128 };

    ex_byte_range_remove(&r, 0, 16);
    assert_range(r, 16, 128);
    ex_byte_range_remove(&r, 112, 128);
    assert_range(r, 16, 112);
    ex_byte_range_remove(&r, 48, 80);
    assert_range(r, 16, 48);
    ex_byte_range_remove(&r, 0, 64);
    g_assert_true(ex_byte_range_empty(r));

    r = (ExByteRange) { 0, 128 };
    ex_byte_range_remove(&r, 16, 96);
    assert_range(r, 96, 128);
    ex_byte_range_remove(&r, 96, 128);
    g_assert_true(ex_byte_range_empty(r));
}

static void test_overlap(void)
{
    ExByteRange r = { 16, 32 };

    g_assert_false(ex_byte_range_overlaps(r, 0, 16));
    g_assert_true(ex_byte_range_overlaps(r, 15, 17));
    g_assert_true(ex_byte_range_overlaps(r, 31, 64));
    g_assert_false(ex_byte_range_overlaps(r, 32, 64));
    g_assert_false(ex_byte_range_overlaps((ExByteRange) { 0 }, 0, 128));
}

static void test_partial_overwrite(void)
{
    ExByteRange lost = { 0, 64 };

    /* A device replaced its prefix; reads there no longer prove a loss. */
    ex_byte_range_remove(&lost, 0, 16);
    g_assert_false(ex_byte_range_overlaps(lost, 0, 16));
    g_assert_true(ex_byte_range_overlaps(lost, 16, 20));

    /* Sequential CPU reconstruction eventually cancels the pending loss. */
    for (unsigned i = 16; i < 64; i += 8) {
        ex_byte_range_remove(&lost, i, i + 8);
    }
    g_assert_true(ex_byte_range_empty(lost));
}

static void test_invalidate_state(void)
{
    ExByteRange lost = { 0, 64 };
    uint8_t state = DC_DIRTY;

    g_assert_true(ex_dcache_by_va_invalidate(&state, &lost));
    g_assert_cmpuint(state, ==, DC_DROPPED);
    assert_range(lost, 0, 64);

    /* A completion-time invalidate cannot restore the map-time loss. */
    g_assert_false(ex_dcache_by_va_invalidate(&state, &lost));
    g_assert_cmpuint(state, ==, DC_DROPPED);
    assert_range(lost, 0, 64);
}

static void test_uncertain_setway(void)
{
    ExByteRange dirty = { 8, 24 };
    uint8_t state = DC_DIRTY;

    g_assert_false(ex_dcache_uncertain_setway(&state, &dirty, 'i'));
    g_assert_cmpuint(state, ==, DC_DIRTY);
    assert_range(dirty, 8, 24);

    state = DC_DMA_WRITTEN;
    g_assert_true(ex_dcache_uncertain_setway(&state, &dirty, 'i'));
    g_assert_cmpuint(state, ==, DC_CLEAN);
    g_assert_true(ex_byte_range_empty(dirty));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/exact-dcache/add", test_add);
    g_test_add_func("/exact-dcache/remove", test_remove);
    g_test_add_func("/exact-dcache/overlap", test_overlap);
    g_test_add_func("/exact-dcache/partial-overwrite", test_partial_overwrite);
    g_test_add_func("/exact-dcache/invalidate-state", test_invalidate_state);
    g_test_add_func("/exact-dcache/uncertain-setway", test_uncertain_setway);
    return g_test_run();
}
