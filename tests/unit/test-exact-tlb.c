/*
 * Contract tests using synthetic descriptors; no guest or kernel modification.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "target/arm/exact/tlb-rules.h"

#define TEST_PA UINT64_C(0x10000000000)
#define TEST_VA UINT64_C(0x20000000000)
#define AF (UINT64_C(1) << 10)
#define RO (UINT64_C(1) << 7)
#define UXN (UINT64_C(1) << 54)

static const struct {
    unsigned level, shift, entries;
    uint64_t span;
} layouts[] = {
    { 1, 30, 16, UINT64_C(0x400000000) },
    { 2, 21, 16, UINT64_C(0x2000000) },
    { 3, 12, 16, UINT64_C(0x10000) },
    { 2, 25, 32, UINT64_C(0x40000000) },
    { 3, 14, 128, UINT64_C(0x200000) },
    { 2, 29, 32, UINT64_C(0x400000000) },
    { 3, 16, 32, UINT64_C(0x200000) },
};

static void fill(uint64_t *d, ExContigGeometry g, uint64_t attrs)
{
    unsigned i;

    for (i = 0; i < g.entries; i++) {
        d[i] = (TEST_PA + ((uint64_t)i << g.page_shift)) | attrs |
               EX_DESC_CONT | AF | (g.level == 3 ? 3 : 1);
    }
}

static void test_geometry(void)
{
    unsigned i, level, shift;

    for (i = 0; i < G_N_ELEMENTS(layouts); i++) {
        ExContigGeometry g = ex_contig_geometry(layouts[i].level,
                                               layouts[i].shift);
        g_assert_cmpuint(g.entries, ==, layouts[i].entries);
        g_assert_cmphex((uint64_t)g.entries << g.page_shift,
                        ==, layouts[i].span);
    }
    /* No guessed sizes, including RES0 CONT on 16K/64K level-1 blocks. */
    for (level = 0; level <= 4; level++) {
        for (shift = 0; shift <= 64; shift++) {
            bool known = false;

            for (i = 0; i < G_N_ELEMENTS(layouts); i++) {
                known |= level == layouts[i].level && shift == layouts[i].shift;
            }
            if (!known) {
                g_assert_cmpuint(ex_contig_geometry(level, shift).entries,
                                 ==, 0);
            }
        }
    }
}

static void test_invalid_and_hint(void)
{
    ExContigGeometry g = ex_contig_geometry(3, 12);
    uint64_t d[EX_CONTIG_MAX];
    ExContigSummary s;
    ExContigChoice c = { 0 };
    unsigned i;

    fill(d, g, 0);
    /* Invalid #0 contains apparent CONT, permissions and an unrelated OA. */
    d[0] = UINT64_C(0x00f123456789a002);
    s = ex_contig_analyze(d, g);
    g_assert_cmpint(s.state, ==, EX_CONTIG_UNIFORM);
    g_assert_cmpuint(s.valid, ==, 15);
    g_assert_cmpuint(s.invalid, ==, 1);
    g_assert_cmphex(s.oa_base, ==, TEST_PA);
    g_assert_false(ex_contig_choice(d, g, 0, TEST_VA, &c));
    g_assert_true(ex_contig_choice(d, g, 15, TEST_VA + 123, &c));
    g_assert_cmphex(c.oa, ==, TEST_PA + 123);

    d[1] &= ~EX_DESC_CONT;
    g_assert_cmpint(ex_contig_analyze(d, g).state, ==, EX_CONTIG_MIXED_HINT);
    g_assert_false(ex_contig_choice(d, g, 2, TEST_VA, &c));
    for (i = 1; i < g.entries; i++) {
        d[i] &= ~EX_DESC_CONT;
    }
    g_assert_cmpint(ex_contig_analyze(d, g).state, ==, EX_CONTIG_UNHINTED);
    for (i = 1; i < g.entries; i++) {
        d[i] &= ~UINT64_C(1);
    }
    g_assert_cmpint(ex_contig_analyze(d, g).state, ==, EX_CONTIG_EMPTY);
    g_assert_false(ex_contig_allows(d, g, TEST_VA, c));
}

static void test_tuple_and_offset(void)
{
    unsigned l, source, target;

    for (l = 0; l < G_N_ELEMENTS(layouts); l++) {
        ExContigGeometry g = ex_contig_geometry(layouts[l].level,
                                                layouts[l].shift);
        uint64_t d[EX_CONTIG_MAX] = { 0 };
        ExContigChoice old = { 0 }, newv = { 0 }, candidate = { 0 };

        /* Two whole states: untagged/RW and tagged/RO/guarded/NX. */
        fill(d, g, UINT64_C(5) << 2);
        d[g.entries / 2] &= ~EX_DESC_ATTRINDX;
        d[g.entries / 2] |= (UINT64_C(2) << 2) | RO | EX_DESC_GP | UXN;
        g_assert_cmpint(ex_contig_analyze(d, g).state, ==, EX_CONTIG_BOUNDED);
        for (target = 0; target < g.entries; target++) {
            uint64_t offset = ((uint64_t)target << g.page_shift) + 123;

            for (source = 0; source < g.entries; source++) {
                g_assert_true(ex_contig_choice(d, g, source,
                                               TEST_VA + offset, &candidate));
                g_assert_cmphex(candidate.oa, ==, TEST_PA + offset);
            }
            g_assert_true(ex_contig_choice(d, g, 0, TEST_VA + offset, &old));
            g_assert_true(ex_contig_choice(d, g, g.entries / 2,
                                           TEST_VA + offset, &newv));
            g_assert_true(ex_contig_allows(d, g, TEST_VA + offset, old));
            g_assert_true(ex_contig_allows(d, g, TEST_VA + offset, newv));
            candidate = newv;
            candidate.attrs &= ~RO;  /* fabricated tagged/RW combination */
            g_assert_false(ex_contig_allows(d, g, TEST_VA + offset, candidate));
            candidate = old;
            candidate.attrs |= RO;   /* fabricated untagged/RO combination */
            g_assert_false(ex_contig_allows(d, g, TEST_VA + offset, candidate));
            candidate = old;
            candidate.oa ^= UINT64_C(1) << g.page_shift; /* neighboring page */
            g_assert_false(ex_contig_allows(d, g, TEST_VA + offset, candidate));
        }
        g_assert_true(ex_contig_choice(d, g, 0,
                      TEST_VA + layouts[l].span - 1, &candidate));
        g_assert_cmphex(candidate.oa, ==, TEST_PA + layouts[l].span - 1);
    }
}

static void test_repaint(void)
{
    unsigned l, i, style;

    for (l = 0; l < G_N_ELEMENTS(layouts); l++) {
        ExContigGeometry g = ex_contig_geometry(layouts[l].level,
                                                layouts[l].shift);
        uint64_t d[EX_CONTIG_MAX], next[EX_CONTIG_MAX];
        ExContigChoice old = { 0 }, newv = { 0 };

        for (style = 0; style < 3; style++) {
            fill(d, g, UINT64_C(5) << 2);
            fill(next, g, (UINT64_C(2) << 2) | RO | EX_DESC_GP | UXN);
            g_assert_true(ex_contig_choice(d, g, 0, TEST_VA, &old));
            g_assert_true(ex_contig_choice(next, g, 0, TEST_VA, &newv));
            /* style 0: live writes; 1: whole-set break; 2: per-entry break. */
            if (style == 1) {
                for (i = 0; i < g.entries; i++) {
                    d[i] = 0;
                    g_assert_cmpint(ex_contig_analyze(d, g).state, ==,
                        i + 1 == g.entries ? EX_CONTIG_EMPTY :
                                            EX_CONTIG_UNIFORM);
                }
                g_assert_false(ex_contig_allows(d, g, TEST_VA, old));
            }
            for (i = 0; i < g.entries; i++) {
                if (style == 2) {
                    d[i] = 0;
                    g_assert_cmpint(ex_contig_analyze(d, g).state, !=,
                                    EX_CONTIG_MIXED_HINT);
                }
                d[i] = next[i];
                g_assert_true(ex_contig_allows(d, g, TEST_VA, newv));
                g_assert_cmpint(ex_contig_analyze(d, g).state, ==,
                    style == 1 || i + 1 == g.entries ?
                    EX_CONTIG_UNIFORM : EX_CONTIG_BOUNDED);
            }
            /* Final-table contract; this is not a simulation of TLBI/DSB. */
            g_assert_false(ex_contig_allows(d, g, TEST_VA, old));
            g_assert_true(ex_contig_allows(d, g, TEST_VA, newv));
        }
    }
}

static void test_unknown(void)
{
    ExContigGeometry g = ex_contig_geometry(2, 21);
    uint64_t d[EX_CONTIG_MAX];
    ExContigChoice c;

    fill(d, g, 0);
    d[7] += UINT64_C(1) << 21;
    g_assert_cmpint(ex_contig_analyze(d, g).state, ==, EX_CONTIG_OA_UNRESOLVED);
    g_assert_false(ex_contig_choice(d, g, 0, TEST_VA, &c));
    fill(d, g, 0);
    for (unsigned i = 0; i < g.entries; i++) {
        d[i] += UINT64_C(1) << 21; /* consecutive, but not group-aligned */
    }
    g_assert_cmpint(ex_contig_analyze(d, g).state, ==, EX_CONTIG_OA_UNRESOLVED);
    fill(d, g, 0);
    d[7] |= 2;  /* valid table, not a block */
    g_assert_cmpint(ex_contig_analyze(d, g).state, ==, EX_CONTIG_UNAVAILABLE);
    fill(d, g, 0);
    d[7] |= UINT64_C(1) << 48;
    g_assert_cmpint(ex_contig_analyze(d, g).state, ==, EX_CONTIG_UNAVAILABLE);
    g = ex_contig_geometry(3, 16);
    fill(d, g, 0);
    d[7] |= 0x1000; /* extended OA bits of a 64K descriptor */
    g_assert_cmpint(ex_contig_analyze(d, g).state, ==, EX_CONTIG_UNAVAILABLE);
}

static void test_leaf_changes(void)
{
    ExLeafPolicy p = {
        .benign_mask = AF | RO | UXN | EX_DESC_GP | (UINT64_C(15) << 59),
        .s1_attrs = true,
        .mte = true,
        .mair = (UINT64_C(0xff) << 40) | (UINT64_C(0xf0) << 16) |
                (UINT64_C(0xff) << 8),
    };
    uint64_t old = TEST_PA | EX_DESC_CONT | AF | 3 | (UINT64_C(5) << 2);
    uint64_t newv = (old & ~EX_DESC_ATTRINDX) | (UINT64_C(2) << 2) |
                    RO | EX_DESC_GP;

    g_assert_true(ex_leaf_change_benign(p, old, old ^ RO));
    g_assert_true(ex_leaf_change_benign(p, old, old ^ UXN));
    g_assert_true(ex_leaf_change_benign(p, old, old ^ EX_DESC_GP));
    g_assert_true(ex_leaf_change_benign(p, old, newv));
    g_assert_true(ex_leaf_change_benign(p, newv, old));
    /* An AttrIndx change to an alias of the same MAIR byte is harmless. */
    g_assert_true(ex_leaf_change_benign(p, old,
                  (old & ~EX_DESC_ATTRINDX) | (UINT64_C(1) << 2)));
    p.mte = false;
    g_assert_false(ex_leaf_change_benign(p, old, newv));
    p.mte = true;
    /* Device, OA, shareability and CONT changes are not whitelisted. */
    g_assert_false(ex_leaf_change_benign(p, old, old & ~EX_DESC_ATTRINDX));
    g_assert_false(ex_leaf_change_benign(p, old, old ^ 0x1000));
    g_assert_false(ex_leaf_change_benign(p, old, old ^ 0x100));
    g_assert_false(ex_leaf_change_benign(p, old, old ^ EX_DESC_CONT));
    p.aie = true;
    g_assert_false(ex_leaf_change_benign(p, old, old | (UINT64_C(1) << 59)));
    p.mair2 = p.mair;
    g_assert_true(ex_leaf_change_benign(p, old, old | (UINT64_C(1) << 59)));
    /* The stage-1 MAIR exception must never be applied to stage-2 MemAttr. */
    p.s1_attrs = false;
    g_assert_false(ex_leaf_change_benign(p, old, newv));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/exact-tlb/geometry", test_geometry);
    g_test_add_func("/exact-tlb/invalid-and-hint", test_invalid_and_hint);
    g_test_add_func("/exact-tlb/tuple-and-offset", test_tuple_and_offset);
    g_test_add_func("/exact-tlb/repaint", test_repaint);
    g_test_add_func("/exact-tlb/unknown", test_unknown);
    g_test_add_func("/exact-tlb/leaf-changes", test_leaf_changes);
    return g_test_run();
}
