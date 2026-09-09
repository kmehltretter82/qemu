/*
 * Executable contracts for the Exact TLB observer.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Arm DDI 0487 M.c, D8.7.1: R_SKQHL, R_CBXXM, R_JQQTC, I_PGVGZ.
 * A snapshot is not an update-completion event. In particular, differences
 * between members do not by themselves establish a missing BBM operation.
 *
 * The contiguous contract below covers 64-bit stage-1 leaf descriptors with
 * the conventional <=48-bit OA layout. The caller must exclude DS, AIE, PIE,
 * extended OA layouts and non-leaf descriptors. No hardware outcome is
 * injected; these functions describe and check choices in a captured set.
 */
#ifndef TARGET_ARM_EXACT_TLB_RULES_H
#define TARGET_ARM_EXACT_TLB_RULES_H

#include <stdbool.h>
#include <stdint.h>

#define EX_CONTIG_MAX 128
#define EX_DESC_CONT (UINT64_C(1) << 52)
#define EX_DESC_GP (UINT64_C(1) << 50)
#define EX_DESC_ATTRINDX (UINT64_C(7) << 2)

typedef struct ExContigGeometry {
    unsigned entries;
    unsigned page_shift;
    unsigned level;
} ExContigGeometry;

static inline ExContigGeometry ex_contig_geometry(unsigned level,
                                                 unsigned page_shift)
{
    unsigned entries = 0;

    /* D8.7.1 Table D8-104, including 4K level-1 blocks. */
    if (level == 3) {
        switch (page_shift) {
        case 12:
            entries = 16;
            break;
        case 14:
            entries = 128;
            break;
        case 16:
            entries = 32;
            break;
        }
    } else if (level == 2) {
        switch (page_shift) {
        case 21:
            entries = 16;
            break;
        case 25:
            entries = 32;
            break;
        case 29:
            entries = 32;
            break;
        }
    } else if (level == 1 && page_shift == 30) {
        entries = 16;
    }
    return (ExContigGeometry) { entries, page_shift, level };
}

typedef enum ExContigState {
    EX_CONTIG_UNAVAILABLE,
    EX_CONTIG_EMPTY,
    EX_CONTIG_UNHINTED,
    EX_CONTIG_UNIFORM,
    EX_CONTIG_BOUNDED,
    EX_CONTIG_MIXED_HINT,
    EX_CONTIG_OA_UNRESOLVED,
    EX_CONTIG_STATES,
} ExContigState;

typedef struct ExContigSummary {
    ExContigState state;
    unsigned valid;
    unsigned invalid;
    uint64_t oa_base;
} ExContigSummary;

typedef struct ExContigChoice {
    uint64_t oa;
    uint64_t attrs;
} ExContigChoice;

static inline uint64_t ex_contig_oa_mask(ExContigGeometry g)
{
    return ((UINT64_C(1) << 48) - 1) &
           ~((UINT64_C(1) << g.page_shift) - 1);
}

static inline uint64_t ex_contig_attrs(uint64_t desc, ExContigGeometry g)
{
    /* Keep permissions, GP, AttrIndx, AF and DBM together. Ignore SW bits. */
    return desc & ~(ex_contig_oa_mask(g) | (UINT64_C(15) << 55));
}

static inline ExContigSummary ex_contig_analyze(const uint64_t *desc,
                                               ExContigGeometry g)
{
    ExContigSummary s = { .state = EX_CONTIG_UNAVAILABLE };
    uint64_t span, mask, first_attrs = 0;
    bool hinted = false, unhinted = false, oa_bad = false, attrs_differ = false;
    unsigned i;

    if (!g.entries || g.entries !=
        ex_contig_geometry(g.level, g.page_shift).entries) {
        return s;
    }
    span = (uint64_t)g.entries << g.page_shift;
    mask = ex_contig_oa_mask(g);
    for (i = 0; i < g.entries; i++) {
        uint64_t d = desc[i], oa, attrs;

        if (!(d & 1)) {
            s.invalid++;
            continue;               /* I_PGVGZ: no CONT bit or OA candidate */
        }
        if ((d & 3) != (g.level == 3 ? 3 : 1)) {
            return s;               /* valid table/reserved, not a leaf */
        }
        if ((d & (UINT64_C(3) << 48)) ||
            ((g.page_shift == 16 || g.page_shift == 29) && (d & 0xf000))) {
            return s;               /* an extended OA layout */
        }
        hinted |= (d & EX_DESC_CONT) != 0;
        unhinted |= (d & EX_DESC_CONT) == 0;
        oa = d & mask;
        attrs = ex_contig_attrs(d, g);
        if (!s.valid) {
            s.oa_base = oa & ~(span - 1);
            first_attrs = attrs;
        }
        /* Every member must imply the same aligned mapping, even if #0 is 0. */
        oa_bad |= oa != s.oa_base + ((uint64_t)i << g.page_shift);
        attrs_differ |= attrs != first_attrs;
        s.valid++;
    }
    s.state = !s.valid ? EX_CONTIG_EMPTY :
              !hinted ? EX_CONTIG_UNHINTED :
              unhinted ? EX_CONTIG_MIXED_HINT :
              oa_bad ? EX_CONTIG_OA_UNRESOLVED :
              attrs_differ ? EX_CONTIG_BOUNDED : EX_CONTIG_UNIFORM;
    return s;
}

/*
 * Construct one whole-descriptor witness, never independently chosen fields.
 * For inconsistent OAs the wording is disputed: deliberately return unknown
 * (false), rather than inventing a physical-address selection algorithm.
 */
static inline bool ex_contig_choice(const uint64_t *desc, ExContigGeometry g,
                                    unsigned member, uint64_t va,
                                    ExContigChoice *out)
{
    ExContigSummary s = ex_contig_analyze(desc, g);
    uint64_t span;

    if ((s.state != EX_CONTIG_UNIFORM && s.state != EX_CONTIG_BOUNDED) ||
        member >= g.entries || !(desc[member] & 1)) {
        return false;
    }
    span = (uint64_t)g.entries << g.page_shift;
    out->oa = s.oa_base + (va & (span - 1));
    out->attrs = ex_contig_attrs(desc[member], g);
    return true;
}

/* A candidate is permitted only if ONE valid descriptor witnesses the tuple. */
static inline bool ex_contig_allows(const uint64_t *desc, ExContigGeometry g,
                                    uint64_t va, ExContigChoice candidate)
{
    unsigned i;
    ExContigChoice witness;

    for (i = 0; i < g.entries; i++) {
        if (ex_contig_choice(desc, g, i, va, &witness) &&
            witness.oa == candidate.oa && witness.attrs == candidate.attrs) {
            return true;
        }
    }
    return false;
}

/* Captured at a walk, so the store observer uses the descriptor's regime. */
typedef struct ExLeafPolicy {
    uint64_t benign_mask;
    uint64_t mair;
    uint64_t mair2;
    bool s1_attrs;
    bool aie;
    bool mte;
} ExLeafPolicy;

static inline uint8_t ex_leaf_memattr(ExLeafPolicy p, uint64_t desc)
{
    uint64_t mair = p.aie && (desc & (UINT64_C(1) << 59)) ? p.mair2 : p.mair;

    return mair >> (((desc >> 2) & 7) * 8);
}

static inline bool ex_leaf_change_benign(ExLeafPolicy p,
                                        uint64_t old, uint64_t newv)
{
    uint64_t mask = p.benign_mask;

    if (p.s1_attrs) {
        uint64_t idx = EX_DESC_ATTRINDX | (p.aie ? UINT64_C(1) << 59 : 0);
        uint8_t a = ex_leaf_memattr(p, old), b = ex_leaf_memattr(p, newv);

        mask &= ~idx;
        /* B2.11: Tagged alone is not a mismatched memory attribute. */
        if (a == b || (p.mte && ((a == 0xff && b == 0xf0) ||
                                (a == 0xf0 && b == 0xff)))) {
            mask |= idx;
        }
    }
    return !((old ^ newv) & ~mask);
}

#endif
