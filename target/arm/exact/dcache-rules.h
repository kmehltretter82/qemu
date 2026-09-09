/*
 * Small executable contracts for the Exact D-cache observer.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef TARGET_ARM_EXACT_DCACHE_RULES_H
#define TARGET_ARM_EXACT_DCACHE_RULES_H

#include <stdbool.h>
#include <stdint.h>

/*
 * One byte interval that is definitely interesting.  When accesses create
 * disjoint intervals, retain one of them instead of filling the gap: this is
 * an under-approximation, so it can miss a report but cannot invent one.
 */
typedef struct ExByteRange {
    uint8_t lo;
    uint8_t hi;
} ExByteRange;

enum {
    DC_DIRTY_DEFAULT = 0,
    DC_CLEAN,
    DC_DIRTY,
    DC_DMA_WRITTEN,
    DC_DROPPED,
    DC_NR_STATES,
};

static inline bool ex_byte_range_empty(ExByteRange r)
{
    return r.lo == r.hi;
}

static inline bool ex_byte_range_overlaps(ExByteRange r,
                                          unsigned lo, unsigned hi)
{
    return !ex_byte_range_empty(r) && lo < r.hi && hi > r.lo;
}

static inline void ex_byte_range_reset(ExByteRange *r)
{
    r->lo = 0;
    r->hi = 0;
}

static inline void ex_byte_range_add(ExByteRange *r,
                                     unsigned lo, unsigned hi)
{
    if (lo >= hi) {
        return;
    }
    if (ex_byte_range_empty(*r)) {
        r->lo = lo;
        r->hi = hi;
    } else if (lo <= r->hi && hi >= r->lo) {
        r->lo = lo < r->lo ? lo : r->lo;
        r->hi = hi > r->hi ? hi : r->hi;
    }
}

/*
 * Remove overwritten bytes.  A middle removal leaves two intervals; retain
 * the longer definite remainder (the left one on a tie).
 */
static inline void ex_byte_range_remove(ExByteRange *r,
                                        unsigned lo, unsigned hi)
{
    unsigned left, right;

    if (!ex_byte_range_overlaps(*r, lo, hi)) {
        return;
    }
    if (lo <= r->lo && hi >= r->hi) {
        ex_byte_range_reset(r);
    } else if (lo <= r->lo) {
        r->lo = hi;
    } else if (hi >= r->hi) {
        r->hi = lo;
    } else {
        left = lo - r->lo;
        right = r->hi - hi;
        if (left >= right) {
            r->hi = lo;
        } else {
            r->lo = hi;
        }
    }
}

/* Return true only when this invalidate newly discards definite dirty data. */
static inline bool ex_dcache_by_va_invalidate(uint8_t *state,
                                               ExByteRange *tracked)
{
    if (*state == DC_DROPPED) {
        return false;
    }
    if (*state == DC_DIRTY && !ex_byte_range_empty(*tracked)) {
        *state = DC_DROPPED;
        return true;
    }
    *state = DC_CLEAN;
    ex_byte_range_reset(tracked);
    return false;
}

/*
 * Exact sees each set/way instruction but does not model cache geometry, so
 * it cannot identify the physical line selected by one operation.  Ignore a
 * possible dirty-line discard.  Updating any other state can only suppress a
 * later report, which is the safe direction.
 */
static inline bool ex_dcache_uncertain_setway(uint8_t *state,
                                              ExByteRange *tracked, char kind)
{
    if (*state == DC_DROPPED ||
        (kind == 'i' && *state != DC_DMA_WRITTEN)) {
        return false;
    }
    *state = DC_CLEAN;
    ex_byte_range_reset(tracked);
    return true;
}

#endif
