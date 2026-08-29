/*
 * qemu-exact: report what the guest does that the architecture leaves
 * CONSTRAINED UNPREDICTABLE, or that writes bits the architecture reserves.
 *
 * QEMU always picks *a* behaviour at these points, and the choice is usually
 * the benign one, so a kernel can rely on it here and fail on hardware that
 * chooses differently. Logging them (-d unpred) turns each silent choice into
 * a line naming the PC and the exception level, which is enough to find the
 * guest code responsible.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "internals.h"
#include "cpregs.h"

/* One report per site: a loop over a bad encoding must not fill the log. */
static GHashTable *up_sites;

static bool up_site_seen(uint64_t pc, const char *what)
{
    gpointer key = (gpointer)(uintptr_t)(pc ^ (uintptr_t)what);

    if (!up_sites) {
        up_sites = g_hash_table_new(NULL, NULL);
    }
    if (g_hash_table_contains(up_sites, key)) {
        return true;
    }
    g_hash_table_add(up_sites, key);
    return false;
}

void arm_log_unpred(int el, uint64_t pc, const char *what, const char *fmt, ...)
{
    va_list ap;
    char detail[256];

    if (!qemu_loglevel_mask(LOG_UNPRED) || up_site_seen(pc, what)) {
        return;
    }

    va_start(ap, fmt);
    vsnprintf(detail, sizeof(detail), fmt, ap);
    va_end(ap);

    qemu_log("unpred: %s at pc=0x%" PRIx64 " el%d: %s\n", what, pc, el, detail);
}

/*
 * Bits the architecture reserves in a few registers the kernel writes on every
 * boot and context switch. A guest that sets one of them is relying on this
 * implementation ignoring it; another may fault, or may have defined the bit
 * for a feature the guest has not checked for. The masks below are the RES0
 * bits that hold regardless of which features are implemented, so a report is
 * never a "you do not have that feature" false positive.
 */
typedef struct UnpredRes0 {
    uint8_t op0, op1, crn, crm, op2;
    const char *name;
    uint64_t res0;
} UnpredRes0;

static const UnpredRes0 unpred_res0[] = {
    /*
     * Deliberately conservative: only bits with no architected meaning in any
     * revision published so far. SCTLR_EL1 and TCR_EL1 are *not* here even
     * though their top bits look reserved - SCTLR_EL1[63] is TIDCP and [60] is
     * EnTP2, which Linux sets, and a table that has to track every new feature
     * bit produces false reports the first time the guest is newer than it.
     */
    /* CPACR_EL1: no fields below bit 16, none at or above 32 */
    { 3, 0, 1, 0, 2, "CPACR_EL1", MAKE_64BIT_MASK(32, 32) | MAKE_64BIT_MASK(0, 16) },
    /* CONTEXTIDR_EL1: the ASID lives in 31:0, the rest is RES0 in AArch64 */
    { 3, 0, 13, 0, 1, "CONTEXTIDR_EL1", MAKE_64BIT_MASK(32, 32) },
    /* VBAR_EL1: the vector base is 2 KiB aligned, so 10:0 must be zero */
    { 3, 0, 12, 0, 0, "VBAR_EL1", MAKE_64BIT_MASK(0, 11) },
};
void arm_check_res0(const void *rip, uint64_t value, uint64_t pc, int el)
{
    const ARMCPRegInfo *ri = rip;

    if (!qemu_loglevel_mask(LOG_UNPRED)) {
        return;
    }
    for (size_t i = 0; i < ARRAY_SIZE(unpred_res0); i++) {
        const UnpredRes0 *r = &unpred_res0[i];

        if (r->op0 == ri->opc0 && r->op1 == ri->opc1 && r->crn == ri->crn &&
            r->crm == ri->crm && r->op2 == ri->opc2 && (value & r->res0)) {
            arm_log_unpred(el, pc, "RES0 bits written",
                           "%s = 0x%" PRIx64 " sets reserved bits 0x%" PRIx64,
                           r->name, value, value & r->res0);
        }
    }
}
