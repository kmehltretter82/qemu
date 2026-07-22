/*
 * QTest tests for the AT91 PMC master clock and timer consumers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define G45_PMC_BASE       0xfffffc00
#define G45_PIT_BASE       0xfffffd30
#define G45_TCB0_BASE      0xfff7c000

#define G35_PMC_BASE       0xfffffc00
#define G35_TCB0_BASE      0xf8008000

#define PMC_MCFR           0x24
#define PMC_PLLAR          0x28
#define PMC_MCKR           0x30

#define PMC_PLLAR_792MHZ   0x00410001
#define PMC_PLLAR_396MHZ   0x00200001
#define G45_MCKR_132MHZ    0x00000306
#define G35_MCKR_132MHZ    0x00000312
#define MCKR_MAINCK        0x00000001

#define PIT_MR             0x00
#define PIT_SR             0x04
#define PIT_PIIR           0x0c
#define PIT_MR_PITEN       (1u << 24)

#define TC_CCR             0x00
#define TC_CMR             0x04
#define TC_CV              0x10
#define TC_CCR_CLKEN       (1u << 0)
#define TC_CCR_SWTRG       (1u << 2)

static void wait_for_migration_complete(QTestState *qts)
{
    while (true) {
        QDict *response = qtest_qmp(qts,
                                    "{ 'execute': 'query-migrate' }");
        QDict *result = qdict_get_qdict(response, "return");
        const char *status = qdict_get_str(result, "status");
        bool complete = !strcmp(status, "completed");

        g_assert_cmpstr(status, !=, "failed");
        g_assert_cmpstr(status, !=, "cancelled");
        qobject_unref(response);
        if (complete) {
            return;
        }
        g_usleep(1000);
    }
}

static void start_tc_mck_div2(QTestState *qts, uint64_t base)
{
    qtest_writel(qts, base + TC_CMR, 0);
    qtest_writel(qts, base + TC_CCR, TC_CCR_CLKEN | TC_CCR_SWTRG);
}

static void test_g45_master_clock(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek");

    g_assert_cmphex(qtest_readl(qts, G45_PMC_BASE + PMC_PLLAR), ==,
                    PMC_PLLAR_792MHZ);
    g_assert_cmphex(qtest_readl(qts, G45_PMC_BASE + PMC_MCKR), ==,
                    G45_MCKR_132MHZ);
    g_assert_cmphex(qtest_readl(qts, G45_PMC_BASE + PMC_MCFR), ==,
                    0x000116e3);

    start_tc_mck_div2(qts, G45_TCB0_BASE);
    qtest_clock_step(qts, 1000);
    g_assert_cmpuint(qtest_readl(qts, G45_TCB0_BASE + TC_CV), ==, 66);

    /* Switching to 12 MHz MAINCK preserves CV, then counts at 6 MHz. */
    qtest_writel(qts, G45_PMC_BASE + PMC_MCKR, MCKR_MAINCK);
    g_assert_cmpuint(qtest_readl(qts, G45_TCB0_BASE + TC_CV), ==, 66);
    qtest_clock_step(qts, 1000);
    g_assert_cmpuint(qtest_readl(qts, G45_TCB0_BASE + TC_CV), ==, 72);

    /* Reprogram PLLA to 396 MHz while on MAINCK, then select 66 MHz MCK. */
    qtest_writel(qts, G45_PMC_BASE + PMC_PLLAR, PMC_PLLAR_396MHZ);
    qtest_writel(qts, G45_PMC_BASE + PMC_MCKR, G45_MCKR_132MHZ);
    g_assert_cmpuint(qtest_readl(qts, G45_TCB0_BASE + TC_CV), ==, 72);
    qtest_clock_step(qts, 1000);
    g_assert_cmpuint(qtest_readl(qts, G45_TCB0_BASE + TC_CV), ==, 105);

    qtest_quit(qts);
}

static void test_g45_pit_clock_change(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek");
    const uint32_t piv = 8249;

    /* 8250 ticks are 1 ms at the reset PIT clock (132 MHz / 16). */
    qtest_writel(qts, G45_PIT_BASE + PIT_MR, PIT_MR_PITEN | piv);
    qtest_clock_step(qts, 500000);
    g_assert_cmpuint(qtest_readl(qts, G45_PIT_BASE + PIT_PIIR) & 0xfffff,
                     ==, 4125);

    /* At 12 MHz MCK, the remaining 4125 ticks take 5.5 ms. */
    qtest_writel(qts, G45_PMC_BASE + PMC_MCKR, MCKR_MAINCK);
    g_assert_cmpuint(qtest_readl(qts, G45_PIT_BASE + PIT_PIIR) & 0xfffff,
                     ==, 4125);
    qtest_clock_step(qts, 500000);
    g_assert_cmphex(qtest_readl(qts, G45_PIT_BASE + PIT_SR), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, G45_PIT_BASE + PIT_PIIR) & 0xfffff,
                     ==, 4500);
    qtest_clock_step(qts, 4999999);
    g_assert_cmphex(qtest_readl(qts, G45_PIT_BASE + PIT_SR), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_PIT_BASE + PIT_SR), ==, 1);

    qtest_quit(qts);
}

static void test_g45_programmed_clock_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("at91-pmc-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    start_tc_mck_div2(src, G45_TCB0_BASE);
    qtest_clock_step(src, 1000);
    g_assert_cmpuint(qtest_readl(src, G45_TCB0_BASE + TC_CV), ==, 66);

    qtest_writel(src, G45_PMC_BASE + PMC_MCKR, MCKR_MAINCK);
    qtest_clock_step(src, 1000);
    g_assert_cmpuint(qtest_readl(src, G45_TCB0_BASE + TC_CV), ==, 72);

    /* Keep a PIT interval part-way complete across the same migration. */
    qtest_writel(src, G45_PIT_BASE + PIT_MR, PIT_MR_PITEN | 749);
    qtest_clock_step(src, 400000);
    g_assert_cmpuint(qtest_readl(src, G45_TCB0_BASE + TC_CV), ==, 2472);
    g_assert_cmpuint(qtest_readl(src, G45_PIT_BASE + PIT_PIIR) & 0xfffff,
                     ==, 300);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    g_assert_cmphex(qtest_readl(dst, G45_PMC_BASE + PMC_MCKR), ==,
                    MCKR_MAINCK);
    g_assert_cmpuint(qtest_readl(dst, G45_TCB0_BASE + TC_CV), ==, 2472);
    g_assert_cmpuint(qtest_readl(dst, G45_PIT_BASE + PIT_PIIR) & 0xfffff,
                     ==, 300);
    qtest_clock_step(dst, 100000);
    g_assert_cmpuint(qtest_readl(dst, G45_TCB0_BASE + TC_CV), ==, 3072);
    g_assert_cmpuint(qtest_readl(dst, G45_PIT_BASE + PIT_PIIR) & 0xfffff,
                     ==, 375);
    qtest_quit(dst);

    unlink(state_path);
}

static void test_g35_master_clock_layout(void)
{
    QTestState *qts = qtest_init("-machine sam9g35ek");

    g_assert_cmphex(qtest_readl(qts, G35_PMC_BASE + PMC_MCKR), ==,
                    G35_MCKR_132MHZ);
    start_tc_mck_div2(qts, G35_TCB0_BASE);
    qtest_clock_step(qts, 1000);
    g_assert_cmpuint(qtest_readl(qts, G35_TCB0_BASE + TC_CV), ==, 66);

    qtest_writel(qts, G35_PMC_BASE + PMC_MCKR, MCKR_MAINCK);
    g_assert_cmpuint(qtest_readl(qts, G35_TCB0_BASE + TC_CV), ==, 66);
    qtest_clock_step(qts, 1000);
    g_assert_cmpuint(qtest_readl(qts, G35_TCB0_BASE + TC_CV), ==, 72);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-pmc/g45/master-clock", test_g45_master_clock);
    qtest_add_func("/at91-pmc/g45/pit-clock-change",
                   test_g45_pit_clock_change);
    qtest_add_func("/at91-pmc/g45/programmed-clock-migration",
                   test_g45_programmed_clock_migration);
    qtest_add_func("/at91-pmc/g35/master-clock-layout",
                   test_g35_master_clock_layout);

    return g_test_run();
}
