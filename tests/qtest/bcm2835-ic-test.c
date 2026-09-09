/*
 * QTest testcase for the BCM2835/BCM2838 ARMCTRL interrupt controller
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define IRQ_PENDING_BASIC   0x00
#define IRQ_PENDING_2       0x08
#define FIQ_CONTROL         0x0c
#define IRQ_ENABLE_1        0x10
#define IRQ_ENABLE_2        0x14
#define IRQ_DISABLE_1       0x1c

/* raspi3b (BCM2837): peripheral base 0x3f000000 */
#define RASPI3_IC_BASE      0x3f00b200
#define RASPI3_SYSTMR_BASE  0x3f003000
/* raspi4b (BCM2838): peripheral base 0xfe000000 */
#define RASPI4_IC_BASE      0xfe00b200
#define RASPI3_MPHI_BASE    0x3f006000
#define MPHI_SWIRQ_SET      0x1f0
#define MPHI_IRQ_BIT        0x1
#define SYSTMR_CS           0x00
#define SYSTMR_CLO          0x04
#define SYSTMR_C0           0x0c

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

/*
 * Regression test: FIQ_CONTROL takes a 7-bit source (0-127) but only 0-71 are
 * valid (64 GPU + 8 ARM). A source >= 96 used to feed a shift of 32 to
 * extract32() and abort QEMU on a single guest MMIO write. Check the controller
 * survives it. Uses raspi3b, where the legacy IC is directly reachable.
 */
static void test_ic_fiq_out_of_range(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    /* Enable FIQ (bit 7) with source 96 (0x60): out of range. */
    qtest_writel(qts, RASPI3_IC_BASE + FIQ_CONTROL, 0x80 | 96);

    /* If QEMU did not abort we can still read a register. */
    qtest_readl(qts, RASPI3_IC_BASE + IRQ_PENDING_BASIC);

    qtest_quit(qts);
}

/*
 * Regression test: on BCM2838 (raspi4b) the MPHI region used to be mapped on
 * top of the legacy interrupt controller at 0xb200, so guest reads of the IC
 * returned the MPHI's 0 instead of the controller state. IRQ_DISABLE_1 reads
 * ~gpu_irq_enable, which is 0xffffffff at reset when the IC is reachable.
 */
static void test_ic_reachable_on_raspi4b(void)
{
    QTestState *qts = qtest_init("-M raspi4b");

    g_assert_cmphex(qtest_readl(qts, RASPI4_IC_BASE + IRQ_DISABLE_1), ==,
                    0xffffffff);

    qtest_quit(qts);
}

static void test_mphi_reset_clears_irq(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    qtest_writel(qts, RASPI3_MPHI_BASE + MPHI_SWIRQ_SET, 1);
    qtest_writel(qts, RASPI3_IC_BASE + IRQ_ENABLE_2, MPHI_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, RASPI3_IC_BASE + IRQ_PENDING_2) &
                    MPHI_IRQ_BIT, ==, MPHI_IRQ_BIT);

    qtest_system_reset(qts);

    qtest_writel(qts, RASPI3_IC_BASE + IRQ_ENABLE_2, MPHI_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, RASPI3_IC_BASE + IRQ_PENDING_2) &
                    MPHI_IRQ_BIT, ==, 0);

    qtest_quit(qts);
}

static void test_mphi_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-mphi-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b -S");
    qtest_irq_intercept_out_named(src, "/machine/soc/peripherals/mphi",
                                  "sysbus-irq");
    qtest_writel(src, RASPI3_MPHI_BASE + MPHI_SWIRQ_SET, 1);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/mphi",
                                  "sysbus-irq");
    g_assert_false(qtest_get_irq(dst, 0));
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_true(qtest_get_irq(dst, 0));
    qtest_quit(dst);

    unlink(state_path);
}

static void test_systmr_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    uint32_t counter;
    int fd;

    fd = g_file_open_tmp("bcm2835-systmr-migration-XXXXXX", &state_path,
                         NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b");
    qtest_irq_intercept_out_named(src, "/machine/soc/peripherals/systimer",
                                  "sysbus-irq");
    counter = qtest_readl(src, RASPI3_SYSTMR_BASE + SYSTMR_CLO);
    qtest_writel(src, RASPI3_SYSTMR_BASE + SYSTMR_C0, counter + 10);
    qtest_clock_step(src, 20 * 1000);
    g_assert_cmphex(qtest_readl(src, RASPI3_SYSTMR_BASE + SYSTMR_CS) & 1, ==,
                    1);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/systimer",
                                  "sysbus-irq");
    g_assert_false(qtest_get_irq(dst, 0));
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, RASPI3_SYSTMR_BASE + SYSTMR_CS) & 1, ==,
                    1);
    g_assert_true(qtest_get_irq(dst, 0));
    qtest_quit(dst);

    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2835/bcm2835-ic/fiq_out_of_range",
                   test_ic_fiq_out_of_range);
    qtest_add_func("/bcm2835/bcm2838-ic/reachable_on_raspi4b",
                   test_ic_reachable_on_raspi4b);
    qtest_add_func("/bcm2835/bcm2835-mphi/reset_clears_irq",
                   test_mphi_reset_clears_irq);
    qtest_add_func("/bcm2835/bcm2835-mphi/migration_irq",
                   test_mphi_irq_migration);
    qtest_add_func("/bcm2835/bcm2835-systmr/migration_irq",
                   test_systmr_irq_migration);

    return g_test_run();
}
