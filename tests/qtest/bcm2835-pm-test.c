/*
 * QTest testcase for the BCM2835 power management watchdog
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

/* raspi3b (BCM2837): peripheral base 0x3f000000, PM at 0x7e100000 */
#define PM_BASE     0x3f100000

#define PASSWORD    0x5a000000u
#define R_RSTC      0x1c
#define R_RSTS      0x20
#define R_WDOG      0x24
#define RSTC_RESET  0x20

/*
 * Regression test: arming the watchdog (WDOG countdown, RSTC full-reset) used
 * to reset the machine immediately, ignoring the countdown. With the fix the
 * reset is deferred (armed for the countdown), so state set just before arming
 * survives. Use a distinct RSTS partition as the marker; an immediate reset
 * would restore its reset value.
 */
static void test_pm_watchdog_deferred(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    qtest_writel(qts, PM_BASE + R_RSTS, PASSWORD | 0x220);
    qtest_writel(qts, PM_BASE + R_WDOG, PASSWORD | 1000);
    qtest_writel(qts, PM_BASE + R_RSTC, PASSWORD | RSTC_RESET);

    /* Reset must be deferred, not immediate: the RSTS marker survives. */
    g_assert_cmphex(qtest_readl(qts, PM_BASE + R_RSTS) & 0xfff, ==, 0x220);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2835/bcm2835-pm/watchdog_deferred",
                   test_pm_watchdog_deferred);
    return g_test_run();
}
