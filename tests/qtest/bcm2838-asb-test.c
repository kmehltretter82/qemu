/*
 * QTest testcase for the BCM2838 async AXI bridge (ASB) identifier
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

/* raspi4b (BCM2838): ASB at 0x7e00a000, RPiVid ASB at 0x7ec11000 */
#define ASB_BASE        0xfe00a000
#define RPIVID_ASB_BASE 0xfec11000
#define ASB_AXI_BRDG_ID 0x20
#define BRDG_MAGIC      0x62726467 /* "brdg" */

/*
 * Regression test: the Linux bcm2835-power driver gates all power-domain
 * registration on reading the ASCII "brdg" identifier from both ASB windows.
 * The unimplemented stubs read 0, so no power domain registered on raspi4b.
 */
static void test_asb_bridge_id(void)
{
    QTestState *qts = qtest_init("-M raspi4b");

    g_assert_cmphex(qtest_readl(qts, ASB_BASE + ASB_AXI_BRDG_ID), ==,
                    BRDG_MAGIC);
    g_assert_cmphex(qtest_readl(qts, RPIVID_ASB_BASE + ASB_AXI_BRDG_ID), ==,
                    BRDG_MAGIC);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2838/bcm2838-asb/bridge_id", test_asb_bridge_id);
    return g_test_run();
}
