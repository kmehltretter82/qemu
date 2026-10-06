/*
 * QTests for the local Vivante GC register stub.
 *
 * This is experimental dma-buf importer test infrastructure and is not
 * intended for submission to upstream QEMU.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define VIVANTE_BASE              0x54000000
#define VIVANTE_SECONDARY_BASE    0x54010000
#define REG_CLOCK_CONTROL         0x00
#define   CLOCK_CONTROL_IDLE      0x00030000
#define REG_IDLE_STATE            0x04
#define REG_CHIP_FEATURE          0x1c
#define   CHIP_FEATURE_PIPE_3D    0x00000004
#define REG_CHIP_MODEL            0x20
#define REG_CHIP_REV              0x24

static QTestState *vivante_start(void)
{
    return qtest_init("-machine xilinx-zynq-a9,vivante-stub=on");
}

static void test_identity(void)
{
    QTestState *qts = vivante_start();

    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_CHIP_MODEL),
                    ==, 0x2000);
    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_CHIP_REV),
                    ==, 0x5108);
    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_CHIP_FEATURE),
                    ==, CHIP_FEATURE_PIPE_3D);
    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_IDLE_STATE),
                    ==, 0x7fffffff);

    qtest_quit(qts);
}

static void test_clock_control(void)
{
    const uint32_t value = 0x012c005a;
    QTestState *qts = vivante_start();

    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_CLOCK_CONTROL),
                    ==, CLOCK_CONTROL_IDLE);

    qtest_writel(qts, VIVANTE_BASE + REG_CLOCK_CONTROL, value);
    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_CLOCK_CONTROL),
                    ==, value | CLOCK_CONTROL_IDLE);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_CLOCK_CONTROL),
                    ==, CLOCK_CONTROL_IDLE);

    qtest_quit(qts);
}

static void test_two_cores(void)
{
    const uint32_t primary = 0x012c005a;
    const uint32_t secondary = 0x021c0035;
    QTestState *qts;

    qts = qtest_init("-machine xilinx-zynq-a9,vivante-stub=on,"
                     "vivante-stub2=on");

    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_CHIP_MODEL),
                    ==, 0x2000);
    g_assert_cmphex(qtest_readl(qts, VIVANTE_SECONDARY_BASE + REG_CHIP_MODEL),
                    ==, 0x2000);

    qtest_writel(qts, VIVANTE_BASE + REG_CLOCK_CONTROL, primary);
    qtest_writel(qts, VIVANTE_SECONDARY_BASE + REG_CLOCK_CONTROL, secondary);
    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_CLOCK_CONTROL),
                    ==, primary | CLOCK_CONTROL_IDLE);
    g_assert_cmphex(qtest_readl(qts,
                               VIVANTE_SECONDARY_BASE + REG_CLOCK_CONTROL),
                    ==, secondary | CLOCK_CONTROL_IDLE);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, VIVANTE_BASE + REG_CLOCK_CONTROL),
                    ==, CLOCK_CONTROL_IDLE);
    g_assert_cmphex(qtest_readl(qts,
                               VIVANTE_SECONDARY_BASE + REG_CLOCK_CONTROL),
                    ==, CLOCK_CONTROL_IDLE);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("vivante-gc-stub/identity", test_identity);
    qtest_add_func("vivante-gc-stub/clock-control", test_clock_control);
    qtest_add_func("vivante-gc-stub/two-cores", test_two_cores);

    return g_test_run();
}
