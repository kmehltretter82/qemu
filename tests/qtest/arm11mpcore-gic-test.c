/*
 * QTest testcase for the ARM11 MPCore GIC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define REALVIEW_GIC_CPU_BASE 0x10100100
#define REALVIEW_GIC_DIST_BASE 0x10101000

#define GICC_BPR 0x08
#define GICC_RPR 0x14

static char *guest_error_log;

static void test_cpu_priority_registers(void)
{
    /* ARM11 MPCore implements four priority bits. */
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE + GICC_BPR), ==, 0x03);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE + GICC_RPR), ==, 0xf0);

    /* BPR values below the implemented minimum behave as the minimum. */
    writel(REALVIEW_GIC_CPU_BASE + GICC_BPR, 0);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE + GICC_BPR), ==, 0x03);
    writel(REALVIEW_GIC_CPU_BASE + GICC_BPR, 7);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE + GICC_BPR), ==, 0x07);
}

static void test_active_registers(void)
{
    /* The implemented active registers are read-only. */
    g_assert_cmphex(readl(REALVIEW_GIC_DIST_BASE + 0x300), ==, 0);
    writel(REALVIEW_GIC_DIST_BASE + 0x300, 0xffffffff);
    g_assert_cmphex(readl(REALVIEW_GIC_DIST_BASE + 0x300), ==, 0);

    /* The rest of the active-register range is reserved and RAZ/WI. */
    writel(REALVIEW_GIC_DIST_BASE + 0x320, 0xffffffff);
    g_assert_cmphex(readl(REALVIEW_GIC_DIST_BASE + 0x320), ==, 0);
    writel(REALVIEW_GIC_DIST_BASE + 0x380, 0xffffffff);
    g_assert_cmphex(readl(REALVIEW_GIC_DIST_BASE + 0x380), ==, 0);
    writel(REALVIEW_GIC_DIST_BASE + 0x3fc, 0xffffffff);
    g_assert_cmphex(readl(REALVIEW_GIC_DIST_BASE + 0x3fc), ==, 0);
}

int main(int argc, char **argv)
{
    g_autofree char *args = NULL;
    g_autofree char *contents = NULL;
    g_autofree char *quoted_log = NULL;
    gsize length;
    GError *error = NULL;
    int log_fd;
    int ret;

    g_test_init(&argc, &argv, NULL);
    log_fd = g_file_open_tmp("arm11mpcore-gic-test-XXXXXX.log",
                             &guest_error_log, NULL);
    g_assert_cmpint(log_fd, >=, 0);
    close(log_fd);
    quoted_log = g_shell_quote(guest_error_log);
    args = g_strdup_printf("-machine realview-eb-mpcore -display none "
                           "-d guest_errors -D %s", quoted_log);

    qtest_start(args);
    qtest_add_func("/arm11mpcore/gic/cpu-priority-registers",
                   test_cpu_priority_registers);
    qtest_add_func("/arm11mpcore/gic/active-registers",
                   test_active_registers);
    ret = g_test_run();

    /* Stop QEMU to flush its -D output before checking the log. */
    qtest_end();
    g_assert_true(g_file_get_contents(guest_error_log, &contents, &length,
                                      &error));
    g_assert_no_error(error);
    g_assert_cmpuint(length, ==, 0);

    unlink(guest_error_log);
    g_free(guest_error_log);
    return ret;
}
