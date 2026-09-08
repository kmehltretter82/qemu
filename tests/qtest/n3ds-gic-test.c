/*
 * QTest testcase for the Nintendo 3DS ARM11 MPCore GIC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define N3DS_GIC_DIST_BASE 0x17e01000

static char *guest_error_log;

static void test_active_registers(void)
{
    g_autofree char *contents = NULL;
    gsize length;
    GError *error = NULL;

    /* The implemented active registers are read-only. */
    g_assert_cmphex(readl(N3DS_GIC_DIST_BASE + 0x300), ==, 0);
    writel(N3DS_GIC_DIST_BASE + 0x300, 0xffffffff);
    g_assert_cmphex(readl(N3DS_GIC_DIST_BASE + 0x300), ==, 0);

    /* The rest of the active-register range is reserved and RAZ/WI. */
    writel(N3DS_GIC_DIST_BASE + 0x320, 0xffffffff);
    g_assert_cmphex(readl(N3DS_GIC_DIST_BASE + 0x320), ==, 0);
    writel(N3DS_GIC_DIST_BASE + 0x380, 0xffffffff);
    g_assert_cmphex(readl(N3DS_GIC_DIST_BASE + 0x380), ==, 0);
    writel(N3DS_GIC_DIST_BASE + 0x3fc, 0xffffffff);
    g_assert_cmphex(readl(N3DS_GIC_DIST_BASE + 0x3fc), ==, 0);

    /* Stop QEMU to flush its -D output before checking the log. */
    qtest_end();
    g_assert_true(g_file_get_contents(guest_error_log, &contents, &length,
                                      &error));
    g_assert_no_error(error);
    g_assert_cmpuint(length, ==, 0);
}

int main(int argc, char **argv)
{
    g_autofree char *args = NULL;
    g_autofree char *quoted_log = NULL;
    int log_fd;
    int ret;

    g_test_init(&argc, &argv, NULL);
    log_fd = g_file_open_tmp("n3ds-gic-test-XXXXXX.log", &guest_error_log,
                             NULL);
    g_assert_cmpint(log_fd, >=, 0);
    close(log_fd);
    quoted_log = g_shell_quote(guest_error_log);
    args = g_strdup_printf("-machine n3ds -display none "
                           "-d guest_errors -D %s", quoted_log);

    qtest_start(args);
    qtest_add_func("/n3ds/gic/active-registers", test_active_registers);
    ret = g_test_run();

    unlink(guest_error_log);
    g_free(guest_error_log);
    return ret;
}
