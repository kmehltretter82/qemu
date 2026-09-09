/*
 * QTest testcase for ARM11 MPCore software-generated interrupts
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define REALVIEW_GIC_DIST_BASE 0x10101000
#define REALVIEW_GIC_CPU_BASE(cpu) (0x10100200 + (cpu) * 0x100)

#define GICD_CTLR 0x000
#define GICD_ICFGR0 0xc00
#define GICD_SGIR 0xf00

#define GICC_CTLR 0x00
#define GICC_PMR 0x04
#define GICC_IAR 0x0c
#define GICC_EOIR 0x10
#define GICC_HPPIR 0x18

#define SGI_ID 1
#define SPURIOUS_IRQ 1023

static char *guest_error_log;

static void enable_cpu_interface(unsigned int cpu)
{
    writel(REALVIEW_GIC_CPU_BASE(cpu) + GICC_PMR, 0xf0);
    writel(REALVIEW_GIC_CPU_BASE(cpu) + GICC_CTLR, 1);
}

static void send_sgi_to_cpus_1_and_2(void)
{
    writel(REALVIEW_GIC_DIST_BASE + GICD_SGIR, (0x6 << 16) | SGI_ID);
}

static void test_sgi_delivery_models(void)
{
    uint32_t config;

    g_assert_cmphex(readl(REALVIEW_GIC_DIST_BASE + GICD_ICFGR0), ==,
                    0xaaaaaaaa);

    enable_cpu_interface(1);
    enable_cpu_interface(2);
    writel(REALVIEW_GIC_DIST_BASE + GICD_CTLR, 1);

    /* In N:N mode, both targets acknowledge the SGI independently. */
    send_sgi_to_cpus_1_and_2();
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE(1) + GICC_HPPIR), ==,
                    SGI_ID);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE(2) + GICC_HPPIR), ==,
                    SGI_ID);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE(1) + GICC_IAR), ==,
                    SGI_ID);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE(2) + GICC_IAR), ==,
                    SGI_ID);
    writel(REALVIEW_GIC_CPU_BASE(1) + GICC_EOIR, SGI_ID);
    writel(REALVIEW_GIC_CPU_BASE(2) + GICC_EOIR, SGI_ID);

    /* Select 1:N for SGI 1 sent by CPU0. */
    config = readl(REALVIEW_GIC_DIST_BASE + GICD_ICFGR0);
    config |= 1U << (SGI_ID * 2);
    writel(REALVIEW_GIC_DIST_BASE + GICD_ICFGR0, config);
    g_assert_cmphex(readl(REALVIEW_GIC_DIST_BASE + GICD_ICFGR0), ==,
                    config);

    /* The first acknowledge clears CPU0's SGI from every target. */
    send_sgi_to_cpus_1_and_2();
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE(1) + GICC_HPPIR), ==,
                    SGI_ID);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE(2) + GICC_HPPIR), ==,
                    SGI_ID);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE(1) + GICC_IAR), ==,
                    SGI_ID);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE(2) + GICC_HPPIR), ==,
                    SPURIOUS_IRQ);
    g_assert_cmphex(readl(REALVIEW_GIC_CPU_BASE(2) + GICC_IAR), ==,
                    SPURIOUS_IRQ);
    writel(REALVIEW_GIC_CPU_BASE(1) + GICC_EOIR, SGI_ID);
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
    log_fd = g_file_open_tmp("arm11mpcore-sgi-test-XXXXXX.log",
                             &guest_error_log, NULL);
    g_assert_cmpint(log_fd, >=, 0);
    close(log_fd);
    quoted_log = g_shell_quote(guest_error_log);
    args = g_strdup_printf("-machine realview-eb-mpcore -smp 4 "
                           "-display none -d guest_errors -D %s",
                           quoted_log);

    qtest_start(args);
    qtest_add_func("/arm11mpcore/gic/sgi-delivery-models",
                   test_sgi_delivery_models);
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
