/*
 * QTest testcase for the BCM2838 GPIO event detection
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

/* raspi4b (BCM2838): peripheral base 0xfe000000, GPIO at 0x7e200000 */
#define GPIO_BASE   0xfe200000

/* raspi3b (BCM2837): peripheral base 0x3f000000 */
#define RASPI3_GPIO_BASE 0x3f200000
#define RASPI3_IC_BASE   0x3f00b200

#define GPSET0      0x1c
#define GPFSEL0     0x00
#define GPLEV0      0x34
#define GPEDS0      0x40
#define GPREN0      0x4c
#define IRQ_PENDING_2 0x08
#define IRQ_ENABLE_2  0x14
#define GPIO0_IRQ_BIT (1U << (49 - 32))

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
 * Regression test: GPIO event detection (GPEDS/GPREN/...) was unimplemented.
 * With rising-edge detect enabled for a pin, driving it high must latch the
 * corresponding bit in GPEDS.
 */
static void test_gpio_rising_edge_detect(void)
{
    QTestState *qts = qtest_init("-M raspi4b");

    qtest_irq_intercept_out_named(qts, "/machine/soc/peripherals/gpio",
                                  "sysbus-irq");

    /* Enable rising-edge detection on GPIO 5. */
    qtest_writel(qts, GPIO_BASE + GPREN0, 1u << 5);
    /* Drive GPIO 5 high: a 0->1 edge. */
    qtest_writel(qts, GPIO_BASE + GPSET0, 1u << 5);

    /* The event must be latched in GPEDS0. */
    g_assert_cmphex(qtest_readl(qts, GPIO_BASE + GPEDS0) & (1u << 5), ==,
                    1u << 5);
    g_assert_true(qtest_get_irq(qts, 0));

    qtest_system_reset(qts);
    g_assert_false(qtest_get_irq(qts, 0));

    qtest_quit(qts);
}

static void test_raspi3_gpio_irq_reset(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    qtest_writel(qts, RASPI3_GPIO_BASE + GPREN0, 1U << 5);
    qtest_writel(qts, RASPI3_GPIO_BASE + GPSET0, 1U << 5);
    g_assert_cmphex(qtest_readl(qts, RASPI3_GPIO_BASE + GPEDS0) & (1U << 5),
                    ==, 1U << 5);

    qtest_writel(qts, RASPI3_IC_BASE + IRQ_ENABLE_2, GPIO0_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, RASPI3_IC_BASE + IRQ_PENDING_2) &
                    GPIO0_IRQ_BIT, ==, GPIO0_IRQ_BIT);

    qtest_system_reset(qts);

    qtest_writel(qts, RASPI3_IC_BASE + IRQ_ENABLE_2, GPIO0_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, RASPI3_IC_BASE + IRQ_PENDING_2) &
                    GPIO0_IRQ_BIT, ==, 0);

    qtest_quit(qts);
}

static void test_gpio_irq_migration(const char *machine, uint64_t gpio_base)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-gpio-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_initf("-M %s -S", machine);
    qtest_irq_intercept_out_named(src, "/machine/soc/peripherals/gpio",
                                  "sysbus-irq");
    qtest_writel(src, gpio_base + GPREN0, 1U << 5);
    qtest_writel(src, gpio_base + GPSET0, 1U << 5);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-M %s -S -incoming defer", machine);
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/gpio",
                                  "sysbus-irq");
    g_assert_false(qtest_get_irq(dst, 0));
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, gpio_base + GPEDS0) & (1U << 5), ==,
                    1U << 5);
    g_assert_true(qtest_get_irq(dst, 0));
    qtest_quit(dst);

    unlink(state_path);
}

static void test_bcm2838_gpio_irq_migration(void)
{
    test_gpio_irq_migration("raspi4b", GPIO_BASE);
}

static void test_bcm2835_gpio_irq_migration(void)
{
    test_gpio_irq_migration("raspi3b", RASPI3_GPIO_BASE);
}

static void test_gpio_output_migration(const char *machine, uint64_t gpio_base)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-gpio-output-migration-XXXXXX", &state_path,
                         NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_initf("-M %s -S", machine);
    qtest_irq_intercept_out(src, "/machine/soc/peripherals/gpio");
    qtest_writel(src, gpio_base + GPFSEL0, 1U << (3 * 5));
    qtest_writel(src, gpio_base + GPSET0, 1U << 5);
    g_assert_true(qtest_get_irq(src, 5));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-M %s -S -incoming defer", machine);
    qtest_irq_intercept_out(dst, "/machine/soc/peripherals/gpio");
    g_assert_false(qtest_get_irq(dst, 5));
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, gpio_base + GPFSEL0) &
                    (7U << (3 * 5)), ==, 1U << (3 * 5));
    g_assert_cmphex(qtest_readl(dst, gpio_base + GPLEV0) & (1U << 5), ==,
                    1U << 5);
    g_assert_true(qtest_get_irq(dst, 5));
    qtest_quit(dst);

    unlink(state_path);
}

static void test_bcm2838_gpio_output_migration(void)
{
    test_gpio_output_migration("raspi4b", GPIO_BASE);
}

static void test_bcm2835_gpio_output_migration(void)
{
    test_gpio_output_migration("raspi3b", RASPI3_GPIO_BASE);
}

static void test_gpio_output_reset(const char *machine, uint64_t gpio_base)
{
    QTestState *qts = qtest_initf("-M %s", machine);

    qtest_irq_intercept_out(qts, "/machine/soc/peripherals/gpio");
    qtest_writel(qts, gpio_base + GPFSEL0, 1U << (3 * 5));
    qtest_writel(qts, gpio_base + GPSET0, 1U << 5);
    g_assert_true(qtest_get_irq(qts, 5));

    qtest_system_reset(qts);
    g_assert_false(qtest_get_irq(qts, 5));

    qtest_quit(qts);
}

static void test_bcm2838_gpio_output_reset(void)
{
    test_gpio_output_reset("raspi4b", GPIO_BASE);
}

static void test_bcm2835_gpio_output_reset(void)
{
    test_gpio_output_reset("raspi3b", RASPI3_GPIO_BASE);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2838/bcm2838-gpio/rising_edge_detect",
                   test_gpio_rising_edge_detect);
    qtest_add_func("/bcm2835/bcm2835-gpio/irq_reset",
                   test_raspi3_gpio_irq_reset);
    qtest_add_func("/bcm2838/bcm2838-gpio/migration_irq",
                   test_bcm2838_gpio_irq_migration);
    qtest_add_func("/bcm2835/bcm2835-gpio/migration_irq",
                   test_bcm2835_gpio_irq_migration);
    qtest_add_func("/bcm2838/bcm2838-gpio/migration_output",
                   test_bcm2838_gpio_output_migration);
    qtest_add_func("/bcm2835/bcm2835-gpio/migration_output",
                   test_bcm2835_gpio_output_migration);
    qtest_add_func("/bcm2838/bcm2838-gpio/output_reset",
                   test_bcm2838_gpio_output_reset);
    qtest_add_func("/bcm2835/bcm2835-gpio/output_reset",
                   test_bcm2835_gpio_output_reset);
    return g_test_run();
}
