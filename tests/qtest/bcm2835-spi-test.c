/*
 * QTest testcase for the BCM2835 SPI master
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "hw/ssi/bcm2835_spi.h"

/* raspi3b (BCM2837): peripheral base 0x3f000000, SPI0 at 0x7e204000 */
#define SPI_BASE    0x3f204000
#define IC_BASE     0x3f00b200
#define IRQ_PENDING_2 0x08
#define IRQ_ENABLE_2  0x14
#define SPI_IRQ_BIT  (1U << (54 - 32))

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
 * Regression test: the CLEAR_RX/CLEAR_TX bits (CS bits 4-5) are a write-only
 * one-shot control that flushes the FIFOs; they must not persist in CS. The old
 * model left them set, so a read-back after a clear showed them still set.
 */
static void test_spi_clear_autoclears(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    qtest_writel(qts, SPI_BASE + BCM2835_SPI_CS,
                 BCM2835_SPI_CLEAR_RX | BCM2835_SPI_CLEAR_TX);

    g_assert_cmphex(qtest_readl(qts, SPI_BASE + BCM2835_SPI_CS) &
                    (BCM2835_SPI_CLEAR_RX | BCM2835_SPI_CLEAR_TX), ==, 0);

    qtest_quit(qts);
}

static void test_spi_reset_clears_irq(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    qtest_writel(qts, SPI_BASE + BCM2835_SPI_CS,
                 BCM2835_SPI_CS_TA | BCM2835_SPI_CS_INTD);
    g_assert_cmphex(qtest_readl(qts, SPI_BASE + BCM2835_SPI_CS) &
                    BCM2835_SPI_CS_DONE, ==, BCM2835_SPI_CS_DONE);
    qtest_writel(qts, IC_BASE + IRQ_ENABLE_2, SPI_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, IC_BASE + IRQ_PENDING_2) & SPI_IRQ_BIT,
                    ==, SPI_IRQ_BIT);

    qtest_system_reset(qts);

    qtest_writel(qts, IC_BASE + IRQ_ENABLE_2, SPI_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, IC_BASE + IRQ_PENDING_2) & SPI_IRQ_BIT,
                    ==, 0);

    qtest_quit(qts);
}

static void test_spi_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-spi-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b -S");
    qtest_irq_intercept_out_named(src, "/machine/soc/peripherals/bcm2835-spi0",
                                  "sysbus-irq");
    qtest_writel(src, SPI_BASE + BCM2835_SPI_CS,
                 BCM2835_SPI_CS_TA | BCM2835_SPI_CS_INTD);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/bcm2835-spi0",
                                  "sysbus-irq");
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_true(qtest_get_irq(dst, 0));
    qtest_quit(dst);

    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2835/bcm2835-spi/clear_autoclears",
                   test_spi_clear_autoclears);
    qtest_add_func("/bcm2835/bcm2835-spi/reset_clears_irq",
                   test_spi_reset_clears_irq);
    qtest_add_func("/bcm2835/bcm2835-spi/migration_irq",
                   test_spi_irq_migration);
    return g_test_run();
}
