/*
 * QTest testcase for the BCM2835 SDHost controller
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

/* raspi3b (BCM2837): peripheral base 0x3f000000, SDHost at 0x7e202000 */
#define SDHOST_BASE     0x3f202000

#define SDCMD   0x00
#define SDARG   0x04
#define SDTOUT  0x08
#define SDCDIV  0x0c
#define SDHSTS  0x20
#define SDVDD   0x30
#define SDHCFG  0x38

#define SDCMD_NEW_FLAG          0x8000
#define SDCMD_BUSYWAIT           0x0800
#define SDCMD_NO_RESPONSE        0x0400
#define SDHSTS_CMD_TIME_OUT     0x40
#define SDHSTS_BUSY_IRPT         0x400
#define SDHCFG_BUSY_IRPT_EN      (1 << 10)

#define IC_BASE                  0x3f00b200
#define IRQ_PENDING_2            0x08
#define IRQ_ENABLE_2             0x14
#define SDHOST_IRQ_BIT           (1 << (56 - 32))

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
 * Regression test: SDARG, SDTOUT and SDCDIV are R/W registers, but the read
 * handler had no case for them, so a driver that programmed them read back 0.
 * Check they round-trip.
 */
static void test_sdhost_regs_readable(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    qtest_writel(qts, SDHOST_BASE + SDARG, 0x12345678);
    qtest_writel(qts, SDHOST_BASE + SDTOUT, 0x0000cafe);
    qtest_writel(qts, SDHOST_BASE + SDCDIV, 0x00000123);

    g_assert_cmphex(qtest_readl(qts, SDHOST_BASE + SDARG), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, SDHOST_BASE + SDTOUT), ==, 0x0000cafe);
    g_assert_cmphex(qtest_readl(qts, SDHOST_BASE + SDCDIV), ==, 0x00000123);

    qtest_quit(qts);
}

static void test_sdhost_reset_clears_state(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    /* No card is attached, so a command produces CMD_TIME_OUT. */
    qtest_writel(qts, SDHOST_BASE + SDVDD, 1);
    qtest_writel(qts, SDHOST_BASE + SDCMD, SDCMD_NEW_FLAG);
    g_assert_cmphex(qtest_readl(qts, SDHOST_BASE + SDHSTS) &
                    SDHSTS_CMD_TIME_OUT, ==, SDHSTS_CMD_TIME_OUT);

    /* A no-response busywait command raises the SDHost's interrupt. */
    qtest_writel(qts, SDHOST_BASE + SDHCFG, SDHCFG_BUSY_IRPT_EN);
    qtest_writel(qts, SDHOST_BASE + SDCMD,
                 SDCMD_NEW_FLAG | SDCMD_BUSYWAIT | SDCMD_NO_RESPONSE);
    g_assert_cmphex(qtest_readl(qts, SDHOST_BASE + SDHSTS) &
                    SDHSTS_BUSY_IRPT, ==, SDHSTS_BUSY_IRPT);
    qtest_writel(qts, IC_BASE + IRQ_ENABLE_2, SDHOST_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, IC_BASE + IRQ_PENDING_2) &
                    SDHOST_IRQ_BIT, ==, SDHOST_IRQ_BIT);

    qtest_system_reset(qts);
    qtest_writel(qts, IC_BASE + IRQ_ENABLE_2, SDHOST_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, IC_BASE + IRQ_PENDING_2) &
                    SDHOST_IRQ_BIT, ==, 0);
    g_assert_cmphex(qtest_readl(qts, SDHOST_BASE + SDHSTS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SDHOST_BASE + SDVDD), ==, 0);

    qtest_quit(qts);
}

static void test_sdhost_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-sdhost-migration-XXXXXX", &state_path,
                         NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b -S");
    qtest_irq_intercept_out_named(src, "/machine/soc/peripherals/sdhost",
                                  "sysbus-irq");
    qtest_writel(src, SDHOST_BASE + SDHCFG, SDHCFG_BUSY_IRPT_EN);
    qtest_writel(src, SDHOST_BASE + SDCMD,
                 SDCMD_NEW_FLAG | SDCMD_BUSYWAIT | SDCMD_NO_RESPONSE);
    g_assert_cmphex(qtest_readl(src, SDHOST_BASE + SDHSTS) &
                    SDHSTS_BUSY_IRPT, ==, SDHSTS_BUSY_IRPT);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/sdhost",
                                  "sysbus-irq");
    g_assert_false(qtest_get_irq(dst, 0));
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, SDHOST_BASE + SDHSTS) &
                    SDHSTS_BUSY_IRPT, ==, SDHSTS_BUSY_IRPT);
    g_assert_true(qtest_get_irq(dst, 0));
    qtest_quit(dst);

    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2835/bcm2835-sdhost/regs_readable",
                   test_sdhost_regs_readable);
    qtest_add_func("/bcm2835/bcm2835-sdhost/reset_clears_state",
                   test_sdhost_reset_clears_state);
    qtest_add_func("/bcm2835/bcm2835-sdhost/migration_irq",
                   test_sdhost_irq_migration);
    return g_test_run();
}
