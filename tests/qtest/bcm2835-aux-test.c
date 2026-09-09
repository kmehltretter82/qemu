/*
 * QTest testcase for the BCM2835 AUX interrupt output
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define RASPI3_AUX_BASE     0x3f215000
#define AUX_MU_IER           0x44
#define AUX_MU_IER_TX_INT    0x2

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

static void test_aux_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-aux-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b -S");
    qtest_irq_intercept_out_named(src, "/machine/soc/peripherals/aux",
                                  "sysbus-irq");
    qtest_writel(src, RASPI3_AUX_BASE + AUX_MU_IER, AUX_MU_IER_TX_INT);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/aux",
                                  "sysbus-irq");
    g_assert_false(qtest_get_irq(dst, 0));
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, RASPI3_AUX_BASE + AUX_MU_IER) &
                    AUX_MU_IER_TX_INT, ==, AUX_MU_IER_TX_INT);
    g_assert_true(qtest_get_irq(dst, 0));
    qtest_quit(dst);

    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2835/bcm2835-aux/migration_irq",
                   test_aux_irq_migration);
    return g_test_run();
}
