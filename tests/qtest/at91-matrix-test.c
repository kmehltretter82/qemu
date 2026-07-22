/*
 * QTest tests for the SAM9G45 MATRIX TCM/SRAM partitioning.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define MATRIX_BASE        0xffffea00
#define MATRIX_TCMR        0x110
#define ITCM_AHB_BASE      0x00100000
#define DTCM_AHB_BASE      0x00200000
#define SRAM_AHB_BASE      0x00300000

#define TCMR_ALL_SRAM      0x00000000
#define TCMR_DTCM_64K      0x00000070
#define TCMR_ITCM_DTCM_32K 0x00000066
#define TCMR_TCM_NWS       0x00000800

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

static void test_partition_and_retention(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek");

    g_assert_cmphex(qtest_readl(qts, MATRIX_BASE + MATRIX_TCMR), ==,
                    TCMR_ALL_SRAM);
    qtest_writel(qts, SRAM_AHB_BASE + 0x100, 0x11223344);
    qtest_writel(qts, SRAM_AHB_BASE + 0x8100, 0x55667788);

    /* All four SRAM blocks become one contiguous 64 KiB DTCM. */
    qtest_writel(qts, MATRIX_BASE + MATRIX_TCMR,
                 TCMR_DTCM_64K | TCMR_TCM_NWS);
    g_assert_cmphex(qtest_readl(qts, MATRIX_BASE + MATRIX_TCMR), ==,
                    TCMR_DTCM_64K | TCMR_TCM_NWS);
    g_assert_cmphex(qtest_readl(qts, DTCM_AHB_BASE + 0x100), ==,
                    0x11223344);
    g_assert_cmphex(qtest_readl(qts, DTCM_AHB_BASE + 0x8100), ==,
                    0x55667788);
    qtest_writel(qts, DTCM_AHB_BASE + 0x200, 0xa1b2c3d4);

    /* Split SRAM A (first 32 KiB) into ITCM and SRAM B into DTCM. */
    qtest_writel(qts, MATRIX_BASE + MATRIX_TCMR, TCMR_ITCM_DTCM_32K);
    g_assert_cmphex(qtest_readl(qts, ITCM_AHB_BASE + 0x100), ==,
                    0x11223344);
    g_assert_cmphex(qtest_readl(qts, DTCM_AHB_BASE + 0x100), ==,
                    0x55667788);
    qtest_writel(qts, ITCM_AHB_BASE + 0x300, 0x0badc0de);
    qtest_writel(qts, DTCM_AHB_BASE + 0x300, 0xfeedface);

    /* Reserved size combinations are ignored rather than losing SRAM. */
    qtest_writel(qts, MATRIX_BASE + MATRIX_TCMR, 0x60);
    g_assert_cmphex(qtest_readl(qts, MATRIX_BASE + MATRIX_TCMR), ==,
                    TCMR_ITCM_DTCM_32K);
    g_assert_cmphex(qtest_readl(qts, ITCM_AHB_BASE + 0x300), ==,
                    0x0badc0de);

    /* Recombining the blocks exposes every write through ordinary SRAM. */
    qtest_writel(qts, MATRIX_BASE + MATRIX_TCMR, TCMR_ALL_SRAM);
    g_assert_cmphex(qtest_readl(qts, SRAM_AHB_BASE + 0x200), ==,
                    0xa1b2c3d4);
    g_assert_cmphex(qtest_readl(qts, SRAM_AHB_BASE + 0x300), ==,
                    0x0badc0de);
    g_assert_cmphex(qtest_readl(qts, SRAM_AHB_BASE + 0x8300), ==,
                    0xfeedface);

    qtest_quit(qts);
}

static void test_partition_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("at91-matrix-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    qtest_writel(src, MATRIX_BASE + MATRIX_TCMR, TCMR_ITCM_DTCM_32K);
    qtest_writel(src, ITCM_AHB_BASE + 0x40, 0x13579bdf);
    qtest_writel(src, DTCM_AHB_BASE + 0x40, 0x2468ace0);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    g_assert_cmphex(qtest_readl(dst, MATRIX_BASE + MATRIX_TCMR), ==,
                    TCMR_ITCM_DTCM_32K);
    g_assert_cmphex(qtest_readl(dst, ITCM_AHB_BASE + 0x40), ==,
                    0x13579bdf);
    g_assert_cmphex(qtest_readl(dst, DTCM_AHB_BASE + 0x40), ==,
                    0x2468ace0);

    qtest_writel(dst, MATRIX_BASE + MATRIX_TCMR, TCMR_ALL_SRAM);
    g_assert_cmphex(qtest_readl(dst, SRAM_AHB_BASE + 0x40), ==,
                    0x13579bdf);
    g_assert_cmphex(qtest_readl(dst, SRAM_AHB_BASE + 0x8040), ==,
                    0x2468ace0);
    qtest_quit(dst);

    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-matrix/g45/partition-and-retention",
                   test_partition_and_retention);
    qtest_add_func("/at91-matrix/g45/partition-migration",
                   test_partition_migration);

    return g_test_run();
}
