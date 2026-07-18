/*
 * QTest testcase for parallel flash with Intel command set
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

/* The first virt-machine flash is a 32-bit bank starting at address zero. */
#define CFI_QUERY_ADDR  0x154
#define CFI_Q_ADDR      0x40
#define CFI_R_ADDR      0x44
#define CFI_Y_ADDR      0x48
#define CFI_QUERY_CMD   0x00980098

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

static void assert_cfi_query_mode(QTestState *qts)
{
    g_assert_cmphex(qtest_readl(qts, CFI_Q_ADDR) & 0xff, ==, 'Q');
    g_assert_cmphex(qtest_readl(qts, CFI_R_ADDR) & 0xff, ==, 'R');
    g_assert_cmphex(qtest_readl(qts, CFI_Y_ADDR) & 0xff, ==, 'Y');
}

static void test_cfi_query_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("pflash-cfi01-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M virt -nodefaults");
    qtest_writel(src, CFI_QUERY_ADDR, CFI_QUERY_CMD);
    assert_cfi_query_mode(src);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-M virt -nodefaults -incoming %s", uri);
    assert_cfi_query_mode(dst);
    qtest_quit(dst);

    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/pflash-cfi01/migration/query-mode",
                   test_cfi_query_migration);
    return g_test_run();
}
