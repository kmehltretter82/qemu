/*
 * QTest tests for the AT91 USART/DBGU transmit paths.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define G45_DBGU_BASE          0xffffee00
#define G45_SDRAM_BASE         0x70000000

#define US_CSR                 0x14
#define US_THR                 0x1c
#define US_TPR                 0x108
#define US_TCR                 0x10c
#define US_PTCR                0x120

#define US_TXRDY               (1u << 1)
#define US_ENDTX               (1u << 4)
#define US_TXEMPTY             (1u << 9)
#define PTCR_TXTEN             (1u << 8)

/* Large enough to exceed a socket chardev's finite nonblocking output queue. */
#define PDC_TEST_SIZE          65535
#define PDC_TEST_ADDR          (G45_SDRAM_BASE + 0x100000)

static void receive_exact(int fd, uint8_t *buffer, size_t length)
{
    size_t received = 0;

    while (received < length) {
        GPollFD pollfd = {
            .fd = fd,
            .events = G_IO_IN | G_IO_HUP | G_IO_ERR,
        };
        int ready = g_poll(&pollfd, 1, 5000);
        ssize_t ret;

        g_assert_cmpint(ready, >, 0);
        /* HUP may accompany the final readable bytes after a source exits. */
        g_assert_false(pollfd.revents & G_IO_ERR);
        ret = recv(fd, buffer + received, length - received, 0);
        g_assert_cmpint(ret, >, 0);
        received += ret;
    }
}

static void test_thr_character(void)
{
    QTestState *qts;
    uint8_t byte;
    int sock_fd;

    qts = qtest_init_with_serial("-machine sam9m10g45ek -S", &sock_fd);

    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_CSR) &
                    (US_TXRDY | US_TXEMPTY), ==, US_TXRDY | US_TXEMPTY);
    qtest_writel(qts, G45_DBGU_BASE + US_THR, 'G');
    receive_exact(sock_fd, &byte, 1);
    g_assert_cmphex(byte, ==, 'G');
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_CSR) &
                    (US_TXRDY | US_TXEMPTY), ==, US_TXRDY | US_TXEMPTY);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_pdc_backpressure(void)
{
    g_autofree uint8_t *expected = g_malloc(PDC_TEST_SIZE);
    g_autofree uint8_t *actual = g_malloc(PDC_TEST_SIZE);
    QTestState *qts;
    uint32_t status;
    int sock_fd;
    size_t i;

    for (i = 0; i < PDC_TEST_SIZE; i++) {
        expected[i] = (i * 37u + (i >> 8) + 0x45u) & 0xff;
    }

    qts = qtest_init_with_serial("-machine sam9m10g45ek -S", &sock_fd);
    qtest_bufwrite(qts, PDC_TEST_ADDR, expected, PDC_TEST_SIZE);
    qtest_writel(qts, G45_DBGU_BASE + US_TPR, PDC_TEST_ADDR);
    qtest_writel(qts, G45_DBGU_BASE + US_TCR, PDC_TEST_SIZE);
    qtest_writel(qts, G45_DBGU_BASE + US_PTCR, PTCR_TXTEN);

    /*
     * The test deliberately does not read the socket until here.  A correct
     * nonblocking implementation must retain a suffix when the chardev fills;
     * the old implementation discarded that suffix and reported TCR == 0.
     */
    g_assert_cmpuint(qtest_readl(qts, G45_DBGU_BASE + US_TCR), >, 0);
    status = qtest_readl(qts, G45_DBGU_BASE + US_CSR);
    g_assert_cmphex(status & (US_ENDTX | US_TXEMPTY), ==, 0);

    receive_exact(sock_fd, actual, PDC_TEST_SIZE);
    g_assert_cmpmem(actual, PDC_TEST_SIZE, expected, PDC_TEST_SIZE);

    for (i = 0; i < 1000; i++) {
        if (qtest_readl(qts, G45_DBGU_BASE + US_TCR) == 0) {
            break;
        }
        g_usleep(1000);
    }
    g_assert_cmpuint(i, <, 1000);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_TPR), ==,
                    PDC_TEST_ADDR + PDC_TEST_SIZE);
    status = qtest_readl(qts, G45_DBGU_BASE + US_CSR);
    g_assert_cmphex(status & (US_TXRDY | US_ENDTX | US_TXEMPTY), ==,
                    US_TXRDY | US_ENDTX | US_TXEMPTY);

    close(sock_fd);
    qtest_quit(qts);
}

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

static void test_pdc_backpressure_migration(void)
{
    g_autofree uint8_t *expected = g_malloc(PDC_TEST_SIZE);
    g_autofree uint8_t *actual = g_malloc(PDC_TEST_SIZE);
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *destination_args = NULL;
    QTestState *src, *dst;
    uint32_t remaining;
    size_t prefix;
    size_t i;
    int src_fd, dst_fd, state_fd;

    for (i = 0; i < PDC_TEST_SIZE; i++) {
        expected[i] = (i * 19u + (i >> 7) + 0x91u) & 0xff;
    }

    state_fd = g_file_open_tmp("at91-usart-migration-XXXXXX",
                               &state_path, NULL);
    g_assert_cmpint(state_fd, >=, 0);
    close(state_fd);
    uri = g_strdup_printf("file:%s", state_path);

    src = qtest_init_with_serial("-machine sam9m10g45ek -S", &src_fd);
    qtest_bufwrite(src, PDC_TEST_ADDR, expected, PDC_TEST_SIZE);
    qtest_writel(src, G45_DBGU_BASE + US_TPR, PDC_TEST_ADDR);
    qtest_writel(src, G45_DBGU_BASE + US_TCR, PDC_TEST_SIZE);
    qtest_writel(src, G45_DBGU_BASE + US_PTCR, PTCR_TXTEN);
    remaining = qtest_readl(src, G45_DBGU_BASE + US_TCR);
    g_assert_cmpuint(remaining, >, 0);
    g_assert_cmpuint(remaining, <, PDC_TEST_SIZE);
    prefix = PDC_TEST_SIZE - remaining;

    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    /* No receiver space was released while the snapshot was taken. */
    g_assert_cmphex(qtest_readl(src, G45_DBGU_BASE + US_TCR), ==, remaining);

    /* Stop the source before draining its already-accepted prefix. */
    qtest_quit(src);
    receive_exact(src_fd, actual, prefix);
    close(src_fd);

    destination_args = g_strdup_printf(
        "-machine sam9m10g45ek -S -incoming %s", uri);
    dst = qtest_init_with_serial(destination_args, &dst_fd);
    wait_for_migration_complete(dst);
    receive_exact(dst_fd, actual + prefix, remaining);
    g_assert_cmpmem(actual, PDC_TEST_SIZE, expected, PDC_TEST_SIZE);

    for (i = 0; i < 1000; i++) {
        if (qtest_readl(dst, G45_DBGU_BASE + US_TCR) == 0) {
            break;
        }
        g_usleep(1000);
    }
    g_assert_cmpuint(i, <, 1000);
    g_assert_cmphex(qtest_readl(dst, G45_DBGU_BASE + US_TPR), ==,
                    PDC_TEST_ADDR + PDC_TEST_SIZE);

    close(dst_fd);
    qtest_quit(dst);
    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-usart/thr/character", test_thr_character);
    qtest_add_func("/at91-usart/pdc/backpressure", test_pdc_backpressure);
    qtest_add_func("/at91-usart/pdc/backpressure-migration",
                   test_pdc_backpressure_migration);

    return g_test_run();
}
