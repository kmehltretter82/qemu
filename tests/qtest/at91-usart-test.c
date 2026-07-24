/*
 * QTest tests for the AT91 USART/DBGU transmit paths.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define G45_DBGU_BASE          0xffffee00
#define G35_DBGU_BASE          0xfffff200
#define G45_SDRAM_BASE         0x70000000

#define US_CR                  0x00
#define US_CSR                 0x14
#define US_RHR                 0x18
#define US_THR                 0x1c
#define US_RPR                 0x100
#define US_RCR                 0x104
#define US_TPR                 0x108
#define US_TCR                 0x10c
#define US_RNPR                0x110
#define US_RNCR                0x114
#define US_TNPR                0x118
#define US_TNCR                0x11c
#define US_PTCR                0x120
#define US_PTSR                0x124

#define US_CR_STTTO            (1u << 11)
#define US_RXRDY               (1u << 0)
#define US_TXRDY               (1u << 1)
#define US_ENDRX               (1u << 3)
#define US_ENDTX               (1u << 4)
#define US_TIMEOUT             (1u << 8)
#define US_TXEMPTY             (1u << 9)
#define US_RXBUFF              (1u << 12)
#define PTCR_RXTEN             (1u << 0)
#define PTCR_RXTDIS            (1u << 1)
#define PTCR_TXTEN             (1u << 8)

/* The model flushes a partial PDC RX buffer after 2 ms of idle time. */
#define RX_TIMEOUT_NS          2000000

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

static void run_thr_character(const char *machine, uint64_t dbgu_base)
{
    g_autofree char *args = g_strdup_printf("-machine %s -S", machine);
    QTestState *qts;
    uint8_t byte;
    int sock_fd;

    qts = qtest_init_with_serial(args, &sock_fd);

    g_assert_cmphex(qtest_readl(qts, dbgu_base + US_CSR) &
                    (US_TXRDY | US_TXEMPTY), ==, US_TXRDY | US_TXEMPTY);
    qtest_writel(qts, dbgu_base + US_THR, 'G');
    receive_exact(sock_fd, &byte, 1);
    g_assert_cmphex(byte, ==, 'G');
    g_assert_cmphex(qtest_readl(qts, dbgu_base + US_CSR) &
                    (US_TXRDY | US_TXEMPTY), ==, US_TXRDY | US_TXEMPTY);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_thr_character(void)
{
    run_thr_character("sam9m10g45ek", G45_DBGU_BASE);
}

static void test_g35_thr_character(void)
{
    run_thr_character("sam9g35ek", G35_DBGU_BASE);
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

/*
 * Poll until the PDC RX pointer/counter pair matches (input arrives via the
 * chardev asynchronously to the qtest stream).  Counter values repeat
 * across a buffer handoff, so a counter alone is ambiguous.
 */
static void wait_for_rx(QTestState *qts, uint32_t rpr, uint32_t rcr)
{
    int i;

    for (i = 0; i < 5000; i++) {
        if (qtest_readl(qts, G45_DBGU_BASE + US_RPR) == rpr &&
            qtest_readl(qts, G45_DBGU_BASE + US_RCR) == rcr) {
            return;
        }
        g_usleep(1000);
    }
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RPR), ==, rpr);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RCR), ==, rcr);
}

static void send_exact(int fd, const uint8_t *buffer, size_t length)
{
    size_t sent = 0;

    while (sent < length) {
        ssize_t ret = send(fd, buffer + sent, length - sent, 0);

        g_assert_cmpint(ret, >, 0);
        sent += ret;
    }
}

/*
 * Current buffer fills, the chained next buffer is promoted inline, and
 * ENDRX/RXBUFF behave as level indicators of the counter state.
 */
static void test_pdc_rx_buffer_handoff(void)
{
    static const uint8_t data[12] = "HANDOFF-TEST";
    const uint32_t buf1 = G45_SDRAM_BASE + 0x110000;
    const uint32_t buf2 = G45_SDRAM_BASE + 0x110100;
    uint8_t readback[12];
    QTestState *qts;
    uint32_t status;
    int sock_fd;

    qts = qtest_init_with_serial("-machine sam9m10g45ek -S", &sock_fd);
    qtest_writel(qts, G45_DBGU_BASE + US_RPR, buf1);
    qtest_writel(qts, G45_DBGU_BASE + US_RCR, 8);
    qtest_writel(qts, G45_DBGU_BASE + US_RNPR, buf2);
    qtest_writel(qts, G45_DBGU_BASE + US_RNCR, 4);
    qtest_writel(qts, G45_DBGU_BASE + US_PTCR, PTCR_RXTEN);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_PTSR) & PTCR_RXTEN,
                    ==, PTCR_RXTEN);
    status = qtest_readl(qts, G45_DBGU_BASE + US_CSR);
    g_assert_cmphex(status & (US_ENDRX | US_RXBUFF), ==, 0);

    /* Fill the current buffer exactly: the next buffer must be promoted. */
    send_exact(sock_fd, data, 8);
    wait_for_rx(qts, buf2, 4);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RPR), ==, buf2);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RNCR), ==, 0);
    status = qtest_readl(qts, G45_DBGU_BASE + US_CSR);
    g_assert_cmphex(status & (US_ENDRX | US_RXBUFF), ==, 0);

    /* Fill the promoted buffer: both counters at zero raise both levels. */
    send_exact(sock_fd, data + 8, 4);
    wait_for_rx(qts, buf2 + 4, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RPR), ==, buf2 + 4);
    status = qtest_readl(qts, G45_DBGU_BASE + US_CSR);
    g_assert_cmphex(status & (US_ENDRX | US_RXBUFF), ==,
                    US_ENDRX | US_RXBUFF);

    qtest_memread(qts, buf1, readback, 8);
    qtest_memread(qts, buf2, readback + 8, 4);
    g_assert_cmpmem(readback, sizeof(data), data, sizeof(data));
    close(sock_fd);
    qtest_quit(qts);
}

/*
 * A next buffer programmed AFTER the current one already emptied must be
 * promoted immediately - the PDC keeps no separate "waiting" state.  The
 * old model refused further input forever in this arrangement.
 */
static void test_pdc_rx_late_next_promotion(void)
{
    static const uint8_t data[8] = "LATENEXT";
    const uint32_t buf1 = G45_SDRAM_BASE + 0x111000;
    const uint32_t buf2 = G45_SDRAM_BASE + 0x111100;
    uint8_t readback[8];
    QTestState *qts;
    uint32_t status;
    int sock_fd;

    qts = qtest_init_with_serial("-machine sam9m10g45ek -S", &sock_fd);
    qtest_writel(qts, G45_DBGU_BASE + US_RPR, buf1);
    qtest_writel(qts, G45_DBGU_BASE + US_RCR, 4);
    qtest_writel(qts, G45_DBGU_BASE + US_PTCR, PTCR_RXTEN);
    send_exact(sock_fd, data, 4);
    wait_for_rx(qts, buf1 + 4, 0);
    status = qtest_readl(qts, G45_DBGU_BASE + US_CSR);
    g_assert_cmphex(status & (US_ENDRX | US_RXBUFF), ==,
                    US_ENDRX | US_RXBUFF);

    qtest_writel(qts, G45_DBGU_BASE + US_RNPR, buf2);
    qtest_writel(qts, G45_DBGU_BASE + US_RNCR, 4);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RCR), ==, 4);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RPR), ==, buf2);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RNCR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_CSR) & US_ENDRX,
                    ==, 0);

    send_exact(sock_fd, data + 4, 4);
    wait_for_rx(qts, buf2 + 4, 0);
    qtest_memread(qts, buf1, readback, 4);
    qtest_memread(qts, buf2, readback + 4, 4);
    g_assert_cmpmem(readback, sizeof(data), data, sizeof(data));
    close(sock_fd);
    qtest_quit(qts);
}

/* A partial buffer raises US_TIMEOUT after the idle window; STTTO rearms. */
static void test_pdc_rx_timeout_flush(void)
{
    static const uint8_t data[5] = "IDLE!";
    const uint32_t buf = G45_SDRAM_BASE + 0x112000;
    uint8_t readback[5];
    QTestState *qts;
    int sock_fd;

    qts = qtest_init_with_serial("-machine sam9m10g45ek -S", &sock_fd);
    /* Timer callbacks stay suppressed while the VM is paused. */
    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    qtest_writel(qts, G45_DBGU_BASE + US_RPR, buf);
    qtest_writel(qts, G45_DBGU_BASE + US_RCR, 16);
    qtest_writel(qts, G45_DBGU_BASE + US_PTCR, PTCR_RXTEN);
    send_exact(sock_fd, data, sizeof(data));
    wait_for_rx(qts, buf + sizeof(data), 16 - sizeof(data));

    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_CSR) & US_TIMEOUT,
                    ==, 0);
    qtest_clock_step(qts, RX_TIMEOUT_NS);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_CSR) & US_TIMEOUT,
                    ==, US_TIMEOUT);
    qtest_memread(qts, buf, readback, sizeof(data));
    g_assert_cmpmem(readback, sizeof(data), data, sizeof(data));

    qtest_writel(qts, G45_DBGU_BASE + US_CR, US_CR_STTTO);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_CSR) & US_TIMEOUT,
                    ==, 0);
    close(sock_fd);
    qtest_quit(qts);
}

/* RXTDIS falls back to the one-byte RHR path; RXTEN resumes the DMA. */
static void test_pdc_rx_disable_reenable(void)
{
    const uint32_t buf = G45_SDRAM_BASE + 0x113000;
    uint8_t byte;
    QTestState *qts;
    int sock_fd;

    qts = qtest_init_with_serial("-machine sam9m10g45ek -S", &sock_fd);
    qtest_writel(qts, G45_DBGU_BASE + US_RPR, buf);
    qtest_writel(qts, G45_DBGU_BASE + US_RCR, 8);
    qtest_writel(qts, G45_DBGU_BASE + US_PTCR, PTCR_RXTEN);
    send_exact(sock_fd, (const uint8_t *)"AB", 2);
    wait_for_rx(qts, buf + 2, 6);

    qtest_writel(qts, G45_DBGU_BASE + US_PTCR, PTCR_RXTDIS);
    send_exact(sock_fd, (const uint8_t *)"C", 1);
    {
        int i;

        for (i = 0; i < 5000; i++) {
            if (qtest_readl(qts, G45_DBGU_BASE + US_CSR) & US_RXRDY) {
                break;
            }
            g_usleep(1000);
        }
    }
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RHR), ==, 'C');
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_RCR), ==, 6);

    qtest_writel(qts, G45_DBGU_BASE + US_PTCR, PTCR_RXTEN);
    send_exact(sock_fd, (const uint8_t *)"DE", 2);
    wait_for_rx(qts, buf + 4, 4);
    byte = 0;
    qtest_memread(qts, buf + 2, &byte, 1);
    g_assert_cmphex(byte, ==, 'D');
    close(sock_fd);
    qtest_quit(qts);
}

/*
 * TX chain: an inline next buffer follows the current one, and a next
 * buffer programmed after the current drained restarts transmission.
 */
static void test_pdc_tx_chain_and_late_next(void)
{
    static const uint8_t part1[8] = "CURRENT-";
    static const uint8_t part2[4] = "NEXT";
    static const uint8_t part3[5] = "LATER";
    const uint32_t buf1 = G45_SDRAM_BASE + 0x114000;
    const uint32_t buf2 = G45_SDRAM_BASE + 0x114100;
    const uint32_t buf3 = G45_SDRAM_BASE + 0x114200;
    uint8_t actual[17];
    QTestState *qts;
    uint32_t status;
    int sock_fd;

    qts = qtest_init_with_serial("-machine sam9m10g45ek -S", &sock_fd);
    qtest_bufwrite(qts, buf1, part1, sizeof(part1));
    qtest_bufwrite(qts, buf2, part2, sizeof(part2));
    qtest_bufwrite(qts, buf3, part3, sizeof(part3));

    qtest_writel(qts, G45_DBGU_BASE + US_TPR, buf1);
    qtest_writel(qts, G45_DBGU_BASE + US_TCR, sizeof(part1));
    qtest_writel(qts, G45_DBGU_BASE + US_TNPR, buf2);
    qtest_writel(qts, G45_DBGU_BASE + US_TNCR, sizeof(part2));
    qtest_writel(qts, G45_DBGU_BASE + US_PTCR, PTCR_TXTEN);
    receive_exact(sock_fd, actual, sizeof(part1) + sizeof(part2));
    g_assert_cmpmem(actual, sizeof(part1), part1, sizeof(part1));
    g_assert_cmpmem(actual + sizeof(part1), sizeof(part2),
                    part2, sizeof(part2));
    {
        int i;

        for (i = 0; i < 5000; i++) {
            status = qtest_readl(qts, G45_DBGU_BASE + US_CSR);
            if ((status & (US_ENDTX | US_TXEMPTY)) ==
                (US_ENDTX | US_TXEMPTY)) {
                break;
            }
            g_usleep(1000);
        }
    }
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_TCR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_TNCR), ==, 0);

    /* A next buffer programmed after the drain must restart transmission. */
    qtest_writel(qts, G45_DBGU_BASE + US_TNPR, buf3);
    qtest_writel(qts, G45_DBGU_BASE + US_TNCR, sizeof(part3));
    receive_exact(sock_fd, actual + 12, sizeof(part3));
    g_assert_cmpmem(actual + 12, sizeof(part3), part3, sizeof(part3));
    g_assert_cmphex(qtest_readl(qts, G45_DBGU_BASE + US_TPR), ==,
                    buf3 + sizeof(part3));
    close(sock_fd);
    qtest_quit(qts);
}

/* Migrate mid-buffer with a partial fill and an armed idle timeout. */
static void test_pdc_rx_handoff_migration(void)
{
    static const uint8_t data[10] = "MIGRATEPDC";
    const uint32_t buf1 = G45_SDRAM_BASE + 0x115000;
    const uint32_t buf2 = G45_SDRAM_BASE + 0x115100;
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *destination_args = NULL;
    uint8_t readback[10];
    QTestState *src, *dst;
    int src_fd, dst_fd, state_fd;

    state_fd = g_file_open_tmp("at91-usart-rx-migration-XXXXXX",
                               &state_path, NULL);
    g_assert_cmpint(state_fd, >=, 0);
    close(state_fd);
    uri = g_strdup_printf("file:%s", state_path);

    src = qtest_init_with_serial("-machine sam9m10g45ek -S", &src_fd);
    qtest_writel(src, G45_DBGU_BASE + US_RPR, buf1);
    qtest_writel(src, G45_DBGU_BASE + US_RCR, 4);
    qtest_writel(src, G45_DBGU_BASE + US_RNPR, buf2);
    qtest_writel(src, G45_DBGU_BASE + US_RNCR, 6);
    qtest_writel(src, G45_DBGU_BASE + US_PTCR, PTCR_RXTEN);

    /* Fill the first buffer and two bytes of the promoted second one. */
    send_exact(src_fd, data, 6);
    wait_for_rx(src, buf2 + 2, 4);
    g_assert_cmphex(qtest_readl(src, G45_DBGU_BASE + US_RPR), ==, buf2 + 2);

    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);
    close(src_fd);

    destination_args = g_strdup_printf(
        "-machine sam9m10g45ek -S -incoming %s", uri);
    dst = qtest_init_with_serial(destination_args, &dst_fd);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, G45_DBGU_BASE + US_RPR), ==, buf2 + 2);
    g_assert_cmphex(qtest_readl(dst, G45_DBGU_BASE + US_RCR), ==, 4);
    qtest_qmp_assert_success(dst, "{ 'execute': 'cont' }");

    /* The armed idle timeout must still deliver on the destination. */
    qtest_clock_step(dst, 4 * RX_TIMEOUT_NS);
    g_assert_cmphex(qtest_readl(dst, G45_DBGU_BASE + US_CSR) & US_TIMEOUT,
                    ==, US_TIMEOUT);

    /* The remaining bytes complete the promoted buffer exactly. */
    send_exact(dst_fd, data + 6, 4);
    wait_for_rx(dst, buf2 + 6, 0);
    qtest_memread(dst, buf1, readback, 4);
    qtest_memread(dst, buf2, readback + 4, 6);
    g_assert_cmpmem(readback, sizeof(data), data, sizeof(data));
    g_assert_cmphex(qtest_readl(dst, G45_DBGU_BASE + US_CSR) &
                    (US_ENDRX | US_RXBUFF), ==, US_ENDRX | US_RXBUFF);
    close(dst_fd);
    qtest_quit(dst);
    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-usart/thr/character", test_thr_character);
    qtest_add_func("/at91-usart/g35/thr/character", test_g35_thr_character);
    qtest_add_func("/at91-usart/pdc/rx-buffer-handoff",
                   test_pdc_rx_buffer_handoff);
    qtest_add_func("/at91-usart/pdc/rx-late-next-promotion",
                   test_pdc_rx_late_next_promotion);
    qtest_add_func("/at91-usart/pdc/rx-timeout-flush",
                   test_pdc_rx_timeout_flush);
    qtest_add_func("/at91-usart/pdc/rx-disable-reenable",
                   test_pdc_rx_disable_reenable);
    qtest_add_func("/at91-usart/pdc/tx-chain-and-late-next",
                   test_pdc_tx_chain_and_late_next);
    qtest_add_func("/at91-usart/pdc/rx-handoff-migration",
                   test_pdc_rx_handoff_migration);
    qtest_add_func("/at91-usart/pdc/backpressure", test_pdc_backpressure);
    qtest_add_func("/at91-usart/pdc/backpressure-migration",
                   test_pdc_backpressure_migration);

    return g_test_run();
}
