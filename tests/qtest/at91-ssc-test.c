/*
 * QTest tests for the AT91 SSC embedded PDC state machine.
 *
 * The receiver loopback returns every transmitted word, so a PDC TX ring
 * paired with a PDC RX ring exercises both current/next engines with no
 * external codec.  Transmission is synchronous: the drain happens inside
 * the register write that makes the channel runnable.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define G45_SSC0_BASE          0xfff9c000
#define G45_SDRAM_BASE         0x70000000

#define SSC_CR                 0x00
#define SSC_RFMR               0x14
#define SSC_TFMR               0x1c
#define SSC_RHR                0x20
#define SSC_THR                0x24
#define SSC_SR                 0x40
#define PDC_RPR                0x100
#define PDC_RCR                0x104
#define PDC_TPR                0x108
#define PDC_TCR                0x10c
#define PDC_RNPR               0x110
#define PDC_RNCR               0x114
#define PDC_TNPR               0x118
#define PDC_TNCR               0x11c
#define PDC_PTCR               0x120
#define PDC_PTSR               0x124

#define CR_RXEN                (1u << 0)
#define CR_TXEN                (1u << 8)
#define RFMR_LOOP              (1u << 5)
#define WORD_32BIT             0x1f

#define SR_TXRDY               (1u << 0)
#define SR_ENDTX               (1u << 2)
#define SR_TXBUFE              (1u << 3)
#define SR_RXRDY               (1u << 4)
#define SR_OVRUN               (1u << 5)
#define SR_ENDRX               (1u << 6)
#define SR_RXBUFF              (1u << 7)

#define PDC_RXTEN              (1u << 0)
#define PDC_RXTDIS             (1u << 1)
#define PDC_TXTEN              (1u << 8)
#define PDC_TXTDIS             (1u << 9)

static const uint32_t pattern[4] = {
    0x11213141, 0xa55a0ff0, 0xc3c3d4d4, 0x9e8f7a6b,
};

static void ssc_write(QTestState *qts, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, G45_SSC0_BASE + reg, value);
}

static uint32_t ssc_read(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, G45_SSC0_BASE + reg);
}

static QTestState *ssc_start_loopback(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");

    ssc_write(qts, SSC_TFMR, WORD_32BIT);
    ssc_write(qts, SSC_RFMR, WORD_32BIT | RFMR_LOOP);
    ssc_write(qts, SSC_CR, CR_RXEN | CR_TXEN);
    return qts;
}

/* Both rings chain inline; every level bit reflects the final state. */
static void test_ssc_pdc_ring_roundtrip(void)
{
    const uint32_t src1 = G45_SDRAM_BASE + 0x120000;
    const uint32_t src2 = G45_SDRAM_BASE + 0x120100;
    const uint32_t dst1 = G45_SDRAM_BASE + 0x120200;
    const uint32_t dst2 = G45_SDRAM_BASE + 0x120300;
    QTestState *qts = ssc_start_loopback();
    uint32_t status;
    int i;

    for (i = 0; i < 2; i++) {
        qtest_writel(qts, src1 + 4 * i, pattern[i]);
        qtest_writel(qts, src2 + 4 * i, pattern[2 + i]);
        qtest_writel(qts, dst1 + 4 * i, 0xdeadbeef);
        qtest_writel(qts, dst2 + 4 * i, 0xdeadbeef);
    }
    ssc_write(qts, PDC_RPR, dst1);
    ssc_write(qts, PDC_RCR, 2);
    ssc_write(qts, PDC_RNPR, dst2);
    ssc_write(qts, PDC_RNCR, 2);
    ssc_write(qts, PDC_TPR, src1);
    ssc_write(qts, PDC_TCR, 2);
    ssc_write(qts, PDC_TNPR, src2);
    ssc_write(qts, PDC_TNCR, 2);
    ssc_write(qts, PDC_PTCR, PDC_RXTEN | PDC_TXTEN);

    for (i = 0; i < 2; i++) {
        g_assert_cmphex(qtest_readl(qts, dst1 + 4 * i), ==, pattern[i]);
        g_assert_cmphex(qtest_readl(qts, dst2 + 4 * i), ==, pattern[2 + i]);
    }
    g_assert_cmphex(ssc_read(qts, PDC_RPR), ==, dst2 + 8);
    g_assert_cmphex(ssc_read(qts, PDC_RCR), ==, 0);
    g_assert_cmphex(ssc_read(qts, PDC_RNCR), ==, 0);
    g_assert_cmphex(ssc_read(qts, PDC_TPR), ==, src2 + 8);
    g_assert_cmphex(ssc_read(qts, PDC_TCR), ==, 0);
    g_assert_cmphex(ssc_read(qts, PDC_TNCR), ==, 0);
    status = ssc_read(qts, SSC_SR);
    g_assert_cmphex(status & (SR_TXRDY | SR_ENDTX | SR_TXBUFE |
                              SR_ENDRX | SR_RXBUFF | SR_RXRDY | SR_OVRUN),
                    ==,
                    SR_TXRDY | SR_ENDTX | SR_TXBUFE | SR_ENDRX | SR_RXBUFF);
    qtest_quit(qts);
}

/*
 * A next buffer programmed after the current one drained must start (TX)
 * or be promoted into the current registers immediately (RX).
 */
static void test_ssc_pdc_late_next(void)
{
    const uint32_t src1 = G45_SDRAM_BASE + 0x121000;
    const uint32_t src2 = G45_SDRAM_BASE + 0x121100;
    const uint32_t dst1 = G45_SDRAM_BASE + 0x121200;
    const uint32_t dst2 = G45_SDRAM_BASE + 0x121300;
    QTestState *qts = ssc_start_loopback();
    uint32_t status;
    int i;

    for (i = 0; i < 2; i++) {
        qtest_writel(qts, src1 + 4 * i, pattern[i]);
        qtest_writel(qts, src2 + 4 * i, pattern[2 + i]);
        qtest_writel(qts, dst1 + 4 * i, 0xdeadbeef);
        qtest_writel(qts, dst2 + 4 * i, 0xdeadbeef);
    }
    ssc_write(qts, PDC_RPR, dst1);
    ssc_write(qts, PDC_RCR, 4);
    ssc_write(qts, PDC_TPR, src1);
    ssc_write(qts, PDC_TCR, 2);
    ssc_write(qts, PDC_PTCR, PDC_RXTEN | PDC_TXTEN);
    g_assert_cmphex(ssc_read(qts, PDC_TCR), ==, 0);
    g_assert_cmphex(ssc_read(qts, PDC_RCR), ==, 2);

    /* TX: a late-queued next buffer restarts transmission. */
    ssc_write(qts, PDC_TNPR, src2);
    ssc_write(qts, PDC_TNCR, 2);
    g_assert_cmphex(ssc_read(qts, PDC_TPR), ==, src2 + 8);
    g_assert_cmphex(ssc_read(qts, PDC_TNCR), ==, 0);
    for (i = 0; i < 2; i++) {
        g_assert_cmphex(qtest_readl(qts, dst1 + 4 * i), ==, pattern[i]);
        g_assert_cmphex(qtest_readl(qts, dst1 + 8 + 4 * i), ==,
                        pattern[2 + i]);
    }

    /* RX: the ring is exhausted; clear the sticky end events first. */
    g_assert_cmphex(ssc_read(qts, PDC_RCR), ==, 0);
    (void)ssc_read(qts, SSC_SR);
    status = ssc_read(qts, SSC_SR);
    g_assert_cmphex(status & (SR_ENDRX | SR_RXBUFF), ==,
                    SR_ENDRX | SR_RXBUFF);

    /* A late-programmed RX next buffer is promoted immediately. */
    ssc_write(qts, PDC_RNPR, dst2);
    ssc_write(qts, PDC_RNCR, 2);
    g_assert_cmphex(ssc_read(qts, PDC_RPR), ==, dst2);
    g_assert_cmphex(ssc_read(qts, PDC_RCR), ==, 2);
    g_assert_cmphex(ssc_read(qts, PDC_RNCR), ==, 0);
    (void)ssc_read(qts, SSC_SR);
    status = ssc_read(qts, SSC_SR);
    g_assert_cmphex(status & (SR_ENDRX | SR_RXBUFF), ==, 0);

    ssc_write(qts, PDC_TPR, src1);
    ssc_write(qts, PDC_TCR, 2);
    for (i = 0; i < 2; i++) {
        g_assert_cmphex(qtest_readl(qts, dst2 + 4 * i), ==, pattern[i]);
    }
    qtest_quit(qts);
}

/* An exhausted RX ring falls back to RHR; a further word overruns. */
static void test_ssc_pdc_rx_overflow_to_rhr(void)
{
    const uint32_t src = G45_SDRAM_BASE + 0x122000;
    const uint32_t dst = G45_SDRAM_BASE + 0x122100;
    QTestState *qts = ssc_start_loopback();
    uint32_t status;
    int i;

    for (i = 0; i < 3; i++) {
        qtest_writel(qts, src + 4 * i, pattern[i]);
    }
    qtest_writel(qts, dst, 0xdeadbeef);
    ssc_write(qts, PDC_RPR, dst);
    ssc_write(qts, PDC_RCR, 1);
    ssc_write(qts, PDC_TPR, src);
    ssc_write(qts, PDC_TCR, 3);
    ssc_write(qts, PDC_PTCR, PDC_RXTEN | PDC_TXTEN);

    g_assert_cmphex(qtest_readl(qts, dst), ==, pattern[0]);
    status = ssc_read(qts, SSC_SR);
    g_assert_cmphex(status & (SR_RXRDY | SR_OVRUN), ==,
                    SR_RXRDY | SR_OVRUN);
    /* The overrun keeps the newest word; reading SR cleared OVRUN. */
    g_assert_cmphex(ssc_read(qts, SSC_RHR), ==, pattern[2]);
    status = ssc_read(qts, SSC_SR);
    g_assert_cmphex(status & (SR_RXRDY | SR_OVRUN), ==, 0);
    qtest_quit(qts);
}

/* A disabled transmitter holds the ring; TXTEN drains it unchanged. */
static void test_ssc_pdc_txtdis_gate(void)
{
    const uint32_t src = G45_SDRAM_BASE + 0x123000;
    const uint32_t dst = G45_SDRAM_BASE + 0x123100;
    QTestState *qts = ssc_start_loopback();
    int i;

    for (i = 0; i < 2; i++) {
        qtest_writel(qts, src + 4 * i, pattern[i]);
        qtest_writel(qts, dst + 4 * i, 0xdeadbeef);
    }
    ssc_write(qts, PDC_RPR, dst);
    ssc_write(qts, PDC_RCR, 2);
    ssc_write(qts, PDC_PTCR, PDC_RXTEN | PDC_TXTDIS);
    ssc_write(qts, PDC_TPR, src);
    ssc_write(qts, PDC_TCR, 2);
    g_assert_cmphex(ssc_read(qts, PDC_PTSR), ==, PDC_RXTEN);
    g_assert_cmphex(ssc_read(qts, PDC_TCR), ==, 2);
    g_assert_cmphex(ssc_read(qts, PDC_TPR), ==, src);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);

    ssc_write(qts, PDC_PTCR, PDC_TXTEN);
    g_assert_cmphex(ssc_read(qts, PDC_TCR), ==, 0);
    for (i = 0; i < 2; i++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * i), ==, pattern[i]);
    }
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

/* Migrate with the RX ring mid-buffer and a pending next buffer. */
static void test_ssc_pdc_ring_migration(void)
{
    const uint32_t src = G45_SDRAM_BASE + 0x124000;
    const uint32_t dst1 = G45_SDRAM_BASE + 0x124100;
    const uint32_t dst2 = G45_SDRAM_BASE + 0x124200;
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *sqts, *dqts;
    int state_fd;
    int i;

    state_fd = g_file_open_tmp("at91-ssc-migration-XXXXXX",
                               &state_path, NULL);
    g_assert_cmpint(state_fd, >=, 0);
    close(state_fd);
    uri = g_strdup_printf("file:%s", state_path);

    sqts = qtest_init("-machine sam9m10g45ek -S");
    ssc_write(sqts, SSC_TFMR, WORD_32BIT);
    ssc_write(sqts, SSC_RFMR, WORD_32BIT | RFMR_LOOP);
    ssc_write(sqts, SSC_CR, CR_RXEN | CR_TXEN);
    for (i = 0; i < 4; i++) {
        qtest_writel(sqts, src + 4 * i, pattern[i]);
    }
    ssc_write(sqts, PDC_RPR, dst1);
    ssc_write(sqts, PDC_RCR, 4);
    ssc_write(sqts, PDC_RNPR, dst2);
    ssc_write(sqts, PDC_RNCR, 2);
    ssc_write(sqts, PDC_TPR, src);
    ssc_write(sqts, PDC_TCR, 3);
    ssc_write(sqts, PDC_PTCR, PDC_RXTEN | PDC_TXTEN);
    g_assert_cmphex(ssc_read(sqts, PDC_RCR), ==, 1);

    qtest_qmp_assert_success(sqts,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(sqts);
    qtest_quit(sqts);

    dqts = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    wait_for_migration_complete(dqts);
    g_assert_cmphex(ssc_read(dqts, PDC_RPR), ==, dst1 + 12);
    g_assert_cmphex(ssc_read(dqts, PDC_RCR), ==, 1);
    g_assert_cmphex(ssc_read(dqts, PDC_RNCR), ==, 2);

    /* Three more words finish the current buffer and enter the next. */
    ssc_write(dqts, PDC_TPR, src);
    ssc_write(dqts, PDC_TCR, 3);
    for (i = 0; i < 3; i++) {
        g_assert_cmphex(qtest_readl(dqts, dst1 + 4 * i), ==, pattern[i]);
    }
    g_assert_cmphex(qtest_readl(dqts, dst1 + 12), ==, pattern[0]);
    g_assert_cmphex(qtest_readl(dqts, dst2), ==, pattern[1]);
    g_assert_cmphex(qtest_readl(dqts, dst2 + 4), ==, pattern[2]);
    g_assert_cmphex(ssc_read(dqts, PDC_RCR), ==, 0);
    g_assert_cmphex(ssc_read(dqts, PDC_RNCR), ==, 0);
    qtest_quit(dqts);
    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-ssc/pdc/ring-roundtrip",
                   test_ssc_pdc_ring_roundtrip);
    qtest_add_func("/at91-ssc/pdc/late-next", test_ssc_pdc_late_next);
    qtest_add_func("/at91-ssc/pdc/rx-overflow-to-rhr",
                   test_ssc_pdc_rx_overflow_to_rhr);
    qtest_add_func("/at91-ssc/pdc/txtdis-gate", test_ssc_pdc_txtdis_gate);
    qtest_add_func("/at91-ssc/pdc/ring-migration",
                   test_ssc_pdc_ring_migration);

    return g_test_run();
}
