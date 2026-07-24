/*
 * QTest tests for the AT91SAM9G45 High Speed MultiMedia Card Interface.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "libqtest.h"

#define G45_HSMCI0_BASE       0xfff80000
#define G45_HSMCI0_PATH       "/machine/hsmci0"
#define G45_AIC_BASE          0xfffff000
#define G45_AIC_IPR           0x10c
#define G45_AIC_HSMCI0        (1u << 11)
#define G45_PMC_BASE          0xfffffc00
#define G45_PMC_MCKR          0x30
#define G45_MCKR_MAINCK       0x00000001
#define G45_PIOD_BASE         0xfffff800
#define PIO_PDSR              0x3c
#define MMC1_WP_PIN           (1u << 29)

#define HSMCI_CR              0x00
#define HSMCI_MR              0x04
#define HSMCI_DTOR            0x08
#define HSMCI_SDCR            0x0c
#define HSMCI_ARGR            0x10
#define HSMCI_CMDR            0x14
#define HSMCI_BLKR            0x18
#define HSMCI_CSTOR           0x1c
#define HSMCI_RDR             0x30
#define HSMCI_SR              0x40
#define HSMCI_IER             0x44
#define HSMCI_DMA             0x50
#define HSMCI_CFG             0x54
#define HSMCI_WPMR            0xe4
#define HSMCI_WPSR            0xe8

#define HSMCI_CR_MCIEN        (1u << 0)
#define HSMCI_CR_SWRST        (1u << 7)
#define HSMCI_CMDR_START      (1u << 16)
#define HSMCI_CMDR_READ       (1u << 18)
#define HSMCI_CMDR_SDIO_BYTE  (4u << 19)
#define HSMCI_SR_CMDRDY       (1u << 0)
#define HSMCI_SR_RXRDY        (1u << 1)
#define HSMCI_SR_DTIP         (1u << 4)
#define HSMCI_SR_NOTBUSY      (1u << 5)
#define HSMCI_SR_SDIOIRQA     (1u << 8)
#define HSMCI_SR_XFRDONE      (1u << 27)
#define HSMCI_WPMR_KEY        (0x4d4349u << 8)
#define HSMCI_WPMR_WPEN       (1u << 0)
#define HSMCI_WPSR_WPVS_WRITE 1u
#define HSMCI_WPSR_WPVS_RESET 2u

/* Reset MCK is 132 MHz and MR.CLKDIV=0 gives a 66 MHz card clock. */
#define CMD_66MHZ_NS          728
#define CMD_33MHZ_NS          1455
#define DATA_END_66MHZ_NS     258
#define CMD_CLKDIV255_NS      186182

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

static void hsmci_write(QTestState *qts, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, G45_HSMCI0_BASE + reg, value);
}

static uint32_t hsmci_read(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, G45_HSMCI0_BASE + reg);
}

static void hsmci_start_norsp(QTestState *qts, uint32_t extra)
{
    hsmci_write(qts, HSMCI_ARGR, 0);
    hsmci_write(qts, HSMCI_CMDR, extra);
}

/*
 * Full card enumeration at a given board's HSMCI base: the identify
 * sequence a real driver runs (CMD0, ACMD41, CMD2, CMD3) must yield a
 * nonzero RCA.  Shared between the G45 and the SAM9x5-family G35 so a
 * board-wiring regression (missing card, broken bus routing) surfaces
 * here rather than as a silent Linux mount timeout.
 */
static void run_card_enumeration(const char *machine, uint64_t base)
{
    g_autofree char *image_path = NULL;
    QTestState *qts;
    uint32_t rca;
    int fd;

    fd = g_file_open_tmp("at91-hsmci-enum-XXXXXX", &image_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(ftruncate(fd, 4 * 1024 * 1024), ==, 0);
    close(fd);

    qts = qtest_initf("-machine %s -S "
                      "-drive if=sd,index=0,format=raw,file=%s",
                      machine, image_path);
    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");

    qtest_writel(qts, base + HSMCI_CR, HSMCI_CR_MCIEN);
    qtest_writel(qts, base + HSMCI_ARGR, 0);
    qtest_writel(qts, base + HSMCI_CMDR, 0);
    qtest_clock_step(qts, 4000);
    qtest_writel(qts, base + HSMCI_ARGR, 0);
    qtest_writel(qts, base + HSMCI_CMDR, 55 | (1u << 6));
    qtest_clock_step(qts, 4000);
    qtest_writel(qts, base + HSMCI_ARGR, 0x00ff8000);
    qtest_writel(qts, base + HSMCI_CMDR, 41 | (1u << 6));
    qtest_clock_step(qts, 4000);
    qtest_writel(qts, base + HSMCI_ARGR, 0);
    qtest_writel(qts, base + HSMCI_CMDR, 2 | (2u << 6));
    qtest_clock_step(qts, 4000);
    qtest_writel(qts, base + HSMCI_ARGR, 0);
    qtest_writel(qts, base + HSMCI_CMDR, 3 | (1u << 6));
    qtest_clock_step(qts, 4000);

    rca = qtest_readl(qts, base + 0x20) & 0xffff0000;
    g_assert_cmphex(rca, !=, 0);
    qtest_quit(qts);
    unlink(image_path);
}

static void test_g45_card_enumeration(void)
{
    run_card_enumeration("sam9m10g45ek", G45_HSMCI0_BASE);
}

static void test_g35_card_enumeration(void)
{
    run_card_enumeration("sam9g35ek", 0xf0008000);
}

static void test_write_protection_and_reset(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");

    hsmci_write(qts, HSMCI_MR, 0x1234);
    hsmci_write(qts, HSMCI_WPMR, 0x12345601);
    g_assert_cmphex(hsmci_read(qts, HSMCI_WPMR), ==, 0);

    hsmci_write(qts, HSMCI_WPMR,
                HSMCI_WPMR_KEY | HSMCI_WPMR_WPEN);
    g_assert_cmphex(hsmci_read(qts, HSMCI_WPMR), ==,
                    HSMCI_WPMR_WPEN);

    hsmci_write(qts, HSMCI_MR, 0x5678);
    g_assert_cmphex(hsmci_read(qts, HSMCI_MR), ==, 0x1234);
    g_assert_cmphex(hsmci_read(qts, HSMCI_WPSR), ==,
                    (HSMCI_MR << 8) | HSMCI_WPSR_WPVS_WRITE);

    /* BLKR is not one of the six WPMR-protected configuration registers. */
    hsmci_write(qts, HSMCI_BLKR, 0x02000001);
    g_assert_cmphex(hsmci_read(qts, HSMCI_BLKR), ==, 0x02000001);

    /* A valid WPMR write also acknowledges the prior violation. */
    hsmci_write(qts, HSMCI_WPMR,
                HSMCI_WPMR_KEY | HSMCI_WPMR_WPEN);
    g_assert_cmphex(hsmci_read(qts, HSMCI_WPSR), ==, 0);

    hsmci_write(qts, HSMCI_DMA, 0x101);
    g_assert_cmphex(hsmci_read(qts, HSMCI_DMA), ==, 0);
    g_assert_cmphex(hsmci_read(qts, HSMCI_WPSR), ==,
                    (HSMCI_DMA << 8) | HSMCI_WPSR_WPVS_WRITE);

    /* SWRST still occurs, but records that protection was enabled. */
    hsmci_write(qts, HSMCI_CR, HSMCI_CR_SWRST);
    g_assert_cmphex(hsmci_read(qts, HSMCI_MR), ==, 0);
    g_assert_cmphex(hsmci_read(qts, HSMCI_BLKR), ==, 0);
    g_assert_cmphex(hsmci_read(qts, HSMCI_WPSR) & 3, ==,
                    HSMCI_WPSR_WPVS_WRITE | HSMCI_WPSR_WPVS_RESET);

    hsmci_write(qts, HSMCI_WPMR, HSMCI_WPMR_KEY);
    g_assert_cmphex(hsmci_read(qts, HSMCI_WPMR), ==, 0);
    g_assert_cmphex(hsmci_read(qts, HSMCI_WPSR), ==, 0);
    qtest_quit(qts);
}

static void pulse_sdio_irq(QTestState *qts)
{
    qtest_set_irq_in(qts, G45_HSMCI0_PATH, "sdio-irq", 0, 1);
    qtest_set_irq_in(qts, G45_HSMCI0_PATH, "sdio-irq", 0, 0);
}

static void test_sdio_interrupt(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");

    pulse_sdio_irq(qts);
    g_assert_cmphex(qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_HSMCI0, ==, 0);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_SDIOIRQA, ==,
                    HSMCI_SR_SDIOIRQA);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_SDIOIRQA, ==, 0);

    hsmci_write(qts, HSMCI_IER, HSMCI_SR_SDIOIRQA);
    pulse_sdio_irq(qts);
    g_assert_cmphex(qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_HSMCI0, ==, G45_AIC_HSMCI0);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_SDIOIRQA, ==,
                    HSMCI_SR_SDIOIRQA);
    g_assert_cmphex(qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_HSMCI0, ==, 0);
    qtest_quit(qts);
}

static void test_command_clock_divider(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");

    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    hsmci_write(qts, HSMCI_CR, HSMCI_CR_MCIEN);
    hsmci_start_norsp(qts, 0);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_CMDRDY, ==, 0);
    qtest_clock_step(qts, CMD_66MHZ_NS - 1);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_CMDRDY, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) &
                    (HSMCI_SR_CMDRDY | HSMCI_SR_XFRDONE), ==,
                    HSMCI_SR_CMDRDY | HSMCI_SR_XFRDONE);

    hsmci_write(qts, HSMCI_MR, 1);
    hsmci_start_norsp(qts, 0);
    qtest_clock_step(qts, CMD_33MHZ_NS - 1);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_CMDRDY, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_CMDRDY, ==,
                    HSMCI_SR_CMDRDY);
    qtest_quit(qts);
}

static void test_live_mck_change(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");

    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    hsmci_write(qts, HSMCI_CR, HSMCI_CR_MCIEN);
    hsmci_start_norsp(qts, 0);
    qtest_clock_step(qts, 100);
    qtest_writel(qts, G45_PMC_BASE + G45_PMC_MCKR, G45_MCKR_MAINCK);

    /* The remaining command clocks now run at 6 MHz, not 66 MHz. */
    qtest_clock_step(qts, 6500);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_CMDRDY, ==, 0);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_CMDRDY, ==,
                    HSMCI_SR_CMDRDY);
    qtest_quit(qts);
}

static void test_sdio_byte_tail_and_status(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    uint32_t status;

    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    hsmci_write(qts, HSMCI_CR, HSMCI_CR_MCIEN);
    /* SDIO byte mode ignores BLKLEN; BCNT=3 means exactly three bytes. */
    hsmci_write(qts, HSMCI_BLKR, (0x1234u << 16) | 3);
    hsmci_start_norsp(qts, HSMCI_CMDR_START | HSMCI_CMDR_READ |
                      HSMCI_CMDR_SDIO_BYTE);
    qtest_clock_step(qts, CMD_66MHZ_NS);
    status = hsmci_read(qts, HSMCI_SR);
    g_assert_cmphex(status & (HSMCI_SR_CMDRDY | HSMCI_SR_RXRDY |
                             HSMCI_SR_DTIP | HSMCI_SR_XFRDONE), ==,
                    HSMCI_SR_CMDRDY | HSMCI_SR_RXRDY | HSMCI_SR_DTIP);

    (void)hsmci_read(qts, HSMCI_RDR);
    status = hsmci_read(qts, HSMCI_SR);
    g_assert_cmphex(status & HSMCI_SR_RXRDY, ==, 0);
    g_assert_cmphex(status & HSMCI_SR_DTIP, ==, HSMCI_SR_DTIP);
    g_assert_cmphex(status & HSMCI_SR_XFRDONE, ==, 0);
    qtest_clock_step(qts, DATA_END_66MHZ_NS - 1);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_XFRDONE, ==, 0);
    qtest_clock_step(qts, 1);
    status = hsmci_read(qts, HSMCI_SR);
    g_assert_cmphex(status & HSMCI_SR_DTIP, ==, 0);
    g_assert_cmphex(status & (HSMCI_SR_NOTBUSY | HSMCI_SR_XFRDONE), ==,
                    HSMCI_SR_NOTBUSY | HSMCI_SR_XFRDONE);
    qtest_quit(qts);
}

static void test_active_command_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("at91-hsmci-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    qtest_qmp_assert_success(src, "{ 'execute': 'cont' }");
    hsmci_write(src, HSMCI_CR, HSMCI_CR_MCIEN);
    hsmci_write(src, HSMCI_MR, 0xff);
    hsmci_start_norsp(src, 0);
    qtest_clock_step(src, 50000);
    g_assert_cmphex(hsmci_read(src, HSMCI_SR) & HSMCI_SR_CMDRDY, ==, 0);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(hsmci_read(dst, HSMCI_SR) & HSMCI_SR_CMDRDY, ==, 0);
    qtest_qmp_assert_success(dst, "{ 'execute': 'cont' }");
    qtest_clock_step(dst, CMD_CLKDIV255_NS - 50000 - 1);
    g_assert_cmphex(hsmci_read(dst, HSMCI_SR) & HSMCI_SR_CMDRDY, ==, 0);
    qtest_clock_step(dst, 1);
    g_assert_cmphex(hsmci_read(dst, HSMCI_SR) & HSMCI_SR_CMDRDY, ==,
                    HSMCI_SR_CMDRDY);
    qtest_quit(dst);
    unlink(state_path);
}

/*
 * DTOR counts MCK cycles between two data accesses.  DTOCYC=15/DTOMUL=0 is
 * 15 cycles of the 132 MHz reset MCK = 114 ns (rounded up).  An access
 * restarts the counter; expiry raises read-to-clear DTOE and ends the data
 * phase like the driver expects before it issues its recovery STOP.
 */
#define DTOE_15CYC_NS         114

static void test_data_timeout(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    uint32_t status;

    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    hsmci_write(qts, HSMCI_CR, HSMCI_CR_MCIEN);
    hsmci_write(qts, HSMCI_DTOR, 15);
    hsmci_write(qts, HSMCI_IER, 1u << 22);
    hsmci_write(qts, HSMCI_BLKR, (0x1234u << 16) | 3);
    hsmci_start_norsp(qts, HSMCI_CMDR_START | HSMCI_CMDR_READ |
                      HSMCI_CMDR_SDIO_BYTE);
    qtest_clock_step(qts, CMD_66MHZ_NS);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_DTIP, ==,
                    HSMCI_SR_DTIP);

    /* Nobody reads RDR: DTOE exactly one timeout period after data start. */
    qtest_clock_step(qts, DTOE_15CYC_NS - 1);
    g_assert_cmphex(qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_HSMCI0, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_HSMCI0, ==, G45_AIC_HSMCI0);
    status = hsmci_read(qts, HSMCI_SR);
    g_assert_cmphex(status & (1u << 22), ==, 1u << 22);
    g_assert_cmphex(status & HSMCI_SR_DTIP, ==, 0);
    g_assert_cmphex(status & (HSMCI_SR_NOTBUSY | HSMCI_SR_XFRDONE), ==,
                    HSMCI_SR_NOTBUSY | HSMCI_SR_XFRDONE);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & (1u << 22), ==, 0);

    /* An RDR access inside the window restarts the timeout counter. */
    hsmci_write(qts, HSMCI_BLKR, (0x1234u << 16) | 8);
    hsmci_start_norsp(qts, HSMCI_CMDR_START | HSMCI_CMDR_READ |
                      HSMCI_CMDR_SDIO_BYTE);
    qtest_clock_step(qts, CMD_66MHZ_NS);
    qtest_clock_step(qts, DTOE_15CYC_NS - 10);
    (void)hsmci_read(qts, HSMCI_RDR);
    qtest_clock_step(qts, DTOE_15CYC_NS - 1);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & (1u << 22), ==, 0);
    g_assert_cmphex(hsmci_read(qts, HSMCI_SR) & HSMCI_SR_DTIP, ==,
                    HSMCI_SR_DTIP);
    qtest_clock_step(qts, 1);
    status = hsmci_read(qts, HSMCI_SR);
    g_assert_cmphex(status & (1u << 22), ==, 1u << 22);
    g_assert_cmphex(status & HSMCI_SR_DTIP, ==, 0);
    qtest_quit(qts);
}

/* DTOCYC=15/DTOMUL=4 is 15,360 MCK cycles = 116,364 ns at reset MCK. */
#define DTOE_15X1024CYC_NS    116364

static void test_data_timeout_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    uint32_t status;
    int fd;

    fd = g_file_open_tmp("at91-hsmci-dtoe-migration-XXXXXX",
                         &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    qtest_qmp_assert_success(src, "{ 'execute': 'cont' }");
    hsmci_write(src, HSMCI_CR, HSMCI_CR_MCIEN);
    hsmci_write(src, HSMCI_DTOR, (4u << 4) | 15);
    hsmci_write(src, HSMCI_BLKR, (0x1234u << 16) | 3);
    hsmci_start_norsp(src, HSMCI_CMDR_START | HSMCI_CMDR_READ |
                      HSMCI_CMDR_SDIO_BYTE);
    qtest_clock_step(src, CMD_66MHZ_NS);
    qtest_clock_step(src, 50000);
    g_assert_cmphex(hsmci_read(src, HSMCI_SR) & HSMCI_SR_DTIP, ==,
                    HSMCI_SR_DTIP);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    wait_for_migration_complete(dst);
    qtest_qmp_assert_success(dst, "{ 'execute': 'cont' }");
    qtest_clock_step(dst, DTOE_15X1024CYC_NS - 50000 - 1);
    status = hsmci_read(dst, HSMCI_SR);
    g_assert_cmphex(status & ((1u << 22) | HSMCI_SR_DTIP), ==,
                    HSMCI_SR_DTIP);
    qtest_clock_step(dst, 1);
    status = hsmci_read(dst, HSMCI_SR);
    g_assert_cmphex(status & (1u << 22), ==, 1u << 22);
    g_assert_cmphex(status & HSMCI_SR_DTIP, ==, 0);
    qtest_quit(dst);
    unlink(state_path);
}

static void test_mmc1_write_protect_pin(void)
{
    g_autofree char *image_path = NULL;
    QTestState *qts;
    int fd;

    fd = g_file_open_tmp("at91-hsmci-sd-XXXXXX.img", &image_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(ftruncate(fd, 64 * MiB), ==, 0);
    close(fd);

    qts = qtest_initf("-machine sam9m10g45ek -S "
                      "-drive if=sd,index=1,format=raw,file=%s", image_path);
    /* QEMU SD media is writable, so the active-high socket WP pin is low. */
    g_assert_cmphex(qtest_readl(qts, G45_PIOD_BASE + PIO_PDSR) & MMC1_WP_PIN,
                    ==, 0);
    qtest_quit(qts);
    unlink(image_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-hsmci/g45/card-enumeration",
                   test_g45_card_enumeration);
    qtest_add_func("/at91-hsmci/g35/card-enumeration",
                   test_g35_card_enumeration);
    qtest_add_func("/at91-hsmci/write-protection-reset",
                   test_write_protection_and_reset);
    qtest_add_func("/at91-hsmci/sdio-interrupt", test_sdio_interrupt);
    qtest_add_func("/at91-hsmci/command-clock-divider",
                   test_command_clock_divider);
    qtest_add_func("/at91-hsmci/live-mck-change", test_live_mck_change);
    qtest_add_func("/at91-hsmci/sdio-byte-tail-status",
                   test_sdio_byte_tail_and_status);
    qtest_add_func("/at91-hsmci/data-timeout", test_data_timeout);
    qtest_add_func("/at91-hsmci/data-timeout-migration",
                   test_data_timeout_migration);
    qtest_add_func("/at91-hsmci/active-command-migration",
                   test_active_command_migration);
    qtest_add_func("/at91-hsmci/mmc1-write-protect-pin",
                   test_mmc1_write_protect_pin);
    return g_test_run();
}
