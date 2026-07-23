/*
 * QTest tests for AT91 peripheral-paced DMA.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "libqtest.h"

#define G45_DMAC_BASE          0xffffec00
#define G45_HSMCI0_BASE        0xfff80000
#define G45_HSMCI1_BASE        0xfffd0000
#define G45_PMC_BASE           0xfffffc00
#define G45_SDRAM_BASE         0x70000000
#define G45_EBI_CS0_BASE       0x10000000

#define G35_DMAC0_BASE         0xffffec00
#define G35_DMAC1_BASE         0xffffee00
#define G35_HSMCI0_BASE        0xf0008000
#define G35_HSMCI1_BASE        0xf000c000
#define G35_SDRAM_BASE         0x20000000

#define DMAC_CHER              0x28
#define DMAC_CHDR              0x2c
#define DMAC_CHSR              0x30
#define DMAC_GCFG              0x00
#define DMAC_EN                0x04
#define DMAC_SREQ              0x08
#define DMAC_CREQ              0x0c
#define DMAC_LAST              0x10
#define DMAC_EBCIER            0x18
#define DMAC_EBCIDR            0x1c
#define DMAC_EBCIMR            0x20
#define DMAC_EBCISR            0x24
#define DMAC_CH0_BASE          0x3c
#define DMAC_CH_STRIDE         0x28
#define DMAC_SADDR             0x00
#define DMAC_DADDR             0x04
#define DMAC_DSCR              0x08
#define DMAC_CTRLA             0x0c
#define DMAC_CTRLB             0x10
#define DMAC_CFG               0x14
#define DMAC_SPIP              0x18
#define DMAC_DPIP              0x1c

#define DMAC_CTRLA_BTSIZE(x)   (x)
#define DMAC_CTRLA_SCSIZE(x)   ((x) << 16)
#define DMAC_CTRLA_DCSIZE(x)   ((x) << 20)
#define DMAC_CTRLA_SRC_WIDTH_4 (2u << 24)
#define DMAC_CTRLA_DST_WIDTH_4 (2u << 28)
#define DMAC_CTRLA_SRC_WIDTH_4_ALIAS (3u << 24)
#define DMAC_CTRLA_DST_WIDTH_4_ALIAS (3u << 28)
#define DMAC_CTRLB_FC_MEM2PER  (1u << 21)
#define DMAC_CTRLB_FC_PER2MEM  (2u << 21)
#define DMAC_CTRLB_FC_PER2PER  (3u << 21)
#define DMAC_CTRLB_FC_PER2MEM_PER (4u << 21)
#define DMAC_CTRLB_SRC_PIP     (1u << 8)
#define DMAC_CTRLB_DST_PIP     (1u << 12)
#define DMAC_CTRLB_SRC_DECREMENT (1u << 24)
#define DMAC_CTRLB_DST_DECREMENT (1u << 28)
#define DMAC_CTRLB_SRC_FIXED   (2u << 24)
#define DMAC_CTRLB_DST_FIXED   (2u << 28)
#define DMAC_CTRLB_SRC_DSCR_DIS (1u << 16)
#define DMAC_CTRLB_DST_DSCR_DIS (1u << 20)
#define DMAC_CTRLB_IEN         (1u << 30)
#define DMAC_CTRLB_AUTO        (1u << 31)
#define DMAC_CTRLA_DONE        (1u << 31)
#define DMAC_CFG_SRC_REP       (1u << 8)
#define DMAC_CFG_SRC_H2SEL     (1u << 9)
#define DMAC_CFG_DST_REP       (1u << 12)
#define DMAC_CFG_DST_H2SEL     (1u << 13)
#define DMAC_CFG_SOD           (1u << 16)
#define DMAC_BTC(x)            (1u << (x))
#define DMAC_CBTC(x)           (1u << (8 + (x)))
#define DMAC_ERR(x)            (1u << (16 + (x)))
#define DMAC_ENA(x)            (1u << (x))
#define DMAC_SUSP(x)           (1u << (8 + (x)))
#define DMAC_EMPTY(x)          (1u << (16 + (x)))
#define DMAC_STALL(x)          (1u << (24 + (x)))
#define DMAC_RES(x)            (1u << (8 + (x)))
#define DMAC_KEEPON(x)         (1u << (24 + (x)))
#define DMAC_SSREQ(x)          (1u << (2 * (x)))
#define DMAC_DSREQ(x)          (1u << (1 + 2 * (x)))
#define DMAC_SCREQ(x)          (1u << (2 * (x)))
#define DMAC_DCREQ(x)          (1u << (1 + 2 * (x)))
#define DMAC_SLAST(x)          (1u << (2 * (x)))
#define DMAC_PIP(hole, boundary) \
    (((hole) & 0xffffu) | (((boundary) & 0x3ffu) << 16))

#define G45_AIC_QOM_PATH       "/machine/unattached/device[2]"
#define G45_DMAC_QOM_PATH      "/machine/dmac"
#define G45_DMAC_IRQ           21
#define G45_SDRAM_SIZE         UINT64_C(0x08000000)
#define DMAC_TEST_GUARD_SIZE   32

#define HSMCI_ARGR             0x10
#define HSMCI_CR               0x00
#define HSMCI_CMDR             0x14
#define HSMCI_BLKR             0x18
#define HSMCI_RSPR             0x20
#define HSMCI_RDR              0x30
#define HSMCI_TDR              0x34
#define HSMCI_SR               0x40
#define HSMCI_IER              0x44
#define HSMCI_SR_BLKE          (1u << 3)
#define HSMCI_SR_NOTBUSY       (1u << 5)
#define HSMCI_SR_XFRDONE       (1u << 27)
#define HSMCI_CR_MCIEN         (1u << 0)
#define HSMCI_CMDR_RSP_48      (1u << 6)
#define HSMCI_CMDR_RSP_136     (2u << 6)
#define HSMCI_CMDR_MAXLAT_64   (1u << 12)
#define HSMCI_CMDR_START       (1u << 16)
#define HSMCI_CMDR_READ        (1u << 18)
#define HSMCI_CMDR_MULTI       (1u << 19)

typedef struct DmaRoute {
    const char *machine;
    uint64_t dmac_base;
    uint64_t hsmci_base;
    uint64_t ram_base;
    unsigned request_id;
} DmaRoute;

static const DmaRoute routes[] = {
    { "sam9m10g45ek", G45_DMAC_BASE, G45_HSMCI0_BASE,
      G45_SDRAM_BASE, 0 },
    { "sam9m10g45ek", G45_DMAC_BASE, G45_HSMCI1_BASE,
      G45_SDRAM_BASE, 13 },
    { "sam9g35ek", G35_DMAC0_BASE, G35_HSMCI0_BASE,
      G35_SDRAM_BASE, 0 },
    { "sam9g35ek", G35_DMAC1_BASE, G35_HSMCI1_BASE,
      G35_SDRAM_BASE, 0 },
};

static void ensure_vm_running(QTestState *qts)
{
    QDict *response = qtest_qmp(qts, "{ 'execute': 'query-status' }");
    QDict *result = qdict_get_qdict(response, "return");
    bool running = qdict_get_bool(result, "running");

    qobject_unref(response);
    if (!running) {
        qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    }
}

static uint32_t dmac_cfg_src_per(unsigned request_id)
{
    return (request_id & 0xf) | ((request_id & 0x30) << 6);
}

static uint32_t dmac_cfg_dst_per(unsigned request_id)
{
    return ((request_id & 0xf) << 4) | ((request_id & 0x30) << 10);
}

static void wait_for_dmac_channel_disabled(QTestState *qts,
                                           uint64_t dmac_base,
                                           unsigned int channel)
{
    int i;

    for (i = 0; i < 8192; i++) {
        if (!(qtest_readl(qts, dmac_base + DMAC_CHSR) &
              DMAC_ENA(channel))) {
            return;
        }
        qtest_clock_step(qts, 1);
    }
    g_assert_not_reached();
}

static uint64_t configure_hsmci_dma(QTestState *qts, const DmaRoute *route,
                                    bool enable)
{
    const uint64_t lli = route->ram_base + 0x1000;
    const uint64_t dst = route->ram_base + 0x2000;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(4) |
                            DMAC_CTRLA_SRC_WIDTH_4 |
                            DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrlb = DMAC_CTRLB_FC_PER2MEM |
                            DMAC_CTRLB_SRC_FIXED;

    /* One 16-byte peripheral-to-memory descriptor for this controller. */
    qtest_writel(qts, lli + 0, route->hsmci_base + HSMCI_RDR);
    qtest_writel(qts, lli + 4, dst);
    qtest_writel(qts, lli + 8, ctrla);
    qtest_writel(qts, lli + 12, ctrlb);
    qtest_writel(qts, lli + 16, 0);
    qtest_writel(qts, dst, 0xdeadbeef);

    qtest_writel(qts, route->dmac_base + DMAC_EN, 1);
    qtest_writel(qts, route->dmac_base + DMAC_CH0_BASE + DMAC_DSCR, lli);
    qtest_writel(qts, route->dmac_base + DMAC_CH0_BASE + DMAC_CFG,
                 dmac_cfg_src_per(route->request_id) | DMAC_CFG_SRC_H2SEL);
    if (enable) {
        qtest_writel(qts, route->dmac_base + DMAC_CHER, 1);
        qtest_clock_step(qts, 1);

        /* Linux arms DMA before issuing MMC.  The channel must wait. */
        g_assert_cmphex(qtest_readl(qts, route->dmac_base + DMAC_CHSR) & 1,
                        ==, 1);
    }
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);

    return dst;
}

static void start_hsmci_read(QTestState *qts, const DmaRoute *route)
{
    ensure_vm_running(qts);
    qtest_writel(qts, route->hsmci_base + HSMCI_CR, HSMCI_CR_MCIEN);
    qtest_writel(qts, route->hsmci_base + HSMCI_BLKR,
                 (16u << 16) | 1);
    qtest_writel(qts, route->hsmci_base + HSMCI_ARGR, 0);
    qtest_writel(qts, route->hsmci_base + HSMCI_CMDR,
                 17 | HSMCI_CMDR_START | HSMCI_CMDR_READ);
    qtest_clock_step(qts, 2000);
}

static void test_hsmci_dma_waits_for_request(const void *data)
{
    const DmaRoute *route = data;
    QTestState *qts = qtest_initf("-machine %s -S", route->machine);
    uint64_t dst = configure_hsmci_dma(qts, route, true);

    /* CMD17 starts a 16-byte read and raises the HSMCI DMA request. */
    start_hsmci_read(qts, route);
    wait_for_dmac_channel_disabled(qts, route->dmac_base, 0);

    g_assert_cmphex(qtest_readl(qts, route->dmac_base + DMAC_CHSR) & 1,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0);

    qtest_quit(qts);
}

static void test_hsmci_tx_dma_request(void)
{
    const DmaRoute *route = &routes[1];
    QTestState *qts = qtest_initf("-machine %s -S", route->machine);
    const uint64_t lli = route->ram_base + 0x3000;
    const uint64_t src = route->ram_base + 0x4000;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(4) |
                            DMAC_CTRLA_SRC_WIDTH_4 |
                            DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrlb = DMAC_CTRLB_FC_MEM2PER |
                            DMAC_CTRLB_DST_FIXED;

    ensure_vm_running(qts);
    qtest_writel(qts, src + 0, 0x03020100);
    qtest_writel(qts, src + 4, 0x07060504);
    qtest_writel(qts, src + 8, 0x0b0a0908);
    qtest_writel(qts, src + 12, 0x0f0e0d0c);
    qtest_writel(qts, lli + 0, src);
    qtest_writel(qts, lli + 4, route->hsmci_base + HSMCI_TDR);
    qtest_writel(qts, lli + 8, ctrla);
    qtest_writel(qts, lli + 12, ctrlb);
    qtest_writel(qts, lli + 16, 0);

    qtest_writel(qts, route->dmac_base + DMAC_CH0_BASE + DMAC_DSCR, lli);
    qtest_writel(qts, route->dmac_base + DMAC_CH0_BASE + DMAC_CFG,
                 dmac_cfg_dst_per(route->request_id) | DMAC_CFG_DST_H2SEL);
    qtest_writel(qts, route->dmac_base + DMAC_EN, 1);
    qtest_writel(qts, route->dmac_base + DMAC_CHER, 1);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, route->dmac_base + DMAC_CHSR) & 1,
                    ==, 1);

    qtest_writel(qts, route->hsmci_base + HSMCI_BLKR,
                 (16u << 16) | 1);
    qtest_writel(qts, route->hsmci_base + HSMCI_CR, HSMCI_CR_MCIEN);
    qtest_writel(qts, route->hsmci_base + HSMCI_ARGR, 0);
    qtest_writel(qts, route->hsmci_base + HSMCI_CMDR,
                 24 | HSMCI_CMDR_START);
    qtest_clock_step(qts, 2000);
    wait_for_dmac_channel_disabled(qts, route->dmac_base, 0);

    g_assert_cmphex(qtest_readl(qts, route->dmac_base + DMAC_CHSR) & 1,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, route->hsmci_base + HSMCI_SR) &
                    HSMCI_SR_BLKE, ==, HSMCI_SR_BLKE);
    qtest_quit(qts);
}

static void hsmci_command(QTestState *qts, uint32_t command,
                          uint32_t argument)
{
    qtest_writel(qts, G45_HSMCI0_BASE + HSMCI_ARGR, argument);
    qtest_writel(qts, G45_HSMCI0_BASE + HSMCI_CMDR, command);
    qtest_clock_step(qts, 4000);
}

static uint32_t hsmci_select_sd_card(QTestState *qts)
{
    uint32_t rca;

    qtest_writel(qts, G45_HSMCI0_BASE + HSMCI_CR, HSMCI_CR_MCIEN);
    hsmci_command(qts, 0, 0);
    hsmci_command(qts, 55 | HSMCI_CMDR_RSP_48, 0);
    hsmci_command(qts, 41 | HSMCI_CMDR_RSP_48, 0x00ff8000);
    hsmci_command(qts, 2 | HSMCI_CMDR_RSP_136, 0);
    hsmci_command(qts, 3 | HSMCI_CMDR_RSP_48, 0);
    rca = qtest_readl(qts, G45_HSMCI0_BASE + HSMCI_RSPR) & 0xffff0000;
    g_assert_cmphex(rca, !=, 0);
    hsmci_command(qts, 7 | HSMCI_CMDR_RSP_48, rca);
    return rca;
}

/*
 * Mirror the Linux atmel-mci + at_hdmac ACMD13 SD-Status read: a 64-byte
 * DMA read through an LLI armed before the command, completed by the
 * HSMCI XFRDONE/NOTBUSY status the driver sleeps on.  mmc_read_ssr() is
 * the first DMA transfer of every Linux boot, so a missing completion
 * here is exactly "mmc0: problem reading SD Status register".
 */
static void test_hsmci_acmd13_completion(void)
{
    g_autofree char *image_path = NULL;
    const uint64_t lli = G45_SDRAM_BASE + 0x30000;
    const uint64_t dst = G45_SDRAM_BASE + 0x31000;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(16) |
                           DMAC_CTRLA_SRC_WIDTH_4 |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrlb = DMAC_CTRLB_FC_PER2MEM |
                           DMAC_CTRLB_SRC_FIXED;
    const unsigned int hsmci_irq = 11;
    QTestState *qts;
    uint32_t status;
    uint32_t rca;
    int fd;
    int i;

    fd = g_file_open_tmp("at91-dmac-ssr-XXXXXX.img", &image_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(ftruncate(fd, 64 * MiB), ==, 0);
    close(fd);

    qts = qtest_initf("-machine sam9m10g45ek -S "
                      "-drive if=sd,index=0,format=raw,file=%s", image_path);
    ensure_vm_running(qts);
    qtest_irq_intercept_in(qts, G45_AIC_QOM_PATH);
    rca = hsmci_select_sd_card(qts);

    qtest_memset(qts, dst, 0xcc, 64);
    qtest_writel(qts, lli + 0, G45_HSMCI0_BASE + HSMCI_RDR);
    qtest_writel(qts, lli + 4, dst);
    qtest_writel(qts, lli + 8, ctrla);
    qtest_writel(qts, lli + 12, ctrlb);
    qtest_writel(qts, lli + 16, 0);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_DSCR, lli);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_CFG,
                 dmac_cfg_src_per(0) | DMAC_CFG_SRC_H2SEL);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));

    qtest_writel(qts, G45_HSMCI0_BASE + HSMCI_BLKR, (64u << 16) | 1);
    hsmci_command(qts, 55 | HSMCI_CMDR_RSP_48, rca);
    hsmci_command(qts, 13 | HSMCI_CMDR_RSP_48 | HSMCI_CMDR_MAXLAT_64 |
                  HSMCI_CMDR_START | HSMCI_CMDR_READ, 0);
    wait_for_dmac_channel_disabled(qts, G45_DMAC_BASE, 0);

    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR) &
                    (DMAC_BTC(0) | DMAC_ERR(0)), ==, DMAC_BTC(0));
    for (i = 0; i < 64; i += 4) {
        g_assert_cmphex(qtest_readl(qts, dst + i), !=, 0xcccccccc);
    }

    /* The driver enables NOTBUSY and sleeps; the data-end IRQ must fire. */
    for (i = 0; i < 8192; i++) {
        status = qtest_readl(qts, G45_HSMCI0_BASE + HSMCI_SR);
        if ((status & (HSMCI_SR_XFRDONE | HSMCI_SR_NOTBUSY)) ==
            (HSMCI_SR_XFRDONE | HSMCI_SR_NOTBUSY)) {
            break;
        }
        qtest_clock_step(qts, 1000);
    }
    g_assert_cmphex(status & (HSMCI_SR_XFRDONE | HSMCI_SR_NOTBUSY), ==,
                    HSMCI_SR_XFRDONE | HSMCI_SR_NOTBUSY);
    qtest_writel(qts, G45_HSMCI0_BASE + HSMCI_IER, HSMCI_SR_NOTBUSY);
    g_assert_true(qtest_get_irq(qts, hsmci_irq));

    qtest_quit(qts);
    unlink(image_path);
}

static void test_hsmci_dma_block_refill_reentrancy(void)
{
    g_autofree char *image_path = NULL;
    const uint64_t lli = G45_SDRAM_BASE + 0x10000;
    const uint64_t dst = G45_SDRAM_BASE + 0x20000;
    const uint64_t guard = dst + 4096;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(1024) |
                           DMAC_CTRLA_SRC_WIDTH_4 |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrlb = DMAC_CTRLB_FC_PER2MEM |
                           DMAC_CTRLB_SRC_FIXED |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    uint8_t image_data[4096];
    uint8_t dma_data[4096];
    uint8_t guard_data[64];
    QTestState *qts;
    int fd;
    int i;

    for (i = 0; i < sizeof(image_data); i++) {
        image_data[i] = i ^ (i >> 8);
    }
    memset(guard_data, 0xa5, sizeof(guard_data));

    fd = g_file_open_tmp("at91-dmac-sd-XXXXXX.img", &image_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(ftruncate(fd, 64 * MiB), ==, 0);
    g_assert_cmpint(pwrite(fd, image_data, sizeof(image_data), 0), ==,
                    sizeof(image_data));
    close(fd);

    qts = qtest_initf("-machine sam9m10g45ek -S "
                      "-drive if=sd,index=0,format=raw,file=%s", image_path);
    ensure_vm_running(qts);
    hsmci_select_sd_card(qts);

    qtest_memset(qts, dst, 0xcc, sizeof(dma_data));
    qtest_memwrite(qts, guard, guard_data, sizeof(guard_data));
    qtest_writel(qts, lli + 0, G45_HSMCI0_BASE + HSMCI_RDR);
    qtest_writel(qts, lli + 4, dst);
    qtest_writel(qts, lli + 8, ctrla);
    qtest_writel(qts, lli + 12, ctrlb);
    qtest_writel(qts, lli + 16, 0);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_DSCR, lli);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_CFG,
                 dmac_cfg_src_per(0) | DMAC_CFG_SRC_H2SEL);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));

    qtest_writel(qts, G45_HSMCI0_BASE + HSMCI_BLKR,
                 (512u << 16) | 8);
    hsmci_command(qts, 18 | HSMCI_CMDR_RSP_48 | HSMCI_CMDR_MAXLAT_64 |
                  HSMCI_CMDR_START | HSMCI_CMDR_READ |
                  HSMCI_CMDR_MULTI, 0);
    wait_for_dmac_channel_disabled(qts, G45_DMAC_BASE, 0);

    qtest_memread(qts, dst, dma_data, sizeof(dma_data));
    g_assert_cmpmem(dma_data, sizeof(dma_data),
                    image_data, sizeof(image_data));
    qtest_memread(qts, guard, dma_data, sizeof(guard_data));
    g_assert_cmpmem(dma_data, sizeof(guard_data),
                    guard_data, sizeof(guard_data));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR) &
                    DMAC_ERR(0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, lli + 8), ==,
                    (ctrla & ~0xffffu) | DMAC_CTRLA_DONE);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(0), ==, 0);

    qtest_quit(qts);
    unlink(image_path);
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

static void dmac_program_one_word(QTestState *qts, unsigned int channel,
                                  uint64_t source, uint64_t destination)
{
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;

    qtest_writel(qts, channel_base + DMAC_SADDR, source);
    qtest_writel(qts, channel_base + DMAC_DADDR, destination);
    qtest_writel(qts, channel_base + DMAC_DSCR, 0);
    qtest_writel(qts, channel_base + DMAC_CTRLA,
                 DMAC_CTRLA_BTSIZE(1) |
                 DMAC_CTRLA_SRC_WIDTH_4 |
                 DMAC_CTRLA_DST_WIDTH_4);
    qtest_writel(qts, channel_base + DMAC_CTRLB,
                 DMAC_CTRLB_SRC_DSCR_DIS |
                 DMAC_CTRLB_DST_DSCR_DIS);
}

static uint8_t dmac_test_pattern(uint32_t seed, size_t offset)
{
    uint32_t value = seed ^ (offset * UINT32_C(0x9e3779b9));

    value ^= value >> 16;
    value *= UINT32_C(0x7feb352d);
    value ^= value >> 15;
    return value ^ (value >> 8) ^ (value >> 24);
}

static void dmac_fill_test_pattern(uint8_t *buffer, size_t length,
                                   uint32_t seed)
{
    size_t offset;

    for (offset = 0; offset < length; offset++) {
        buffer[offset] = dmac_test_pattern(seed, offset);
    }
}

static void dmac_run_memory_copy(QTestState *qts, unsigned int channel,
                                 uint64_t source, uint64_t destination,
                                 uint32_t ctrla, uint32_t ctrlb,
                                 uint32_t expected_source,
                                 uint32_t expected_destination)
{
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    uint32_t events;

    (void)qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    qtest_writel(qts, channel_base + DMAC_SADDR, source);
    qtest_writel(qts, channel_base + DMAC_DADDR, destination);
    qtest_writel(qts, channel_base + DMAC_DSCR, 0);
    qtest_writel(qts, channel_base + DMAC_CTRLA, ctrla);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, channel_base + DMAC_CFG, 0x01000000);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);

    events = qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    g_assert_cmphex(events, ==, DMAC_BTC(channel));
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_SADDR), ==,
                    expected_source);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DADDR), ==,
                    expected_destination);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    (DMAC_ENA(channel) | DMAC_EMPTY(channel)), ==,
                    DMAC_EMPTY(channel));
}

static void dmac_assert_guard(QTestState *qts, uint64_t address,
                              const uint8_t *expected)
{
    uint8_t actual[DMAC_TEST_GUARD_SIZE];

    qtest_memread(qts, address, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual), expected, DMAC_TEST_GUARD_SIZE);
}

static void dmac_assert_nonoverlap_copy(QTestState *qts, unsigned int channel,
                                        uint64_t source, uint64_t destination,
                                        uint32_t transfers, unsigned int width,
                                        uint32_t seed, bool guard_after)
{
    g_autofree uint8_t *source_data = NULL;
    g_autofree uint8_t *source_after = NULL;
    g_autofree uint8_t *destination_data = NULL;
    uint8_t before[DMAC_TEST_GUARD_SIZE];
    uint8_t after[DMAC_TEST_GUARD_SIZE];
    size_t length = (size_t)transfers * width;
    uint32_t widths = width == 4 ?
                      DMAC_CTRLA_SRC_WIDTH_4 | DMAC_CTRLA_DST_WIDTH_4 : 0;
    size_t offset;

    source_data = g_malloc(length);
    source_after = g_malloc(length);
    destination_data = g_malloc(length);
    dmac_fill_test_pattern(source_data, length, seed);
    memset(destination_data, 0xd3, length);
    for (offset = 0; offset < DMAC_TEST_GUARD_SIZE; offset++) {
        before[offset] = 0x69 ^ (offset * 0x35);
        after[offset] = 0xee ^ (offset * 0x35);
    }

    qtest_memwrite(qts, source, source_data, length);
    qtest_memwrite(qts, destination - sizeof(before), before, sizeof(before));
    qtest_memwrite(qts, destination, destination_data, length);
    if (guard_after) {
        qtest_memwrite(qts, destination + length, after, sizeof(after));
    }

    dmac_run_memory_copy(qts, channel, source, destination,
                         DMAC_CTRLA_BTSIZE(transfers) | widths,
                         DMAC_CTRLB_SRC_DSCR_DIS |
                         DMAC_CTRLB_DST_DSCR_DIS,
                         source + length, destination + length);

    qtest_memread(qts, destination, destination_data, length);
    g_assert_cmpmem(destination_data, length, source_data, length);
    qtest_memread(qts, source, source_after, length);
    g_assert_cmpmem(source_after, length, source_data, length);
    dmac_assert_guard(qts, destination - sizeof(before), before);
    if (guard_after) {
        dmac_assert_guard(qts, destination + length, after);
    }
}

static void test_dmac_maximum_and_boundaries(void)
{
    static const struct {
        uint64_t source;
        uint64_t destination;
        uint32_t transfers;
        unsigned int width;
    } cases[] = {
        { G45_SDRAM_BASE + 0x00003ff0, G45_SDRAM_BASE + 0x00013ff0,
          0x40, 1 },
        { G45_SDRAM_BASE + 0x0000fff0, G45_SDRAM_BASE + 0x0002fff0,
          0x40, 1 },
        { G45_SDRAM_BASE + 0x000ffff0, G45_SDRAM_BASE + 0x002ffff0,
          0x40, 1 },
        { G45_SDRAM_BASE + 0x010e0000, G45_SDRAM_BASE + 0x012e0000,
          0xffff, 4 },
        { G45_SDRAM_BASE + 0x0500000,
          G45_SDRAM_BASE + G45_SDRAM_SIZE - 0x1000, 0x1000, 1 },
    };
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(cases); i++) {
        dmac_assert_nonoverlap_copy(qts, i, cases[i].source,
                                    cases[i].destination, cases[i].transfers,
                                    cases[i].width,
                                    UINT32_C(0xb07d0000) ^ i,
                                    i + 1 < ARRAY_SIZE(cases));
    }
    qtest_quit(qts);
}

static void dmac_assert_overlap_copy(QTestState *qts, uint64_t region,
                                     size_t region_size, size_t source_offset,
                                     size_t destination_offset, size_t length,
                                     bool decrement, uint32_t seed)
{
    g_autofree uint8_t *initial = g_malloc(region_size);
    g_autofree uint8_t *expected = g_malloc(region_size);
    g_autofree uint8_t *actual = g_malloc(region_size);
    uint8_t before[DMAC_TEST_GUARD_SIZE];
    uint8_t after[DMAC_TEST_GUARD_SIZE];
    uint64_t source = region + source_offset;
    uint64_t destination = region + destination_offset;
    uint32_t ctrlb = DMAC_CTRLB_SRC_DSCR_DIS |
                     DMAC_CTRLB_DST_DSCR_DIS;
    size_t offset;

    dmac_fill_test_pattern(initial, region_size, seed);
    memcpy(expected, initial, region_size);
    memmove(expected + destination_offset, expected + source_offset, length);
    for (offset = 0; offset < DMAC_TEST_GUARD_SIZE; offset++) {
        before[offset] = 0x96 ^ (offset * 0x53);
        after[offset] = 0x71 ^ (offset * 0x53);
    }

    qtest_memwrite(qts, region - sizeof(before), before, sizeof(before));
    qtest_memwrite(qts, region, initial, region_size);
    qtest_memwrite(qts, region + region_size, after, sizeof(after));
    if (decrement) {
        source += length - 1;
        destination += length - 1;
        ctrlb |= DMAC_CTRLB_SRC_DECREMENT | DMAC_CTRLB_DST_DECREMENT;
    }

    dmac_run_memory_copy(qts, 7, source, destination,
                         DMAC_CTRLA_BTSIZE(length), ctrlb,
                         decrement ? region + source_offset - 1 :
                                     region + source_offset + length,
                         decrement ? region + destination_offset - 1 :
                                     region + destination_offset + length);

    qtest_memread(qts, region, actual, region_size);
    g_assert_cmpmem(actual, region_size, expected, region_size);
    dmac_assert_guard(qts, region - sizeof(before), before);
    dmac_assert_guard(qts, region + region_size, after);
}

static void test_dmac_overlap_traversal(void)
{
    const uint64_t region = G45_SDRAM_BASE + 0x0600000;
    const size_t region_size = 9 * KiB;
    const size_t overlap_offset = 1 * KiB;
    const size_t length = 8 * KiB;
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");

    /* Increment safely when destination is below the overlapping source. */
    dmac_assert_overlap_copy(qts, region, region_size, overlap_offset, 0,
                             length, false, UINT32_C(0xf04d0001));

    /* Decrement safely when destination is above the overlapping source. */
    dmac_assert_overlap_copy(qts, region, region_size, 0, overlap_offset,
                             length, true, UINT32_C(0xbac40002));
    qtest_quit(qts);
}

static void test_dmac_arbitration_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    const uint64_t source0 = G45_SDRAM_BASE + 0x30000;
    const uint64_t source1 = G45_SDRAM_BASE + 0x30004;
    const uint64_t seed_destination = G45_SDRAM_BASE + 0x31000;
    const uint64_t shared_destination = G45_SDRAM_BASE + 0x31004;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("at91-dmac-arbitration-migration-XXXXXX",
                         &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    qtest_writel(src, source0, 0x11111111);
    qtest_writel(src, source1, 0x22222222);
    qtest_writel(src, seed_destination, 0xdeadbeef);
    dmac_program_one_word(src, 0, source0, seed_destination);
    qtest_writel(src, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(src, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(src, 1);
    g_assert_cmphex(qtest_readl(src, seed_destination), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(src, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_GCFG), ==, 0x10);

    /*
     * Channel 0 was the last round-robin grant before migration.  With both
     * one-word channels pending, channel 1 must run first and channel 0 last.
     */
    qtest_writel(dst, shared_destination, 0xdeadbeef);
    dmac_program_one_word(dst, 0, source0, shared_destination);
    dmac_program_one_word(dst, 1, source1, shared_destination);
    qtest_writel(dst, G45_DMAC_BASE + DMAC_CHER,
                 DMAC_ENA(0) | DMAC_ENA(1));
    qtest_clock_step(dst, 1);
    g_assert_cmphex(qtest_readl(dst, shared_destination), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0) | DMAC_BTC(1));

    /* Fixed priority grants the lower channel first.  Channel 1 writes last. */
    qtest_writel(dst, G45_DMAC_BASE + DMAC_GCFG, 0);
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_GCFG), ==, 0);
    qtest_writel(dst, shared_destination, 0xdeadbeef);
    dmac_program_one_word(dst, 0, source0, shared_destination);
    dmac_program_one_word(dst, 1, source1, shared_destination);
    qtest_writel(dst, G45_DMAC_BASE + DMAC_CHER,
                 DMAC_ENA(0) | DMAC_ENA(1));
    qtest_clock_step(dst, 1);
    g_assert_cmphex(qtest_readl(dst, shared_destination), ==, 0x22222222);
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0) | DMAC_BTC(1));

    qtest_quit(dst);
    unlink(state_path);
}

static void test_dmac_reset_contract(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    unsigned int channel;

    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_GCFG), ==, 0x10);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCIMR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR), ==,
                    0x00ff0000);
    for (channel = 0; channel < 8; channel++) {
        g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CH0_BASE +
                                    channel * DMAC_CH_STRIDE + DMAC_CFG),
                        ==, 0x01000000);
    }

    qtest_writel(qts, G45_DMAC_BASE + DMAC_GCFG, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_GCFG), ==, 0x10);
    qtest_quit(qts);
}

static void test_dmac_global_enable_gates_transfer(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x5000;
    const uint64_t dst = G45_SDRAM_BASE + 0x6000;
    const uint32_t ctrlb = DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;

    qtest_writel(qts, src, 0x76543210);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_SADDR, src);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_DADDR, dst);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_DSCR, 0);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_CTRLA, 4);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_CTRLB, ctrlb);

    /* CHER cannot start a channel while the global enable is clear. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, 1);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR) & 1,
                    ==, 0);

    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHDR, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, 1);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x76543210);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR) & 1,
                    ==, 1);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR), ==,
                    0x00ff0000);
    qtest_quit(qts);
}

static void test_dmac_linked_list_writeback(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t lli0 = G45_SDRAM_BASE + 0x7000;
    const uint64_t lli1 = G45_SDRAM_BASE + 0x7020;
    const uint64_t src = G45_SDRAM_BASE + 0x8000;
    const uint64_t dst = G45_SDRAM_BASE + 0x9000;
    const uint32_t intermediate = DMAC_CTRLB_IEN;
    const uint32_t last = DMAC_CTRLB_SRC_DSCR_DIS |
                          DMAC_CTRLB_DST_DSCR_DIS;
    const unsigned int channel = 3;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;

    qtest_writel(qts, src + 0, 0x03020100);
    qtest_writel(qts, src + 4, 0x07060504);
    qtest_writel(qts, dst + 0, 0xdeadbeef);
    qtest_writel(qts, dst + 4, 0xdeadbeef);

    qtest_writel(qts, lli0 + 0, src);
    qtest_writel(qts, lli0 + 4, dst);
    qtest_writel(qts, lli0 + 8, 4);
    qtest_writel(qts, lli0 + 12, intermediate);
    qtest_writel(qts, lli0 + 16, lli1);
    qtest_writel(qts, lli1 + 0, src + 4);
    qtest_writel(qts, lli1 + 4, dst + 4);
    qtest_writel(qts, lli1 + 8, 4);
    qtest_writel(qts, lli1 + 12, last);
    qtest_writel(qts, lli1 + 16, 0);

    qtest_writel(qts, channel_base + DMAC_DSCR, lli0);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, 1u << channel);
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, dst + 0), ==, 0x03020100);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0x07060504);
    g_assert_cmphex(qtest_readl(qts, lli0 + 8), ==, DMAC_CTRLA_DONE);
    g_assert_cmphex(qtest_readl(qts, lli1 + 8), ==, DMAC_CTRLA_DONE);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DSCR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR) &
                    ((1u << channel) | (1u << (8 + channel))), ==,
                    (1u << channel) | (1u << (8 + channel)));
    qtest_quit(qts);
}

static void test_dmac_stop_on_done(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x1a000;
    const uint64_t dst = G45_SDRAM_BASE + 0x1b000;
    const uint64_t lli = G45_SDRAM_BASE + 0x1c000;
    const uint32_t channel = 3;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(4) | DMAC_CTRLA_DONE;

    qtest_writel(qts, src, 0x13579bdf);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, lli + 0, src);
    qtest_writel(qts, lli + 4, dst);
    qtest_writel(qts, lli + 8, ctrla);
    qtest_writel(qts, lli + 12, 0);
    qtest_writel(qts, lli + 16, 0);
    qtest_writel(qts, channel_base + DMAC_DSCR, lli);
    qtest_writel(qts, channel_base + DMAC_CFG, 0x01000000 | DMAC_CFG_SOD);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(channel), ==, 0);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA), ==, ctrla);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==, 0);

    /* With SOD clear, the same DONE descriptor is deliberately executed. */
    qtest_writel(qts, channel_base + DMAC_DSCR, lli);
    qtest_writel(qts, channel_base + DMAC_CFG, 0x01000000);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x13579bdf);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel) | DMAC_CBTC(channel));
    qtest_quit(qts);
}

static void test_dmac_partial_descriptor_reload(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x20000;
    const uint64_t dst = G45_SDRAM_BASE + 0x21000;
    const uint64_t poison = G45_SDRAM_BASE + 0x22000;
    const uint64_t lli = G45_SDRAM_BASE + 0x23000;
    const uint64_t channel0 = G45_DMAC_BASE + DMAC_CH0_BASE;
    const uint64_t channel1 = channel0 + DMAC_CH_STRIDE;
    const uint32_t both_events = DMAC_BTC(0) | DMAC_BTC(1) |
                                 DMAC_CBTC(0) | DMAC_CBTC(1);

    qtest_writel(qts, src + 0, 0x11111111);
    qtest_writel(qts, src + 4, 0x22222222);
    qtest_writel(qts, src + 8, 0x33333333);
    qtest_writel(qts, src + 12, 0x44444444);
    qtest_writel(qts, dst + 0, 0xdeadbeef);
    qtest_writel(qts, dst + 4, 0xdeadbeef);
    qtest_writel(qts, dst + 8, 0xdeadbeef);
    qtest_writel(qts, dst + 12, 0xdeadbeef);
    qtest_writel(qts, poison + 0, 0xa5a5a5a5);
    qtest_writel(qts, poison + 4, 0x5a5a5a5a);

    /* Channel 0 keeps its incremented destination for the second LLI. */
    qtest_writel(qts, lli + 0, src + 0);
    qtest_writel(qts, lli + 4, dst + 0);
    qtest_writel(qts, lli + 8, DMAC_CTRLA_BTSIZE(4));
    qtest_writel(qts, lli + 12, DMAC_CTRLB_DST_DSCR_DIS);
    qtest_writel(qts, lli + 16, lli + 0x20);
    qtest_writel(qts, lli + 0x20, src + 4);
    qtest_writel(qts, lli + 0x24, poison + 0);
    qtest_writel(qts, lli + 0x28, DMAC_CTRLA_BTSIZE(4));
    qtest_writel(qts, lli + 0x2c,
                 DMAC_CTRLB_SRC_DSCR_DIS | DMAC_CTRLB_DST_DSCR_DIS);
    qtest_writel(qts, lli + 0x30, 0);

    /* Channel 1 keeps its incremented source for the second LLI. */
    qtest_writel(qts, lli + 0x40, src + 8);
    qtest_writel(qts, lli + 0x44, dst + 8);
    qtest_writel(qts, lli + 0x48, DMAC_CTRLA_BTSIZE(4));
    qtest_writel(qts, lli + 0x4c, DMAC_CTRLB_SRC_DSCR_DIS);
    qtest_writel(qts, lli + 0x50, lli + 0x60);
    qtest_writel(qts, lli + 0x60, poison + 4);
    qtest_writel(qts, lli + 0x64, dst + 12);
    qtest_writel(qts, lli + 0x68, DMAC_CTRLA_BTSIZE(4));
    qtest_writel(qts, lli + 0x6c,
                 DMAC_CTRLB_SRC_DSCR_DIS | DMAC_CTRLB_DST_DSCR_DIS);
    qtest_writel(qts, lli + 0x70, 0);

    qtest_writel(qts, channel0 + DMAC_DSCR, lli);
    qtest_writel(qts, channel1 + DMAC_DSCR, lli + 0x40);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER,
                 DMAC_ENA(0) | DMAC_ENA(1));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, dst + 0), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0x22222222);
    g_assert_cmphex(qtest_readl(qts, dst + 8), ==, 0x33333333);
    g_assert_cmphex(qtest_readl(qts, dst + 12), ==, 0x44444444);
    g_assert_cmphex(qtest_readl(qts, poison + 0), ==, 0xa5a5a5a5);
    g_assert_cmphex(qtest_readl(qts, poison + 4), ==, 0x5a5a5a5a);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    both_events);
    qtest_quit(qts);
}

static void test_dmac_descriptor_auto_btsize_replay(void)
{
    static const uint8_t row7_source[] = {
        0x10, 0x11, 0x12, 0x13,
        0x20, 0x21, 0x22, 0x23,
        0x30, 0x31, 0x32, 0x33,
    };
    static const uint8_t row8_source[] = {
        0x40, 0x41, 0x42, 0x43,
        0x50, 0x51, 0x52, 0x53,
        0x60, 0x61, 0x62, 0x63,
    };
    static const uint8_t untouched[] = {
        0xee, 0xee, 0xee, 0xee,
    };
    const uint64_t lli = G45_SDRAM_BASE + 0x24000;
    const uint64_t source = G45_SDRAM_BASE + 0x25000;
    const uint64_t destination = G45_SDRAM_BASE + 0x26000;
    const uint64_t poison = G45_SDRAM_BASE + 0x27000;
    const uint64_t channel0 = G45_DMAC_BASE + DMAC_CH0_BASE;
    const uint64_t channel1 = channel0 + DMAC_CH_STRIDE;
    const uint32_t replay_ctrla = DMAC_CTRLA_BTSIZE(4) |
                                  DMAC_CTRLA_SRC_WIDTH_4 |
                                  DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t row7 = DMAC_CTRLB_AUTO | DMAC_CTRLB_DST_DSCR_DIS;
    const uint32_t row8 = DMAC_CTRLB_AUTO | DMAC_CTRLB_SRC_DSCR_DIS;
    const uint32_t last = DMAC_CTRLB_SRC_DSCR_DIS |
                          DMAC_CTRLB_DST_DSCR_DIS;
    uint8_t actual[sizeof(row7_source)];
    unsigned int descriptor;

    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");

    qtest_memwrite(qts, source, row7_source, sizeof(row7_source));
    qtest_memwrite(qts, source + 0x40, row8_source, sizeof(row8_source));
    memset(actual, 0xee, sizeof(actual));
    qtest_memwrite(qts, destination, actual, sizeof(actual));
    memset(actual, 0xba, sizeof(actual));
    qtest_memwrite(qts, poison, actual, sizeof(actual));

    /*
     * Row 7: BTSIZE is replayed, source comes from each LLI and destination
     * remains contiguous.  The LLI sizes and destination addresses are poison.
     */
    for (descriptor = 0; descriptor < 3; descriptor++) {
        uint64_t entry = lli + descriptor * 0x20;

        qtest_writel(qts, entry + 0, source + descriptor * 4);
        qtest_writel(qts, entry + 4, poison + descriptor * 4);
        qtest_writel(qts, entry + 8,
                     DMAC_CTRLA_BTSIZE(descriptor + 1));
        qtest_writel(qts, entry + 12, descriptor == 2 ? last : row7);
        qtest_writel(qts, entry + 16,
                     descriptor == 2 ? 0 : entry + 0x20);
    }

    /*
     * Row 8: BTSIZE is replayed, source remains contiguous and destination
     * comes from each LLI.  The LLI sizes and source addresses are poison.
     */
    for (descriptor = 0; descriptor < 3; descriptor++) {
        uint64_t entry = lli + 0x80 + descriptor * 0x20;

        qtest_writel(qts, entry + 0, poison + descriptor * 4);
        qtest_writel(qts, entry + 4,
                     destination + 0x40 + descriptor * 0x10);
        qtest_writel(qts, entry + 8,
                     DMAC_CTRLA_BTSIZE(descriptor + 1));
        qtest_writel(qts, entry + 12, descriptor == 2 ? last : row8);
        qtest_writel(qts, entry + 16,
                     descriptor == 2 ? 0 : entry + 0x20);
        qtest_memwrite(qts, destination + 0x40 + descriptor * 0x10,
                       untouched, sizeof(untouched));
    }

    qtest_writel(qts, channel0 + DMAC_DADDR, destination);
    qtest_writel(qts, channel0 + DMAC_DSCR, lli);
    qtest_writel(qts, channel0 + DMAC_CTRLA, replay_ctrla);
    qtest_writel(qts, channel0 + DMAC_CTRLB, row7);
    qtest_writel(qts, channel1 + DMAC_SADDR, source + 0x40);
    qtest_writel(qts, channel1 + DMAC_DSCR, lli + 0x80);
    qtest_writel(qts, channel1 + DMAC_CTRLA, replay_ctrla);
    qtest_writel(qts, channel1 + DMAC_CTRLB, row8);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER,
                 DMAC_ENA(0) | DMAC_ENA(1));
    qtest_clock_step(qts, 1);

    qtest_memread(qts, destination, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual),
                    row7_source, sizeof(row7_source));
    qtest_memread(qts, poison, actual, sizeof(actual));
    for (descriptor = 0; descriptor < sizeof(actual); descriptor++) {
        g_assert_cmphex(actual[descriptor], ==, 0xba);
    }
    for (descriptor = 0; descriptor < 3; descriptor++) {
        uint64_t row7_entry = lli + descriptor * 0x20;
        uint64_t row8_entry = lli + 0x80 + descriptor * 0x20;

        qtest_memread(qts, destination + 0x40 + descriptor * 0x10,
                      actual, sizeof(untouched));
        g_assert_cmpmem(actual, sizeof(untouched),
                        row8_source + descriptor * 4, sizeof(untouched));
        g_assert_cmphex(qtest_readl(qts, row7_entry + 8), ==,
                        DMAC_CTRLA_DONE);
        g_assert_cmphex(qtest_readl(qts, row8_entry + 8), ==,
                        DMAC_CTRLA_DONE);
    }
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_DADDR), ==,
                    destination + sizeof(row7_source));
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_SADDR), ==,
                    source + 0x40 + sizeof(row8_source));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0) | DMAC_BTC(1) |
                    DMAC_CBTC(0) | DMAC_CBTC(1));
    qtest_quit(qts);
}

static void test_dmac_descriptor_page_boundary(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t lli0 = G45_SDRAM_BASE + 0x2aff8;
    const uint64_t lli1 = G45_SDRAM_BASE + 0x2b020;
    const uint64_t src = G45_SDRAM_BASE + 0x2c000;
    const uint64_t dst = G45_SDRAM_BASE + 0x2d000;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(1) |
                           DMAC_CTRLA_SRC_WIDTH_4 |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t last = DMAC_CTRLB_SRC_DSCR_DIS |
                          DMAC_CTRLB_DST_DSCR_DIS;

    qtest_writel(qts, src, 0x11223344);
    qtest_writel(qts, src + 4, 0x55667788);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, dst + 4, 0xdeadbeef);

    /*
     * LLI0 starts eight bytes before a page boundary, so CTRLA, CTRLB and
     * DSCR are fetched from the following page.  DSCR[1:0] select the AHB
     * interface and are not descriptor-address bits.
     */
    qtest_writel(qts, lli0 + 0, src);
    qtest_writel(qts, lli0 + 4, dst);
    qtest_writel(qts, lli0 + 8, ctrla);
    qtest_writel(qts, lli0 + 12, 0);
    qtest_writel(qts, lli0 + 16, lli1 | 2);
    qtest_writel(qts, lli1 + 0, src + 4);
    qtest_writel(qts, lli1 + 4, dst + 4);
    qtest_writel(qts, lli1 + 8, ctrla);
    qtest_writel(qts, lli1 + 12, last);
    qtest_writel(qts, lli1 + 16, 0);

    qtest_writel(qts, channel_base + DMAC_DSCR, lli0 | 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x11223344);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0x55667788);
    g_assert_cmphex(qtest_readl(qts, lli0 + 8), ==,
                    DMAC_CTRLA_SRC_WIDTH_4 |
                    DMAC_CTRLA_DST_WIDTH_4 | DMAC_CTRLA_DONE);
    g_assert_cmphex(qtest_readl(qts, lli1 + 8), ==,
                    DMAC_CTRLA_SRC_WIDTH_4 |
                    DMAC_CTRLA_DST_WIDTH_4 | DMAC_CTRLA_DONE);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DSCR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0) | DMAC_CBTC(0));
    qtest_quit(qts);
}

static void test_dmac_cyclic_descriptor_policy(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t self = G45_SDRAM_BASE + 0x2e000;
    const uint64_t head = G45_SDRAM_BASE + 0x2e100;
    const uint64_t tail = G45_SDRAM_BASE + 0x2e120;
    const uint64_t src = G45_SDRAM_BASE + 0x2f000;
    const uint64_t dst = G45_SDRAM_BASE + 0x30000;
    const uint64_t channel0 = G45_DMAC_BASE + DMAC_CH0_BASE;
    const uint64_t channel1 = channel0 + DMAC_CH_STRIDE;
    const uint32_t channels = DMAC_ENA(0) | DMAC_ENA(1);
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(1) |
                           DMAC_CTRLA_SRC_WIDTH_4 |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t single = DMAC_CTRLB_SRC_DSCR_DIS |
                            DMAC_CTRLB_DST_DSCR_DIS;
    uint32_t status;

    qtest_writel(qts, src, 0x11223344);
    qtest_writel(qts, src + 4, 0x55667788);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, dst + 4, 0xdeadbeef);

    /* Channel 0 is a conventional one-descriptor cyclic ring. */
    qtest_writel(qts, self + 0, src);
    qtest_writel(qts, self + 4, dst);
    qtest_writel(qts, self + 8, ctrla);
    qtest_writel(qts, self + 12, 0);
    qtest_writel(qts, self + 16, self);

    /*
     * Channel 1 has a malformed tail cycle which does not return to its head.
     * It is still an indefinite graph and must be parked rather than executed
     * repeatedly or converted into a synthetic transfer error.
     */
    qtest_writel(qts, head + 0, src);
    qtest_writel(qts, head + 4, dst);
    qtest_writel(qts, head + 8, ctrla);
    qtest_writel(qts, head + 12, 0);
    qtest_writel(qts, head + 16, tail);
    qtest_writel(qts, tail + 0, src + 4);
    qtest_writel(qts, tail + 4, dst + 4);
    qtest_writel(qts, tail + 8, ctrla);
    qtest_writel(qts, tail + 12, 0);
    qtest_writel(qts, tail + 16, tail);

    qtest_writel(qts, channel0 + DMAC_DSCR, self);
    qtest_writel(qts, channel1 + DMAC_DSCR, head);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, channels);
    qtest_clock_step(qts, 1);

    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & (channels | DMAC_EMPTY(0) | DMAC_EMPTY(1)), ==,
                    channels | DMAC_EMPTY(0) | DMAC_EMPTY(1));
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_DSCR), ==, self);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_DSCR), ==, tail);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_CTRLA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_CTRLA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==, 0);

    /* Explicit termination clears the parked state and permits immediate use. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHDR, channels);
    qtest_writel(qts, channel0 + DMAC_SADDR, src);
    qtest_writel(qts, channel0 + DMAC_DADDR, dst);
    qtest_writel(qts, channel0 + DMAC_DSCR, 0);
    qtest_writel(qts, channel0 + DMAC_CTRLA, ctrla);
    qtest_writel(qts, channel0 + DMAC_CTRLB, single);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x11223344);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0));
    qtest_quit(qts);
}

static void test_dmac_row1_terminates_nonzero_next(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t lli = G45_SDRAM_BASE + 0x31000;
    const uint64_t src = G45_SDRAM_BASE + 0x32000;
    const uint64_t dst = G45_SDRAM_BASE + 0x33000;
    const uint64_t invalid = 0x90000000;
    const uint32_t channel = 4;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(1) |
                           DMAC_CTRLA_SRC_WIDTH_4 |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t row1 = DMAC_CTRLB_SRC_DSCR_DIS |
                          DMAC_CTRLB_DST_DSCR_DIS;
    uint32_t status;

    qtest_writel(qts, src, 0x13579bdf);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, lli + 0, src);
    qtest_writel(qts, lli + 4, dst);
    qtest_writel(qts, lli + 8, ctrla);
    qtest_writel(qts, lli + 12, row1);

    /*
     * Table 40-2 row 1 terminates after this buffer.  A nonzero next pointer
     * is deliberately poisoned to prove row classification, not DSCR alone,
     * controls termination.
     */
    qtest_writel(qts, lli + 16, invalid);
    qtest_writel(qts, channel_base + DMAC_DSCR, lli);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x13579bdf);
    g_assert_cmphex(qtest_readl(qts, lli + 8), ==,
                    (ctrla & ~0xffffu) | DMAC_CTRLA_DONE);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DSCR), ==, invalid);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & DMAC_ENA(channel), ==, 0);
    g_assert_cmphex(status & DMAC_EMPTY(channel), ==, DMAC_EMPTY(channel));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel) | DMAC_CBTC(channel));
    qtest_quit(qts);
}

static void test_dmac_live_next_descriptor_update(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t lli0 = G45_SDRAM_BASE + 0x34000;
    const uint64_t lli1 = G45_SDRAM_BASE + 0x34020;
    const uint64_t src = G45_SDRAM_BASE + 0x35000;
    const uint64_t dst = G45_SDRAM_BASE + 0x36000;
    const uint64_t poison = G45_SDRAM_BASE + 0x37000;
    const uint64_t invalid = 0x90000000;
    const uint32_t channel = 5;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t ctrla0 = DMAC_CTRLA_BTSIZE(2) |
                            DMAC_CTRLA_SRC_WIDTH_4 |
                            DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrla1 = DMAC_CTRLA_BTSIZE(1) |
                            DMAC_CTRLA_SRC_WIDTH_4 |
                            DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t paced = DMAC_CTRLB_FC_PER2MEM;
    const uint32_t last = paced | DMAC_CTRLB_SRC_DSCR_DIS |
                          DMAC_CTRLB_DST_DSCR_DIS;
    uint32_t status;

    qtest_writel(qts, src + 0, 0x11111111);
    qtest_writel(qts, src + 4, 0x22222222);
    qtest_writel(qts, src + 8, 0x33333333);
    qtest_writel(qts, dst + 0, 0xdeadbeef);
    qtest_writel(qts, dst + 4, 0xdeadbeef);
    qtest_writel(qts, dst + 8, 0xdeadbeef);
    qtest_writel(qts, poison + 0, 0xa5a5a5a5);
    qtest_writel(qts, poison + 4, 0x5a5a5a5a);

    qtest_writel(qts, lli0 + 0, src);
    qtest_writel(qts, lli0 + 4, dst);
    qtest_writel(qts, lli0 + 8, ctrla0);
    qtest_writel(qts, lli0 + 12, paced);
    qtest_writel(qts, lli0 + 16, lli1);

    /* Poison every mutable field in the not-yet-fetched second LLI. */
    qtest_writel(qts, lli1 + 0, poison);
    qtest_writel(qts, lli1 + 4, poison + 4);
    qtest_writel(qts, lli1 + 8, DMAC_CTRLA_BTSIZE(2));
    qtest_writel(qts, lli1 + 12, last);
    qtest_writel(qts, lli1 + 16, invalid);

    qtest_writel(qts, channel_base + DMAC_DSCR, lli0);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));

    /* One software request leaves the first descriptor active with residue. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_SSREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DSCR), ==, lli1);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 1);

    /*
     * Software may update a future LLI before it is fetched.  Change its
     * addresses, BTSIZE and next pointer while descriptor 0 is still active.
     */
    qtest_writel(qts, lli1 + 0, src + 8);
    qtest_writel(qts, lli1 + 4, dst + 8);
    qtest_writel(qts, lli1 + 8, ctrla1);
    qtest_writel(qts, lli1 + 16, 0);

    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_SSREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0x22222222);
    g_assert_cmphex(qtest_readl(qts, lli0 + 8), ==,
                    (ctrla0 & ~0xffffu) | DMAC_CTRLA_DONE);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & DMAC_ENA(channel), ==, DMAC_ENA(channel));
    g_assert_cmphex(status & DMAC_EMPTY(channel), ==, DMAC_EMPTY(channel));

    /* The next request fetches and executes the updated descriptor exactly. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_SSREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst + 8), ==, 0x33333333);
    g_assert_cmphex(qtest_readl(qts, poison + 0), ==, 0xa5a5a5a5);
    g_assert_cmphex(qtest_readl(qts, poison + 4), ==, 0x5a5a5a5a);
    g_assert_cmphex(qtest_readl(qts, lli1 + 8), ==,
                    (ctrla1 & ~0xffffu) | DMAC_CTRLA_DONE);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel) | DMAC_CBTC(channel));
    qtest_quit(qts);
}

static void test_dmac_hardware_request_chunk_pacing(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x3c000;
    const uint64_t dst = G45_SDRAM_BASE + 0x3d000;
    const uint32_t channel = 2;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(12) |
                           DMAC_CTRLA_SCSIZE(2) |
                           DMAC_CTRLA_SRC_WIDTH_4 |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrlb = DMAC_CTRLB_FC_PER2MEM |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    uint32_t status;
    int i;

    for (i = 0; i < 12; i++) {
        qtest_writel(qts, src + i * 4, 0x10203040 + i);
        qtest_writel(qts, dst + i * 4, 0xdeadbeef);
    }

    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA, ctrla);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, channel_base + DMAC_CFG,
                 dmac_cfg_src_per(0) | DMAC_CFG_SRC_H2SEL);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);

    /*
     * SCSIZE=2 grants eight source beats.  Holding the request high must not
     * turn that single handshake into permission to drain the descriptor.
     */
    qtest_set_irq_in(qts, G45_DMAC_QOM_PATH, "peripheral-request", 0, 1);
    qtest_clock_step(qts, 1);
    for (i = 0; i < 8; i++) {
        g_assert_cmphex(qtest_readl(qts, dst + i * 4), ==,
                        0x10203040 + i);
    }
    for (i = 8; i < 12; i++) {
        g_assert_cmphex(qtest_readl(qts, dst + i * 4), ==, 0xdeadbeef);
    }
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_SADDR), ==,
                    src + 8 * 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DADDR), ==,
                    dst + 8 * 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 4);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & (DMAC_ENA(channel) | DMAC_EMPTY(channel)), ==,
                    DMAC_ENA(channel) | DMAC_EMPTY(channel));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==, 0);

    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 4);
    g_assert_cmphex(qtest_readl(qts, dst + 8 * 4), ==, 0xdeadbeef);

    qtest_set_irq_in(qts, G45_DMAC_QOM_PATH, "peripheral-request", 0, 0);
    qtest_set_irq_in(qts, G45_DMAC_QOM_PATH, "peripheral-request", 0, 1);
    qtest_clock_step(qts, 1);
    for (i = 8; i < 12; i++) {
        g_assert_cmphex(qtest_readl(qts, dst + i * 4), ==,
                        0x10203040 + i);
    }
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & (DMAC_ENA(channel) | DMAC_EMPTY(channel)), ==,
                    DMAC_EMPTY(channel));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));
    qtest_quit(qts);
}

static void pulse_dmac_request(QTestState *qts, unsigned int request)
{
    qtest_set_irq_in(qts, G45_DMAC_QOM_PATH, "peripheral-request",
                     request, 1);
    qtest_set_irq_in(qts, G45_DMAC_QOM_PATH, "peripheral-request",
                     request, 0);
    qtest_clock_step(qts, 1);
}

static void test_dmac_hardware_request_independent_sides(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x3e000;
    const uint64_t dst = G45_SDRAM_BASE + 0x3f000;
    const uint32_t channel = 3;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t ctrlb = DMAC_CTRLB_FC_PER2PER |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint8_t source[] = {
        0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87,
    };
    int i;

    qtest_memwrite(qts, src, source, sizeof(source));
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, dst + 4, 0xdeadbeef);
    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA,
                 DMAC_CTRLA_BTSIZE(sizeof(source)) |
                 DMAC_CTRLA_DST_WIDTH_4);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, channel_base + DMAC_CFG,
                 dmac_cfg_src_per(0) | DMAC_CFG_SRC_H2SEL |
                 dmac_cfg_dst_per(13) | DMAC_CFG_DST_H2SEL);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));

    /* Source data can enter the FIFO without a destination grant. */
    pulse_dmac_request(qts, 0);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_SADDR), ==, src + 1);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 7);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_EMPTY(channel), ==, 0);

    /*
     * A destination request waits until four source bytes form one word.
     * Its credit must survive the two intermediate source handshakes.
     */
    pulse_dmac_request(qts, 13);
    for (i = 0; i < 2; i++) {
        pulse_dmac_request(qts, 0);
        g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    }
    pulse_dmac_request(qts, 0);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x43322110);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DADDR), ==, dst + 4);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_EMPTY(channel), ==, DMAC_EMPTY(channel));

    /* Reverse the arrival order for the second packed word. */
    pulse_dmac_request(qts, 13);
    for (i = 0; i < 3; i++) {
        pulse_dmac_request(qts, 0);
        g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0xdeadbeef);
    }
    pulse_dmac_request(qts, 0);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0x87766554);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    (DMAC_ENA(channel) | DMAC_EMPTY(channel)), ==,
                    DMAC_EMPTY(channel));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));
    qtest_quit(qts);
}

static void dmac_init_picture_in_picture_memory(QTestState *qts,
                                                uint64_t src,
                                                uint64_t dst)
{
    static const uint32_t source_words[] = {
        0x11111111, 0x22222222, 0xfeedface, 0x33333333,
        0x44444444, 0xbad00bad, 0xcafef00d, 0x5a5aa5a5,
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(source_words); i++) {
        qtest_writel(qts, src + i * sizeof(uint32_t), source_words[i]);
        qtest_writel(qts, dst + i * sizeof(uint32_t), 0xdeadbeef);
    }
}

static void dmac_assert_picture_in_picture_result(QTestState *qts,
                                                  uint64_t src,
                                                  uint64_t dst)
{
    static const uint32_t expected[] = {
        0x11111111, 0x22222222, 0xdeadbeef, 0xdeadbeef,
        0x33333333, 0x44444444, 0xdeadbeef, 0xdeadbeef,
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(expected); i++) {
        g_assert_cmphex(qtest_readl(qts, dst + i * sizeof(uint32_t)), ==,
                        expected[i]);
    }
    g_assert_cmphex(qtest_readl(qts, src + 2 * sizeof(uint32_t)), ==,
                    0xfeedface);
}

static void test_dmac_picture_in_picture(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x28000;
    const uint64_t dst = G45_SDRAM_BASE + 0x29000;
    const uint32_t channel = 4;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t spip = DMAC_PIP(2, 2); /* Skip one word every two. */
    const uint32_t dpip = DMAC_PIP(3, 2); /* Skip two words every two. */
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(4) |
                           DMAC_CTRLA_SRC_WIDTH_4 |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrlb = DMAC_CTRLB_SRC_PIP | DMAC_CTRLB_DST_PIP |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;

    dmac_init_picture_in_picture_memory(qts, src, dst);
    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA, ctrla);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, channel_base + DMAC_SPIP, spip);
    qtest_writel(qts, channel_base + DMAC_DPIP, dpip);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_SPIP), ==, spip);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DPIP), ==, dpip);

    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);

    dmac_assert_picture_in_picture_result(qts, src, dst);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));
    qtest_quit(qts);
}

static void test_dmac_picture_in_picture_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    const uint64_t src_addr = G45_SDRAM_BASE + 0x2a000;
    const uint64_t dst_addr = G45_SDRAM_BASE + 0x2b000;
    const uint32_t channel = 5;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t spip = DMAC_PIP(2, 2);
    const uint32_t dpip = DMAC_PIP(3, 2);
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(4) |
                           DMAC_CTRLA_SRC_WIDTH_4 |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrlb = DMAC_CTRLB_SRC_PIP | DMAC_CTRLB_DST_PIP |
                           DMAC_CTRLB_FC_PER2PER |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint32_t requests = DMAC_SSREQ(channel) | DMAC_DSREQ(channel);
    QTestState *src, *dst;
    int fd;
    int i;

    fd = g_file_open_tmp("at91-dmac-pip-migration-XXXXXX",
                         &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    dmac_init_picture_in_picture_memory(src, src_addr, dst_addr);
    qtest_writel(src, channel_base + DMAC_SADDR, src_addr);
    qtest_writel(src, channel_base + DMAC_DADDR, dst_addr);
    qtest_writel(src, channel_base + DMAC_CTRLA, ctrla);
    qtest_writel(src, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(src, channel_base + DMAC_SPIP, spip);
    qtest_writel(src, channel_base + DMAC_DPIP, dpip);
    qtest_writel(src, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(src, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));

    /* Stop after one transaction: both PiP boundary counters equal one. */
    qtest_writel(src, G45_DMAC_BASE + DMAC_SREQ, requests);
    qtest_clock_step(src, 1);
    g_assert_cmphex(qtest_readl(src, dst_addr), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(src, channel_base + DMAC_SADDR), ==,
                    src_addr + 4);
    g_assert_cmphex(qtest_readl(src, channel_base + DMAC_DADDR), ==,
                    dst_addr + 4);
    g_assert_cmphex(qtest_readl(src, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 3);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, channel_base + DMAC_SPIP), ==, spip);
    g_assert_cmphex(qtest_readl(dst, channel_base + DMAC_DPIP), ==, dpip);
    for (i = 0; i < 3; i++) {
        qtest_writel(dst, G45_DMAC_BASE + DMAC_SREQ, requests);
        qtest_clock_step(dst, 1);
    }

    dmac_assert_picture_in_picture_result(dst, src_addr, dst_addr);
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(channel), ==, 0);
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));
    qtest_quit(dst);
    unlink(state_path);
}

static void test_dmac_auto_replay_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    const uint64_t src_addr = G45_SDRAM_BASE + 0x2c000;
    const uint64_t dst_addr = G45_SDRAM_BASE + 0x2d000;
    const uint32_t channel = 6;
    const uint32_t channel_bit = DMAC_ENA(channel);
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(2) |
                           DMAC_CTRLA_SRC_WIDTH_4 |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrlb = DMAC_CTRLB_AUTO |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint32_t cfg = DMAC_CFG_SRC_REP | DMAC_CFG_DST_REP;
    const uint32_t stalled = channel_bit | DMAC_EMPTY(channel) |
                             DMAC_STALL(channel);
    QTestState *src, *dst;
    uint32_t status;
    int fd;

    fd = g_file_open_tmp("at91-dmac-auto-migration-XXXXXX",
                         &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    qtest_writel(src, src_addr, 0x11111111);
    qtest_writel(src, src_addr + 4, 0x22222222);
    qtest_writel(src, dst_addr, 0xdeadbeef);
    qtest_writel(src, dst_addr + 4, 0xdeadbeef);
    qtest_writel(src, channel_base + DMAC_SADDR, src_addr);
    qtest_writel(src, channel_base + DMAC_DADDR, dst_addr);
    qtest_writel(src, channel_base + DMAC_DSCR, 0);
    qtest_writel(src, channel_base + DMAC_CTRLA, ctrla);
    qtest_writel(src, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(src, channel_base + DMAC_CFG, cfg);
    qtest_writel(src, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(src, G45_DMAC_BASE + DMAC_EBCIER, DMAC_BTC(channel));
    qtest_writel(src, G45_DMAC_BASE + DMAC_CHER, channel_bit);
    qtest_clock_step(src, 1);

    /* AUTO reloads row 10 and stalls at the unmasked BTC boundary. */
    g_assert_cmphex(qtest_readl(src, dst_addr), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(src, dst_addr + 4), ==, 0x22222222);
    status = qtest_readl(src, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & stalled, ==, stalled);
    g_assert_cmphex(qtest_readl(src, channel_base + DMAC_SADDR), ==,
                    src_addr);
    g_assert_cmphex(qtest_readl(src, channel_base + DMAC_DADDR), ==,
                    dst_addr);
    g_assert_cmphex(qtest_readl(src, channel_base + DMAC_CTRLA), ==, ctrla);
    g_assert_cmphex(qtest_readl(src, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    wait_for_migration_complete(dst);
    status = qtest_readl(dst, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & stalled, ==, stalled);
    g_assert_cmphex(qtest_readl(dst, channel_base + DMAC_CTRLB), ==, ctrlb);
    g_assert_cmphex(qtest_readl(dst, channel_base + DMAC_CTRLA), ==, ctrla);

    /* Clear AUTO while stalled; KEEPON executes one final row-1 buffer. */
    qtest_writel(dst, src_addr, 0x33333333);
    qtest_writel(dst, src_addr + 4, 0x44444444);
    qtest_writel(dst, dst_addr, 0xdeadbeef);
    qtest_writel(dst, dst_addr + 4, 0xdeadbeef);
    qtest_writel(dst, channel_base + DMAC_CTRLB,
                 ctrlb & ~DMAC_CTRLB_AUTO);
    qtest_writel(dst, G45_DMAC_BASE + DMAC_CHER, DMAC_KEEPON(channel));
    qtest_clock_step(dst, 1);

    g_assert_cmphex(qtest_readl(dst, dst_addr), ==, 0x33333333);
    g_assert_cmphex(qtest_readl(dst, dst_addr + 4), ==, 0x44444444);
    status = qtest_readl(dst, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & (channel_bit | DMAC_STALL(channel) |
                              DMAC_EMPTY(channel)), ==,
                    DMAC_EMPTY(channel));
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel) | DMAC_CBTC(channel));
    qtest_quit(dst);
    unlink(state_path);
}

static void test_dmac_suspend_resume(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0xa000;
    const uint64_t dst = G45_SDRAM_BASE + 0xb000;
    const uint32_t ctrlb = DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint32_t channel = 2;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    uint32_t status;

    qtest_writel(qts, src, 0x76543210);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA, 4);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);

    /* Enable the channel already suspended: no byte may move. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER,
                 DMAC_ENA(channel) | DMAC_SUSP(channel));
    qtest_clock_step(qts, 1);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & (DMAC_ENA(channel) | DMAC_SUSP(channel) |
                              DMAC_EMPTY(channel)), ==,
                    DMAC_ENA(channel) | DMAC_SUSP(channel) |
                    DMAC_EMPTY(channel));
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==, 0);

    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHDR, DMAC_RES(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x76543210);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & (DMAC_ENA(channel) | DMAC_SUSP(channel) |
                              DMAC_EMPTY(channel)), ==,
                    DMAC_EMPTY(channel));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));
    qtest_quit(qts);
}

static void test_dmac_software_requests(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0xc000;
    const uint64_t dst = G45_SDRAM_BASE + 0xd000;
    const uint32_t ctrlb = DMAC_CTRLB_FC_PER2MEM |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint32_t channel = 1;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    unsigned int word;

    for (word = 0; word < 8; word++) {
        qtest_writel(qts, src + 4 * word, 0x11110000 + word);
        qtest_writel(qts, dst + 4 * word, 0xdeadbeef);
    }
    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA,
                 DMAC_CTRLA_BTSIZE(8) | DMAC_CTRLA_SCSIZE(1) |
                 DMAC_CTRLA_SRC_WIDTH_4 | DMAC_CTRLA_DST_WIDTH_4);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);

    /* H2SEL=0: enabling alone must not transfer a software-paced source. */
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(channel), ==, DMAC_ENA(channel));

    /* A single request advances exactly one 32-bit source transaction. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_SSREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_SREQ), ==, 0);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x11110000);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_SADDR), ==, src + 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DADDR), ==, dst + 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 7);

    /* SCSIZE=4: each chunk request advances four source transactions. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CREQ, DMAC_SCREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CREQ), ==, 0);
    for (word = 0; word < 5; word++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * word), ==,
                        0x11110000 + word);
    }
    g_assert_cmphex(qtest_readl(qts, dst + 20), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 3);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(channel), ==, DMAC_ENA(channel));

    qtest_writel(qts, G45_DMAC_BASE + DMAC_CREQ, DMAC_SCREQ(channel));
    qtest_clock_step(qts, 1);
    for (word = 0; word < 8; word++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * word), ==,
                        0x11110000 + word);
    }
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(channel), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));
    qtest_quit(qts);
}

static void test_dmac_software_last(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0xe000;
    const uint64_t dst = G45_SDRAM_BASE + 0xf000;
    const uint32_t ctrlb = DMAC_CTRLB_FC_PER2MEM_PER |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE;
    unsigned int word;

    for (word = 0; word < 8; word++) {
        qtest_writel(qts, src + 4 * word, 0x22220000 + word);
        qtest_writel(qts, dst + 4 * word, 0xdeadbeef);
    }
    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA,
                 DMAC_CTRLA_BTSIZE(8) | DMAC_CTRLA_SCSIZE(1) |
                 DMAC_CTRLA_SRC_WIDTH_4 | DMAC_CTRLA_DST_WIDTH_4);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_SSREQ(0));
    qtest_clock_step(qts, 1);

    qtest_writel(qts, G45_DMAC_BASE + DMAC_LAST, DMAC_SLAST(0));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_LAST), ==,
                    DMAC_SLAST(0));
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CREQ, DMAC_SCREQ(0));
    qtest_clock_step(qts, 1);

    for (word = 0; word < 5; word++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * word), ==,
                        0x22220000 + word);
    }
    for (; word < 8; word++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * word), ==, 0xdeadbeef);
    }
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_LAST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) & 1, ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0));
    qtest_quit(qts);
}

static void test_dmac_software_destination_requests(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x18000;
    const uint64_t dst = G45_SDRAM_BASE + 0x19000;
    const uint32_t ctrlb = DMAC_CTRLB_FC_MEM2PER |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint32_t channel = 5;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    unsigned int word;

    for (word = 0; word < 8; word++) {
        qtest_writel(qts, src + 4 * word, 0x33330000 + word);
        qtest_writel(qts, dst + 4 * word, 0xdeadbeef);
    }
    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA,
                 DMAC_CTRLA_BTSIZE(8) | DMAC_CTRLA_DCSIZE(1) |
                 DMAC_CTRLA_SRC_WIDTH_4 | DMAC_CTRLA_DST_WIDTH_4);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);

    /*
     * Odd request bits pace the destination side.  The memory source side is
     * not paced, so it prefetches into the two-word conversion FIFO: one
     * destination grant emits exactly one word while SADDR/BTSIZE run ahead
     * by the refilled FIFO contents.
     */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_DSREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_SREQ), ==, 0);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x33330000);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_SADDR), ==, src + 12);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DADDR), ==, dst + 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 5);

    qtest_writel(qts, G45_DMAC_BASE + DMAC_CREQ, DMAC_DCREQ(channel));
    qtest_clock_step(qts, 1);
    for (word = 0; word < 5; word++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * word), ==,
                        0x33330000 + word);
    }
    g_assert_cmphex(qtest_readl(qts, dst + 20), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 1);

    qtest_writel(qts, G45_DMAC_BASE + DMAC_CREQ, DMAC_DCREQ(channel));
    qtest_clock_step(qts, 1);
    for (word = 0; word < 8; word++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * word), ==,
                        0x33330000 + word);
    }
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(channel), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));
    qtest_quit(qts);
}

static void test_dmac_software_mixed_requests(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x24000;
    const uint64_t dst = G45_SDRAM_BASE + 0x25000;
    const uint32_t channel = 6;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t ctrlb = DMAC_CTRLB_FC_PER2PER |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;

    qtest_writel(qts, src + 0, 0x12345678);
    qtest_writel(qts, src + 4, 0x89abcdef);
    qtest_writel(qts, dst + 0, 0xdeadbeef);
    qtest_writel(qts, dst + 4, 0xdeadbeef);
    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA,
                 DMAC_CTRLA_BTSIZE(2) |
                 DMAC_CTRLA_SRC_WIDTH_4 | DMAC_CTRLA_DST_WIDTH_4);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);

    /*
     * A source request issued before global/channel enable must not be lost:
     * once the channel starts, the source side proceeds without waiting for
     * a destination grant and its beat enters the conversion FIFO.
     */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_SSREQ(channel));
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_SREQ), ==, 0);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_SADDR), ==, src + 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 1);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_EMPTY(channel), ==, 0);

    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_DSREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_SREQ), ==, 0);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 1);

    /* Reverse request arrival order for the final transaction. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_DSREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_SREQ), ==,
                    DMAC_DSREQ(channel));
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0xdeadbeef);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_SSREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0x89abcdef);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_SREQ), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(channel), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));
    qtest_quit(qts);
}

static void test_dmac_access_errors(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x10000;
    const uint64_t dst = G45_SDRAM_BASE + 0x11000;
    const uint64_t invalid = 0x90000000; /* datasheet undefined/abort window */
    const uint32_t ctrlb = DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint32_t channel2 = 2;
    const uint64_t channel2_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                   channel2 * DMAC_CH_STRIDE;
    uint32_t status;

    qtest_irq_intercept_in(qts, G45_AIC_QOM_PATH);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_SADDR, invalid);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_DADDR, dst);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_CTRLA, 4);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EBCIER, DMAC_ERR(0));
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(qts, 1);
    g_assert_true(qtest_get_irq(qts, G45_DMAC_IRQ));
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    g_assert_cmphex(status, ==, DMAC_ERR(0));
    g_assert_false(qtest_get_irq(qts, G45_DMAC_IRQ));
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) & 1, ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CH0_BASE +
                                DMAC_CTRLA) & 0xffff, ==, 4);

    /* A failed descriptor fetch is an ERR, not a zero-filled transfer. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_CH_STRIDE +
                 DMAC_DSCR, invalid);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(1));
    qtest_clock_step(qts, 1);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    g_assert_cmphex(status, ==, DMAC_ERR(1));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(1), ==, 0);

    /* A destination abort retains the completed source beat as residue. */
    qtest_writel(qts, src, 0x11223344);
    qtest_writel(qts, channel2_base + DMAC_SADDR, src);
    qtest_writel(qts, channel2_base + DMAC_DADDR, invalid);
    qtest_writel(qts, channel2_base + DMAC_CTRLA, 4);
    qtest_writel(qts, channel2_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EBCIER, DMAC_ERR(channel2));
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel2));
    qtest_clock_step(qts, 1);
    g_assert_true(qtest_get_irq(qts, G45_DMAC_IRQ));
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    g_assert_cmphex(status, ==, DMAC_ERR(channel2));
    g_assert_false(qtest_get_irq(qts, G45_DMAC_IRQ));
    g_assert_cmphex(qtest_readl(qts, channel2_base + DMAC_SADDR), ==,
                    src + 1);
    g_assert_cmphex(qtest_readl(qts, channel2_base + DMAC_DADDR), ==,
                    invalid);
    g_assert_cmphex(qtest_readl(qts, channel2_base + DMAC_CTRLA) & 0xffff,
                    ==, 3);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR) &
                    DMAC_ENA(channel2), ==, 0);

    /* Reprogramming the failed channel must recover cleanly. */
    qtest_writel(qts, src, 0xa5a55a5a);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_SADDR, src);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_DADDR, dst);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CH0_BASE + DMAC_CTRLA, 4);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xa5a55a5a);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0));
    qtest_quit(qts);
}

static void test_dmac_partial_access_error_residue(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t ram_end = G45_SDRAM_BASE + G45_SDRAM_SIZE;
    const uint64_t src = G45_SDRAM_BASE + 0x32000;
    const uint64_t dst = G45_SDRAM_BASE + 0x33000;
    const uint64_t channel0 = G45_DMAC_BASE + DMAC_CH0_BASE;
    const uint64_t channel1 = channel0 + DMAC_CH_STRIDE;
    const uint64_t channel2 = channel1 + DMAC_CH_STRIDE;
    const uint32_t ctrlb = DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint8_t source_bytes[] = { 0x11, 0x22, 0x33, 0x44 };
    const uint8_t edge_bytes[] = { 0xa1, 0xb2 };
    const uint8_t untouched[] = { 0xee, 0xee, 0xee, 0xee };
    uint8_t actual[sizeof(source_bytes)];
    uint32_t status;

    qtest_memwrite(qts, src, source_bytes, sizeof(source_bytes));
    qtest_memwrite(qts, dst, untouched, sizeof(untouched));
    qtest_memwrite(qts, ram_end - sizeof(edge_bytes),
                   edge_bytes, sizeof(edge_bytes));
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);

    /* Two source bytes complete before the third read crosses out of DDR. */
    qtest_writel(qts, channel0 + DMAC_SADDR, ram_end - 2);
    qtest_writel(qts, channel0 + DMAC_DADDR, dst);
    qtest_writel(qts, channel0 + DMAC_CTRLA, DMAC_CTRLA_BTSIZE(4));
    qtest_writel(qts, channel0 + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(qts, 1);

    qtest_memread(qts, dst, actual, sizeof(actual));
    g_assert_cmphex(actual[0], ==, edge_bytes[0]);
    g_assert_cmphex(actual[1], ==, edge_bytes[1]);
    g_assert_cmphex(actual[2], ==, untouched[2]);
    g_assert_cmphex(actual[3], ==, untouched[3]);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_SADDR), ==, ram_end);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_DADDR), ==, dst + 2);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_CTRLA) & 0xffff, ==, 2);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    g_assert_cmphex(status, ==, DMAC_ERR(0));

    /*
     * Two destination writes complete, then the third source byte remains
     * consumed when its write crosses out of DDR.
     */
    qtest_memwrite(qts, ram_end - sizeof(edge_bytes),
                   untouched, sizeof(edge_bytes));
    qtest_writel(qts, channel1 + DMAC_SADDR, src);
    qtest_writel(qts, channel1 + DMAC_DADDR, ram_end - 2);
    qtest_writel(qts, channel1 + DMAC_CTRLA, DMAC_CTRLA_BTSIZE(4));
    qtest_writel(qts, channel1 + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(1));
    qtest_clock_step(qts, 1);

    qtest_memread(qts, ram_end - sizeof(edge_bytes),
                  actual, sizeof(edge_bytes));
    g_assert_cmphex(actual[0], ==, source_bytes[0]);
    g_assert_cmphex(actual[1], ==, source_bytes[1]);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_SADDR), ==, src + 3);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_DADDR), ==, ram_end);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_CTRLA) & 0xffff, ==, 1);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    g_assert_cmphex(status, ==, DMAC_ERR(1));

    /*
     * Fetching a five-word descriptor across DDR end is atomic: the first
     * two readable words must not leak into live channel state.
     */
    qtest_writel(qts, ram_end - 8, src);
    qtest_writel(qts, ram_end - 4, dst);
    qtest_writel(qts, channel2 + DMAC_SADDR, 0x11111111);
    qtest_writel(qts, channel2 + DMAC_DADDR, 0x22222222);
    qtest_writel(qts, channel2 + DMAC_DSCR, ram_end - 8);
    qtest_writel(qts, channel2 + DMAC_CTRLA, 0x33330004);
    qtest_writel(qts, channel2 + DMAC_CTRLB, 0x44440000);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(2));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, channel2 + DMAC_SADDR), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(qts, channel2 + DMAC_DADDR), ==, 0x22222222);
    g_assert_cmphex(qtest_readl(qts, channel2 + DMAC_DSCR), ==, ram_end - 8);
    g_assert_cmphex(qtest_readl(qts, channel2 + DMAC_CTRLA), ==, 0x33330004);
    g_assert_cmphex(qtest_readl(qts, channel2 + DMAC_CTRLB), ==, 0x44440000);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    g_assert_cmphex(status, ==, DMAC_ERR(2));
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & (DMAC_ENA(0) | DMAC_ENA(1) | DMAC_ENA(2)),
                    ==, 0);
    g_assert_cmphex(status & (DMAC_EMPTY(0) | DMAC_EMPTY(1) | DMAC_EMPTY(2)),
                    ==, DMAC_EMPTY(0) | DMAC_EMPTY(1) | DMAC_EMPTY(2));

    /* The last failed channel can be reprogrammed and completed immediately. */
    qtest_writel(qts, channel2 + DMAC_SADDR, src);
    qtest_writel(qts, channel2 + DMAC_DADDR, dst);
    qtest_writel(qts, channel2 + DMAC_DSCR, 0);
    qtest_writel(qts, channel2 + DMAC_CTRLA, DMAC_CTRLA_BTSIZE(1));
    qtest_writel(qts, channel2 + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(2));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readb(qts, dst), ==, source_bytes[0]);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(2));
    qtest_quit(qts);
}

static void test_dmac_rejected_destination_and_writeback(void)
{
    g_autofree char *image_path = NULL;
    QTestState *qts;
    const uint64_t descriptor = G45_EBI_CS0_BASE + 0x100;
    const uint64_t rejected_destination = G45_EBI_CS0_BASE + 0x200;
    const uint64_t src = G45_SDRAM_BASE + 0x38000;
    const uint64_t dst = G45_SDRAM_BASE + 0x39000;
    const uint64_t channel0 = G45_DMAC_BASE + DMAC_CH0_BASE;
    const uint64_t channel1 = channel0 + DMAC_CH_STRIDE;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(4);
    const uint32_t last = DMAC_CTRLB_SRC_DSCR_DIS |
                          DMAC_CTRLB_DST_DSCR_DIS;
    uint32_t lli[5];
    uint32_t status;
    int fd;

    /*
     * A secure CFI window permits nonsecure array reads but rejects writes
     * with MEMTX_ERROR.  It therefore provides deterministic address-space
     * failures without adding a test-only hook to the DMAC.
     */
    lli[0] = GUINT32_TO_LE(src);
    lli[1] = GUINT32_TO_LE(dst);
    lli[2] = GUINT32_TO_LE(ctrla);
    lli[3] = GUINT32_TO_LE(last);
    lli[4] = 0;
    fd = g_file_open_tmp("at91-dmac-pflash-XXXXXX.img", &image_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(ftruncate(fd, 64 * KiB), ==, 0);
    g_assert_cmpint(pwrite(fd, lli, sizeof(lli), 0x100), ==, sizeof(lli));
    close(fd);

    qts = qtest_initf("-machine sam9m10g45ek,ebi-nor-cs=0 "
                      "-global driver=cfi.pflash01,property=secure,value=on "
                      "-drive if=pflash,format=raw,file=%s -S", image_path);
    qtest_writel(qts, src, 0x44332211);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, channel0 + DMAC_SADDR, src);
    qtest_writel(qts, channel0 + DMAC_DADDR, rejected_destination);
    qtest_writel(qts, channel0 + DMAC_CTRLA, ctrla);
    qtest_writel(qts, channel0 + DMAC_CTRLB, last);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readb(qts, rejected_destination), ==, 0);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_SADDR), ==, src + 1);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_DADDR), ==,
                    rejected_destination);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_CTRLA) & 0xffff, ==, 3);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_ERR(0));

    /*
     * Fetching the descriptor from the same window succeeds.  Data reaches
     * DDR, then the DONE writeback is rejected: BTC records the completed
     * buffer, ERR records the failed AHB write, and CBTC is not asserted.
     */
    qtest_writel(qts, channel1 + DMAC_DSCR, descriptor);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(1));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x44332211);
    g_assert_cmphex(qtest_readl(qts, descriptor + 8), ==, ctrla);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_SADDR), ==, src + 4);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_DADDR), ==, dst + 4);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_CTRLA) & 0xffff, ==, 0);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & DMAC_ENA(1), ==, 0);
    g_assert_cmphex(status & DMAC_EMPTY(1), ==, DMAC_EMPTY(1));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(1) | DMAC_ERR(1));
    qtest_quit(qts);
    unlink(image_path);
}

static void test_dmac_peripheral_access_abort(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x3a000;
    const uint64_t dst = G45_SDRAM_BASE + 0x3b000;
    const uint64_t pmc_mckr = G45_PMC_BASE + 0x30;
    const uint64_t channel0 = G45_DMAC_BASE + DMAC_CH0_BASE;
    const uint64_t channel1 = channel0 + DMAC_CH_STRIDE;
    const uint32_t ctrlb = DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    uint32_t status;

    qtest_writel(qts, src, 0x44332211);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);

    /*
     * The PMC accepts only aligned 32-bit accesses.  A byte source transfer
     * is rejected before any data or live-address state can advance.
     */
    qtest_writel(qts, channel0 + DMAC_SADDR, pmc_mckr);
    qtest_writel(qts, channel0 + DMAC_DADDR, dst);
    qtest_writel(qts, channel0 + DMAC_CTRLA, DMAC_CTRLA_BTSIZE(4));
    qtest_writel(qts, channel0 + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_SADDR), ==, pmc_mckr);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_DADDR), ==, dst);
    g_assert_cmphex(qtest_readl(qts, channel0 + DMAC_CTRLA) & 0xffff, ==, 4);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    g_assert_cmphex(status, ==, DMAC_ERR(0));

    /*
     * A byte destination transfer consumes one source byte before the PMC
     * rejects the write.  The source and BTSIZE residue expose that ordering.
     */
    qtest_writel(qts, channel1 + DMAC_SADDR, src);
    qtest_writel(qts, channel1 + DMAC_DADDR, pmc_mckr);
    qtest_writel(qts, channel1 + DMAC_CTRLA, DMAC_CTRLA_BTSIZE(4));
    qtest_writel(qts, channel1 + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(1));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_SADDR), ==, src + 1);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_DADDR), ==, pmc_mckr);
    g_assert_cmphex(qtest_readl(qts, channel1 + DMAC_CTRLA) & 0xffff, ==, 3);
    status = qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR);
    g_assert_cmphex(status, ==, DMAC_ERR(1));
    qtest_quit(qts);
}

static void test_dmac_late_irq_enable(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x26000;
    const uint64_t dst = G45_SDRAM_BASE + 0x27000;
    const uint32_t channel = 7;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE +
                                  channel * DMAC_CH_STRIDE;
    const uint32_t ctrlb = DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;

    qtest_irq_intercept_in(qts, G45_AIC_QOM_PATH);
    qtest_writel(qts, src, 0x55aa33cc);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA, 4);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);

    g_assert_false(qtest_get_irq(qts, G45_DMAC_IRQ));
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x55aa33cc);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EBCIER, DMAC_BTC(channel));
    g_assert_true(qtest_get_irq(qts, G45_DMAC_IRQ));
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EBCIDR, DMAC_BTC(channel));
    g_assert_false(qtest_get_irq(qts, G45_DMAC_IRQ));
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EBCIER, DMAC_BTC(channel));
    g_assert_true(qtest_get_irq(qts, G45_DMAC_IRQ));
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(channel));
    g_assert_false(qtest_get_irq(qts, G45_DMAC_IRQ));
    qtest_quit(qts);
}

static void test_dmac_word_width_alias(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint64_t src = G45_SDRAM_BASE + 0x12000;
    const uint64_t dst = G45_SDRAM_BASE + 0x13000;
    const uint32_t ctrlb = DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(1) |
                           DMAC_CTRLA_SRC_WIDTH_4_ALIAS |
                           DMAC_CTRLA_DST_WIDTH_4_ALIAS;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE;

    /*
     * The datasheet defines both SRC/DST_WIDTH encodings 10b and 11b as
     * WORD.  A mistaken 8-byte interpretation corrupts the guard word.
     */
    qtest_writel(qts, src, 0x44332211);
    qtest_writel(qts, src + 4, 0xa5a55a5a);
    qtest_writel(qts, dst, 0xdeadbeef);
    qtest_writel(qts, dst + 4, 0xc001d00d);
    qtest_writel(qts, channel_base + DMAC_SADDR, src);
    qtest_writel(qts, channel_base + DMAC_DADDR, dst);
    qtest_writel(qts, channel_base + DMAC_CTRLA, ctrla);
    qtest_writel(qts, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x44332211);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0xc001d00d);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_SADDR), ==, src + 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DADDR), ==, dst + 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0));
    qtest_quit(qts);
}

static void test_hsmci_dma_wait_migration(void)
{
    const DmaRoute *route = &routes[1];
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    uint64_t data_addr;
    int fd;

    fd = g_file_open_tmp("at91-dmac-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_initf("-machine %s -S", route->machine);
    data_addr = configure_hsmci_dma(src, route, true);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine %s -S -incoming %s", route->machine, uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, route->dmac_base + DMAC_CHSR) & 1,
                    ==, 1);
    g_assert_cmphex(qtest_readl(dst, data_addr), ==, 0xdeadbeef);

    start_hsmci_read(dst, route);
    wait_for_dmac_channel_disabled(dst, route->dmac_base, 0);
    g_assert_cmphex(qtest_readl(dst, route->dmac_base + DMAC_CHSR) & 1,
                    ==, 0);
    g_assert_cmphex(qtest_readl(dst, data_addr), ==, 0);
    qtest_quit(dst);

    unlink(state_path);
}

static void test_dmac_software_request_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    const uint64_t src_addr = G45_SDRAM_BASE + 0x14000;
    const uint64_t dst_addr = G45_SDRAM_BASE + 0x15000;
    const uint32_t ctrla = DMAC_CTRLA_BTSIZE(4) |
                           DMAC_CTRLA_SCSIZE(1) |
                           DMAC_CTRLA_DST_WIDTH_4;
    const uint32_t ctrlb = DMAC_CTRLB_FC_PER2MEM |
                           DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE;
    QTestState *src, *dst;
    uint32_t status;
    int fd;

    fd = g_file_open_tmp("at91-dmac-software-migration-XXXXXX",
                         &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    qtest_writel(src, src_addr, 0x44332211);
    qtest_writel(src, dst_addr, 0xdeadbeef);
    qtest_writel(src, channel_base + DMAC_SADDR, src_addr);
    qtest_writel(src, channel_base + DMAC_DADDR, dst_addr);
    qtest_writel(src, channel_base + DMAC_CTRLA, ctrla);
    qtest_writel(src, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(src, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(src, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));

    /*
     * One byte is resident in the conversion FIFO; no destination word has
     * been emitted yet.  Suspend and queue the final chunk before migration.
     */
    qtest_writel(src, G45_DMAC_BASE + DMAC_SREQ, DMAC_SSREQ(0));
    qtest_clock_step(src, 1);
    g_assert_cmphex(qtest_readl(src, channel_base + DMAC_SADDR), ==,
                    src_addr + 1);
    g_assert_cmphex(qtest_readl(src, channel_base + DMAC_DADDR), ==,
                    dst_addr);
    g_assert_cmphex(qtest_readl(src, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 3);
    g_assert_cmphex(qtest_readl(src, dst_addr), ==, 0xdeadbeef);
    qtest_writel(src, G45_DMAC_BASE + DMAC_CHER, DMAC_SUSP(0));
    qtest_writel(src, G45_DMAC_BASE + DMAC_CREQ, DMAC_SCREQ(0));
    qtest_clock_step(src, 1);
    g_assert_cmphex(qtest_readl(src, G45_DMAC_BASE + DMAC_CREQ), ==,
                    DMAC_SCREQ(0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    wait_for_migration_complete(dst);
    status = qtest_readl(dst, G45_DMAC_BASE + DMAC_CHSR);
    g_assert_cmphex(status & (DMAC_ENA(0) | DMAC_SUSP(0) | DMAC_EMPTY(0)),
                    ==, DMAC_ENA(0) | DMAC_SUSP(0));
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_CREQ), ==,
                    DMAC_SCREQ(0));
    g_assert_cmphex(qtest_readl(dst, channel_base + DMAC_SADDR), ==,
                    src_addr + 1);
    g_assert_cmphex(qtest_readl(dst, channel_base + DMAC_DADDR), ==,
                    dst_addr);
    g_assert_cmphex(qtest_readl(dst, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 3);
    g_assert_cmphex(qtest_readl(dst, dst_addr), ==, 0xdeadbeef);

    qtest_writel(dst, G45_DMAC_BASE + DMAC_CHDR, DMAC_RES(0));
    qtest_clock_step(dst, 1);
    g_assert_cmphex(qtest_readl(dst, dst_addr), ==, 0x44332211);
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_CREQ), ==, 0);
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_CHSR) &
                    (DMAC_ENA(0) | DMAC_SUSP(0) | DMAC_EMPTY(0)), ==,
                    DMAC_EMPTY(0));
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0));
    qtest_quit(dst);

    unlink(state_path);
}

static void test_dmac_pending_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    const uint64_t src_addr = G45_SDRAM_BASE + 0x16000;
    const uint64_t dst_addr = G45_SDRAM_BASE + 0x17000;
    const uint32_t ctrlb = DMAC_CTRLB_SRC_DSCR_DIS |
                           DMAC_CTRLB_DST_DSCR_DIS;
    const uint64_t channel_base = G45_DMAC_BASE + DMAC_CH0_BASE;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("at91-dmac-irq-migration-XXXXXX",
                         &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    qtest_irq_intercept_in(src, G45_AIC_QOM_PATH);
    qtest_writel(src, src_addr, 0x5aa5c33c);
    qtest_writel(src, dst_addr, 0xdeadbeef);
    qtest_writel(src, channel_base + DMAC_SADDR, src_addr);
    qtest_writel(src, channel_base + DMAC_DADDR, dst_addr);
    qtest_writel(src, channel_base + DMAC_CTRLA, 4);
    qtest_writel(src, channel_base + DMAC_CTRLB, ctrlb);
    qtest_writel(src, G45_DMAC_BASE + DMAC_EBCIER, DMAC_BTC(0));
    qtest_writel(src, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(src, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(0));
    qtest_clock_step(src, 1);
    g_assert_true(qtest_get_irq(src, G45_DMAC_IRQ));
    g_assert_cmphex(qtest_readl(src, G45_DMAC_BASE + DMAC_EBCIMR), ==,
                    DMAC_BTC(0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    /*
     * Install the line interceptor before incoming state is loaded so the
     * device post-load callback must reconstruct the output level.
     */
    dst = qtest_init("-machine sam9m10g45ek -S -incoming defer");
    qtest_irq_intercept_in(dst, G45_AIC_QOM_PATH);
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_true(qtest_get_irq(dst, G45_DMAC_IRQ));
    g_assert_cmphex(qtest_readl(dst, G45_DMAC_BASE + DMAC_EBCISR), ==,
                    DMAC_BTC(0));
    g_assert_false(qtest_get_irq(dst, G45_DMAC_IRQ));
    qtest_quit(dst);

    unlink(state_path);
}

static void test_hsmci_active_request_migration(void)
{
    const DmaRoute *route = &routes[1];
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    uint64_t data_addr;
    int fd;

    fd = g_file_open_tmp("at91-hsmci-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    /* Leave the DMA channel disabled while HSMCI holds request 13 high. */
    src = qtest_initf("-machine %s -S", route->machine);
    data_addr = configure_hsmci_dma(src, route, false);
    start_hsmci_read(src, route);
    g_assert_cmphex(qtest_readl(src, route->dmac_base + DMAC_CHSR) & 1,
                    ==, 0);
    g_assert_cmphex(qtest_readl(src, data_addr), ==, 0xdeadbeef);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine %s -S -incoming %s", route->machine, uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(qtest_readl(dst, data_addr), ==, 0xdeadbeef);

    /* HSMCI post-load must reassert request 13 before the channel is armed. */
    qtest_writel(dst, route->dmac_base + DMAC_CHER, 1);
    qtest_clock_step(dst, 1);
    wait_for_dmac_channel_disabled(dst, route->dmac_base, 0);
    g_assert_cmphex(qtest_readl(dst, route->dmac_base + DMAC_CHSR) & 1,
                    ==, 0);
    g_assert_cmphex(qtest_readl(dst, data_addr), ==, 0);
    qtest_quit(dst);

    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_data_func("/at91-dmac/g45/hsmci0/request-0", &routes[0],
                        test_hsmci_dma_waits_for_request);
    qtest_add_func("/at91-dmac/g45/reset-contract",
                   test_dmac_reset_contract);
    qtest_add_func("/at91-dmac/g45/global-enable-gates-transfer",
                   test_dmac_global_enable_gates_transfer);
    qtest_add_func("/at91-dmac/g45/linked-list-writeback",
                   test_dmac_linked_list_writeback);
    qtest_add_func("/at91-dmac/g45/stop-on-done",
                   test_dmac_stop_on_done);
    qtest_add_func("/at91-dmac/g45/partial-descriptor-reload",
                   test_dmac_partial_descriptor_reload);
    qtest_add_func("/at91-dmac/g45/descriptor-auto-btsize-replay",
                   test_dmac_descriptor_auto_btsize_replay);
    qtest_add_func("/at91-dmac/g45/descriptor-page-boundary",
                   test_dmac_descriptor_page_boundary);
    qtest_add_func("/at91-dmac/g45/cyclic-descriptor-policy",
                   test_dmac_cyclic_descriptor_policy);
    qtest_add_func("/at91-dmac/g45/row1-terminates-nonzero-next",
                   test_dmac_row1_terminates_nonzero_next);
    qtest_add_func("/at91-dmac/g45/live-next-descriptor-update",
                   test_dmac_live_next_descriptor_update);
    qtest_add_func("/at91-dmac/g45/hardware-request-chunk-pacing",
                   test_dmac_hardware_request_chunk_pacing);
    qtest_add_func("/at91-dmac/g45/hardware-request-independent-sides",
                   test_dmac_hardware_request_independent_sides);
    qtest_add_func("/at91-dmac/g45/picture-in-picture",
                   test_dmac_picture_in_picture);
    qtest_add_func("/at91-dmac/g45/suspend-resume",
                   test_dmac_suspend_resume);
    qtest_add_func("/at91-dmac/g45/software-requests",
                   test_dmac_software_requests);
    qtest_add_func("/at91-dmac/g45/software-last",
                   test_dmac_software_last);
    qtest_add_func("/at91-dmac/g45/software-destination-requests",
                   test_dmac_software_destination_requests);
    qtest_add_func("/at91-dmac/g45/software-mixed-requests",
                   test_dmac_software_mixed_requests);
    qtest_add_func("/at91-dmac/g45/access-errors",
                   test_dmac_access_errors);
    qtest_add_func("/at91-dmac/g45/partial-access-error-residue",
                   test_dmac_partial_access_error_residue);
    qtest_add_func("/at91-dmac/g45/rejected-destination-and-writeback",
                   test_dmac_rejected_destination_and_writeback);
    qtest_add_func("/at91-dmac/g45/peripheral-access-abort",
                   test_dmac_peripheral_access_abort);
    qtest_add_func("/at91-dmac/g45/late-irq-enable",
                   test_dmac_late_irq_enable);
    qtest_add_func("/at91-dmac/g45/word-width-alias",
                   test_dmac_word_width_alias);
    qtest_add_data_func("/at91-dmac/g45/hsmci1/request-13", &routes[1],
                        test_hsmci_dma_waits_for_request);
    qtest_add_func("/at91-dmac/g45/hsmci1/tx-request-13",
                   test_hsmci_tx_dma_request);
    qtest_add_func("/at91-dmac/g45/hsmci0/acmd13-completion",
                   test_hsmci_acmd13_completion);
    qtest_add_func("/at91-dmac/g45/hsmci0/block-refill-reentrancy",
                   test_hsmci_dma_block_refill_reentrancy);
    qtest_add_data_func("/at91-dmac/g35/hsmci0/dmac0-request-0", &routes[2],
                        test_hsmci_dma_waits_for_request);
    qtest_add_data_func("/at91-dmac/g35/hsmci1/dmac1-request-0", &routes[3],
                        test_hsmci_dma_waits_for_request);
    qtest_add_func("/at91-dmac/g45/hsmci1/wait-migration",
                   test_hsmci_dma_wait_migration);
    qtest_add_func("/at91-dmac/g45/hsmci1/active-request-migration",
                   test_hsmci_active_request_migration);
    qtest_add_func("/at91-dmac/g45/software-request-migration",
                   test_dmac_software_request_migration);
    qtest_add_func("/at91-dmac/g45/pending-irq-migration",
                   test_dmac_pending_irq_migration);
    qtest_add_func("/at91-dmac/g45/picture-in-picture-migration",
                   test_dmac_picture_in_picture_migration);
    qtest_add_func("/at91-dmac/g45/auto-replay-migration",
                   test_dmac_auto_replay_migration);
    qtest_add_func("/at91-dmac/g45/arbitration-migration",
                   test_dmac_arbitration_migration);
    qtest_add_func("/at91-dmac/g45/maximum-boundaries",
                   test_dmac_maximum_and_boundaries);
    qtest_add_func("/at91-dmac/g45/overlap-traversal",
                   test_dmac_overlap_traversal);

    return g_test_run();
}
