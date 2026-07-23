/*
 * QTest tests for AT91 peripheral-paced DMA.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define G45_DMAC_BASE          0xffffec00
#define G45_HSMCI0_BASE        0xfff80000
#define G45_HSMCI1_BASE        0xfffd0000
#define G45_SDRAM_BASE         0x70000000

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
#define DMAC_CTRLB_SRC_FIXED   (2u << 24)
#define DMAC_CTRLB_DST_FIXED   (2u << 28)
#define DMAC_CTRLB_SRC_DSCR_DIS (1u << 16)
#define DMAC_CTRLB_DST_DSCR_DIS (1u << 20)
#define DMAC_CTRLB_IEN         (1u << 30)
#define DMAC_CTRLA_DONE        (1u << 31)
#define DMAC_CFG_SRC_H2SEL     (1u << 9)
#define DMAC_CFG_DST_H2SEL     (1u << 13)
#define DMAC_CFG_SOD           (1u << 16)
#define DMAC_BTC(x)            (1u << (x))
#define DMAC_CBTC(x)           (1u << (8 + (x)))
#define DMAC_ERR(x)            (1u << (16 + (x)))
#define DMAC_ENA(x)            (1u << (x))
#define DMAC_SUSP(x)           (1u << (8 + (x)))
#define DMAC_EMPTY(x)          (1u << (16 + (x)))
#define DMAC_RES(x)            (1u << (8 + (x)))
#define DMAC_SSREQ(x)          (1u << (2 * (x)))
#define DMAC_DSREQ(x)          (1u << (1 + 2 * (x)))
#define DMAC_SCREQ(x)          (1u << (2 * (x)))
#define DMAC_DCREQ(x)          (1u << (1 + 2 * (x)))
#define DMAC_SLAST(x)          (1u << (2 * (x)))
#define DMAC_PIP(hole, boundary) \
    (((hole) & 0xffffu) | (((boundary) & 0x3ffu) << 16))

#define G45_AIC_QOM_PATH       "/machine/unattached/device[2]"
#define G45_DMAC_IRQ           21

#define HSMCI_ARGR             0x10
#define HSMCI_CMDR             0x14
#define HSMCI_BLKR             0x18
#define HSMCI_RDR              0x30
#define HSMCI_TDR              0x34
#define HSMCI_SR               0x40
#define HSMCI_SR_BLKE          (1u << 3)
#define HSMCI_CMDR_RSP_48      (1u << 6)
#define HSMCI_CMDR_MAXLAT_64   (1u << 12)
#define HSMCI_CMDR_START       (1u << 16)
#define HSMCI_CMDR_READ        (1u << 18)

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

static uint32_t dmac_cfg_src_per(unsigned request_id)
{
    return (request_id & 0xf) | ((request_id & 0x30) << 6);
}

static uint32_t dmac_cfg_dst_per(unsigned request_id)
{
    return ((request_id & 0xf) << 4) | ((request_id & 0x30) << 10);
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
    qtest_writel(qts, route->hsmci_base + HSMCI_BLKR,
                 (16u << 16) | 1);
    qtest_writel(qts, route->hsmci_base + HSMCI_ARGR, 0);
    qtest_writel(qts, route->hsmci_base + HSMCI_CMDR,
                 17 | HSMCI_CMDR_RSP_48 | HSMCI_CMDR_MAXLAT_64 |
                 HSMCI_CMDR_START | HSMCI_CMDR_READ);
    qtest_clock_step(qts, 1);
}

static void test_hsmci_dma_waits_for_request(const void *data)
{
    const DmaRoute *route = data;
    QTestState *qts = qtest_initf("-machine %s -S", route->machine);
    uint64_t dst = configure_hsmci_dma(qts, route, true);

    /* CMD17 starts a 16-byte read and raises the HSMCI DMA request. */
    start_hsmci_read(qts, route);

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
    qtest_writel(qts, route->hsmci_base + HSMCI_ARGR, 0);
    qtest_writel(qts, route->hsmci_base + HSMCI_CMDR,
                 24 | HSMCI_CMDR_RSP_48 | HSMCI_CMDR_MAXLAT_64 |
                 HSMCI_CMDR_START);
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, route->dmac_base + DMAC_CHSR) & 1,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, route->hsmci_base + HSMCI_SR) &
                    HSMCI_SR_BLKE, ==, HSMCI_SR_BLKE);
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

    /* Odd request bits pace the destination side. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_DSREQ(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_SREQ), ==, 0);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0x33330000);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_SADDR), ==, src + 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_DADDR), ==, dst + 4);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 7);

    qtest_writel(qts, G45_DMAC_BASE + DMAC_CREQ, DMAC_DCREQ(channel));
    qtest_clock_step(qts, 1);
    for (word = 0; word < 5; word++) {
        g_assert_cmphex(qtest_readl(qts, dst + 4 * word), ==,
                        0x33330000 + word);
    }
    g_assert_cmphex(qtest_readl(qts, dst + 20), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, channel_base + DMAC_CTRLA) & 0xffff,
                    ==, 3);

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

    /* A source request issued before global/channel enable must be retained. */
    qtest_writel(qts, G45_DMAC_BASE + DMAC_SREQ, DMAC_SSREQ(channel));
    qtest_writel(qts, G45_DMAC_BASE + DMAC_EN, 1);
    qtest_writel(qts, G45_DMAC_BASE + DMAC_CHER, DMAC_ENA(channel));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_DMAC_BASE + DMAC_SREQ), ==,
                    DMAC_SSREQ(channel));
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xdeadbeef);

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
    qtest_add_func("/at91-dmac/g45/late-irq-enable",
                   test_dmac_late_irq_enable);
    qtest_add_func("/at91-dmac/g45/word-width-alias",
                   test_dmac_word_width_alias);
    qtest_add_data_func("/at91-dmac/g45/hsmci1/request-13", &routes[1],
                        test_hsmci_dma_waits_for_request);
    qtest_add_func("/at91-dmac/g45/hsmci1/tx-request-13",
                   test_hsmci_tx_dma_request);
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

    return g_test_run();
}
