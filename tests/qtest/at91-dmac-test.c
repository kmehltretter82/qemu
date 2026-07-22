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
#define DMAC_CHSR              0x30
#define DMAC_CH0_BASE          0x3c
#define DMAC_DSCR              0x08
#define DMAC_CFG               0x14

#define DMAC_CTRLA_BTSIZE(x)   (x)
#define DMAC_CTRLA_SRC_WIDTH_4 (2u << 24)
#define DMAC_CTRLA_DST_WIDTH_4 (2u << 28)
#define DMAC_CTRLB_FC_MEM2PER  (1u << 21)
#define DMAC_CTRLB_FC_PER2MEM  (2u << 21)
#define DMAC_CTRLB_SRC_FIXED   (2u << 24)
#define DMAC_CTRLB_DST_FIXED   (2u << 28)

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

    qtest_writel(qts, route->dmac_base + DMAC_CH0_BASE + DMAC_DSCR, lli);
    qtest_writel(qts, route->dmac_base + DMAC_CH0_BASE + DMAC_CFG,
                 dmac_cfg_src_per(route->request_id));
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
                 dmac_cfg_dst_per(route->request_id));
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

    return g_test_run();
}
