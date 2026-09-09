/*
 * QTest testcase for BCM283x DMA engine (on Raspberry Pi 3)
 * and its interrupts coming to Interrupt Controller.
 *
 * Copyright (c) 2022 Auriga LLC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

/* Offsets in raspi3b platform: */
#define RASPI3_DMA_BASE 0x3f007000
#define RASPI3_IC_BASE  0x3f00b200

/* Used register/fields definitions */

/* DMA engine registers: */
#define BCM2708_DMA_CS         0
#define BCM2708_DMA_ACTIVE     (1 << 0)
#define BCM2708_DMA_INT        (1 << 2)

#define BCM2708_DMA_ADDR       0x04

#define BCM2708_DMA_INT_STATUS 0xfe0

/* DMA Transfer Info fields: */
#define BCM2708_DMA_INT_EN     (1 << 0)
#define BCM2708_DMA_D_INC      (1 << 4)
#define BCM2708_DMA_S_INC      (1 << 8)

/* Interrupt controller registers: */
#define IRQ_PENDING_BASIC      0x00
#define IRQ_GPU_PENDING1_AGGR  (1 << 8)
#define IRQ_PENDING_1          0x04
#define IRQ_ENABLE_1           0x10

/* Data for the test: */
#define SCB_ADDR   256
#define S_ADDR     32
#define D_ADDR     64
#define TXFR_LEN   32
const uint32_t check_data = 0x12345678;

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

static void bcm2835_dma_test_interrupt(int dma_c, int irq_line)
{
    uint64_t dma_base = RASPI3_DMA_BASE + dma_c * 0x100;
    int gpu_irq_line = 16 + irq_line;

    /* Check that interrupts are silent by default: */
    writel(RASPI3_IC_BASE + IRQ_ENABLE_1, 1 << gpu_irq_line);
    int isr = readl(dma_base + BCM2708_DMA_INT_STATUS);
    g_assert_cmpint(isr, ==, 0);
    uint32_t reg0 = readl(dma_base + BCM2708_DMA_CS);
    g_assert_cmpint(reg0, ==, 0);
    uint32_t ic_pending = readl(RASPI3_IC_BASE + IRQ_PENDING_BASIC);
    g_assert_cmpint(ic_pending, ==, 0);
    uint32_t gpu_pending1 = readl(RASPI3_IC_BASE + IRQ_PENDING_1);
    g_assert_cmpint(gpu_pending1, ==, 0);

    /* Prepare Control Block: */
    writel(SCB_ADDR + 0, BCM2708_DMA_S_INC | BCM2708_DMA_D_INC |
                         BCM2708_DMA_INT_EN); /* transfer info */
    writel(SCB_ADDR + 4, S_ADDR);             /* source address */
    writel(SCB_ADDR + 8, D_ADDR);             /* destination address */
    writel(SCB_ADDR + 12, TXFR_LEN);          /* transfer length */
    writel(dma_base + BCM2708_DMA_ADDR, SCB_ADDR);

    writel(S_ADDR, check_data);
    for (int word = S_ADDR + 4; word < S_ADDR + TXFR_LEN; word += 4) {
        writel(word, ~check_data);
    }
    /* Perform the transfer: */
    writel(dma_base + BCM2708_DMA_CS, BCM2708_DMA_ACTIVE);

    /* Check that destination == source: */
    uint32_t data = readl(D_ADDR);
    g_assert_cmpint(data, ==, check_data);
    for (int word = D_ADDR + 4; word < D_ADDR + TXFR_LEN; word += 4) {
        data = readl(word);
        g_assert_cmpint(data, ==, ~check_data);
    }

    /* Check that interrupt status is set both in DMA and IC controllers: */
    isr = readl(RASPI3_DMA_BASE + BCM2708_DMA_INT_STATUS);
    g_assert_cmpint(isr, ==, 1 << dma_c);

    ic_pending = readl(RASPI3_IC_BASE + IRQ_PENDING_BASIC);
    g_assert_cmpint(ic_pending, ==, IRQ_GPU_PENDING1_AGGR);

    gpu_pending1 = readl(RASPI3_IC_BASE + IRQ_PENDING_1);
    g_assert_cmpint(gpu_pending1, ==, 1 << gpu_irq_line);

    /* Clean up, clear interrupt: */
    writel(dma_base + BCM2708_DMA_CS, BCM2708_DMA_INT);
}

static void bcm2835_dma_test_interrupts(void)
{
    /* DMA engines 0--10 have separate IRQ lines, 11--14 - only one: */
    bcm2835_dma_test_interrupt(0,  0);
    bcm2835_dma_test_interrupt(10, 10);
    bcm2835_dma_test_interrupt(11, 11);
    bcm2835_dma_test_interrupt(14, 11);
}

static void test_dma_reset_clears_irq(void)
{
    uint64_t dma_base = RASPI3_DMA_BASE;

    writel(SCB_ADDR + 0, BCM2708_DMA_S_INC | BCM2708_DMA_D_INC |
                         BCM2708_DMA_INT_EN);
    writel(SCB_ADDR + 4, S_ADDR);
    writel(SCB_ADDR + 8, D_ADDR);
    writel(SCB_ADDR + 12, TXFR_LEN);
    writel(dma_base + BCM2708_DMA_ADDR, SCB_ADDR);
    writel(S_ADDR, check_data);
    writel(dma_base + BCM2708_DMA_CS, BCM2708_DMA_ACTIVE);

    writel(RASPI3_IC_BASE + IRQ_ENABLE_1, 1 << 16);
    g_assert_cmphex(readl(RASPI3_IC_BASE + IRQ_PENDING_1) & (1 << 16),
                    ==, 1 << 16);

    qtest_system_reset(global_qtest);

    writel(RASPI3_IC_BASE + IRQ_ENABLE_1, 1 << 16);
    g_assert_cmphex(readl(RASPI3_IC_BASE + IRQ_PENDING_1) & (1 << 16),
                    ==, 0);
}

static void test_dma_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    const uint64_t dma_base = RASPI3_DMA_BASE;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-dma-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b -S");
    qtest_irq_intercept_out_named(src, "/machine/soc/peripherals/dma",
                                  "sysbus-irq");
    qtest_writel(src, SCB_ADDR + 0,
                 BCM2708_DMA_S_INC | BCM2708_DMA_D_INC | BCM2708_DMA_INT_EN);
    qtest_writel(src, SCB_ADDR + 4, S_ADDR);
    qtest_writel(src, SCB_ADDR + 8, D_ADDR);
    qtest_writel(src, SCB_ADDR + 12, TXFR_LEN);
    qtest_writel(src, dma_base + BCM2708_DMA_ADDR, SCB_ADDR);
    qtest_writel(src, S_ADDR, check_data);
    qtest_writel(src, dma_base + BCM2708_DMA_CS, BCM2708_DMA_ACTIVE);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/dma",
                                  "sysbus-irq");
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_true(qtest_get_irq(dst, 0));
    qtest_quit(dst);

    unlink(state_path);
}

int main(int argc, char **argv)
{
    int ret;
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2835/dma/test_interrupts",
                   bcm2835_dma_test_interrupts);
    qtest_add_func("/bcm2835/dma/reset_clears_irq",
                   test_dma_reset_clears_irq);
    qtest_add_func("/bcm2835/dma/migration_irq",
                   test_dma_irq_migration);
    qtest_start("-machine raspi3b");
    ret = g_test_run();
    qtest_end();
    return ret;
}
