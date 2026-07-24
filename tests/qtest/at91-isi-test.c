/*
 * QTest tests for the AT91 Image Sensor Interface frame descriptors.
 *
 * Frame content is synthetic and deterministic (bytes 0..3 are the
 * little-endian frame counter, byte j >= 4 is j*3 + counter*11), and
 * the 30 fps frame timer runs on the virtual clock, so clock stepping
 * drives capture exactly.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define G45_ISI_BASE           0xfffb4000
#define G45_SDRAM_BASE         0x70000000

#define ISI_CFG1               0x00
#define ISI_CFG2               0x04
#define ISI_PSIZE              0x08
#define ISI_CTRL               0x24
#define ISI_STATUS             0x28
#define ISI_DMA_CHER           0x38
#define ISI_DMA_CHSR           0x40
#define ISI_DMA_P_DSCR         0x4c
#define ISI_DMA_C_ADDR         0x50
#define ISI_DMA_C_DSCR         0x58

#define ISI_CTRL_EN            (1u << 0)
#define ISI_CTRL_CDC           (1u << 8)
#define ISI_SR_PXFR_DONE       (1u << 16)
#define ISI_SR_CXFR_DONE       (1u << 17)
#define ISI_DMA_P_CH           (1u << 0)
#define ISI_DMA_C_CH           (1u << 1)
#define ISI_FBD_CTRL_WB        (1u << 1)
#define ISI_FBD_CTRL_DONE      (1u << 3)

#define FRAME_INTERVAL_NS      33333333

/* 16x8 pixels at 2 bytes per pixel. */
#define FRAME_W                16
#define FRAME_H                8
#define FRAME_LEN              (FRAME_W * FRAME_H * 2)

static uint8_t expected_byte(unsigned j, uint32_t counter)
{
    if (j < 4) {
        return counter >> (8 * j);
    }
    return (uint8_t)(j * 3 + counter * 11);
}

static void check_frame(QTestState *qts, uint32_t buf, uint32_t counter)
{
    uint8_t data[FRAME_LEN];
    unsigned j;

    qtest_memread(qts, buf, data, FRAME_LEN);
    for (j = 0; j < FRAME_LEN; j++) {
        g_assert_cmphex(data[j], ==, expected_byte(j, counter));
    }
}

/*
 * Codec-channel descriptor chain: each armed frame fetches one fbd,
 * fills the buffer, writes DONE back when WB is set, advances DSCR to
 * the chained descriptor and idles the channel until the driver
 * re-arms - exactly the atmel-isi interrupt-handler cycle.
 */
static void test_isi_codec_descriptor_chain(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint32_t fbd0 = G45_SDRAM_BASE + 0x160000;
    const uint32_t fbd1 = G45_SDRAM_BASE + 0x160020;
    const uint32_t buf0 = G45_SDRAM_BASE + 0x161000;
    const uint32_t buf1 = G45_SDRAM_BASE + 0x162000;
    uint32_t status;

    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    qtest_writel(qts, fbd0 + 0, buf0);
    qtest_writel(qts, fbd0 + 4, ISI_FBD_CTRL_WB);
    qtest_writel(qts, fbd0 + 8, fbd1);
    qtest_writel(qts, fbd1 + 0, buf1);
    qtest_writel(qts, fbd1 + 4, ISI_FBD_CTRL_WB);
    qtest_writel(qts, fbd1 + 8, 0);
    qtest_memset(qts, buf0, 0xcc, FRAME_LEN);
    qtest_memset(qts, buf1, 0xcc, FRAME_LEN);

    qtest_writel(qts, G45_ISI_BASE + ISI_CFG2,
                 ((FRAME_W - 1) << 16) | (FRAME_H - 1));
    qtest_writel(qts, G45_ISI_BASE + ISI_DMA_C_DSCR, fbd0);
    qtest_writel(qts, G45_ISI_BASE + ISI_DMA_CHER, ISI_DMA_C_CH);
    qtest_writel(qts, G45_ISI_BASE + ISI_CTRL, ISI_CTRL_EN | ISI_CTRL_CDC);

    qtest_clock_step(qts, FRAME_INTERVAL_NS);
    check_frame(qts, buf0, 0);
    g_assert_cmphex(qtest_readl(qts, fbd0 + 4) &
                    (ISI_FBD_CTRL_WB | ISI_FBD_CTRL_DONE), ==,
                    ISI_FBD_CTRL_WB | ISI_FBD_CTRL_DONE);
    g_assert_cmphex(qtest_readl(qts, G45_ISI_BASE + ISI_DMA_C_DSCR), ==,
                    fbd1);
    g_assert_cmphex(qtest_readl(qts, G45_ISI_BASE + ISI_DMA_C_ADDR), ==,
                    buf0 + FRAME_LEN);
    g_assert_cmphex(qtest_readl(qts, G45_ISI_BASE + ISI_DMA_CHSR) &
                    ISI_DMA_C_CH, ==, 0);
    status = qtest_readl(qts, G45_ISI_BASE + ISI_STATUS);
    g_assert_cmphex(status & ISI_SR_CXFR_DONE, ==, ISI_SR_CXFR_DONE);
    /* STATUS is clear-on-read. */
    g_assert_cmphex(qtest_readl(qts, G45_ISI_BASE + ISI_STATUS) &
                    ISI_SR_CXFR_DONE, ==, 0);

    /* An un-rearmed channel captures nothing on the next tick. */
    qtest_clock_step(qts, FRAME_INTERVAL_NS);
    g_assert_cmphex(qtest_readl(qts, buf1), ==, 0xcccccccc);

    /* Re-arm like the driver's IRQ handler: next frame uses fbd1. */
    qtest_writel(qts, G45_ISI_BASE + ISI_DMA_CHER, ISI_DMA_C_CH);
    qtest_writel(qts, G45_ISI_BASE + ISI_CTRL, ISI_CTRL_EN | ISI_CTRL_CDC);
    qtest_clock_step(qts, FRAME_INTERVAL_NS);
    check_frame(qts, buf1, 1);   /* the un-armed tick captured nothing */
    g_assert_cmphex(qtest_readl(qts, fbd1 + 4) &
                    (ISI_FBD_CTRL_WB | ISI_FBD_CTRL_DONE), ==,
                    ISI_FBD_CTRL_WB | ISI_FBD_CTRL_DONE);
    g_assert_cmphex(qtest_readl(qts, G45_ISI_BASE + ISI_DMA_C_DSCR), ==, 0);
    qtest_quit(qts);
}

/* Preview channel: PSIZE dimensions, no WB bit means no writeback. */
static void test_isi_preview_no_writeback(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint32_t fbd = G45_SDRAM_BASE + 0x163000;
    const uint32_t buf = G45_SDRAM_BASE + 0x164000;
    uint32_t ctrl_word;

    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    qtest_writel(qts, fbd + 0, buf);
    qtest_writel(qts, fbd + 4, 0);
    qtest_writel(qts, fbd + 8, 0);
    qtest_memset(qts, buf, 0xcc, FRAME_LEN);

    qtest_writel(qts, G45_ISI_BASE + ISI_PSIZE,
                 ((FRAME_W - 1) << 16) | (FRAME_H - 1));
    qtest_writel(qts, G45_ISI_BASE + ISI_DMA_P_DSCR, fbd);
    qtest_writel(qts, G45_ISI_BASE + ISI_DMA_CHER, ISI_DMA_P_CH);
    qtest_writel(qts, G45_ISI_BASE + ISI_CTRL, ISI_CTRL_EN);

    qtest_clock_step(qts, FRAME_INTERVAL_NS);
    check_frame(qts, buf, 0);
    ctrl_word = qtest_readl(qts, fbd + 4);
    g_assert_cmphex(ctrl_word, ==, 0);   /* untouched: WB clear */
    g_assert_cmphex(qtest_readl(qts, G45_ISI_BASE + ISI_STATUS) &
                    ISI_SR_PXFR_DONE, ==, ISI_SR_PXFR_DONE);
    g_assert_cmphex(qtest_readl(qts, G45_ISI_BASE + ISI_DMA_CHSR) &
                    ISI_DMA_P_CH, ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-isi/codec-descriptor-chain",
                   test_isi_codec_descriptor_chain);
    qtest_add_func("/at91-isi/preview-no-writeback",
                   test_isi_preview_no_writeback);

    return g_test_run();
}
