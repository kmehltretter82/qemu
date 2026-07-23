/*
 * QTest tests for the AT91 AC97C channel-A PDC.
 *
 * The data path is paced by the QEMU audio subsystem: with the default
 * (none) backend its mixing timer runs on the virtual clock, so stepping
 * the qtest clock advances playback/capture deterministically at the
 * codec-programmed rate.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define G45_AC97_BASE          0xfffac000
#define G45_SDRAM_BASE         0x70000000

#define AC97C_MR               0x008
#define AC97C_OCA              0x014
#define AC97C_CASR             0x028
#define AC97C_CAMR             0x02c
#define AC97C_COTHR            0x044
#define AC97C_SR               0x050
#define PDC_TPR                0x108
#define PDC_TCR                0x10c
#define PDC_TNPR               0x118
#define PDC_TNCR               0x11c
#define PDC_PTCR               0x120

#define MR_ENA                 (1u << 0)
#define CSR_ENDTX              (1u << 10)
#define CSR_TXEMPTY            (1u << 1)
#define CMR_CENA               (1u << 21)
#define CMR_DMAEN              (1u << 22)
#define SR_CAEVT               (1u << 3)
#define PDC_TXTEN              (1u << 8)

/* Channel A on slots 3 and 4 (three bits per slot). */
#define OCA_CHANNEL_A_FRONT    ((1u << 0) | (1u << 3))

static void ac97_write(QTestState *qts, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, G45_AC97_BASE + reg, value);
}

static uint32_t ac97_read(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, G45_AC97_BASE + reg);
}

/*
 * A current buffer and a chained next buffer drain at the 48 kHz default
 * rate.  Both period completions latch read-to-clear ENDTX in CASR and
 * CAEVT in SR; the counters promote at the period boundary.
 */
static void test_ac97_pdc_playback_chain(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint32_t buf1 = G45_SDRAM_BASE + 0x130000;
    const uint32_t buf2 = G45_SDRAM_BASE + 0x131000;
    const uint32_t samples = 1024;        /* 16-bit halfword transfers */
    int i;

    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    for (i = 0; i < samples; i++) {
        qtest_writew(qts, buf1 + 2 * i, 0x1000 + i);
        qtest_writew(qts, buf2 + 2 * i, 0x9000 + i);
    }
    ac97_write(qts, AC97C_MR, MR_ENA);
    ac97_write(qts, AC97C_OCA, OCA_CHANNEL_A_FRONT);
    ac97_write(qts, AC97C_CAMR, CMR_CENA | CMR_DMAEN);
    ac97_write(qts, PDC_TPR, buf1);
    ac97_write(qts, PDC_TCR, samples);
    ac97_write(qts, PDC_TNPR, buf2);
    ac97_write(qts, PDC_TNCR, samples);
    ac97_write(qts, PDC_PTCR, PDC_TXTEN);

    /*
     * 2048 halfwords of 48 kHz stereo are ~10.7 ms of audio.  Step well
     * past that and require both buffers consumed with the chain promoted.
     */
    for (i = 0; i < 400; i++) {
        qtest_clock_step(qts, 1000000);
        if (ac97_read(qts, PDC_TCR) == 0 && ac97_read(qts, PDC_TNCR) == 0) {
            break;
        }
    }
    g_assert_cmphex(ac97_read(qts, PDC_TCR), ==, 0);
    g_assert_cmphex(ac97_read(qts, PDC_TNCR), ==, 0);
    g_assert_cmphex(ac97_read(qts, PDC_TPR), ==, buf2 + 2 * samples);

    /* Channel completion is ENDTX; TXEMPTY belongs to the command channel. */
    g_assert_cmphex(ac97_read(qts, AC97C_CASR) & CSR_ENDTX, ==, CSR_ENDTX);
    /* CASR is read-to-clear. */
    g_assert_cmphex(ac97_read(qts, AC97C_CASR), ==, 0);
    qtest_quit(qts);
}

/* A next buffer programmed after the current one drained must restart. */
static void test_ac97_pdc_late_next(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint32_t buf1 = G45_SDRAM_BASE + 0x132000;
    const uint32_t buf2 = G45_SDRAM_BASE + 0x133000;
    const uint32_t samples = 512;
    int i;

    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    for (i = 0; i < samples; i++) {
        qtest_writew(qts, buf1 + 2 * i, 0x2000 + i);
        qtest_writew(qts, buf2 + 2 * i, 0xa000 + i);
    }
    ac97_write(qts, AC97C_MR, MR_ENA);
    ac97_write(qts, AC97C_OCA, OCA_CHANNEL_A_FRONT);
    ac97_write(qts, AC97C_CAMR, CMR_CENA | CMR_DMAEN);
    ac97_write(qts, PDC_TPR, buf1);
    ac97_write(qts, PDC_TCR, samples);
    ac97_write(qts, PDC_PTCR, PDC_TXTEN);
    for (i = 0; i < 400; i++) {
        qtest_clock_step(qts, 1000000);
        if (ac97_read(qts, PDC_TCR) == 0) {
            break;
        }
    }
    g_assert_cmphex(ac97_read(qts, PDC_TCR), ==, 0);

    ac97_write(qts, PDC_TNPR, buf2);
    ac97_write(qts, PDC_TNCR, samples);
    for (i = 0; i < 400; i++) {
        qtest_clock_step(qts, 1000000);
        if (ac97_read(qts, PDC_TCR) == 0 && ac97_read(qts, PDC_TNCR) == 0 &&
            ac97_read(qts, PDC_TPR) == buf2 + 2 * samples) {
            break;
        }
    }
    g_assert_cmphex(ac97_read(qts, PDC_TPR), ==, buf2 + 2 * samples);
    g_assert_cmphex(ac97_read(qts, PDC_TNCR), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-ac97/pdc/playback-chain",
                   test_ac97_pdc_playback_chain);
    qtest_add_func("/at91-ac97/pdc/late-next", test_ac97_pdc_late_next);

    return g_test_run();
}
