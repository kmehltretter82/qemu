/*
 * QTest tests for the AT91SAM9G45 LCD controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "libqtest.h"

#define G45_LCDC_BASE          0x00500000
#define G45_SDRAM_BASE         0x70000000
#define G45_AIC_BASE           0xfffff000
#define G45_AIC_IPR            0x10c
#define G45_AIC_LCDC           (1u << 23)

#define LCDC_DMABADDR1         0x0000
#define LCDC_DMAFRMCFG         0x0018
#define LCDC_DMACON            0x001c
#define LCDC_LCDCON1           0x0800
#define LCDC_LCDCON2           0x0804
#define LCDC_TIM1              0x0808
#define LCDC_TIM2              0x080c
#define LCDC_LCDFRMCFG         0x0810
#define LCDC_PWRCON            0x083c
#define LCDC_IER               0x0848
#define LCDC_IDR               0x084c
#define LCDC_IMR               0x0850
#define LCDC_ISR               0x0854
#define LCDC_ICR               0x0858
#define LCDC_LUT(n)            (0x0c00 + (n) * 4)

#define LCDC_DMACON_DMAEN      (1u << 0)
#define LCDC_LCDCON1_BYPASS    (1u << 0)
#define LCDC_LCDCON2_TFT       (2u << 0)
#define LCDC_LCDCON2_BPP(code) ((code) << 5)
#define LCDC_LCDCON2_LITTLE    (1u << 31)
#define LCDC_PWRCON_PWR        (1u << 0)
#define LCDC_IRQ_EOF           (1u << 2)

#define TEST_WIDTH             8
#define TEST_HEIGHT            2
#define TEST_FB                (G45_SDRAM_BASE + 0x40000)
#define TEST_FRAME_NS          250

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

static void lcdc_write(QTestState *qts, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, G45_LCDC_BASE + reg, value);
}

static uint32_t lcdc_read(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, G45_LCDC_BASE + reg);
}

static void configure_lcdc(QTestState *qts, unsigned bpp_code)
{
    lcdc_write(qts, LCDC_PWRCON, 0);
    lcdc_write(qts, LCDC_DMACON, 0);
    lcdc_write(qts, LCDC_DMABADDR1, TEST_FB);
    lcdc_write(qts, LCDC_DMAFRMCFG, 2);
    lcdc_write(qts, LCDC_LCDCON1, LCDC_LCDCON1_BYPASS);
    lcdc_write(qts, LCDC_LCDCON2, LCDC_LCDCON2_LITTLE |
               LCDC_LCDCON2_TFT | LCDC_LCDCON2_BPP(bpp_code));
    lcdc_write(qts, LCDC_TIM1, 0);
    lcdc_write(qts, LCDC_TIM2, 0);
    lcdc_write(qts, LCDC_LCDFRMCFG,
               ((TEST_WIDTH - 1) << 21) | (TEST_HEIGHT - 1));
    lcdc_write(qts, LCDC_DMACON, LCDC_DMACON_DMAEN);
    lcdc_write(qts, LCDC_PWRCON, LCDC_PWRCON_PWR);
}

static void write_indexed_frame(QTestState *qts, unsigned bpp)
{
    uint8_t frame[16] = { 0 };
    unsigned stride = DIV_ROUND_UP(TEST_WIDTH * bpp, 32) * 4;
    unsigned mask = (1u << bpp) - 1;
    int x, y;

    for (y = 0; y < TEST_HEIGHT; y++) {
        for (x = 0; x < TEST_WIDTH; x++) {
            unsigned index = MIN((unsigned)x & mask, 3u);
            unsigned bit = x * bpp;

            frame[y * stride + bit / 8] |= index << (bit % 8);
        }
    }
    qtest_bufwrite(qts, TEST_FB, frame, stride * TEST_HEIGHT);
}

static void assert_ppm_palette(const char *path, unsigned bpp)
{
    static const uint8_t colors[4][3] = {
        { 0x00, 0x00, 0x00 },
        { 0xff, 0x00, 0x00 },
        { 0x00, 0xff, 0x00 },
        { 0x00, 0x00, 0xff },
    };
    g_autofree gchar *contents = NULL;
    g_autoptr(GError) error = NULL;
    gsize length;
    int width, height, header = 0;
    unsigned mask = (1u << bpp) - 1;
    int x, y;

    g_assert_true(g_file_get_contents(path, &contents, &length, &error));
    g_assert_cmpint(sscanf(contents, "P6\n%d %d\n255\n%n",
                           &width, &height, &header), ==, 2);
    g_assert_cmpint(width, ==, TEST_WIDTH);
    g_assert_cmpint(height, ==, TEST_HEIGHT);
    g_assert_cmpuint(length, >=, header + TEST_WIDTH * TEST_HEIGHT * 3);

    for (y = 0; y < TEST_HEIGHT; y++) {
        for (x = 0; x < TEST_WIDTH; x++) {
            unsigned index = MIN((unsigned)x & mask, 3u);
            const uint8_t *pixel = (uint8_t *)contents + header +
                                   (y * TEST_WIDTH + x) * 3;

            g_assert_cmpmem(pixel, 3, colors[index], 3);
        }
    }
}

static void capture_and_assert_palette(QTestState *qts, unsigned bpp)
{
    g_autofree char *path = NULL;
    int fd;

    fd = g_file_open_tmp("at91-lcdc-palette-XXXXXX.ppm", &path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qtest_qmp_assert_success(qts,
        "{ 'execute': 'screendump', "
        "  'arguments': { 'filename': %s, 'format': 'ppm' } }", path);
    assert_ppm_palette(path, bpp);
    unlink(path);
}

static void test_indexed_palette_scanout(void)
{
    static const uint16_t palette[4] = {
        0x0000, 0xf800, 0x07e0, 0x001f,
    };
    static const unsigned bpp[] = { 1, 2, 4, 8 };
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    int i, n;

    for (n = 0; n < ARRAY_SIZE(palette); n++) {
        lcdc_write(qts, LCDC_LUT(n), 0xdead0000 | palette[n]);
        g_assert_cmphex(lcdc_read(qts, LCDC_LUT(n)), ==, palette[n]);
    }

    for (i = 0; i < ARRAY_SIZE(bpp); i++) {
        write_indexed_frame(qts, bpp[i]);
        configure_lcdc(qts, i);
        capture_and_assert_palette(qts, bpp[i]);
    }

    qtest_quit(qts);
}

static void test_eof_interrupt(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek");

    lcdc_write(qts, LCDC_IER, LCDC_IRQ_EOF);
    configure_lcdc(qts, 6);
    qtest_clock_step(qts, TEST_FRAME_NS - 1);
    g_assert_cmphex(lcdc_read(qts, LCDC_ISR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_LCDC, ==, 0);

    qtest_clock_step(qts, 1);
    g_assert_cmphex(lcdc_read(qts, LCDC_ISR), ==, LCDC_IRQ_EOF);
    g_assert_cmphex(qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_LCDC, ==, G45_AIC_LCDC);
    lcdc_write(qts, LCDC_ICR, LCDC_IRQ_EOF);
    g_assert_cmphex(lcdc_read(qts, LCDC_ISR), ==, 0);

    /* Status continues while masked, and late enable asserts the level IRQ. */
    lcdc_write(qts, LCDC_IDR, LCDC_IRQ_EOF);
    g_assert_cmphex(lcdc_read(qts, LCDC_IMR), ==, 0);
    qtest_clock_step(qts, TEST_FRAME_NS);
    g_assert_cmphex(lcdc_read(qts, LCDC_ISR), ==, LCDC_IRQ_EOF);
    g_assert_cmphex(qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_LCDC, ==, 0);
    lcdc_write(qts, LCDC_IER, LCDC_IRQ_EOF);
    g_assert_cmphex(qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_LCDC, ==, G45_AIC_LCDC);
    lcdc_write(qts, LCDC_ICR, LCDC_IRQ_EOF);

    /* Power-down cancels future frame events. */
    lcdc_write(qts, LCDC_PWRCON, 0);
    qtest_clock_step(qts, 4 * TEST_FRAME_NS);
    g_assert_cmphex(lcdc_read(qts, LCDC_ISR), ==, 0);
    qtest_quit(qts);
}

static void test_active_frame_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("at91-lcdc-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    lcdc_write(src, LCDC_LUT(1), 0xf800);
    lcdc_write(src, LCDC_IER, LCDC_IRQ_EOF);
    configure_lcdc(src, 3);
    qtest_clock_step(src, TEST_FRAME_NS / 2);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(lcdc_read(dst, LCDC_LUT(1)), ==, 0xf800);
    g_assert_cmphex(lcdc_read(dst, LCDC_IMR), ==, LCDC_IRQ_EOF);
    g_assert_cmphex(lcdc_read(dst, LCDC_ISR), ==, 0);
    qtest_qmp_assert_success(dst, "{ 'execute': 'cont' }");
    qtest_clock_step(dst, TEST_FRAME_NS / 2 - 1);
    g_assert_cmphex(lcdc_read(dst, LCDC_ISR), ==, 0);
    qtest_clock_step(dst, 1);
    g_assert_cmphex(lcdc_read(dst, LCDC_ISR), ==, LCDC_IRQ_EOF);
    g_assert_cmphex(qtest_readl(dst, G45_AIC_BASE + G45_AIC_IPR) &
                    G45_AIC_LCDC, ==, G45_AIC_LCDC);
    qtest_quit(dst);
    unlink(state_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-lcdc/g45/indexed-palette",
                   test_indexed_palette_scanout);
    qtest_add_func("/at91-lcdc/g45/eof-interrupt", test_eof_interrupt);
    qtest_add_func("/at91-lcdc/g45/active-frame-migration",
                   test_active_frame_migration);

    return g_test_run();
}
