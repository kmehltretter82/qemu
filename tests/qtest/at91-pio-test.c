/*
 * QTest tests for the AT91SAM9G45 Parallel I/O controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define G45_PIOA_BASE          0xfffff200
#define G45_PIOA_PATH          "/machine/pioa"
#define G45_AIC_BASE           0xfffff000
#define G45_AIC_IPR            0x10c
#define G45_AIC_PIOA           (1u << 2)
#define G45_PMC_BASE           0xfffffc00
#define PMC_MCKR               0x30
#define MCKR_MAINCK            0x00000001

#define PIO_PER                0x00
#define PIO_PDR                0x04
#define PIO_PSR                0x08
#define PIO_OER                0x10
#define PIO_ODR                0x14
#define PIO_OSR                0x18
#define PIO_IFER               0x20
#define PIO_IFDR               0x24
#define PIO_IFSR               0x28
#define PIO_SODR               0x30
#define PIO_CODR               0x34
#define PIO_ODSR               0x38
#define PIO_PDSR               0x3c
#define PIO_IER                0x40
#define PIO_IDR                0x44
#define PIO_IMR                0x48
#define PIO_ISR                0x4c
#define PIO_MDER               0x50
#define PIO_MDDR               0x54
#define PIO_MDSR               0x58
#define PIO_PUDR               0x60
#define PIO_PUER               0x64
#define PIO_PUSR               0x68
#define PIO_ASR                0x70
#define PIO_BSR                0x74
#define PIO_ABSR               0x78
#define PIO_OWER               0xa0
#define PIO_OWDR               0xa4
#define PIO_OWSR               0xa8

/* The board reset clock tree supplies a 132 MHz MCK: ceil(1e9 / 132e6). */
#define FILTER_PERIOD_NS       8
#define MAINCK_FILTER_NS       84

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

static void pio_write(QTestState *qts, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, G45_PIOA_BASE + reg, value);
}

static uint32_t pio_read(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, G45_PIOA_BASE + reg);
}

static void set_pin(QTestState *qts, unsigned pin, int level)
{
    qtest_set_irq_in(qts, G45_PIOA_PATH, NULL, pin, level);
}

static void assert_aic_pioa(QTestState *qts, bool asserted)
{
    uint32_t pending = qtest_readl(qts, G45_AIC_BASE + G45_AIC_IPR);

    g_assert_cmphex(pending & G45_AIC_PIOA,
                    ==, asserted ? G45_AIC_PIOA : 0);
}

static void test_registers_mux_and_sync_output(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek");
    const uint32_t pin = 1u << 0;

    qtest_irq_intercept_out(qts, G45_PIOA_PATH);
    g_assert_cmphex(pio_read(qts, PIO_PSR), ==, UINT32_MAX);
    g_assert_cmphex(pio_read(qts, PIO_OSR), ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_IFSR), ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_MDSR), ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_PUSR), ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_ABSR), ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_OWSR), ==, 0);

    pio_write(qts, PIO_CODR, pin);
    pio_write(qts, PIO_OER, pin);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* Peripheral ownership releases the preconfigured GPIO output. */
    pio_write(qts, PIO_PDR, pin);
    g_assert_cmphex(pio_read(qts, PIO_PSR) & pin, ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, pin);
    pio_write(qts, PIO_SODR, pin);
    g_assert_cmphex(pio_read(qts, PIO_ODSR) & pin, ==, pin);
    g_assert_false(qtest_get_irq(qts, 0));

    /* GPIO state programmed while muxed takes effect on PIO re-enable. */
    pio_write(qts, PIO_PER, pin);
    g_assert_cmphex(pio_read(qts, PIO_PSR) & pin, ==, pin);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, pin);
    g_assert_true(qtest_get_irq(qts, 0));

    pio_write(qts, PIO_BSR, pin);
    g_assert_cmphex(pio_read(qts, PIO_ABSR) & pin, ==, pin);
    pio_write(qts, PIO_ASR, pin);
    g_assert_cmphex(pio_read(qts, PIO_ABSR) & pin, ==, 0);

    /* ODSR direct writes affect only OWER-selected pins. */
    pio_write(qts, PIO_ODSR, 0);
    g_assert_cmphex(pio_read(qts, PIO_ODSR) & pin, ==, pin);
    pio_write(qts, PIO_OWER, pin);
    pio_write(qts, PIO_ODSR, 0);
    g_assert_cmphex(pio_read(qts, PIO_ODSR) & pin, ==, 0);
    pio_write(qts, PIO_OWDR, pin);

    qtest_quit(qts);
}

static void test_multidrive_resolution(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek -S");
    const uint32_t pin = 1u << 1;

    set_pin(qts, 1, -1);                 /* floating, internal pull-up */
    pio_write(qts, PIO_CODR, pin);
    pio_write(qts, PIO_OER, pin);
    pio_write(qts, PIO_MDER, pin);
    g_assert_cmphex(pio_read(qts, PIO_MDSR) & pin, ==, pin);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, 0);

    pio_write(qts, PIO_SODR, pin);       /* open-drain release */
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, pin);
    set_pin(qts, 1, 0);                  /* another bus participant wins */
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, 0);
    set_pin(qts, 1, 1);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, pin);

    pio_write(qts, PIO_CODR, pin);       /* PIO actively pulls low */
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, 0);
    pio_write(qts, PIO_MDDR, pin);
    g_assert_cmphex(pio_read(qts, PIO_MDSR) & pin, ==, 0);

    qtest_quit(qts);
}

static void test_interrupt_and_glitch_filter(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek");
    const uint32_t edge_pin = 1u << 2;
    const uint32_t filter_pin = 1u << 3;

    set_pin(qts, 2, 0);
    set_pin(qts, 3, 0);
    pio_read(qts, PIO_ISR);
    pio_write(qts, PIO_IER, edge_pin | filter_pin);
    g_assert_cmphex(pio_read(qts, PIO_IMR), ==, edge_pin | filter_pin);

    /* The old G45 PIO supports both-edge input-change interrupts. */
    set_pin(qts, 2, 1);
    assert_aic_pioa(qts, true);
    g_assert_cmphex(pio_read(qts, PIO_ISR), ==, edge_pin);
    assert_aic_pioa(qts, false);
    set_pin(qts, 2, 0);
    g_assert_cmphex(pio_read(qts, PIO_ISR), ==, edge_pin);

    pio_write(qts, PIO_IFER, filter_pin);
    g_assert_cmphex(pio_read(qts, PIO_IFSR) & filter_pin, ==, filter_pin);
    set_pin(qts, 3, 1);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & filter_pin, ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_ISR), ==, 0);
    qtest_clock_step(qts, FILTER_PERIOD_NS - 1);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & filter_pin, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & filter_pin, ==, filter_pin);
    g_assert_cmphex(pio_read(qts, PIO_ISR), ==, filter_pin);

    /* A pulse shorter than one MCK cycle never reaches PDSR or ISR. */
    set_pin(qts, 3, 0);
    qtest_clock_step(qts, FILTER_PERIOD_NS / 2 - 1);
    set_pin(qts, 3, 1);
    qtest_clock_step(qts, FILTER_PERIOD_NS * 2);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & filter_pin, ==, filter_pin);
    g_assert_cmphex(pio_read(qts, PIO_ISR), ==, 0);

    /* Disabling the filter exposes a pending raw level immediately. */
    set_pin(qts, 3, 0);
    pio_write(qts, PIO_IFDR, filter_pin);
    g_assert_cmphex(pio_read(qts, PIO_IFSR) & filter_pin, ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & filter_pin, ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_ISR), ==, filter_pin);
    pio_write(qts, PIO_IDR, edge_pin | filter_pin);

    qtest_quit(qts);
}

static void test_filter_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    const uint32_t pin = 1u << 4;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("at91-pio-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -S");
    set_pin(src, 4, 0);
    pio_read(src, PIO_ISR);
    pio_write(src, PIO_IFER, pin);
    pio_write(src, PIO_IER, pin);
    set_pin(src, 4, 1);
    qtest_qmp_assert_success(src, "{ 'execute': 'cont' }");
    qtest_clock_step(src, 3);
    g_assert_cmphex(pio_read(src, PIO_PDSR) & pin, ==, 0);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -S -incoming %s", uri);
    wait_for_migration_complete(dst);
    g_assert_cmphex(pio_read(dst, PIO_IFSR) & pin, ==, pin);
    g_assert_cmphex(pio_read(dst, PIO_IMR) & pin, ==, pin);
    g_assert_cmphex(pio_read(dst, PIO_PDSR) & pin, ==, 0);
    qtest_qmp_assert_success(dst, "{ 'execute': 'cont' }");
    qtest_clock_step(dst, FILTER_PERIOD_NS - 3 - 1);
    g_assert_cmphex(pio_read(dst, PIO_PDSR) & pin, ==, 0);
    qtest_clock_step(dst, 1);
    g_assert_cmphex(pio_read(dst, PIO_PDSR) & pin, ==, pin);
    g_assert_cmphex(pio_read(dst, PIO_ISR), ==, pin);
    assert_aic_pioa(dst, false);
    qtest_quit(dst);

    unlink(state_path);
}

static void test_filter_clock_change(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek");
    const uint32_t pin = 1u << 5;

    set_pin(qts, 5, 0);
    pio_read(qts, PIO_ISR);
    pio_write(qts, PIO_IFER, pin);
    pio_write(qts, PIO_IER, pin);
    set_pin(qts, 5, 1);
    qtest_clock_step(qts, 3);

    /*
     * Switching from 132 MHz MCK to 12 MHz MAINCK restarts sampling at the
     * new one-cycle period; a pending edge must not retain the old deadline.
     */
    qtest_writel(qts, G45_PMC_BASE + PMC_MCKR, MCKR_MAINCK);
    qtest_clock_step(qts, MAINCK_FILTER_NS - 1);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, 0);
    g_assert_cmphex(pio_read(qts, PIO_ISR), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(pio_read(qts, PIO_PDSR) & pin, ==, pin);
    g_assert_cmphex(pio_read(qts, PIO_ISR), ==, pin);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-pio/g45/registers-mux-sync-output",
                   test_registers_mux_and_sync_output);
    qtest_add_func("/at91-pio/g45/multidrive-resolution",
                   test_multidrive_resolution);
    qtest_add_func("/at91-pio/g45/interrupt-glitch-filter",
                   test_interrupt_and_glitch_filter);
    qtest_add_func("/at91-pio/g45/filter-migration",
                   test_filter_migration);
    qtest_add_func("/at91-pio/g45/filter-clock-change",
                   test_filter_clock_change);

    return g_test_run();
}
