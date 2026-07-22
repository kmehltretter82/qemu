/*
 * QTest tests for the AT91SAM9 watchdog timer.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "libqtest.h"

#define G45_WDT_BASE       0xfffffd40
#define G35_WDT_BASE       0xfffffe40
#define AIC_BASE           0xfffff000
#define AIC_IPR            0x10c
#define AIC_SYS_IRQ        (1u << 1)

#define WDT_CR             0x00
#define WDT_MR             0x04
#define WDT_SR             0x08

#define WDT_CR_KEY         0xa5000000
#define WDT_CR_WDRSTT      (1u << 0)
#define WDT_MR_WDFIEN      (1u << 12)
#define WDT_MR_WDRSTEN     (1u << 13)
#define WDT_MR_WDDIS       (1u << 15)
#define WDT_MR_WDD(value)  ((value) << 16)
#define WDT_MR_RESET       0x3fff2fff
#define WDT_SR_WDUNF       (1u << 0)
#define WDT_SR_WDERR       (1u << 1)

#define WDT_TICK_NS        (NANOSECONDS_PER_SECOND / 256)

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

static int64_t clock_step_to(QTestState *qts, int64_t now, int64_t target)
{
    while (now < target) {
        int64_t next = qtest_clock_step(qts, target - now);

        g_assert_cmpint(next, >, now);
        now = next;
    }
    return now;
}

static void test_mode_write_once_and_reset(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek "
                                 "-watchdog-action none");
    uint32_t disabled = WDT_MR_WDDIS | WDT_MR_WDD(5) | 5;

    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_MR), ==,
                    WDT_MR_RESET);
    qtest_writel(qts, G45_WDT_BASE + WDT_MR, disabled);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_MR), ==, disabled);

    /* The first write locks WDT_MR until the next processor reset. */
    qtest_writel(qts, G45_WDT_BASE + WDT_MR, 0);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_MR), ==, disabled);
    qtest_clock_step(qts, NANOSECONDS_PER_SECOND);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_SR), ==, 0);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_MR), ==,
                    WDT_MR_RESET);
    qtest_quit(qts);
}

static void test_underflow_interrupt(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek "
                                 "-watchdog-action none");
    uint32_t mr = WDT_MR_WDFIEN | WDT_MR_WDD(3) | 3;

    qtest_writel(qts, G45_WDT_BASE + WDT_MR, mr);
    qtest_clock_step(qts, 4 * WDT_TICK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_SR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AIC_BASE + AIC_IPR) & AIC_SYS_IRQ,
                    ==, 0);

    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, AIC_BASE + AIC_IPR) & AIC_SYS_IRQ,
                    ==, AIC_SYS_IRQ);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_SR), ==,
                    WDT_SR_WDUNF);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_SR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AIC_BASE + AIC_IPR) & AIC_SYS_IRQ,
                    ==, 0);
    qtest_quit(qts);
}

static void test_restart_window(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek "
                                 "-watchdog-action none");
    uint32_t mr = WDT_MR_WDFIEN | WDT_MR_WDD(1) | 3;

    qtest_writel(qts, G45_WDT_BASE + WDT_MR, mr);

    /* An incorrect key has no effect; a valid early restart is a fault. */
    qtest_writel(qts, G45_WDT_BASE + WDT_CR, WDT_CR_WDRSTT);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_SR), ==, 0);
    qtest_writel(qts, G45_WDT_BASE + WDT_CR,
                 WDT_CR_KEY | WDT_CR_WDRSTT);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_SR), ==,
                    WDT_SR_WDERR);

    /* At counter value 1 the same command is legal and reloads WDV. */
    qtest_clock_step(qts, 2 * WDT_TICK_NS);
    qtest_writel(qts, G45_WDT_BASE + WDT_CR,
                 WDT_CR_KEY | WDT_CR_WDRSTT);
    qtest_clock_step(qts, 4 * WDT_TICK_NS - 1);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_SR), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_SR), ==,
                    WDT_SR_WDUNF);
    qtest_quit(qts);
}

static void test_watchdog_reset_action(void)
{
    QTestState *qts = qtest_init("-machine sam9m10g45ek "
                                 "-watchdog-action reset");
    QDict *event, *data;

    qtest_writel(qts, G45_WDT_BASE + WDT_MR, WDT_MR_WDRSTEN);
    qtest_clock_step(qts, WDT_TICK_NS);

    event = qtest_qmp_eventwait_ref(qts, "WATCHDOG");
    data = qdict_get_qdict(event, "data");
    g_assert_cmpstr(qdict_get_str(data, "action"), ==, "reset");
    qobject_unref(event);
    qtest_qmp_eventwait(qts, "RESET");
    g_assert_cmphex(qtest_readl(qts, G45_WDT_BASE + WDT_MR), ==,
                    WDT_MR_RESET);
    qtest_quit(qts);
}

static void test_active_timer_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    uint32_t mr = WDT_MR_WDFIEN | WDT_MR_WDD(7) | 7;
    QTestState *src, *dst;
    int64_t destination_clock, target_clock;
    int fd;

    fd = g_file_open_tmp("at91-wdt-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-machine sam9m10g45ek -watchdog-action none -S");
    qtest_writel(src, G45_WDT_BASE + WDT_MR, mr);
    clock_step_to(src, 0, 3 * WDT_TICK_NS);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_initf("-machine sam9m10g45ek -watchdog-action none "
                      "-S -incoming %s", uri);
    g_assert_cmphex(qtest_readl(dst, G45_WDT_BASE + WDT_MR), ==, mr);
    qtest_qmp_assert_success(dst, "{ 'execute': 'cont' }");
    target_clock = 5 * WDT_TICK_NS - 1;
    destination_clock = 0;
    destination_clock = clock_step_to(dst, destination_clock, target_clock);
    g_assert_cmphex(qtest_readl(dst, G45_WDT_BASE + WDT_SR), ==, 0);
    clock_step_to(dst, destination_clock, target_clock + 1);
    g_assert_cmphex(qtest_readl(dst, AIC_BASE + AIC_IPR) & AIC_SYS_IRQ,
                    ==, AIC_SYS_IRQ);
    g_assert_cmphex(qtest_readl(dst, G45_WDT_BASE + WDT_SR), ==,
                    WDT_SR_WDUNF);

    /* The write-once lock is migration state too. */
    qtest_writel(dst, G45_WDT_BASE + WDT_MR, WDT_MR_WDDIS);
    g_assert_cmphex(qtest_readl(dst, G45_WDT_BASE + WDT_MR), ==, mr);
    qtest_quit(dst);
    unlink(state_path);
}

static void test_g35_interrupt_wiring(void)
{
    QTestState *qts = qtest_init("-machine sam9g35ek "
                                 "-watchdog-action none");

    qtest_writel(qts, G35_WDT_BASE + WDT_MR, WDT_MR_WDFIEN);
    qtest_clock_step(qts, WDT_TICK_NS);
    g_assert_cmphex(qtest_readl(qts, AIC_BASE + AIC_IPR) & AIC_SYS_IRQ,
                    ==, AIC_SYS_IRQ);
    g_assert_cmphex(qtest_readl(qts, G35_WDT_BASE + WDT_SR), ==,
                    WDT_SR_WDUNF);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-wdt/g45/mode-write-once-reset",
                   test_mode_write_once_and_reset);
    qtest_add_func("/at91-wdt/g45/underflow-interrupt",
                   test_underflow_interrupt);
    qtest_add_func("/at91-wdt/g45/restart-window", test_restart_window);
    qtest_add_func("/at91-wdt/g45/reset-action", test_watchdog_reset_action);
    qtest_add_func("/at91-wdt/g45/active-timer-migration",
                   test_active_timer_migration);
    qtest_add_func("/at91-wdt/g35/interrupt-wiring",
                   test_g35_interrupt_wiring);

    return g_test_run();
}
