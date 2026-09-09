/*
 * QTest testcase for the BCM2835 firmware property interface
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

/* raspi3b (BCM2837): mailbox region at 0x7e00b800; MAIL1_WRITE at +0xa0 */
#define MBOX_BASE       0x3f00b800
#define MAIL0_STATUS    (MBOX_BASE + 0x98)
#define MAIL0_CONFIG    (MBOX_BASE + 0x9c)
#define MAIL1_WRITE     (MBOX_BASE + 0xa0)
#define MBOX_CHAN_FB    1
#define MBOX_CHAN_PROP  8
#define ARM_MS_FULL     (1U << 31)
#define ARM_MC_IHAVEDATAIRQEN 1U

#define RASPI3_IC_BASE  0x3f00b200
#define IRQ_PENDING_BASIC 0x00
#define IRQ_ENABLE_BASIC  0x18
#define ARM_MAILBOX_IRQ_BIT (1U << 1)

/*
 * A 16-byte-aligned RAM buffer, readable by the property device via the
 * GPU-bus RAM alias (RAM at 0x0 maps to bus 0x0).
 */
#define BUF             0x100000u

#define GET_GPIO_STATE  0x00030041u

static void write_gpio_request(QTestState *qts)
{
    qtest_writel(qts, BUF + 0x00, 0x28);            /* total buffer size */
    qtest_writel(qts, BUF + 0x04, 0);               /* request code */
    qtest_writel(qts, BUF + 0x08, GET_GPIO_STATE);  /* tag id */
    qtest_writel(qts, BUF + 0x0c, 8);               /* value buffer size */
    qtest_writel(qts, BUF + 0x10, 0);               /* request/response */
    qtest_writel(qts, BUF + 0x14, 128);             /* expander gpio number */
    qtest_writel(qts, BUF + 0x18, 0);               /* state (response) */
    qtest_writel(qts, BUF + 0x1c, 0);               /* end tag */
}

static void submit_gpio_request(QTestState *qts)
{
    write_gpio_request(qts);
    qtest_writel(qts, MAIL1_WRITE, BUF | MBOX_CHAN_PROP);
    g_assert_cmphex(qtest_readl(qts, BUF + 0x04), ==, 0x80000000);
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

/*
 * Regression test: the firmware GPIO expander tags were unimplemented, so the
 * bcm2711 expander driver failed and the SD regulators never probed. Submit a
 * GET_GPIO_STATE tag and check the property interface marks it handled (the
 * tag's response indicator carries a non-zero response length).
 */
static void test_property_gpio_expander(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    submit_gpio_request(qts);

    /* Handled tag: response indicator = present bit + length 8. */
    g_assert_cmphex(qtest_readl(qts, BUF + 0x10), ==, 0x80000008);
    /* Whole-buffer response code set. */
    g_assert_cmphex(qtest_readl(qts, BUF + 0x04), ==, 0x80000000);

    qtest_quit(qts);
}

static void test_property_reset_pending(void)
{
    QTestState *qts = qtest_init("-M raspi3b");
    unsigned int i;

    /*
     * Fill the response FIFO, then leave one response pending in the
     * property child by submitting one more request without reading MAIL0.
     */
    for (i = 0; !(qtest_readl(qts, MAIL0_STATUS) & ARM_MS_FULL); i++) {
        g_assert_cmpuint(i, <, 64);
        submit_gpio_request(qts);
    }

    submit_gpio_request(qts);

    qtest_system_reset(qts);

    write_gpio_request(qts);
    qtest_writel(qts, MAIL1_WRITE, BUF | MBOX_CHAN_PROP);
    g_assert_cmphex(qtest_readl(qts, BUF + 0x04), ==, 0x80000000);

    qtest_quit(qts);
}

static void test_mbox_reset_clears_irq(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    qtest_writel(qts, MAIL0_CONFIG, ARM_MC_IHAVEDATAIRQEN);
    submit_gpio_request(qts);
    qtest_writel(qts, RASPI3_IC_BASE + IRQ_ENABLE_BASIC,
                 ARM_MAILBOX_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, RASPI3_IC_BASE + IRQ_PENDING_BASIC) &
                    ARM_MAILBOX_IRQ_BIT, ==, ARM_MAILBOX_IRQ_BIT);

    qtest_system_reset(qts);

    qtest_writel(qts, RASPI3_IC_BASE + IRQ_ENABLE_BASIC,
                 ARM_MAILBOX_IRQ_BIT);
    g_assert_cmphex(qtest_readl(qts, RASPI3_IC_BASE + IRQ_PENDING_BASIC) &
                    ARM_MAILBOX_IRQ_BIT, ==, 0);

    qtest_quit(qts);
}

static void test_fb_reset_clears_irq(void)
{
    QTestState *qts = qtest_init("-M raspi3b");

    qtest_irq_intercept_out_named(qts, "/machine/soc/peripherals/fb",
                                  "sysbus-irq");
    qtest_writel(qts, BUF + 0x00, 640);
    qtest_writel(qts, BUF + 0x04, 480);
    qtest_writel(qts, BUF + 0x08, 640);
    qtest_writel(qts, BUF + 0x0c, 480);
    qtest_writel(qts, BUF + 0x14, 32);
    qtest_writel(qts, BUF + 0x18, 0);
    qtest_writel(qts, BUF + 0x1c, 0);
    qtest_writel(qts, MAIL1_WRITE, BUF | MBOX_CHAN_FB);
    g_assert_true(qtest_get_irq(qts, 0));

    qtest_system_reset(qts);
    g_assert_false(qtest_get_irq(qts, 0));

    qtest_quit(qts);
}

static void test_mbox_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-mbox-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b -S");
    qtest_writel(src, MAIL0_CONFIG, ARM_MC_IHAVEDATAIRQEN);
    submit_gpio_request(src);

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    /* The mailbox device must restore this output after loading state. */
    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/mbox",
                                  "sysbus-irq");
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_true(qtest_get_irq(dst, 0));
    qtest_quit(dst);

    unlink(state_path);
}

static void test_fb_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-fb-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b -S");
    qtest_irq_intercept_out_named(src, "/machine/soc/peripherals/fb",
                                  "sysbus-irq");
    qtest_writel(src, BUF + 0x00, 640);
    qtest_writel(src, BUF + 0x04, 480);
    qtest_writel(src, BUF + 0x08, 640);
    qtest_writel(src, BUF + 0x0c, 480);
    qtest_writel(src, BUF + 0x14, 32);
    qtest_writel(src, BUF + 0x18, 0);
    qtest_writel(src, BUF + 0x1c, 0);
    qtest_writel(src, MAIL1_WRITE, BUF | MBOX_CHAN_FB);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/fb",
                                  "sysbus-irq");
    qtest_qmp_assert_success(dst,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    wait_for_migration_complete(dst);
    g_assert_true(qtest_get_irq(dst, 0));
    qtest_quit(dst);

    unlink(state_path);
}

static void test_property_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-property-migration-XXXXXX", &state_path,
                         NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b -S");
    qtest_irq_intercept_out_named(src, "/machine/soc/peripherals/property",
                                  "sysbus-irq");
    submit_gpio_request(src);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst, "/machine/soc/peripherals/property",
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
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2835/bcm2835-property/gpio_expander",
                   test_property_gpio_expander);
    qtest_add_func("/bcm2835/bcm2835-property/reset_pending",
                   test_property_reset_pending);
    qtest_add_func("/bcm2835/bcm2835-mbox/reset_clears_irq",
                   test_mbox_reset_clears_irq);
    qtest_add_func("/bcm2835/bcm2835-fb/reset_clears_irq",
                   test_fb_reset_clears_irq);
    qtest_add_func("/bcm2835/bcm2835-mbox/migration_irq",
                   test_mbox_irq_migration);
    qtest_add_func("/bcm2835/bcm2835-fb/migration_irq",
                   test_fb_irq_migration);
    qtest_add_func("/bcm2835/bcm2835-property/migration_irq",
                   test_property_irq_migration);
    return g_test_run();
}
