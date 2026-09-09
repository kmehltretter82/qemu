/*
 * QTest testcase for Broadcom Serial Controller (BSC)
 *
 * Copyright (c) 2024 Rayhan Faizel <rayhan.faizel@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#include "hw/i2c/bcm2835_i2c.h"
#include "hw/sensor/tmp105_regs.h"

static const uint32_t bsc_base_addrs[] = {
    0x3f205000,                         /* I2C0 */
    0x3f804000,                         /* I2C1 */
    0x3f805000,                         /* I2C2 */
};

#define IC_BASE          0x3f00b200
#define IRQ_PENDING_2    0x08
#define IRQ_ENABLE_2     0x14
#define I2C_IRQ_BIT      (1U << (53 - 32))

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

static void bcm2835_i2c_init_transfer(uint32_t base_addr, bool read)
{
    /* read flag is bit 0 so we can write it directly */
    int interrupt = read ? BCM2835_I2C_C_INTR : BCM2835_I2C_C_INTT;

    writel(base_addr + BCM2835_I2C_C,
           BCM2835_I2C_C_I2CEN | BCM2835_I2C_C_INTD |
           BCM2835_I2C_C_ST | BCM2835_I2C_C_CLEAR | interrupt | read);
}

static void test_i2c_read_write(gconstpointer data)
{
    uint32_t i2cdata;
    intptr_t index = (intptr_t) data;
    uint32_t base_addr = bsc_base_addrs[index];

    /* Write to TMP105 register */
    writel(base_addr + BCM2835_I2C_A, 0x50);
    writel(base_addr + BCM2835_I2C_DLEN, 3);

    bcm2835_i2c_init_transfer(base_addr, 0);

    writel(base_addr + BCM2835_I2C_FIFO, TMP105_REG_T_HIGH);
    writel(base_addr + BCM2835_I2C_FIFO, 0xde);
    writel(base_addr + BCM2835_I2C_FIFO, 0xad);

    /* Clear flags */
    writel(base_addr + BCM2835_I2C_S, BCM2835_I2C_S_DONE | BCM2835_I2C_S_ERR |
                                      BCM2835_I2C_S_CLKT);

    /* Read from TMP105 register */
    writel(base_addr + BCM2835_I2C_A, 0x50);
    writel(base_addr + BCM2835_I2C_DLEN, 1);

    bcm2835_i2c_init_transfer(base_addr, 0);

    writel(base_addr + BCM2835_I2C_FIFO, TMP105_REG_T_HIGH);

    writel(base_addr + BCM2835_I2C_DLEN, 2);
    bcm2835_i2c_init_transfer(base_addr, 1);

    i2cdata = readl(base_addr + BCM2835_I2C_FIFO);
    g_assert_cmpint(i2cdata, ==, 0xde);

    i2cdata = readl(base_addr + BCM2835_I2C_FIFO);
    g_assert_cmpint(i2cdata, ==, 0xa0);

    /* Clear flags */
    writel(base_addr + BCM2835_I2C_S, BCM2835_I2C_S_DONE | BCM2835_I2C_S_ERR |
                                      BCM2835_I2C_S_CLKT);

}

/*
 * Regression test for the DLEN-underflow bus wedge: writing DLEN = 0 while a
 * transfer is active and then touching the FIFO used to decrement DLEN to
 * 0xFFFFFFFF, so the completion test never fired and the bus stayed in TA
 * forever. After the fix the transfer completes and TA clears.
 */
static void test_i2c_dlen_underflow(gconstpointer data)
{
    intptr_t index = (intptr_t) data;
    uint32_t base_addr = bsc_base_addrs[index];
    uint32_t status;

    writel(base_addr + BCM2835_I2C_A, 0x50);
    writel(base_addr + BCM2835_I2C_DLEN, 3);
    bcm2835_i2c_init_transfer(base_addr, 0);

    g_assert_cmpint(readl(base_addr + BCM2835_I2C_S) & BCM2835_I2C_S_TA, ==,
                    BCM2835_I2C_S_TA);

    writel(base_addr + BCM2835_I2C_DLEN, 0);
    writel(base_addr + BCM2835_I2C_FIFO, TMP105_REG_T_HIGH);

    status = readl(base_addr + BCM2835_I2C_S);
    g_assert_cmpint(status & BCM2835_I2C_S_TA, ==, 0);
    g_assert_cmpint(status & BCM2835_I2C_S_DONE, ==, BCM2835_I2C_S_DONE);

    writel(base_addr + BCM2835_I2C_S, BCM2835_I2C_S_DONE | BCM2835_I2C_S_ERR |
                                      BCM2835_I2C_S_CLKT);
}

/*
 * Regression test for the spurious start on I2CEN: only ST (start) may begin a
 * transfer, not merely enabling the controller. Writing C with I2CEN but not ST
 * must leave the controller idle (TA clear).
 */
static void test_i2c_i2cen_no_start(gconstpointer data)
{
    intptr_t index = (intptr_t) data;
    uint32_t base_addr = bsc_base_addrs[index];

    writel(base_addr + BCM2835_I2C_S, BCM2835_I2C_S_DONE | BCM2835_I2C_S_ERR |
                                      BCM2835_I2C_S_CLKT);
    writel(base_addr + BCM2835_I2C_A, 0x50);
    writel(base_addr + BCM2835_I2C_DLEN, 3);

    /* Enable the controller without ST: no transfer must start. */
    writel(base_addr + BCM2835_I2C_C, BCM2835_I2C_C_I2CEN);

    g_assert_cmpint(readl(base_addr + BCM2835_I2C_S) & BCM2835_I2C_S_TA, ==, 0);
}

static void test_i2c_reset_clears_state(void)
{
    uint32_t base_addr = bsc_base_addrs[0];

    writel(base_addr + BCM2835_I2C_A, 0x50);
    writel(base_addr + BCM2835_I2C_DLEN, 1);
    writel(base_addr + BCM2835_I2C_C,
           BCM2835_I2C_C_I2CEN | BCM2835_I2C_C_INTT |
           BCM2835_I2C_C_INTD | BCM2835_I2C_C_ST);
    g_assert_cmphex(readl(base_addr + BCM2835_I2C_S) & BCM2835_I2C_S_TXW,
                    ==, BCM2835_I2C_S_TXW);
    writel(base_addr + BCM2835_I2C_FIFO, TMP105_REG_T_HIGH);
    g_assert_cmphex(readl(base_addr + BCM2835_I2C_S) & BCM2835_I2C_S_DONE,
                    ==, BCM2835_I2C_S_DONE);

    writel(IC_BASE + IRQ_ENABLE_2, I2C_IRQ_BIT);
    g_assert_cmphex(readl(IC_BASE + IRQ_PENDING_2) & I2C_IRQ_BIT,
                    ==, I2C_IRQ_BIT);

    qtest_system_reset(global_qtest);

    writel(IC_BASE + IRQ_ENABLE_2, I2C_IRQ_BIT);
    g_assert_cmphex(readl(IC_BASE + IRQ_PENDING_2) & I2C_IRQ_BIT, ==, 0);

    /* Reset must discard the DLEN value saved for a later DONE acknowledge. */
    writel(base_addr + BCM2835_I2C_A, 0x50);
    writel(base_addr + BCM2835_I2C_C,
           BCM2835_I2C_C_I2CEN | BCM2835_I2C_C_ST);
    g_assert_cmphex(readl(base_addr + BCM2835_I2C_S) & BCM2835_I2C_S_DONE,
                    ==, BCM2835_I2C_S_DONE);
    writel(base_addr + BCM2835_I2C_S, BCM2835_I2C_S_DONE);
    g_assert_cmphex(readl(base_addr + BCM2835_I2C_DLEN), ==, 0);
}

static void test_i2c_irq_migration(void)
{
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    const uint32_t base_addr = bsc_base_addrs[0];
    QTestState *src, *dst;
    int fd;

    fd = g_file_open_tmp("bcm2835-i2c-migration-XXXXXX", &state_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);

    src = qtest_init("-M raspi3b -S");
    qtest_irq_intercept_out_named(src,
                                  "/machine/soc/peripherals/bcm2835-i2c0",
                                  "sysbus-irq");
    qtest_writel(src, base_addr + BCM2835_I2C_C,
                 BCM2835_I2C_C_I2CEN | BCM2835_I2C_C_INTD |
                 BCM2835_I2C_C_ST);
    g_assert_true(qtest_get_irq(src, 0));

    uri = g_strdup_printf("file:%s", state_path);
    qtest_qmp_assert_success(src,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(src);
    qtest_quit(src);

    dst = qtest_init("-M raspi3b -S -incoming defer");
    qtest_irq_intercept_out_named(dst,
                                  "/machine/soc/peripherals/bcm2835-i2c0",
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
    int i;

    g_test_init(&argc, &argv, NULL);

    for (i = 0; i < 3; i++) {
        g_autofree char *test_name =
        g_strdup_printf("/bcm2835/bcm2835-i2c%d/read_write", i);
        qtest_add_data_func(test_name, (void *)(intptr_t) i,
                            test_i2c_read_write);
    }

    for (i = 0; i < 3; i++) {
        g_autofree char *test_name =
        g_strdup_printf("/bcm2835/bcm2835-i2c%d/dlen_underflow", i);
        qtest_add_data_func(test_name, (void *)(intptr_t) i,
                            test_i2c_dlen_underflow);
    }

    for (i = 0; i < 3; i++) {
        g_autofree char *test_name =
        g_strdup_printf("/bcm2835/bcm2835-i2c%d/i2cen_no_start", i);
        qtest_add_data_func(test_name, (void *)(intptr_t) i,
                            test_i2c_i2cen_no_start);
    }

    qtest_add_func("/bcm2835/bcm2835-i2c/reset_clears_state",
                   test_i2c_reset_clears_state);
    qtest_add_func("/bcm2835/bcm2835-i2c/migration_irq",
                   test_i2c_irq_migration);

    /* Run I2C tests with TMP105 slaves on all three buses */
    qtest_start("-M raspi3b "
                "-device tmp105,address=0x50,bus=i2c-bus.0 "
                "-device tmp105,address=0x50,bus=i2c-bus.1 "
                "-device tmp105,address=0x50,bus=i2c-bus.2");
    ret = g_test_run();
    qtest_end();

    return ret;
}
