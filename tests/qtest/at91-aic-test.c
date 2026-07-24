/*
 * QTest tests for the AT91 Advanced Interrupt Controller: vectored
 * dispatch, the spurious-vector path and priority-stack nesting.  The
 * qtest MMIO stream stands in for the CPU, which makes the
 * deassert-before-IVR spurious window deterministic - the reason these
 * live here and not in the RT-Thread guest suite.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define AIC_BASE        0xfffff000
#define AIC_SMR(n)      (AIC_BASE + 0x000 + 4 * (n))
#define AIC_SVR(n)      (AIC_BASE + 0x080 + 4 * (n))
#define AIC_IVR         (AIC_BASE + 0x100)
#define AIC_ISR         (AIC_BASE + 0x108)
#define AIC_IPR         (AIC_BASE + 0x10c)
#define AIC_CISR        (AIC_BASE + 0x114)
#define AIC_IECR        (AIC_BASE + 0x120)
#define AIC_IDCR        (AIC_BASE + 0x124)
#define AIC_ICCR        (AIC_BASE + 0x128)
#define AIC_ISCR        (AIC_BASE + 0x12c)
#define AIC_EOICR       (AIC_BASE + 0x130)
#define AIC_SPU         (AIC_BASE + 0x134)

#define CISR_NIRQ       (1u << 1)

#define SMR_EDGE        (1u << 5)   /* SRCTYPE: positive edge */

/* Unused-by-tests peripheral ids with distinct vectors. */
#define SRC_LOW         13
#define SRC_MID         19
#define SRC_HIGH        20

static QTestState *aic_start(void)
{
    return qtest_init("-machine sam9m10g45ek");
}

static void aic_teardown(QTestState *qts, int src)
{
    qtest_writel(qts, AIC_IDCR, 1u << src);
    qtest_writel(qts, AIC_ICCR, 1u << src);
}

static void test_vectored_dispatch(void)
{
    QTestState *qts = aic_start();

    qtest_writel(qts, AIC_SVR(SRC_MID), 0xaa550000 + SRC_MID);
    qtest_writel(qts, AIC_SMR(SRC_MID), SMR_EDGE | 3);
    qtest_writel(qts, AIC_IECR, 1u << SRC_MID);

    qtest_writel(qts, AIC_ISCR, 1u << SRC_MID);
    g_assert_cmphex(qtest_readl(qts, AIC_IPR) & (1u << SRC_MID), ==,
                    1u << SRC_MID);
    g_assert_cmphex(qtest_readl(qts, AIC_CISR) & CISR_NIRQ, ==, CISR_NIRQ);

    /* IVR: vector, ISR source number, edge acked, line dropped. */
    g_assert_cmphex(qtest_readl(qts, AIC_IVR), ==, 0xaa550000 + SRC_MID);
    g_assert_cmphex(qtest_readl(qts, AIC_ISR), ==, SRC_MID);
    g_assert_cmphex(qtest_readl(qts, AIC_IPR) & (1u << SRC_MID), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AIC_CISR) & CISR_NIRQ, ==, 0);

    qtest_writel(qts, AIC_EOICR, 0);
    aic_teardown(qts, SRC_MID);
    qtest_quit(qts);
}

static void test_spurious_vector(void)
{
    QTestState *qts = aic_start();

    qtest_writel(qts, AIC_SPU, 0x5b0b0b0b);
    qtest_writel(qts, AIC_SVR(SRC_MID), 0xaa550000 + SRC_MID);
    qtest_writel(qts, AIC_SMR(SRC_MID), SMR_EDGE | 3);
    qtest_writel(qts, AIC_IECR, 1u << SRC_MID);

    /* Assert, then clear the edge before the (CPU's) IVR read. */
    qtest_writel(qts, AIC_ISCR, 1u << SRC_MID);
    g_assert_cmphex(qtest_readl(qts, AIC_CISR) & CISR_NIRQ, ==, CISR_NIRQ);
    qtest_writel(qts, AIC_ICCR, 1u << SRC_MID);

    g_assert_cmphex(qtest_readl(qts, AIC_IVR), ==, 0x5b0b0b0b);
    g_assert_cmphex(qtest_readl(qts, AIC_ISR), ==, 0);
    /* The handler contract still writes one EOICR per IVR read. */
    qtest_writel(qts, AIC_EOICR, 0);

    /* A real assertion afterwards must dispatch normally. */
    qtest_writel(qts, AIC_ISCR, 1u << SRC_MID);
    g_assert_cmphex(qtest_readl(qts, AIC_IVR), ==, 0xaa550000 + SRC_MID);
    qtest_writel(qts, AIC_EOICR, 0);

    aic_teardown(qts, SRC_MID);
    qtest_quit(qts);
}

static void test_priority_nesting(void)
{
    QTestState *qts = aic_start();

    qtest_writel(qts, AIC_SVR(SRC_LOW), 0xaa550000 + SRC_LOW);
    qtest_writel(qts, AIC_SVR(SRC_MID), 0xaa550000 + SRC_MID);
    qtest_writel(qts, AIC_SVR(SRC_HIGH), 0xaa550000 + SRC_HIGH);
    qtest_writel(qts, AIC_SMR(SRC_LOW), SMR_EDGE | 1);
    qtest_writel(qts, AIC_SMR(SRC_MID), SMR_EDGE | 2);
    qtest_writel(qts, AIC_SMR(SRC_HIGH), SMR_EDGE | 6);
    qtest_writel(qts, AIC_IECR,
                 (1u << SRC_LOW) | (1u << SRC_MID) | (1u << SRC_HIGH));

    /* Take the mid-priority interrupt. */
    qtest_writel(qts, AIC_ISCR, 1u << SRC_MID);
    g_assert_cmphex(qtest_readl(qts, AIC_IVR), ==, 0xaa550000 + SRC_MID);

    /* A higher source nests: nIRQ re-asserts and IVR serves it. */
    qtest_writel(qts, AIC_ISCR, 1u << SRC_HIGH);
    g_assert_cmphex(qtest_readl(qts, AIC_CISR) & CISR_NIRQ, ==, CISR_NIRQ);
    g_assert_cmphex(qtest_readl(qts, AIC_IVR), ==, 0xaa550000 + SRC_HIGH);
    g_assert_cmphex(qtest_readl(qts, AIC_ISR), ==, SRC_HIGH);

    /* A lower source must stay blocked behind the active levels. */
    qtest_writel(qts, AIC_ISCR, 1u << SRC_LOW);
    g_assert_cmphex(qtest_readl(qts, AIC_CISR) & CISR_NIRQ, ==, 0);

    /* Pop the high level: still blocked behind the mid level. */
    qtest_writel(qts, AIC_EOICR, 0);
    g_assert_cmphex(qtest_readl(qts, AIC_CISR) & CISR_NIRQ, ==, 0);

    /* Pop the mid level: the low source finally delivers. */
    qtest_writel(qts, AIC_EOICR, 0);
    g_assert_cmphex(qtest_readl(qts, AIC_CISR) & CISR_NIRQ, ==, CISR_NIRQ);
    g_assert_cmphex(qtest_readl(qts, AIC_IVR), ==, 0xaa550000 + SRC_LOW);
    qtest_writel(qts, AIC_EOICR, 0);

    aic_teardown(qts, SRC_LOW);
    aic_teardown(qts, SRC_MID);
    aic_teardown(qts, SRC_HIGH);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/at91-aic/g45/vectored-dispatch", test_vectored_dispatch);
    qtest_add_func("/at91-aic/g45/spurious-vector", test_spurious_vector);
    qtest_add_func("/at91-aic/g45/priority-nesting", test_priority_nesting);
    return g_test_run();
}
