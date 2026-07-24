/*
 * AT91SAM9G45 control-peripheral tests for RT-Thread: RTC, RTT, GPBR.
 *
 * R4 track of the plan: silicon-interface tests (no QEMU escape hatch) so
 * the same binary can differentiate emulator and BSP defects on a real
 * SAM9M10-G45-EK.
 *
 * The RTC, RTT, PIT and WDT share the wired-OR system interrupt (AIC
 * source 1) with the RT-Thread tick.  Cases that make the RTC/RTT assert
 * that line mask AIC source 1 and temporarily disable the PIT interrupt
 * enable, prove pendingness via AIC_IPR and the device SR, then restore
 * the OS tick.  While masked the scheduler tick is frozen, so all waiting
 * is bounded by the RTT free-running value register (computed from the
 * slow clock, independent of interrupts) instead of rt_tick_get().
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <rtthread.h>
#include <rthw.h>

#include "g45test.h"

#define G45_RTT_BASE    0xfffffd20U
#define G45_GPBR_BASE   0xfffffd60U
#define G45_RTC_BASE    0xfffffdb0U
#define G45_AIC_BASE    0xfffff000U
#define G45_PIT_BASE    0xfffffd30U
#define G45_TRNG_BASE   0xfffcc000U
#define G45_PMC_PCER    0xfffffc10U
#define G45_TRNG_PID    6U

#define RTC_CR          0x00U
#define RTC_CR_UPDTIM   (1U << 0)
#define RTC_CR_UPDCAL   (1U << 1)
#define RTC_MR          0x04U
#define RTC_TIMR        0x08U
#define RTC_CALR        0x0cU
#define RTC_TIMALR      0x10U
#define RTC_CALALR      0x14U
#define RTC_SR          0x18U
#define RTC_SR_ACKUPD   (1U << 0)
#define RTC_SR_ALARM    (1U << 1)
#define RTC_SR_ALL      0x1fU
#define RTC_SCCR        0x1cU
#define RTC_IER         0x20U
#define RTC_IDR         0x24U
#define RTC_IMR         0x28U
#define RTC_ALR_SECEN   (1U << 7)

#define RTT_MR          0x00U
#define RTT_MR_RTPRES   0xffffU
#define RTT_MR_ALMIEN   (1U << 16)
#define RTT_MR_RTTINCIEN (1U << 17)
#define RTT_MR_RTTRST   (1U << 18)
#define RTT_MR_RESET    0x00008000U
#define RTT_AR          0x04U
#define RTT_VR          0x08U
#define RTT_SR          0x0cU
#define RTT_SR_ALMS     (1U << 0)
#define RTT_SR_RTTINC   (1U << 1)

#define AIC_IPR         0x10cU
#define AIC_IMR         0x110U
#define AIC_IECR        0x120U
#define AIC_IDCR        0x124U
#define AIC_ICCR        0x128U
#define AIC_SYS         (1U << 1)

#define PIT_MR          0x00U
#define PIT_MR_PITIEN   (1U << 25)

#define G45_PWM_BASE    0xfffb8000U
#define G45_PWM_PID     19U
#define G45_ADC_BASE    0xfffb0000U
#define G45_ADC_PID     20U

#define ADC_CR          0x00U
#define ADC_CR_SWRST    (1U << 0)
#define ADC_CR_START    (1U << 1)
#define ADC_MR          0x04U
#define ADC_CHER        0x10U
#define ADC_CHDR        0x14U
#define ADC_CHSR        0x18U
#define ADC_SR          0x1cU
#define ADC_LCDR        0x20U
#define ADC_IER         0x24U
#define ADC_IDR         0x28U
#define ADC_IMR         0x2cU
#define ADC_CDR(n)      (0x30U + 4U * (n))
#define ADC_SR_EOC(n)   (1U << (n))
#define ADC_SR_OVRE(n)  (1U << ((n) + 8U))
#define ADC_SR_DRDY     (1U << 16)
#define ADC_SR_GOVRE    (1U << 17)

#define PWM_MR          0x00U
#define PWM_ENA         0x04U
#define PWM_DIS         0x08U
#define PWM_SR          0x0cU
#define PWM_IER         0x10U
#define PWM_IDR         0x14U
#define PWM_IMR         0x18U
#define PWM_ISR         0x1cU
#define PWM_CH(n, r)    (0x200U + 0x20U * (n) + (r))
#define PWM_CMR         0x00U
#define PWM_CDTY        0x04U
#define PWM_CPRD        0x08U
#define PWM_CUPD        0x10U
#define PWM_CMR_UPD_CDTY (1U << 10)

#define TRNG_CR         0x00U
#define TRNG_MR         0x04U
#define TRNG_ISR        0x1cU
#define TRNG_ODATA      0x50U
#define TRNG_KEY        0x524e4700U
#define TRNG_CR_ENABLE  (1U << 0)
#define TRNG_ISR_DATRDY (1U << 0)

/* RTPRES for a ~1 ms RTT unit: 32 slow-clock cycles / 32768 Hz. */
#define RTT_FAST_PRES   0x20U

static volatile rt_uint32_t *g45_reg(rt_uint32_t base, rt_uint32_t offset)
{
    return (volatile rt_uint32_t *)(base + offset);
}

static rt_uint32_t g45_bcd(rt_uint32_t value)
{
    return ((value / 10U) << 4) | (value % 10U);
}

/* Free-running ~1 ms clock from the RTT (no interrupts involved). */
static void g45ctrl_clock_start(void)
{
    *g45_reg(G45_RTT_BASE, RTT_MR) = RTT_FAST_PRES | RTT_MR_RTTRST;
}

static rt_uint32_t g45ctrl_clock_ms(void)
{
    return *g45_reg(G45_RTT_BASE, RTT_VR);
}

static void g45ctrl_clock_stop(void)
{
    *g45_reg(G45_RTT_BASE, RTT_AR) = 0xffffffffU;
    *g45_reg(G45_RTT_BASE, RTT_MR) = RTT_MR_RESET;
}

/* Wait until *reg == want or budget_ms elapses; returns the last value. */
static rt_uint32_t g45ctrl_wait_value(volatile rt_uint32_t *reg,
                                      rt_uint32_t want,
                                      rt_uint32_t budget_ms)
{
    rt_uint32_t start = g45ctrl_clock_ms();
    rt_uint32_t v;

    for (;;) {
        v = *reg;
        if (v == want || g45ctrl_clock_ms() - start > budget_ms) {
            return v;
        }
    }
}

/* Wait until cond_reg & mask (!= 0 when set is true) or budget_ms elapses. */
static rt_bool_t g45ctrl_wait(volatile rt_uint32_t *cond_reg,
                              rt_uint32_t mask, rt_bool_t set,
                              rt_uint32_t budget_ms)
{
    rt_uint32_t start = g45ctrl_clock_ms();

    for (;;) {
        rt_uint32_t v = *cond_reg & mask;

        if (set ? v != 0U : v == 0U) {
            return RT_TRUE;
        }
        if (g45ctrl_clock_ms() - start > budget_ms) {
            return RT_FALSE;
        }
    }
}

/*
 * Set the RTC through the real update protocol: request update, wait for
 * ACKUPD, write both latches, release.  Returns the ACKUPD observation.
 */
static rt_bool_t g45ctrl_rtc_set(rt_uint32_t timr, rt_uint32_t calr)
{
    rt_bool_t acked;

    *g45_reg(G45_RTC_BASE, RTC_CR) = RTC_CR_UPDTIM | RTC_CR_UPDCAL;
    acked = g45ctrl_wait(g45_reg(G45_RTC_BASE, RTC_SR), RTC_SR_ACKUPD,
                         RT_TRUE, 2000U);
    *g45_reg(G45_RTC_BASE, RTC_SCCR) = RTC_SR_ACKUPD;
    *g45_reg(G45_RTC_BASE, RTC_TIMR) = timr;
    *g45_reg(G45_RTC_BASE, RTC_CALR) = calr;
    *g45_reg(G45_RTC_BASE, RTC_CR) = 0;
    return acked;
}

/* Coherent time read: loop until two consecutive TIMR/CALR pairs agree. */
static rt_uint32_t g45ctrl_rtc_read(rt_uint32_t *calr_out,
                                    struct g45test_result *result,
                                    rt_uint32_t offset)
{
    rt_uint32_t timr_a, calr_a, timr_b, calr_b;
    rt_uint32_t spins = 0;

    do {
        timr_a = *g45_reg(G45_RTC_BASE, RTC_TIMR);
        calr_a = *g45_reg(G45_RTC_BASE, RTC_CALR);
        timr_b = *g45_reg(G45_RTC_BASE, RTC_TIMR);
        calr_b = *g45_reg(G45_RTC_BASE, RTC_CALR);
        spins++;
    } while ((timr_a != timr_b || calr_a != calr_b) && spins < 16U);
    g45test_check(result, spins < 16U, 16U, spins, offset);
    *calr_out = calr_b;
    return timr_b;
}

void g45test_r4_rtc_update_protocol(struct g45test_result *result)
{
    rt_uint32_t timr, calr, frozen;

    g45ctrl_clock_start();
    *g45_reg(G45_RTC_BASE, RTC_SCCR) = RTC_SR_ALL;

    /* Entering update mode freezes the counter and raises ACKUPD. */
    *g45_reg(G45_RTC_BASE, RTC_CR) = RTC_CR_UPDTIM | RTC_CR_UPDCAL;
    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_RTC_BASE, RTC_SR), RTC_SR_ACKUPD,
                               RT_TRUE, 2000U),
                  RTC_SR_ACKUPD, *g45_reg(G45_RTC_BASE, RTC_SR), 0);
    g45test_check(result,
                  *g45_reg(G45_RTC_BASE, RTC_CR) ==
                  (RTC_CR_UPDTIM | RTC_CR_UPDCAL),
                  RTC_CR_UPDTIM | RTC_CR_UPDCAL,
                  *g45_reg(G45_RTC_BASE, RTC_CR), 1);
    *g45_reg(G45_RTC_BASE, RTC_SCCR) = RTC_SR_ACKUPD;
    g45test_check(result,
                  (*g45_reg(G45_RTC_BASE, RTC_SR) & RTC_SR_ACKUPD) == 0U,
                  0, *g45_reg(G45_RTC_BASE, RTC_SR), 2);

    frozen = *g45_reg(G45_RTC_BASE, RTC_TIMR);
    {
        rt_uint32_t start = g45ctrl_clock_ms();

        while (g45ctrl_clock_ms() - start < 1200U) {
        }
    }
    g45test_check(result, *g45_reg(G45_RTC_BASE, RTC_TIMR) == frozen,
                  frozen, *g45_reg(G45_RTC_BASE, RTC_TIMR), 3);

    /* Set 2026-07-24 22:30:45 and verify it starts ticking again. */
    *g45_reg(G45_RTC_BASE, RTC_TIMR) =
        g45_bcd(45) | (g45_bcd(30) << 8) | (g45_bcd(22) << 16);
    *g45_reg(G45_RTC_BASE, RTC_CALR) =
        g45_bcd(20) | (g45_bcd(26) << 8) | (g45_bcd(7) << 16) |
        (g45_bcd(6) << 21) | (g45_bcd(24) << 24);
    *g45_reg(G45_RTC_BASE, RTC_CR) = 0;

    timr = g45ctrl_rtc_read(&calr, result, 4);
    g45test_check(result, (timr >> 16) == g45_bcd(22),
                  g45_bcd(22), timr >> 16, 5);
    g45test_check(result, (calr >> 24) == g45_bcd(24),
                  g45_bcd(24), calr >> 24, 6);
    {
        rt_uint32_t start = g45ctrl_clock_ms();

        while (g45ctrl_clock_ms() - start < 1200U) {
        }
    }
    g45test_check(result, *g45_reg(G45_RTC_BASE, RTC_TIMR) != timr,
                  timr, *g45_reg(G45_RTC_BASE, RTC_TIMR), 7);

    rt_kprintf("G45TEST DATA case=r4.rtc-update-protocol timr=0x%08x "
               "calr=0x%08x\n", timr, calr);
    g45ctrl_clock_stop();
}

static void g45ctrl_rollover(struct g45test_result *result,
                             rt_uint32_t year_hi, rt_uint32_t year_lo,
                             rt_uint32_t exp_month, rt_uint32_t exp_mday,
                             rt_uint32_t offset)
{
    rt_uint32_t timr, calr;
    rt_uint32_t start;

    /* 23:59:58 on Feb 28 of the given year. */
    g45ctrl_rtc_set(g45_bcd(58) | (g45_bcd(59) << 8) | (g45_bcd(23) << 16),
                    g45_bcd(year_hi) | (g45_bcd(year_lo) << 8) |
                    (g45_bcd(2) << 16) | (1U << 21) | (g45_bcd(28) << 24));

    start = g45ctrl_clock_ms();
    do {
        timr = g45ctrl_rtc_read(&calr, result, offset);
    } while ((calr >> 24) == g45_bcd(28) &&
             g45ctrl_clock_ms() - start < 4000U);

    g45test_check(result, (calr >> 16 & 0x1fU) == g45_bcd(exp_month),
                  g45_bcd(exp_month), calr >> 16 & 0x1fU, offset + 1U);
    g45test_check(result, (calr >> 24) == g45_bcd(exp_mday),
                  g45_bcd(exp_mday), calr >> 24, offset + 2U);
    g45test_check(result, (timr >> 16) == 0U, 0, timr >> 16, offset + 3U);
    rt_kprintf("G45TEST DATA case=r4.rtc-leap-rollover year=%u%u "
               "calr=0x%08x\n", year_hi, year_lo, calr);
}

void g45test_r4_rtc_leap_rollover(struct g45test_result *result)
{
    g45ctrl_clock_start();
    /* 2028 is a leap year: Feb 28 rolls to Feb 29 ... */
    g45ctrl_rollover(result, 20, 28, 2, 29, 0);
    /* ... 2027 is not: Feb 28 rolls straight to Mar 1. */
    g45ctrl_rollover(result, 20, 27, 3, 1, 4);
    g45ctrl_clock_stop();
}

void g45test_r4_rtc_alarm_irq(struct g45test_result *result)
{
    rt_uint32_t pit_mr;
    rt_uint32_t sr;

    g45ctrl_clock_start();

    /*
     * Isolate the shared system interrupt: mask it at the AIC and park
     * the PIT interrupt enable so a pending line can only mean RTC.
     */
    *g45_reg(G45_AIC_BASE, AIC_IDCR) = AIC_SYS;
    pit_mr = *g45_reg(G45_PIT_BASE, PIT_MR);
    *g45_reg(G45_PIT_BASE, PIT_MR) = pit_mr & ~PIT_MR_PITIEN;
    *g45_reg(G45_AIC_BASE, AIC_ICCR) = AIC_SYS;
    *g45_reg(G45_RTC_BASE, RTC_SCCR) = RTC_SR_ALL;

    /* 10:00:00, alarm on seconds == 03. */
    g45ctrl_rtc_set(g45_bcd(0) | (g45_bcd(0) << 8) | (g45_bcd(10) << 16),
                    g45_bcd(20) | (g45_bcd(26) << 8) | (g45_bcd(7) << 16) |
                    (1U << 21) | (g45_bcd(24) << 24));
    *g45_reg(G45_RTC_BASE, RTC_TIMALR) = RTC_ALR_SECEN | g45_bcd(3);
    *g45_reg(G45_RTC_BASE, RTC_IER) = RTC_SR_ALARM;
    g45test_check(result, *g45_reg(G45_RTC_BASE, RTC_IMR) == RTC_SR_ALARM,
                  RTC_SR_ALARM, *g45_reg(G45_RTC_BASE, RTC_IMR), 0);

    /* The 1 Hz tick whose time matches must latch ALARM and assert. */
    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_RTC_BASE, RTC_SR), RTC_SR_ALARM,
                               RT_TRUE, 6000U),
                  RTC_SR_ALARM, *g45_reg(G45_RTC_BASE, RTC_SR), 1);
    g45test_check(result,
                  (*g45_reg(G45_AIC_BASE, AIC_IPR) & AIC_SYS) != 0U,
                  AIC_SYS, *g45_reg(G45_AIC_BASE, AIC_IPR) & AIC_SYS, 2);

    /* Clearing the source must drop the (level) system line. */
    sr = *g45_reg(G45_RTC_BASE, RTC_SR);
    *g45_reg(G45_RTC_BASE, RTC_SCCR) = RTC_SR_ALL;
    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_AIC_BASE, AIC_IPR), AIC_SYS,
                               RT_FALSE, 100U),
                  0, *g45_reg(G45_AIC_BASE, AIC_IPR) & AIC_SYS, 3);

    /* Teardown: RTC quiet again, OS tick back. */
    *g45_reg(G45_RTC_BASE, RTC_IDR) = RTC_SR_ALL;
    *g45_reg(G45_RTC_BASE, RTC_TIMALR) = 0;
    *g45_reg(G45_AIC_BASE, AIC_ICCR) = AIC_SYS;
    *g45_reg(G45_PIT_BASE, PIT_MR) = pit_mr;
    *g45_reg(G45_AIC_BASE, AIC_IECR) = AIC_SYS;

    rt_kprintf("G45TEST DATA case=r4.rtc-alarm-irq sr=0x%08x\n", sr);
    g45ctrl_clock_stop();
}

void g45test_r4_rtt_increment_alarm(struct g45test_result *result)
{
    rt_uint32_t pit_mr;
    rt_uint32_t sr, v1, v2;

    *g45_reg(G45_AIC_BASE, AIC_IDCR) = AIC_SYS;
    pit_mr = *g45_reg(G45_PIT_BASE, PIT_MR);
    *g45_reg(G45_PIT_BASE, PIT_MR) = pit_mr & ~PIT_MR_PITIEN;
    *g45_reg(G45_AIC_BASE, AIC_ICCR) = AIC_SYS;
    (void)*g45_reg(G45_RTT_BASE, RTT_SR);   /* drain stale status */

    /* ~1 ms increments with the increment interrupt enabled. */
    *g45_reg(G45_RTT_BASE, RTT_MR) =
        RTT_FAST_PRES | RTT_MR_RTTINCIEN | RTT_MR_RTTRST;

    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_AIC_BASE, AIC_IPR), AIC_SYS,
                               RT_TRUE, 100U),
                  AIC_SYS, *g45_reg(G45_AIC_BASE, AIC_IPR) & AIC_SYS, 0);

    /*
     * Freeze latching (interrupt enables off) before proving the
     * clear-on-read contract, so a tick cannot re-latch status between
     * the two reads under TCG scheduling stalls.
     */
    *g45_reg(G45_RTT_BASE, RTT_MR) = RTT_FAST_PRES;
    sr = *g45_reg(G45_RTT_BASE, RTT_SR);
    g45test_check(result, (sr & RTT_SR_RTTINC) != 0U,
                  RTT_SR_RTTINC, sr, 1);
    g45test_check(result, *g45_reg(G45_RTT_BASE, RTT_SR) == 0U,
                  0, *g45_reg(G45_RTT_BASE, RTT_SR), 2);
    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_AIC_BASE, AIC_IPR), AIC_SYS,
                               RT_FALSE, 100U),
                  0, *g45_reg(G45_AIC_BASE, AIC_IPR) & AIC_SYS, 3);

    /* The free-running value is monotonic. */
    v1 = *g45_reg(G45_RTT_BASE, RTT_VR);
    g45ctrl_wait(g45_reg(G45_RTT_BASE, RTT_VR), 0xffffffffU, RT_FALSE, 5U);
    v2 = *g45_reg(G45_RTT_BASE, RTT_VR);
    g45test_check(result, v2 >= v1, v1, v2, 4);

    /* Alarm three increments out. */
    *g45_reg(G45_RTT_BASE, RTT_AR) = v2 + 3U;
    *g45_reg(G45_RTT_BASE, RTT_MR) =
        RTT_FAST_PRES | RTT_MR_RTTINCIEN | RTT_MR_ALMIEN;
    {
        rt_uint32_t alarmed = 0;

        for (;;) {
            sr = *g45_reg(G45_RTT_BASE, RTT_SR);
            if (sr & RTT_SR_ALMS) {
                alarmed = 1;
                break;
            }
            if (*g45_reg(G45_RTT_BASE, RTT_VR) > v2 + 64U) {
                break;
            }
        }
        g45test_check(result, alarmed == 1U, 1, alarmed, 5);
        g45test_check(result, *g45_reg(G45_RTT_BASE, RTT_VR) + 1U >= v2 + 3U,
                      v2 + 3U, *g45_reg(G45_RTT_BASE, RTT_VR), 6);
    }

    /* Teardown. */
    *g45_reg(G45_RTT_BASE, RTT_AR) = 0xffffffffU;
    *g45_reg(G45_RTT_BASE, RTT_MR) = RTT_MR_RESET;
    (void)*g45_reg(G45_RTT_BASE, RTT_SR);
    *g45_reg(G45_AIC_BASE, AIC_ICCR) = AIC_SYS;
    *g45_reg(G45_PIT_BASE, PIT_MR) = pit_mr;
    *g45_reg(G45_AIC_BASE, AIC_IECR) = AIC_SYS;

    rt_kprintf("G45TEST DATA case=r4.rtt-increment-alarm sr=0x%08x "
               "vr=%u\n", sr, v2);
}

/*
 * PWM checks are written in board-portable shapes: where real silicon
 * defers an effect to the next period boundary (CUPD, first period
 * event) the check polls with a time budget, so it converges both on
 * the immediate-effect model and on hardware.  Deliberately unchecked
 * model gaps (see working notes): CCNT always reads 0, ISR is not
 * clear-on-read, and the AIC line is never asserted.
 */
void g45test_r4_pwm(struct g45test_result *result)
{
    rt_uint32_t v;

    g45ctrl_clock_start();
    *(volatile rt_uint32_t *)G45_PMC_PCER = 1U << G45_PWM_PID;

    *g45_reg(G45_PWM_BASE, PWM_DIS) = 0xfU;
    *g45_reg(G45_PWM_BASE, PWM_IDR) = 0xfU;

    /* Valid-field MR round trip: DIVA = DIVB = 1, PREA = PREB = 0. */
    *g45_reg(G45_PWM_BASE, PWM_MR) = 0x00010001U;
    g45test_check(result, *g45_reg(G45_PWM_BASE, PWM_MR) == 0x00010001U,
                  0x00010001U, *g45_reg(G45_PWM_BASE, PWM_MR), 0);

    /* Independent per-channel registers. */
    *g45_reg(G45_PWM_BASE, PWM_CH(0, PWM_CMR)) = PWM_CMR_UPD_CDTY;
    *g45_reg(G45_PWM_BASE, PWM_CH(0, PWM_CPRD)) = 100U;
    *g45_reg(G45_PWM_BASE, PWM_CH(0, PWM_CDTY)) = 25U;
    *g45_reg(G45_PWM_BASE, PWM_CH(1, PWM_CMR)) = 0U;
    *g45_reg(G45_PWM_BASE, PWM_CH(1, PWM_CPRD)) = 400U;
    *g45_reg(G45_PWM_BASE, PWM_CH(1, PWM_CDTY)) = 300U;
    g45test_check(result,
                  *g45_reg(G45_PWM_BASE, PWM_CH(0, PWM_CPRD)) == 100U,
                  100U, *g45_reg(G45_PWM_BASE, PWM_CH(0, PWM_CPRD)), 1);
    g45test_check(result,
                  *g45_reg(G45_PWM_BASE, PWM_CH(1, PWM_CDTY)) == 300U,
                  300U, *g45_reg(G45_PWM_BASE, PWM_CH(1, PWM_CDTY)), 2);

    /* Enable bitmap semantics: only CHID0-3 exist. */
    *g45_reg(G45_PWM_BASE, PWM_ENA) = 0x3U;
    g45test_check(result, *g45_reg(G45_PWM_BASE, PWM_SR) == 0x3U,
                  0x3U, *g45_reg(G45_PWM_BASE, PWM_SR), 3);
    *g45_reg(G45_PWM_BASE, PWM_ENA) = 0xf0U;
    g45test_check(result, *g45_reg(G45_PWM_BASE, PWM_SR) == 0x3U,
                  0x3U, *g45_reg(G45_PWM_BASE, PWM_SR), 4);

    /* A running channel reports period events (eventually on silicon). */
    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_PWM_BASE, PWM_ISR), 0x3U,
                               RT_TRUE, 200U),
                  0x3U, *g45_reg(G45_PWM_BASE, PWM_ISR) & 0x3U, 5);

    /* CUPD steers to CDTY or CPRD per CMR.UPD_CDTY, by the next period. */
    *g45_reg(G45_PWM_BASE, PWM_CH(0, PWM_CUPD)) = 50U;
    v = g45ctrl_wait_value(g45_reg(G45_PWM_BASE, PWM_CH(0, PWM_CDTY)),
                           50U, 200U);
    g45test_check(result, v == 50U, 50U, v, 6);
    g45test_check(result,
                  *g45_reg(G45_PWM_BASE, PWM_CH(0, PWM_CPRD)) == 100U,
                  100U, *g45_reg(G45_PWM_BASE, PWM_CH(0, PWM_CPRD)), 7);
    *g45_reg(G45_PWM_BASE, PWM_CH(1, PWM_CUPD)) = 350U;
    v = g45ctrl_wait_value(g45_reg(G45_PWM_BASE, PWM_CH(1, PWM_CPRD)),
                           350U, 200U);
    g45test_check(result, v == 350U, 350U, v, 8);

    /* Interrupt mask bookkeeping (line assertion is a documented gap). */
    *g45_reg(G45_PWM_BASE, PWM_IER) = 0x5U;
    g45test_check(result, *g45_reg(G45_PWM_BASE, PWM_IMR) == 0x5U,
                  0x5U, *g45_reg(G45_PWM_BASE, PWM_IMR), 9);
    *g45_reg(G45_PWM_BASE, PWM_IDR) = 0x1U;
    g45test_check(result, *g45_reg(G45_PWM_BASE, PWM_IMR) == 0x4U,
                  0x4U, *g45_reg(G45_PWM_BASE, PWM_IMR), 10);

    /* Disable everything again. */
    *g45_reg(G45_PWM_BASE, PWM_DIS) = 0xfU;
    *g45_reg(G45_PWM_BASE, PWM_IDR) = 0xfU;
    v = *g45_reg(G45_PWM_BASE, PWM_SR);
    g45test_check(result, v == 0U, 0, v, 11);

    rt_kprintf("G45TEST DATA case=r4.pwm imr_final=0x%08x\n",
               *g45_reg(G45_PWM_BASE, PWM_IMR));
    g45ctrl_clock_stop();
}

void g45test_r4_trng(struct g45test_result *result)
{
    rt_uint32_t words[8];
    rt_uint32_t distinct = 0;
    rt_uint32_t nonzero = 0;
    rt_uint32_t i;

    *(volatile rt_uint32_t *)G45_PMC_PCER = 1U << G45_TRNG_PID;

    /* Correct-key disable first: a known-quiet starting state. */
    *g45_reg(G45_TRNG_BASE, TRNG_CR) = TRNG_KEY;
    g45test_check(result, *g45_reg(G45_TRNG_BASE, TRNG_ISR) == 0U,
                  0, *g45_reg(G45_TRNG_BASE, TRNG_ISR), 0);
    g45test_check(result, *g45_reg(G45_TRNG_BASE, TRNG_ODATA) == 0U,
                  0, *g45_reg(G45_TRNG_BASE, TRNG_ODATA), 1);

    /* The enable is key-protected: a wrong key must be ignored. */
    *g45_reg(G45_TRNG_BASE, TRNG_CR) = 0x12345600U | TRNG_CR_ENABLE;
    g45test_check(result, *g45_reg(G45_TRNG_BASE, TRNG_ISR) == 0U,
                  0, *g45_reg(G45_TRNG_BASE, TRNG_ISR), 2);

    *g45_reg(G45_TRNG_BASE, TRNG_CR) = TRNG_KEY | TRNG_CR_ENABLE;
    g45test_check(result,
                  (*g45_reg(G45_TRNG_BASE, TRNG_ISR) & TRNG_ISR_DATRDY)
                  != 0U,
                  TRNG_ISR_DATRDY, *g45_reg(G45_TRNG_BASE, TRNG_ISR), 3);

    /*
     * Health check, not a statistical proof: eight words must not all
     * be identical and must not all be zero.
     */
    for (i = 0; i < 8U; i++) {
        words[i] = *g45_reg(G45_TRNG_BASE, TRNG_ODATA);
        if (words[i] != words[0]) {
            distinct = 1;
        }
        if (words[i] != 0U) {
            nonzero = 1;
        }
    }
    g45test_check(result, distinct == 1U, 1, distinct, 4);
    g45test_check(result, nonzero == 1U, 1, nonzero, 5);

    /* A wrong-key disable must be ignored too... */
    *g45_reg(G45_TRNG_BASE, TRNG_CR) = 0x00000100U;
    g45test_check(result,
                  (*g45_reg(G45_TRNG_BASE, TRNG_ISR) & TRNG_ISR_DATRDY)
                  != 0U,
                  TRNG_ISR_DATRDY, *g45_reg(G45_TRNG_BASE, TRNG_ISR), 6);

    /* ...and the correct-key disable silences data and status. */
    *g45_reg(G45_TRNG_BASE, TRNG_CR) = TRNG_KEY;
    g45test_check(result, *g45_reg(G45_TRNG_BASE, TRNG_ISR) == 0U,
                  0, *g45_reg(G45_TRNG_BASE, TRNG_ISR), 7);
    g45test_check(result, *g45_reg(G45_TRNG_BASE, TRNG_ODATA) == 0U,
                  0, *g45_reg(G45_TRNG_BASE, TRNG_ODATA), 8);

    /* MR keeps only the half-rate bit. */
    *g45_reg(G45_TRNG_BASE, TRNG_MR) = 0xffffffffU;
    g45test_check(result, *g45_reg(G45_TRNG_BASE, TRNG_MR) == 1U,
                  1, *g45_reg(G45_TRNG_BASE, TRNG_MR), 9);
    *g45_reg(G45_TRNG_BASE, TRNG_MR) = 0U;

    rt_kprintf("G45TEST DATA case=r4.trng w0=0x%08x w1=0x%08x\n",
               words[0], words[1]);
}

/*
 * ADC core of the TSADCC.  Board-portable by construction: converted
 * values are bounds-checked and recorded, never asserted (they are
 * synthetic in the model and analog on hardware), and everything a
 * conversion takes time for on silicon is polled with a budget.  Touch
 * press/move/release needs host-side input injection and stays with
 * the harness-integrated tests.
 */
void g45test_r4_tsadcc(struct g45test_result *result)
{
    rt_uint32_t sr, cdr0, cdr1, lcdr;
    rt_uint32_t both = ADC_SR_EOC(0) | ADC_SR_EOC(1) | ADC_SR_DRDY;
    rt_uint32_t ovr = ADC_SR_OVRE(0) | ADC_SR_OVRE(1) | ADC_SR_GOVRE;

    g45ctrl_clock_start();
    *(volatile rt_uint32_t *)G45_PMC_PCER = 1U << G45_ADC_PID;

    *g45_reg(G45_ADC_BASE, ADC_CR) = ADC_CR_SWRST;
    g45test_check(result, *g45_reg(G45_ADC_BASE, ADC_CHSR) == 0U,
                  0, *g45_reg(G45_ADC_BASE, ADC_CHSR), 0);

    /* Channel enable bitmap. */
    *g45_reg(G45_ADC_BASE, ADC_CHER) = 0x05U;
    g45test_check(result, *g45_reg(G45_ADC_BASE, ADC_CHSR) == 0x05U,
                  0x05U, *g45_reg(G45_ADC_BASE, ADC_CHSR), 1);
    *g45_reg(G45_ADC_BASE, ADC_CHDR) = 0x04U;
    *g45_reg(G45_ADC_BASE, ADC_CHER) = 0x02U;
    g45test_check(result, *g45_reg(G45_ADC_BASE, ADC_CHSR) == 0x03U,
                  0x03U, *g45_reg(G45_ADC_BASE, ADC_CHSR), 2);

    g45test_check(result, (*g45_reg(G45_ADC_BASE, ADC_SR) & both) == 0U,
                  0, *g45_reg(G45_ADC_BASE, ADC_SR) & both, 3);

    /* One software-triggered sequence over ch0+ch1. */
    *g45_reg(G45_ADC_BASE, ADC_CR) = ADC_CR_START;
    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_ADC_BASE, ADC_SR), ADC_SR_DRDY,
                               RT_TRUE, 100U),
                  ADC_SR_DRDY, *g45_reg(G45_ADC_BASE, ADC_SR), 4);
    sr = *g45_reg(G45_ADC_BASE, ADC_SR);
    g45test_check(result, (sr & both) == both, both, sr & both, 5);

    /* CDR read clears its EOC only. */
    cdr0 = *g45_reg(G45_ADC_BASE, ADC_CDR(0));
    g45test_check(result, cdr0 <= 0x3ffU, 0x3ffU, cdr0, 6);
    sr = *g45_reg(G45_ADC_BASE, ADC_SR);
    g45test_check(result,
                  (sr & ADC_SR_EOC(0)) == 0U &&
                  (sr & ADC_SR_EOC(1)) != 0U,
                  ADC_SR_EOC(1), sr & (ADC_SR_EOC(0) | ADC_SR_EOC(1)), 7);

    /* LCDR is the last converted word; reading clears DRDY + its EOC. */
    lcdr = *g45_reg(G45_ADC_BASE, ADC_LCDR);
    cdr1 = *g45_reg(G45_ADC_BASE, ADC_CDR(1));
    g45test_check(result, lcdr == cdr1, cdr1, lcdr, 8);
    sr = *g45_reg(G45_ADC_BASE, ADC_SR);
    g45test_check(result,
                  (sr & (ADC_SR_DRDY | ADC_SR_EOC(1))) == 0U,
                  0, sr & (ADC_SR_DRDY | ADC_SR_EOC(1)), 9);

    /* Two triggers without draining overrun both channels + globally. */
    *g45_reg(G45_ADC_BASE, ADC_CR) = ADC_CR_START;
    g45ctrl_wait(g45_reg(G45_ADC_BASE, ADC_SR), ADC_SR_DRDY, RT_TRUE, 100U);
    *g45_reg(G45_ADC_BASE, ADC_CR) = ADC_CR_START;
    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_ADC_BASE, ADC_SR), ADC_SR_GOVRE,
                               RT_TRUE, 100U),
                  ADC_SR_GOVRE, *g45_reg(G45_ADC_BASE, ADC_SR) & ovr, 10);
    sr = *g45_reg(G45_ADC_BASE, ADC_SR);   /* read-clears overrun bits */
    sr = *g45_reg(G45_ADC_BASE, ADC_SR);
    g45test_check(result, (sr & ovr) == 0U, 0, sr & ovr, 11);
    g45test_check(result, (sr & both) == both, both, sr & both, 12);
    (void)*g45_reg(G45_ADC_BASE, ADC_CDR(0));
    (void)*g45_reg(G45_ADC_BASE, ADC_LCDR);

    /* DRDY interrupt reaches the (masked) dedicated AIC source. */
    *g45_reg(G45_ADC_BASE, ADC_IER) = ADC_SR_DRDY;
    g45test_check(result, *g45_reg(G45_ADC_BASE, ADC_IMR) == ADC_SR_DRDY,
                  ADC_SR_DRDY, *g45_reg(G45_ADC_BASE, ADC_IMR), 13);
    *g45_reg(G45_ADC_BASE, ADC_CR) = ADC_CR_START;
    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_AIC_BASE, AIC_IPR),
                               1U << G45_ADC_PID, RT_TRUE, 100U),
                  1U << G45_ADC_PID,
                  *g45_reg(G45_AIC_BASE, AIC_IPR) & (1U << G45_ADC_PID), 14);
    (void)*g45_reg(G45_ADC_BASE, ADC_LCDR);
    g45test_check(result,
                  g45ctrl_wait(g45_reg(G45_AIC_BASE, AIC_IPR),
                               1U << G45_ADC_PID, RT_FALSE, 100U),
                  0, *g45_reg(G45_AIC_BASE, AIC_IPR) & (1U << G45_ADC_PID),
                  15);
    *g45_reg(G45_ADC_BASE, ADC_IDR) = 0xffffffffU;
    *g45_reg(G45_ADC_BASE, ADC_CR) = ADC_CR_SWRST;

    rt_kprintf("G45TEST DATA case=r4.tsadcc cdr0=0x%03x cdr1=0x%03x\n",
               cdr0, cdr1);
    g45ctrl_clock_stop();
}

void g45test_r4_gpbr(struct g45test_result *result)
{
    static const rt_uint32_t patterns[4] = {
        0xa5170001U, 0x5ac3fe02U, 0x00000000U, 0xffffffffU,
    };
    rt_uint32_t i;

    for (i = 0; i < 4U; i++) {
        *g45_reg(G45_GPBR_BASE, 4U * i) = patterns[i];
    }
    for (i = 0; i < 4U; i++) {
        g45test_check(result, *g45_reg(G45_GPBR_BASE, 4U * i) == patterns[i],
                      patterns[i], *g45_reg(G45_GPBR_BASE, 4U * i), i);
    }
    for (i = 0; i < 4U; i++) {
        *g45_reg(G45_GPBR_BASE, 4U * i) = ~patterns[i];
    }
    for (i = 0; i < 4U; i++) {
        g45test_check(result, *g45_reg(G45_GPBR_BASE, 4U * i) == ~patterns[i],
                      ~patterns[i], *g45_reg(G45_GPBR_BASE, 4U * i), 4U + i);
    }
    rt_kprintf("G45TEST DATA case=r4.gpbr words=4\n");
}
