/*
 * Shared declarations for the AT91SAM9G45 RT-Thread differential tests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef G45TEST_H
#define G45TEST_H

#include <rtthread.h>

struct g45test_result {
    const char *name;
    rt_uint32_t seed;
    rt_uint32_t checks;
    rt_uint32_t failed_check;
    rt_uint32_t expected;
    rt_uint32_t actual;
    rt_uint32_t offset;
    rt_tick_t started;
    rt_bool_t failed;
};

typedef void (*g45test_case_fn)(struct g45test_result *result);

struct g45test_case {
    const char *name;
    const char *suite;
    const char *phase;
    g45test_case_fn run;
};

void g45test_check(struct g45test_result *result, rt_bool_t condition,
                   rt_uint32_t expected, rt_uint32_t actual,
                   rt_uint32_t offset);

void g45test_d2_subbuffer_arbitration(struct g45test_result *result);
void g45test_d1_contract(struct g45test_result *result);
void g45test_d1_mem2mem(struct g45test_result *result);
void g45test_d1_channels(struct g45test_result *result);
void g45test_d1_arbitration(struct g45test_result *result);
void g45test_d1_boundary_overlap(struct g45test_result *result);
void g45test_d1_linked_list(struct g45test_result *result);
void g45test_d1_descriptor_auto_boundary(struct g45test_result *result);
void g45test_d1_live_descriptor_update(struct g45test_result *result);
void g45test_d1_suspend_resume(struct g45test_result *result);
void g45test_d1_software_requests(struct g45test_result *result);
void g45test_d1_software_last(struct g45test_result *result);
void g45test_d1_word_width_alias(struct g45test_result *result);
void g45test_d1_bus_error(struct g45test_result *result);
void g45test_d1_stop_on_done(struct g45test_result *result);
void g45test_d1_partial_descriptor_reload(struct g45test_result *result);
void g45test_d1_picture_in_picture(struct g45test_result *result);
void g45test_d1_auto_replay(struct g45test_result *result);
void g45test_d1_mixed_software_requests(struct g45test_result *result);
void g45test_d1_irq(struct g45test_result *result);
void g45test_d1_irq_mask(struct g45test_result *result);

void g45test_r4_rtc_update_protocol(struct g45test_result *result);
void g45test_r4_rtc_leap_rollover(struct g45test_result *result);
void g45test_r4_rtc_alarm_irq(struct g45test_result *result);
void g45test_r4_rtt_increment_alarm(struct g45test_result *result);
void g45test_r4_pwm(struct g45test_result *result);
void g45test_r4_tsadcc(struct g45test_result *result);
void g45test_r4_twi(struct g45test_result *result);
void g45test_r4_spi(struct g45test_result *result);
void g45test_r4_touch(struct g45test_result *result);
void g45test_r4_reset_seed(struct g45test_result *result);
void g45test_r4_reset_verify(struct g45test_result *result);
void g45test_irq_wired_or(struct g45test_result *result);
void g45test_irq_priority_order(struct g45test_result *result);
void g45test_irq_pit_ack(struct g45test_result *result);
void g45test_irq_tc_oneshot(struct g45test_result *result);
void g45test_r4_trng(struct g45test_result *result);
void g45test_r4_gpbr(struct g45test_result *result);

#endif /* G45TEST_H */
