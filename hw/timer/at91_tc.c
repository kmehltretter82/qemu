/*
 * Atmel/Microchip AT91 Timer Counter (TC) block - 3 x 16-bit channels.
 *
 * Modelled for the Linux tcb clocksource, PWM and counter drivers:
 *   - a channel free-running at an MCK-derived rate (WAVSEL=UP), read
 *     through CV: clocksource low word;
 *   - the next channel clocked from XC1 = TIOA0 (BMR TC1XC1S), counting
 *     one tick per 65536 ticks of channel 0 (channel 0 toggles TIOA0 via
 *     ACPA at RA=0 / ACPC at RC=0x8000): clocksource high word.  This
 *     chaining is modelled as a rate divider rather than as edge events;
 *   - a channel in WAVSEL=UP_RC with an RC compare interrupt (CPCS) as
 *     the tick clockevent, periodic (auto-restart) or one-shot
 *     (CPCSTOP/CPCDIS), usually on TIMER_CLOCK5 = the 32.768 kHz slow
 *     clock.
 *
 *   - all four waveform count modes, RA/RB/RC compare events and the
 *     programmable TIOA/TIOB set/clear/toggle actions used by pwm-atmel-tcb;
 *   - capture-mode TIOA edge loading into RA/RB, TIOA/TIOB external triggers,
 *     and external XC clocks/events exposed as QEMU GPIO inputs.
 *
 * The SAM9G45 has no quadrature decoder.  BURST clock gating and stepper-motor
 * gray-count mode are retained in their registers but do not alter counting.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/timer/at91_tc.h"
#include "trace.h"

#define TC_NCH        3
#define TC_CH_SPAN    0x40

/* Per-channel registers (offset within the channel) */
#define TC_CCR        0x00   /* Channel Control (write-only)     */
#define TC_CMR        0x04   /* Channel Mode                     */
#define TC_SMMR       0x08   /* Stepper Motor Mode               */
#define TC_CV         0x10   /* Counter Value (read-only)        */
#define TC_RA         0x14
#define TC_RB         0x18
#define TC_RC         0x1C
#define TC_SR         0x20   /* Status (read-only, clear-on-read) */
#define TC_IER        0x24
#define TC_IDR        0x28
#define TC_IMR        0x2C

/* Block registers */
#define TC_BCR        0xC0
#define TC_BMR        0xC4

#define TC_CCR_CLKEN  (1u << 0)
#define TC_CCR_CLKDIS (1u << 1)
#define TC_CCR_SWTRG  (1u << 2)

#define TC_CMR_TCCLKS(cmr)  ((cmr) & 7)
#define TC_CMR_CLKI         (1u << 3)
#define TC_CMR_BURST        (3u << 4)
#define TC_CMR_CPCSTOP      (1u << 6)
#define TC_CMR_CPCDIS       (1u << 7)
#define TC_CMR_EDGE(cmr)    (((cmr) >> 8) & 3)
#define TC_CMR_EEVT(cmr)    (((cmr) >> 10) & 3)
#define TC_CMR_ABETRG       (1u << 10)
#define TC_CMR_ENETRG       (1u << 12)
#define TC_CMR_WAVSEL(cmr)  (((cmr) >> 13) & 3)
#define TC_CMR_CPCTRG       (1u << 14)
#define TC_CMR_WAVE         (1u << 15)
#define TC_CMR_LDRA(cmr)    (((cmr) >> 16) & 3)
#define TC_CMR_LDRB(cmr)    (((cmr) >> 18) & 3)
#define TC_CMR_ACTION(cmr, shift) (((cmr) >> (shift)) & 3)

#define TC_WAVSEL_UP        0
#define TC_WAVSEL_UPDOWN    1
#define TC_WAVSEL_UP_RC     2
#define TC_WAVSEL_UPDOWN_RC 3

#define TC_SR_COVFS   (1u << 0)
#define TC_SR_LOVRS   (1u << 1)
#define TC_SR_CPAS    (1u << 2)
#define TC_SR_CPBS    (1u << 3)
#define TC_SR_CPCS    (1u << 4)
#define TC_SR_LDRAS   (1u << 5)
#define TC_SR_LDRBS   (1u << 6)
#define TC_SR_ETRGS   (1u << 7)
#define TC_SR_CLKSTA  (1u << 16)
#define TC_SR_MTIOA   (1u << 17)
#define TC_SR_MTIOB   (1u << 18)
#define TC_SR_EVENTS  0xFF

#define TC_BCR_SYNC   (1u << 0)

#define TC_BMR_TC1XC1S(bmr)  (((bmr) >> 2) & 3)
#define TC_XC1S_TIOA0        2

#define TC_ACT_NONE    0
#define TC_ACT_SET     1
#define TC_ACT_CLEAR   2
#define TC_ACT_TOGGLE  3

typedef struct AT91TcChan {
    uint32_t cmr;
    uint32_t smmr;
    uint32_t ra, rb, rc;
    uint32_t imr;
    uint32_t sr;           /* event bits only (COVFS..ETRGS) */
    bool clken;
    int64_t epoch;         /* vtime when the counter was last (re)started */
    uint32_t cv_frozen;    /* CV held while the clock is disabled */
    uint64_t ext_ticks;    /* selected external-XC edges since trigger */
    uint64_t chain_origin; /* channel-0 tick corresponding to ext_ticks */
    uint64_t event_tick;   /* raw tick represented by the armed timer */
    uint64_t last_event_tick;
    bool tioa;
    bool tiob;
    QEMUTimer *timer;
    struct AT91TcState *parent;   /* back-link for the timer callback */
    int idx;
} AT91TcChan;

struct AT91TcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *mck;
    uint32_t mck_freq;
    uint32_t slck_freq;
    uint32_t counter_width;   /* 16 (rm9200/sam9g45) or 32 (sam9x5/sama5) */
    uint32_t bmr;
    bool tclk[TC_NCH];
    qemu_irq tioa_out[TC_NCH];
    qemu_irq tiob_out[TC_NCH];

    AT91TcChan ch[TC_NCH];
    uint64_t migration_ticks[TC_NCH];
};

/* Free-running counter modulus (0x10000 or 0x1_0000_0000) and top value
 * (0xffff or 0xffffffff), per the TC block's counter width. */
static inline uint64_t tc_wrap(const AT91TcChan *c)
{
    return UINT64_C(1) << c->parent->counter_width;
}

static inline uint32_t tc_top(const AT91TcChan *c)
{
    return (uint32_t)(tc_wrap(c) - 1);
}

/* Input clock in Hz for a channel; 0 if unmodelled (external XC input
 * other than the TIOA0 chain). */
static uint64_t tc_rate(AT91TcState *s, int n)
{
    static const uint32_t mck_div[4] = { 2, 8, 32, 128 };
    uint32_t tcclks = TC_CMR_TCCLKS(s->ch[n].cmr);

    if (tcclks < 4) {
        return s->mck_freq / mck_div[tcclks];
    }
    if (tcclks == 4) {
        return s->slck_freq;
    }
    /* XC inputs: model only XC1 = TIOA0 (the tcb_clksrc 16-bit chain):
     * channel 0 produces one TIOA0 period per 65536 of its own ticks. */
    if (n == 1 && tcclks == 6 && TC_BMR_TC1XC1S(s->bmr) == TC_XC1S_TIOA0) {
        return tc_rate(s, 0) >> 16;
    }
    return 0;
}

static uint64_t tc_ticks(AT91TcState *s, int n, int64_t now)
{
    uint64_t rate = tc_rate(s, n);

    if (!rate) {
        return s->ch[n].ext_ticks;
    }
    /*
     * The chained channel is the exact high word of channel 0.  Do not
     * approximate it by multiplying elapsed time by (rate >> 16): integer
     * truncation would make the high word lag the low word and the composed
     * 32-bit clocksource would jump backwards whenever channel 0 wraps.
     */
    if (n == 1 && TC_CMR_TCCLKS(s->ch[n].cmr) == 6) {
        uint64_t low_ticks = tc_ticks(s, 0, now);
        uint64_t delta = low_ticks >= s->ch[n].chain_origin ?
                         low_ticks - s->ch[n].chain_origin : 0;

        return s->ch[n].ext_ticks + (delta >> 16);
    }
    return s->ch[n].ext_ticks +
           muldiv64(now - s->ch[n].epoch, rate, NANOSECONDS_PER_SECOND);
}

static uint32_t tc_cv_for_ticks(const AT91TcChan *c, uint64_t ticks)
{
    uint32_t mode = (c->cmr & TC_CMR_WAVE) ? TC_CMR_WAVSEL(c->cmr) :
                    ((c->cmr & TC_CMR_CPCTRG) && c->rc ?
                     TC_WAVSEL_UP_RC : TC_WAVSEL_UP);
    uint64_t period;
    uint32_t top;

    switch (mode) {
    case TC_WAVSEL_UP_RC:
        if (c->rc) {
            return ticks % c->rc;
        }
        break;
    case TC_WAVSEL_UPDOWN:
        period = 2 * (uint64_t)tc_top(c);
        ticks %= period;
        return ticks <= tc_top(c) ? ticks : period - ticks;
    case TC_WAVSEL_UPDOWN_RC:
        top = c->rc;
        if (top) {
            period = 2 * (uint64_t)top;
            ticks %= period;
            return ticks <= top ? ticks : period - ticks;
        }
        break;
    default:
        break;
    }
    return ticks & tc_top(c);
}

static uint32_t tc_cv(AT91TcState *s, int n, int64_t now)
{
    AT91TcChan *c = &s->ch[n];

    return c->clken ? tc_cv_for_ticks(c, tc_ticks(s, n, now)) :
                      c->cv_frozen;
}

static bool tc_edge_matches(unsigned selector, bool old_level, bool level)
{
    bool rising = !old_level && level;
    bool falling = old_level && !level;

    return (rising && (selector & 1)) || (falling && (selector & 2));
}

static void tc_update_irq(AT91TcState *s);
static void tc_rearm(AT91TcState *s, int n);

static void tc_handle_xc_edge(AT91TcState *s, int xc,
                              bool old_level, bool level);

static void tc_set_output(AT91TcState *s, int n, bool is_a, bool level)
{
    AT91TcChan *c = &s->ch[n];
    bool old_level = is_a ? c->tioa : c->tiob;

    if (old_level == level) {
        return;
    }
    if (is_a) {
        c->tioa = level;
        qemu_set_irq(s->tioa_out[n], level);
    } else {
        c->tiob = level;
        qemu_set_irq(s->tiob_out[n], level);
    }
    trace_at91_tc_output(n, is_a ? 'A' : 'B', level);

    /* Each XC input can be sourced from one of the other channels' TIOA. */
    if (is_a) {
        int xc;

        for (xc = 0; xc < TC_NCH; xc++) {
            unsigned sel = (s->bmr >> (xc * 2)) & 3;
            int source = -1;

            if (xc == 0) {
                source = sel == 2 ? 1 : sel == 3 ? 2 : -1;
            } else if (xc == 1) {
                source = sel == 2 ? 0 : sel == 3 ? 2 : -1;
            } else {
                source = sel == 2 ? 0 : sel == 3 ? 1 : -1;
            }
            if (source == n) {
                tc_handle_xc_edge(s, xc, old_level, level);
            }
        }
    }
}

static void tc_apply_action(AT91TcState *s, int n, bool is_a,
                            unsigned action)
{
    AT91TcChan *c = &s->ch[n];
    bool level = is_a ? c->tioa : c->tiob;

    switch (action) {
    case TC_ACT_SET:
        level = true;
        break;
    case TC_ACT_CLEAR:
        level = false;
        break;
    case TC_ACT_TOGGLE:
        level = !level;
        break;
    default:
        return;
    }
    tc_set_output(s, n, is_a, level);
}

static void tc_update_irq(AT91TcState *s)
{
    int n;
    bool level = false;

    for (n = 0; n < TC_NCH; n++) {
        if (s->ch[n].sr & s->ch[n].imr) {
            level = true;
        }
    }
    qemu_set_irq(s->irq, level);
}

static void tc_add_next(uint64_t *next, uint64_t candidate)
{
    if (candidate < *next) {
        *next = candidate;
    }
}

static uint64_t tc_next_phase(uint64_t ticks, uint64_t period, uint64_t phase)
{
    uint64_t base = ticks - ticks % period;
    uint64_t candidate = base + phase;

    if (candidate <= ticks) {
        candidate += period;
    }
    return candidate;
}

static void tc_add_triangle_matches(uint64_t *next, uint64_t ticks,
                                    uint32_t top, uint32_t value)
{
    uint64_t period;

    if (!top || value > top) {
        return;
    }
    period = 2 * (uint64_t)top;
    tc_add_next(next, tc_next_phase(ticks, period, value));
    if (value && value != top) {
        tc_add_next(next, tc_next_phase(ticks, period, period - value));
    }
}

static uint64_t tc_next_event(const AT91TcChan *c, uint64_t ticks)
{
    uint64_t next = UINT64_MAX;
    unsigned mode;

    if (!(c->cmr & TC_CMR_WAVE)) {
        if ((c->cmr & TC_CMR_CPCTRG) && c->rc) {
            tc_add_next(&next, tc_next_phase(ticks, c->rc, 0));
        } else {
            tc_add_next(&next, tc_next_phase(ticks, tc_wrap(c), c->rc));
            tc_add_next(&next, tc_next_phase(ticks, tc_wrap(c), 0));
        }
        return next;
    }

    mode = TC_CMR_WAVSEL(c->cmr);
    switch (mode) {
    case TC_WAVSEL_UP_RC:
        if (c->rc) {
            if (c->ra < c->rc) {
                tc_add_next(&next, tc_next_phase(ticks, c->rc, c->ra));
            }
            if (c->rb < c->rc) {
                tc_add_next(&next, tc_next_phase(ticks, c->rc, c->rb));
            }
            tc_add_next(&next, tc_next_phase(ticks, c->rc, 0));
            break;
        }
        /* RC=0 behaves as a normal free-running counter. */
        /* fall through */
    case TC_WAVSEL_UP:
        tc_add_next(&next, tc_next_phase(ticks, tc_wrap(c), c->ra));
        tc_add_next(&next, tc_next_phase(ticks, tc_wrap(c), c->rb));
        tc_add_next(&next, tc_next_phase(ticks, tc_wrap(c), c->rc));
        tc_add_next(&next, tc_next_phase(ticks, tc_wrap(c), 0));
        break;
    case TC_WAVSEL_UPDOWN:
        tc_add_triangle_matches(&next, ticks, tc_top(c), c->ra);
        tc_add_triangle_matches(&next, ticks, tc_top(c), c->rb);
        tc_add_triangle_matches(&next, ticks, tc_top(c), c->rc);
        tc_add_next(&next, tc_next_phase(ticks, 2 * (uint64_t)tc_top(c), 0));
        break;
    case TC_WAVSEL_UPDOWN_RC:
        if (c->rc) {
            tc_add_triangle_matches(&next, ticks, c->rc, c->ra);
            tc_add_triangle_matches(&next, ticks, c->rc, c->rb);
            tc_add_triangle_matches(&next, ticks, c->rc, c->rc);
            tc_add_next(&next, tc_next_phase(ticks, 2 * (uint64_t)c->rc, 0));
        }
        break;
    }
    return next;
}

static uint32_t tc_event_flags(const AT91TcChan *c, uint64_t ticks)
{
    uint32_t cv = tc_cv_for_ticks(c, ticks);
    uint32_t flags = 0;
    unsigned mode;

    if (!(c->cmr & TC_CMR_WAVE)) {
        if ((c->cmr & TC_CMR_CPCTRG) && c->rc) {
            if (ticks % c->rc == 0) {
                flags |= TC_SR_CPCS;
            }
        } else {
            if (cv == c->rc) {
                flags |= TC_SR_CPCS;
            }
            if ((ticks & tc_top(c)) == 0) {
                flags |= TC_SR_COVFS;
            }
        }
        return flags;
    }

    mode = TC_CMR_WAVSEL(c->cmr);
    if (cv == c->ra) {
        flags |= TC_SR_CPAS;
    }
    if (cv == c->rb) {
        flags |= TC_SR_CPBS;
    }
    if (mode == TC_WAVSEL_UP_RC && c->rc) {
        if (ticks % c->rc == 0) {
            flags |= TC_SR_CPCS;
        }
    } else if (cv == c->rc) {
        flags |= TC_SR_CPCS;
    }

    if ((mode == TC_WAVSEL_UP && (ticks & tc_top(c)) == 0) ||
        (mode == TC_WAVSEL_UPDOWN && ticks % (2 * (uint64_t)tc_top(c)) == 0) ||
        (mode == TC_WAVSEL_UPDOWN_RC && c->rc &&
         ticks % (2 * (uint64_t)c->rc) == 0)) {
        flags |= TC_SR_COVFS;
    }
    return flags;
}

static void tc_process_event(AT91TcState *s, int n, uint64_t ticks)
{
    AT91TcChan *c = &s->ch[n];
    uint32_t flags = tc_event_flags(c, ticks);

    if (!flags) {
        return;
    }
    c->sr |= flags;
    if (c->cmr & TC_CMR_WAVE) {
        if (flags & TC_SR_CPAS) {
            tc_apply_action(s, n, true, TC_CMR_ACTION(c->cmr, 16));
        }
        if (flags & TC_SR_CPBS) {
            tc_apply_action(s, n, false, TC_CMR_ACTION(c->cmr, 24));
        }
        if (flags & TC_SR_CPCS) {
            tc_apply_action(s, n, true, TC_CMR_ACTION(c->cmr, 18));
            tc_apply_action(s, n, false, TC_CMR_ACTION(c->cmr, 26));
        }
    }
    trace_at91_tc_event(n, flags, tc_cv_for_ticks(c, ticks));

    if ((flags & TC_SR_CPCS) && (c->cmr & TC_CMR_WAVE) &&
        (c->cmr & (TC_CMR_CPCDIS | TC_CMR_CPCSTOP))) {
        c->cv_frozen = tc_cv_for_ticks(c, ticks);
        c->clken = false;
        timer_del(c->timer);
    }
    tc_update_irq(s);
}

/* Arm the next compare/overflow event for an internally clocked channel. */
static void tc_rearm(AT91TcState *s, int n)
{
    AT91TcChan *c = &s->ch[n];
    uint64_t rate = tc_rate(s, n);
    uint64_t ticks;
    uint64_t next;
    int64_t deadline;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!c->clken || !rate) {
        timer_del(c->timer);
        return;
    }
    ticks = MAX(tc_ticks(s, n, now), c->last_event_tick);
    next = tc_next_event(c, ticks);
    if (next == UINT64_MAX) {
        timer_del(c->timer);
        return;
    }
    c->event_tick = next;
    deadline = c->epoch + muldiv64(next - c->ext_ticks,
                                   NANOSECONDS_PER_SECOND, rate);
    if (deadline <= now) {
        deadline = now + 1;
    }
    timer_mod(c->timer, deadline);
}

static void tc_clock_update(void *opaque, ClockEvent event)
{
    AT91TcState *s = AT91_TC(opaque);
    uint64_t ticks[TC_NCH];
    int64_t now;
    int n;

    switch (event) {
    case ClockPreUpdate:
        if (!s->ch[0].timer) {
            break;
        }
        now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        for (n = 0; n < TC_NCH; n++) {
            ticks[n] = s->ch[n].clken ? tc_ticks(s, n, now) :
                                        s->ch[n].ext_ticks;
        }
        for (n = 0; n < TC_NCH; n++) {
            AT91TcChan *c = &s->ch[n];

            c->ext_ticks = ticks[n];
            c->epoch = now;
            c->chain_origin = ticks[0];
            c->last_event_tick = ticks[n];
        }
        break;
    case ClockUpdate:
        s->mck_freq = clock_get_hz(s->mck);
        if (s->ch[0].timer) {
            for (n = 0; n < TC_NCH; n++) {
                tc_rearm(s, n);
            }
        }
        break;
    default:
        break;
    }
}

static void tc_compare_fire(void *opaque)
{
    AT91TcChan *c = opaque;
    AT91TcState *s = c->parent;
    int n = c->idx;

    c->last_event_tick = c->event_tick;
    tc_process_event(s, n, c->event_tick);
    if (c->clken) {
        tc_rearm(s, n);
    }
}

static void tc_trigger(AT91TcState *s, int n)
{
    AT91TcChan *c = &s->ch[n];

    c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    c->ext_ticks = 0;
    c->chain_origin = n == 1 ? tc_ticks(s, 0, c->epoch) : 0;
    c->event_tick = 0;
    c->last_event_tick = 0;
    c->cv_frozen = 0;
    tc_rearm(s, n);
}

static void tc_wave_external_event(AT91TcState *s, int n, unsigned source,
                                   bool old_level, bool level)
{
    AT91TcChan *c = &s->ch[n];

    if (!(c->cmr & TC_CMR_WAVE) || TC_CMR_EEVT(c->cmr) != source ||
        !tc_edge_matches(TC_CMR_EDGE(c->cmr), old_level, level)) {
        return;
    }
    c->sr |= TC_SR_ETRGS;
    tc_apply_action(s, n, true, TC_CMR_ACTION(c->cmr, 20));
    tc_apply_action(s, n, false, TC_CMR_ACTION(c->cmr, 28));
    trace_at91_tc_event(n, TC_SR_ETRGS,
                        tc_cv(s, n, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
    if (c->cmr & TC_CMR_ENETRG) {
        tc_trigger(s, n);
    }
    tc_update_irq(s);
}

static bool tc_is_analytic_chain(AT91TcState *s, int n, unsigned xc)
{
    return n == 1 && xc == 1 && TC_BMR_TC1XC1S(s->bmr) == TC_XC1S_TIOA0;
}

static void tc_handle_xc_edge(AT91TcState *s, int xc,
                              bool old_level, bool level)
{
    int n;

    for (n = 0; n < TC_NCH; n++) {
        AT91TcChan *c = &s->ch[n];

        tc_wave_external_event(s, n, xc + 1, old_level, level);
        if (!c->clken || TC_CMR_TCCLKS(c->cmr) != xc + 5 ||
            tc_is_analytic_chain(s, n, xc)) {
            continue;
        }
        if ((!old_level && level && !(c->cmr & TC_CMR_CLKI)) ||
            (old_level && !level && (c->cmr & TC_CMR_CLKI))) {
            c->ext_ticks++;
            c->cv_frozen = tc_cv_for_ticks(c, c->ext_ticks);
            tc_process_event(s, n, c->ext_ticks);
        }
    }
}

static void tc_capture_input(AT91TcState *s, int n, bool is_a, bool level)
{
    AT91TcChan *c = &s->ch[n];
    bool old_level = is_a ? c->tioa : c->tiob;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t cv;
    bool loaded_b = false;

    if (old_level == level) {
        return;
    }

    if (c->cmr & TC_CMR_WAVE) {
        /* TIOA is an output in waveform mode.  TIOB can instead be selected
         * as the external-event input by EEVT. */
        if (!is_a) {
            c->tiob = level;
            tc_wave_external_event(s, n, 0, old_level, level);
        }
        return;
    }
    if (is_a) {
        c->tioa = level;
    } else {
        c->tiob = level;
    }

    cv = tc_cv(s, n, now);
    if (is_a && tc_edge_matches(TC_CMR_LDRA(c->cmr), old_level, level)) {
        if (c->sr & TC_SR_LDRAS) {
            c->sr |= TC_SR_LOVRS;
        }
        c->ra = cv;
        c->sr |= TC_SR_LDRAS;
        trace_at91_tc_capture(n, 'A', cv);
    }
    if (is_a && tc_edge_matches(TC_CMR_LDRB(c->cmr), old_level, level)) {
        if (c->sr & TC_SR_LDRBS) {
            c->sr |= TC_SR_LOVRS;
        }
        c->rb = cv;
        c->sr |= TC_SR_LDRBS;
        loaded_b = true;
        trace_at91_tc_capture(n, 'B', cv);
    }

    if (is_a == !!(c->cmr & TC_CMR_ABETRG) &&
        tc_edge_matches(TC_CMR_EDGE(c->cmr), old_level, level)) {
        c->sr |= TC_SR_ETRGS;
        tc_trigger(s, n);
    }
    if (loaded_b && (c->cmr & (TC_CMR_CPCSTOP | TC_CMR_CPCDIS))) {
        c->cv_frozen = cv;
        c->clken = false;
        timer_del(c->timer);
    }
    tc_update_irq(s);
}

static void tc_tioa_input(void *opaque, int n, int level)
{
    tc_capture_input(AT91_TC(opaque), n, true, level);
}

static void tc_tiob_input(void *opaque, int n, int level)
{
    tc_capture_input(AT91_TC(opaque), n, false, level);
}

static void tc_tclk_input(void *opaque, int xc, int level)
{
    AT91TcState *s = AT91_TC(opaque);
    bool old_level = s->tclk[xc];
    unsigned source = (s->bmr >> (xc * 2)) & 3;

    s->tclk[xc] = level;
    if (source == 0) {
        tc_handle_xc_edge(s, xc, old_level, level);
    }
}

static uint64_t tc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91TcState *s = AT91_TC(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t r = 0;
    int n = offset / TC_CH_SPAN;

    if (n < TC_NCH) {
        AT91TcChan *c = &s->ch[n];

        switch (offset % TC_CH_SPAN) {
        case TC_CMR:  r = c->cmr; break;
        case TC_SMMR: r = c->smmr; break;
        case TC_CV:   r = tc_cv(s, n, now); break;
        case TC_RA:   r = c->ra; break;
        case TC_RB:   r = c->rb; break;
        case TC_RC:   r = c->rc; break;
        case TC_SR:
            r = c->sr | (c->clken ? TC_SR_CLKSTA : 0) |
                (c->tioa ? TC_SR_MTIOA : 0) |
                (c->tiob ? TC_SR_MTIOB : 0);
            c->sr &= ~TC_SR_EVENTS;
            tc_update_irq(s);
            break;
        case TC_IMR:  r = c->imr; break;
        default:
            qemu_log_mask(LOG_UNIMP, "at91-tc: read from unimplemented "
                          "ch%d offset 0x%02x\n", n,
                          (unsigned)(offset % TC_CH_SPAN));
            break;
        }
    } else if (offset == TC_BMR) {
        r = s->bmr;
    } else {
        qemu_log_mask(LOG_UNIMP, "at91-tc: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
    }
    return r;
}

static void tc_write(void *opaque, hwaddr offset, uint64_t value,
                     unsigned size)
{
    AT91TcState *s = AT91_TC(opaque);
    int n = offset / TC_CH_SPAN;

    if (n < TC_NCH) {
        AT91TcChan *c = &s->ch[n];
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        switch (offset % TC_CH_SPAN) {
        case TC_CCR:
            trace_at91_tc_ccr(n, (uint32_t)value);
            if (value & TC_CCR_CLKDIS) {
                c->ext_ticks = tc_ticks(s, n, now);
                c->cv_frozen = tc_cv_for_ticks(c, c->ext_ticks);
                c->clken = false;
                timer_del(c->timer);
            }
            if (value & TC_CCR_CLKEN) {
                if (!c->clken) {
                    c->clken = true;
                    c->epoch = now;
                    if (n == 1) {
                        c->chain_origin = tc_ticks(s, 0, now);
                    }
                    c->last_event_tick = c->ext_ticks;
                    tc_rearm(s, n);
                }
            }
            if (value & TC_CCR_SWTRG) {
                if (c->cmr & TC_CMR_WAVE) {
                    tc_apply_action(s, n, true,
                                    TC_CMR_ACTION(c->cmr, 22));
                    tc_apply_action(s, n, false,
                                    TC_CMR_ACTION(c->cmr, 30));
                }
                tc_trigger(s, n);
            }
            break;
        case TC_CMR:
            if (c->clken) {
                c->ext_ticks = tc_ticks(s, n, now);
                c->epoch = now;
            }
            c->cmr = value;
            if (n == 1) {
                c->chain_origin = tc_ticks(s, 0, now);
            }
            c->last_event_tick = c->ext_ticks;
            trace_at91_tc_cmr(n, (uint32_t)value);
            if (value & TC_CMR_BURST) {
                qemu_log_mask(LOG_UNIMP, "at91-tc: ch%d BURST clock "
                              "gating is not modelled\n", n);
            }
            tc_rearm(s, n);
            break;
        case TC_RA:
            c->ra = value & 0xFFFF;
            tc_rearm(s, n);
            break;
        case TC_RB:
            c->rb = value & 0xFFFF;
            tc_rearm(s, n);
            break;
        case TC_RC:
            c->rc = value & 0xFFFF;
            tc_rearm(s, n);
            break;
        case TC_IER:
            c->imr |= value & TC_SR_EVENTS;
            tc_update_irq(s);
            break;
        case TC_IDR:
            c->imr &= ~(value & TC_SR_EVENTS);
            tc_update_irq(s);
            break;
        case TC_SMMR:
            c->smmr = value & 3;
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "at91-tc: write to unimplemented "
                          "ch%d offset 0x%02x = 0x%08x\n", n,
                          (unsigned)(offset % TC_CH_SPAN), (uint32_t)value);
            break;
        }
    } else if (offset == TC_BCR) {
        if (value & TC_BCR_SYNC) {
            for (n = 0; n < TC_NCH; n++) {
                if (s->ch[n].clken) {
                    tc_trigger(s, n);
                }
            }
        }
    } else if (offset == TC_BMR) {
        uint64_t ticks[TC_NCH];
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        for (n = 0; n < TC_NCH; n++) {
            ticks[n] = tc_ticks(s, n, now);
        }
        s->bmr = value;
        for (n = 0; n < TC_NCH; n++) {
            s->ch[n].ext_ticks = ticks[n];
            s->ch[n].epoch = now;
            s->ch[n].chain_origin = tc_ticks(s, 0, now);
            s->ch[n].last_event_tick = ticks[n];
            tc_rearm(s, n);
        }
    } else {
        qemu_log_mask(LOG_UNIMP, "at91-tc: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n",
                      offset, (uint32_t)value);
    }
}

static const MemoryRegionOps tc_ops = {
    .read = tc_read,
    .write = tc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void tc_reset(DeviceState *dev)
{
    AT91TcState *s = AT91_TC(dev);
    int n;

    for (n = 0; n < TC_NCH; n++) {
        AT91TcChan *c = &s->ch[n];

        if (c->timer) {
            timer_del(c->timer);
        }
        c->cmr = c->smmr = c->ra = c->rb = c->rc = 0;
        c->imr = c->sr = 0;
        c->clken = false;
        c->cv_frozen = 0;
        c->ext_ticks = 0;
        c->chain_origin = 0;
        c->event_tick = 0;
        c->last_event_tick = 0;
        c->tioa = false;
        c->tiob = false;
        c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->tclk[n] = false;
        qemu_set_irq(s->tioa_out[n], 0);
        qemu_set_irq(s->tiob_out[n], 0);
    }
    s->bmr = 0;
    tc_update_irq(s);
}

static void tc_realize(DeviceState *dev, Error **errp)
{
    AT91TcState *s = AT91_TC(dev);
    int n;

    if (clock_has_source(s->mck)) {
        s->mck_freq = clock_get_hz(s->mck);
    } else if (s->mck_freq != 0) {
        clock_set_hz(s->mck, s->mck_freq);
    } else {
        error_setg(errp, "at91-tc: mck-frequency must be set");
        return;
    }
    if (s->counter_width != 16 && s->counter_width != 32) {
        error_setg(errp, "at91-tc: counter-width must be 16 or 32");
        return;
    }
    for (n = 0; n < TC_NCH; n++) {
        s->ch[n].parent = s;
        s->ch[n].idx = n;
        s->ch[n].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc_compare_fire,
                                      &s->ch[n]);
    }
}

static void tc_dev_init(Object *obj)
{
    AT91TcState *s = AT91_TC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &tc_ops, s, "at91-tc", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), tc_tioa_input, "tioa-in", TC_NCH);
    qdev_init_gpio_in_named(DEVICE(obj), tc_tiob_input, "tiob-in", TC_NCH);
    qdev_init_gpio_in_named(DEVICE(obj), tc_tclk_input, "tclk-in", TC_NCH);
    qdev_init_gpio_out_named(DEVICE(obj), s->tioa_out, "tioa-out", TC_NCH);
    qdev_init_gpio_out_named(DEVICE(obj), s->tiob_out, "tiob-out", TC_NCH);
    s->mck = qdev_init_clock_in(DEVICE(obj), "mck", tc_clock_update, s,
                                ClockPreUpdate | ClockUpdate);
}

static const Property tc_properties[] = {
    DEFINE_PROP_UINT32("mck-frequency", AT91TcState, mck_freq, 100000000),
    DEFINE_PROP_UINT32("slck-frequency", AT91TcState, slck_freq, 32768),
    DEFINE_PROP_UINT32("counter-width", AT91TcState, counter_width, 16),
};

static const VMStateDescription vmstate_at91_tc_chan = {
    .name = "at91-tc-chan",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cmr, AT91TcChan),
        VMSTATE_UINT32_V(smmr, AT91TcChan, 2),
        VMSTATE_UINT32(ra, AT91TcChan),
        VMSTATE_UINT32(rb, AT91TcChan),
        VMSTATE_UINT32(rc, AT91TcChan),
        VMSTATE_UINT32(imr, AT91TcChan),
        VMSTATE_UINT32(sr, AT91TcChan),
        VMSTATE_BOOL(clken, AT91TcChan),
        VMSTATE_INT64(epoch, AT91TcChan),
        VMSTATE_UINT32(cv_frozen, AT91TcChan),
        VMSTATE_UINT64_V(ext_ticks, AT91TcChan, 2),
        VMSTATE_UINT64_V(chain_origin, AT91TcChan, 2),
        VMSTATE_UINT64_V(event_tick, AT91TcChan, 2),
        VMSTATE_UINT64_V(last_event_tick, AT91TcChan, 2),
        VMSTATE_BOOL_V(tioa, AT91TcChan, 2),
        VMSTATE_BOOL_V(tiob, AT91TcChan, 2),
        VMSTATE_TIMER_PTR(timer, AT91TcChan),
        VMSTATE_END_OF_LIST()
    }
};

static int tc_pre_save(void *opaque)
{
    AT91TcState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int n;

    for (n = 0; n < TC_NCH; n++) {
        s->migration_ticks[n] = s->ch[n].clken ? tc_ticks(s, n, now) :
                                                 s->ch[n].ext_ticks;
    }
    return 0;
}

static int tc_post_load(void *opaque, int version_id)
{
    AT91TcState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int n;

    s->mck_freq = clock_get_hz(s->mck);

    if (version_id < 2) {
        uint64_t ticks[TC_NCH];

        for (n = 0; n < TC_NCH; n++) {
            ticks[n] = tc_ticks(s, n, now);
        }
        for (n = 0; n < TC_NCH; n++) {
            AT91TcChan *c = &s->ch[n];

            c->ext_ticks = ticks[n];
            c->epoch = now;
            c->chain_origin = ticks[0];
            c->last_event_tick = ticks[n];
            c->event_tick = c->last_event_tick;
            tc_rearm(s, n);
        }
    } else if (version_id < 3) {
        /*
         * epoch is an absolute QEMU virtual-clock timestamp, while the
         * timer deadline describes the same tick.  Reconstructing epoch from
         * the deadline is the best available compatibility path for an old
         * version-2 stream, which did not carry an explicit counter snapshot.
         */
        for (n = 0; n < TC_NCH; n++) {
            AT91TcChan *c = &s->ch[n];
            uint64_t rate = tc_rate(s, n);

            if (c->clken && rate && timer_pending(c->timer) &&
                c->event_tick >= c->ext_ticks) {
                uint64_t delta = muldiv64(c->event_tick - c->ext_ticks,
                                          NANOSECONDS_PER_SECOND, rate);

                c->epoch = (int64_t)timer_expire_time_ns(c->timer) - delta;
            }
        }
    } else {
        /*
         * Rebase each live counter snapshot onto the destination virtual
         * clock, then derive a fresh deadline.  Migrating epoch directly is
         * incorrect because it is an absolute timestamp in the source
         * process's clock domain.
         */
        for (n = 0; n < TC_NCH; n++) {
            AT91TcChan *c = &s->ch[n];

            c->ext_ticks = s->migration_ticks[n];
            c->epoch = now;
            c->chain_origin = s->migration_ticks[0];
        }
        for (n = 0; n < TC_NCH; n++) {
            tc_rearm(s, n);
        }
    }
    for (n = 0; n < TC_NCH; n++) {
        qemu_set_irq(s->tioa_out[n], s->ch[n].tioa);
        qemu_set_irq(s->tiob_out[n], s->ch[n].tiob);
    }
    tc_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_at91_tc = {
    .name = "at91-tc",
    .version_id = 3,
    .minimum_version_id = 1,
    .pre_save = tc_pre_save,
    .post_load = tc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(bmr, AT91TcState),
        VMSTATE_BOOL_ARRAY_V(tclk, AT91TcState, TC_NCH, 2),
        VMSTATE_STRUCT_ARRAY(ch, AT91TcState, TC_NCH, 1,
                             vmstate_at91_tc_chan, AT91TcChan),
        VMSTATE_CLOCK_V(mck, AT91TcState, 3),
        VMSTATE_UINT64_ARRAY_V(migration_ticks, AT91TcState, TC_NCH, 3),
        VMSTATE_END_OF_LIST()
    }
};

static void tc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tc_realize;
    device_class_set_legacy_reset(dc, tc_reset);
    device_class_set_props(dc, tc_properties);
    dc->vmsd = &vmstate_at91_tc;
}

static const TypeInfo tc_type = {
    .name = TYPE_AT91_TC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91TcState),
    .instance_init = tc_dev_init,
    .class_init = tc_class_init,
};

static void at91_tc_register_types(void)
{
    type_register_static(&tc_type);
}

type_init(at91_tc_register_types)
