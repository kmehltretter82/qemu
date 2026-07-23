/*
 * AT91SAM9G45 differential-test payload for RT-Thread 5.2.2 and 4.1.1.
 *
 * This file is copied into the external RT-Thread BSP by
 * build-rtthread-g45.sh.  Keep the output protocol line-oriented and stable:
 * the host harness and physical-board log collector both consume it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <rtthread.h>
#include <finsh.h>

#include "g45test.h"

#define G45TEST_PROTOCOL          1U
#define G45TEST_DEFAULT_SEED      0x45d0a11cU
#define G45TEST_GUARD_SIZE        32U
#define G45TEST_DATA_SIZE         1040U
#define G45TEST_TRANSFER_SIZE     1024U
#define G45TEST_WDT_MR            0xfffffd44U
#define G45TEST_WDT_MR_WDDIS      (1U << 15)
#define G45TEST_SCHEDULER_TICKS   (10U * RT_TICK_PER_SECOND)

#define G45TEST_PRINT(...)                         \
    do {                                           \
        rt_enter_critical();                       \
        rt_kprintf(__VA_ARGS__);                   \
        rt_exit_critical();                        \
    } while (0)

struct g45test_guarded_buffer {
    rt_uint8_t before[G45TEST_GUARD_SIZE];
    rt_uint8_t data[G45TEST_DATA_SIZE];
    rt_uint8_t after[G45TEST_GUARD_SIZE];
} __attribute__((aligned(32)));

static struct g45test_guarded_buffer g45test_src;
static struct g45test_guarded_buffer g45test_dst;
static rt_uint8_t g45test_expected[G45TEST_DATA_SIZE]
    __attribute__((aligned(32)));
static volatile rt_bool_t g45test_workers_run;
static volatile rt_uint32_t g45test_worker_a_count;
static volatile rt_uint32_t g45test_worker_b_count;
static volatile rt_uint32_t g45test_barrier_words[2];
static rt_bool_t g45test_watchdog_disabled;

static rt_uint32_t g45test_prng_next(rt_uint32_t *state)
{
    rt_uint32_t value = *state;

    if (value == 0U) {
        value = 0x6d2b79f5U;
    }
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    *state = value;
    return value;
}

static rt_uint32_t g45test_crc32(const rt_uint8_t *data, rt_size_t length)
{
    rt_uint32_t crc = 0xffffffffU;
    rt_size_t i;

    for (i = 0; i < length; i++) {
        unsigned int bit;

        crc ^= data[i];
        for (bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

static rt_uint8_t g45test_guard_value(unsigned int side, rt_size_t offset)
{
    return (rt_uint8_t)(0xa7U ^ (side * 0x39U) ^ (offset * 0x1dU));
}

static void g45test_reset_buffer(struct g45test_guarded_buffer *buffer,
                                 rt_uint8_t data_value)
{
    rt_size_t i;

    for (i = 0; i < G45TEST_GUARD_SIZE; i++) {
        buffer->before[i] = g45test_guard_value(0, i);
        buffer->after[i] = g45test_guard_value(1, i);
    }
    rt_memset(buffer->data, data_value, sizeof(buffer->data));
}

static rt_bool_t g45test_guards_valid(
    const struct g45test_guarded_buffer *buffer, rt_uint32_t *offset,
    rt_uint32_t *expected, rt_uint32_t *actual)
{
    rt_size_t i;

    for (i = 0; i < G45TEST_GUARD_SIZE; i++) {
        rt_uint8_t value = g45test_guard_value(0, i);

        if (buffer->before[i] != value) {
            *offset = (rt_uint32_t)i;
            *expected = value;
            *actual = buffer->before[i];
            return RT_FALSE;
        }
    }
    for (i = 0; i < G45TEST_GUARD_SIZE; i++) {
        rt_uint8_t value = g45test_guard_value(1, i);

        if (buffer->after[i] != value) {
            *offset = G45TEST_GUARD_SIZE + G45TEST_DATA_SIZE +
                      (rt_uint32_t)i;
            *expected = value;
            *actual = buffer->after[i];
            return RT_FALSE;
        }
    }
    return RT_TRUE;
}

void g45test_check(struct g45test_result *result, rt_bool_t condition,
                   rt_uint32_t expected, rt_uint32_t actual,
                   rt_uint32_t offset)
{
    result->checks++;
    if (!condition && !result->failed) {
        result->failed = RT_TRUE;
        result->failed_check = result->checks;
        result->expected = expected;
        result->actual = actual;
        result->offset = offset;
    }
}

static void g45test_check_guards(struct g45test_result *result,
                                 const struct g45test_guarded_buffer *buffer)
{
    rt_uint32_t offset = 0;
    rt_uint32_t expected = 0;
    rt_uint32_t actual = 0;
    rt_bool_t valid;

    valid = g45test_guards_valid(buffer, &offset, &expected, &actual);
    g45test_check(result, valid, expected, actual, offset);
}

static void g45test_reference_copy(rt_uint8_t *destination,
                                   const rt_uint8_t *source,
                                   rt_size_t length)
{
    rt_size_t i;

    for (i = 0; i < length; i++) {
        destination[i] = source[i];
    }
}

enum g45test_pattern {
    G45TEST_PATTERN_WALKING_ONE,
    G45TEST_PATTERN_ADDRESS,
    G45TEST_PATTERN_ALTERNATING,
    G45TEST_PATTERN_PRNG,
};

static void g45test_fill_pattern(rt_uint8_t *data, rt_size_t length,
                                 enum g45test_pattern pattern,
                                 rt_uint32_t seed)
{
    rt_uint32_t state = seed;
    rt_size_t i;

    for (i = 0; i < length; i++) {
        switch (pattern) {
        case G45TEST_PATTERN_WALKING_ONE:
            data[i] = (rt_uint8_t)(1U << (i & 7U));
            break;
        case G45TEST_PATTERN_ADDRESS:
            data[i] = (rt_uint8_t)((i * 37U) ^ (i >> 3) ^ (i >> 8));
            break;
        case G45TEST_PATTERN_ALTERNATING:
            data[i] = (i & 1U) ? 0x55U : 0xaaU;
            break;
        case G45TEST_PATTERN_PRNG:
            if ((i & 3U) == 0U) {
                state = g45test_prng_next(&state);
            }
            data[i] = (rt_uint8_t)(state >> ((i & 3U) * 8U));
            break;
        }
    }
}

static void g45test_verify_copy(struct g45test_result *result,
                                rt_size_t source_offset,
                                rt_size_t destination_offset,
                                rt_size_t length, rt_uint8_t untouched)
{
    rt_size_t i;

    for (i = 0; i < G45TEST_DATA_SIZE; i++) {
        g45test_check(result,
                      g45test_src.data[i] == g45test_expected[i],
                      g45test_expected[i], g45test_src.data[i],
                      (rt_uint32_t)i);
    }
    for (i = 0; i < length; i++) {
        rt_uint8_t expected = g45test_expected[source_offset + i];
        rt_uint8_t actual = g45test_dst.data[destination_offset + i];

        g45test_check(result, expected == actual, expected, actual,
                      (rt_uint32_t)i);
    }
    for (i = 0; i < G45TEST_DATA_SIZE; i++) {
        rt_bool_t inside = i >= destination_offset &&
                           i < destination_offset + length;

        if (!inside) {
            g45test_check(result, g45test_dst.data[i] == untouched,
                          untouched, g45test_dst.data[i], (rt_uint32_t)i);
        }
    }
    g45test_check_guards(result, &g45test_src);
    g45test_check_guards(result, &g45test_dst);
}

static void g45test_d0_patterns(struct g45test_result *result)
{
    static const char *const names[] = {
        "walking-one", "address", "alternating", "prng"
    };
    unsigned int pattern;

    for (pattern = 0; pattern < 4; pattern++) {
        rt_uint32_t crc;

        g45test_reset_buffer(&g45test_src, 0);
        g45test_reset_buffer(&g45test_dst, 0xd3U);
        g45test_fill_pattern(g45test_src.data, G45TEST_TRANSFER_SIZE,
                             (enum g45test_pattern)pattern,
                             result->seed ^ (pattern * 0x9e3779b9U));
        rt_memset(g45test_expected, 0, sizeof(g45test_expected));
        g45test_fill_pattern(g45test_expected, G45TEST_TRANSFER_SIZE,
                             (enum g45test_pattern)pattern,
                             result->seed ^ (pattern * 0x9e3779b9U));
        crc = g45test_crc32(g45test_src.data, G45TEST_TRANSFER_SIZE);
        g45test_reference_copy(g45test_dst.data, g45test_src.data,
                               G45TEST_TRANSFER_SIZE);
        g45test_verify_copy(result, 0, 0, G45TEST_TRANSFER_SIZE, 0xd3U);
        G45TEST_PRINT("G45TEST DATA case=d0.patterns pattern=%s length=%u "
                   "crc32=0x%08x\n", names[pattern],
                   G45TEST_TRANSFER_SIZE, crc);
    }
}

static void g45test_d0_boundaries(struct g45test_result *result)
{
    static const rt_uint16_t lengths[] = {
        0, 1, 2, 3, 4, 5, 7, 8, 15, 16, 31, 32, 63, 64,
        127, 128, 255, 256, 511, 512, 1023, 1024
    };
    rt_size_t length_index;
    unsigned int source_offset;
    unsigned int destination_offset;

    for (length_index = 0;
         length_index < sizeof(lengths) / sizeof(lengths[0]);
         length_index++) {
        rt_size_t length = lengths[length_index];

        for (source_offset = 0; source_offset < 4; source_offset++) {
            for (destination_offset = 0; destination_offset < 4;
                 destination_offset++) {
                rt_uint32_t vector_seed = result->seed ^
                    ((rt_uint32_t)length << 16) ^
                    (source_offset << 8) ^ destination_offset;

                g45test_reset_buffer(&g45test_src, 0x3cU);
                g45test_reset_buffer(&g45test_dst, 0xd3U);
                g45test_fill_pattern(g45test_src.data,
                                     sizeof(g45test_src.data),
                                     G45TEST_PATTERN_PRNG, vector_seed);
                g45test_fill_pattern(g45test_expected,
                                     sizeof(g45test_expected),
                                     G45TEST_PATTERN_PRNG, vector_seed);
                g45test_reference_copy(
                    g45test_dst.data + destination_offset,
                    g45test_src.data + source_offset, length);
                g45test_verify_copy(result, source_offset,
                                    destination_offset, length, 0xd3U);
            }
        }
    }
    G45TEST_PRINT("G45TEST DATA case=d0.boundaries vectors=%u max_length=%u "
               "source_offsets=4 destination_offsets=4\n",
               (unsigned int)(sizeof(lengths) / sizeof(lengths[0]) * 16U),
               G45TEST_TRANSFER_SIZE);
}

static void g45test_d0_canary(struct g45test_result *result)
{
    rt_uint32_t offset;
    rt_uint32_t expected;
    rt_uint32_t actual;
    rt_bool_t valid;

    g45test_reset_buffer(&g45test_src, 0);
    g45test_src.before[7] ^= 1U;
    valid = g45test_guards_valid(&g45test_src, &offset, &expected, &actual);
    g45test_check(result, !valid, 0, valid, 7);

    g45test_reset_buffer(&g45test_src, 0);
    g45test_src.after[11] ^= 1U;
    valid = g45test_guards_valid(&g45test_src, &offset, &expected, &actual);
    g45test_check(result, !valid, 0, valid,
                  G45TEST_GUARD_SIZE + G45TEST_DATA_SIZE + 11U);

    g45test_reset_buffer(&g45test_src, 0);
    g45test_check_guards(result, &g45test_src);
}

static void g45test_d0_prng(struct g45test_result *result)
{
    static const rt_uint8_t crc_vector[] = "123456789";
    rt_uint32_t state = 0x12345678U;
    rt_uint32_t first;
    rt_uint32_t crc;
    rt_size_t i;
    rt_bool_t differs = RT_FALSE;

    first = g45test_prng_next(&state);
    g45test_check(result, first == 0x87985aa5U,
                  0x87985aa5U, first, 0);

    crc = g45test_crc32(crc_vector, sizeof(crc_vector) - 1U);
    g45test_check(result, crc == 0xcbf43926U,
                  0xcbf43926U, crc, 0);

    g45test_fill_pattern(g45test_src.data, G45TEST_TRANSFER_SIZE,
                         G45TEST_PATTERN_PRNG, result->seed);
    g45test_fill_pattern(g45test_dst.data, G45TEST_TRANSFER_SIZE,
                         G45TEST_PATTERN_PRNG, result->seed);
    for (i = 0; i < G45TEST_TRANSFER_SIZE; i++) {
        g45test_check(result, g45test_src.data[i] == g45test_dst.data[i],
                      g45test_src.data[i], g45test_dst.data[i],
                      (rt_uint32_t)i);
    }

    g45test_fill_pattern(g45test_dst.data, G45TEST_TRANSFER_SIZE,
                         G45TEST_PATTERN_PRNG,
                         result->seed ^ 0xa5a5a5a5U);
    for (i = 0; i < G45TEST_TRANSFER_SIZE; i++) {
        if (g45test_src.data[i] != g45test_dst.data[i]) {
            differs = RT_TRUE;
            break;
        }
    }
    g45test_check(result, differs, 1, differs, (rt_uint32_t)i);
    G45TEST_PRINT("G45TEST DATA case=d0.prng seed=0x%08x crc32=0x%08x\n",
               result->seed,
               g45test_crc32(g45test_src.data, G45TEST_TRANSFER_SIZE));
}

static void g45test_d0_barrier(struct g45test_result *result)
{
    rt_uint32_t first = result->seed ^ 0x96a5c3f0U;
    rt_uint32_t second = ~first;

    g45test_barrier_words[0] = first;
    __sync_synchronize();
    g45test_barrier_words[1] = second;
    __sync_synchronize();

    g45test_check(result, g45test_barrier_words[0] == first,
                  first, g45test_barrier_words[0], 0);
    g45test_check(result, g45test_barrier_words[1] == second,
                  second, g45test_barrier_words[1], 1);
    G45TEST_PRINT("G45TEST DATA case=d0.barrier implementation=cp15-drain "
               "first=0x%08x second=0x%08x\n", first, second);
}

static void g45test_worker_a(void *parameter)
{
    while (g45test_workers_run) {
        g45test_worker_a_count++;
        rt_thread_delay(1);
    }
}

static void g45test_worker_b(void *parameter)
{
    while (g45test_workers_run) {
        g45test_worker_b_count++;
        rt_thread_delay(1);
    }
}

static void g45test_core_scheduler(struct g45test_result *result)
{
    rt_thread_t worker_a;
    rt_thread_t worker_b;
    rt_tick_t before;
    rt_tick_t elapsed;

    g45test_worker_a_count = 0;
    g45test_worker_b_count = 0;
    g45test_workers_run = RT_TRUE;
    worker_a = rt_thread_create("g45wa", g45test_worker_a, RT_NULL,
                                1024, 18, 5);
    worker_b = rt_thread_create("g45wb", g45test_worker_b, RT_NULL,
                                1024, 19, 5);
    g45test_check(result, worker_a != RT_NULL, 1,
                  worker_a != RT_NULL, 0);
    g45test_check(result, worker_b != RT_NULL, 1,
                  worker_b != RT_NULL, 1);
    if (worker_a == RT_NULL || worker_b == RT_NULL) {
        g45test_workers_run = RT_FALSE;
        return;
    }

    before = rt_tick_get();
    rt_thread_startup(worker_a);
    rt_thread_startup(worker_b);
    rt_thread_delay(G45TEST_SCHEDULER_TICKS);
    elapsed = rt_tick_get() - before;
    g45test_workers_run = RT_FALSE;
    rt_thread_delay(2);

    g45test_check(result, elapsed >= G45TEST_SCHEDULER_TICKS,
                  G45TEST_SCHEDULER_TICKS, elapsed, 2);
    g45test_check(result, g45test_worker_a_count >= 5000U,
                  5000, g45test_worker_a_count, 3);
    g45test_check(result, g45test_worker_b_count >= 5000U,
                  5000, g45test_worker_b_count, 4);
    G45TEST_PRINT("G45TEST DATA case=core.scheduler elapsed_ticks=%u "
               "worker_a=%u worker_b=%u\n", elapsed,
               g45test_worker_a_count, g45test_worker_b_count);
}

static const struct g45test_case g45test_cases[] = {
    { "d0.patterns", "d0", "D0", g45test_d0_patterns },
    { "d0.boundaries", "d0", "D0", g45test_d0_boundaries },
    { "d0.canary", "d0", "D0", g45test_d0_canary },
    { "d0.prng", "d0", "D0", g45test_d0_prng },
    { "d0.barrier", "d0", "D0", g45test_d0_barrier },
    { "d1.contract", "d1", "D1", g45test_d1_contract },
    { "d1.mem2mem", "d1", "D1", g45test_d1_mem2mem },
    { "d1.channels", "d1", "D1", g45test_d1_channels },
    { "d1.linked-list", "d1", "D1", g45test_d1_linked_list },
    { "d1.suspend-resume", "d1", "D1", g45test_d1_suspend_resume },
    { "d1.software-requests", "d1", "D1", g45test_d1_software_requests },
    { "d1.software-last", "d1", "D1", g45test_d1_software_last },
    { "d1.word-width-alias", "d1", "D1",
      g45test_d1_word_width_alias },
    { "d1.bus-error", "d1", "D1", g45test_d1_bus_error },
    { "d1.stop-on-done", "d1", "D1", g45test_d1_stop_on_done },
    { "d1.partial-descriptor-reload", "d1", "D1",
      g45test_d1_partial_descriptor_reload },
    { "d1.picture-in-picture", "d1", "D1",
      g45test_d1_picture_in_picture },
    { "d1.mixed-software-requests", "d1", "D1",
      g45test_d1_mixed_software_requests },
    { "d1.irq", "d1", "D1", g45test_d1_irq },
    { "d1.irq-mask", "d1", "D1", g45test_d1_irq_mask },
    { "core.scheduler", "core", "CORE", g45test_core_scheduler },
};

struct g45test_totals {
    rt_uint32_t passed;
    rt_uint32_t failed;
    rt_uint32_t skipped;
};

static void g45test_run_case(const struct g45test_case *test,
                             rt_uint32_t seed,
                             struct g45test_totals *totals)
{
    struct g45test_result result;
    rt_tick_t elapsed;

    rt_memset(&result, 0, sizeof(result));
    result.name = test->name;
    result.seed = seed;
    result.started = rt_tick_get();
    G45TEST_PRINT("G45TEST BEGIN %s seed=0x%08x phase=%s\n",
               test->name, seed, test->phase);
    test->run(&result);
    elapsed = rt_tick_get() - result.started;
    if (result.failed) {
        totals->failed++;
        G45TEST_PRINT("G45TEST FAIL %s check=%u expected=0x%08x "
                   "actual=0x%08x offset=%u checks=%u elapsed_ticks=%u\n",
                   test->name, result.failed_check, result.expected,
                   result.actual, result.offset, result.checks, elapsed);
    } else {
        totals->passed++;
        G45TEST_PRINT("G45TEST PASS %s checks=%u elapsed_ticks=%u\n",
                   test->name, result.checks, elapsed);
    }
}

static rt_bool_t g45test_parse_seed(const char *text, rt_uint32_t *seed)
{
    rt_uint32_t value = 0;
    unsigned int base = 10;
    unsigned int digit_count = 0;

    if (text == RT_NULL || *text == '\0') {
        return RT_FALSE;
    }
    if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
        base = 16;
        text += 2;
    }
    while (*text != '\0') {
        unsigned int digit;

        if (*text >= '0' && *text <= '9') {
            digit = (unsigned int)(*text - '0');
        } else if (*text >= 'a' && *text <= 'f') {
            digit = (unsigned int)(*text - 'a') + 10U;
        } else if (*text >= 'A' && *text <= 'F') {
            digit = (unsigned int)(*text - 'A') + 10U;
        } else {
            return RT_FALSE;
        }
        if (digit >= base) {
            return RT_FALSE;
        }
        value = value * base + digit;
        digit_count++;
        text++;
    }
    if (digit_count == 0U) {
        return RT_FALSE;
    }
    *seed = value;
    return RT_TRUE;
}

static void g45test_list(void)
{
    rt_size_t i;

    for (i = 0; i < sizeof(g45test_cases) / sizeof(g45test_cases[0]); i++) {
        G45TEST_PRINT("G45TEST CASE name=%s suite=%s phase=%s destructive=0\n",
                   g45test_cases[i].name, g45test_cases[i].suite,
                   g45test_cases[i].phase);
    }
    G45TEST_PRINT("G45TEST LIST count=%u protocol=%u\n",
               (unsigned int)(sizeof(g45test_cases) /
                              sizeof(g45test_cases[0])),
               G45TEST_PROTOCOL);
}

static void g45test_status(void)
{
    rt_uint32_t watchdog_mr =
        *(volatile rt_uint32_t *)G45TEST_WDT_MR;

    G45TEST_PRINT("G45TEST STATUS protocol=%u tick=%u tick_hz=%u "
               "watchdog_disabled=%u watchdog_mr=0x%08x\n",
               G45TEST_PROTOCOL, rt_tick_get(), RT_TICK_PER_SECOND,
               g45test_watchdog_disabled, watchdog_mr);
}

static int g45test_run_selection(const char *selection, rt_uint32_t seed)
{
    struct g45test_totals totals = { 0, 0, 0 };
    rt_bool_t found = RT_FALSE;
    rt_size_t i;

    for (i = 0; i < sizeof(g45test_cases) / sizeof(g45test_cases[0]); i++) {
        const struct g45test_case *test = &g45test_cases[i];

        if (rt_strcmp(selection, "all") == 0 ||
            rt_strcmp(selection, test->suite) == 0 ||
            rt_strcmp(selection, test->name) == 0) {
            found = RT_TRUE;
            g45test_run_case(test, seed, &totals);
        }
    }
    if (!found) {
        G45TEST_PRINT("G45TEST ERROR reason=unknown-selection selection=%s\n",
                   selection);
        return -RT_ERROR;
    }
    G45TEST_PRINT("G45TEST END suite=%s passed=%u failed=%u skipped=%u "
               "seed=0x%08x\n", selection, totals.passed, totals.failed,
               totals.skipped, seed);
    return totals.failed == 0U ? RT_EOK : -RT_ERROR;
}

static int g45test_command(int argc, char **argv)
{
    rt_uint32_t seed = G45TEST_DEFAULT_SEED;

    if (argc == 1 || (argc == 2 && rt_strcmp(argv[1], "status") == 0)) {
        g45test_status();
        return RT_EOK;
    }
    if (argc == 2 && rt_strcmp(argv[1], "list") == 0) {
        g45test_list();
        return RT_EOK;
    }
    if (rt_strcmp(argv[1], "all") == 0) {
        if (argc == 3 && !g45test_parse_seed(argv[2], &seed)) {
            G45TEST_PRINT("G45TEST ERROR reason=bad-seed value=%s\n", argv[2]);
            return -RT_ERROR;
        }
        if (argc > 3) {
            G45TEST_PRINT("G45TEST ERROR reason=usage\n");
            return -RT_ERROR;
        }
        return g45test_run_selection("all", seed);
    }
    if (rt_strcmp(argv[1], "run") == 0 && (argc == 3 || argc == 4)) {
        if (argc == 4 && !g45test_parse_seed(argv[3], &seed)) {
            G45TEST_PRINT("G45TEST ERROR reason=bad-seed value=%s\n", argv[3]);
            return -RT_ERROR;
        }
        return g45test_run_selection(argv[2], seed);
    }

    G45TEST_PRINT("G45TEST ERROR reason=usage usage=\"g45test "
               "list|status|run_SELECTION_[SEED]|all_[SEED]\"\n");
    return -RT_ERROR;
}
MSH_CMD_EXPORT_ALIAS(g45test_command, g45test,
                     run AT91SAM9G45 differential tests);

static int g45test_board_init(void)
{
    volatile rt_uint32_t *watchdog_mr =
        (volatile rt_uint32_t *)G45TEST_WDT_MR;

    *watchdog_mr = G45TEST_WDT_MR_WDDIS;
    g45test_watchdog_disabled =
        ((*watchdog_mr & G45TEST_WDT_MR_WDDIS) != 0U);
    return RT_EOK;
}
INIT_BOARD_EXPORT(g45test_board_init);

static int g45test_ready(void)
{
    G45TEST_PRINT("G45TEST READY protocol=%u tick_hz=%u "
               "watchdog_disabled=%u\n", G45TEST_PROTOCOL,
               RT_TICK_PER_SECOND, g45test_watchdog_disabled);
    return RT_EOK;
}
INIT_APP_EXPORT(g45test_ready);
