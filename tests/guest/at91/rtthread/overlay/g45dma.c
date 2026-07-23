/*
 * Direct AT91SAM9G45 HDMAC tests for RT-Thread.
 *
 * These tests deliberately use the silicon register interface rather than a
 * QEMU-only escape hatch.  The same binary protocol and byte oracles can be
 * used on a SAM9M10-G45-EK to distinguish emulator defects from BSP defects.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <rtthread.h>
#include <rthw.h>

#include "g45test.h"

#define G45_DMAC_BASE                  0xffffec00U
#define G45_PMC_PCER                   0xfffffc10U
#define G45_DMAC_IRQ                   21

#define DMAC_GCFG                      0x000U
#define DMAC_EN                        0x004U
#define DMAC_SREQ                      0x008U
#define DMAC_CREQ                      0x00cU
#define DMAC_LAST                      0x010U
#define DMAC_EBCIER                    0x018U
#define DMAC_EBCIDR                    0x01cU
#define DMAC_EBCIMR                    0x020U
#define DMAC_EBCISR                    0x024U
#define DMAC_CHER                      0x028U
#define DMAC_CHDR                      0x02cU
#define DMAC_CHSR                      0x030U
#define DMAC_CH_BASE                   0x03cU
#define DMAC_CH_STRIDE                 0x028U
#define DMAC_SADDR                     0x000U
#define DMAC_DADDR                     0x004U
#define DMAC_DSCR                      0x008U
#define DMAC_CTRLA                     0x00cU
#define DMAC_CTRLB                     0x010U
#define DMAC_CFG                       0x014U
#define DMAC_SPIP                      0x018U
#define DMAC_DPIP                      0x01cU

#define DMAC_ENABLE                    (1U << 0)
#define DMAC_BTC(channel)              (1U << (channel))
#define DMAC_CBTC(channel)             (1U << (8U + (channel)))
#define DMAC_ERR(channel)              (1U << (16U + (channel)))
#define DMAC_ALL_EVENTS                0x00ffffffU
#define DMAC_ALL_CHANNELS              0x000000ffU
#define DMAC_ALL_EMPTY                 0x00ff0000U
#define DMAC_CHANNEL_ENABLE(channel)   (1U << (channel))
#define DMAC_CHANNEL_SUSPEND(channel)  (1U << (8U + (channel)))
#define DMAC_CHANNEL_EMPTY(channel)    (1U << (16U + (channel)))
#define DMAC_CHANNEL_RESUME(channel)   (1U << (8U + (channel)))
#define DMAC_SOURCE_REQUEST(channel)   (1U << (2U * (channel)))
#define DMAC_DESTINATION_REQUEST(channel) (1U << (1U + 2U * (channel)))
#define DMAC_SOURCE_LAST(channel)      (1U << (2U * (channel)))

#define DMAC_CTRLA_BTSIZE(value)       ((value) & 0xffffU)
#define DMAC_CTRLA_SCSIZE(value)       (((value) & 7U) << 16)
#define DMAC_CTRLA_DCSIZE(value)       (((value) & 7U) << 20)
#define DMAC_CTRLA_SRC_WIDTH(value)    (((value) & 3U) << 24)
#define DMAC_CTRLA_DST_WIDTH(value)    (((value) & 3U) << 28)
#define DMAC_CTRLA_DONE                (1U << 31)
#define DMAC_CTRLB_SRC_PIP             (1U << 8)
#define DMAC_CTRLB_DST_PIP             (1U << 12)
#define DMAC_CTRLB_SRC_DSCR_DISABLE    (1U << 16)
#define DMAC_CTRLB_DST_DSCR_DISABLE    (1U << 20)
#define DMAC_CTRLB_FC(value)           (((value) & 7U) << 21)
#define DMAC_CTRLB_SRC_MODE(value)     (((value) & 3U) << 24)
#define DMAC_CTRLB_DST_MODE(value)     (((value) & 3U) << 28)
#define DMAC_CTRLB_IEN                 (1U << 30)
#define DMAC_CFG_SOD                   (1U << 16)
#define DMAC_PIP(hole, boundary)       \
    (((hole) & 0xffffU) | (((boundary) & 0x3ffU) << 16))

#define DMAC_FC_PERIPHERAL_TO_MEMORY_DMA         2U
#define DMAC_FC_PERIPHERAL_TO_PERIPHERAL_DMA     3U
#define DMAC_FC_PERIPHERAL_TO_MEMORY_PERIPHERAL  4U

#define DMAC_MODE_INCREMENT            0U
#define DMAC_MODE_DECREMENT            1U
#define DMAC_MODE_FIXED                2U

#define G45_DMA_GUARD_SIZE             32U
#define G45_DMA_DATA_SIZE              2048U
#define G45_DMA_CHANNEL_BYTES          128U
#define G45_DMA_LLI_COUNT              4U
#define G45_DMA_TIMEOUT_TICKS          RT_TICK_PER_SECOND

extern void mmu_clean_dcache(rt_uint32_t buffer, rt_uint32_t size);
extern void mmu_invalidate_dcache(rt_uint32_t buffer, rt_uint32_t size);
extern void mmu_clean_invalidated_dcache(rt_uint32_t buffer,
                                         rt_uint32_t size);

struct g45_dma_guarded_buffer {
    rt_uint8_t before[G45_DMA_GUARD_SIZE];
    rt_uint8_t data[G45_DMA_DATA_SIZE];
    rt_uint8_t after[G45_DMA_GUARD_SIZE];
} __attribute__((aligned(32)));

struct g45_dma_lli {
    rt_uint32_t saddr;
    rt_uint32_t daddr;
    rt_uint32_t ctrla;
    rt_uint32_t ctrlb;
    rt_uint32_t dscr;
};

static struct g45_dma_guarded_buffer dma_source;
static struct g45_dma_guarded_buffer dma_destination;
static rt_uint8_t dma_expected[G45_DMA_DATA_SIZE]
    __attribute__((aligned(32)));
static rt_uint8_t dma_channel_source[8][G45_DMA_CHANNEL_BYTES]
    __attribute__((aligned(32)));
static rt_uint8_t dma_channel_destination[8][G45_DMA_CHANNEL_BYTES]
    __attribute__((aligned(32)));
static struct g45_dma_lli dma_lli[G45_DMA_LLI_COUNT]
    __attribute__((aligned(32)));
static volatile rt_uint32_t dma_irq_count;
static volatile rt_uint32_t dma_irq_status;

static volatile rt_uint32_t *g45_dma_reg(rt_uint32_t offset)
{
    return (volatile rt_uint32_t *)(G45_DMAC_BASE + offset);
}

static rt_uint32_t g45_dma_read(rt_uint32_t offset)
{
    return *g45_dma_reg(offset);
}

static void g45_dma_write(rt_uint32_t offset, rt_uint32_t value)
{
    *g45_dma_reg(offset) = value;
}

static rt_uint32_t g45_dma_channel_reg(unsigned int channel,
                                       rt_uint32_t offset)
{
    return DMAC_CH_BASE + channel * DMAC_CH_STRIDE + offset;
}

static rt_uint32_t g45_dma_physical(const void *pointer)
{
    rt_uint32_t address = (rt_uint32_t)(rt_ubase_t)pointer;

    /* The BSP's uncached 0xf0000000 alias maps physical DDR at 0x70000000. */
    if (address >= 0xf0000000U && address < 0xf8000000U) {
        address -= 0x80000000U;
    }
    return address;
}

static rt_uint8_t g45_dma_pattern(rt_uint32_t seed, rt_size_t offset)
{
    rt_uint32_t value = seed ^ ((rt_uint32_t)offset * 0x9e3779b9U);

    value ^= value >> 16;
    value *= 0x7feb352dU;
    value ^= value >> 15;
    return (rt_uint8_t)(value ^ (value >> 8) ^ (value >> 24));
}

static rt_uint8_t g45_dma_guard(unsigned int side, rt_size_t offset)
{
    return (rt_uint8_t)(0x69U ^ (side * 0x87U) ^ (offset * 0x35U));
}

static void g45_dma_reset_buffers(rt_uint32_t seed)
{
    rt_size_t i;

    for (i = 0; i < G45_DMA_GUARD_SIZE; i++) {
        dma_source.before[i] = g45_dma_guard(0, i);
        dma_source.after[i] = g45_dma_guard(1, i);
        dma_destination.before[i] = g45_dma_guard(0, i);
        dma_destination.after[i] = g45_dma_guard(1, i);
    }
    for (i = 0; i < G45_DMA_DATA_SIZE; i++) {
        dma_source.data[i] = g45_dma_pattern(seed, i);
        dma_destination.data[i] = 0xd3U;
        dma_expected[i] = 0xd3U;
    }
}

static void g45_dma_check_guards(struct g45test_result *result,
                                 const struct g45_dma_guarded_buffer *buffer,
                                 rt_uint32_t base)
{
    rt_size_t i;

    for (i = 0; i < G45_DMA_GUARD_SIZE; i++) {
        rt_uint8_t expected = g45_dma_guard(0, i);

        g45test_check(result, buffer->before[i] == expected, expected,
                      buffer->before[i], base + (rt_uint32_t)i);
    }
    for (i = 0; i < G45_DMA_GUARD_SIZE; i++) {
        rt_uint8_t expected = g45_dma_guard(1, i);

        g45test_check(result, buffer->after[i] == expected, expected,
                      buffer->after[i], base + G45_DMA_GUARD_SIZE +
                      G45_DMA_DATA_SIZE + (rt_uint32_t)i);
    }
}

static void g45_dma_enable_clock(void)
{
    *(volatile rt_uint32_t *)G45_PMC_PCER = 1U << G45_DMAC_IRQ;
}

static void g45_dma_quiesce(void)
{
    g45_dma_write(DMAC_EBCIDR, DMAC_ALL_EVENTS);
    g45_dma_write(DMAC_CHDR, DMAC_ALL_CHANNELS);
    g45_dma_write(DMAC_EN, 0);
    (void)g45_dma_read(DMAC_EBCISR);
}

static rt_bool_t g45_dma_wait_channels(rt_uint32_t mask)
{
    rt_tick_t deadline = rt_tick_get() + G45_DMA_TIMEOUT_TICKS;

    while ((g45_dma_read(DMAC_CHSR) & mask) != 0U) {
        if ((rt_int32_t)(rt_tick_get() - deadline) >= 0) {
            return RT_FALSE;
        }
        rt_thread_yield();
    }
    return RT_TRUE;
}

static rt_bool_t g45_dma_wait_clear(rt_uint32_t offset, rt_uint32_t mask)
{
    rt_tick_t deadline = rt_tick_get() + G45_DMA_TIMEOUT_TICKS;

    while ((g45_dma_read(offset) & mask) != 0U) {
        if ((rt_int32_t)(rt_tick_get() - deadline) >= 0) {
            return RT_FALSE;
        }
        rt_thread_yield();
    }
    return RT_TRUE;
}

static void g45_dma_program(unsigned int channel, rt_uint32_t source,
                            rt_uint32_t destination, rt_uint32_t descriptor,
                            rt_uint32_t ctrla, rt_uint32_t ctrlb,
                            rt_uint32_t cfg)
{
    g45_dma_write(g45_dma_channel_reg(channel, DMAC_SADDR), source);
    g45_dma_write(g45_dma_channel_reg(channel, DMAC_DADDR), destination);
    g45_dma_write(g45_dma_channel_reg(channel, DMAC_DSCR), descriptor);
    g45_dma_write(g45_dma_channel_reg(channel, DMAC_CTRLA), ctrla);
    g45_dma_write(g45_dma_channel_reg(channel, DMAC_CTRLB), ctrlb);
    g45_dma_write(g45_dma_channel_reg(channel, DMAC_CFG), 0x01000000U | cfg);
}

static int g45_dma_address_step(unsigned int mode, unsigned int width)
{
    if (mode == DMAC_MODE_INCREMENT) {
        return (int)width;
    }
    if (mode == DMAC_MODE_DECREMENT) {
        return -(int)width;
    }
    return 0;
}

static void g45_dma_oracle(rt_size_t source_offset,
                           rt_size_t destination_offset,
                           rt_size_t total_bytes, unsigned int source_width,
                           unsigned int destination_width,
                           unsigned int source_mode,
                           unsigned int destination_mode)
{
    rt_uint8_t fifo[8];
    rt_size_t source_transfers = total_bytes / source_width;
    int source = (int)source_offset;
    int destination = (int)destination_offset;
    unsigned int fill = 0;
    rt_size_t transfer;

    for (transfer = 0; transfer < source_transfers; transfer++) {
        rt_memcpy(fifo + fill, dma_source.data + source, source_width);
        fill += source_width;
        source += g45_dma_address_step(source_mode, source_width);

        while (fill >= destination_width) {
            rt_memcpy(dma_expected + destination, fifo, destination_width);
            destination += g45_dma_address_step(destination_mode,
                                                destination_width);
            fill -= destination_width;
            if (fill != 0U) {
                rt_memmove(fifo, fifo + destination_width, fill);
            }
        }
    }
}

static void g45_dma_run_vector(struct g45test_result *result,
                               unsigned int vector, unsigned int channel,
                               rt_size_t total_bytes,
                               unsigned int source_width_code,
                               unsigned int destination_width_code,
                               unsigned int source_mode,
                               unsigned int destination_mode,
                               unsigned int source_chunk,
                               unsigned int destination_chunk)
{
    unsigned int source_width = 1U << source_width_code;
    unsigned int destination_width = 1U << destination_width_code;
    rt_size_t source_offset = 128U;
    rt_size_t destination_offset = 128U;
    rt_uint32_t ctrla;
    rt_uint32_t ctrlb;
    rt_uint32_t status;
    rt_size_t i;

    if (source_mode == DMAC_MODE_DECREMENT) {
        source_offset += total_bytes - source_width;
    }
    if (destination_mode == DMAC_MODE_DECREMENT) {
        destination_offset += total_bytes - destination_width;
    }

    g45_dma_reset_buffers(result->seed ^ (vector * 0x6d2b79f5U));
    g45_dma_oracle(source_offset, destination_offset, total_bytes,
                   source_width, destination_width, source_mode,
                   destination_mode);

    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    __sync_synchronize();

    ctrla = DMAC_CTRLA_BTSIZE(total_bytes / source_width) |
            DMAC_CTRLA_SCSIZE(source_chunk) |
            DMAC_CTRLA_DCSIZE(destination_chunk) |
            DMAC_CTRLA_SRC_WIDTH(source_width_code) |
            DMAC_CTRLA_DST_WIDTH(destination_width_code);
    ctrlb = DMAC_CTRLB_SRC_DSCR_DISABLE |
            DMAC_CTRLB_DST_DSCR_DISABLE |
            DMAC_CTRLB_SRC_MODE(source_mode) |
            DMAC_CTRLB_DST_MODE(destination_mode);

    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_program(channel,
                    g45_dma_physical(dma_source.data + source_offset),
                    g45_dma_physical(dma_destination.data +
                                     destination_offset),
                    0, ctrla, ctrlb, 0);
    g45_dma_write(DMAC_CHER, 1U << channel);
    g45test_check(result, g45_dma_wait_channels(1U << channel),
                  1, 0, vector);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, (status & DMAC_BTC(channel)) != 0U,
                  DMAC_BTC(channel), status, vector);
    g45test_check(result, (status & DMAC_ERR(channel)) == 0U,
                  0, status & DMAC_ERR(channel), vector);

    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    for (i = 0; i < G45_DMA_DATA_SIZE; i++) {
        g45test_check(result, dma_destination.data[i] == dma_expected[i],
                      dma_expected[i], dma_destination.data[i],
                      vector * G45_DMA_DATA_SIZE + (rt_uint32_t)i);
    }
    g45_dma_check_guards(result, &dma_source, 0x10000000U + vector * 0x10000U);
    g45_dma_check_guards(result, &dma_destination,
                         0x20000000U + vector * 0x10000U);
}

void g45test_d1_contract(struct g45test_result *result)
{
    rt_uint32_t status;
    rt_uint32_t before;
    unsigned int channel;

    g45_dma_enable_clock();
    g45_dma_quiesce();

    g45test_check(result, g45_dma_read(DMAC_GCFG) == 0x10U,
                  0x10U, g45_dma_read(DMAC_GCFG), 0);
    g45test_check(result, g45_dma_read(DMAC_EN) == 0U,
                  0, g45_dma_read(DMAC_EN), 1);
    g45test_check(result, g45_dma_read(DMAC_EBCIMR) == 0U,
                  0, g45_dma_read(DMAC_EBCIMR), 2);
    g45test_check(result, g45_dma_read(DMAC_CHSR) == DMAC_ALL_EMPTY,
                  DMAC_ALL_EMPTY, g45_dma_read(DMAC_CHSR), 3);
    for (channel = 0; channel < 8; channel++) {
        status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_CFG));
        g45test_check(result, status == 0x01000000U,
                      0x01000000U, status, 4U + channel);
    }

    g45_dma_write(DMAC_GCFG, 0xffffffffU);
    status = g45_dma_read(DMAC_GCFG);
    g45test_check(result, status == 0x10U, 0x10U, status, 12);
    g45_dma_write(DMAC_GCFG, 0x10U);

    g45_dma_reset_buffers(result->seed);
    before = *(rt_uint32_t *)(dma_destination.data + 128U);
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    g45_dma_program(0, g45_dma_physical(dma_source.data + 128U),
                    g45_dma_physical(dma_destination.data + 128U), 0,
                    DMAC_CTRLA_BTSIZE(32) |
                    DMAC_CTRLA_SRC_WIDTH(0) |
                    DMAC_CTRLA_DST_WIDTH(0),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE, 0);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, 1U);
    rt_thread_delay(2);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result,
                  *(rt_uint32_t *)(dma_destination.data + 128U) == before,
                  before, *(rt_uint32_t *)(dma_destination.data + 128U), 13);
    g45test_check(result, (status & DMAC_BTC(0)) == 0U,
                  0, status & DMAC_BTC(0), 14);

    g45_dma_write(DMAC_CHDR, 1U);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, 1U);
    g45test_check(result, g45_dma_wait_channels(1U), 1, 0, 15);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, (status & DMAC_BTC(0)) != 0U,
                  DMAC_BTC(0), status, 16);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    g45test_check(
        result,
        *(rt_uint32_t *)(dma_destination.data + 128U) ==
        *(rt_uint32_t *)(dma_source.data + 128U),
        *(rt_uint32_t *)(dma_source.data + 128U),
        *(rt_uint32_t *)(dma_destination.data + 128U), 17);

    g45_dma_quiesce();
}

void g45test_d1_mem2mem(struct g45test_result *result)
{
    static const struct {
        rt_uint16_t bytes;
        rt_uint8_t source_width;
        rt_uint8_t destination_width;
        rt_uint8_t source_mode;
        rt_uint8_t destination_mode;
        rt_uint8_t source_chunk;
        rt_uint8_t destination_chunk;
    } vectors[] = {
        { 1,    0, 0, 0, 0, 0, 0 },
        { 2,    0, 0, 0, 0, 1, 1 },
        { 31,   0, 0, 0, 0, 2, 3 },
        { 128,  0, 0, 0, 0, 7, 7 },
        { 2,    1, 1, 0, 0, 0, 0 },
        { 64,   1, 1, 0, 0, 4, 5 },
        { 4,    2, 2, 0, 0, 0, 0 },
        { 256,  2, 2, 0, 0, 6, 7 },
        { 256,  2, 0, 0, 0, 3, 3 },
        { 256,  1, 0, 0, 0, 2, 4 },
        { 256,  0, 1, 0, 0, 1, 5 },
        { 256,  0, 2, 0, 0, 7, 6 },
        { 128,  0, 0, 1, 0, 0, 0 },
        { 128,  1, 1, 1, 0, 1, 1 },
        { 128,  2, 2, 1, 0, 2, 2 },
        { 128,  0, 0, 0, 1, 3, 3 },
        { 128,  1, 1, 0, 1, 4, 4 },
        { 128,  2, 2, 0, 1, 5, 5 },
        { 128,  0, 0, 2, 0, 6, 6 },
        { 128,  1, 1, 2, 0, 7, 7 },
        { 128,  2, 2, 2, 0, 0, 7 },
        { 128,  0, 0, 0, 2, 7, 0 },
        { 128,  1, 1, 0, 2, 6, 1 },
        { 128,  2, 2, 0, 2, 5, 2 },
    };
    rt_size_t i;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    for (i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++) {
        g45_dma_run_vector(result, (unsigned int)i, (unsigned int)(i & 7U),
                           vectors[i].bytes, vectors[i].source_width,
                           vectors[i].destination_width,
                           vectors[i].source_mode,
                           vectors[i].destination_mode,
                           vectors[i].source_chunk,
                           vectors[i].destination_chunk);
    }
    g45_dma_quiesce();
}

void g45test_d1_channels(struct g45test_result *result)
{
    rt_uint32_t status;
    unsigned int channel;
    unsigned int byte;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    for (channel = 0; channel < 8; channel++) {
        for (byte = 0; byte < G45_DMA_CHANNEL_BYTES; byte++) {
            dma_channel_source[channel][byte] =
                g45_dma_pattern(result->seed ^ (channel * 0x10203U), byte);
            dma_channel_destination[channel][byte] = 0xc7U;
        }
        g45_dma_program(channel,
                        g45_dma_physical(dma_channel_source[channel]),
                        g45_dma_physical(dma_channel_destination[channel]),
                        0, DMAC_CTRLA_BTSIZE(G45_DMA_CHANNEL_BYTES) |
                        DMAC_CTRLA_SRC_WIDTH(0) |
                        DMAC_CTRLA_DST_WIDTH(0),
                        DMAC_CTRLB_SRC_DSCR_DISABLE |
                        DMAC_CTRLB_DST_DSCR_DISABLE, 0);
    }
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)dma_channel_source,
                     sizeof(dma_channel_source));
    mmu_clean_invalidated_dcache(
        (rt_uint32_t)(rt_ubase_t)dma_channel_destination,
        sizeof(dma_channel_destination));
    __sync_synchronize();
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    g45_dma_write(DMAC_CHER, DMAC_ALL_CHANNELS);
    g45test_check(result, g45_dma_wait_channels(DMAC_ALL_CHANNELS),
                  1, 0, 0);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, (status & DMAC_ALL_CHANNELS) == DMAC_ALL_CHANNELS,
                  DMAC_ALL_CHANNELS, status & DMAC_ALL_CHANNELS, 1);
    g45test_check(result,
                  (g45_dma_read(DMAC_CHSR) & DMAC_ALL_EMPTY) ==
                  DMAC_ALL_EMPTY, DMAC_ALL_EMPTY,
                  g45_dma_read(DMAC_CHSR) & DMAC_ALL_EMPTY, 2);

    mmu_invalidate_dcache(
        (rt_uint32_t)(rt_ubase_t)dma_channel_destination,
        sizeof(dma_channel_destination));
    __sync_synchronize();
    for (channel = 0; channel < 8; channel++) {
        for (byte = 0; byte < G45_DMA_CHANNEL_BYTES; byte++) {
            g45test_check(result,
                          dma_channel_destination[channel][byte] ==
                          dma_channel_source[channel][byte],
                          dma_channel_source[channel][byte],
                          dma_channel_destination[channel][byte],
                          channel * G45_DMA_CHANNEL_BYTES + byte);
        }
    }
    g45_dma_quiesce();
}

void g45test_d1_linked_list(struct g45test_result *result)
{
    const unsigned int channel = 3;
    const unsigned int bytes = 64;
    rt_uint32_t status;
    unsigned int descriptor;
    unsigned int byte;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    for (descriptor = 0; descriptor < G45_DMA_LLI_COUNT; descriptor++) {
        rt_uint32_t source_offset = descriptor * bytes;
        rt_uint32_t destination_offset = 512U + descriptor * bytes;

        for (byte = 0; byte < bytes; byte++) {
            dma_source.data[source_offset + byte] =
                g45_dma_pattern(result->seed ^ 0x11ddccaaU,
                                source_offset + byte);
            dma_destination.data[destination_offset + byte] = 0x5aU;
        }
        dma_lli[descriptor].saddr =
            g45_dma_physical(dma_source.data + source_offset);
        dma_lli[descriptor].daddr =
            g45_dma_physical(dma_destination.data + destination_offset);
        dma_lli[descriptor].ctrla = DMAC_CTRLA_BTSIZE(bytes) |
            DMAC_CTRLA_SRC_WIDTH(0) | DMAC_CTRLA_DST_WIDTH(0);
        dma_lli[descriptor].ctrlb = DMAC_CTRLB_IEN;
        dma_lli[descriptor].dscr = descriptor + 1U < G45_DMA_LLI_COUNT ?
            g45_dma_physical(&dma_lli[descriptor + 1U]) : 0;
    }
    dma_lli[G45_DMA_LLI_COUNT - 1U].ctrlb =
        DMAC_CTRLB_SRC_DSCR_DISABLE | DMAC_CTRLB_DST_DSCR_DISABLE;

    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)dma_lli,
                                 sizeof(dma_lli));
    __sync_synchronize();
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_program(channel, 0, 0, g45_dma_physical(dma_lli), 0, 0, 0);
    g45_dma_write(DMAC_CHER, 1U << channel);
    g45test_check(result, g45_dma_wait_channels(1U << channel), 1, 0, 0);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result,
                  (status & (DMAC_BTC(channel) | DMAC_CBTC(channel))) ==
                  (DMAC_BTC(channel) | DMAC_CBTC(channel)),
                  DMAC_BTC(channel) | DMAC_CBTC(channel), status, 1);

    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)dma_lli,
                          sizeof(dma_lli));
    __sync_synchronize();
    for (descriptor = 0; descriptor < G45_DMA_LLI_COUNT; descriptor++) {
        rt_uint32_t source_offset = descriptor * bytes;
        rt_uint32_t destination_offset = 512U + descriptor * bytes;

        for (byte = 0; byte < bytes; byte++) {
            g45test_check(result,
                          dma_destination.data[destination_offset + byte] ==
                          dma_source.data[source_offset + byte],
                          dma_source.data[source_offset + byte],
                          dma_destination.data[destination_offset + byte],
                          descriptor * bytes + byte);
        }
        g45test_check(result,
                      (dma_lli[descriptor].ctrla &
                       (DMAC_CTRLA_DONE | 0xffffU)) == DMAC_CTRLA_DONE,
                      DMAC_CTRLA_DONE,
                      dma_lli[descriptor].ctrla &
                      (DMAC_CTRLA_DONE | 0xffffU),
                      0x1000U + descriptor);
    }
    g45test_check(result,
                  g45_dma_read(g45_dma_channel_reg(channel, DMAC_DSCR)) == 0U,
                  0, g45_dma_read(g45_dma_channel_reg(channel, DMAC_DSCR)),
                  0x2000U);
    g45_dma_quiesce();
}

void g45test_d1_suspend_resume(struct g45test_result *result)
{
    const unsigned int channel = 2;
    const rt_uint32_t channel_bit = DMAC_CHANNEL_ENABLE(channel);
    rt_uint32_t status;
    rt_uint32_t before;
    unsigned int byte;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_reset_buffers(result->seed ^ 0x5a5a3c3cU);
    before = *(rt_uint32_t *)(dma_destination.data + 128U);
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    __sync_synchronize();

    g45_dma_program(channel,
                    g45_dma_physical(dma_source.data + 128U),
                    g45_dma_physical(dma_destination.data + 128U),
                    0, DMAC_CTRLA_BTSIZE(4),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE, 0);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);

    /* ENA and SUSP in one write must expose an enabled, frozen, empty
     * channel without moving even the first byte. */
    g45_dma_write(DMAC_CHER, channel_bit |
                  DMAC_CHANNEL_SUSPEND(channel));
    rt_thread_delay(2);
    status = g45_dma_read(DMAC_CHSR);
    g45test_check(result,
                  (status & (channel_bit |
                             DMAC_CHANNEL_SUSPEND(channel) |
                             DMAC_CHANNEL_EMPTY(channel))) ==
                  (channel_bit | DMAC_CHANNEL_SUSPEND(channel) |
                   DMAC_CHANNEL_EMPTY(channel)),
                  channel_bit | DMAC_CHANNEL_SUSPEND(channel) |
                  DMAC_CHANNEL_EMPTY(channel), status, 0);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    g45test_check(result,
                  *(rt_uint32_t *)(dma_destination.data + 128U) == before,
                  before,
                  *(rt_uint32_t *)(dma_destination.data + 128U), 1);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, (status & DMAC_BTC(channel)) == 0U,
                  0, status & DMAC_BTC(channel), 2);

    g45_dma_write(DMAC_CHDR, DMAC_CHANNEL_RESUME(channel));
    g45test_check(result, g45_dma_wait_channels(channel_bit), 1, 0, 3);
    status = g45_dma_read(DMAC_CHSR);
    g45test_check(result,
                  (status & (channel_bit |
                             DMAC_CHANNEL_SUSPEND(channel) |
                             DMAC_CHANNEL_EMPTY(channel))) ==
                  DMAC_CHANNEL_EMPTY(channel),
                  DMAC_CHANNEL_EMPTY(channel), status, 4);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, (status & DMAC_BTC(channel)) != 0U,
                  DMAC_BTC(channel), status, 5);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    for (byte = 0; byte < 4; byte++) {
        g45test_check(result,
                      dma_destination.data[128U + byte] ==
                      dma_source.data[128U + byte],
                      dma_source.data[128U + byte],
                      dma_destination.data[128U + byte], 6U + byte);
    }
    g45_dma_quiesce();
}

void g45test_d1_software_requests(struct g45test_result *result)
{
    const unsigned int channel = 1;
    const rt_uint32_t request = DMAC_SOURCE_REQUEST(channel);
    const rt_uint32_t source_address =
        g45_dma_physical(dma_source.data + 256U);
    const rt_uint32_t destination_address =
        g45_dma_physical(dma_destination.data + 256U);
    rt_uint32_t *source = (rt_uint32_t *)(dma_source.data + 256U);
    rt_uint32_t *destination =
        (rt_uint32_t *)(dma_destination.data + 256U);
    rt_uint32_t status;
    unsigned int word;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_reset_buffers(result->seed ^ 0x1ad0c0deU);
    for (word = 0; word < 8; word++) {
        source[word] = 0x11110000U + word;
        destination[word] = 0xdeadbeefU;
    }
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    __sync_synchronize();

    g45_dma_program(channel, source_address, destination_address, 0,
                    DMAC_CTRLA_BTSIZE(8) |
                    DMAC_CTRLA_SCSIZE(1) |
                    DMAC_CTRLA_SRC_WIDTH(2) |
                    DMAC_CTRLA_DST_WIDTH(2),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE |
                    DMAC_CTRLB_FC(DMAC_FC_PERIPHERAL_TO_MEMORY_DMA), 0);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, DMAC_CHANNEL_ENABLE(channel));
    rt_thread_delay(2);
    status = g45_dma_read(DMAC_CHSR);
    g45test_check(result,
                  (status & DMAC_CHANNEL_ENABLE(channel)) != 0U,
                  DMAC_CHANNEL_ENABLE(channel), status, 0);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    g45test_check(result, destination[0] == 0xdeadbeefU,
                  0xdeadbeefU, destination[0], 1);

    g45_dma_write(DMAC_SREQ, request);
    g45test_check(result, g45_dma_wait_clear(DMAC_SREQ, request), 1, 0, 2);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    g45test_check(result, destination[0] == source[0],
                  source[0], destination[0], 3);
    g45test_check(result, destination[1] == 0xdeadbeefU,
                  0xdeadbeefU, destination[1], 4);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_SADDR));
    g45test_check(result, status == source_address + 4U,
                  source_address + 4U, status, 5);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_DADDR));
    g45test_check(result, status == destination_address + 4U,
                  destination_address + 4U, status, 6);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_CTRLA));
    g45test_check(result, (status & 0xffffU) == 7U, 7, status & 0xffffU, 7);

    g45_dma_write(DMAC_CREQ, request);
    g45test_check(result, g45_dma_wait_clear(DMAC_CREQ, request), 1, 0, 8);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    for (word = 0; word < 5; word++) {
        g45test_check(result, destination[word] == source[word],
                      source[word], destination[word], 9U + word);
    }
    g45test_check(result, destination[5] == 0xdeadbeefU,
                  0xdeadbeefU, destination[5], 14);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_CTRLA));
    g45test_check(result, (status & 0xffffU) == 3U, 3, status & 0xffffU, 15);

    g45_dma_write(DMAC_CREQ, request);
    g45test_check(result,
                  g45_dma_wait_channels(DMAC_CHANNEL_ENABLE(channel)),
                  1, 0, 16);
    g45test_check(result, g45_dma_read(DMAC_CREQ) == 0U,
                  0, g45_dma_read(DMAC_CREQ), 17);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    for (word = 0; word < 8; word++) {
        g45test_check(result, destination[word] == source[word],
                      source[word], destination[word], 18U + word);
    }
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, status == DMAC_BTC(channel),
                  DMAC_BTC(channel), status, 26);
    g45_dma_quiesce();
}

void g45test_d1_software_last(struct g45test_result *result)
{
    const unsigned int channel = 0;
    const rt_uint32_t request = DMAC_SOURCE_REQUEST(channel);
    rt_uint32_t *source = (rt_uint32_t *)(dma_source.data + 384U);
    rt_uint32_t *destination =
        (rt_uint32_t *)(dma_destination.data + 384U);
    rt_uint32_t status;
    unsigned int word;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_reset_buffers(result->seed ^ 0x1a57f10fU);
    for (word = 0; word < 8; word++) {
        source[word] = 0x22220000U + word;
        destination[word] = 0xdeadbeefU;
    }
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    __sync_synchronize();

    g45_dma_program(channel, g45_dma_physical(source),
                    g45_dma_physical(destination), 0,
                    DMAC_CTRLA_BTSIZE(8) |
                    DMAC_CTRLA_SCSIZE(1) |
                    DMAC_CTRLA_SRC_WIDTH(2) |
                    DMAC_CTRLA_DST_WIDTH(2),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE |
                    DMAC_CTRLB_FC(
                        DMAC_FC_PERIPHERAL_TO_MEMORY_PERIPHERAL), 0);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, DMAC_CHANNEL_ENABLE(channel));
    g45_dma_write(DMAC_SREQ, request);
    g45test_check(result, g45_dma_wait_clear(DMAC_SREQ, request), 1, 0, 0);

    g45_dma_write(DMAC_LAST, DMAC_SOURCE_LAST(channel));
    status = g45_dma_read(DMAC_LAST);
    g45test_check(result, status == DMAC_SOURCE_LAST(channel),
                  DMAC_SOURCE_LAST(channel), status, 1);
    g45_dma_write(DMAC_CREQ, request);
    g45test_check(result,
                  g45_dma_wait_channels(DMAC_CHANNEL_ENABLE(channel)),
                  1, 0, 2);
    g45test_check(result, g45_dma_read(DMAC_LAST) == 0U,
                  0, g45_dma_read(DMAC_LAST), 3);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    for (word = 0; word < 5; word++) {
        g45test_check(result, destination[word] == source[word],
                      source[word], destination[word], 4U + word);
    }
    for (; word < 8; word++) {
        g45test_check(result, destination[word] == 0xdeadbeefU,
                      0xdeadbeefU, destination[word], 4U + word);
    }
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, status == DMAC_BTC(channel),
                  DMAC_BTC(channel), status, 12);
    g45_dma_quiesce();
}

void g45test_d1_word_width_alias(struct g45test_result *result)
{
    const unsigned int channel = 4;
    rt_uint32_t *source = (rt_uint32_t *)(dma_source.data + 512U);
    rt_uint32_t *destination =
        (rt_uint32_t *)(dma_destination.data + 512U);
    rt_uint32_t source_address;
    rt_uint32_t destination_address;
    rt_uint32_t status;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_reset_buffers(result->seed ^ 0x0bad7137U);
    source[0] = 0x44332211U;
    source[1] = 0xa5a55a5aU;
    destination[0] = 0xdeadbeefU;
    destination[1] = 0xc001d00dU;
    source_address = g45_dma_physical(source);
    destination_address = g45_dma_physical(destination);
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    __sync_synchronize();

    /* Width 11b is the documented alias of WORD, not an 8-byte access. */
    g45_dma_program(channel, source_address, destination_address, 0,
                    DMAC_CTRLA_BTSIZE(1) |
                    DMAC_CTRLA_SRC_WIDTH(3) |
                    DMAC_CTRLA_DST_WIDTH(3),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE, 0);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, DMAC_CHANNEL_ENABLE(channel));
    g45test_check(result,
                  g45_dma_wait_channels(DMAC_CHANNEL_ENABLE(channel)),
                  1, 0, 0);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    g45test_check(result, destination[0] == source[0],
                  source[0], destination[0], 1);
    g45test_check(result, destination[1] == 0xc001d00dU,
                  0xc001d00dU, destination[1], 2);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_SADDR));
    g45test_check(result, status == source_address + 4U,
                  source_address + 4U, status, 3);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_DADDR));
    g45test_check(result, status == destination_address + 4U,
                  destination_address + 4U, status, 4);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, status == DMAC_BTC(channel),
                  DMAC_BTC(channel), status, 5);
    g45_dma_check_guards(result, &dma_destination, 0x31000000U);
    g45_dma_quiesce();
}

void g45test_d1_bus_error(struct g45test_result *result)
{
    const unsigned int channel = 6;
    const rt_uint32_t channel_bit = DMAC_CHANNEL_ENABLE(channel);
    const rt_uint32_t invalid_address = 0x90000000U;
    rt_uint32_t *source = (rt_uint32_t *)(dma_source.data + 640U);
    rt_uint32_t *destination =
        (rt_uint32_t *)(dma_destination.data + 640U);
    rt_uint32_t source_address;
    rt_uint32_t destination_address;
    rt_uint32_t status;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_reset_buffers(result->seed ^ 0xe7700badU);
    source[0] = 0xa5a55a5aU;
    destination[0] = 0xdeadbeefU;
    source_address = g45_dma_physical(source);
    destination_address = g45_dma_physical(destination);
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    __sync_synchronize();

    /* 0x90000000 is in the datasheet's undefined/abort address window. */
    g45_dma_program(channel, invalid_address, destination_address, 0,
                    DMAC_CTRLA_BTSIZE(4),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE, 0);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, channel_bit);
    g45test_check(result, g45_dma_wait_channels(channel_bit), 1, 0, 0);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, status == DMAC_ERR(channel),
                  DMAC_ERR(channel), status, 1);
    status = g45_dma_read(DMAC_CHSR);
    g45test_check(result, (status & channel_bit) == 0U,
                  0, status & channel_bit, 2);
    g45test_check(result,
                  (status & DMAC_CHANNEL_EMPTY(channel)) != 0U,
                  DMAC_CHANNEL_EMPTY(channel), status, 3);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_SADDR));
    g45test_check(result, status == invalid_address,
                  invalid_address, status, 4);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_DADDR));
    g45test_check(result, status == destination_address,
                  destination_address, status, 5);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_CTRLA));
    g45test_check(result, (status & 0xffffU) == 4U,
                  4, status & 0xffffU, 6);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    g45test_check(result, destination[0] == 0xdeadbeefU,
                  0xdeadbeefU, destination[0], 7);

    /* Reprogramming the failed channel must make it usable immediately. */
    g45_dma_program(channel, source_address, destination_address, 0,
                    DMAC_CTRLA_BTSIZE(4),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE, 0);
    g45_dma_write(DMAC_CHER, channel_bit);
    g45test_check(result, g45_dma_wait_channels(channel_bit), 1, 0, 8);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, status == DMAC_BTC(channel),
                  DMAC_BTC(channel), status, 9);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    g45test_check(result, destination[0] == source[0],
                  source[0], destination[0], 10);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_CTRLA));
    g45test_check(result, (status & 0xffffU) == 0U,
                  0, status & 0xffffU, 11);
    g45_dma_check_guards(result, &dma_source, 0x32000000U);
    g45_dma_check_guards(result, &dma_destination, 0x33000000U);
    g45_dma_quiesce();
}

void g45test_d1_stop_on_done(struct g45test_result *result)
{
    const unsigned int channel = 5;
    const rt_uint32_t channel_bit = DMAC_CHANNEL_ENABLE(channel);
    rt_uint32_t *source = (rt_uint32_t *)(dma_source.data + 768U);
    rt_uint32_t *destination =
        (rt_uint32_t *)(dma_destination.data + 768U);
    rt_uint32_t descriptor_address;
    rt_uint32_t descriptor_ctrla;
    rt_uint32_t status;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_reset_buffers(result->seed ^ 0x50d00e11U);
    source[0] = 0x13579bdfU;
    destination[0] = 0xdeadbeefU;
    descriptor_ctrla = DMAC_CTRLA_BTSIZE(4) | DMAC_CTRLA_DONE;
    dma_lli[0].saddr = g45_dma_physical(source);
    dma_lli[0].daddr = g45_dma_physical(destination);
    dma_lli[0].ctrla = descriptor_ctrla;
    dma_lli[0].ctrlb = 0;
    dma_lli[0].dscr = 0;
    descriptor_address = g45_dma_physical(&dma_lli[0]);
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_lli, sizeof(dma_lli));
    __sync_synchronize();

    g45_dma_program(channel, 0, 0, descriptor_address, 0, 0,
                    DMAC_CFG_SOD);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, channel_bit);
    g45test_check(result, g45_dma_wait_channels(channel_bit), 1, 0, 0);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    g45test_check(result, destination[0] == 0xdeadbeefU,
                  0xdeadbeefU, destination[0], 1);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_CTRLA));
    g45test_check(result, status == descriptor_ctrla,
                  descriptor_ctrla, status, 2);
    status = g45_dma_read(DMAC_CHSR);
    g45test_check(result, (status & channel_bit) == 0U,
                  0, status & channel_bit, 3);
    g45test_check(result,
                  (status & DMAC_CHANNEL_EMPTY(channel)) != 0U,
                  DMAC_CHANNEL_EMPTY(channel), status, 4);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, status == 0U, 0, status, 5);

    /* SOD clear deliberately ignores DONE and executes the same descriptor. */
    g45_dma_program(channel, 0, 0, descriptor_address, 0, 0, 0);
    g45_dma_write(DMAC_CHER, channel_bit);
    g45test_check(result, g45_dma_wait_channels(channel_bit), 1, 0, 6);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result,
                  status == (DMAC_BTC(channel) | DMAC_CBTC(channel)),
                  DMAC_BTC(channel) | DMAC_CBTC(channel), status, 7);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_lli,
                          sizeof(dma_lli));
    __sync_synchronize();
    g45test_check(result, destination[0] == source[0],
                  source[0], destination[0], 8);
    status = dma_lli[0].ctrla;
    g45test_check(result, (status & (DMAC_CTRLA_DONE | 0xffffU)) ==
                  DMAC_CTRLA_DONE, DMAC_CTRLA_DONE,
                  status & (DMAC_CTRLA_DONE | 0xffffU), 9);
    g45_dma_check_guards(result, &dma_source, 0x34000000U);
    g45_dma_check_guards(result, &dma_destination, 0x35000000U);
    g45_dma_quiesce();
}

void g45test_d1_partial_descriptor_reload(struct g45test_result *result)
{
    const rt_uint32_t channels = DMAC_CHANNEL_ENABLE(0) |
                                 DMAC_CHANNEL_ENABLE(1);
    rt_uint32_t *source = (rt_uint32_t *)(dma_source.data + 1024U);
    rt_uint32_t *destination =
        (rt_uint32_t *)(dma_destination.data + 1024U);
    rt_uint32_t *poison_source =
        (rt_uint32_t *)(dma_source.data + 1200U);
    rt_uint32_t *poison_destination =
        (rt_uint32_t *)(dma_destination.data + 1200U);
    rt_uint32_t expected_events;
    rt_uint32_t status;
    unsigned int descriptor;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_reset_buffers(result->seed ^ 0xd35c120dU);
    source[0] = 0x11111111U;
    source[1] = 0x22222222U;
    source[2] = 0x33333333U;
    source[3] = 0x44444444U;
    poison_source[0] = 0xbad00badU;
    destination[0] = 0xdeadbeefU;
    destination[1] = 0xdeadbeefU;
    destination[2] = 0xdeadbeefU;
    destination[3] = 0xdeadbeefU;
    poison_destination[0] = 0xa5a5a5a5U;

    dma_lli[0].saddr = g45_dma_physical(&source[0]);
    dma_lli[0].daddr = g45_dma_physical(&destination[0]);
    dma_lli[0].ctrla = DMAC_CTRLA_BTSIZE(4);
    dma_lli[0].ctrlb = DMAC_CTRLB_DST_DSCR_DISABLE;
    dma_lli[0].dscr = g45_dma_physical(&dma_lli[1]);
    dma_lli[1].saddr = g45_dma_physical(&source[1]);
    dma_lli[1].daddr = g45_dma_physical(poison_destination);
    dma_lli[1].ctrla = DMAC_CTRLA_BTSIZE(4);
    dma_lli[1].ctrlb = DMAC_CTRLB_SRC_DSCR_DISABLE |
                       DMAC_CTRLB_DST_DSCR_DISABLE;
    dma_lli[1].dscr = 0;

    dma_lli[2].saddr = g45_dma_physical(&source[2]);
    dma_lli[2].daddr = g45_dma_physical(&destination[2]);
    dma_lli[2].ctrla = DMAC_CTRLA_BTSIZE(4);
    dma_lli[2].ctrlb = DMAC_CTRLB_SRC_DSCR_DISABLE;
    dma_lli[2].dscr = g45_dma_physical(&dma_lli[3]);
    dma_lli[3].saddr = g45_dma_physical(poison_source);
    dma_lli[3].daddr = g45_dma_physical(&destination[3]);
    dma_lli[3].ctrla = DMAC_CTRLA_BTSIZE(4);
    dma_lli[3].ctrlb = DMAC_CTRLB_SRC_DSCR_DISABLE |
                       DMAC_CTRLB_DST_DSCR_DISABLE;
    dma_lli[3].dscr = 0;

    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_lli, sizeof(dma_lli));
    __sync_synchronize();
    g45_dma_program(0, 0, 0, g45_dma_physical(&dma_lli[0]), 0, 0, 0);
    g45_dma_program(1, 0, 0, g45_dma_physical(&dma_lli[2]), 0, 0, 0);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, channels);
    g45test_check(result, g45_dma_wait_channels(channels), 1, 0, 0);

    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_lli,
                          sizeof(dma_lli));
    __sync_synchronize();
    for (descriptor = 0; descriptor < 4; descriptor++) {
        g45test_check(result, destination[descriptor] == source[descriptor],
                      source[descriptor], destination[descriptor],
                      1U + descriptor);
        status = dma_lli[descriptor].ctrla;
        g45test_check(result,
                      (status & (DMAC_CTRLA_DONE | 0xffffU)) ==
                      DMAC_CTRLA_DONE, DMAC_CTRLA_DONE,
                      status & (DMAC_CTRLA_DONE | 0xffffU),
                      5U + descriptor);
    }
    g45test_check(result, poison_destination[0] == 0xa5a5a5a5U,
                  0xa5a5a5a5U, poison_destination[0], 9);
    expected_events = DMAC_BTC(0) | DMAC_BTC(1) |
                      DMAC_CBTC(0) | DMAC_CBTC(1);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, status == expected_events,
                  expected_events, status, 10);
    g45_dma_check_guards(result, &dma_source, 0x36000000U);
    g45_dma_check_guards(result, &dma_destination, 0x37000000U);
    g45_dma_quiesce();
}

void g45test_d1_picture_in_picture(struct g45test_result *result)
{
    static const rt_uint32_t expected[] = {
        0x11111111U, 0x22222222U, 0xdeadbeefU, 0xdeadbeefU,
        0x33333333U, 0x44444444U, 0xdeadbeefU, 0xdeadbeefU,
    };
    const unsigned int channel = 4;
    const rt_uint32_t channel_bit = DMAC_CHANNEL_ENABLE(channel);
    const rt_uint32_t spip = DMAC_PIP(2, 2);
    const rt_uint32_t dpip = DMAC_PIP(3, 2);
    rt_uint32_t *source = (rt_uint32_t *)(dma_source.data + 1728U);
    rt_uint32_t *destination =
        (rt_uint32_t *)(dma_destination.data + 1728U);
    rt_uint32_t status;
    unsigned int i;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_reset_buffers(result->seed ^ 0x91c7f00dU);
    source[0] = 0x11111111U;
    source[1] = 0x22222222U;
    source[2] = 0xfeedfaceU;
    source[3] = 0x33333333U;
    source[4] = 0x44444444U;
    source[5] = 0xbad00badU;
    source[6] = 0xcafef00dU;
    source[7] = 0x5a5aa5a5U;
    for (i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        destination[i] = 0xdeadbeefU;
    }

    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    __sync_synchronize();
    g45_dma_program(channel, g45_dma_physical(source),
                    g45_dma_physical(destination), 0,
                    DMAC_CTRLA_BTSIZE(4) |
                    DMAC_CTRLA_SRC_WIDTH(2) |
                    DMAC_CTRLA_DST_WIDTH(2),
                    DMAC_CTRLB_SRC_PIP | DMAC_CTRLB_DST_PIP |
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE, 0);
    g45_dma_write(g45_dma_channel_reg(channel, DMAC_SPIP), spip);
    g45_dma_write(g45_dma_channel_reg(channel, DMAC_DPIP), dpip);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_SPIP));
    g45test_check(result, status == spip, spip, status, 0);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_DPIP));
    g45test_check(result, status == dpip, dpip, status, 1);

    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, channel_bit);
    g45test_check(result, g45_dma_wait_channels(channel_bit), 1, 0, 2);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    for (i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        g45test_check(result, destination[i] == expected[i], expected[i],
                      destination[i], 3U + i);
    }
    g45test_check(result, source[2] == 0xfeedfaceU,
                  0xfeedfaceU, source[2], 11);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, status == DMAC_BTC(channel),
                  DMAC_BTC(channel), status, 12);
    g45_dma_check_guards(result, &dma_source, 0x3c000000U);
    g45_dma_check_guards(result, &dma_destination, 0x3d000000U);
    g45_dma_quiesce();
}

void g45test_d1_mixed_software_requests(struct g45test_result *result)
{
    const unsigned int channel = 6;
    const rt_uint32_t channel_bit = DMAC_CHANNEL_ENABLE(channel);
    const rt_uint32_t source_request = DMAC_SOURCE_REQUEST(channel);
    const rt_uint32_t destination_request =
        DMAC_DESTINATION_REQUEST(channel);
    rt_uint32_t *source = (rt_uint32_t *)(dma_source.data + 1408U);
    rt_uint32_t *destination =
        (rt_uint32_t *)(dma_destination.data + 1408U);
    rt_uint32_t status;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    g45_dma_reset_buffers(result->seed ^ 0x50f7faceU);
    source[0] = 0x12345678U;
    source[1] = 0x89abcdefU;
    destination[0] = 0xdeadbeefU;
    destination[1] = 0xdeadbeefU;
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    __sync_synchronize();
    g45_dma_program(channel, g45_dma_physical(source),
                    g45_dma_physical(destination), 0,
                    DMAC_CTRLA_BTSIZE(2) |
                    DMAC_CTRLA_SRC_WIDTH(2) |
                    DMAC_CTRLA_DST_WIDTH(2),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE |
                    DMAC_CTRLB_FC(
                        DMAC_FC_PERIPHERAL_TO_PERIPHERAL_DMA), 0);

    /* The source request predates both global and channel enable. */
    g45_dma_write(DMAC_SREQ, source_request);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, channel_bit);
    g45test_check(result, g45_dma_read(DMAC_SREQ) == source_request,
                  source_request, g45_dma_read(DMAC_SREQ), 0);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    g45test_check(result, destination[0] == 0xdeadbeefU,
                  0xdeadbeefU, destination[0], 1);

    g45_dma_write(DMAC_SREQ, destination_request);
    g45test_check(result,
                  g45_dma_wait_clear(DMAC_SREQ,
                                      source_request | destination_request),
                  1, 0, 2);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    g45test_check(result, destination[0] == source[0],
                  source[0], destination[0], 3);
    g45test_check(result, destination[1] == 0xdeadbeefU,
                  0xdeadbeefU, destination[1], 4);
    status = g45_dma_read(g45_dma_channel_reg(channel, DMAC_CTRLA));
    g45test_check(result, (status & 0xffffU) == 1U,
                  1, status & 0xffffU, 5);

    /* Reverse the source/destination request order for the final word. */
    g45_dma_write(DMAC_SREQ, destination_request);
    g45test_check(result, g45_dma_read(DMAC_SREQ) == destination_request,
                  destination_request, g45_dma_read(DMAC_SREQ), 6);
    g45_dma_write(DMAC_SREQ, source_request);
    g45test_check(result,
                  g45_dma_wait_clear(DMAC_SREQ,
                                      source_request | destination_request),
                  1, 0, 7);
    g45test_check(result, g45_dma_wait_channels(channel_bit), 1, 0, 8);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    g45test_check(result, destination[1] == source[1],
                  source[1], destination[1], 9);
    status = g45_dma_read(DMAC_EBCISR);
    g45test_check(result, status == DMAC_BTC(channel),
                  DMAC_BTC(channel), status, 10);
    g45_dma_check_guards(result, &dma_source, 0x38000000U);
    g45_dma_check_guards(result, &dma_destination, 0x39000000U);
    g45_dma_quiesce();
}

static void g45_dma_irq_handler(int vector, void *parameter)
{
    dma_irq_count++;
    dma_irq_status |= g45_dma_read(DMAC_EBCISR);
}

void g45test_d1_irq(struct g45test_result *result)
{
    rt_isr_handler_t old_handler;
    rt_tick_t deadline;
    rt_uint32_t status;
    const unsigned int channel = 7;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    dma_irq_count = 0;
    dma_irq_status = 0;
    rt_hw_interrupt_mask(G45_DMAC_IRQ);
    old_handler = rt_hw_interrupt_install(G45_DMAC_IRQ,
                                           g45_dma_irq_handler, RT_NULL,
                                           "g45dma");

    g45_dma_reset_buffers(result->seed ^ 0x55aa33ccU);
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    g45_dma_program(channel, g45_dma_physical(dma_source.data + 64U),
                    g45_dma_physical(dma_destination.data + 64U), 0,
                    DMAC_CTRLA_BTSIZE(64) | DMAC_CTRLA_SRC_WIDTH(0) |
                    DMAC_CTRLA_DST_WIDTH(0),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE, 0);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    g45_dma_write(DMAC_EBCIER, DMAC_BTC(channel));
    g45test_check(result,
                  (g45_dma_read(DMAC_EBCIMR) & DMAC_BTC(channel)) != 0U,
                  DMAC_BTC(channel), g45_dma_read(DMAC_EBCIMR), 0);
    rt_hw_interrupt_umask(G45_DMAC_IRQ);
    g45_dma_write(DMAC_CHER, 1U << channel);

    deadline = rt_tick_get() + G45_DMA_TIMEOUT_TICKS;
    while (dma_irq_count == 0U &&
           (rt_int32_t)(rt_tick_get() - deadline) < 0) {
        rt_thread_yield();
    }
    g45test_check(result, dma_irq_count == 1U, 1, dma_irq_count, 1);
    g45test_check(result,
                  (dma_irq_status & DMAC_BTC(channel)) != 0U,
                  DMAC_BTC(channel), dma_irq_status, 2);
    g45test_check(result, (dma_irq_status & DMAC_ERR(channel)) == 0U,
                  0, dma_irq_status & DMAC_ERR(channel), 3);
    g45test_check(result, g45_dma_read(DMAC_EBCISR) == 0U,
                  0, g45_dma_read(DMAC_EBCISR), 4);

    g45_dma_write(DMAC_EBCIDR, DMAC_BTC(channel));
    status = g45_dma_read(DMAC_EBCIMR);
    g45test_check(result, (status & DMAC_BTC(channel)) == 0U,
                  0, status & DMAC_BTC(channel), 5);
    rt_hw_interrupt_mask(G45_DMAC_IRQ);
    g45_dma_quiesce();
    if (old_handler != RT_NULL) {
        rt_hw_interrupt_install(G45_DMAC_IRQ, old_handler, RT_NULL,
                                "restored");
    }
}

void g45test_d1_irq_mask(struct g45test_result *result)
{
    rt_isr_handler_t old_handler;
    rt_tick_t deadline;
    const unsigned int channel = 7;
    const rt_uint32_t channel_bit = DMAC_CHANNEL_ENABLE(channel);
    rt_uint32_t status;

    g45_dma_enable_clock();
    g45_dma_quiesce();
    dma_irq_count = 0;
    dma_irq_status = 0;
    rt_hw_interrupt_mask(G45_DMAC_IRQ);
    old_handler = rt_hw_interrupt_install(G45_DMAC_IRQ,
                                           g45_dma_irq_handler, RT_NULL,
                                           "g45mask");
    g45_dma_reset_buffers(result->seed ^ 0x1a7e1a7eU);
    mmu_clean_dcache((rt_uint32_t)(rt_ubase_t)&dma_source,
                     sizeof(dma_source));
    mmu_clean_invalidated_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                                 sizeof(dma_destination));
    g45_dma_program(channel, g45_dma_physical(dma_source.data + 1600U),
                    g45_dma_physical(dma_destination.data + 1600U), 0,
                    DMAC_CTRLA_BTSIZE(32),
                    DMAC_CTRLB_SRC_DSCR_DISABLE |
                    DMAC_CTRLB_DST_DSCR_DISABLE, 0);
    g45_dma_write(DMAC_EN, DMAC_ENABLE);
    (void)g45_dma_read(DMAC_EBCISR);
    g45_dma_write(DMAC_CHER, channel_bit);
    g45test_check(result, g45_dma_wait_channels(channel_bit), 1, 0, 0);
    g45test_check(result, dma_irq_count == 0U, 0, dma_irq_count, 1);

    /* Preserve the pending status while its device and AIC masks change. */
    g45_dma_write(DMAC_EBCIER, DMAC_BTC(channel));
    g45test_check(result, dma_irq_count == 0U, 0, dma_irq_count, 2);
    status = g45_dma_read(DMAC_EBCIMR);
    g45test_check(result, (status & DMAC_BTC(channel)) != 0U,
                  DMAC_BTC(channel), status, 3);
    g45_dma_write(DMAC_EBCIDR, DMAC_BTC(channel));
    rt_hw_interrupt_umask(G45_DMAC_IRQ);
    rt_thread_yield();
    g45test_check(result, dma_irq_count == 0U, 0, dma_irq_count, 4);

    g45_dma_write(DMAC_EBCIER, DMAC_BTC(channel));
    deadline = rt_tick_get() + G45_DMA_TIMEOUT_TICKS;
    while (dma_irq_count == 0U &&
           (rt_int32_t)(rt_tick_get() - deadline) < 0) {
        rt_thread_yield();
    }
    g45test_check(result, dma_irq_count == 1U, 1, dma_irq_count, 5);
    g45test_check(result, dma_irq_status == DMAC_BTC(channel),
                  DMAC_BTC(channel), dma_irq_status, 6);
    g45test_check(result, g45_dma_read(DMAC_EBCISR) == 0U,
                  0, g45_dma_read(DMAC_EBCISR), 7);
    mmu_invalidate_dcache((rt_uint32_t)(rt_ubase_t)&dma_destination,
                          sizeof(dma_destination));
    __sync_synchronize();
    g45test_check(result,
                  rt_memcmp(dma_source.data + 1600U,
                            dma_destination.data + 1600U, 32) == 0,
                  0, 1, 8);
    g45_dma_check_guards(result, &dma_source, 0x3a000000U);
    g45_dma_check_guards(result, &dma_destination, 0x3b000000U);

    g45_dma_write(DMAC_EBCIDR, DMAC_BTC(channel));
    rt_hw_interrupt_mask(G45_DMAC_IRQ);
    g45_dma_quiesce();
    if (old_handler != RT_NULL) {
        rt_hw_interrupt_install(G45_DMAC_IRQ, old_handler, RT_NULL,
                                "restored");
    }
}
