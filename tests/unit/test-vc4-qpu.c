/*
 * Unit tests for the VideoCore IV QPU interpreter (hw/display/vc4_qpu.c).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The instruction encoder here is deliberately independent of the decoder
 * under test: it builds the 64-bit words from the field positions in
 * drivers/gpu/drm/vc4/vc4_qpu_defines.h, so a decode mistake cannot cancel
 * out against an encode mistake.
 */

#include "qemu/osdep.h"
#include "hw/display/vc4_qpu.h"

/* Signals. */
#define SIG_NONE            1
#define SIG_PROG_END        3
#define SIG_LOAD_TMU0       10
#define SIG_SMALL_IMM       13
#define SIG_LOAD_IMM        14
#define SIG_BRANCH          15

/* Add ALU. */
#define A_NOP   0
#define A_FADD  1
#define A_FSUB  2
#define A_FMIN  3
#define A_FMAX  4
#define A_FTOI  7
#define A_ITOF  8
#define A_ADD   12
#define A_SUB   13
#define A_SHR   14
#define A_ASR   15
#define A_ROR   16
#define A_SHL   17
#define A_MIN   18
#define A_MAX   19
#define A_AND   20
#define A_OR    21
#define A_XOR   22
#define A_NOT   23
#define A_CLZ   24
#define A_V8ADDS 30
#define A_V8SUBS 31

/* Mul ALU. */
#define M_NOP    0
#define M_FMUL   1
#define M_MUL24  2
#define M_V8MIN  4
#define M_V8MAX  5

/* Muxes. */
#define MUX_R0  0
#define MUX_R1  1
#define MUX_R2  2
#define MUX_R4  4
#define MUX_A   6
#define MUX_B   7

/* Conditions. */
#define COND_NEVER  0
#define COND_ALWAYS 1
#define COND_ZS     2
#define COND_ZC     3
#define COND_NS     4
#define COND_CS     6

/* Branch conditions. */
#define BCOND_ALL_ZS  0
#define BCOND_ALL_ZC  1
#define BCOND_ALWAYS  15

/* Register addresses. */
#define R_UNIF      32
#define R_ELEM_QPU  38
#define R_NOP       39
#define W_ACC0      32
#define W_ACC1      33
#define W_ACC2      34
#define W_ACC5      37
#define W_HOST_INT  38
#define W_NOP       39
#define W_VPM       48
#define W_TMU0_S    56

/* Load-immediate variants. */
#define LI_32               0
#define LI_PER_ELEM_SIGNED  1
#define LI_PER_ELEM_UNSIGNED 3

typedef struct AluInst {
    unsigned sig;
    unsigned unpack;
    unsigned pack;
    bool pm;
    unsigned cond_add;
    unsigned cond_mul;
    bool sf;
    bool ws;
    unsigned waddr_add;
    unsigned waddr_mul;
    unsigned op_mul;
    unsigned op_add;
    unsigned raddr_a;
    unsigned raddr_b;
    unsigned add_a;
    unsigned add_b;
    unsigned mul_a;
    unsigned mul_b;
} AluInst;

/* Field defaults that make an instruction a well-formed no-op. */
#define ALU_DEFAULTS                                                    \
    .sig = SIG_NONE, .cond_add = COND_ALWAYS, .cond_mul = COND_ALWAYS,  \
    .waddr_add = W_NOP, .waddr_mul = W_NOP,                             \
    .raddr_a = R_NOP, .raddr_b = R_NOP

static uint64_t alu(AluInst i)
{
    return ((uint64_t)i.sig << 60) |
           ((uint64_t)i.unpack << 57) |
           ((uint64_t)(i.pm ? 1 : 0) << 56) |
           ((uint64_t)i.pack << 52) |
           ((uint64_t)i.cond_add << 49) |
           ((uint64_t)i.cond_mul << 46) |
           ((uint64_t)(i.sf ? 1 : 0) << 45) |
           ((uint64_t)(i.ws ? 1 : 0) << 44) |
           ((uint64_t)i.waddr_add << 38) |
           ((uint64_t)i.waddr_mul << 32) |
           ((uint64_t)i.op_mul << 29) |
           ((uint64_t)i.op_add << 24) |
           ((uint64_t)i.raddr_a << 18) |
           ((uint64_t)i.raddr_b << 12) |
           ((uint64_t)i.add_a << 9) |
           ((uint64_t)i.add_b << 6) |
           ((uint64_t)i.mul_a << 3) |
           ((uint64_t)i.mul_b);
}

/* A load-immediate instruction: same layout, but the low 32 bits are the
 * immediate and the unpack field selects the variant. */
static uint64_t load_imm(unsigned variant, uint32_t imm, unsigned waddr_add,
                         unsigned waddr_mul, bool sf)
{
    return ((uint64_t)SIG_LOAD_IMM << 60) |
           ((uint64_t)variant << 57) |
           ((uint64_t)COND_ALWAYS << 49) |
           ((uint64_t)COND_ALWAYS << 46) |
           ((uint64_t)(sf ? 1 : 0) << 45) |
           ((uint64_t)waddr_add << 38) |
           ((uint64_t)waddr_mul << 32) |
           imm;
}

static uint64_t branch(unsigned cond, bool rel, int32_t imm,
                       unsigned waddr_add)
{
    return ((uint64_t)SIG_BRANCH << 60) |
           ((uint64_t)cond << 52) |
           ((uint64_t)(rel ? 1 : 0) << 51) |
           ((uint64_t)waddr_add << 38) |
           ((uint64_t)W_NOP << 32) |
           (uint32_t)imm;
}

/* An immediate into r0, for setting up operands. */
static uint64_t set_r0(uint32_t v)
{
    return load_imm(LI_32, v, W_ACC0, W_NOP, false);
}

static uint64_t set_r1(uint32_t v)
{
    return load_imm(LI_32, v, W_ACC1, W_NOP, false);
}

static uint64_t nop_inst(void)
{
    return alu((AluInst){ ALU_DEFAULTS });
}

static uint64_t prog_end(void)
{
    return alu((AluInst){ ALU_DEFAULTS, .sig = SIG_PROG_END });
}

/* ------------------------------------------------------------------ */

typedef struct TestEnv {
    const uint64_t *prog;
    size_t prog_len;

    /* Uniform stream handed out on reads of raddr 32. */
    const uint32_t *uniforms;
    size_t uniform_count;
    size_t uniform_pos;

    /* Last special-register write seen. */
    unsigned write_count;
    unsigned last_waddr;
    bool last_file_b;
    uint16_t last_mask;
    uint32_t last_value[VC4_QPU_LANES];

    /* Value delivered into r4 on a load signal. */
    uint32_t tmu_value;
    bool have_tmu;
} TestEnv;

static bool test_fetch(void *opaque, uint32_t addr, uint64_t *inst)
{
    TestEnv *e = opaque;

    if ((addr & 7) || addr / 8 >= e->prog_len) {
        return false;
    }
    *inst = e->prog[addr / 8];
    return true;
}

static bool test_read_special(void *opaque, unsigned raddr, bool file_b,
                              uint32_t lanes[VC4_QPU_LANES])
{
    TestEnv *e = opaque;
    unsigned i;

    if (raddr != R_UNIF || file_b) {
        return false;
    }
    if (e->uniform_pos >= e->uniform_count) {
        return false;
    }
    for (i = 0; i < VC4_QPU_LANES; i++) {
        lanes[i] = e->uniforms[e->uniform_pos];
    }
    e->uniform_pos++;
    return true;
}

static void test_write_special(void *opaque, unsigned waddr, bool file_b,
                               const uint32_t lanes[VC4_QPU_LANES],
                               uint16_t mask)
{
    TestEnv *e = opaque;

    e->write_count++;
    e->last_waddr = waddr;
    e->last_file_b = file_b;
    e->last_mask = mask;
    memcpy(e->last_value, lanes, sizeof(e->last_value));
}

static bool test_signal_load(void *opaque, unsigned sig,
                             uint32_t r4[VC4_QPU_LANES])
{
    TestEnv *e = opaque;
    unsigned i;

    if (!e->have_tmu) {
        return false;
    }
    for (i = 0; i < VC4_QPU_LANES; i++) {
        r4[i] = e->tmu_value + sig;
    }
    return true;
}

static const V3DQPUOps test_ops = {
    .fetch = test_fetch,
    .read_special = test_read_special,
    .write_special = test_write_special,
    .signal_load = test_signal_load,
};

/* Run a program to completion and assert it ended cleanly. */
static V3DQPUResult run_prog(V3DQPUState *q, TestEnv *e,
                             const uint64_t *prog, size_t len)
{
    e->prog = prog;
    e->prog_len = len;
    vc4_qpu_init(q, &test_ops, e);
    return vc4_qpu_run(q, 0, 1000);
}

static void assert_all_lanes(const uint32_t *v, uint32_t expected)
{
    unsigned i;

    for (i = 0; i < VC4_QPU_LANES; i++) {
        g_assert_cmphex(v[i], ==, expected);
    }
}

/* ------------------------------------------------------------------ */

static void test_integer_add(void)
{
    uint64_t p[] = {
        set_r0(7),
        set_r1(35),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_ADD, .waddr_add = 0,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        prog_end(),
        nop_inst(),
        nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    /* The add ALU writes regfile A by default (ws clear). */
    assert_all_lanes(q.ra[0], 42);
    /* Program end plus its two delay slots: six instructions retired. */
    g_assert_cmpuint(q.instructions, ==, 6);
}

static void test_ws_swaps_files(void)
{
    uint64_t p[] = {
        set_r0(3),
        set_r1(4),
        /* ws set: the add ALU writes regfile B, the mul ALU regfile A. */
        alu((AluInst){ ALU_DEFAULTS, .ws = true,
                       .op_add = A_ADD, .waddr_add = 5,
                       .add_a = MUX_R0, .add_b = MUX_R1,
                       .op_mul = M_MUL24, .waddr_mul = 6,
                       .mul_a = MUX_R0, .mul_b = MUX_R1 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    assert_all_lanes(q.rb[5], 7);
    assert_all_lanes(q.ra[6], 12);
}

/* Reinterpret a float as the 32-bit pattern the QPU holds in a register. */
static uint32_t fbits(float f)
{
    uint32_t u;

    memcpy(&u, &f, sizeof(u));
    return u;
}

static void test_float_ops(void)
{
    uint64_t p[] = {
        set_r0(fbits(1.5f)),
        set_r1(fbits(2.25f)),
        load_imm(LI_32, 3, W_ACC2, W_NOP, false),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_FADD, .waddr_add = 0,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_FSUB, .waddr_add = 1,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_mul = M_FMUL, .waddr_mul = 2,
                       .mul_a = MUX_R0, .mul_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_FMIN, .waddr_add = 3,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_FMAX, .waddr_add = 4,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_FTOI, .waddr_add = 5,
                       .add_a = MUX_R1, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_ITOF, .waddr_add = 6,
                       .add_a = MUX_R2, .add_b = MUX_R2 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);

    assert_all_lanes(q.ra[0], fbits(3.75f));
    assert_all_lanes(q.ra[1], fbits(-0.75f));
    /* The mul ALU writes regfile B when ws is clear. */
    assert_all_lanes(q.rb[2], fbits(3.375f));
    assert_all_lanes(q.ra[3], fbits(1.5f));
    assert_all_lanes(q.ra[4], fbits(2.25f));
    /* FTOI rounds to nearest even: 2.25 -> 2. */
    assert_all_lanes(q.ra[5], 2);
    assert_all_lanes(q.ra[6], fbits(3.0f));
}

static void test_bitwise_and_shifts(void)
{
    uint64_t p[] = {
        set_r0(0xf000000fu),
        set_r1(4),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_SHR, .waddr_add = 0,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_ASR, .waddr_add = 1,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_ROR, .waddr_add = 2,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_SHL, .waddr_add = 3,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_NOT, .waddr_add = 4,
                       .add_a = MUX_R0, .add_b = MUX_R0 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_CLZ, .waddr_add = 5,
                       .add_a = MUX_R1, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_XOR, .waddr_add = 6,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    assert_all_lanes(q.ra[0], 0x0f000000u);
    assert_all_lanes(q.ra[1], 0xff000000u);
    assert_all_lanes(q.ra[2], 0xff000000u);
    assert_all_lanes(q.ra[3], 0x000000f0u);
    assert_all_lanes(q.ra[4], 0x0ffffff0u);
    assert_all_lanes(q.ra[5], 29);          /* clz(4) */
    assert_all_lanes(q.ra[6], 0xf000000bu);
}

static void test_v8_byte_ops(void)
{
    uint64_t p[] = {
        set_r0(0x10ff0180u),
        set_r1(0x20020280u),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_V8ADDS, .waddr_add = 0,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_V8SUBS, .waddr_add = 1,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_mul = M_V8MIN, .waddr_mul = 2,
                       .mul_a = MUX_R0, .mul_b = MUX_R1 }),
        alu((AluInst){ ALU_DEFAULTS, .op_mul = M_V8MAX, .waddr_mul = 3,
                       .mul_a = MUX_R0, .mul_b = MUX_R1 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    /* Per byte, saturating: 80+80=ff(sat 100->ff), 01+02=03,
     * ff+02=ff(sat), 10+20=30. */
    assert_all_lanes(q.ra[0], 0x30ff03ffu);
    /* Per byte, clamped at zero: 80-80=00, 01-02=00, ff-02=fd, 10-20=00. */
    assert_all_lanes(q.ra[1], 0x00fd0000u);
    assert_all_lanes(q.rb[2], 0x10020180u);
    assert_all_lanes(q.rb[3], 0x20ff0280u);
}

static void test_small_immediates(void)
{
    uint64_t p[] = {
        set_r0(100),
        /* Small immediate 5 -> the integer 5. */
        alu((AluInst){ ALU_DEFAULTS, .sig = SIG_SMALL_IMM, .raddr_b = 5,
                       .op_add = A_ADD, .waddr_add = 0,
                       .add_a = MUX_R0, .add_b = MUX_B }),
        /* Small immediate 31 -> the integer -1. */
        alu((AluInst){ ALU_DEFAULTS, .sig = SIG_SMALL_IMM, .raddr_b = 31,
                       .op_add = A_ADD, .waddr_add = 1,
                       .add_a = MUX_R0, .add_b = MUX_B }),
        /* Small immediate 32 -> the float 1.0. */
        alu((AluInst){ ALU_DEFAULTS, .sig = SIG_SMALL_IMM, .raddr_b = 32,
                       .op_add = A_OR, .waddr_add = 2,
                       .add_a = MUX_B, .add_b = MUX_B }),
        /* Small immediate 47 -> the float 0.5. */
        alu((AluInst){ ALU_DEFAULTS, .sig = SIG_SMALL_IMM, .raddr_b = 47,
                       .op_add = A_OR, .waddr_add = 3,
                       .add_a = MUX_B, .add_b = MUX_B }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    assert_all_lanes(q.ra[0], 105);
    assert_all_lanes(q.ra[1], 99);
    assert_all_lanes(q.ra[2], fbits(1.0f));
    assert_all_lanes(q.ra[3], fbits(0.5f));
}

static void test_vector_rotation_is_refused(void)
{
    uint64_t p[] = {
        /* Small immediate 48 asks the mul ALU to rotate lanes. */
        alu((AluInst){ ALU_DEFAULTS, .sig = SIG_SMALL_IMM, .raddr_b = 48,
                       .op_mul = M_FMUL, .waddr_mul = 0,
                       .mul_a = MUX_R0, .mul_b = MUX_B }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==,
                    V3D_QPU_UNSUPPORTED);
    g_assert_nonnull(q.unsupported);
}

static void test_conditional_write_and_flags(void)
{
    uint64_t p[] = {
        set_r0(5),
        set_r1(5),
        /* SUB sets the flags: r0 - r1 == 0, so Z is set in every lane. */
        alu((AluInst){ ALU_DEFAULTS, .sf = true, .op_add = A_SUB,
                       .waddr_add = W_NOP,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        /* Writes only where Z is set: everywhere. */
        alu((AluInst){ ALU_DEFAULTS, .cond_add = COND_ZS, .op_add = A_ADD,
                       .waddr_add = 0, .add_a = MUX_R0, .add_b = MUX_R1 }),
        /* Writes only where Z is clear: nowhere. */
        alu((AluInst){ ALU_DEFAULTS, .cond_add = COND_ZC, .op_add = A_ADD,
                       .waddr_add = 1, .add_a = MUX_R0, .add_b = MUX_R1 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    g_assert_cmphex(q.flag_z, ==, 0xffff);
    g_assert_cmphex(q.flag_n, ==, 0);
    /* SUB of equal operands does not borrow, so carry is set. */
    g_assert_cmphex(q.flag_c, ==, 0xffff);
    assert_all_lanes(q.ra[0], 10);
    assert_all_lanes(q.ra[1], 0);
}

static void test_negative_flag(void)
{
    uint64_t p[] = {
        set_r0(1),
        set_r1(3),
        alu((AluInst){ ALU_DEFAULTS, .sf = true, .op_add = A_SUB,
                       .waddr_add = 0, .add_a = MUX_R0, .add_b = MUX_R1 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    assert_all_lanes(q.ra[0], 0xfffffffeu);
    g_assert_cmphex(q.flag_n, ==, 0xffff);
    g_assert_cmphex(q.flag_z, ==, 0);
    g_assert_cmphex(q.flag_c, ==, 0);   /* 1 - 3 borrows */
}

static void test_per_element_load_imm(void)
{
    /* Lane i takes bit i of the low half as its low bit and bit i of the
     * high half as its high bit.  Lanes 0..3 get 0, 1, 2, 3. */
    uint32_t imm = (0xcu << 16) | 0xau;
    uint64_t p[] = {
        load_imm(LI_PER_ELEM_UNSIGNED, imm, 0, W_NOP, false),
        load_imm(LI_PER_ELEM_SIGNED, imm, 1, W_NOP, false),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    /* low half 0xa = 1010b, high half 0xc = 1100b. */
    g_assert_cmphex(q.ra[0][0], ==, 0);   /* hi 0 lo 0 */
    g_assert_cmphex(q.ra[0][1], ==, 1);   /* hi 0 lo 1 */
    g_assert_cmphex(q.ra[0][2], ==, 2);   /* hi 1 lo 0 */
    g_assert_cmphex(q.ra[0][3], ==, 3);   /* hi 1 lo 1 */
    /* Signed: 2 and 3 become -2 and -1. */
    g_assert_cmphex(q.ra[1][0], ==, 0);
    g_assert_cmphex(q.ra[1][1], ==, 1);
    g_assert_cmphex(q.ra[1][2], ==, 0xfffffffeu);
    g_assert_cmphex(q.ra[1][3], ==, 0xffffffffu);
}

static void test_element_number(void)
{
    uint64_t p[] = {
        alu((AluInst){ ALU_DEFAULTS, .raddr_a = R_ELEM_QPU,
                       .op_add = A_OR, .waddr_add = 0,
                       .add_a = MUX_A, .add_b = MUX_A }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };
    unsigned i;

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    for (i = 0; i < VC4_QPU_LANES; i++) {
        g_assert_cmpuint(q.ra[0][i], ==, i);
    }
}

static void test_uniform_stream(void)
{
    static const uint32_t uniforms[] = { 11, 22 };
    uint64_t p[] = {
        alu((AluInst){ ALU_DEFAULTS, .raddr_a = R_UNIF,
                       .op_add = A_OR, .waddr_add = 0,
                       .add_a = MUX_A, .add_b = MUX_A }),
        alu((AluInst){ ALU_DEFAULTS, .raddr_a = R_UNIF,
                       .op_add = A_OR, .waddr_add = 1,
                       .add_a = MUX_A, .add_b = MUX_A }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    e.uniforms = uniforms;
    e.uniform_count = ARRAY_SIZE(uniforms);

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    assert_all_lanes(q.ra[0], 11);
    assert_all_lanes(q.ra[1], 22);
    /* Naming the uniform register is what pops the stream. */
    g_assert_cmpuint(e.uniform_pos, ==, 2);
}

static void test_special_register_write(void)
{
    uint64_t p[] = {
        set_r0(0xabcdu),
        set_r1(0),
        /* Set Z everywhere except... nothing; just check routing and mask. */
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_OR, .waddr_add = W_VPM,
                       .add_a = MUX_R0, .add_b = MUX_R0 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    g_assert_cmpuint(e.write_count, ==, 1);
    g_assert_cmpuint(e.last_waddr, ==, W_VPM);
    g_assert_cmphex(e.last_mask, ==, 0xffff);
    g_assert_false(e.last_file_b);
    assert_all_lanes(e.last_value, 0xabcdu);
}

static void test_load_signal_fills_r4(void)
{
    uint64_t p[] = {
        alu((AluInst){ ALU_DEFAULTS, .sig = SIG_LOAD_TMU0 }),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_OR, .waddr_add = 0,
                       .add_a = MUX_R4, .add_b = MUX_R4 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    e.have_tmu = true;
    e.tmu_value = 0x1000;

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    assert_all_lanes(q.ra[0], 0x1000 + SIG_LOAD_TMU0);
}

static void test_load_signal_without_env_is_refused(void)
{
    uint64_t p[] = {
        alu((AluInst){ ALU_DEFAULTS, .sig = SIG_LOAD_TMU0 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==,
                    V3D_QPU_UNSUPPORTED);
}

/*
 * A taken relative branch lands on the instruction after the three delay
 * slots, plus the immediate -- the arithmetic vc4_validate_branches() uses.
 */
static void test_relative_branch(void)
{
    uint64_t p[] = {
        /* 0: branch over instruction 4. */
        branch(BCOND_ALWAYS, true, 8, W_NOP),
        /* 1-3: delay slots, all of which execute. */
        set_r0(1),
        set_r1(2),
        nop_inst(),
        /* 4: skipped. */
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_ADD, .waddr_add = 7,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        /* 5: the branch target (instruction 4 + 8 bytes). */
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_ADD, .waddr_add = 8,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    assert_all_lanes(q.ra[7], 0);   /* never executed */
    assert_all_lanes(q.ra[8], 3);
}

static void test_branch_not_taken(void)
{
    uint64_t p[] = {
        set_r0(1),
        set_r1(2),
        /* Flags are all clear, so "all Z set" fails. */
        branch(BCOND_ALL_ZS, true, 8, W_NOP),
        nop_inst(), nop_inst(), nop_inst(),
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_ADD, .waddr_add = 7,
                       .add_a = MUX_R0, .add_b = MUX_R1 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    assert_all_lanes(q.ra[7], 3);
}

static void test_branch_link_address(void)
{
    uint64_t p[] = {
        branch(BCOND_ALWAYS, true, 8, 3 /* regfile A r3 */),
        nop_inst(), nop_inst(), nop_inst(),
        nop_inst(),                 /* skipped */
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    /* The instruction after the delay slots is at byte 32. */
    assert_all_lanes(q.ra[3], 32);
}

static void test_fetch_fault(void)
{
    uint64_t p[] = {
        nop_inst(),     /* runs off the end of the program */
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_FAULT);
}

static void test_instruction_limit(void)
{
    uint64_t p[] = {
        /* An unconditional backwards branch to itself. */
        branch(BCOND_ALWAYS, true, -32, W_NOP),
        nop_inst(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    e.prog = p;
    e.prog_len = ARRAY_SIZE(p);
    vc4_qpu_init(&q, &test_ops, &e);
    g_assert_cmpint(vc4_qpu_run(&q, 0, 64), ==, V3D_QPU_LIMIT);
    g_assert_cmpuint(q.instructions, ==, 64);
}

/*
 * A write happens because the write address names a register, not because the
 * ALU is doing arithmetic.  Texture setups are encoded exactly this way -- a
 * write to a TMU register with the add ALU left at nop -- so gating writes on
 * the opcode would silently drop the fetch.
 */
static void test_write_with_nop_alu(void)
{
    uint64_t p[] = {
        alu((AluInst){ ALU_DEFAULTS, .op_add = A_NOP, .waddr_add = W_TMU0_S,
                       .add_a = MUX_R0, .add_b = MUX_R0 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    g_assert_cmpuint(e.write_count, ==, 1);
    g_assert_cmpuint(e.last_waddr, ==, W_TMU0_S);
}

/* ... and a plain nop, which writes to the discard address, must not. */
static void test_plain_nop_writes_nothing(void)
{
    uint64_t p[] = {
        nop_inst(), nop_inst(),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==, V3D_QPU_DONE);
    g_assert_cmpuint(e.write_count, ==, 0);
}

static void test_pack_is_refused(void)
{
    uint64_t p[] = {
        alu((AluInst){ ALU_DEFAULTS, .pack = 1, .op_add = A_ADD,
                       .waddr_add = 0, .add_a = MUX_R0, .add_b = MUX_R0 }),
        prog_end(), nop_inst(), nop_inst(),
    };
    V3DQPUState q;
    TestEnv e = { 0 };

    g_assert_cmpint(run_prog(&q, &e, p, ARRAY_SIZE(p)), ==,
                    V3D_QPU_UNSUPPORTED);
    g_assert_cmpstr(q.unsupported, ==, "pack mode");
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/vc4-qpu/integer-add", test_integer_add);
    g_test_add_func("/vc4-qpu/ws-swaps-files", test_ws_swaps_files);
    g_test_add_func("/vc4-qpu/float-ops", test_float_ops);
    g_test_add_func("/vc4-qpu/bitwise-and-shifts", test_bitwise_and_shifts);
    g_test_add_func("/vc4-qpu/v8-byte-ops", test_v8_byte_ops);
    g_test_add_func("/vc4-qpu/small-immediates", test_small_immediates);
    g_test_add_func("/vc4-qpu/vector-rotation-refused",
                    test_vector_rotation_is_refused);
    g_test_add_func("/vc4-qpu/conditional-write",
                    test_conditional_write_and_flags);
    g_test_add_func("/vc4-qpu/negative-flag", test_negative_flag);
    g_test_add_func("/vc4-qpu/per-element-load-imm", test_per_element_load_imm);
    g_test_add_func("/vc4-qpu/element-number", test_element_number);
    g_test_add_func("/vc4-qpu/uniform-stream", test_uniform_stream);
    g_test_add_func("/vc4-qpu/special-register-write",
                    test_special_register_write);
    g_test_add_func("/vc4-qpu/load-signal", test_load_signal_fills_r4);
    g_test_add_func("/vc4-qpu/load-signal-refused",
                    test_load_signal_without_env_is_refused);
    g_test_add_func("/vc4-qpu/relative-branch", test_relative_branch);
    g_test_add_func("/vc4-qpu/branch-not-taken", test_branch_not_taken);
    g_test_add_func("/vc4-qpu/branch-link-address", test_branch_link_address);
    g_test_add_func("/vc4-qpu/fetch-fault", test_fetch_fault);
    g_test_add_func("/vc4-qpu/instruction-limit", test_instruction_limit);
    g_test_add_func("/vc4-qpu/write-with-nop-alu", test_write_with_nop_alu);
    g_test_add_func("/vc4-qpu/plain-nop-writes-nothing",
                    test_plain_nop_writes_nothing);
    g_test_add_func("/vc4-qpu/pack-refused", test_pack_is_refused);

    return g_test_run();
}
