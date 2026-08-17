/*
 * VideoCore IV QPU (Quad Processing Unit) instruction-set interpreter.
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/display/vc4_qpu.h for what this models and what it does not.
 * Instruction encoding follows drivers/gpu/drm/vc4/vc4_qpu_defines.h; the
 * branch target arithmetic follows vc4_validate_branches() in
 * drivers/gpu/drm/vc4/vc4_validate_shaders.c, which is the authoritative
 * in-tree statement of it.
 *
 * Floating point uses the host's binary32 arithmetic.  The VC4 ALU is not
 * IEEE-exact anyway (it flushes denormals and only rounds to nearest even), so
 * this is a functional approximation, not a bit-exact one.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/display/vc4_qpu.h"

#include <math.h>

/* Instruction fields (vc4_qpu_defines.h). */
#define QPU_MASK(high, low) \
    ((((uint64_t)1 << ((high) - (low) + 1)) - 1) << (low))
#define QPU_GET(word, field) \
    ((uint32_t)(((word) & QPU_##field##_MASK) >> QPU_##field##_SHIFT))

#define QPU_SIG_SHIFT           60
#define QPU_SIG_MASK            QPU_MASK(63, 60)
#define QPU_UNPACK_SHIFT        57
#define QPU_UNPACK_MASK         QPU_MASK(59, 57)
#define QPU_PM                  ((uint64_t)1 << 56)
#define QPU_PACK_SHIFT          52
#define QPU_PACK_MASK           QPU_MASK(55, 52)
#define QPU_COND_ADD_SHIFT      49
#define QPU_COND_ADD_MASK       QPU_MASK(51, 49)
#define QPU_COND_MUL_SHIFT      46
#define QPU_COND_MUL_MASK       QPU_MASK(48, 46)
#define QPU_SF                  ((uint64_t)1 << 45)
#define QPU_WS                  ((uint64_t)1 << 44)
#define QPU_WADDR_ADD_SHIFT     38
#define QPU_WADDR_ADD_MASK      QPU_MASK(43, 38)
#define QPU_WADDR_MUL_SHIFT     32
#define QPU_WADDR_MUL_MASK      QPU_MASK(37, 32)
#define QPU_OP_MUL_SHIFT        29
#define QPU_OP_MUL_MASK         QPU_MASK(31, 29)
#define QPU_OP_ADD_SHIFT        24
#define QPU_OP_ADD_MASK         QPU_MASK(28, 24)
#define QPU_RADDR_A_SHIFT       18
#define QPU_RADDR_A_MASK        QPU_MASK(23, 18)
#define QPU_RADDR_B_SHIFT       12
#define QPU_RADDR_B_MASK        QPU_MASK(17, 12)
#define QPU_SMALL_IMM_SHIFT     12
#define QPU_SMALL_IMM_MASK      QPU_MASK(17, 12)
#define QPU_ADD_A_SHIFT         9
#define QPU_ADD_A_MASK          QPU_MASK(11, 9)
#define QPU_ADD_B_SHIFT         6
#define QPU_ADD_B_MASK          QPU_MASK(8, 6)
#define QPU_MUL_A_SHIFT         3
#define QPU_MUL_A_MASK          QPU_MASK(5, 3)
#define QPU_MUL_B_SHIFT         0
#define QPU_MUL_B_MASK          QPU_MASK(2, 0)
#define QPU_LOAD_IMM_SHIFT      0
#define QPU_LOAD_IMM_MASK       QPU_MASK(31, 0)

/* Branch-only fields, which overlap the ALU ones. */
#define QPU_BRANCH_COND_SHIFT   52
#define QPU_BRANCH_COND_MASK    QPU_MASK(55, 52)
#define QPU_BRANCH_REL          ((uint64_t)1 << 51)
#define QPU_BRANCH_REG          ((uint64_t)1 << 50)
#define QPU_BRANCH_RADDR_A_SHIFT 45
#define QPU_BRANCH_RADDR_A_MASK QPU_MASK(49, 45)
#define QPU_BRANCH_TARGET_SHIFT 0
#define QPU_BRANCH_TARGET_MASK  QPU_MASK(31, 0)

enum {
    QPU_SIG_SW_BREAKPOINT, QPU_SIG_NONE, QPU_SIG_THREAD_SWITCH,
    QPU_SIG_PROG_END, QPU_SIG_WAIT_FOR_SCOREBOARD, QPU_SIG_SCOREBOARD_UNLOCK,
    QPU_SIG_LAST_THREAD_SWITCH, QPU_SIG_COVERAGE_LOAD, QPU_SIG_COLOR_LOAD,
    QPU_SIG_COLOR_LOAD_END, QPU_SIG_LOAD_TMU0, QPU_SIG_LOAD_TMU1,
    QPU_SIG_ALPHA_MASK_LOAD, QPU_SIG_SMALL_IMM, QPU_SIG_LOAD_IMM,
    QPU_SIG_BRANCH,
};

enum {
    QPU_A_NOP, QPU_A_FADD, QPU_A_FSUB, QPU_A_FMIN, QPU_A_FMAX, QPU_A_FMINABS,
    QPU_A_FMAXABS, QPU_A_FTOI, QPU_A_ITOF,
    QPU_A_ADD = 12, QPU_A_SUB, QPU_A_SHR, QPU_A_ASR, QPU_A_ROR, QPU_A_SHL,
    QPU_A_MIN, QPU_A_MAX, QPU_A_AND, QPU_A_OR, QPU_A_XOR, QPU_A_NOT,
    QPU_A_CLZ, QPU_A_V8ADDS = 30, QPU_A_V8SUBS = 31,
};

enum {
    QPU_M_NOP, QPU_M_FMUL, QPU_M_MUL24, QPU_M_V8MULD, QPU_M_V8MIN,
    QPU_M_V8MAX, QPU_M_V8ADDS, QPU_M_V8SUBS,
};

enum {
    QPU_MUX_R0, QPU_MUX_R1, QPU_MUX_R2, QPU_MUX_R3, QPU_MUX_R4, QPU_MUX_R5,
    QPU_MUX_A, QPU_MUX_B,
};

enum {
    QPU_COND_NEVER, QPU_COND_ALWAYS, QPU_COND_ZS, QPU_COND_ZC, QPU_COND_NS,
    QPU_COND_NC, QPU_COND_CS, QPU_COND_CC,
};

/* Write addresses the core handles itself; everything else >= 32 is the
 * environment's (VPM, TMU, TLB, SFU, ...). */
enum {
    QPU_W_ACC0 = 32, QPU_W_ACC1, QPU_W_ACC2, QPU_W_ACC3,
    QPU_W_TMU_NOSWAP, QPU_W_ACC5, QPU_W_HOST_INT, QPU_W_NOP,
    QPU_W_SFU_RECIP = 52, QPU_W_SFU_RECIPSQRT, QPU_W_SFU_EXP, QPU_W_SFU_LOG,
};

/* Read addresses handled here; the rest go to ops->read_special. */
enum {
    QPU_R_UNIF = 32,
    QPU_R_ELEM_QPU = 38,
    QPU_R_NOP = 39,
};

/* Load-immediate variants, in the unpack field. */
enum {
    QPU_LOAD_IMM_32 = 0,
    QPU_LOAD_IMM_PER_ELEM_SIGNED = 1,
    QPU_LOAD_IMM_PER_ELEM_UNSIGNED = 3,
    QPU_LOAD_IMM_SEMAPHORE = 4,
};

/* Branch conditions (a different encoding from the ALU condition codes). */
enum {
    QPU_BRANCH_COND_ALL_ZS, QPU_BRANCH_COND_ALL_ZC,
    QPU_BRANCH_COND_ANY_ZS, QPU_BRANCH_COND_ANY_ZC,
    QPU_BRANCH_COND_ALL_NS, QPU_BRANCH_COND_ALL_NC,
    QPU_BRANCH_COND_ANY_NS, QPU_BRANCH_COND_ANY_NC,
    QPU_BRANCH_COND_ALL_CS, QPU_BRANCH_COND_ALL_CC,
    QPU_BRANCH_COND_ANY_CS, QPU_BRANCH_COND_ANY_CC,
    QPU_BRANCH_COND_ALWAYS = 15,
};

#define ALL_LANES ((uint16_t)0xffff)

typedef uint32_t V3DQPUVec[VC4_QPU_LANES];

static inline float u2f(uint32_t u)
{
    float f;

    memcpy(&f, &u, sizeof(f));
    return f;
}

static inline uint32_t f2u(float f)
{
    uint32_t u;

    memcpy(&u, &f, sizeof(u));
    return u;
}

static void qpu_unsupported(V3DQPUState *q, const char *what)
{
    if (!q->unsupported) {
        q->unsupported = what;
        qemu_log_mask(LOG_UNIMP, "vc4-qpu: %s not implemented (pc 0x%08x)\n",
                      what, q->pc);
    }
}

/*
 * ALU helpers.  Every operation is per lane on a 32-bit value; the float ops
 * reinterpret it as binary32.
 */

static uint32_t qpu_ftoi(uint32_t a)
{
    float f = u2f(a);

    if (isnan(f)) {
        return 0;
    }
    if (f >= 2147483648.0f) {
        return 0x7fffffffu;
    }
    if (f <= -2147483648.0f) {
        return 0x80000000u;
    }
    return (uint32_t)(int32_t)nearbyintf(f);
}

/* Per-byte helpers for the V8* operations, which treat the word as four
 * independent unsigned bytes. */
static uint32_t qpu_v8_map(uint32_t a, uint32_t b,
                           uint32_t (*op)(uint32_t, uint32_t))
{
    uint32_t r = 0;
    unsigned i;

    for (i = 0; i < 4; i++) {
        uint32_t byte = op((a >> (i * 8)) & 0xff, (b >> (i * 8)) & 0xff);

        r |= (byte & 0xff) << (i * 8);
    }
    return r;
}

static uint32_t qpu_byte_adds(uint32_t a, uint32_t b)
{
    uint32_t sum = a + b;

    return sum > 255 ? 255 : sum;
}

static uint32_t qpu_byte_subs(uint32_t a, uint32_t b)
{
    return a > b ? a - b : 0;
}

static uint32_t qpu_byte_min(uint32_t a, uint32_t b)
{
    return MIN(a, b);
}

static uint32_t qpu_byte_max(uint32_t a, uint32_t b)
{
    return MAX(a, b);
}

static uint32_t qpu_byte_muld(uint32_t a, uint32_t b)
{
    /* Multiply two 0..255 values as if they were 0..1, rounding. */
    return (a * b + 127) / 255;
}

/*
 * The add ALU.  Returns the result and, for the integer add/sub, the carry
 * out -- the only ops for which the hardware defines the C flag.
 */
static uint32_t qpu_add_alu(V3DQPUState *q, unsigned op, uint32_t a,
                            uint32_t b, bool *carry)
{
    *carry = false;

    switch (op) {
    case QPU_A_FADD:
        return f2u(u2f(a) + u2f(b));
    case QPU_A_FSUB:
        return f2u(u2f(a) - u2f(b));
    case QPU_A_FMIN:
        return u2f(a) < u2f(b) ? a : b;
    case QPU_A_FMAX:
        return u2f(a) > u2f(b) ? a : b;
    case QPU_A_FMINABS:
        return fabsf(u2f(a)) < fabsf(u2f(b)) ? f2u(fabsf(u2f(a)))
                                             : f2u(fabsf(u2f(b)));
    case QPU_A_FMAXABS:
        return fabsf(u2f(a)) > fabsf(u2f(b)) ? f2u(fabsf(u2f(a)))
                                             : f2u(fabsf(u2f(b)));
    case QPU_A_FTOI:
        return qpu_ftoi(a);
    case QPU_A_ITOF:
        return f2u((float)(int32_t)a);
    case QPU_A_ADD:
        *carry = (uint64_t)a + b > 0xffffffffu;
        return a + b;
    case QPU_A_SUB:
        /* Carry is the "no borrow" sense, as on other RISC cores. */
        *carry = a >= b;
        return a - b;
    case QPU_A_SHR:
        return a >> (b & 31);
    case QPU_A_ASR:
        return (uint32_t)((int32_t)a >> (b & 31));
    case QPU_A_ROR:
        return (a >> (b & 31)) | (a << ((32 - (b & 31)) & 31));
    case QPU_A_SHL:
        return a << (b & 31);
    case QPU_A_MIN:
        return (int32_t)a < (int32_t)b ? a : b;
    case QPU_A_MAX:
        return (int32_t)a > (int32_t)b ? a : b;
    case QPU_A_AND:
        return a & b;
    case QPU_A_OR:
        return a | b;
    case QPU_A_XOR:
        return a ^ b;
    case QPU_A_NOT:
        return ~a;
    case QPU_A_CLZ:
        return a ? clz32(a) : 32;
    case QPU_A_V8ADDS:
        return qpu_v8_map(a, b, qpu_byte_adds);
    case QPU_A_V8SUBS:
        return qpu_v8_map(a, b, qpu_byte_subs);
    default:
        qpu_unsupported(q, "add ALU opcode");
        return 0;
    }
}

static uint32_t qpu_mul_alu(V3DQPUState *q, unsigned op, uint32_t a,
                            uint32_t b)
{
    switch (op) {
    case QPU_M_FMUL:
        return f2u(u2f(a) * u2f(b));
    case QPU_M_MUL24:
        return (a & 0xffffff) * (b & 0xffffff);
    case QPU_M_V8MULD:
        return qpu_v8_map(a, b, qpu_byte_muld);
    case QPU_M_V8MIN:
        return qpu_v8_map(a, b, qpu_byte_min);
    case QPU_M_V8MAX:
        return qpu_v8_map(a, b, qpu_byte_max);
    case QPU_M_V8ADDS:
        return qpu_v8_map(a, b, qpu_byte_adds);
    case QPU_M_V8SUBS:
        return qpu_v8_map(a, b, qpu_byte_subs);
    default:
        qpu_unsupported(q, "mul ALU opcode");
        return 0;
    }
}

/*
 * Small immediates replace regfile B when the instruction signals
 * QPU_SIG_SMALL_IMM.  0-15 are the integers 0..15, 16-31 the integers
 * -16..-1, 32-39 the floats 1.0 .. 128.0 and 40-47 the floats 1/256 .. 1/2.
 * 48-63 request a mul-ALU vector rotation, which this model does not do.
 */
static bool qpu_small_imm(V3DQPUState *q, unsigned imm, uint32_t *out)
{
    if (imm < 16) {
        *out = imm;
    } else if (imm < 32) {
        *out = (uint32_t)((int32_t)imm - 32);
    } else if (imm < 40) {
        *out = f2u(ldexpf(1.0f, (int)imm - 32));
    } else if (imm < 48) {
        *out = f2u(ldexpf(1.0f, (int)imm - 48));
    } else {
        qpu_unsupported(q, "mul ALU vector rotation");
        *out = 0;
        return false;
    }
    return true;
}

/* Read regfile A or B, or a special register, into a whole vector. */
static void qpu_read_reg(V3DQPUState *q, unsigned raddr, bool file_b,
                         V3DQPUVec out)
{
    if (raddr < VC4_QPU_NUM_REGS) {
        memcpy(out, file_b ? q->rb[raddr] : q->ra[raddr], sizeof(V3DQPUVec));
        return;
    }

    /* The element number is core state, so answer it here. */
    if (raddr == QPU_R_ELEM_QPU && !file_b) {
        unsigned i;

        for (i = 0; i < VC4_QPU_LANES; i++) {
            out[i] = i;
        }
        return;
    }

    if (raddr == QPU_R_NOP) {
        memset(out, 0, sizeof(V3DQPUVec));
        return;
    }

    if (q->ops->read_special &&
        q->ops->read_special(q->env, raddr, file_b, out)) {
        return;
    }

    memset(out, 0, sizeof(V3DQPUVec));
    qpu_unsupported(q, raddr == QPU_R_UNIF ? "uniform read"
                                           : "special register read");
}

/* Resolve one of the four operand muxes. */
static const uint32_t *qpu_mux(V3DQPUState *q, unsigned mux,
                               const V3DQPUVec ra, const V3DQPUVec rb)
{
    switch (mux) {
    case QPU_MUX_R0:
    case QPU_MUX_R1:
    case QPU_MUX_R2:
    case QPU_MUX_R3:
    case QPU_MUX_R4:
    case QPU_MUX_R5:
        return q->acc[mux];
    case QPU_MUX_A:
        return ra;
    default:
        return rb;
    }
}

/* Lanes a condition code lets through, given the flags from the last
 * set-flags instruction. */
static uint16_t qpu_cond_mask(const V3DQPUState *q, unsigned cond)
{
    switch (cond) {
    case QPU_COND_NEVER:
        return 0;
    case QPU_COND_ALWAYS:
        return ALL_LANES;
    case QPU_COND_ZS:
        return q->flag_z;
    case QPU_COND_ZC:
        return (uint16_t)~q->flag_z;
    case QPU_COND_NS:
        return q->flag_n;
    case QPU_COND_NC:
        return (uint16_t)~q->flag_n;
    case QPU_COND_CS:
        return q->flag_c;
    default:
        return (uint16_t)~q->flag_c;
    }
}

/*
 * Start a special function.  The unit takes one operand per lane and delivers
 * its result to r4 three instructions later, which is the latency Mesa's
 * scheduler assigns it: it will not read r4 while
 * tick - last_sfu_write_tick <= 2 (vc4_qpu_schedule.c).
 *
 * The functions are computed at host float precision.  The real unit is a
 * lookup table accurate to roughly twelve mantissa bits, so this is a
 * functional model rather than a bit-exact one -- as it is for the ALUs.
 */
static void qpu_sfu_start(V3DQPUState *q, unsigned waddr,
                          const V3DQPUVec value, uint16_t mask)
{
    unsigned i;

    for (i = 0; i < VC4_QPU_LANES; i++) {
        float x = u2f(value[i]);

        switch (waddr) {
        case QPU_W_SFU_RECIP:
            q->sfu_result[i] = f2u(1.0f / x);
            break;
        case QPU_W_SFU_RECIPSQRT:
            q->sfu_result[i] = f2u(1.0f / sqrtf(x));
            break;
        case QPU_W_SFU_EXP:
            q->sfu_result[i] = f2u(exp2f(x));
            break;
        default:
            q->sfu_result[i] = f2u(log2f(x));
            break;
        }
    }
    q->sfu_mask = mask;
    q->sfu_delay = VC4_QPU_SFU_LATENCY;
}

/* Deliver a finished special function into r4. */
static void qpu_sfu_retire(V3DQPUState *q)
{
    unsigned i;

    for (i = 0; i < VC4_QPU_LANES; i++) {
        if (q->sfu_mask & (1u << i)) {
            q->acc[4][i] = q->sfu_result[i];
        }
    }
}

static void qpu_write_reg(V3DQPUState *q, unsigned waddr, bool file_b,
                          const V3DQPUVec value, uint16_t mask)
{
    uint32_t *dst = NULL;
    unsigned i;

    if (!mask || waddr == QPU_W_NOP) {
        return;
    }

    if (waddr < VC4_QPU_NUM_REGS) {
        dst = file_b ? q->rb[waddr] : q->ra[waddr];
    } else if (waddr <= QPU_W_ACC3) {
        dst = q->acc[waddr - QPU_W_ACC0];
    } else if (waddr == QPU_W_ACC5) {
        dst = q->acc[5];
    }

    if (dst) {
        for (i = 0; i < VC4_QPU_LANES; i++) {
            if (mask & (1u << i)) {
                dst[i] = value[i];
            }
        }
        return;
    }

    if (waddr >= QPU_W_SFU_RECIP && waddr <= QPU_W_SFU_LOG) {
        qpu_sfu_start(q, waddr, value, mask);
        return;
    }

    if (q->ops->write_special) {
        q->ops->write_special(q->env, waddr, file_b, value, mask);
    } else {
        qpu_unsupported(q, "special register write");
    }
}

static void qpu_set_flags(V3DQPUState *q, const V3DQPUVec result,
                          uint16_t carry)
{
    unsigned i;

    q->flag_z = 0;
    q->flag_n = 0;
    q->flag_c = carry;
    for (i = 0; i < VC4_QPU_LANES; i++) {
        if (result[i] == 0) {
            q->flag_z |= 1u << i;
        }
        if (result[i] & 0x80000000u) {
            q->flag_n |= 1u << i;
        }
    }
}

static void qpu_exec_alu(V3DQPUState *q, uint64_t inst, unsigned sig)
{
    unsigned op_add = QPU_GET(inst, OP_ADD);
    unsigned op_mul = QPU_GET(inst, OP_MUL);
    unsigned raddr_a = QPU_GET(inst, RADDR_A);
    unsigned raddr_b = QPU_GET(inst, RADDR_B);
    unsigned waddr_add = QPU_GET(inst, WADDR_ADD);
    unsigned waddr_mul = QPU_GET(inst, WADDR_MUL);
    bool ws = !!(inst & QPU_WS);
    V3DQPUVec ra, rb, add_result, mul_result;
    const uint32_t *add_a, *add_b, *mul_a, *mul_b;
    uint16_t add_mask, mul_mask, carry = 0;
    unsigned i;

    if (QPU_GET(inst, PACK) != 0) {
        /* Both the regfile-A pack and the mul pack change the value written;
         * refusing is better than writing an unpacked one. */
        qpu_unsupported(q, "pack mode");
        return;
    }
    if (QPU_GET(inst, UNPACK) != 0) {
        qpu_unsupported(q, "unpack mode");
        return;
    }

    /* The register file reads happen because the raddr field names them, not
     * because a mux selects them: naming the uniform register pops the
     * uniform FIFO either way. */
    qpu_read_reg(q, raddr_a, false, ra);
    if (sig == QPU_SIG_SMALL_IMM) {
        uint32_t imm;

        if (!qpu_small_imm(q, raddr_b, &imm)) {
            return;
        }
        for (i = 0; i < VC4_QPU_LANES; i++) {
            rb[i] = imm;
        }
    } else {
        qpu_read_reg(q, raddr_b, true, rb);
    }
    if (q->unsupported) {
        return;
    }

    add_a = qpu_mux(q, QPU_GET(inst, ADD_A), ra, rb);
    add_b = qpu_mux(q, QPU_GET(inst, ADD_B), ra, rb);
    mul_a = qpu_mux(q, QPU_GET(inst, MUL_A), ra, rb);
    mul_b = qpu_mux(q, QPU_GET(inst, MUL_B), ra, rb);

    for (i = 0; i < VC4_QPU_LANES; i++) {
        bool lane_carry = false;

        if (op_add != QPU_A_NOP) {
            add_result[i] = qpu_add_alu(q, op_add, add_a[i], add_b[i],
                                        &lane_carry);
            if (lane_carry) {
                carry |= 1u << i;
            }
        } else {
            add_result[i] = 0;
        }
        mul_result[i] = op_mul != QPU_M_NOP
                      ? qpu_mul_alu(q, op_mul, mul_a[i], mul_b[i]) : 0;
    }
    if (q->unsupported) {
        return;
    }

    /* Condition codes test the flags left by an earlier instruction; this
     * instruction's own flags land after its writes. */
    add_mask = qpu_cond_mask(q, QPU_GET(inst, COND_ADD));
    mul_mask = qpu_cond_mask(q, QPU_GET(inst, COND_MUL));

    /*
     * The write address decides whether a result is written, not the opcode:
     * that is what the dedicated "write to nowhere" address (QPU_W_NOP) is
     * for, and it is how the kernel's shader validator reads an instruction
     * too -- check_instruction_writes() looks at waddr_add/waddr_mul and never
     * at the opcodes.  It matters: a texture setup is commonly encoded as a
     * write to a TMU register with the add ALU left at nop, and skipping that
     * write would silently lose the fetch.
     */
    qpu_write_reg(q, waddr_add, ws, add_result, add_mask);
    qpu_write_reg(q, waddr_mul, !ws, mul_result, mul_mask);

    if (inst & QPU_SF) {
        /* The add ALU sets the flags unless it is idle, in which case the
         * mul ALU does. */
        if (op_add != QPU_A_NOP) {
            qpu_set_flags(q, add_result, carry);
        } else {
            qpu_set_flags(q, mul_result, 0);
        }
    }
}

static void qpu_exec_load_imm(V3DQPUState *q, uint64_t inst)
{
    unsigned variant = QPU_GET(inst, UNPACK);
    uint32_t imm = QPU_GET(inst, LOAD_IMM);
    unsigned waddr_add = QPU_GET(inst, WADDR_ADD);
    unsigned waddr_mul = QPU_GET(inst, WADDR_MUL);
    bool ws = !!(inst & QPU_WS);
    V3DQPUVec value;
    unsigned i;

    if (QPU_GET(inst, PACK) != 0) {
        qpu_unsupported(q, "pack mode");
        return;
    }

    switch (variant) {
    case QPU_LOAD_IMM_32:
        for (i = 0; i < VC4_QPU_LANES; i++) {
            value[i] = imm;
        }
        break;

    case QPU_LOAD_IMM_PER_ELEM_SIGNED:
    case QPU_LOAD_IMM_PER_ELEM_UNSIGNED:
        /* Two bits per lane: the high bit from the top half of the immediate,
         * the low bit from the bottom half. */
        for (i = 0; i < VC4_QPU_LANES; i++) {
            uint32_t bits = (((imm >> (16 + i)) & 1) << 1) | ((imm >> i) & 1);

            value[i] = variant == QPU_LOAD_IMM_PER_ELEM_SIGNED && (bits & 2)
                     ? (uint32_t)((int32_t)bits - 4) : bits;
        }
        break;

    case QPU_LOAD_IMM_SEMAPHORE:
        qpu_unsupported(q, "semaphore instruction");
        return;

    default:
        qpu_unsupported(q, "load immediate variant");
        return;
    }

    qpu_write_reg(q, waddr_add, ws, value, qpu_cond_mask(q,
                                                QPU_GET(inst, COND_ADD)));
    qpu_write_reg(q, waddr_mul, !ws, value, qpu_cond_mask(q,
                                                QPU_GET(inst, COND_MUL)));

    if (inst & QPU_SF) {
        qpu_set_flags(q, value, 0);
    }
}

static bool qpu_branch_taken(const V3DQPUState *q, unsigned cond)
{
    switch (cond) {
    case QPU_BRANCH_COND_ALWAYS:
        return true;
    case QPU_BRANCH_COND_ALL_ZS:
        return q->flag_z == ALL_LANES;
    case QPU_BRANCH_COND_ALL_ZC:
        return q->flag_z == 0;
    case QPU_BRANCH_COND_ANY_ZS:
        return q->flag_z != 0;
    case QPU_BRANCH_COND_ANY_ZC:
        return q->flag_z != ALL_LANES;
    case QPU_BRANCH_COND_ALL_NS:
        return q->flag_n == ALL_LANES;
    case QPU_BRANCH_COND_ALL_NC:
        return q->flag_n == 0;
    case QPU_BRANCH_COND_ANY_NS:
        return q->flag_n != 0;
    case QPU_BRANCH_COND_ANY_NC:
        return q->flag_n != ALL_LANES;
    case QPU_BRANCH_COND_ALL_CS:
        return q->flag_c == ALL_LANES;
    case QPU_BRANCH_COND_ALL_CC:
        return q->flag_c == 0;
    case QPU_BRANCH_COND_ANY_CS:
        return q->flag_c != 0;
    case QPU_BRANCH_COND_ANY_CC:
        return q->flag_c != ALL_LANES;
    default:
        return false;
    }
}

/*
 * Branches take effect after three delay slots.  A relative target is added
 * to the address of the instruction after those delay slots -- the same
 * arithmetic vc4_validate_branches() uses to bounds-check them.
 */
static void qpu_exec_branch(V3DQPUState *q, uint64_t inst)
{
    uint32_t imm = QPU_GET(inst, BRANCH_TARGET);
    uint32_t after_delay = q->pc + (VC4_QPU_BRANCH_DELAY + 1) * 8;
    uint32_t target = imm;
    V3DQPUVec link;
    unsigned i;

    if (!qpu_branch_taken(q, QPU_GET(inst, BRANCH_COND))) {
        return;
    }

    if (inst & QPU_BRANCH_REL) {
        target += after_delay;
    }
    if (inst & QPU_BRANCH_REG) {
        unsigned raddr_a = QPU_GET(inst, BRANCH_RADDR_A);
        V3DQPUVec ra;

        qpu_read_reg(q, raddr_a, false, ra);
        if (q->unsupported) {
            return;
        }
        /* The register add uses lane 0. */
        target += ra[0];
    }

    /* The link address -- where a called routine returns to -- is the
     * instruction after the delay slots. */
    for (i = 0; i < VC4_QPU_LANES; i++) {
        link[i] = after_delay;
    }
    qpu_write_reg(q, QPU_GET(inst, WADDR_ADD), !!(inst & QPU_WS), link,
                  ALL_LANES);
    qpu_write_reg(q, QPU_GET(inst, WADDR_MUL), !(inst & QPU_WS), link,
                  ALL_LANES);

    q->branch_target = target;
    q->branch_delay = VC4_QPU_BRANCH_DELAY;
}

void vc4_qpu_init(V3DQPUState *q, const V3DQPUOps *ops, void *env)
{
    memset(q, 0, sizeof(*q));
    q->ops = ops;
    q->env = env;
    q->end_delay = -1;
    q->branch_delay = -1;
}

V3DQPUResult vc4_qpu_step(V3DQPUState *q)
{
    uint64_t inst;
    unsigned sig;

    if (!q->ops->fetch(q->env, q->pc, &inst)) {
        return V3D_QPU_FAULT;
    }
    q->instructions++;
    sig = QPU_GET(inst, SIG);

    switch (sig) {
    case QPU_SIG_BRANCH:
        qpu_exec_branch(q, inst);
        break;

    case QPU_SIG_LOAD_IMM:
        qpu_exec_load_imm(q, inst);
        break;

    case QPU_SIG_PROG_END:
        /* The two instructions after the signal still execute. */
        if (q->end_delay < 0) {
            q->end_delay = VC4_QPU_END_DELAY;
        }
        qpu_exec_alu(q, inst, sig);
        break;

    case QPU_SIG_COVERAGE_LOAD:
    case QPU_SIG_COLOR_LOAD:
    case QPU_SIG_COLOR_LOAD_END:
    case QPU_SIG_LOAD_TMU0:
    case QPU_SIG_LOAD_TMU1:
    case QPU_SIG_ALPHA_MASK_LOAD:
        /* These deliver a value into r4 alongside the ALU work. */
        if (q->ops->signal_load &&
            q->ops->signal_load(q->env, sig, q->acc[4])) {
            qpu_exec_alu(q, inst, sig);
        } else {
            qpu_unsupported(q, "load signal");
        }
        break;

    case QPU_SIG_SW_BREAKPOINT:
    case QPU_SIG_THREAD_SWITCH:
    case QPU_SIG_LAST_THREAD_SWITCH:
    case QPU_SIG_WAIT_FOR_SCOREBOARD:
    case QPU_SIG_SCOREBOARD_UNLOCK:
        /* Scheduling and synchronisation hints: nothing to do in a model
         * that runs one thread to completion. */
        qpu_exec_alu(q, inst, sig);
        break;

    default:
        /* QPU_SIG_NONE and QPU_SIG_SMALL_IMM. */
        qpu_exec_alu(q, inst, sig);
        break;
    }

    if (q->unsupported) {
        return V3D_QPU_UNSUPPORTED;
    }

    q->pc += 8;

    /* A special function lands in r4 three instructions after it was started,
     * so the two instructions in between still see the old r4. */
    if (q->sfu_delay > 0 && --q->sfu_delay == 0) {
        qpu_sfu_retire(q);
    }

    /* Delay slots retire after the instruction that follows them. */
    if (q->branch_delay >= 0 && q->branch_delay-- == 0) {
        q->pc = q->branch_target;
    }
    if (q->end_delay >= 0 && q->end_delay-- == 0) {
        return V3D_QPU_DONE;
    }
    return V3D_QPU_RUNNING;
}

V3DQPUResult vc4_qpu_run(V3DQPUState *q, uint32_t pc, uint64_t max_inst)
{
    uint64_t budget = max_inst;

    q->pc = pc;
    q->end_delay = -1;
    q->branch_delay = -1;
    q->sfu_delay = 0;
    q->unsupported = NULL;
    q->instructions = 0;

    while (budget--) {
        V3DQPUResult r = vc4_qpu_step(q);

        if (r != V3D_QPU_RUNNING) {
            return r;
        }
    }
    return V3D_QPU_LIMIT;
}

const char *vc4_qpu_result_name(V3DQPUResult r)
{
    switch (r) {
    case V3D_QPU_RUNNING:
        return "running";
    case V3D_QPU_DONE:
        return "done";
    case V3D_QPU_FAULT:
        return "instruction fetch fault";
    case V3D_QPU_LIMIT:
        return "instruction limit";
    case V3D_QPU_UNSUPPORTED:
        return "unsupported instruction";
    default:
        return "unknown";
    }
}
