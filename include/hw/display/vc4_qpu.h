/*
 * VideoCore IV QPU (Quad Processing Unit) instruction-set interpreter.
 *
 * Copyright (c) 2026 ...
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A functional model of one VideoCore IV shader core: 16 SIMD lanes, the two
 * 32-entry register files, accumulators r0-r5, the add and mul ALUs, condition
 * codes, small immediates, load-immediate and branches.  Instruction encoding
 * mirrors drivers/gpu/drm/vc4/vc4_qpu_defines.h.
 *
 * The core owns only what is inside the shader core.  Everything around it --
 * the uniform stream, varyings, the VPM, the TMU, the tile buffer, and the
 * memory the instructions are fetched from -- belongs to the caller and is
 * reached through V3DQPUOps.  That keeps this file free of device state, so it
 * can be unit tested on its own (tests/unit/test-vc4-qpu.c).
 *
 * Not modelled yet, and reported as V3D_QPU_UNSUPPORTED rather than quietly
 * computing the wrong answer: pack/unpack modes other than NOP and the mul-ALU
 * vector rotations (small immediates 48-63).
 *
 * The special-function unit is modelled functionally: reciprocal, reciprocal
 * square root, exp2 and log2 are computed at host float precision.  Real
 * hardware uses a lookup table good for about twelve bits of mantissa, so
 * nothing here is bit-exact with silicon -- deliberately, as it is for the
 * ALUs.
 */

#ifndef HW_DISPLAY_VC4_QPU_H
#define HW_DISPLAY_VC4_QPU_H

/* One instruction operates on 16 lanes: 4 quads of 4 pixels. */
#define VC4_QPU_LANES       16
#define VC4_QPU_NUM_REGS    32
#define VC4_QPU_NUM_ACC     6       /* r0-r5 */

/* Program end is followed by two delay slots, a branch by three. */
#define VC4_QPU_END_DELAY   2
#define VC4_QPU_BRANCH_DELAY 3

/* A special-function result reaches r4 three instructions after it starts. */
#define VC4_QPU_SFU_LATENCY 3

/* Read address of the uniform FIFO: the one special register nearly every
 * caller has to serve. */
#define VC4_QPU_RADDR_UNIF  32

typedef enum V3DQPUResult {
    V3D_QPU_RUNNING,        /* stepped one instruction, more to come */
    V3D_QPU_DONE,           /* program end plus its delay slots retired */
    V3D_QPU_FAULT,          /* instruction fetch failed */
    V3D_QPU_LIMIT,          /* ran out of the caller's instruction budget */
    V3D_QPU_UNSUPPORTED,    /* hit something this model does not implement */
} V3DQPUResult;

/*
 * Hooks into everything outside the shader core.  All of them are optional
 * except fetch(); a NULL hook (or one returning false) reads as zero and is
 * reported through V3DQPUState::unsupported.
 */
typedef struct V3DQPUOps {
    /* Read the 64-bit instruction at a byte address.  false = fault. */
    bool (*fetch)(void *env, uint32_t addr, uint64_t *inst);

    /* Read a special register (raddr >= 32): uniforms, varyings, VPM, ... */
    bool (*read_special)(void *env, unsigned raddr, bool file_b,
                         uint32_t lanes[VC4_QPU_LANES]);

    /* Write a special register (waddr >= 32 that is not an accumulator):
     * VPM, TMU, TLB, host interrupt, ...  lane_mask has a bit per lane the
     * condition code let through. */
    void (*write_special)(void *env, unsigned waddr, bool file_b,
                          const uint32_t lanes[VC4_QPU_LANES],
                          uint16_t lane_mask);

    /* An instruction signalled a load into r4 (TMU, colour, coverage).
     * Return false if the environment cannot supply it. */
    bool (*signal_load)(void *env, unsigned sig, uint32_t r4[VC4_QPU_LANES]);
} V3DQPUOps;

typedef struct V3DQPUState {
    const V3DQPUOps *ops;
    void *env;

    /* Register files A and B, and accumulators r0-r5.  r4 (index 4) is only
     * written by loads and the SFU, never by an ALU. */
    uint32_t ra[VC4_QPU_NUM_REGS][VC4_QPU_LANES];
    uint32_t rb[VC4_QPU_NUM_REGS][VC4_QPU_LANES];
    uint32_t acc[VC4_QPU_NUM_ACC][VC4_QPU_LANES];

    /* Condition flags, one bit per lane, from the last set-flags instruction. */
    uint16_t flag_z;
    uint16_t flag_n;
    uint16_t flag_c;

    uint32_t pc;            /* byte address of the next instruction */

    /* Delay-slot bookkeeping.  Negative means "not pending". */
    int end_delay;
    int branch_delay;
    uint32_t branch_target;

    /* A special-function result on its way to r4.  The unit takes three
     * instructions, which is the latency Mesa's scheduler assigns it and the
     * window in which it refuses to read r4 (vc4_qpu_schedule.c). */
    uint32_t sfu_result[VC4_QPU_LANES];
    uint16_t sfu_mask;
    int sfu_delay;

    uint64_t instructions;  /* retired this run */

    /* Set when a step returns V3D_QPU_UNSUPPORTED, naming the feature. */
    const char *unsupported;
} V3DQPUState;

void vc4_qpu_init(V3DQPUState *q, const V3DQPUOps *ops, void *env);

/* Execute one instruction (including delay slots and the branch that ends
 * the program). */
V3DQPUResult vc4_qpu_step(V3DQPUState *q);

/* Run from pc until the program ends, faults, or burns max_inst
 * instructions. */
V3DQPUResult vc4_qpu_run(V3DQPUState *q, uint32_t pc, uint64_t max_inst);

/* Human-readable result, for logs. */
const char *vc4_qpu_result_name(V3DQPUResult r);

#endif /* HW_DISPLAY_VC4_QPU_H */
