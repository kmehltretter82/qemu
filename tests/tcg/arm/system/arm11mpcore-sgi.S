/*
 * Bare-metal ARM11 MPCore SGI model test for QEMU's RealView EB MPCore.
 *
 * CPU0 programs SGI1 as N:N, CPU1 programs the same SGI as 1:N.  CPU0
 * first sends SGI1 to CPUs 2 and 3 and both must acknowledge it.  CPU1
 * then sends SGI1 to the same targets; CPU2 must acknowledge source 1 and
 * CPU3 must see a spurious interrupt after the 1:N instance was consumed.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

        .syntax unified
        .arch armv6k
        .arm

        .equ GICC,             0x10100100
        .equ GICC_CTLR,        0x000
        .equ GICC_PMR,         0x004
        .equ GICC_BPR,         0x008
        .equ GICC_IAR,         0x00c
        .equ GICC_EOIR,        0x010

        .equ GICD,             0x10101000
        .equ GICD_CTLR,        0x000
        .equ GICD_ISENABLER0,  0x100
        .equ GICD_ICPENDR0,    0x280
        .equ GICD_ICFGR0,      0xc00
        .equ GICD_SGIR,        0xf00

        .equ SHARED,           0x00020000
        .equ START,            0x00
        .equ READY0,           0x04
        .equ READY1,           0x08
        .equ READY2,           0x0c
        .equ READY3,           0x10
        .equ CONFIG_STAGE,     0x14
        .equ PHASE,            0x18
        .equ ACK2_A,           0x1c
        .equ ACK3_A,           0x20
        .equ IAR2_A,           0x24
        .equ IAR3_A,           0x28
        .equ ACK2_B,           0x2c
        .equ ACK3_B,           0x30
        .equ IAR2_B,           0x34
        .equ IAR3_B,           0x38
        .equ RESULT,           0x3c

        .equ START_MAGIC,      0x53474931
        .equ SGI1,             0x00000002
        .equ SGI1_TARGET_2_3,  0x000c0001
        .equ SPURIOUS,         1023
        .equ SGI1_FROM_CPU1,   1025

        .macro memory_barrier
        mov     r12, #0
        mcr     p15, 0, r12, c7, c10, 5
        .endm

        .macro sync_barrier
        mov     r12, #0
        mcr     p15, 0, r12, c7, c10, 4
        .endm

        .section .text.boot, "ax"
        .global _start
        .type _start, %function
_start:
        cpsid   if
        mrc     p15, 0, r4, c0, c0, 5
        and     r4, r4, #3
        ldr     r5, =SHARED

        /* Give each CPU a separate 4 KiB stack. */
        ldr     sp, =0x00080000
        add     sp, sp, r4, lsl #12

        cmp     r4, #0
        bne     wait_for_start

        mov     r0, #0
        mov     r1, #0x40
clear_shared:
        subs    r1, r1, #4
        str     r0, [r5, r1]
        bne     clear_shared
        memory_barrier
        ldr     r0, =START_MAGIC
        str     r0, [r5, #START]
        memory_barrier
        b       setup_cpu_interface

wait_for_start:
        ldr     r0, =START_MAGIC
1:      ldr     r1, [r5, #START]
        cmp     r1, r0
        bne     1b
        memory_barrier

setup_cpu_interface:
        ldr     r6, =GICC
        mov     r0, #0xff
        str     r0, [r6, #GICC_PMR]
        mov     r0, #0
        str     r0, [r6, #GICC_BPR]
        mov     r0, #1
        str     r0, [r6, #GICC_CTLR]

        ldr     r7, =GICD
        mov     r0, #SGI1
        str     r0, [r7, #GICD_ICPENDR0]
        str     r0, [r7, #GICD_ISENABLER0]
        sync_barrier

        add     r0, r5, #READY0
        mov     r1, #1
        str     r1, [r0, r4, lsl #2]
        memory_barrier

        cmp     r4, #0
        beq     cpu0
        cmp     r4, #1
        beq     cpu1
        cmp     r4, #2
        beq     cpu2
        b       cpu3

cpu0:
        /* Wait for all four banked CPU interfaces to be ready. */
1:      ldr     r0, [r5, #READY0]
        ldr     r1, [r5, #READY1]
        and     r0, r0, r1
        ldr     r1, [r5, #READY2]
        and     r0, r0, r1
        ldr     r1, [r5, #READY3]
        ands    r0, r0, r1
        beq     1b

        mov     r0, #1
        str     r0, [r7, #GICD_CTLR]

        /* SGI1 sent by CPU0 uses N:N (bit 2 clear). */
        ldr     r0, =0xaaaaaaaa
        str     r0, [r7, #GICD_ICFGR0]
        sync_barrier
        mov     r0, #1
        str     r0, [r5, #CONFIG_STAGE]
        memory_barrier

        /* CPU1 must subsequently install its own, banked 1:N setting. */
1:      ldr     r0, [r5, #CONFIG_STAGE]
        cmp     r0, #2
        bne     1b
        memory_barrier

        ldr     r0, =SGI1_TARGET_2_3
        str     r0, [r7, #GICD_SGIR]
        sync_barrier
        mov     r0, #1
        str     r0, [r5, #PHASE]
        memory_barrier

1:      ldr     r0, [r5, #ACK3_A]
        cmp     r0, #1
        bne     1b
        memory_barrier

        ldr     r0, [r5, #IAR2_A]
        cmp     r0, #1
        bne     fail_a_cpu2
        ldr     r0, [r5, #IAR3_A]
        cmp     r0, #1
        bne     fail_a_cpu3

        mov     r0, #2
        str     r0, [r5, #PHASE]
        memory_barrier

1:      ldr     r0, [r5, #ACK3_B]
        cmp     r0, #1
        bne     1b
        memory_barrier

        ldr     r0, [r5, #IAR2_B]
        ldr     r1, =SGI1_FROM_CPU1
        cmp     r0, r1
        bne     fail_b_cpu2
        ldr     r0, [r5, #IAR3_B]
        ldr     r1, =SPURIOUS
        cmp     r0, r1
        bne     fail_b_cpu3

        mov     r0, #0
        str     r0, [r5, #RESULT]
        ldr     r1, =pass_message
        bl      write0
        ldr     r1, =0x00020026
        b       semi_exit

fail_a_cpu2:
        mov     r0, #1
        ldr     r1, =fail_a_cpu2_message
        b       fail
fail_a_cpu3:
        mov     r0, #2
        ldr     r1, =fail_a_cpu3_message
        b       fail
fail_b_cpu2:
        mov     r0, #3
        ldr     r1, =fail_b_cpu2_message
        b       fail
fail_b_cpu3:
        mov     r0, #4
        ldr     r1, =fail_b_cpu3_message
fail:
        str     r0, [r5, #RESULT]
        bl      write0
        ldr     r1, =0x00020024
semi_exit:
        mov     r0, #0x18
        svc     0x123456
        b       .

cpu1:
1:      ldr     r0, [r5, #CONFIG_STAGE]
        cmp     r0, #1
        bne     1b
        memory_barrier

        /* SGI1 sent by CPU1 uses 1:N (bit 2 set). */
        ldr     r0, =0xaaaaaaae
        str     r0, [r7, #GICD_ICFGR0]
        sync_barrier
        mov     r0, #2
        str     r0, [r5, #CONFIG_STAGE]
        memory_barrier

1:      ldr     r0, [r5, #PHASE]
        cmp     r0, #2
        bne     1b
        memory_barrier
        ldr     r0, =SGI1_TARGET_2_3
        str     r0, [r7, #GICD_SGIR]
        sync_barrier
        mov     r0, #3
        str     r0, [r5, #PHASE]
        memory_barrier
        b       .

cpu2:
1:      ldr     r0, [r5, #PHASE]
        cmp     r0, #1
        bne     1b
        memory_barrier
        ldr     r0, [r6, #GICC_IAR]
        str     r0, [r5, #IAR2_A]
        ldr     r1, =SPURIOUS
        cmp     r0, r1
        strne   r0, [r6, #GICC_EOIR]
        sync_barrier
        mov     r0, #1
        str     r0, [r5, #ACK2_A]
        memory_barrier

1:      ldr     r0, [r5, #PHASE]
        cmp     r0, #3
        bne     1b
        memory_barrier
        ldr     r0, [r6, #GICC_IAR]
        str     r0, [r5, #IAR2_B]
        ldr     r1, =SPURIOUS
        cmp     r0, r1
        strne   r0, [r6, #GICC_EOIR]
        sync_barrier
        mov     r0, #1
        str     r0, [r5, #ACK2_B]
        memory_barrier
        b       .

cpu3:
1:      ldr     r0, [r5, #PHASE]
        cmp     r0, #1
        bne     1b
1:      ldr     r0, [r5, #ACK2_A]
        cmp     r0, #1
        bne     1b
        memory_barrier
        ldr     r0, [r6, #GICC_IAR]
        str     r0, [r5, #IAR3_A]
        ldr     r1, =SPURIOUS
        cmp     r0, r1
        strne   r0, [r6, #GICC_EOIR]
        sync_barrier
        mov     r0, #1
        str     r0, [r5, #ACK3_A]
        memory_barrier

1:      ldr     r0, [r5, #PHASE]
        cmp     r0, #3
        bne     1b
1:      ldr     r0, [r5, #ACK2_B]
        cmp     r0, #1
        bne     1b
        memory_barrier
        ldr     r0, [r6, #GICC_IAR]
        str     r0, [r5, #IAR3_B]
        ldr     r1, =SPURIOUS
        cmp     r0, r1
        strne   r0, [r6, #GICC_EOIR]
        sync_barrier
        mov     r0, #1
        str     r0, [r5, #ACK3_B]
        memory_barrier
        b       .

write0:
        mov     r0, #4
        svc     0x123456
        bx      lr

        .size _start, . - _start

        .section .rodata, "a"
pass_message:
        .asciz  "ARM11 SGI bank/source test: PASS\n"
fail_a_cpu2_message:
        .asciz  "ARM11 SGI bank/source test: FAIL N:N CPU2\n"
fail_a_cpu3_message:
        .asciz  "ARM11 SGI bank/source test: FAIL N:N CPU3\n"
fail_b_cpu2_message:
        .asciz  "ARM11 SGI bank/source test: FAIL 1:N CPU2/source\n"
fail_b_cpu3_message:
        .asciz  "ARM11 SGI bank/source test: FAIL 1:N CPU3\n"
