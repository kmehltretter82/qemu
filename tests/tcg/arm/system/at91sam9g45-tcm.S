/*
 * ARM926 ITCM/DTCM and SAM9G45 MATRIX integration test.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

    .syntax unified
    .cpu arm926ej-s
    .arm

    .equ MATRIX_TCMR, 0xffffeb10
    .equ ITCM_AHB,   0x00100000
    .equ DTCM_AHB,   0x00200000
    .equ SRAM_AHB,   0x00300000

    .section .text
    .global _start
    .type _start, %function
_start:
    mov r9, #0
    mrc p15, 0, r0, c0, c0, 2
    cmp r0, #0
    bne fail_1

    /* Partition the shared SRAM as 32 KiB ITCM + 32 KiB DTCM. */
    ldr r4, =MATRIX_TCMR
    mov r0, #0x66
    str r0, [r4]

    mrc p15, 0, r0, c0, c0, 2
    ldr r1, =0x00010001
    cmp r0, r1
    bne fail_2
    mrc p15, 0, r0, c9, c1, 1
    and r0, r0, #0x3c
    cmp r0, #0x18
    bne fail_3
    mrc p15, 0, r0, c9, c1, 0
    and r0, r0, #0x3c
    cmp r0, #0x18
    bne fail_4

    /* Copy a position-independent two-instruction routine into SRAM A. */
    ldr r0, =itcm_code_start
    ldr r1, =ITCM_AHB
    ldr r2, =itcm_code_end
copy_itcm:
    cmp r0, r2
    ldrlo r3, [r0], #4
    strlo r3, [r1], #4
    blo copy_itcm

    /* Map ITCM at 0x10000000 and execute it. */
    ldr r0, =0x10000019
    mcr p15, 0, r0, c9, c1, 1
    mov r0, #0
    mcr p15, 0, r0, c7, c5, 0
    mcr p15, 0, r0, c7, c5, 4
    mov r7, #0
    ldr r4, =0x10000000
    blx r4
    cmp r7, #0x5a
    bne fail_5

    /* ITCM is also visible to data accesses. */
    ldr r0, [r4]
    ldr r1, =itcm_code_start
    ldr r1, [r1]
    cmp r0, r1
    bne fail_6

    /* Map SRAM B as DTCM at 0x20000000 and read it as data. */
    ldr r0, =DTCM_AHB
    ldr r1, =0xd7c05eed
    str r1, [r0]
    ldr r0, =0x20000019
    mcr p15, 0, r0, c9, c1, 0
    ldr r0, =0x20000000
    ldr r0, [r0]
    cmp r0, r1
    bne fail_7

    /* If regions overlap, DTCM wins data while ITCM wins instructions. */
    ldr r0, =0x10000019
    mcr p15, 0, r0, c9, c1, 0
    ldr r4, =0x10000000
    ldr r0, [r4]
    cmp r0, r1
    bne fail_8
    mov r7, #0
    blx r4
    cmp r7, #0x5a
    bne fail_9

    /*
     * Instruction fetches never use DTCM.  Put a routine in ordinary DDR,
     * map DTCM over the same physical range, and verify code still executes
     * from DDR while a data load sees SRAM B.
     */
    ldr r0, =dtcm_fallthrough_start
    ldr r1, =0x70020000
    ldr r2, =dtcm_fallthrough_end
copy_ddr:
    cmp r0, r2
    ldrlo r3, [r0], #4
    strlo r3, [r1], #4
    blo copy_ddr
    ldr r0, =0x70020019
    mcr p15, 0, r0, c9, c1, 0
    mov r8, #0
    ldr r4, =0x70020000
    ldr r0, [r4]
    ldr r1, =0xd7c05eed
    cmp r0, r1
    bne fail_10
    blx r4
    cmp r8, #0xa5
    bne fail_11

    /* Recombining SRAM retains both physical blocks' contents. */
    ldr r4, =MATRIX_TCMR
    mov r0, #0
    str r0, [r4]
    mrc p15, 0, r0, c0, c0, 2
    cmp r0, #0
    bne fail_12
    ldr r0, =SRAM_AHB
    ldr r1, [r0]
    ldr r2, =itcm_code_start
    ldr r2, [r2]
    cmp r1, r2
    bne fail_13
    add r0, r0, #0x8000
    ldr r1, [r0]
    ldr r2, =0xd7c05eed
    cmp r1, r2
    bne fail_14

    ldr r1, =pass_msg
    b finish

fail_1:  mov r9, #1; ldr r1, =fail_1_msg;  b finish
fail_2:  mov r9, #1; ldr r1, =fail_2_msg;  b finish
fail_3:  mov r9, #1; ldr r1, =fail_3_msg;  b finish
fail_4:  mov r9, #1; ldr r1, =fail_4_msg;  b finish
fail_5:  mov r9, #1; ldr r1, =fail_5_msg;  b finish
fail_6:  mov r9, #1; ldr r1, =fail_6_msg;  b finish
fail_7:  mov r9, #1; ldr r1, =fail_7_msg;  b finish
fail_8:  mov r9, #1; ldr r1, =fail_8_msg;  b finish
fail_9:  mov r9, #1; ldr r1, =fail_9_msg;  b finish
fail_10: mov r9, #1; ldr r1, =fail_10_msg; b finish
fail_11: mov r9, #1; ldr r1, =fail_11_msg; b finish
fail_12: mov r9, #1; ldr r1, =fail_12_msg; b finish
fail_13: mov r9, #1; ldr r1, =fail_13_msg; b finish
fail_14: mov r9, #1; ldr r1, =fail_14_msg

finish:
    mov r0, #4
    svc 0x123456
    cmp r9, #0
    ldreq r1, =0x20026
    ldrne r1, =0x20024
    mov r0, #0x18
    svc 0x123456
1:  b 1b

    .align 2
itcm_code_start:
    mov r7, #0x5a
    bx lr
itcm_code_end:

dtcm_fallthrough_start:
    mov r8, #0xa5
    bx lr
dtcm_fallthrough_end:

pass_msg:    .asciz "AT91 TCM PASS\n"
fail_1_msg:  .asciz "AT91 TCM FAIL 1\n"
fail_2_msg:  .asciz "AT91 TCM FAIL 2\n"
fail_3_msg:  .asciz "AT91 TCM FAIL 3\n"
fail_4_msg:  .asciz "AT91 TCM FAIL 4\n"
fail_5_msg:  .asciz "AT91 TCM FAIL 5\n"
fail_6_msg:  .asciz "AT91 TCM FAIL 6\n"
fail_7_msg:  .asciz "AT91 TCM FAIL 7\n"
fail_8_msg:  .asciz "AT91 TCM FAIL 8\n"
fail_9_msg:  .asciz "AT91 TCM FAIL 9\n"
fail_10_msg: .asciz "AT91 TCM FAIL 10\n"
fail_11_msg: .asciz "AT91 TCM FAIL 11\n"
fail_12_msg: .asciz "AT91 TCM FAIL 12\n"
fail_13_msg: .asciz "AT91 TCM FAIL 13\n"
fail_14_msg: .asciz "AT91 TCM FAIL 14\n"
