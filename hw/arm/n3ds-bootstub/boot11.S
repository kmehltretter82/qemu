/*
 * n3ds boot11 stub ROM (64 KiB at ARM11 physical 0, mirrored at 0x10000 and
 * 0xFFFF0000). Clean-room stand-in for the parts of the real boot ROM that
 * firmware relies on:
 *  - reset: every core enables its GIC CPU interface; core 0 enables the
 *    distributor and waits for the Luma3DS ARM11 entry mailbox (0x1FFFFFFC),
 *    cores 1-3 park in the SMP wait routine
 *  - 0x004C: SMP wait routine (WFI, then branch to the word at 0x1FFFFFDC)
 *    that firmlaunch code jumps to on cores 1-3
 *  - the utility functions homebrew calls by address (GodMode9 common/bfn.h):
 *    cache maintenance, barriers, critical sections, waitCycles, cpuSet/Cpy
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
    .arm
    .text
    .org 0x0000
vectors:
    b   reset
    b   .
    b   .
    b   .
    b   .
    b   .
    b   .
    b   .

    .org 0x004C
smp_wait:                       @ LEGACY_BOOT_ROUTINE_SMP
    ldr r3, =0x17E00100
    mov r4, #1
    str r4, [r3]                @ GICC_CTLR: enable CPU interface
    mov r4, #0xF0
    str r4, [r3, #4]            @ GICC_PMR
    ldr r1, =0x1FFFFFDC
1:  wfi
    ldr r2, [r1]
    cmp r2, #0
    beq 1b
    bx  r2
    .ltorg

    .org 0x0100
reset:
    ldr r3, =0x17E00100
    mov r4, #1
    str r4, [r3]                @ GICC_CTLR
    mov r4, #0xF0
    str r4, [r3, #4]            @ GICC_PMR
    mrc p15, 0, r0, c0, c0, 5
    and r0, r0, #3
    cmp r0, #0
    bne smp_wait
    ldr r3, =0x17E01000
    mov r4, #1
    str r4, [r3]                @ GICD_CTLR: distributor enable
    ldr r1, =0x1FFFFFFC
1:  ldr r2, [r1]
    cmp r2, #0
    beq 1b
    bx  r2
    .ltorg

    .org 0x1288                 @ bool enableDCache(void)
    mrc p15, 0, r1, c1, c0, 0
    and r0, r1, #4
    orr r1, r1, #4
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r0, #2
    bx  lr
    .org 0x12A0                 @ bool disableDCache(void)
    mrc p15, 0, r1, c1, c0, 0
    and r0, r1, #4
    bic r1, r1, #4
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r0, #2
    bx  lr
    .org 0x12B8                 @ bool setDCache(bool enable)
    mrc p15, 0, r1, c1, c0, 0
    and r2, r1, #4
    cmp r0, #0
    biceq r1, r1, #4
    orrne r1, r1, #4
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r2, #2
    bx  lr
    .org 0x12E0                 @ void invalidateDCache(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c6, 0
    bx  lr
    .org 0x12EC                 @ void writebackDCache(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c10, 0
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x1320                 @ void writebackInvalidateDCache(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c14, 0
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x1358                 @ void invalidateDCacheRange(u32 start, u32 len)
    add r1, r0, r1
    bic r0, r0, #31
1:  mcr p15, 0, r0, c7, c6, 1
    add r0, r0, #32
    cmp r0, r1
    blo 1b
    bx  lr
    .org 0x1374                 @ void writebackDCacheRange(u32 start, u32 len)
    add r1, r0, r1
    bic r0, r0, #31
1:  mcr p15, 0, r0, c7, c10, 1
    add r0, r0, #32
    cmp r0, r1
    blo 1b
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x1398                 @ void writebackInvalidateDCacheRange(u32 start, u32 len)
    add r1, r0, r1
    bic r0, r0, #31
1:  mcr p15, 0, r0, c7, c14, 1
    add r0, r0, #32
    cmp r0, r1
    blo 1b
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x13C0                 @ void dataSynchronizationBarrier(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x13E8                 @ void dataMemoryBarrier(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c10, 5
    bx  lr
    .org 0x13F4                 @ bool enableICache(void)
    mrc p15, 0, r1, c1, c0, 0
    and r0, r1, #0x1000
    orr r1, r1, #0x1000
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r0, #12
    bx  lr
    .org 0x140C                 @ bool disableICache(void)
    mrc p15, 0, r1, c1, c0, 0
    and r0, r1, #0x1000
    bic r1, r1, #0x1000
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r0, #12
    bx  lr
    .org 0x1424                 @ bool setICache(bool enable)
    mrc p15, 0, r1, c1, c0, 0
    and r2, r1, #0x1000
    cmp r0, #0
    biceq r1, r1, #0x1000
    orrne r1, r1, #0x1000
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r2, #12
    bx  lr
    .org 0x144C                 @ void invalidateICache(void)  (+ branch target cache)
    mov r0, #0
    mcr p15, 0, r0, c7, c5, 0
    bx  lr
    .org 0x1458                 @ void invalidateICacheRange(u32 start, u32 len)
    add r1, r0, r1
    bic r0, r0, #31
1:  mcr p15, 0, r0, c7, c5, 1
    add r0, r0, #32
    cmp r0, r1
    blo 1b
    bx  lr
    .org 0x1490                 @ void instructionSynchronizationBarrier(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c5, 4
    bx  lr
    .org 0x14F4                 @ void invalidateBranchTargetCacheRange(u32 start, u32 len)
    add r1, r0, r1
    bic r0, r0, #31
1:  mcr p15, 0, r0, c7, c5, 7
    add r0, r0, #32
    cmp r0, r1
    blo 1b
    bx  lr
    .org 0x16E4                 @ void cpuSet(u32 val, u32 *dest, u32 count) (words)
    cmp r2, #0
    bxeq lr
1:  str r0, [r1], #4
    subs r2, r2, #1
    bne 1b
    bx  lr
    .org 0x1730                 @ void cpuCpy(const u32 *src, u32 *dest, u32 count) (words)
    cmp r2, #0
    bxeq lr
1:  ldr r3, [r0], #4
    str r3, [r1], #4
    subs r2, r2, #1
    bne 1b
    bx  lr
    .org 0x1A38                 @ void waitCycles(u32 cycles)
1:  subs r0, r0, #4
    bhi 1b
    bx  lr
    .org 0x1AC4                 @ u32 enterCriticalSection(void)
    mrs r0, cpsr
    cpsid i
    and r0, r0, #0x80
    bx  lr
    .org 0x1AD8                 @ void leaveCriticalSection(u32 state)
    mrs r1, cpsr
    bic r1, r1, #0x80
    and r0, r0, #0x80
    orr r1, r1, r0
    msr cpsr_c, r1
    bx  lr
    .org 0x1B00
