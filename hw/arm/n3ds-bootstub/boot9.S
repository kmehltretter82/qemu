/*
 * n3ds boot9 stub ROM (64 KiB at ARM9 0xFFFF0000). Clean-room stand-in:
 *  - exception vectors branch to the 8-byte slots in ARM9 WRAM that
 *    firmware installs handlers in, in the boot ROM's order (IRQ 0x08000000,
 *    FIQ +8, SVC +0x10, UNDEF +0x18, PABT +0x20, DABT +0x28); reset hangs
 *  - the utility functions firmware calls by address (GodMode9 common/bfn.h,
 *    Luma3DS): cache maintenance, barriers, critical sections, waitCycles,
 *    cpuSet/Cpy, MPU enable/disable, resetControlRegisters
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
    .arm
    .text
    .org 0x0000
    ldr pc, [pc, #0x18]         @ reset  -> hang
    ldr pc, [pc, #0x18]         @ undef  -> 0x08000018
    ldr pc, [pc, #0x18]         @ svc    -> 0x08000010
    ldr pc, [pc, #0x18]         @ pabt   -> 0x08000020
    ldr pc, [pc, #0x18]         @ dabt   -> 0x08000028
    ldr pc, [pc, #0x18]         @ reserved -> hang
    ldr pc, [pc, #0x18]         @ irq    -> 0x08000000
    ldr pc, [pc, #0x18]         @ fiq    -> 0x08000008
    .word 0xFFFF0040
    .word 0x08000018
    .word 0x08000010
    .word 0x08000020
    .word 0x08000028
    .word 0xFFFF0040
    .word 0x08000000
    .word 0x08000008
    .org 0x0040
hang:
    b   hang

    .org 0x0198                 @ void waitCycles(u32 cycles)
1:  subs r0, r0, #4
    bhi 1b
    bx  lr
    .org 0x03A4                 @ void cpuSet(u32 val, u32 *dest, u32 count) (words)
    cmp r2, #0
    bxeq lr
1:  str r0, [r1], #4
    subs r2, r2, #1
    bne 1b
    bx  lr
    .org 0x03F0                 @ void cpuCpy(const u32 *src, u32 *dest, u32 count) (words)
    cmp r2, #0
    bxeq lr
1:  ldr r3, [r0], #4
    str r3, [r1], #4
    subs r2, r2, #1
    bne 1b
    bx  lr
    .org 0x06EC                 @ u32 enterCriticalSection(void)
    mrs r0, cpsr
    orr r1, r0, #0x80
    msr cpsr_c, r1
    and r0, r0, #0x80
    bx  lr
    .org 0x0700                 @ void leaveCriticalSection(u32 state)
    mrs r1, cpsr
    bic r1, r1, #0x80
    and r0, r0, #0x80
    orr r1, r1, r0
    msr cpsr_c, r1
    bx  lr
    .org 0x0798                 @ bool enableDCache(void)
    mrc p15, 0, r1, c1, c0, 0
    and r0, r1, #4
    orr r1, r1, #4
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r0, #2
    bx  lr
    .org 0x07B0                 @ bool disableDCache(void)
    mrc p15, 0, r1, c1, c0, 0
    and r0, r1, #4
    bic r1, r1, #4
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r0, #2
    bx  lr
    .org 0x07C8                 @ bool setDCache(bool enable)
    mrc p15, 0, r1, c1, c0, 0
    and r2, r1, #4
    cmp r0, #0
    biceq r1, r1, #4
    orrne r1, r1, #4
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r2, #2
    bx  lr
    .org 0x07F0                 @ void invalidateDCache(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c6, 0
    bx  lr
    .org 0x07FC                 @ void writebackDCache(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c10, 0
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x0830                 @ void writebackInvalidateDCache(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c14, 0
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x0868                 @ void invalidateDCacheRange(u32 start, u32 len)
    add r1, r0, r1
    bic r0, r0, #31
1:  mcr p15, 0, r0, c7, c6, 1
    add r0, r0, #32
    cmp r0, r1
    blo 1b
    bx  lr
    .org 0x0884                 @ void writebackDCacheRange(u32 start, u32 len)
    add r1, r0, r1
    bic r0, r0, #31
1:  mcr p15, 0, r0, c7, c10, 1
    add r0, r0, #32
    cmp r0, r1
    blo 1b
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x08A8                 @ void writebackInvalidateDCacheRange(u32 start, u32 len)
    add r1, r0, r1
    bic r0, r0, #31
1:  mcr p15, 0, r0, c7, c14, 1
    add r0, r0, #32
    cmp r0, r1
    blo 1b
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x096C                 @ void dataSynchronizationBarrier(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c10, 4
    bx  lr
    .org 0x0A5C                 @ bool enableICache(void)
    mrc p15, 0, r1, c1, c0, 0
    and r0, r1, #0x1000
    orr r1, r1, #0x1000
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r0, #12
    bx  lr
    .org 0x0A74                 @ bool disableICache(void)
    mrc p15, 0, r1, c1, c0, 0
    and r0, r1, #0x1000
    bic r1, r1, #0x1000
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r0, #12
    bx  lr
    .org 0x0A8C                 @ bool setICache(bool enable)
    mrc p15, 0, r1, c1, c0, 0
    and r2, r1, #0x1000
    cmp r0, #0
    biceq r1, r1, #0x1000
    orrne r1, r1, #0x1000
    mcr p15, 0, r1, c1, c0, 0
    lsr r0, r2, #12
    bx  lr
    .org 0x0AB4                 @ void invalidateICache(void)
    mov r0, #0
    mcr p15, 0, r0, c7, c5, 0
    bx  lr
    .org 0x0AC0                 @ void invalidateICacheRange(u32 start, u32 len)
    add r1, r0, r1
    bic r0, r0, #31
1:  mcr p15, 0, r0, c7, c5, 1
    add r0, r0, #32
    cmp r0, r1
    blo 1b
    bx  lr
    .org 0x0C38                 @ void enableMPU(void)
    mrc p15, 0, r0, c1, c0, 0
    orr r0, r0, #1
    mcr p15, 0, r0, c1, c0, 0
    bx  lr
    .org 0x0C48                 @ void disableMPU(void)
    mrc p15, 0, r0, c1, c0, 0
    bic r0, r0, #1
    mcr p15, 0, r0, c1, c0, 0
    bx  lr
    .org 0x0C58                 @ void resetControlRegisters(void)
    ldr r0, =0x00052078         @ MPU/caches off, high vectors, TCMs on
    mcr p15, 0, r0, c1, c0, 0
    mov r0, #0
    mcr p15, 0, r0, c7, c5, 0
    mcr p15, 0, r0, c7, c6, 0
    bx  lr
    .ltorg
    .org 0x0C80
