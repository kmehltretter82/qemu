/* Minimal reset entry: set up a stack, jump to C, never return. */
.syntax unified
.arm

.section .text
.global _start
_start:
    ldr sp, =stack_top
    bl c_main
1:  b 1b

.section .bss
.align 4
stack:
    .space 4096
stack_top:
