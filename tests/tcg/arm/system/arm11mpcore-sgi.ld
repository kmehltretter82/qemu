/* SPDX-License-Identifier: GPL-2.0-or-later */

ENTRY(_start)

SECTIONS
{
    . = 0x00010000;
    .text : { *(.text.boot) *(.text*) }
    .rodata : { *(.rodata*) }
    /DISCARD/ : { *(.ARM.attributes) *(.comment) }
}
