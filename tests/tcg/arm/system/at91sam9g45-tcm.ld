/* SPDX-License-Identifier: GPL-2.0-or-later */

ENTRY(_start)
SECTIONS
{
    . = 0x70010000;
    .text : { *(.text*) }
    /DISCARD/ : { *(.ARM.attributes) *(.comment) }
}
