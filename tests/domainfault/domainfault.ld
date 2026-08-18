ENTRY(_vectors)

SECTIONS
{
    . = 0x00000000;
    .vectors : { *(.vectors) }
    .text : { *(.text*) }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    .bss : { *(.bss*) *(COMMON) }
    /DISCARD/ : { *(.comment) *(.ARM.attributes) }
}
