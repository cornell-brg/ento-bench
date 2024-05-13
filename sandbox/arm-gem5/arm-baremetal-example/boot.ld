ENTRY(_Reset)
SECTIONS
{
    . = 0x10000;
    .boot . : { boot.o(.text) }
    .text : { *(.text) }
    .data : { *(.data) }
    .bss : { *(.bss COMMON) }
    . = ALIGN(8);
    . = . + 0x500; /* 2kB of stack memory */
    stack_top = .;
    PROVIDE (end = .)   ;
}
