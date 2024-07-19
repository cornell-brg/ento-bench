ENTRY(_Reset)
SECTIONS
{
    . = 0x10000;
    .boot . : { *(.text) }
    .text : { *(.text) }
    .data : { *(.data) }
    .bss : { *(.bss COMMON) }
    . = ALIGN(8);
    . = . + 0x1000; /* 2kB of stack memory */
    stack_top = .;

    /* Adjust heap size, needed for printf float support */
    .heap (NOLOAD) :
    {
      _heap_start = .;
      /* Printf float support apparently works with a heap size of 1 byte... */
      . = . + 0x1000;
      _heap_end = .;
    }

    PROVIDE (end = .)   ;
}
