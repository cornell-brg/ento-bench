/* Entry Point */
ENTRY(_Reset)

/* Define the start and size of memory regions */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x10000, LENGTH = 2M
}

/* Minimum heap and stack sizes */
_Min_Heap_Size = 0x2000;   /* 512 bytes of heap */
_Min_Stack_Size = 0x80000; /* 32KB of stack */

/* Sections */
SECTIONS
{
  /* Code section */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* All .text sections */
    *(.text*)          /* All .text* sections */
    . = ALIGN(4);
    _etext = .;        /* End of code section */
  } > RAM


  /* Read-only data */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* All .rodata sections */
    *(.rodata*)        /* All .rodata* sections */
    . = ALIGN(4);
  } > RAM

  /* Initialized data */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* Start of data section */
    *(.data)           /* All .data sections */
    *(.data*)          /* All .data* sections */
    . = ALIGN(4);
    _edata = .;        /* End of data section */
  } > RAM

  /* Uninitialized data (BSS) */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;         /* Start of bss section */
    *(.bss)            /* All .bss sections */
    *(.bss*)           /* All .bss* sections */
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;         /* End of bss section */
  } > RAM

  /* Heap and Stack allocation */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( _end = . );          /* End of all previous sections */
    PROVIDE ( end = _end );        /* Define 'end' symbol for sbrk */
    . = . + _Min_Heap_Size;        /* Allocate heap */
    . = . + _Min_Stack_Size;       /* Allocate stack */
    PROVIDE ( stack_top = . );     /* Define 'stack_top' for boot.s */
    . = ALIGN(8);
  } > RAM

  /* Remove unnecessary information */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }
}

