// startup_cm4.s
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global _start
.global Reset_Handler

// Stack configuration
.equ STACK_TOP, 0x20001000  // Adjust stack size as per your MCU's RAM size

// Vector Table
.section .isr_vector, "a", %progbits
.type vector_table, %object
.size vector_table, .-vector_table
vector_table:
    .word   STACK_TOP                  // Initial Stack Pointer
    .word   Reset_Handler              // Reset Handler
    .word   NMI_Handler                // NMI Handler
    .word   HardFault_Handler          // Hard Fault Handler
    .word   MemManage_Handler          // MPU Fault Handler
    .word   BusFault_Handler           // Bus Fault Handler
    .word   UsageFault_Handler         // Usage Fault Handler
    .word   0                          // Reserved
    .word   0                          // Reserved
    .word   0                          // Reserved
    .word   0                          // Reserved
    .word   SVC_Handler                // SVCall Handler
    .word   DebugMon_Handler           // Debug Monitor Handler
    .word   0                          // Reserved
    .word   PendSV_Handler             // PendSV Handler
    .word   SysTick_Handler            // SysTick Handler

// Default handlers
.section .text
Default_Handler:
    b .

NMI_Handler:
    b Default_Handler

HardFault_Handler:
    b Default_Handler

MemManage_Handler:
    b Default_Handler

BusFault_Handler:
    b Default_Handler

UsageFault_Handler:
    b Default_Handler

SVC_Handler:
    b Default_Handler

DebugMon_Handler:
    b Default_Handler

PendSV_Handler:
    b Default_Handler

SysTick_Handler:
    b Default_Handler

// Reset handler
.section .text
Reset_Handler:
    // Initialize data and bss sections
    ldr r0, =_etext
    ldr r1, =_sdata
    ldr r2, =_edata
    ldr r3, =_sbss
    ldr r4, =_ebss

    // Copy data section
CopyData:
    cmp r1, r2
    bcc CopyDataLoop
    b CopyBss

CopyDataLoop:
    ldr r5, [r0], #4
    str r5, [r1], #4
    b CopyData

// Zero initialize bss section
CopyBss:
    cmp r3, r4

