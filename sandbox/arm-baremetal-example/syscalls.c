#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>

#define UNUSED __attribute__((unused))

//GEM5_MACHINETYPE=VExpress_GEM5_V1
//volatile unsigned int *const UART0DR = (unsigned int *)0x1c090000;

__attribute__((section(".txt")))
const uint32_t __exidx_start = 0;
__attribute__((section(".txt")))
const uint32_t __exidx_end = 0;


// Prototype of _write, may need to be adjusted if not already defined
ssize_t _write(int file, const void *ptr, size_t len);

ssize_t write(int file, const void *ptr, size_t len) {
    return _write(file, ptr, len);  // Call low-level _write directly
}

// Define _write if gem5 does not provide it
ssize_t _write(int file, const void *ptr, size_t len)
{
    ssize_t result;

    asm volatile(
        "mov r0, %[file]\n"     // Move file descriptor into r0
        "mov r1, %[ptr]\n"      // Move pointer to data into r1
        "mov r2, %[len]\n"      // Move length of data into r2
        "mov r7, #4\n"          // syscall number for 'write' on ARM Linux
        "svc 0\n"               // Trigger software interrupt to invoke syscall
        "mov %[result], r0\n"   // Move result from r0 to C variable 'result'
        : [result] "=r" (result) // Output operand
        : [file] "r" (file), [ptr] "r" (ptr), [len] "r" (len) // Input operands
        : "r0", "r1", "r2", "r7" // Clobbered regs
    );

    return result;  // Simplistic, assuming full write success
}

