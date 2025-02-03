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

ssize_t _read(int file, void *ptr, size_t len)
{
  ssize_t result;
  asm volatile(
    "mov r0, %[file]\n"     // Move file descriptor into r0
    "mov r1, %[ptr]\n"      // Move pointer to buffer into r1
    "mov r2, %[len]\n"      // Move length of data to read into r2
    "mov r7, #3\n"          // syscall number for 'read' on ARM Linux
    "svc 0\n"               // Trigger software interrupt to invoke syscall
    "mov %[result], r0\n"   // Move result from r0 to C variable 'result'
    : [result] "=r" (result) // Output operand
    : [file] "r" (file), [ptr] "r" (ptr), [len] "r" (len) // Input operands
    : "r0", "r1", "r2", "r7" // Clobbered regs
  );

  return result;
}

int _open(const char *path, int flags, int mode)
{
    int result;

    asm volatile(
      "mov r0, %[path]\n"     // Move file path into r0
      "mov r1, %[flags]\n"    // Move flags into r1
      "mov r2, %[mode]\n"     // Move mode into r2
      "mov r7, #5\n"          // syscall number for 'open' on ARM Linux
      "svc 0\n"               // Trigger software interrupt to invoke syscall
      "mov %[result], r0\n"   // Move result from r0 to C variable 'result'
      : [result] "=r" (result) // Output operand
      : [path] "r" (path), [flags] "r" (flags), [mode] "r" (mode) // Input operands
      : "r0", "r1", "r2", "r7" // Clobbered regs
    );

    return result;  // Return the file descriptor or error code
}


int _close(int file)
{
  int result;

  asm volatile(
    "mov r0, %[file]\n"     // Move file descriptor into r0
    "mov r7, #6\n"          // syscall number for 'close' on ARM Linux
    "svc 0\n"               // Trigger software interrupt to invoke syscall
    "mov %[result], r0\n"   // Move result from r0 to C variable 'result'
    : [result] "=r" (result) // Output operand
    : [file] "r" (file)      // Input operand
    : "r0", "r7"            // Clobbered regs
  );

  return result;  // Return the success or failure code
}

off_t _lseek(int file, off_t offset, int whence)
{
  off_t result;

  asm volatile(
    "mov r0, %[file]\n"     // Move file descriptor into r0
    "mov r1, %[offset]\n"   // Move offset into r1
    "mov r2, %[whence]\n"   // Move whence into r2
    "mov r7, #19\n"         // syscall number for 'lseek' on ARM Linux
    "svc 0\n"               // Trigger software interrupt to invoke syscall
    "mov %[result], r0\n"   // Move result from r0 to C variable 'result'
    : [result] "=r" (result) // Output operand
    : [file] "r" (file), [offset] "r" (offset), [whence] "r" (whence) // Input operands
    : "r0", "r1", "r2", "r7" // Clobbered regs
  );

  return result;  // Return the new offset or error code
}
