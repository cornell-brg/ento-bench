#ifndef ROI_H
#define ROI_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef STM32_BUILD
#include <stm32-util/timing.h>
#endif

#include <stdio.h>

#ifdef GEM5
#include <sys/syscalls.c>
//#include <gem5/m5ops.h>
#endif

  
#ifdef ARM_GEM5
#define M5OP_RESET_STATS \
  __asm__ __volatile__ ( \
      ".align 2\n\t" \
      ".short 0xEE00 | (0x40) \n\t" \
      ".short 0x0110\n\t" \
      : : : "r0", "r1", "r2", "r3" \
  )
#define M5OP_DUMP_STATS \
  __asm__ __volatile__ ( \
      "mov r0, #0\n\t" \
      "mov r1, #0\n\t" \
      ".align 2\n\t" \
      ".short 0xEE00 | (0x41) \n\t" \
      ".short 0x0110\n\t" \
      : : : "r0", "r1", "r2", "r3" \
  )
#define M5OP_DUMP_RESET_STATS \
  __asm__ __volatile__ ( \
      ".align 2\n\t" \
      ".short 0xEE00 | (0x42) \n\t" \
      ".short 0x0110\n\t" \
      : : : "r0", "r1", "r2", "r3" \
  )
#define M5OP_EXIT \
  __asm__ __volatile__ ( \
      ".align 2\n\t" \
      ".short 0xEE00 | (0x21) \n\t" \
      ".short 0x0110\n\t" \
      : : : "r0", "r1", "r2", "r3" \
  )
#endif


/*
#ifdef ARM_GEM5
//#define M5OP_RESET_STATS 
// Define the macro for m5op functions
#define M5OP_FUNC(name, func) \
  __asm__ __volatile__ ( \
    ".syntax unified\n\t" \
    ".thumb\n\t" \
    ".align 2\n\t" \
    ".globl " #name "\n\t" \
    ".thumb_func\n\t" \
    #name ":\n\t" \
    ".short 0xEE00 | (" #func ")\n\t" \
    ".short 0x0110\n\t" \
    "bx lr\n\t" \
    : : : "r0", "r1", "r2", "r3" \
  )
// Use the macro to define the required m5op functions
M5OP_FUNC(m5_reset_stats, 0x40)       // Example function, replace func values as needed
M5OP_FUNC(m5_dump_stats, 0x41)
M5OP_FUNC(m5_dump_reset_stats, 0x42)
M5OP_FUNC(m5_exit, 0x21)

#undef M5OP_FUNC
#endif
*/
// Add other m5op functions as needed


static int g_cycles = 0;
static inline void start_roi(void) __attribute__((always_inline));
static inline void end_roi(void) __attribute__ ((always_inline));
static inline int  get_cycles(void) __attribute__((always_inline));

// Inline Implementations
static inline
void start_roi(void)
{
#ifdef STM32_BUILD
  if (is_cycle_counter_init()) update_last_cycle_count();
  else init_cycle_counter();
  init_cycle_counter();
#elif defined(RISCV_GEM5) || defined(ARM_GEM5)
  M5OP_RESET_STATS;
  //m5_dump_reset_stats();
   
#endif
}

static inline
void end_roi(void)
{
#ifdef STM32_BUILD
  uint32_t cycles = get_elapsed_cycles();
  g_cycles = cycles;
  //printf("Cycles elapsed: %lu\n", cycles);
#elif defined(RISCV_GEM5) || defined(ARM_GEM5)
  M5OP_DUMP_RESET_STATS;
  //m5_dump_reset_stats();
#endif
}

static inline
int get_cycles()
{
#ifdef STM32_BUILD
  return g_cycles;
#endif
  return 0;

}

#ifdef __cplusplus
}
#endif

#endif // ROI_H
