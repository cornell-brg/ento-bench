#include <stdlib.h>
#include <stdio.h>
#include <ento-bench/harness.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

static inline float fixed16_16_to_float(int32_t x)
{
  // Convert 16.16 fixed-point to float
  return (float)x / (1 << 16);
}

void __attribute__((noinline)) hello_host_computer()
{
  printf("Hello host computer!\n");
}

void __attribute__((noinline)) add_fixed()
{
  constexpr int reps = 10000;
  start_roi();
  const int32_t one_fixed = (1 << 16);
  volatile int32_t sum = 0;

  for (int i = 0; i < reps; ++i) {
    sum += one_fixed;
  }
  end_roi();

  printf("Final sum: %d\n", sum);
}


// // Fixed point addition benchmark
// void __attribute__((noinline)) fixed_point_add() {
//   constexpr int reps = 800;
//   for (int i = 0; i < reps; i++)
//     asm volatile (
//       ".rept 8       \n"   // Repeat 8 times
//       // Simple fixed point addition in Q16.16 format
//       "add r0, r0, r1  \n"  // r0 += r1
//       "add r2, r2, r3  \n"  // r2 += r3
//       "add r4, r4, r5  \n"  // r4 += r5
//       "add r6, r6, r7  \n"  // r6 += r7
      
//       // Saturated addition using QADD (if available on target)
//       "qadd r0, r0, r2 \n"  // Saturating add
//       "qadd r4, r4, r6 \n"  // Saturating add
      
//       // More fixed point operations
//       "add r0, r0, #0x10000 \n"  // Add 1.0 in Q16.16
//       "sub r4, r4, #0x8000  \n"  // Subtract 0.5 in Q16.16
//       ".endr           \n"       // End repeat block
//       : // No outputs
//       : // No inputs
//       : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"  // Clobbered registers
//     );
//     end_roi();
// }


int main()
{
  using namespace EntoBench;

#if defined(SEMIHOSTING)
  initialise_monitor_handles();
#endif

  bool is_systick_enabled = (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0;
  printf("Is systick enabled: %i\n", is_systick_enabled);

  // Configure max clock rate and set flash latency
  sys_clk_cfg();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  printf("==========================\n");
  printf("Running fixed-point addition microbenchmark.\n");
  printf("==========================\n\n");

  uint32_t clk_freq = get_sys_clk_freq();
  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

  uint32_t flash_latency = get_flash_latency();
  printf("Current flash latency: %i\n", flash_latency);
  printf("==========================\n\n");

  // Define the benchmarking problem
  auto add_problem = EntoBench::make_basic_problem(add_fixed);

  using AddHarness = EntoBench::Harness<decltype(add_problem), true, 1>;
  AddHarness add_harness(add_problem, "Fixed-Point Addition Microbenchmark");
  add_harness.run();

  printf("==========================\n\n");

  // Exit
  exit(1);
  return 0;
}

