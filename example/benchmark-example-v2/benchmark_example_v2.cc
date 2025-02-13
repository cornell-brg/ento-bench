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

void __attribute__((noinline)) hello_host_computer()
{
  printf("Hello host computer!\n");
}

void __attribute__((noinline)) add64x8()
{
  start_roi();
  asm (
    ".rept 64        \n"   // Repeat 8 times
    "add r0, r0, #1  \n"   // Add 1 to r0
    "add r1, r1, #1  \n"   // Add 1 to r1
    "add r2, r2, #1  \n"   // Add 1 to r2
    "add r3, r3, #1  \n"   // Add 1 to r3
    "add r4, r4, #1  \n"   // Add 1 to r4
    "add r5, r5, #1  \n"   // Add 1 to r5
    "add r6, r6, #1  \n"   // Add 1 to r6
    "add r7, r7, #1  \n"   // Add 1 to r7
    ".endr            \n"   // End repeat block
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"  // Clobbered registers
  );
  end_roi();
}

void __attribute__((noinline)) add16x8()
{
  start_roi();
  asm (
    ".rept 16        \n"   // Repeat 8 times
    "add r0, r0, #1  \n"   // Add 1 to r0
    "add r1, r1, #1  \n"   // Add 1 to r1
    "add r2, r2, #1  \n"   // Add 1 to r2
    "add r3, r3, #1  \n"   // Add 1 to r3
    "add r4, r4, #1  \n"   // Add 1 to r4
    "add r5, r5, #1  \n"   // Add 1 to r5
    "add r6, r6, #1  \n"   // Add 1 to r6
    "add r7, r7, #1  \n"   // Add 1 to r7
    ".endr            \n"   // End repeat block
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"  // Clobbered registers
  );
  end_roi();
}

inline void add4x8()
{
  start_roi();
  asm volatile (
    ".rept 4        \n"   // Repeat 8 times
    "add r0, r0, #1  \n"   // Add 1 to r0
    "add r1, r1, #1  \n"   // Add 1 to r1
    "add r2, r2, #1  \n"   // Add 1 to r2
    "add r3, r3, #1  \n"   // Add 1 to r3
    "add r4, r4, #1  \n"   // Add 1 to r4
    "add r5, r5, #1  \n"   // Add 1 to r5
    "add r6, r6, #1  \n"   // Add 1 to r6
    "add r7, r7, #1  \n"   // Add 1 to r7
    ".endr            \n"   // End repeat block
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"  // Clobbered registers
  );
  end_roi();
}

void __attribute__((noinline)) add4096x8()
{
  start_roi();
#if !defined(STM32G0)
  constexpr int reps = 128;
  for (int i = 0; i < reps; i++)
    asm volatile (
      ".rept 32       \n"   // Repeat 8 times
      "add r0, r0, #1  \n"   // Add 1 to r0
      "add r1, r1, #1  \n"   // Add 1 to r1
      "add r2, r2, #1  \n"   // Add 1 to r2
      "add r3, r3, #1  \n"   // Add 1 to r3
      "add r4, r4, #1  \n"   // Add 1 to r4
      "add r5, r5, #1  \n"   // Add 1 to r5
      "add r6, r6, #1  \n"   // Add 1 to r6
      "add r7, r7, #1  \n"   // Add 1 to r7
      ".endr           \n"   // End repeat block
      : // No outputs
      : // No inputs
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"  // Clobbered registers
    );
#else
  constexpr int reps = 512;
  for (int i = 0; i < reps; i++)
    asm volatile (
      ".rept 8       \n"   // Repeat 8 times
      "add r0, r0, #1  \n"   // Add 1 to r0
      "add r1, r1, #1  \n"   // Add 1 to r1
      "add r2, r2, #1  \n"   // Add 1 to r2
      "add r3, r3, #1  \n"   // Add 1 to r3
      "add r4, r4, #1  \n"   // Add 1 to r4
      "add r5, r5, #1  \n"   // Add 1 to r5
      "add r6, r6, #1  \n"   // Add 1 to r6
      "add r7, r7, #1  \n"   // Add 1 to r7
      ".endr           \n"   // End repeat block
      : // No outputs
      : // No inputs
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"  // Clobbered registers
    );

#endif
  end_roi();

}

int main()
{
  using namespace EntoBench;
#if defined(SEMIHOSTING)
  initialise_monitor_handles();
#endif

  bool is_systick_enabled = (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0;

  printf("Is systick enabled: %i\n", is_systick_enabled);
  constexpr int reps = 5;

  // Configure max clock rate and set flash latency
  sys_clk_cfg();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  printf("==========================\n");
  printf("Running example microbenchmarks.\n");
  printf("==========================\n\n");

  uint32_t clk_freq = get_sys_clk_freq();

  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

  uint32_t flash_latency = get_flash_latency();

  printf("Current flash latency: %i\n", flash_latency);
  printf("==========================\n\n");

  const char add4x8_name[] = "Add 4x8 Microbenchmark";
  const char add16x8_name[] = "Add 16x8 Microbenchmark";
  const char add64x8_name[] = "Add 64x8 Microbenchmark";
  const char add4096x8_name[] = "Add 4096x8 Microbenchmark";
  auto problem_add4x8    = EntoBench::make_basic_problem(add4x8);
  auto problem_add16x8   = EntoBench::make_basic_problem(add16x8);
  auto problem_add64x8   = EntoBench::make_basic_problem(add64x8);
  auto problem_add4096x8 = EntoBench::make_basic_problem(add4096x8);
  using Harness4x8    = EntoBench::Harness<decltype(problem_add4x8),    true, 1>;
  using Harness16x8   = EntoBench::Harness<decltype(problem_add16x8),   true, 1>;
  using Harness64x8   = EntoBench::Harness<decltype(problem_add64x8),   true, 1>;
  using Harness4096x8 = EntoBench::Harness<decltype(problem_add4096x8), true, 1>;
  Harness4x8    add4x8_harness(problem_add4x8, add4x8_name);
  Harness16x8   add16x8_harness(problem_add16x8, add16x8_name);
  Harness64x8   add64x8_harness(problem_add64x8, add64x8_name);
  Harness4096x8 add4096x8_harness(problem_add4096x8, add4096x8_name);

  add4x8_harness.run();
  add16x8_harness.run();
  add64x8_harness.run();
  add4096x8_harness.run();

  printf("==========================\n\n");


  exit(1);

  return 0;
}
