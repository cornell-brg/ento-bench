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

void __attribute__((noinline)) add11200x8() {
  constexpr int reps = 2000;
  start_roi();
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


  const char add11200x8_name[] = "Add 11200x8 Microbenchmark";
  auto problem_add11200x8 = EntoBench::make_basic_problem(add11200x8);
  using Harness11200x8 = EntoBench::Harness<decltype(problem_add11200x8), true, 1>;
  Harness11200x8 add11200x8_harness(problem_add11200x8, add11200x8_name);

  add11200x8_harness.run();

  printf("==========================\n\n");


  exit(1);

  return 0;
}
