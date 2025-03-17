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

void __attribute__((noinline)) add6400x8(){
  unsigned int r0, r1, r2, r3, r4, r5, r6, r7;
  asm volatile(
    "mov %0, #0x55555555    \n"
    "mov %1, #0xAAAAAAAA    \n"
    "mov %2, #0x55555555    \n"
    "mov %3, #0xAAAAAAAA    \n"
    "mov %4, #0x55555555    \n"
    "mov %5, #0xAAAAAAAA    \n"
    "mov %6, #0x55555555    \n"
    "mov %7, #0xAAAAAAAA    \n"
    : "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3),
      "=r" (r4), "=r" (r5), "=r" (r6), "=r" (r7) 
    : 
    : 
  );
  constexpr int reps = 10000;
  start_roi();

  for (int i = 0; i < reps; i++) {
    asm volatile (
      ".rept 8         \n"
      "add r8, %[r0], %[r1]  \n"
      "add r8, %[r2], %[r3]  \n"
      "add r8, %[r4], %[r5]  \n"
      "add r8, %[r6], %[r7]  \n"
  
      "add r8, %[r0], %[r1]  \n"
      "add r8, %[r2], %[r3]  \n"
      "add r8, %[r4], %[r5]  \n"
      "add r8, %[r6], %[r7]  \n"
      ".endr           \n"
      : 
      : [r0] "r" (r0), [r1] "r" (r1), [r2] "r" (r2), [r3] "r" (r3), 
        [r4] "r" (r4), [r5] "r" (r5), [r6] "r" (r6), [r7] "r" (r7) // Input operands
      : "r8"  // Clobbered registers
    );
  
  }
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

  const char add6400x8_name[] = "ADD 6400x8 Microbenchmark";
  auto problem_add6400x8 = EntoBench::make_basic_problem(add6400x8);
  using Harness6400x8 = EntoBench::Harness<decltype(problem_add6400x8), true, 1>;
  Harness6400x8 add6400x8_harness(problem_add6400x8, add6400x8_name);

  add6400x8_harness.run();

  printf("==========================\n\n");

  exit(1);

  return 0;
}
