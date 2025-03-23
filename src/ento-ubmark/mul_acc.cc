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

void __attribute__((noinline)) mla6400x8()
{
  // Same approach as add6400x8, but zero-initialize the registers for MLA
  unsigned int r0, r1, r2, r3, r4, r5, r6, r7;
  asm volatile(
    "mov %0, #0    \n"
    "mov %1, #0    \n"
    "mov %2, #0    \n"
    "mov %3, #0    \n"
    "mov %4, #0    \n"
    "mov %5, #0    \n"
    "mov %6, #0    \n"
    "mov %7, #0    \n"
    : "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3),
      "=r" (r4), "=r" (r5), "=r" (r6), "=r" (r7)
    :  // no inputs
    :  
  );

  constexpr int reps = 20000;
  start_roi(); 

  for (int i = 0; i < reps; i++) {
    asm volatile (
      ".rept 8                        \n"
      "  mla r8, %[r0], %[r1], r8     \n"
      "  mla r8, %[r2], %[r3], r8     \n"
      "  mla r8, %[r4], %[r5], r8     \n"
      "  mla r8, %[r6], %[r7], r8     \n"

      "  mla r8, %[r0], %[r1], r8     \n"
      "  mla r8, %[r2], %[r3], r8     \n"
      "  mla r8, %[r4], %[r5], r8     \n"
      "  mla r8, %[r6], %[r7], r8     \n"
      ".endr                          \n"
      :
      : [r0] "r" (r0), [r1] "r" (r1),
        [r2] "r" (r2), [r3] "r" (r3),
        [r4] "r" (r4), [r5] "r" (r5),
        [r6] "r" (r6), [r7] "r" (r7)
      : "r8"
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
  printf("Running MLA microbenchmark with zero-initialized registers.\n");
  printf("==========================\n\n");

  uint32_t clk_freq = get_sys_clk_freq();
  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

  uint32_t flash_latency = get_flash_latency();
  printf("Current flash latency: %i\n", flash_latency);
  printf("==========================\n\n");

  const char mla6400x8_name[] = "MLA 6400x8 Microbenchmark (zero-init)";
  auto problem_mla6400x8 = EntoBench::make_basic_problem(mla6400x8);

  // The template parameters for the harness:
  //  - The type of our problem
  //  - `true` to collect default metrics
  //  - `1` for a single run
  using Harness6400x8 = EntoBench::Harness<decltype(problem_mla6400x8), true, 1>;
  Harness6400x8 mla6400x8_harness(problem_mla6400x8, mla6400x8_name);

  mla6400x8_harness.run();

  printf("==========================\n\n");

  // end the program
  exit(1);
  return 0;
}
