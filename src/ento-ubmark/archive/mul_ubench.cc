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

void __attribute__((noinline)) mul6400x8(){
  constexpr int reps = 2000;
  start_roi();
  for (int i = 0; i < reps; i++)
    asm volatile (
      "mov r8, #1         \n"
      ".rept 8       \n"   // Repeat 8 times
      "mul r0, r0, r8  \n"   
      "mul r1, r1, r8  \n"   
      "mul r2, r2, r8  \n"   
      "mul r3, r3, r8  \n"   
      "mul r4, r4, r8  \n"   
      "mul r6, r6, r8  \n"   
      "mul r5, r5, r8  \n"   
      "mul r7, r7, r8  \n"  
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


  const char mul6400x8_name[] = "Mul 6400x8 Microbenchmark";
  auto problem_mul6400x8 = EntoBench::make_basic_problem(mul6400x8);
  using Harness6400x8 = EntoBench::Harness<decltype(problem_mul6400x8), true, 1>;
  Harness6400x8 mul6400x8_harness(problem_mul6400x8, mul6400x8_name);

  mul6400x8_harness.run();

  printf("==========================\n\n");


  exit(1);

  return 0;
}
