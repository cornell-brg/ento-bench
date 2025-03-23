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

void __attribute__((noinline)) nop16000x8() {
  constexpr int reps = 15000;
  start_roi();
  for (int i = 0; i < reps; i++)
    asm volatile (
      ".rept 8       \n"   
      "nop           \n"   
      "nop           \n"   
      "nop           \n"   
      "nop           \n"   
      "nop           \n"   
      "nop           \n"   
      "nop           \n"   
      "nop           \n"   
      ".endr           \n" 
      : // No outputs
      : // No inputs
      : // No clobbered registers
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


  const char nop16000x8_name[] = "NOP 16000x8 Microbenchmark";
  auto problem_nop16000x8 = EntoBench::make_basic_problem(nop16000x8);
  using HarnessNOP16000x8 = EntoBench::Harness<decltype(problem_nop16000x8), true, 1>;
  HarnessNOP16000x8 nop16000x8_harness(problem_nop16000x8, nop16000x8_name);

  nop16000x8_harness.run();

  printf("==========================\n\n");


  exit(1);

  return 0;
}