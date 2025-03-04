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


void __attribute__((noinline)) multiply_random_numbers()
{
  //start_roi();
  constexpr int reps = 1000;
  volatile long long product = 1;

  for (int i = 0; i < reps; ++i)
  {
    int a = rand() % 100;  
    int b = rand() % 100;
    product = a * b;  
  }
  //end_roi();
  printf("Final product: %lld\n", product); 
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
  printf("Running random multiplication microbenchmark.\n");
  printf("==========================\n\n");

  uint32_t clk_freq = get_sys_clk_freq();
  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

  uint32_t flash_latency = get_flash_latency();
  printf("Current flash latency: %i\n", flash_latency);
  printf("==========================\n\n");

  auto multiply_problem = EntoBench::make_basic_problem(multiply_random_numbers);
  using MultiplyHarness = EntoBench::Harness<decltype(multiply_problem), true, 1>;
  MultiplyHarness multiply_harness(multiply_problem, "Multiply Random Numbers Microbenchmark");


  multiply_harness.run();

  printf("==========================\n\n");

  exit(1);

  return 0;
}
