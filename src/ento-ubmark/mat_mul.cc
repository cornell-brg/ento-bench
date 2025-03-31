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

  uint32_t clk_freq = get_sys_clk_freq();
  uint32_t flash_latency = get_flash_latency();


  //const char mul6400x8_name[] = "Mul 6400x8 Microbenchmark";
  //auto problem_mul6400x8 = EntoBench::make_basic_problem(mul6400x8);
  //using Harness6400x8 = EntoBench::Harness<decltype(problem_mul6400x8), true, 1>;
  //Harness6400x8 mul6400x8_harness(problem_mul6400x8, mul6400x8_name);

  //mul6400x8_harness.run();


  exit(1);

  return 0;
}

