#include <stdlib.h>
#include <stdio.h>
#include "bench/harness.h"
#include "mcu-util/cache_util.h"
#include "mcu-util/flash_util.h"
#include "mcu-util/clk_util.h"
#include "mcu-util/pwr_util.h"

extern "C" void initialise_monitor_handles(void);

template <int N>

inline void __attribute__((always_inline)) add_template_recursion_always_inline() {
  //for (int i = 0; i < N; i++)
  if constexpr (N > 0)
  {
    asm volatile (
      "add r0, r0, r0 \n\t"
      "add r1, r1, r1 \n\t"
      "add r2, r2, r2 \n\t"
      "add r3, r3, r3 \n\t"
      "add r4, r4, r4 \n\t"
      "add r5, r5, r5 \n\t"
      "add r6, r6, r6 \n\t"
      "add r8, r8, r8 \n\t"
      : // No outputs
      : // No inputs
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r8" // Clobber list
    );
    add_template_recursion_always_inline<N-1>();
  }
}

template<int N>
void __attribute__((noinline)) add_template_recursion() {
  if constexpr (N > 0)
  {
    add_template_recursion_always_inline<N>();
  }
}



int main()
{
  using namespace bench;
  initialise_monitor_handles();

  bool is_systick_enabled = (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0;

  printf("Is systick enabled: %i\n", is_systick_enabled);

  // Configure max clock rate and set flash latency
  sys_clk_cfg();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  printf("==========================");
  printf("Running example microbenchmarks.\n");
  printf("==========================\n\n");

  uint32_t clk_freq = get_sys_clk_freq();

  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

  uint32_t flash_latency = get_flash_latency();

  printf("Current flash latency: %li\n", flash_latency);
  printf("==========================\n\n");
  printf("Running examples from default startup parameters (see above).");


  constexpr int reps = 100; 

  auto add_harness = bench::make_harness<reps>([&]() { add_template_recursion<reps>; },
                                          "Simple Add Benchmark");
  add_harness.run();

  printf("Finished running independent add benchmark example!\n");

  exit(1);

  return 0;
}




