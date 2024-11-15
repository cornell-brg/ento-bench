#include <stdlib.h>
#include <stdio.h>
#include "bench/harness.h"
#include "mcu-util/cache_util.h"
#include "mcu-util/flash_util.h"
#include "mcu-util/clk_util.h"
#include <bench/roi.h>
#include "mcu-util/pwr_util.h"
#include <Eigen/Dense>

extern "C" void initialise_monitor_handles(void);

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
    "add r3, r3, #1  \n"   // Add 1 to r7
    ".endr            \n"   // End repeat block
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6"   // Clobbered registers
  );
  end_roi();
}

void __attribute__((noinline)) add4x8()
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
    "add r3, r3, #1  \n"   // Add 1 to r7
    ".endr            \n"   // End repeat block
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6"  // Clobbered registers
  );
  end_roi();
}

void __attribute__((noinline)) add4096x8()
{
  start_roi();
  constexpr int reps = 128;
  for (int i = 0; i < reps; i++)
    asm volatile (
      ".rept 64       \n"   // Repeat 8 times
      "add r0, r0, #1  \n"   // Add 1 to r0
      "add r1, r1, #1  \n"   // Add 1 to r1
      "add r2, r2, #1  \n"   // Add 1 to r2
      "add r3, r3, #1  \n"   // Add 1 to r3
      "add r4, r4, #1  \n"   // Add 1 to r4
      "add r5, r5, #1  \n"   // Add 1 to r5
      "add r6, r6, #1  \n"   // Add 1 to r6
      "add r3, r3, #1  \n"   // Add 1 to r7
      ".endr           \n"   // End repeat block
      : // No outputs
      : // No inputs
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6"  // Clobbered registers
    );
  end_roi();

}

int main()
{
  using namespace bench;
  initialise_monitor_handles();

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
  printf("ICache is enabled!\n\n");
  printf("==========================\n\n");

  auto add64x8_harness = make_harness<reps>(add64x8,
                                            "Add64x8 Benchmark Example");
  auto add4x8_harness = make_harness<reps>(add4x8,
                                           "Add4x8 Benchmark Example");
  auto add4096x8_harness = make_harness<reps>(add4096x8,
                                             "Add4096x8 Benchmark Example");
  add64x8_harness.run();
  printf("Finished running add64x8 benchmark example.\n\n");
  add4x8_harness.run();
  printf("Finished running add4x8 benchmark example.\n\n");
  add4096x8_harness.run();
  printf("Finished running add4096x8 benchmark example.\n\n");

  printf("==========================\n\n");


  exit(1);

  return 0;
}
