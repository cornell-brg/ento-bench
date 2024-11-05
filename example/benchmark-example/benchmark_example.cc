#include <stdlib.h>
#include <stdio.h>
#include "bench/harness.h"
#include "mcu-util/flash_util.h"
#include "mcu-util/clk_util.h"
#include "mcu-util/pwr_util.h"
#include <Eigen/Dense>

extern "C" void initialise_monitor_handles(void);

void __attribute__((noinline)) hello_host_computer()
{
  printf("Hello host computer!\n");
}

void __attribute__((noinline)) add16x8()
{
  asm volatile (
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
}

void __attribute__((noinline)) add4x8()
{
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
}

int main()
{
  using namespace bench;
  initialise_monitor_handles();


  constexpr int reps = 10;

  printf("==========================");
  printf("Running example microbenchmarks.\n");
  printf("==========================\n\n");

  uint32_t clk_freq = get_sys_clk_freq();

  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

  uint32_t vcore_range = get_vcore_range();

  printf("Vcore Range: %li\n", vcore_range);

  uint32_t flash_latency = get_flash_latency();

  printf("Current flash latency: %li\n", flash_latency);
  printf("==========================\n\n");
  printf("Running examples from default startup parameters (see above).");

  auto add16x8_harness = make_harness<reps>(add16x8,
                                            "Add16x8 Benchmark Example");
  auto add4x8_harness = make_harness<reps>(add4x8,
                                           "Add4x8 Benchmark Example");
  add16x8_harness.run();
  printf("Finished running add16x8 benchmark example with default parameters.\n\n");
  add4x8_harness.run();
  printf("Finished running add4x8 benchmark example with default parameters.\n\n");

  printf("==========================\n\n");
  printf("Maxing out clock.");
  printf("Enabling cache and prefetch!\n");

  clk_freq = get_sys_clk_freq();

  printf("Current clk frequency (MHz): %.2f\n", float(clk_freq) / 1000000.0);

  sys_clk_cfg();
  clk_freq = get_sys_clk_freq();

  printf("Current clk frequency (MHz): %.2f\n", float(clk_freq) / 1000000.0);
  uint32_t is_prefetch_en = is_instruction_cache_prefetch_enabled();
  printf("Is prefetch enabled: %li\n", is_prefetch_en);

  enable_instruction_cache();

  enable_instruction_cache_prefetch();

  is_prefetch_en = is_instruction_cache_prefetch_enabled();
  printf("Is prefetch enabled: %li\n\n", is_prefetch_en);

  // Add Benchmark
  add16x8_harness.run();
  printf("Finished running add16x8 benchmark example with cache and prefetch enabled.\n\n");
  add4x8_harness.run();
  printf("Finished running add4x8 benchmark example with cache and prefetch enabled.\n\n");

  printf("==========================\n\n");

  disable_instruction_cache();

  disable_instruction_cache_prefetch();

  add16x8_harness.run();
  printf("Finished running add16x8 benchmark example with cache and prefetch disabled.\n\n");
  add4x8_harness.run();
  printf("Finished running add4x8 benchmark example with cache and prefetch disabled.\n\n");
  printf("==========================\n\n");

  enable_instruction_cache();

  add16x8_harness.run();
  printf("Finished running add16x8 benchmark example with cache enabled and prefetch disabled.\n\n");
  add4x8_harness.run();
  printf("Finished running add4x8 benchmark example with cache enabled and prefetch disabled.\n\n");
  printf("==========================\n\n");


  exit(1);

  return 0;
}
