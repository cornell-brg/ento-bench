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

void __attribute__((noinline)) xor6400x8(){
  constexpr int reps = 6500;
  asm volatile(
        "mov r8, #0x00000000    \n"
        "ldr r9, =0xFFFFFFFF    \n"

        "mov r0, #0x00000000    \n"
        "mov r1, #0x00000000    \n"
        "mov r2, #0x00000000    \n"
        "mov r3, #0x00000000    \n"
        "mov r4, #0x00000000    \n"
        "mov r5, #0x00000000    \n"
        "mov r6, #0x00000000    \n"
        "mov r7, #0x00000000    \n"
        :
        :
        : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9" // Clobbered so compiler knows regs were changed
  );
  
  start_roi();
  for (int i = 0; i < reps; i++) {
    asm volatile (
      ".rept 8         \n" 
      "eor r0, r0, r8  \n"  
      "eor r1, r1, r8  \n"
      "eor r2, r2, r8  \n"
      "eor r3, r3, r8  \n"
      "eor r4, r4, r8  \n"
      "eor r5, r5, r8  \n"
      "eor r6, r6, r8  \n"
      "eor r7, r7, r8  \n"

      "eor r0, r0, r9  \n"  
      "eor r1, r1, r9  \n"
      "eor r2, r2, r9  \n"
      "eor r3, r3, r9  \n"
      "eor r4, r4, r9  \n"
      "eor r5, r5, r9  \n"
      "eor r6, r6, r9  \n"
      "eor r7, r7, r9  \n"
      ".endr           \n"  
      : // No outputs
      : // No inputs
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"  // Clobbered registers
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

  const char xor6400x8_name[] = "XOR 6400x8 Microbenchmark";
  auto problem_xor6400x8 = EntoBench::make_basic_problem(xor6400x8);
  using Harness6400x8 = EntoBench::Harness<decltype(problem_xor6400x8), true, 1>;
  Harness6400x8 xor6400x8_harness(problem_xor6400x8, xor6400x8_name);

  xor6400x8_harness.run();

  printf("==========================\n\n");

  exit(1);

  return 0;
}
