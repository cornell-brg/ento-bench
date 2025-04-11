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

void __attribute__((noinline)) toggle_all_bits() {
  constexpr int reps = 128000;
  // Initialize all registers to 0
  asm volatile (
    "mov r0, #0         \n" 
    "mov r1, #0         \n"   
    "mov r2, #0         \n"    
    "mov r3, #0         \n"   
    "mov r4, #0         \n" 
    "mov r5, #0         \n" 
    :
    :
    : "r0", "r1", "r2", "r3", "r4", "r5"
  );
  
  start_roi();
  asm volatile (
    "add r0, r0, #0x55555555 \n"  
    "add r1, r1, #0xAAAAAAAA \n"  
    "add r2, r2, #0x33333333 \n"  
    "add r3, r3, #0xCCCCCCCC \n"  
    "add r4, r4, #0x0F0F0F0F \n" 
    "add r5, r5, #0xF0F0F0F0 \n"  
    :
    :
    : "r0", "r1", "r2", "r3", "r4", "r5"
  );

  for (int i = 0; i < reps; i++) {
    int val = (i % 32) + 1; 
    asm volatile (
        "add r0, r0, %[val] \n"       
        "add r1, r1, %[val], lsl #4 \n" 
        "add r2, r2, %[val], lsl #8 \n"  
        "add r3, r3, %[val], lsl #12 \n"
        "add r4, r4, %[val], lsl #16 \n"
        "add r5, r5, %[val], lsl #20 \n" 

        : 
        : [val] "r" (val)
        : "r0", "r1", "r2", "r3", "r4", "r5"
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

  const char toggle_bits_name[] = "Toggle All Bits Microbenchmark";
  auto problem_toggle_bits = EntoBench::make_basic_problem(toggle_all_bits);
  using HarnessToggleBits = EntoBench::Harness<decltype(problem_toggle_bits), true, 1>;
  HarnessToggleBits toggle_bits_harness(problem_toggle_bits, toggle_bits_name);

  toggle_bits_harness.run();

  printf("==========================\n\n");

  exit(1);

  return 0;
}