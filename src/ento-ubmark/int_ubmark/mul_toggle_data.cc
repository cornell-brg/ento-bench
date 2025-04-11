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

void __attribute__((noinline)) toggle_all_bits_mul() {
  constexpr int reps = 128000;
  asm volatile (
    "mov r0, #1         \n" 
    "mov r1, #1         \n"   
    "mov r2, #1         \n"    
    "mov r3, #1         \n"   
    "mov r4, #1         \n" 
    "mov r5, #1         \n" 
    :
    :
    : "r0", "r1", "r2", "r3", "r4", "r5"
  );
  
  start_roi();
  asm volatile (
    "ldr r6, =#0x55555555    \n" 
    "mul r0, r0, r6          \n" 
    "ldr r6, =#0xAAAAAAAA    \n" 
    "mul r1, r1, r6          \n" 
    "ldr r6, =#0x33333333    \n" 
    "mul r2, r2, r6          \n" 
    "ldr r6, =#0xCCCCCCCC    \n" 
    "mul r3, r3, r6          \n"  
    "ldr r6, =#0x0F0F0F0F    \n" 
    "mul r4, r4, r6          \n"  
    "ldr r6, =#0xF0F0F0F0    \n"  
    "mul r5, r5, r6          \n"  
    :
    :
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6"
  );

  for (int i = 0; i < reps; i++) {
    int val = ((i % 15) * 2) + 3; 
    asm volatile (
        "mov r6, %[val]             \n" 
        "mul r0, r0, r6             \n" 
        "lsl r6, %[val], #2         \n" 
        "mul r1, r1, r6             \n"
        "lsl r6, %[val], #3         \n" 
        "mul r2, r2, r6             \n"
        "lsl r6, %[val], #5         \n" 
        "mul r3, r3, r6             \n" 
        "lsl r6, %[val], #7         \n" 
        "mul r4, r4, r6             \n" 
        "lsl r6, %[val], #11        \n" 
        "mul r5, r5, r6             \n" 
        : 
        : [val] "r" (val)
        : "r0", "r1", "r2", "r3", "r4", "r5", "r6"
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

  const char toggle_bits_name[] = "Toggle All Bits Multiply Microbenchmark";
  auto problem_toggle_bits = EntoBench::make_basic_problem(toggle_all_bits_mul);
  using HarnessToggleBits = EntoBench::Harness<decltype(problem_toggle_bits), true, 1>;
  HarnessToggleBits toggle_bits_harness(problem_toggle_bits, toggle_bits_name);

  toggle_bits_harness.run();

  printf("==========================\n\n");

  exit(1);

  return 0;
}