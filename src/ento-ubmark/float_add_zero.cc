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

void __attribute__((noinline)) fp_add_benchmark() {
  constexpr int reps = 20000;

  float s0_ = 0.0;
  float s1_ = 0.0;
  float s2_ = 0.0;
  float s3_ = 0.0;
  float s4_ = 0.0;
  float s5_ = 0.0;
  
  start_roi();
  for (int i = 0; i < reps; i++) {
    asm volatile (
      ".rept 8                      \n"
      "  vadd.f32 s6, s0, s1        \n" 
      "  vadd.f32 s7, s2, s3        \n"
      "  vadd.f32 s8, s4, s5        \n" 
      "  vadd.f32 s6, s0, s1        \n" 
      "  vadd.f32 s7, s2, s3        \n"
      "  vadd.f32 s8, s4, s5        \n"
      ".endr                        \n"
      :
      :
      : "s0", "s1", "s2", "s3", "s4", "s5",
        "s6", "s7", "s8"  // clobbered
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
  printf("Running floating-point addition microbenchmark.\n");
  printf("==========================\n\n");

  uint32_t clk_freq = get_sys_clk_freq();
  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

  uint32_t flash_latency = get_flash_latency();
  printf("Current flash latency: %i\n", flash_latency);
  printf("==========================\n\n");

  const char fp_add_name[] = "Floating-Point Zero-Addition Microbenchmark";
  auto problem_fp_add = EntoBench::make_basic_problem(fp_add_benchmark);
  using HarnessFpAdd = EntoBench::Harness<decltype(problem_fp_add), true, 1>;
  HarnessFpAdd fp_add_harness(problem_fp_add, fp_add_name);

  fp_add_harness.run();

  printf("==========================\n\n");

  exit(1);
  return 0;
}