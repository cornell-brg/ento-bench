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

void __attribute__((noinline)) fp_add_benchmark() {
  constexpr int reps = 50000;
  float zero = 0.0f;
  float const1 = 1.5f;
  float const2 = 2.25f;
  float const3 = 3.75f;
  float const4 = 4.5f;
  float const5 = 5.25f;
  float const6 = 6.75f;
  
  asm volatile (
    "vmov.f32 s0, %[zero]     \n" 
    "vmov.f32 s1, %[zero]     \n"   
    "vmov.f32 s2, %[zero]     \n"    
    "vmov.f32 s3, %[zero]     \n"   
    "vmov.f32 s4, %[zero]     \n" 
    "vmov.f32 s5, %[zero]     \n" 
    :
    : [zero] "t" (zero)
    : "s0", "s1", "s2", "s3", "s4", "s5"
  );
  
  start_roi();
  asm volatile (
    "vmov.f32 s6, %[const1]      \n" 
    "vadd.f32 s0, s0, s6         \n" 
    "vmov.f32 s6, %[const2]      \n"  
    "vadd.f32 s1, s1, s6         \n"  
    "vmov.f32 s6, %[const3]      \n" 
    "vadd.f32 s2, s2, s6         \n" 
    "vmov.f32 s6, %[const4]      \n" 
    "vadd.f32 s3, s3, s6         \n" 
    "vmov.f32 s6, %[const5]      \n" 
    "vadd.f32 s4, s4, s6         \n"  
    "vmov.f32 s6, %[const6]      \n"  
    "vadd.f32 s5, s5, s6         \n" 
    :
    : [const1] "t" (const1), [const2] "t" (const2), [const3] "t" (const3),
      [const4] "t" (const4), [const5] "t" (const5), [const6] "t" (const6)
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6"
  );

  for (int i = 0; i < reps; i++) {
    float val = (i % 32 + 1) * 0.125f; 
    asm volatile (
        "vmov.f32 s6, %[val]            \n" 
        "vadd.f32 s0, s0, s6            \n"  
        "vadd.f32 s1, s1, s6            \n"  
        "vadd.f32 s2, s2, s6            \n"  
        "vadd.f32 s3, s3, s6            \n"  
        "vadd.f32 s4, s4, s6            \n"  
        "vadd.f32 s5, s5, s6            \n"  
        : 
        : [val] "t" (val)
        : "s0", "s1", "s2", "s3", "s4", "s5", "s6"
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
  printf("Running floating-point addition microbenchmark.\n");
  printf("==========================\n\n");

  uint32_t clk_freq = get_sys_clk_freq();

  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

  uint32_t flash_latency = get_flash_latency();

  printf("Current flash latency: %i\n", flash_latency);
  printf("==========================\n\n");

  const char fp_add_name[] = "Floating-Point Addition Microbenchmark";
  auto problem_fp_add = EntoBench::make_basic_problem(fp_add_benchmark);
  using HarnessFpAdd = EntoBench::Harness<decltype(problem_fp_add), true, 1>;
  HarnessFpAdd fp_add_harness(problem_fp_add, fp_add_name);

  fp_add_harness.run();

  printf("==========================\n\n");

  exit(1);

  return 0;
}