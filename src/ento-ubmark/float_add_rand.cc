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

static inline float rand_float_01() {
    return ((float)rand() / (float)RAND_MAX) * 160.0f;
}


void __attribute__((noinline)) fp_add_benchmark() {
    constexpr int reps = 20000;

    register float r0 asm("s0") = rand_float_01();
    register float r1 asm("s1") = rand_float_01();
    register float r2 asm("s2") = rand_float_01();
    register float r3 asm("s3") = rand_float_01();
    register float r4 asm("s4") = rand_float_01();
    register float r5 asm("s5") = rand_float_01();

    register float r6 asm("s6") = 0.0f;
    register float r7 asm("s7") = 0.0f;
    register float r8 asm("s8") = 0.0f;
    register float r9 asm("s9") = 0.0f;
    register float r10 asm("s10") = 0.0f;
    register float r11 asm("s11") = 0.0f;

    start_roi();
    for (int i = 0; i < reps; i++) {
        asm volatile (
            ".rept 8                 \n"
            "  vadd.f32 s6, s0, s6   \n"
            "  vadd.f32 s7, s1, s7   \n"
            "  vadd.f32 s8, s2, s8   \n"
            "  vadd.f32 s9, s3, s9   \n"
            "  vadd.f32 s10, s4, s10 \n"
            "  vadd.f32 s11, s5, s11 \n"
            ".endr                   \n"
            : "+t"(r6), "+t"(r7), "+t"(r8), "+t"(r9), "+t"(r10), "+t"(r11) // output
            : "t"(r0), "t"(r1), "t"(r2), "t"(r3), "t"(r4), "t"(r5),        // input
              "t"(r6), "t"(r7), "t"(r8), "t"(r9), "t"(r10), "t"(r11)       // input
            : 
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
    printf("Running floating-point addition microbenchmark (random values).\n");
    printf("==========================\n\n");

    uint32_t clk_freq = get_sys_clk_freq();
    printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

    uint32_t flash_latency = get_flash_latency();
    printf("Current flash latency: %i\n", flash_latency);
    printf("==========================\n\n");

    const char fp_add_name[] = "Floating-Point Random-Addition Microbenchmark";
    auto problem_fp_add = EntoBench::make_basic_problem(fp_add_benchmark);
    using HarnessFpAdd = EntoBench::Harness<decltype(problem_fp_add), true, 1>;
    HarnessFpAdd fp_add_harness(problem_fp_add, fp_add_name);

    fp_add_harness.run();

    printf("==========================\n\n");

    exit(1);
    return 0;
}
