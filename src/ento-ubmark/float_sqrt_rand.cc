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

void __attribute__((noinline)) fp_sqrt_benchmark() {
    constexpr int reps = 600000;

    register float r0 asm("s0") = rand_float_01();
    register float r1 asm("s1") = rand_float_01();
    register float r2 asm("s2") = rand_float_01();
    register float r3 asm("s3") = rand_float_01();
    register float r4 asm("s4") = rand_float_01();
    register float r5 asm("s5") = rand_float_01();

    register float r6 asm("s6")   = 0.0f;
    register float r7 asm("s7")   = 0.0f;
    register float r8 asm("s8")   = 0.0f;
    register float r9 asm("s9")   = 0.0f;
    register float r10 asm("s10") = 0.0f;
    register float r11 asm("s11") = 0.0f;

    start_roi();
    for (int i = 0; i < reps; i++) {
        asm volatile (
            ".rept 8                  \n"
            "  vsqrt.f32 s6, s0       \n"
            "  vsqrt.f32 s7, s1       \n"
            "  vsqrt.f32 s8, s2       \n"
            "  vsqrt.f32 s9, s3       \n"
            "  vsqrt.f32 s10, s4      \n"
            "  vsqrt.f32 s11, s5      \n"
            ".endr                    \n"
            : "+t"(r6), "+t"(r7), "+t"(r8), "+t"(r9), "+t"(r10), "+t"(r11)
            : "t"(r0), "t"(r1), "t"(r2), "t"(r3), "t"(r4), "t"(r5)
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

    sys_clk_cfg();

    enable_instruction_cache();
    enable_instruction_cache_prefetch();
    icache_enable();

    printf("==========================\n");
    printf("Running floating-point square root microbenchmark (random values).\n");
    printf("==========================\n\n");

    uint32_t clk_freq = get_sys_clk_freq();
    printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

    uint32_t flash_latency = get_flash_latency();
    printf("Current flash latency: %i\n", flash_latency);
    printf("==========================\n\n");

    const char fp_sqrt_name[] = "Floating-Point Random-SquareRoot Microbenchmark";
    auto problem_fp_sqrt = EntoBench::make_basic_problem(fp_sqrt_benchmark);
    using HarnessFpSqrt = EntoBench::Harness<decltype(problem_fp_sqrt), true, 1>;
    HarnessFpSqrt fp_sqrt_harness(problem_fp_sqrt, fp_sqrt_name);

    fp_sqrt_harness.run();

    printf("==========================\n\n");

    exit(1);
    return 0;
}
