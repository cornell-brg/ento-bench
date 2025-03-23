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
    float s0_, s1_, s2_, s3_, s4_, s5_;

    asm volatile (
        "vmov.f32 %0, s0     \n"
        "vmov.f32 %1, s1     \n"
        "vmov.f32 %2, s2     \n"
        "vmov.f32 %3, s3     \n"
        "vmov.f32 %4, s4     \n"
        "vmov.f32 %5, s5     \n"

        : "=t"(s0_), "=t"(s1_), "=t"(s2_), "=t"(s3_), "=t"(s4_), "=t"(s5_)
        : // no inputs
        : "s0", "s1", "s2", "s3", "s4", "s5"
    );
    start_roi();
    for (int i = 0; i < reps; i++) {
        s0_ = rand_float_01();
        s1_ = rand_float_01();
        s2_ = rand_float_01();
        s3_ = rand_float_01();
        s4_ = rand_float_01();
        s5_ = rand_float_01();

        asm volatile (
            "vmov.f32 s0, %0     \n"
            "vmov.f32 s1, %1     \n"
            "vmov.f32 s2, %2     \n"
            "vmov.f32 s3, %3     \n"
            "vmov.f32 s4, %4     \n"
            "vmov.f32 s5, %5     \n"
            :
            : "t"(s0_), "t"(s1_), "t"(s2_), "t"(s3_), "t"(s4_), "t"(s5_)
            : "s0", "s1", "s2", "s3", "s4", "s5"
        );

        asm volatile (
            ".rept 8              \n"
            "  vadd.f32 s6, s0, s1 \n"
            "  vadd.f32 s7, s2, s3 \n"
            "  vadd.f32 s8, s4, s5 \n"
            "  vadd.f32 s6, s0, s1 \n"
            "  vadd.f32 s7, s2, s3 \n"
            "  vadd.f32 s8, s4, s5 \n"
            ".endr                 \n"
            :
            :
            : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8"
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

    // Seed the run
    srand(5555);

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
