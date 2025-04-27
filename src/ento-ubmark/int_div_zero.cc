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


void __attribute__((noinline)) int_div_benchmark() {
    constexpr int reps = 600000;

    register int r0 asm("r0") = 1;
    register int r1 asm("r1") = 1;
    register int r2 asm("r2") = 1;
    register int r3 asm("r3") = 1;
    register int r4 asm("r4") = 1;
    register int r5 asm("r5") = 1;

    register int r6 asm("r6")   = 0;
    register int r7 asm("r7")   = 0;
    register int r8 asm("r8")   = 0;
    register int r9 asm("r9")   = 0;
    register int r10 asm("r10") = 0;
    register int r11 asm("r11") = 0;

    start_roi();
    for (int i = 0; i < reps; i++) {
        asm volatile (
            ".rept 8             \n"
            "  sdiv r6, r6, r0   \n"
            "  sdiv r7, r7, r1   \n"
            "  sdiv r8, r8, r2   \n"
            "  sdiv r9, r9, r3   \n"
            "  sdiv r10, r10, r4 \n"
            "  sdiv r11, r11, r5 \n"
            ".endr               \n"
            : "+r"(r6), "+r"(r7), "+r"(r8), "+r"(r9), "+r"(r10), "+r"(r11)
            : "r"(r0), "r"(r1), "r"(r2), "r"(r3), "r"(r4), "r"(r5)
        );
    }
    end_roi();
}

int main() {
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
    printf("Running integer division microbenchmark.\n");
    printf("==========================\n\n");

    uint32_t clk_freq = get_sys_clk_freq();
    printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

    uint32_t flash_latency = get_flash_latency();
    printf("Current flash latency: %i\n", flash_latency);
    printf("==========================\n\n");

    const char int_div_name[] = "Integer Division Microbenchmark";
    auto problem_int_div = EntoBench::make_basic_problem(int_div_benchmark);
    using HarnessIntDiv = EntoBench::Harness<decltype(problem_int_div), true, 1>;
    HarnessIntDiv int_div_harness(problem_int_div, int_div_name);

    int_div_harness.run();

    printf("==========================\n\n");

    exit(1);
    return 0;
}
