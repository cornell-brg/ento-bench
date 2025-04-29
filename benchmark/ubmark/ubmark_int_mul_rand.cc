#include <ento-bench/harness.h>
#include <ento-util/debug.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>
#include <ento-ubmark/int_mul_rand.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;


int main() {
    using namespace EntoBench;
#if defined(SEMIHOSTING)
    initialise_monitor_handles();
#endif

    bool is_systick_enabled = (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0;
    printf("Is systick enabled: %i\n", is_systick_enabled);

    sys_clk_cfg();
    SysTick_Setup();
    __enable_irq();
    
    enable_instruction_cache();
    enable_instruction_cache_prefetch();
    icache_enable();

    printf("==========================\n");
    printf("Running integer multiplication microbenchmark.\n");
    printf("==========================\n\n");

    uint32_t clk_freq = get_sys_clk_freq();
    printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

    uint32_t flash_latency = get_flash_latency();
    printf("Current flash latency: %i\n", flash_latency);
    printf("==========================\n\n");

    const char int_mul_name[] = "Integer Multiplication Microbenchmark";
    auto problem_int_mul = EntoBench::make_basic_problem(int_mul_benchmark);
    using HarnessIntMul = EntoBench::Harness<decltype(problem_int_mul), true, 1>;
    HarnessIntMul int_mul_harness(problem_int_mul, int_mul_name);

    int_mul_harness.run();

    printf("==========================\n\n");

    exit(1);
    return 0;
}
