#include <ento-bench/harness.h>
#include <ento-util/debug.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>
#include <ento-ubmark/fp_sqrt_zero.h>

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
    printf("Running floating-point square root microbenchmark.\n");
    printf("==========================\n\n");

    uint32_t clk_freq = get_sys_clk_freq();
    printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

    uint32_t flash_latency = get_flash_latency();
    printf("Current flash latency: %i\n", flash_latency);
    printf("==========================\n\n");

    const char fp_sqrt_name[] = "Floating-Point Square Root Microbenchmark";
    auto problem_fp_sqrt = EntoBench::make_basic_problem(fp_sqrt_benchmark);
    using HarnessFpSqrt = EntoBench::Harness<decltype(problem_fp_sqrt), true, 1>;
    HarnessFpSqrt fp_sqrt_harness(problem_fp_sqrt, fp_sqrt_name);

    fp_sqrt_harness.run();

    printf("==========================\n\n");

    exit(1);
    return 0;
}
