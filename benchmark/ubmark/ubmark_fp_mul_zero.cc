#include <ento-bench/harness.h>
#include <ento-util/debug.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>
#include <ento-ubmark/fp_mul_zero.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;


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
    SysTick_Setup();
    __enable_irq();

    // Turn on caches if applicable
    enable_instruction_cache();
    enable_instruction_cache_prefetch();
    icache_enable();

    printf("==========================\n");
    printf("Running floating-point multiplication microbenchmark (zero values).\n");
    printf("==========================\n\n");

    uint32_t clk_freq = get_sys_clk_freq();
    printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

    uint32_t flash_latency = get_flash_latency();
    printf("Current flash latency: %i\n", flash_latency);
    printf("==========================\n\n");

    // Updated benchmark name to reflect multiplication
    const char fp_mul_name[] = "Floating-Point Zero-Multiplication Microbenchmark";
    auto problem_fp_mul = EntoBench::make_basic_problem(fp_mul_benchmark);
    using HarnessFpMul = EntoBench::Harness<decltype(problem_fp_mul), true, 1>;
    HarnessFpMul fp_mul_harness(problem_fp_mul, fp_mul_name);

    fp_mul_harness.run();

    printf("==========================\n\n");

    exit(1);
    return 0;
}
