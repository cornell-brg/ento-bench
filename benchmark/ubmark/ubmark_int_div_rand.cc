#include <ento-bench/harness.h>
#include <ento-util/debug.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>
#include <ento-ubmark/int_div_rand.h>

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
