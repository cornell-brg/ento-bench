#include <ento-bench/harness.h>
#include <ento-util/debug.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>
#include <ento-ubmark/int_add_zero.h>

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
    printf("Running integer addition microbenchmark.\n");
    printf("==========================\n\n");

    uint32_t clk_freq = get_sys_clk_freq();
    printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

    uint32_t flash_latency = get_flash_latency();
    printf("Current flash latency: %i\n", flash_latency);
    printf("==========================\n\n");
    software_delay_cycles(10000);

    const char int_add_name[] = "Integer Addition Microbenchmark";
    auto problem_int_add = EntoBench::make_basic_problem(int_add_benchmark);
    using HarnessIntAdd = EntoBench::Harness<decltype(problem_int_add), true, 1>;
    HarnessIntAdd int_add_harness(problem_int_add, int_add_name);

    int_add_harness.run();

    printf("==========================\n\n");

    exit(1);
    return 0;
}
