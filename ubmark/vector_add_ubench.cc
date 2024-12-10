#include <stdlib.h>
#include <stdio.h>
#include <ento-bench/harness.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>
#include <Eigen/Dense>

extern "C" void initialise_monitor_handles(void);

template<int N>
void __attribute__((noinline)) vvadd(const Eigen::Matrix<int, N, 1>& x, const Eigen::Matrix<int, N, 1>& y, Eigen::Matrix<int, N, 1>& z) {
    if constexpr (N > 0) {
        start_roi();
        asm volatile( "" ::: "memory" );
        z.noalias() = x + y;
        asm volatile( "" ::: "memory" );
        end_roi();
    }
}

template<int size>
void vector_init() {
    static Eigen::Matrix<int, size, 1> x = Eigen::Matrix<int, size, 1>::Ones();
    static Eigen::Matrix<int, size, 1> y = Eigen::Matrix<int, size, 1>::Ones();
    static Eigen::Matrix<int, size, 1> z;

    constexpr int reps = 5;
    printf("Vector size N: %d\n", size);
    auto vvadd_harness = bench::make_harness<reps>([&]() { vvadd<size>(x, y, z); },
                                            "Vector-Vector Add Benchmark Example");
    vvadd_harness.run();
}

template<int size>
void sweep_vector_sizes() {
    if constexpr (size <= 100) {
        vector_init<size>();
        sweep_vector_sizes<size + 1>();
    }
}

int main() {
    using namespace bench;
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
    printf("Running example microbenchmarks.\n");
    printf("==========================\n\n");
    uint32_t clk_freq = get_sys_clk_freq();
    printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);
    uint32_t flash_latency = get_flash_latency();
    printf("Current flash latency: %li\n", flash_latency);
    printf("==========================\n\n");
    printf("Running examples from default startup parameters (see above).\n");

    sweep_vector_sizes<2>(); // Start sweeping vector sizes from 2x1 to 100x1

    printf("Finished running Eigen vvadd benchmark example!\n");

    exit(1);

    return 0;
}
