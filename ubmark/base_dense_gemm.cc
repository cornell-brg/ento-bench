#include <stdlib.h>
#include <stdio.h>
#include <ento-bench/harness.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>

extern "C" void initialise_monitor_handles(void);

// Dense GEMM
template<int N>
void __attribute__((noinline)) dense_gemm(const int A[N][N], const int B[N][N], int C[N][N]) {
    if constexpr (N > 0) {
        start_roi();
        // Perform matrix multiplication C = A * B
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                C[i][j] = 0;
                for (int k = 0; k < N; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        end_roi();
    }
}

// Templated function to initialize matrices
template<int size>
void matrix_init() {
    int A[size][size];
    int B[size][size];
    int C[size][size];

    // Initialize matrices A and B to ones
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            A[i][j] = 1;
            B[i][j] = 1;
        }
    }

    constexpr int reps = 5;
    printf("Matrix size N: %d\n", size);
    auto dense_harness = bench::make_harness<reps>([&]() { dense_gemm<size>(A, B, C); },
                                            "GEMM Dense Benchmark Example");
    dense_harness.run();
}

// Base case to end recursion
template<int size>
void run_matrix_init() {
    if constexpr (size <= 100) {
        matrix_init<size>();
        run_matrix_init<size + 1>(); 
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

    printf("==========================");
    printf("Running example microbenchmarks.\n");
    printf("==========================\n\n");
    uint32_t clk_freq = get_sys_clk_freq();
    printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);
    uint32_t flash_latency = get_flash_latency();
    printf("Current flash latency: %li\n", flash_latency);
    printf("==========================\n\n");
    printf("Running examples from default startup parameters (see above).");

    run_matrix_init<2>(); 
    // matrix_init<58>();

    printf("Finished running custom GEMM dense benchmark example!\n");

    exit(1);

    return 0;
}
