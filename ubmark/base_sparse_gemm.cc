#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <set>
#include <random>
#include <string>
#include <ento-bench/harness.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>

extern "C" void initialise_monitor_handles(void);

// Sparse GEMM using raw arrays
template<int N>
void __attribute__((noinline)) sparse_gemm(const int A[N][N], const int B[N][N], int C[N][N]) {
    // Initialize result matrix C to zero
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            C[i][j] = 0;
        }
    }

    // Perform sparse matrix multiplication
    start_roi();
    for (int i = 0; i < N; ++i) {
        for (int k = 0; k < N; ++k) {
            if (A[i][k] != 0) { // Skip zero elements in A
                for (int j = 0; j < N; ++j) {
                    if (B[k][j] != 0) { // Skip zero elements in B
                        C[i][j] += A[i][k] * B[k][j];
                    }
                }
            }
        }
    }
    end_roi();
}

// Function to initialize sparse matrices with a given sparsity
template<int N>
void initialize_sparse_matrix(int matrix[N][N], float sparsity) {
    int elements = N * N;
    int non_zero_elements = static_cast<int>(elements * (1.0f - sparsity));

    std::set<std::pair<int, int>> occupied_positions;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, N - 1);

    // Initialize all elements to zero
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            matrix[i][j] = 0;
        }
    }

    for (int i = 0; i < non_zero_elements; ++i) {
        int row, col;
        std::pair<int, int> position;

        // Generate unique random positions for non-zero entries
        do {
            row = dist(gen);
            col = dist(gen);
            position = {row, col};
        } while (occupied_positions.find(position) != occupied_positions.end());

        occupied_positions.insert(position);
        matrix[row][col] = 1; // Set non-zero element
    }
}

// Benchmark function with different sparsity levels
template<int N>
void sparse_benchmark() {
    int A[N][N];
    int B[N][N];
    int C[N][N];

    constexpr int reps = 5;
    float sparsity_levels[] = {0.0f, 0.1f, 0.3f, 0.5f, 0.7f, 0.9f, 1.0f}; 

    for (float sparsity : sparsity_levels) {
        initialize_sparse_matrix<N>(A, sparsity);
        initialize_sparse_matrix<N>(B, sparsity);

        printf("Matrix size N: %d, Sparsity: %.1f\n", N, sparsity);
        std::string benchmark_name = "GEMM Sparse Benchmark with Sparsity " + std::to_string(sparsity);
        auto sparse_harness = bench::make_harness<reps>([&]() { sparse_gemm<N>(A, B, C); }, benchmark_name.c_str());
        sparse_harness.run();
    }
}

template<int size>
void run_matrix_init() {
    if constexpr (size <= 20) { 
        sparse_benchmark<size>();
        run_matrix_init<size + 1>();
    }
}

int main() {
    using namespace bench;
    initialise_monitor_handles();

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

    constexpr int starting_size = 2; // Start sweeping matrix at this size
    run_matrix_init<starting_size>();

    printf("Finished running custom GEMM sparse benchmark with varying sparsity levels!\n");

    exit(1);

    return 0;
}
