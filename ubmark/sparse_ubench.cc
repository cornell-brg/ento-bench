#include <stdlib.h>
#include <stdio.h>
#include <ento-bench/harness.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>
#include <set> 

#include <Eigen/Sparse>

extern "C" void initialise_monitor_handles(void);

// Sparse GEMM
void __attribute__((noinline)) sparse_gemm(const Eigen::SparseMatrix<int>& A, const Eigen::SparseMatrix<int>& B, Eigen::SparseMatrix<int>& C) {
    start_roi();
    C = A * B; // Sparse matrix multiplication
    end_roi();
}

// Function to initialize sparse matrices with a given sparsity
void initialize_sparse_matrix(Eigen::SparseMatrix<int>& matrix, float sparsity) {
    int size = matrix.rows();
    int elements = size * size;
    int non_zero_elements = static_cast<int>(elements * (1.0f - sparsity)); // Number of non-zero elements

    std::set<std::pair<int, int>> occupied_positions; // To track used positions

    for (int i = 0; i < non_zero_elements; ++i) {
        int row, col;
        std::pair<int, int> position;

        // Generate unique random positions
        do {
            row = rand() % size;
            col = rand() % size;
            position = {row, col};
        } while (occupied_positions.find(position) != occupied_positions.end());

        // Insert the position into the set and matrix
        occupied_positions.insert(position);
        matrix.insert(row, col) = 1;
    }
    matrix.makeCompressed(); 
}

// Benchmark function with different sparsity levels
template<int size>
void sparse_benchmark() {
    Eigen::SparseMatrix<int> A(size, size);
    Eigen::SparseMatrix<int> B(size, size);
    Eigen::SparseMatrix<int> C(size, size);

    constexpr int reps = 5;
    float sparsity_levels[] = {0.0f, 0.1f, 0.3f, 0.5f, 0.7f, 0.9f, 1.0f}; // Different sparsity levels to test

    for (float sparsity : sparsity_levels) {
        initialize_sparse_matrix(A, sparsity);
        initialize_sparse_matrix(B, sparsity);

        printf("Matrix size N: %d\n", size);
        std::string benchmark_name = "GEMM Sparse Benchmark with Sparsity " + std::to_string(sparsity);
        auto sparse_harness = bench::make_harness<reps>([&]() { sparse_gemm(A, B, C); }, benchmark_name.c_str());
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
    // sparse_benchmark<starting_size>();

    printf("Finished running Eigen GEMM sparse benchmark with varying sparsity levels!\n");

    exit(1);

    return 0;
}





// #include "/Users/amyle/entomoton-bench/external/eigen/Eigen/Sparse"
// #include <stdlib.h>
// #include <stdio.h>
// #include <unordered_set>
// #include <random> // Required for random_device and mt19937

// #include "bench/harness.h"
// #include "mcu-util/flash_util.h"
// #include "mcu-util/clk_util.h"
// #include "mcu-util/pwr_util.h"

// // Sparse GEMM
// void __attribute__((noinline)) sparse_gemm(const Eigen::SparseMatrix<int>& A, const Eigen::SparseMatrix<int>& B, Eigen::SparseMatrix<int>& C) {
//     C = A * B; // Sparse matrix multiplication
// }

// // Function to initialize sparse matrices with a given sparsity
// void initialize_sparse_matrix(Eigen::SparseMatrix<int>& matrix, float sparsity) {
//     int size = matrix.rows();
//     int elements = size * size;
//     int non_zero_elements = static_cast<int>(elements * (1.0f - sparsity));

//     matrix.reserve(non_zero_elements); // Reserve space to reduce reallocations

//     // Set up a random number generator
//     std::random_device rd;  // Hardware random number generator
//     std::mt19937 gen(rd()); // Mersenne Twister for randomness
//     std::uniform_int_distribution<> dist(0, size - 1);

//     std::unordered_set<int> used_positions; // Track used positions for efficiency

//     int inserted_elements = 0;
//     while (inserted_elements < non_zero_elements) {
//         int row = dist(gen);
//         int col = dist(gen);
//         int pos = row * size + col; // Unique identifier for (row, col)

//         // Insert only if the position is unused
//         if (matrix.coeff(row, col) == 0 && used_positions.find(pos) == used_positions.end()) {
//             matrix.insert(row, col) = 1;
//             used_positions.insert(pos); // Mark the position as used
//             inserted_elements++;
//         }
//     }

//     matrix.makeCompressed();
// }

// // Benchmark function with different sparsity levels
// template<int size>
// void sparse_benchmark() {
//     Eigen::SparseMatrix<int> A(size, size);
//     Eigen::SparseMatrix<int> B(size, size);
//     Eigen::SparseMatrix<int> C(size, size);

//     constexpr int reps = 5;
//     float sparsity_levels[] = {0.1f, 0.3f, 0.5f, 0.7f, 0.9f}; // Different sparsity levels to test

//     for (float sparsity : sparsity_levels) {
//         initialize_sparse_matrix(A, sparsity);
//         initialize_sparse_matrix(B, sparsity);

//         printf("Matrix size N: %d\n", size);
//         std::string benchmark_name = "GEMM Sparse Benchmark with Sparsity " + std::to_string(sparsity);
//         auto sparse_harness = bench::make_harness<reps>([&]() { sparse_gemm(A, B, C); }, benchmark_name.c_str());
//         sparse_harness.run();
//     }
// }

// template<int size>
// void run_matrix_init() {
//     if constexpr (size <= 100) {
//         sparse_benchmark<size>();
//         run_matrix_init<size + 1>(); 
//     }
// }

// int main() {
//     using namespace bench;
//     initialise_monitor_handles();

//     constexpr int starting_size = 2; // Start sweeping matrix at this size
//     run_matrix_init<starting_size>(); 

//     printf("Finished running Eigen GEMM sparse benchmark with varying sparsity levels!\n");

//     exit(1);

//     return 0;
// }


