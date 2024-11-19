#include "/Users/amyle/entomoton-bench/external/eigen/Eigen/Dense"
#include <stdlib.h>
#include <stdio.h>
#include <ento-bench/harness.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>

extern "C" void initialise_monitor_handles(void);

// Dense GEMV
template<int N>
void __attribute__((noinline)) dense_gemv(const Eigen::Matrix<int, N, N>& A, const Eigen::Matrix<int, N, 1>& x, Eigen::Matrix<int, N, 1>& y) {
    if constexpr (N > 0) {
        // printf("Start multiply\n");
        y.noalias() = A * x;
        // printf("Finished multiply\n");
    }
}

// Templated function to initialize matrix and vector
template<int size>
void matrix_vector_init() {
    static Eigen::Matrix<int, size, size> A = Eigen::Matrix<int, size, size>::Ones();
    static Eigen::Matrix<int, size, 1> x = Eigen::Matrix<int, size, 1>::Ones();
    static Eigen::Matrix<int, size, 1> y;
    y.noalias();

    constexpr int reps = 5;
    printf("Matrix size N: %d\n", size);
    auto dense_harness = bench::make_harness<reps>([&]() { dense_gemv<size>(A, x, y); },
                                            "GEMV Dense Benchmark Example");
    dense_harness.run();
}

// Base case to end recursion
template<int size>
void run_matrix_vector_init() {
    if constexpr (size <= 40) {
        matrix_vector_init<size>();
        run_matrix_vector_init<size + 1>(); 
    }
}

int main() {
    using namespace bench;
    initialise_monitor_handles();
    run_matrix_vector_init<31>(); 
    // matrix_vector_init<4>();

    printf("Finished running Eigen GEMV dense benchmark example!\n");

    exit(1);

    return 0;
}
