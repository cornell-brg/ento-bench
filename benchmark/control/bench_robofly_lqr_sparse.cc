#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/bench_config.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/systick_config.h>
#include <ento-control/lqr_traits_robofly.h>

// Include Eigen Sparse support
#include <Eigen/Sparse>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;

constexpr int num_states = 10;
constexpr int num_inputs = 3;

// Sparse matrix types - use float to match RoboFlyLQRTraits
using SparseMatrixf = Eigen::SparseMatrix<float>;
using SparseTriplet = Eigen::Triplet<float>;

// Convert dense matrix to sparse, exploiting known sparsity pattern
template<typename DenseMatrix>
SparseMatrixf to_sparse(const DenseMatrix& dense, float threshold = 1e-12f) {
    std::vector<SparseTriplet> triplets;
    triplets.reserve(dense.rows() * dense.cols() / 4); // Estimate 25% non-zero
    
    for (int i = 0; i < dense.rows(); ++i) {
        for (int j = 0; j < dense.cols(); ++j) {
            if (std::abs(dense(i, j)) > threshold) {
                triplets.emplace_back(i, j, dense(i, j));
            }
        }
    }
    
    SparseMatrixf sparse(dense.rows(), dense.cols());
    sparse.setFromTriplets(triplets.begin(), triplets.end());
    sparse.makeCompressed();
    return sparse;
}

// Sparse LQR controller
class SparseLQRController {
private:
    SparseMatrixf A_sparse;
    SparseMatrixf B_sparse;
    Eigen::MatrixXf K_dense;  // Keep K dense since it's small (3x10)
    
public:
    template<typename AMatrix, typename BMatrix, typename KMatrix>
    SparseLQRController(const AMatrix& A, const BMatrix& B, const KMatrix& K) 
        : A_sparse(to_sparse(A))
        , B_sparse(to_sparse(B))
        , K_dense(K.template cast<float>()) 
    {
        // Print sparsity statistics
        float A_sparsity = 100.0f * (1.0f - float(A_sparse.nonZeros()) / (A_sparse.rows() * A_sparse.cols()));
        float B_sparsity = 100.0f * (1.0f - float(B_sparse.nonZeros()) / (B_sparse.rows() * B_sparse.cols()));
        
        printf("A matrix sparsity: %.1f%% (%d/%d zeros)\n", 
               A_sparsity, A_sparse.rows() * A_sparse.cols() - A_sparse.nonZeros(), 
               A_sparse.rows() * A_sparse.cols());
        printf("B matrix sparsity: %.1f%% (%d/%d zeros)\n", 
               B_sparsity, B_sparse.rows() * B_sparse.cols() - B_sparse.nonZeros(),
               B_sparse.rows() * B_sparse.cols());
    }
    
    // Sparse matrix-vector multiplication
    Eigen::VectorXf sparse_control_step(const Eigen::VectorXf& x_ref, const Eigen::VectorXf& x_current) {
        Eigen::VectorXf error = x_ref - x_current;
        return K_dense * error;  // K is small, keep dense
    }
    
    // For comparison: simulate dynamics with sparse matrices
    Eigen::VectorXf sparse_dynamics_step(const Eigen::VectorXf& x, const Eigen::VectorXf& u) {
        return A_sparse * x + B_sparse * u;
    }
};

// Simple LQR controller for comparison
class DenseLQRController {
public:
    Eigen::VectorXf control_step(const Eigen::VectorXf& x_ref, const Eigen::VectorXf& x_current) {
        Eigen::VectorXf error = x_ref - x_current;
        return RoboFlyLQRTraits::K * error;
    }
};

void benchmark_sparse_lqr() {
    // Create dense LQR controller (original)
    DenseLQRController dense_controller;
    
    // Create sparse LQR controller
    SparseLQRController sparse_controller(
        RoboFlyLQRTraits::Adyn, 
        RoboFlyLQRTraits::Bdyn, 
        RoboFlyLQRTraits::K
    );
    
    // Benchmark variables
    Eigen::VectorXf x_ref(num_states), x_current(num_states);
    Eigen::VectorXf u_dense(num_inputs), u_sparse(num_inputs);
    
    // Initialize with some test data
    x_ref.setRandom();
    x_current.setRandom();
    x_current *= 0.1f; // Small perturbation
    
    printf("\n=== LQR Sparse vs Dense Comparison ===\n");
    printf("States: %d, Inputs: %d\n", num_states, num_inputs);
    
    // Benchmark dense LQR
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 1000; i++) {
            u_dense = dense_controller.control_step(x_ref, x_current);
            // Prevent optimization
            asm volatile("" : : "m"(u_dense) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        uint32_t dense_cycles = end_cycles - start_cycles;
        printf("Dense LQR Control Step: %lu cycles (1000 iterations)\n", (unsigned long)dense_cycles);
    }
    
    // Benchmark sparse LQR  
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 1000; i++) {
            u_sparse = sparse_controller.sparse_control_step(x_ref, x_current);
            // Prevent optimization
            asm volatile("" : : "m"(u_sparse) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        uint32_t sparse_cycles = end_cycles - start_cycles;
        printf("Sparse LQR Control Step: %lu cycles (1000 iterations)\n", (unsigned long)sparse_cycles);
    }
    
    // Verify results are identical (within numerical precision)
    float control_error = (u_dense - u_sparse).norm();
    printf("Control output difference: %.2e (should be ~0)\n", control_error);
    
    if (control_error > 1e-5f) {
        printf("WARNING: Sparse and dense results differ significantly!\n");
        printf("Dense:  [%.6f, %.6f, %.6f]\n", u_dense(0), u_dense(1), u_dense(2));
        printf("Sparse: [%.6f, %.6f, %.6f]\n", u_sparse(0), u_sparse(1), u_sparse(2));
    }
    
    // Additional benchmark: dynamics propagation (for EKF relevance)
    Eigen::VectorXf x_next_dense, x_next_sparse;
    
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 1000; i++) {
            x_next_dense = RoboFlyLQRTraits::Adyn * x_current + RoboFlyLQRTraits::Bdyn * u_dense;
            asm volatile("" : : "m"(x_next_dense) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        uint32_t dense_dyn_cycles = end_cycles - start_cycles;
        printf("Dense Dynamics Step: %lu cycles (1000 iterations)\n", (unsigned long)dense_dyn_cycles);
    }
    
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 1000; i++) {
            x_next_sparse = sparse_controller.sparse_dynamics_step(x_current, u_sparse);
            asm volatile("" : : "m"(x_next_sparse) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        uint32_t sparse_dyn_cycles = end_cycles - start_cycles;
        printf("Sparse Dynamics Step: %lu cycles (1000 iterations)\n", (unsigned long)sparse_dyn_cycles);
    }
    
    float dynamics_error = (x_next_dense - x_next_sparse).norm();
    printf("Dynamics output difference: %.2e (should be ~0)\n", dynamics_error);
}

int main() {
#if defined(SEMIHOSTING)
    initialise_monitor_handles();
#endif

    // Configure clock
    sys_clk_cfg();
    SysTick_Setup();
    __enable_irq();

    // Generic cache setup via config macro
    ENTO_BENCH_SETUP();

    printf("=== RoboFly Sparse LQR Benchmark ===\n");
    printf("Testing sparse matrix optimization for LQR control\n");
    printf("This could inform EKF sparse implementations\n\n");

    benchmark_sparse_lqr();

    printf("\n=== Analysis ===\n");
    printf("If sparse is faster:\n");
    printf("  - Validates sparse optimization for small matrices\n");
    printf("  - Could be applied to EKF Jacobians\n");
    printf("  - Might explain paper's 500 FLOP claim\n");
    printf("If sparse is slower:\n");
    printf("  - Eigen sparse overhead dominates for small matrices\n");
    printf("  - Custom sparse implementation might be needed\n");
    printf("  - Dense operations are already well-optimized\n");

    return 0;
} 