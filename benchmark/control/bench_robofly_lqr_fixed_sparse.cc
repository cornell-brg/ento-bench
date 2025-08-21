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

extern "C" void initialise_monitor_handles(void);


using namespace EntoBench;
using namespace EntoUtil;

// Fixed-size arrays - no dynamic allocation
using StateVector = float[10];
using ControlVector = float[3];

// Sparse matrix types
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

// Truly sparse LQR controller - everything is fixed-size arrays
class TrulySparseLQRController {
public:
    // Pre-computed sparse K matrix indices and values (no runtime computation!)
    // K matrix non-zero elements: K[0,2]=0.070711, K[0,5]=0.876310, etc.
    static void sparse_control_step(const StateVector& x_ref, const StateVector& x_current, ControlVector& u) {
        // Compute error first - INCLUDED FOR FAIR COMPARISON
        StateVector error;
        for (int i = 0; i < 10; i++) {
            error[i] = x_ref[i] - x_current[i];
        }
        
        // Direct sparse K*error - NO BRANCHES, NO FUNCTION CALLS!
        // Pre-computed from actual K matrix values
        u[0] = 7.07106814e-02f * error[2] + 8.76309752e-01f * error[5];
        
        u[1] = -1.41421363e-01f * error[1] + -4.78060901e-01f * error[4] + 
                4.45830584e+00f * error[6] + 2.01443744e+00f * error[8];
        
        u[2] = 1.41421363e-01f * error[0] + 4.79498327e-01f * error[3] + 
               4.50604486e+00f * error[7] + 2.05993366e+00f * error[9];
    }
    
    // Truly sparse A*x + B*u (hand-optimized for RoboFly)
    static void sparse_dynamics_step(const StateVector& x, const ControlVector& u, StateVector& x_next) {
        // Direct sparse operations - NO LOOPS, NO BRANCHES!
        
        // A*x: Only non-zero elements (from RoboFly dynamics)
        x_next[0] = x[3];                           // A[0,3] = 1
        x_next[1] = x[4];                           // A[1,4] = 1  
        x_next[2] = x[5];                           // A[2,5] = 1
        x_next[3] = 9.81000042e+00f * x[7];         // A[3,7] = 9.81000042e+00f
        x_next[4] = -9.81000042e+00f * x[6];        // A[4,6] = -9.81000042e+00f
        x_next[5] = 1.96987957e-01f * u[0];         // B[5,0] = 1.96987957e-01f (A*x contributes 0)
        x_next[6] = x[8];                           // A[6,8] = 1
        x_next[7] = x[9];                           // A[7,9] = 1
        x_next[8] = 1.53846161e+02f * u[1];         // B[8,1] = 1.53846161e+02f (A*x contributes 0)
        x_next[9] = 3.70370369e+01f * u[2];         // B[9,2] = 3.70370369e+01f (A*x contributes 0)
    }
};

// Smart Eigen sparse controller - uses static allocation where possible
class SmartEigenSparseController {
private:
    static SparseMatrixf A_sparse;
    static SparseMatrixf B_sparse;
    static SparseMatrixf K_sparse;
    static bool initialized;
    
public:
    static void initialize() {
        if (!initialized) {
            A_sparse = to_sparse(RoboFlyLQRTraits::Adyn);
            B_sparse = to_sparse(RoboFlyLQRTraits::Bdyn);
            K_sparse = to_sparse(RoboFlyLQRTraits::K);
            initialized = true;
            
            printf("Sparse matrix stats:\n");
            printf("  A: %d/%d non-zeros (%.1f%% sparse)\n", 
                   A_sparse.nonZeros(), A_sparse.rows() * A_sparse.cols(),
                   100.0f * (1.0f - float(A_sparse.nonZeros()) / (A_sparse.rows() * A_sparse.cols())));
            printf("  B: %d/%d non-zeros (%.1f%% sparse)\n", 
                   B_sparse.nonZeros(), B_sparse.rows() * B_sparse.cols(),
                   100.0f * (1.0f - float(B_sparse.nonZeros()) / (B_sparse.rows() * B_sparse.cols())));
            printf("  K: %d/%d non-zeros (%.1f%% sparse)\n", 
                   K_sparse.nonZeros(), K_sparse.rows() * K_sparse.cols(),
                   100.0f * (1.0f - float(K_sparse.nonZeros()) / (K_sparse.rows() * K_sparse.cols())));
        }
    }
    
    // Sparse control using fixed-size output
    static void sparse_control_step(const StateVector& x_ref, const StateVector& x_current, ControlVector& u) {
        // Use Eigen::Map to avoid allocation
        Eigen::Map<const Eigen::VectorXf> x_ref_map(x_ref, 10);
        Eigen::Map<const Eigen::VectorXf> x_current_map(x_current, 10);
        Eigen::Map<Eigen::VectorXf> u_map(u, 3);
        
        // Compute error (this allocates, but small) - INCLUDED FOR FAIR COMPARISON
        Eigen::VectorXf error = x_ref_map - x_current_map;
        
        // Sparse matrix-vector multiply
        u_map = K_sparse * error;
    }
    
    // Sparse dynamics using fixed-size output
    static void sparse_dynamics_step(const StateVector& x, const ControlVector& u, StateVector& x_next) {
        // Use Eigen::Map to avoid allocation for inputs/outputs
        Eigen::Map<const Eigen::VectorXf> x_map(x, 10);
        Eigen::Map<const Eigen::VectorXf> u_map(u, 3);
        Eigen::Map<Eigen::VectorXf> x_next_map(x_next, 10);
        
        // Sparse matrix-vector multiply (this may allocate temporaries)
        x_next_map = A_sparse * x_map + B_sparse * u_map;
    }
    
    // Alternative: pre-allocate temporaries to avoid allocation in hot path
    static void sparse_dynamics_step_preallocated(const StateVector& x, const ControlVector& u, StateVector& x_next) {
        static Eigen::VectorXf temp_Ax(10);
        static Eigen::VectorXf temp_Bu(10);
        
        Eigen::Map<const Eigen::VectorXf> x_map(x, 10);
        Eigen::Map<const Eigen::VectorXf> u_map(u, 3);
        Eigen::Map<Eigen::VectorXf> x_next_map(x_next, 10);
        
        // Use pre-allocated temporaries
        temp_Ax = A_sparse * x_map;
        temp_Bu = B_sparse * u_map;
        x_next_map = temp_Ax + temp_Bu;
    }
};

// Static member definitions
SparseMatrixf SmartEigenSparseController::A_sparse;
SparseMatrixf SmartEigenSparseController::B_sparse;
SparseMatrixf SmartEigenSparseController::K_sparse;
bool SmartEigenSparseController::initialized = false;

// Dense controller using Eigen (for comparison)
class DenseEigenController {
public:
    static void dense_control_step(const StateVector& x_ref, const StateVector& x_current, ControlVector& u) {
        Eigen::Map<const Eigen::VectorXf> x_ref_eigen(x_ref, 10);
        Eigen::Map<const Eigen::VectorXf> x_current_eigen(x_current, 10);
        Eigen::Map<Eigen::VectorXf> u_eigen(u, 3);
        
        // Compute error - INCLUDED FOR FAIR COMPARISON
        Eigen::VectorXf error = x_ref_eigen - x_current_eigen;
        u_eigen = RoboFlyLQRTraits::K * error;
    }
    
    static void dense_dynamics_step(const StateVector& x, const ControlVector& u, StateVector& x_next) {
        Eigen::Map<const Eigen::VectorXf> x_eigen(x, 10);
        Eigen::Map<const Eigen::VectorXf> u_eigen(u, 3);
        Eigen::Map<Eigen::VectorXf> x_next_eigen(x_next, 10);
        
        x_next_eigen = RoboFlyLQRTraits::Adyn * x_eigen + RoboFlyLQRTraits::Bdyn * u_eigen;
    }
};

// Raw array dense controller (no Eigen overhead)
class DenseArrayController {
public:
    static void dense_control_step(const StateVector& x_ref, const StateVector& x_current, ControlVector& u) {
        StateVector error;
        // Compute error - INCLUDED FOR FAIR COMPARISON
        for (int i = 0; i < 10; i++) {
            error[i] = x_ref[i] - x_current[i];
        }
        
        // Full K*error multiplication
        for (int i = 0; i < 3; i++) {
            u[i] = 0.0f;
            for (int j = 0; j < 10; j++) {
                u[i] += RoboFlyLQRTraits::K(i, j) * error[j];
            }
        }
    }
    
    static void dense_dynamics_step(const StateVector& x, const ControlVector& u, StateVector& x_next) {
        // Full A*x + B*u multiplication
        for (int i = 0; i < 10; i++) {
            x_next[i] = 0.0f;
            // A*x
            for (int j = 0; j < 10; j++) {
                x_next[i] += RoboFlyLQRTraits::Adyn(i, j) * x[j];
            }
            // B*u
            for (int j = 0; j < 3; j++) {
                x_next[i] += RoboFlyLQRTraits::Bdyn(i, j) * u[j];
            }
        }
    }
};

void benchmark_truly_sparse() {
    printf("=== Comprehensive Sparse vs Dense Comparison ===\n");
    
    // Initialize sparse matrices
    SmartEigenSparseController::initialize();
    
    // Test data - fixed arrays
    StateVector x_ref, x_current;
    StateVector x_next_hand_sparse, x_next_eigen_sparse, x_next_eigen_sparse_prealloc, x_next_eigen_dense, x_next_array_dense;
    ControlVector u_hand_sparse, u_eigen_sparse, u_eigen_dense, u_array_dense;
    
    // Initialize with random data
    for (int i = 0; i < 10; i++) {
        x_ref[i] = (float)rand() / RAND_MAX;
        x_current[i] = (float)rand() / RAND_MAX * 0.1f;
    }
    
    printf("\nTesting LQR Control Step (K * error):\n");
    
    // Benchmark hand-optimized sparse control
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            TrulySparseLQRController::sparse_control_step(x_ref, x_current, u_hand_sparse);
            asm volatile("" : : "m"(u_hand_sparse) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Hand-Optimized Sparse: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Benchmark Eigen sparse control
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            SmartEigenSparseController::sparse_control_step(x_ref, x_current, u_eigen_sparse);
            asm volatile("" : : "m"(u_eigen_sparse) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Eigen Sparse Control: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Benchmark Eigen dense control
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            DenseEigenController::dense_control_step(x_ref, x_current, u_eigen_dense);
            asm volatile("" : : "m"(u_eigen_dense) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Eigen Dense Control: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Benchmark array dense control
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            DenseArrayController::dense_control_step(x_ref, x_current, u_array_dense);
            asm volatile("" : : "m"(u_array_dense) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Array Dense Control: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    printf("\nTesting Dynamics Step (A*x + B*u):\n");
    
    // Benchmark hand-optimized sparse dynamics
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            TrulySparseLQRController::sparse_dynamics_step(x_current, u_hand_sparse, x_next_hand_sparse);
            asm volatile("" : : "m"(x_next_hand_sparse) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Hand-Optimized Sparse: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Benchmark Eigen sparse dynamics
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            SmartEigenSparseController::sparse_dynamics_step(x_current, u_eigen_sparse, x_next_eigen_sparse);
            asm volatile("" : : "m"(x_next_eigen_sparse) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Eigen Sparse Dynamics: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Benchmark Eigen sparse dynamics (pre-allocated)
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            SmartEigenSparseController::sparse_dynamics_step_preallocated(x_current, u_eigen_sparse, x_next_eigen_sparse_prealloc);
            asm volatile("" : : "m"(x_next_eigen_sparse_prealloc) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Eigen Sparse (Pre-alloc): %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Benchmark Eigen dense dynamics
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            DenseEigenController::dense_dynamics_step(x_current, u_eigen_dense, x_next_eigen_dense);
            asm volatile("" : : "m"(x_next_eigen_dense) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Eigen Dense Dynamics: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Benchmark array dense dynamics
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            DenseArrayController::dense_dynamics_step(x_current, u_array_dense, x_next_array_dense);
            asm volatile("" : : "m"(x_next_array_dense) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Array Dense Dynamics: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Verify correctness
    float max_control_error = 0.0f, max_dynamics_error = 0.0f;
    for (int i = 0; i < 3; i++) {
        float error = fabsf(u_hand_sparse[i] - u_eigen_dense[i]);
        if (error > max_control_error) max_control_error = error;
    }
    for (int i = 0; i < 10; i++) {
        float error = fabsf(x_next_hand_sparse[i] - x_next_eigen_dense[i]);
        if (error > max_dynamics_error) max_dynamics_error = error;
    }
    
    printf("\nVerification:\n");
    printf("Max control error (hand sparse vs Eigen dense): %.2e\n", max_control_error);
    printf("Max dynamics error (hand sparse vs Eigen dense): %.2e\n", max_dynamics_error);
    
    // FLOP analysis
    printf("\nFLOP Analysis (including error computation):\n");
    printf("Control (error + K*error): Dense=40 FLOPs, Hand Sparse=27 FLOPs\n");
    printf("Dynamics (A*x+B*u): Dense=130 FLOPs, Hand Sparse=10 FLOPs\n");
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

    printf("=== RoboFly Comprehensive Sparse Optimization ===\n");
    printf("Comparing all approaches: Hand-optimized, Eigen Sparse, Eigen Dense, Raw Arrays\n\n");

    benchmark_truly_sparse();

    printf("\n=== Analysis ===\n");
    printf("This comprehensive test compares:\n");
    printf("1. Hand-optimized sparse (best case)\n");
    printf("2. Eigen sparse with smart allocation\n");
    printf("3. Eigen sparse with pre-allocation\n");
    printf("4. Eigen dense (baseline)\n");
    printf("5. Raw array dense (no library overhead)\n");
    printf("\nShould reveal the true cost of library overhead vs sparsity benefits\n");

    return 0;
} 