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

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;

constexpr int num_states = 10;
constexpr int num_inputs = 3;

// Custom sparse implementation exploiting known RoboFly sparsity pattern
class CustomSparseLQRController {
public:
    // Hand-optimized A*x for RoboFly dynamics
    // A matrix pattern (only non-zeros):
    // A[0,3] = 1, A[1,4] = 1, A[2,5] = 1
    // A[3,7] = 9.81, A[4,6] = -9.81
    // A[6,8] = 1, A[7,9] = 1
    static void custom_A_multiply(const float* x, float* result) {
        // Zero out result
        for (int i = 0; i < 10; i++) result[i] = 0.0f;
        
        // Only compute non-zero elements
        result[0] = x[3];           // A[0,3] * x[3]
        result[1] = x[4];           // A[1,4] * x[4]  
        result[2] = x[5];           // A[2,5] * x[5]
        result[3] = 9.81f * x[7];   // A[3,7] * x[7]
        result[4] = -9.81f * x[6];  // A[4,6] * x[6]
        result[6] = x[8];           // A[6,8] * x[8]
        result[7] = x[9];           // A[7,9] * x[9]
        // result[5], result[8], result[9] remain 0
    }
    
    // Hand-optimized B*u for RoboFly dynamics  
    // B matrix pattern (only non-zeros):
    // B[5,0] = 1, B[8,1] = 1, B[9,2] = 1
    static void custom_B_multiply(const float* u, float* result) {
        // Zero out result
        for (int i = 0; i < 10; i++) result[i] = 0.0f;
        
        // Only compute non-zero elements
        result[5] = u[0];  // B[5,0] * u[0]
        result[8] = u[1];  // B[8,1] * u[1]
        result[9] = u[2];  // B[9,2] * u[2]
        // All other elements remain 0
    }
    
    // Combined A*x + B*u optimized
    static void custom_dynamics_step(const float* x, const float* u, float* result) {
        // Zero out result
        for (int i = 0; i < 10; i++) result[i] = 0.0f;
        
        // A*x contribution
        result[0] = x[3];           
        result[1] = x[4];           
        result[2] = x[5];           
        result[3] = 9.81f * x[7];   
        result[4] = -9.81f * x[6];  
        result[6] = x[8];           
        result[7] = x[9];           
        
        // B*u contribution (add to existing)
        result[5] += u[0];  
        result[8] += u[1];  
        result[9] += u[2];  
    }
};

// SIMD-optimized version (if available)
class SIMDSparseLQRController {
public:
    // Use ARM NEON if available for vectorized operations
    static void simd_dynamics_step(const float* x, const float* u, float* result) {
        // For now, same as custom but could use NEON intrinsics
        CustomSparseLQRController::custom_dynamics_step(x, u, result);
    }
};

// Dense controller for comparison
class DenseLQRController {
public:
    static void dense_dynamics_step(const float* x, const float* u, float* result) {
        // Full matrix multiplication
        for (int i = 0; i < 10; i++) {
            result[i] = 0.0f;
            for (int j = 0; j < 10; j++) {
                result[i] += RoboFlyLQRTraits::Adyn(i, j) * x[j];
            }
            for (int j = 0; j < 3; j++) {
                result[i] += RoboFlyLQRTraits::Bdyn(i, j) * u[j];
            }
        }
    }
};

void benchmark_custom_sparse() {
    printf("=== Custom Sparse vs Dense Comparison ===\n");
    
    // Test data
    float x[10], u[3];
    float result_dense[10], result_custom[10], result_simd[10];
    
    // Initialize with random data
    for (int i = 0; i < 10; i++) x[i] = (float)rand() / RAND_MAX;
    for (int i = 0; i < 3; i++) u[i] = (float)rand() / RAND_MAX;
    
    // Benchmark dense dynamics
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            DenseLQRController::dense_dynamics_step(x, u, result_dense);
            asm volatile("" : : "m"(result_dense) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Dense Dynamics: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Benchmark custom sparse dynamics
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            CustomSparseLQRController::custom_dynamics_step(x, u, result_custom);
            asm volatile("" : : "m"(result_custom) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("Custom Sparse Dynamics: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Benchmark SIMD sparse dynamics
    {
        uint32_t start_cycles = get_cycle_count();
        for (int i = 0; i < 10000; i++) {
            SIMDSparseLQRController::simd_dynamics_step(x, u, result_simd);
            asm volatile("" : : "m"(result_simd) : "memory");
        }
        uint32_t end_cycles = get_cycle_count();
        printf("SIMD Sparse Dynamics: %lu cycles (10000 iterations)\n", 
               (unsigned long)(end_cycles - start_cycles));
    }
    
    // Verify correctness
    float max_error = 0.0f;
    for (int i = 0; i < 10; i++) {
        float error = fabsf(result_dense[i] - result_custom[i]);
        if (error > max_error) max_error = error;
    }
    printf("Max error (dense vs custom): %.2e\n", max_error);
    
    // Count actual operations
    printf("\nOperation count analysis:\n");
    printf("Dense: %d FLOPs (10×10 + 10×3 = 130 multiplies + 120 adds)\n", 130 + 120);
    printf("Custom sparse: %d FLOPs (7 + 3 = 10 multiplies + 3 adds)\n", 10 + 3);
    printf("Theoretical speedup: %.1fx\n", (130.0f + 120.0f) / (10.0f + 3.0f));
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

    printf("=== RoboFly Custom Sparse Optimization ===\n");
    printf("Hand-optimized sparse kernels vs Eigen dense\n\n");

    benchmark_custom_sparse();

    printf("\n=== Analysis ===\n");
    printf("Custom sparse should be much faster because:\n");
    printf("- No library overhead\n");
    printf("- Only 13 FLOPs vs 250 FLOPs (19x reduction)\n");
    printf("- Better cache locality\n");
    printf("- Compile-time optimized\n");

    return 0;
} 