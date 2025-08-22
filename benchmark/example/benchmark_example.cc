// Vector-Vector Add Benchmark Example
// Part of ento-bench example benchmarks

#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/bench_config.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/systick_config.h>

#include <Eigen/Dense>
#include <array>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;

// =============================================================================
// Vector-Vector Add Kernel (Defined Directly in Benchmark)
// =============================================================================

template<typename Scalar, size_t N>
class VectorAddKernel {
public:
    using VectorType = std::array<Scalar, N>;
    
    VectorAddKernel() = default;
    
    // Perform vector addition: result = a + b
    void solve(const VectorType& a, const VectorType& b, VectorType& result) const {
        // Use volatile to prevent compiler from optimizing away the computation
        for (size_t i = 0; i < N; ++i) {
            volatile Scalar temp_a = a[i];
            volatile Scalar temp_b = b[i];
            result[i] = temp_a + temp_b;
        }
        
        // Additional memory barrier to ensure computation isn't optimized away
        asm volatile("" : : "m"(result) : "memory");
    }
    
    // Validate the result (basic sanity check)
    bool validate(const VectorType& a, const VectorType& b, const VectorType& result) const {
        constexpr Scalar tolerance = Scalar(1e-6);
        
        for (size_t i = 0; i < N; ++i) {
            Scalar expected = a[i] + b[i];
            Scalar error = (result[i] > expected) ? (result[i] - expected) : (expected - result[i]);
            
            if (error > tolerance) {
                ENTO_DEBUG("Validation failed at index %zu: expected %.6f, got %.6f (error: %.6f)", 
                          i, static_cast<float>(expected), static_cast<float>(result[i]), static_cast<float>(error));
                return false;
            }
        }
        return true;
    }
};

// =============================================================================
// Simple Problem Class for Vector Addition
// =============================================================================

template<typename Scalar, size_t VectorSize>
class VectorAddProblem {
public:
    using VectorType = std::array<Scalar, VectorSize>;
    using KernelType = VectorAddKernel<Scalar, VectorSize>;
    
    static constexpr bool RequiresDataset_ = false;
    static constexpr bool SaveResults_ = false;
    
    VectorAddProblem() : kernel_() {
        // Initialize two static test vectors
        initialize_test_data();
    }
    
    void solve() {
        // Perform the vector addition
        kernel_.solve(vector_a_, vector_b_, result_);
        
        // Store result for validation
        last_result_ = result_;
    }
    
    void validate() {
        bool is_valid = kernel_.validate(vector_a_, vector_b_, last_result_);
        
        if (!is_valid) {
            ENTO_DEBUG("ERROR: Vector addition validation failed!");
        } else {
            ENTO_DEBUG("Vector addition validation passed");
        }
        
        // Print first few elements for verification
        if (VectorSize >= 3) {
            ENTO_DEBUG("Sample results: [%.3f, %.3f, %.3f, ...]", 
                      static_cast<float>(last_result_[0]),
                      static_cast<float>(last_result_[1]), 
                      static_cast<float>(last_result_[2]));
        }
    }

private:
    void initialize_test_data() {
        // Initialize vector A with a simple pattern
        for (size_t i = 0; i < VectorSize; ++i) {
            vector_a_[i] = static_cast<Scalar>(i * 0.1 + 1.0);  // 1.0, 1.1, 1.2, ...
        }
        
        // Initialize vector B with a different pattern  
        for (size_t i = 0; i < VectorSize; ++i) {
            vector_b_[i] = static_cast<Scalar>(i * 0.05 + 0.5);  // 0.5, 0.55, 0.6, ...
        }
        
        ENTO_DEBUG("Initialized test vectors of size %zu", VectorSize);
        ENTO_DEBUG("Vector A sample: [%.3f, %.3f, %.3f, ...]", 
                  static_cast<float>(vector_a_[0]),
                  static_cast<float>(vector_a_[1]), 
                  static_cast<float>(vector_a_[2]));
        ENTO_DEBUG("Vector B sample: [%.3f, %.3f, %.3f, ...]", 
                  static_cast<float>(vector_b_[0]),
                  static_cast<float>(vector_b_[1]), 
                  static_cast<float>(vector_b_[2]));
    }

private:
    KernelType kernel_;
    
    // Static test data (protected with volatile access in kernel)
    VectorType vector_a_;
    VectorType vector_b_;
    VectorType result_;
    VectorType last_result_;
};

// =============================================================================
// Main Benchmark Function
// =============================================================================

int main()
{
    using Scalar = float;
    constexpr size_t VectorSize = 1000;  // Large enough to measure meaningful performance
    using Problem = VectorAddProblem<Scalar, VectorSize>;
    
    initialise_monitor_handles();

    // Configure max clock rate and set flash latency
    sys_clk_cfg();
    SysTick_Setup();
    __enable_irq();

    // NEW IDIOM: Generic cache setup using configuration
    ENTO_BENCH_SETUP();

    // Print benchmark configuration
    ENTO_BENCH_PRINT_CONFIG();

    ENTO_INFO("Vector-Vector Add Example: %zu float elements", VectorSize);

    // Create problem instance
    Problem problem;

    // NEW IDIOM: Configuration-driven harness type
    ENTO_BENCH_HARNESS_TYPE(Problem);
    BenchHarness harness(problem, "Bench Vector-Vector Add [float]");

    // Run the benchmark
    harness.run();

    ENTO_INFO("Finished Vector-Vector Add Example: %zu elements", VectorSize);

    exit(1);
    return 0;
} 