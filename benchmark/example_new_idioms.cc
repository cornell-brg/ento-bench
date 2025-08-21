// Example Benchmark with New Cache and Configuration Idioms
// This demonstrates the new standardized patterns for ento-bench

#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

using namespace EntoBench;
using namespace EntoUtil;

// Example problem class for demonstration
class ExampleProblem {
public:
    static constexpr bool RequiresDataset_ = false;
    static constexpr bool SaveResults_ = false;
    
    ExampleProblem() = default;
    
    void solve() {
        // Simulate some computation
        volatile int sum = 0;
        for (int i = 0; i < 1000; ++i) {
            sum += i * i;
        }
    }
};

int main()
{
    #if defined(SEMIHOSTING)
    initialise_monitor_handles();
    #endif
    
    // Configure max clock rate and set flash latency
    sys_clk_cfg();
    
    // NEW IDIOM: Generic cache enablement using abstracted function
    // This replaces the old pattern of:
    //   enable_instruction_cache();
    //   enable_instruction_cache_prefetch(); 
    //   icache_enable();
    // With a single call that handles MCU-specific sequences:
    ENTO_BENCH_SETUP();  // This enables caches if DefaultEnableCaches is true
    
    // Alternative explicit cache control:
    // enable_all_caches();  // Enable all available caches for this MCU
    // disable_all_caches(); // Disable all caches
    
    printf("=== Example Benchmark with New Idioms ===\n");
    
    // Create problem instance
    ExampleProblem problem;
    
    // NEW IDIOM: Standardized Harness type using configuration
    // This replaces manual template parameter specification with 
    // configuration-driven defaults that can be overridden via CMake
    ENTO_BENCH_HARNESS_TYPE(ExampleProblem);
    
    // Alternative: Custom harness configuration
    // ENTO_BENCH_CUSTOM_HARNESS_TYPE(ExampleProblem, 
    //                                true,    // DoWarmup
    //                                10,      // Reps
    //                                5,       // InnerReps  
    //                                100,     // MaxProblems
    //                                2);      // Verbosity
    
    // Create harness instance with standardized type
    BenchHarness harness(problem, "Example Benchmark [New Idioms]");
    
    // Run the benchmark
    harness.run();
    
    printf("=== Benchmark Complete ===\n");
    printf("Configuration used:\n");
    printf("  Reps: %zu\n", DefaultReps);
    printf("  InnerReps: %zu\n", DefaultInnerReps);
    printf("  Verbosity: %d\n", DefaultVerbosity);
    printf("  MaxProblems: %zu\n", DefaultMaxProblems);
    printf("  DoWarmup: %s\n", DefaultDoWarmup ? "true" : "false");
    printf("  EnableCaches: %s\n", DefaultEnableCaches ? "true" : "false");
    
    exit(1);
    return 0;
} 