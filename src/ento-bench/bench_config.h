#ifndef ENTO_BENCH_CONFIG_H
#define ENTO_BENCH_CONFIG_H

#include <cstddef>

// Include cache utilities for ENTO_BENCH_SETUP macro
#ifndef NATIVE
#include <ento-mcu/cache_util.h>
#endif

namespace EntoBench {

// Forward declaration of Harness template (no default arguments to avoid redefinition)
template<typename Problem,
         bool DoWarmup,
         std::size_t Reps,
         std::size_t InnerReps,
         std::size_t MaxProblems,
         int Verbosity>
class Harness;

//=============================================================================
// Automatic H7 Performance Scaling Configuration
//=============================================================================

// Detect H7 automatically and set performance multiplier
#if defined(STM32H7) || defined(STM32H7xx) || defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx)
constexpr std::size_t H7_PERFORMANCE_MULTIPLIER = 1;  // @TODO: Fix this in cmake before making 2. H7 is ~2x faster than other STM32s
#else
constexpr std::size_t H7_PERFORMANCE_MULTIPLIER = 1;  // No scaling for non-H7
#endif

// Compile-time verification that CMake and C++ multipliers are in sync
#ifdef H7_CMAKE_MULTIPLIER
static_assert(H7_PERFORMANCE_MULTIPLIER == H7_CMAKE_MULTIPLIER, 
              "H7 Performance Multiplier mismatch between CMake and C++! "
              "Update both benchmark/state-est/ekf/CMakeLists.txt and src/ento-bench/bench_config.h");
#endif

//=============================================================================
// Benchmark Configuration with constexpr defaults and automatic H7 scaling
// Override these values with preprocessor defines if needed
//=============================================================================

#ifndef REPS
constexpr std::size_t DefaultReps = 1;
#else
constexpr std::size_t DefaultReps = REPS;
#endif

#ifndef INNER_REPS
constexpr std::size_t BaseInnerReps = 1;
#else
constexpr std::size_t BaseInnerReps = INNER_REPS;
#endif

// Apply automatic H7 scaling to INNER_REPS only (REPS comes from JSON config unchanged)
constexpr std::size_t DefaultInnerReps = BaseInnerReps * H7_PERFORMANCE_MULTIPLIER;

#ifndef VERBOSITY
constexpr int DefaultVerbosity = 1;
#else
constexpr int DefaultVerbosity = VERBOSITY;
#endif

#ifndef MAX_PROBLEMS
constexpr std::size_t DefaultMaxProblems = 0;  // 0 means no limit
#else
constexpr std::size_t DefaultMaxProblems = MAX_PROBLEMS;
#endif

#ifndef DO_WARMUP
constexpr bool DefaultDoWarmup = false;
#else
constexpr bool DefaultDoWarmup = (DO_WARMUP != 0);
#endif

#ifndef ENABLE_CACHES
constexpr bool DefaultEnableCaches = true;
#else
constexpr bool DefaultEnableCaches = (ENABLE_CACHES != 0);
#endif

//=============================================================================
// Standardized Harness Type Alias Template
// Usage: using BenchHarness = DefaultHarness<Problem>;
// Or with custom parameters: using BenchHarness = CustomHarness<Problem, true, 10, 5>;
//=============================================================================

template<typename Problem>
using DefaultHarness = Harness<Problem, DefaultDoWarmup, DefaultReps, DefaultInnerReps, DefaultMaxProblems, DefaultVerbosity>;

template<typename Problem,
         bool DoWarmup,
         std::size_t Reps,
         std::size_t InnerReps = DefaultInnerReps,
         std::size_t MaxProblems = DefaultMaxProblems,
         int Verbosity = DefaultVerbosity>
using CustomHarness = Harness<Problem, DoWarmup, Reps, InnerReps, MaxProblems, Verbosity>;

} // namespace EntoBench

//=============================================================================
// Convenience Macros for Standard Benchmark Setup
//=============================================================================

#define ENTO_BENCH_SETUP() \
    do { \
        if constexpr (EntoBench::DefaultEnableCaches) { \
            enable_all_caches(); \
        } \
    } while(0)

#define ENTO_BENCH_HARNESS_TYPE(Problem) \
    using BenchHarness = EntoBench::DefaultHarness<Problem>

#define ENTO_BENCH_CUSTOM_HARNESS_TYPE(Problem, DoWarmup, Reps, InnerReps, MaxProblems, Verbosity) \
    using BenchHarness = EntoBench::CustomHarness<Problem, DoWarmup, Reps, InnerReps, MaxProblems, Verbosity>

#endif // ENTO_BENCH_CONFIG_H 