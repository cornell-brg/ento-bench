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
// Benchmark Configuration with constexpr defaults
// Override these values with preprocessor defines if needed
//=============================================================================

#ifndef REPS
constexpr std::size_t DefaultReps = 1;
#else
constexpr std::size_t DefaultReps = REPS;
#endif

#ifndef INNER_REPS
constexpr std::size_t DefaultInnerReps = 1;
#else
constexpr std::size_t DefaultInnerReps = INNER_REPS;
#endif

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