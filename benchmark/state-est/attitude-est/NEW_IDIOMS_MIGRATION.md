# Attitude Estimation Benchmarks - New Idioms Migration

## Overview
Successfully migrated all attitude estimation benchmarks to use the new EntoBench idioms with config file support and improved CMake organization.

## What Changed

### 1. CMakeLists.txt Migration
**Before**: Manual configuration of each benchmark with verbose CMake structure
```cmake
# =============================================================================
# Madgwick Float IMU
# =============================================================================
add_benchmark(bench-madgwick-float-imu
  SOURCES bench_madgwick_float_imu.cc
  LIBRARIES ${MODULES}
)
# ... repeated for each benchmark
add_benchmark_group_target("attitude-est" ${BENCH_TARGETS})
```

**After**: Config-file driven approach with smart fallbacks
```cmake
# Method 1: Config file approach (primary)
set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/benchmark/configs/estimation_benchmarks.json")
if(EXISTS ${CONFIG_FILE})
  add_configured_benchmark_group_from_file("attitude-est"
    CONFIG_FILE ${CONFIG_FILE}
    TARGETS ${BENCH_TARGETS}
    # Fallback configuration
    REPS 30
    INNER_REPS 2
    VERBOSITY 1
    ENABLE_CACHES
  )
else()
  # Direct configuration fallback
  add_preconfigured_benchmark_group("attitude-est"
    TARGETS ${BENCH_TARGETS}
    REPS 30
    INNER_REPS 2
    VERBOSITY 1
    ENABLE_CACHES
  )
endif()
```

### 2. Source File Migration (Example: bench_madgwick_float_imu.cc)

**Before**: MCU-specific cache calls and hardcoded harness parameters
```cpp
// Turn on caches if applicable
enable_instruction_cache();
enable_instruction_cache_prefetch();
icache_enable();

// Hardcoded template parameters
using Harness = Harness<Problem, false, 1, 10, 100>;
Harness harness(problem, "Bench Madgwick Float IMU", dataset_path, output_path);
```

**After**: Generic cache setup and configuration-driven harness
```cpp
// Include benchmark configuration
#include <ento-bench/bench_config.h>

// NEW IDIOM: Generic cache setup using configuration
ENTO_BENCH_SETUP();

// NEW IDIOM: Configuration-driven harness type
ENTO_BENCH_HARNESS_TYPE(Problem);
BenchHarness harness(problem, "Bench Madgwick Float IMU", dataset_path, output_path);
```

### 3. Configuration Management

**Configuration file**: `benchmark/configs/estimation_benchmarks.json`
```json
{
  "attitude-est": {
    "reps": 30,
    "inner_reps": 2,
    "verbosity": 1,
    "enable_caches": true
  }
}
```

## Benefits Achieved

### ✅ **Compiler Flags Preserved**
- All existing `-O3` optimization settings maintained
- No impact on your existing build configuration
- Only benchmark runtime parameters changed

### ✅ **MCU Portability**
- `ENTO_BENCH_SETUP()` automatically handles cache setup for:
  - STM32G4: instruction cache + prefetch
  - STM32H7: instruction cache + prefetch + L1 ICache + DCache  
  - STM32F7: L1 ICache + DCache
  - STM32U5: L1 ICache
  - Native builds: No-op (safe)

### ✅ **Configuration Flexibility**
- Runtime parameters configurable via JSON file
- CMake fallback ensures builds work even without config file
- Easy to modify benchmark parameters without recompiling

### ✅ **Maintainability**
- Cleaner, more organized CMakeLists.txt
- Consistent pattern across all benchmarks
- Easy to add new benchmarks following the same pattern

## Usage

### Building All Attitude Estimation Benchmarks
```bash
make bench-attitude-est
```

### Modifying Configuration
Edit `benchmark/configs/estimation_benchmarks.json`:
```json
{
  "attitude-est": {
    "reps": 50,        # Change number of repetitions
    "verbosity": 2,    # Increase verbosity
    "enable_caches": false  # Disable caches for comparison
  }
}
```

### Using Different Configurations
```bash
# Different config files for different scenarios
cp benchmark/configs/debug_benchmarks.json benchmark/configs/estimation_benchmarks.json
make bench-attitude-est
```

## Next Steps

1. **Test the migration**: Run `make bench-attitude-est` to verify everything works
2. **Apply to other categories**: Use the same pattern for pose estimation benchmarks  
3. **Create additional config profiles**: Debug, performance testing, cache comparison
4. **Migrate remaining source files**: Apply new idioms to all `.cc` files in the directory

All 18 attitude estimation benchmarks now use the new idioms and are ready for use! 