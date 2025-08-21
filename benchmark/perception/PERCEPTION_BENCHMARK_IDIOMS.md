# Perception Benchmarks - EntoBench Idioms Guide

## Overview
This document defines the idiomatic patterns for perception benchmarks in EntoBench, based on the successful migration patterns from attitude estimation benchmarks.

## Key Idioms

### 1. Header Includes Pattern
```cpp
#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

// Problem-specific includes
#include <ento-feature2d/feature_recognition_problem.h>
#include <ento-feature2d/[algorithm].h>

// Include benchmark configuration
#include <ento-bench/bench_config.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoFeature2D;
```

### 2. Main Function Setup Pattern
```cpp
int main()
{
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // NEW IDIOM: Generic cache setup using configuration
  ENTO_BENCH_SETUP();

  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();
  
  // ... rest of main function
}
```

### 3. Cache Setup Migration
**OLD (Non-idiomatic)**:
```cpp
enable_instruction_cache();
enable_instruction_cache_prefetch();
icache_enable();
```

**NEW (Idiomatic)**:
```cpp
// NEW IDIOM: Generic cache setup using configuration
ENTO_BENCH_SETUP();
```

### 4. Harness Declaration Pattern
**OLD (Non-idiomatic)**:
```cpp
EntoBench::Harness<Problem, false, 10> harness(problem, "Benchmark Name", dataset_path, output_path);
```

**NEW (Idiomatic)**:
```cpp
// NEW IDIOM: Configuration-driven harness type
ENTO_BENCH_HARNESS_TYPE(Problem);
BenchHarness harness(problem, "Benchmark Name", dataset_path, output_path);
```

### 5. CMakeLists.txt Pattern
```cmake
# Config file approach with smart fallbacks
set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/benchmark/configs/perception_benchmarks.json")
if(EXISTS ${CONFIG_FILE})
  add_configured_benchmark_group_from_file("perception"
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
  add_preconfigured_benchmark_group("perception"
    TARGETS ${BENCH_TARGETS}
    REPS 30
    INNER_REPS 2
    VERBOSITY 1
    ENABLE_CACHES
  )
endif()
```

### 6. STM32G4 Exclusion Pattern
Large benchmarks are excluded from STM32G4 builds due to RAM/Flash limitations:
```cmake
# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-[algorithm]-large
    SOURCES bench_[algorithm]_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-[algorithm]-large)
endif()
```

## Algorithm-Specific Patterns

### FastBRIEF Benchmarks
```cpp
// Size-specific MaxFeats values (PRESERVE THESE!)
constexpr int MaxFeats = 5;    // tiny: 5 features, 31x31
constexpr int MaxFeats = 50;   // small: 50 features, 80x80  
constexpr int MaxFeatures = 100; // medium: 100 features, 160x160
constexpr int MaxFeatures = 250; // large: 250 features, 320x320

using Kernel  = FastBriefKernel<MaxFeats>; // or MaxFeatures
using PixT    = uint8_t;
constexpr int Rows = [size-specific]; // tiny=31, small=80, medium=160, large=320
constexpr int Cols = [size-specific];
using Problem = FeatureRecognitionProblem<Kernel, MaxFeats, Rows, Cols, PixT, true, true>;

// Dataset path pattern
const char* rel_path = "feat2d/fastbrief_[size]_books_data.txt";
```

### ORB Benchmarks
```cpp
// Size-specific MaxFeats values (PRESERVE THESE!)
constexpr int MaxFeats = 5;    // tiny: 5 features, 31x31
constexpr int MaxFeats = 50;   // small: 50 features, 80x80
constexpr int MaxFeats = 100;  // medium: 100 features, 160x160
constexpr int MaxFeats = 250;  // large: 250 features, 320x320

constexpr int Threshold = 20;
using Kernel  = ORBKernel<MaxFeats, Threshold>;
using PixT    = uint8_t;
constexpr int Rows = [size-specific]; // tiny=31, small=80, medium=160, large=320
constexpr int Cols = [size-specific];
using Problem = FeatureRecognitionProblem<Kernel, MaxFeats, Rows, Cols, PixT, true, true>;

// Dataset path pattern
const char* rel_path = "feat2d/test_orb_[size]_dataset.txt";
```

### LKOF (Lucas-Kanade Optical Flow) Benchmarks
```cpp
// Size-specific configurations (PRESERVE THESE!)
// Small: NUM_LEVELS=2, 80x80, decimal_bits=20, int64_t
// Medium: NUM_LEVELS=2, 160x160, decimal_bits=20, int64_t  
// Large: NUM_LEVELS=3, 320x320, decimal_bits=10, int32_t

const int decimal_bits = [size-specific]; // small/medium=20, large=10
using fp_t = FixedPoint<[size-specific], decimal_bits, [size-specific]>; // small/medium=int64_t, large=int32_t

constexpr size_t NUM_LEVELS = [size-specific]; // small/medium=2, large=3
constexpr size_t IMG_WIDTH  = [size-specific]; // small=80, medium=160, large=320
constexpr size_t IMG_HEIGHT = [size-specific];
constexpr size_t WIN_DIM    = 15;
constexpr size_t NumFeats   = 10;
using PixelT = uint8_t;
using CoordT = float;

using LK = LucasKanadeOFKernel<
  NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT,
  WIN_DIM, CoordT, PixelT, NumFeats>;

using Prob = SparseOpticalFlowProblem<
  LK, IMG_HEIGHT, IMG_WIDTH, NumFeats,
  Keypoint<CoordT>, PixelT>;

// Dataset path pattern
const char* rel_path = "feat2d/sparse_of_[size].txt";
```

## Size Variants
Each algorithm should support these size variants with their **ORIGINAL** tuned parameters:

### FastBRIEF
- **tiny**: MaxFeats=5, 31x31
- **small**: MaxFeats=50, 80x80  
- **medium**: MaxFeatures=100, 160x160
- **large**: MaxFeatures=250, 320x320

### ORB  
- **tiny**: MaxFeats=5, 31x31
- **small**: MaxFeats=50, 80x80
- **medium**: MaxFeats=100, 160x160
- **large**: MaxFeats=250, 320x320

### LKOF
- **small**: NUM_LEVELS=2, 80x80, decimal_bits=20, int64_t
- **medium**: NUM_LEVELS=2, 160x160, decimal_bits=20, int64_t
- **large**: NUM_LEVELS=3, 320x320, decimal_bits=10, int32_t

## ⚠️ CRITICAL: DO NOT CHANGE ALGORITHM PARAMETERS

**NEVER** modify these carefully tuned values:
- `MaxFeats` / `MaxFeatures` counts
- Image dimensions (`Rows`, `Cols`)  
- `NUM_LEVELS`, `WIN_DIM`, `decimal_bits`
- Fixed point types (`int32_t` vs `int64_t`)
- Threshold values

**ONLY** change the idiomatic patterns:
- Cache setup: `enable_instruction_cache()` → `ENTO_BENCH_SETUP()`
- Harness: `EntoBench::Harness<Problem, false, 10>` → `ENTO_BENCH_HARNESS_TYPE(Problem)` and `BenchHarness`
- Add configuration includes and print statements

## Configuration File Structure
Create `benchmark/configs/perception_benchmarks.json`:
```json
{
  "perception": {
    "reps": 30,
    "inner_reps": 2,
    "verbosity": 1,
    "enable_caches": true,
    "enable_vectorization": true
  }
}
```

### Configuration Options
- **`reps`**: Number of benchmark repetitions
- **`inner_reps`**: Number of inner loop repetitions per benchmark
- **`verbosity`**: Debug output level (0=quiet, 1=normal, 2=verbose)
- **`enable_caches`**: Enable/disable CPU caches during benchmarking
- **`enable_vectorization`**: Enable/disable ARM SIMD optimizations (e.g., USADA8 for SAD computation in block-based optical flow)

### Vectorization Control
The `enable_vectorization` option controls ARM SIMD optimizations:
- **`true`** (default): Uses ARM assembly with USADA8 instructions for SAD computation
- **`false`**: Uses portable C++ implementation

This is particularly important for block-based optical flow benchmarks that use Sum of Absolute Differences (SAD) computation. When enabled on ARM targets, it uses the USADA8 instruction for 4x faster SAD computation.

## Benefits of These Idioms

### ✅ **MCU Portability**
- `ENTO_BENCH_SETUP()` handles cache setup across different MCU families
- Automatic detection and configuration for STM32G4, H7, F7, U5 series

### ✅ **Configuration Flexibility**
- Runtime parameters configurable via JSON
- Easy to modify benchmark parameters without recompiling
- Fallback ensures builds work without config files

### ✅ **Maintainability**
- Consistent pattern across all perception benchmarks
- Cleaner, more organized code structure
- Easy to add new benchmarks following the same pattern

### ✅ **Performance**
- Preserves all existing compiler optimizations
- Only changes runtime configuration, not compilation

## Migration Checklist

For each perception benchmark file:
- [ ] Add `#include <ento-bench/bench_config.h>`
- [ ] Replace manual cache setup with `ENTO_BENCH_SETUP()`
- [ ] Add `ENTO_BENCH_PRINT_CONFIG()` after setup
- [ ] Replace hardcoded harness template with `ENTO_BENCH_HARNESS_TYPE(Problem)` and `BenchHarness`
- [ ] Update CMakeLists.txt to use config file approach
- [ ] Test the migrated benchmark

## Priority Migration Order
1. **FastBRIEF** benchmarks (all sizes)
2. **ORB** benchmarks (all sizes)  
3. **LKOF** benchmarks (all sizes)
4. Other perception algorithms as needed 