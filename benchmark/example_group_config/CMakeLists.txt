# =============================================================================
# Example: Benchmark Group Configuration
# =============================================================================
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if (STM32_BUILD)
set(MODULES
  bench
  ento-mcu-semihosted
  Eigen
  ento_util
  CMSIS::STM32::${STM_PRODUCT}
)
set(BENCH_TARGETS) # List of targets

# =============================================================================
# Create individual benchmark targets first
# =============================================================================

add_benchmark(bench-group-test-1
  SOURCES group_test_1.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-group-test-1)

add_benchmark(bench-group-test-2
  SOURCES group_test_2.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-group-test-2)

add_benchmark(bench-group-test-3
  SOURCES group_test_3.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-group-test-3)

add_benchmark(bench-group-test-4
  SOURCES group_test_4.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-group-test-4)

# =============================================================================
# Method 1: Basic group (no configuration)
# =============================================================================

add_benchmark_group_target("basic-group" 
  bench-group-test-1 
  bench-group-test-2
)

# =============================================================================
# Method 2: Configured group - applies configuration to all targets in group
# =============================================================================

add_configured_benchmark_group_target("performance-group"
  bench-group-test-1
  bench-group-test-2
  bench-group-test-3
  REPS 20              # Apply to all targets in group
  INNER_REPS 3         # Apply to all targets in group
  VERBOSITY 1          # Apply to all targets in group
  ENABLE_CACHES        # Apply to all targets in group
)

# =============================================================================
# Method 3: Pre-configured group using TARGETS syntax
# =============================================================================

add_preconfigured_benchmark_group("debug-group"
  TARGETS bench-group-test-2 bench-group-test-3 bench-group-test-4
  REPS 3               # Fewer reps for debug
  VERBOSITY 2          # More verbose for debugging
  DO_WARMUP            # Enable warmup for debug analysis
  ENABLE_CACHES        # Caches enabled
)

# =============================================================================
# Method 4: Different configurations for different groups
# =============================================================================

# High-performance group with many repetitions
add_configured_benchmark_group_target("high-perf"
  bench-group-test-1
  bench-group-test-4
  REPS 100             # Many reps for statistical significance
  INNER_REPS 1         # Single inner loop for clean measurements
  VERBOSITY 1          # Minimal output
  ENABLE_CACHES        # Max performance
)

# Cache comparison group - same targets, different cache settings
add_preconfigured_benchmark_group("cache-disabled"
  TARGETS bench-group-test-1 bench-group-test-2
  REPS 10
  VERBOSITY 2
  # ENABLE_CACHES omitted = disabled
)

add_preconfigured_benchmark_group("cache-enabled"
  TARGETS bench-group-test-1 bench-group-test-2
  REPS 10
  VERBOSITY 2
  ENABLE_CACHES        # Explicitly enabled
)

# =============================================================================
# Method 5: Mixed approach - some targets with individual config
# =============================================================================

# Configure one target individually
configure_benchmark_target(bench-group-test-4
  REPS 50
  VERBOSITY 2
  DO_WARMUP
)

# Then include it in a group (keeping its individual config)
add_benchmark_group_target("mixed-config"
  bench-group-test-3   # Uses default config
  bench-group-test-4   # Uses individual config from above
)

# =============================================================================
# Method 6: MCU-specific group configurations
# =============================================================================

if(STM32_FAMILY STREQUAL "STM32H7")
    # H7 has powerful caches and more resources
    add_configured_benchmark_group_target("mcu-optimized"
      bench-group-test-1
      bench-group-test-2
      bench-group-test-3
      REPS 200           # H7 can handle more reps
      INNER_REPS 10      # More inner reps
      VERBOSITY 1
      ENABLE_CACHES      # H7 benefits greatly from caches
    )
elseif(STM32_FAMILY STREQUAL "STM32G4")
    # G4 is more resource constrained
    add_configured_benchmark_group_target("mcu-optimized"
      bench-group-test-1
      bench-group-test-2
      REPS 20            # Fewer targets and reps for G4
      INNER_REPS 2
      VERBOSITY 1
      ENABLE_CACHES      # Still beneficial
    )
else()
    # Default configuration for other MCUs
    add_configured_benchmark_group_target("mcu-optimized"
      bench-group-test-1
      bench-group-test-2
      bench-group-test-3
      REPS 50
      INNER_REPS 5
      VERBOSITY 1
      ENABLE_CACHES
    )
endif()

# =============================================================================
# Method 7: Global configuration variables
# =============================================================================

# Define common configurations
set(FAST_CONFIG
  REPS 5
  VERBOSITY 1
  ENABLE_CACHES
)

set(THOROUGH_CONFIG
  REPS 30
  INNER_REPS 3
  VERBOSITY 2
  DO_WARMUP
  ENABLE_CACHES
)

# Apply global configs to different groups
add_configured_benchmark_group_target("fast-test"
  bench-group-test-1
  bench-group-test-2
  ${FAST_CONFIG}       # Expand the configuration variables
)

add_configured_benchmark_group_target("thorough-test"
  bench-group-test-3
  bench-group-test-4
  ${THOROUGH_CONFIG}   # Expand the configuration variables
)

# =============================================================================
# Add STM32 targets
# =============================================================================
message(STATUS "Group Config Example Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

# =============================================================================
# Summary of created groups
# =============================================================================
message(STATUS "Created benchmark groups:")
message(STATUS "  bench-basic-group      - No configuration")
message(STATUS "  bench-performance-group - REPS=20, CACHES=ON")
message(STATUS "  bench-debug-group      - REPS=3, VERBOSITY=2, WARMUP=ON")
message(STATUS "  bench-high-perf        - REPS=100, minimal output")
message(STATUS "  bench-cache-disabled   - REPS=10, CACHES=OFF")
message(STATUS "  bench-cache-enabled    - REPS=10, CACHES=ON")
message(STATUS "  bench-mixed-config     - Mixed individual/group config")
message(STATUS "  bench-mcu-optimized    - MCU-specific configuration")
message(STATUS "  bench-fast-test        - Quick testing config")
message(STATUS "  bench-thorough-test    - Comprehensive testing config")

endif() # STM32_BUILD 