# =============================================================================
# Example: New Benchmark Idioms - CMake Configuration
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

add_benchmark(bench-example-1
  SOURCES example_benchmark_1.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-example-1)

add_benchmark(bench-example-2
  SOURCES example_benchmark_2.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-example-2)

add_benchmark(bench-example-3
  SOURCES example_benchmark_3.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-example-3)

add_benchmark(bench-example-4
  SOURCES example_benchmark_4.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-example-4)

# =============================================================================
# Method 1: Individual target configuration
# =============================================================================

configure_benchmark_target(bench-example-1
  REPS 10
  VERBOSITY 2
  ENABLE_CACHES
)

# =============================================================================
# Method 2: Integrated configuration during creation
# =============================================================================

add_configured_benchmark(bench-configured-example
  SOURCES example_benchmark_integrated.cc
  LIBRARIES ${MODULES}
  REPS 15
  INNER_REPS 3
  VERBOSITY 1
  DO_WARMUP
  ENABLE_CACHES
)

# =============================================================================
# Method 3: Group configuration (traditional)
# =============================================================================

add_configured_benchmark_group_target("manual-config-group"
  bench-example-1
  bench-example-2
  REPS 20
  VERBOSITY 1
  ENABLE_CACHES
)

# =============================================================================
# Method 4: NEW - Config file approach (simple)
# =============================================================================

# Load configuration from JSON file and apply to targets
configure_benchmark_group_from_file("performance-group" 
  "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks.json"
  bench-example-2
  bench-example-3
)

# =============================================================================
# Method 5: NEW - Config file with fallback
# =============================================================================

# Try config file first, fall back to parameters if file not found or group missing
add_configured_benchmark_group_from_file("debug-group"
  CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks.json"
  TARGETS bench-example-3 bench-example-4
  # Fallback configuration (used if config file fails)
  REPS 8
  VERBOSITY 2
  DO_WARMUP
)

# =============================================================================
# Method 6: NEW - Multiple config file groups
# =============================================================================

# Quick testing configuration
add_configured_benchmark_group_from_file("quick-test"
  CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks.json"
  TARGETS bench-example-1 bench-example-2
)

# Thorough testing configuration
add_configured_benchmark_group_from_file("thorough-test"
  CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks.json"
  TARGETS bench-example-3 bench-example-4 bench-configured-example
)

# Cache comparison groups
add_configured_benchmark_group_from_file("cache-comparison"
  CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks.json"
  TARGETS bench-example-1 bench-example-3
)

# =============================================================================
# Method 7: NEW - Conditional config file usage
# =============================================================================

# Use different config files based on build type or MCU family
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CONFIG_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/debug_benchmarks.json")
else()
  set(CONFIG_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks.json")
endif()

add_configured_benchmark_group_from_file("adaptive-config"
  CONFIG_FILE "${CONFIG_FILE_PATH}"
  TARGETS bench-example-2 bench-example-4
  # Fallback if config file not found
  REPS 10
  VERBOSITY 1
  ENABLE_CACHES
)

# =============================================================================
# Method 8: NEW - MCU-specific config file groups
# =============================================================================

if(STM32_FAMILY STREQUAL "STM32H7")
    add_configured_benchmark_group_from_file("mcu-optimized"
      CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks.json"
      TARGETS bench-example-1 bench-example-2 bench-example-3 bench-example-4
    )
elseif(STM32_FAMILY STREQUAL "STM32G4")
    # For G4, use a more conservative config or fallback
    add_configured_benchmark_group_from_file("mcu-optimized"
      CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks.json"
      TARGETS bench-example-1 bench-example-2  # Fewer targets
      # Fallback config for resource-constrained G4
      REPS 10
      INNER_REPS 1
      VERBOSITY 1
      ENABLE_CACHES
    )
else()
    # Default MCU configuration
    add_configured_benchmark_group_from_file("mcu-optimized"
      CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks.json"
      TARGETS bench-example-1 bench-example-3
    )
endif()

# =============================================================================
# Add STM32 targets
# =============================================================================
list(APPEND BENCH_TARGETS bench-configured-example)
message(STATUS "New Idioms CMake Example targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

# =============================================================================
# Summary of created groups
# =============================================================================
message(STATUS "Created benchmark groups with config file support:")
message(STATUS "  bench-manual-config-group     - Traditional CMake parameters")
message(STATUS "  bench-performance-group       - From benchmarks.json")
message(STATUS "  bench-debug-group             - From benchmarks.json (with fallback)")
message(STATUS "  bench-quick-test              - From benchmarks.json") 
message(STATUS "  bench-thorough-test           - From benchmarks.json")
message(STATUS "  bench-cache-comparison        - From benchmarks.json")
message(STATUS "  bench-adaptive-config         - Conditional config file")
message(STATUS "  bench-mcu-optimized           - MCU-specific config")

endif() # STM32_BUILD 