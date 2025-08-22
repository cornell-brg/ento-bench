# =============================================================================
# Example Benchmarks - Demonstration of EntoBench Idioms
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
# Create Example Benchmark Targets
# =============================================================================

# Vector-Vector Add Example
add_benchmark(benchmark-example
  SOURCES benchmark_example.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS benchmark-example)

# =============================================================================
# Configure Example Benchmarks
# =============================================================================

# Use simple direct configuration for examples
add_preconfigured_benchmark_group("example"
  TARGETS ${BENCH_TARGETS}
  REPS 10
  INNER_REPS 100  # More inner reps to see vector add performance clearly
  VERBOSITY 2     # More verbose for educational purposes
  ENABLE_CACHES
  MAX_PROBLEMS 1  # Simple example, just one "problem"
)

# =============================================================================
# Add STM32 targets for flashing/debugging
# =============================================================================
message(VERBOSE "Example Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

# =============================================================================
# Help target for examples
# =============================================================================
add_custom_target(bench-example-help
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "EntoBench Example Benchmarks:"
  COMMAND ${CMAKE_COMMAND} -E echo "============================"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Available Examples:"
  COMMAND ${CMAKE_COMMAND} -E echo "  make stm32-flash-benchmark-example-semihosted"
  COMMAND ${CMAKE_COMMAND} -E echo "    - Vector-vector add example (1000 float elements)"
  COMMAND ${CMAKE_COMMAND} -E echo "    - Demonstrates all EntoBench idioms"
  COMMAND ${CMAKE_COMMAND} -E echo "    - Self-contained kernel definition"
  COMMAND ${CMAKE_COMMAND} -E echo "    - Static test data with volatile protection"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "This example is perfect for:"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Learning EntoBench patterns"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Testing STM32 setup"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Verifying build configuration"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Understanding benchmark structure"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMENT "Show available example benchmarks"
)

endif() # STM32_BUILD 