# =============================================================================
# Attitude Estimation Benchmarks
# =============================================================================
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)
if (STM32_BUILD)
set(MODULES
  bench
  mcu-util
  Eigen
  ento-state-est
  ento_math
  ento_util
  CMSIS::STM32::${STM_PRODUCT}
)
set(BENCH_TARGETS) # List of targets
# =============================================================================
# Madgwick Filter
# =============================================================================
add_benchmark(bench-madgwick
  SOURCES bench_madgwick.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick)
# =============================================================================
# Mahoney Filter
# =============================================================================
add_benchmark(bench-mahoney
  SOURCES bench_mahoney.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahoney)

# =============================================================================
# Fourati Filter
# =============================================================================
add_benchmark(bench-fourati
  SOURCES bench_fourati.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fourati)
# =============================================================================
# =============================================================================
# Add stm32 targets
# =============================================================================
message(STATUS "Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")
endif()