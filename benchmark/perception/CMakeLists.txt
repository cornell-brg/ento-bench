# =============================================================================
# Perception Benchmarks
# =============================================================================
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if (STM32_BUILD)
set(MODULES
  bench
  mcu-util
  Eigen
  ento-pose
  ento_util
  CMSIS::STM32::${STM_PRODUCT}
)
set(BENCH_TARGETS) # List of targets


# =============================================================================
# FAST+BRIEF Tiny Image Benchmark
# =============================================================================

add_benchmark(bench-fastbrief-tiny
  SOURCES bench_fastbrief_tiny.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-tiny)

# =============================================================================
# FAST+BRIEF Small Image Benchmark
# =============================================================================

add_benchmark(bench-fastbrief-small
  SOURCES bench_fastbrief_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-small)

# =============================================================================
# FAST+BRIEF Medium Image Benchmark
# =============================================================================

add_benchmark(bench-fastbrief-medium
  SOURCES bench_fastbrief_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-medium)

# =============================================================================
# FAST+BRIEF Large Image Benchmark
# =============================================================================

add_benchmark(bench-fastbrief-large
  SOURCES bench_fastbrief_large.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-large)

# =============================================================================
# FAST Tiny Image Benchmark
# =============================================================================

add_benchmark(bench-fast-tiny
  SOURCES bench_fast_tiny.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fast-tiny)


# =============================================================================
# Fast Small Image Benchmark
# =============================================================================

add_benchmark(bench-fast-small
  SOURCES bench_fast_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fast-small)

# =============================================================================
# Fast Medium Image Benchmark 
# =============================================================================

add_benchmark(bench-fast-medium
  SOURCES bench_fast_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fast-medium)


# =============================================================================
# Fast Large Image Benchmark 
# =============================================================================

add_benchmark(bench-fast-large
  SOURCES bench_fast_large.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fast-large)

# =============================================================================
# Brief Tiny Image Benchmark
# =============================================================================

add_benchmark(bench-brief-tiny
  SOURCES bench_brief_tiny.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-brief-tiny)


# =============================================================================
# Brief Small Image Benchmark
# =============================================================================

add_benchmark(bench-brief-small
  SOURCES bench_brief_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-brief-small)

# =============================================================================
# Brief Medium Image Benchmark
# =============================================================================

add_benchmark(bench-brief-medium
  SOURCES bench_brief_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-brief-medium)

# =============================================================================
# Brief Large Image Benchmark
# =============================================================================

add_benchmark(bench-brief-large
  SOURCES bench_brief_large.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-brief-large)


# =============================================================================
# ORB Tiny Image Benchmark
# =============================================================================

add_benchmark(bench-orb-tiny
  SOURCES bench_orb_tiny.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-tiny)

# =============================================================================
# ORB Small Image Benchmark
# =============================================================================

add_benchmark(bench-orb-small
  SOURCES bench_orb_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-small)

# =============================================================================
# ORB Medium Image Benchmark
# =============================================================================

add_benchmark(bench-orb-medium
  SOURCES bench_fastbrief_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-medium)

# =============================================================================
# ORB Large Image Benchmark
# =============================================================================

add_benchmark(bench-orb-large
  SOURCES bench_orb_large.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-large)


# =============================================================================
# Add stm32 targets
# =============================================================================
message(STATUS "Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

endif()

