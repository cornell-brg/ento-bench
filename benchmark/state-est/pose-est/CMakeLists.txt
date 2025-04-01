# =============================================================================
# Geometric Pose Estimation Benchmarks
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
# P3P
# =============================================================================

add_benchmark(bench-p3p
  SOURCES bench_p3p.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-p3p)

# =============================================================================
# UP2P
# =============================================================================

add_benchmark(bench-up2p
  SOURCES bench_up2p.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-up2p)

# =============================================================================
# Homography
# =============================================================================

add_benchmark(bench-homography
  SOURCES bench_homography.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-homography)

# =============================================================================
# DLT
# =============================================================================

add_benchmark(bench-dlt
  SOURCES bench_dlt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt)

# =============================================================================
# 8pt Algorithm
# =============================================================================

add_benchmark(bench-8pt
  SOURCES bench_8pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt)

# =============================================================================
# 5pt Algorithm
# =============================================================================

add_benchmark(bench-5pt
  SOURCES bench_5pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-5pt)

# =============================================================================
# Upright Planar Two Point
# =============================================================================

add_benchmark(bench-up2pt
  SOURCES bench_up2pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-up2pt)

# =============================================================================
# Upright Three Point
# =============================================================================

add_benchmark(bench-u3pt
  SOURCES bench_u3pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-u3pt)


# =============================================================================
# =============================================================================
# Add stm32 targets
# =============================================================================
message(STATUS "Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

endif()
