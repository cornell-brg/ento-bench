# =============================================================================
# Geometric Pose Estimation Benchmarks - New Idioms
# =============================================================================
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if (STM32_BUILD)
set(MODULES
  bench
  ento-mcu-semihosted
  Eigen
  ento-pose
  ento_util
  CMSIS::STM32::${STM_PRODUCT}
)
set(BENCH_TARGETS) # List of targets

# =============================================================================
# Create all benchmark targets first
# =============================================================================

# =============================================================================
# Absolute Pose Solvers - P3P
# =============================================================================

add_benchmark(bench-p3p-float
  SOURCES bench_p3p_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-p3p-float)

add_benchmark(bench-p3p-double
  SOURCES bench_p3p_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-p3p-double)

# =============================================================================
# Absolute Pose Solvers - UP2P
# =============================================================================

add_benchmark(bench-up2p-float
  SOURCES bench_up2p_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-up2p-float)

add_benchmark(bench-up2p-double
  SOURCES bench_up2p_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-up2p-double)

# =============================================================================
# Absolute Pose Solvers - DLT (Original + Scaling)
# =============================================================================

add_benchmark(bench-dlt-float
  SOURCES bench_dlt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt-float)

add_benchmark(bench-dlt-double
  SOURCES bench_dlt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt-double)

# DLT Scaling Benchmarks
add_benchmark(bench-dlt8-float
  SOURCES bench_dlt8_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt8-float)

add_benchmark(bench-dlt16-float
  SOURCES bench_dlt16_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt16-float)

add_benchmark(bench-dlt16-double
  SOURCES bench_dlt16_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt16-double)

add_benchmark(bench-dlt32-float
  SOURCES bench_dlt32_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt32-float)

add_benchmark(bench-dlt32-double
  SOURCES bench_dlt32_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt32-double)

add_benchmark(bench-dlt64-float
  SOURCES bench_dlt64_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt64-float)

add_benchmark(bench-dlt64-double
  SOURCES bench_dlt64_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt64-double)

# =============================================================================
# Relative Pose Solvers - 8pt (Original + Scaling)
# =============================================================================

add_benchmark(bench-8pt-float
  SOURCES bench_8pt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt-float)

add_benchmark(bench-8pt-double
  SOURCES bench_8pt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt-double)

# 8pt Scaling Benchmarks
add_benchmark(bench-8pt16-float
  SOURCES bench_8pt16_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt16-float)

add_benchmark(bench-8pt16-double
  SOURCES bench_8pt16_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt16-double)

add_benchmark(bench-8pt32-float
  SOURCES bench_8pt32_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt32-float)

add_benchmark(bench-8pt32-double
  SOURCES bench_8pt32_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt32-double)

add_benchmark(bench-8pt64-float
  SOURCES bench_8pt64_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt64-float)

add_benchmark(bench-8pt64-double
  SOURCES bench_8pt64_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt64-double)

# =============================================================================
# Relative Pose Solvers - 5pt
# =============================================================================

add_benchmark(bench-5pt-float
  SOURCES bench_5pt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-5pt-float)

add_benchmark(bench-5pt-double
  SOURCES bench_5pt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-5pt-double)

# =============================================================================
# Relative Pose Solvers - Upright 3pt
# =============================================================================

add_benchmark(bench-upright-3pt-float
  SOURCES bench_upright_3pt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-3pt-float)

add_benchmark(bench-upright-3pt-double
  SOURCES bench_upright_3pt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-3pt-double)

# =============================================================================
# Relative Pose Solvers - Upright Planar 3pt (Original + Scaling)
# =============================================================================

add_benchmark(bench-upright-planar-3pt-float
  SOURCES bench_upright_planar_3pt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-3pt-float)

add_benchmark(bench-upright-planar-3pt-double
  SOURCES bench_upright_planar_3pt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-3pt-double)

# Upright Planar 3pt Scaling Benchmarks
add_benchmark(bench-upright-planar-3pt8-float
  SOURCES bench_upright_planar_3pt8_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-3pt8-float)

add_benchmark(bench-upright-planar-3pt8-double
  SOURCES bench_upright_planar_3pt8_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-3pt8-double)

add_benchmark(bench-upright-planar-3pt16-float
  SOURCES bench_upright_planar_3pt16_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-3pt16-float)

add_benchmark(bench-upright-planar-3pt16-double
  SOURCES bench_upright_planar_3pt16_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-3pt16-double)

add_benchmark(bench-upright-planar-3pt32-float
  SOURCES bench_upright_planar_3pt32_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-3pt32-float)

add_benchmark(bench-upright-planar-3pt32-double
  SOURCES bench_upright_planar_3pt32_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-3pt32-double)

# =============================================================================
# Relative Pose Solvers - Upright Planar 2pt
# =============================================================================

add_benchmark(bench-upright-planar-2pt-float
  SOURCES bench_upright_planar_2pt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-2pt-float)

add_benchmark(bench-upright-planar-2pt-double
  SOURCES bench_upright_planar_2pt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-2pt-double)

# =============================================================================
# Apply Configuration Using Config File with Target-Specific Support
# =============================================================================

# Method 1: Enhanced config file approach with target-specific support
set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/benchmark/configs/estimation_benchmarks.json")
if(EXISTS ${CONFIG_FILE})
  message(STATUS "Using enhanced config file with target-specific support: ${CONFIG_FILE}")
  add_configured_benchmark_group_with_target_configs("pose-est"
    CONFIG_FILE ${CONFIG_FILE}
    TARGETS ${BENCH_TARGETS}
  )
else()
  # Method 2: Direct configuration (fallback)
  message(STATUS "Config file not found, using direct configuration")
  add_preconfigured_benchmark_group("pose-est"
    TARGETS ${BENCH_TARGETS}
    REPS 30
    INNER_REPS 2
    VERBOSITY 1
    ENABLE_CACHES
  )
endif()

# =============================================================================
# Add STM32 targets for flashing/debugging
# =============================================================================
message(STATUS "Pose Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

endif()
