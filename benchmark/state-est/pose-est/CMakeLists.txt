# =============================================================================
# Geometric Pose Estimation Benchmarks
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

add_benchmark(bench-p3p
  SOURCES bench_p3p.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-p3p)

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

add_benchmark(bench-up2p
  SOURCES bench_up2p.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-up2p)

# =============================================================================
# Absolute Pose Solvers - DLT
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

add_benchmark(bench-dlt
  SOURCES bench_dlt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-dlt)

# =============================================================================
# Relative Pose Solvers - 8pt
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

add_benchmark(bench-8pt
  SOURCES bench_8pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-8pt)

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

add_benchmark(bench-5pt
  SOURCES bench_5pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-5pt)

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

add_benchmark(bench-u3pt
  SOURCES bench_u3pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-u3pt)

# =============================================================================
# Relative Pose Solvers - Upright Planar 3pt
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

add_benchmark(bench-upright-planar-3pt
  SOURCES bench_upright_planar_3pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-3pt)

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

add_benchmark(bench-upright-planar-2pt
  SOURCES bench_upright_planar_2pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-upright-planar-2pt)

# =============================================================================
# Other existing benchmarks (Homography, Up2pt, Gold Standard)
# =============================================================================

add_benchmark(bench-homography
  SOURCES bench_homography.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-homography)

add_benchmark(bench-homography-rel
  SOURCES bench_homography_rel.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-homography-rel)

add_benchmark(bench-up2pt
  SOURCES bench_up2pt.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-up2pt)

add_benchmark(bench-gold-standard-abs
  SOURCES bench_gold_standard_abs.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-gold-standard-abs)

add_benchmark(bench-gold-standard-rel
  SOURCES bench_gold_standard_rel.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-gold-standard-rel)

# =============================================================================
# Add stm32 targets
# =============================================================================
message(STATUS "Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

# =============================================================================
# Create benchmark group target: bench-pose-est
# =============================================================================
add_benchmark_group_target("pose-est" ${BENCH_TARGETS})

endif()
