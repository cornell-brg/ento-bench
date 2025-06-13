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
# Gold Standard Benchmarks (H&Z Reference Implementations)
# =============================================================================

add_benchmark(bench-gold-standard-abs-float
  SOURCES bench_gold_standard_abs_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-gold-standard-abs-float)

add_benchmark(bench-gold-standard-abs-double
  SOURCES bench_gold_standard_abs_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-gold-standard-abs-double)

add_benchmark(bench-gold-standard-rel-float
  SOURCES bench_gold_standard_rel_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-gold-standard-rel-float)

add_benchmark(bench-gold-standard-rel-double
  SOURCES bench_gold_standard_rel_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-gold-standard-rel-double)

# =============================================================================
# Benchmark Group Definitions for Runners
# =============================================================================

# Define benchmark groups
set(ABSOLUTE_POSE_TARGETS 
  bench-p3p-float bench-p3p-double
  bench-up2p-float bench-up2p-double
  bench-dlt-float bench-dlt-double
)

set(RELATIVE_POSE_TARGETS
  bench-8pt-float bench-8pt-double
  bench-5pt-float bench-5pt-double
  bench-upright-3pt-float bench-upright-3pt-double
)

set(DLT_SCALING_TARGETS
  bench-dlt8-float
  bench-dlt16-float bench-dlt16-double
  bench-dlt32-float bench-dlt32-double
  #bench-dlt64-float bench-dlt64-double # 64 points too large for current deserialize ExperimentIO
)

set(8PT_SCALING_TARGETS
  bench-8pt16-float bench-8pt16-double
  bench-8pt32-float bench-8pt32-double
  #bench-8pt64-float bench-8pt64-double # 64 points too large for current deserialize ExperimentIO
)

set(UPRIGHT_PLANAR_TARGETS
  bench-upright-planar-3pt-float bench-upright-planar-3pt-double
  bench-upright-planar-2pt-float bench-upright-planar-2pt-double
)

set(UPRIGHT_PLANAR_SCALING_TARGETS
  bench-upright-planar-3pt8-float bench-upright-planar-3pt8-double
  bench-upright-planar-3pt16-float bench-upright-planar-3pt16-double
  #bench-upright-planar-3pt32-float bench-upright-planar-3pt32-double # 32 points too large for current deserialize ExperimentIO
)

set(GOLD_STANDARD_TARGETS
  bench-gold-standard-abs-float bench-gold-standard-abs-double
  bench-gold-standard-rel-float bench-gold-standard-rel-double
)

set(ALL_POSE_TARGETS ${ABSOLUTE_POSE_TARGETS} ${RELATIVE_POSE_TARGETS})
set(QUICK_TEST_TARGETS bench-p3p-float bench-8pt-float bench-dlt-float)
set(PERFORMANCE_TARGETS ${ALL_POSE_TARGETS} ${DLT_SCALING_TARGETS} ${8PT_SCALING_TARGETS})

# =============================================================================
# Add Benchmark Group Runners with Timeout Support
# =============================================================================

# Create the benchmark runner script
create_benchmark_runner_script()

# Quick test group (fast benchmarks for testing)
add_benchmark_group_runner(quick-test
  TARGETS ${QUICK_TEST_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

# Absolute pose benchmarks
add_benchmark_group_runner(absolute-pose
  TARGETS ${ABSOLUTE_POSE_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

# Relative pose benchmarks
add_benchmark_group_runner(relative-pose
  TARGETS ${RELATIVE_POSE_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

# DLT scaling benchmarks (longer timeout for larger problems)
add_benchmark_group_runner(dlt-scaling
  TARGETS ${DLT_SCALING_TARGETS}
  TIMEOUT 90
  OUTPUT_TIMEOUT 15
)

# 8pt scaling benchmarks
add_benchmark_group_runner(8pt-scaling
  TARGETS ${8PT_SCALING_TARGETS}
  TIMEOUT 90
  OUTPUT_TIMEOUT 15
)

# Upright planar benchmarks
add_benchmark_group_runner(upright-planar
  TARGETS ${UPRIGHT_PLANAR_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

# Upright planar scaling benchmarks
add_benchmark_group_runner(upright-planar-scaling
  TARGETS ${UPRIGHT_PLANAR_SCALING_TARGETS}
  TIMEOUT 90
  OUTPUT_TIMEOUT 15
)

# Gold standard benchmarks (longer timeout for bundle adjustment)
add_benchmark_group_runner(gold-standard
  TARGETS ${GOLD_STANDARD_TARGETS}
  TIMEOUT 120
  OUTPUT_TIMEOUT 20
)

# All basic pose estimation benchmarks
add_benchmark_group_runner(all-pose
  TARGETS ${ALL_POSE_TARGETS}
  TIMEOUT 90
  OUTPUT_TIMEOUT 15
)

# Comprehensive performance evaluation
add_benchmark_group_runner(performance
  TARGETS ${PERFORMANCE_TARGETS}
  TIMEOUT 120
  OUTPUT_TIMEOUT 20
)

# =============================================================================
# Convenience Targets for Common Use Cases
# =============================================================================

# Add convenience aliases that match common naming patterns
add_custom_target(bench-pose-quick
  DEPENDS bench-quick-test-run
  COMMENT "Alias for bench-quick-test-run"
)

add_custom_target(bench-pose-absolute
  DEPENDS bench-absolute-pose-run
  COMMENT "Alias for bench-absolute-pose-run"
)

add_custom_target(bench-pose-relative
  DEPENDS bench-relative-pose-run
  COMMENT "Alias for bench-relative-pose-run"
)

add_custom_target(bench-pose-gold-standard
  DEPENDS bench-gold-standard-run
  COMMENT "Alias for bench-gold-standard-run"
)

add_custom_target(bench-pose-all
  DEPENDS bench-all-pose-run
  COMMENT "Alias for bench-all-pose-run"
)

add_custom_target(bench-pose-performance
  DEPENDS bench-performance-run
  COMMENT "Alias for bench-performance-run"
)

# Help target to list available benchmark groups
add_custom_target(bench-pose-help
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "EntoBench Pose Estimation Benchmark Groups:"
  COMMAND ${CMAKE_COMMAND} -E echo "==========================================="
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Quick Testing:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-quick-test-run        Fast subset for testing (P3P, 8pt, DLT float)"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-pose-quick            Alias for bench-quick-test-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Basic Pose Estimation:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-absolute-pose-run     P3P, UP2P, DLT benchmarks"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-relative-pose-run     8pt, 5pt, upright 3pt benchmarks"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-pose-absolute         Alias for bench-absolute-pose-run"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-pose-relative         Alias for bench-relative-pose-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Scaling Studies:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-dlt-scaling-run       DLT with different point counts (8, 16, 32, 64)"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-8pt-scaling-run       8pt with different point counts (16, 32, 64)"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-upright-planar-run    Upright planar 2pt and 3pt benchmarks"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-upright-planar-scaling-run  Upright planar with different point counts"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Reference Implementations:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-gold-standard-run     H&Z gold standard absolute and relative pose"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-pose-gold-standard    Alias for bench-gold-standard-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Comprehensive Testing:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-all-pose-run          All basic pose estimation benchmarks"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-performance-run       Comprehensive performance evaluation"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-pose-all              Alias for bench-all-pose-run"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-pose-performance      Alias for bench-performance-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Timeout Configuration:"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Basic benchmarks: 60s timeout, 10s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Scaling benchmarks: 90s timeout, 15s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Gold standard: 120s timeout, 20s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Performance: 120s timeout, 20s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Results are saved to benchmark_results_<group-name>/ directory"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMENT "Show available pose estimation benchmark groups"
)

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
