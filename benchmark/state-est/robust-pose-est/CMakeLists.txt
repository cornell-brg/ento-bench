cmake_minimum_required(VERSION 3.16)

# =============================================================================
# Robust Pose Estimation Benchmarks - Modern EntoBench Idioms
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
# RANSAC Relative Pose Benchmarks - Float Precision
# =============================================================================

# RANSAC 5pt Benchmark - Float
add_benchmark(bench-ransac-5pt-float
  SOURCES bench_ransac_5pt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-5pt-float)

# RANSAC Upright 3pt Benchmark - Float
add_benchmark(bench-ransac-upright-3pt-float
  SOURCES bench_ransac_upright_3pt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-upright-3pt-float)

# RANSAC Upright Planar 2pt Benchmark - Float
add_benchmark(bench-ransac-upright-planar-2pt-float
  SOURCES bench_ransac_upright_planar_2pt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-upright-planar-2pt-float)

# RANSAC Upright Planar 3pt Benchmark - Float
add_benchmark(bench-ransac-upright-planar-3pt-float
  SOURCES bench_ransac_upright_planar_3pt_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-upright-planar-3pt-float)

# RANSAC Upright Planar 3pt Hybrid Benchmark - Float
add_benchmark(bench-ransac-upright-planar-3pt-hybrid-float
  SOURCES bench_ransac_upright_planar_3pt_hybrid_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-upright-planar-3pt-hybrid-float)

# =============================================================================
# RANSAC Relative Pose Benchmarks - Double Precision
# =============================================================================

# RANSAC 5pt Benchmark - Double
add_benchmark(bench-ransac-5pt-double
  SOURCES bench_ransac_5pt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-5pt-double)

# RANSAC Upright 3pt Benchmark - Double
add_benchmark(bench-ransac-upright-3pt-double
  SOURCES bench_ransac_upright_3pt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-upright-3pt-double)

# RANSAC Upright Planar 2pt Benchmark - Double
add_benchmark(bench-ransac-upright-planar-2pt-double
  SOURCES bench_ransac_upright_planar_2pt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-upright-planar-2pt-double)

# RANSAC Upright Planar 3pt Benchmark - Double
add_benchmark(bench-ransac-upright-planar-3pt-double
  SOURCES bench_ransac_upright_planar_3pt_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-upright-planar-3pt-double)

# RANSAC Upright Planar 3pt Hybrid Benchmark - Double
add_benchmark(bench-ransac-upright-planar-3pt-hybrid-double
  SOURCES bench_ransac_upright_planar_3pt_hybrid_double.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ransac-upright-planar-3pt-hybrid-double)

# =============================================================================

# Define target groups for different types of benchmarks
set(RANSAC_FLOAT_TARGETS
  bench-ransac-5pt-float
  bench-ransac-upright-3pt-float
  bench-ransac-upright-planar-2pt-float
  bench-ransac-upright-planar-3pt-float
  bench-ransac-upright-planar-3pt-hybrid-float
)

set(RANSAC_DOUBLE_TARGETS
  bench-ransac-5pt-double
  bench-ransac-upright-3pt-double
  bench-ransac-upright-planar-2pt-double
  bench-ransac-upright-planar-3pt-double
  bench-ransac-upright-planar-3pt-hybrid-double
)

set(QUICK_TEST_TARGETS
  bench-ransac-5pt-float
  bench-ransac-upright-3pt-float
)

# Create the benchmark runner script
create_benchmark_runner_script()

# =============================================================================
# Benchmark Group Runners
# =============================================================================

# Float precision RANSAC benchmarks
add_benchmark_group_runner(ransac-float
  TARGETS ${RANSAC_FLOAT_TARGETS}
  TIMEOUT 180
  OUTPUT_TIMEOUT 30
)

# Double precision RANSAC benchmarks
add_benchmark_group_runner(ransac-double
  TARGETS ${RANSAC_DOUBLE_TARGETS}
  TIMEOUT 180
  OUTPUT_TIMEOUT 30
)

# All new RANSAC benchmarks (float + double)
add_benchmark_group_runner(ransac-all
  TARGETS ${RANSAC_FLOAT_TARGETS} ${RANSAC_DOUBLE_TARGETS}
  TIMEOUT 240
  OUTPUT_TIMEOUT 40
)


# Quick test subset
add_benchmark_group_runner(robust-quick
  TARGETS ${QUICK_TEST_TARGETS}
  TIMEOUT 120
  OUTPUT_TIMEOUT 20
)

# All robust pose benchmarks
add_benchmark_group_runner(robust-all
  TARGETS ${BENCH_TARGETS}
  TIMEOUT 300
  OUTPUT_TIMEOUT 50
)

# =============================================================================
# Convenience Targets for Common Use Cases
# =============================================================================

add_custom_target(bench-robust-float
  DEPENDS bench-ransac-float-run
  COMMENT "Alias for bench-ransac-float-run"
)

add_custom_target(bench-robust-double
  DEPENDS bench-ransac-double-run
  COMMENT "Alias for bench-ransac-double-run"
)

add_custom_target(bench-robust-quick
  DEPENDS bench-robust-quick-run
  COMMENT "Alias for bench-robust-quick-run"
)

add_custom_target(bench-robust-all
  DEPENDS bench-robust-all-run
  COMMENT "Alias for bench-robust-all-run"
)

# Help target to list available benchmark groups
add_custom_target(bench-robust-help
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "EntoBench Robust Pose Estimation Benchmark Groups:"
  COMMAND ${CMAKE_COMMAND} -E echo "=================================================="
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Quick Testing:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-robust-quick-run      Fast subset - 5pt, upright 3pt float"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-robust-quick          Alias for bench-robust-quick-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "RANSAC Benchmarks by Precision:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-ransac-float-run      All float precision RANSAC algorithms"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-ransac-double-run     All double precision RANSAC algorithms"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-robust-float          Alias for bench-ransac-float-run"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-robust-double         Alias for bench-ransac-double-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Comprehensive Testing:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-ransac-all-run        All new RANSAC benchmarks - float + double"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-robust-all-run        All robust pose estimation benchmarks"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-robust-all            Alias for bench-robust-all-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Supported Algorithms:"
  COMMAND ${CMAKE_COMMAND} -E echo "  - 5pt - standard relative pose"
  COMMAND ${CMAKE_COMMAND} -E echo "  - upright_3pt - upright motion"
  COMMAND ${CMAKE_COMMAND} -E echo "  - upright_planar_2pt - planar + upright"
  COMMAND ${CMAKE_COMMAND} -E echo "  - upright_planar_3pt - planar + upright, overdetermined"
  COMMAND ${CMAKE_COMMAND} -E echo "  - upright_planar_3pt_hybrid - mixed solver"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Timeout Configuration:"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Quick tests: 120s timeout, 20s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Single precision: 180s timeout, 30s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo "  - All benchmarks: 300s timeout, 50s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Results are saved to benchmark_results_group-name/ directory"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMENT "Show available robust pose estimation benchmark groups"
)

# =============================================================================
# Apply Configuration Using Config File with Target-Specific Support
# =============================================================================

# Method 1: Enhanced config file approach with target-specific support
set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/benchmark/configs/robust_pose_benchmarks.json")
if(EXISTS ${CONFIG_FILE})
  message(STATUS "Using enhanced config file with target-specific support: ${CONFIG_FILE}")
  add_configured_benchmark_group_with_target_configs("robust-pose-est"
    CONFIG_FILE ${CONFIG_FILE}
    TARGETS ${BENCH_TARGETS}
  )
else()
  # Method 2: Direct configuration (fallback)
  message(STATUS "Config file not found, using direct configuration")
  add_preconfigured_benchmark_group("robust-pose-est"
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
message(STATUS "Robust Pose Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

endif() 