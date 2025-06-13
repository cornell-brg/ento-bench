# =============================================================================
# Attitude Estimation Benchmarks - New Idioms
# =============================================================================
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if (STM32_BUILD)
set(MODULES
  bench
  ento-mcu-semihosted
  Eigen
  ento-state-est
  ento-attitude-est
  ento_util
  ento_math
  CMSIS::STM32::${STM_PRODUCT}
)
set(BENCH_TARGETS) # List of targets

# =============================================================================
# Create all benchmark targets first
# =============================================================================

# Mahony benchmarks
add_benchmark(bench-mahony-float-imu
  SOURCES bench_mahony_float_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-float-imu)

add_benchmark(bench-mahony-float-marg
  SOURCES bench_mahony_float_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-float-marg)

add_benchmark(bench-mahony-double-imu
  SOURCES bench_mahony_double_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-double-imu)

add_benchmark(bench-mahony-double-marg
  SOURCES bench_mahony_double_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-double-marg)

add_benchmark(bench-mahony-q7-24-imu
  SOURCES bench_mahony_q7_24_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-q7-24-imu)

add_benchmark(bench-mahony-q7-24-marg
  SOURCES bench_mahony_q7_24_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-q7-24-marg)

add_benchmark(bench-mahony-q3-12-imu
  SOURCES bench_mahony_q3_12_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-q3-12-imu)

add_benchmark(bench-mahony-q3-12-marg
  SOURCES bench_mahony_q3_12_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-q3-12-marg)

# Madgwick benchmarks
add_benchmark(bench-madgwick-float-imu
  SOURCES bench_madgwick_float_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-float-imu)

add_benchmark(bench-madgwick-float-marg
  SOURCES bench_madgwick_float_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-float-marg)

add_benchmark(bench-madgwick-double-imu
  SOURCES bench_madgwick_double_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-double-imu)

add_benchmark(bench-madgwick-double-marg
  SOURCES bench_madgwick_double_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-double-marg)

add_benchmark(bench-madgwick-q7-24-imu
  SOURCES bench_madgwick_q7_24_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-q7-24-imu)

add_benchmark(bench-madgwick-q7-24-marg
  SOURCES bench_madgwick_q7_24_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-q7-24-marg)

add_benchmark(bench-madgwick-q3-12-imu
  SOURCES bench_madgwick_q3_12_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-q3-12-imu)

add_benchmark(bench-madgwick-q3-12-marg
  SOURCES bench_madgwick_q3_12_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-q3-12-marg)

# Fourati benchmarks
add_benchmark(bench-fourati-float-marg
  SOURCES bench_fourati_float_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fourati-float-marg)

add_benchmark(bench-fourati-double-marg
  SOURCES bench_fourati_double_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fourati-double-marg)

# =============================================================================
# Define Benchmark Groups for Organized Testing
# =============================================================================

# Group by algorithm type
set(MAHONY_TARGETS
  bench-mahony-float-imu bench-mahony-float-marg
  bench-mahony-double-imu bench-mahony-double-marg
  bench-mahony-q7-24-imu bench-mahony-q7-24-marg
  bench-mahony-q3-12-imu bench-mahony-q3-12-marg
)

set(MADGWICK_TARGETS
  bench-madgwick-float-imu bench-madgwick-float-marg
  bench-madgwick-double-imu bench-madgwick-double-marg
  bench-madgwick-q7-24-imu bench-madgwick-q7-24-marg
  bench-madgwick-q3-12-imu bench-madgwick-q3-12-marg
)

set(FOURATI_TARGETS
  bench-fourati-float-marg bench-fourati-double-marg
)

# Group by data type
set(FLOAT_TARGETS
  bench-mahony-float-imu bench-mahony-float-marg
  bench-madgwick-float-imu bench-madgwick-float-marg
  bench-fourati-float-marg
)

set(DOUBLE_TARGETS
  bench-mahony-double-imu bench-mahony-double-marg
  bench-madgwick-double-imu bench-madgwick-double-marg
  bench-fourati-double-marg
)

set(FIXED_POINT_TARGETS
  bench-mahony-q7-24-imu bench-mahony-q7-24-marg
  bench-madgwick-q7-24-imu bench-madgwick-q7-24-marg
  bench-mahony-q3-12-imu bench-mahony-q3-12-marg
  bench-madgwick-q3-12-imu bench-madgwick-q3-12-marg
)

# Group by sensor configuration
set(IMU_ONLY_TARGETS
  bench-mahony-float-imu bench-mahony-double-imu
  bench-madgwick-float-imu bench-madgwick-double-imu
  bench-mahony-q7-24-imu bench-madgwick-q7-24-imu
  bench-mahony-q3-12-imu bench-madgwick-q3-12-imu
)

set(MARG_TARGETS
  bench-mahony-float-marg bench-mahony-double-marg
  bench-madgwick-float-marg bench-madgwick-double-marg
  bench-fourati-float-marg bench-fourati-double-marg
  bench-mahony-q7-24-marg bench-madgwick-q7-24-marg
  bench-mahony-q3-12-marg bench-madgwick-q3-12-marg
)

set(ALL_ATTITUDE_TARGETS ${BENCH_TARGETS})
set(QUICK_ATTITUDE_TEST_TARGETS bench-mahony-float-imu bench-madgwick-float-imu bench-fourati-float-marg)
set(PERFORMANCE_ATTITUDE_TARGETS ${ALL_ATTITUDE_TARGETS})

# =============================================================================
# Add Benchmark Group Runners with Timeout Support
# =============================================================================

# Create the benchmark runner script (reuse from pose estimation)
create_benchmark_runner_script()

# Quick test group (fast benchmarks for testing)
add_benchmark_group_runner(attitude-quick-test
  TARGETS ${QUICK_ATTITUDE_TEST_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

# Algorithm-based groups
add_benchmark_group_runner(mahony
  TARGETS ${MAHONY_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

add_benchmark_group_runner(madgwick
  TARGETS ${MADGWICK_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

add_benchmark_group_runner(fourati
  TARGETS ${FOURATI_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

# Data type groups
add_benchmark_group_runner(attitude-float
  TARGETS ${FLOAT_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

add_benchmark_group_runner(attitude-double
  TARGETS ${DOUBLE_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

add_benchmark_group_runner(attitude-fixed-point
  TARGETS ${FIXED_POINT_TARGETS}
  TIMEOUT 90
  OUTPUT_TIMEOUT 15
)

# Sensor configuration groups
add_benchmark_group_runner(attitude-imu-only
  TARGETS ${IMU_ONLY_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

add_benchmark_group_runner(attitude-marg
  TARGETS ${MARG_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

# Comprehensive groups
add_benchmark_group_runner(all-attitude
  TARGETS ${ALL_ATTITUDE_TARGETS}
  TIMEOUT 90
  OUTPUT_TIMEOUT 15
)

add_benchmark_group_runner(attitude-performance
  TARGETS ${PERFORMANCE_ATTITUDE_TARGETS}
  TIMEOUT 120
  OUTPUT_TIMEOUT 20
)

# =============================================================================
# Convenience Targets for Common Use Cases
# =============================================================================

# Add convenience aliases that match common naming patterns
add_custom_target(bench-attitude-quick
  DEPENDS bench-attitude-quick-test-run
  COMMENT "Alias for bench-attitude-quick-test-run"
)

add_custom_target(bench-attitude-mahony
  DEPENDS bench-mahony-run
  COMMENT "Alias for bench-mahony-run"
)

add_custom_target(bench-attitude-madgwick
  DEPENDS bench-madgwick-run
  COMMENT "Alias for bench-madgwick-run"
)

add_custom_target(bench-attitude-fourati
  DEPENDS bench-fourati-run
  COMMENT "Alias for bench-fourati-run"
)

add_custom_target(bench-attitude-all
  DEPENDS bench-all-attitude-run
  COMMENT "Alias for bench-all-attitude-run"
)

add_custom_target(bench-attitude-performance
  DEPENDS bench-attitude-performance-run
  COMMENT "Alias for bench-attitude-performance-run"
)

# Help target to list available benchmark groups
add_custom_target(bench-attitude-help
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "EntoBench Attitude Estimation Benchmark Groups:"
  COMMAND ${CMAKE_COMMAND} -E echo "=============================================="
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Quick Testing:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-quick-test-run    Fast subset for testing (Mahony, Madgwick, Fourati float)"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-quick             Alias for bench-attitude-quick-test-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Algorithm Groups:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-mahony-run                 All Mahony filter variants"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-madgwick-run               All Madgwick filter variants"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-fourati-run                All Fourati filter variants"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-mahony            Alias for bench-mahony-run"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-madgwick          Alias for bench-madgwick-run"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-fourati           Alias for bench-fourati-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Data Type Groups:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-float-run         All floating-point benchmarks"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-double-run        All double-precision benchmarks"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-fixed-point-run   All fixed-point benchmarks (Q7.24, Q3.12)"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Sensor Configuration Groups:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-imu-only-run      IMU-only benchmarks (gyro + accel)"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-marg-run          MARG benchmarks (gyro + accel + mag)"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Comprehensive Testing:"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-all-attitude-run           All attitude estimation benchmarks"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-performance-run   Comprehensive performance evaluation"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-all               Alias for bench-all-attitude-run"
  COMMAND ${CMAKE_COMMAND} -E echo "  bench-attitude-performance       Alias for bench-attitude-performance-run"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Timeout Configuration:"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Basic benchmarks: 60s timeout, 10s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Fixed-point benchmarks: 90s timeout, 15s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo "  - Performance: 120s timeout, 20s output timeout"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMAND ${CMAKE_COMMAND} -E echo "Results are saved to benchmark_results_<group-name>_<config>/ directory"
  COMMAND ${CMAKE_COMMAND} -E echo ""
  COMMENT "Show available attitude estimation benchmark groups"
)

# =============================================================================
# Apply Configuration Using Config File with Target-Specific Support
# =============================================================================

# Method 1: Enhanced config file approach with target-specific support
set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/benchmark/configs/estimation_benchmarks.json")
if(EXISTS ${CONFIG_FILE})
  message(STATUS "Using enhanced config file with target-specific support: ${CONFIG_FILE}")
  add_configured_benchmark_group_with_target_configs("attitude-est"
    CONFIG_FILE ${CONFIG_FILE}
    TARGETS ${BENCH_TARGETS}
  )
else()
  # Method 2: Direct configuration (fallback)
  message(STATUS "Config file not found, using direct configuration")
  add_preconfigured_benchmark_group("attitude-est"
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
message(STATUS "Attitude Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

endif() 
