# =============================================================================
# Extended Kalman Filter (EKF) Benchmarks - New Idioms
# =============================================================================
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if (STM32_BUILD)

# =============================================================================
# H7 Performance Scaling Configuration
# =============================================================================

# Detect H7 and set default performance multiplier
if(STM_PRODUCT MATCHES "H7")
  set(DEFAULT_H7_MULTIPLIER 2)  # H7 is ~2x faster than other STM32s
else()
  set(DEFAULT_H7_MULTIPLIER 1)
endif()

# Allow override via command line: cmake -DH7_PERFORMANCE_MULTIPLIER=3
if(NOT DEFINED H7_PERFORMANCE_MULTIPLIER)
  set(H7_PERFORMANCE_MULTIPLIER ${DEFAULT_H7_MULTIPLIER})
endif()

# Calculate scaled repetitions for H7
math(EXPR H7_SCALED_REPS "30 * ${H7_PERFORMANCE_MULTIPLIER}")

message(STATUS "EKF H7 Performance Multiplier: ${H7_PERFORMANCE_MULTIPLIER}")
message(STATUS "EKF H7 Scaled Reps: ${H7_SCALED_REPS}")

set(MODULES
  bench
  ento-mcu-semihosted
  Eigen
  ento-state-est
  ento_util
  CMSIS::STM32::${STM_PRODUCT}
)
set(BENCH_TARGETS) # List of targets

# =============================================================================
# Create all benchmark targets first
# =============================================================================

# RoboFly EKF benchmarks
add_benchmark(bench-robofly-ekf-float
  SOURCES bench_robofly_ekf_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-robofly-ekf-float)

# RoboBee EKF benchmarks
add_benchmark(bench-robobee-ekf-float
  SOURCES bench_robobee_ekf_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-robobee-ekf-float)

# EKF Update Method benchmarks
add_benchmark(bench-ekf-sync-update-float
  SOURCES bench_ekf_sync_update_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ekf-sync-update-float)

add_benchmark(bench-ekf-seq-update-float
  SOURCES bench_ekf_seq_update_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ekf-seq-update-float)

add_benchmark(bench-ekf-truncated-update-float
  SOURCES bench_ekf_truncated_update_float.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-ekf-truncated-update-float)

# =============================================================================
# Apply H7 Performance Scaling to All EKF Targets
# =============================================================================

foreach(target ${BENCH_TARGETS})
  # Set the REPS preprocessor define to scale repetitions based on platform
  target_compile_definitions(${target} PRIVATE 
    REPS=${H7_SCALED_REPS}
  )
  
  # Add H7-specific define if on H7
  if(STM_PRODUCT MATCHES "H7")
    target_compile_definitions(${target} PRIVATE STM32H7_BUILD=1)
  endif()
endforeach()

# =============================================================================
# Define Benchmark Groups for Organized Testing
# =============================================================================

# Group by robot platform
set(ROBOFLY_TARGETS
  bench-robofly-ekf-float
  bench-ekf-sync-update-float
  bench-ekf-seq-update-float
  bench-ekf-truncated-update-float
)

set(ROBOBEE_TARGETS
  bench-robobee-ekf-float
)

# Group by update method
set(SYNC_UPDATE_TARGETS
  bench-ekf-sync-update-float
)

set(ASYNC_UPDATE_TARGETS
  bench-ekf-seq-update-float
  bench-ekf-truncated-update-float
)

# Group by data type
set(FLOAT_TARGETS
  bench-robofly-ekf-float
  bench-robobee-ekf-float
  bench-ekf-sync-update-float
  bench-ekf-seq-update-float
  bench-ekf-truncated-update-float
)

set(ALL_EKF_TARGETS ${BENCH_TARGETS})
set(QUICK_EKF_TEST_TARGETS bench-robofly-ekf-float bench-robobee-ekf-float)
set(PERFORMANCE_EKF_TARGETS ${ALL_EKF_TARGETS})

# =============================================================================
# Add Benchmark Group Runners with Timeout Support
# =============================================================================

# Create the benchmark runner script (reuse from pose estimation)
create_benchmark_runner_script()

# Quick test runner (subset for fast validation)
add_benchmark_group_runner(ekf-quick-test
  TARGETS ${QUICK_EKF_TEST_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

# Performance test runner (all benchmarks)
add_benchmark_group_runner(ekf-performance
  TARGETS ${PERFORMANCE_EKF_TARGETS}
  TIMEOUT 120
  OUTPUT_TIMEOUT 20
)

# Platform-specific runners
add_benchmark_group_runner(robofly-ekf
  TARGETS ${ROBOFLY_TARGETS}
  TIMEOUT 90
  OUTPUT_TIMEOUT 15
)

add_benchmark_group_runner(robobee-ekf
  TARGETS ${ROBOBEE_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

# Update method comparison runners
add_benchmark_group_runner(ekf-sync-update
  TARGETS ${SYNC_UPDATE_TARGETS}
  TIMEOUT 60
  OUTPUT_TIMEOUT 10
)

add_benchmark_group_runner(ekf-async-update
  TARGETS ${ASYNC_UPDATE_TARGETS}
  TIMEOUT 90
  OUTPUT_TIMEOUT 15
)

# =============================================================================
# Convenience Targets for Common Use Cases
# =============================================================================

# Add convenience aliases that match common naming patterns
add_custom_target(bench-ekf-quick
  DEPENDS bench-ekf-quick-test-run
  COMMENT "Alias for bench-ekf-quick-test-run"
)

add_custom_target(bench-ekf-robofly
  DEPENDS bench-robofly-ekf-run
  COMMENT "Alias for bench-robofly-ekf-run"
)

add_custom_target(bench-ekf-robobee
  DEPENDS bench-robobee-ekf-run
  COMMENT "Alias for bench-robobee-ekf-run"
)

add_custom_target(bench-ekf-all
  DEPENDS bench-ekf-performance-run
  COMMENT "Alias for bench-ekf-performance-run"
)

# =============================================================================
# Apply Configuration Using Config File with Target-Specific Support
# =============================================================================

# Method 1: Enhanced config file approach with target-specific support
set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/benchmark/configs/estimation_benchmarks.json")
if(EXISTS ${CONFIG_FILE})
  message(STATUS "Using enhanced config file with target-specific support: ${CONFIG_FILE}")
  add_configured_benchmark_group_with_target_configs("ekf"
    CONFIG_FILE ${CONFIG_FILE}
    TARGETS ${BENCH_TARGETS}
  )
else()
  # Method 2: Direct configuration (fallback)
  message(STATUS "Config file not found, using direct configuration")
  add_preconfigured_benchmark_group("ekf"
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
message(STATUS "EKF Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

endif() # STM32_BUILD 