# =============================================================================
# Extended Kalman Filter (EKF) Benchmarks - New Idioms
# =============================================================================
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if (STM32_BUILD)

# =============================================================================
# H7 Performance Multiplier Detection (mirrors bench_config.h logic)
# =============================================================================

# Detect H7 and set performance multiplier (same logic as in bench_config.h)
if(STM_PRODUCT MATCHES "H7")
  set(H7_PERFORMANCE_MULTIPLIER 1)  # H7 is ~2x faster than other STM32s
else()
  set(H7_PERFORMANCE_MULTIPLIER 1)  # No scaling for non-H7
endif()

# Allow override via command line: cmake -DH7_PERFORMANCE_MULTIPLIER=3
if(DEFINED H7_PERFORMANCE_MULTIPLIER_OVERRIDE)
  set(H7_PERFORMANCE_MULTIPLIER ${H7_PERFORMANCE_MULTIPLIER_OVERRIDE})
  message(STATUS "H7 Performance Multiplier overridden to: ${H7_PERFORMANCE_MULTIPLIER}")
endif()

message(STATUS "EKF H7 Performance Multiplier: ${H7_PERFORMANCE_MULTIPLIER}")

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
# H7-specific defines (automatic scaling handled in bench_config.h)
# =============================================================================

foreach(target ${BENCH_TARGETS})
  # Add H7-specific define if on H7 (for any H7-specific code paths)
  if(STM_PRODUCT MATCHES "H7")
    target_compile_definitions(${target} PRIVATE STM32H7_BUILD=1)
  endif()
  
  # Pass the multiplier to C++ as a verification define (optional)
  target_compile_definitions(${target} PRIVATE H7_CMAKE_MULTIPLIER=${H7_PERFORMANCE_MULTIPLIER})
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

# =============================================================================
# Configure benchmark groups with JSON config and H7 scaling awareness
# =============================================================================

# Get base inner_reps from JSON config for proper log file naming
set(BASE_INNER_REPS 2)  # From estimation_benchmarks.json "ekf" section
math(EXPR EFFECTIVE_INNER_REPS "${BASE_INNER_REPS} * ${H7_PERFORMANCE_MULTIPLIER}")

message(STATUS "Base INNER_REPS from JSON: ${BASE_INNER_REPS}")
message(STATUS "Effective INNER_REPS (with H7 scaling): ${EFFECTIVE_INNER_REPS}")

# Configure EKF benchmarks using the estimation_benchmarks.json config
# The config system will use the base values, but log files will reflect the effective values
add_configured_benchmark_group_with_target_configs("ekf"
  CONFIG_FILE "${CMAKE_SOURCE_DIR}/benchmark/configs/estimation_benchmarks.json"
  TARGETS ${BENCH_TARGETS}
)

# Override the global config string to reflect H7 scaling in log file names
if(H7_PERFORMANCE_MULTIPLIER GREATER 1)
  set(H7_CONFIG_SUFFIX "_h7x${H7_PERFORMANCE_MULTIPLIER}")
  set(GLOBAL_BENCHMARK_CONFIG_STR "${GLOBAL_BENCHMARK_CONFIG_STR}${H7_CONFIG_SUFFIX}" CACHE INTERNAL "Current benchmark configuration string with H7 scaling")
  message(STATUS "Updated benchmark config string for H7 scaling: ${GLOBAL_BENCHMARK_CONFIG_STR}")
endif()

message(STATUS "EKF benchmarks configured with automatic H7 scaling")
if(STM_PRODUCT MATCHES "H7")
  message(STATUS "H7 detected: INNER_REPS will be automatically scaled by ${H7_PERFORMANCE_MULTIPLIER}x in bench_config.h")
  message(STATUS "Log files will include H7 scaling suffix: ${H7_CONFIG_SUFFIX}")
else()
  message(STATUS "Non-H7 platform: INNER_REPS will use base values from JSON config")
endif()

# =============================================================================
# Add STM32 targets for flashing/debugging
# =============================================================================
message(STATUS "EKF Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

endif() # STM32_BUILD 
