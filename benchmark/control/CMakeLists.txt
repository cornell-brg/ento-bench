# =============================================================================
# Control Benchmarks 
# =============================================================================
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if(STM32_BUILD)

# =============================================================================
# H7 Performance Multiplier Detection (mirrors bench_config.h logic)
# =============================================================================

# Detect H7 and set performance multiplier (same logic as in bench_config.h)
if(STM_PRODUCT MATCHES "H7")
  set(H7_PERFORMANCE_MULTIPLIER 2)  # H7 is ~2x faster than other STM32s
else()
  set(H7_PERFORMANCE_MULTIPLIER 1)  # No scaling for non-H7
endif()

# Allow override via command line: cmake -DH7_PERFORMANCE_MULTIPLIER_OVERRIDE=3
if(DEFINED H7_PERFORMANCE_MULTIPLIER_OVERRIDE)
  set(H7_PERFORMANCE_MULTIPLIER ${H7_PERFORMANCE_MULTIPLIER_OVERRIDE})
  message(STATUS "Control H7 Performance Multiplier overridden to: ${H7_PERFORMANCE_MULTIPLIER}")
endif()

message(STATUS "Control H7 Performance Multiplier: ${H7_PERFORMANCE_MULTIPLIER}")

set(MODULES
    bench
    ento-mcu-semihosted
    Eigen
    ento-control
    ento_util
    CMSIS::STM32::${STM_PRODUCT}
)
set(BENCH_TARGETS)

# =============================================================================
# Create Control Benchmark Targets
# =============================================================================

# TinyMPC Solver Benchmark
add_benchmark(bench-tinympc
    SOURCES bench_tinympc.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-tinympc)

# RoboFly LQR Controller Benchmark
add_benchmark(bench-robofly-lqr
    SOURCES bench_robofly_lqr.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-robofly-lqr)

# RoboFly Sparse LQR Controller Benchmark
add_benchmark(bench-robofly-lqr-sparse
    SOURCES bench_robofly_lqr_sparse.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-robofly-lqr-sparse)

# RoboFly Custom Sparse LQR Controller Benchmark
add_benchmark(bench-robofly-lqr-fixed-sparse
    SOURCES bench_robofly_lqr_fixed_sparse.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-robofly-lqr-fixed-sparse)

# RoboFly TinyMPC Controller Benchmark
add_benchmark(bench-robofly-tinympc
    SOURCES bench_robofly_tinympc.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-robofly-tinympc)

# RoboFly TinyMPC Hover Trajectory Benchmark
add_benchmark(bench-robofly-tinympc-hover
    SOURCES bench_robofly_tinympc_hover.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-robofly-tinympc-hover)

# RoboFly TinyMPC Square Trajectory Benchmark
add_benchmark(bench-robofly-tinympc-square
    SOURCES bench_robofly_tinympc_square.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-robofly-tinympc-square)

# RoboFly TinyMPC Circle Trajectory Benchmark
add_benchmark(bench-robofly-tinympc-circle
    SOURCES bench_robofly_tinympc_circle.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-robofly-tinympc-circle)

# TinyMPC Paper Specification Benchmark (10 states, 3 inputs, 5 horizon)
add_benchmark(bench-tinympc-paper-spec
    SOURCES bench_tinympc_paper_spec.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-tinympc-paper-spec)

# TinyMPC Horizon Length Comparison Benchmark
add_benchmark(bench-tinympc-horizon-comparison
    SOURCES bench_tinympc_horizon_comparison.cc
    LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-tinympc-horizon-comparison)

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
# Configure benchmark groups with H7 scaling awareness
# =============================================================================

# Get base inner_reps for control benchmarks
set(BASE_INNER_REPS 1)  # Default for control benchmarks
math(EXPR EFFECTIVE_INNER_REPS "${BASE_INNER_REPS} * ${H7_PERFORMANCE_MULTIPLIER}")

message(STATUS "Base INNER_REPS for control: ${BASE_INNER_REPS}")
message(STATUS "Effective INNER_REPS (with H7 scaling): ${EFFECTIVE_INNER_REPS}")

# Create control benchmark configuration
add_preconfigured_benchmark_group("control"
  TARGETS ${BENCH_TARGETS}
  REPS 1
  INNER_REPS ${BASE_INNER_REPS}  # Will be automatically scaled in bench_config.h
  VERBOSITY 1
  ENABLE_CACHES
)

# Override the global config string to reflect H7 scaling in log file names
if(H7_PERFORMANCE_MULTIPLIER GREATER 1)
  set(H7_CONFIG_SUFFIX "_h7x${H7_PERFORMANCE_MULTIPLIER}")
  set(GLOBAL_BENCHMARK_CONFIG_STR "${GLOBAL_BENCHMARK_CONFIG_STR}${H7_CONFIG_SUFFIX}" CACHE INTERNAL "Current benchmark configuration string with H7 scaling")
  message(STATUS "Updated control benchmark config string for H7 scaling: ${GLOBAL_BENCHMARK_CONFIG_STR}")
endif()

message(STATUS "Control benchmarks configured with automatic H7 scaling")
if(STM_PRODUCT MATCHES "H7")
  message(STATUS "H7 detected: INNER_REPS will be automatically scaled by ${H7_PERFORMANCE_MULTIPLIER}x in bench_config.h")
  message(STATUS "Log files will include H7 scaling suffix: ${H7_CONFIG_SUFFIX}")
else()
  message(STATUS "Non-H7 platform: INNER_REPS will use base values from config")
endif()

message(STATUS "Control Bench targets: ${BENCH_TARGETS}")

# =============================================================================
# Add STM32 targets for flashing/debugging
# =============================================================================
add_stm32_targets("${BENCH_TARGETS}")

endif() # STM32_BUILD
