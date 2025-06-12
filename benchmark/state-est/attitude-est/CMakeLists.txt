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