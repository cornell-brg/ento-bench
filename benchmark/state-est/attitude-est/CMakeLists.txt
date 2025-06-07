# =============================================================================
# Attitude Estimation Benchmarks
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
# Mahony Float IMU
# =============================================================================

add_benchmark(bench-mahony-float-imu
  SOURCES bench_mahony_float_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-float-imu)

# =============================================================================
# Mahony Float MARG
# =============================================================================

add_benchmark(bench-mahony-float-marg
  SOURCES bench_mahony_float_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-float-marg)

# =============================================================================
# Mahony Double IMU
# =============================================================================

add_benchmark(bench-mahony-double-imu
  SOURCES bench_mahony_double_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-double-imu)

# =============================================================================
# Mahony Double MARG
# =============================================================================

add_benchmark(bench-mahony-double-marg
  SOURCES bench_mahony_double_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-double-marg)

# =============================================================================
# Mahony Q7.24 IMU
# =============================================================================

add_benchmark(bench-mahony-q7-24-imu
  SOURCES bench_mahony_q7_24_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-q7-24-imu)

# =============================================================================
# Mahony Q7.24 MARG
# =============================================================================

add_benchmark(bench-mahony-q7-24-marg
  SOURCES bench_mahony_q7_24_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-q7-24-marg)

# =============================================================================
# Mahony Q3.12 IMU
# =============================================================================

add_benchmark(bench-mahony-q3-12-imu
  SOURCES bench_mahony_q3_12_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-q3-12-imu)

# =============================================================================
# Mahony Q3.12 MARG
# =============================================================================

add_benchmark(bench-mahony-q3-12-marg
  SOURCES bench_mahony_q3_12_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-mahony-q3-12-marg)

# =============================================================================
# Madgwick Float IMU
# =============================================================================

add_benchmark(bench-madgwick-float-imu
  SOURCES bench_madgwick_float_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-float-imu)

# =============================================================================
# Madgwick Float MARG
# =============================================================================

add_benchmark(bench-madgwick-float-marg
  SOURCES bench_madgwick_float_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-float-marg)

# =============================================================================
# Madgwick Double IMU
# =============================================================================

add_benchmark(bench-madgwick-double-imu
  SOURCES bench_madgwick_double_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-double-imu)

# =============================================================================
# Madgwick Double MARG
# =============================================================================

add_benchmark(bench-madgwick-double-marg
  SOURCES bench_madgwick_double_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-double-marg)

# =============================================================================
# Madgwick Q7.24 IMU
# =============================================================================

add_benchmark(bench-madgwick-q7-24-imu
  SOURCES bench_madgwick_q7_24_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-q7-24-imu)

# =============================================================================
# Madgwick Q7.24 MARG
# =============================================================================

add_benchmark(bench-madgwick-q7-24-marg
  SOURCES bench_madgwick_q7_24_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-q7-24-marg)

# =============================================================================
# Madgwick Q3.12 IMU
# =============================================================================

add_benchmark(bench-madgwick-q3-12-imu
  SOURCES bench_madgwick_q3_12_imu.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-q3-12-imu)

# =============================================================================
# Madgwick Q3.12 MARG
# =============================================================================

add_benchmark(bench-madgwick-q3-12-marg
  SOURCES bench_madgwick_q3_12_marg.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-madgwick-q3-12-marg)

# =============================================================================
# Add stm32 targets
# =============================================================================
message(STATUS "Attitude Bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

endif() 