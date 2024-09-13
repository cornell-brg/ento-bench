include(${CMAKE_CURRENT_SOURCE_DIR}/../apps_common.cmake)

# Build svd_bench.cc
set(SVD_BENCH_SOURCES
  svd_bench.cc
  ekf_bench.cc
)

message("GSL_DIR: ${GSL_DIR}")
message("GSL_INC: ${GSL_INC}")
message("GSL_LIB: ${GSL_LIB}")

if (STM32_BUILD)
  message("Building microbenchmarks for STM32 Semihosting")
  add_arm_semihosting_executable(svd_bench
                                 SOURCES svd_bench.cc
                                 LIBRARIES bench mcu-util linalg)
elseif(GEM5_ARMV7E-M_BUILD)
  message("Building microbenchmarks for gem5 ARM")
  add_arm_baremetal_gem5_se_executable(svd_bench
                                       SOURCES svd_bench.cc
                                       LIBRARIES bench startup linalg sys)
else()
  message("Building microbenchmarks for ${CMAKE_SYSTEM_PROCESSOR}")
  add_non_arm_executable(svd_bench
                         SOURCES svd_bench.cc
                         LIBRARIES bench util linalg)
endif()

target_include_directories(svd_bench PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_DIR}
)


if (STM32_BUILD)
  message("Building microbenchmarks for STM32 Semihosting")
  add_arm_semihosting_executable(ekf_bench
                                 SOURCES ekf_bench.cc
                                 LIBRARIES bench mcu-util linalg)
elseif(GEM5_ARMV7E-M_BUILD)
  message("Building microbenchmarks for gem5 ARM")
  add_arm_baremetal_gem5_se_executable(ekf_bench
                                       SOURCES ekf_bench.cc
                                       LIBRARIES bench startup linalg sys)
else()
  message("Building microbenchmarks for ${CMAKE_SYSTEM_PROCESSOR}")
  add_non_arm_executable(ekf_bench
                         SOURCES ekf_bench.cc
                         LIBRARIES bench util linalg)
endif()

target_include_directories(ekf_bench PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_DIR}
)
