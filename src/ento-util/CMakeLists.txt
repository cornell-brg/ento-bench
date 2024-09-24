cmake_minimum_required(VERSION 3.16)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../apps/apps_common.cmake)

include (CTest)
set(TEST_DIR "test")


if (GEM5_ARMV7E-M_BUILD)
  add_arm_baremetal_gem5_se_executable(test_matrix_reader
    SOURCES ${TEST_DIR}/test_matrix_reader.cc
    LIBRARIES ento_math m startup sys)
else()
  #add_non_arm_executable(test_svd
  #                       SOURCES ${TEST_DIR}/test_svd.cc
  #                       LIBRARIES ento_math m)
  add_executable(test_matrix_reader ${TEST_DIR}/test_matrix_reader.cc)
  target_link_libraries(test_matrix_reader
    PUBLIC
    m
    ento_math)
endif()

target_include_directories(test_matrix_reader 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${EIGEN_DIR}
)

add_test(
  NAME TestMatrixReader
  COMMAND test_matrix_reader
)
