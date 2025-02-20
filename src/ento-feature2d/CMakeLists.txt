cmake_minimum_required(VERSION 3.16)

include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

add_library(
  feature2d
  INTERFACE
)

#=====================================================
# Testing
#=====================================================
include (CTest)

set(TEST_FILES
  test_fast.cc
  test_feature2d_util.cc
  test_lk_optical_flow.cc
  test_lk_optical_flow_iter.cc
  test_image_pyramid.cc
)

set(LIBRARIES_test_fast
  m image_io ento_util Eigen)

set(LIBRARIES_test_feature2d_util
  m feature2d ento_util Eigen)

set(LIBRARIES_test_lk_optical_flow
  m ento_util png image_io Eigen)

set(LIBRARIES_test_lk_optical_flow_iter
  m ento_util png image_io Eigen)

set(LIBRARIES_test_image_pyramid
  m ento_util png image_io Eigen)

foreach(TEST_FILE ${TEST_FILES})
  message(STATUS "Test file: ${TEST_FILE}")
  set(TEST_SRC "test/${TEST_FILE}")
  get_filename_component(TEST_BIN ${TEST_FILE} NAME_WE)
  message(STATUS "Test bin: ${TEST_BIN}")
  string(CONCAT LIBRARY_VAR_NAME "LIBRARIES_" ${TEST_BIN})
  message(STATUS ${${LIBRARY_VAR_NAME}})
  set(TEST_LIBRARIES ${${LIBRARY_VAR_NAME}})
  add_ento_test(
    ${TEST_BIN}
    SOURCES   ${TEST_SRC}
    LIBRARIES ${TEST_LIBRARIES}
  )
endforeach()

#if (GEM5_ARMV7E-M_BUILD)
#add_arm_baremetal_gem5_se_executable(test_fast
#SOURCES ${TEST_DIR}/test_fast.cc
#LIBRARIES image_io m startup sys)
#elseif(STM32_BUILD)
#add_arm_semihosting_executable(
#test_fast
#SOURCES ${TEST_DIR}/test_fast.cc
#LIBRARIES image_io m mcu-util
#)
#else()
  #add_non_arm_executable(test_svd
  #                       SOURCES ${TEST_DIR}/test_svd.cc
  #                       LIBRARIES ento_math m)
  #add_executable(test_fast ${TEST_DIR}/test_fast.cc)
  #target_link_libraries(test_fast PUBLIC m image_io)
  #endif()
  #target_include_directories(test_fast
  #PUBLIC
  #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  #${EIGEN_DIR}
  #)
