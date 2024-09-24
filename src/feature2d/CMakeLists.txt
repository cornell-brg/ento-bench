cmake_minimum_required(VERSION 3.16)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../apps/apps_common.cmake)

#=====================================================
# Testing
#=====================================================
include (CTest)
set(TEST_DIR "test")


if (GEM5_ARMV7E-M_BUILD)
  add_arm_baremetal_gem5_se_executable(test_fast
    SOURCES ${TEST_DIR}/test_fast.cc
    LIBRARIES image_io m startup sys)
else()
  #add_non_arm_executable(test_svd
  #                       SOURCES ${TEST_DIR}/test_svd.cc
  #                       LIBRARIES ento_math m)
  add_executable(test_fast ${TEST_DIR}/test_fast.cc)
  target_link_libraries(test_fast PUBLIC m image_io)
endif()
target_include_directories(test_fast
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${EIGEN_DIR}
)

add_test(
  NAME TestFast
  COMMAND test_fast
)
