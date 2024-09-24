cmake_minimum_required(VERSION 3.16)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../apps/apps_common.cmake)

set(SRC_FILES
  linalg.cc
)

set(TEST_FILES
  test_svd.cc
)

add_library(ento_math
  ${SRC_FILES}
)

target_link_libraries(ento_math
  PUBLIC
  m
)

#=========================================

include(CTest)
set(TEST_DIR "test")

if (GEM5_ARMV7E-M_BUILD)
  add_arm_baremetal_gem5_se_executable(test_svd
    SOURCES ${TEST_DIR}/test_svd.cc
    LIBRARIES ento_math m startup sys)
else()
  #add_non_arm_executable(test_svd
  #                       SOURCES ${TEST_DIR}/test_svd.cc
  #                       LIBRARIES ento_math m)
  add_executable(test_svd ${TEST_DIR}/test_svd.cc)
  target_link_libraries(test_svd PUBLIC m ento_math)
endif()
target_include_directories(test_svd 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${EIGEN_DIR}
)

add_test(
  NAME TestSVD
  COMMAND test_svd
)

