cmake_minimum_required(VERSION 3.16)

add_library(ento-robust-est
  INTERFACE
)

target_link_libraries(ento-robust-est
  INTERFACE
  ento_math
)

#=========================================
# Testing
#=========================================

include(CTest)

set(TEST_FILES
  test_ransac.cc
  test_estimators.cc
  test_robust_absolute_pose.cc
)

set(LIBRARIES_test_ransac
  ento_util ento-robust-est ento-pose Eigen
)

set(LIBRARIES_test_estimators
  ento_util ento-robust-est ento-pose Eigen
)

set(LIBRARIES_test_robust_absolute_pose
  ento_util ento-robust-est ento-pose Eigen
)

foreach(TEST_FILE ${TEST_FILES})
  message(STATUS "Test file: ${TEST_FILE}")
  set(TEST_SRC "test/${TEST_FILE}")
  get_filename_component(TEST_BIN ${TEST_FILE} NAME_WE)
  string(CONCAT LIBRARY_VAR_NAME "LIBRARIES_" ${TEST_BIN})
  message(STATUS ${${LIBRARY_VAR_NAME}})
  set(TEST_LIBRARIES ${${LIBRARY_VAR_NAME}})
  add_ento_test(
    ${TEST_BIN}
    SOURCES   ${TEST_SRC}
    LIBRARIES ${TEST_LIBRARIES}
  )
endforeach() 