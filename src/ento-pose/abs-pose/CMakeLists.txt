cmake_minimum_required(VERSION 3.16)

set(SRC_FILES
  dlt.cc
  p3p.cc
  p4p.cc
)

add_library(ento_abs_pose
  STATIC
  ${SRC_FILES}
)

target_link_libraries(ento_abs_pose
  PUBLIC
  ento_math
)

#=========================================
# Testing
#=========================================

include(CTest)

set(TEST_FILES
  test_p4p.cc
  test_dlt.cc
)

set(LIBRARIES_test_dlt
  ento_util ento_abs_pose feature2d Eigen
)

set(LIBRARIES_test_p4p
  ento_util ento_abs_pose feature2d Eigen
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

