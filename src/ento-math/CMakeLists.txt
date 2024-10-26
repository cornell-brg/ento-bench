cmake_minimum_required(VERSION 3.16)

include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

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

set(TEST_FILES
  test_svd.cc)

set(LIBRARIES_test_svd
  m ento_math ento_util
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

