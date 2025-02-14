cmake_minimum_required(VERSION 3.16)

add_library(ento-attitude-est
  INTERFACE
)

target_link_libraries(ento-attitude-est
  INTERFACE
  ento_math
)

# =============================================================================
# Testing
# =============================================================================

include(CTest)

set(TEST_FILES
  test_attitude_problem.cc
  #test_saam.cc
)

set(LIBRARIES_test_attitude_problem
  ento_util ento_math Eigen ento-attitude-est
)

#set(LIBRARIES_test_saam
#  ento_util ento_math Eigen
#)

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
