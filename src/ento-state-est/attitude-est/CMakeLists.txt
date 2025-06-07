cmake_minimum_required(VERSION 3.16)
add_library(ento-attitude-est
  INTERFACE
)

target_link_libraries(ento-attitude-est
  INTERFACE
  ento-state-est
  ento_math
)

# =============================================================================
# Testing
# =============================================================================

include(CTest)

set(TEST_FILES
  test_attitude_problem.cc
  test_madgwick.cc
  test_mahoney.cc
  test_fourati.cc
  test_mahoney_fixed.cc
  test_madgwick_fixed.cc
  benchmark_attitude_filters.cc
)

set(LIBRARIES_test_attitude_problem
  ento_util ento_math Eigen ento-attitude-est ento-state-est
)

set(LIBRARIES_test_madgwick
  ento_util ento_math Eigen ento-state-est ento-attitude-est 
)

set(LIBRARIES_test_mahoney
  ento_util ento_math Eigen ento-state-est ento-attitude-est
)

set(LIBRARIES_test_fourati
  ento_util ento_math Eigen ento-state-est ento-attitude-est
)

set(LIBRARIES_test_mahoney_fixed
  ento_util ento_math Eigen ento-state-est ento-attitude-est
)

set(LIBRARIES_test_madgwick_fixed
  ento_util ento_math Eigen ento-state-est ento-attitude-est
)

set(LIBRARIES_benchmark_attitude_filters
  ento_util ento_math Eigen ento-state-est ento-attitude-est bench
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

# Add debug test for MARG issue
add_executable(debug_marg_issue test/debug_marg_issue.cc)
target_link_libraries(debug_marg_issue ento_util ento_math ento-state-est ento-attitude-est)
target_include_directories(debug_marg_issue PRIVATE ${CMAKE_SOURCE_DIR}/src ${EIGEN_DIR})
set_target_properties(debug_marg_issue PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
