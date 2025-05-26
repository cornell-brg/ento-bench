include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

# Main interface library
add_library(ento-control INTERFACE)

# Add includes to ento-control
target_include_directories(ento-control INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}  # For ento-control headers
)

# =============================================================================
# Testing
# =============================================================================

include(CTest)

set(TEST_FILES
  test_tiny_mpc.cc
  test_lqr.cc
  test_robofly_tinympc.cc
)

set(LIBRARIES_test_tiny_mpc
  ento_util ento_math Eigen ento-control
)

set(LIBRARIES_test_lqr
  ento_util ento_math Eigen ento-control
)

set(LIBRARIES_test_robofly_tinympc
  ento_util ento_math Eigen ento-control
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

