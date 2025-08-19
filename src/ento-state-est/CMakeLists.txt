cmake_minimum_required(VERSION 3.16)
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

# add_subdirectory(factor-graph-chain)
# add_subdirectory(robofly-ekf)
# add_subdirectory(tiny-ekf)

add_library(ento-state-est
  INTERFACE
)

add_subdirectory(attitude-est)

# Experimental attitude estimation with failure checking
if(ENTO_ENABLE_FAILURE_CHECKS)
  add_subdirectory(attitude-est-exp)
endif()

#target_link_libraries(ento-state-est
  #PRIVATE
  #Eigen
  #)

# =============================================================================
# Testing
# =============================================================================

include(CTest)

set(TEST_FILES
  test_ekf.cc
  test_robobee_ekf_problem.cc
  test_robofly_ekf_problem.cc
  test_robobee_ekf_validation.cc
  test_robofly_ekf_robognat_validation.cc
)

set(LIBRARIES_test_ekf
  ento_util ento_math Eigen ento-state-est
)

set(LIBRARIES_test_robobee_ekf_problem
  ento_util ento_math Eigen ento-state-est
)

set(LIBRARIES_test_robofly_ekf_problem
  ento_util ento_math Eigen ento-state-est
)

set(LIBRARIES_test_robobee_ekf_validation
  ento_util ento_math Eigen ento-state-est
)

set(LIBRARIES_test_robofly_ekf_robognat_validation
  ento_util
  ento_math
  Eigen
  ento-state-est
)

#set(LIBRARIES_test_saam
#  ento_util ento_math Eigen
#)

foreach(TEST_FILE ${TEST_FILES})

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

