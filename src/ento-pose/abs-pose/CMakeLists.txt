cmake_minimum_required(VERSION 3.16)

set(SRC_FILES
  #dlt.cc
  #p3p.cc
  #p4p.cc
)

add_library(ento-abs-pose
  INTERFACE
  #${SRC_FILES}
)

target_link_libraries(ento-abs-pose
  INTERFACE
  ento_math
)

#=========================================
# Data Generation
#=========================================

add_executable(generate_minimal_solver_data_abs
  test/generate_minimal_solver_data.cc
)

target_include_directories(generate_minimal_solver_data_abs
  PRIVATE
  ${EIGEN_DIR}
  
)

target_link_libraries(generate_minimal_solver_data_abs
  ento_util ento-abs-pose ento-pose
)

# Enhanced version with parameterized noise levels and realistic camera parameters
add_executable(generate_minimal_solver_data_abs_enhanced
  test/generate_minimal_solver_data_enhanced.cc
)

target_include_directories(generate_minimal_solver_data_abs_enhanced
  PRIVATE
  ${EIGEN_DIR}
  
)

target_link_libraries(generate_minimal_solver_data_abs_enhanced
  ento_util ento-abs-pose ento-pose
)

#=========================================
# Testing
#=========================================

include(CTest)

set(TEST_FILES
  test_p3p.cc
  #test_up2p.cc
  #test_p4p.cc
  test_dlt.cc
)

set(LIBRARIES_test_p4p
  ento_util ento-abs-pose ento-pose Eigen
)

set(LIBRARIES_test_p3p
  ento_util ento-abs-pose ento-pose Eigen
)

set(LIBRARIES_test_up2p
  ento_util ento-abs-pose ento-pose Eigen
)

set(LIBRARIES_test_dlt
  ento_util ento-abs-pose ento-pose Eigen
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

