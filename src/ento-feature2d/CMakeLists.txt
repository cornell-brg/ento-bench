cmake_minimum_required(VERSION 3.16)

include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

add_library(
  ento-perception
  INTERFACE
)

target_link_libraries(ento-perception
  INTERFACE 
  image_io
)

target_include_directories(ento-perception
  INTERFACE
  image_io
  ento_util
)

#=====================================================
# Testing
#=====================================================
include (CTest)

set(TEST_FILES
  test_fast.cc
  test_feature2d_util.cc
  test_lk_optical_flow.cc
  test_lk_optical_flow_iter.cc
  test_image_pyramid.cc
  test_sparse_optical_flow_problem.cc
  test_feature_recognition_problem.cc
  test_sift.cc
  test_simple_gaussian.cc
)

set(LIBRARIES_test_fast
  m ento-perception ento_util Eigen)

set(LIBRARIES_test_sift
  m ento-perception ento_util Eigen)

set(LIBRARIES_test_simple_gaussian
  m ento-perception ento_util Eigen)

set(LIBRARIES_test_feature2d_util
  m ento-perception ento_util Eigen)

set(LIBRARIES_test_sparse_optical_flow_problem
  m ento-perception ento_util Eigen image_io)

set(LIBRARIES_test_lk_optical_flow
  m ento_util image_io Eigen)

set(LIBRARIES_test_lk_optical_flow_iter
  m ento_util image_io Eigen)

set(LIBRARIES_test_image_pyramid
  m ento_util image_io Eigen)

set(LIBRARIES_test_feature_recognition_problem
  m ento-perception ento_util image_io Eigen )

foreach(TEST_FILE ${TEST_FILES})
  
  set(TEST_SRC "test/${TEST_FILE}")
  get_filename_component(TEST_BIN ${TEST_FILE} NAME_WE)
  string(CONCAT LIBRARY_VAR_NAME "LIBRARIES_" ${TEST_BIN})
  set(TEST_LIBRARIES ${${LIBRARY_VAR_NAME}})
  add_ento_test(
    ${TEST_BIN}
    SOURCES   ${TEST_SRC}
    LIBRARIES ${TEST_LIBRARIES}
  )
endforeach()
