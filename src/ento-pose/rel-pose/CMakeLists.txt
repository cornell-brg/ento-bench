cmake_minimum_required(VERSION 3.16)

set(SRC_FILES
  #eight_pt.cc
  #five_pt_nister.cc
  #upright_three_pt.cc
  #upright_planar_two_pt.cc
  #upright_planar_three_pt.cc
)

add_library(ento-rel-pose
  INTERFACE
  #${SRC_FILES}
)

target_link_libraries(ento-rel-pose
  INTERFACE
  ento_math
)

#=========================================
# Testing
#=========================================

include(CTest)

set(TEST_FILES
  test_eight_pt.cc
  test_five_pt.cc
  test_upright_planar_two_pt.cc
  test_upright_planar_three_pt.cc
  test_upright_three_pt.cc
)

set(LIBRARIES_test_eight_pt
  ento_util ento-rel-pose ento-pose Eigen
)

set(LIBRARIES_test_five_pt
  ento_util ento-rel-pose ento-pose Eigen
)

set(LIBRARIES_test_upright_planar_two_pt
  ento_util ento-rel-pose ento-pose Eigen
)

set(LIBRARIES_test_upright_planar_three_pt
  ento_util ento-rel-pose ento-pose Eigen
)

set(LIBRARIES_test_upright_three_pt
  ento_util ento-rel-pose ento-pose Eigen
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
