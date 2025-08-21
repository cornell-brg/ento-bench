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
# Data Generation
#=========================================

# add_executable(generate_minimal_solver_data_rel
#   test/generate_minimal_solver_data.cc
# )
# 
# target_include_directories(generate_minimal_solver_data_rel
#   PRIVATE
#   ${EIGEN_DIR}
# )
# target_link_libraries(generate_minimal_solver_data_rel
#   ento_util ento-rel-pose ento-pose
# )

# Enhanced data generation tool with comprehensive statistics and unified noise model
add_executable(generate_minimal_solver_data_rel_enhanced
  test/generate_minimal_solver_data_enhanced.cc
)

target_include_directories(generate_minimal_solver_data_rel_enhanced
  PRIVATE
  ${EIGEN_DIR}
)

target_link_libraries(generate_minimal_solver_data_rel_enhanced
  ento_util ento-rel-pose ento-pose
)

# Focused robust relative pose data generator and tester
add_executable(generate_robust_relative_pose_data
  test/generate_robust_relative_pose_data.cc
)

target_include_directories(generate_robust_relative_pose_data
  PRIVATE
  ${EIGEN_DIR}
)

target_link_libraries(generate_robust_relative_pose_data
  ento_util ento-rel-pose ento-pose
)

# Standalone upright planar 3pt test to compare different data generation methods
add_executable(test_upright_planar_3pt_standalone
  test/test_upright_planar_3pt_standalone.cc
)

target_include_directories(test_upright_planar_3pt_standalone
  PRIVATE
  ${EIGEN_DIR}
)

target_link_libraries(test_upright_planar_3pt_standalone
  ento_util ento-rel-pose ento-pose
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

add_executable(bench_rel_min_solvers
  test/bench_rel_min_solvers.cc
)

target_include_directories(bench_rel_min_solvers
  PRIVATE
  ${EIGEN_DIR}
)

target_link_libraries(bench_rel_min_solvers
  ento_util ento-rel-pose ento-pose
)
