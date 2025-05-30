include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

# Main interface library
add_library(ento-control INTERFACE)

# Add includes to ento-control
target_include_directories(ento-control INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}  # For ento-control headers
  ${EIGEN_DIR}  # For Eigen headers
)

# =============================================================================
# UprightMPC2 Template Controller (RoboBee)
# =============================================================================

# Add the uprightmpc2 template controller library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2)

# Include the uprightmpc2 library in the interface
target_include_directories(ento-control INTERFACE ${UPRIGHTMPC2_INCLUDE_DIRS})
target_link_libraries(ento-control INTERFACE ${UPRIGHTMPC2_LIBRARIES})

# Add the RoboBeeMPC wrapper
add_library(robobee_mpc_wrapper
  ${CMAKE_CURRENT_SOURCE_DIR}/robobee_mpc.cc
)

target_include_directories(robobee_mpc_wrapper PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${UPRIGHTMPC2_INCLUDE_DIRS}
)

target_link_libraries(robobee_mpc_wrapper PUBLIC
  ${UPRIGHTMPC2_LIBRARIES}
  ento_util
)

# Link the RoboBeeMPC wrapper to the main interface
target_link_libraries(ento-control INTERFACE robobee_mpc_wrapper)

# =============================================================================
# RoboBee MPC (OSQP Generated)
# =============================================================================

# Add the RoboBee MPC code as a library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/generated/robobee_mpc)

# Include the RoboBee MPC library in the interface
target_include_directories(ento-control INTERFACE ${ROBOBEE_MPC_INCLUDE_DIRS})
target_link_libraries(ento-control INTERFACE ${ROBOBEE_MPC_LIBRARIES})

# =============================================================================
# RoboFly MPC (OSQP Generated)
# =============================================================================

# Add the RoboFly MPC code as a library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/generated/robofly_mpc)

# Include the RoboFly MPC library in the interface
target_include_directories(ento-control INTERFACE ${ROBOFLY_MPC_INCLUDE_DIRS})
target_link_libraries(ento-control INTERFACE ${ROBOFLY_MPC_LIBRARIES})

# =============================================================================
# Adaptive Controller (Simulink Generated)
# =============================================================================

# Add the Simulink-generated code as a library
add_library(adaptive_controller 
  ${CMAKE_CURRENT_SOURCE_DIR}/robobee_adaptive_controller/integrated_controller.c
  ${CMAKE_CURRENT_SOURCE_DIR}/robobee_adaptive_controller/integrated_controller_data.c
)

target_include_directories(adaptive_controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link the adaptive controller to the main interface
target_link_libraries(ento-control INTERFACE adaptive_controller)

# =============================================================================
# Geometric Controller (FDCL-inspired, heavily templated)
# =============================================================================

# Add geometric controller sources directly to ento-control
target_sources(ento-control INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/geometric_controller.cc
)

# Add dependencies for geometric controller
target_link_libraries(ento-control INTERFACE
  ento_util
  ento_math
)

# =============================================================================
# Testing
# =============================================================================

# Find jsoncpp using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

include(CTest)

set(TEST_FILES
  test_tiny_mpc.cc
  test_lqr.cc
  test_robofly_tinympc.cc
  test_adaptive_controller.cc
  test_adaptive_controller_trajectory.cc
  test_robobee_mpc.cc
  test_robobee3d_mpc.cc
  test_osqp_mpc.cc
  test_template_mpc_opt_control.cc
  test_debug_comparison.cc
  test_controller_validation.cc
  test_geometric_controller_control_problem.cc
  test_trajectory_benchmark.cc
  test_position_control_comparison.cc
  test_geometric_controller_reference.cc
)

set(LIBRARIES_test_tiny_mpc
  ento_util ento_math ento-control
)

set(LIBRARIES_test_lqr
  ento_util ento_math ento-control
)

set(LIBRARIES_test_robofly_tinympc
  ento_util ento_math ento-control
)

set(LIBRARIES_test_adaptive_controller
  ento_util ento_math ento-control adaptive_controller
)

set(LIBRARIES_test_adaptive_controller_trajectory
  ento_util ento_math ento-control adaptive_controller
)

set(LIBRARIES_test_robobee_mpc
  ento_util ento_math ento-control ${ROBOBEE_MPC_LIBRARIES}
)

set(LIBRARIES_test_robobee3d_mpc
  ento_util ento_math ento-control
)

set(LIBRARIES_test_osqp_mpc
  ento_util ento_math ento-control ${ROBOFLY_MPC_LIBRARIES}
)

set(LIBRARIES_test_template_mpc_opt_control
  ento_util ento_math ento-control robobee_mpc_wrapper ${UPRIGHTMPC2_LIBRARIES}
)


set(LIBRARIES_test_debug_comparison
  ento_util ento_math ento-control ${JSONCPP_LIBRARIES}
)

set(LIBRARIES_test_controller_validation
  ento_util ento_math ento-control
)

set(LIBRARIES_test_geometric_controller_control_problem
  ento_util ento_math ento-control
)

set(LIBRARIES_test_trajectory_benchmark
  ento_util ento_math ento-control
)

set(LIBRARIES_test_position_control_comparison
  ento_util ento_math ento-control
)

set(LIBRARIES_test_geometric_controller_reference
  ento_util ento_math ento-control
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
  
  # Special handling for debug comparison test to add jsoncpp includes
  if(TEST_BIN STREQUAL "test_debug_comparison")
    target_include_directories(${TEST_BIN} PRIVATE ${JSONCPP_INCLUDE_DIRS})
    target_link_directories(${TEST_BIN} PRIVATE ${JSONCPP_LIBRARY_DIRS})
  endif()
endforeach()

# Real data benchmark test
add_executable(test_real_data_benchmark test/test_real_data_benchmark.cc)
target_link_libraries(test_real_data_benchmark ento-control ento-math ento-util)
target_include_directories(test_real_data_benchmark PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Test: Dynamics comparison
add_executable(test_dynamics_comparison test/test_dynamics_comparison.cc)
target_link_libraries(test_dynamics_comparison 
  ento_math 
  ento_util
  ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(test_dynamics_comparison PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../ento-math
  ${CMAKE_CURRENT_SOURCE_DIR}/../ento-util
)

