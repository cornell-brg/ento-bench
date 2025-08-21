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

# Add uprightmpc2 sources directly to ento-control to avoid VFP ABI mismatch
target_sources(ento-control INTERFACE
  # OSQP sources
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/auxil.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/error.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/kkt.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/lin_alg.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/osqp.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/proj.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/qdldl.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/qdldl_interface.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/scaling.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/util.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/workspace.c
  # UprightMPC2 controller sources
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/uprightmpc2.c
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/funapprox.c
  # Eigen matrix multiplication wrapper
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2/py/ematmult.cpp
  # RoboBeeMPC wrapper
  ${CMAKE_CURRENT_SOURCE_DIR}/robobee_mpc.cc
)

# Add uprightmpc2 include directory
target_include_directories(ento-control INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/external/uprightmpc2
)

# =============================================================================
# RoboBee MPC (OSQP Generated)
# =============================================================================

# Add the RoboBee MPC code as a library
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/generated/robobee_mpc)

# Include the RoboBee MPC library in the interface
#target_include_directories(ento-control INTERFACE ${ROBOBEE_MPC_INCLUDE_DIRS})
#target_link_libraries(ento-control INTERFACE ${ROBOBEE_MPC_LIBRARIES})

# =============================================================================
# RoboFly MPC (OSQP Generated)
# =============================================================================

# Add the RoboFly MPC code as a library
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/generated/robofly_mpc)

# Include the RoboFly MPC library in the interface
#target_include_directories(ento-control INTERFACE ${ROBOFLY_MPC_INCLUDE_DIRS})
#target_link_libraries(ento-control INTERFACE ${ROBOFLY_MPC_LIBRARIES})

# =============================================================================
# Adaptive Controller (Simulink Generated)
# =============================================================================

# Add the Simulink-generated code directly to the main ento-control interface
target_sources(ento-control INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/robobee_adaptive_controller/integrated_controller.c
  ${CMAKE_CURRENT_SOURCE_DIR}/robobee_adaptive_controller/integrated_controller_data.c
)

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

include(CTest)

set(TEST_FILES
  test_lqr.cc
  test_control_benchmarks.cc
)

set(LIBRARIES_test_lqr
  ento_util ento_math ento-control
)

set(LIBRARIES_test_control_benchmarks
  ento_util ento_math ento-control
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