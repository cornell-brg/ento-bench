# UprightMPC2 Template Controller Library
# Based on external/robobee3d/template/uprightmpc2

cmake_minimum_required(VERSION 3.5.0)

# Collect C source files for OSQP
file(GLOB OSQP_SOURCES 
  auxil.c
  error.c
  kkt.c
  lin_alg.c
  osqp.c
  proj.c
  qdldl.c
  qdldl_interface.c
  scaling.c
  util.c
  workspace.c
)

# Add the uprightmpc2 controller and function approximation
list(APPEND UPRIGHTMPC2_SOURCES
  uprightmpc2.c
  funapprox.c
)

# Add the Eigen matrix multiplication wrapper
list(APPEND UPRIGHTMPC2_SOURCES
  py/ematmult.cpp
)

# Create the static library
add_library(uprightmpc2 STATIC ${OSQP_SOURCES} ${UPRIGHTMPC2_SOURCES})

# Position-independent code for better compatibility
set_property(TARGET uprightmpc2 PROPERTY POSITION_INDEPENDENT_CODE ON)

# Include directories
target_include_directories(uprightmpc2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_DIR}
)

# Export variables for parent scope
set(UPRIGHTMPC2_INCLUDE_DIRS 
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_DIR}
  PARENT_SCOPE
)

set(UPRIGHTMPC2_LIBRARIES uprightmpc2 PARENT_SCOPE)
