# RoboBee MPC library
# This is part of the ento-control module, not a separate project

# Embedded OSQP mode
set(OSQP_EMBEDDED_MODE "1")

# Collect source files
file(GLOB OSQP_SOURCES src/*.c)

# Add our custom API implementation
list(APPEND OSQP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/robobee_api.c)

# Create the static library
add_library(osqpstatic STATIC ${OSQP_SOURCES})

# Position-independent code for better compatibility
set_property(TARGET osqpstatic PROPERTY POSITION_INDEPENDENT_CODE ON)

# Set include directories for OSQP
target_include_directories(osqpstatic PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/public 
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/private 
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create the RoboBee MPC library
add_library(robobee_mpc INTERFACE)

# Set include directories for the interface
target_include_directories(robobee_mpc INTERFACE 
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/public
)

# Link against the OSQP static library
target_link_libraries(robobee_mpc INTERFACE osqpstatic)

# Export the include directories
set(ROBOBEE_MPC_INCLUDE_DIRS 
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/public
  PARENT_SCOPE
)

# Make this library available to the parent scope
set(ROBOBEE_MPC_LIBRARIES robobee_mpc osqpstatic PARENT_SCOPE) 