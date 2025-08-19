# OSQP static library for robofly MPC
# This is part of the ento-control module, not a separate project

# Embedded OSQP mode
set(OSQP_EMBEDDED_MODE "1")

# Collect source files
file(GLOB OSQP_SOURCES src/*.c)

# Add the generated workspace (API is now header-only)
list(APPEND OSQP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/roboflyworkspace.c)

# Create the static library with unique name
add_library(robofly_osqpstatic STATIC ${OSQP_SOURCES})

# Position-independent code for better compatibility
set_property(TARGET robofly_osqpstatic PROPERTY POSITION_INDEPENDENT_CODE ON)

# Set include directories
target_include_directories(robofly_osqpstatic PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/public 
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/private 
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create the RoboFly MPC interface library
add_library(robofly_mpc INTERFACE)

# Set include directories for the interface
target_include_directories(robofly_mpc INTERFACE 
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/public
)

# Link against the OSQP static library
target_link_libraries(robofly_mpc INTERFACE robofly_osqpstatic)

# Export the include directories
set(ROBOFLY_MPC_INCLUDE_DIRS 
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/public
  PARENT_SCOPE
)

# Make this library available to the parent scope
set(ROBOFLY_MPC_LIBRARIES robofly_mpc robofly_osqpstatic PARENT_SCOPE)
