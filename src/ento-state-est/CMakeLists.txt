cmake_minimum_required(VERSION 3.16)
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

# add_subdirectory(factor-graph-chain)
# add_subdirectory(robofly-ekf)
# add_subdirectory(tiny-ekf)

add_library(ento-state-est
  INTERFACE
)

add_subdirectory(attitude-est)
#target_link_libraries(ento-state-est
  #PRIVATE
  #Eigen
  #)
