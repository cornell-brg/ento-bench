cmake_minimum_required(VERSION 3.16)

# Core failure tracking implementation
add_library(ento-attitude-failure-core
  fp_overflow_tracker.cc
)

target_include_directories(ento-attitude-failure-core
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ento-attitude-failure-core
  PUBLIC
  ento_util
) 