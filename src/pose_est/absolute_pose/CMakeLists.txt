cmake_minimum_required(VERSION 3.16)

set(ABS_POSE_SRC
  dlt.cc
  p3p.cc
  p4p.cc
)

add_library(pose-est, STATIC
  ${SRC_FILES}
)

target_include_directories(pose-est PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_DIR}
)


