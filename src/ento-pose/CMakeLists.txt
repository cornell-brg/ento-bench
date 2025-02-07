add_subdirectory(abs-pose)
add_subdirectory(rel-pose)

add_library(ento-pose
  INTERFACE
)

add_non_arm_executable(
  generate_pose_data
  SOURCES data_gen.cc
  LIBRARIES ento-pose ento_util
)

# TODO: Uncomment once Robust pose data generation
#  supports new Problem interfaces and refactored prob/data_gen.h
# add_non_arm_executable(
#   generate_robust_pose_data
#   SOURCES robust_data_gen.cc
#   LIBRARIES ento-pose ento_util
#)

target_compile_definitions(
  generate_pose_data
  PRIVATE
  -DNATIVE
)

