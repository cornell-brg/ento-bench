add_subdirectory(abs-pose)
add_subdirectory(rel-pose)
add_subdirectory(robust-est)

add_library(ento-pose
  INTERFACE
)

#add_non_arm_executable(
#  generate_pose_data
#  SOURCES data_gen.cc
#  LIBRARIES ento-pose ento_util
#)

#target_compile_definitions(
#  generate_pose_data
#  PRIVATE
#  -DNATIVE
#)

