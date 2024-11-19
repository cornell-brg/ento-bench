add_subdirectory(abs-pose)
add_subdirectory(rel-pose)

add_library(ento-pose
  INTERFACE
)

set(NATIVE_SRC_FILES
  data_gen.cc
)

add_non_arm_executable(
  generate_pose_data
  SOURCES ${NATIVE_SRC_FILES}
  LIBRARIES ento-pose
)

target_compile_definitions(
  generate_pose_data
  PRIVATE
  -DNATIVE
)

