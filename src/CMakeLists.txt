#add_subdirectory(linalg)
#target_include_directories(linalg PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
#)

add_subdirectory(image_io)
target_include_directories(image_io
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

add_subdirectory(ento-mcu)
if(STM32_BUILD)
  target_include_directories(mcu-util
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  )
elseif(GEM5_ARMV7E-M_BUILD)
  add_subdirectory(startup)
  target_include_directories(startup
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  )
  add_subdirectory(sys)
  target_include_directories(sys
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  )
  #target_include_directories(util
  #  INTERFACE
  #  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  #)
else()
  #target_include_directories(util
  #  INTERFACE
  #  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  #)
endif()
# Bench is an interface for now, no source files, header only

add_subdirectory(ento-bench)
target_include_directories(bench
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

add_subdirectory(ento-math)
target_include_directories(ento_math
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${EIGEN_DIR}
)

add_subdirectory(ento-feature2d)

add_subdirectory(ento-util)
target_include_directories(ento_util
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

add_subdirectory(control)
add_subdirectory(planning)
add_subdirectory(state_est)

add_subdirectory(ento-pose)
target_include_directories(ento_abs_pose
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${EIGEN_DIR}
)

add_subdirectory(ento-state-est)
