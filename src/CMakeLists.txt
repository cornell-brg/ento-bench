add_subdirectory(linalg)
target_include_directories(linalg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

add_subdirectory(mcu-util)
if(STM32_BUILD)
  target_include_directories(mcu-util PUBLIC
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

add_subdirectory(workload/microbench/assembly)
target_include_directories(assembly
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/workload/microbench/assembly>
)

# Bench is an interface for now, no source files, header only
add_subdirectory(bench)
target_include_directories(bench
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

