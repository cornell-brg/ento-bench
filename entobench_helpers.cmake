# This file defines CMake helper functions

include(CMakeParseArguments)

function(add_arm_semihosting_executable TARGET_NAME)
  # Extract source files and libs from arguments
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES" ${ARGN})
  
  add_executable(${TARGET_NAME} ${ARG_SOURCES})

  target_link_options(${TARGET_NAME}
    PRIVATE
    "--specs=rdimon.specs"
    "-lc"
    "-lrdimon"
  )

  # Include Eigen if it's one of the libraries
  if("Eigen" IN_LIST ARG_LIBRARIES)
    target_include_directories(${TARGET_NAME} PRIVATE ${EIGEN_DIR})
    list(REMOVE_ITEM ARG_LIBRARIES Eigen)  # Remove Eigen from the libraries to avoid linking it
  endif()

  message("[ARM semihosting build] Libs to link for ${TARGET_NAME}: ${ARG_LIBRARIES}")
  message(STATUS "Linker Options: ${CMAKE_EXE_LINKER_FLAGS}")
  target_link_libraries(${TARGET_NAME}
    PUBLIC
    ${ARG_LIBRARIES}
    mcu-util
  )
endfunction()


function(add_arm_baremetal_gem5_se_executable TARGET_NAME)
  # Extract source files and libs from arguments
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES" ${ARGN})
  
  add_executable(${TARGET_NAME} ${ARG_SOURCES})

  target_link_options(${TARGET_NAME}
    PRIVATE
    -L$ENV{ARM_NONE_EABI_LIB}/thumb/v7e-m+fp/hard #Link hard float lib
    -lc_nano
    -nostartfiles
    --data-sections
    -Xlinker -T${STARTUP_DIR}/boot.ld
    -static # gem5 SE mode needs a static binary
    -Wl,--undefined,_printf_float
    -Wl,--undefined,_scanf_float
  )

  message("[ARM gem5-SE build] Libs for ${TARGET_NAME}: ${ARG_LIBRARIES}")
  target_link_libraries(${TARGET_NAME}
    PUBLIC
    ${ARG_LIBRARIES}
  )

  if("Eigen" IN_LIST ARG_LIBRARIES)
    target_include_directories(${TARGET_NAME} PRIVATE ${EIGEN_DIR})
    list(REMOVE_ITEM ARG_LIBRARIES Eigen)  # Remove Eigen from the libraries to avoid linking it
  endif()

  target_link_directories(${TARGET_NAME}
    PRIVATE
    ${CMAKE_BINARY_DIR}/src/startup/CMakeFiles/startup_lib.dir
  )

endfunction()

function(add_non_arm_executable TARGET_NAME)
  cmake_parse_arguments(ARG "" "EXCLUDE" "SOURCES;LIBRARIES" ${ARGN})

  if(ARG_EXCLUDE)
    add_executable(${TARGET_NAME} EXCLUDE_FROM_ALL ${ARG_SOURCES})
  else()
    add_executable(${TARGET_NAME} ${ARG_SOURCES})
  endif()

  message(STATUS "Building ${TARGET_NAME} with the following libs/includes: ${ARG_LIBRARIES}")
  
  if("Eigen" IN_LIST ARG_LIBRARIES)
    message(STATUS "Eigen in libraries list. Adding as include dir...")
    target_include_directories(${TARGET_NAME} PRIVATE ${EIGEN_DIR})
    list(REMOVE_ITEM ARG_LIBRARIES Eigen)  # Remove Eigen from the libraries to avoid linking it
  else()
    message(STATUS "Eigen not in libraries list: ${ARG_LIBRARIES}")
  endif()

  message(STATUS "linking ${ARG_LIBRARIES} to ${TARGET_NAME}")
  target_link_libraries(${TARGET_NAME}
    PRIVATE
    ${ARG_LIBRARIES}
  )
endfunction()

function(add_benchmark TARGET_NAME)
  cmake_parse_arguments(ARG "" "EXCLUDE" "SOURCES;LIBRARIES" ${ARGN})

  # Determine the build type (semihosting, gem5-SE, or native)
  if(STM32_BUILD)
    add_arm_semihosting_executable(${TARGET_NAME}
      SOURCES ${SOURCE_FILE}
      LIBRARIES ${ARG_LIBRARIES}
    )
  elseif(GEM5_BUILD)
    add_arm_baremetal_gem5_se_executable(${TARGET_NAME}
      SOURCES ${SOURCE_FILE}
      LIBRARIES ${ARG_LIBRARIES}
    )
  else()
    add_non_arm_executable(${TARGET_NAME}
      SOURCES ${SOURCE_FILE}
      LIBRARIES ${ARG_LIBRARIES}
    )
  endif()

  # Set output directory based on the target's category
  get_filename_component(BENCHMARK_PATH ${SOURCE_FILE} DIRECTORY)
  if(BENCHMARK_PATH MATCHES "kernels")
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/kernels/bin)
  elseif(BENCHMARK_PATH MATCHES "microbenchmarks")
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/microbenchmarks/bin)
  elseif(BENCHMARK_PATH MATCHES "scenarios")
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/scenarios/bin)
  elseif(BENCHMARK_PATH MATCHES "workloads")
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/workloads/bin)
  else()
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/other/bin)
  endif()
endfunction()


# Helper function to create custom targets for flashing and debugging STM32 binaries using OpenOCD
function(add_stm32_flash_and_debug_targets target_name)
  # Flash target
  add_custom_target(stm32-flash-${target_name}
    COMMAND openocd
      -f ${OPENOCD_INTERFACE}
      -f ${CMAKE_SOURCE_DIR}/openocd/${STM32_DEVICE}
      -c "init"
      -c "reset halt"
      -c "arm semihosting enable"
      -c "program $<TARGET_FILE:${target_name}> verify reset"
      -c "arm semihosting_cmdline --shutdown-on-exit"
    DEPENDS ${target_name}
    COMMENT "Flashing ${target_name} to target (${STM32_DEVICE})"
  )

  # Debug target
  add_custom_target(stm32-debug-${target_name}
    COMMAND openocd
      -f ${OPENOCD_INTERFACE}
      -f ${CMAKE_SOURCE_DIR}/openocd/${STM32_DEVICE}
      -c "init"
      -c "reset halt"
      -c "arm semihosting enable"
      -c "program $<TARGET_FILE:${target_name}> verify reset halt"
    COMMAND arm-none-eabi-gdb
      -ex "target remote localhost:3333"
      -ex "monitor reset halt"
      -ex "load" $<TARGET_FILE:${target_name}>
    DEPENDS ${target_name}
    COMMENT "Starting debug session for ${target_name} on ${STM32_DEVICE}"
  )
endfunction()

function(add_ento_test TARGET_NAME)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES" ${ARGN})

  if (NOT GEM5_ARMV7E-M_BUILD AND NOT STM32_BUILD)
    message(STATUS "Building ${TEST_BIN} for native")
    add_non_arm_executable(${TARGET_NAME}
      EXCLUDE TRUE
      SOURCES ${ARG_SOURCES}
      LIBRARIES ${ARG_LIBRARIES})
    add_dependencies(check ${TARGET_NAME})
    add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
  endif()
endfunction()
