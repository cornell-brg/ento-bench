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

  target_compile_definitions(${TARGET_NAME}
    PRIVATE
    SEMIHOSTING
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
    ento-mcu-semihosted
  )
endfunction()

function(add_arm_executable TARGET_NAME)
  # Extract source files and libs from arguments
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES" ${ARGN})
  
  add_executable(${TARGET_NAME} ${ARG_SOURCES})

  target_link_options(${TARGET_NAME}
    PRIVATE
    "--specs=nano.specs"
    "--specs=nosys.specs"
  )

  # Include Eigen if it's one of the libraries
  if("Eigen" IN_LIST ARG_LIBRARIES)
    target_include_directories(${TARGET_NAME} PRIVATE ${EIGEN_DIR})
   list(REMOVE_ITEM ARG_LIBRARIES Eigen)  # Remove Eigen from the libraries to avoid linking it
  endif()

  message(STATUS "[ARM non semihosting build] Libs to link for ${TARGET_NAME}: ${ARG_LIBRARIES}")
  message(STATUS "Linker Options: ${CMAKE_EXE_LINKER_FLAGS}")
  target_link_libraries(${TARGET_NAME}
    PUBLIC
    ${ARG_LIBRARIES}
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
      SOURCES ${ARG_SOURCES}
      LIBRARIES ${ARG_LIBRARIES}
    )
    add_arm_executable(${TARGET_NAME}-no-semihosting
      SOURCES ${SOURCE_FILE}
      LIBRARIES ${ARG_LIBRARIES})

  elseif(GEM5_BUILD)
    add_arm_baremetal_gem5_se_executable(${TARGET_NAME}
      SOURCES ${ARG_SOURCES}
      LIBRARIES ${ARG_LIBRARIES}
    )
  else()
    #add_non_arm_executable(${TARGET_NAME}
    #  SOURCES ${ARG_SOURCES}
    #  LIBRARIES ${ARG_LIBRARIES}
    #)
  endif()

  # Set output directory based on the target's category
  # get_filename_component(BENCHMARK_PATH ${SOURCE_FILE} DIRECTORY)
  #if(BENCHMARK_PATH MATCHES "kernels")
  #  set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/kernels/bin)
  #elseif(BENCHMARK_PATH MATCHES "ubmarks")
  #  set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/microbenchmarks/bin)
  #elseif(BENCHMARK_PATH MATCHES "scenarios")
  #  set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/scenarios/bin)
  #elseif(BENCHMARK_PATH MATCHES "workloads")
  #  set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/workloads/bin)
    #else()
    #set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks/other/bin)
    #endif()
endfunction()


# Helper function to create custom targets for flashing and debugging STM32 binaries using OpenOCD
function(add_stm32_target target_name)
  # Flash target
  get_target_property(TARGET_BUILD_DIR ${target_name} BINARY_DIR)
  file(RELATIVE_PATH RELATIVE_TARGET_BUILD_DIR ${CMAKE_BINARY_DIR} ${TARGET_BUILD_DIR})

  add_custom_target(stm32-flash-${target_name}-semihosted
    COMMAND openocd
      -f ${OPENOCD_INTERFACE}
      -f ${CMAKE_SOURCE_DIR}/openocd/${OPENOCD_CFG}
      -c "init"
      -c "reset halt"
      -c "arm semihosting enable"
      -c "program bin/${target_name}.elf verify"
      -c "reset run"
    DEPENDS ${target_name}
    COMMENT "Flashing ${target_name} to target (${OPENOCD_CFG})"
  )

  # Debug target. User must open up another terminal and use arm-none-eabi-gdb/gdb/lldb...
  add_custom_target(stm32-debug-${target_name}-semihosted
    COMMAND openocd
      -f ${OPENOCD_INTERFACE}
      -f ${CMAKE_SOURCE_DIR}/openocd/${OPENOCD_CFG}
      -c "init"
      -c "reset halt"
      -c "arm semihosting_cmdline '12'" # 12 to see how strings are passed by semihosting
      -c "arm semihosting enable"
      -c "program bin/${target_name}.elf verify"
      -c "reset halt"
    DEPENDS ${target_name}
    COMMENT "Starting debug session for ${target_name} on ${OPENOCD_CFG}"
  )

endfunction()

function(add_stm32_no_semihosting_target target_name)
  # Flash target without semihosting
  add_custom_target(stm32-flash-${target_name}
    COMMAND openocd
      -f ${OPENOCD_INTERFACE}
      -f ${CMAKE_SOURCE_DIR}/openocd/${OPENOCD_CFG}
      -c "init"
      -c "reset halt"
      -c "program $<TARGET_FILE:${target_name}> verify"
      -c "reset run"
      -c "exit"
    DEPENDS ${target_name}
    COMMENT "Flashing ${target_name} to target (${OPENOCD_CFG}) without semihosting"
  )

  # Debug target without semihosting
  add_custom_target(stm32-debug-${target_name}
    COMMAND openocd
      -f ${OPENOCD_INTERFACE}
      -f ${CMAKE_SOURCE_DIR}/openocd/${OPENOCD_CFG}
      -c "init"
      -c "reset halt"
      -c "program $<TARGET_FILE:${target_name}> verify"
      -c "reset halt"
    DEPENDS ${target_name}
    COMMENT "Starting debug session for ${target_name} on ${OPENOCD_CFG} without semihosting"
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

function(add_stm32_targets target_list)
  message(STATUS "Inside add_stm32_targets: ${target_list}")
  foreach(target_name IN LISTS target_list)
    MESSAGE(INFO "Added stm32 target: ${target_name}")
    add_stm32_target(${target_name})
    #add_stm32_no_semihosting_target(${target_name})
  endforeach()
endfunction()

## Get all properties that cmake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)
## Convert command output into a CMake list
STRING(REGEX REPLACE ";" "\\\\;" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
STRING(REGEX REPLACE "\n" ";" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")

list(REMOVE_DUPLICATES CMAKE_PROPERTY_LIST)

function(print_target_properties tgt)
    if(NOT TARGET ${tgt})
      message("There is no target named '${tgt}'")
      return()
    endif()

    foreach (prop ${CMAKE_PROPERTY_LIST})
        string(REPLACE "<CONFIG>" "${CMAKE_BUILD_TYPE}" prop ${prop})
        get_target_property(propval ${tgt} ${prop})
        if (propval)
            message ("${tgt} ${prop} = ${propval}")
        endif()
    endforeach(prop)
endfunction(print_target_properties)

# Helper function to build all benchmarks in a specific subfolder with build counting
function(add_subfolder_benchmark_target SUBFOLDER_NAME TARGET_LIST)
  set(SUBFOLDER_TARGET "build-${SUBFOLDER_NAME}-benchmarks")
  
  # Create a custom target that builds all benchmarks in the subfolder
  add_custom_target(${SUBFOLDER_TARGET}
    COMMAND ${CMAKE_COMMAND} -E echo "Building ${SUBFOLDER_NAME} benchmarks..."
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${TARGET_LIST} 2>&1 | 
            ${CMAKE_COMMAND} -E env bash -c "
              tee build_log.tmp | 
              grep -E '(\\[[ 0-9]*%\\]|Built target|Error:|Warning:)' |
              tee filtered_output.tmp &&
              BUILT_COUNT=\$$(grep 'Built target' filtered_output.tmp | wc -l) &&
              ERROR_COUNT=\$$(grep 'Error:' filtered_output.tmp | wc -l) &&
              WARNING_COUNT=\$$(grep 'Warning:' filtered_output.tmp | wc -l) &&
              echo '' &&
              echo '=== BUILD SUMMARY FOR ${SUBFOLDER_NAME} ===' &&
              echo \"Successfully built: \$$BUILT_COUNT targets\" &&
              echo \"Errors: \$$ERROR_COUNT\" &&
              echo \"Warnings: \$$WARNING_COUNT\" &&
              rm -f build_log.tmp filtered_output.tmp
            "
    COMMENT "Building all ${SUBFOLDER_NAME} benchmarks with summary"
    VERBATIM
  )
  
  # Make the subfolder target depend on all the individual benchmark targets
  foreach(target IN LISTS TARGET_LIST)
    if(TARGET ${target})
      add_dependencies(${SUBFOLDER_TARGET} ${target})
    endif()
  endforeach()
endfunction()

# Generic function to create benchmark group targets
# Usage: add_benchmark_group_target("attitude-est" LIST_OF_TARGETS)
# Creates target: bench-attitude-est
function(add_benchmark_group_target GROUP_NAME)
  # Parse the remaining arguments as the target list
  set(TARGET_LIST ${ARGN})
  
  # Create the benchmark group target name
  set(GROUP_TARGET "bench-${GROUP_NAME}")
  
  # Convert target list to space-separated string
  string(REPLACE ";" " " TARGET_LIST_STR "${TARGET_LIST}")
  
  # Path to the build progress script
  set(PROGRESS_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/build_with_progress.sh")
  
  # Check if script exists
  if(NOT EXISTS ${PROGRESS_SCRIPT})
    message(WARNING "Build progress script not found at: ${PROGRESS_SCRIPT}")
    message(WARNING "Falling back to simple build target")
    
    # Fallback to simple target
    add_custom_target(${GROUP_TARGET}
      COMMAND ${CMAKE_COMMAND} -E echo "🔨 Building ${GROUP_NAME} benchmarks..."
      COMMENT "Building all ${GROUP_NAME} benchmarks"
    )
  else()
    # Create a custom target that uses the external script for colored progress
    add_custom_target(${GROUP_TARGET}
      COMMAND bash ${PROGRESS_SCRIPT} "${GROUP_NAME}" "${TARGET_LIST_STR}" "${CMAKE_BINARY_DIR}"
      COMMENT "Building all ${GROUP_NAME} benchmarks with progress"
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      USES_TERMINAL
    )
  endif()
  
  # DON'T add dependencies - let the script handle building the targets
  # This allows 'make bench-attitude-est' to work properly
  
  # Print summary
  list(LENGTH TARGET_LIST TARGET_COUNT)
  message(STATUS "Created benchmark group '${GROUP_TARGET}' with ${TARGET_COUNT} targets")
  if(EXISTS ${PROGRESS_SCRIPT})
    message(STATUS "Using progress script: ${PROGRESS_SCRIPT}")
  endif()
endfunction()



