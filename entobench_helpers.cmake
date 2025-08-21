# This file defines CMake helper functions

include(CMakeParseArguments)

# =============================================================================
# CONFIG FILE PARSING FUNCTIONS
# =============================================================================

# Function to parse benchmark configuration from JSON file
# Usage: parse_benchmark_config_file("configs/benchmarks.json" "performance-group" OUTPUT_VAR)
function(parse_benchmark_config_file CONFIG_FILE GROUP_NAME OUTPUT_VAR)
  if(NOT EXISTS ${CONFIG_FILE})
    message(WARNING "Benchmark config file not found: ${CONFIG_FILE}")
    return()
  endif()
  
  # Read the JSON file
  file(READ ${CONFIG_FILE} JSON_CONTENT)
  
  # Check if group exists in JSON
  string(JSON GROUP_EXISTS ERROR_VARIABLE json_error GET ${JSON_CONTENT} ${GROUP_NAME})
  if(json_error)
    message(WARNING "Group '${GROUP_NAME}' not found in config file: ${CONFIG_FILE}")
    return()
  endif()
  
  # Parse group configuration
  set(CONFIG_ARGS "")
  
  # Helper macro to extract JSON values
  macro(extract_json_value PARAM_NAME CMAKE_VAR JSON_KEY)
    string(JSON ${CMAKE_VAR} ERROR_VARIABLE json_error GET ${JSON_CONTENT} ${GROUP_NAME} ${JSON_KEY})
    if(NOT json_error AND DEFINED ${CMAKE_VAR})
      list(APPEND CONFIG_ARGS ${PARAM_NAME} ${${CMAKE_VAR}})
    endif()
  endmacro()
  
  # Helper macro for boolean flags
  macro(extract_json_bool_flag FLAG_NAME JSON_KEY)
    string(JSON BOOL_VALUE ERROR_VARIABLE json_error GET ${JSON_CONTENT} ${GROUP_NAME} ${JSON_KEY})
    if(NOT json_error AND BOOL_VALUE)
      list(APPEND CONFIG_ARGS ${FLAG_NAME})
    endif()
  endmacro()
  
  # Extract configuration parameters
  extract_json_value("REPS" REPS_VALUE "reps")
  extract_json_value("INNER_REPS" INNER_REPS_VALUE "inner_reps") 
  extract_json_value("VERBOSITY" VERBOSITY_VALUE "verbosity")
  extract_json_value("MAX_PROBLEMS" MAX_PROBLEMS_VALUE "max_problems")
  extract_json_bool_flag("DO_WARMUP" "do_warmup")
  extract_json_bool_flag("ENABLE_CACHES" "enable_caches")
  extract_json_bool_flag("ENABLE_VECTORIZATION" "enable_vectorization")
  
  # Set output variable
  set(${OUTPUT_VAR} ${CONFIG_ARGS} PARENT_SCOPE)
  
  # Print what was loaded
  if(CONFIG_ARGS)
    list(LENGTH CONFIG_ARGS ARG_COUNT)
    math(EXPR PAIRS "${ARG_COUNT} / 2")
    message(VERBOSE "Loaded config for '${GROUP_NAME}' (${PAIRS} parameters)")
  endif()
endfunction()

# NEW: Function to parse target-specific configuration from JSON file
# Usage: parse_target_config_file("configs/benchmarks.json" "bench-madgwick-float-imu" OUTPUT_VAR)
function(parse_target_config_file CONFIG_FILE TARGET_NAME OUTPUT_VAR)
  if(NOT EXISTS ${CONFIG_FILE})
    message(WARNING "Benchmark config file not found: ${CONFIG_FILE}")
    return()
  endif()
  
  # Read the JSON file
  file(READ ${CONFIG_FILE} JSON_CONTENT)
  
  # Check if targets section exists
  string(JSON TARGETS_EXISTS ERROR_VARIABLE json_error GET ${JSON_CONTENT} "targets")
  if(json_error)
    # No targets section, return empty
    set(${OUTPUT_VAR} "" PARENT_SCOPE)
    return()
  endif()
  
  # Check if specific target exists in targets section
  string(JSON TARGET_EXISTS ERROR_VARIABLE json_error GET ${JSON_CONTENT} "targets" ${TARGET_NAME})
  if(json_error)
    # Target not found, return empty
    set(${OUTPUT_VAR} "" PARENT_SCOPE)
    return()
  endif()
  
  # Parse target-specific configuration
  set(CONFIG_ARGS "")
  
  # Helper macro to extract JSON values for targets
  macro(extract_target_json_value PARAM_NAME CMAKE_VAR JSON_KEY)
    string(JSON ${CMAKE_VAR} ERROR_VARIABLE json_error GET ${JSON_CONTENT} "targets" ${TARGET_NAME} ${JSON_KEY})
    if(NOT json_error AND DEFINED ${CMAKE_VAR})
      list(APPEND CONFIG_ARGS ${PARAM_NAME} ${${CMAKE_VAR}})
    endif()
  endmacro()
  
  # Helper macro for boolean flags for targets
  macro(extract_target_json_bool_flag FLAG_NAME JSON_KEY)
    string(JSON BOOL_VALUE ERROR_VARIABLE json_error GET ${JSON_CONTENT} "targets" ${TARGET_NAME} ${JSON_KEY})
    if(NOT json_error AND BOOL_VALUE)
      list(APPEND CONFIG_ARGS ${FLAG_NAME})
    endif()
  endmacro()
  
  # Extract target-specific configuration parameters
  extract_target_json_value("REPS" REPS_VALUE "reps")
  extract_target_json_value("INNER_REPS" INNER_REPS_VALUE "inner_reps") 
  extract_target_json_value("VERBOSITY" VERBOSITY_VALUE "verbosity")
  extract_target_json_value("MAX_PROBLEMS" MAX_PROBLEMS_VALUE "max_problems")
  extract_target_json_bool_flag("DO_WARMUP" "do_warmup")
  extract_target_json_bool_flag("ENABLE_CACHES" "enable_caches")
  extract_json_bool_flag("ENABLE_VECTORIZATION" "enable_vectorization")
  
  # Set output variable
  set(${OUTPUT_VAR} ${CONFIG_ARGS} PARENT_SCOPE)
  
  # Print what was loaded
  if(CONFIG_ARGS)
    list(LENGTH CONFIG_ARGS ARG_COUNT)
    math(EXPR PAIRS "${ARG_COUNT} / 2")
    message(VERBOSE "Loaded target-specific config for '${TARGET_NAME}' (${PAIRS} parameters)")
  endif()
endfunction()

# Function to configure benchmark group from config file
# Usage: configure_benchmark_group_from_file("performance-group" "configs/benchmarks.json" target1 target2 target3)
function(configure_benchmark_group_from_file GROUP_NAME CONFIG_FILE)
  set(TARGET_LIST ${ARGN})
  
  # Parse configuration from file
  parse_benchmark_config_file(${CONFIG_FILE} ${GROUP_NAME} CONFIG_ARGS)
  
  if(CONFIG_ARGS)
    # Apply configuration to all targets
    foreach(target_name IN LISTS TARGET_LIST)
      if(TARGET ${target_name})
        configure_benchmark_target(${target_name} ${CONFIG_ARGS})
      else()
        message(WARNING "Target ${target_name} does not exist in group ${GROUP_NAME}")
      endif()
    endforeach()
    
    # Create the group target
    add_benchmark_group_target(${GROUP_NAME} ${TARGET_LIST})
    
    message(VERBOSE "Configured group '${GROUP_NAME}' from ${CONFIG_FILE}")
  else()
    message(WARNING "No configuration found for group '${GROUP_NAME}' in ${CONFIG_FILE}")
  endif()
endfunction()

# Function to configure benchmark group with config file support and fallback
# Usage: add_configured_benchmark_group_from_file("performance-group" 
#          CONFIG_FILE "configs/benchmarks.json"
#          TARGETS target1 target2 target3
#          REPS 10  # fallback values
#          VERBOSITY 1)
function(add_configured_benchmark_group_from_file GROUP_NAME)
  cmake_parse_arguments(GROUP 
    "DO_WARMUP;ENABLE_CACHES" 
    "CONFIG_FILE;REPS;INNER_REPS;VERBOSITY;MAX_PROBLEMS" 
    "TARGETS" 
    ${ARGN}
  )
  
  if(NOT GROUP_TARGETS)
    message(FATAL_ERROR "add_configured_benchmark_group_from_file: TARGETS must be specified")
  endif()
  
  set(FINAL_CONFIG_ARGS "")
  
  # Try to load from config file first
  if(GROUP_CONFIG_FILE AND EXISTS ${GROUP_CONFIG_FILE})
    parse_benchmark_config_file(${GROUP_CONFIG_FILE} ${GROUP_NAME} FILE_CONFIG_ARGS)
    if(FILE_CONFIG_ARGS)
      set(FINAL_CONFIG_ARGS ${FILE_CONFIG_ARGS})
      message(VERBOSE "Using config file ${GROUP_CONFIG_FILE} for group '${GROUP_NAME}'")
    endif()
  endif()
  
  # If no config from file, use function parameters as fallback
  if(NOT FINAL_CONFIG_ARGS)
    if(DEFINED GROUP_REPS)
      list(APPEND FINAL_CONFIG_ARGS "REPS" "${GROUP_REPS}")
    endif()
    if(DEFINED GROUP_INNER_REPS)
      list(APPEND FINAL_CONFIG_ARGS "INNER_REPS" "${GROUP_INNER_REPS}")
    endif()
    if(DEFINED GROUP_VERBOSITY)
      list(APPEND FINAL_CONFIG_ARGS "VERBOSITY" "${GROUP_VERBOSITY}")
    endif()
    if(DEFINED GROUP_MAX_PROBLEMS)
      list(APPEND FINAL_CONFIG_ARGS "MAX_PROBLEMS" "${GROUP_MAX_PROBLEMS}")
    endif()
    if(GROUP_DO_WARMUP)
      list(APPEND FINAL_CONFIG_ARGS "DO_WARMUP")
    endif()
    if(GROUP_ENABLE_CACHES)
      list(APPEND FINAL_CONFIG_ARGS "ENABLE_CACHES")
    endif()
    
    if(FINAL_CONFIG_ARGS)
      message(VERBOSE "Using fallback configuration for group '${GROUP_NAME}'")
    endif()
  endif()
  
  # Configure all targets
  foreach(target_name IN LISTS GROUP_TARGETS)
    if(TARGET ${target_name})
      if(FINAL_CONFIG_ARGS)
        configure_benchmark_target(${target_name} ${FINAL_CONFIG_ARGS})
      endif()
    else()
      message(WARNING "Target ${target_name} does not exist in group ${GROUP_NAME}")
    endif()
  endforeach()
  
  # Create the group target
  add_benchmark_group_target(${GROUP_NAME} ${GROUP_TARGETS})
endfunction()

# =============================================================================
# EXISTING FUNCTIONS (unchanged)
# =============================================================================

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

    if("Eigen" IN_LIST ARG_LIBRARIES)
    target_include_directories(${TARGET_NAME} PRIVATE ${EIGEN_DIR})
   list(REMOVE_ITEM ARG_LIBRARIES Eigen)  # Remove Eigen from the libraries to avoid linking it
  endif()

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
  target_link_options(${TARGET_NAME} PRIVATE
      -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.map
      -Wl,--cref
      -Wl,--gc-sections
      -Wl,--print-memory-usage
    )

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
# Usage: add_stm32_target(target_name [CONFIG_STR config_string])
function(add_stm32_target target_name)
  cmake_parse_arguments(STM32 "" "CONFIG_STR" "" ${ARGN})
  get_target_property(TARGET_BUILD_DIR ${target_name} BINARY_DIR)
  file(RELATIVE_PATH RELATIVE_TARGET_BUILD_DIR ${CMAKE_BINARY_DIR} ${TARGET_BUILD_DIR})
  # Use CONFIG_STR argument if provided, else use target property
  if(STM32_CONFIG_STR)
    set(CONFIG_STR "${STM32_CONFIG_STR}")
  else()
    get_target_property(CONFIG_STR ${target_name} BENCH_CONFIG_STR)
    if(NOT CONFIG_STR)
      set(CONFIG_STR "")
    endif()
  endif()
  if(CONFIG_STR)
    set(FLASH_LOG "${TARGET_BUILD_DIR}/flash-${target_name}${CONFIG_STR}.log")
    set(DEBUG_LOG "${TARGET_BUILD_DIR}/debug-${target_name}${CONFIG_STR}.log")
  else()
    set(FLASH_LOG "${TARGET_BUILD_DIR}/flash-${target_name}.log")
    set(DEBUG_LOG "${TARGET_BUILD_DIR}/debug-${target_name}.log")
  endif()
  # Determine the programming command based on the OpenOCD config
  set(PROGRAM_CMD "program bin/${target_name}.elf verify")
  add_custom_target(stm32-flash-${target_name}-semihosted
    COMMAND bash -c "\
      ${OPENOCD_EXECUTABLE} \
        -f \"${OPENOCD_INTERFACE}\" \
        -f \"${CMAKE_SOURCE_DIR}/openocd/${OPENOCD_CFG}\" \
        -c 'init' \
        -c 'reset halt' \
        -c 'arm semihosting enable' \
        -c '${PROGRAM_CMD}' \
        -c 'reset run' \
        2>&1 | tee \"${FLASH_LOG}\""
    DEPENDS ${target_name}
    USES_TERMINAL
    VERBATIM
    COMMENT "Flashing ${target_name} and logging to ${FLASH_LOG}"
  )
  add_custom_target(stm32-debug-${target_name}-semihosted
    COMMAND bash -c "\
      ${OPENOCD_EXECUTABLE} \
        -f \"${OPENOCD_INTERFACE}\" \
        -f \"${CMAKE_SOURCE_DIR}/openocd/${OPENOCD_CFG}\" \
        -c 'init' \
        -c 'reset halt' \
        -c 'arm semihosting enable' \
        -c '${PROGRAM_CMD}' \
        -c 'reset halt' \
        2>&1 | tee \"${DEBUG_LOG}\""
    DEPENDS ${target_name}
    USES_TERMINAL
    VERBATIM
    COMMENT "Starting debug session for ${target_name} and logging to ${DEBUG_LOG}"
  )
endfunction()

function(add_stm32_no_semihosting_target target_name)
  # Parse optional configuration string argument
  cmake_parse_arguments(STM32 "" "CONFIG_STR" "" ${ARGN})
  
  # Flash target without semihosting
  get_target_property(TARGET_BUILD_DIR ${target_name} BINARY_DIR)
  file(RELATIVE_PATH RELATIVE_TARGET_BUILD_DIR ${CMAKE_BINARY_DIR} ${TARGET_BUILD_DIR})

  # Create log file names with optional configuration string
  if(STM32_CONFIG_STR)
    set(FLASH_LOG "${TARGET_BUILD_DIR}/flash-${target_name}${STM32_CONFIG_STR}.log")
    set(DEBUG_LOG "${TARGET_BUILD_DIR}/debug-${target_name}${STM32_CONFIG_STR}.log")
  else()
  set(FLASH_LOG "${TARGET_BUILD_DIR}/flash-${target_name}.log")
  set(DEBUG_LOG "${TARGET_BUILD_DIR}/debug-${target_name}.log")
  endif()

  add_custom_target(stm32-flash-${target_name}
    COMMAND bash -c "\
      ${OPENOCD_EXECUTABLE} \
        -f \"${OPENOCD_INTERFACE}\" \
        -f \"${CMAKE_SOURCE_DIR}/openocd/${OPENOCD_CFG}\" \
        -c 'init' \
        -c 'reset halt' \
        -c 'program \$<TARGET_FILE:${target_name}> verify' \
        -c 'reset run' \
        -c 'exit' \
        2>&1 | tee \"${FLASH_LOG}\""
    DEPENDS ${target_name}
    USES_TERMINAL
    VERBATIM
    COMMENT "Flashing ${target_name} to target (${OPENOCD_CFG}) without semihosting and logging to ${FLASH_LOG}"
  )

  # Debug target without semihosting
  add_custom_target(stm32-debug-${target_name}
    COMMAND bash -c "\
      ${OPENOCD_EXECUTABLE} \
        -f \"${OPENOCD_INTERFACE}\" \
        -f \"${CMAKE_SOURCE_DIR}/openocd/${OPENOCD_CFG}\" \
        -c 'init' \
        -c 'reset halt' \
        -c 'program \$<TARGET_FILE:${target_name}> verify' \
        -c 'reset halt' \
        2>&1 | tee \"${DEBUG_LOG}\""
    DEPENDS ${target_name}
    USES_TERMINAL
    VERBATIM
    COMMENT "Starting debug session for ${target_name} on ${OPENOCD_CFG} without semihosting and logging to ${DEBUG_LOG}"
  )
endfunction()

function(add_ento_test TARGET_NAME)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES" ${ARGN})

  if (NOT GEM5_ARMV7E-M_BUILD AND NOT STM32_BUILD)
    message(VERBOSE "Building ${TEST_BIN} for native")
    add_non_arm_executable(${TARGET_NAME}
      EXCLUDE TRUE
      SOURCES ${ARG_SOURCES}
      LIBRARIES ${ARG_LIBRARIES})
    add_dependencies(check ${TARGET_NAME})
    add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
  endif()
endfunction()

function(add_stm32_targets target_list)
  # Parse optional configuration string argument
  cmake_parse_arguments(STM32 "" "CONFIG_STR" "" ${ARGN})
  
  message(VERBOSE "Inside add_stm32_targets: ${target_list}")
  foreach(target_name IN LISTS target_list)
    MESSAGE(VERBOSE "Added stm32 target: ${target_name}")
    if(STM32_CONFIG_STR)
      add_stm32_target(${target_name} CONFIG_STR ${STM32_CONFIG_STR})
    else()
      add_stm32_target(${target_name})
    endif()
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
  message(VERBOSE "Created benchmark group '${GROUP_TARGET}' with ${TARGET_COUNT} targets")
  if(EXISTS ${PROGRESS_SCRIPT})
    message(VERBOSE "Using progress script: ${PROGRESS_SCRIPT}")
  endif()
endfunction()

# Enhanced function to create benchmark group targets with configuration parameters
# Usage: add_benchmark_group_target_with_config("attitude-est" 
#          TARGETS target1 target2 target3
#          REPS 30 VERBOSITY 1 ENABLE_CACHES)
function(add_benchmark_group_target_with_config GROUP_NAME)
  cmake_parse_arguments(GROUP 
    "DO_WARMUP;ENABLE_CACHES" 
    "REPS;INNER_REPS;VERBOSITY;MAX_PROBLEMS" 
    "TARGETS" 
    ${ARGN}
  )
  
  if(NOT GROUP_TARGETS)
    message(FATAL_ERROR "add_benchmark_group_target_with_config: TARGETS must be specified")
  endif()
  
  # Create the benchmark group target name
  set(GROUP_TARGET "bench-${GROUP_NAME}")
  
  # Convert target list to space-separated string
  string(REPLACE ";" " " TARGET_LIST_STR "${GROUP_TARGETS}")
  
  # Build configuration string for log file naming
  set(CONFIG_STR "")
  if(DEFINED GROUP_REPS)
    set(CONFIG_STR "${CONFIG_STR}_r${GROUP_REPS}")
  endif()
  if(DEFINED GROUP_INNER_REPS)
    set(CONFIG_STR "${CONFIG_STR}_ir${GROUP_INNER_REPS}")
  endif()
  if(DEFINED GROUP_VERBOSITY)
    set(CONFIG_STR "${CONFIG_STR}_v${GROUP_VERBOSITY}")
  endif()
  if(DEFINED GROUP_MAX_PROBLEMS)
    set(CONFIG_STR "${CONFIG_STR}_mp${GROUP_MAX_PROBLEMS}")
  endif()
  if(GROUP_DO_WARMUP)
    set(CONFIG_STR "${CONFIG_STR}_wu")
  endif()
  if(GROUP_ENABLE_CACHES)
    set(CONFIG_STR "${CONFIG_STR}_cache")
  else()
    set(CONFIG_STR "${CONFIG_STR}_nocache")
  endif()
  
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
    # Create a custom target that uses the external script for colored progress with config
    add_custom_target(${GROUP_TARGET}
      COMMAND bash ${PROGRESS_SCRIPT} "${GROUP_NAME}" "${TARGET_LIST_STR}" "${CMAKE_BINARY_DIR}" "${CONFIG_STR}"
      COMMENT "Building all ${GROUP_NAME} benchmarks with config${CONFIG_STR}"
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      USES_TERMINAL
    )
  endif()
  
  # Print summary
  list(LENGTH GROUP_TARGETS TARGET_COUNT)
  message(VERBOSE "Created benchmark group '${GROUP_TARGET}' with ${TARGET_COUNT} targets")
  if(EXISTS ${PROGRESS_SCRIPT})
    message(VERBOSE "Using progress script: ${PROGRESS_SCRIPT}")
    message(VERBOSE "Config string: ${CONFIG_STR}")
  endif()
  
  # Pass config string to STM32 targets for OpenOCD log naming
  if(STM32_BUILD AND CONFIG_STR)
    message(VERBOSE "Setting up STM32 targets with config: ${CONFIG_STR}")
    # We'll call add_stm32_targets with config later in the process
  endif()
endfunction()

# NEW: Function to configure benchmark parameters via CMake
# Usage: configure_benchmark_target(target_name
#          REPS 10
#          INNER_REPS 5  
#          VERBOSITY 2
#          MAX_PROBLEMS 100
#          DO_WARMUP ON
#          ENABLE_CACHES OFF)
function(configure_benchmark_target TARGET_NAME)
  cmake_parse_arguments(BENCH 
    "DO_WARMUP;ENABLE_CACHES;ENABLE_VECTORIZATION" 
    "REPS;INNER_REPS;VERBOSITY;MAX_PROBLEMS" 
    "" 
    ${ARGN}
  )
  
  if(TARGET ${TARGET_NAME})
    # Set benchmark configuration defines
    if(DEFINED BENCH_REPS)
      target_compile_definitions(${TARGET_NAME} PRIVATE REPS=${BENCH_REPS})
    endif()
    if(DEFINED BENCH_INNER_REPS)
      target_compile_definitions(${TARGET_NAME} PRIVATE INNER_REPS=${BENCH_INNER_REPS})
    endif()
    if(DEFINED BENCH_VERBOSITY)
      target_compile_definitions(${TARGET_NAME} PRIVATE VERBOSITY=${BENCH_VERBOSITY})
    endif()
    if(DEFINED BENCH_MAX_PROBLEMS)
      target_compile_definitions(${TARGET_NAME} PRIVATE MAX_PROBLEMS=${BENCH_MAX_PROBLEMS})
    endif()
    if(BENCH_DO_WARMUP)
      target_compile_definitions(${TARGET_NAME} PRIVATE DO_WARMUP=1)
    else()
      target_compile_definitions(${TARGET_NAME} PRIVATE DO_WARMUP=0)
    endif()
    if(BENCH_ENABLE_CACHES)
      target_compile_definitions(${TARGET_NAME} PRIVATE ENABLE_CACHES=1)
    else()
      target_compile_definitions(${TARGET_NAME} PRIVATE ENABLE_CACHES=0)
    endif()
    if(BENCH_ENABLE_VECTORIZATION)
      target_compile_definitions(${TARGET_NAME} PRIVATE ENABLE_VECTORIZATION=1)
    else()
      target_compile_definitions(${TARGET_NAME} PRIVATE ENABLE_VECTORIZATION=0)
    endif()
    # Generate config string for this target
    set(CONFIG_STR "")
    if(DEFINED BENCH_REPS)
      set(CONFIG_STR "${CONFIG_STR}_r${BENCH_REPS}")
    endif()
    if(DEFINED BENCH_INNER_REPS)
      set(CONFIG_STR "${CONFIG_STR}_ir${BENCH_INNER_REPS}")
    endif()
    if(DEFINED BENCH_VERBOSITY)
      set(CONFIG_STR "${CONFIG_STR}_v${BENCH_VERBOSITY}")
    endif()
    if(DEFINED BENCH_MAX_PROBLEMS)
      set(CONFIG_STR "${CONFIG_STR}_mp${BENCH_MAX_PROBLEMS}")
    endif()
    if(BENCH_DO_WARMUP)
      set(CONFIG_STR "${CONFIG_STR}_wu")
    endif()
    if(BENCH_ENABLE_VECTORIZATION)
      set(CONFIG_STR "${CONFIG_STR}_vec")
    endif()
    if(BENCH_ENABLE_CACHES)
      set(CONFIG_STR "${CONFIG_STR}_cache")
    else()
      set(CONFIG_STR "${CONFIG_STR}_nocache")
    endif()
    set_target_properties(${TARGET_NAME} PROPERTIES BENCH_CONFIG_STR "${CONFIG_STR}")
    message(VERBOSE "Configured ${TARGET_NAME} with:")
    if(DEFINED BENCH_REPS)
      message(VERBOSE "  REPS=${BENCH_REPS}")
    endif()
    if(DEFINED BENCH_INNER_REPS)
      message(VERBOSE "  INNER_REPS=${BENCH_INNER_REPS}")
    endif()
    if(DEFINED BENCH_VERBOSITY)
      message(VERBOSE "  VERBOSITY=${BENCH_VERBOSITY}")
    endif()
    if(DEFINED BENCH_MAX_PROBLEMS)
      message(VERBOSE "  MAX_PROBLEMS=${BENCH_MAX_PROBLEMS}")
    endif()
    message(VERBOSE "  DO_WARMUP=${BENCH_DO_WARMUP}")
    message(VERBOSE "  ENABLE_CACHES=${BENCH_ENABLE_CACHES}")
    message(VERBOSE "  ENABLE_VECTORIZATION=${BENCH_ENABLE_VECTORIZATION}")
    message(VERBOSE "  BENCH_CONFIG_STR=${CONFIG_STR}")
  else()
    message(WARNING "Target ${TARGET_NAME} does not exist, cannot configure benchmark parameters")
  endif()
endfunction()

# Enhanced add_benchmark function with configuration support
function(add_configured_benchmark TARGET_NAME)
  cmake_parse_arguments(ARG 
    "DO_WARMUP;ENABLE_CACHES;ENABLE_VECTORIZATION" 
    "EXCLUDE;REPS;INNER_REPS;VERBOSITY;MAX_PROBLEMS" 
    "SOURCES;LIBRARIES" 
    ${ARGN}
  )

  # Create the basic benchmark target first
  if(STM32_BUILD)
    add_arm_semihosting_executable(${TARGET_NAME}
      SOURCES ${ARG_SOURCES}
      LIBRARIES ${ARG_LIBRARIES}
    )
    add_arm_executable(${TARGET_NAME}-no-semihosting
      SOURCES ${ARG_SOURCES}
      LIBRARIES ${ARG_LIBRARIES}
    )
  elseif(GEM5_BUILD)
    add_arm_baremetal_gem5_se_executable(${TARGET_NAME}
      SOURCES ${ARG_SOURCES}
      LIBRARIES ${ARG_LIBRARIES}
    )
  endif()

  # Apply configuration parameters
  configure_benchmark_target(${TARGET_NAME}
    REPS ${ARG_REPS}
    INNER_REPS ${ARG_INNER_REPS}
    VERBOSITY ${ARG_VERBOSITY}
    MAX_PROBLEMS ${ARG_MAX_PROBLEMS}
    ${ARG_DO_WARMUP}
    ${ARG_ENABLE_CACHES}
    ${ARG_ENABLE_VECTORIZATION}
  )
endfunction()

# NEW: Enhanced benchmark group function with configuration support
function(add_configured_benchmark_group_target GROUP_NAME)
  # Parse configuration arguments and target list
  cmake_parse_arguments(GROUP 
    "DO_WARMUP;ENABLE_CACHES;ENABLE_VECTORIZATION" 
    "REPS;INNER_REPS;VERBOSITY;MAX_PROBLEMS" 
    "" 
    ${ARGN}
  )
  
  # The remaining unparsed arguments are the target list
  set(TARGET_LIST ${GROUP_UNPARSED_ARGUMENTS})
  
  # Apply configuration to all targets in the group
  foreach(target_name IN LISTS TARGET_LIST)
    if(TARGET ${target_name})
      configure_benchmark_target(${target_name}
        REPS ${GROUP_REPS}
        INNER_REPS ${GROUP_INNER_REPS}
        VERBOSITY ${GROUP_VERBOSITY}
        MAX_PROBLEMS ${GROUP_MAX_PROBLEMS}
        ${GROUP_DO_WARMUP}
        ${GROUP_ENABLE_CACHES}
        ${GROUP_ENABLE_VECTORIZATION}
      )
    else()
      message(WARNING "Target ${target_name} does not exist in group ${GROUP_NAME}")
    endif()
  endforeach()
  
  # Create the group target using the enhanced function with config
  add_benchmark_group_target_with_config(${GROUP_NAME}
    TARGETS ${TARGET_LIST}
    REPS ${GROUP_REPS}
    INNER_REPS ${GROUP_INNER_REPS}
    VERBOSITY ${GROUP_VERBOSITY}
    MAX_PROBLEMS ${GROUP_MAX_PROBLEMS}
    ${GROUP_DO_WARMUP}
    ${GROUP_ENABLE_CACHES}
    ${GROUP_ENABLE_VECTORIZATION}
  )
  
  # Print configuration summary for the group
  list(LENGTH TARGET_LIST TARGET_COUNT)
  message(VERBOSE "Applied configuration to ${TARGET_COUNT} targets in group '${GROUP_NAME}':")
  if(DEFINED GROUP_REPS)
    message(VERBOSE "  Group REPS=${GROUP_REPS}")
  endif()
  if(DEFINED GROUP_INNER_REPS)
    message(VERBOSE "  Group INNER_REPS=${GROUP_INNER_REPS}")
  endif()
  if(DEFINED GROUP_VERBOSITY)
    message(VERBOSE "  Group VERBOSITY=${GROUP_VERBOSITY}")
  endif()
  if(DEFINED GROUP_MAX_PROBLEMS)
    message(VERBOSE "  Group MAX_PROBLEMS=${GROUP_MAX_PROBLEMS}")
  endif()
  message(VERBOSE "  Group DO_WARMUP=${GROUP_DO_WARMUP}")
  message(VERBOSE "  Group ENABLE_CACHES=${GROUP_ENABLE_CACHES}")
  message(VERBOSE "  Group ENABLE_VECTORIZATION=${GROUP_ENABLE_VECTORIZATION}")
endfunction()

# Enhanced function that configures targets first, then creates group
# Usage: add_preconfigured_benchmark_group("performance-tests"
#          TARGETS target1 target2 target3
#          REPS 50
#          VERBOSITY 1
#          ENABLE_CACHES)
function(add_preconfigured_benchmark_group GROUP_NAME)
  cmake_parse_arguments(GROUP 
    "DO_WARMUP;ENABLE_CACHES;ENABLE_VECTORIZATION" 
    "REPS;INNER_REPS;VERBOSITY;MAX_PROBLEMS" 
    "TARGETS" 
    ${ARGN}
  )
  
  if(NOT GROUP_TARGETS)
    message(FATAL_ERROR "add_preconfigured_benchmark_group: TARGETS must be specified")
  endif()
  
  # Build configuration string for log file naming
  set(CONFIG_STR "")
  if(DEFINED GROUP_REPS)
    set(CONFIG_STR "${CONFIG_STR}_r${GROUP_REPS}")
  endif()
  if(DEFINED GROUP_INNER_REPS)
    set(CONFIG_STR "${CONFIG_STR}_ir${GROUP_INNER_REPS}")
  endif()
  if(DEFINED GROUP_VERBOSITY)
    set(CONFIG_STR "${CONFIG_STR}_v${GROUP_VERBOSITY}")
  endif()
  if(DEFINED GROUP_MAX_PROBLEMS)
    set(CONFIG_STR "${CONFIG_STR}_mp${GROUP_MAX_PROBLEMS}")
  endif()
  if(GROUP_DO_WARMUP)
    set(CONFIG_STR "${CONFIG_STR}_wu")
  endif()
  if(GROUP_ENABLE_VECTORIZATION)
    set(CONFIG_STR "${CONFIG_STR}_vec")
  endif()
  if(GROUP_ENABLE_CACHES)
    set(CONFIG_STR "${CONFIG_STR}_cache")
  else()
    set(CONFIG_STR "${CONFIG_STR}_nocache")
  endif()
  
  # Store configuration string globally for use by add_stm32_targets
  set(GLOBAL_BENCHMARK_CONFIG_STR "${CONFIG_STR}" CACHE INTERNAL "Current benchmark configuration string")
  
  # Configure all targets first
  foreach(target_name IN LISTS GROUP_TARGETS)
    if(TARGET ${target_name})
      # Build configuration arguments dynamically
      set(CONFIG_ARGS "")
      if(DEFINED GROUP_REPS)
        list(APPEND CONFIG_ARGS "REPS" "${GROUP_REPS}")
      endif()
      if(DEFINED GROUP_INNER_REPS)
        list(APPEND CONFIG_ARGS "INNER_REPS" "${GROUP_INNER_REPS}")
      endif()
      if(DEFINED GROUP_VERBOSITY)
        list(APPEND CONFIG_ARGS "VERBOSITY" "${GROUP_VERBOSITY}")
      endif()
      if(DEFINED GROUP_MAX_PROBLEMS)
        list(APPEND CONFIG_ARGS "MAX_PROBLEMS" "${GROUP_MAX_PROBLEMS}")
      endif()
      if(GROUP_DO_WARMUP)
        list(APPEND CONFIG_ARGS "DO_WARMUP")
      endif()
      if(GROUP_ENABLE_CACHES)
        list(APPEND CONFIG_ARGS "ENABLE_CACHES")
      endif()
      if(GROUP_ENABLE_VECTORIZATION)
        list(APPEND CONFIG_ARGS "ENABLE_VECTORIZATION")
      endif()
      
      # Apply configuration if any parameters were specified
      if(CONFIG_ARGS)
        configure_benchmark_target(${target_name} ${CONFIG_ARGS})
      endif()
    else()
      message(WARNING "Target ${target_name} does not exist in group ${GROUP_NAME}")
    endif()
  endforeach()
  
  # Create the group target with configuration parameters
  add_benchmark_group_target_with_config(${GROUP_NAME}
    TARGETS ${GROUP_TARGETS}
    REPS ${GROUP_REPS}
    INNER_REPS ${GROUP_INNER_REPS}
    VERBOSITY ${GROUP_VERBOSITY}
    MAX_PROBLEMS ${GROUP_MAX_PROBLEMS}
    ${GROUP_DO_WARMUP}
    ${GROUP_ENABLE_CACHES}
    ${GROUP_ENABLE_VECTORIZATION}
  )
  
  message(VERBOSE "Stored benchmark config string: ${CONFIG_STR}")
endfunction()

# Enhanced function to configure benchmark group with both group and target-specific configs
# Usage: add_configured_benchmark_group_with_target_configs("attitude-est"
#          CONFIG_FILE "configs/benchmarks.json"
#          TARGETS target1 target2 target3)
function(add_configured_benchmark_group_with_target_configs GROUP_NAME)
  cmake_parse_arguments(GROUP 
    "DO_WARMUP;ENABLE_CACHES" 
    "CONFIG_FILE;REPS;INNER_REPS;VERBOSITY;MAX_PROBLEMS" 
    "TARGETS" 
    ${ARGN}
  )
  
  if(NOT GROUP_TARGETS)
    message(FATAL_ERROR "add_configured_benchmark_group_with_target_configs: TARGETS must be specified")
  endif()
  
  # Get group-level configuration
  set(GROUP_CONFIG_ARGS "")
  if(GROUP_CONFIG_FILE AND EXISTS ${GROUP_CONFIG_FILE})
    parse_benchmark_config_file(${GROUP_CONFIG_FILE} ${GROUP_NAME} GROUP_CONFIG_ARGS)
  endif()
  
  # Extract configuration values for log file naming from GROUP_CONFIG_ARGS
  set(LOG_REPS "")
  set(LOG_INNER_REPS "")
  set(LOG_VERBOSITY "")
  set(LOG_MAX_PROBLEMS "")
  set(LOG_DO_WARMUP OFF)
  set(LOG_ENABLE_CACHES OFF)
  set(LOG_VECTORIZATION OFF)
  
  # Parse GROUP_CONFIG_ARGS to extract values
  list(LENGTH GROUP_CONFIG_ARGS CONFIG_LEN)
  if(CONFIG_LEN GREATER 0)
    set(i 0)
    while(i LESS CONFIG_LEN)
      list(GET GROUP_CONFIG_ARGS ${i} KEY)
      math(EXPR i "${i} + 1")
      if(i LESS CONFIG_LEN)
        if(KEY STREQUAL "REPS")
          list(GET GROUP_CONFIG_ARGS ${i} LOG_REPS)
        elseif(KEY STREQUAL "INNER_REPS")
          list(GET GROUP_CONFIG_ARGS ${i} LOG_INNER_REPS)
        elseif(KEY STREQUAL "VERBOSITY")
          list(GET GROUP_CONFIG_ARGS ${i} LOG_VERBOSITY)
        elseif(KEY STREQUAL "MAX_PROBLEMS")
          list(GET GROUP_CONFIG_ARGS ${i} LOG_MAX_PROBLEMS)
        endif()
        math(EXPR i "${i} + 1")
      else()
        # Boolean flags don't have values
        if(KEY STREQUAL "DO_WARMUP")
          set(LOG_DO_WARMUP ON)
        elseif(KEY STREQUAL "ENABLE_CACHES")
          set(LOG_ENABLE_CACHES ON)
        elseif(KEY STREQUAL "ENABLE_VECTORIZATION")
          set(LOG_VECTORIZATION ON)
        endif()
        math(EXPR i "${i} + 1")
      endif()
    endwhile()
  endif()
  
  # Build configuration string for log file naming
  set(CONFIG_STR "")
  if(LOG_REPS)
    set(CONFIG_STR "${CONFIG_STR}_r${LOG_REPS}")
  endif()
  if(LOG_INNER_REPS)
    set(CONFIG_STR "${CONFIG_STR}_ir${LOG_INNER_REPS}")
  endif()
  if(LOG_VERBOSITY)
    set(CONFIG_STR "${CONFIG_STR}_v${LOG_VERBOSITY}")
  endif()
  if(LOG_MAX_PROBLEMS)
    set(CONFIG_STR "${CONFIG_STR}_mp${LOG_MAX_PROBLEMS}")
  endif()
  if(LOG_DO_WARMUP)
    set(CONFIG_STR "${CONFIG_STR}_wu")
  endif()
  if(LOG_VECTORIZATION)
    set(CONFIG_STR "${CONFIG_STR}_vec")
  endif()
  if(LOG_ENABLE_CACHES)
    set(CONFIG_STR "${CONFIG_STR}_cache")
  else()
    set(CONFIG_STR "${CONFIG_STR}_nocache")
  endif()
  
  # Store configuration string globally for use by add_stm32_targets
  set(GLOBAL_BENCHMARK_CONFIG_STR "${CONFIG_STR}" CACHE INTERNAL "Current benchmark configuration string")
  
  # Apply configuration to each target
  foreach(target_name IN LISTS GROUP_TARGETS)
    if(TARGET ${target_name})
      # Start with group configuration
      set(FINAL_CONFIG_ARGS ${GROUP_CONFIG_ARGS})
      
      # Check for target-specific overrides
      if(GROUP_CONFIG_FILE AND EXISTS ${GROUP_CONFIG_FILE})
        parse_target_config_file(${GROUP_CONFIG_FILE} ${target_name} TARGET_CONFIG_ARGS)
        
        # If target-specific config exists, it overrides group config
        if(TARGET_CONFIG_ARGS)
          set(FINAL_CONFIG_ARGS ${TARGET_CONFIG_ARGS})
          message(VERBOSE "Using target-specific config for ${target_name}")
        else()
          message(VERBOSE "Using group config for ${target_name}")
        endif()
      endif()
      
      # Apply final configuration
      if(FINAL_CONFIG_ARGS)
        configure_benchmark_target(${target_name} ${FINAL_CONFIG_ARGS})
      endif()
    else()
      message(WARNING "Target ${target_name} does not exist in group ${GROUP_NAME}")
    endif()
  endforeach()
  
  # Create the group target with configuration parameters for log naming
  set(CONFIG_PARAMS TARGETS ${GROUP_TARGETS})
  if(LOG_REPS)
    list(APPEND CONFIG_PARAMS REPS ${LOG_REPS})
  endif()
  if(LOG_INNER_REPS)
    list(APPEND CONFIG_PARAMS INNER_REPS ${LOG_INNER_REPS})
  endif()
  if(LOG_VERBOSITY)
    list(APPEND CONFIG_PARAMS VERBOSITY ${LOG_VERBOSITY})
  endif()
  if(LOG_MAX_PROBLEMS)
    list(APPEND CONFIG_PARAMS MAX_PROBLEMS ${LOG_MAX_PROBLEMS})
  endif()
  if(LOG_DO_WARMUP)
    list(APPEND CONFIG_PARAMS DO_WARMUP)
  endif()
  if(LOG_ENABLE_CACHES)
    list(APPEND CONFIG_PARAMS ENABLE_CACHES)
  endif()
  if(LOG_VECTORIZATION)
    list(APPEND CONFIG_PARAMS ENABLE_VECTORIZATION)
  endif()
  
  add_benchmark_group_target_with_config(${GROUP_NAME} ${CONFIG_PARAMS})
  
  message(VERBOSE "Stored benchmark config string: ${CONFIG_STR}")
endfunction()

# =============================================================================
# BENCHMARK GROUP RUNNER FUNCTIONS (with timeout support)
# =============================================================================

# Function to create a CMake script for running benchmark groups with timeout
# Usage: create_benchmark_runner_script()
function(create_benchmark_runner_script)
  set(RUNNER_SCRIPT "${CMAKE_BINARY_DIR}/run_benchmark_group.cmake")
  
  file(WRITE ${RUNNER_SCRIPT} "
# EntoBench Group Runner Script with Timeout Support
# Generated by CMake - DO NOT EDIT MANUALLY

# Parse command line arguments
if(NOT DEFINED BENCHMARK_LIST)
  message(FATAL_ERROR \"BENCHMARK_LIST must be defined\")
endif()

if(NOT DEFINED TIMEOUT)
  set(TIMEOUT 60)
endif()

if(NOT DEFINED OUTPUT_TIMEOUT)
  set(OUTPUT_TIMEOUT 10)
endif()

if(NOT DEFINED RESULTS_DIR)
  set(RESULTS_DIR \"benchmark_results\")
endif()

if(NOT DEFINED GROUP_NAME)
  set(GROUP_NAME \"unknown\")
endif()

if(NOT DEFINED CONFIG_STR)
  set(CONFIG_STR \"\")
endif()

# Create results directory
file(MAKE_DIRECTORY \"\${RESULTS_DIR}\")

# Initialize timing CSV with config-based naming
if(CONFIG_STR)
  set(TIMING_FILE \"\${RESULTS_DIR}/timing\${CONFIG_STR}.csv\")
  set(SUMMARY_FILE \"\${RESULTS_DIR}/summary\${CONFIG_STR}.txt\")
else()
  set(TIMING_FILE \"\${RESULTS_DIR}/timing.csv\")
  set(SUMMARY_FILE \"\${RESULTS_DIR}/summary.txt\")
endif()

file(WRITE \"\${TIMING_FILE}\" \"benchmark_name,config_str,start_time,end_time,duration_s,exit_code,timed_out,output_timed_out\\n\")

# Initialize counters
set(SUCCESS_COUNT 0)
set(TIMEOUT_COUNT 0)
set(OUTPUT_TIMEOUT_COUNT 0)
set(FAILURE_COUNT 0)
set(TOTAL_DURATION 0)

# Convert benchmark list from semicolon-separated to list
string(REPLACE \";\" \";\" BENCHMARKS \"\${BENCHMARK_LIST}\")

# Get total count
list(LENGTH BENCHMARKS TOTAL_COUNT)
set(CURRENT 1)

message(STATUS \"[EntoBench] Starting benchmark group: \${GROUP_NAME}\")
message(STATUS \"[EntoBench] Benchmarks to run: \${TOTAL_COUNT}\")
message(STATUS \"[EntoBench] Timeout per benchmark: \${TIMEOUT}s\")
message(STATUS \"[EntoBench] Output timeout: \${OUTPUT_TIMEOUT}s\")
message(STATUS \"[EntoBench] Results directory: \${RESULTS_DIR}\")

foreach(benchmark IN LISTS BENCHMARKS)
  message(STATUS \"[EntoBench] Progress: \${CURRENT}/\${TOTAL_COUNT} - \${benchmark}\")
  
  # Try to find benchmark binary in common locations
  set(BENCHMARK_PATH \"\")
  set(POSSIBLE_PATHS
    \"\${CMAKE_BINARY_DIR}/bin/\${benchmark}\"
    \"\${CMAKE_BINARY_DIR}/bin/\${benchmark}.elf\"
    \"\${CMAKE_BINARY_DIR}/benchmark/state-est/pose-est/bin/\${benchmark}\"
    \"\${CMAKE_BINARY_DIR}/benchmark/state-est/pose-est/bin/\${benchmark}.elf\"
    \"\${CMAKE_BINARY_DIR}/benchmark/state-est/attitude-est/bin/\${benchmark}\"
    \"\${CMAKE_BINARY_DIR}/benchmark/state-est/attitude-est/bin/\${benchmark}.elf\"
  )
  
  foreach(path IN LISTS POSSIBLE_PATHS)
    if(EXISTS \"\${path}\")
      set(BENCHMARK_PATH \"\${path}\")
      break()
    endif()
  endforeach()
  
  if(NOT BENCHMARK_PATH)
    message(WARNING \"[EntoBench] Benchmark binary not found for: \${benchmark}\")
    message(STATUS \"[EntoBench] Searched paths: \${POSSIBLE_PATHS}\")
    file(APPEND \"\${TIMING_FILE}\" \"\${benchmark},0,0,0,127,false,false\\n\")
    file(WRITE \"\${RESULTS_DIR}/\${benchmark}.log\" \"BENCHMARK BINARY NOT FOUND\\n\")
    math(EXPR CURRENT \"\${CURRENT} + 1\")
    math(EXPR FAILURE_COUNT \"\${FAILURE_COUNT} + 1\")
    continue()
  endif()
  
  message(STATUS \"[EntoBench] Found binary: \${BENCHMARK_PATH}\")
  
  # Record start time
  string(TIMESTAMP START_TIME \"%s\")
  
  # Run benchmark with timeout - use config-based log file naming
  if(CONFIG_STR)
    set(LOG_FILE \"\${RESULTS_DIR}/\${benchmark}\${CONFIG_STR}.log\")
  else()
    set(LOG_FILE \"\${RESULTS_DIR}/\${benchmark}.log\")
  endif()
  
  # Check if this is an STM32 build (binary has .elf extension)
  if(BENCHMARK_PATH MATCHES \"\\\\.elf$\")
    # For STM32 builds, we need to use OpenOCD to flash and run
    message(STATUS \"[EntoBench] STM32 binary detected, using OpenOCD flash target\")
    
    # Use the STM32 flash target instead of direct execution
    # Suppress build output but capture semihosting output
    execute_process(
      COMMAND \${CMAKE_COMMAND} --build . --target stm32-flash-\${benchmark}-semihosted
      WORKING_DIRECTORY \"\${CMAKE_BINARY_DIR}\"
      RESULT_VARIABLE RESULT
      OUTPUT_VARIABLE BUILD_OUTPUT
      ERROR_VARIABLE BUILD_ERROR
      TIMEOUT \${TIMEOUT}
    )
    
    # Extract only the benchmark results from the output (lines after \"File path:\")
    string(FIND \"\${BUILD_OUTPUT}\" \"File path:\" BENCHMARK_START)
    if(BENCHMARK_START GREATER -1)
      string(SUBSTRING \"\${BUILD_OUTPUT}\" \${BENCHMARK_START} -1 BENCHMARK_RESULTS)
      file(WRITE \"\${LOG_FILE}\" \"\${BENCHMARK_RESULTS}\")
    else()
      # Fallback: look for benchmark output patterns
      string(REGEX MATCH \"(Iteration [0-9]+:.*|Results from running.*|Average cycles:.*|Max cycles:.*|Min cycles:.*)\" BENCHMARK_RESULTS \"\${BUILD_OUTPUT}\")
      if(BENCHMARK_RESULTS)
        file(WRITE \"\${LOG_FILE}\" \"\${BENCHMARK_RESULTS}\")
      else()
        # If no benchmark results found, log the error
        file(WRITE \"\${LOG_FILE}\" \"No benchmark results found. Build output:\\n\${BUILD_OUTPUT}\\n\\nBuild error:\\n\${BUILD_ERROR}\")
      endif()
    endif()
  else()
    # Native build - run directly with timeout
    if(WIN32)
      # Windows timeout command
      execute_process(
        COMMAND timeout /t \${TIMEOUT} \"\${BENCHMARK_PATH}\"
        WORKING_DIRECTORY \"\${CMAKE_BINARY_DIR}\"
        RESULT_VARIABLE RESULT
        OUTPUT_FILE \"\${LOG_FILE}\"
        ERROR_FILE \"\${LOG_FILE}\"
        TIMEOUT \${TIMEOUT}
      )
    else()
      # Unix timeout command
      execute_process(
        COMMAND timeout \${TIMEOUT}s \"\${BENCHMARK_PATH}\"
        WORKING_DIRECTORY \"\${CMAKE_BINARY_DIR}\"
        RESULT_VARIABLE RESULT
        OUTPUT_FILE \"\${LOG_FILE}\"
        ERROR_FILE \"\${LOG_FILE}\"
        TIMEOUT \${TIMEOUT}
      )
    endif()
  endif()
  
  # Record end time
  string(TIMESTAMP END_TIME \"%s\")
  math(EXPR DURATION \"\${END_TIME} - \${START_TIME}\")
  math(EXPR TOTAL_DURATION \"\${TOTAL_DURATION} + \${DURATION}\")
  
  # Determine result status
  set(TIMED_OUT false)
  set(OUTPUT_TIMED_OUT false)
  
  if(RESULT EQUAL 124)
    set(TIMED_OUT true)
    message(WARNING \"[EntoBench] \${benchmark} timed out after \${TIMEOUT}s\")
    file(APPEND \"\${LOG_FILE}\" \"\\nBENCHMARK TIMED OUT AFTER \${TIMEOUT}s\\n\")
    math(EXPR TIMEOUT_COUNT \"\${TIMEOUT_COUNT} + 1\")
  elseif(RESULT EQUAL 0)
    message(STATUS \"[EntoBench] \${benchmark} completed successfully (\${DURATION}s)\")
    math(EXPR SUCCESS_COUNT \"\${SUCCESS_COUNT} + 1\")
  else()
    message(WARNING \"[EntoBench] \${benchmark} failed with exit code \${RESULT}\")
    file(APPEND \"\${LOG_FILE}\" \"\\nBENCHMARK FAILED WITH EXIT CODE \${RESULT}\\n\")
    math(EXPR FAILURE_COUNT \"\${FAILURE_COUNT} + 1\")
  endif()
  
  # Record timing information
  file(APPEND \"\${TIMING_FILE}\" \"\${benchmark},\${CONFIG_STR},\${START_TIME},\${END_TIME},\${DURATION},\${RESULT},\${TIMED_OUT},\${OUTPUT_TIMED_OUT}\\n\")
  
  math(EXPR CURRENT \"\${CURRENT} + 1\")
endforeach()

# Generate summary report
file(WRITE \"\${SUMMARY_FILE}\" \"EntoBench Pose Estimation Benchmark Results
==========================================

Group: \${GROUP_NAME}
Configuration: \${CONFIG_STR}
Date: \${CMAKE_CURRENT_TIMESTAMP}
Total Benchmarks: \${TOTAL_COUNT}

Summary Statistics:
==================
Successful:        \${SUCCESS_COUNT}
Timed out:         \${TIMEOUT_COUNT}
Output timed out:  \${OUTPUT_TIMEOUT_COUNT}
Failed:            \${FAILURE_COUNT}
Total duration:    \${TOTAL_DURATION}s

Configuration Details:
====================
Timeout per benchmark: \${TIMEOUT}s
Output timeout:        \${OUTPUT_TIMEOUT}s
Config string:         \${CONFIG_STR}

Log files location: \${RESULTS_DIR}/
Timing data:       \${TIMING_FILE}
\")

# Print final summary
message(STATUS \"[EntoBench] Benchmark Results Summary:\")
message(STATUS \"[EntoBench]   Successful:        \${SUCCESS_COUNT}\")
message(STATUS \"[EntoBench]   Timed out:         \${TIMEOUT_COUNT}\")
message(STATUS \"[EntoBench]   Output timed out:  \${OUTPUT_TIMEOUT_COUNT}\")
message(STATUS \"[EntoBench]   Failed:            \${FAILURE_COUNT}\")
message(STATUS \"[EntoBench]   Total duration:    \${TOTAL_DURATION}s\")
message(STATUS \"[EntoBench] Full report: \${SUMMARY_FILE}\")
message(STATUS \"[EntoBench] Benchmark run completed!\")
")

  message(VERBOSE "Created benchmark runner script: ${RUNNER_SCRIPT}")
endfunction()

# Function to add a benchmark group runner target with timeout support
# Usage: add_benchmark_group_runner("absolute-pose"
#          TARGETS bench-p3p-float bench-p3p-double bench-up2p-float
#          TIMEOUT 60
#          OUTPUT_TIMEOUT 10)
function(add_benchmark_group_runner GROUP_NAME)
  cmake_parse_arguments(RUNNER
    ""
    "TIMEOUT;OUTPUT_TIMEOUT;RESULTS_DIR"
    "TARGETS"
    ${ARGN}
  )
  
  if(NOT RUNNER_TARGETS)
    message(FATAL_ERROR "add_benchmark_group_runner: TARGETS must be specified")
  endif()
  
  # Set defaults
  if(NOT DEFINED RUNNER_TIMEOUT)
    set(RUNNER_TIMEOUT 60)
  endif()
  
  if(NOT DEFINED RUNNER_OUTPUT_TIMEOUT)
    set(RUNNER_OUTPUT_TIMEOUT 10)
  endif()
  
  # Try to get config string from the first target for consistent naming
  set(CONFIG_STR "")
  list(GET RUNNER_TARGETS 0 FIRST_TARGET)
  if(TARGET ${FIRST_TARGET})
    get_target_property(TARGET_CONFIG_STR ${FIRST_TARGET} BENCH_CONFIG_STR)
    if(TARGET_CONFIG_STR)
      set(CONFIG_STR "${TARGET_CONFIG_STR}")
    endif()
  endif()
  
  if(NOT DEFINED RUNNER_RESULTS_DIR)
    if(CONFIG_STR)
      set(RUNNER_RESULTS_DIR "benchmark_results_${GROUP_NAME}${CONFIG_STR}")
    else()
      set(RUNNER_RESULTS_DIR "benchmark_results_${GROUP_NAME}")
    endif()
  endif()
  
  # Create the runner script if it doesn't exist
  set(RUNNER_SCRIPT "${CMAKE_BINARY_DIR}/run_benchmark_group.cmake")
  if(NOT EXISTS ${RUNNER_SCRIPT})
    create_benchmark_runner_script()
  endif()
  
  # Convert target list to properly escaped semicolon-separated string for CMake command line
  string(REPLACE ";" "\\;" TARGET_LIST_STR "${RUNNER_TARGETS}")
  
  # Create the runner target
  set(RUNNER_TARGET "bench-${GROUP_NAME}-run")
  
  add_custom_target(${RUNNER_TARGET}
    COMMAND ${CMAKE_COMMAND}
      "-DBENCHMARK_LIST=${TARGET_LIST_STR}"
      -DTIMEOUT=${RUNNER_TIMEOUT}
      -DOUTPUT_TIMEOUT=${RUNNER_OUTPUT_TIMEOUT}
      -DRESULTS_DIR=${RUNNER_RESULTS_DIR}
      -DGROUP_NAME=${GROUP_NAME}
      "-DCONFIG_STR=${CONFIG_STR}"
      -P "${RUNNER_SCRIPT}"
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running ${GROUP_NAME} benchmark group with timeout support"
    USES_TERMINAL
  )
  
  # Create dependencies on the individual benchmark targets
  foreach(target IN LISTS RUNNER_TARGETS)
    if(TARGET ${target})
      add_dependencies(${RUNNER_TARGET} ${target})
    endif()
  endforeach()
  
  # Print summary
  list(LENGTH RUNNER_TARGETS TARGET_COUNT)
  message(VERBOSE "Created benchmark runner '${RUNNER_TARGET}' with ${TARGET_COUNT} targets")
  message(VERBOSE "  Timeout: ${RUNNER_TIMEOUT}s, Output timeout: ${RUNNER_OUTPUT_TIMEOUT}s")
  message(VERBOSE "  Results directory: ${RUNNER_RESULTS_DIR}")
  if(CONFIG_STR)
    message(VERBOSE "  Config string: ${CONFIG_STR}")
  endif()
endfunction()

# Convenience function to add multiple benchmark group runners
# Usage: add_benchmark_group_runners(
#          GROUPS "absolute-pose;relative-pose;gold-standard"
#          ABSOLUTE_POSE_TARGETS "bench-p3p-float;bench-p3p-double"
#          RELATIVE_POSE_TARGETS "bench-8pt-float;bench-8pt-double"
#          GOLD_STANDARD_TARGETS "bench-gold-standard-abs-float"
#          TIMEOUT 60)
function(add_benchmark_group_runners)
  cmake_parse_arguments(RUNNERS
    ""
    "TIMEOUT;OUTPUT_TIMEOUT"
    "GROUPS"
    ${ARGN}
  )
  
  if(NOT RUNNERS_GROUPS)
    message(FATAL_ERROR "add_benchmark_group_runners: GROUPS must be specified")
  endif()
  
  # Set defaults
  if(NOT DEFINED RUNNERS_TIMEOUT)
    set(RUNNERS_TIMEOUT 60)
  endif()
  
  if(NOT DEFINED RUNNERS_OUTPUT_TIMEOUT)
    set(RUNNERS_OUTPUT_TIMEOUT 10)
  endif()
  
  foreach(group IN LISTS RUNNERS_GROUPS)
    # Convert group name to variable name (replace hyphens with underscores, uppercase)
    string(REPLACE "-" "_" GROUP_VAR ${group})
    string(TOUPPER ${GROUP_VAR} GROUP_VAR)
    set(TARGETS_VAR "${GROUP_VAR}_TARGETS")
    
    # Check if targets variable exists in the remaining arguments
    set(GROUP_TARGETS "")
    set(FOUND_TARGETS FALSE)
    
    # Parse remaining arguments to find the targets for this group
    set(REMAINING_ARGS ${RUNNERS_UNPARSED_ARGUMENTS})
    list(LENGTH REMAINING_ARGS ARG_COUNT)
    set(i 0)
    while(i LESS ARG_COUNT)
      list(GET REMAINING_ARGS ${i} ARG)
      if(ARG STREQUAL ${TARGETS_VAR})
        math(EXPR i "${i} + 1")
        if(i LESS ARG_COUNT)
          list(GET REMAINING_ARGS ${i} TARGETS_VALUE)
          string(REPLACE ";" ";" GROUP_TARGETS ${TARGETS_VALUE})
          set(FOUND_TARGETS TRUE)
        endif()
        break()
      endif()
      math(EXPR i "${i} + 1")
    endwhile()
    
    if(FOUND_TARGETS AND GROUP_TARGETS)
      add_benchmark_group_runner(${group}
        TARGETS ${GROUP_TARGETS}
        TIMEOUT ${RUNNERS_TIMEOUT}
        OUTPUT_TIMEOUT ${RUNNERS_OUTPUT_TIMEOUT}
      )
    else()
      message(WARNING "No targets found for group '${group}' (looked for ${TARGETS_VAR})")
    endif()
  endforeach()
endfunction()

# =============================================================================



