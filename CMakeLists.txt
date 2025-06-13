cmake_minimum_required(VERSION 3.16)

project(ento-bench CXX C ASM)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(entobench_helpers.cmake)

get_property(languages GLOBAL PROPERTY ENABLED_LANGUAGES)
message(STATUS "Languages in main CMakeLists.txt after project(): ${languages}")

option(FETCH_ST_SOURCES "Fetch STMicro CMSIS/HAL Libraries." OFF)
option(STM32_BUILD "Build for STM32." OFF)
option(GEM5_BUILD "Build for gem5." OFF)
option(SEMIHOSTING "Enable semihosting when deployed on MCU." OFF)

enable_testing()

# Add a custom make check target that runs ctest with 60s timeout
add_custom_target(check COMMAND ctest "--timeout" "60")

# =============================================================================
# Pose Estimation Benchmark Runner Targets
# =============================================================================

# Make the benchmark script executable
set(BENCHMARK_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/run_pose_benchmarks.sh")

# Create the scripts directory and copy the script
file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/scripts")
configure_file("${CMAKE_SOURCE_DIR}/scripts/run_pose_benchmarks.sh" 
               "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" 
               COPYONLY)

# Make script executable
if(UNIX)
    execute_process(COMMAND chmod +x "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh")
endif()

# Benchmark group targets
add_custom_target(bench-pose-quick
    COMMAND "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" --group quick-test --timeout 60 --output-timeout 10
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running quick pose estimation benchmarks (P3P, 8pt, DLT float)"
    USES_TERMINAL
)

add_custom_target(bench-pose-absolute
    COMMAND "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" --group absolute-pose --timeout 60 --output-timeout 10
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running absolute pose benchmarks (P3P, UP2P, DLT)"
    USES_TERMINAL
)

add_custom_target(bench-pose-relative
    COMMAND "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" --group relative-pose --timeout 60 --output-timeout 10
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running relative pose benchmarks (8pt, 5pt, upright 3pt)"
    USES_TERMINAL
)

add_custom_target(bench-pose-gold-standard
    COMMAND "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" --group gold-standard --timeout 120 --output-timeout 15
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running gold standard pose benchmarks (H&Z reference implementations)"
    USES_TERMINAL
)

add_custom_target(bench-pose-scaling
    COMMAND "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" --group dlt-scaling --timeout 90 --output-timeout 15
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running DLT scaling benchmarks (8, 16, 32, 64 points)"
    USES_TERMINAL
)

add_custom_target(bench-pose-all
    COMMAND "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" --group all-pose --timeout 90 --output-timeout 15
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running all basic pose estimation benchmarks"
    USES_TERMINAL
)

add_custom_target(bench-pose-performance
    COMMAND "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" --group performance --timeout 120 --output-timeout 20
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running comprehensive pose estimation performance evaluation"
    USES_TERMINAL
)

# Convenience target to list available benchmark groups
add_custom_target(bench-pose-list
    COMMAND "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" --list-groups --verbose
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "List available pose estimation benchmark groups"
    USES_TERMINAL
)

# Help target for pose benchmarks
add_custom_target(bench-pose-help
    COMMAND "${CMAKE_BINARY_DIR}/run_pose_benchmarks.sh" --help
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Show pose benchmark runner help"
    USES_TERMINAL
)

# Set path to compile_commands.json based on the current build directory
set(COMPILE_COMMANDS_DIR "${CMAKE_BINARY_DIR}")

# Path to the .clangd file
set(CLANGD_FILE "${CMAKE_SOURCE_DIR}/.clangd")
message(STATUS ".clangd file: ${CLANGD_FILE}")

# Path to the .clangd file
set(CLANGD_FILE "${CMAKE_SOURCE_DIR}/.clangd")
message(STATUS ".clangd file: ${CLANGD_FILE}")

# Start building the .clangd file with the compilation database path
add_custom_target(update_clangd ALL
  COMMAND ${CMAKE_COMMAND} -E echo "CompileFlags:" > ${CLANGD_FILE}
  COMMAND ${CMAKE_COMMAND} -E echo "  CompilationDatabase: ${COMPILE_COMMANDS_DIR}" >> ${CLANGD_FILE}
  COMMENT "Updating .clangd file with compile_commands.json path"
)


# Eigen Flags
add_definitions(
    -DEIGEN_DEFAULT_L1_CACHE_SIZE=1000000
    -DEIGEN_DEFAULT_L2_CACHE_SIZE=0
    -DEIGEN_DEFAULT_L3_CACHE_SIZE=0
    -DEIGEN_UNROLLING_LIMIT=200
    -DEIGEN_NO_MALLOC
    -DEIGEN_NO_DEBUG
    -DEIGEN_FAST_MATH=1
    -DEIGEN_DONT_PARALLELIZE
    -DEIGEN_DONT_VECTORIZE
)

# Conditionally add ARM-specific include paths if STM32_BUILD is set
if(STM32_BUILD)
  # Find the ARM toolchain executable
  find_program(ARM_GCC arm-none-eabi-gcc)
  find_program(ARM_GXX arm-none-eabi-g++)

  if(LATENCY_MEASUREMENT)
    add_definitions(-DLATENCY_MEASUREMENT)
  endif()

  if(ARM_GCC AND ARM_GXX)
    get_filename_component(ARM_TOOLCHAIN_BASE "${ARM_GCC}" DIRECTORY)
    get_filename_component(ARM_TOOLCHAIN_BASE "${ARM_TOOLCHAIN_BASE}" DIRECTORY)

    # Retrieve the paths for newlib and GCC-specific includes
    execute_process(
      COMMAND ${ARM_GCC} -print-file-name=include
      OUTPUT_VARIABLE ARM_GCC_INCLUDE
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set(ARM_NEWLIB_INCLUDE "${ARM_TOOLCHAIN_BASE}/arm-none-eabi/include")
    set(ARM_NEWLIB_CXX_INCLUDE "${ARM_TOOLCHAIN_BASE}/arm-none-eabi/include/c++/13.3.1")


    message(STATUS "Found ${ARM_GCC}... \n ${ARM_GCC_INCLUDE}, ${ARM_NEWLIB_INCLUDE}")

    # Append ARM-specific paths directly to .clangd within the same target
    add_custom_command(
      TARGET update_clangd
      COMMAND ${CMAKE_COMMAND} -E echo "  Add:" >> ${CLANGD_FILE}
      COMMAND ${CMAKE_COMMAND} -E echo "   - -I${ARM_NEWLIB_INCLUDE}" >> ${CLANGD_FILE}
      COMMAND ${CMAKE_COMMAND} -E echo "   - -I${ARM_NEWLIB_CXX_INCLUDE}" >> ${CLANGD_FILE}
      COMMAND ${CMAKE_COMMAND} -E echo "   - -I${ARM_GCC_INCLUDE}" >> ${CLANGD_FILE}
      COMMENT "Appending ARM-specific include paths to .clangd"
    )


  else()
    message(WARNING "arm-none-eabi-gcc not found; .clangd will not include ARM-specific include paths.")
  endif()
endif()

# Ensure update_clangd runs after the project is built
#:add_dependencies(update_clangd check)

# Needed to use inline versions of critical routines in GSL
# add_definitions(-DHAVE_INLINE)

add_definitions(-DDATASET_PATH="${CMAKE_SOURCE_DIR}/datasets")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

add_compile_definitions($<$<CONFIG:Debug>:DEBUG>)
add_compile_definitions($<$<CONFIG:Release>:NDEBUG>)

if(STM32_BUILD)
  add_definitions(-DSTM32_BUILD=1)
  message(STATUS "Building for ${STM_FAMILY_LONG_NAME}")
  message(STATUS "HAL FAMILY PATH ${HAL_${FAMILY}_PATH}")

  message(STATUS "STM Product: ${STM_PRODUCT}")
  message(STATUS "STM Family: ${STM_FAMILY}")

  # Set HAL and CMSIS target library variables dynamically
  if(${STM_FAMILY} STREQUAL "H7")
    # Special case for H7 family which requires an additional M7
    set(CORE_U M7)
    set(CORE_C "::M7")
    set(CMSIS_FAMILY_TARGET "CMSIS::STM32::${STM_FAMILY}${CORE_C}")
    set(CMSIS_TYPE_TARGET "CMSIS::STM32::${STM_TYPE}${CORE_C}")
    set(HAL_RCC_TARGET "HAL::STM32::${STM_FAMILY}::M7::LL_RCC")
    set(HAL_GPIO_TARGET "HAL::STM32::${STM_FAMILY}::M7::LL_GPIO")
    set(HAL_FLASH_TARGET "HAL::STM32::${STM_FAMILY}::M7::LL_FLASH")
    set(HAL_UTILS_TARGET "HAL::STM32::${STM_FAMILY}::M7::LL_UTILS")
    set(HAL_PWR_TARGET "HAL::STM32::${STM_FAMILY}::M7::LL_PWR")
    set(HAL_CORTEX_TARGET "HAL::STM32::${STM_FAMILY}::M7::CORTEX")
    set(STM_FAMILY_LONG_NAME ${STM_FAMILY_LONG_NAME}_M7)
    set(STM32_TARGET "STM32::${STM_TYPE}::M7")
  else()
    set(CORE_U "")
    set(CORE_C "")
    set(CMSIS_FAMILY_TARGET "CMSIS::STM32::${STM_FAMILY}")
    set(CMSIS_TYPE_TARGET "CMSIS::STM32::${STM_TYPE}")
    set(HAL_RCC_TARGET "HAL::STM32::${STM_FAMILY}::LL_RCC")
    set(HAL_GPIO_TARGET "HAL::STM32::${STM_FAMILY}::LL_GPIO")
    set(HAL_FLASH_TARGET "HAL::STM32::${STM_FAMILY}::LL_FLASH")
    set(HAL_UTILS_TARGET "HAL::STM32::${STM_FAMILY}::LL_UTILS")
    set(HAL_PWR_TARGET "HAL::STM32::${STM_FAMILY}::LL_PWR")
    set(HAL_CORTEX_TARGET "HAL::STM32::${STM_FAMILY}::CORTEX")
    set(STM32_TARGET "STM32::${STM_TYPE}${CORE_C}")
  endif()


  # Log HAL and CMSIS target library variables
  message(STATUS "CMSIS family target: ${CMSIS_FAMILY_TARGET}")
  message(STATUS "CMSIS type target: ${CMSIS_TYPE_TARGET}")
  message(STATUS "HAL RCC target: ${HAL_RCC_TARGET}")
  message(STATUS "HAL GPIO target: ${HAL_GPIO_TARGET}")
  message(STATUS "HAL FLASH target: ${HAL_FLASH_TARGET}")
  message(STATUS "HAL UTILS target: ${HAL_UTILS_TARGET}")
  message(STATUS "HAL PWR target: ${HAL_PWR_TARGET}")

  #if (FETCH_ST_SOURCES)
  #  stm32_fetch_cmsis(${STM32_FAMILY})
  #  stm32_fetch_hal(${STM32_FAMILY})
  #endif()



  #find_package(CMSIS COMPONENTS ${STM_FAMILY_LONG_NAME} REQUIRED)
  find_package(CMSIS COMPONENTS ${STM_FAMILY_LONG_NAME}::${CORE_C} REQUIRED)
  #find_package(CMSIS COMPONENTS CMSIS::STM32::H7::M7 REQUIRED)
  find_package(HAL COMPONENTS ${STM_FAMILY_LONG_NAME}::${CORE_C} REQUIRED)


  get_property(all_definitions DIRECTORY PROPERTY COMPILE_DEFINITIONS)

  #cmsis_generate_parametrizable_linker_script(
  #  ${STM_FAMILY}
  #  ${STM_PRODUCT}
  #  ""
  #  STACK_SIZE ${STACK_SIZE}
  #  )

  set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

  # ========================================================
  # Setup OpenOCD flash and debug targets.
  # ========================================================

  option(OPENOCD_CFG "STM32 device target (e.g., st32g4x.cfg)" "stm32g4x.cfg")

  if(DEFINED ENV{OPENOCD_HOME})
      set(OPENOCD_ROOT "$ENV{OPENOCD_HOME}")
    elseif(DEFINED ENV{OPENOCD_PATH})
      set(OPENOCD_ROOT "$ENV{OPENOCD_PATH}")
    else()
      set(OPENOCD_ROOT "/usr/local/share/openocd/scripts")  # Default fallback
    endif()

  # Search for the OpenOCD interface config file (stlink.cfg)
  find_file(OPENOCD_INTERFACE
    NAMES stlink.cfg
    PATHS ${OPENOCD_ROOT}/interface
          ~/research/openocd/tcl/interface
          /usr/share/openocd/scripts/interface
          /opt/local/share/openocd/scripts/interface
          /opt/homebrew/Cellar/open-ocd/0.12.0_1/share/openocd/scripts/interface
          /opt/homebrew/Cellar/open-ocd/HEAD-1f3f635_1/share/openocd/scripts/interface
  )

  if(NOT OPENOCD_INTERFACE)
    message(FATAL_ERROR "Could not find OpenOCD interface configuration (stlink.cfg). Please set OPENOCD_INTERFACE manually.")
  else()
    message(STATUS "Using OpenOCD interface: ${OPENOCD_INTERFACE}")
  endif()

  # The STM32 device configuration must be passed in manually
  if(NOT DEFINED OPENOCD_CFG)
    message(FATAL_ERROR "Please provide the OpenOCD configuration file (e.g., st_nucleo_g4.cfg).")
  else()
    message(STATUS "Using STM32 device config: ${OPENOCD_CFG}")
  endif()


elseif(GEM5_BUILD)
	message(STATUS "Building for gem5.")

  # If env variable for GEM5_PATH is not set, then look for it.
  set(GEM5_PATH "$ENV{GEM5_PATH}" CACHE PATH "Path to gem5 installation.")
  set(STARTUP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/startup" CACHE PATH "Path to startup directory.")

  if(NOT GEM5_PATH)
    find_path(GEM5_PATH NAMES gem5 PATH)
  endif()

else() # Native Build
  if(APPLE)
    find_program(GCC_PATH NAMES gcc-14 gcc)
    find_program(GXX_PATH NAMES g++-14 g++)
    
    if(GCC_PATH AND GXX_PATH)
      message(STATUS "Using GCC: ${GCC_PATH} / ${GXX_PATH}")
      set(CMAKE_C_COMPILER ${GCC_PATH} CACHE STRING "GCC Compiler" FORCE)
      set(CMAKE_CXX_COMPILER ${GXX_PATH} CACHE STRING "G++ Compiler" FORCE)
    else()
      message(WARNING "GCC 14 not found, falling back to Clang")
      set(CMAKE_C_COMPILER clang)
      set(CMAKE_CXX_COMPILER clang++)
    endif()
  endif()

	message(STATUS "Building for native.")
  set(CMAKE_C_COMPILER gcc)
  set(CMAKE_CXX_COMPILER g++)
  set(CMAKE_ASM_COMPILER gcc)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "-std=c++20 -g3 -O3")
  else()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++20 -O3")
  endif()
  add_definitions(-DNATIVE)
endif()

set(SRC_DIR src)
set(BIN_DIR bin)
set(APP_DIR apps)
set(UB_DIR ubench)
set(SANDBOX_DIR sandbox)
set(BENCHMARK_DIR benchmark)
set(EXAMPLE_DIR example)

# Figure out arch
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "riscv64")
  message(STATUS "Detected arch: riscv64")
  add_definitions(-DRV64)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7e-m")
  message(STATUS "Detected arch: armv7e-m")
  add_definitions(-DARMV7E_M)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "riscv32")
  message(STATUS "Detected arch: riscv32")
  add_definitions(-DRV32)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|i686|i386")
  message(STATUS "Detected arch: x86_64|i686|i386")
  add_definitions(-DX86)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
  message(STATUS "Detected arch: arm64|aarch64")
  add_definitions(-DARM64)
else()
  message(WARNING "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  add_definitions(-DGENERIC_ARCH)
endif()

set(EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen CACHE PATH "Path to Eigen library")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})

# Subdirs are only for Cmake to traverse down and autogenerate Makefiles
add_subdirectory(${SRC_DIR})
add_subdirectory(${SANDBOX_DIR})
add_subdirectory(${EXAMPLE_DIR})
add_subdirectory(${BENCHMARK_DIR})


if(BUILD_H7A3ZIQ)
    # Set the path to the stm32h7xx.h file in the build directory
    set(STM32H7_HEADER_PATH "${CMAKE_BINARY_DIR}/_deps/stm32-cmsis-h7-src/Include/stm32h7xx.h")

    # Only attempt to patch if the file exists (i.e., after it has been downloaded or generated)
    if(EXISTS "${STM32H7_HEADER_PATH}")
        # Read the contents of stm32h7xx.h
        file(READ "${STM32H7_HEADER_PATH}" FILE_CONTENTS)
        if(FILE_CONTENTS MATCHES "#elif defined\\(STM32H7A3xx\\) && !defined\\(STM32H7A3xxQ\\)")
            message(STATUS "The file ${STM32H7_HEADER_PATH} has already been patched. Skipping modification.")
        else()
            # Perform the replacement if the line hasn't been modified
            string(REPLACE "#elif defined(STM32H7A3xx)" "#elif defined(STM32H7A3xx) && !defined(STM32H7A3xxQ)" FILE_CONTENTS "${FILE_CONTENTS}")

            # Write the modified content back to stm32h7xx.h
            file(WRITE "${STM32H7_HEADER_PATH}" "${FILE_CONTENTS}")

            message(STATUS "Patched ${STM32H7_HEADER_PATH} for H7A3ZIQ build.")
        endif()
    else()
        message(WARNING "File ${STM32H7_HEADER_PATH} not found. Patch not applied.")
    endif()
endif()
