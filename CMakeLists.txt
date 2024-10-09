cmake_minimum_required(VERSION 3.16)

project(insect-scale-benchmark-suite C CXX ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(FETCH_ST_SOURCES "Fetch STMicro CMSIS/HAL Libraries." OFF)
option(STM32_BUILD "Build for STM32." OFF)
option(GEM5_BUILD "Build for gem5." OFF)
option(SEMIHOSTING "Enable semihosting when deployed on MCU." OFF)

# Needed to use inline versions of critical routines in GSL
add_definitions(-DHAVE_INLINE)
message("CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message("CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

add_compile_definitions($<$<CONFIG:Debug>:DEBUG>)
add_compile_definitions($<$<CONFIG:Release>:NDEBUG>)

if(STM32_BUILD)
  message(STATUS "Building for ${STM_FAMILY_LONG_NAME}")
  find_package(CMSIS COMPONENTS ${STM_FAMILY_LONG_NAME} REQUIRED)
  find_package(HAL COMPONENTS ${STM_FAMILY_LONG_NAME} LL_RCC LL_GPIO REQUIRED)
  add_definitions(-DSTM32_BUILD=1)

  # Setup OpenOCD flash and debug targets.
  option(STM32_DEVICE "STM32 device target (e.g., st32g4x.cfg)" "stm32g4x.cfg")

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
          /usr/share/openocd/scripts/interface
          /opt/local/share/openocd/scripts/interface
          /opt/homebrew/Cellar/open-ocd/0.12.0_1/share/openocd/scripts/interface/stlink.cfg
  )

  if(NOT OPENOCD_INTERFACE)
    message(FATAL_ERROR "Could not find OpenOCD interface configuration (stlink.cfg). Please set OPENOCD_INTERFACE manually.")
  else()
    message(STATUS "Using OpenOCD interface: ${OPENOCD_INTERFACE}")
  endif()

  # The STM32 device configuration must be passed in manually
  if(NOT DEFINED STM32_DEVICE)
    message(FATAL_ERROR "Please provide the STM32_DEVICE configuration file (e.g., st_nucleo_g4.cfg).")
  else()
    message(STATUS "Using STM32 device config: ${STM32_DEVICE}")
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
  set(CMAKE_C_COMPILER clang)
  set(CMAKE_CXX_COMPILER clang++)
  set(CMAKE_ASM_COMPILER clang)
  set(CMAKE_CXX_FLAGS "-Wall -Wextra -g -gdwarf-4 -std=c++20 -O0")
	message(STATUS "Building for native.")
endif()

set(SRC_DIR src)
set(BIN_DIR bin)
set(APP_DIR apps)
set(UB_DIR ubench)
set(SANDBOX_DIR sandbox)
set(BENCHMARK_DIR benchmark)
set(EXAMPLE_DIR example)

# Get GSL lib. This depends on the arch we are building for.
# Figure out arch
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "riscv64")
  set(GSL_BUILD gsl-riscv64)
  add_definitions(-DRV64)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7e-m")
  set(GSL_BUILD gsl-armv7e-m)
  add_definitions(-DARMV7E_M)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "riscv32")
  set(GSL_BUILD gsl-riscv32)
  add_definitions(-DRV32)
else()
  # Default or other specific configurations
endif()


# Setup Linking GSL
set(GSL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/gsl/build/${GSL_BUILD} CACHE PATH "Path to compiled GSL")
set(GSL_LIB ${GSL_DIR}/lib/libgsl.a CACHE PATH "Path to GSL static lib.")
set(GSL_BLAS_LIB ${GSL_DIR}/lib/libgslcblas.a CACHE PATH "Path to GSL CBLAS static lib.")
set(GSL_INC ${GSL_DIR}/include CACHE PATH "Path to include directory of GSL")
message("GSL_DIR in top: ${GSL_DIR}")

set(EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen CACHE PATH "Path to Eigen library")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})

# Subdirs are only for Cmake to traverse down and autogenerate Makefiles
add_subdirectory(${SRC_DIR})
add_subdirectory(${APP_DIR})
add_subdirectory(${SANDBOX_DIR})
add_subdirectory(${EXAMPLE_DIR})

message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "Linker options: ${CMAKE_EXE_LINKER_FLAGS}")
