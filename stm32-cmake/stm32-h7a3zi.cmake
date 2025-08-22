cmake_minimum_required(VERSION 3.16.0)

get_filename_component(STM32_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
list(APPEND CMAKE_MODULE_PATH ${STM32_CMAKE_DIR})

add_definitions(-DSTM32H7)
add_definitions(-DSTM32H7A3xxQ)
add_definitions(-DPWR)
add_definitions(-DLL_USE_FULL_DRIVER)
add_definitions(-DUSE_FULL_LL_DRIVER)
add_definitions(-DSMPS)

# FPU-related definitions for debugging and optimization
add_definitions(-D__FPU_PRESENT=1)             # Hardware FPU is present
add_definitions(-D__FPU_USED=1)                # Hardware FPU is being used
add_definitions(-DARM_MATH_CM7)                # Enable ARM CMSIS-DSP optimizations for Cortex-M7
add_definitions(-DARM_MATH_MATRIX_CHECK)       # Enable matrix operation bounds checking
add_definitions(-DARM_MATH_ROUNDING)           # Enable proper rounding in ARM math functions
add_definitions(-DHARDWARE_FPU_DOUBLE)         # Custom flag to indicate double-precision FPU usage

option(BUILD_H7A3ZIQ "Build for Nucleo board STM32H7A3ZI..." ON)


# Force specific device component to avoid processing entire H7 family
set(CMSIS_FIND_COMPONENTS "STM32H7A3ZI_M7" CACHE INTERNAL "Specific CMSIS component for H7A3ZI")
set(CMSIS_FIND_COMPONENTS_FAMILIES "STM32H7A3ZI_M7" CACHE INTERNAL "Specific CMSIS families for H7A3ZI")
# Override the default to prevent fallback to all families
set(STM32_SUPPORTED_FAMILIES_LONG_NAME "STM32H7A3ZI_M7" CACHE INTERNAL "Override default families for H7A3ZI")

include(stm32/common)
include(stm32/devices)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(CMAKE_C_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CXX_COMPILER NAMES ${STM32_TARGET_TRIPLET}-g++ HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_ASM_COMPILER NAMES ${STM32_TARGET_TRIPLET}-g++ HINTS ${TOOLCHAIN_BIN_PATH})

set(CMAKE_EXECUTABLE_SUFFIX_C   .elf)
set(CMAKE_EXECUTABLE_SUFFIX_CXX .elf)
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)

set(CMAKE_ASM_FLAGS_INIT "")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -fno-exceptions -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fno-exceptions -O0 -Wl,-Map,output.map -fno-rtti -std=c++20")
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -fno-exceptions -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fno-exceptions -O3 -Wl,-Map,output.map -fno-rtti -std=c++20 -fstack-usage")
endif()

# Explicit FPU flags for STM32H7 double-precision performance
set(FPU_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard")

# Apply FPU flags to both C and C++ compilers
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FPU_FLAGS} ${FPU_OPTIMIZATION_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FPU_FLAGS} ${FPU_OPTIMIZATION_FLAGS}")

# Apply FPU flags to linker
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${FPU_FLAGS} -Wl,--no-warn-mismatch")

set(CMAKE_ASM_FLAGS "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Other variable defs
set(STM32_BUILD ON CACHE BOOL "Build for STM32H7")

if(NOT STM_FAMILY)
  set(STM_FAMILY H7)
endif()

if(NOT STM_PRODUCT)
  set(STM_PRODUCT H7A3ZI::M7)
  set(STM_TYPE H7A3xx)
endif()

list(TRANSFORM STM_FAMILY PREPEND STM32 OUTPUT_VARIABLE STM32_FAMILY_LONG_NAME)
set(STM32_FAMILY_LONG_NAME "${STM32_FAMILY_LONG_NAME}_M7")
# Ensure we have the HAL and CMSIS libraries
if (FETCH_ST_SOURCES)
  stm32_fetch_cmsis(H7)
  stm32_fetch_hal(H7)
endif()

#message("LONG_NAME ${STM_FAMILY_LONG_NAME}")
#find_package(CMSIS REQUIRED ${STM_FAMILY_LONG_NAME})
#find_package(HAL REQUIRED ${STM_FAMILY_LONG_NAME})

set(CMAKE_SYSTEM_PROCESSOR armv7e-m)



set(STACK_SIZE "0x200")
set(HEAP_SIZE "0x400")
set(RAM_SIZE "1024K")
set(HEAP_ORIGIN "")
set(STM32_CHIP "STM32H7A3ZI" CACHE STRING "Specify the STM32 chip model")
