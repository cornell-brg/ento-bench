cmake_minimum_required(VERSION 3.16.0)

get_filename_component(STM32_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
list(APPEND CMAKE_MODULE_PATH ${STM32_CMAKE_DIR})

add_definitions(-DUSE_FULL_LL_DRIVER)
add_definitions(-DSTM32F756xx)
add_definitions(-DPWR)
add_definitions(-DRCC)
add_definitions(-DHSE_VALUE=8000000)
add_definitions(-DHSE_STARTUP_TIMEOUT=100)
add_definitions(-DLSE_STARTUP_TIMEOUT=5000)
add_definitions(-DLSE_VALUE=32768)
add_definitions(-DEXTERNAL_CLOCK_VALUE=12288000)
add_definitions(-DHSI_VALUE=160000000)
add_definitions(-DLSI_VALUE=32000)
add_definitions(-DVDD_VALUE=3300)
add_definitions(-DPREFETCH_ENABLE=0)
add_definitions(-DART_ACCELERATOR_ENABLE=1)

include(stm32/common)
include(stm32/devices)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(CMAKE_C_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CXX_COMPILER NAMES ${STM32_TARGET_TRIPLET}-g++ HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_ASM_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc HINTS ${TOOLCHAIN_BIN_PATH})
		
set(CMAKE_EXECUTABLE_SUFFIX_C   .elf)
set(CMAKE_EXECUTABLE_SUFFIX_CXX .elf)
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -fno-exceptions -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fno-exceptions -O3 -Wl,-Map,output.map -fno-rtti -std=c++20")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-u,cmdline_buffer")


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Other variable defs
set(STM32_BUILD ON CACHE BOOL "Build for STM32 Microcontroller...")

if(NOT STM_FAMILY)
  set(STM_FAMILY F7)
endif()

if(NOT STM_PRODUCT)
  set(STM_PRODUCT F756ZG)
endif()

if (NOT STM_TYPE)
  set(STM_TYPE F756xx)
endif()

list(TRANSFORM STM_FAMILY PREPEND STM32 OUTPUT_VARIABLE STM_FAMILY_LONG_NAME)
if (FETCH_ST_SOURCES)
  stm32_fetch_cmsis(${STM_FAMILY})
  stm32_fetch_hal(${STM_FAMILY})
endif()

set(CMAKE_SYSTEM_PROCESSOR armv7e-m)

add_definitions(-DSTM32F7)
# Ensure we have the HAL and CMSIS libraries
#find_package(CMSIS REQUIRED ${STM_FAMILY_LONG_NAME})
#find_package(HAL REQUIRED ${STM_FAMILY_LONG_NAME})

# Set the chip model (e.g., STM32G474RE) if not explicitly provided
set(STM32_CHIP "STM32756ZG" CACHE STRING "Specify the STM32 chip model (e.g., STM32G474RE)")

