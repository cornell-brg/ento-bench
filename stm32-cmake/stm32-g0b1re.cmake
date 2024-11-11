cmake_minimum_required(VERSION 3.16.0)

get_filename_component(STM32_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
list(APPEND CMAKE_MODULE_PATH ${STM32_CMAKE_DIR})

add_definitions(-DSTM32G0)
add_definitions(-DPWR)
add_definitions(-DLL_USE_FULL_DRIVER)
add_definitions(-DUSE_FULL_LL_DRIVER)

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


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Other variable defs
set(STM32_BUILD ON CACHE BOOL "Build for STM32G0")

if(NOT STM_FAMILY)
  set(STM_FAMILY G0)
endif()

if(NOT STM_PRODUCT)
  set(STM_PRODUCT G01BRE)
endif()

list(TRANSFORM STM_FAMILY PREPEND STM32 OUTPUT_VARIABLE STM_FAMILY_LONG_NAME)
if (FETCH_ST_SOURCES)
  stm32_fetch_cmsis(G0)
  stm32_fetch_hal(G0)
endif()

set(CMAKE_SYSTEM_PROCESSOR armv7e-m)


# Set the chip model (e.g., STM32G01BRE) if not explicitly provided
set(STM32_CHIP "STM32G01BRE" CACHE STRING "Specify the STM32 chip model (e.g., STM32G01BRE)")



