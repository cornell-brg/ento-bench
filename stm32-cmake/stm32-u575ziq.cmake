cmake_minimum_required(VERSION 3.16.0)

get_filename_component(STM32_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
list(APPEND CMAKE_MODULE_PATH ${STM32_CMAKE_DIR})

add_definitions(-DSTM32U5)
#add_definitions(-DSTM32U575xxQ)
add_definitions(-DPWR)
add_definitions(-DSMPS)
add_definitions(-DLL_USE_FULL_DRIVER)
add_definitions(-DUSE_FULL_LL_DRIVER)
add_definitions(-DHSE_VALUE=16000000)
add_definitions(-DHSE_STARTUP_TIMEOUT=100)
add_definitions(-DLSE_STARTUP_TIMEOUT=5000)
add_definitions(-DLSE_VALUE=32768)
add_definitions(-DMSI_VALUE=4000000)
add_definitions(-DEXTERNALSA1_CLOCK_VALUE=48000)
add_definitions(-DHSI_VALUE=16000000)
add_definitions(-DLSI_VALUE=32000)
add_definitions(-DVDD_VALUE=3300)

option(BUILD_U575ZIQ "Build for Nucleo board STM32U575ZIQ..." ON)


set(CMSIS_FIND_COMPONENTS "STM32U575ZI")
set(CMSIS_FIND_COMPONENTS_FAMILIES "STM32U575ZI")

include(stm32/common)
include(stm32/devices)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(CMAKE_C_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CXX_COMPILER NAMES ${STM32_TARGET_TRIPLET}-g++ HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_ASM_COMPILER NAMES ${STM32_TARGET_TRIPLET}-g++ HINTS ${TOOLCHAIN_BIN_PATH})

set(CMAKE_EXECUTABLE_SUFFIX_C   .elf)
set(CMAKE_EXECUTABLE_SUFFIX_CXX .elf)
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -fno-exceptions -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fno-exceptions -O0 -Wl,-Map,output.map -fno-rtti -std=c++20")
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -fno-exceptions -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fno-exceptions -O3 -Wl,-Map,output.map -fno-rtti -std=c++20")
endif()

message(STATUS "ASM COMPILER, ASM FLAGS: ${CMAKE_ASM_COMPILER}, ${CMAKE_ASM_FLAGS}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Other variable defs
set(STM32_BUILD ON CACHE BOOL "Build for STM32U5")

if(NOT STM_FAMILY)
  set(STM_FAMILY U5)
endif()

if(NOT STM_PRODUCT)
  set(STM_PRODUCT U575ZI)
  set(STM_TYPE U575xx)
endif()

list(TRANSFORM STM_FAMILY PREPEND STM32 OUTPUT_VARIABLE STM_FAMILY_LONG_NAME)
set(STM32_FAMILY_LONG_NAME "${STM32_FAMILY_LONG_NAME}")
message(STATUS "STM_FAMILY_LONG_NAME=${STM_FAMILY_LONG_NAME}")

message(STATUS "FETCH_ST_SOURCES=${FETCH_ST_SOURCES}")
# Ensure we have the HAL and CMSIS libraries
if (FETCH_ST_SOURCES)
  stm32_fetch_cmsis(U5)
  stm32_fetch_hal(U5)
endif()


set(CMAKE_SYSTEM_PROCESSOR armv7e-m)

set(STM32_CHIP "STM32U575ZI" CACHE STRING "Specify the STM32 chip model")

