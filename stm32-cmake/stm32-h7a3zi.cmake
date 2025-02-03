cmake_minimum_required(VERSION 3.16.0)

get_filename_component(STM32_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
list(APPEND CMAKE_MODULE_PATH ${STM32_CMAKE_DIR})

add_definitions(-DSTM32H7)
add_definitions(-DSTM32H7A3xxQ)
add_definitions(-DPWR)
add_definitions(-DLL_USE_FULL_DRIVER)
add_definitions(-DUSE_FULL_LL_DRIVER)
add_definitions(-DSMPS)

option(BUILD_H7A3ZIQ "Build for Nucleo board STM32H7A3ZI..." ON)


set(CMSIS_FIND_COMPONENTS "STM32H7A3ZI_M7")
set(CMSIS_FIND_COMPONENTS_FAMILIES "STM32H7A3ZI_M7")

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
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -fno-exceptions -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fno-exceptions -O3 -Wl,-Map,output.map -fno-rtti -std=c++20")
set(CMAKE_ASM_FLAGS "")

message(STATUS "ASM COMPILER, ASM FLAGS: ${CMAKE_ASM_COMPILER}, ${CMAKE_ASM_FLAGS}")

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

list(TRANSFORM STM_FAMILY PREPEND STM32 OUTPUT_VARIABLE STM_FAMILY_LONG_NAME)
set(STM32_FAMILY_LONG_NAME "${STM32_FAMILY_LONG_NAME}_M7")
message(STATUS "STM_FAMILY_LONG_NAME=${STM_FAMILY_LONG_NAME}")

message(STATUS "FETCH_ST_SOURCES=${FETCH_ST_SOURCES}")
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

#message(STATUS "Stack size: ${STACK_SIZE}")
#message(STATUS "Stack origin: ${STACK_ORIGIN}")
#message(STATUS "Heap size: ${HEAP_SIZE}")
#message(STATUS "Heap origin: ${HEAP_ORIGIN}")

# Customize stack and heap sizes
#set(MY_STACK_SIZE "0x400")  # For example, set to 2KB
#set(MY_HEAP_SIZE "0x19000")   # For example, set to 1KB

# Pass these custom sizes to the linker
#add_compile_definitions(
#    STACK_SIZE=${MY_STACK_SIZE}
#    HEAP_SIZE=${MY_HEAP_SIZE}
#)

