# rv32-gem5.cmake
# Use this toolchain file with the -DCMAKE_TOOLCHAIN_FILE=arm-gem5.cmake flag
# Builds EntomotonBench for RV32
# Author: Derin Ozturk
# Date of Creation: April, 2024

#ifdef analog

if(RISCV_TOOLCHAIN_INCLUDED)
  return()
endif(RISCV_TOOLCHAIN_INCLUDED)
set(RISCV_TOOLCHAIN_INCLUDED true)

FIND_FILE(RISCV_GCC_COMPILER "riscv32-unknown-linux-gnu-gcc" PATHS ENV INCLUDE)
if (EXISTS ${RISCV_GCC_COMPILER})
message("Found RISC-V GCC Toolchain: ${RISCV_GCC_COMPILER}")
else()
message(FATAL_ERROR "RISC-V GCC Toolchain not found!")
endif()

get_filename_component(RISCV_TOOLCHAIN_BIN_PATH ${RISCV_GCC_COMPILER} DIRECTORY)
get_filename_component(RISCV_TOOLCHAIN_BIN_GCC ${RISCV_GCC_COMPILER} NAME_WE)
get_filename_component(RISCV_TOOLCHAIN_BIN_EXT ${RISCV_GCC_COMPILER} EXT)
message("Found RISCV toolchain path, prefix, and ext: ${RISCV_TOOLCHAIN_BIN_PATH}, ${RISCV_TOOLCHAIN_BIN_GCC}, ${RISCV_TOOLCHAIN_BIN_EXT}")

set(TOOLCHAIN_PREFIX riscv32-unknown-linux-gnu-)
set(CMAKE_C_COMPILER ${RISCV_TOOLCHAIN_BIN_PATH}/${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${RISCV_TOOLCHAIN_BIN_PATH}/${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${RISCV_TOOLCHAIN_BIN_PATH}/${TOOLCHAIN_PREFIX}as)
set(CMAKE_OBJCOPY ${RISCV_TOOLCHAIN_BIN_PATH}/${CROSS_COMPILE}objcopy CACHE FILEPATH "The toolchain objcopy command " FORCE)
set(CMAKE_OBJDUMP ${RISCV_TOOLCHAIN_BIN_PATH}/${CROSS_COMPILE}objdump CACHE FILEPATH "The toolchain objdump command " FORCE)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -fno-exceptions -O3 -flto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wl,-Map,output.map")


set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR RISCV)

add_compile_definitions(GEM5)
