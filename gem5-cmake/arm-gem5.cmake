# arm-gem5.cmake
# Use this toolchain file with the -DCMAKE_TOOLCHAIN_FILE=arm-gem5.cmake flag
# Author: Derin Ozturk
# Date of Creation: April, 2024

#ifdef analog
if(ARM_GEM5_TOOLCHAIN_INCLUDED)
  return()
endif(ARM_GEM5_TOOLCHAIN_INCLUDED)
set(ARM_GEM5_TOOLCHAIN_INCLUDED true)

# Locate ARM GNU Embedded Toolchain
FIND_FILE(ARM_BM_GCC_COMPILER "arm-none-eabi-gcc" PATHS ENV INCLUDE)
message("File: ${ARM_BM_GCC_COMPILER}")
if (EXISTS ${ARM_BM_GCC_COMPILER})
  message("Found ARM Baremetal GCC Toolchain: ${ARM_BM_GCC_COMPILER}")
else()
  message(FATAL_ERROR "ARM Baremetal GCC Toolchain not found!")
endif()

# Set GEM5_BUILD CMake var. Propagated everywhere else 
set(GEM5_BUILD ON CACHE BOOL "Build for gem5")
set(GEM5_ARMV7E-M_BUILD ON CACHE BOOL "Build for armv7e-m arch.")

# Get directory, name, and extension for toolchain
get_filename_component(ARM_BM_TOOLCHAIN_BIN_PATH ${ARM_BM_GCC_COMPILER} DIRECTORY)
get_filename_component(ARM_BM_TOOLCHAIN_BIN_GCC ${ARM_BM_GCC_COMPILER} NAME_WE)
get_filename_component(ARM_BM_TOOLCHAIN_BIN_EXT ${ARM_BM_GCC_COMPILER} EXT)

# Specify the cross compilers
set(CROSS_COMPILE arm-none-eabi-)
set(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++)
set(CMAKE_ASM_COMPILER ${CROSS_COMPILE}as)
set(CMAKE_OBJCOPY ${CROSS_COMPILE}objcopy)
set(CMAKE_OBJDUMP ${CROSS_COMPILE}objdump)

# Compiler and linker flags can be configured here or in the main CMakeLists.txt
set(CMAKE_C_FLAGS_INIT "-Wall -fno-exceptions -O3")
set(CMAKE_CXX_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} -std=c++20 -Wl,-Map,output.map")

# Platform specific configurations can also be set via cache variables or environment
set(CORE "cortex-m4" CACHE STRING "Set the processor core type")
set(FPU "fpv4-sp-d16" CACHE STRING "Set the FPU type")
set(FLOAT_ABI "hard" CACHE STRING "Set the float ABI type")
set(MARCH "armv7e-m" CACHE STRING "Set the architecture")

# Apply architecture-specific flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_INIT} ${CMAKE_C_FLAGS} -mcpu=${CORE} -mfpu=${FPU} -mfloat-abi=${FLOAT_ABI} -mthumb -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_INIT} ${CMAKE_CXX_FLAGS} -mcpu=${CORE} -mfpu=${FPU} -mfloat-abi=${FLOAT_ABI} -mthumb -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "-static -mcpu=${CORE} -mfpu=${FPU} -mfloat-abi=${FLOAT_ABI} --specs=nosys.specs --specs=nano.specs -nostartfiles -fno-exceptions --data-sections")

message("Using arm-gem5 toolchain file. \nCMAKE_C_FLAGS: ${CMAKE_C_FLAGS}\nCMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message("Toolchain path, prefix, and ext: ${RISCV_TOOLCHAIN_BIN_PATH}, ${RISCV_TOOLCHAIN_BIN_GCC}, ${RISCV_TOOLCHAIN_BIN_EXT}")

STRING(REGEX REPLACE "\-gcc" "-" CROSS_COMPILE ${ARM_BM_TOOLCHAIN_BIN_GCC})
message("Cross Compiling for ARM Cortex-M4 using ${CROSS_COMPILE} toolchain...")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR armv7e-m)

set(CMAKE_OBJCOPY ${ARM_BM_TOOLCHAIN_BIN_PATH}/${CROSS_COMPILE}objcopy CACHE FILEPATH "The toolchain objcopy command " FORCE)
set(CMAKE_OBJDUMP ${ARM_BM_TOOLCHAIN_BIN_PATH}/${CROSS_COMPILE}objdump CACHE FILEPATH "The toolchain objdump command " FORCE)

add_definitions(-DGEM5=1)
add_definitions(-DARM_GEM5=1)
