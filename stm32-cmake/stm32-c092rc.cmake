cmake_minimum_required(VERSION 3.16.0)

get_filename_component(STM32_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
list(APPEND CMAKE_MODULE_PATH ${STM32_CMAKE_DIR})

add_definitions(-DSTM32C0)
add_definitions(-DSTM32C092xx)
add_definitions(-DPWR)
add_definitions(-DLL_USE_FULL_DRIVER)
add_definitions(-DUSE_FULL_LL_DRIVER)
add_definitions(-DGPIOD)
add_definitions(-DRCC_CR_SYSDIV=1)

# Workaround: HAL header only defines GPIOD for STM32C031xx, but C092 also has GPIOD
add_definitions(-DLL_IOP_GRP1_PERIPH_GPIOD=RCC_IOPENR_GPIODEN)

include(stm32/common)
include(stm32/devices)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(CMAKE_C_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CXX_COMPILER NAMES ${STM32_TARGET_TRIPLET}-g++ HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_ASM_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc HINTS ${TOOLCHAIN_BIN_PATH})
		
set(CMAKE_EXECUTABLE_SUFFIX_C   .elf)
set(CMAKE_EXECUTABLE_SUFFIX_CXX .elf)
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

set(CMAKE_C_COMPILER_FORCED   TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_ASM_COMPILER_FORCED TRUE)

set(CMAKE_STATIC_LIBRARY_PREFIX)
set(CMAKE_STATIC_LIBRARY_SUFFIX .a)

set(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
set(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

set(CMAKE_FIND_LIBRARY_PREFIXES "")
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")

# Other variable defs
set(STM32_BUILD ON CACHE BOOL "Build for STM32C0")

# C0 Family Configuration
set(STM_FAMILY C0)
set(STM_FAMILY_LONG_NAME STM32C0)

# C092RC specific configuration
# C0: Family, 92: Product line, R: 64-pin package, C: 256KB Flash
set(STM_PRODUCT C092RC)
set(STM_TYPE C092xx)

# Memory configuration for C092RC
# Flash: 256KB (C = 256KB), RAM: 32KB (typical for C092 series)
set(FLASH_SIZE 256K)
set(RAM_SIZE 32K)
set(CCRAM_SIZE 0K)
set(STACK_SIZE 0x4000)
set(HEAP_SIZE 0x200)

# Set the chip model (e.g., STM32C092RC) if not explicitly provided
set(STM32_CHIP "STM32C092RC" CACHE STRING "Specify the STM32 chip model (e.g., STM32C092RC)")

# Optimize for size to fit in 256KB flash
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -mthumb -Os -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-m0plus -mthumb -Os -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m0plus -mthumb")

# Override Release flags to use -Os instead of -O3 for STM32C0
set(CMAKE_C_FLAGS_RELEASE "-Os -g -DNDEBUG" CACHE STRING "C Release flags for STM32C0" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g -DNDEBUG" CACHE STRING "CXX Release flags for STM32C0" FORCE)

# Also set debug flags for completeness
set(CMAKE_C_FLAGS_DEBUG "-Og -g3 -DDEBUG" CACHE STRING "C Debug flags for STM32C0" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3 -DDEBUG" CACHE STRING "CXX Debug flags for STM32C0" FORCE)

# Linker flags to remove unused sections and optimize for size
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map,--cref")

# C0 Family Configuration
if (FETCH_ST_SOURCES)
  stm32_fetch_cmsis(C0)
  stm32_fetch_hal(C0)
endif()

# Explicitly set HAL components to avoid fallback to all families
set(HAL_FIND_COMPONENTS "STM32C0")

set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus) 
