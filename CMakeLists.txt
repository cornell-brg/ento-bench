cmake_minimum_required(VERSION 3.16)

project(insect-scale-benchmark-suite C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(FETCH_ST_SOURCES "Fetch STMicro CMSIS/HAL Libraries." OFF)
option(IS_STM32G4_BUILD "Build for STM32G4." OFF)
option(IS_GEM5 "Build for gem5." OFF)


if(IS_STM32G4_BUILD)
	message(STATUS "Building for STM32G4")
  find_package(CMSIS COMPONENTS STM32G4 REQUIRED)
  find_package(HAL COMPONENTS STM32G4 LL_RCC LL_GPIO REQUIRED)
elseif(IS_GEM5_BUILD)
	message(STATUS "Building for gem5.")

  # If env variable for GEM5_PATH is not set, then look for it.
  set(GEM5_PATH "$ENV{GEM5_PATH}" CACHE PATH "Path to gem5 installation.")
  if(NOT GEM5_PATH)
    find_path(GEM5_PATH NAMES gem5 PATH)
  endif()

  # Define the expected sub-directory for libm5.a
  set(GEM5_LIB_PATH "${GEM5_PATH}/util/m5/build/arm/out")

  # Check if the specific subdirectory exists
  if(EXISTS "${GEM5_LIB_PATH}")
      message(STATUS "Found GEM5 library path: ${GEM5_LIB_PATH}")
  else()
      message(FATAL_ERROR "GEM5 library path not found at ${GEM5_LIB_PATH}")
  endif()
  if(EXISTS "${GEM5_LIB_PATH}/libm5.a")
      message(STATUS "Found gem5 libm5.a path: ${GEM5_LIB_PATH}")
  else()
      message(FATAL_ERROR "GEM5 library path not found at ${GEM5_LIB_PATH}")
  endif()
else()
	message(STATUS "Building for native.")
endif()

set(SRC_DIR src)
set(BIN_DIR bin)
set(APP_DIR apps)
set(UB_DIR ubench)
set(SANDBOX_DIR sandbox)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})


set(CMAKE_C_FLAGS "-Wall -Wextra -O3 -std=c11")

set(MODULES
  linalg
  stm32-util
)

set(APPS
  ubench
)

include_directories(
  linalg
  stm32-util
)

foreach(MODULE ${MODULES})
  add_subdirectory(${SRC_DIR}/${MODULE})
endforeach()

add_subdirectory(${APP_DIR})
add_subdirectory(${SANDBOX_DIR})

#===============================
#======Microbenchmarking========
#[===[
if (IS_STM32G4_BUILD)
add_executable(ub_vvadd ${UB_DIR}/vv_add.c)
target_link_libraries(
  ub_vvadd
  PRIVATE
  linalg_lib
  HAL::STM32::G4::LL_RCC
  HAL::STM32::G4::LL_GPIO
  CMSIS::STM32::G474RE
  STM32::NoSys
)

add_executable(ub_sumreduction ${UB_DIR}/sum_reduction.c)
target_link_libraries(
  ub_sumreduction
  PRIVATE
  linalg_lib
  HAL::STM32::G4::LL_RCC
  HAL::STM32::G4::LL_GPIO
  CMSIS::STM32::G474RE
  STM32::NoSys
)
add_executable(ub_dotprod ${APP_DIR}/ubench/dot_prod.c)
target_link_libraries(
  ub_dotprod
  PRIVATE
  linalg_lib
  stm32util_lib
  HAL::STM32::G4::LL_RCC
  HAL::STM32::G4::LL_GPIO
  CMSIS::STM32::G474RE
  STM32::NoSys
)
elseif(IS_GEM5_BUILD)

else()
add_executable(ub_vvadd ${UB_DIR}/vv_add.c)
target_link_libraries(
  ub_vvadd
  PRIVATE
  linalg_lib
)

add_executable(ub_sumreduction ${UB_DIR}/sum_reduction.c)
target_link_libraries(
  ub_sumreduction
  PRIVATE
  linalg_lib
)

add_executable(ub_dotprod ${UB_DIR}/dot_prod.c)
target_link_libraries(
  ub_dotprod
  PRIVATE
  linalg_lib
)
endif()
#]===]
