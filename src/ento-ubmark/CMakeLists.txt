cmake_minimum_required(VERSION 3.16)

include (${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

set(SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/float_sqrt_zero.cc)

add_library(
  ento-ubmark
  INTERFACE
)

if (STM32_BUILD)
  set(MODULES
    bench
    mcu-util
    Eigen
    CMSIS::STM32::${STM_PRODUCT}
  )

  message("Building ubmark")
  add_benchmark(ubmark
                SOURCES ${SOURCE_FILE}
                LIBRARIES ${MODULES})

  add_stm32_flash_and_debug_targets(ubmark)
endif()



