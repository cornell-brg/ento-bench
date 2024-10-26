#include (${CMAKE_SOURCE_DIR}/apps/apps_common.cmake)
include (${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

set(SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/benchmark_example.cc)

if (STM32_BUILD)
  set(MODULES
    bench
    mcu-util
    CMSIS::STM32::G474RE
  )

  message("Building benchmark_example")
  add_benchmark(benchmark-example
                SOURCES ${SOURCE_FILE}
                LIBRARIES ${MODULES})

  add_stm32_flash_and_debug_targets(benchmark-example)
endif()
