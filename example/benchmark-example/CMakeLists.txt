#include (${CMAKE_SOURCE_DIR}/apps/apps_common.cmake)
include (${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

set(SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/benchmark_example.cc)

if (STM32_BUILD)
  set(MODULES
    bench
    ento-mcu-semihosted
    Eigen
    CMSIS::STM32::${STM_PRODUCT}
  )

  message("Building benchmark_example")
  add_benchmark(benchmark-example
                SOURCES ${SOURCE_FILE}
                LIBRARIES ${MODULES})

  add_stm32_target(benchmark-example)
  add_stm32_no_semihosting_target(benchmark-example-no-semihosting)
endif()
