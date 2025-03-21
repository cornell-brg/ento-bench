#include (${CMAKE_SOURCE_DIR}/apps/apps_common.cmake)
include (${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

set(SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/blas-comparison.cc)

if (STM32_BUILD)
  set(MODULES
    bench
    mcu-util
    ento_math
    Eigen
    CMSIS::STM32::${STM_PRODUCT}
  )

  message("Building blas comparison")
  add_benchmark(blas-comparison
                SOURCES ${SOURCE_FILE}
                LIBRARIES ${MODULES})

  add_stm32_target(blas-comparison)
  add_stm32_no_semihosting_target(blas-comparison-no-semihosting)
endif()
