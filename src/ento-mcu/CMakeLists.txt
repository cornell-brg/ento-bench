cmake_minimum_required(VERSION 3.16)

if(STM32_BUILD)
  # List source files for this module
  set(SRC_FILES
    led_util.c
    systick_config.c
    flash_util.cc
    clk_util.c
    pwr_util.c
    cache_util.cc
    timing.cc
    gpio_util.cc
  )

  message(VERBOSE "MCU targets: CMSIS=${CMSIS_TYPE_TARGET}, HAL RCC/GPIO/PWR configured")
  message(VERBOSE "ento-mcu source files: ${SRC_FILES}")
  add_library(ento-mcu STATIC
    ${SRC_FILES})
  add_library(ento-mcu-semihosted STATIC
    ${SRC_FILES})

  target_link_libraries(ento-mcu
    PUBLIC
    m
    ${HAL_BUS_TARGET}
    ${HAL_RCC_TARGET}
    ${HAL_PWR_TARGET}
    ${HAL_UTILS_TARGET}
    ${HAL_GPIO_TARGET}
    ${CMSIS_TYPE_TARGET}
    STM32::NoSys
    STM32::Nano
  )

  target_link_libraries(ento-mcu-semihosted
    PUBLIC
    m
    ${HAL_BUS_TARGET}
    ${HAL_RCC_TARGET}
    ${HAL_PWR_TARGET}
    ${HAL_UTILS_TARGET}
    ${HAL_GPIO_TARGET}
    ${CMSIS_TYPE_TARGET}
    STM32::Nano::FloatPrint
    STM32::Nano::FloatScan
  )


# =============================================

  include (CTest)
  set(TEST_DIR "test")

  set(TEST_FILES
      test_clock_cfg.cc
      test_cache_cfg.cc
  )

  set(TEST_LIBRARIES_test_clock_cfg
    ento-mcu-semihosted
    ento_util
    CMSIS::STM32::${STM_PRODUCT}
 )

  set(TEST_LIBRARIES_test_cache_cfg
    ento-mcu-semihosted
    ento_util
    CMSIS::STM32::${STM_PRODUCT}
  )


  foreach(TEST_FILE ${TEST_FILES})
    get_filename_component(TEST_BIN ${TEST_FILE} NAME_WE)
    set(TEST_SRC "test/${TEST_FILE}")

    string(CONCAT LIBRARY_VAR_NAME "TEST_LIBRARIES_" ${TEST_BIN})
    set(TEST_LIBRARIES ${${LIBRARY_VAR_NAME}})

    add_arm_semihosting_executable(
      ${TEST_BIN}
      SOURCES ${TEST_SRC}
      LIBRARIES ${TEST_LIBRARIES}
    )
    add_stm32_target(${TEST_BIN})
  endforeach()


elseif(GEM5_ARMV7E-M_BUILD) # Do nothing, we don't need util currently
else() # Do nothing, we don't need util currently
endif()
