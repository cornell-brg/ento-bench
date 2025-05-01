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

  message(STATUS "HAL RCC Target: ${HAL_RCC_TARGET}")
  message(STATUS "HAL GPIO Target ${HAL_GPIO_TARGET}")
  message(STATUS "CMSIS Target: ${CMSIS_TYPE_TARGET}")
  message(STATUS "CMSIS Family Target in MCU Util: ${CMSIS_FAMILY_TARGET}")
  message(STATUS "HAL Bus Target: ${HAL_BUS_TARGET}")
  message(STATUS "HAL Pwr Target: ${HAL_PWR_TARGET}")
  message(STATUS "HAL Utils Target: ${HAL_UTILS_TARGET}")
  message(STATUS "Src files for mcu-util: ${SRC_FILES}")
  add_library(mcu-util STATIC
    ${SRC_FILES})

  target_link_libraries(mcu-util
    PUBLIC
    m
    ${HAL_BUS_TARGET}
    ${HAL_RCC_TARGET}
    ${HAL_PWR_TARGET}
    ${HAL_UTILS_TARGET}
    ${HAL_GPIO_TARGET}
    ${CMSIS_TYPE_TARGET}
    #STM32::Nano::FloatPrint
    #STM32::Nano::FloatScan
    #STM32::NoSys
    #STM32::Nano
  )

# =============================================

  include (CTest)
  set(TEST_DIR "test")

  set(TEST_FILES
      test_clock_cfg.cc
      test_cache_cfg.cc
  )

  set(TEST_LIBRARIES_test_clock_cfg
    mcu-util
    ento_util
    CMSIS::STM32::${STM_PRODUCT}
 )

  set(TEST_LIBRARIES_test_cache_cfg
    mcu-util
    ento_util
    CMSIS::STM32::${STM_PRODUCT}
  )


  foreach(TEST_FILE ${TEST_FILES})
    message(STATUS "Building ${TEST_FILE} in mcu-util/tests...")
    get_filename_component(TEST_BIN ${TEST_FILE} NAME_WE)
    set(TEST_SRC "test/${TEST_FILE}")

    # Construct the library variable name and retrieve the list
    string(CONCAT LIBRARY_VAR_NAME "TEST_LIBRARIES_" ${TEST_BIN})
    set(TEST_LIBRARIES ${${LIBRARY_VAR_NAME}})

    add_arm_semihosting_executable(
      ${TEST_BIN}
      SOURCES ${TEST_SRC}
      LIBRARIES ${TEST_LIBRARIES}
    )
    message(STATUS "Building flash and debug target for: ${TEST_BIN}")
    add_stm32_target(${TEST_BIN})
  endforeach()


elseif(GEM5_ARMV7E-M_BUILD) # Do nothing, we don't need util currently
else() # Do nothing, we don't need util currently
endif()
