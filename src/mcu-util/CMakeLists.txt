cmake_minimum_required(VERSION 3.16)

if(STM32_BUILD)
  # List source files for this module
  set(SRC_FILES
    led_util.c
    systick_config.c
    flash_util.c
    clk_util.c
    pwr_util.c
  )

  message("HAL RCC Target: ${HAL_RCC_TARGET}")
  message("HAL GPIO Target ${HAL_GPIO_TARGET}")
  message("CMSIS Target: ${CMSIS_TARGET}")
  message("CMSIS Family Target in MCU Util: ${CMSIS_FAMILY_TARGET}")
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
    STM32::NoSys
    STM32::Nano
    STM32::Nano::FloatPrint
    STM32::Nano::FloatScan
  )
elseif(GEM5_ARMV7E-M_BUILD) # Do nothing, we don't need util currently
else() # Do nothing, we don't need util currently
endif()
