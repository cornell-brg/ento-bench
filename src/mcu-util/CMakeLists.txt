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
  set(HAL_RCC_TARGET "HAL::STM32::${STM_FAMILY}::LL_RCC")
  set(HAL_GPIO_TARGET "HAL::STM32::${STM_FAMILY}::LL_GPIO")
  set(HAL_FLASH_TARGET "HAL::STM32::${STM_FAMILY}::LL_FLASH")
  set(HAL_UTILS_TARGET "HAL::STM32::${STM_FAMILY}::LL_UTILS")
  set(HAL_PWR_TARGET "HAL::STM32::${STM_FAMILY}::LL_PWR")
  set(CMSIS_TARGET "CMSIS::STM32::${STM_PRODUCT}")

  message("HAL RCC Target: ${HAL_RCC_TARGET}")
  message("HAL GPIO Target ${HAL_GPIO_TARGET}")
  message("CMSIS Target: ${CMSIS_TARGET}")
  add_library(mcu-util STATIC
    ${SRC_FILES})

  target_link_libraries(mcu-util
    PUBLIC
    ${HAL_RCC_TARGET}
    ${HAL_GPIO_TARGET}
    ${HAL_UTILS_TARGET}
    ${HAL_PWR_TARGET}
    ${CMSIS_TARGET}
  )
elseif(GEM5_ARMV7E-M_BUILD) # Do nothing, we don't need util currently
else() # Do nothing, we don't need util currently
endif()
