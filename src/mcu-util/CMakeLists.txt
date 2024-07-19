cmake_minimum_required(VERSION 3.16)

if(STM32_BUILD)
  # List source files for this module
  set(SRC_FILES
    led_util.c
    systick_config.c
  )
  set(HAL_RCC_TARGET "HAL::STM32::${STM_FAMILY}::LL_RCC")
  set(HAL_GPIO_TARGET "HAL::STM32::${STM_FAMILY}::LL_GPIO")
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
    ${CMSIS_TARGET}
  )
elseif(GEM5_ARMV7E-M_BUILD) # Do nothing, we don't need util currently
else() # Do nothing, we don't need util currently
endif()
