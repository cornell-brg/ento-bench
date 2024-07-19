cmake_minimum_required(VERSION 3.16)

# Module-specific CMakeLists.txt for linalg module
# List source files for this module
set(SRC_FILES
  led_util.c
  systick_config.c
  timing.c
    # Add more source files as needed
)

# Create a library for this module
add_library(stm32util STATIC
  ${SRC_FILES})

# Include directories for this module
#target_include_directories(stm32util_lib PUBLIC 
#  ${CMAKE_CURRENT_SOURCE_DIR})

# Link any additional libraries specific to this module
#find_package(SomeDependency REQUIRED)  # Find and specify the required dependency
if(STM32_BUILD)

  set(HAL_RCC_TARGET "HAL::STM32::${STM_FAMILY}::LL_RCC")
  set(HAL_GPIO_TARGET "HAL::STM32::${STM_FAMILY}::LL_GPIO")
  set(CMSIS_TARGET "CMSIS::STM32::${STM_PRODUCT}")
  message("HAL_RCC_TARGET: ${HAL_RCC_TARGET}")
  message("HAL_GPIO_TARGET: ${HAL_GPIO_TARGET}")
  message("CMSIS_TARGET: ${CMSIS_TARGET}")
  target_link_libraries(stm32util
    PUBLIC
    ${HAL_RCC_TARGET}
    ${HAL_GPIO_TARGET}
    ${CMSIS_TARGET}
  )

endif()
