cmake_minimum_required(VERSION 3.12)

# Module-specific CMakeLists.txt for linalg module
# List source files for this module
set(SRC_FILES
  led_util
  systick_config
    # Add more source files as needed
)

# Create a library for this module
add_library(stm32util_lib ${SRC_FILES})

# Include directories for this module
target_include_directories(stm32util_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# Link any additional libraries specific to this module
#find_package(SomeDependency REQUIRED)  # Find and specify the required dependency
if(IS_STM32G4_BUILD)
message("building stm32-util for stm32g4.")
target_link_libraries(stm32util_lib
  PRIVATE
  HAL::STM32::G4::LL_RCC
  HAL::STM32::G4::LL_GPIO
  CMSIS::STM32::G474RE
  STM32::NoSys
)
endif()

