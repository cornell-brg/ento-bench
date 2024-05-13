if (IS_STM32G4_BUILD)
# Add the source files
add_executable(fopen_example fopen_example.c)


target_compile_options(fopen_example
  PRIVATE
  -mcpu=cortex-m4
  -mthumb
  -g
  -O0
)

target_link_options(fopen_example
  PRIVATE
  "--specs=rdimon.specs"
  "-lc"
  "-lrdimon"
)

target_link_libraries(
  fopen_example
  PRIVATE
  HAL::STM32::G4::LL_RCC
  HAL::STM32::G4::LL_GPIO
  CMSIS::STM32::G474RE
  stm32util_lib
)

# Add compile options
endif()
