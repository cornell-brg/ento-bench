if (STM32_BUILD)
# Add the source files
add_executable(fopen-example fopen_example.cc)


target_compile_options(fopen-example
  PRIVATE
  -mcpu=cortex-m4
  -mthumb
  -g
  -O0
)

target_link_options(fopen-example
  PRIVATE
  "--specs=rdimon.specs"
  "-lc"
  "-lrdimon"
)

target_link_libraries(
  fopen-example
  PRIVATE
  HAL::STM32::G4::LL_RCC
  HAL::STM32::G4::LL_GPIO
  CMSIS::STM32::G474RE
  mcu-util
)

# Add compile options
endif()
