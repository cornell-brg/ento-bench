#===============================
#======Microbenchmarking========
if (IS_STM32G4_BUILD)
add_executable(ub_vvadd vv_add.c)
target_link_libraries(
  ub_vvadd
  PRIVATE
  linalg_lib
  HAL::STM32::G4::LL_RCC
  HAL::STM32::G4::LL_GPIO
  CMSIS::STM32::G474RE
  STM32::NoSys
)

add_executable(ub_sumreduction sum_reduction.c)
target_link_libraries(
  ub_sumreduction
  PRIVATE
  linalg_lib
  HAL::STM32::G4::LL_RCC
  HAL::STM32::G4::LL_GPIO
  CMSIS::STM32::G474RE
  STM32::NoSys
)

add_executable(ub_dotprod dot_prod.c)
target_link_libraries(
  ub_dotprod
  PRIVATE
  linalg_lib
  stm32util_lib
  HAL::STM32::G4::LL_RCC
  HAL::STM32::G4::LL_GPIO
  CMSIS::STM32::G474RE
  STM32::NoSys
)
elseif(IS_GEM5_BUILD)

else()
add_executable(ub_vvadd vv_add.c)
target_link_libraries(
  ub_vvadd
  PRIVATE
  linalg_lib
)

add_executable(ub_sumreduction sum_reduction.c)
target_link_libraries(
  ub_sumreduction
  PRIVATE
  linalg_lib
)

add_executable(ub_dotprod dot_prod.c)
target_link_libraries(
  ub_dotprod
  PRIVATE
  linalg_lib
)
endif()
