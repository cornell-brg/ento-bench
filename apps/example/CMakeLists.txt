# Include common utility functions for apps
include(${CMAKE_CURRENT_SOURCE_DIR}/../apps_common.cmake)

set(EXAMPLE_SOURCES
  harness_example1.cc
  harness_example2.cc
)

if (STM32_BUILD)
  message("Configuring Examples for STM32 Semihosting")
  add_arm_semihosting_executable(harness_example1
                                 SOURCES harness_example1.cc
                                 LIBRARIES bench mcu-util assembly)
  add_arm_semihosting_executable(harness_example2
                                 SOURCES harness_example2.cc
                                 LIBRARIES bench mcu-util assembly)
elseif(GEM5_ARMV7E-M_BUILD)
  message("Configuring Examples for gem5 ARM")
  #add_arm_baremetal_gem5_se_executable(harness_example1
  #SOURCES harness_example1.cc
  #LIBRARIES bench assembly startup)
  #add_arm_baremetal_gem5_se_executable(harness_example2
  #SOURCES harness_example2.cc
  #LIBRARIES bench startup)
else()
  message("Configuring Examples for native")
  #add_non_arm_executable(harness_example1
  #                       SOURCES harness_example1.cc
  #                       LIBRARIES bench assembly)
  #add_non_arm_executable(harness_example2
  #                       SOURCES harness_example2.cc
  #                       LIBRARIES bench)
endif()
