include(${CMAKE_CURRENT_SOURCE_DIR}/../apps_common.cmake)

# Build ubench.cc 
set(UBENCH_SOURCES
  ubench.cc
  # armv7em_base.cc
  # armv7em_dsp.cc
  # armv7em_fpu.cc
)

if (STM32_BUILD)
  message("Building microbenchmarks for STM32 Semihosting")
  #add_arm_semihosting_executable(ubench
    #SOURCES ${UBENCH_SOURCES}
                                 #LIBRARIES bench mcu-util assembly)
elseif(GEM5_ARMV7E-M_BUILD)
  message("Building microbenchmarks for gem5 ARM")
  #add_arm_baremetal_gem5_se_executable(ubench
  #SOURCES ${UBENCH_SOURCES}
  #LIBRARIES bench assembly startup sys)
else()
  message("Building microbenchmarks for native")
  #add_non_arm_executable(ubench
  #                       SOURCES ${UBENCH_SOURCES}
  #                       LIBRARIES bench assembly)
endif()

#target_include_directories(ubench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
