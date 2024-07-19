cmake_minimum_required(VERSION 3.16)

set(SRC_FILES
  boot.s
)

if(GEM5_BUILD)
  set_source_files_properties(boot.s PROPERTIES COMPILE_FLAGS "-mfloat-abi=${FLOAT_ABI} -mfpu=${FPU} -mcpu=${CORE}" OBJECT_OUTPUTS "${CMAKE_CURRENT_BINARY_DIR}/boot.o")
  set(BOOT_O ${CMAKE_CURRENT_BINARY_DIR}/boot.o)
  add_custom_command(
    OUTPUT ${BOOT_O}
    COMMAND arm-none-eabi-cpp -mfloat-abi=${FLOAT_ABI} -mfpu=${FPU} -mcpu=${CORE} ${CMAKE_CURRENT_SOURCE_DIR}/boot.s -o boot.i
    COMMAND arm-none-eabi-as -EL boot.i -o ${BOOT_O}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/boot.s
    COMMENT "Preprocessing and assembling boot.s"
  )
  add_custom_target(boot_s_target DEPENDS ${BOOT_O})

  add_library(startup
    STATIC
    ${BOOT_O}
  )
  add_dependencies(startup boot_s_target)
  
  set_target_properties(startup PROPERTIES LINKER_LANGUAGE C)
  
endif()
