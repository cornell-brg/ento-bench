# ARM Baremetal Example for gem5 SE Mode
# This is a simple test example that demonstrates basic gem5 baremetal functionality

# Only build this example when GEM5_BUILD is enabled
if(GEM5_BUILD)
  # Define the target name
  set(TARGET_NAME "gem5-baremetal-example")
  
  # Source files for the example (main.c includes syscalls.c)
  set(C_SOURCES
    main.c
  )
  
  # Handle boot.s with custom preprocessing like the startup library does
  set(BOOT_O ${CMAKE_CURRENT_BINARY_DIR}/boot.o)
  add_custom_command(
    OUTPUT ${BOOT_O}
    COMMAND arm-none-eabi-cpp -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ${CMAKE_CURRENT_SOURCE_DIR}/boot.s -o boot.i
    COMMAND arm-none-eabi-as -EL boot.i -o ${BOOT_O}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/boot.s
    COMMENT "Preprocessing and assembling boot.s"
  )
  add_custom_target(gem5_boot_s_target DEPENDS ${BOOT_O})
  
  # Create the baremetal executable
  add_executable(${TARGET_NAME} ${C_SOURCES} ${BOOT_O})
  add_dependencies(${TARGET_NAME} gem5_boot_s_target)
  
  # Set gem5 baremetal-specific link options
  target_link_options(${TARGET_NAME} PRIVATE
    -static
    -nostartfiles
    -Wl,-T${CMAKE_CURRENT_SOURCE_DIR}/boot.ld
    -Wl,--gc-sections
    -Wl,--print-memory-usage
  )
  
  # Add custom defines based on build configuration
  target_compile_definitions(${TARGET_NAME} PRIVATE
    GEM5_BAREMETAL_EXAMPLE=1
  )
  
  # Fix ASM compiler flags to avoid --defsym issues
  set_property(TARGET ${TARGET_NAME} PROPERTY ASM_COMPILE_FLAGS "")
  
  # Create a simple test target to run this in gem5 (if gem5 is available)
  # This would need to be extended with actual gem5 execution
  add_custom_target(run-${TARGET_NAME}
    COMMAND echo "To run: gem5 [config] ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}"
    DEPENDS ${TARGET_NAME}
    COMMENT "Instructions for running ${TARGET_NAME} in gem5"
  )
  
  message(STATUS "Added gem5 baremetal example: ${TARGET_NAME}")
  
else()
  message(STATUS "Skipping gem5 baremetal example (GEM5_BUILD not enabled)")
endif()