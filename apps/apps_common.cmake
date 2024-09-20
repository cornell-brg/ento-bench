# This file defines CMake helper functions

include(CMakeParseArguments)

function(add_arm_semihosting_executable TARGET_NAME)
  # Extract source files and libs from arguments
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES" ${ARGN})
  
  add_executable(${TARGET_NAME} ${ARG_SOURCES})

  target_link_options(${TARGET_NAME}
    PRIVATE
    "--specs=rdimon.specs"
    "-lc"
    "-lrdimon"
  )

  message("[ARM semihosting build] Libs to link for ${TARGET_NAME}: ${ARG_LIBRARIES}")
  target_link_libraries(${TARGET_NAME}
    PUBLIC
    ${ARG_LIBRARIES}
  )
endfunction()


function(add_arm_baremetal_gem5_se_executable TARGET_NAME)
  # Extract source files and libs from arguments
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES" ${ARGN})
  
  add_executable(${TARGET_NAME} ${ARG_SOURCES})

  target_link_options(${TARGET_NAME}
    PRIVATE
    -L$ENV{ARM_NONE_EABI_LIB}/thumb/v7e-m+fp/hard #Link hard float lib
    -lc_nano
    -u _printf_float
    -nostartfiles
    --data-sections
    -Xlinker -T${STARTUP_DIR}/boot.ld
    -static # gem5 SE mode needs a static binary
  )

  message("[ARM gem5-SE build] Libs for ${TARGET_NAME}: ${ARG_LIBRARIES}")
  target_link_libraries(${TARGET_NAME}
    PUBLIC
    ${ARG_LIBRARIES}
  )

  target_link_directories(${TARGET_NAME}
    PRIVATE
    ${CMAKE_BINARY_DIR}/src/startup/CMakeFiles/startup_lib.dir
  )

endfunction()

function(add_non_arm_executable TARGET_NAME)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES" ${ARGN})

  add_executable(${TARGET_NAME} ${ARG_SOURCES})
  message("Native exe librs: ${ARG_LIBRARIES}")

  #target_link_options(${TARGET_NAME}
  #  PRIVATE
  #  "${CMAKE_C_FLAGS}"
  #)

  target_link_libraries(${TARGET_NAME}
    PRIVATE
    ${ARG_LIBRARIES}
  )
endfunction()

