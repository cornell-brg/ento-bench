cmake_minimum_required(VERSION 3.16)

add_library(bench INTERFACE) # No src files for now

if (GEM5_BUILD)
  set(GEM5_PATH $ENV{GEM5_INSTALL_PATH})
  if (NOT DEFINED GEM5_PATH)
    message(FATAL_ERROR "GEM5 PATH was not found")
  endif()

  message("Including and linking GEM5 m5ops.")
  message("Found env path: ${GEM5_PATH}")

  set(TARGET_ISA arm)
  #target_include_directories(bench
    #INTERFACE
    #${GEM5_PATH}/include
    #)
#target_link_libraries(bench
    #INTERFACE
    #${GEM5_PATH}/util/m5/build/${TARGET_ISA}/out/libm5.a
    #)
endif()

# Include directories for this module
#target_include_directories(util_lib PUBLIC 
#  ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Link any additional libraries specific to this module
#find_package(SomeDependency REQUIRED)  # Find and specify the required dependency
#if(STM32_BUILD)
#  set(SRC_FILES
#    )
#  add_library(util STATIC
#    ${SRC_FILES}
#  )
#  # Create a library for this module
#  target_link_libraries(util
#   PUBLIC
#    stm32util
#  )
#elseif(GEM5_BUILD)
#  set(SRC_FILES
#    syscalls.c
#  )

# add_library(util STATIC
#    ${SRC_FILES}
#  )
#  set_source_files_properties(syscalls.c
#    PROPERTIES COMPILE_FLAGS "-fomit-frame-pointer")
#  # Currently no libs to link for util_lib in gem5
#else() # Native build
#  add_library(util INTERFACE)
#endif()


