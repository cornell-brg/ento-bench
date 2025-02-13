cmake_minimum_required(VERSION 3.16)

set(SRC_FILES
  roi.cc
)

add_library(bench 
  ${SRC_FILES}
) 

set(PROBLEMS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/problems)

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
elseif (STM32_BUILD)
  target_link_libraries(bench
    PUBLIC
    mcu-util
  )
else()
  target_link_libraries(bench
    PUBLIC
  )
endif()

#=====================================================
# Testing
#=====================================================
include (CTest)
set (TEST_FILES
  test_harness.cc
)

set(LIBRARIES_test_harness
  ento_util
)

foreach(TEST_FILE ${TEST_FILES})
  message(STATUS "Test file: ${TEST_FILE}")
  set(TEST_SRC "test/${TEST_FILE}")
  get_filename_component(TEST_BIN ${TEST_FILE} NAME_WE)
  message(STATUS "Test bin: ${TEST_BIN}")
  string(CONCAT LIBRARY_VAR_NAME "LIBRARIES_" ${TEST_BIN})
  message(STATUS ${${LIBRARY_VAR_NAME}})
  set(TEST_LIBRARIES ${${LIBRARY_VAR_NAME}})
  add_ento_test(
    ${TEST_BIN}
    SOURCES   ${TEST_SRC}
    LIBRARIES ${TEST_LIBRARIES}
  )
endforeach()


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


