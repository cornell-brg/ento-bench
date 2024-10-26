cmake_minimum_required(VERSION 3.16)

include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

# List source files for this module
set(SRC_FILES
    #MonoImage8.h
    #MonoImage8.c
    #MonoImage16.h
    #MonoImage16.c
    #imageIO.h
    #imageIO.c
    #NaneyeImage.h
    #NaneyeImage.c
    # Add more source files as needed
)

# Create a library for this module
add_library(image_io INTERFACE) # No src files for now

# Link any libraries to this module
target_link_libraries(image_io
  INTERFACE
  ento_util
)

# Link any libraries specific to this module
#include (FindPkgConfig)
#if (NOT PKG_CONFIG_FOUND)
#  message(FATAL_ERROR "pkg-config not found")
#endif()

#pkg_check_modules(LIBPNG libpng16 REQUIRED)
#find_package(PNG REQUIRED)

#if (NOT LIBPNG_FOUND)
#  message(FATAL_ERROR "You do not seem to have libpng16 development libraries installed.")
#else ()
#target_include_directories(image_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${PNG_INCLUDE_DIR})

#target_link_libraries(image_io_library PUBLIC PNG::PNG)
#endif()

#else()
#  message(FATAL_ERROR "libpng not found. Please install libpng")
# endif()

# Optionally, if this module also needs to create an executable, you can do so:
# add_executable(image_io_executable ${SRC_FILES})
# target_link_libraries(image_io_executable image_io_library)

#=====================================================
include (CTest)

set(TEST_FILES
  test_image.cc
)

set(LIBRARIES_test_image
  m image_io ento_util
)

foreach(TEST_FILE ${TEST_FILES})
  message(STATUS "Test file: ${TEST_FILE}")
  set(TEST_SRC "test/${TEST_FILE}")
  get_filename_component(TEST_BIN ${TEST_FILE} NAME_WE)
  string(CONCAT LIBRARY_VAR_NAME "LIBRARIES_" ${TEST_BIN})
  message(STATUS ${${LIBRARY_VAR_NAME}})
  set(TEST_LIBRARIES ${${LIBRARY_VAR_NAME}})
  add_ento_test(
    ${TEST_BIN}
    SOURCES   ${TEST_SRC}
    LIBRARIES ${TEST_LIBRARIES}
  )
endforeach()
