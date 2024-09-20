cmake_minimum_required(VERSION 3.16)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../apps/apps_common.cmake)

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
#add_library(image_io ${SRC_FILES})
add_library(image_io INTERFACE) # No src files for now

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
set(TEST_DIR "test")


if (GEM5_ARMV7E-M_BUILD)
  add_arm_baremetal_gem5_se_executable(test_image
    SOURCES ${TEST_DIR}/test_image.cc
    LIBRARIES image_io m startup sys)
else()
  #add_non_arm_executable(test_svd
  #                       SOURCES ${TEST_DIR}/test_svd.cc
  #                       LIBRARIES ento_math m)
  add_executable(test_image ${TEST_DIR}/test_image.cc)
  target_link_libraries(test_image PUBLIC m image_io)
endif()
target_include_directories(test_image
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

add_test(
  NAME TestImage
  COMMAND test_image
)

