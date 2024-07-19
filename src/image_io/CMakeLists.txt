# Module-specific CMakeLists.txt for image_io module

# List source files for this module
set(SRC_FILES
    #MonoImage8.h
    #MonoImage8.c
    #MonoImage16.h
    #MonoImage16.c
    #imageIO.h
    #imageIO.c
    NaneyeImage.h
    NaneyeImage.c
    # Add more source files as needed
)

# Create a library for this module
add_library(image_io_library ${SRC_FILES})

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
target_include_directories(image_io_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${PNG_INCLUDE_DIR})

#target_link_libraries(image_io_library PUBLIC PNG::PNG)
#endif()

#else()
#  message(FATAL_ERROR "libpng not found. Please install libpng")
# endif()

# Optionally, if this module also needs to create an executable, you can do so:
# add_executable(image_io_executable ${SRC_FILES})
# target_link_libraries(image_io_executable image_io_library)

