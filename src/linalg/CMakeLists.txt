cmake_minimum_required(VERSION 3.12)

# Module-specific CMakeLists.txt for linalg module
# List source files for this module
set(SRC_FILES
    blas.h
    blas.c
    core.h
    core.c
    # Add more source files as needed
)

# Create a library for this module
add_library(linalg ${SRC_FILES})

# Include directories for this module
# target_include_directories(linalg_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# Link any additional libraries specific to this module
#find_package(SomeDependency REQUIRED)  # Find and specify the required dependency
target_link_libraries(linalg m)

# Optionally, if this module also needs to create an executable, you can do so:
# add_executable(linalg_executable ${SRC_FILES})
# target_link_libraries(linalg_executable linalg_library)

