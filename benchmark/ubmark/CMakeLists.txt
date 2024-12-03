cmake_minimum_required(VERSION 3.16)
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

set(UBMARKS
  # add files below

)

# @TODO: ento-mcu "linked" always. when using harness with gem5
#   or even native performance testing should not include ento-mcu

# Derin: I don't think any other libs should be need to be linked...
set(LIBS
  ento-ubmark
  ento-bench
  ento-util
  ento-mcu
  Eigen
)

foreach(UBMARK ${UBMARKS})
  get_filename_component(UBMARK_BIN ${UBMARK} NAME_WE)
  add_benchmark(
    UBMARK_BIN
    EXCLUDE
    SOURCES ${UBMARK}
    LIBRARIES ${MODULES}
  )
  add_stm32_flash_and_debug_targets(${UBMARK_BIN})
  add_stm32_no_semihosting_flash_targets(${UBMARK_BIN})
endforeach()
