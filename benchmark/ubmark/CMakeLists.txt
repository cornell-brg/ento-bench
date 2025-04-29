cmake_minimum_required(VERSION 3.16)
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)
if (STM32_BUILD)
set(MODULES
  bench
  mcu-util
  ento_util
  CMSIS::STM32::${STM_PRODUCT}
)
set(UBMARKS) # List of targets

# =============================================================================
# Floating Point ADD Random
# =============================================================================

add_benchmark(ubmark-fp-add-rand
  SOURCES ubmark_fp_add_rand.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-fp-add-rand)

# =============================================================================
# Floating Point ADD Zero
# =============================================================================

add_benchmark(ubmark-fp-add-zero
  SOURCES ubmark_fp_add_zero.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-fp-add-zero)

# =============================================================================
# Floating Point DIV Random
# =============================================================================

add_benchmark(ubmark-fp-div-rand
  SOURCES ubmark_fp_div_rand.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-fp-div-rand)

# =============================================================================
# Floating Point DIV Zero
# =============================================================================

add_benchmark(ubmark-fp-div-zero
  SOURCES ubmark_fp_div_zero.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-fp-div-zero)

# =============================================================================
# Floating Point MUL Random
# =============================================================================

add_benchmark(ubmark-fp-mul-rand
  SOURCES ubmark_fp_mul_rand.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-fp-mul-rand)

# =============================================================================
# Floating Point MUL Zero
# =============================================================================

add_benchmark(ubmark-fp-mul-zero
  SOURCES ubmark_fp_mul_zero.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-fp-mul-zero)

# =============================================================================
# Floating Point SQRT Random
# =============================================================================

add_benchmark(ubmark-fp-sqrt-rand
  SOURCES ubmark_fp_sqrt_rand.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-fp-sqrt-rand)

# =============================================================================
# Floating Point SQRT Zero
# =============================================================================

add_benchmark(ubmark-fp-sqrt-zero
  SOURCES ubmark_fp_sqrt_zero.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-fp-sqrt-zero)

# =============================================================================
# Integer ADD Random
# =============================================================================

add_benchmark(ubmark-int-add-rand
  SOURCES ubmark_int_add_rand.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-int-add-rand)

# =============================================================================
# Integer ADD Zero
# =============================================================================

add_benchmark(ubmark-int-add-zero
  SOURCES ubmark_int_add_zero.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-int-add-zero)

# =============================================================================
# Integer DIV Random
# =============================================================================

add_benchmark(ubmark-int-div-rand
  SOURCES ubmark_int_div_rand.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-int-div-rand)

# =============================================================================
# Integer DIV Zero
# =============================================================================

add_benchmark(ubmark-int-div-zero
  SOURCES ubmark_int_div_zero.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-int-div-zero)

# =============================================================================
# Integer MUL Random
# =============================================================================

add_benchmark(ubmark-int-mul-rand
  SOURCES ubmark_int_mul_rand.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-int-mul-rand)

# =============================================================================
# Integer MUL Zero
# =============================================================================

add_benchmark(ubmark-int-mul-zero
  SOURCES ubmark_int_mul_zero.cc
  LIBRARIES ${MODULES}
)
list(APPEND UBMARKS ubmark-int-mul-zero)

# =============================================================================
# =============================================================================
# Add stm32 targets
# =============================================================================
message(STATUS "Bench targets: ${UBMARKS}")
add_stm32_targets("${UBMARKS}")

endif()






# set(UBMARKS
#   # add files below

# )

# # @TODO: ento-mcu "linked" always. when using harness with gem5
# #   or even native performance testing should not include ento-mcu

# # Derin: I don't think any other libs should be need to be linked...
# set(LIBS
#   ento-ubmark
#   ento-bench
#   ento-util
#   ento-mcu
#   Eigen
# )

# foreach(UBMARK ${UBMARKS})
#   get_filename_component(UBMARK_BIN ${UBMARK} NAME_WE)
#   add_benchmark(
#     UBMARK_BIN
#     EXCLUDE
#     SOURCES ${UBMARK}
#     LIBRARIES ${MODULES}
#   )
#   add_stm32_flash_and_debug_targets(${UBMARK_BIN})
#   add_stm32_no_semihosting_flash_targets(${UBMARK_BIN})
# endforeach()
