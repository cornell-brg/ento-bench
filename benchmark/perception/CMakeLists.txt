# =============================================================================
# Perception Benchmarks
# =============================================================================
include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if (STM32_BUILD)
set(MODULES
  bench
  ento-mcu-semihosted
  Eigen
  ento-pose
  ento_util
  CMSIS::STM32::${STM_PRODUCT}
)
set(BENCH_TARGETS) # List of targets

# =============================================================================
# FAST+BRIEF Benchmarks
# =============================================================================

add_benchmark(bench-fastbrief-tiny
  SOURCES bench_fastbrief_tiny.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-tiny)

add_benchmark(bench-fastbrief-small
  SOURCES bench_fastbrief_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-small)

add_benchmark(bench-fastbrief-medium
  SOURCES bench_fastbrief_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-fastbrief-large
    SOURCES bench_fastbrief_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-fastbrief-large)
endif()

# =============================================================================
# FAST+BRIEF Naneye Benchmarks
# =============================================================================

# Naneye Lights Dataset
add_benchmark(bench-fastbrief-naneye-lights-small
  SOURCES bench_fastbrief_naneye_lights_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-naneye-lights-small)

add_benchmark(bench-fastbrief-naneye-lights-medium
  SOURCES bench_fastbrief_naneye_lights_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-naneye-lights-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-fastbrief-naneye-lights-large
    SOURCES bench_fastbrief_naneye_lights_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-fastbrief-naneye-lights-large)
endif()

# Naneye April Board Dataset
add_benchmark(bench-fastbrief-naneye-april-board-small
  SOURCES bench_fastbrief_naneye_april_board_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-naneye-april-board-small)

add_benchmark(bench-fastbrief-naneye-april-board-medium
  SOURCES bench_fastbrief_naneye_april_board_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fastbrief-naneye-april-board-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-fastbrief-naneye-april-board-large
    SOURCES bench_fastbrief_naneye_april_board_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-fastbrief-naneye-april-board-large)
endif()

# =============================================================================
# FAST Benchmarks
# =============================================================================

add_benchmark(bench-fast-tiny
  SOURCES bench_fast_tiny.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fast-tiny)

add_benchmark(bench-fast-small
  SOURCES bench_fast_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fast-small)

add_benchmark(bench-fast-medium
  SOURCES bench_fast_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-fast-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-fast-large
    SOURCES bench_fast_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-fast-large)
endif()

# =============================================================================
# BRIEF Benchmarks
# =============================================================================

add_benchmark(bench-brief-tiny
  SOURCES bench_brief_tiny.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-brief-tiny)

add_benchmark(bench-brief-small
  SOURCES bench_brief_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-brief-small)

add_benchmark(bench-brief-medium
  SOURCES bench_brief_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-brief-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-brief-large
    SOURCES bench_brief_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-brief-large)
endif()

# =============================================================================
# ORB Benchmarks
# =============================================================================

add_benchmark(bench-orb-tiny
  SOURCES bench_orb_tiny.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-tiny)

add_benchmark(bench-orb-small
  SOURCES bench_orb_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-small)

add_benchmark(bench-orb-medium
  SOURCES bench_orb_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-orb-large
    SOURCES bench_orb_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-orb-large)
endif()

# =============================================================================
# ORB Naneye Benchmarks
# =============================================================================

# Naneye Lights Dataset
add_benchmark(bench-orb-naneye-lights-small
  SOURCES bench_orb_naneye_lights_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-naneye-lights-small)

add_benchmark(bench-orb-naneye-lights-medium
  SOURCES bench_orb_naneye_lights_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-naneye-lights-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-orb-naneye-lights-large
    SOURCES bench_orb_naneye_lights_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-orb-naneye-lights-large)
endif()

# Naneye April Board Dataset
add_benchmark(bench-orb-naneye-april-board-small
  SOURCES bench_orb_naneye_april_board_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-naneye-april-board-small)

add_benchmark(bench-orb-naneye-april-board-medium
  SOURCES bench_orb_naneye_april_board_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-orb-naneye-april-board-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-orb-naneye-april-board-large
    SOURCES bench_orb_naneye_april_board_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-orb-naneye-april-board-large)
endif()

# =============================================================================
# Lucas Kanade Optical Flow Benchmarks
# =============================================================================

add_benchmark(bench-lkof-small
  SOURCES bench_lkof_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-lkof-small)

add_benchmark(bench-lkof-medium
  SOURCES bench_lkof_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-lkof-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-lkof-large
    SOURCES bench_lkof_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-lkof-large)
endif()

# =============================================================================
# Image Interpolation Optical Flow Benchmarks
# =============================================================================

add_benchmark(bench-image-interp-of-small
  SOURCES bench_image_interp_of_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-image-interp-of-small)

add_benchmark(bench-image-interp-of-medium
  SOURCES bench_image_interp_of_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-image-interp-of-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-image-interp-of-large
    SOURCES bench_image_interp_of_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-image-interp-of-large)
endif()

# =============================================================================
# Block-Based Optical Flow Benchmarks
# =============================================================================

add_benchmark(bench-block-of-small
  SOURCES bench_block_of_small.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-block-of-small)

add_benchmark(bench-block-of-medium
  SOURCES bench_block_of_medium.cc
  LIBRARIES ${MODULES}
)
list(APPEND BENCH_TARGETS bench-block-of-medium)

# Large benchmarks - exclude from STM32G4 due to RAM/Flash limitations
if(NOT STM_FAMILY STREQUAL "G4")
  add_benchmark(bench-block-of-large
    SOURCES bench_block_of_large.cc
    LIBRARIES ${MODULES}
  )
  list(APPEND BENCH_TARGETS bench-block-of-large)
endif()

# =============================================================================
# Configure Benchmark Group with New Idioms
# =============================================================================

# Config file approach with smart fallbacks
set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/benchmark/configs/perception_benchmarks.json")
if(EXISTS ${CONFIG_FILE})
  add_configured_benchmark_group_with_target_configs("perception"
    CONFIG_FILE ${CONFIG_FILE}
    TARGETS ${BENCH_TARGETS}
  )
  add_configured_benchmark_group_from_file("perception"
    CONFIG_FILE ${CONFIG_FILE}
    TARGETS ${BENCH_TARGETS}
    # Fallback configuration
    REPS 30
    INNER_REPS 2
    VERBOSITY 1
    ENABLE_CACHES
    ENABLE_VECTORIZATION
  )
else()
  # Direct configuration fallback
  add_preconfigured_benchmark_group("perception"
    TARGETS ${BENCH_TARGETS}
    REPS 30
    INNER_REPS 2
    VERBOSITY 1
    ENABLE_CACHES
    ENABLE_VECTORIZATION
  )
endif()

# =============================================================================
# Add STM32 targets
# =============================================================================
message(STATUS "Perception bench targets: ${BENCH_TARGETS}")
add_stm32_targets("${BENCH_TARGETS}")

endif()

