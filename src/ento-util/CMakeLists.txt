cmake_minimum_required(VERSION 3.16)

include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

# ======================================
set(SRC_FILES
  unittest.cc
)

add_library(ento_util
  ${SRC_FILES}
)

target_link_libraries(
  ento_util
  PUBLIC
)

if (STM32_BUILD)
  target_link_libraries(
    ento_util
    PUBLIC
    ${STM32_TARGET}
    )
else()

endif()


# ======================================

include (CTest)
set(TEST_DIR "test")

set(TEST_FILES
    test_matrix_reader.cc
    test_pose_est_reader.cc
)

# Define library lists based on source names
if (GEM5_ARMV7E-M_BUILD)
  set(TEST_LIBRARIES_test_matrix_reader ento_math m startup ento_util sys Eigen)
  set(TEST_LIBRARIES_test_pose_est_reader ento_math m startup ento_util sys Eigen)
elseif (STM32_BUILD)
  set(TEST_LIBRARIES_test_matrix_reader ento_math m mcu-util ento_util Eigen)
  set(TEST_LIBRARIES_test_pose_est_reader ento_math m mcu-util ento_util Eigen)
else ()
  set(TEST_LIBRARIES_test_matrix_reader ento_math m ento_util Eigen)
  set(TEST_LIBRARIES_test_pose_est_reader ento_math m ento_util Eigen)
endif ()

foreach(TEST_FILE ${TEST_FILES})
  get_filename_component(TEST_BIN ${TEST_FILE} NAME_WE)
  set(TEST_SRC "test/${TEST_FILE}")

  # Construct the library variable name and retrieve the list
  string(CONCAT LIBRARY_VAR_NAME "TEST_LIBRARIES_" ${TEST_BIN})
  set(TEST_LIBRARIES ${${LIBRARY_VAR_NAME}})

  if (GEM5_ARMV7E-M_BUILD)
    add_arm_baremetal_gem5_se_executable(
      ${TEST_BIN}
      SOURCES ${TEST_SRC}
      LIBRARIES ${TEST_LIBRARIES})
  elseif(STM32_BUILD)
    add_arm_semihosting_executable(
      ${TEST_BIN}
      SOURCES ${TEST_SRC}
      LIBRARIES ${TEST_LIBRARIES}
    )
  else()
    message(STATUS "Building ${TEST_BIN} for native")
    add_non_arm_executable(${TEST_BIN}
      EXCLUDE TRUE
      SOURCES ${TEST_SRC}
      LIBRARIES ${TEST_LIBRARIES})
    add_dependencies(check ${TEST_BIN})
    add_test(NAME ${TEST_BIN} COMMAND ${TEST_BIN})
  endif()
endforeach()

