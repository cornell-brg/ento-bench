# add_library(pidlib STATIC
#     pid.cc
# )

include(CTest)
set(TEST_DIR "test")

if (GEM5_ARMV7E-M_BUILD)
  add_arm_baremetal_gem5_se_executable(test_pid
    SOURCES ${TEST_DIR}/test_pid.cc
    LIBRARIES startup sys)
elseif(STM32_BUILD)
  add_arm_semihosting_executable(
    test_pid
    SOURCES ${TEST_DIR}/test_pid.cc
    LIBRARIES mcu-util
  )
else()
  add_non_arm_executable(test_pid
                         SOURCES ${TEST_DIR}/test_pid.cc
                         LIBRARIES )
endif()
target_include_directories(test_pid
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_DIR}
)

add_dependencies(check test_pid)
add_test(
  NAME TestPID
  COMMAND test_pid
)
