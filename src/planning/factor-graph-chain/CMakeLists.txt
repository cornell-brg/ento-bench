# add_library(fglib STATIC
#     fg_chain_unicycle_interpol.cc
# )

include(CTest)
set(TEST_DIR "test")

if (GEM5_ARMV7E-M_BUILD)
  add_arm_baremetal_gem5_se_executable(test_fg
    SOURCES ${TEST_DIR}/test_fg.cc
    LIBRARIES startup sys)
elseif(STM32_BUILD)
  add_arm_semihosting_executable(
    test_fg
    SOURCES ${TEST_DIR}/test_fg.cc
    LIBRARIES mcu-util
  )
else()
  add_non_arm_executable(test_fg
                         SOURCES ${TEST_DIR}/test_fg.cc
                         LIBRARIES )
endif()
target_include_directories(test_fg
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_DIR}
)

add_dependencies(check test_fg)
add_test(
  NAME TestFg
  COMMAND test_fg
)
