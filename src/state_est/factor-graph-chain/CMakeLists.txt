# add_library(fglib STATIC
#     fg_chain_unicycle_interpol.cc
# )

include(CTest)
set(TEST_DIR "test")

if (GEM5_ARMV7E-M_BUILD)
  add_arm_baremetal_gem5_se_executable(
    test_fg_chain_state_est
    SOURCES ${TEST_DIR}/fg-chain-state-est-test.cc
    LIBRARIES startup sys
  )
elseif(STM32_BUILD)
  add_arm_semihosting_executable(
    test_fg_chain_state_est
    SOURCES ${TEST_DIR}/fg-chain-state-est-test.cc
    LIBRARIES mcu-util
  )
else()
  add_non_arm_executable(
    test_fg_chain_state_est
    SOURCES ${TEST_DIR}/fg-chain-state-est-test.cc
    LIBRARIES
  )
endif()
target_include_directories(test_fg_chain_state_est
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_DIR}
)

add_dependencies(check test_fg_chain_state_est)
add_test(
  NAME TestFgChainStateEst
  COMMAND test_fg_chain_state_est
)
