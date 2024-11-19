# add_library(fglib STATIC
#     fg_chain_unicycle_interpol.cc
# )

include(CTest)

set(TEST_FILES
  fg-chain-state-est-test.cc
)

set(LIBRARIES_fg-chain-state-est-test
  ento_util Eigen
)

foreach(TEST_FILE ${TEST_FILES})
  message(STATUS "Test file: ${TEST_FILE}")
  set(TEST_SRC "test/${TEST_FILE}")
  get_filename_component(TEST_BIN ${TEST_FILE} NAME_WE)
  string(CONCAT LIBRARY_VAR_NAME "LIBRARIES_" ${TEST_BIN})
  message(STATUS ${${LIBRARY_VAR_NAME}})
  set(TEST_LIBRARIES ${${LIBRARY_VAR_NAME}})
  add_ento_test(
    ${TEST_BIN}
    SOURCES   ${TEST_SRC}
    LIBRARIES ${TEST_LIBRARIES}
  )
endforeach()

# set(TEST_DIR "test")

# if (GEM5_ARMV7E-M_BUILD)
#   add_arm_baremetal_gem5_se_executable(
#     test_fg_chain_state_est
#     SOURCES ${TEST_DIR}/fg-chain-state-est-test.cc
#     LIBRARIES startup sys
#   )
# elseif(STM32_BUILD)
#   add_arm_semihosting_executable(
#     test_fg_chain_state_est
#     SOURCES ${TEST_DIR}/fg-chain-state-est-test.cc
#     LIBRARIES mcu-util
#   )
# else()
#   add_non_arm_executable(
#     test_fg_chain_state_est
#     SOURCES ${TEST_DIR}/fg-chain-state-est-test.cc
#     LIBRARIES
#   )
# endif()
# target_include_directories(test_fg_chain_state_est
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
#   ${CMAKE_CURRENT_SOURCE_DIR}
#   ${EIGEN_DIR}
# )

# add_dependencies(check test_fg_chain_state_est)
# add_test(
#   NAME TestFgChainStateEst
#   COMMAND test_fg_chain_state_est
# )
