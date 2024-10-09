add_library(tinympcstatic STATIC
    admm.cpp
    tiny_api.cpp
    codegen.cpp
)

set_property(TARGET tinympcstatic PROPERTY POSITION_INDEPENDENT_CODE ON)

# target_link_libraries(tinympcstatic PUBLIC Eigen)
target_include_directories(tinympcstatic PUBLIC ${EIGEN_DIR})

include(CTest)
set(TEST_DIR "test")

if (GEM5_ARMV7E-M_BUILD)
  add_arm_baremetal_gem5_se_executable(test_tinympc
    SOURCES ${TEST_DIR}/test_tinympc.cc
    LIBRARIES tinympcstatic startup sys)
elseif(STM32_BUILD)
  add_arm_semihosting_executable(
    test_tinympc
    SOURCES ${TEST_DIR}/test_tinympc.cc
    LIBRARIES tinympcstatic mcu-util
  )
else()
  add_non_arm_executable(test_tinympc
                         SOURCES ${TEST_DIR}/test_tinympc.cc
                         LIBRARIES tinympcstatic)
endif()
target_include_directories(test_tinympc
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN_DIR}
)

add_test(
  NAME TestTinyMPC
  COMMAND test_tinympc
)
