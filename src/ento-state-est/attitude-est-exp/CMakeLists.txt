cmake_minimum_required(VERSION 3.16)

# Experimental attitude estimation library with failure checking
add_library(ento-attitude-est-exp INTERFACE)

target_include_directories(ento-attitude-est-exp
  INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ento-attitude-est-exp
  INTERFACE
  ento-attitude-est
  ento-attitude-failure-core
  ento_math
  ento_util
)

target_include_directories(ento-attitude-est-exp
  INTERFACE
  ${EIGEN_DIR}
)

# Add subdirectories for core components
add_subdirectory(core)

# Conditionally build the experimental benchmarks
if(ENTO_ENABLE_FAILURE_CHECKS)
  # Original 6-format benchmark
  add_executable(bench_attitude_checked bench_attitude_checked.cc)
  target_link_libraries(bench_attitude_checked 
    ento-attitude-est-exp
    ento-attitude-est
    ento_util
    ento_math
  )
  
  target_include_directories(bench_attitude_checked PRIVATE
    ${EIGEN_DIR}
  )
  target_include_directories(bench_attitude_checked PRIVATE 
    ${CMAKE_SOURCE_DIR}/src 
    ${EIGEN_DIR}
  )
  
  # Comprehensive 48-format sweep
  add_executable(comprehensive_qformat_sweep comprehensive_sweep.cc)
  target_link_libraries(comprehensive_qformat_sweep
    ento-attitude-est-exp
    ento-attitude-est
    ento_util
    ento_math
  )
  
  target_include_directories(comprehensive_qformat_sweep PRIVATE
    ${EIGEN_DIR}
  )
  target_include_directories(comprehensive_qformat_sweep PRIVATE 
    ${CMAKE_SOURCE_DIR}/src 
    ${EIGEN_DIR}
  )
  
  # Multi-kernel comprehensive sweep
  add_executable(comprehensive_multi_kernel_sweep comprehensive_multi_kernel_sweep.cc)
  target_link_libraries(comprehensive_multi_kernel_sweep
    ento-attitude-est-exp
    ento-attitude-est
    ento_util
    ento_math
  )
  
  target_include_directories(comprehensive_multi_kernel_sweep PRIVATE
    ${EIGEN_DIR}
  )
  target_include_directories(comprehensive_multi_kernel_sweep PRIVATE 
    ${CMAKE_SOURCE_DIR}/src 
    ${EIGEN_DIR}
  )
  
  # Debug test for 32-bit Mahoney vs Madgwick
  add_executable(debug_32bit_test debug_32bit_test.cc)
  target_link_libraries(debug_32bit_test
    ento-attitude-est-exp
    ento-attitude-est
    ento_util
    ento_math
  )
  
  target_include_directories(debug_32bit_test PRIVATE
    ${EIGEN_DIR}
  )
  target_include_directories(debug_32bit_test PRIVATE 
    ${CMAKE_SOURCE_DIR}/src 
    ${EIGEN_DIR}
  )
endif() 