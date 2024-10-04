include (${CMAKE_SOURCE_DIR}/apps/apps_common.cmake)

set(SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/benchmark_example.cc)

set(MODULES bench CMSIS::STM32::G474RE)

message("Building benchmark_example")
add_benchmark(benchmark_example
              SOURCES ${SOURCE_FILE}
              LIBRARIES ${MODULES})
