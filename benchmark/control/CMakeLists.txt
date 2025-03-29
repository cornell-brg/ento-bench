include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if(STM32_BUILD)
    set(MODULES
        bench
        ento-mcu-semihosted
        Eigen
        ento_util
        ento-control
        CMSIS::STM32::${STM_PRODUCT}
    )
    set(BENCH_TARGETS)

    add_benchmark(bench-tinympc
        SOURCES bench_tinympc.cc
        LIBRARIES ${MODULES}
    )
    list(APPEND BENCH_TARGETS bench-tinympc)

    message(STATUS "Bench Targets: ${BENCH_TARGETS}")
    add_stm32_targets("${BENCH_TARGETS}")
endif()
