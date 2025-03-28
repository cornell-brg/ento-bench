include(${CMAKE_SOURCE_DIR}/entobench_helpers.cmake)

if(STM32_BUILD)
    set(MODULES
        bench
        mcu-util
        Eigen
        ento_util
        ento-control
        CMSIS::STM32::${STM_PRODUCT}
    )
    set(BENCH_TARGETS)

    add_benchmark(bench-TinyMPC
        SOURCES tiny_mpc.cc
        LIBRARIES ${MODULES}
    )
    list(APPEND BENCH_TARGETS bench-TinyMPC)

    message(STATUS "Bench Targets: ${BENCH_TARGETS}")
    add_stm32_targets("${BENCH_TARGETS}")
endif()
