#include <stdlib.h>
#include <stdio.h>
#include <ento-bench/harness.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-bench/roi.h>

#if defined(SEMIHOSTING)
extern "C" void initialise_monitor_handles(void);
#endif

static inline float rand_float_01() {
    return ((float)rand() / (float)RAND_MAX) * 160.0f;
}

static inline float rand_float() {
    return ((float)rand() / (float)RAND_MAX) * 160.0f + 1.0f;
}


void __attribute__((noinline)) fp_div_benchmark() {
    constexpr int reps = 600000;

    register float r0 asm("s0") = rand_float();
    register float r1 asm("s1") = rand_float();
    register float r2 asm("s2") = rand_float();
    register float r3 asm("s3") = rand_float();
    register float r4 asm("s4") = rand_float();
    register float r5 asm("s5") = rand_float();

    register float r6 asm("s6")   = rand_float_01();
    register float r7 asm("s7")   = rand_float_01();
    register float r8 asm("s8")   = rand_float_01();
    register float r9 asm("s9")   = rand_float_01();
    register float r10 asm("s10") = rand_float_01();
    register float r11 asm("s11") = rand_float_01();

    start_roi();
    for (int i = 0; i < reps; i++) {
        asm volatile (
            ".rept 8                  \n"
            "  vdiv.f32 s6, s6, s0    \n"
            "  vdiv.f32 s7, s7, s1    \n"
            "  vdiv.f32 s8, s8, s2    \n"
            "  vdiv.f32 s9, s9, s3    \n"
            "  vdiv.f32 s10, s10, s4  \n"
            "  vdiv.f32 s11, s11, s5  \n"
            ".endr                    \n"
            : "+t"(r6), "+t"(r7), "+t"(r8), "+t"(r9), "+t"(r10), "+t"(r11) // outputs 
            : "t"(r0), "t"(r1), "t"(r2), "t"(r3), "t"(r4), "t"(r5)         // inputs 
            :
        );
    }
    end_roi();
}

