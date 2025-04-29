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


void __attribute__((noinline)) fp_mul_benchmark() {
    constexpr int reps = 600000;

    register float r0 asm("s0") = 0.0f;
    register float r1 asm("s1") = 0.0f;
    register float r2 asm("s2") = 0.0f;
    register float r3 asm("s3") = 0.0f;
    register float r4 asm("s4") = 0.0f;
    register float r5 asm("s5") = 0.0f;

    register float r6 asm("s6") = 0.0f;
    register float r7 asm("s7") = 0.0f;
    register float r8 asm("s8") = 0.0f;
    register float r9 asm("s9") = 0.0f;
    register float r10 asm("s10") = 0.0f;
    register float r11 asm("s11") = 0.0f;

    start_roi();
    for (int i = 0; i < reps; i++) {
        asm volatile (
            ".rept 8                 \n"
            "  vmul.f32 s6, s0, s6   \n"
            "  vmul.f32 s7, s1, s7   \n"
            "  vmul.f32 s8, s2, s8   \n"
            "  vmul.f32 s9, s3, s9   \n"
            "  vmul.f32 s10, s4, s10 \n"
            "  vmul.f32 s11, s5, s11 \n"
            ".endr                   \n"
            : "+t"(r6), "+t"(r7), "+t"(r8), "+t"(r9), "+t"(r10), "+t"(r11) // output
            : "t"(r0), "t"(r1), "t"(r2), "t"(r3), "t"(r4), "t"(r5)        // input
            : 
        );
    }
    end_roi();
}
