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

static inline int rand_int_nonzero() {
    return (rand() % 161) + 1;
}

static inline int rand_int() {
    return (rand() % 161);
}

void __attribute__((noinline)) int_div_benchmark() {
    constexpr int reps = 600000;

    register int r0 asm("r0") = rand_int_nonzero();
    register int r1 asm("r1") = rand_int_nonzero();
    register int r2 asm("r2") = rand_int_nonzero();
    register int r3 asm("r3") = rand_int_nonzero();
    register int r4 asm("r4") = rand_int_nonzero();
    register int r5 asm("r5") = rand_int_nonzero();

    register int r6 asm("r6")   = rand_int();
    register int r7 asm("r7")   = rand_int();
    register int r8 asm("r8")   = rand_int();
    register int r9 asm("r9")   = rand_int();
    register int r10 asm("r10") = rand_int();
    register int r11 asm("r11") = rand_int();

    start_roi();
    for (int i = 0; i < reps; i++) {
        asm volatile (
            ".rept 8             \n"
            "  sdiv r6, r6, r0   \n"
            "  sdiv r7, r7, r1   \n"
            "  sdiv r8, r8, r2   \n"
            "  sdiv r9, r9, r3   \n"
            "  sdiv r10, r10, r4 \n"
            "  sdiv r11, r11, r5 \n"
            ".endr               \n"
            : "+r"(r6), "+r"(r7), "+r"(r8), "+r"(r9), "+r"(r10), "+r"(r11)
            : "r"(r0), "r"(r1), "r"(r2), "r"(r3), "r"(r4), "r"(r5)
        );
    }
    end_roi();
}
