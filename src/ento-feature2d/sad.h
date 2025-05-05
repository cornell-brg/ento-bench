#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <math/FixedPoint.hh>
#include <image_io/Image.h>
#include <ento-feature2d/feat2d_util.h>


#ifdef NATIVE

uint32_t usada8(uint32_t a, uint32_t b, uint32_t acc) {
    for (int i = 0; i < 4; ++i) {
        uint8_t byte_a = (a >> (8 * i)) & 0xFF;
        uint8_t byte_b = (b >> (8 * i)) & 0xFF;
        acc += std::abs((int)byte_a - (int)byte_b);
    }
    return acc;
}


int sad(const uint8_t* frame1, const uint8_t* frame2) {
    const int stride = 320; // assumed row size
    uint32_t result = 0;

    for (int row = 0; row < 8; row++) {
        int base = row * stride;

        // first 4 pixels in the row
        uint32_t a0 = *reinterpret_cast<const uint32_t*>(&frame1[base + 0]);
        uint32_t b0 = *reinterpret_cast<const uint32_t*>(&frame2[base + 0]);
        result = usada8(a0, b0, result);

        // second 4 pixels in the row
        uint32_t a1 = *reinterpret_cast<const uint32_t*>(&frame1[base + 4]);
        uint32_t b1 = *reinterpret_cast<const uint32_t*>(&frame2[base + 4]);
        result = usada8(a1, b1, result);
    }

    return result;
}
#else
int sad(const uint8_t* frame1, const uint8_t* frame2)
{
 int result = 0;
 asm volatile( \
  "mov %[result], #0\n"           /* accumulator */ \
 \
  "ldr r4, [%[src], #0]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #0]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #4]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #4]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(320 * 1)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(320 * 1)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(320 * 1 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(320 * 1 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(320 * 2)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(320 * 2)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(320 * 2 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(320 * 2 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(320 * 3)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(320 * 3)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(320 * 3 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(320 * 3 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(320 * 4)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(320 * 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(320 * 4 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(320 * 4 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(320 * 5)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(320 * 5)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(320 * 5 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(320 * 5 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(320 * 6)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(320 * 6)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(320 * 6 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(320 * 6 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(320 * 7)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(320 * 7)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(320 * 7 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(320 * 7 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  : [result] "+r" (result) \
  : [src] "r" (frame1), [dst] "r" (frame2) \
  : "r4", "r5" \
  );
  return result;
}
#endif