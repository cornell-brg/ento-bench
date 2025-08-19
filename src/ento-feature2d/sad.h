#ifndef SAD_H
#define SAD_H

#include <cstdint>

// Include benchmark configuration for vectorization control
#include <ento-bench/bench_config.h>

// Include debug utilities for fallback implementation
#include <ento-util/debug.h>

// Sum of Absolute Differences for 8x8 blocks
// Optimized for ARM Cortex-M with USADA8 instruction
// frame1 and frame2 should point to the top-left corner of 8x8 blocks
// with stride of 320 pixels

// Safe version with bounds checking for debugging
inline int sad_safe(const uint8_t* frame1, const uint8_t* frame2, int stride = 320, int width = 320, int height = 320) {
    if (!frame1 || !frame2) {
        return -1; // Invalid pointers
    }
    
    int result = 0;
    
    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 8; ++col) {
            // Check bounds
            int offset = row * stride + col;
            if (offset < 0 || offset >= width * height) {
                return -2; // Out of bounds
            }
            
            int diff = static_cast<int>(frame1[offset]) - static_cast<int>(frame2[offset]);
            result += (diff < 0) ? -diff : diff;  // abs(diff)
        }
    }
    return result;
}

inline int sad(const uint8_t* frame1, const uint8_t* frame2, int stride = 80) {
#if ENABLE_VECTORIZATION==1
    // ARM assembly version using USADA8 instruction
    // This is enabled when ENABLE_VECTORIZATION=true in config and running on ARM targets
    int result = 0;
    asm volatile(
        "mov %[result], #0\n"           // accumulator

        "ldr r4, [%[src], #0]\n"        // read data from address + offset
        "ldr r5, [%[dst], #0]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference
        "ldr r4, [%[src], #4]\n"        // read data from address + offset
        "ldr r5, [%[dst], #4]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference

        "ldr r4, [%[src], #(80 * 1)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 1)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference
        "ldr r4, [%[src], #(80 * 1 + 4)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 1 + 4)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference

        "ldr r4, [%[src], #(80 * 2)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 2)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference
        "ldr r4, [%[src], #(80 * 2 + 4)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 2 + 4)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference

        "ldr r4, [%[src], #(80 * 3)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 3)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference
        "ldr r4, [%[src], #(80 * 3 + 4)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 3 + 4)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference

        "ldr r4, [%[src], #(80 * 4)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 4)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference
        "ldr r4, [%[src], #(80 * 4 + 4)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 4 + 4)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference

        "ldr r4, [%[src], #(80 * 5)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 5)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference
        "ldr r4, [%[src], #(80 * 5 + 4)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 5 + 4)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference

        "ldr r4, [%[src], #(80 * 6)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 6)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference
        "ldr r4, [%[src], #(80 * 6 + 4)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 6 + 4)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference

        "ldr r4, [%[src], #(80 * 7)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 7)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference
        "ldr r4, [%[src], #(80 * 7 + 4)]\n"        // read data from address + offset
        "ldr r5, [%[dst], #(80 * 7 + 4)]\n"
        "usada8 %[result], r4, r5, %[result]\n"      // difference

        : [result] "+r" (result)
        : [src] "r" (frame1), [dst] "r" (frame2)
        : "r4", "r5"
    );
    return result;
#else
    // Portable C++ version - compute SAD for 8x8 block
    // This is used when ENABLE_VECTORIZATION=false or on non-ARM targets
    ENTO_DEBUG("sad: Starting with frame1=%p, frame2=%p, stride=%d", frame1, frame2, stride);
    
    // Add basic null pointer check
    if (!frame1 || !frame2) {
        ENTO_DEBUG("sad: Null pointer detected!");
        return -1;
    }
    
    int result = 0;
    
    ENTO_DEBUG("sad: Starting 8x8 loop");
    for (int row = 0; row < 8; ++row) {
        ENTO_DEBUG("sad: Processing row %d", row);
        for (int col = 0; col < 8; ++col) {
            int offset = row * stride + col;
            ENTO_DEBUG("sad: row=%d, col=%d, offset=%d", row, col, offset);
            int diff = static_cast<int>(frame1[offset]) - static_cast<int>(frame2[offset]);
            result += (diff < 0) ? -diff : diff;  // abs(diff)
        }
    }
    ENTO_DEBUG("sad: Completed, result=%d", result);
    return result;
#endif
}

// Debug version that can be used for testing
inline int absdiff(const uint8_t* frame1, const uint8_t* frame2, int stride = 80) {
    return sad(frame1, frame2, stride);
}

#endif // SAD_H 
