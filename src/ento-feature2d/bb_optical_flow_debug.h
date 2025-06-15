#ifndef BB_OPTICAL_FLOW_DEBUG_H
#define BB_OPTICAL_FLOW_DEBUG_H

#include <cstdint>

// Debug version of absdiff for testing
inline int absdiff(const uint8_t* frame1, const uint8_t* frame2) {
    int result = 0;
    constexpr int stride = 320;
    
    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 8; ++col) {
            int diff = static_cast<int>(frame1[row * stride + col]) - 
                      static_cast<int>(frame2[row * stride + col]);
            result += (diff < 0) ? -diff : diff;  // abs(diff)
        }
    }
    return result;
}

#endif // BB_OPTICAL_FLOW_DEBUG_H 