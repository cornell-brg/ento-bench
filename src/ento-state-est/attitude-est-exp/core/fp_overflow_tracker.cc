#include "fp_overflow_tracker.h"

namespace EntoAttitudeExp {

// Thread-local storage for overflow flag
// Each thread gets its own copy of this flag
thread_local bool g_fp_overflow_flag = false;

void clearFpOverflowFlag() {
    g_fp_overflow_flag = false;
}

bool getFpOverflowFlag() {
    return g_fp_overflow_flag;
}

void setFpOverflowFlag() {
    g_fp_overflow_flag = true;
}

} // namespace EntoAttitudeExp 