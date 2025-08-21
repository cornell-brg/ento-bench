#ifndef ENTO_ATTITUDE_FP_OVERFLOW_TRACKER_H
#define ENTO_ATTITUDE_FP_OVERFLOW_TRACKER_H

namespace EntoAttitudeExp {

/**
 * @brief Clear the fixed-point overflow flag for the current thread
 * 
 * This should be called before each attitude filter update to reset
 * the overflow detection state.
 */
void clearFpOverflowFlag();

/**
 * @brief Check if a fixed-point overflow occurred since the last clear
 * 
 * @return true if any fixed-point arithmetic operation resulted in 
 *         overflow/saturation since the last call to clearFpOverflowFlag()
 */
bool getFpOverflowFlag();

/**
 * @brief Internal function to set the overflow flag (called by instrumented FP ops)
 * 
 * This function is called internally by instrumented fixed-point arithmetic
 * operations when they detect overflow/saturation. User code should not call
 * this directly.
 */
void setFpOverflowFlag();

} // namespace EntoAttitudeExp

#endif // ENTO_ATTITUDE_FP_OVERFLOW_TRACKER_H 