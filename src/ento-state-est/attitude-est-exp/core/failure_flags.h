#ifndef ENTO_ATTITUDE_FAILURE_FLAGS_H
#define ENTO_ATTITUDE_FAILURE_FLAGS_H

namespace EntoAttitudeExp {

/**
 * @brief Enumeration of different failure types that can occur during attitude estimation
 */
enum class FailureReason {
    Overflow = 0,      ///< Fixed-point arithmetic overflow/saturation
    BadNorm = 1,       ///< Quaternion norm deviates by more than 1e-3 from unity  
    NearZeroDiv = 2,   ///< Early exit due to near-zero divisors (e.g., zero accel/mag norm)
    ExcessiveErr = 3   ///< Instantaneous attitude error exceeds 2.5° (ground truth required)
};

/**
 * @brief Structure to track which failure conditions occurred during a single update
 */
struct FailureFlags {
    bool hit[4] = {false, false, false, false};
    
    /**
     * @brief Reset all failure flags to false
     */
    void reset() {
        for (int i = 0; i < 4; ++i) {
            hit[i] = false;
        }
    }
    
    /**
     * @brief Check if any failure occurred
     * @return true if any failure flag is set
     */
    bool any() const {
        for (int i = 0; i < 4; ++i) {
            if (hit[i]) return true;
        }
        return false;
    }
    
    /**
     * @brief Get failure flag for specific reason
     * @param reason The failure reason to check
     * @return true if this failure occurred
     */
    bool operator[](FailureReason reason) const {
        return hit[static_cast<int>(reason)];
    }
    
    /**
     * @brief Set failure flag for specific reason
     * @param reason The failure reason to set
     * @param value The value to set (default true)
     */
    void set(FailureReason reason, bool value = true) {
        hit[static_cast<int>(reason)] = value;
    }
};

} // namespace EntoAttitudeExp

#endif // ENTO_ATTITUDE_FAILURE_FLAGS_H 