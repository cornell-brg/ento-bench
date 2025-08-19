#ifndef MADGWICK_CHECKED_H
#define MADGWICK_CHECKED_H

#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <ento-state-est/attitude-est/madgwick.h> // Original implementation
#include <ento-state-est/attitude-est-exp/core/failure_flags.h>
#include <ento-state-est/attitude-est-exp/core/fp_overflow_tracker.h>
#include <cmath>

namespace EntoAttitudeExp {

//------------------------------------------------------------------------------
// Checked floating-point Madgwick filter with failure detection
//------------------------------------------------------------------------------
template <typename Scalar, bool UseMag>
struct FilterMadgwickChecked
{
    // Reference to the original filter implementation
    EntoAttitude::FilterMadgwick<Scalar, UseMag> inner_filter_;

    static constexpr bool UseMag_ = UseMag;
    
    // Pointer to failure flags - set by external code before each call
    FailureFlags* failure_flags_;
    
    FilterMadgwickChecked() : failure_flags_(nullptr) {}
    
    // Set the failure flags pointer
    void setFailureFlags(FailureFlags* flags) {
        failure_flags_ = flags;
    }
    
    // 4-parameter interface for AttitudeProblem
    inline void operator()(const Eigen::Quaternion<Scalar>& q,
                          const EntoAttitude::AttitudeMeasurement<Scalar, UseMag>& meas,
                          Scalar dt,
                          Eigen::Quaternion<Scalar>* q_out)
    {
        // Use default gain of 0.001 for Madgwick
        *q_out = (*this)(q, meas, dt, Scalar(0.001));
    }

    // Madgwick-style interface (returns quaternion)
    inline Eigen::Quaternion<Scalar>
    operator()(const Eigen::Quaternion<Scalar>& q,
               const EntoAttitude::AttitudeMeasurement<Scalar, UseMag>& meas,
               Scalar dt,
               Scalar gain)
    {
        // Reset failure flags for this update
        if (failure_flags_) {
            failure_flags_->reset();
        }
        
        // Mahoney-style saturation/overflow check
        auto check_saturation = [&](const auto& val, const char* label) -> bool {
            float val_as_float = static_cast<float>(val);
            bool ok = std::isfinite(val_as_float);
            if (!ok && failure_flags_) {
                failure_flags_->set(FailureReason::Overflow);
            }
            return ok;
        };
        // Check all inputs for overflow/NaN
        if (!check_saturation(dt, "dt") || !check_saturation(gain, "gain")) {
            return q;
        }
        for (int i = 0; i < 3; ++i) {
            if (!check_saturation(meas.gyr[i], "gyr") || !check_saturation(meas.acc[i], "acc")) {
                return q;
            }
            if constexpr (UseMag) {
                if (!check_saturation(meas.mag[i], "mag")) {
                    return q;
                }
            }
        }
        if (!check_saturation(q.w(), "q.w") || !check_saturation(q.x(), "q.x") ||
            !check_saturation(q.y(), "q.y") || !check_saturation(q.z(), "q.z")) {
            return q;
        }
        // Clear fixed-point overflow flag (not applicable for float, but consistent interface)
        clearFpOverflowFlag();
        
        // Check for early exit conditions (near-zero divisors)
        bool early_exit = false;
        
        // Check if gyroscope norm is zero (early exit condition)
        if (meas.gyr.norm() == Scalar(0)) {
            early_exit = true;
        }
        
        // Check if accelerometer norm is zero (early exit condition)
        if (meas.acc.norm() == Scalar(0)) {
            early_exit = true;
        }
        
        // For MARG: check if magnetometer norm is zero (early exit condition)
        if constexpr (UseMag) {
            if (meas.mag.norm() == Scalar(0)) {
                early_exit = true;
            }
        }
        
        // Set near-zero divisor flag if early exit detected
        if (early_exit && failure_flags_) {
            failure_flags_->set(FailureReason::NearZeroDiv);
        }
        
        // Call the original filter implementation
        Eigen::Quaternion<Scalar> result = inner_filter_(q, meas, dt, gain);
        
        // Check for fixed-point overflow
        if (getFpOverflowFlag() && failure_flags_) {
            failure_flags_->set(FailureReason::Overflow);
        }
        
        // Check quaternion norm deviation
        Scalar norm_deviation = std::abs(result.norm() - Scalar(1.0));
        if (norm_deviation > Scalar(1e-3) && failure_flags_) {
            failure_flags_->set(FailureReason::BadNorm);
        }
        
        return result;
    }

    static constexpr const char* name()
    {
        if constexpr (UseMag)
        {
            return "Madgwick Attitude MARG Filter (Checked)";
        }
        else
        {
            return "Madgwick Attitude IMU Filter (Checked)";
        }
    }
};

} // namespace EntoAttitudeExp

#endif // MADGWICK_CHECKED_H 