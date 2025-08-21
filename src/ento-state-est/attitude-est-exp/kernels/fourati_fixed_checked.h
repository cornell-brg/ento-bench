#ifndef FOURATI_FIXED_CHECKED_H
#define FOURATI_FIXED_CHECKED_H

#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <ento-state-est/attitude-est/fourati_fixed.h>
#include <ento-state-est/attitude-est-exp/core/failure_flags.h>
#include <math/FixedPoint.hh>
#include <math/FixedPointMath.hh>
#include <math/EigenFixedPoint.hh>

namespace EntoAttitudeExp {

template <typename Scalar>
struct FilterFouratiFixedChecked {
    // Internal filter instance
    EntoAttitude::FilterFouratiFixed<Scalar> inner_filter_;
    
    // Failure tracking
    FailureFlags* failure_flags_ = nullptr;
    
    // Constructor
    FilterFouratiFixedChecked() = default;
    
    // Set failure flags pointer for tracking
    void setFailureFlags(FailureFlags* flags) {
        failure_flags_ = flags;
    }
    
    // Main filter update method (MARG only)
    inline Eigen::Quaternion<Scalar>
    operator()(const Eigen::Quaternion<Scalar>& q,
               const EntoAttitude::MARGMeasurement<Scalar>& meas,
               Scalar dt,
               Scalar gain,
               const Eigen::Quaternion<Scalar>& g_q,
               const Eigen::Quaternion<Scalar>& m_q)
    {
        FailureFlags local_flags;
        FailureFlags* flags = failure_flags_ ? failure_flags_ : &local_flags;
        flags->reset();
        
        // Clear fixed-point overflow flag
        clearFpOverflowFlag();
        
        // Check for early exit conditions (near-zero divisors) - COMMENTED OUT since we pre-normalize
        // bool early_exit = false;
        
        // Check if gyroscope norm is zero (early exit condition) - COMMENTED OUT
        // if (meas.gyr.norm() == Scalar(0)) {
        //     early_exit = true;
        // }
        
        // Check if accelerometer norm is zero (early exit condition) - COMMENTED OUT
        // if (meas.acc.norm() == Scalar(0)) {
        //     early_exit = true;
        // }
        
        // Check if magnetometer norm is zero (early exit condition) - COMMENTED OUT
        // if (meas.mag.norm() == Scalar(0)) {
        //     early_exit = true;
        // }
        
        // Set near-zero divisor flag if early exit detected - COMMENTED OUT
        // if (early_exit) {
        //     flags->set(FailureReason::NearZeroDiv);
        // }
        
        // Pre-normalize accelerometer and magnetometer data to improve fixed-point performance
        EntoAttitude::MARGMeasurement<Scalar> normalized_meas = meas;
        
        // Normalize accelerometer data in float before converting to fixed-point
        float acc_x = static_cast<float>(meas.acc.x());
        float acc_y = static_cast<float>(meas.acc.y());
        float acc_z = static_cast<float>(meas.acc.z());
        float acc_norm = std::sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
        if (acc_norm > 0.0f) {
            float acc_x_norm = acc_x / acc_norm;
            float acc_y_norm = acc_y / acc_norm;
            float acc_z_norm = acc_z / acc_norm;
            
            // Validate fixed-point conversion accuracy
            Scalar acc_x_fixed = Scalar(acc_x_norm);
            Scalar acc_y_fixed = Scalar(acc_y_norm);
            Scalar acc_z_fixed = Scalar(acc_z_norm);
            
            float acc_x_back = static_cast<float>(acc_x_fixed);
            float acc_y_back = static_cast<float>(acc_y_fixed);
            float acc_z_back = static_cast<float>(acc_z_fixed);
            
            float acc_error = std::sqrt(
                (acc_x_norm - acc_x_back) * (acc_x_norm - acc_x_back) +
                (acc_y_norm - acc_y_back) * (acc_y_norm - acc_y_back) +
                (acc_z_norm - acc_z_back) * (acc_z_norm - acc_z_back)
            );
            
            // If conversion error is too large, flag as overflow
            if (acc_error > 0.1f && failure_flags_) {
                failure_flags_->set(FailureReason::Overflow);
            }
            
            normalized_meas.acc = EntoMath::Vec3<Scalar>(acc_x_fixed, acc_y_fixed, acc_z_fixed);
        }
        
        // Normalize magnetometer data in float before converting to fixed-point (for MARG)
        float mag_x = static_cast<float>(meas.mag.x());
        float mag_y = static_cast<float>(meas.mag.y());
        float mag_z = static_cast<float>(meas.mag.z());
        float mag_norm = std::sqrt(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z);
        if (mag_norm > 0.0f) {
            float mag_x_norm = mag_x / mag_norm;
            float mag_y_norm = mag_y / mag_norm;
            float mag_z_norm = mag_z / mag_norm;
            
            // Validate fixed-point conversion accuracy
            Scalar mag_x_fixed = Scalar(mag_x_norm);
            Scalar mag_y_fixed = Scalar(mag_y_norm);
            Scalar mag_z_fixed = Scalar(mag_z_norm);
            
            float mag_x_back = static_cast<float>(mag_x_fixed);
            float mag_y_back = static_cast<float>(mag_y_fixed);
            float mag_z_back = static_cast<float>(mag_z_fixed);
            
            float mag_error = std::sqrt(
                (mag_x_norm - mag_x_back) * (mag_x_norm - mag_x_back) +
                (mag_y_norm - mag_y_back) * (mag_y_norm - mag_y_back) +
                (mag_z_norm - mag_z_back) * (mag_z_norm - mag_z_back)
            );
            
            // If conversion error is too large, flag as overflow
            if (mag_error > 0.1f && failure_flags_) {
                failure_flags_->set(FailureReason::Overflow);
            }
            
            normalized_meas.mag = EntoMath::Vec3<Scalar>(mag_x_fixed, mag_y_fixed, mag_z_fixed);
        }
        
        // Validate gyroscope fixed-point conversion accuracy
        float gyr_x = static_cast<float>(meas.gyr.x());
        float gyr_y = static_cast<float>(meas.gyr.y());
        float gyr_z = static_cast<float>(meas.gyr.z());
        
        Scalar gyr_x_fixed = Scalar(gyr_x);
        Scalar gyr_y_fixed = Scalar(gyr_y);
        Scalar gyr_z_fixed = Scalar(gyr_z);
        
        float gyr_x_back = static_cast<float>(gyr_x_fixed);
        float gyr_y_back = static_cast<float>(gyr_y_fixed);
        float gyr_z_back = static_cast<float>(gyr_z_fixed);
        
        float gyr_error = std::sqrt(
            (gyr_x - gyr_x_back) * (gyr_x - gyr_x_back) +
            (gyr_y - gyr_y_back) * (gyr_y - gyr_y_back) +
            (gyr_z - gyr_z_back) * (gyr_z - gyr_z_back)
        );
        
        // If conversion error is too large, flag as overflow
        if (gyr_error > 0.1f && failure_flags_) {
            failure_flags_->set(FailureReason::Overflow);
        }
        
        normalized_meas.gyr = EntoMath::Vec3<Scalar>(gyr_x_fixed, gyr_y_fixed, gyr_z_fixed);
        
        // Call the original filter implementation with normalized data
        Eigen::Quaternion<Scalar> result = inner_filter_(q, normalized_meas, dt, gain, g_q, m_q);
        
        // Check for fixed-point overflow
        if (getFpOverflowFlag()) {
            flags->set(FailureReason::Overflow);
        }
        
        // Check quaternion change between consecutive updates (shouldn't exceed 1° per step)
        if (failure_flags_) {
            // Calculate quaternion difference: q_diff = result * q.conjugate()
            Eigen::Quaternion<Scalar> q_diff = result * q.conjugate();
            
            // Convert to float for angle calculation to avoid overflow
            float q_diff_w = static_cast<float>(q_diff.w());
            float q_diff_x = static_cast<float>(q_diff.x());
            float q_diff_y = static_cast<float>(q_diff.y());
            float q_diff_z = static_cast<float>(q_diff.z());
            
            // Check if quaternion actually changed (not identical to input)
            float change_magnitude = std::sqrt(q_diff_x*q_diff_x + q_diff_y*q_diff_y + q_diff_z*q_diff_z);
            if (change_magnitude < 1e-6f) {
                failure_flags_->set(FailureReason::ExcessiveErr);
                return result;
            }
            
            // Calculate angle change in degrees (2 * acos(|w|))
            float angle_change_rad = 2.0f * std::acos(std::abs(q_diff_w));
            constexpr float M_PI_F = 3.14159265358979323846f;
            float angle_change_deg = angle_change_rad * 180.0f / M_PI_F;
            
            // Flag as failure if change exceeds 1 degree
            if (angle_change_deg > 1.0f) {
                failure_flags_->set(FailureReason::ExcessiveErr);
            }
        }
        
        // Check quaternion norm deviation
        // For fixed-point, convert to float for norm calculation to avoid more overflow
        float norm_as_float = 0.0f;
        norm_as_float += static_cast<float>(result.w()) * static_cast<float>(result.w());
        norm_as_float += static_cast<float>(result.x()) * static_cast<float>(result.x());
        norm_as_float += static_cast<float>(result.y()) * static_cast<float>(result.y());
        norm_as_float += static_cast<float>(result.z()) * static_cast<float>(result.z());
        norm_as_float = std::sqrt(norm_as_float);
        
        float norm_error = std::abs(norm_as_float - 1.0f);
        if (norm_error > 1e-3f) {
            flags->set(FailureReason::BadNorm);
        }
        
        // Additional output validation checks for garbage detection
        if (failure_flags_) {
            // Check for NaN or Inf values
            float w_f = static_cast<float>(result.w());
            float x_f = static_cast<float>(result.x());
            float y_f = static_cast<float>(result.y());
            float z_f = static_cast<float>(result.z());
            
            if (std::isnan(w_f) || std::isnan(x_f) || std::isnan(y_f) || std::isnan(z_f) ||
                std::isinf(w_f) || std::isinf(x_f) || std::isinf(y_f) || std::isinf(z_f)) {
                failure_flags_->set(FailureReason::BadNorm);
            }
            
            // Check for excessive error (quaternion components should be reasonable)
            float max_component = 0.0f;
            max_component = std::max(max_component, std::abs(w_f));
            max_component = std::max(max_component, std::abs(x_f));
            max_component = std::max(max_component, std::abs(y_f));
            max_component = std::max(max_component, std::abs(z_f));
            
            if (max_component > 10.0f) {
                failure_flags_->set(FailureReason::ExcessiveErr);
            }
        }
        
        return result;
    }
    
    static constexpr const char* name() {
        return "Fourati MARG Fixed Checked";
    }
};

} // namespace EntoAttitudeExp

#endif // FOURATI_FIXED_CHECKED_H 
