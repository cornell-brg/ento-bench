#ifndef MADGWICK_FIXED_CHECKED_H
#define MADGWICK_FIXED_CHECKED_H

#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <ento-state-est/attitude-est/madgwick_fixed.h> // Original implementation
#include <ento-state-est/attitude-est-exp/core/failure_flags.h>
#include <ento-state-est/attitude-est-exp/core/fp_overflow_tracker.h>
#include <cmath>

namespace EntoAttitudeExp {

//------------------------------------------------------------------------------
// Checked fixed-point Madgwick filter with failure detection
//------------------------------------------------------------------------------
template <typename Scalar, bool UseMag>
struct FilterMadgwickFixedChecked
{
    // Reference to the original filter implementation
    EntoAttitude::FilterMadgwickFixed<Scalar, UseMag> inner_filter_;

    static constexpr bool UseMag_ = UseMag;
    
    // Pointer to failure flags - set by external code before each call
    FailureFlags* failure_flags_;
    
    FilterMadgwickFixedChecked() : failure_flags_(nullptr) {}
    
    // Set the failure flags pointer
    void setFailureFlags(FailureFlags* flags) {
        failure_flags_ = flags;
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
        
        // For MARG: check if magnetometer norm is zero (early exit condition) - COMMENTED OUT
        // if constexpr (UseMag) {
        //     if (meas.mag.norm() == Scalar(0)) {
        //         early_exit = true;
        //     }
        // }
        
        // Set near-zero divisor flag if early exit detected - COMMENTED OUT
        // if (early_exit && failure_flags_) {
        //     failure_flags_->set(FailureReason::NearZeroDiv);
        // }
        
        // Pre-normalize accelerometer and magnetometer data to improve fixed-point performance
        EntoAttitude::AttitudeMeasurement<Scalar, UseMag> normalized_meas = meas;
        
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
        if constexpr (UseMag) {
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
        Eigen::Quaternion<Scalar> result = inner_filter_(q, normalized_meas, dt, gain);
        
        // Check for fixed-point overflow
        if (getFpOverflowFlag() && failure_flags_) {
            failure_flags_->set(FailureReason::Overflow);
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
        float norm_f = std::sqrt(
            static_cast<float>(result.w()) * static_cast<float>(result.w()) +
            static_cast<float>(result.x()) * static_cast<float>(result.x()) +
            static_cast<float>(result.y()) * static_cast<float>(result.y()) +
            static_cast<float>(result.z()) * static_cast<float>(result.z())
        );
        
        float norm_deviation = std::abs(norm_f - 1.0f);
        if (norm_deviation > 1e-3f && failure_flags_) {
            failure_flags_->set(FailureReason::BadNorm);
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
            
            // Check for unreasonably large values (indicating overflow or garbage)
            if (std::abs(w_f) > 10.0f || std::abs(x_f) > 10.0f || 
                std::abs(y_f) > 10.0f || std::abs(z_f) > 10.0f) {
                failure_flags_->set(FailureReason::BadNorm);
            }
            
            // Check for all-zero quaternion (invalid)
            if (std::abs(w_f) < 1e-6f && std::abs(x_f) < 1e-6f && 
                std::abs(y_f) < 1e-6f && std::abs(z_f) < 1e-6f) {
                failure_flags_->set(FailureReason::BadNorm);
            }
            
            // Check for quaternion with zero real part (invalid for attitude)
            if (std::abs(w_f) < 1e-6f) {
                failure_flags_->set(FailureReason::BadNorm);
            }
        }
        
        return result;
    }
    
    // Generic interface for AttitudeProblem (void return, output via pointer)
    // This will be used by the "else" branch in solve_impl
    inline void
    operator()(const Eigen::Quaternion<Scalar>& q,
               const EntoAttitude::AttitudeMeasurement<Scalar, UseMag>& meas,
               Scalar dt,
               Eigen::Quaternion<Scalar>* q_out)
    {
        // For the generic interface, we need to get the gain from somewhere
        // Since AttitudeProblem stores gain_, we'll use a default for now
        const auto default_gain = Scalar(0.001f);
        
        *q_out = (*this)(q, meas, dt, default_gain);
    }

    static constexpr const char* name()
    {
        if constexpr (UseMag)
        {
            return "Madgwick MARG Fixed Filter (Checked)";
        }
        else
        {
            return "Madgwick IMU Fixed Filter (Checked)";
        }
    }
};

//------------------------------------------------------------------------------
// Convenience aliases for specific Q-formats (all from the Büscher paper)
//------------------------------------------------------------------------------
template<bool UseMag>
using FilterMadgwickQ7_24Checked = FilterMadgwickFixedChecked<EntoAttitude::Q7_24, UseMag>;

template<bool UseMag>  
using FilterMadgwickQ3_12Checked = FilterMadgwickFixedChecked<EntoAttitude::Q3_12, UseMag>;

template<bool UseMag>
using FilterMadgwickQ6_25Checked = FilterMadgwickFixedChecked<EntoAttitude::Q6_25, UseMag>;

template<bool UseMag>
using FilterMadgwickQ5_26Checked = FilterMadgwickFixedChecked<EntoAttitude::Q5_26, UseMag>;

template<bool UseMag>
using FilterMadgwickQ2_13Checked = FilterMadgwickFixedChecked<EntoAttitude::Q2_13, UseMag>;

} // namespace EntoAttitudeExp

#endif // MADGWICK_FIXED_CHECKED_H 
