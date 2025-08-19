#ifndef MAHONEY_CHECKED_H
#define MAHONEY_CHECKED_H

#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <ento-state-est/attitude-est-exp/core/failure_flags.h>

namespace EntoAttitudeExp {

template <typename Scalar>
Eigen::Quaternion<Scalar> mahoney_update_imu_checked(
    const Eigen::Quaternion<Scalar>& q,
    const EntoMath::Vec3<Scalar>& gyr,
    const EntoMath::Vec3<Scalar>& acc,
    Scalar dt,
    Scalar k_p,
    Scalar k_i,
    EntoMath::Vec3<Scalar>& bias,
    FailureFlags& flags)
{
    flags.reset();
    
    // Check for overflow/saturation in inputs
    if (!std::isfinite(gyr.norm()) || !std::isfinite(acc.norm()) || 
        !std::isfinite(dt) || !std::isfinite(k_p) || !std::isfinite(k_i)) {
        flags.set(FailureReason::Overflow);
        return q;
    }
    
    // If the gyroscope reading is zero, return the current quaternion.
    if (gyr.norm() == Scalar(0))
        return q;

    // Make a local copy of the quaternion
    Eigen::Quaternion<Scalar> q_new = q;

    // Copy the gyroscope measurement.
    Eigen::Matrix<Scalar, 3, 1> omega = gyr;
    Scalar a_norm = acc.norm();

    // Only perform correction if accelerometer reading is valid.
    if (a_norm > Scalar(0))
    {
        // Check for near-zero divisor
        if (a_norm < std::numeric_limits<Scalar>::epsilon() * 1000) {
            flags.set(FailureReason::NearZeroDiv);
        }
        
        // Compute the rotation matrix from the current quaternion.
        const Eigen::Matrix<Scalar, 3, 3> R = q_new.toRotationMatrix();
        
        // Check rotation matrix validity
        if (!std::isfinite(R.norm())) {
            flags.set(FailureReason::Overflow);
            return q;
        }
        
        // Expected direction of gravity in the body frame
        const Eigen::Matrix<Scalar, 3, 1> v_a = R.transpose() *
            Eigen::Matrix<Scalar, 3, 1>(Scalar(0), Scalar(0), Scalar(1));

        // Normalize the accelerometer reading.
        const Eigen::Matrix<Scalar, 3, 1> acc_normalized = acc / a_norm;

        // Innovation term (error) computed as the cross product
        const Eigen::Matrix<Scalar, 3, 1> omega_mes = acc_normalized.cross(v_a);
        
        // Check for overflow in omega_mes
        if (!std::isfinite(omega_mes.norm())) {
            flags.set(FailureReason::Overflow);
            return q;
        }

        // Compute the derivative of the bias (integral term).
        const Eigen::Matrix<Scalar, 3, 1> b_dot = -k_i * omega_mes;

        // Update the bias.
        bias += b_dot * dt;
        
        // Check bias for overflow
        if (!std::isfinite(bias.norm())) {
            flags.set(FailureReason::Overflow);
        }

        // Correct the measured angular velocity.
        omega = omega - bias + k_p * omega_mes;
        
        // Check corrected omega for overflow
        if (!std::isfinite(omega.norm())) {
            flags.set(FailureReason::Overflow);
        }
    }

    // Create a pure quaternion from the angular velocity: (0, omega).
    const Eigen::Quaternion<Scalar> p(Scalar(0), omega.x(), omega.y(), omega.z());

    // Compute the quaternion derivative: q_dot = 0.5 * q * p.
    Eigen::Quaternion<Scalar> q_dot = q_new * p;
    q_dot.coeffs() *= Scalar(0.5);
    
    // Check q_dot for overflow
    if (!std::isfinite(q_dot.norm())) {
        flags.set(FailureReason::Overflow);
        return q;
    }

    // Integrate to get the new quaternion.
    q_new.coeffs() += q_dot.coeffs() * dt;
    
    // Check pre-normalization quaternion
    if (!std::isfinite(q_new.norm())) {
        flags.set(FailureReason::Overflow);
        return q;
    }
    
    // Check if quaternion norm is reasonable before normalization
    Scalar norm_before = q_new.norm();
    if (norm_before < Scalar(1e-6)) {
        flags.set(FailureReason::NearZeroDiv);
        return q;
    }
    
    q_new.normalize();
    
    // Check quaternion norm after normalization (should be close to 1)
    Scalar norm_error = std::abs(q_new.norm() - Scalar(1.0));
    if (norm_error > Scalar(1e-3)) {
        flags.set(FailureReason::BadNorm);
    }
    
    // Check for excessive error (quaternion components should be reasonable)
    if (q_new.coeffs().array().abs().maxCoeff() > Scalar(10.0)) {
        flags.set(FailureReason::ExcessiveErr);
    }

    return q_new;
}

template <typename Scalar>
Eigen::Quaternion<Scalar> mahoney_update_marg_checked(
    const Eigen::Quaternion<Scalar>& q,
    const Eigen::Matrix<Scalar, 3, 1>& gyr,
    const Eigen::Matrix<Scalar, 3, 1>& acc,
    const Eigen::Matrix<Scalar, 3, 1>& mag,
    Scalar dt,
    Scalar k_p,
    Scalar k_i,
    EntoMath::Vec3<Scalar>& bias,
    FailureFlags& flags)
{
    flags.reset();
    
    // Check for overflow/saturation in inputs
    if (!std::isfinite(gyr.norm()) || !std::isfinite(acc.norm()) || 
        !std::isfinite(mag.norm()) || !std::isfinite(dt) || !std::isfinite(k_p) || !std::isfinite(k_i)) {
        flags.set(FailureReason::Overflow);
        return q;
    }
    
    if (gyr.norm() == Scalar(0))
        return q;

    Eigen::Quaternion<Scalar> q_new = q;
    Eigen::Matrix<Scalar, 3, 1> omega = gyr;
    const Scalar a_norm = acc.norm();

    if (a_norm > Scalar(0))
    {
        const Scalar m_norm = mag.norm();
        // If magnetometer data is not available, fall back to the IMU update.
        if (m_norm == Scalar(0))
            return mahoney_update_imu_checked(q, gyr, acc, dt, k_p, k_i, bias, flags);

        // Check for near-zero divisors
        if (a_norm < std::numeric_limits<Scalar>::epsilon() * 1000 ||
            m_norm < std::numeric_limits<Scalar>::epsilon() * 1000) {
            flags.set(FailureReason::NearZeroDiv);
        }

        // Normalize the accelerometer and magnetometer measurements.
        const Eigen::Matrix<Scalar, 3, 1> a_normalized = acc / a_norm;
        const Eigen::Matrix<Scalar, 3, 1> m_normalized = mag / m_norm;

        const Eigen::Matrix<Scalar, 3, 3> R = q_new.toRotationMatrix();
        
        // Check rotation matrix validity
        if (!std::isfinite(R.norm())) {
            flags.set(FailureReason::Overflow);
            return q;
        }
        
        // Expected gravity vector in the body frame.
        const Eigen::Matrix<Scalar, 3, 1> v_a = R.transpose() *
            Eigen::Matrix<Scalar, 3, 1>(Scalar(0), Scalar(0), Scalar(1));

        // Rotate the normalized magnetometer reading into the inertial frame.
        const Eigen::Matrix<Scalar, 3, 1> h = R * m_normalized;
        
        // Check h for overflow
        if (!std::isfinite(h.norm())) {
            flags.set(FailureReason::Overflow);
            return q;
        }
        
        // Compute the horizontal (xy-plane) norm of h.
        const Scalar h_xy2 = h.x() * h.x() + h.y() * h.y();
        const Scalar h_xy_norm = (h_xy2 == Scalar(0)) ? Scalar(0) : std::sqrt(h_xy2);

        // Check for near-zero h_xy_norm
        if (h_xy_norm < std::numeric_limits<Scalar>::epsilon() * 1000) {
            flags.set(FailureReason::NearZeroDiv);
        }

        // Construct an artificial vector for magnetometer correction.
        Eigen::Matrix<Scalar, 3, 1> v_m = R.transpose() *
            Eigen::Matrix<Scalar, 3, 1>(Scalar(0), h_xy_norm, h.z());

        // Normalize v_m - handle potential division by zero
        const Scalar v_m_norm = v_m.norm();
        if (v_m_norm < std::numeric_limits<Scalar>::epsilon() * 1000) {
            flags.set(FailureReason::NearZeroDiv);
        }
        v_m /= v_m_norm;

        // Innovation terms
        const Eigen::Matrix<Scalar, 3, 1> omega_mes_a = a_normalized.cross(v_a);
        const Eigen::Matrix<Scalar, 3, 1> omega_mes_m = m_normalized.cross(v_m);
        
        // Check for overflow in innovation terms
        if (!std::isfinite(omega_mes_a.norm()) || !std::isfinite(omega_mes_m.norm())) {
            flags.set(FailureReason::Overflow);
            return q;
        }

        const Eigen::Matrix<Scalar, 3, 1> omega_mes = omega_mes_a + omega_mes_m;

        // Compute the derivative of the bias (integral term).
        const Eigen::Matrix<Scalar, 3, 1> b_dot = -k_i * omega_mes;

        // Update the bias.
        bias += b_dot * dt;
        
        // Check bias for overflow
        if (!std::isfinite(bias.norm())) {
            flags.set(FailureReason::Overflow);
        }

        // Correct the measured angular velocity.
        omega = omega - bias + k_p * omega_mes;
        
        // Check corrected omega for overflow
        if (!std::isfinite(omega.norm())) {
            flags.set(FailureReason::Overflow);
        }
    }

    // Create a pure quaternion from the angular velocity: (0, omega).
    const Eigen::Quaternion<Scalar> p(Scalar(0), omega.x(), omega.y(), omega.z());

    // Compute the quaternion derivative: q_dot = 0.5 * q * p.
    Eigen::Quaternion<Scalar> q_dot = q_new * p;
    q_dot.coeffs() *= Scalar(0.5);
    
    // Check q_dot for overflow
    if (!std::isfinite(q_dot.norm())) {
        flags.set(FailureReason::Overflow);
        return q;
    }

    // Integrate to get the new quaternion.
    q_new.coeffs() += q_dot.coeffs() * dt;
    
    // Check pre-normalization quaternion
    if (!std::isfinite(q_new.norm())) {
        flags.set(FailureReason::Overflow);
        return q;
    }
    
    // Check if quaternion norm is reasonable before normalization
    Scalar norm_before = q_new.norm();
    if (norm_before < Scalar(1e-6)) {
        flags.set(FailureReason::NearZeroDiv);
        return q;
    }
    
    q_new.normalize();
    
    // Check quaternion norm after normalization (should be close to 1)
    Scalar norm_error = std::abs(q_new.norm() - Scalar(1.0));
    if (norm_error > Scalar(1e-3)) {
        flags.set(FailureReason::BadNorm);
    }
    
    // Check for excessive error (quaternion components should be reasonable)
    if (q_new.coeffs().array().abs().maxCoeff() > Scalar(10.0)) {
        flags.set(FailureReason::ExcessiveErr);
    }

    return q_new;
}

// Template wrapper that matches the expected interface
template <typename Scalar, bool UseMag>
struct FilterMahoneyChecked
{
    // Internal state - set by AttitudeProblem constructor
    Scalar kp_ = Scalar(0.1);  // Default parameters
    Scalar ki_ = Scalar(0.01);
    EntoMath::Vec3<Scalar> bias_ = EntoMath::Vec3<Scalar>::Zero();
    
    static constexpr bool UseMag_ = UseMag;

    // Pointer to failure flags - set by external code before each call
    FailureFlags* failure_flags_ = nullptr;
    
    FilterMahoneyChecked() = default;
    
    // Set the failure flags pointer (SAME AS MADGWICK)
    void setFailureFlags(FailureFlags* flags) {
        failure_flags_ = flags;
    }
    
    // Set parameters (called by AttitudeProblem constructor)
    void setParameters(Scalar kp, Scalar ki) {
        kp_ = kp;
        ki_ = ki;
    }

    // 4-parameter interface for AttitudeProblem (SAME AS MADGWICK)
    inline void
    operator()(const Eigen::Quaternion<Scalar>& q,
               const EntoAttitude::AttitudeMeasurement<Scalar, UseMag>& meas,
               Scalar dt,
               Eigen::Quaternion<Scalar>* q_out)
    {
        FailureFlags local_flags;
        if (!failure_flags_) {
            failure_flags_ = &local_flags;
        }
        
        if constexpr (UseMag) {
            *q_out = mahoney_update_marg_checked(q, meas.gyr, meas.acc, meas.mag, dt, kp_, ki_, bias_, *failure_flags_);
        } else {
            *q_out = mahoney_update_imu_checked(q, meas.gyr, meas.acc, dt, kp_, ki_, bias_, *failure_flags_);
        }
    }

    // 7-parameter interface for manual calls (kept for compatibility)
    inline Eigen::Quaternion<Scalar>
    operator()(const Eigen::Quaternion<Scalar>& q,
               const EntoAttitude::AttitudeMeasurement<Scalar, UseMag>& meas,
               Scalar dt,
               Scalar k_p,
               Scalar k_i,
               EntoMath::Vec3<Scalar>& bias,
               FailureFlags& flags)
    {
        if constexpr (UseMag) {
            return mahoney_update_marg_checked(q, meas.gyr, meas.acc, meas.mag, dt, k_p, k_i, bias, flags);
        } else {
            return mahoney_update_imu_checked(q, meas.gyr, meas.acc, dt, k_p, k_i, bias, flags);
        }
    }

    static constexpr const char* name()
    {
        return UseMag ? "Mahoney MARG Float (Checked)" : "Mahoney IMU Float (Checked)";
    }
};

} // namespace EntoAttitudeExp

#endif // MAHONEY_CHECKED_H 