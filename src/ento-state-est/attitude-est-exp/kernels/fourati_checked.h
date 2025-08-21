#ifndef FOURATI_CHECKED_H
#define FOURATI_CHECKED_H

#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <ento-state-est/attitude-est-exp/core/failure_flags.h>

namespace EntoAttitudeExp {

template <typename Scalar>
Eigen::Quaternion<Scalar> fourati_update_checked(
    const Eigen::Quaternion<Scalar>& q,
    const EntoMath::Vec3<Scalar>& gyr,
    const EntoMath::Vec3<Scalar>& acc,
    const EntoMath::Vec3<Scalar>& mag,
    Scalar dt,
    Scalar gain,
    const Eigen::Quaternion<Scalar>& g_q,
    const Eigen::Quaternion<Scalar>& m_q,
    FailureFlags& flags)
{
    flags.reset();
    
    // Check for overflow/saturation in inputs
    if (!std::isfinite(gyr.norm()) || !std::isfinite(acc.norm()) || 
        !std::isfinite(mag.norm()) || !std::isfinite(dt) || !std::isfinite(gain) ||
        !std::isfinite(g_q.norm()) || !std::isfinite(m_q.norm())) {
        flags.set(FailureReason::Overflow);
        return q;
    }
    
    // If the gyroscope measurement is zero, return the current orientation.
    if (gyr.norm() == Scalar(0))
        return q;

    // Local copy of current quaternion.
    Eigen::Quaternion<Scalar> q_new = q;

    // Compute the quaternion derivative from gyroscope (using pure quaternion for ω).
    Eigen::Quaternion<Scalar> omega(Scalar(0), gyr.x(), gyr.y(), gyr.z());
    Eigen::Quaternion<Scalar> qDot = (q_new * omega);
    qDot.coeffs() *= Scalar(0.5);
    
    // Check qDot for overflow
    if (!std::isfinite(qDot.norm())) {
        flags.set(FailureReason::Overflow);
        return q;
    }

    // If gain > 0, apply a LM‐style correction step.
    if (gain > Scalar(0))
    {
        // Check for near-zero norms (early exit conditions)
        Scalar acc_norm = acc.norm();
        Scalar mag_norm = mag.norm();
        
        if (acc_norm < std::numeric_limits<Scalar>::epsilon() * 1000) {
            flags.set(FailureReason::NearZeroDiv);
            return q;
        }
        
        if (mag_norm < std::numeric_limits<Scalar>::epsilon() * 1000) {
            flags.set(FailureReason::NearZeroDiv);
            return q;
        }

        // If both norms are valid, proceed with correction
        if (acc_norm > Scalar(0) && mag_norm > Scalar(0))
        {
            // Quaternion representations of expected directions in the world frame.
            Eigen::Quaternion<Scalar> expected_g_w = g_q;   // Expected gravity in world frame
            Eigen::Quaternion<Scalar> expected_m_w = m_q;   // Expected magnetic field in world frame

            // Rotate expected directions to the body frame using the current quaternion estimate.
            Eigen::Quaternion<Scalar> q_inv = q_new.conjugate();
            Eigen::Quaternion<Scalar> expected_g_b = q_new * expected_g_w * q_inv;
            Eigen::Quaternion<Scalar> expected_m_b = q_new * expected_m_w * q_inv;

            // Check for overflow in expected directions
            if (!std::isfinite(expected_g_b.norm()) || !std::isfinite(expected_m_b.norm())) {
                flags.set(FailureReason::Overflow);
                return q;
            }

            // Normalize the measurements.
            EntoMath::Vec3<Scalar> acc_normalized = acc / acc_norm;
            EntoMath::Vec3<Scalar> mag_normalized = mag / mag_norm;
            
            // Check normalized measurements for overflow
            if (!std::isfinite(acc_normalized.norm()) || !std::isfinite(mag_normalized.norm())) {
                flags.set(FailureReason::Overflow);
                return q;
            }

            // Build the error vector e = [e_g; e_m] where:
            // e_g = a_normalized x expected_g_b.vec()
            // e_m = m_normalized x expected_m_b.vec()
            
            EntoMath::Vec3<Scalar> expected_g_vec = expected_g_b.vec();
            EntoMath::Vec3<Scalar> expected_m_vec = expected_m_b.vec();
            
            EntoMath::Vec3<Scalar> e_g = acc_normalized.cross(expected_g_vec);
            EntoMath::Vec3<Scalar> e_m = mag_normalized.cross(expected_m_vec);

            // Check error vectors for overflow
            if (!std::isfinite(e_g.norm()) || !std::isfinite(e_m.norm())) {
                flags.set(FailureReason::Overflow);
                return q;
            }

            // Combine into 6D error vector
            Eigen::Matrix<Scalar, 6, 1> e;
            e.segment(0, 3) = e_g;
            e.segment(3, 3) = e_m;
            
            // Check combined error vector
            if (!std::isfinite(e.norm())) {
                flags.set(FailureReason::Overflow);
                return q;
            }

            // Build the Jacobian J (6x3) for the LM step
            // This is a simplified approximation - for full Fourati algorithm, 
            // would need proper Jacobian computation
            
            Eigen::Matrix<Scalar, 6, 3> J = Eigen::Matrix<Scalar, 6, 3>::Identity();
            J.block(0, 0, 3, 3) = Eigen::Matrix<Scalar, 3, 3>::Identity();
            J.block(3, 0, 3, 3) = Eigen::Matrix<Scalar, 3, 3>::Identity();
            
            // Check Jacobian for overflow
            if (!std::isfinite(J.norm())) {
                flags.set(FailureReason::Overflow);
                return q;
            }

            // LM step: solve (J^T J + λI) Δq = -J^T e
            Eigen::Matrix<Scalar, 3, 3> JtJ = J.transpose() * J;
            Eigen::Matrix<Scalar, 3, 1> Jte = J.transpose() * e;
            
            // Check intermediate calculations
            if (!std::isfinite(JtJ.norm()) || !std::isfinite(Jte.norm())) {
                flags.set(FailureReason::Overflow);
                return q;
            }

            // Damping factor for LM
            Scalar lambda = gain;
            Eigen::Matrix<Scalar, 3, 3> A = JtJ + lambda * Eigen::Matrix<Scalar, 3, 3>::Identity();
            
            // Check if determinant is near zero (singular matrix)
            if (A.determinant() < std::numeric_limits<Scalar>::epsilon() * 1000) {
                flags.set(FailureReason::NearZeroDiv);
                return q;
            }
            
            // Solve for correction
            Eigen::Matrix<Scalar, 3, 1> delta_q = A.ldlt().solve(-Jte);
            
            // Check solution for overflow
            if (!std::isfinite(delta_q.norm())) {
                flags.set(FailureReason::Overflow);
                return q;
            }

                         // Apply correction to quaternion derivative
             Eigen::Quaternion<Scalar> delta_quat(Scalar(0), delta_q.x(), delta_q.y(), delta_q.z());
             Eigen::Quaternion<Scalar> correction = q_new * delta_quat;
             correction.coeffs() *= Scalar(0.5);
             qDot.coeffs() += correction.coeffs();
            
            // Check final qDot
            if (!std::isfinite(qDot.norm())) {
                flags.set(FailureReason::Overflow);
                return q;
            }
        }
    }

    // Integrate to update the quaternion.
    q_new.coeffs() += qDot.coeffs() * dt;
    
    // Check integrated quaternion
    if (!std::isfinite(q_new.norm())) {
        flags.set(FailureReason::Overflow);
        return q;
    }

    // Check quaternion norm after integration (should be close to 1 after normalization)
    Scalar norm_before = q_new.norm();
    if (norm_before < Scalar(1e-6)) {
        flags.set(FailureReason::NearZeroDiv);
        return q;
    }
    
    // Normalize the quaternion.
    q_new.normalize();
    
    // Check quaternion norm deviation after normalization
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
struct FilterFouratiChecked {
    FailureFlags* failure_flags_ = nullptr;
    
    FilterFouratiChecked() = default;
    
    void setFailureFlags(FailureFlags* flags) { failure_flags_ = flags; }
    
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
        return fourati_update_checked(q, meas.gyr, meas.acc, meas.mag, dt, gain, g_q, m_q, *flags);
    }
    
    static constexpr const char* name() {
        return "Fourati MARG Checked";
    }
};

} // namespace EntoAttitudeExp

#endif // FOURATI_CHECKED_H 