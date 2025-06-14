#ifndef FOURATI_FIXED_H
#define FOURATI_FIXED_H

#include <Eigen/Dense>
#include <cmath>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <math/FixedPoint.hh>
#include <math/FixedPointMath.hh>
#include <math/EigenFixedPoint.hh>
#include <ento-util/debug.h>

// Include mahoney_fixed.h to reuse the helper functions
#include <ento-state-est/attitude-est/mahoney_fixed.h>

#ifdef USE_CMSIS_INV_SQRT
#include "arm_math.h"
#endif

namespace EntoAttitude
{

// ---------------------------------------------------------------------------
// 1. Q-format types from the Büscher paper (reuse from mahoney_fixed.h)
// ---------------------------------------------------------------------------
// These are already defined in mahoney_fixed.h, so we don't need to redefine them

// ---------------------------------------------------------------------------
// 2. Core Fourati update — MARG variant (gyro + accel + mag)
// ---------------------------------------------------------------------------
template<typename S>
Eigen::Quaternion<S> fourati_update_fixed(
    const Eigen::Quaternion<S>& q,
    const Eigen::Matrix<S,3,1>& gyr,
    const Eigen::Matrix<S,3,1>& acc,
    const Eigen::Matrix<S,3,1>& mag,
    S dt,
    S gain,
    const Eigen::Quaternion<S>& g_q,
    const Eigen::Quaternion<S>& m_q)
{
  // If the gyroscope measurement is zero, return the current orientation.
  if (gyr.squaredNorm() == S(0))
    return q;

  // Local copy of current quaternion.
  Eigen::Quaternion<S> q_new = q;

  // Compute the quaternion derivative from gyroscope (using pure quaternion for ω).
  Eigen::Quaternion<S> omega(S(0), gyr.x(), gyr.y(), gyr.z());
  Eigen::Quaternion<S> qDot = (q_new * omega);
  qDot.coeffs() *= S(0.5);

  // If gain > 0, apply a LM-style correction step.
  if (gain > S(0))
  {
    S a_norm2 = acc.squaredNorm();
    if (a_norm2 == S(0))
        return q_new; // or simply skip correction if no accelerometer data

    S m_norm2 = mag.squaredNorm();
    if (m_norm2 == S(0))
        return q_new; // or skip correction if no magnetometer data

    // Normalize the accelerometer and magnetometer measurements using the shared helper
    Eigen::Matrix<S, 3, 1> a_normalized = vec_normalise(acc);
    Eigen::Matrix<S, 3, 1> m_normalized = vec_normalise(mag);

    // Compute estimated measurements:
    // fhat = q_new.conjugate() * (g_q * q_new)
    // hhat = q_new.conjugate() * (m_q * q_new)
    Eigen::Quaternion<S> fhat = q_new.conjugate() * (g_q * q_new);
    Eigen::Quaternion<S> hhat = q_new.conjugate() * (m_q * q_new);

    // Extract the vector parts (x, y, z) of fhat and hhat.
    Eigen::Matrix<S, 3, 1> fhat_vec(fhat.x(), fhat.y(), fhat.z());
    Eigen::Matrix<S, 3, 1> hhat_vec(hhat.x(), hhat.y(), hhat.z());

    // Form the measurement vector y (6×1) from normalized accelerometer and magnetometer.
    Eigen::Matrix<S, 6, 1> y;
    y.template segment<3>(0) = a_normalized;
    y.template segment<3>(3) = m_normalized;

    // Form the estimated measurement vector yhat (6×1) from the vector parts.
    Eigen::Matrix<S, 6, 1> yhat;
    yhat.template segment<3>(0) = fhat_vec;
    yhat.template segment<3>(3) = hhat_vec;

    // Modeling error: dq = y - yhat.
    Eigen::Matrix<S, 6, 1> dq = y - yhat;

    // Compute the Jacobian matrix X as in equation (23):
    // X = -2 * [ skew(fhat_vec), skew(hhat_vec) ]^T.
    // First, form a 3×6 matrix A = [ skew(fhat_vec)  |  skew(hhat_vec) ].
    Eigen::Matrix<S, 3, 3> skew_f = EntoMath::skew(fhat_vec);
    Eigen::Matrix<S, 3, 3> skew_h = EntoMath::skew(hhat_vec);
    Eigen::Matrix<S, 3, 6> A;
    A.template block<3,3>(0,0) = skew_f;
    A.template block<3,3>(0,3) = skew_h;
    // Then, X = -2 * A^T gives a 6×3 matrix.
    Eigen::Matrix<S, 6, 3> X = S(-2) * A.transpose();

    // Damping factor (λ) to guarantee inversion.
    S lam = S(1e-5);

    // Compute the gain matrix K as in LM update:
    // K = gain * [ (X^T X + λ I)^{-1} X^T ].
    Eigen::Matrix<S, 3, 3> temp = X.transpose() * X + lam * Eigen::Matrix<S, 3, 3>::Identity();
    Eigen::Matrix<S, 3, 6> K = gain * temp.inverse() * X.transpose();

    // Correction vector eta (3×1): eta = K * dq.
    Eigen::Matrix<S, 3, 1> eta = K * dq;

    // Form Delta as a quaternion with scalar part 1 and vector part eta.
    Eigen::Quaternion<S> Delta(S(1), eta(0), eta(1), eta(2));
    // Correct the quaternion derivative.
    qDot = qDot * Delta;
  }

  // Integrate the corrected quaternion derivative:
  // q_updated = q_new + qDot * dt, then normalize.
  Eigen::Matrix<S, 4, 1> q_vec;
  q_vec(0) = q_new.w(); q_vec(1) = q_new.x(); q_vec(2) = q_new.y(); q_vec(3) = q_new.z();
  Eigen::Matrix<S, 4, 1> qDot_vec;
  qDot_vec(0) = qDot.w(); qDot_vec(1) = qDot.x(); qDot_vec(2) = qDot.y(); qDot_vec(3) = qDot.z();
  Eigen::Matrix<S, 4, 1> q_updated_vec = q_vec + qDot_vec * dt;
  
  // Normalize using the shared helper
  q_updated_vec = vec_normalise(q_updated_vec);

  return Eigen::Quaternion<S>(q_updated_vec(0), q_updated_vec(1), q_updated_vec(2), q_updated_vec(3));
}

// ---------------------------------------------------------------------------
// 3. Compile-time dispatcher (matching existing API)
// ---------------------------------------------------------------------------
template<typename S>
inline Eigen::Quaternion<S> fourati_fixed(const Eigen::Quaternion<S> &q,
                                          const MARGMeasurement<S> &meas,
                                          S dt,
                                          S gain,
                                          const Eigen::Quaternion<S>& g_q,
                                          const Eigen::Quaternion<S>& m_q)
{
  return fourati_update_fixed(q, meas.gyr, meas.acc, meas.mag, dt, gain, g_q, m_q);
}

// ---------------------------------------------------------------------------
// 4. Functor wrapper (compatible with existing interface)
// ---------------------------------------------------------------------------
template<typename S>
struct FilterFouratiFixed
{
  // Default constructor
  FilterFouratiFixed() = default;
  
  // Compatible with existing interface - match FilterFourati signature
  inline Eigen::Quaternion<S> 
  operator()(const Eigen::Quaternion<S>& q,
             const MARGMeasurement<S>& meas,
             S dt,
             S gain,
             const Eigen::Quaternion<S>& g_q,
             const Eigen::Quaternion<S>& m_q)
  {
    return fourati_update_fixed(q, meas.gyr, meas.acc, meas.mag, dt, gain, g_q, m_q);
  }

  static constexpr const char *name()
  {
    return "Fourati MARG Fixed (Q-fxp)";
  }
};

// ---------------------------------------------------------------------------
// 5. Convenience type aliases for the paper's Q-formats
// ---------------------------------------------------------------------------
using FilterFouratiQ7_24 = FilterFouratiFixed<Q7_24>;
using FilterFouratiQ3_12 = FilterFouratiFixed<Q3_12>;
using FilterFouratiQ5_26 = FilterFouratiFixed<Q5_26>;

} // namespace EntoAttitude

#endif // FOURATI_FIXED_H 