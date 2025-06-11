#ifndef MADGWICK_FIXED_H
#define MADGWICK_FIXED_H

#include <Eigen/Dense>
#include <cmath>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <math/FixedPoint.hh>
#include <math/FixedPointMath.hh>
#include <math/EigenFixedPoint.hh>
#include <ento-util/debug.h>

#ifdef USE_CMSIS_INV_SQRT
#include "arm_math.h"
#endif

namespace EntoAttitude
{

// ---------------------------------------------------------------------------
// 1. Q-format types from the Büscher paper
// ---------------------------------------------------------------------------
using Q7_24 = FixedPoint<7, 24, int32_t>;   // 32-bit path from paper
using Q3_12 = FixedPoint<3, 12, int16_t>;   // 16-bit path from paper
using Q6_25 = FixedPoint<6, 25, int32_t>;   // 32-bit with more fractional precision
using Q5_26 = FixedPoint<5, 26, int32_t>;   // 32-bit with even more fractional precision
using Q2_13 = FixedPoint<2, 13, int16_t>;   // 16-bit with higher precision than Q3.12

} // namespace EntoAttitude

namespace EntoAttitude
{

// ---------------------------------------------------------------------------
// 3. Core Madgwick update — IMU variant (gyro + accel)
// ---------------------------------------------------------------------------
template<typename S>
Eigen::Quaternion<S> madgwick_update_imu_fixed(
    const Eigen::Quaternion<S>& q,
    const Eigen::Matrix<S,3,1>& gyr,
    const Eigen::Matrix<S,3,1>& acc,
    S dt,    // Changed to Scalar type
    S gain)
{
  // If no rotation is measured, return the current orientation.
  if (gyr.norm() == S(0.0f))
    return q;

  // Compute quaternion derivative from gyroscope measurements.
  Eigen::Quaternion<S> omega(S(0.0f), gyr.x(), gyr.y(), gyr.z());
  Eigen::Quaternion<S> q_dot = (q * omega);
  q_dot.coeffs() *= S(0.5f);

  // Only if accelerometer measurement is valid:
  S a_norm = acc.norm();
  if (a_norm > S(0.0f))
  {
    // Normalize accelerometer reading.
    const Eigen::Matrix<S,3,1> a = acc / a_norm;
    // Normalize the current orientation.
    const Eigen::Quaternion<S> q_norm = q.normalized();
    // Extract quaternion components in (w, x, y, z) order.
    const S qw = q_norm.w();
    const S qx = q_norm.x();
    const S qy = q_norm.y();
    const S qz = q_norm.z();

    // Compute objective function f (a 3-vector) per Madgwick's eq. (25):
    Eigen::Matrix<S, 3, 1> f;
    f.data()[0] = S(2.0f) * (qx * qz - qw * qy) - a.data()[0];
    f.data()[1] = S(2.0f) * (qw * qx + qy * qz) - a.data()[1];
    f.data()[2] = S(2.0f) * (S(0.5f) - qx * qx - qy * qy) - a.data()[2];

    // If f is nonzero, compute the Jacobian J (3x4) and its gradient.
    if (f.norm() > S(0.0f))
    {
      // Build J, with columns in order: [qw, qx, qy, qz]
      Eigen::Matrix<S, 3, 4> J;
      J.data()[0*4+0] = -S(2.0f)*qy;   J.data()[0*4+1] =  S(2.0f)*qz;   J.data()[0*4+2] = -S(2.0f)*qw;   J.data()[0*4+3] =  S(2.0f)*qx;
      J.data()[1*4+0] =  S(2.0f)*qx;   J.data()[1*4+1] =  S(2.0f)*qw;   J.data()[1*4+2] =  S(2.0f)*qz;   J.data()[1*4+3] =  S(2.0f)*qy;
      J.data()[2*4+0] = S(0.0f);       J.data()[2*4+1] = -S(4.0f)*qx;   J.data()[2*4+2] = -S(4.0f)*qy;   J.data()[2*4+3] = S(0.0f);

      // Compute the gradient (a 4-vector) = J^T * f.
      Eigen::Matrix<S, 4, 1> gradient = J.transpose() * f;
      // Normalize the gradient.
      gradient.normalize();

      // Represent q_dot as a 4-vector in (w, x, y, z) order.
      Eigen::Matrix<S, 4, 1> q_dot_vec;
      q_dot_vec.data()[0] = q_dot.w(); 
      q_dot_vec.data()[1] = q_dot.x(); 
      q_dot_vec.data()[2] = q_dot.y(); 
      q_dot_vec.data()[3] = q_dot.z();
      // Subtract the correction term scaled by the gain.
      q_dot_vec -= gain * gradient;
      // Rebuild q_dot.
      q_dot = Eigen::Quaternion<S>(q_dot_vec.data()[0], q_dot_vec.data()[1], q_dot_vec.data()[2], q_dot_vec.data()[3]);
    }
  }

  // Integrate: q_new = q + q_dot * dt
  // Use fixed-point arithmetic throughout
  Eigen::Matrix<S, 4, 1> q_vec;
  q_vec.data()[0] = q.w(); 
  q_vec.data()[1] = q.x(); 
  q_vec.data()[2] = q.y(); 
  q_vec.data()[3] = q.z();
  
  // Fixed-point integration
  Eigen::Matrix<S, 4, 1> q_new_vec;
  q_new_vec.data()[0] = q_vec.data()[0] + q_dot.w() * dt;
  q_new_vec.data()[1] = q_vec.data()[1] + q_dot.x() * dt;
  q_new_vec.data()[2] = q_vec.data()[2] + q_dot.y() * dt;
  q_new_vec.data()[3] = q_vec.data()[3] + q_dot.z() * dt;
  
  // Normalize the updated quaternion.
  q_new_vec = q_new_vec / q_new_vec.norm();
  Eigen::Quaternion<S> q_new(q_new_vec.data()[0], q_new_vec.data()[1], q_new_vec.data()[2], q_new_vec.data()[3]);
  return q_new;
}

// ---------------------------------------------------------------------------
// 4. Core Madgwick update — MARG variant (gyro + accel + mag)
// ---------------------------------------------------------------------------
template<typename S>
Eigen::Quaternion<S> madgwick_update_marg_fixed(
    const Eigen::Quaternion<S>& q,
    const Eigen::Matrix<S,3,1>& gyr,
    const Eigen::Matrix<S,3,1>& acc,
    const Eigen::Matrix<S,3,1>& mag,
    S dt,    // Changed to Scalar type
    S gain)
{
  // If no rotation is measured, return the current orientation.
  if (gyr.norm() == S(0.0f))
    return q;
  // If magnetometer data is missing, fall back to the IMU update.
  if (mag.norm() == S(0.0f))
    return madgwick_update_imu_fixed(q, gyr, acc, dt, gain);

  // Compute quaternion derivative from gyroscope measurements.
  Eigen::Quaternion<S> omega(S(0.0f), gyr.x(), gyr.y(), gyr.z());
  Eigen::Quaternion<S> q_dot = (q * omega);
  q_dot.coeffs() *= S(0.5f);

  S a_norm = acc.norm();
  if (a_norm > S(0.0f))
  {
    // Normalize accelerometer and magnetometer measurements.
    const Eigen::Matrix<S,3,1> a = acc / a_norm;
    const Eigen::Matrix<S,3,1> m = mag / mag.norm();

    // Rotate the magnetometer measurement into the Earth frame.
    Eigen::Quaternion<S> m_quat(S(0.0f), m.x(), m.y(), m.z());
    Eigen::Quaternion<S> h = q * m_quat * q.conjugate();
    // Let bx = norm( h_x, h_y ) and bz = h_z.
    const S bx = sqrt(h.x() * h.x() + h.y() * h.y());
    const S bz = h.z();

    // Normalize the current orientation.
    const Eigen::Quaternion<S> q_norm = q.normalized();
    const S qw = q_norm.w();
    const S qx = q_norm.x();
    const S qy = q_norm.y();
    const S qz = q_norm.z();

    // Compute the objective function f (a 6-vector)
    Eigen::Matrix<S, 6, 1> f;
    f.data()[0] = S(2.0f) * (qx * qz - qw * qy) - a.data()[0];
    f.data()[1] = S(2.0f) * (qw * qx + qy * qz) - a.data()[1];
    f.data()[2] = S(2.0f) * (S(0.5f) - qx * qx - qy * qy) - a.data()[2];
    f.data()[3] = S(2.0f) * bx * (S(0.5f) - qy * qy - qz * qz) + S(2.0f) * bz * (qx * qz - qw * qy) - m.data()[0];
    f.data()[4] = S(2.0f) * bx * (qx * qy - qw * qz) + S(2.0f) * bz * (qw * qx + qy * qz) - m.data()[1];
    f.data()[5] = S(2.0f) * bx * (qw * qy + qx * qz) + S(2.0f) * bz * (S(0.5f) - qx * qx - qy * qy) - m.data()[2];

    if (f.norm() > S(0.0f))
    {
      // Build the Jacobian J (6x4)
      Eigen::Matrix<S, 6, 4> J;

      // First three rows (as in the IMU case):
      J.data()[0*4+0] = -S(2.0f)*qy;  J.data()[0*4+1] =  S(2.0f)*qz;  J.data()[0*4+2] = -S(2.0f)*qw;  J.data()[0*4+3] =  S(2.0f)*qx;
      J.data()[1*4+0] =  S(2.0f)*qx;  J.data()[1*4+1] =  S(2.0f)*qw;  J.data()[1*4+2] =  S(2.0f)*qz;  J.data()[1*4+3] =  S(2.0f)*qy;
      J.data()[2*4+0] = S(0.0f);      J.data()[2*4+1] = -S(4.0f)*qx;  J.data()[2*4+2] = -S(4.0f)*qy;  J.data()[2*4+3] = S(0.0f);

      // Next three rows (for the magnetometer):
      J.data()[3*4+0] = -S(2.0f)*bz*qy;
      J.data()[3*4+1] =  S(2.0f)*bz*qz; 
      J.data()[3*4+2] = -S(4.0f)*bx*qy - S(2.0f)*bz*qw;
      J.data()[3*4+3] = -S(4.0f)*bx*qz + S(2.0f)*bz*qx;
      
      J.data()[4*4+0] = -S(2.0f)*bx*qz + S(2.0f)*bz*qx;
      J.data()[4*4+1] =  S(2.0f)*bx*qy + S(2.0f)*bz*qw;
      J.data()[4*4+2] =  S(2.0f)*bx*qx + S(2.0f)*bz*qz;
      J.data()[4*4+3] = -S(2.0f)*bx*qw + S(2.0f)*bz*qy;
      
      J.data()[5*4+0] =  S(2.0f)*bx*qy;
      J.data()[5*4+1] =  S(2.0f)*bx*qz - S(4.0f)*bz*qx;
      J.data()[5*4+2] =  S(2.0f)*bx*qw - S(4.0f)*bz*qy;
      J.data()[5*4+3] =  S(2.0f)*bx*qx;

      // Compute the gradient (4-vector) = J^T * f.
      Eigen::Matrix<S, 4, 1> gradient = J.transpose() * f;
      gradient.normalize();

      // Represent q_dot as a 4-vector (w, x, y, z).
      Eigen::Matrix<S, 4, 1> q_dot_vec;
      q_dot_vec.data()[0] = q_dot.w(); 
      q_dot_vec.data()[1] = q_dot.x(); 
      q_dot_vec.data()[2] = q_dot.y(); 
      q_dot_vec.data()[3] = q_dot.z();
      // Subtract the correction term.
      q_dot_vec -= gain * gradient;
      q_dot = Eigen::Quaternion<S>(q_dot_vec.data()[0], q_dot_vec.data()[1], q_dot_vec.data()[2], q_dot_vec.data()[3]);
    }
  }

  // Integrate: q_new = q + q_dot * dt
  // Use fixed-point arithmetic throughout
  Eigen::Matrix<S, 4, 1> q_vec;
  q_vec.data()[0] = q.w(); 
  q_vec.data()[1] = q.x(); 
  q_vec.data()[2] = q.y(); 
  q_vec.data()[3] = q.z();
  
  // Fixed-point integration
  Eigen::Matrix<S, 4, 1> q_new_vec;
  q_new_vec.data()[0] = q_vec.data()[0] + q_dot.w() * dt;
  q_new_vec.data()[1] = q_vec.data()[1] + q_dot.x() * dt;
  q_new_vec.data()[2] = q_vec.data()[2] + q_dot.y() * dt;
  q_new_vec.data()[3] = q_vec.data()[3] + q_dot.z() * dt;
  
  q_new_vec.normalize();
  Eigen::Quaternion<S> q_new(q_new_vec.data()[0], q_new_vec.data()[1], q_new_vec.data()[2], q_new_vec.data()[3]);
  return q_new;
}

// ---------------------------------------------------------------------------
// 5. Compile-time dispatcher (matching existing API)
// ---------------------------------------------------------------------------
template<typename S,bool UseMag>
inline Eigen::Quaternion<S> madgwick_fixed(const Eigen::Quaternion<S> &q,
                                           const AttitudeMeasurement<S,UseMag> &meas,
                                           S dt,    // Changed to Scalar type
                                           S gain)
{
  if constexpr (UseMag)
    return madgwick_update_marg_fixed(q, meas.gyr, meas.acc, meas.mag, dt, gain);
  else
    return madgwick_update_imu_fixed(q, meas.gyr, meas.acc, dt, gain);
}

// ---------------------------------------------------------------------------
// 6. Functor wrapper (compatible with existing interface)
// ---------------------------------------------------------------------------
template<typename S,bool UseMag>
struct FilterMadgwickFixed
{
  // Default constructor
  FilterMadgwickFixed() = default;
  
  // Compatible with existing interface - match FilterMadgwick signature
  inline Eigen::Quaternion<S> 
  operator()(const Eigen::Quaternion<S>& q,
             const AttitudeMeasurement<S, UseMag>& meas,
             S dt,    // Changed to Scalar type
             S gain)
  {
    if constexpr (UseMag)
      return madgwick_update_marg_fixed(q, meas.gyr, meas.acc, meas.mag, dt, gain);
    else
      return madgwick_update_imu_fixed(q, meas.gyr, meas.acc, dt, gain);
  }

  static constexpr const char *name()
  {
    if constexpr (UseMag)
      return "Madgwick MARG Fixed (Q-fxp)";
    else
      return "Madgwick IMU Fixed (Q-fxp)";
  }
};

// ---------------------------------------------------------------------------
// 7. Convenience type aliases for the paper's Q-formats
// ---------------------------------------------------------------------------
template<bool UseMag>
using FilterMadgwickQ7_24 = FilterMadgwickFixed<Q7_24, UseMag>;

template<bool UseMag>  
using FilterMadgwickQ3_12 = FilterMadgwickFixed<Q3_12, UseMag>;

} // namespace EntoAttitude

#endif // MADGWICK_FIXED_H 