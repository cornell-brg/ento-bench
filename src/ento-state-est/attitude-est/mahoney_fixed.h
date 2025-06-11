#ifndef MAHONEY_FIXED_H
#define MAHONEY_FIXED_H

#include <type_traits>
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
// Instantiate the FixedPoint template properly
using Q7_24 = FixedPoint<7, 24, int32_t>;   // 32-bit path from paper
using Q3_12 = FixedPoint<3, 12, int16_t>;   // 16-bit path from paper
using Q6_25 = FixedPoint<6, 25, int32_t>;   // 32-bit with more fractional precision
using Q5_26 = FixedPoint<5, 26, int32_t>;   // 32-bit with even more fractional precision
using Q2_13 = FixedPoint<2, 13, int16_t>;   // 16-bit with higher precision than Q3.12

// Time-specific format: Q1.31 for high-precision time representation
using Q1_31 = FixedPoint<1, 31, int32_t>;   // Time format: ±2s range, 4.66e-10s precision

// Also provide the Eigen-compatible formats for testing
using Q1_15_t = FixedPoint<1, 15, uint16_t>;   // 16-bit Eigen compatible
using Q1_31_t = FixedPoint<1, 31, uint32_t>;   // 32-bit Eigen compatible

} // namespace EntoAttitude

namespace EntoAttitude
{

// ---------------------------------------------------------------------------
// 3. Helper: fast inverse square-root (simplified for now)
// ---------------------------------------------------------------------------
namespace detail
{
template<typename S>
inline S inv_sqrt_newton(const S &x)
{
  // Use the global sqrt function
  return S(1.0f) / sqrt(x);
}

inline float inv_sqrt(const float &x)
{
  return 1.0f / std::sqrt(x);
}

// For FixedPoint types, use the global sqrt function
template<int I, int F, typename T>
inline FixedPoint<I, F, T> inv_sqrt(const FixedPoint<I, F, T> &x)
{
  return FixedPoint<I, F, T>(1.0f) / sqrt(x);
}

} // namespace detail

// ---------------------------------------------------------------------------
// 4. Vector normalisation helper (templated on Eigen 3-vec)
// ---------------------------------------------------------------------------
template<typename Derived>
inline Derived vec_normalise(const Eigen::MatrixBase<Derived> &v)
{
  using S = typename Derived::Scalar;
  S n2 = v.squaredNorm();
  if ( n2 == S(0.0f) )
    return v.derived();
  S invn = detail::inv_sqrt(n2);
  return (v * invn).eval();
}

// ---------------------------------------------------------------------------
// 5. Core Mahony update — IMU variant (gyro + accel)
// ---------------------------------------------------------------------------
template<typename S>
Eigen::Quaternion<S> mahony_update_imu_fixed(
    const Eigen::Quaternion<S>       &q_in,
    const Eigen::Matrix<S,3,1>       &gyr,
    const Eigen::Matrix<S,3,1>       &acc,
    S                                dt,    // Changed to Scalar type
    S                                k_p,
    S                                k_i,
    Eigen::Matrix<S,3,1>             &bias)
{
  // Debug: Print input values
  ENTO_DEBUG("MAHONY DEBUG: gyr=[%f, %f, %f]", 
    static_cast<float>(gyr.x()), static_cast<float>(gyr.y()), static_cast<float>(gyr.z()));
  
  S gyr_norm2 = gyr.squaredNorm();
  ENTO_DEBUG("MAHONY DEBUG: gyr_norm2=%f", static_cast<float>(gyr_norm2));
  
  if ( gyr_norm2 == S(0.0f) )
  {
    ENTO_DEBUG("MAHONY DEBUG: Gyro norm is zero - returning unchanged quaternion");
    return q_in;
  }

  ENTO_DEBUG("MAHONY DEBUG: Gyro norm is non-zero - proceeding with algorithm");
  Eigen::Quaternion<S>   q = q_in;

  // -------------------------------------------------- accelerometer path
  S a_norm2 = acc.squaredNorm();
  ENTO_DEBUG("MAHONY DEBUG: acc_norm2=%f", static_cast<float>(a_norm2));
  
  if ( a_norm2 > S(0.0f) )
  {
    ENTO_DEBUG("MAHONY DEBUG: Processing accelerometer data");
    // Expected gravity from attitude (body-frame z)
    auto a_hat = vec_normalise(acc);
    const auto R  = q.toRotationMatrix();
    Eigen::Matrix<S,3,1> v_a = R.transpose() * Eigen::Matrix<S,3,1>(S(0.0f), S(0.0f), S(1.0f));

    // Error term
    Eigen::Matrix<S,3,1> omega_mes = a_hat.cross( v_a );
    ENTO_DEBUG("MAHONY DEBUG: omega_mes=[%f, %f, %f]", 
      static_cast<float>(omega_mes.x()), static_cast<float>(omega_mes.y()), static_cast<float>(omega_mes.z()));

    // PI feedback - compute bias update in fixed-point precision
    Eigen::Matrix<S,3,1> bias_update = (-k_i * omega_mes) * dt;
    bias += bias_update;
    
    Eigen::Matrix<S,3,1> omega = gyr - bias + k_p * omega_mes;
    ENTO_DEBUG("MAHONY DEBUG: omega=[%f, %f, %f]", 
      static_cast<float>(omega.x()), static_cast<float>(omega.y()), static_cast<float>(omega.z()));

    Eigen::Quaternion<S> p(S(0.0f), omega.x(), omega.y(), omega.z());
    Eigen::Quaternion<S> q_dot = q * p;
    q_dot.coeffs() *= S(0.5f);
    
    // Quaternion integration in fixed-point
    ENTO_DEBUG("FIXED-POINT INTEGRATION DEBUG: dt=%f, q_dot=[%f, %f, %f, %f]", 
      static_cast<float>(dt),
      static_cast<float>(q_dot.coeffs()[0]), static_cast<float>(q_dot.coeffs()[1]), 
      static_cast<float>(q_dot.coeffs()[2]), static_cast<float>(q_dot.coeffs()[3]));
    
    q.coeffs()[0] += q_dot.coeffs()[0] * dt;
    q.coeffs()[1] += q_dot.coeffs()[1] * dt;
    q.coeffs()[2] += q_dot.coeffs()[2] * dt;
    q.coeffs()[3] += q_dot.coeffs()[3] * dt;
    
    q.normalize();
    
    ENTO_DEBUG("MAHONY DEBUG: Updated quaternion=[%f, %f, %f, %f]", 
      static_cast<float>(q.coeffs()[0]), static_cast<float>(q.coeffs()[1]), 
      static_cast<float>(q.coeffs()[2]), static_cast<float>(q.coeffs()[3]));
  }
  else
  {
    ENTO_DEBUG("MAHONY DEBUG: Accelerometer norm is zero - skipping accelerometer correction");
  }
  return q;
}

// ---------------------------------------------------------------------------
// 6. Core Mahony update — MARG variant (gyro + accel + mag)
// ---------------------------------------------------------------------------
template<typename S>
Eigen::Quaternion<S> mahony_update_marg_fixed(
    const Eigen::Quaternion<S>       &q_in,
    const Eigen::Matrix<S,3,1>       &gyr,
    const Eigen::Matrix<S,3,1>       &acc,
    const Eigen::Matrix<S,3,1>       &mag,
    S                                dt,    // Changed to Scalar type
    S                                k_p,
    S                                k_i,
    Eigen::Matrix<S,3,1>             &bias)
{

  ENTO_DEBUG("MAHONY MARG DEBUG: gyr=[%f, %f, %f]", 
    static_cast<float>(gyr.x()), static_cast<float>(gyr.y()), static_cast<float>(gyr.z()));
  ENTO_DEBUG("MAHONY MARG DEBUG: acc=[%f, %f, %f]", 
    static_cast<float>(acc.x()), static_cast<float>(acc.y()), static_cast<float>(acc.z()));
  ENTO_DEBUG("MAHONY MARG DEBUG: acc=[%f, %f, %f]", 
    static_cast<float>(acc.x()), static_cast<float>(acc.y()), static_cast<float>(acc.z()));
  
  if ( gyr.squaredNorm() == S(0.0f) )
  {
    ENTO_DEBUG("MAHONY MARG DEBUG: Gyro norm is zero - returning unchanged quaternion");
    return q_in;
  }

  ENTO_DEBUG("MAHONY MARG DEBUG: Gyro norm is non-zero - proceeding with algorithm");
  Eigen::Quaternion<S> q = q_in;
  S a_norm2 = acc.squaredNorm();
  ENTO_DEBUG("MAHONY MARG DEBUG: acc_norm2=%f", static_cast<float>(a_norm2));
  
  if ( a_norm2 == S(0.0f) )
  {
    ENTO_DEBUG("MAHONY MARG DEBUG: Accelerometer norm is zero - returning unchanged quaternion");
    return q;
  }

  S m_norm2 = mag.squaredNorm();
  ENTO_DEBUG("MAHONY MARG DEBUG: mag_norm2=%f", static_cast<float>(m_norm2));
  
  if ( m_norm2 == S(0.0f) )
  {
    ENTO_DEBUG("MAHONY MARG DEBUG: Magnetometer norm is zero - falling back to IMU-only update");
    return mahony_update_imu_fixed(q_in, gyr, acc, dt, k_p, k_i, bias);
  }

  ENTO_DEBUG("MAHONY MARG DEBUG: Processing full MARG data");
  Eigen::Matrix<S,3,1> a_hat = vec_normalise(acc);
  Eigen::Matrix<S,3,1> m_hat = vec_normalise(mag);

  const Eigen::Matrix<S,3,3> R  = q.toRotationMatrix();
  Eigen::Matrix<S,3,1> v_a = R.transpose() * Eigen::Matrix<S,3,1>(S(0.0f), S(0.0f), S(1.0f));

  // Magnetic correction
  Eigen::Matrix<S,3,1> h = R * m_hat;
  S h_xy2 = h.x()*h.x() + h.y()*h.y();
  S inv_hxy = ( h_xy2 == S(0.0f) ) ? S(0.0f) : detail::inv_sqrt(h_xy2);
  S h_xy = h_xy2 == S(0.0f) ? S(0.0f) : inv_hxy * Eigen::Matrix<S,2,1>(h.x(), h.y()).norm();

  Eigen::Matrix<S,3,1> v_m = R.transpose() * Eigen::Matrix<S,3,1>(S(0.0f), h_xy, h.z());
  v_m = vec_normalise( v_m );

  Eigen::Matrix<S,3,1> omega_mes = a_hat.cross( v_a ) + m_hat.cross( v_m );
  ENTO_DEBUG("MAHONY MARG DEBUG: omega_mes=[%f, %f, %f]", 
    static_cast<float>(omega_mes.x()), static_cast<float>(omega_mes.y()), static_cast<float>(omega_mes.z()));

  // PI feedback - compute bias update in fixed-point precision
  Eigen::Matrix<S,3,1> bias_update = (-k_i * omega_mes) * dt;
  bias += bias_update;
  
  Eigen::Matrix<S,3,1> omega = gyr - bias + k_p * omega_mes;
  ENTO_DEBUG("MAHONY MARG DEBUG: omega=[%f, %f, %f]", 
    static_cast<float>(omega.x()), static_cast<float>(omega.y()), static_cast<float>(omega.z()));

  Eigen::Quaternion<S> p(S(0.0f), omega.x(), omega.y(), omega.z());
  Eigen::Quaternion<S> q_dot = q * p;
  q_dot.coeffs() *= S(0.5f);
  
  // Quaternion integration in fixed-point
  ENTO_DEBUG("FIXED-POINT INTEGRATION DEBUG: dt=%f, q_dot=[%f, %f, %f, %f]", 
    static_cast<float>(dt),
    static_cast<float>(q_dot.coeffs()[0]), static_cast<float>(q_dot.coeffs()[1]), 
    static_cast<float>(q_dot.coeffs()[2]), static_cast<float>(q_dot.coeffs()[3]));
  
  q.coeffs()[0] += q_dot.coeffs()[0] * dt;
  q.coeffs()[1] += q_dot.coeffs()[1] * dt;
  q.coeffs()[2] += q_dot.coeffs()[2] * dt;
  q.coeffs()[3] += q_dot.coeffs()[3] * dt;
  
  q.normalize();

  ENTO_DEBUG("MAHONY MARG DEBUG: Updated quaternion=[%f, %f, %f, %f]", 
    static_cast<float>(q.coeffs()[0]), static_cast<float>(q.coeffs()[1]), 
    static_cast<float>(q.coeffs()[2]), static_cast<float>(q.coeffs()[3]));

  return q;
}

// ---------------------------------------------------------------------------
// 7. Compile-time dispatcher (matching existing API)
// ---------------------------------------------------------------------------
template<typename S,bool UseMag>
inline Eigen::Quaternion<S> mahony_fixed(const Eigen::Quaternion<S> &q,
                                          const AttitudeMeasurement<S,UseMag> &meas,
                                          S dt,    // Changed to Scalar type
                                          S k_p,
                                          S k_i,
                                          Eigen::Matrix<S,3,1> &bias)
{
  if constexpr (UseMag)
    return mahony_update_marg_fixed(q, meas.gyr, meas.acc, meas.mag, dt, k_p, k_i, bias);
  else
    return mahony_update_imu_fixed(q, meas.gyr, meas.acc, dt, k_p, k_i, bias);
}

// ---------------------------------------------------------------------------
// 8. Functor wrapper conforming to THE RULE (no internal state, external parameters only)
// ---------------------------------------------------------------------------
template<typename S,bool UseMag>
struct FilterMahonyFixed
{
  // Default constructor - no internal state
  constexpr FilterMahonyFixed() = default;

  // Compatible with existing interface - external parameters only
  inline Eigen::Quaternion<S> 
  operator()(const Eigen::Quaternion<S>& q,
             const AttitudeMeasurement<S, UseMag>& meas,
             S dt,    // Changed to Scalar type
             S k_p,
             S k_i,
             Eigen::Matrix<S,3,1>& bias)
  {
    if constexpr (UseMag)
      return mahony_update_marg_fixed(q, meas.gyr, meas.acc, meas.mag, dt, k_p, k_i, bias);
    else
      return mahony_update_imu_fixed(q, meas.gyr, meas.acc, dt, k_p, k_i, bias);
  }

  static constexpr const char *name()
  {
    if constexpr (UseMag)
      return "Mahony MARG Fixed (Q-fxp)";
    else
      return "Mahony IMU Fixed (Q-fxp)";
  }
};

// ---------------------------------------------------------------------------
// 9. Convenience type aliases for the paper's Q-formats
// ---------------------------------------------------------------------------
template<bool UseMag>
using FilterMahonyQ7_24 = FilterMahonyFixed<Q7_24, UseMag>;

template<bool UseMag>  
using FilterMahonyQ3_12 = FilterMahonyFixed<Q3_12, UseMag>;

} // namespace EntoAttitude

#endif // MAHONEY_FIXED_H 