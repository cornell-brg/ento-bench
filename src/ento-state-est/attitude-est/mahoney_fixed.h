#ifndef MAHONEY_FIXED_H
#define MAHONEY_FIXED_H

#include <Eigen/Dense>
#include <cmath>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <math/FixedPoint.hh>
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

// Also provide the Eigen-compatible formats for testing
using Q1_15_t = FixedPoint<1, 15, uint16_t>;   // 16-bit Eigen compatible
using Q1_31_t = FixedPoint<1, 31, uint32_t>;   // 32-bit Eigen compatible

} // namespace EntoAttitude

// ---------------------------------------------------------------------------
// 2. Math functions for FixedPoint types (in global namespace for Eigen)
// ---------------------------------------------------------------------------

// sqrt function for FixedPoint types using Newton-Raphson method
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
inline FixedPoint<IntegerBits, FractionalBits, UnderlyingType> 
sqrt(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &x)
{
  using FP = FixedPoint<IntegerBits, FractionalBits, UnderlyingType>;
  
  if (x == FP(0)) return FP(0);
  
  // Initial estimate via float
  float fx = static_cast<float>(x);
  FP y = FP(std::sqrt(fx));
  
  // Newton-Raphson iteration: y = 0.5 * (y + x/y)
  const FP half = FP(0.5f);
  for (int i = 0; i < 3; ++i) {  // 3 iterations should be enough
    y = half * (y + x / y);
  }
  
  return y;
}

// abs function for FixedPoint types
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
inline FixedPoint<IntegerBits, FractionalBits, UnderlyingType> 
abs(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &x)
{
  return (x < FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(0)) ? -x : x;
}

// abs2 function for FixedPoint types (Eigen specific)
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
inline FixedPoint<IntegerBits, FractionalBits, UnderlyingType> 
abs2(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &x)
{
  return x * x;
}

// conj function for FixedPoint types (identity for real numbers)
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
inline const FixedPoint<IntegerBits, FractionalBits, UnderlyingType>& 
conj(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &x)
{
  return x;
}

// real function for FixedPoint types (identity for real numbers)
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
inline const FixedPoint<IntegerBits, FractionalBits, UnderlyingType>& 
real(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &x)
{
  return x;
}

// imag function for FixedPoint types (always zero for real numbers)
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
inline FixedPoint<IntegerBits, FractionalBits, UnderlyingType> 
imag(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &)
{
  return FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(0);
}

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
  if ( n2 == S(0) )
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
    S                                 dt,
    S                                 k_p,
    S                                 k_i,
    Eigen::Matrix<S,3,1>             &bias)
{
  // Debug: Print input values
  ENTO_DEBUG("MAHONY DEBUG: gyr=[%f, %f, %f]", 
    static_cast<float>(gyr.x()), static_cast<float>(gyr.y()), static_cast<float>(gyr.z()));
  
  S gyr_norm2 = gyr.squaredNorm();
  ENTO_DEBUG("MAHONY DEBUG: gyr_norm2=%f", static_cast<float>(gyr_norm2));
  
  if ( gyr_norm2 == S(0) )
  {
    ENTO_DEBUG("MAHONY DEBUG: Gyro norm is zero - returning unchanged quaternion");
    return q_in;
  }

  ENTO_DEBUG("MAHONY DEBUG: Gyro norm is non-zero - proceeding with algorithm");
  Eigen::Quaternion<S>   q = q_in;

  // -------------------------------------------------- accelerometer path
  S a_norm2 = acc.squaredNorm();
  ENTO_DEBUG("MAHONY DEBUG: acc_norm2=%f", static_cast<float>(a_norm2));
  
  if ( a_norm2 > S(0) )
  {
    ENTO_DEBUG("MAHONY DEBUG: Processing accelerometer data");
    // Expected gravity from attitude (body-frame z)
    auto a_hat = vec_normalise(acc);
    const auto R  = q.toRotationMatrix();
    Eigen::Matrix<S,3,1> v_a = R.transpose() * Eigen::Matrix<S,3,1>(S(0), S(0), S(1));

    // Error term
    Eigen::Matrix<S,3,1> omega_mes = a_hat.cross( v_a );
    ENTO_DEBUG("MAHONY DEBUG: omega_mes=[%f, %f, %f]", 
      static_cast<float>(omega_mes.x()), static_cast<float>(omega_mes.y()), static_cast<float>(omega_mes.z()));

    // PI feedback
    bias += ( -k_i * omega_mes ) * dt;
    Eigen::Matrix<S,3,1> omega = gyr - bias + k_p * omega_mes;
    ENTO_DEBUG("MAHONY DEBUG: omega=[%f, %f, %f]", 
      static_cast<float>(omega.x()), static_cast<float>(omega.y()), static_cast<float>(omega.z()));

    // Quaternion integration (first-order)
    Eigen::Quaternion<S> p(S(0), omega.x(), omega.y(), omega.z());
    Eigen::Quaternion<S> q_dot = q * p;
    q_dot.coeffs() *= S(0.5f);
    q.coeffs() += q_dot.coeffs() * dt;
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
    S                                 dt,
    S                                 k_p,
    S                                 k_i,
    Eigen::Matrix<S,3,1>             &bias)
{
  if ( gyr.squaredNorm() == S(0) )
    return q_in;

  Eigen::Quaternion<S> q = q_in;
  S a_norm2 = acc.squaredNorm();
  if ( a_norm2 == S(0) )
    return q;

  S m_norm2 = mag.squaredNorm();
  if ( m_norm2 == S(0) )
    return mahony_update_imu_fixed(q_in, gyr, acc, dt, k_p, k_i, bias);

  auto a_hat = vec_normalise(acc);
  auto m_hat = vec_normalise(mag);

  const auto R  = q.toRotationMatrix();
  Eigen::Matrix<S,3,1> v_a = R.transpose() * Eigen::Matrix<S,3,1>(S(0), S(0), S(1));

  // Magnetic correction
  Eigen::Matrix<S,3,1> h = R * m_hat;
  S h_xy2 = h.x()*h.x() + h.y()*h.y();
  auto inv_hxy = ( h_xy2 == S(0) ) ? S(0) : detail::inv_sqrt(h_xy2);
  S h_xy = h_xy2 == S(0) ? S(0) : inv_hxy * Eigen::Matrix<S,2,1>(h.x(), h.y()).norm();

  Eigen::Matrix<S,3,1> v_m = R.transpose() * Eigen::Matrix<S,3,1>(S(0), h_xy, h.z());
  v_m = vec_normalise( v_m );

  Eigen::Matrix<S,3,1> omega_mes = a_hat.cross( v_a ) + m_hat.cross( v_m );

  bias += ( -k_i * omega_mes ) * dt;
  Eigen::Matrix<S,3,1> omega = gyr - bias + k_p * omega_mes;

  Eigen::Quaternion<S> p(S(0), omega.x(), omega.y(), omega.z());
  Eigen::Quaternion<S> q_dot = q * p;
  q_dot.coeffs() *= S(0.5f);
  q.coeffs() += q_dot.coeffs() * dt;
  q.normalize();

  return q;
}

// ---------------------------------------------------------------------------
// 7. Compile-time dispatcher (matching existing API)
// ---------------------------------------------------------------------------
template<typename S,bool UseMag>
inline Eigen::Quaternion<S> mahony_fixed(const Eigen::Quaternion<S> &q,
                                          const AttitudeMeasurement<S,UseMag> &meas,
                                          S dt,
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
// 8. Functor wrapper owning gains & bias (compatible with existing interface)
// ---------------------------------------------------------------------------
template<typename S,bool UseMag>
struct FilterMahonyFixed
{
  constexpr FilterMahonyFixed(S kp, S ki)
    : kp_{kp}, ki_{ki}, bias_{S(0),S(0),S(0)} {}

  // Compatible with existing interface - match FilterMahony signature
  inline Eigen::Quaternion<S> 
  operator()(const Eigen::Quaternion<S>& q,
             const AttitudeMeasurement<S, UseMag>& meas,
             S dt,
             S k_p,
             S k_i,
             Eigen::Matrix<S,3,1>& bias)
  {
    if constexpr (UseMag)
      return mahony_update_marg_fixed(q, meas.gyr, meas.acc, meas.mag, dt, k_p, k_i, bias);
    else
      return mahony_update_imu_fixed(q, meas.gyr, meas.acc, dt, k_p, k_i, bias);
  }

  // Alternative interface using internal state
  Eigen::Quaternion<S> operator()(const Eigen::Quaternion<S>          &q_prev,
                                  const AttitudeMeasurement<S,UseMag> &meas,
                                  S                                    dt,
                                  Eigen::Quaternion<S>               *q_out)
  {
    if constexpr (UseMag)
      *q_out = mahony_update_marg_fixed(q_prev, meas.gyr, meas.acc, meas.mag,
                                        dt, kp_, ki_, bias_);
    else
      *q_out = mahony_update_imu_fixed(q_prev, meas.gyr, meas.acc,
                                       dt, kp_, ki_, bias_);
    return *q_out;
  }

  static constexpr const char *name()
  {
    if constexpr (UseMag)
      return "Mahony MARG Fixed (Q-fxp)";
    else
      return "Mahony IMU Fixed (Q-fxp)";
  }

 private:
  S               kp_;
  S               ki_;
  Eigen::Matrix<S,3,1> bias_;
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