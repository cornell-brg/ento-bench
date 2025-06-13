#ifndef MADGWICK_H
#define MADGWICK_H

#include <Eigen/Dense>
#include <ento-math/core.h>         // Provides EntoMath::Vec3<Scalar>
#include <ento-state-est/attitude-est/attitude_measurement.h> // Our measurement container header
#include <cmath>
#include <ento-util/debug.h>

namespace EntoAttitude
{

//------------------------------------------------------------------------------
// Madgwick IMU-only update function
//------------------------------------------------------------------------------
// Computes an updated orientation quaternion from gyroscope and accelerometer
// measurements using Madgwick's gradient descent algorithm.
// The filter gain (gain) controls the convergence rate.
template <typename Scalar>
Eigen::Quaternion<Scalar> madgwick_update_imu(
    const Eigen::Quaternion<Scalar>& q,
    const EntoMath::Vec3<Scalar>& gyr,
    const EntoMath::Vec3<Scalar>& acc,
    Scalar dt,
    Scalar gain)
{
  // If no rotation is measured, return the current orientation.
  if (gyr.norm() == Scalar(0))
    return q;

  // Compute quaternion derivative from gyroscope measurements.
  // Create a pure quaternion from gyr: (0, gyr_x, gyr_y, gyr_z)
  Eigen::Quaternion<Scalar> omega(Scalar(0), gyr.x(), gyr.y(), gyr.z());
  Eigen::Quaternion<Scalar> q_dot = (q * omega);
  q_dot.coeffs() *= Scalar(0.5);

  // Only if accelerometer measurement is valid:
  Scalar a_norm = acc.norm();
  if (a_norm > Scalar(0))
  {
    // Normalize accelerometer reading.
    const Eigen::Matrix<Scalar, 3, 1> a = acc / a_norm;
    // Normalize the current orientation.
    const Eigen::Quaternion<Scalar> q_norm = q.normalized();
    // Extract quaternion components in (w, x, y, z) order.
    const Scalar qw = q_norm.w();
    const Scalar qx = q_norm.x();
    const Scalar qy = q_norm.y();
    const Scalar qz = q_norm.z();

    // Compute objective function f (a 3-vector) per Madgwick's eq. (25):
    Eigen::Matrix<Scalar, 3, 1> f;
    f(0) = Scalar(2) * (qx * qz - qw * qy) - a(0);
    f(1) = Scalar(2) * (qw * qx + qy * qz) - a(1);
    f(2) = Scalar(2) * (Scalar(0.5) - qx * qx - qy * qy) - a(2);

    // If f is nonzero, compute the Jacobian J (3x4) and its gradient.
    if (f.norm() > Scalar(0))
    {
      // Build J, with columns in order: [qw, qx, qy, qz]
      Eigen::Matrix<Scalar, 3, 4> J;
      J(0, 0) = -Scalar(2)*qy;   J(0, 1) =  Scalar(2)*qz;   J(0, 2) = -Scalar(2)*qw;   J(0, 3) =  Scalar(2)*qx;
      J(1, 0) =  Scalar(2)*qx;   J(1, 1) =  Scalar(2)*qw;   J(1, 2) =  Scalar(2)*qz;   J(1, 3) =  Scalar(2)*qy;
      J(2, 0) = Scalar(0);       J(2, 1) = -Scalar(4)*qx;   J(2, 2) = -Scalar(4)*qy;   J(2, 3) = Scalar(0);

      // Compute the gradient (a 4-vector) = J^T * f.
      Eigen::Matrix<Scalar, 4, 1> gradient = J.transpose() * f;
      // Normalize the gradient.
      gradient.normalize();

      // Represent q_dot as a 4-vector in (w, x, y, z) order.
      Eigen::Matrix<Scalar, 4, 1> q_dot_vec;
      q_dot_vec << q_dot.w(), q_dot.x(), q_dot.y(), q_dot.z();
      // Subtract the correction term scaled by the gain.
      q_dot_vec -= gain * gradient;
      // Rebuild q_dot.
      q_dot = Eigen::Quaternion<Scalar>(q_dot_vec(0), q_dot_vec(1), q_dot_vec(2), q_dot_vec(3));
    }
  }

  // Integrate to obtain the new quaternion: q_new = q + q_dot * dt
  Eigen::Matrix<Scalar, 4, 1> q_vec;
  q_vec << q.w(), q.x(), q.y(), q.z();
  Eigen::Matrix<Scalar, 4, 1> q_dot_vec;
  q_dot_vec << q_dot.w(), q_dot.x(), q_dot.y(), q_dot.z();
  Eigen::Matrix<Scalar, 4, 1> q_new_vec = q_vec + q_dot_vec * dt;
  // Normalize the updated quaternion.
  q_new_vec = q_new_vec / q_new_vec.norm();  // More efficient than normalize()
  Eigen::Quaternion<Scalar> q_new(q_new_vec(0), q_new_vec(1), q_new_vec(2), q_new_vec(3));
  return q_new;
}

//------------------------------------------------------------------------------
// Madgwick MARG update function
//------------------------------------------------------------------------------
// Computes an updated orientation quaternion from gyroscope, accelerometer,
// and magnetometer measurements using Madgwick's gradient descent algorithm.
template <typename Scalar>
Eigen::Quaternion<Scalar> madgwick_update_marg(
    const Eigen::Quaternion<Scalar>& q,
    const EntoMath::Vec3<Scalar>& gyr,
    const EntoMath::Vec3<Scalar>& acc,
    const EntoMath::Vec3<Scalar>& mag,
    Scalar dt,
    Scalar gain)
{
  // If no rotation is measured, return the current orientation.
  if (gyr.norm() == Scalar(0))
    return q;
  // If magnetometer data is missing, fall back to the IMU update.
  if (mag.norm() == Scalar(0))
    return madgwick_update_imu(q, gyr, acc, dt, gain);

  // Compute quaternion derivative from gyroscope measurements.
  Eigen::Quaternion<Scalar> omega(Scalar(0), gyr.x(), gyr.y(), gyr.z());
  Eigen::Quaternion<Scalar> q_dot = (q * omega);
  q_dot.coeffs() *= Scalar(0.5);

  Scalar a_norm = acc.norm();
  if (a_norm > Scalar(0))
  {
    // Normalize accelerometer and magnetometer measurements.
    const Eigen::Matrix<Scalar, 3, 1> a = acc / a_norm;
    const Eigen::Matrix<Scalar, 3, 1> m = mag / mag.norm();

    // Rotate the magnetometer measurement into the Earth frame.
    // Compute h = q * (0, m) * q.conjugate()
    Eigen::Quaternion<Scalar> m_quat(Scalar(0), m.x(), m.y(), m.z());
    Eigen::Quaternion<Scalar> h = q * m_quat * q.conjugate();
    // In our notation, h = [h_w, h_x, h_y, h_z] (with h_w ≈ 0).
    // Let bx = norm( h_x, h_y ) and bz = h_z.
    const Scalar bx = std::sqrt(h.x() * h.x() + h.y() * h.y());
    const Scalar bz = h.z();

    // Normalize the current orientation.
    const Eigen::Quaternion<Scalar> q_norm = q.normalized();
    const Scalar qw = q_norm.w();
    const Scalar qx = q_norm.x();
    const Scalar qy = q_norm.y();
    const Scalar qz = q_norm.z();

    // Compute the objective function f (a 6-vector) as in eq. (31):
    Eigen::Matrix<Scalar, 6, 1> f;
    f(0) = Scalar(2) * (qx * qz - qw * qy) - a(0);
    f(1) = Scalar(2) * (qw * qx + qy * qz) - a(1);
    f(2) = Scalar(2) * (Scalar(0.5) - qx * qx - qy * qy) - a(2);
    f(3) = Scalar(2) * bx * (Scalar(0.5) - qy * qy - qz * qz) + Scalar(2) * bz * (qx * qz - qw * qy) - m(0);
    f(4) = Scalar(2) * bx * (qx * qy - qw * qz) + Scalar(2) * bz * (qw * qx + qy * qz) - m(1);
    f(5) = Scalar(2) * bx * (qw * qy + qx * qz) + Scalar(2) * bz * (Scalar(0.5) - qx * qx - qy * qy) - m(2);

    if (f.norm() > Scalar(0))
    {
      // Build the Jacobian J (6x4) with rows as follows:
      Eigen::Matrix<Scalar, 6, 4> J;

      // First three rows (as in the IMU case):
      J(0, 0) = -Scalar(2)*qy;  
      J(0, 1) =  Scalar(2)*qz;
      J(0, 2) = -Scalar(2)*qw;
      J(0, 3) =  Scalar(2)*qx;

      J(1, 0) =  Scalar(2)*qx;  
      J(1, 1) =  Scalar(2)*qw;
      J(1, 2) =  Scalar(2)*qz;
      J(1, 3) =  Scalar(2)*qy;
      J(2, 0) = Scalar(0);
      J(2, 1) = -Scalar(4)*qx;
      J(2, 2) = -Scalar(4)*qy;
      J(2, 3) = Scalar(0);

      // Next three rows (for the magnetometer):
      J(3, 0) = -Scalar(2)*bz*qy;
      J(3, 1) =  Scalar(2)*bz*qz; 
      J(3, 2) = -Scalar(4)*bx*qy - Scalar(2)*bz*qw;
      J(3, 3) = -Scalar(4)*bx*qz + Scalar(2)*bz*qx;
      
      J(4, 0) = -Scalar(2)*bx*qz + Scalar(2)*bz*qx;
      J(4, 1) =  Scalar(2)*bx*qy + Scalar(2)*bz*qw;
      J(4, 2) =  Scalar(2)*bx*qx + Scalar(2)*bz*qz;
      J(4, 3) = -Scalar(2)*bx*qw + Scalar(2)*bz*qy;
      
      J(5, 0) =  Scalar(2)*bx*qy;
      J(5, 1) =  Scalar(2)*bx*qz - Scalar(4)*bz*qx;
      J(5, 2) =  Scalar(2)*bx*qw - Scalar(4)*bz*qy;
      J(5, 3) =  Scalar(2)*bx*qx;

      // Compute the gradient (4-vector) = J^T * f.
      Eigen::Matrix<Scalar, 4, 1> gradient = J.transpose() * f;
      gradient.normalize();

      // Represent q_dot as a 4-vector (w, x, y, z).
      Eigen::Matrix<Scalar, 4, 1> q_dot_vec;
      q_dot_vec << q_dot.w(), q_dot.x(), q_dot.y(), q_dot.z();
      // Subtract the correction term.
      q_dot_vec -= gain * gradient;
      q_dot = Eigen::Quaternion<Scalar>(q_dot_vec(0), q_dot_vec(1), q_dot_vec(2), q_dot_vec(3));
    }
  }

  // Integrate: q_new = q + q_dot * dt
  Eigen::Matrix<Scalar, 4, 1> q_vec;
  q_vec << q.w(), q.x(), q.y(), q.z();
  Eigen::Matrix<Scalar, 4, 1> q_dot_vec;
  q_dot_vec << q_dot.w(), q_dot.x(), q_dot.y(), q_dot.z();
  Eigen::Matrix<Scalar, 4, 1> q_new_vec = q_vec + q_dot_vec * dt;
  q_new_vec = q_new_vec / q_new_vec.norm();  // More efficient than normalize()
  Eigen::Quaternion<Scalar> q_new(q_new_vec(0), q_new_vec(1), q_new_vec(2), q_new_vec(3));
  return q_new;
}

//------------------------------------------------------------------------------
// Dispatcher function: madgwick
//------------------------------------------------------------------------------
// This function dispatches to the appropriate Madgwick update function based on
// the compile-time flag UseMag. Use UseMag==false for IMU-only (accelerometer +
// gyroscope) and UseMag==true for MARG (with magnetometer).
template <typename Scalar, bool UseMag>
Eigen::Quaternion<Scalar> madgwick(
    const Eigen::Quaternion<Scalar>& q,
    const AttitudeMeasurement<Scalar, UseMag>& meas,
    Scalar dt,
    Scalar gain)
{
  if constexpr (UseMag)
  {
    return madgwick_update_marg(q, meas.gyr, meas.acc, meas.mag, dt, gain);
  }
  else
  {
    return madgwick_update_imu(q, meas.gyr, meas.acc, dt, gain);
  }
}

template <typename Scalar, bool UseMag>
struct FilterMadgwick
{
  // No internal state - gain passed as parameter for flexibility
  FilterMadgwick() = default;

  inline Eigen::Quaternion<Scalar>
  operator()(const Eigen::Quaternion<Scalar>& q,
             const AttitudeMeasurement<Scalar, UseMag>& meas,
             Scalar dt,
             Scalar gain)
  {
    if constexpr (UseMag)
    {
      return madgwick_update_marg(q, meas.gyr, meas.acc, meas.mag, dt, gain);
    }
    else
    {
      return madgwick_update_imu(q, meas.gyr, meas.acc, dt, gain);
    }
  }

  static constexpr const char* name()
  {
    if constexpr (UseMag)
    {
      return "Madgwick Attitude MARG Filter";
    }
    else
    {
      return "Madgwick Attitude IMU Filter";
    }
  }
};

} // namespace EntoAttitude

#endif // MADGWICK_H
