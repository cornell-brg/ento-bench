#ifndef MAHONEY_H
#define MAHONEY_H

#include <Eigen/Dense>

#include <ento-math/core.h>

#include <ento-state-est/attitude-est/attitude_measurement.h>

namespace EntoAttitude
{

template <typename Scalar>
Eigen::Quaternion<Scalar> mahoney_imu(const Eigen::Quaternion<Scalar>& q,
                                      const EntoMath::Vec3<Scalar>& gyr,
                                      const EntoMath::Vec3<Scalar>& acc,
                                      Scalar dt,
                                      Scalar k_p,
                                      Scalar k_i,
                                      EntoMath::Vec3<Scalar>& bias)
{
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
    // Compute the rotation matrix from the current quaternion.
    const Eigen::Matrix<Scalar, 3, 3> R = q_new.toRotationMatrix();

    // Expected direction of gravity in the body frame is given by:
    // v_a = R^T * [0, 0, 1]^T.
    const Eigen::Matrix<Scalar, 3, 1> v_a = R.transpose() *
        Eigen::Matrix<Scalar, 3, 1>(Scalar(0), Scalar(0), Scalar(1));

    // Normalize the accelerometer reading.
    const Eigen::Matrix<Scalar, 3, 1> acc_normalized = acc / a_norm;

    // Innovation term (error) computed as the cross product between the measured and
    // expected gravity vectors.
    const Eigen::Matrix<Scalar, 3, 1> omega_mes = acc_normalized.cross(v_a);

    // Compute the derivative of the bias (integral term).
    const Eigen::Matrix<Scalar, 3, 1> b_dot = -k_i * omega_mes;

    // Update the bias.
    bias += b_dot * dt;

    // Correct the measured angular velocity.
    omega = omega - bias + k_p * omega_mes;
  }

  // Create a pure quaternion from the angular velocity: (0, omega).
  const Eigen::Quaternion<Scalar> p(Scalar(0), omega.x(), omega.y(), omega.z());

  // Compute the quaternion derivative: q_dot = 0.5 * q * p.
  Eigen::Quaternion<Scalar> q_dot = q_new * p;
  q_dot.coeffs() *= Scalar(0.5);

  // Integrate to get the new quaternion.
  q_new.coeffs() += q_dot.coeffs() * dt;
  q_new.normalize();

  return q_new;
}

template <typename Scalar>
Eigen::Quaternion<Scalar> mahoney_marg(
    const Eigen::Quaternion<Scalar>& q,
    const Eigen::Matrix<Scalar, 3, 1>& gyr,
    const Eigen::Matrix<Scalar, 3, 1>& acc,
    const Eigen::Matrix<Scalar, 3, 1>& mag,
    Scalar dt,
    Scalar k_p,
    Scalar k_i,
    Eigen::Matrix<Scalar, 3, 1>& bias)
{
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
          return update_imu(q, gyr, acc, dt, k_p, k_i, bias);

      // Normalize the accelerometer and magnetometer measurements.
      const Eigen::Matrix<Scalar, 3, 1> a_normalized = acc / a_norm;
      const Eigen::Matrix<Scalar, 3, 1> m_normalized = mag / m_norm;

      const Eigen::Matrix<Scalar, 3, 3> R = q_new.toRotationMatrix();
      // Expected gravity vector in the body frame.
      const Eigen::Matrix<Scalar, 3, 1> v_a = R.transpose() *
          Eigen::Matrix<Scalar, 3, 1>(Scalar(0), Scalar(0), Scalar(1));

      // Rotate the normalized magnetometer reading into the inertial frame.
      const Eigen::Matrix<Scalar, 3, 1> h = R * m_normalized;
      // Compute the horizontal (xy-plane) norm of h.
      const Scalar h_xy_norm = std::sqrt(h.x() * h.x() + h.y() * h.y());

      // Construct an artificial vector for magnetometer correction.
      Eigen::Matrix<Scalar, 3, 1> v_m = R.transpose() *
          Eigen::Matrix<Scalar, 3, 1>(Scalar(0), h_xy_norm, h.z());

      // Normalize v_m if its norm is nonzero.
      const Scalar v_m_norm = v_m.norm();
      if (v_m_norm > Scalar(0))
          v_m /= v_m_norm;

      // Compute the innovation term as the sum of two cross products.
      const Eigen::Matrix<Scalar, 3, 1> omega_mes =
          a_normalized.cross(v_a) + m_normalized.cross(v_m);

      const Eigen::Matrix<Scalar, 3, 1> b_dot = -k_i * omega_mes;
      bias += b_dot * dt;
      omega = omega - bias + k_p * omega_mes;
  }

  // Create a pure quaternion from omega.
  const Eigen::Quaternion<Scalar> p(Scalar(0), omega.x(), omega.y(), omega.z());
  Eigen::Quaternion<Scalar> q_dot = q_new * p;
  q_dot.coeffs() *= Scalar(0.5);
  q_new.coeffs() += q_dot.coeffs() * dt;
  q_new.normalize();

  return q_new;
}


template <typename Scalar>
Eigen::Quaternion<Scalar> mahoney_imu(
    const Eigen::Quaternion<Scalar>& q,
    const MARGMeasurement<Scalar>& imu_meas,
    Scalar dt,
    Scalar k_p,
    Scalar k_i,
    Eigen::Matrix<Scalar, 3, 1>& bias)
{
  mahoney_marg(q, imu_meas.gyr, imu_meas.acc, imu_meas.mag, dt, k_p, k_i, bias);
}

template <typename Scalar>
Eigen::Quaternion<Scalar> mahoney_marg(
    const Eigen::Quaternion<Scalar>& q,
    const MARGMeasurement<Scalar>& marg_meas,
    Scalar dt,
    Scalar k_p,
    Scalar k_i,
    Eigen::Matrix<Scalar, 3, 1>& bias)
{
  mahoney_marg(q, marg_meas.gyr, marg_meas.acc, marg_meas.mag, dt, k_p, k_i, bias);
}



// Compile-time dispatcher
template <typename Scalar, bool UseMag>
Eigen::Quaternion<Scalar> mahoney(const Eigen::Quaternion<Scalar>& q,
                                  const AttitudeMeasurement<Scalar, UseMag>& meas,
                                  Scalar dt,
                                  Scalar k_p,
                                  Scalar k_i,
                                  EntoMath::Vec3<Scalar>& bias)
{
  if constexpr (UseMag)
  {
    return mahoney_marg(q, meas, dt, k_p, k_i, bias);
  }
  else
  {
    return mahoney_imu(q, meas, dt, k_p, k_i, bias);
  }
}


#ifdef NATIVE
template <typename Scalar>
bool deserialize_mahoney(
    const char* line,
    Eigen::Quaternion<Scalar>& quat_gt,
    EntoMath::Vec3<Scalar>& gyro,
    EntoMath::Vec3<Scalar>& acc,
    EntoMath::Vec3<Scalar>& mag)
{
    std::istringstream iss(line);
    char comma;

    // Parse ground truth quaternion (quat_gt)
    for (int i = 0; i < 4; ++i) {
        if (!(iss >> quat_gt.coeffs()[i] >> comma) || comma != ',') {
            return false; // Parsing failed
        }
    }

    // Parse gyroscope data (gyro_x, gyro_y, gyro_z)
    for (int i = 0; i < 3; ++i) {
        if (!(iss >> gyro[i] >> comma) || comma != ',') {
            return false; // Parsing failed
        }
    }

    // Parse accelerometer data (acc_x, acc_y, acc_z)
    for (int i = 0; i < 3; ++i) {
        if (!(iss >> acc[i] >> comma) || comma != ',') {
            return false; // Parsing failed
        }
    }

    // Parse magnetometer data (mag_x, mag_y, mag_z)
    for (int i = 0; i < 3; ++i) {
        if (i < 2) {
            if (!(iss >> mag[i] >> comma) || comma != ',') {
                return false; // Parsing failed
            }
        } else {
            if (!(iss >> mag[i])) {
                return false; // Parsing failed
            }
        }
    }

    return true; // Successfully parsed
}
#else
template <typename Scalar>
bool deserialize_mahoney(
    const char* line,
    Eigen::Quaternion<Scalar>& quat_gt,
    
    EntoMath::Vec3<Scalar>& gyro,
    EntoMath::Vec3<Scalar>& acc,
    EntoMath::Vec3<Scalar>& mag)
{
    char* pos = const_cast<char*>(line);

    // Parse ground truth quaternion (quat_gt)
    for (int i = 0; i < 4; ++i) {
        if (sscanf(pos, "%lf,", &quat_gt.coeffs()[i]) != 1) {
            return false; // Parsing failed
        }
        pos = strchr(pos, ',') + 1;
    }

    // Parse gyroscope data (gyro_x, gyro_y, gyro_z)
    for (int i = 0; i < 3; ++i) {
        if (sscanf(pos, "%lf,", &gyro[i]) != 1) {
            return false; // Parsing failed
        }
        pos = strchr(pos, ',') + 1;
    }

    // Parse accelerometer data (acc_x, acc_y, acc_z)
    for (int i = 0; i < 3; ++i) {
        if (sscanf(pos, "%lf,", &acc[i]) != 1) {
            return false; // Parsing failed
        }
        pos = strchr(pos, ',') + 1;
    }

    // Parse magnetometer data (mag_x, mag_y, mag_z)
    for (int i = 0; i < 3; ++i) {
        if (i < 2) {
            if (sscanf(pos, "%lf,", &mag[i]) != 1) {
                return false; // Parsing failed
            }
            pos = strchr(pos, ',') + 1;
        } else {
            if (sscanf(pos, "%lf", &mag[i]) != 1) {
                return false; // Parsing failed
            }
        }
    }

    return true; // Successfully parsed
}
#endif




template <typename Scalar, bool UseMag>
struct FilterMahoney
{
  inline Eigen::Quaternion<Scalar> 
  operator()(const Eigen::Quaternion<Scalar>& q,
             const AttitudeMeasurement<Scalar, UseMag>& meas,
             Scalar dt,
             Scalar k_p,
             Scalar k_i,
             EntoMath::Vec3<Scalar>& bias)
  {
    if constexpr (UseMag)
    {
      return mahoney_marg(q, meas, dt, k_p, k_i, bias);
    }
    else
    {
      return mahoney_imu(q, meas, dt, k_p, k_i, bias);
    }
  }

  static constexpr const char* name()
  {
    if constexpr (UseMag)
    {
      return "Mahoney MARG Filter";
    }
    else
    {
      return "Mahoney IMU Filter";
    }
  }
};

} // namespace EntoAttitude

#endif // MAHONEY_H
