#ifndef FOURATI_NONLINEAR_H
#define FOURATI_NONLINEAR_H


#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>

namespace EntoAttitude
{

/**
 * @brief Fourati’s nonlinear filter update (MARG version).
 * 
 * This function performs one update step of Fourati’s algorithm using a 
 * Levenberg–Marquardt–style correction. It accepts a priori quaternion `q`, 
 * a single measurement (gyroscope, accelerometer, magnetometer), a time step `dt`,
 * and a filter gain. It also uses two reference pure quaternions: one for gravity 
 * (`g_q`) and one for the magnetic field (`m_q`).
 * 
 * @tparam Scalar Floating‐point type.
 * @param q The a priori attitude as a unit quaternion.
 * @param gyr 3×1 gyroscope measurement (rad/s).
 * @param acc 3×1 accelerometer measurement (m/s²).
 * @param mag 3×1 magnetometer measurement (in appropriate units, e.g. mT or nT).
 * @param dt Sampling time (s).
 * @param gain Filter gain factor.
 * @param g_q Reference gravity pure quaternion (e.g. [0, 0, 0, 1]).
 * @param m_q Reference magnetic field pure quaternion (e.g. computed from magnetic dip).
 * @return Updated attitude quaternion.
 */
template <typename Scalar>
Eigen::Quaternion<Scalar> fourati_update(
    const Eigen::Quaternion<Scalar>& q,
    const EntoMath::Vec3<Scalar>& gyr,
    const EntoMath::Vec3<Scalar>& acc,
    const EntoMath::Vec3<Scalar>& mag,
    Scalar dt,
    Scalar gain,
    const Eigen::Quaternion<Scalar>& g_q,
    const Eigen::Quaternion<Scalar>& m_q)
{
  // If the gyroscope measurement is zero, return the current orientation.
  if (gyr.norm() == Scalar(0))
    return q;

  // Local copy of current quaternion.
  Eigen::Quaternion<Scalar> q_new = q;

  // Compute the quaternion derivative from gyroscope (using pure quaternion for ω).
  Eigen::Quaternion<Scalar> omega(Scalar(0), gyr.x(), gyr.y(), gyr.z());
  Eigen::Quaternion<Scalar> qDot = (q_new * omega);
  qDot.coeffs() *= Scalar(0.5);

  // If gain > 0, apply a LM‐style correction step.
  if (gain > Scalar(0))
  {
    Scalar a_norm = acc.norm();
    if (a_norm == Scalar(0))
        return q_new; // or simply skip correction if no accelerometer data

    Scalar m_norm = mag.norm();
    if (m_norm == Scalar(0))
        return q_new; // or skip correction if no magnetometer data

    // Normalize the accelerometer and magnetometer measurements.
    Eigen::Matrix<Scalar, 3, 1> a_normalized = acc / a_norm;
    Eigen::Matrix<Scalar, 3, 1> m_normalized = mag / m_norm;

    // Compute estimated measurements:
    // fhat = q_new.conjugate() * (g_q * q_new)
    // hhat = q_new.conjugate() * (m_q * q_new)
    Eigen::Quaternion<Scalar> fhat = q_new.conjugate() * (g_q * q_new);
    Eigen::Quaternion<Scalar> hhat = q_new.conjugate() * (m_q * q_new);

    // Extract the vector parts (x, y, z) of fhat and hhat.
    Eigen::Matrix<Scalar, 3, 1> fhat_vec(fhat.x(), fhat.y(), fhat.z());
    Eigen::Matrix<Scalar, 3, 1> hhat_vec(hhat.x(), hhat.y(), hhat.z());

    // Form the measurement vector y (6×1) from normalized accelerometer and magnetometer.
    Eigen::Matrix<Scalar, 6, 1> y;
    y << a_normalized, m_normalized;

    // Form the estimated measurement vector yhat (6×1) from the vector parts.
    Eigen::Matrix<Scalar, 6, 1> yhat;
    yhat << fhat_vec, hhat_vec;

    // Modeling error: dq = y - yhat.
    Eigen::Matrix<Scalar, 6, 1> dq = y - yhat;

    // Compute the Jacobian matrix X as in equation (23):
    // X = -2 * [ skew(fhat_vec), skew(hhat_vec) ]^T.
    // First, form a 3×6 matrix A = [ skew(fhat_vec)  |  skew(hhat_vec) ].
    Eigen::Matrix<Scalar, 3, 3> skew_f = EntoMath::skew(fhat_vec);
    Eigen::Matrix<Scalar, 3, 3> skew_h = EntoMath::skew(hhat_vec);
    Eigen::Matrix<Scalar, 3, 6> A;
    A << skew_f, skew_h;
    // Then, X = -2 * A^T gives a 6×3 matrix.
    Eigen::Matrix<Scalar, 6, 3> X = -2 * A.transpose();

    // Damping factor (λ) to guarantee inversion.
    Scalar lam = Scalar(1e-5);

    // Compute the gain matrix K as in LM update:
    // K = gain * [ (X^T X + λ I)^{-1} X^T ].
    Eigen::Matrix<Scalar, 3, 3> temp = X.transpose() * X + lam * Eigen::Matrix<Scalar, 3, 3>::Identity();
    Eigen::Matrix<Scalar, 3, 6> K = gain * temp.inverse() * X.transpose();

    // Correction vector eta (3×1): eta = K * dq.
    Eigen::Matrix<Scalar, 3, 1> eta = K * dq;

    // Form Delta as a quaternion with scalar part 1 and vector part eta.
    Eigen::Quaternion<Scalar> Delta(Scalar(1), eta(0), eta(1), eta(2));
    // Correct the quaternion derivative.
    qDot = qDot * Delta;
  }

  // Integrate the corrected quaternion derivative:
  // q_updated = q_new + qDot * dt, then normalize.
  Eigen::Matrix<Scalar, 4, 1> q_vec;
  q_vec << q_new.w(), q_new.x(), q_new.y(), q_new.z();
  Eigen::Matrix<Scalar, 4, 1> qDot_vec;
  qDot_vec << qDot.w(), qDot.x(), qDot.y(), qDot.z();
  Eigen::Matrix<Scalar, 4, 1> q_updated_vec = q_vec + qDot_vec * dt;
  q_updated_vec.normalize();

  return Eigen::Quaternion<Scalar>(q_updated_vec(0),
                                   q_updated_vec(1),
                                   q_updated_vec(2),
                                   q_updated_vec(3));
}


/**
 * @brief Functor for Fourati's Nonlinear Quaternion Attitude MARG Filter.
 *
 * This functor implements the attitude estimation algorithm proposed by Fourati et al.,
 * which combines a quaternion-based nonlinear filter with a single-step Levenberg–Marquardt
 * correction. The algorithm fuses measurements from a tri-axial gyroscope, accelerometer,
 * and magnetometer (i.e. a MARG configuration) to update the current attitude estimate.
 *
 * The filter operates by first integrating the gyroscope measurements (to obtain a predicted
 * attitude) and then applying a correction based on the discrepancy between the measured and
 * estimated accelerometer and magnetometer outputs. The correction is computed using a
 * Levenberg–Marquardt–inspired step where a Jacobian (formed from the skew-symmetric matrices
 * of the estimated gravity and magnetic field vectors) is used to minimize the modeling error.
 *
 * @tparam Scalar Floating-point type.
 *
 * @param q   The a priori quaternion representing the current attitude.
 * @param meas A MARGMeasurement containing the sensor data (gyroscope, accelerometer, and magnetometer).
 * @param dt  Sampling time (in seconds) between consecutive updates.
 * @param gain Filter gain factor that scales the LM-based correction.
 * @param g_q Reference gravity pure quaternion (e.g., [0, 0, 0, 1]).
 * @param m_q Reference magnetic field pure quaternion representing the local geomagnetic field.
 * @return Updated quaternion after applying the Fourati filter update.
 *
 * @note Fourati’s algorithm is designed for MARG systems. In this implementation,
 *       if magnetometer data is not available (i.e. UseMag==false), the functor falls back to
 *       basic gyroscope integration.
 */
template <typename Scalar>
struct FilterFourati
{
  inline Eigen::Quaternion<Scalar>
  operator()(const Eigen::Quaternion<Scalar>& q,
             const MARGMeasurement<Scalar>& meas,
             Scalar dt,
             Scalar gain,
             const Eigen::Quaternion<Scalar>& g_q,
             const Eigen::Quaternion<Scalar>& m_q)
  {
    return fourati_update(q, meas.gyr, meas.acc, meas.mag, dt, gain, g_q, m_q);
  }

  static constexpr const char* name()
  {
    return "Fourati Nonlinear Quaternion Attitude MARG Filter";
  }

};

} // namespace EntoAttitude

#endif // FOURATI_NONLINEAR_H
