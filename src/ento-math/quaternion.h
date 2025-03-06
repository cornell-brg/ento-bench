#ifndef QUAT_H
#define QUAT_H

#include <Eigen/Dense>

template <typename Scalar>
inline Eigen::Matrix<Scalar, 3, 3> quat_to_rotmat(const Eigen::Matrix<Scalar, 4, 1> &q) {
  return Eigen::Quaternion<Scalar>(q(0), q(1), q(2), q(3)).toRotationMatrix();
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 9, 1> quat_to_rotmatvec(const Eigen::Matrix<Scalar, 4, 1> &q) {
  Eigen::Matrix<Scalar, 3, 3> R = quat_to_rotmat(q);
  return Eigen::Map<Eigen::Matrix<Scalar, 9, 1>>(R.data());
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 4, 1> rotmat_to_quat(const Eigen::Matrix<Scalar, 3, 3> &R) {
  Eigen::Quaternion<Scalar> q_flip(R);
  Eigen::Matrix<Scalar, 4, 1> q;
  q << q_flip.w(), q_flip.x(), q_flip.y(), q_flip.z();
  q.normalize();
  return q;
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 4, 1> quat_multiply(const Eigen::Matrix<Scalar, 4, 1> &qa,
                                                 const Eigen::Matrix<Scalar, 4, 1> &qb)
{
  const Scalar qaw = qa(0), qax = qa(1), qay = qa(2), qaz = qa(3);
  const Scalar qbw = qb(0), qbx = qb(1), qby = qb(2), qbz = qb(3);

  return Eigen::Matrix<Scalar, 4, 1>(qaw * qbw - qax * qbx - qay * qby - qaz * qbz,
                                     qaw * qbx + qax * qbw + qay * qbz - qaz * qby,
                                     qaw * qby - qax * qbz + qay * qbw + qaz * qbx,
                                     qaw * qbz + qax * qby - qay * qbx + qaz * qbw);
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1> quat_rotate(const Eigen::Matrix<Scalar, 4, 1> &q,
                                               const Eigen::Matrix<Scalar, 3, 1> &p) {
  const Scalar q1 = q(0), q2 = q(1), q3 = q(2), q4 = q(3);
  const Scalar p1 = p(0), p2 = p(1), p3 = p(2);
  const Scalar px1 = -p1 * q2 - p2 * q3 - p3 * q4;
  const Scalar px2 = p1 * q1 - p2 * q4 + p3 * q3;
  const Scalar px3 = p2 * q1 + p1 * q4 - p3 * q2;
  const Scalar px4 = p2 * q2 - p1 * q3 + p3 * q1;
  return Eigen::Matrix<Scalar, 3, 1>(px2 * q1 - px1 * q2 - px3 * q4 + px4 * q3,
                                     px3 * q1 - px1 * q3 + px2 * q4 - px4 * q2,
                                     px3 * q2 - px2 * q3 - px1 * q4 + px4 * q1);
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 4, 1> quat_conj(const Eigen::Matrix<Scalar, 4, 1> &q) {
  return Eigen::Matrix<Scalar, 4, 1>(q(0), -q(1), -q(2), -q(3));
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 4, 1> quat_exp(const Eigen::Matrix<Scalar, 3, 1> &w) {
  const Scalar theta2 = w.squaredNorm();
  const Scalar theta = std::sqrt(theta2);
  const Scalar theta_half = 0.5 * theta;

  Scalar re, im;
  if (theta > Scalar(1e-6)) {
    re = std::cos(theta_half);
    im = std::sin(theta_half) / theta;
  } else {
    const Scalar theta4 = theta2 * theta2;
    re = Scalar(1.0) - (Scalar(1.0) / Scalar(8.0)) * theta2 +
         (Scalar(1.0) / Scalar(384.0)) * theta4;
    im = Scalar(0.5) - (Scalar(1.0) / Scalar(48.0)) * theta2 +
         (Scalar(1.0) / Scalar(3840.0)) * theta4;

    const Scalar s = std::sqrt(re * re + im * im * theta2);
    re /= s;
    im /= s;
  }
  return Eigen::Matrix<Scalar, 4, 1>(re, im * w(0), im * w(1), im * w(2));
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 4, 1> quat_step_pre(const Eigen::Matrix<Scalar, 4, 1> &q,
                                                 const Eigen::Matrix<Scalar, 3, 1> &w_delta) {
  return quat_multiply(quat_exp(w_delta), q);
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 4, 1> quat_step_post(const Eigen::Matrix<Scalar, 4, 1> &q,
                                                  const Eigen::Matrix<Scalar, 3, 1> &w_delta) {
  return quat_multiply(q, quat_exp(w_delta));
}


#endif // QUAT_H
