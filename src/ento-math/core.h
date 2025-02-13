#ifndef ENTO_MATH_CORE_H
#define ENTO_MATH_CORE_H

#include <Eigen/Dense>  // Ensure Eigen is included

namespace EntoMath
{

constexpr float ENTO_EPS = 1e-6;

template <typename Scalar>
inline Scalar sign(const Scalar x)
{
  return x < Scalar(0) ? Scalar(-1) : Scalar(1);
}

template <typename Scalar>
inline Scalar sign2(const std::complex<Scalar> &z)
{
  if (std::abs(z.real()) > std::abs(z.imag()))
  {
    return z.real() < Scalar(0) ? Scalar(-1) : Scalar(1);
  }
  else
  {
    return z.imag() < Scalar(0) ? Scalar(-1) : Scalar(1);
  }
}

template <typename Scalar, int MaxM, int MaxN, int Order = 0>
using BoundedMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Order, MaxM, MaxN>;

template <typename Scalar, int MaxN>
using BoundedRowVector = Eigen::Matrix<Scalar, 1, Eigen::Dynamic, 1, 1, MaxN>;

template <typename Scalar, int MaxM>
using BoundedColVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxM, 1>;

template <typename Scalar, int Order = 0>
using Vec4 = Eigen::Matrix<Scalar, 4, 1, Order>;

template <typename Scalar, int Order = 0>
using Vec3 = Eigen::Matrix<Scalar, 3, 1, Order>;

template <typename Scalar, int Order = 0>
using Vec2 = Eigen::Matrix<Scalar, 2, 1, Order>;

template <typename Scalar, int Order=0>
using Matrix3x3 = Eigen::Matrix<Scalar, 3, 3, Order>;

template <typename Scalar, int Order=0>
using Matrix2x2 = Eigen::Matrix<Scalar, 2, 2, Order>;


} // namespace EntoMath

#endif  // ENTO_MATH_CORE_H

