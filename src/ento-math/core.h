#ifndef ENTO_MATH_CORE_H
#define ENTO_MATH_CORE_H

#include <Eigen/Dense>  // Ensure Eigen is included

namespace EntoMath {

  template <typename Scalar, int MaxM, int MaxN, int Order = 0>
  using BoundedMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Order, MaxM, MaxN>;

  template <typename Scalar, int MaxN>
  using BoundedRowVector = Eigen::Matrix<Scalar, 1, Eigen::Dynamic, 1, 1, MaxN>;

  template <typename Scalar, int MaxM>
  using BoundedColVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxM, 1>;

  template <typename Scalar, int Order = 0>
  using Vec4 = Eigen::Matrix<Scalar, 4, 1, Order>;

}

#endif  // ENTO_MATH_CORE_H

