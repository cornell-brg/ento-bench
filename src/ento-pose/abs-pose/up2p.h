#ifndef UP2P_H
#define UP2P_H

#include <ento-util/containers.h>
#include <ento-math/core.h>
#include <ento-pose/pose_util.h>

using namespace EntoUtil;
using namespace EntoMath;

namespace EntoPose
{

// =====================================================
// Prototypes

template <typename Scalar, size_t N>
int up2p(const EntoArray<Vec3<Scalar>, N> &x,
         const EntoArray<Vec3<Scalar>, N> &X,
         EntoArray<CameraPose<Scalar>, 4>   &output);

// Wrapper for non-upright gravity (g_cam = R*g_world)
template <typename Scalar, size_t N>
int up2p(const EntoArray<Vec3<Scalar>, N> &x,
         const EntoArray<Vec3<Scalar>, N> &X,
         const Vec3<Scalar>               &g_cam,
         const Vec3<Scalar>               &g_world,
         EntoArray<CameraPose<Scalar>, 4> &output);

#if defined(SEMIHOSTING)
#include <vector>
template <typename Scalar>
int up2p(const std::vector<Vec3<Scalar>> &x,
         const std::vector<Vec3<Scalar>> &X,
               std::vector<CameraPose<Scalar>>   &output);

// Wrapper for non-upright gravity (g_cam = R*g_world)
template <typename Scalar>
int up2p(const std::vector<Vec3<Scalar>, N> &x,
         const std::vector<Vec3<Scalar>, N> &X,
         const Vec3<Scalar>               &g_cam,
         const Vec3<Scalar>               &g_world,
               std::vector<CameraPose<Scalar>> &output);
#endif // defined(SEMIHOSTING)



// =====================================================
// Implementations
//
//

template <typename Scalar, std::size_t N>
int up2p(const EntoArray<Vec3<Scalar>, N> &x,
         const EntoArray<Vec3<Scalar>, N> &X,
         EntoArray<CameraPose<Scalar>, N> *output) {
    Eigen::Matrix<Scalar, 4, 4> A;
    Eigen::Matrix<Scalar, 4, 2> b;

    A << -x[0](2), 0, x[0](0), X[0](0) * x[0](2) - X[0](2) * x[0](0), 0, -x[0](2), x[0](1),
        -X[0](1) * x[0](2) - X[0](2) * x[0](1), -x[1](2), 0, x[1](0), X[1](0) * x[1](2) - X[1](2) * x[1](0), 0,
        -x[1](2), x[1](1), -X[1](1) * x[1](2) - X[1](2) * x[1](1);
    b << -2 * X[0](0) * x[0](0) - 2 * X[0](2) * x[0](2), X[0](2) * x[0](0) - X[0](0) * x[0](2), -2 * X[0](0) * x[0](1),
        X[0](2) * x[0](1) - X[0](1) * x[0](2), -2 * X[1](0) * x[1](0) - 2 * X[1](2) * x[1](2),
        X[1](2) * x[1](0) - X[1](0) * x[1](2), -2 * X[1](0) * x[1](1), X[1](2) * x[1](1) - X[1](1) * x[1](2);

    b = A.inverse() * b;

    const Scalar c2 = b(3, 0);
    const Scalar c3 = b(3, 1);

    Scalar qq[2];
    const int sols = solve_quadratic_real(1.0, c2, c3, qq);

    output->clear();
    for (int i = 0; i < sols; ++i) {
        const Scalar q = qq[i];
        const Scalar q2 = q * q;
        const Scalar inv_norm = 1.0 / (1 + q2);
        const Scalar cq = (1 - q2) * inv_norm;
        const Scalar sq = 2 * q * inv_norm;

        Matrix3x3<Scalar> R;
        R.setIdentity();
        R(0, 0) = cq;
        R(0, 2) = sq;
        R(2, 0) = -sq;
        R(2, 2) = cq;

        Vec3<Scalar> t;
        t = b.template block<3, 1>(0, 0) * q + b.template block<3, 1>(0, 1);
        t *= -inv_norm;

        output->emplace_back(R, t);
    }
    return sols;
}

template <typename Scalar, std::size_t N>
int up2p(const EntoArray<Vec3<Scalar>, N> &x,
         const EntoArray<Vec3<Scalar>, N> &X,
         const Vec3<Scalar> &g_cam,
         const Vec3<Scalar> &g_world,
         EntoArray<CameraPose<Scalar>, N> *output) {

  // Rotate camera world coordinate system
  // @TODO: Can we write our own Quaternion from two vectors?
  Matrix3x3<Scalar> Rc = Eigen::Quaterniond::FromTwoVectors(g_cam, Vec3<Scalar>::UnitY()).toRotationMatrix();
  Matrix3x3<Scalar> Rw = Eigen::Quaterniond::FromTwoVectors(g_world, Vec3<Scalar>::UnitY()).toRotationMatrix();

  EntoArray<Vec3<Scalar>, N> x_upright = x;
  EntoArray<Vec3<Scalar>, N> X_upright = X;

  for (int i = 0; i < 2; ++i)
  {
    x_upright[i] = Rc * x[i];
    X_upright[i] = Rw * X[i];
  }

  int n_sols = up2p(x_upright, X_upright, output);

  // De-rotate coordinate systems
  for (int i = 0; i < n_sols; ++i)
  {
    Matrix3x3<Scalar> R = (*output)[i].R();
    Vec3<Scalar> t = (*output)[i].t;
    t = Rc.transpose() * t;
    R = Rc.transpose() * R * Rw;
    (*output)[i] = CameraPose(R, t);
  }
  return n_sols;
}

#if defined(NATIVE)
template <typename Scalar>
int up2p(const std::vector<Vec3<Scalar>> &x,
         const std::vector<Vec3<Scalar>> &X,
               std::vector<CameraPose<Scalar>> *output) {
  Eigen::Matrix<Scalar, 4, 4> A;
  Eigen::Matrix<Scalar, 4, 2> b;

  A << -x[0](2), 0, x[0](0), X[0](0) * x[0](2) - X[0](2) * x[0](0), 0, -x[0](2), x[0](1),
      -X[0](1) * x[0](2) - X[0](2) * x[0](1), -x[1](2), 0, x[1](0), X[1](0) * x[1](2) - X[1](2) * x[1](0), 0,
      -x[1](2), x[1](1), -X[1](1) * x[1](2) - X[1](2) * x[1](1);
  b << -2 * X[0](0) * x[0](0) - 2 * X[0](2) * x[0](2), X[0](2) * x[0](0) - X[0](0) * x[0](2), -2 * X[0](0) * x[0](1),
      X[0](2) * x[0](1) - X[0](1) * x[0](2), -2 * X[1](0) * x[1](0) - 2 * X[1](2) * x[1](2),
      X[1](2) * x[1](0) - X[1](0) * x[1](2), -2 * X[1](0) * x[1](1), X[1](2) * x[1](1) - X[1](1) * x[1](2);

  b = A.inverse() * b;

  const Scalar c2 = b(3, 0);
  const Scalar c3 = b(3, 1);

  Scalar qq[2];
  const int sols = solve_quadratic_real(Scalar(1.0), c2, c3, qq);

  output->clear();
  for (int i = 0; i < sols; ++i)
  {
    const Scalar q = qq[i];
    const Scalar q2 = q * q;
    const Scalar inv_norm = 1.0 / (1 + q2);
    const Scalar cq = (1 - q2) * inv_norm;
    const Scalar sq = 2 * q * inv_norm;

    Matrix3x3<Scalar> R;
    R.setIdentity();
    R(0, 0) = cq;
    R(0, 2) = sq;
    R(2, 0) = -sq;
    R(2, 2) = cq;

    Vec3<Scalar> t;
    t = b.template block<3, 1>(0, 0) * q + b.template block<3, 1>(0, 1);
    t *= -inv_norm;

    output->emplace_back(R, t);
  }
  return sols;
}

template <typename Scalar>
int up2p(const std::vector<Vec3<Scalar>> &x,
         const std::vector<Vec3<Scalar>> &X,
         const Vec3<Scalar> &g_cam,
         const Vec3<Scalar> &g_world,
         std::vector<CameraPose<Scalar>> *output) {

  // Rotate camera world coordinate system
  // @TODO: Can we write our own Quaternion from two vectors?
  Matrix3x3<Scalar> Rc = Eigen::Quaterniond::FromTwoVectors(g_cam, Vec3<Scalar>::UnitY()).toRotationMatrix();
  Matrix3x3<Scalar> Rw = Eigen::Quaterniond::FromTwoVectors(g_world, Vec3<Scalar>::UnitY()).toRotationMatrix();

  std::vector<Vec3<Scalar>> x_upright = x;
  std::vector<Vec3<Scalar>> X_upright = X;

  for (int i = 0; i < 2; ++i)
  {
      x_upright[i] = Rc * x[i];
      X_upright[i] = Rw * X[i];
  }

  int n_sols = up2p(x_upright, X_upright, output);

  // De-rotate coordinate systems
  for (int i = 0; i < n_sols; ++i) {
      Matrix3x3<Scalar> R = (*output)[i].R();
      Vec3<Scalar> t = (*output)[i].t;
      t = Rc.transpose() * t;
      R = Rc.transpose() * R * Rw;
      (*output)[i] = CameraPose(R, t);
  }
  return n_sols;
}
#endif // if defined(NATIVE)



}

#endif // UP2P_H

