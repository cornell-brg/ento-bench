#ifndef UPRIGHT_PLANAR_TWO_PT_H
#define UPRIGHT_PLANAR_TWO_PT_H

#include <ento-math/core.h>
#include <ento-util/containers.h>
#include <ento-pose/pose_util.h>

using namespace EntoMath;
using namespace EntoUtil;

namespace EntoPose
{

template <typename Scalar, std::size_t N>
int relpose_upright_planar_2pt(const EntoArray<Vec3<Scalar>, N> &x1,
                               const EntoArray<Vec3<Scalar>, N> &x2,
                               EntoArray<CameraPose<Scalar>, N> *output);

template <typename Scalar>
inline bool recover_a_b(const Eigen::Matrix<Scalar, 2, 2> &C,
                        Scalar cos2phi,
                        Scalar sin2phi,
                        Vec2<Scalar> &a,
                        Vec2<Scalar> &b) {

  if (std::abs(cos2phi) >= 1.0)
      return false;

  const Scalar inv_sq2 = 1.0 / std::sqrt(2.0);
  a << std::sqrt(1 + cos2phi) * inv_sq2, std::sqrt(1 - cos2phi) * inv_sq2;

  if (sin2phi < 0)
      a(1) = -a(1);

  b = C * a;

  return true;
}

template <typename Scalar, std::size_t N>
int relpose_upright_planar_2pt(const EntoArray<Vec3<Scalar>, N> &x1,
                               const EntoArray<Vec3<Scalar>, N> &x2,
                               EntoArray<CameraPose<Scalar>, N> *output)
{

  Eigen::Matrix<Scalar, 2, 2> A, B, C;
  Eigen::Vector2d a, b;

  A << x2[0](1) * x1[0](0), -x2[0](1) * x1[0](2), x2[1](1) * x1[1](0), -x2[1](1) * x1[1](2);
  B << x2[0](0) * x1[0](1), x2[0](2) * x1[0](1), x2[1](0) * x1[1](1), x2[1](2) * x1[1](1);
  C = B.inverse() * A;

  // There is a bug in the paper here where the factor 2 is missing from beta;
  const Scalar alpha = C.col(0).dot(C.col(0));
  const Scalar beta = 2.0 * C.col(0).dot(C.col(1));
  const Scalar gamma = C.col(1).dot(C.col(1));
  const Scalar alphap = alpha - gamma;
  const Scalar gammap = alpha + gamma - 2.0;
  Scalar inv_norm = 1.0 / (alphap * alphap + beta * beta);
  const Scalar disc2 = alphap * alphap + beta * beta - gammap * gammap;

  output->clear();
  if (disc2 < 0)
  {
    // Degenerate situation. In this case we return the closest non-degen solution
    // See equation (27) in the paper
    inv_norm = std::sqrt(inv_norm);
    if (gammap < 0)
      inv_norm = -inv_norm;

    if (recover_a_b(C, -alphap * inv_norm, -beta * inv_norm, a, b))
    {
      b.normalize();
      motion_from_essential_planar(b(0), b(1), -a(0), a(1), x1, x2, output);
    }
    return output->size();
  }

  const Scalar disc = std::sqrt(disc2);

  // First set of solutions
  if (recover_a_b(C,
                  (-alphap * gammap + beta * disc) * inv_norm,
                  (-beta * gammap - alphap * disc) * inv_norm, a,
                  b))
  {
      motion_from_essential_planar(b(0), b(1), -a(0), a(1), x1, x2, output);
  }

  // Second set of solutions
  if (recover_a_b(C,
                  (-alphap * gammap - beta * disc) * inv_norm,
                  (-beta * gammap + alphap * disc) * inv_norm,
                  a,
                  b))
  {
      motion_from_essential_planar(b(0), b(1), -a(0), a(1), x1, x2, output);
  }

  return output->size();
}

} // namespace EntoPose

#endif // UPRIGHT_PLANAR_TWO_PT_H
