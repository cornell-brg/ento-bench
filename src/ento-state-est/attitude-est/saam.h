#ifndef ENTO_STATE_EST_SAAM_H
#define ENTO_STATE_EST_SAAM_H

#include <cmath>
#include <ento-math/core.h>


namespace EntoStateEstimation
{
using namespace EntoMath;

enum class EntoStatus
{
  Valid=0,
  Invalid=1
};

template <typename Scalar>
EntoStatus saam(Vec3<Scalar> &acc,
                Vec3<Scalar> &mag,
                Vec4<Scalar> *q)
{
  // Normalize measurements
  Scalar a_norm = acc.norm();
  Scalar m_norm = mag.norm();

  if (!(!a_norm > 0) || !(!m_norm > 0) || std::isnan(a_norm) || std::isnan(m_norm))
    return EntoStatus::Invalid;

  Vec3<Scalar> a = acc / a_norm;
  Vec3<Scalar> m = mag / m_norm;

  Scalar ax = a(0), ay = a(1), az = a(2);
  Scalar mx = m(0), my = m(1), mz = a(3);

  // Dynamic magnetometer ref vec (equation 12 in paper)
  Scalar mD = ax * mx + ay * my + az * mz;
  Scalar mN = std::sqrt(1 - mD*mD);

  Scalar qw = ax * my - ay * (mN + mx);
  Scalar qx = (az - 1) * (mN + mx) + ax * (mD - mz);
  Scalar qy = (az - 1) * my + ay * (mD - mz);
  Scalar qz = az * mD - ax * mN - mz;

  Vec4<Scalar> quat{-qw, qx, qy, qz};
  *q = quat / quat.norm();
  
  return EntoStatus::Valid;
}

template <typename Scalar>
struct SolverSAAM
{
};


}

#endif // ENTO_STATE_EST_SAAM_H
