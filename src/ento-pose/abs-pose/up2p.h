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

template <typename Scalar>
int up2p(const std::vector<Eigen::Vector3d> &x,
         const std::vector<Eigen::Vector3d> &X,
         EntoArray<CameraPose<Scalar>, 4>   &output);

// Wrapper for non-upright gravity (g_cam = R*g_world)
template <typename Scalar, size_t N>
int up2p(const EntoArray<Vec3<Scalar>, N> &x,
         const EntoArray<Vec3<Scalar>, N> &X,
         const Vec3<Scalar>               &g_cam,
         const Vec3<Scalar>               &g_world,
         EntoArray<CameraPose<Scalar>, 4> &output);


// =====================================================
// Implementations



}

#endif
