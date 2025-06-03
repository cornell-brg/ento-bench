#ifndef UPRIGHT_PLANAR_THREE_PT_H
#define UPRIGHT_PLANAR_THREE_PT_H

#include <ento-math/core.h>
#include <ento-util/containers.h>
#include <ento-pose/pose_util.h>

using namespace EntoMath;
using namespace EntoUtil;

namespace EntoPose
{

template <typename Scalar, std::size_t N>
int relpose_upright_planar_3pt(const EntoContainer<Vec3<Scalar>, N> &x1,
                               const EntoContainer<Vec3<Scalar>, N> &x2,
                               EntoContainer<CameraPose<Scalar>, 2> *output);

template <typename Scalar, std::size_t N>
int relpose_upright_planar_3pt(const EntoContainer<Vec3<Scalar>, N> &x1,
                               const EntoContainer<Vec3<Scalar>, N> &x2,
                               EntoContainer<CameraPose<Scalar>, 2> *output)
{
    // Build the action matrix -> see (6,7) in the paper
    Eigen::Matrix<Scalar, 4, 3> A;
    for (const int i : {0, 1, 2}) {
        const auto &bearing_a_i = x1[i];
        const auto &bearing_b_i = x2[i];
        A.col(i) << bearing_a_i.x() * bearing_b_i.y(), -bearing_a_i.z() * bearing_b_i.y(),
            -bearing_b_i.x() * bearing_a_i.y(), -bearing_b_i.z() * bearing_a_i.y();
    }

    const Eigen::Matrix<Scalar, 4, 4> Q = A.householderQr().householderQ();
    const Vec4<Scalar> nullspace = Q.col(3);

    output->clear();
    motion_from_essential_planar<Scalar, N>(nullspace(2), nullspace(3), -nullspace(0), nullspace(1), x1, x2, output);
    return output->size();
}

} // namespace EntoPose

#endif // UPRIGHT_THREE_PT_H
