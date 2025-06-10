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
    output->clear();
    
    const size_t n = x1.size();
    
    if (n < 3) {
        // Not enough points
        return 0;
    }
    
    if (n == 3) {
        // MINIMAL CASE: Use original 3-point algorithm
        Eigen::Matrix<Scalar, 4, 3> A;
        for (const int i : {0, 1, 2}) {
            const auto &bearing_a_i = x1[i];
            const auto &bearing_b_i = x2[i];
            A.col(i) << bearing_a_i.x() * bearing_b_i.y(), -bearing_a_i.z() * bearing_b_i.y(),
                -bearing_b_i.x() * bearing_a_i.y(), -bearing_b_i.z() * bearing_a_i.y();
        }

        const Eigen::Matrix<Scalar, 4, 4> Q = A.householderQr().householderQ();
        const Vec4<Scalar> nullspace = Q.col(3);
        
        motion_from_essential_planar<Scalar, N>(nullspace(2), nullspace(3), -nullspace(0), nullspace(1), x1, x2, output);
    } else {
        // OVERDETERMINED CASE: Use ALL points with SVD (like linear refinement)
        // Build constraint matrix A where each row corresponds to one point correspondence
        // Each point gives us one constraint: a_x * b_y * c1 - a_z * b_y * c2 - b_x * a_y * c3 - b_z * a_y * c4 = 0
        
        Eigen::Matrix<Scalar, Eigen::Dynamic, 4, Eigen::RowMajor, N, 4> A(n, 4);
        
        for (size_t i = 0; i < n; ++i) {
            const auto& bearing_a_i = x1[i];
            const auto& bearing_b_i = x2[i];
            
            // Build constraint row for point i
            A(i, 0) = bearing_a_i.x() * bearing_b_i.y();
            A(i, 1) = -bearing_a_i.z() * bearing_b_i.y();
            A(i, 2) = -bearing_b_i.x() * bearing_a_i.y();
            A(i, 3) = -bearing_b_i.z() * bearing_a_i.y();
        }
        
        // Solve the overdetermined system using SVD
        Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, 4, Eigen::RowMajor, N, 4>> svd(A, Eigen::ComputeFullV);
        Vec4<Scalar> nullspace = svd.matrixV().col(3);
        
        motion_from_essential_planar<Scalar, N>(nullspace(2), nullspace(3), -nullspace(0), nullspace(1), x1, x2, output);
    }
    
    return output->size();
}

} // namespace EntoPose

#endif // UPRIGHT_THREE_PT_H
