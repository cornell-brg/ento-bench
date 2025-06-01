#ifndef ENTO_POSE_DLT_H
#define ENTO_POSE_DLT_H

#include <Eigen/Dense>
#include <ento-pose/pose_util.h>
#include <ento-util/containers.h>
#include <vector>

namespace EntoPose {

// DLT for general (non-planar) case, EntoArray version
// x: normalized 2D points (homogeneous, N x 3)
// X: 3D points (N x 3)
// solutions: output camera poses
// Returns: number of solutions (1 for DLT)

template <typename Scalar, size_t N>
int dlt(const EntoArray<Vec3<Scalar>, N>& x,
        const EntoArray<Vec3<Scalar>, N>& X,
        EntoArray<CameraPose<Scalar>, 1>* solutions)
{
    static_assert(N >= 6, "DLT requires at least 6 points");
    Eigen::Matrix<Scalar, Eigen::Dynamic, 12> A(2*N, 12);
    for (size_t i = 0; i < N; ++i) {
        const auto& Xw = X[i];
        const auto& xh = x[i];
        // First row: xh[2] * (P[0] * Xw) - xh[0] * (P[2] * Xw) = 0
        A.row(2*i) << Xw.x() * xh.z(), Xw.y() * xh.z(), Xw.z() * xh.z(), xh.z(),
                     0, 0, 0, 0,
                     -Xw.x() * xh.x(), -Xw.y() * xh.x(), -Xw.z() * xh.x(), -xh.x();
        // Second row: xh[2] * (P[1] * Xw) - xh[1] * (P[2] * Xw) = 0
        A.row(2*i+1) << 0, 0, 0, 0,
                        Xw.x() * xh.z(), Xw.y() * xh.z(), Xw.z() * xh.z(), xh.z(),
                        -Xw.x() * xh.y(), -Xw.y() * xh.y(), -Xw.z() * xh.y(), -xh.y();
    }
    // Solve Ap = 0 using SVD
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 12, 1> p = svd.matrixV().col(11);
    Eigen::Matrix<Scalar, 3, 4> P = Eigen::Map<Eigen::Matrix<Scalar, 3, 4, Eigen::RowMajor>>(p.data());
    
    // Check for degeneracy
    if (!P.allFinite() || std::abs(P.determinant()) < 1e-8) return 0;
    
    // Normalize so that last entry is 1 (if possible)
    if (std::abs(P(2,3)) > Scalar(1e-8)) P /= P(2,3);
    
    // Extract R and t from P
    solutions->clear();
    CameraPose<Scalar> pose;
    pose.from_projection(P);
    solutions->push_back(pose);
    return 1;
}

#if defined(NATIVE)
// std::vector overload for NATIVE
// x: normalized 2D points (homogeneous, N x 3)
// X: 3D points (N x 3)
// solutions: output camera poses
// Returns: number of solutions (1 for DLT)
template <typename Scalar>
int dlt(const std::vector<Vec3<Scalar>>& x,
        const std::vector<Vec3<Scalar>>& X,
        std::vector<CameraPose<Scalar>>* solutions)
{
    size_t N = x.size();
    if (N < 6 || X.size() != N) return 0;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 12> A(2*N, 12);
    for (size_t i = 0; i < N; ++i) {
        const auto& Xw = X[i];
        const auto& xh = x[i];
        // First row: xh[2] * (P[0] * Xw) - xh[0] * (P[2] * Xw) = 0
        A.row(2*i) << Xw.x() * xh.z(), Xw.y() * xh.z(), Xw.z() * xh.z(), xh.z(),
                     0, 0, 0, 0,
                     -Xw.x() * xh.x(), -Xw.y() * xh.x(), -Xw.z() * xh.x(), -xh.x();
        // Second row: xh[2] * (P[1] * Xw) - xh[1] * (P[2] * Xw) = 0
        A.row(2*i+1) << 0, 0, 0, 0,
                        Xw.x() * xh.z(), Xw.y() * xh.z(), Xw.z() * xh.z(), xh.z(),
                        -Xw.x() * xh.y(), -Xw.y() * xh.y(), -Xw.z() * xh.y(), -xh.y();
    }
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 12, 1> p = svd.matrixV().col(11);
    Eigen::Matrix<Scalar, 3, 4> P = Eigen::Map<Eigen::Matrix<Scalar, 3, 4, Eigen::RowMajor>>(p.data());
    
    if (!P.allFinite() || std::abs(P.determinant()) < 1e-8) return 0;
    if (std::abs(P(2,3)) > Scalar(1e-8)) P /= P(2,3);
    
    // Extract R and t from P
    solutions->clear();
    CameraPose<Scalar> pose;
    pose.from_projection(P);
    solutions->push_back(pose);
    return 1;
}
#endif

} // namespace EntoPose

#endif // ENTO_POSE_DLT_H 