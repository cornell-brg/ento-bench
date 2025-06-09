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

    // Correct DLT normalization (Hartley & Zisserman style)
    ENTO_DEBUG("[DLT] Using correct DLT normalization (Hartley & Zisserman)");
    
    // 1. Normalize 2D points: centroid to origin, mean distance to sqrt(2)
    Vec2<Scalar> centroid_2d = Vec2<Scalar>::Zero();
    for (size_t i = 0; i < N; ++i) {
        centroid_2d += x[i].template head<2>();
    }
    centroid_2d /= Scalar(N);
    
    Scalar mean_dist_2d = Scalar(0);
    for (size_t i = 0; i < N; ++i) {
        mean_dist_2d += (x[i].template head<2>() - centroid_2d).norm();
    }
    mean_dist_2d /= Scalar(N);
    Scalar scale_2d = std::sqrt(Scalar(2)) / (mean_dist_2d > Scalar(1e-12) ? mean_dist_2d : Scalar(1));
    
    // Build 2D transformation matrix T1
    EntoMath::Matrix3x3<Scalar> T1;
    T1.setIdentity();
    T1(0,0) = scale_2d; T1(1,1) = scale_2d;
    T1(0,2) = -scale_2d * centroid_2d.x();
    T1(1,2) = -scale_2d * centroid_2d.y();
    
    // Apply 2D normalization
    EntoArray<Vec3<Scalar>, N> x_norm;
    for (size_t i = 0; i < N; ++i) {
        Vec3<Scalar> x_h = x[i]; // Already homogeneous
        Vec3<Scalar> x_n = T1 * x_h;
        x_norm.push_back(x_n);
    }
    
    // 2. Normalize 3D points: centroid to origin, mean distance to sqrt(3)
    Vec3<Scalar> centroid_3d = Vec3<Scalar>::Zero();
    for (size_t i = 0; i < N; ++i) {
        centroid_3d += X[i];
    }
    centroid_3d /= Scalar(N);
    
    Scalar mean_dist_3d = Scalar(0);
    for (size_t i = 0; i < N; ++i) {
        mean_dist_3d += (X[i] - centroid_3d).norm();
    }
    mean_dist_3d /= Scalar(N);
    Scalar scale_3d = std::sqrt(Scalar(3)) / (mean_dist_3d > Scalar(1e-12) ? mean_dist_3d : Scalar(1));
    
    // Build 3D transformation matrix T2 (4x4 for homogeneous coordinates)
    EntoMath::Matrix4x4<Scalar> T2;
    T2.setIdentity();
    T2(0,0) = scale_3d; T2(1,1) = scale_3d; T2(2,2) = scale_3d;
    T2(0,3) = -scale_3d * centroid_3d.x();
    T2(1,3) = -scale_3d * centroid_3d.y();
    T2(2,3) = -scale_3d * centroid_3d.z();
    
    // Apply 3D normalization
    EntoArray<Vec3<Scalar>, N> X_norm;
    for (size_t i = 0; i < N; ++i) {
        Vec3<Scalar> X_centered = X[i] - centroid_3d;
        Vec3<Scalar> X_scaled = X_centered * scale_3d;
        X_norm.push_back(X_scaled);
    }
    
    ENTO_DEBUG("[DLT] 2D: centroid=(%f,%f), mean_dist=%f, scale=%f", 
               centroid_2d.x(), centroid_2d.y(), mean_dist_2d, scale_2d);
    ENTO_DEBUG("[DLT] 3D: centroid=(%f,%f,%f), mean_dist=%f, scale=%f", 
               centroid_3d.x(), centroid_3d.y(), centroid_3d.z(), mean_dist_3d, scale_3d);

    // Use fixed-size matrix for EntoArray version (compile-time known size)
    Eigen::Matrix<Scalar, 2*N, 12> A;
    for (size_t i = 0; i < N; ++i) {
        // No auto for above
        const Vec3<Scalar>& Xw = X_norm[i];
        const Vec3<Scalar>& xh = x_norm[i];

        // First row: xh[2] * (P[0] * Xw) - xh[0] * (P[2] * Xw) = 0
        A.row(2*i) << Xw.x() * xh.z(), Xw.y() * xh.z(), Xw.z() * xh.z(), xh.z(),
                     0, 0, 0, 0,
                     -Xw.x() * xh.x(), -Xw.y() * xh.x(), -Xw.z() * xh.x(), -xh.x();
        // Second row: xh[2] * (P[1] * Xw) - xh[1] * (P[2] * Xw) = 0
        A.row(2*i+1) << 0, 0, 0, 0,
                        Xw.x() * xh.z(), Xw.y() * xh.z(), Xw.z() * xh.z(), xh.z(),
                        -Xw.x() * xh.y(), -Xw.y() * xh.y(), -Xw.z() * xh.y(), -xh.y();
    }
    // Solve Ap = 0 using SVD with fixed-size matrix
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2*N, 12>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 12, 1> p = svd.matrixV().col(11);
    Eigen::Matrix<Scalar, 3, 4> P = Eigen::Map<Eigen::Matrix<Scalar, 3, 4, Eigen::RowMajor>>(p.data());
    
    // Check for degeneracy
    if (!P.allFinite() || std::abs(P.determinant()) < 1e-8) return 0;
    
    ENTO_DEBUG("[DLT] Projection matrix P (before unnormalization):");
    ENTO_DEBUG("  [%f %f %f %f]", P(0,0), P(0,1), P(0,2), P(0,3));
    ENTO_DEBUG("  [%f %f %f %f]", P(1,0), P(1,1), P(1,2), P(1,3));
    ENTO_DEBUG("  [%f %f %f %f]", P(2,0), P(2,1), P(2,2), P(2,3));
    
    // FIXED: Use H&Z standard normalization constraint ||p|| = 1
    // The SVD already gives us the solution with ||p|| = 1 (unit vector from right null space)
    // DO NOT normalize by P(2,3) as this imposes incorrect K matrix constraints!
    ENTO_DEBUG("[DLT] Using H&Z standard: ||p|| = 1 constraint (from SVD), norm = %f", p.norm());
    
    // Unnormalize P: P_orig = T1^(-1) * P_norm * T2
    P = T1.inverse() * P * T2;
    
    ENTO_DEBUG("[DLT] Final unnormalized projection matrix P:");
    ENTO_DEBUG("  [%f %f %f %f]", P(0,0), P(0,1), P(0,2), P(0,3));
    ENTO_DEBUG("  [%f %f %f %f]", P(1,0), P(1,1), P(1,2), P(1,3));
    ENTO_DEBUG("  [%f %f %f %f]", P(2,0), P(2,1), P(2,2), P(2,3));
    
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
    
    // Add normalization like the EntoArray version
    EntoMath::Matrix3x3<Scalar> T1;
    EntoMath::Matrix4x4<Scalar> T2;
    std::vector<Vec3<Scalar>> x_norm, X_norm;
    
    // Convert to EntoContainer for normalization
    EntoUtil::EntoContainer<Vec3<Scalar>, 0> x_container, X_container;
    for (const auto& pt : x) x_container.push_back(pt);
    for (const auto& pt : X) X_container.push_back(pt);
    
    EntoUtil::EntoContainer<Vec3<Scalar>, 0> x_norm_container, X_norm_container;
    isotropic_normalize_points<Scalar>(x_container, X_container, x_norm_container, X_norm_container, T1, T2);
    
    // Convert back to std::vector
    x_norm.reserve(N);
    X_norm.reserve(N);
    for (const auto& pt : x_norm_container) x_norm.push_back(pt);
    for (const auto& pt : X_norm_container) X_norm.push_back(pt);
    
    Eigen::Matrix<Scalar, Eigen::Dynamic, 12> A(2*N, 12);
    for (size_t i = 0; i < N; ++i) {
        const auto& Xw = X_norm[i];
        const auto& xh = x_norm[i];
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
    
    // Check for degeneracy
    if (!P.allFinite() || std::abs(P.determinant()) < 1e-8) return 0;
    
    ENTO_DEBUG("[DLT] Projection matrix P (before unnormalization):");
    ENTO_DEBUG("  [%f %f %f %f]", P(0,0), P(0,1), P(0,2), P(0,3));
    ENTO_DEBUG("  [%f %f %f %f]", P(1,0), P(1,1), P(1,2), P(1,3));
    ENTO_DEBUG("  [%f %f %f %f]", P(2,0), P(2,1), P(2,2), P(2,3));
    
    // FIXED: Use H&Z standard normalization constraint ||p|| = 1
    // The SVD already gives us the solution with ||p|| = 1 (unit vector from right null space)
    // DO NOT normalize by P(2,3) as this imposes incorrect K matrix constraints!
    ENTO_DEBUG("[DLT] Using H&Z standard: ||p|| = 1 constraint (from SVD), norm = %f", p.norm());
    
    // Unnormalize P: P_orig = T1^(-1) * P_norm * T2
    P = T1.inverse() * P * T2;
    
    ENTO_DEBUG("[DLT] Final unnormalized projection matrix P:");
    ENTO_DEBUG("  [%f %f %f %f]", P(0,0), P(0,1), P(0,2), P(0,3));
    ENTO_DEBUG("  [%f %f %f %f]", P(1,0), P(1,1), P(1,2), P(1,3));
    ENTO_DEBUG("  [%f %f %f %f]", P(2,0), P(2,1), P(2,2), P(2,3));
    
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