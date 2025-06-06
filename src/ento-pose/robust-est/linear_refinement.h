#pragma once
#include <ento-pose/pose_util.h>
#include <ento-util/containers.h>
#include <Eigen/Dense>
#include <ento-pose/rel-pose/eight_pt.h>
#include <ento-pose/rel-pose/upright_planar_three_pt.h>
#include <ento-pose/robust-est/ransac_util.h>
#include <ento-pose/abs-pose/dlt.h>
#include <ento-pose/pose_util.h>

namespace EntoPose {

// Local isotropic normalization utility for linear refinement
// Normalizes 2D points so mean distance from origin is sqrt(2)
template <typename Scalar, size_t N>
static inline void iso_normalize_points(const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points,
                                       EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& normed_points,
                                       Eigen::Matrix<Scalar,3,3>& T)
{
    const size_t n = points.size();
    normed_points.clear();
    // For dynamic-size EntoContainer (N==0), reserve to avoid reallocations
    if constexpr (N == 0)
        normed_points.reserve(n);
    // Compute centroid
    Eigen::Matrix<Scalar,2,1> centroid = Eigen::Matrix<Scalar,2,1>::Zero();
    for (size_t i = 0; i < n; ++i) centroid += points[i];
    centroid /= Scalar(n);
    // Center points
    Scalar mean_dist = 0;
    for (size_t i = 0; i < n; ++i) {
        Eigen::Matrix<Scalar,2,1> p = points[i] - centroid;
        normed_points.push_back(p);
        mean_dist += p.norm();
    }
    mean_dist /= Scalar(n);
    Scalar scale = std::sqrt(2) / (mean_dist > Scalar(1e-12) ? mean_dist : Scalar(1));
    for (size_t i = 0; i < n; ++i) normed_points[i] *= scale;
    // Build normalization matrix
    T.setIdentity();
    T(0,0) = scale; T(1,1) = scale;
    T(0,2) = -scale * centroid(0);
    T(1,2) = -scale * centroid(1);
}

// Note: Isotropic normalization for DLT is now unified in pose_util.h as isotropic_normalize_points()
// This avoids duplication between dlt.h and linear_refinement.h

// Linear refinement using the 8-point algorithm (all inliers)
// N can be 0 (dynamic) or >0 (fixed size)
template<typename Scalar, size_t N>
void linear_refine_eight_point(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2,
    CameraPose<Scalar>* pose_out)
{
    const size_t n = x1.size();
    ENTO_DEBUG("linear_refine_eight_point: n = %zu", n);
    // Isotropic normalization of 2D points (not used for essential matrix)
    // EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N> x1_norm, x2_norm;
    // Eigen::Matrix<Scalar,3,3> T1, T2;
    // iso_normalize_points<Scalar, N>(x1, x1_norm, T1);
    // iso_normalize_points<Scalar, N>(x2, x2_norm, T2);
    // Convert 2D points to bearing vectors (homogeneous, normalized to unit length)

    Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor, N, 3> x1_bear(n, 3), x2_bear(n, 3);
    for (size_t k = 0; k < n; ++k)
    {
      x1_bear.row(k) = x1[k].homogeneous().normalized();
      x2_bear.row(k) = x2[k].homogeneous().normalized();
    }
    
    Eigen::Matrix<Scalar, Eigen::Dynamic, 9, Eigen::RowMajor, N, 9> A(n, 9);
    for (size_t i = 0; i < n; ++i) {
        const auto& x1v = x1_bear.row(i);
        const auto& x2v = x2_bear.row(i);
        A.row(i) << x2v(0) * x1v(0), x2v(0) * x1v(1), x2v(0) * x1v(2),
                    x2v(1) * x1v(0), x2v(1) * x1v(1), x2v(1) * x1v(2),
                    x2v(2) * x1v(0), x2v(2) * x1v(1), x2v(2) * x1v(2);
    }
    //ENTO_DEBUG_EIGEN_MATRIX(A);
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, 9, Eigen::RowMajor, N, 9>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 9, 1> e = svd.matrixV().col(8);
    //ENTO_DEBUG("[linear_refine_eight_point] SVD (V.col(8)): %f %f %f ...", e(0), e(1), e(2));
    Eigen::Matrix<Scalar, 3, 3> E = Eigen::Map<Eigen::Matrix<Scalar,3,3,Eigen::RowMajor>>(e.data());
    //ENTO_DEBUG("[linear_refine_eight_point] Estimated E:\n%f %f %f\n%f %f %f\n%f %f %f", E(0,0), E(0,1), E(0,2), E(1,0), E(1,1), E(1,2), E(2,0), E(2,1), E(2,2));
    Eigen::JacobiSVD<Eigen::Matrix<Scalar,3,3>> svdE(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<Scalar,3,1> d = svdE.singularValues();
    //ENTO_DEBUG("[linear_refine_eight_point] SVD singular values: %f %f %f", d(0), d(1), d(2));
    Scalar a = d[0], b = d[1];
    d << (a + b) / 2., (a + b) / 2., 0.0;
    E = svdE.matrixU() * d.asDiagonal() * svdE.matrixV().transpose();
    EntoUtil::EntoArray<CameraPose<Scalar>, 4> candidates;
    // Convert x1_bear/x2_bear to EntoContainer for motion_from_essential
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N> x1_bear_c, x2_bear_c;
    for (size_t i = 0; i < n; ++i) {
        x1_bear_c.push_back(x1_bear.row(i).transpose().eval());
        x2_bear_c.push_back(x2_bear.row(i).transpose().eval());
    }
    motion_from_essential<Scalar, N, 4>(E, x1_bear_c, x2_bear_c, &candidates);
    int best_idx = -1;
    Scalar best_score = std::numeric_limits<Scalar>::max();
    for (size_t i = 0; i < candidates.capacity(); ++i) {
        size_t dummy;
        Scalar score = compute_sampson_msac_score<Scalar, N>(candidates[i], x1, x2, Scalar(1e-3), &dummy);
        //ENTO_DEBUG("Candidate %zu: Sampson score = %f", i, score);
        if (score < best_score) {
            best_score = score;
            best_idx = int(i);
        }
    }
    // Debug: print candidate poses
    //for (size_t i = 0; i < candidates.size(); ++i) {
    //    ENTO_DEBUG("Candidate %zu:\nR=\n%f %f %f\n%f %f %f\n%f %f %f\nt= %f %f %f", i,
    //        candidates[i].R()(0,0), candidates[i].R()(0,1), candidates[i].R()(0,2),
    //        candidates[i].R()(1,0), candidates[i].R()(1,1), candidates[i].R()(1,2),
    //        candidates[i].R()(2,0), candidates[i].R()(2,1), candidates[i].R()(2,2),
    //        candidates[i].t(0), candidates[i].t(1), candidates[i].t(2));
    //}
    if (best_idx >= 0) {
        *pose_out = candidates[best_idx];
        //ENTO_DEBUG("Selected pose: R=\n%f %f %f\n%f %f %f\n%f %f %f\nt= %f %f %f",
        //    pose_out->R()(0,0), pose_out->R()(0,1), pose_out->R()(0,2),
        //    pose_out->R()(1,0), pose_out->R()(1,1), pose_out->R()(1,2),
        //    pose_out->R()(2,0), pose_out->R()(2,1), pose_out->R()(2,2),
        //    pose_out->t(0), pose_out->t(1), pose_out->t(2));
    }
}

// Linear refinement using upright planar 3pt (all inliers, N>=3)
// N can be 0 (dynamic) or >0 (fixed size)
template<typename Scalar, size_t N>
void linear_refine_upright_planar_3pt(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2,
    CameraPose<Scalar>* pose_out)
{
    const size_t n = x1.size();
    Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor, N, 3> x1_bear(n, 3), x2_bear(n, 3);
    for (size_t i = 0; i < n; ++i) {
        Eigen::Matrix<Scalar,3,1> h1;
        h1 << x1[i](0), x1[i](1), Scalar(1);
        x1_bear(i, 0) = h1(0);
        x1_bear(i, 1) = h1(1);
        x1_bear(i, 2) = h1(2);
        Eigen::Matrix<Scalar,3,1> h2;
        h2 << x2[i](0), x2[i](1), Scalar(1);
        x2_bear(i, 0) = h2(0);
        x2_bear(i, 1) = h2(1);
        x2_bear(i, 2) = h2(2);
    }
    // Hybrid Eigen matrix for A (n x 4, max N x 4)
    Eigen::Matrix<Scalar, Eigen::Dynamic, 4, Eigen::RowMajor, N, 4> A(n, 4);
    for (size_t i = 0; i < n; ++i) {
        const auto& a = x1_bear.row(i);
        const auto& b = x2_bear.row(i);
        A(i,0) = a(0) * b(1);
        A(i,1) = -a(2) * b(1);
        A(i,2) = -b(0) * a(1);
        A(i,3) = -b(2) * a(1);
    }
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, 4, Eigen::RowMajor, N, 4>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 4, 1> nullspace = svd.matrixV().col(3);
    EntoUtil::EntoArray<CameraPose<Scalar>, 2> candidates;
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N> x1_bear_c, x2_bear_c;
    for (size_t i = 0; i < n; ++i) {
        x1_bear_c.push_back(x1_bear.row(i).transpose().eval());
        x2_bear_c.push_back(x2_bear.row(i).transpose().eval());
    }
    motion_from_essential_planar<Scalar, N>(nullspace(2), nullspace(3), -nullspace(0), nullspace(1), x1_bear_c, x2_bear_c, &candidates);
    int best_idx = -1;
    Scalar best_score = std::numeric_limits<Scalar>::max();
    for (size_t i = 0; i < candidates.size(); ++i) {
        size_t dummy;
        Scalar score = compute_sampson_msac_score<Scalar, N>(candidates[i], x1, x2, Scalar(1e-6), &dummy);
        ENTO_DEBUG("Candidate %zu: Sampson score = %f", i, score);
        if (score < best_score) {
            best_score = score;
            best_idx = int(i);
        }
    }
    if (best_idx >= 0) *pose_out = candidates[best_idx];
}

// Linear refinement using DLT (for abs pose, homography, etc.)
// N can be 0 (dynamic) or >0 (fixed size)
// UseIsoNormalization: whether to use isotropic normalization (default true)
template<typename Scalar, size_t N, bool UseIsoNormalization = true>
void linear_refine_dlt(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    CameraPose<Scalar>* pose_out)
{
    const size_t n = points2D.size();
    ENTO_DEBUG("linear_refine_dlt: n = %zu, iso_norm = %s", n, UseIsoNormalization ? "true" : "false");
    
    if (n < 6) {
        ENTO_DEBUG("linear_refine_dlt: Not enough points for DLT (need at least 6, got %zu)", n);
        return;
    }

    // Prepare data for DLT
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N> points2D_homo, points3D_work;
    Eigen::Matrix<Scalar, 3, 3> T1 = Eigen::Matrix<Scalar, 3, 3>::Identity();
    Eigen::Matrix<Scalar, 4, 4> T2 = Eigen::Matrix<Scalar, 4, 4>::Identity();
    
    if constexpr (UseIsoNormalization) {
        // Use isotropic normalization
        ENTO_DEBUG("linear_refine_dlt: Using isotropic normalization");
        isotropic_normalize_points<Scalar>(points2D, points3D, points2D_homo, points3D_work, T1, T2);
    } else {
        // No normalization - just convert 2D to homogeneous
        ENTO_DEBUG("linear_refine_dlt: No normalization");
        if constexpr (N == 0) {
            points2D_homo.reserve(n);
            points3D_work.reserve(n);
        }
        for (size_t i = 0; i < n; ++i) {
            Eigen::Matrix<Scalar,3,1> x_h;
            x_h << points2D[i](0), points2D[i](1), Scalar(1);
            points2D_homo.push_back(x_h);
            points3D_work.push_back(points3D[i]);
        }
    }

    // Build DLT system: A * p = 0 where p is the vectorized projection matrix
    // Use hybrid Eigen matrix pattern: Dynamic with compile-time max size
    Eigen::Matrix<Scalar, Eigen::Dynamic, 12, Eigen::RowMajor, N*2, 12> A(2*n, 12);
    
    for (size_t i = 0; i < n; ++i) {
        const auto& X = points3D_work[i];  // 3D point (normalized or original)
        const auto& x = points2D_homo[i];  // homogeneous 2D point (normalized or original)
        
        // First row: x[2] * (P[0] * X) - x[0] * (P[2] * X) = 0
        A.row(2*i) << X(0) * x(2), X(1) * x(2), X(2) * x(2), x(2),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                      -x(0) * X(0), -x(0) * X(1), -x(0) * X(2), -x(0);
        
        // Second row: x[2] * (P[1] * X) - x[1] * (P[2] * X) = 0  
        A.row(2*i+1) << Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                        X(0) * x(2), X(1) * x(2), X(2) * x(2), x(2),
                        -x(1) * X(0), -x(1) * X(1), -x(1) * X(2), -x(1);
    }
    
    // Solve least squares: A * p = 0
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, 12, Eigen::RowMajor, N*2, 12>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 12, 1> p = svd.matrixV().col(11);
    
    // Reshape to 3x4 projection matrix
    Eigen::Matrix<Scalar, 3, 4> P = Eigen::Map<Eigen::Matrix<Scalar, 3, 4, Eigen::RowMajor>>(p.data());
    
    // Check for degeneracy
    if (!P.allFinite() || std::abs(P.determinant()) < Scalar(1e-8)) {
        ENTO_DEBUG("linear_refine_dlt: Degenerate solution");
        return;
    }
    
    // Normalize so that last entry is 1 (if possible)
    if (std::abs(P(2,3)) > Scalar(1e-8)) {
        P /= P(2,3);
    }
    
    if constexpr (UseIsoNormalization) {
        // Unnormalize P: P_original = T1^-1 * P_normalized * T2
        P = T1.inverse() * P * T2;
    }
    
    // Extract pose from projection matrix
    pose_out->from_projection(P);
    ENTO_DEBUG("linear_refine_dlt: Successfully refined pose using DLT");
}

// Weighted linear refinement using DLT (for absolute pose)
// N can be 0 (dynamic) or >0 (fixed size)
// UseIsoNormalization: whether to use isotropic normalization (default true)
template<typename Scalar, size_t N, bool UseIsoNormalization = true>
void linear_refine_dlt_weighted(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    const EntoUtil::EntoContainer<Scalar, N>& weights,
    CameraPose<Scalar>* pose_out)
{
    const size_t n = points2D.size();
    ENTO_DEBUG("linear_refine_dlt_weighted: n = %zu, iso_norm = %s", n, UseIsoNormalization ? "true" : "false");
    
    if (n < 6) {
        ENTO_DEBUG("linear_refine_dlt_weighted: Not enough points for DLT (need at least 6, got %zu)", n);
        return;
    }

    // Prepare data for DLT
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N> points2D_homo, points3D_work;
    Eigen::Matrix<Scalar, 3, 3> T1 = Eigen::Matrix<Scalar, 3, 3>::Identity();
    Eigen::Matrix<Scalar, 4, 4> T2 = Eigen::Matrix<Scalar, 4, 4>::Identity();
    
    if constexpr (UseIsoNormalization) {
        // Use isotropic normalization
        ENTO_DEBUG("linear_refine_dlt_weighted: Using isotropic normalization");
        isotropic_normalize_points<Scalar>(points2D, points3D, points2D_homo, points3D_work, T1, T2);
    } else {
        // No normalization - just convert 2D to homogeneous
        ENTO_DEBUG("linear_refine_dlt_weighted: No normalization");
        if constexpr (N == 0) {
            points2D_homo.reserve(n);
            points3D_work.reserve(n);
        }
        for (size_t i = 0; i < n; ++i) {
            Eigen::Matrix<Scalar,3,1> x_h;
            x_h << points2D[i](0), points2D[i](1), Scalar(1);
            points2D_homo.push_back(x_h);
            points3D_work.push_back(points3D[i]);
        }
    }

    // Build weighted DLT system: A * p = 0 where p is the vectorized projection matrix
    // Use hybrid Eigen matrix pattern: Dynamic with compile-time max size
    Eigen::Matrix<Scalar, Eigen::Dynamic, 12, Eigen::RowMajor, N*2, 12> A(2*n, 12);
    
    for (size_t i = 0; i < n; ++i) {
        const auto& X = points3D_work[i];  // 3D point (normalized or original)
        const auto& x = points2D_homo[i];  // homogeneous 2D point (normalized or original)
        Scalar w = std::sqrt(weights[i]);  // Square root of weight
        
        // First row: w * (x[2] * (P[0] * X) - x[0] * (P[2] * X)) = 0
        // Note: x is homogeneous 3D, so x[2] is the homogeneous coordinate (should be 1)
        A.row(2*i) << w * X(0) * x(2), w * X(1) * x(2), w * X(2) * x(2), w * x(2),
                      Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                      -w * x(0) * X(0), -w * x(0) * X(1), -w * x(0) * X(2), -w * x(0);
        
        // Second row: w * (x[2] * (P[1] * X) - x[1] * (P[2] * X)) = 0  
        A.row(2*i+1) << Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                        w * X(0) * x(2), w * X(1) * x(2), w * X(2) * x(2), w * x(2),
                        -w * x(1) * X(0), -w * x(1) * X(1), -w * x(1) * X(2), -w * x(1);
    }
    
    // Solve weighted least squares: A * p = 0
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, 12, Eigen::RowMajor, N*2, 12>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 12, 1> p = svd.matrixV().col(11);
    
    // Reshape to 3x4 projection matrix
    Eigen::Matrix<Scalar, 3, 4> P = Eigen::Map<Eigen::Matrix<Scalar, 3, 4, Eigen::RowMajor>>(p.data());
    
    // Check for degeneracy
    if (!P.allFinite() || std::abs(P.determinant()) < Scalar(1e-8)) {
        ENTO_DEBUG("linear_refine_dlt_weighted: Degenerate solution");
        return;
    }
    
    // Normalize so that last entry is 1 (if possible)
    if (std::abs(P(2,3)) > Scalar(1e-8)) {
        P /= P(2,3);
    }
    
    if constexpr (UseIsoNormalization) {
        // Unnormalize P: P_original = T1^-1 * P_normalized * T2
        P = T1.inverse() * P * T2;
    }
    
    // Extract pose from projection matrix
    pose_out->from_projection(P);
    ENTO_DEBUG("linear_refine_dlt_weighted: Successfully refined pose using weighted DLT");
}

// IRLS Huber refinement for DLT (absolute pose)
// N can be 0 (dynamic) or >0 (fixed size)
template<typename Scalar, size_t N>
void linear_refine_irls_huber_dlt(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& points2D,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& points3D,
    CameraPose<Scalar>* pose_out,
    int max_iters = 10,
    Scalar huber_thresh = Scalar(1.0))
{
    const size_t n = points2D.size();
    ENTO_DEBUG("linear_refine_irls_huber_dlt: n = %zu, max_iters = %d, huber_thresh = %f", 
               n, max_iters, huber_thresh);
    
    if (n < 6) {
        ENTO_DEBUG("linear_refine_irls_huber_dlt: Not enough points for DLT (need at least 6, got %zu)", n);
        return;
    }

    // Initialize weights to uniform
    EntoUtil::EntoContainer<Scalar, N> weights;
    if constexpr (N == 0) {
        weights.reserve(n);
    }
    for (size_t i = 0; i < n; ++i) {
        weights.push_back(Scalar(1.0));
    }
    
    // Initial DLT solution
    CameraPose<Scalar> pose;
    ENTO_DEBUG("linear_refine_irls_huber_dlt: Starting with initial DLT solution");
    linear_refine_dlt_weighted<Scalar, N>(points2D, points3D, weights, &pose);
    
    // Compute initial reprojection error statistics
    Scalar initial_mean_error = Scalar(0);
    Scalar initial_max_error = Scalar(0);
    for (size_t i = 0; i < n; ++i) {
        Eigen::Matrix<Scalar,3,1> X_cam = pose.R() * points3D[i] + pose.t;
        if (X_cam(2) > Scalar(1e-6)) {
            Eigen::Matrix<Scalar,2,1> x_proj;
            x_proj << X_cam(0) / X_cam(2), X_cam(1) / X_cam(2);
            Scalar error = (x_proj - points2D[i]).norm();
            initial_mean_error += error;
            initial_max_error = std::max(initial_max_error, error);
        }
    }
    initial_mean_error /= Scalar(n);
    ENTO_DEBUG("linear_refine_irls_huber_dlt: Initial errors - mean: %f, max: %f", 
               initial_mean_error, initial_max_error);
    
    // IRLS iterations
    for (int iter = 0; iter < max_iters; ++iter) {
        ENTO_DEBUG("linear_refine_irls_huber_dlt: iteration %d", iter);
        
        // Compute reprojection residuals
        EntoUtil::EntoContainer<Scalar, N> residuals;
        if constexpr (N == 0) {
            residuals.reserve(n);
        } else {
            residuals.clear();
        }
        
        Scalar mean_error = Scalar(0);
        Scalar max_error = Scalar(0);
        size_t valid_points = 0;
        
        for (size_t i = 0; i < n; ++i) {
            // Project 3D point using current pose
            Eigen::Matrix<Scalar,3,1> X_cam = pose.R() * points3D[i] + pose.t;
            
            if (X_cam(2) > Scalar(1e-6)) {  // Check for positive depth
                Eigen::Matrix<Scalar,2,1> x_proj;
                x_proj << X_cam(0) / X_cam(2), X_cam(1) / X_cam(2);
                
                // Compute reprojection error
                Scalar error = (x_proj - points2D[i]).norm();
                residuals.push_back(error);
                mean_error += error;
                max_error = std::max(max_error, error);
                valid_points++;
            } else {
                // Point behind camera - assign large residual
                residuals.push_back(huber_thresh * Scalar(10));
            }
        }
        
        if (valid_points > 0) {
            mean_error /= Scalar(valid_points);
        }
        
        // Update Huber weights
        weights.clear();
        Scalar total_weight = Scalar(0);
        size_t downweighted_points = 0;
        for (size_t i = 0; i < n; ++i) {
            Scalar r = residuals[i];
            Scalar w = (r <= huber_thresh) ? Scalar(1.0) : huber_thresh / r;
            if (w < Scalar(0.99)) downweighted_points++;
            weights.push_back(w);
            total_weight += w;
            if (iter == 0 || iter == max_iters - 1) {  // Only log first and last iteration to avoid spam
                ENTO_DEBUG("Point %zu: residual = %f, weight = %f", i, r, w);
            }
        }
        ENTO_DEBUG("linear_refine_irls_huber_dlt: iteration %d, mean_error = %f, max_error = %f, downweighted = %zu/%zu, total_weight = %f", 
                   iter, mean_error, max_error, downweighted_points, n, total_weight);
        
        // Weighted DLT with updated weights
        CameraPose<Scalar> pose_new;
        ENTO_DEBUG("linear_refine_irls_huber_dlt: Calling weighted DLT for iteration %d", iter);
        linear_refine_dlt_weighted<Scalar, N>(points2D, points3D, weights, &pose_new);
        
        // Check for convergence (optional)
        Scalar pose_change = (pose_new.R() - pose.R()).norm() + (pose_new.t - pose.t).norm();
        ENTO_DEBUG("linear_refine_irls_huber_dlt: pose change = %f", pose_change);
        
        pose = pose_new;
        
        if (pose_change < Scalar(1e-6)) {
            ENTO_DEBUG("linear_refine_irls_huber_dlt: Converged after %d iterations", iter + 1);
            break;
        }
    }
    
    // Final error statistics
    Scalar final_mean_error = Scalar(0);
    Scalar final_max_error = Scalar(0);
    size_t final_valid_points = 0;
    for (size_t i = 0; i < n; ++i) {
        Eigen::Matrix<Scalar,3,1> X_cam = pose.R() * points3D[i] + pose.t;
        if (X_cam(2) > Scalar(1e-6)) {
            Eigen::Matrix<Scalar,2,1> x_proj;
            x_proj << X_cam(0) / X_cam(2), X_cam(1) / X_cam(2);
            Scalar error = (x_proj - points2D[i]).norm();
            final_mean_error += error;
            final_max_error = std::max(final_max_error, error);
            final_valid_points++;
        }
    }
    if (final_valid_points > 0) {
        final_mean_error /= Scalar(final_valid_points);
    }
    
    ENTO_DEBUG("linear_refine_irls_huber_dlt: Error improvement - mean: %f -> %f (%.1f%%), max: %f -> %f (%.1f%%)", 
               initial_mean_error, final_mean_error, 
               initial_mean_error > 0 ? (initial_mean_error - final_mean_error) / initial_mean_error * 100 : 0,
               initial_max_error, final_max_error,
               initial_max_error > 0 ? (initial_max_error - final_max_error) / initial_max_error * 100 : 0);
    
    *pose_out = pose;
    ENTO_DEBUG("linear_refine_irls_huber_dlt: Completed IRLS refinement");
}

// Weighted linear refinement using the 8-point algorithm (all inliers)
// N can be 0 (dynamic) or >0 (fixed size)
template<typename Scalar, size_t N>
void linear_refine_eight_point_weighted(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2,
    const EntoUtil::EntoContainer<Scalar, N>& weights,
    CameraPose<Scalar>* pose_out)
{
    const size_t n = x1.size();
    Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor, N, 3> x1_bear(n, 3), x2_bear(n, 3);
    for (size_t i = 0; i < n; ++i) {
        Eigen::Matrix<Scalar,3,1> h1;
        h1 << x1[i](0), x1[i](1), Scalar(1);
        x1_bear(i, 0) = h1(0);
        x1_bear(i, 1) = h1(1);
        x1_bear(i, 2) = h1(2);
        Eigen::Matrix<Scalar,3,1> h2;
        h2 << x2[i](0), x2[i](1), Scalar(1);
        x2_bear(i, 0) = h2(0);
        x2_bear(i, 1) = h2(1);
        x2_bear(i, 2) = h2(2);
    }
    Eigen::Matrix<Scalar, Eigen::Dynamic, 9, Eigen::RowMajor, N, 9> A(n, 9);
    for (size_t i = 0; i < n; ++i) {
        Scalar w = std::sqrt(weights[i]);
        const auto& x1v = x1_bear.row(i);
        const auto& x2v = x2_bear.row(i);
        A.row(i) << w * x2v(0) * x1v(0), w * x2v(0) * x1v(1), w * x2v(0) * x1v(2),
                    w * x2v(1) * x1v(0), w * x2v(1) * x1v(1), w * x2v(1) * x1v(2),
                    w * x2v(2) * x1v(0), w * x2v(2) * x1v(1), w * x2v(2) * x1v(2);
    }
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, 9, Eigen::RowMajor, N, 9>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 9, 1> e = svd.matrixV().col(8);
    Eigen::Matrix<Scalar, 3, 3> E = Eigen::Map<Eigen::Matrix<Scalar,3,3,Eigen::RowMajor>>(e.data());
    Eigen::JacobiSVD<Eigen::Matrix<Scalar,3,3>> svdE(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<Scalar,3,1> d = svdE.singularValues();
    Scalar a = d[0], b = d[1];
    d << (a + b) / 2., (a + b) / 2., 0.0;
    E = svdE.matrixU() * d.asDiagonal() * svdE.matrixV().transpose();
    EntoUtil::EntoArray<CameraPose<Scalar>, 4> candidates;
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N> x1_bear_c, x2_bear_c;
    for (size_t i = 0; i < n; ++i) {
        x1_bear_c.push_back(x1_bear.row(i).transpose().eval());
        x2_bear_c.push_back(x2_bear.row(i).transpose().eval());
    }
    motion_from_essential<Scalar, N, 4>(E, x1_bear_c, x2_bear_c, &candidates);
    int best_idx = -1;
    Scalar best_score = std::numeric_limits<Scalar>::max();
    for (size_t i = 0; i < candidates.size(); ++i) {
        size_t dummy;
        Scalar score = compute_sampson_msac_score<Scalar, N>(candidates[i], x1, x2, Scalar(1e-6), &dummy);
        ENTO_DEBUG("Candidate %zu: Sampson score = %f", i, score);
        if (score < best_score) {
            best_score = score;
            best_idx = int(i);
        }
    }
    if (best_idx >= 0) *pose_out = candidates[best_idx];
}

// Weighted linear refinement for upright planar 3pt
// N can be 0 (dynamic) or >0 (fixed size)
template<typename Scalar, size_t N>
void linear_refine_upright_planar_3pt_weighted(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2,
    const EntoUtil::EntoContainer<Scalar, N>& weights,
    CameraPose<Scalar>* pose_out)
{
    const size_t n = x1.size();
    Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor, N, 3> x1_bear(n, 3), x2_bear(n, 3);
    for (size_t i = 0; i < n; ++i) {
        Eigen::Matrix<Scalar,3,1> h1;
        h1 << x1[i](0), x1[i](1), Scalar(1);
        x1_bear(i, 0) = h1(0);
        x1_bear(i, 1) = h1(1);
        x1_bear(i, 2) = h1(2);
        Eigen::Matrix<Scalar,3,1> h2;
        h2 << x2[i](0), x2[i](1), Scalar(1);
        x2_bear(i, 0) = h2(0);
        x2_bear(i, 1) = h2(1);
        x2_bear(i, 2) = h2(2);
    }
    Eigen::Matrix<Scalar, Eigen::Dynamic, 4, Eigen::RowMajor, N, 4> A(n, 4);
    for (size_t i = 0; i < n; ++i) {
        Scalar w = std::sqrt(weights[i]);
        const auto& a = x1_bear.row(i);
        const auto& b = x2_bear.row(i);
        A(i,0) = w * a(0) * b(1);
        A(i,1) = -w * a(2) * b(1);
        A(i,2) = -w * b(0) * a(1);
        A(i,3) = -w * b(2) * a(1);
    }
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, 4, Eigen::RowMajor, N, 4>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 4, 1> nullspace = svd.matrixV().col(3);
    EntoUtil::EntoArray<CameraPose<Scalar>, 2> candidates;
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N> x1_bear_c, x2_bear_c;
    for (size_t i = 0; i < n; ++i) {
        x1_bear_c.push_back(x1_bear.row(i).transpose().eval());
        x2_bear_c.push_back(x2_bear.row(i).transpose().eval());
    }
    motion_from_essential_planar<Scalar, N>(nullspace(2), nullspace(3), -nullspace(0), nullspace(1), x1_bear_c, x2_bear_c, &candidates);
    int best_idx = -1;
    Scalar best_score = std::numeric_limits<Scalar>::max();
    for (size_t i = 0; i < candidates.size(); ++i) {
        size_t dummy;
        Scalar score = compute_sampson_msac_score<Scalar, N>(candidates[i], x1, x2, Scalar(1e-6), &dummy);
        ENTO_DEBUG("Candidate %zu: Sampson score = %f", i, score);
        if (score < best_score) {
            best_score = score;
            best_idx = int(i);
        }
    }
    if (best_idx >= 0) *pose_out = candidates[best_idx];
}

// IRLS Huber for 8pt and upright planar 3pt
// N can be 0 (dynamic) or >0 (fixed size)
template<typename Scalar, size_t N>
void linear_refine_irls_huber_eight_point(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2,
    CameraPose<Scalar>* pose_out,
    int max_iters = 10,
    Scalar huber_thresh = Scalar(1.0))
{
    const size_t n = x1.size();
    EntoUtil::EntoContainer<Scalar, N> weights;
    weights.clear();
    for (size_t i = 0; i < n; ++i) weights.push_back(Scalar(1.0));
    CameraPose<Scalar> pose;
    linear_refine_eight_point_weighted<Scalar, N>(x1, x2, weights, &pose);
    for (int iter = 0; iter < max_iters; ++iter) {
        // Compute Sampson residuals
        EntoUtil::EntoContainer<Scalar, N> residuals;
        residuals.clear();
        for (size_t i = 0; i < n; ++i) {
            EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, 1> x1i, x2i;
            x1i.push_back(x1[i]);
            x2i.push_back(x2[i]);
            size_t dummy;
            Scalar r2 = compute_sampson_msac_score<Scalar, 1>(pose, x1i, x2i, Scalar(1e-6), &dummy);
            residuals.push_back(std::sqrt(r2));
        }
        // Compute Huber weights
        weights.clear();
        for (size_t i = 0; i < n; ++i) {
            Scalar r = residuals[i];
            weights.push_back((std::abs(r) <= huber_thresh) ? Scalar(1.0) : huber_thresh / std::abs(r));
        }
        linear_refine_eight_point_weighted<Scalar, N>(x1, x2, weights, &pose);
    }
    *pose_out = pose;
}

template<typename Scalar, size_t N>
void linear_refine_irls_huber_upright_planar_3pt(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x2,
    CameraPose<Scalar>* pose_out,
    int max_iters = 10,
    Scalar huber_thresh = Scalar(1.0))
{
    const size_t n = x1.size();
    EntoUtil::EntoContainer<Scalar, N> weights;
    weights.clear();
    for (size_t i = 0; i < n; ++i) weights.push_back(Scalar(1.0));
    CameraPose<Scalar> pose;
    linear_refine_upright_planar_3pt_weighted<Scalar, N>(x1, x2, weights, &pose);
    for (int iter = 0; iter < max_iters; ++iter) {
        // Compute Sampson residuals
        EntoUtil::EntoContainer<Scalar, N> residuals;
        residuals.clear();
        for (size_t i = 0; i < n; ++i) {
            EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, 1> x1i, x2i;
            x1i.push_back(x1[i]);
            x2i.push_back(x2[i]);
            size_t dummy;
            Scalar r2 = compute_sampson_msac_score<Scalar, 1>(pose, x1i, x2i, Scalar(1e-6), &dummy);
            residuals.push_back(std::sqrt(r2));
        }
        // Compute Huber weights
        weights.clear();
        for (size_t i = 0; i < n; ++i) {
            Scalar r = residuals[i];
            weights.push_back((std::abs(r) <= huber_thresh) ? Scalar(1.0) : huber_thresh / std::abs(r));
        }
        linear_refine_upright_planar_3pt_weighted<Scalar, N>(x1, x2, weights, &pose);
    }
    *pose_out = pose;
}


} // namespace EntoPose 