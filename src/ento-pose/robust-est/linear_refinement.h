#pragma once
#include <ento-pose/pose_util.h>
#include <ento-util/containers.h>
#include <Eigen/Dense>
#include <ento-pose/rel-pose/eight_pt.h>
#include <ento-pose/rel-pose/upright_planar_three_pt.h>
#include <ento-pose/robust-est/ransac_util.h>

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
template<typename Scalar, size_t N>
void linear_refine_dlt(
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>, N>& x1,
    const EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& X,
    CameraPose<Scalar>* pose_out)
{
    // TODO: Implement DLT for abs pose/homography
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