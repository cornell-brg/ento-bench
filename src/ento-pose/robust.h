#ifndef ENTO_POSE_ROBUST_H
#define ENTO_POSE_ROBUST_H

#include <ento-pose/pose_util.h>
#include <ento-pose/camera_models.h>
#include <ento-pose/robust-est/ransac_util.h>
#include <ento-pose/robust-est/ransac.h>
#include <ento-pose/robust-est/absolute.h>
#include <ento-pose/robust-est/relative.h>
#include <ento-pose/robust-est/homography.h>
#include <ento-pose/bundle_adjustment.h>
#include <ento-pose/data_gen.h>
#include <ento-util/containers.h>
#include <ento-pose/robust-est/loss.h>
#include <ento-pose/robust-est/linear_refinement.h>

namespace EntoPose
{

// High-level functional interface for robust pose estimation
// These functions handle data normalization, RANSAC, and post-processing

// Estimates absolute pose using LO-RANSAC followed by non-linear refinement
// Threshold for reprojection error is set by RansacOptions.max_reproj_error
template <typename Solver, size_t N = 0>
RansacStats<typename Solver::scalar_type> estimate_absolute_pose(
    const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &points2D,
    const EntoUtil::EntoContainer<Vec3<typename Solver::scalar_type>, N> &points3D,
    const Camera<typename Solver::scalar_type> &camera,
    const RansacOptions<typename Solver::scalar_type> &ransac_opt,
    const BundleOptions<typename Solver::scalar_type> &bundle_opt,
    CameraPose<typename Solver::scalar_type> *pose,
    EntoUtil::EntoContainer<uint8_t, N> *inliers)
{
    using Scalar = typename Solver::scalar_type;
    
    // Normalize image points for RANSAC (unproject to normalized coordinates)
    EntoUtil::EntoContainer<Vec2<Scalar>, N> points2D_calib;
    if constexpr (N == 0) {
        points2D_calib.reserve(points2D.size());
    }
    
    for (size_t k = 0; k < points2D.size(); ++k) {
        Vec2<Scalar> normalized_pt;
        camera.unproject(points2D[k], &normalized_pt);
        points2D_calib.push_back(normalized_pt);
    }

    // Scale threshold by focal length for normalized coordinates
    RansacOptions<Scalar> ransac_opt_scaled = ransac_opt;
    ransac_opt_scaled.max_reproj_error /= camera.focal();

    // Run RANSAC with normalized coordinates
    RansacStats<Scalar> stats = ransac_pnp<Solver, N>(points2D_calib, points3D, ransac_opt_scaled, pose, inliers);
    
    // Debug: After RANSAC
    ENTO_DEBUG("After RANSAC: stats.num_inliers = %zu", stats.num_inliers);
    for (size_t i = 0; i < inliers->size(); ++i) {
        Vec3<Scalar> Z = pose->R() * points3D[i] + pose->t;
        Scalar depth = Z(2);
        Vec2<Scalar> proj = Vec2<Scalar>(Z(0) / Z(2), Z(1) / Z(2));
        Scalar reproj_error = (proj - points2D_calib[i]).squaredNorm();
        ENTO_DEBUG("Point %zu: depth=%f, reproj_error=%f, inlier=%d", 
            i, depth, reproj_error, (int)(*inliers)[i]);
    }

    // Post-RANSAC bundle adjustment if we have enough inliers
    if (stats.num_inliers > 3) {
        // Collect inliers for bundle adjustment
        ENTO_DEBUG("Performing post-RANSAC refinement on inliers: %d", stats.num_inliers);
        EntoUtil::EntoContainer<Vec2<Scalar>, N> points2D_inliers;
        EntoUtil::EntoContainer<Vec3<Scalar>, N> points3D_inliers;
        
        // Scale coordinates and camera for better numerics
        const Scalar scale = Scalar(1.0) / camera.focal();
        Camera<Scalar> norm_camera = camera;
        norm_camera.rescale(scale);
        
        BundleOptions<Scalar> bundle_opt_scaled = bundle_opt;
        bundle_opt_scaled.loss_scale *= scale;
        
        for (size_t k = 0; k < points2D.size(); ++k) {
            if ((*inliers)[k]) {
                points2D_inliers.push_back(points2D[k] * scale);
                points3D_inliers.push_back(points3D[k]);
            }
        }
        bundle_opt_scaled.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
        using CameraModel = decltype(norm_camera)::CameraModel_;
        bundle_adjust<Scalar, UniformWeightVector<Scalar>, CameraModel , TruncatedLoss<Scalar>, N>
            (points2D_inliers, points3D_inliers, norm_camera, pose, bundle_opt_scaled);
    }

    // Recompute inliers for the best pose
    get_inliers<Scalar, N>(*pose, points2D_calib, points3D, ransac_opt_scaled.max_reproj_error * ransac_opt_scaled.max_reproj_error, inliers);
    
    // Debug: After get_inliers
    size_t manual_inlier_count = 0;
    for (size_t i = 0; i < inliers->size(); ++i) {
        if ((*inliers)[i]) manual_inlier_count++;
    }
    ENTO_DEBUG("After get_inliers: manual_inlier_count = %zu", manual_inlier_count);
    for (size_t i = 0; i < inliers->size(); ++i) {
        ENTO_DEBUG("get_inliers mask [%zu]: %d", i, (int)(*inliers)[i]);
    }

    return stats;
}

// Estimates relative pose using LO-RANSAC followed by non-linear refinement  
// Threshold for Sampson error is set by RansacOptions.max_epipolar_error
template <typename Solver, size_t N = 0>
RansacStats<typename Solver::scalar_type> estimate_relative_pose(
    const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &points2D_1,
    const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &points2D_2,
    const Camera<typename Solver::scalar_type> &camera1,
    const Camera<typename Solver::scalar_type> &camera2,
    const RansacOptions<typename Solver::scalar_type> &ransac_opt,
    const BundleOptions<typename Solver::scalar_type> &bundle_opt,
    CameraPose<typename Solver::scalar_type> *relative_pose,
    EntoUtil::EntoContainer<uint8_t, N> *inliers)
{
  using Scalar = typename Solver::scalar_type;
  const size_t num_pts = points2D_1.size();

  ENTO_DEBUG("Hello...");
  // Normalize image points for both cameras
  EntoUtil::EntoContainer<Vec2<Scalar>, N> x1_calib, x2_calib;
  if constexpr (N == 0) 
  {
    x1_calib.resize(num_pts);
    x2_calib.resize(num_pts);
  }
  
  for (size_t k = 0; k < num_pts; ++k)
  {
    Vec2<Scalar> norm1, norm2;
    camera1.unproject(points2D_1[k], &norm1);
    camera2.unproject(points2D_2[k], &norm2);
    
    if constexpr (N == 0) 
    {
      x1_calib[k] = norm1;
      x2_calib[k] = norm2;
    } 
    else 
    {
      x1_calib.push_back(norm1);
      x2_calib.push_back(norm2);
    }
  }

  ENTO_DEBUG("Hello...");
  // Scale threshold for normalized coordinates
  RansacOptions<Scalar> ransac_opt_scaled = ransac_opt;
  ransac_opt_scaled.max_epipolar_error = 
      ransac_opt.max_epipolar_error * Scalar(0.5) * (Scalar(1.0) / camera1.focal() + Scalar(1.0) / camera2.focal());

  // Run RANSAC
  RansacStats<Scalar> stats = ransac_relpose<Solver, N>(x1_calib, x2_calib, ransac_opt_scaled, relative_pose, inliers);
  ENTO_DEBUG("Hello...");

  //if ((ransac_opt.lo_type != LocalRefinementType::None) && stats.num_inliers > 5) {
  //  // Gather inlier correspondences (N=0 for dynamic, but can be fixed)
  //  EntoUtil::EntoContainer<Vec2<Scalar>, N> x1_inliers, x2_inliers;
  //  for (size_t k = 0; k < num_pts; ++k) {
  //    if ((*inliers)[k]) {
  //      x1_inliers.push_back(x1_calib[k]);
  //      x2_inliers.push_back(x2_calib[k]);
  //    }
  //  }
  //  // Local refinement: support both linear and nonlinear in sequence if Both is selected
  //  if (ransac_opt.lo_type == LocalRefinementType::Linear || ransac_opt.lo_type == LocalRefinementType::Both) {
  //    switch (ransac_opt.linear_method) {
  //      case LinearRefinementMethod::EightPoint:
  //        linear_refine_eight_point<Scalar, N>(x1_inliers, x2_inliers, relative_pose);
  //        break;
  //      case LinearRefinementMethod::UprightPlanar3pt:
  //        linear_refine_upright_planar_3pt<Scalar, N>(x1_inliers, x2_inliers, relative_pose);
  //        break;
  //      default:
  //        break;
  //    }
  //  }
  //  if (ransac_opt.lo_type == LocalRefinementType::BundleAdjust || ransac_opt.lo_type == LocalRefinementType::Both) {
  //    BundleOptions<Scalar> scaled_bundle_opt = bundle_opt;
  //    scaled_bundle_opt.loss_scale = bundle_opt.loss_scale * Scalar(0.5) * 
  //        (Scalar(1.0) / camera1.focal() + Scalar(1.0) / camera2.focal());
  //    using WeightT = UniformWeightVector<Scalar>;
  //    using LossFn = TruncatedLoss<Scalar>;
  //    refine_relpose<Scalar, WeightT, LossFn, N>(x1_inliers, x2_inliers, relative_pose, scaled_bundle_opt);
  //  }
  //}

  EntoUtil::EntoContainer<Vec2<Scalar>, N> x1_inliers, x2_inliers;
  for (size_t k = 0; k < num_pts; ++k) {
    if ((*inliers)[k]) {
      x1_inliers.push_back(x1_calib[k]);
      x2_inliers.push_back(x2_calib[k]);
    }
  }
   BundleOptions<Scalar> scaled_bundle_opt = bundle_opt;
   scaled_bundle_opt.loss_scale = bundle_opt.loss_scale * Scalar(0.5) * 
       (Scalar(1.0) / camera1.focal() + Scalar(1.0) / camera2.focal());
   using WeightT = UniformWeightVector<Scalar>;
   using LossFn = TruncatedLoss<Scalar>;
   refine_relpose<Scalar, WeightT, LossFn, N>(x1_inliers, x2_inliers, relative_pose, scaled_bundle_opt);
   stats.refinements++;

  return stats;
}

// Estimates a homography matrix using LO-RANSAC followed by non-linear refinement
// Convention is x2 = H*x1
// Threshold for transfer error is set by RansacOptions.max_reproj_error
// NOTE: This assumes both cameras have the same intrinsics for proper normalization
template <typename Solver, size_t N = 0>
RansacStats<typename Solver::scalar_type> estimate_homography(
    const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &points2D_1,
    const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &points2D_2,
    const Camera<typename Solver::scalar_type> &camera1,
    const Camera<typename Solver::scalar_type> &camera2,
    const RansacOptions<typename Solver::scalar_type> &ransac_opt,
    const BundleOptions<typename Solver::scalar_type> &bundle_opt,
    Matrix3x3<typename Solver::scalar_type> *H,
    EntoUtil::EntoContainer<uint8_t, N> *inliers)
{
    using Scalar = typename Solver::scalar_type;
    const size_t num_pts = points2D_1.size();

    // Normalize image points for both cameras
    EntoUtil::EntoContainer<Vec2<Scalar>, N> x1_calib, x2_calib;
    if constexpr (N == 0) {
        x1_calib.resize(num_pts);
        x2_calib.resize(num_pts);
    }
    
    for (size_t k = 0; k < num_pts; ++k) {
        Vec2<Scalar> norm1, norm2;
        camera1.unproject(points2D_1[k], &norm1);
        camera2.unproject(points2D_2[k], &norm2);
        
        if constexpr (N == 0) {
            x1_calib[k] = norm1;
            x2_calib[k] = norm2;
        } else {
            x1_calib.push_back(norm1);
            x2_calib.push_back(norm2);
        }
    }

    // Scale threshold for normalized coordinates
    RansacOptions<Scalar> ransac_opt_scaled = ransac_opt;
    ransac_opt_scaled.max_reproj_error = 
        ransac_opt.max_reproj_error * Scalar(0.5) * (Scalar(1.0) / camera1.focal() + Scalar(1.0) / camera2.focal());

    // Run RANSAC on normalized coordinates
    Matrix3x3<Scalar> H_normalized;
    RansacStats<Scalar> stats = ransac_homography<Solver, N>(x1_calib, x2_calib, ransac_opt_scaled, &H_normalized, inliers);

    // Convert homography back to pixel coordinates
    // H_pixel = K2 * H_normalized * K1^-1
    Matrix3x3<Scalar> K1, K2, K1_inv;
    camera1.calibration_matrix(&K1);
    camera2.calibration_matrix(&K2);
    K1_inv = K1.inverse();
    *H = K2 * H_normalized * K1_inv;

    // Post-RANSAC refinement if we have enough inliers
    if (stats.num_inliers > 4) {
        // Collect inliers for refinement (use original pixel coordinates)
        EntoUtil::EntoContainer<Vec2<Scalar>, 0> x1_inliers, x2_inliers;
        
        for (size_t k = 0; k < points2D_1.size(); ++k) {
            if ((*inliers)[k]) {
                x1_inliers.push_back(points2D_1[k]);
                x2_inliers.push_back(points2D_2[k]);
            }
        }

        refine_homography(x1_inliers, x2_inliers, H, bundle_opt);
    }

    return stats;
}

} // namespace EntoPose

#endif // ENTO_POSE_ROBUST_H 
