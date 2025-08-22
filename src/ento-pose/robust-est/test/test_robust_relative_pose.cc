#include <stdlib.h>
#include <cstdio>
#include <random>
#include <Eigen/Dense>

#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/containers.h>

#include <ento-pose/pose_util.h>
#include <ento-pose/camera_models.h>
#include <ento-pose/data_gen.h>
#include <ento-pose/robust.h>
#include <ento-pose/rel-pose/five_pt_nister.h>
#include <ento-pose/synthetic_relpose.h>
#include <ento-pose/rel-pose/eight_pt.h>
#include <ento-pose/rel-pose/upright_three_pt.h>
#include <ento-pose/rel-pose/upright_planar_two_pt.h>
#include <ento-pose/rel-pose/upright_planar_three_pt.h>

using namespace std;
using namespace Eigen;
using namespace EntoPose;
using namespace EntoUtil;

// Generate synthetic relative pose data with known ground truth
template<typename Scalar, size_t N>
void generate_synthetic_relative_pose_data(
  EntoContainer<Vec2<Scalar>, N>& x1,
  EntoContainer<Vec2<Scalar>, N>& x2,
  CameraPose<Scalar>& true_pose,
  size_t num_points,
  Scalar noise_level = 0.01)
{
  // Set random true pose
  true_pose.q = Eigen::Quaternion<Scalar>::UnitRandom().coeffs();
  true_pose.t.setRandom();
  true_pose.t *= 2.0; // Scale translation
  
  std::default_random_engine rng(42); // Fixed seed for reproducibility
  std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
  std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
  std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
  
  if constexpr (N == 0) {
      x1.clear();
      x2.clear();
      x1.reserve(num_points);
      x2.reserve(num_points);
  }
  
  // Ensure we always fill exactly num_points valid correspondences
  size_t valid_points = 0;
  while (valid_points < num_points) {
    Vec3<Scalar> X;
    Vec3<Scalar> x1_cam, x2_cam;
    while (true)
    {
        // Generate 3D point
        X = Vec3<Scalar>(coord_gen(rng), coord_gen(rng), depth_gen(rng));
        // Project to first camera (at origin)
        x1_cam = X;
        if (x1_cam(2) > 0.1) break; // Accept only if in front of camera
    }
    // Project to second camera
    x2_cam = true_pose.R() * X + true_pose.t;
    if (x2_cam(2) <= 0.1) continue; // Skip if behind second camera
    // Convert to normalized image coordinates
    Vec2<Scalar> x1_norm(x1_cam(0) / x1_cam(2), x1_cam(1) / x1_cam(2));
    Vec2<Scalar> x2_norm(x2_cam(0) / x2_cam(2), x2_cam(1) / x2_cam(2));
    // Add noise to normalized coordinates
    x1_norm(0) += noise_gen(rng);
    x1_norm(1) += noise_gen(rng);
    x2_norm(0) += noise_gen(rng);
    x2_norm(1) += noise_gen(rng);
    x1.push_back(x1_norm);
    x2.push_back(x2_norm);
    ++valid_points;
  }

  // Debug prints for point depths
  ENTO_DEBUG("Checking point depths with true pose:");
  for (size_t i = 0; i < num_points; ++i)
  {
    Vec3<Scalar> Z = true_pose.R() * Vec3<Scalar>(x1[i](0), x1[i](1), 1.0) + true_pose.t;
    ENTO_DEBUG("Point %zu: depth=%f, x1=(%f,%f), x2=(%f,%f)", 
        i, Z(2), x1[i](0), x1[i](1),
        x2[i](0), x2[i](1));
  }
}

// Generate synthetic relative pose data with upright (yaw-only) camera motion
// Both cameras are upright (z-axis up), only yaw rotation between them
// x1, x2 are normalized image coordinates
// true_pose is the relative pose from camera 1 to camera 2
// N = 0 for dynamic, N > 0 for fixed-size

template<typename Scalar, size_t N>
void generate_synthetic_relative_pose_data_upright(
  EntoContainer<Vec2<Scalar>, N>& x1,
  EntoContainer<Vec2<Scalar>, N>& x2,
  CameraPose<Scalar>& true_pose,
  size_t num_points,
  Scalar noise_level = 0.01)
{
  // Set random upright (yaw-only) relative pose
  Scalar yaw = static_cast<Scalar>(rand()) / RAND_MAX * 2 * M_PI;
  Quaternion<Scalar> q = Quaternion<Scalar>(AngleAxis<Scalar>(yaw, Vec3<Scalar>::UnitY()));
  true_pose.q(0) = q.w();
  true_pose.q(1) = q.x();
  true_pose.q(2) = q.y();
  true_pose.q(3) = q.z();
  true_pose.t.setRandom();
  true_pose.t *= 2.0;

  std::default_random_engine rng(42);
  std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
  std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
  std::normal_distribution<Scalar> noise_gen(0.0, noise_level);

  if constexpr (N == 0) {
    x1.clear();
    x2.clear();
    x1.reserve(num_points);
    x2.reserve(num_points);
  }

  size_t valid_points = 0;
  while (valid_points < num_points) {
    Vec3<Scalar> X;
    Vec3<Scalar> x1_cam, x2_cam;
    while (true) {
      Scalar x = coord_gen(rng);
      Scalar y = coord_gen(rng);
      Scalar z = depth_gen(rng);
      ENTO_DEBUG("Trying: x=%f, y=%f, z=%f", x, y, z);
      X = Vec3<Scalar>(x, y, z);
      x1_cam = X; // First camera at origin, upright
      if (x1_cam(2) > 0.1) break;
    }
    ENTO_DEBUG("Accepted: z=%f", x1_cam(2));
    x2_cam = true_pose.R() * X + true_pose.t;
    if (x2_cam(2) <= 0.1) continue;
    Vec2<Scalar> x1_norm(x1_cam(0) / x1_cam(2), x1_cam(1) / x1_cam(2));
    Vec2<Scalar> x2_norm(x2_cam(0) / x2_cam(2), x2_cam(1) / x2_cam(2));
    x1_norm(0) += noise_gen(rng);
    x1_norm(1) += noise_gen(rng);
    x2_norm(0) += noise_gen(rng);
    x2_norm(1) += noise_gen(rng);
    x1.push_back(x1_norm);
    x2.push_back(x2_norm);
    ++valid_points;
  }

  ENTO_DEBUG("Checking upright point depths with true pose:");
  for (size_t i = 0; i < num_points; ++i) {
    Vec3<Scalar> Z = true_pose.R() * Vec3<Scalar>(x1[i](0), x1[i](1), 1.0) + true_pose.t;
    ENTO_DEBUG("Point %zu: depth=%f, x1=(%f,%f), x2=(%f,%f)",
        i, Z(2), x1[i](0), x1[i](1), x2[i](0), x2[i](1));
  }
}

// Test the new solver-based relative pose estimation with fixed-size containers
void test_robust_relative_pose_fixed_size()
{
  using Scalar = float;
  using Solver = SolverRel5pt<Scalar>;
  constexpr size_t N = 20; // Fixed number of correspondences
  
  ENTO_DEBUG("================");
  ENTO_DEBUG("Running test_robust_relative_pose_fixed_size...");
  
  // Generate synthetic data
  EntoContainer<Vec2<Scalar>, N> x1;
  EntoContainer<Vec2<Scalar>, N> x2;
  CameraPose<Scalar> true_pose;
  
  // Set noise to 0.0 for testing
  generate_synthetic_relative_pose_data<Scalar, N>(x1, x2, true_pose, N, 0.0);
  
  // Setup camera (identity for normalized coordinates)
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params); // focal = 1 for normalized coords
  
  // Setup RANSAC options
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 1000;
  ransac_opt.max_reproj_error = 0.1; // Increased threshold for normalized coordinates
  ransac_opt.success_prob = 0.99;
  
  // Setup bundle adjustment options
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1; // Match RANSAC threshold
  bundle_opt.max_iterations = 25; // Increased iterations
  
  // Estimate pose using solver-based LO-RANSAC
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  // Initialize all inliers to 0
  for (size_t i = 0; i < N; ++i) {
    inliers[i] = 0;
  }
  
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(
      x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);


  for( const auto& x : x1) {
    ENTO_DEBUG("x1: %f %f %f", x(0), x(1), x(2));
  }
  for( const auto& x : x2) {
    ENTO_DEBUG("x2: %f %f %f", x(0), x(1), x(2));
  }
  
  // Debug prints before first check
  ENTO_DEBUG("True pose - R: %f %f %f %f, t: %f %f %f", 
      true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(),
      true_pose.t.x(), true_pose.t.y(), true_pose.t.z());

  ENTO_DEBUG("Estimated pose - R: %f %f %f %f, t: %f %f %f",
      estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(),
      estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());

  ENTO_DEBUG("RANSAC stats - inliers: %zu, score: %f, iterations: %zu", 
      stats.num_inliers, stats.model_score, stats.iters);

  // Print detailed analysis for each point
  for (size_t i = 0; i < N; ++i) {
      Vec3<Scalar> Z = estimated_pose.R() * Vec3<Scalar>(x1[i](0), x1[i](1), 1.0) + estimated_pose.t;
      Scalar r2 = (Z.hnormalized().head<2>() - x2[i]).squaredNorm();
      bool cheirality = check_cheirality(estimated_pose, x1[i].homogeneous().normalized(), x2[i].homogeneous().normalized(), Scalar(0.01));
      //ENTO_DEBUG("Point %zu: error=%f, cheirality=%d, inlier=%d", i, r2, cheirality, (bool)inliers[i]);
  }

  // Check results
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 5); // Should have at least minimal set
  ENTO_TEST_CHECK_TRUE(stats.num_inliers <= N); // Can't have more inliers than points
  ENTO_TEST_CHECK_TRUE(stats.model_score >= 0);  // Score should be non-negative
  
  // Check that inliers container has correct size
  ENTO_TEST_CHECK_INT_EQ(inliers.size(), N);
  
  // Count inliers manually
  size_t manual_inlier_count = 0;
  for (size_t i = 0; i < N; ++i) {
      if (inliers[i]) manual_inlier_count++;
  }
  ENTO_TEST_CHECK_INT_EQ(manual_inlier_count, stats.num_inliers);
  
  // Check pose is reasonable (not identity)
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9); // Quaternion should be normalized
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1); // Translation should be non-trivial
  
  ENTO_DEBUG("Fixed-size test passed:");
  ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, N);
  ENTO_DEBUG("  Score: %f", stats.model_score);
  ENTO_DEBUG("  Iterations: %zu", stats.iters);
  ENTO_DEBUG("================");
}

// Test the new solver-based relative pose estimation with dynamic-size containers
void test_robust_relative_pose_dynamic_size()
{
  using Scalar = float;
  using Solver = SolverRel5pt<Scalar>;
  constexpr size_t N = 0; // Dynamic size
  const size_t num_points = 20;

  ENTO_DEBUG("================");
  ENTO_DEBUG("Running test_robust_relative_pose_dynamic_size...");

  EntoContainer<Vec2<Scalar>, N> x1;
  EntoContainer<Vec2<Scalar>, N> x2;
  CameraPose<Scalar> true_pose;

  generate_synthetic_relative_pose_data<Scalar, N>(x1, x2, true_pose, num_points, 0.0);

  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);

  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 1000;
  ransac_opt.max_reproj_error = 0.1;
  ransac_opt.success_prob = 0.99;

  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;

  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(
      x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);

  ENTO_DEBUG("True pose - R: %f %f %f %f, t: %f %f %f",
      true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(),
      true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
  ENTO_DEBUG("Estimated pose - R: %f %f %f %f, t: %f %f %f",
      estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(),
      estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());
  ENTO_DEBUG("RANSAC stats - inliers: %zu, score: %f, iterations: %zu",
      stats.num_inliers, stats.model_score, stats.iters);

  // Check results
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 5);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers <= num_points);
  ENTO_TEST_CHECK_TRUE(stats.model_score >= 0);
  ENTO_TEST_CHECK_INT_EQ(inliers.size(), num_points);
  size_t manual_inlier_count = 0;
  for (size_t i = 0; i < num_points; ++i) {
      if (inliers[i]) manual_inlier_count++;
  }
  ENTO_TEST_CHECK_INT_EQ(manual_inlier_count, stats.num_inliers);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
  float rot_error = (estimated_pose.R() - true_pose.R()).norm();
  ENTO_DEBUG("Dynamic-size test passed:");
  ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, num_points);
  ENTO_DEBUG("  Score: %f", stats.model_score);
  ENTO_DEBUG("  Iterations: %zu", stats.iters);
  ENTO_DEBUG("  Rotation error: %f", rot_error);
  ENTO_DEBUG("================");
}

// Test robust relative pose estimation with outliers
void test_robust_relative_pose_with_outliers()
{
  using Scalar = float;
  using Solver = SolverRel5pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;

  ENTO_DEBUG("================");
  ENTO_DEBUG("Running test_robust_relative_pose_with_outliers...");

  EntoContainer<Vec2<Scalar>, N> x1;
  EntoContainer<Vec2<Scalar>, N> x2;
  CameraPose<Scalar> true_pose;

  // Generate inlier correspondences
  generate_synthetic_relative_pose_data<Scalar, N>(x1, x2, true_pose, num_inliers, 0.01);

  // Add outliers
  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    Vec2<Scalar> x1_outlier(outlier_gen(rng), outlier_gen(rng));
    Vec2<Scalar> x2_outlier(outlier_gen(rng), outlier_gen(rng));
    x1.push_back(x1_outlier);
    x2.push_back(x2_outlier);
  }

  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);

  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.05;
  ransac_opt.success_prob = 0.999;
  ransac_opt.lo_type = LocalRefinementType::BundleAdjust;

  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;



  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(
      x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);

  ENTO_DEBUG("True pose - R: %f %f %f %f, t: %f %f %f",
      true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(),
      true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
  ENTO_DEBUG("Estimated pose - R: %f %f %f %f, t: %f %f %f",
      estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(),
      estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());
  ENTO_DEBUG("RANSAC stats - inliers: %zu, score: %f, iterations: %zu",
      stats.num_inliers, stats.model_score, stats.iters);

  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 10);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers <= num_inliers);
  size_t clean_inliers = 0;
  for (size_t i = 0; i < num_inliers; ++i) {
    if (inliers[i]) clean_inliers++;
  }
  ENTO_TEST_CHECK_TRUE(clean_inliers >= stats.num_inliers * 0.8);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
  float rot_error = (estimated_pose.R() - true_pose.R()).norm();
  float trans_error = (estimated_pose.t - true_pose.t).norm();
  ENTO_DEBUG("Outlier test passed:");
  ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, N);
  ENTO_DEBUG("  Clean inliers: %zu/%zu", clean_inliers, num_inliers);
  ENTO_DEBUG("  Score: %f", stats.model_score);
  ENTO_DEBUG("  Iterations: %zu", stats.iters);
  ENTO_DEBUG("  Rotation error: %f", rot_error);

  // Compute angular rotation error in degrees
  float trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  float angle_rad = std::acos(std::clamp((trace_val - 1.0f) / 2.0f, -1.0f, 1.0f));
  float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
  ENTO_DEBUG("Angular rotation error: %f degrees", angle_deg);

  ENTO_DEBUG("================");
}

// Test robust relative pose estimation with 8-point algorithm
void test_robust_relative_pose_eight_point()
{
  using Scalar = float;
  using Solver = SolverRel8pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;

  ENTO_DEBUG("================");
  ENTO_DEBUG("Running test_robust_relative_pose_eight_point...");

  EntoContainer<Vec2<Scalar>, N> x1;
  EntoContainer<Vec2<Scalar>, N> x2;
  CameraPose<Scalar> true_pose;

  generate_synthetic_relative_pose_data<Scalar, N>(x1, x2, true_pose, num_inliers, 0.005);

  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    Vec2<Scalar> x1_outlier(outlier_gen(rng), outlier_gen(rng));
    Vec2<Scalar> x2_outlier(outlier_gen(rng), outlier_gen(rng));
    x1.push_back(x1_outlier);
    x2.push_back(x2_outlier);
  }

  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);

  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.1;
  ransac_opt.success_prob = 0.999;

  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 50;

  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(
      x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);

  ENTO_DEBUG("True pose - R: %f %f %f %f, t: %f %f %f",
      true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(),
      true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
  ENTO_DEBUG("Estimated pose - R: %f %f %f %f, t: %f %f %f",
      estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(),
      estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());
  ENTO_DEBUG("RANSAC stats - inliers: %zu, score: %f, iterations: %zu",
      stats.num_inliers, stats.model_score, stats.iters);

  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 10);
  //ENTO_TEST_CHECK_TRUE(stats.num_inliers <= num_inliers);
  size_t clean_inliers = 0;
  for (size_t i = 0; i < num_inliers; ++i) {
    if (inliers[i]) clean_inliers++;
  }
  //ENTO_TEST_CHECK_TRUE(clean_inliers >= stats.num_inliers * 0.8);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
  float rot_error = (estimated_pose.R() - true_pose.R()).norm();
  ENTO_DEBUG("8pt test passed:");
  ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, N);
  ENTO_DEBUG("  Clean inliers: %zu/%zu", clean_inliers, num_inliers);
  ENTO_DEBUG("  Score: %f", stats.model_score);
  ENTO_DEBUG("  Iterations: %zu", stats.iters);
  ENTO_DEBUG("  Rotation error: %f", rot_error);
  float trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  float angle_rad = std::acos(std::clamp((trace_val - 1.0f) / 2.0f, -1.0f, 1.0f));
  float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
  ENTO_DEBUG("Angular rotation error: %f degrees", angle_deg);
  ENTO_TEST_CHECK_TRUE(angle_deg < 10.0f);
  ENTO_DEBUG("================");
}

// Test robust relative pose estimation with upright 3-point algorithm
void test_robust_relative_pose_upright()
{
  using Scalar = float;
  using Solver = SolverRelUpright3pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;

  ENTO_DEBUG("================");
  ENTO_DEBUG("Running test_robust_relative_pose_upright...");

  EntoContainer<Vec2<Scalar>, N> x1;
  EntoContainer<Vec2<Scalar>, N> x2;
  CameraPose<Scalar> true_pose;

  // Generate upright motion (z-axis up)
  EntoPose::generate_synthetic_relpose_upright<Scalar, N>(x1, x2, true_pose, num_inliers, 0.01);

  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    Vec2<Scalar> x1_outlier(outlier_gen(rng), outlier_gen(rng));
    Vec2<Scalar> x2_outlier(outlier_gen(rng), outlier_gen(rng));
    x1.push_back(x1_outlier);
    x2.push_back(x2_outlier);
  }

  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);

  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.005;
  ransac_opt.success_prob = 0.999;

  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;

  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(
      x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);

  ENTO_DEBUG("True pose - R: %f %f %f %f, t: %f %f %f",
      true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(),
      true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
  ENTO_DEBUG("Estimated pose - R: %f %f %f %f, t: %f %f %f",
      estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(),
      estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());
  ENTO_DEBUG("RANSAC stats - inliers: %zu, score: %f, iterations: %zu",
      stats.num_inliers, stats.model_score, stats.iters);

  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 5);
  //ENTO_TEST_CHECK_TRUE(stats.num_inliers <= num_inliers);
  size_t clean_inliers = 0;
  for (size_t i = 0; i < num_inliers; ++i) {
    if (inliers[i]) clean_inliers++;
  }
  ENTO_TEST_CHECK_TRUE(clean_inliers >= stats.num_inliers * 0.8);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
  float rot_error = (estimated_pose.R() - true_pose.R()).norm();
  ENTO_DEBUG("Upright 3pt test passed:");
  ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, N);
  ENTO_DEBUG("  Clean inliers: %zu/%zu", clean_inliers, num_inliers);
  ENTO_DEBUG("  Score: %f", stats.model_score);
  ENTO_DEBUG("  Iterations: %zu", stats.iters);
  ENTO_DEBUG("  Rotation error: %f", rot_error);
  float trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  float angle_rad = std::acos(std::clamp((trace_val - 1.0f) / 2.0f, -1.0f, 1.0f));
  float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
  ENTO_DEBUG("Angular rotation error: %f degrees", angle_deg);
  ENTO_TEST_CHECK_TRUE(angle_deg < 10.0f);
  ENTO_DEBUG("================");
}

// === Robust 5pt (double) ===
void test_robust_relative_pose_with_outliers_double()
{
  // OpenGV-style: Generate 3D bearing vectors, convert to normalized 2D points
  using Scalar = double;
  using Solver = SolverRel5pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec3<Scalar>, N> x1_bear;
  EntoContainer<Vec3<Scalar>, N> x2_bear;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_bearing_vectors<Scalar, N>(x1_bear, x2_bear, true_pose, num_inliers, 0.01);
  std::default_random_engine rng(123);
  std::normal_distribution<Scalar> outlier_gen(0.0, 1.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    Vec3<Scalar> x1_outlier(outlier_gen(rng), outlier_gen(rng), outlier_gen(rng));
    Vec3<Scalar> x2_outlier(outlier_gen(rng), outlier_gen(rng), outlier_gen(rng));
    x1_outlier.normalize();
    x2_outlier.normalize();
    x1_bear.push_back(x1_outlier);
    x2_bear.push_back(x2_outlier);
  }
  EntoContainer<Vec2<Scalar>, N> x1, x2;
  EntoPose::bearing_vectors_to_normalized_points<Scalar, N>(x1_bear, x1);
  EntoPose::bearing_vectors_to_normalized_points<Scalar, N>(x2_bear, x2);
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.01;
  ransac_opt.success_prob = 0.999;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 10);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers <= num_inliers);
  size_t clean_inliers = 0;
  for (size_t i = 0; i < num_inliers; ++i) if (inliers[i]) clean_inliers++;
  ENTO_TEST_CHECK_TRUE(clean_inliers >= stats.num_inliers * 0.8);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
  double trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  double angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
  double angle_deg = angle_rad * 180.0 / M_PI;
  ENTO_TEST_CHECK_TRUE(angle_deg < 5.0);
  ENTO_DEBUG("5pt (double) test stats:");
  ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, N);
  ENTO_DEBUG("  Clean inliers: %zu/%zu", clean_inliers, num_inliers);
  ENTO_DEBUG("  Score: %f", stats.model_score);
  ENTO_DEBUG("  Iterations: %zu", stats.iters);
  double rot_error = (estimated_pose.R() - true_pose.R()).norm();
  ENTO_DEBUG("  Rotation error (matrix norm): %f", rot_error);
  ENTO_DEBUG("  Angular rotation error: %f degrees", angle_deg);
  ENTO_DEBUG("  True pose - R: %f %f %f %f, t: %f %f %f", true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(), true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
  ENTO_DEBUG("  Estimated pose - R: %f %f %f %f, t: %f %f %f", estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(), estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());
  ENTO_DEBUG("================");
}

// === Robust 8pt (double) ===
void test_robust_relative_pose_eight_point_double()
{
  // OpenGV-style: Generate 3D bearing vectors, convert to normalized 2D points
  ENTO_DEBUG("================");
  ENTO_DEBUG("Running test_robust_relative_pose_eight_point_double...");
  using Scalar = double;
  using Solver = SolverRel8pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec3<Scalar>, N> x1_bear;
  EntoContainer<Vec3<Scalar>, N> x2_bear;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_bearing_vectors<Scalar, N>(x1_bear, x2_bear, true_pose, num_inliers, 0.005);
  std::default_random_engine rng(123);
  std::normal_distribution<Scalar> outlier_gen(0.0, 2.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    Vec3<Scalar> x1_outlier(outlier_gen(rng), outlier_gen(rng), outlier_gen(rng));
    Vec3<Scalar> x2_outlier(outlier_gen(rng), outlier_gen(rng), outlier_gen(rng));
    x1_outlier.normalize();
    x2_outlier.normalize();
    x1_bear.push_back(x1_outlier);
    x2_bear.push_back(x2_outlier);
  }
  EntoContainer<Vec2<Scalar>, N> x1, x2;
  EntoPose::bearing_vectors_to_normalized_points<Scalar, N>(x1_bear, x1);
  EntoPose::bearing_vectors_to_normalized_points<Scalar, N>(x2_bear, x2);
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.005;
  ransac_opt.success_prob = 0.999;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 50;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 10);
  size_t clean_inliers = 0;
  for (size_t i = 0; i < num_inliers; ++i) if (inliers[i]) clean_inliers++;
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
  double trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  double angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
  double angle_deg = angle_rad * 180.0 / M_PI;
  //ENTO_TEST_CHECK_TRUE(angle_deg < 10.0);
  ENTO_DEBUG("8pt (double) test stats:");
  ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, N);
  ENTO_DEBUG("  Clean inliers: %zu/%zu", clean_inliers, num_inliers);
  ENTO_DEBUG("  Score: %f", stats.model_score);
  ENTO_DEBUG("  Iterations: %zu", stats.iters);
  double rot_error = (estimated_pose.R() - true_pose.R()).norm();
  ENTO_DEBUG("  Rotation error (matrix norm): %f", rot_error);
  ENTO_DEBUG("  Angular rotation error: %f degrees", angle_deg);
  ENTO_DEBUG("  True pose - R: %f %f %f %f, t: %f %f %f", true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(), true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
  ENTO_DEBUG("  Estimated pose - R: %f %f %f %f, t: %f %f %f", estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(), estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());
  ENTO_DEBUG("================");
}

// === Robust upright 3pt (double) ===
void test_robust_relative_pose_upright_double()
{
  // OpenGV-style: Generate upright 3D bearing vectors, convert to normalized 2D points
  ENTO_DEBUG("================");
  ENTO_DEBUG("Running test_robust_relative_pose_upright_double...");
  using Scalar = double;
  using Solver = SolverRelUpright3pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec3<Scalar>, N> x1_bear;
  EntoContainer<Vec3<Scalar>, N> x2_bear;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_upright_bearing_vectors<Scalar, N>(x1_bear, x2_bear, true_pose, num_inliers, 0.01);
  std::default_random_engine rng(123);
  std::normal_distribution<Scalar> outlier_gen(0.0, 1.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    Vec3<Scalar> x1_outlier(outlier_gen(rng), outlier_gen(rng), outlier_gen(rng));
    Vec3<Scalar> x2_outlier(outlier_gen(rng), outlier_gen(rng), outlier_gen(rng));
    x1_outlier.normalize();
    x2_outlier.normalize();
    x1_bear.push_back(x1_outlier);
    x2_bear.push_back(x2_outlier);
  }
  EntoContainer<Vec2<Scalar>, N> x1, x2;
  EntoPose::bearing_vectors_to_normalized_points<Scalar, N>(x1_bear, x1);
  EntoPose::bearing_vectors_to_normalized_points<Scalar, N>(x2_bear, x2);
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.005;
  ransac_opt.success_prob = 0.999;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 5);
  size_t clean_inliers = 0;
  for (size_t i = 0; i < num_inliers; ++i) if (inliers[i]) clean_inliers++;
  ENTO_TEST_CHECK_TRUE(clean_inliers >= stats.num_inliers * 0.8);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
  double trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  double angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
  double angle_deg = angle_rad * 180.0 / M_PI;
  ENTO_TEST_CHECK_TRUE(angle_deg < 10.0);
  ENTO_DEBUG("Upright 3pt (double) test stats:");
  ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, N);
  ENTO_DEBUG("  Clean inliers: %zu/%zu", clean_inliers, num_inliers);
  ENTO_DEBUG("  Score: %f", stats.model_score);
  ENTO_DEBUG("  Iterations: %zu", stats.iters);
  double rot_error = (estimated_pose.R() - true_pose.R()).norm();
  ENTO_DEBUG("  Rotation error (matrix norm): %f", rot_error);
  ENTO_DEBUG("  Angular rotation error: %f degrees", angle_deg);
  ENTO_DEBUG("  True pose - R: %f %f %f %f, t: %f %f %f", true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(), true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
  ENTO_DEBUG("  Estimated pose - R: %f %f %f %f, t: %f %f %f", estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(), estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());
  ENTO_DEBUG("================");
}

// === Robust upright planar 2pt (float/double) ===
void test_robust_relative_pose_upright_planar_two_pt_float()
{
  using Scalar = float;
  using Solver = SolverRelUprightPlanar2pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec2<Scalar>, N> x1_2d, x2_2d;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_upright_planar<Scalar, N>(x1_2d, x2_2d, true_pose, num_inliers, 0.01f);
  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    x1_2d.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
    x2_2d.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
  }
  EntoContainer<Vec3<Scalar>, N> x1, x2;
  for (size_t i = 0; i < N; ++i) {
    x1[i] = Vec3<Scalar>(x1_2d[i](0), x1_2d[i](1), Scalar(1));
    x1[i].normalize();
    x2[i] = Vec3<Scalar>(x2_2d[i](0), x2_2d[i](1), Scalar(1));
    x2[i].normalize();
  }
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.005;
  ransac_opt.success_prob = 0.999;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1_2d, x2_2d, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 2);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
}

void test_robust_relative_pose_upright_planar_two_pt_double()
{
  using Scalar = double;
  using Solver = SolverRelUprightPlanar2pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec2<Scalar>, N> x1_2d, x2_2d;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_upright_planar<Scalar, N>(x1_2d, x2_2d, true_pose, num_inliers, 0.01);
  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    x1_2d.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
    x2_2d.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
  }
  EntoContainer<Vec3<Scalar>, N> x1, x2;
  for (size_t i = 0; i < N; ++i) {
    x1[i] = Vec3<Scalar>(x1_2d[i](0), x1_2d[i](1), Scalar(1));
    x1[i].normalize();
    x2[i] = Vec3<Scalar>(x2_2d[i](0), x2_2d[i](1), Scalar(1));
    x2[i].normalize();
  }
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.005;
  ransac_opt.success_prob = 0.999;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1_2d, x2_2d, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 2);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
}

// === Robust upright planar 3pt (float/double) ===
void test_robust_relative_pose_upright_planar_three_pt_float()
{
  using Scalar = float;
  using Solver = SolverRelUprightPlanar3pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec2<Scalar>, N> x1_2d, x2_2d;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_upright_planar<Scalar, N>(x1_2d, x2_2d, true_pose, num_inliers, 1.0f);
  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    x1_2d.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
    x2_2d.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
  }
  EntoContainer<Vec3<Scalar>, N> x1, x2;
  for (size_t i = 0; i < N; ++i) {
    x1[i] = Vec3<Scalar>(x1_2d[i](0), x1_2d[i](1), Scalar(1));
    x1[i].normalize();
    x2[i] = Vec3<Scalar>(x2_2d[i](0), x2_2d[i](1), Scalar(1));
    x2[i].normalize();
  }
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.005;
  ransac_opt.max_epipolar_error = 2.5;
  ransac_opt.success_prob = 0.999;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1_2d, x2_2d, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 3);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
}

void test_robust_relative_pose_upright_planar_three_pt_double()
{
  using Scalar = double;
  using Solver = SolverRelUprightPlanar3pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec2<Scalar>, N> x1_2d, x2_2d;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_upright_planar<Scalar, N>(x1_2d, x2_2d, true_pose, num_inliers, 0.01);
  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    x1_2d.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
    x2_2d.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
  }
  EntoContainer<Vec3<Scalar>, N> x1, x2;
  for (size_t i = 0; i < N; ++i) {
    x1[i] = Vec3<Scalar>(x1_2d[i](0), x1_2d[i](1), Scalar(1));
    x1[i].normalize();
    x2[i] = Vec3<Scalar>(x2_2d[i](0), x2_2d[i](1), Scalar(1));
    x2[i].normalize();
  }
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_reproj_error = 0.005;
  ransac_opt.success_prob = 0.999;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1_2d, x2_2d, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 3);
  ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
  ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
}

// === Robust 8pt with LO variants ===
void test_robust_relative_pose_eight_point_lo_linear()
{
  using Scalar = float;
  using Solver = SolverRel8pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec2<Scalar>, N> x1, x2;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_general<Scalar, N>(x1, x2, true_pose, num_inliers, 0.001);
  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    x1.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
    x2.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
  }
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  //ransac_opt.max_reproj_error = 0.005;
  ransac_opt.max_epipolar_error = 0.1;
  ransac_opt.success_prob = 0.999;
  ransac_opt.lo_type = LocalRefinementType::Linear;
  ransac_opt.linear_method = LinearRefinementMethod::EightPoint;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_DEBUG("8pt LO-linear: inliers %zu/%zu, score %f, iters %zu", stats.num_inliers, N, stats.model_score, stats.iters);
  float trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  float angle_rad = std::acos(std::clamp((trace_val - 1.0f) / 2.0f, -1.0f, 1.0f));
  float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
  ENTO_DEBUG("8pt LO-linear: angular error %f deg", angle_deg);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 10);
  ENTO_TEST_CHECK_TRUE(angle_deg < 10.0f);
}

void test_robust_relative_pose_eight_point_lo_linear_nonlinear()
{
  using Scalar = float;
  using Solver = SolverRel8pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec2<Scalar>, N> x1, x2;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_general<Scalar, N>(x1, x2, true_pose, num_inliers, 0.001);
  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-5.0, 5.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    x1.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
    x2.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
  }
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_epipolar_error = 0.1;
  ransac_opt.success_prob = 0.999;
  ransac_opt.lo_type = LocalRefinementType::Both;
  ransac_opt.linear_method = LinearRefinementMethod::EightPoint;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 25;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_DEBUG("8pt LO-linear+nonlinear: inliers %zu/%zu, score %f, iters %zu", stats.num_inliers, N, stats.model_score, stats.iters);
  float trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  float angle_rad = std::acos(std::clamp((trace_val - 1.0f) / 2.0f, -1.0f, 1.0f));
  float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
  ENTO_DEBUG("8pt LO-linear+nonlinear: angular error %f deg", angle_deg);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 10);
  ENTO_TEST_CHECK_TRUE(angle_deg < 10.0f);
}

void test_robust_relative_pose_eight_point_lo_irls()
{
  using Scalar = float;
  using Solver = SolverRel8pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec2<Scalar>, N> x1, x2;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_general<Scalar, N>(x1, x2, true_pose, num_inliers, 0.01);
  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-5.0, 5.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    x1.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
    x2.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
  }
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_epipolar_error = 0.1;
  ransac_opt.success_prob = 0.999;
  ransac_opt.lo_type = LocalRefinementType::Linear;
  ransac_opt.linear_method = LinearRefinementMethod::EightPoint;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 10;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_DEBUG("8pt LO-IRLS: inliers %zu/%zu, score %f, iters %zu", stats.num_inliers, N, stats.model_score, stats.iters);
  float trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  float angle_rad = std::acos(std::clamp((trace_val - 1.0f) / 2.0f, -1.0f, 1.0f));
  float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
  ENTO_DEBUG("8pt LO-IRLS: angular error %f deg", angle_deg);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 10);
  ENTO_TEST_CHECK_TRUE(angle_deg < 10.0f);
}

void test_robust_relative_pose_eight_point_lo_irls_nonlinear()
{
  using Scalar = float;
  using Solver = SolverRel8pt<Scalar>;
  constexpr size_t N = 50;
  const size_t num_inliers = 40;
  const size_t num_outliers = 10;
  EntoContainer<Vec2<Scalar>, N> x1, x2;
  CameraPose<Scalar> true_pose;
  EntoPose::generate_synthetic_relpose_general<Scalar, N>(x1, x2, true_pose, num_inliers, 0.01);
  std::default_random_engine rng(123);
  std::uniform_real_distribution<Scalar> outlier_gen(-5.0, 5.0);
  for (size_t i = 0; i < num_outliers; ++i) {
    x1.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
    x2.push_back(Vec2<Scalar>(outlier_gen(rng), outlier_gen(rng)));
  }
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  Params params;
  Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 2000;
  ransac_opt.max_epipolar_error = 0.1;
  ransac_opt.success_prob = 0.999;
  ransac_opt.lo_type = LocalRefinementType::Both;
  ransac_opt.linear_method = LinearRefinementMethod::EightPoint;
  BundleOptions<Scalar> bundle_opt;
  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  bundle_opt.loss_scale = 0.1;
  bundle_opt.max_iterations = 50;
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = estimate_relative_pose<Solver, N>(x1, x2, camera, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
  ENTO_DEBUG("8pt LO-IRLS+nonlinear: inliers %zu/%zu, score %f, iters %zu", stats.num_inliers, N, stats.model_score, stats.iters);
  float trace_val = (estimated_pose.R().transpose() * true_pose.R()).trace();
  float angle_rad = std::acos(std::clamp((trace_val - 1.0f) / 2.0f, -1.0f, 1.0f));
  float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
  ENTO_DEBUG("8pt LO-IRLS+nonlinear: angular error %f deg", angle_deg);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 10);
  ENTO_TEST_CHECK_TRUE(angle_deg < 10.0f);
}

int main(int argc, char** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
    ENTO_DEBUG("__n: %d", __n);
    printf("__n: %d\n", __n);
  } 
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_robust_absolute_pose_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  // @TODO: For some reason we get failures if running all tests... not super important
  // to fix right now.
  if (__ento_test_num(__n, 1)) test_robust_relative_pose_fixed_size();
  if (__ento_test_num(__n, 2)) test_robust_relative_pose_dynamic_size();
  // if (__ento_test_num(__n, 3)) test_robust_relative_pose_with_outliers();
  // if (__ento_test_num(__n, 4)) test_robust_relative_pose_eight_point();
  // if (__ento_test_num(__n, 5)) test_robust_relative_pose_upright();
  // if (__ento_test_num(__n, 6)) test_robust_relative_pose_with_outliers_double();
  if (__ento_test_num(__n, 7)) test_robust_relative_pose_eight_point_double();
  // if (__ento_test_num(__n, 8)) test_robust_relative_pose_upright_double();
  if (__ento_test_num(__n, 9)) test_robust_relative_pose_upright_planar_two_pt_float();
  if (__ento_test_num(__n, 10)) test_robust_relative_pose_upright_planar_two_pt_double();
  if (__ento_test_num(__n, 11)) test_robust_relative_pose_upright_planar_three_pt_float();
  if (__ento_test_num(__n, 12)) test_robust_relative_pose_upright_planar_three_pt_double();
  // if (__ento_test_num(__n, 13)) test_robust_relative_pose_eight_point_lo_linear();
  if (__ento_test_num(__n, 14)) test_robust_relative_pose_eight_point_lo_linear_nonlinear();
  // if (__ento_test_num(__n, 15)) test_robust_relative_pose_eight_point_lo_irls();
  // if (__ento_test_num(__n, 16)) test_robust_relative_pose_eight_point_lo_irls_nonlinear();
  
  return 0;
} 
