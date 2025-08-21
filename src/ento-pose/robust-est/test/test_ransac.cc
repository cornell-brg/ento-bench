#include <stdlib.h>
#include <cstdio>
#include <limits>
#include <random>
#include <Eigen/Dense>

#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/containers.h>

#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/ransac.h>
#include <ento-pose/robust-est/absolute.h>
#include <ento-pose/robust-est/relative.h>

using namespace std;
using namespace Eigen;
using namespace EntoPose;
using namespace EntoUtil;

// Simple synthetic data generation for testing
template<typename Scalar, size_t N>
void generate_synthetic_absolute_pose_data(
    EntoContainer<Vec2<Scalar>, N>& points2D,
    EntoContainer<Vec3<Scalar>, N>& points3D,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level,
    Scalar outlier_ratio)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<Scalar> coord_gen(-2.0, 2.0);
  std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
  std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
  std::uniform_real_distribution<Scalar> outlier_gen(-5.0, 5.0);
  std::uniform_real_distribution<Scalar> prob_gen(0.0, 1.0);
  
  // Generate random true pose
  true_pose.q << 1.0, 0.1, 0.05, 0.02;
  true_pose.q.normalize();
  true_pose.t << 0.5, 0.3, 0.1;
  
  points2D.clear();
  points3D.clear();
  
  // Reserve space for dynamic containers
  if constexpr (N == 0) {
    points2D.reserve(num_points);
    points3D.reserve(num_points);
  }
  
  size_t num_outliers = static_cast<size_t>(num_points * outlier_ratio);
  
  for (size_t i = 0; i < num_points; ++i) {
    if (i < num_outliers) {
      // Generate outliers
      Vec2<Scalar> x2d(outlier_gen(gen), outlier_gen(gen));
      Vec3<Scalar> X3d(outlier_gen(gen), outlier_gen(gen), outlier_gen(gen));
      points2D.push_back(x2d);
      points3D.push_back(X3d);
    } else {
      // Generate inliers
      Vec3<Scalar> X3d(coord_gen(gen), coord_gen(gen), depth_gen(gen));
      Vec3<Scalar> x3d_cam = true_pose.R() * X3d + true_pose.t;
      Vec2<Scalar> x2d(x3d_cam(0) / x3d_cam(2), x3d_cam(1) / x3d_cam(2));
      
      // Add noise
      x2d(0) += noise_gen(gen);
      x2d(1) += noise_gen(gen);
      
      points2D.push_back(x2d);
      points3D.push_back(X3d);
    }
  }
}

template<typename Scalar, size_t N>
void generate_synthetic_relative_pose_data(
    EntoContainer<Vec2<Scalar>, N>& points1,
    EntoContainer<Vec2<Scalar>, N>& points2,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level,
    Scalar outlier_ratio)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
  std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
  std::normal_distribution<Scalar> noise_gen(0.0, noise_level);
  std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
  
  // Generate random relative pose
  true_pose.q << 1.0, 0.1, 0.05, 0.02;
  true_pose.q.normalize();
  true_pose.t << 0.2, 0.1, 0.05;
  true_pose.t.normalize(); // Relative pose translation is up to scale
  
  points1.clear();
  points2.clear();
  
  size_t num_outliers = static_cast<size_t>(num_points * outlier_ratio);
  
  for (size_t i = 0; i < num_points; ++i) {
    if (i < num_outliers) {
      // Generate outliers
      Vec2<Scalar> x1(outlier_gen(gen), outlier_gen(gen));
      Vec2<Scalar> x2(outlier_gen(gen), outlier_gen(gen));
      points1.push_back(x1);
      points2.push_back(x2);
    } else {
      // Generate inliers
      Vec3<Scalar> X(coord_gen(gen), coord_gen(gen), depth_gen(gen));
      
      // Project to first camera (identity)
      Vec2<Scalar> x1(X(0) / X(2), X(1) / X(2));
      
      // Project to second camera
      Vec3<Scalar> X2 = true_pose.R() * X + true_pose.t;
      Vec2<Scalar> x2(X2(0) / X2(2), X2(1) / X2(2));
      
      // Add noise
      x1(0) += noise_gen(gen);
      x1(1) += noise_gen(gen);
      x2(0) += noise_gen(gen);
      x2(1) += noise_gen(gen);
      
      points1.push_back(x1);
      points2.push_back(x2);
    }
  }
}

// Test with dynamic containers (N=0)
void test_ransac_pnp_basic()
{
  using Scalar = float;
  constexpr Scalar tol = 1e-1;  // Relaxed tolerance for synthetic data
  constexpr size_t num_points = 50;  // Total points
  constexpr size_t K = 3;            // P3P sample size
  constexpr size_t N = 0;            // Dynamic containers
  
  ENTO_DEBUG("================\\n");
  ENTO_DEBUG("Running test_ransac_pnp_basic...");
  
  // Generate synthetic data with outliers - use dynamic containers
  EntoContainer<Vec2<Scalar>, N> points2D;
  EntoContainer<Vec3<Scalar>, N> points3D;
  CameraPose<Scalar> true_pose;
  
  generate_synthetic_absolute_pose_data<Scalar, N>(points2D, points3D, true_pose, num_points, Scalar(0.01), Scalar(0.2));
  
  // Setup RANSAC options
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 1000;
  ransac_opt.max_reproj_error = 2.0;
  ransac_opt.refinement_type = RefinementType::BUNDLE_ADJUSTMENT;
  
  // Run RANSAC with explicit template parameters
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = ransac_pnp<Scalar, N, K>(
      points2D, points3D, ransac_opt, &estimated_pose, &inliers);
  
  // Check results
  ENTO_TEST_CHECK_TRUE(stats.iterations > 0);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= K);
  ENTO_TEST_CHECK_TRUE(inliers.size() == points2D.size());
  
  // Check pose accuracy (relaxed tolerance due to noise and outliers)
  Scalar rotation_error = (estimated_pose.R() - true_pose.R()).norm();
  Scalar translation_error = (estimated_pose.t - true_pose.t).norm();
  ENTO_TEST_CHECK_TRUE(rotation_error < 1.0);  // Relaxed
  ENTO_TEST_CHECK_TRUE(translation_error < 1.0);  // Relaxed
  
  ENTO_DEBUG("RANSAC PnP basic test passed");
  ENTO_DEBUG("================\\n");
}

// Test with bounded containers (N=100)
void test_ransac_pnp_bounded()
{
  using Scalar = float;
  constexpr Scalar tol = 1e-1;
  constexpr size_t num_points = 50;  // Total points
  constexpr size_t K = 3;            // P3P sample size
  constexpr size_t N = 100;          // Bounded containers
  
  ENTO_DEBUG("================\\n");
  ENTO_DEBUG("Running test_ransac_pnp_bounded...");
  
  // Generate synthetic data with outliers - use bounded containers
  EntoContainer<Vec2<Scalar>, N> points2D;
  EntoContainer<Vec3<Scalar>, N> points3D;
  CameraPose<Scalar> true_pose;
  
  generate_synthetic_absolute_pose_data<Scalar, N>(points2D, points3D, true_pose, num_points, Scalar(0.01), Scalar(0.2));
  
  // Setup RANSAC options
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 1000;
  ransac_opt.max_reproj_error = 2.0;
  ransac_opt.refinement_type = RefinementType::LINEAR_SOLVE;
  
  // Run RANSAC with explicit template parameters
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  RansacStats<Scalar> stats = ransac_pnp<Scalar, N, K>(
      points2D, points3D, ransac_opt, &estimated_pose, &inliers);
  
  // Check results
  ENTO_TEST_CHECK_TRUE(stats.iterations > 0);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= K);
  ENTO_TEST_CHECK_TRUE(inliers.size() == points2D.size());
  
  ENTO_DEBUG("RANSAC PnP bounded test passed");
  ENTO_DEBUG("================\\n");
}

void test_lo_ransac_pnp_bundle_adjustment()
{
  using Scalar = float;
  constexpr Scalar tol = 1e-1;
  constexpr size_t num_points = 50;  // Total points
  constexpr size_t N = 0;            // Dynamic containers
  
  ENTO_DEBUG("================\\n");
  ENTO_DEBUG("Running test_lo_ransac_pnp_bundle_adjustment...");
  
  // Generate synthetic data with outliers - use dynamic containers
  EntoContainer<Vec2<Scalar>, N> points2D;
  EntoContainer<Vec3<Scalar>, N> points3D;
  CameraPose<Scalar> true_pose;
  
  generate_synthetic_absolute_pose_data<Scalar, N>(points2D, points3D, true_pose, num_points, Scalar(0.01), Scalar(0.3));
  
  // Setup LO-RANSAC options with bundle adjustment
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 1000;
  ransac_opt.min_iterations = 10;
  ransac_opt.success_prob = 0.99;
  ransac_opt.max_reproj_error = 2.0;
  ransac_opt.refinement_type = RefinementType::BUNDLE_ADJUSTMENT;
  
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, N> inliers;
  
  // Use default template parameters to avoid conflicts
  RansacStats<Scalar> stats = ransac_pnp<Scalar>(
    points2D, points3D, ransac_opt, &estimated_pose, &inliers);
  
  // Check results
  ENTO_TEST_CHECK_TRUE(stats.iterations > 0);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 3);  // At least P3P sample size
  ENTO_TEST_CHECK_TRUE(inliers.size() == points2D.size());
}

void test_lo_ransac_pnp_linear_solve()
{
  using Scalar = float;
  constexpr Scalar tol = 1e-1;
  constexpr size_t num_points = 50;  // Total points
  
  ENTO_DEBUG("================\\n");
  ENTO_DEBUG("Running test_lo_ransac_pnp_linear_solve...");
  
  // Generate synthetic data with outliers - use dynamic containers
  EntoContainer<Vec2<Scalar>, 0> points2D;
  EntoContainer<Vec3<Scalar>, 0> points3D;
  CameraPose<Scalar> true_pose;
  
  generate_synthetic_absolute_pose_data<Scalar, 0>(points2D, points3D, true_pose, num_points, Scalar(0.01), Scalar(0.3));
  
  // Setup LO-RANSAC options with linear solve
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 1000;
  ransac_opt.min_iterations = 10;
  ransac_opt.success_prob = 0.99;
  ransac_opt.max_reproj_error = 2.0;
  ransac_opt.refinement_type = RefinementType::LINEAR_SOLVE;
  
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, 0> inliers;
  
  // Use default template parameters to avoid conflicts
  RansacStats<Scalar> stats = ransac_pnp<Scalar>(
    points2D, points3D, ransac_opt, &estimated_pose, &inliers);
  
  // Check results
  ENTO_TEST_CHECK_TRUE(stats.iterations > 0);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 3);  // At least P3P sample size
  ENTO_TEST_CHECK_TRUE(inliers.size() == points2D.size());
}

void test_ransac_relative_pose()
{
  using Scalar = float;
  constexpr Scalar tol = 1e-1;
  constexpr size_t num_points = 50;  // Total points
  
  ENTO_DEBUG("================\\n");
  ENTO_DEBUG("Running test_ransac_relative_pose...");
  
  // Generate synthetic data with outliers - use dynamic containers
  EntoContainer<Vec2<Scalar>, 0> points1;
  EntoContainer<Vec2<Scalar>, 0> points2;
  CameraPose<Scalar> true_pose;
  
  generate_synthetic_relative_pose_data<Scalar, 0>(points1, points2, true_pose, num_points, Scalar(0.01), Scalar(0.3));
  
  // Setup RANSAC options
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 1000;
  ransac_opt.min_iterations = 10;
  ransac_opt.success_prob = 0.99;
  ransac_opt.max_epipolar_error = 2.0;
  
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, 0> inliers;
  
  // Use default template parameters to avoid conflicts
  RansacStats<Scalar> stats = EntoPose::ransac_relpose<Scalar>(
    points1, points2, ransac_opt, &estimated_pose, &inliers);
  
  // Check results
  ENTO_TEST_CHECK_TRUE(stats.iterations > 0);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 5);  // At least 5-point sample size
  ENTO_TEST_CHECK_TRUE(inliers.size() == points1.size());
}

void test_lo_ransac_relative_pose()
{
  using Scalar = float;
  constexpr Scalar tol = 1e-1;
  constexpr size_t num_points = 50;  // Total points
  
  ENTO_DEBUG("================\\n");
  ENTO_DEBUG("Running test_lo_ransac_relative_pose...");
  
  // Generate synthetic data with outliers - use dynamic containers
  EntoContainer<Vec2<Scalar>, 0> points1;
  EntoContainer<Vec2<Scalar>, 0> points2;
  CameraPose<Scalar> true_pose;
  
  generate_synthetic_relative_pose_data<Scalar, 0>(points1, points2, true_pose, num_points, Scalar(0.01), Scalar(0.3));
  
  // Setup LO-RANSAC options
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 1000;
  ransac_opt.min_iterations = 10;
  ransac_opt.success_prob = 0.99;
  ransac_opt.max_epipolar_error = 2.0;
  ransac_opt.refinement_type = RefinementType::BUNDLE_ADJUSTMENT;
  
  CameraPose<Scalar> estimated_pose;
  EntoContainer<uint8_t, 0> inliers;
  
  // Use default template parameters to avoid conflicts
  RansacStats<Scalar> stats = EntoPose::ransac_relpose<Scalar>(
    points1, points2, ransac_opt, &estimated_pose, &inliers);
  
  // Check results
  ENTO_TEST_CHECK_TRUE(stats.iterations > 0);
  ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 5);  // At least 5-point sample size
  ENTO_TEST_CHECK_TRUE(inliers.size() == points1.size());
}

void test_ransac_options_validation()
{
  using Scalar = float;
  
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_ransac_options_validation...");
  
  // Test default options
  RansacOptions<Scalar> default_opt;
  ENTO_TEST_CHECK_TRUE(default_opt.max_iterations > 0);
  ENTO_TEST_CHECK_TRUE(default_opt.max_reproj_error > 0);
  ENTO_TEST_CHECK_TRUE(default_opt.success_prob > 0 && default_opt.success_prob < 1);
  ENTO_TEST_CHECK_TRUE(default_opt.refinement_type == RefinementType::BUNDLE_ADJUSTMENT);
  
  // Test custom options
  RansacOptions<Scalar> custom_opt;
  custom_opt.max_iterations = 5000;
  custom_opt.max_reproj_error = 1.5;
  custom_opt.success_prob = 0.95;
  custom_opt.refinement_type = RefinementType::LINEAR_SOLVE;
  
  ENTO_TEST_CHECK_INT_EQ(custom_opt.max_iterations, 5000);
  ENTO_TEST_CHECK_FLOAT_EQ(custom_opt.max_reproj_error, 1.5);
  ENTO_TEST_CHECK_FLOAT_EQ(custom_opt.success_prob, 0.95);
  ENTO_TEST_CHECK_TRUE(custom_opt.refinement_type == RefinementType::LINEAR_SOLVE);
  
  ENTO_DEBUG("RANSAC options validation test passed");
  ENTO_DEBUG("================\n");
}

int main(int argc, char** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_ransac_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_ransac_pnp_basic();
  if (__ento_test_num(__n, 2)) test_ransac_pnp_bounded();
  if (__ento_test_num(__n, 3)) test_lo_ransac_pnp_bundle_adjustment();
  if (__ento_test_num(__n, 4)) test_lo_ransac_pnp_linear_solve();
  if (__ento_test_num(__n, 5)) test_ransac_relative_pose();
  if (__ento_test_num(__n, 6)) test_lo_ransac_relative_pose();
  if (__ento_test_num(__n, 7)) test_ransac_options_validation();

  return 0;
} 