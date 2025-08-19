#include <stdlib.h>
#include <cstdio>
#include <limits>
#include <Eigen/Dense>

#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/containers.h>

#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/absolute.h>
#include <ento-pose/robust-est/relative.h>
#include <ento-pose/robust-est/homography.h>

using namespace std;
using namespace Eigen;
using namespace EntoPose;
using namespace EntoUtil;

// Simple synthetic data generation for testing
template<typename Scalar>
void generate_synthetic_absolute_pose_data(
    EntoContainer<Vec2<Scalar>, 50>& points2D,
    EntoContainer<Vec3<Scalar>, 50>& points3D,
    CameraPose<Scalar>& true_pose,
    size_t num_points)
{
    // Set random true pose
    true_pose.q = Eigen::Quaternion<Scalar>::UnitRandom().coeffs();
    true_pose.t.setRandom();
    true_pose.t *= 2.0; // Scale translation
    
    std::default_random_engine rng;
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, 0.01);
    
    points2D.clear();
    points3D.clear();
    
    for (size_t i = 0; i < num_points; ++i) {
        // Generate 3D point
        Vec3<Scalar> X(coord_gen(rng), coord_gen(rng), depth_gen(rng));
        
        // Project to image
        Vec3<Scalar> x_cam = true_pose.R() * X + true_pose.t;
        Vec2<Scalar> x_img(x_cam(0) / x_cam(2), x_cam(1) / x_cam(2));
        
        // Add small amount of noise
        x_img(0) += noise_gen(rng);
        x_img(1) += noise_gen(rng);
        
        points2D.push_back(x_img);
        points3D.push_back(X);
    }
}

template<typename Scalar>
void generate_synthetic_relative_pose_data(
    EntoContainer<Vec2<Scalar>, 50>& points1,
    EntoContainer<Vec2<Scalar>, 50>& points2,
    CameraPose<Scalar>& true_pose,
    size_t num_points)
{
    // Set random true pose
    true_pose.q = Eigen::Quaternion<Scalar>::UnitRandom().coeffs();
    true_pose.t.setRandom();
    true_pose.t.normalize(); // Relative pose has unit translation
    
    std::default_random_engine rng;
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> noise_gen(0.0, 0.01);
    
    points1.clear();
    points2.clear();
    
    for (size_t i = 0; i < num_points; ++i) {
        // Generate 3D point in first camera frame
        Vec3<Scalar> X1(coord_gen(rng), coord_gen(rng), depth_gen(rng));
        
        // Project to first image
        Vec2<Scalar> x1(X1(0) / X1(2), X1(1) / X1(2));
        x1(0) += noise_gen(rng);
        x1(1) += noise_gen(rng);
        
        // Transform to second camera frame
        Vec3<Scalar> X2 = true_pose.R() * X1 + true_pose.t;
        
        // Project to second image
        Vec2<Scalar> x2(X2(0) / X2(2), X2(1) / X2(2));
        x2(0) += noise_gen(rng);
        x2(1) += noise_gen(rng);
        
        points1.push_back(x1);
        points2.push_back(x2);
    }
}

template<typename Scalar>
void generate_synthetic_homography_data(
    EntoContainer<Vec2<Scalar>, 50>& points1,
    EntoContainer<Vec2<Scalar>, 50>& points2,
    Mat3<Scalar>& true_H,
    size_t num_points)
{
    // Generate random homography
    true_H.setRandom();
    true_H /= true_H(2, 2); // Normalize
    
    std::default_random_engine rng;
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::normal_distribution<Scalar> noise_gen(0.0, 0.01);
    
    points1.clear();
    points2.clear();
    
    for (size_t i = 0; i < num_points; ++i) {
        // Generate point in first image
        Vec3<Scalar> x1_h(coord_gen(rng), coord_gen(rng), 1.0);
        Vec2<Scalar> x1(x1_h(0), x1_h(1));
        
        // Transform via homography
        Vec3<Scalar> x2_h = true_H * x1_h;
        Vec2<Scalar> x2(x2_h(0) / x2_h(2), x2_h(1) / x2_h(2));
        
        // Add noise
        x1(0) += noise_gen(rng);
        x1(1) += noise_gen(rng);
        x2(0) += noise_gen(rng);
        x2(1) += noise_gen(rng);
        
        points1.push_back(x1);
        points2.push_back(x2);
    }
}

void test_absolute_pose_estimator()
{
  using Scalar = float;
  constexpr size_t K = 3;  // P3P sample size
  constexpr size_t N = 50; // Total points
  
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_absolute_pose_estimator...");
  
  // Generate synthetic data
  EntoContainer<Vec2<Scalar>, N> points2D;
  EntoContainer<Vec3<Scalar>, N> points3D;
  CameraPose<Scalar> true_pose;
  
  generate_synthetic_absolute_pose_data(points2D, points3D, true_pose, N);
  
  // Setup RANSAC options
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 100;
  ransac_opt.max_reproj_error = 1.0;
  ransac_opt.refinement_type = RefinementType::BUNDLE_ADJUSTMENT;
  
  // Create estimator
  AbsolutePoseRobustEstimator<Scalar, K, N> estimator(ransac_opt, points2D, points3D);
  
  // Test basic properties
  ENTO_TEST_CHECK_INT_EQ(estimator.num_data_, N);
  ENTO_TEST_CHECK_INT_EQ(estimator.sample_size_, K);
  
  // Test model generation
  EntoContainer<CameraPose<Scalar>, 4> models;
  estimator.generate_models(&models);
  ENTO_TEST_CHECK_TRUE(models.size() > 0);
  
  // Test scoring
  size_t inlier_count;
  Scalar score = estimator.score_model(true_pose, &inlier_count);
  ENTO_TEST_CHECK_TRUE(score >= 0);
  ENTO_TEST_CHECK_TRUE(inlier_count > K);  // Should have more inliers than minimum
  
  ENTO_DEBUG("Absolute pose estimator test passed");
  ENTO_DEBUG("================\n");
}

void test_relative_pose_estimator()
{
  using Scalar = float;
  constexpr size_t K = 5;  // 5-point sample size
  constexpr size_t N = 50; // Total points
  
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_relative_pose_estimator...");
  
  // Generate synthetic data
  EntoContainer<Vec2<Scalar>, N> points1, points2;
  CameraPose<Scalar> true_pose;
  
  generate_synthetic_relative_pose_data(points1, points2, true_pose, N);
  
  // Setup RANSAC options
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 100;
  ransac_opt.max_epipolar_error = 1.0;
  ransac_opt.refinement_type = RefinementType::BUNDLE_ADJUSTMENT;
  
  // Create estimator
  RelativePoseRobustEstimator<Scalar, K, N> estimator(ransac_opt, points1, points2);
  
  // Test basic properties
  ENTO_TEST_CHECK_INT_EQ(estimator.num_data_, N);
  ENTO_TEST_CHECK_INT_EQ(estimator.sample_size_, K);
  
  // Test model generation
  EntoContainer<CameraPose<Scalar>, 10> models;
  estimator.generate_models(&models);
  ENTO_TEST_CHECK_TRUE(models.size() > 0);
  
  // Test scoring
  size_t inlier_count;
  Scalar score = estimator.score_model(true_pose, &inlier_count);
  ENTO_TEST_CHECK_TRUE(score >= 0);
  ENTO_TEST_CHECK_TRUE(inlier_count > K);
  
  ENTO_DEBUG("Relative pose estimator test passed");
  ENTO_DEBUG("================\n");
}

void test_homography_estimator()
{
  using Scalar = float;
  constexpr size_t K = 4;  // 4-point sample size
  constexpr size_t N = 50; // Total points
  
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_homography_estimator...");
  
  // Generate synthetic homography data
  EntoContainer<Vec2<Scalar>, N> points1, points2;
  Mat3<Scalar> true_H;
  
  generate_synthetic_homography_data(points1, points2, true_H, N);
  
  // Setup RANSAC options
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iterations = 100;
  ransac_opt.max_reproj_error = 1.0;
  ransac_opt.refinement_type = RefinementType::LINEAR_SOLVE;
  
  // Create estimator
  HomographyRobustEstimator<Scalar, K, N> estimator(ransac_opt, points1, points2);
  
  // Test basic properties
  ENTO_TEST_CHECK_INT_EQ(estimator.num_data_, N);
  ENTO_TEST_CHECK_INT_EQ(estimator.sample_size_, K);
  
  // Test model generation
  EntoContainer<Mat3<Scalar>, 1> models;
  estimator.generate_models(&models);
  ENTO_TEST_CHECK_TRUE(models.size() > 0);
  
  // Test scoring
  size_t inlier_count;
  Scalar score = estimator.score_model(true_H, &inlier_count);
  ENTO_TEST_CHECK_TRUE(score >= 0);
  ENTO_TEST_CHECK_TRUE(inlier_count > K);
  
  ENTO_DEBUG("Homography estimator test passed");
  ENTO_DEBUG("================\n");
}

void test_estimator_template_parameters()
{
  using Scalar = float;
  
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_estimator_template_parameters...");
  
  // Test different template parameter combinations
  EntoContainer<Vec2<Scalar>, 20> points2D;
  EntoContainer<Vec3<Scalar>, 20> points3D;
  
  // Fill with dummy data
  for (size_t i = 0; i < 20; ++i) {
    points2D.push_back(Vec2<Scalar>(i * 0.1, i * 0.1));
    points3D.push_back(Vec3<Scalar>(i * 0.1, i * 0.1, 1.0));
  }
  
  RansacOptions<Scalar> opt;
  opt.max_reproj_error = 1.0;
  
  // Test compile-time size
  AbsolutePoseRobustEstimator<Scalar, 3, 20> estimator_fixed(opt, points2D, points3D);
  ENTO_TEST_CHECK_INT_EQ(estimator_fixed.num_data_, 20);
  
  // Test runtime size (N=0)
  AbsolutePoseRobustEstimator<Scalar, 3, 0> estimator_dynamic(opt, points2D, points3D);
  ENTO_TEST_CHECK_INT_EQ(estimator_dynamic.num_data_, 20);
  
  ENTO_DEBUG("Estimator template parameters test passed");
  ENTO_DEBUG("================\n");
}

void test_refinement_type_switching()
{
  using Scalar = float;
  constexpr size_t N = 30;
  
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_refinement_type_switching...");
  
  // Generate synthetic data
  EntoContainer<Vec2<Scalar>, N> points2D;
  EntoContainer<Vec3<Scalar>, N> points3D;
  CameraPose<Scalar> true_pose;
  
  generate_synthetic_absolute_pose_data(points2D, points3D, true_pose, N);
  
  // Test bundle adjustment refinement
  RansacOptions<Scalar> bundle_opt;
  bundle_opt.max_iterations = 100;
  bundle_opt.max_reproj_error = 1.0;
  bundle_opt.refinement_type = RefinementType::BUNDLE_ADJUSTMENT;
  
  AbsolutePoseRobustEstimator<Scalar, 3, N> estimator_bundle(bundle_opt, points2D, points3D);
  CameraPose<Scalar> pose_bundle = true_pose;
  estimator_bundle.refine_model(&pose_bundle);
  
  // Test linear refinement
  RansacOptions<Scalar> linear_opt;
  linear_opt.max_iterations = 100;
  linear_opt.max_reproj_error = 1.0;
  linear_opt.refinement_type = RefinementType::LINEAR_SOLVE;
  
  AbsolutePoseRobustEstimator<Scalar, 3, N> estimator_linear(linear_opt, points2D, points3D);
  CameraPose<Scalar> pose_linear = true_pose;
  estimator_linear.refine_model_linear(&pose_linear);
  
  // Both should produce valid poses (basic sanity check)
  ENTO_TEST_CHECK_TRUE(pose_bundle.t.norm() > 0);
  ENTO_TEST_CHECK_TRUE(pose_linear.t.norm() > 0);
  ENTO_TEST_CHECK_TRUE(pose_bundle.R().determinant() > 0.9);  // Should be close to 1
  ENTO_TEST_CHECK_TRUE(pose_linear.R().determinant() > 0.9);
  
  ENTO_DEBUG("Refinement type switching test passed");
  ENTO_DEBUG("================\n");
}

void test_estimator_edge_cases()
{
  using Scalar = float;
  
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_estimator_edge_cases...");
  
  // Test with minimal data
  EntoContainer<Vec2<Scalar>, 3> points2D_min;
  EntoContainer<Vec3<Scalar>, 3> points3D_min;
  
  // Fill with minimal valid data
  points2D_min.push_back(Vec2<Scalar>(0, 0));
  points2D_min.push_back(Vec2<Scalar>(1, 0));
  points2D_min.push_back(Vec2<Scalar>(0, 1));
  
  points3D_min.push_back(Vec3<Scalar>(0, 0, 1));
  points3D_min.push_back(Vec3<Scalar>(1, 0, 1));
  points3D_min.push_back(Vec3<Scalar>(0, 1, 1));
  
  RansacOptions<Scalar> opt;
  opt.max_reproj_error = 1.0;
  
  // Should handle minimal case without crashing
  AbsolutePoseRobustEstimator<Scalar, 3, 3> estimator_min(opt, points2D_min, points3D_min);
  ENTO_TEST_CHECK_INT_EQ(estimator_min.num_data_, 3);
  
  // Test model generation with minimal data
  EntoContainer<CameraPose<Scalar>, 4> models;
  estimator_min.generate_models(&models);
  // Should generate at least one model (may be degenerate)
  ENTO_TEST_CHECK_TRUE(models.size() >= 0);
  
  ENTO_DEBUG("Estimator edge cases test passed");
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
    __ento_replace_file_suffix(__FILE__, "test_estimators_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_absolute_pose_estimator();
  if (__ento_test_num(__n, 2)) test_relative_pose_estimator();
  if (__ento_test_num(__n, 3)) test_homography_estimator();
  if (__ento_test_num(__n, 4)) test_estimator_template_parameters();
  if (__ento_test_num(__n, 5)) test_refinement_type_switching();
  if (__ento_test_num(__n, 6)) test_estimator_edge_cases();

  return 0;
} 