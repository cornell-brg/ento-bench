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

using namespace std;
using namespace Eigen;
using namespace EntoPose;
using namespace EntoUtil;

// Generate synthetic absolute pose data with known ground truth
template<typename Scalar, size_t N>
void generate_synthetic_absolute_pose_data(
    EntoContainer<Vec2<Scalar>, N>& points2D,
    EntoContainer<Vec3<Scalar>, N>& points3D,
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
        points2D.clear();
        points3D.clear();
        points2D.reserve(num_points);
        points3D.reserve(num_points);
    }
    
    for (size_t i = 0; i < num_points; ++i) {
        Vec3<Scalar> X;
        Vec3<Scalar> x_cam;
        Vec2<Scalar> x_norm;
        while (true) {
            // Generate 3D point
            X = Vec3<Scalar>(coord_gen(rng), coord_gen(rng), depth_gen(rng));
            // Project to normalized camera coordinates
            x_cam = true_pose.R() * X + true_pose.t;
            if (x_cam(2) > 0.1) break; // Accept only if in front of camera
        }
        x_norm = Vec2<Scalar>(x_cam(0) / x_cam(2), x_cam(1) / x_cam(2));
        // Add small amount of noise
        x_norm(0) += noise_gen(rng);
        x_norm(1) += noise_gen(rng);

        points2D.push_back(x_norm);
        points3D.push_back(X);
    }

    // Debug prints for point depths
    ENTO_DEBUG("Checking point depths with true pose:");
    for (size_t i = 0; i < num_points; ++i) {
        Vec3<Scalar> Z = true_pose.R() * points3D[i] + true_pose.t;
        ENTO_DEBUG("Point %zu: depth=%f, 2D=(%f,%f), 3D=(%f,%f,%f)", 
            i, Z(2), points2D[i](0), points2D[i](1), 
            points3D[i](0), points3D[i](1), points3D[i](2));
    }
}

// Test the new solver-based absolute pose estimation with fixed-size containers
void test_robust_absolute_pose_fixed_size()
{
    using Scalar = float;
    using Solver = SolverP3P<Scalar>;
    constexpr size_t N = 20; // Fixed number of correspondences
    
    ENTO_DEBUG("================");
    ENTO_DEBUG("Running test_robust_absolute_pose_fixed_size...");
    
    // Generate synthetic data
    EntoContainer<Vec2<Scalar>, N> points2D;
    EntoContainer<Vec3<Scalar>, N> points3D;
    CameraPose<Scalar> true_pose;
    
    generate_synthetic_absolute_pose_data<Scalar, N>(points2D, points3D, true_pose, N, 0.005);
    
    // Setup camera (identity for normalized coordinates)
    using CameraModel = IdentityCameraModel<Scalar>;
    using Params = std::array<Scalar, 0>;
    Params params;
    Camera<Scalar, CameraModel> camera(1.0, 1.0, params); // focal = 1 for normalized coords
    
    // Setup RANSAC options
    RansacOptions<Scalar> ransac_opt;
    ransac_opt.max_iters = 1000;
    ransac_opt.max_reproj_error = 3.0; // 3.0 pixels in normalized coordinates
    ransac_opt.success_prob = 0.99;
    
    // Setup bundle adjustment options
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.max_iterations = 10;
    
    // Estimate pose using solver-based LO-RANSAC
    CameraPose<Scalar> estimated_pose;
    EntoContainer<uint8_t, N> inliers;
    
    RansacStats<Scalar> stats = estimate_absolute_pose<Solver, N>(
        points2D, points3D, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
    
    // Debug prints before first check
    ENTO_DEBUG("True pose - R: %f %f %f %f, t: %f %f %f", 
        true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(),
        true_pose.t.x(), true_pose.t.y(), true_pose.t.z());

    ENTO_DEBUG("Estimated pose - R: %f %f %f %f, t: %f %f %f",
        estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(),
        estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());

    ENTO_DEBUG("RANSAC stats - inliers: %zu, score: %f, iterations: %zu", 
        stats.num_inliers, stats.model_score, stats.iters);

    // Print reprojection errors for each point
    for (size_t i = 0; i < N; ++i) {
        Vec3<Scalar> Z = estimated_pose.R() * points3D[i] + estimated_pose.t;
        Scalar r2 = (Z.hnormalized() - points2D[i]).squaredNorm();
        ENTO_DEBUG("Point %zu: error=%f, inlier=%d", i, r2, (bool)inliers[i]);
    }

    // Check results
    ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 3); // Should have at least minimal set
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

// Test the new solver-based absolute pose estimation with dynamic-size containers
void test_robust_absolute_pose_dynamic_size()
{
    using Scalar = float;  // Use same scalar type as fixed-size test
    using Solver = SolverP3P<Scalar>;
    constexpr size_t N = 0; // Dynamic size
    const size_t num_points = 20;  // Same number of points as fixed-size test
    
    ENTO_DEBUG("================");
    ENTO_DEBUG("Running test_robust_absolute_pose_dynamic_size...");
    
    // Generate synthetic data
    EntoContainer<Vec2<Scalar>, N> points2D;
    EntoContainer<Vec3<Scalar>, N> points3D;
    CameraPose<Scalar> true_pose;

    
    generate_synthetic_absolute_pose_data<Scalar, N>(points2D, points3D, true_pose, num_points);
    
    for ( const auto& point : points2D )
    {
        ENTO_DEBUG("point: %f, %f", point(0), point(1));
    }
    for ( const auto& point : points3D )
    {
        ENTO_DEBUG("point: %f, %f, %f", point(0), point(1), point(2));
    }
    // Debug prints for input data
    ENTO_DEBUG("Input data before estimate_absolute_pose:");
    ENTO_DEBUG("Number of points: %zu", num_points);
    ENTO_DEBUG("True pose - R: %f %f %f %f, t: %f %f %f", 
        true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(),
        true_pose.t.x(), true_pose.t.y(), true_pose.t.z());
    
    for (size_t i = 0; i < num_points; ++i) {
        Vec3<Scalar> Z = true_pose.R() * points3D[i] + true_pose.t;
        ENTO_DEBUG("Point %zu: 2D=(%f,%f), 3D=(%f,%f,%f), depth=%f", 
            i, points2D[i](0), points2D[i](1), 
            points3D[i](0), points3D[i](1), points3D[i](2),
            Z(2));
    }
    
    // Setup camera
    using CameraModel = IdentityCameraModel<Scalar>;
    using Params = std::array<Scalar, 0>;
    Params params;
    Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
    
    // Setup RANSAC options - use same parameters as fixed-size test
    RansacOptions<Scalar> ransac_opt;
    ransac_opt.max_iters = 1000;
    ransac_opt.max_reproj_error = 3.0;
    ransac_opt.success_prob = 0.99;
    
    // Setup bundle adjustment options - use same parameters as fixed-size test
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.max_iterations = 10;
    
    // Estimate pose
    CameraPose<Scalar> estimated_pose;
    EntoContainer<uint8_t, N> inliers;
    
    RansacStats<Scalar> stats = estimate_absolute_pose<Solver, N>(
        points2D, points3D, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
    
    // Debug prints before first check
    ENTO_DEBUG("True pose - R: %f %f %f %f, t: %f %f %f", 
        true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(),
        true_pose.t.x(), true_pose.t.y(), true_pose.t.z());

    ENTO_DEBUG("Estimated pose - R: %f %f %f %f, t: %f %f %f",
        estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(),
        estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());

    ENTO_DEBUG("RANSAC stats - inliers: %zu, score: %f, iterations: %zu", 
        stats.num_inliers, stats.model_score, stats.iters);

    // Print reprojection errors for each point
    for (size_t i = 0; i < num_points; ++i) {
        Vec3<Scalar> Z = estimated_pose.R() * points3D[i] + estimated_pose.t;
        Scalar r2 = (Z.hnormalized() - points2D[i]).squaredNorm();
        ENTO_DEBUG("Point %zu: error=%f, inlier=%d", i, r2, (bool)inliers[i]);
    }

    // Check results
    if (!(stats.num_inliers >= 3)) {
        ENTO_DEBUG("[FAIL] num_inliers: %zu", stats.num_inliers);
        ENTO_DEBUG("Inlier mask: ");
        for (size_t i = 0; i < num_points; ++i) ENTO_DEBUG("%d", (int)inliers[i]);
        ENTO_DEBUG("Model score: %f", stats.model_score);
    }
    ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 3);
    if (!(stats.num_inliers <= num_points)) {
        ENTO_DEBUG("[FAIL] num_inliers: %zu", stats.num_inliers);
    }
    ENTO_TEST_CHECK_TRUE(stats.num_inliers <= num_points);
    if (!(stats.model_score >= 0)) {
        ENTO_DEBUG("[FAIL] model_score: %f", stats.model_score);
    }
    ENTO_TEST_CHECK_TRUE(stats.model_score >= 0);
    if (!(inliers.size() == num_points)) {
        ENTO_DEBUG("[FAIL] inliers.size(): %zu", inliers.size());
    }
    ENTO_TEST_CHECK_INT_EQ(inliers.size(), num_points);
    // Count inliers manually
    size_t manual_inlier_count = 0;
    for (size_t i = 0; i < num_points; ++i) {
        if (inliers[i]) manual_inlier_count++;
    }
    if (!(manual_inlier_count == stats.num_inliers)) {
        ENTO_DEBUG("[FAIL] manual_inlier_count: %zu, stats.num_inliers: %zu", manual_inlier_count, stats.num_inliers);
    }
    ENTO_TEST_CHECK_INT_EQ(manual_inlier_count, stats.num_inliers);
    if (!(estimated_pose.q.norm() > 0.9)) {
        ENTO_DEBUG("[FAIL] estimated_pose.q.norm(): %f", estimated_pose.q.norm());
    }
    ENTO_TEST_CHECK_TRUE(estimated_pose.q.norm() > 0.9);
    if (!(estimated_pose.t.norm() > 0.1)) {
        ENTO_DEBUG("[FAIL] estimated_pose.t.norm(): %f", estimated_pose.t.norm());
    }
    ENTO_TEST_CHECK_TRUE(estimated_pose.t.norm() > 0.1);
    Scalar rot_error = (estimated_pose.R() - true_pose.R()).norm();
    Scalar trans_error = (estimated_pose.t - true_pose.t).norm();
    if (!(rot_error < 0.1)) {
        ENTO_DEBUG("[FAIL] rot_error: %f", rot_error);
    }
    ENTO_TEST_CHECK_TRUE(rot_error < 0.1);
    if (!(trans_error < 0.1)) {
        ENTO_DEBUG("[FAIL] trans_error: %f", trans_error);
    }
    ENTO_TEST_CHECK_TRUE(trans_error < 0.1);
    
    ENTO_DEBUG("Dynamic-size test passed:");
    ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, num_points);
    ENTO_DEBUG("  Score: %f", stats.model_score);
    ENTO_DEBUG("  Iterations: %zu", stats.iters);
    ENTO_DEBUG("  Rotation error: %f", rot_error);
    ENTO_DEBUG("  Translation error: %f", trans_error);
    ENTO_DEBUG("================");
}

// Test with noisy data and outliers
void test_robust_absolute_pose_with_outliers()
{
    using Scalar = float;
    using Solver = SolverP3P<Scalar>;
    constexpr size_t N = 30;
    
    ENTO_DEBUG("================");
    ENTO_DEBUG("Running test_robust_absolute_pose_with_outliers...");
    
    // Generate clean inlier data
    EntoContainer<Vec2<Scalar>, N> points2D;
    EntoContainer<Vec3<Scalar>, N> points3D;
    CameraPose<Scalar> true_pose;
    
    generate_synthetic_absolute_pose_data<Scalar, N>(points2D, points3D, true_pose, 20, 0.01);
    
    // Add some outliers
    std::default_random_engine rng(123);
    std::uniform_real_distribution<Scalar> outlier_gen(-2.0, 2.0);
    
    for (size_t i = 20; i < N; ++i) {
        // Random 3D point
        Vec3<Scalar> X_outlier(outlier_gen(rng), outlier_gen(rng), outlier_gen(rng));
        // Random 2D point (not consistent with pose)
        Vec2<Scalar> x_outlier(outlier_gen(rng), outlier_gen(rng));
        
        points2D.push_back(x_outlier);
        points3D.push_back(X_outlier);
    }
    
    // Setup camera and options
    using CameraModel = IdentityCameraModel<Scalar>;
    using Params = std::array<Scalar, 0>;
    Params params;
    Camera<Scalar, CameraModel> camera(1.0, 1.0, params);
    
    RansacOptions<Scalar> ransac_opt;
    ransac_opt.max_iters = 2000; // More iterations for outliers
    ransac_opt.max_reproj_error = 0.1;
    ransac_opt.success_prob = 0.999;
    
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.max_iterations = 15;
    
    // Estimate pose
    CameraPose<Scalar> estimated_pose;
    EntoContainer<uint8_t, N> inliers;
    
    RansacStats<Scalar> stats = estimate_absolute_pose<Solver, N>(
        points2D, points3D, camera, ransac_opt, bundle_opt, &estimated_pose, &inliers);
    
    // Debug prints before first check
    ENTO_DEBUG("True pose - R: %f %f %f %f, t: %f %f %f", 
        true_pose.q.x(), true_pose.q.y(), true_pose.q.z(), true_pose.q.w(),
        true_pose.t.x(), true_pose.t.y(), true_pose.t.z());

    ENTO_DEBUG("Estimated pose - R: %f %f %f %f, t: %f %f %f",
        estimated_pose.q.x(), estimated_pose.q.y(), estimated_pose.q.z(), estimated_pose.q.w(),
        estimated_pose.t.x(), estimated_pose.t.y(), estimated_pose.t.z());

    ENTO_DEBUG("RANSAC stats - inliers: %zu, score: %f, iterations: %zu", 
        stats.num_inliers, stats.model_score, stats.iters);

    // Print reprojection errors for each point
    for (size_t i = 0; i < N; ++i) {
        Vec3<Scalar> Z = estimated_pose.R() * points3D[i] + estimated_pose.t;
        Scalar r2 = (Z.hnormalized() - points2D[i]).squaredNorm();
        ENTO_DEBUG("Point %zu: error=%f, inlier=%d", i, r2, (bool)inliers[i]);
    }

    // Should still find good solution despite outliers
    if (!(stats.num_inliers >= 10)) {
        ENTO_DEBUG("[FAIL] num_inliers: %zu", stats.num_inliers);
        ENTO_DEBUG("Inlier mask: ");
        for (size_t i = 0; i < N; ++i) ENTO_DEBUG("%d", (int)inliers[i]);
        ENTO_DEBUG("Model score: %f", stats.model_score);
    }
    ENTO_TEST_CHECK_TRUE(stats.num_inliers >= 10);
    if (!(stats.num_inliers <= 20)) {
        ENTO_DEBUG("[FAIL] num_inliers: %zu", stats.num_inliers);
    }
    ENTO_TEST_CHECK_TRUE(stats.num_inliers <= 20);
    
    // Most inliers should be from the first 20 points (clean data)
    size_t clean_inliers = 0;
    for (size_t i = 0; i < 20; ++i) {
        if (inliers[i]) clean_inliers++;
    }
    if (!(clean_inliers >= stats.num_inliers * 0.8)) {
        ENTO_DEBUG("[FAIL] clean_inliers: %zu, stats.num_inliers: %zu", clean_inliers, stats.num_inliers);
    }
    ENTO_TEST_CHECK_TRUE(clean_inliers >= stats.num_inliers * 0.8);
    
    ENTO_DEBUG("Outlier test passed:");
    ENTO_DEBUG("  Inliers: %zu/%zu", stats.num_inliers, N);
    ENTO_DEBUG("  Clean inliers: %zu/20", clean_inliers);
    ENTO_DEBUG("  Score: %f", stats.model_score);
    ENTO_DEBUG("================");
}

int main(int argc, char** argv)
{
    using namespace EntoUtil;
    int __n;
    if (argc > 1) {
        __n = atoi(argv[1]);
    } else {
        __ento_replace_file_suffix(__FILE__, "test_robust_absolute_pose_cmdline.txt");
        __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
    }

    if (__ento_test_num(__n, 1)) test_robust_absolute_pose_fixed_size();
    if (__ento_test_num(__n, 2)) test_robust_absolute_pose_dynamic_size();
    if (__ento_test_num(__n, 3)) test_robust_absolute_pose_with_outliers();
    
    return 0;
} 
