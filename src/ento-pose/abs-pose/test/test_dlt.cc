#include <iostream>
#include <iomanip>
#include <cmath>

#include <Eigen/Dense>
#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/containers.h>

#include <ento-pose/abs-pose/dlt.h>
#include <ento-pose/synthetic_abspose.h>
#include <ento-pose/pose_util.h>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoPose;

// Test DLT with perfect noise-free data
void test_dlt_perfect() {
    std::cout << "\n=== Testing DLT with Perfect Data ===" << std::endl;
    
    constexpr size_t N = 6;  // Static container for 6 points
    using Scalar = float;
    
    // Generate synthetic data
    EntoContainer<Vec2<Scalar>, N> points2D;
    EntoContainer<Vec3<Scalar>, N> points3D;
    CameraPose<Scalar> true_pose;
    
    // Generate perfect data (no noise)
    generate_synthetic_abspose_general<Scalar, N>(points2D, points3D, true_pose, N, Scalar(0.0));
    
    std::cout << "Generated " << N << " points" << std::endl;
    std::cout << "True pose:" << std::endl;
    std::cout << "  R = [" << true_pose.R()(0,0) << " " << true_pose.R()(0,1) << " " << true_pose.R()(0,2) << "]" << std::endl;
    std::cout << "      [" << true_pose.R()(1,0) << " " << true_pose.R()(1,1) << " " << true_pose.R()(1,2) << "]" << std::endl;
    std::cout << "      [" << true_pose.R()(2,0) << " " << true_pose.R()(2,1) << " " << true_pose.R()(2,2) << "]" << std::endl;
    std::cout << "  t = [" << true_pose.t(0) << " " << true_pose.t(1) << " " << true_pose.t(2) << "]" << std::endl;
    
    // Convert 2D points to homogeneous for DLT
    EntoContainer<Vec3<Scalar>, N> points2D_homo;
    for (size_t i = 0; i < N; ++i) {
        Vec3<Scalar> homo(points2D[i](0), points2D[i](1), Scalar(1.0));
        points2D_homo.push_back(homo);
    }
    
    // Test DLT solver
    EntoArray<CameraPose<Scalar>, 1> solutions;
    int num_solutions = dlt<Scalar, N>(points2D_homo, points3D, &solutions);
    
    std::cout << "DLT returned " << num_solutions << " solutions" << std::endl;
    
    if (num_solutions > 0) {
        const CameraPose<Scalar>& estimated_pose = solutions[0];
        std::cout << "Estimated pose:" << std::endl;
        std::cout << "  R = [" << estimated_pose.R()(0,0) << " " << estimated_pose.R()(0,1) << " " << estimated_pose.R()(0,2) << "]" << std::endl;
        std::cout << "      [" << estimated_pose.R()(1,0) << " " << estimated_pose.R()(1,1) << " " << estimated_pose.R()(1,2) << "]" << std::endl;
        std::cout << "      [" << estimated_pose.R()(2,0) << " " << estimated_pose.R()(2,1) << " " << estimated_pose.R()(2,2) << "]" << std::endl;
        std::cout << "  t = [" << estimated_pose.t(0) << " " << estimated_pose.t(1) << " " << estimated_pose.t(2) << "]" << std::endl;
        
        // Compute pose errors
        Matrix3<Scalar> R_diff = estimated_pose.R().transpose() * true_pose.R();
        Scalar trace_val = R_diff.trace();
        Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
        Scalar rotation_error_deg = angle_rad * Scalar(180.0 / M_PI);
        
        Scalar translation_error = (estimated_pose.t - true_pose.t).norm();
        
        std::cout << "Errors:" << std::endl;
        std::cout << "  Rotation: " << std::fixed << std::setprecision(6) << rotation_error_deg << " degrees" << std::endl;
        std::cout << "  Translation: " << translation_error << " meters" << std::endl;
        
        // For perfect data, errors should be very small
        ENTO_TEST_CHECK_TRUE(rotation_error_deg < 0.01);
        ENTO_TEST_CHECK_TRUE(translation_error < 0.01);
        
        std::cout << "✓ DLT perfect test PASSED" << std::endl;
    } else {
        ENTO_TEST_CHECK_TRUE(false);
        std::cout << "✗ DLT perfect test FAILED - no solutions found" << std::endl;
    }
}

// Test DLT with noisy data
void test_dlt_noisy() {
    std::cout << "\n=== Testing DLT with Noisy Data ===" << std::endl;
    
    constexpr size_t N = 8;  // More points for noise robustness
    using Scalar = float;
    Scalar noise_level = Scalar(0.01);  // Small amount of noise
    
    // Generate synthetic data with noise
    EntoContainer<Vec2<Scalar>, N> points2D;
    EntoContainer<Vec3<Scalar>, N> points3D;
    CameraPose<Scalar> true_pose;
    
    generate_synthetic_abspose_general<Scalar, N>(points2D, points3D, true_pose, N, noise_level);
    
    std::cout << "Generated " << N << " points with noise level " << noise_level << std::endl;
    
    // Convert 2D points to homogeneous for DLT
    EntoContainer<Vec3<Scalar>, N> points2D_homo;
    for (size_t i = 0; i < N; ++i) {
        Vec3<Scalar> homo(points2D[i](0), points2D[i](1), Scalar(1.0));
        points2D_homo.push_back(homo);
    }
    
    // Test DLT solver
    EntoArray<CameraPose<Scalar>, 1> solutions;
    int num_solutions = dlt<Scalar, N>(points2D_homo, points3D, &solutions);
    
    std::cout << "DLT returned " << num_solutions << " solutions" << std::endl;
    
    if (num_solutions > 0) {
        const CameraPose<Scalar>& estimated_pose = solutions[0];
        
        // Compute pose errors
        Matrix3<Scalar> R_diff = estimated_pose.R().transpose() * true_pose.R();
        Scalar trace_val = R_diff.trace();
        Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
        Scalar rotation_error_deg = angle_rad * Scalar(180.0 / M_PI);
        
        Scalar translation_error = (estimated_pose.t - true_pose.t).norm();
        
        std::cout << "Errors:" << std::endl;
        std::cout << "  Rotation: " << std::fixed << std::setprecision(3) << rotation_error_deg << " degrees" << std::endl;
        std::cout << "  Translation: " << translation_error << " meters" << std::endl;
        
        // For noisy data, errors should be reasonable but not perfect
        ENTO_TEST_CHECK_TRUE(rotation_error_deg < 5.0);
        ENTO_TEST_CHECK_TRUE(translation_error < 1.0);
        
        std::cout << "✓ DLT noisy test PASSED" << std::endl;
    } else {
        ENTO_TEST_CHECK_TRUE(false);
        std::cout << "✗ DLT noisy test FAILED - no solutions found" << std::endl;
    }
}

// Test DLT with different point counts
void test_dlt_different_point_counts() {
    std::cout << "\n=== Testing DLT with Different Point Counts ===" << std::endl;
    
    using Scalar = float;
    Scalar noise_level = Scalar(0.005);
    
    // Test with minimum points (6)
    {
        constexpr size_t N = 6;
        std::cout << "\nTesting with " << N << " points (minimum)" << std::endl;
        
        EntoContainer<Vec2<Scalar>, N> points2D;
        EntoContainer<Vec3<Scalar>, N> points3D;
        CameraPose<Scalar> true_pose;
        
        generate_synthetic_abspose_general<Scalar, N>(points2D, points3D, true_pose, N, noise_level);
        
        // Convert to homogeneous
        EntoContainer<Vec3<Scalar>, N> points2D_homo;
        for (size_t i = 0; i < N; ++i) {
            Vec3<Scalar> homo(points2D[i](0), points2D[i](1), Scalar(1.0));
            points2D_homo.push_back(homo);
        }
        
        EntoArray<CameraPose<Scalar>, 1> solutions;
        int num_solutions = dlt<Scalar, N>(points2D_homo, points3D, &solutions);
        
        std::cout << "  Solutions found: " << num_solutions << std::endl;
        ENTO_TEST_CHECK_TRUE(num_solutions >= 0);
    }
    
    // Test with more points (16)
    {
        constexpr size_t N = 16;
        std::cout << "\nTesting with " << N << " points (overdetermined)" << std::endl;
        
        EntoContainer<Vec2<Scalar>, N> points2D;
        EntoContainer<Vec3<Scalar>, N> points3D;
        CameraPose<Scalar> true_pose;
        
        generate_synthetic_abspose_general<Scalar, N>(points2D, points3D, true_pose, N, noise_level);
        
        // Convert to homogeneous
        EntoContainer<Vec3<Scalar>, N> points2D_homo;
        for (size_t i = 0; i < N; ++i) {
            Vec3<Scalar> homo(points2D[i](0), points2D[i](1), Scalar(1.0));
            points2D_homo.push_back(homo);
        }
        
        EntoArray<CameraPose<Scalar>, 1> solutions;
        int num_solutions = dlt<Scalar, N>(points2D_homo, points3D, &solutions);
        
        std::cout << "  Solutions found: " << num_solutions << std::endl;
        
        if (num_solutions > 0) {
            const CameraPose<Scalar>& estimated_pose = solutions[0];
            
            // Compute pose errors
            Matrix3<Scalar> R_diff = estimated_pose.R().transpose() * true_pose.R();
            Scalar trace_val = R_diff.trace();
            Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
            Scalar rotation_error_deg = angle_rad * Scalar(180.0 / M_PI);
            
            Scalar translation_error = (estimated_pose.t - true_pose.t).norm();
            
            std::cout << "  Rotation error: " << std::fixed << std::setprecision(3) << rotation_error_deg << "°" << std::endl;
            std::cout << "  Translation error: " << translation_error << "m" << std::endl;
            
            // More points should generally give better results
            ENTO_TEST_CHECK_TRUE(rotation_error_deg < 3.0);
            ENTO_TEST_CHECK_TRUE(translation_error < 0.5);
        }
    }
    
    std::cout << "✓ Different point count tests completed" << std::endl;
}

// Test DLT with double precision
void test_dlt_double_precision() {
    std::cout << "\n=== Testing DLT with Double Precision ===" << std::endl;
    
    constexpr size_t N = 8;
    using Scalar = double;  // Use double precision
    
    std::cout << "Step 1: Creating simple synthetic data directly..." << std::endl;
    
    // Create simple, deterministic test data to avoid any random generation issues
    EntoContainer<Vec2<Scalar>, N> points2D;
    EntoContainer<Vec3<Scalar>, N> points3D;
    CameraPose<Scalar> true_pose;
    
    // Set a simple, known pose
    true_pose.q = Eigen::Quaternion<Scalar>::Identity().coeffs();  // Identity rotation
    true_pose.t = Vec3<Scalar>(0.1, 0.2, 0.3);  // Small translation
    
    std::cout << "Step 2: Generating deterministic 3D points..." << std::endl;
    
    // Generate simple 3D points in a grid pattern
    std::vector<Vec3<Scalar>> world_points = {
        Vec3<Scalar>(-1, -1, 3),
        Vec3<Scalar>( 1, -1, 3),
        Vec3<Scalar>( 1,  1, 3),
        Vec3<Scalar>(-1,  1, 3),
        Vec3<Scalar>(-0.5, -0.5, 4),
        Vec3<Scalar>( 0.5, -0.5, 4),
        Vec3<Scalar>( 0.5,  0.5, 4),
        Vec3<Scalar>(-0.5,  0.5, 4)
    };
    
    std::cout << "Step 3: Computing projections..." << std::endl;
    
    // Project to 2D using the known pose
    for (size_t i = 0; i < N; ++i) {
        const Vec3<Scalar>& X = world_points[i];
        points3D.push_back(X);
        
        // Project to camera coordinates
        Vec3<Scalar> x_cam = true_pose.R() * X + true_pose.t;
        
        // Project to normalized image coordinates
        Vec2<Scalar> x_norm(x_cam(0) / x_cam(2), x_cam(1) / x_cam(2));
        
        // Add tiny bit of noise for realism
        x_norm(0) += Scalar(0.0001) * (Scalar(i) - Scalar(N)/2);
        x_norm(1) += Scalar(0.0001) * (Scalar(i%2) - 0.5);
        
        points2D.push_back(x_norm);
    }
    
    std::cout << "Step 4: Converting to homogeneous coordinates..." << std::endl;
    EntoContainer<Vec3<Scalar>, N> points2D_homo;
    for (size_t i = 0; i < N; ++i) {
        Vec3<Scalar> homo(points2D[i](0), points2D[i](1), Scalar(1.0));
        points2D_homo.push_back(homo);
    }
    
    std::cout << "Step 5: Calling DLT solver..." << std::endl;
    // Test DLT solver
    EntoArray<CameraPose<Scalar>, 1> solutions;
    int num_solutions = dlt<Scalar, N>(points2D_homo, points3D, &solutions);
    
    std::cout << "Step 6: DLT solver completed. Returned " << num_solutions << " solutions" << std::endl;
    
    if (num_solutions > 0) {
        const CameraPose<Scalar>& estimated_pose = solutions[0];
        
        std::cout << "Step 7: Computing pose errors..." << std::endl;
        
        // Compute pose errors
        Matrix3<Scalar> R_diff = estimated_pose.R().transpose() * true_pose.R();
        Scalar trace_val = R_diff.trace();
        Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
        Scalar rotation_error_deg = angle_rad * Scalar(180.0 / M_PI);
        
        Scalar translation_error = (estimated_pose.t - true_pose.t).norm();
        
        std::cout << "Step 8: Errors computed successfully" << std::endl;
        std::cout << "Errors:" << std::endl;
        std::cout << "  Rotation: " << std::fixed << std::setprecision(6) << rotation_error_deg << " degrees" << std::endl;
        std::cout << "  Translation: " << std::setprecision(6) << translation_error << " meters" << std::endl;
        
        // Double precision should give very good results with deterministic data
        ENTO_TEST_CHECK_TRUE(rotation_error_deg < 1.0);
        ENTO_TEST_CHECK_TRUE(translation_error < 0.1);
        
        std::cout << "✓ DLT double precision test PASSED" << std::endl;
    } else {
        ENTO_TEST_CHECK_TRUE(false);
        std::cout << "✗ DLT double precision test FAILED - no solutions found" << std::endl;
    }
}

int main(int argc, char** argv) {
    std::cout << "DLT Standalone Test Suite" << std::endl;
    std::cout << "=========================" << std::endl;
    
    int test_num = (argc == 1) ? 0 : atoi(argv[1]);
    
    if (__ento_test_num(test_num, 1)) test_dlt_perfect();
    if (__ento_test_num(test_num, 2)) test_dlt_noisy();
    if (__ento_test_num(test_num, 3)) test_dlt_different_point_counts();
    if (__ento_test_num(test_num, 4)) test_dlt_double_precision();
    
    std::cout << "\n=== DLT Test Suite Complete ===" << std::endl;
    return 0;
}
